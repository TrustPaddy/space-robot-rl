function results = optimizeAgent(agentType, varargin)
% optimizeAgent  Bayes-Hyperparameter-Optimierung fuer EINEN Agenttyp.
%
%   results = optimizeAgent(agentType)
%   results = optimizeAgent(agentType, 'Name', Value, ...)
%
%   agentType : "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"
%
%   Name-Value-Optionen:
%     'MaxObjectiveEvaluations' (60)      Anzahl bayesopt-Trials
%     'MaxEpisodes'             (500)     Screening-Episoden pro Trial
%     'EvalEpisodes'            (5)        deterministische Eval-Episoden
%     'EvalFrequency'           (20)      Eval alle N Episoden
%     'UseParallel'             (true)    parallele Trials (bayesopt-Ebene)
%     'NumWorkers'              (8)        Poolgroesse (8-Kern-CPU)
%     'MaxTime'                 (12*3600) Sicherheitsnetz [s]
%     'OutDir'    ('SavedAgents/Circular/BO')  Ausgabe-Basisverzeichnis
%
%   Strategie: PARALLELE TRIALS. bayesopt('UseParallel',true) verteilt die
%   Trials auf die Worker; jeder Trial trainiert SEQUENTIELL. Die Simulink-Env
%   wird via parallel.pool.Constant genau einmal pro Worker gebaut und
%   wiederverwendet -> loest das "Worker-Variablen fehlen"-Problem.

    % ---- Argumente ----
    ip = inputParser;
    ip.addRequired('agentType', @(x) any(strcmpi(x, {'PG','PPO','TRPO','DDPG','TD3','SAC'})));
    ip.addParameter('MaxObjectiveEvaluations', 60);
    ip.addParameter('MaxEpisodes', 500);
    ip.addParameter('EvalEpisodes', 5);
    ip.addParameter('EvalFrequency', 20);
    ip.addParameter('UseParallel', true);
    ip.addParameter('NumWorkers', 8);
    ip.addParameter('MaxTime', 12*3600);
    ip.addParameter('OutDir', fullfile('SavedAgents','Circular','BO'));
    ip.addParameter('Preflight', true);   % kurzer serieller Trial vor dem Lauf
    ip.parse(agentType, varargin{:});
    a = ip.Results;
    agentType = upper(string(agentType));

    % ---- Ausgabeverzeichnis ----
    runDir = fullfile(a.OutDir, char(agentType), char(datetime('now','Format','yyyyMMdd_HHmmss')));
    if ~exist(runDir, 'dir'), mkdir(runDir); end
    fprintf('[optimizeAgent] %s -> %s\n', agentType, runDir);

    % ---- Screening-/Eval-Optionen fuer die Objective ----
    % RunDir wird mitgegeben, damit trainAndEvaluate fehlgeschlagene Trials in
    % <RunDir>/trial_errors.log protokollieren kann (Worker-warnings sind sonst
    % unsichtbar).
    opt = struct('MaxEpisodes', a.MaxEpisodes, ...
                 'EvalEpisodes', a.EvalEpisodes, ...
                 'EvalFrequency', a.EvalFrequency, ...
                 'RunDir', runDir);

    % ---- Parallel-Pool + Env-Quelle vorbereiten ----
    if a.UseParallel
        pool = gcp('nocreate');
        if isempty(pool)
            if isempty(a.NumWorkers), pool = parpool; else, pool = parpool(a.NumWorkers); end
        end
        attachIfExists(pool, { ...
            'SpaceRobot.slx', 'SpaceRobot.urdf', ...
            'localResetFunction.m', 'setupSpaceRobotEnv.m', ...
            'collisionCheckWrapper.m', ...   % vom Modell zur Laufzeit aufgerufen
            'buildAgent.m', 'getSearchSpace.m', ...
            'trainAndEvaluate.m', 'computeKPIsFromLogs.m'});
        % Env genau einmal pro Worker (lazy) bauen und cachen:
        envSrc = parallel.pool.Constant(@setupSpaceRobotEnv);
    else
        % Seriell: Env einmal lokal bauen (auch fuer Debug/Build-Check).
        envSrc = setupSpaceRobotEnv();
    end

    % ---- Suchraum ----
    vars = getSearchSpace(agentType);

    % ---- Pre-Flight: 1 kurzer serieller Trial. Bricht in Sekunden ab, wenn die
    %      Pipeline kaputt ist, statt stundenlang stumme 1e6 zu produzieren. ----
    if a.Preflight
        preflight(agentType, opt);
    end

    % ---- Bayes-Optimierung ----
    results = bayesopt( ...
        @(p) trainAndEvaluate(p, agentType, envSrc, opt), ...
        vars, ...
        'UseParallel',              a.UseParallel, ...
        'MaxObjectiveEvaluations',  a.MaxObjectiveEvaluations, ...
        'MaxTime',                  a.MaxTime, ...
        'AcquisitionFunctionName',  'expected-improvement-plus', ...
        'IsObjectiveDeterministic', false, ...
        'OutputFcn',                @(r,s) saveBOState(r,s,runDir), ...
        'Verbose',                  1);

    % ---- Ergebnisse sichern ----
    save(fullfile(runDir,'results.mat'), 'results', '-v7.3');

    [bestAgent, bestParams, bestScore, bestIter] = pickBestAgent(results);
    if ~isempty(bestAgent)
        agent = bestAgent;                                  % bequemer Ladename (fuer 'load')
        save(fullfile(runDir,'bestAgent.mat'), ...
             'agent', 'bestAgent', 'bestParams', 'bestScore', 'agentType', 'bestIter');
        fprintf('[optimizeAgent] Bester Trial: #%d, Score = %.4f\n', bestIter, bestScore);
    else
        warning('optimizeAgent:noValidAgent', ...
            'Kein gueltiger Agent in UserDataTrace gefunden (alle Trials fehlgeschlagen?).');
    end

    writeReport(fullfile(runDir,'report.txt'), results, agentType, bestScore, bestIter);
end

% ===== Hilfsfunktionen =====================================================

function preflight(agentType, opt)
% Schnelle End-to-End-Validierung VOR dem (teuren, parallelen) bayesopt-Lauf.
%   1) checkBuildAgents(): baut alle Agenten (Sekunden, ohne Simulink).
%   2) EIN sehr kurzer serieller Trial ueber trainAndEvaluate -> deckt Env-/
%      Simulink-Fehler auf, die sonst nur als stumme 1e6 erscheinen.
% Schlaegt eine Pruefung fehl, wird mit klarer Meldung abgebrochen.
    fprintf('[optimizeAgent] Pre-Flight-Check ...\n');

    if ~isempty(which('checkBuildAgents'))
        if ~checkBuildAgents()
            error('optimizeAgent:preflightBuild', ...
                'Pre-Flight: checkBuildAgents() fehlgeschlagen (siehe Ausgabe oben).');
        end
    end

    S = setupSpaceRobotEnv();
    p = midParams(getSearchSpace(agentType));
    qopt = struct('MaxEpisodes',2, 'EvalEpisodes',1, 'EvalFrequency',100, ...
                  'RunDir', opt.RunDir);
    obj = trainAndEvaluate(p, agentType, S, qopt);

    if ~isfinite(obj) || obj >= 1e6 - 1
        error('optimizeAgent:preflightTrial', ...
            ['Pre-Flight: kurzer Trial lieferte Objective = %g (>=1e6 bedeutet ', ...
             'Fehler). Details in %s. Lauf abgebrochen, um keine Rechenzeit zu ', ...
             'verschwenden. (Deaktivieren mit ''Preflight'',false.)'], ...
            obj, fullfile(opt.RunDir,'trial_errors.log'));
    end
    fprintf('[optimizeAgent] Pre-Flight OK (Objective = %.4f). Starte Optimierung.\n', obj);
end

function p = midParams(vars)
% Mittelwert (geometrisch bei log, gerundet bei integer) je Suchraum-Parameter.
    p = struct();
    for k = 1:numel(vars)
        v = vars(k);
        r = v.Range;
        if strcmpi(v.Type, 'integer')
            val = round(mean(r));
        elseif strcmpi(v.Transform, 'log')
            val = exp(mean(log(r)));
        else
            val = mean(r);
        end
        p.(v.Name) = val;
    end
end

function attachIfExists(pool, files)
    ex = files(cellfun(@(f) ~isempty(which(f)) || exist(f,'file')>0, files));
    if ~isempty(ex)
        addAttachedFiles(pool, ex);
    end
end

function [agent, params, score, iter] = pickBestAgent(results)
% Waehlt den Agenten mit dem KLEINSTEN Objective, der ein gueltiges 'agent'-Feld
% besitzt (ueberspringt fehlgeschlagene Trials).
    agent = []; params = []; score = NaN; iter = NaN;
    obj = results.ObjectiveTrace(:);
    ud  = results.UserDataTrace(:);
    [~, order] = sort(obj, 'ascend');
    for k = 1:numel(order)
        i = order(k);
        if i <= numel(ud) && isstruct(ud{i}) && isfield(ud{i},'agent') && ~isempty(ud{i}.agent)
            agent  = ud{i}.agent;
            params = ud{i}.params;
            if isfield(ud{i},'score'), score = ud{i}.score; else, score = -obj(i); end
            iter   = i;
            return;
        end
    end
end

function writeReport(fpath, results, agentType, bestScore, bestIter)
    fid = fopen(fpath, 'w');
    if fid < 0, return; end
    c = onCleanup(@() fclose(fid));
    fprintf(fid, 'Bayes-Optimierung Report\n');
    fprintf(fid, 'Agent            : %s\n', char(agentType));
    fprintf(fid, 'Zeitstempel      : %s\n', char(datetime('now')));
    fprintf(fid, 'Evaluations      : %d\n', numel(results.ObjectiveTrace));
    fprintf(fid, 'Bester Trial     : #%d\n', bestIter);
    fprintf(fid, 'Bester Score     : %.6f  (Objective %.6f)\n', bestScore, results.MinObjective);
    fprintf(fid, '\nBeste Hyperparameter (bestPoint):\n');
    try
        bp = bestPoint(results);
        vn = bp.Properties.VariableNames;
        for i = 1:numel(vn)
            fprintf(fid, '  %-22s = %g\n', vn{i}, bp.(vn{i}));
        end
    catch ME
        fprintf(fid, '  (bestPoint nicht verfuegbar: %s)\n', ME.message);
    end
end
