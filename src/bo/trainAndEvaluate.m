function [objective, constraints, userData] = trainAndEvaluate(params, agentType, envSrc, opt)
% trainAndEvaluate  Bayesopt-Zielfunktion: trainiert + bewertet EINEN Agenten.
%
%   [objective, constraints, userData] = trainAndEvaluate(params, agentType, envSrc, opt)
%
%   params   : 1-Zeilen-Table (von bayesopt) ODER Struct mit Hyperparametern
%   agentType: "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"
%   envSrc   : parallel.pool.Constant (envSrc.Value -> Env-Struct von
%              setupSpaceRobotEnv) ODER direkt das Env-Struct S (seriell)
%   opt      : Struct mit Feldern (alle optional):
%                MaxEpisodes   (Default 250)  - kurzes Screening-Training
%                EvalEpisodes  (Default 5)    - deterministische Eval-Episoden
%                EvalFrequency (Default 20)   - alle N Episoden evaluieren
%
%   Rueckgabe (MathWorks-konform, 3 Ausgaben):
%     objective   = -score   (bayesopt MINIMIERT -> wir maximieren den Score)
%     constraints = []        (keine Nebenbedingungen)
%     userData    = struct mit trainiertem Agenten + params + score
%                   -> nach der Suche via results.UserDataTrace{iter} abrufbar
%
%   PARALLEL-SICHER: kein Zugriff auf den Base-Workspace des Clients, kein
%   train('UseParallel',true). Die Env kommt worker-lokal aus envSrc.

    if nargin < 4 || isempty(opt), opt = struct(); end
    opt = withDefaults(opt, struct('MaxEpisodes',250, 'EvalEpisodes',5, 'EvalFrequency',20));

    % --- Env worker-lokal beziehen (Constant ODER direktes Struct) ---
    if isa(envSrc, 'parallel.pool.Constant')
        S = envSrc.Value;
    else
        S = envSrc;
    end

    % --- Hyperparameter als Struct ---
    if istable(params)
        p = table2struct(params);
    else
        p = params;
    end

    constraints = [];

    try
        rng(0,'threefry');   % Reproduzierbarkeit je Trial (fairer Vergleich)

        % --- Agent mit aktuellen HPs bauen ---
        agent = buildAgent(agentType, p, S.obsInfo, S.actInfo, S.Ts_agent);

        % --- Kurzes Screening-Training (sequentiell!) ---
        trainOpts = rlTrainingOptions( ...
            'MaxEpisodes',              opt.MaxEpisodes, ...
            'MaxStepsPerEpisode',       floor(S.T / S.Ts_agent), ...
            'ScoreAveragingWindowLength', 25, ...
            'StopTrainingCriteria',     'none', ...
            'Plots',                    'none', ...        % kein Plot pro Trial
            'UseParallel',              false, ...          % KEIN geschachteltes Parallel
            'Verbose',                  false);

        % --- Deterministische Evaluation waehrend des Trainings ---
        useEval = true;
        try
            evl = rlEvaluator( ...
                NumEpisodes       = opt.EvalEpisodes, ...
                EvaluationFrequency = opt.EvalFrequency);
        catch
            useEval = false;   % aeltere Toolbox ohne rlEvaluator -> Fallback
        end

        if useEval
            result = train(agent, S.env, trainOpts, Evaluator = evl);
        else
            result = train(agent, S.env, trainOpts);
        end

        % --- Score bestimmen ---
        score = extractScore(result);

        objective = -score;    % bayesopt minimiert
        userData  = struct('agent', agent, 'params', p, 'score', score, ...
                           'agentType', char(agentType));

    catch ME
        % Vollstaendigen Bericht (inkl. Ursachenkette) erzeugen - NUR die
        % Kurzmeldung zu speichern hat den eigentlichen Simulink-Grund verdeckt.
        report = getReport(ME, 'extended', 'hyperlinks', 'off');
        warning('trainAndEvaluate:trialFailed', ...
            'Trial (%s) fehlgeschlagen: %s', char(agentType), ME.message);
        % Persistenter Fehlerlog: bei parallelen Trials erreichen Worker-warnings
        % die Client-Konsole NICHT -> ohne diese Datei bleibt der Fehler unsichtbar.
        logTrialError(opt, agentType, p, report);
        objective = 1e6;                       % harte Strafe statt Crash
        userData  = struct('error', ME.message, 'report', report, ...
                           'params', p, 'agentType', char(agentType));
    end
end

% ===== Hilfsfunktionen =====================================================

function score = extractScore(result)
% Beste deterministische Evaluationsbewertung; Fallback: mittlerer
% Trainings-Reward der letzten 25 Episoden.
    score = [];
    if isprop(result,'EvaluationStatistic') || isfield(result,'EvaluationStatistic')
        es = result.EvaluationStatistic;
        es = es(~isnan(es));
        if ~isempty(es)
            score = max(es);
        end
    end
    if isempty(score)
        er = result.EpisodeReward;
        n  = numel(er);
        score = mean(er(max(1, n-24):n));
    end
end

function s = withDefaults(s, def)
    f = fieldnames(def);
    for i = 1:numel(f)
        if ~isfield(s, f{i}) || isempty(s.(f{i}))
            s.(f{i}) = def.(f{i});
        end
    end
end

function logTrialError(opt, agentType, p, report)
% Haengt den vollstaendigen Fehlerbericht an <RunDir>/trial_errors.log an.
% Worker koennen in den gemeinsamen Lauf-Ordner schreiben -> so wird JEDER
% parallele Fehlschlag sichtbar (statt nur als stummes 1e6). Ein Fehler beim
% Loggen darf den Lauf niemals stoeren.
    if ~isstruct(opt) || ~isfield(opt,'RunDir') || isempty(opt.RunDir)
        return;
    end
    try
        fpath = fullfile(opt.RunDir, 'trial_errors.log');
        fid = fopen(fpath, 'a');
        if fid < 0, return; end
        closer = onCleanup(@() fclose(fid));
        fprintf(fid, '==== %s | agent=%s | worker=%s ====\n', ...
            char(datetime('now')), char(agentType), workerTag());
        fprintf(fid, 'params: %s\n', paramsToStr(p));
        fprintf(fid, '%s\n\n', report);
    catch
        % bewusst ignorieren
    end
end

function tag = workerTag()
    tag = 'client';
    try
        t = getCurrentTask();
        if ~isempty(t), tag = sprintf('%d', t.ID); end
    catch
        % kein Pool / kein Worker
    end
end

function s = paramsToStr(p)
    if ~isstruct(p), s = '(keine)'; return; end
    f = fieldnames(p);
    parts = strings(0,1);
    for i = 1:numel(f)
        v = p.(f{i});
        if isnumeric(v) && isscalar(v)
            parts(end+1,1) = sprintf('%s=%.6g', f{i}, v); %#ok<AGROW>
        else
            parts(end+1,1) = string(f{i}) + "=?"; %#ok<AGROW>
        end
    end
    if isempty(parts), s = '(leer)'; else, s = char(strjoin(parts, ', ')); end
end
