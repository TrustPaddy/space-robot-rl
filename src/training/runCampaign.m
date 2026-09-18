function runCampaign(jobs, opt)
% runCampaign  Trainiert viele Agenten parallel (ein Lauf je Worker).
%
%   jobs = table(["PPO";"TRPO"], ["default";"default"], [0;0], ...
%                'VariableNames', {'agent','mode','seed'});
%   runCampaign(jobs, Campaign="benchmark_v2", Workers=6)
%   runCampaign(jobs, Campaign="sens_wori_x2", Config=struct('reward', struct('wori', 400)))
%
%   - Fortsetzbar: Laeufe, deren Agentendatei schon existiert, werden
%     uebersprungen. Nach einem Absturz einfach denselben Aufruf wiederholen.
%   - Ein fehlgeschlagener Lauf stoppt die Kampagne nicht; der Fehler steht in
%     results/<Campaign>/errors.log.
%   - Jeder Worker bekommt einen eigenen Simulink-Cache-Ordner, damit sich
%     parallele Kompilierungen von SpaceRobot.slx nicht gegenseitig stoeren.
%   - Workers = 0 trainiert seriell im aktuellen MATLAB (zum Debuggen).

    arguments
        jobs table
        opt.Campaign (1,1) string
        opt.Config   struct = struct()
        opt.Workers  (1,1) double {mustBeInteger, mustBeNonnegative} = 6
        opt.OutRoot  (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    campDir = fullfile(opt.OutRoot, opt.Campaign);
    if ~exist(fullfile(campDir, 'agents'), 'dir'), mkdir(fullfile(campDir, 'agents')); end
    save(fullfile(campDir, 'campaign.mat'), 'jobs', 'opt');

    % Bereits erledigte Laeufe ueberspringen
    todo = false(height(jobs), 1);
    for i = 1:height(jobs)
        todo(i) = ~isfile(agentFile(campDir, jobs(i,:)));
    end
    fprintf('[runCampaign] %s: %d Laeufe, davon %d offen.\n', opt.Campaign, height(jobs), nnz(todo));
    jobs = jobs(todo, :);
    if isempty(jobs), return; end

    run = @(j) safeTrain(j.agent, j.mode, j.seed, opt);
    t0 = tic;

    if opt.Workers == 0
        for i = 1:height(jobs)
            report(run(jobs(i,:)), i, height(jobs), t0, campDir);
        end
        return;
    end

    pool = gcp('nocreate');
    if isempty(pool) || pool.NumWorkers ~= opt.Workers
        delete(pool);
        pool = parpool('Processes', opt.Workers);
    end
    proot = char(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    wait(parfevalOnAll(pool, @prepareWorker, 0, proot));

    for i = 1:height(jobs)                 % in Tabellenreihenfolge einreihen
        F(i) = parfeval(pool, run, 1, jobs(i,:)); %#ok<AGROW>
    end
    for n = 1:height(jobs)
        [~, res] = fetchNext(F);
        report(res, n, height(jobs), t0, campDir);
    end
end

function f = agentFile(campDir, j)
    f = fullfile(campDir, 'agents', sprintf('%s_%s_s%d.mat', upper(string(j.agent)), lower(string(j.mode)), j.seed));
end

function res = safeTrain(agentType, mode, seed, opt)
    res = struct('agent', string(agentType), 'mode', string(mode), 'seed', seed, 'file', "", 'error', "");
    try
        res.file = string(trainOne(agentType, mode, seed, Campaign=opt.Campaign, ...
            Config=opt.Config, OutRoot=opt.OutRoot, LogRun=false));
    catch ME
        res.error = string(getReport(ME, 'extended', 'hyperlinks', 'off'));
    end
end

function report(res, n, total, t0, campDir)
    el = toc(t0);
    eta = el / n * (total - n);
    if strlength(res.error) == 0
        appendRunLog(fullfile(campDir, 'runs.csv'), res.file);
        fprintf('[runCampaign] %d/%d fertig: %s %s s%d | %.0f min vergangen, noch ca. %.0f min\n', ...
            n, total, res.agent, res.mode, res.seed, el/60, eta/60);
    else
        fid = fopen(fullfile(campDir, 'errors.log'), 'a');
        fprintf(fid, '==== %s | %s %s s%d ====\n%s\n\n', char(datetime('now')), res.agent, res.mode, res.seed, res.error);
        fclose(fid);
        fprintf(2, '[runCampaign] %d/%d FEHLER: %s %s s%d (siehe errors.log)\n', n, total, res.agent, res.mode, res.seed);
    end
end
