function runCampaign(jobs, opt)
% runCampaign  Trainiert viele Agenten parallel (ein Lauf je Worker).
%
%   jobs = table(["PPO";"TRPO"], ["default";"default"], [0;0], ...
%                'VariableNames', {'agent','mode','seed'});
%   runCampaign(jobs, Campaign="benchmark_v2", Workers=6)
%   runCampaign(jobs, Campaign="sens_wori_x2", Config=struct('reward', struct('wori', 400)))
%
%   Mehrere Kampagnen in einem Pool (bessere Auslastung): jobs mit den
%   zusaetzlichen Spalten campaign (string) und config (cell mit Structs),
%   dann ohne Campaign/Config aufrufen (siehe partCJobs, runPartC):
%   runCampaign(jobs, Workers=6)
%
%   - Fortsetzbar: Laeufe, deren Agentendatei schon existiert, werden
%     uebersprungen. Nach einem Absturz einfach denselben Aufruf wiederholen.
%   - Ein fehlgeschlagener Lauf stoppt die Kampagne nicht; der Fehler steht in
%     results/<Campaign>/errors.log.
%   - Jeder Worker bekommt einen eigenen Simulink-Cache-Ordner, damit sich
%     parallele Kompilierungen von SpaceRobot.slx nicht gegenseitig stoeren.
%   - Workers = 0 trainiert seriell im aktuellen MATLAB (zum Debuggen).
%   - Fortschritt: trainOne schreibt je Episode eine Zeile nach
%     results/<Campaign>/progress/. Waehrend des Wartens gibt runCampaign alle
%     StatusEvery Sekunden campaignStatus aus; aus einer zweiten MATLAB-Sitzung
%     geht campaignStatus("<Campaign>") jederzeit.

    arguments
        jobs table
        opt.Campaign (1,1) string = ""
        opt.Config   struct = struct()
        opt.Workers  (1,1) double {mustBeInteger, mustBeNonnegative} = 6
        opt.StatusEvery (1,1) double {mustBePositive} = 600
        opt.OutRoot  (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    % Kampagne und Config je Job (Spalten) oder fuer alle (Optionen)
    if ~ismember('campaign', jobs.Properties.VariableNames)
        if strlength(opt.Campaign) == 0
            error('runCampaign:campaign', 'Campaign angeben oder Spalte jobs.campaign verwenden.');
        end
        jobs.campaign = repmat(opt.Campaign, height(jobs), 1);
    end
    if ~ismember('config', jobs.Properties.VariableNames)
        jobs.config = repmat({opt.Config}, height(jobs), 1);
    end
    jobs.campaign = string(jobs.campaign);

    % campaign.mat je Kampagne: Jobliste und Config (fuer campaignStatus und
    % als Dokumentation). Jobs einer Kampagne muessen dieselbe Config haben.
    camps = unique(jobs.campaign, 'stable');
    for c = camps'
        sel  = jobs.campaign == c;
        cfgs = jobs.config(sel);
        if any(cellfun(@(x) ~isequal(x, cfgs{1}), cfgs))
            error('runCampaign:config', 'Kampagne %s: Jobs mit unterschiedlicher Config.', c);
        end
        campDir = fullfile(opt.OutRoot, c);
        if ~exist(fullfile(campDir, 'agents'), 'dir'), mkdir(fullfile(campDir, 'agents')); end
        saveCampaign(campDir, jobs(sel, {'agent','mode','seed'}), cfgs{1}, opt.Workers);
    end

    % Bereits erledigte Laeufe ueberspringen
    todo = false(height(jobs), 1);
    for i = 1:height(jobs)
        todo(i) = ~isfile(agentFile(fullfile(opt.OutRoot, jobs.campaign(i)), jobs(i,:)));
    end
    fprintf('[runCampaign] %s: %d Laeufe, davon %d offen.\n', strjoin(camps, ', '), height(jobs), nnz(todo));
    jobs = jobs(todo, :);
    if isempty(jobs), return; end

    run = @(j) safeTrain(j.agent, j.mode, j.seed, j.campaign, j.config{1}, opt.OutRoot);
    t0 = tic;

    if opt.Workers == 0
        for i = 1:height(jobs)
            report(run(jobs(i,:)), i, height(jobs), t0, opt.OutRoot);
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
    running = unique(jobs.campaign, 'stable');
    n = 0;
    while n < height(jobs)
        [idx, res] = fetchNext(F, opt.StatusEvery);
        if isempty(idx)                    % Zeitgrenze: Zwischenstand ausgeben
            for c = running'
                try
                    campaignStatus(c, OutRoot=opt.OutRoot);
                catch ME
                    fprintf(2, '[runCampaign] Status %s nicht lesbar: %s\n', c, ME.message);
                end
            end
            continue;
        end
        n = n + 1;
        report(res, n, height(jobs), t0, opt.OutRoot);
    end
end

function saveCampaign(campDir, jobs, config, workers)
% campaign.mat: bei erneutem Aufruf werden neue Jobs ergaenzt, nicht ersetzt.
    f = fullfile(campDir, 'campaign.mat');
    if isfile(f)
        old = load(f, 'jobs');
        if isfield(old, 'jobs')
            jobs = unique([old.jobs(:, {'agent','mode','seed'}); jobs], 'rows', 'stable');
        end
    end
    opt = struct('Config', config, 'Workers', workers); %#ok<NASGU>
    save(f, 'jobs', 'opt');
end

function f = agentFile(campDir, j)
    f = fullfile(campDir, 'agents', sprintf('%s_%s_s%d.mat', upper(string(j.agent)), lower(string(j.mode)), j.seed));
end

function res = safeTrain(agentType, mode, seed, campaign, config, outRoot)
    res = struct('agent', string(agentType), 'mode', string(mode), 'seed', seed, ...
                 'campaign', string(campaign), 'file', "", 'error', "");
    try
        res.file = string(trainOne(agentType, mode, seed, Campaign=campaign, ...
            Config=config, OutRoot=outRoot, LogRun=false));
    catch ME
        res.error = string(getReport(ME, 'extended', 'hyperlinks', 'off'));
    end
end

function report(res, n, total, t0, outRoot)
    el = toc(t0);
    eta = el / n * (total - n);
    campDir = fullfile(outRoot, res.campaign);
    if strlength(res.error) == 0
        appendRunLog(fullfile(campDir, 'runs.csv'), res.file);
        fprintf('[runCampaign] %d/%d fertig: %s %s %s s%d | %.0f min vergangen, noch ca. %.0f min\n', ...
            n, total, res.campaign, res.agent, res.mode, res.seed, el/60, eta/60);
    else
        fid = fopen(fullfile(campDir, 'errors.log'), 'a');
        fprintf(fid, '==== %s | %s %s s%d ====\n%s\n\n', char(datetime('now')), res.agent, res.mode, res.seed, res.error);
        fclose(fid);
        fprintf(2, '[runCampaign] %d/%d FEHLER: %s %s %s s%d (siehe errors.log)\n', n, total, ...
            res.campaign, res.agent, res.mode, res.seed);
    end
end
