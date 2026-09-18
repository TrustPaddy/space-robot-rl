function outFile = trainOne(agentType, mode, seed, opt)
% trainOne  Trainiert EINEN Agenten fuer den Benchmark und speichert ihn.
%
%   outFile = trainOne("PPO", "default", 0)
%   outFile = trainOne("PPO", "optimized", 3, Campaign="benchmark_v2")
%   outFile = trainOne("TRPO", "default", 1, Campaign="sens_wori_x2", ...
%                      Config=struct('reward', struct('wori', 400)))
%
%   Ablauf: benchmarkConfig (+ Config-Ueberschreibungen) -> Umgebung ->
%   rng(seed) -> Agent -> genau cfg.train.maxEpisodes Episoden ohne
%   Stop-Kriterium -> Speichern. Ausgewertet wird spaeter der Agent nach der
%   letzten Episode; es gibt keine Auswahl eines "besten" Checkpoints.
%
%   Ergebnis: <OutRoot>/<Campaign>/agents/<AGENT>_<mode>_s<seed>.mat mit
%     agent, stats (Reward je Episode, gleitender Mittelwert, Schritte),
%     cfg, meta (Git-Commit, Modell-Pruefsumme, MATLAB-Version),
%     wallclock [s], agentType, mode, seed.
%   Zusaetzlich eine Zeile in <OutRoot>/<Campaign>/runs.csv.

    arguments
        agentType (1,1) string
        mode      (1,1) string
        seed      (1,1) double {mustBeInteger, mustBeNonnegative}
        opt.Campaign (1,1) string = "benchmark_v2"
        opt.Config   struct = struct()
        opt.OutRoot  (1,1) string = defaultOutRoot()
        opt.LogRun   (1,1) logical = true    % Zeile in runs.csv (runCampaign schreibt selbst)
    end

    agentType = upper(agentType);
    mode      = lower(mode);

    cfg = benchmarkConfig(opt.Config);
    cfg.train.seed = seed;
    S = setupSpaceRobotEnv(cfg);

    % Ein Seed steuert Netzinitialisierung, Exploration und Startzustaende
    rng(seed, 'twister');
    agent = buildBenchmarkAgent(agentType, mode, S.obsInfo, S.actInfo, S.Ts_agent);

    trainOpts = rlTrainingOptions( ...
        MaxEpisodes                = cfg.train.maxEpisodes, ...
        MaxStepsPerEpisode         = floor(cfg.T / cfg.Ts_agent), ...
        ScoreAveragingWindowLength = 25, ...
        StopTrainingCriteria       = "none", ...
        Plots                      = "none", ...
        Verbose                    = false, ...
        UseParallel                = false);

    t0 = tic;
    result = train(agent, S.env, trainOpts);
    wallclock = toc(t0);

    stats = struct( ...
        'EpisodeReward', result.EpisodeReward(:), ...
        'AverageReward', result.AverageReward(:), ...
        'EpisodeSteps',  result.EpisodeSteps(:), ...
        'TotalAgentSteps', result.TotalAgentSteps(:));
    meta = runMeta();

    outDir = fullfile(opt.OutRoot, opt.Campaign, 'agents');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    outFile = fullfile(outDir, sprintf('%s_%s_s%d.mat', agentType, mode, seed));
    save(outFile, 'agent', 'stats', 'cfg', 'meta', 'wallclock', 'agentType', 'mode', 'seed');

    if opt.LogRun
        appendRunLog(fullfile(opt.OutRoot, opt.Campaign, 'runs.csv'), outFile);
    end
    fprintf('[trainOne] %s %s s%d: %d Episoden, %.1f min -> %s\n', agentType, mode, seed, ...
        numel(stats.EpisodeReward), wallclock/60, outFile);
end

function r = defaultOutRoot()
    r = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'));
end

