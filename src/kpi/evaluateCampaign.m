function T = evaluateCampaign(campaign, opt)
% evaluateCampaign  Wertet alle Agenten einer Kampagne mit denselben Startzustaenden aus.
%
%   T = evaluateCampaign("benchmark_v2")
%   T = evaluateCampaign("benchmark_v2", Agents="PPO_optimized_*", ...
%                        Condition="reduced_sat", Config=struct('tau_sat_scale', 0.75))
%
%   Startzustaende: Episode 1 = nominale Null-Pose, Episoden 2..(1+NumRandom)
%   = evalInitStates(NumRandom, range_deg, InitSeed). Alle Agenten bekommen
%   dieselben Startzustaende -> gepaarte Tests ueber Agenten hinweg.
%
%   Ergebnis: results/<campaign>/eval/<Condition>.csv, eine Zeile je Agent und
%   Episode (Spalte init = "nominal" | "random"), sowie die Tabelle T.
%   Die Startzustaende stehen in results/<campaign>/eval/init_states.csv.

    arguments
        campaign (1,1) string
        opt.Condition (1,1) string = "nominal"
        opt.Config    struct = struct()
        opt.Agents    (1,1) string = "*"
        opt.NumRandom (1,1) double {mustBeInteger, mustBeNonnegative} = 30
        opt.InitSeed  (1,1) double = 2026
        opt.Workers   (1,1) double {mustBeInteger, mustBeNonnegative} = 6
        opt.OutRoot   (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    campDir = fullfile(opt.OutRoot, campaign);
    files = dir(fullfile(campDir, 'agents', opt.Agents + ".mat"));
    if isempty(files)
        error('evaluateCampaign:noAgents', 'Keine Agenten in %s.', fullfile(campDir, 'agents'));
    end
    files = string(fullfile({files.folder}, {files.name}))';

    range_deg = benchmarkConfig().init.range_deg;
    Q = [zeros(4,1), evalInitStates(opt.NumRandom, range_deg, opt.InitSeed)];
    evalDir = fullfile(campDir, 'eval');
    if ~exist(evalDir, 'dir'), mkdir(evalDir); end
    writematrix(rad2deg(Q'), fullfile(evalDir, 'init_states.csv'));   % [deg], Zeile = Episode

    fprintf('[evaluateCampaign] %s / %s: %d Agenten x %d Episoden\n', campaign, opt.Condition, numel(files), size(Q,2));
    t0 = tic;
    parts = cell(numel(files), 1);
    if opt.Workers == 0
        for i = 1:numel(files)
            parts{i} = evaluateAgent(files(i), Q, Condition=opt.Condition, Config=opt.Config);
            fprintf('  %d/%d %s\n', i, numel(files), files(i));
        end
    else
        pool = gcp('nocreate');
        if isempty(pool) || pool.NumWorkers ~= opt.Workers
            delete(pool);
            pool = parpool('Processes', opt.Workers);
        end
        proot = char(fileparts(fileparts(fileparts(mfilename('fullpath')))));
        wait(parfevalOnAll(pool, @prepareWorker, 0, proot));
        for i = numel(files):-1:1
            F(i) = parfeval(pool, @evaluateAgent, 1, files(i), Q, ...
                'Condition', opt.Condition, 'Config', opt.Config);
        end
        for n = 1:numel(files)
            [i, Ti] = fetchNext(F);
            parts{i} = Ti;
            fprintf('  %d/%d %s (%.0f s)\n', n, numel(files), files(i), toc(t0));
        end
    end

    T = vertcat(parts{:});
    init = repmat("random", height(T), 1);
    init(T.episode == 1) = "nominal";
    T = addvars(T, init, 'After', 'episode');
    T = splitvars(T, 'q0_deg', 'NewVariableNames', {'q0_1','q0_2','q0_3','q0_4'});
    writetable(T, fullfile(evalDir, opt.Condition + ".csv"));
    fprintf('[evaluateCampaign] fertig nach %.1f min -> %s\n', toc(t0)/60, fullfile(evalDir, opt.Condition + ".csv"));
end
