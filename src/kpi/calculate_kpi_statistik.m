clc; clear; close all;

% Statistical evaluation script for reviewer comments R2.12, R2.13, R3.7.
% The key point is: keep one KPI row per evaluation episode.
% Do not only store the already averaged KPI values.

%% ------------------------------------------------------------------------
%  Configuration
% -------------------------------------------------------------------------

mdl = 'SpaceRobot';
trajectoryMode = "circle";   % "circle" or "piecewise_linear"
N_eval = 30;                  % Use 30 to match the manuscript protocol.
evalSeeds = 1000 + (1:N_eval);

outputDir = fullfile(pwd, 'EvaluationResults');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

agents = struct( ...
    'name', {'TRPO','TD3','SAC','PPO','PG','DDPG'}, ...
    'path', { ...
        'SavedAgents/Circular/Default/TRPO.mat', ...
        'SavedAgents/Circular/Default/TD3.mat', ...
        'SavedAgents/Circular/Default/SAC.mat', ...
        'SavedAgents/Circular/Default/PPO.mat', ...
        'SavedAgents/Circular/Default/PG.mat', ...
        'SavedAgents/Circular/Default/DDPG.mat' ...
    } ...
);

% Statistical tests:
% true  -> Friedman + paired Wilcoxon signed-rank tests against baselineAgent.
% false -> Kruskal-Wallis + Mann-Whitney U tests against baselineAgent.
usePairedTests = true;
baselineAgent = "PPO";

% KPI variables expected from computeKPIsFromLogs.
kpiNames = "K" + string(1:9);
kpiNames = cellstr(kpiNames);

%% ------------------------------------------------------------------------
%  Safety and simulation parameters
% -------------------------------------------------------------------------

d_safe   = 0.02;
tau_max  = 2.0;
dt_agent = 0.05;
q1_lim   = deg2rad([-85 85]);
qi_lim   = deg2rad([-170 170]);

reward_init = 0;
isdone_init = 0;

assignin('base', 'd_safe', d_safe);
assignin('base', 'tau_max', tau_max);
assignin('base', 'dt_agent', dt_agent);
assignin('base', 'q1_lim', q1_lim);
assignin('base', 'qi_lim', qi_lim);
assignin('base', 'reward_init', reward_init);
assignin('base', 'isdone_init', isdone_init);

params.tau_max = tau_max;
params.q_min   = [q1_lim(1), qi_lim(1), qi_lim(1), qi_lim(1)];
params.q_max   = [q1_lim(2), qi_lim(2), qi_lim(2), qi_lim(2)];

%% ------------------------------------------------------------------------
%  Reference trajectory
% -------------------------------------------------------------------------

T = 8.5;
Ts = 0.01;
Ts_agent = 0.1; %#ok<NASGU>
t = 0:Ts:T;

r = 0.5;
center = [4.5 - r, 0.0, 0.0];

switch trajectoryMode
    case "circle"
        omega = pi / T;
        x = center(1) + r * cos(omega * t);
        y = center(2) + r * sin(omega * t);
        z = center(3) + 0 * t;

    case "piecewise_linear"
        P0 = [center(1) + r, center(2),     center(3)];
        P1 = [center(1),     center(2) + r, center(3)];
        P2 = [center(1) - r, center(2),     center(3)];
        t1 = T / 2;

        x = zeros(size(t));
        y = zeros(size(t));
        z = zeros(size(t));

        idx1 = (t <= t1);
        s1 = t(idx1) / t1;
        x(idx1) = P0(1) + s1 * (P1(1) - P0(1));
        y(idx1) = P0(2) + s1 * (P1(2) - P0(2));
        z(idx1) = P0(3) + s1 * (P1(3) - P0(3));

        idx2 = (t > t1);
        s2 = (t(idx2) - t1) / (T - t1);
        x(idx2) = P1(1) + s2 * (P2(1) - P1(1));
        y(idx2) = P1(2) + s2 * (P2(2) - P1(2));
        z(idx2) = P1(3) + s2 * (P2(3) - P1(3));

    otherwise
        error('Unknown trajectoryMode: %s', trajectoryMode);
end

traj = [x' y' z'];
dt = mean(diff(t));
vref = [zeros(1, 3); diff(traj) / dt];

EE_ref = timeseries(traj, t);
EE_vref = timeseries(vref, t);
assignin('base', 'EE_ref', EE_ref);
assignin('base', 'EE_vref', EE_vref);

% Keep this import if your model or KPI function expects the robot object.
if exist('SpaceRobot.urdf', 'file')
    robot_rbt = importrobot('SpaceRobot.urdf'); %#ok<NASGU>
    robot_rbt.DataFormat = 'row';
    eeBodyName = robot_rbt.BodyNames{end}; %#ok<NASGU>
end

load_system(mdl);
set_param(mdl, ...
    'StopTime', num2str(T), ...
    'Solver', 'ode4', ...
    'FixedStep', num2str(Ts), ...
    'SolverType', 'Fixed-step');

if exist('computeKPIsFromLogs', 'file') ~= 2
    error(['computeKPIsFromLogs was not found on the MATLAB path. ', ...
        'Put the KPI function on the path before running this script.']);
end

%% ------------------------------------------------------------------------
%  Evaluation loop: one row per agent and episode
% -------------------------------------------------------------------------

episodeKpis = table();

for a = 1:numel(agents)
    agentInfo = agents(a);
    fprintf('\nEvaluating %s (%s)\n', agentInfo.name, agentInfo.path);

    loaded = load(agentInfo.path, 'agent');
    agent = loaded.agent; %#ok<NASGU>
    assignin('base', 'agent', agent);

    logsouts = cell(N_eval, 1);

    for ep = 1:N_eval
        rng(evalSeeds(ep), 'twister');
        fprintf('  Episode %02d/%02d, seed %d\n', ep, N_eval, evalSeeds(ep));

        simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
        logsouts{ep} = simOut.logsout;

        kpiOne = computeKPIsFromLogs(logsouts(ep), params);
        row = normalizeKpiRow(kpiOne, agentInfo.name, ep, evalSeeds(ep), kpiNames);
        episodeKpis = [episodeKpis; row]; %#ok<AGROW>
    end

    save(fullfile(outputDir, sprintf('logsouts_%s.mat', sanitizeFileName(agentInfo.name))), ...
        'logsouts', '-v7.3');
end

writetable(episodeKpis, fullfile(outputDir, 'kpi_episode_values.csv'));
save(fullfile(outputDir, 'kpi_episode_values.mat'), 'episodeKpis');

%% ------------------------------------------------------------------------
%  Summary statistics, plots, and significance tests
% -------------------------------------------------------------------------

summaryStats = summarizeKpis(episodeKpis, kpiNames);
writetable(summaryStats, fullfile(outputDir, 'kpi_summary_stats.csv'));

plotKpiDistributions(episodeKpis, {'K2','K4','K7','K9'}, outputDir);
plotKpiConfidenceIntervals(summaryStats, {'K2','K4','K7','K9'}, outputDir);

significance = runSignificanceTests(episodeKpis, kpiNames, baselineAgent, usePairedTests);
writetable(significance, fullfile(outputDir, 'kpi_significance_tests.csv'));

disp('Done. Created:');
disp(fullfile(outputDir, 'kpi_episode_values.csv'));
disp(fullfile(outputDir, 'kpi_summary_stats.csv'));
disp(fullfile(outputDir, 'kpi_significance_tests.csv'));

%% ------------------------------------------------------------------------
%  Local helper functions
% -------------------------------------------------------------------------

function row = normalizeKpiRow(kpiOne, agentName, episode, seed, kpiNames)
    values = nan(1, numel(kpiNames));

    if istable(kpiOne)
        for i = 1:numel(kpiNames)
            values(i) = firstScalarFromTable(kpiOne, kpiNames{i});
        end
    elseif isstruct(kpiOne)
        for i = 1:numel(kpiNames)
            values(i) = firstScalarFromStruct(kpiOne, kpiNames{i});
        end
    elseif isnumeric(kpiOne)
        tmp = kpiOne(:)';
        values(1:min(numel(tmp), numel(values))) = tmp(1:min(numel(tmp), numel(values)));
    else
        error('Unsupported KPI output type: %s', class(kpiOne));
    end

    row = table(string(agentName), episode, seed, ...
        values(1), values(2), values(3), values(4), values(5), ...
        values(6), values(7), values(8), values(9), ...
        'VariableNames', [{'agent','episode','seed'}, kpiNames]);
end

function value = firstScalarFromTable(T, wantedName)
    value = nan;
    names = string(T.Properties.VariableNames);

    metricCols = find(contains(lower(names), "metric") | contains(lower(names), "kpi"), 1);
    valueCols = find(contains(lower(names), "value") | contains(lower(names), "mean") | contains(lower(names), "avg"), 1);
    if ~isempty(metricCols) && ~isempty(valueCols)
        metricValues = string(T{:, metricCols});
        rowIdx = find(strcmpi(metricValues, wantedName) | contains(lower(metricValues), lower(wantedName)), 1);
        if ~isempty(rowIdx)
            raw = T{rowIdx, valueCols};
            if isnumeric(raw) || islogical(raw)
                value = firstFiniteScalar(raw);
                return;
            end
        end
    end

    idx = find(strcmpi(names, wantedName), 1);
    if isempty(idx)
        idx = find(contains(lower(names), lower(wantedName)), 1);
    end
    if isempty(idx)
        return;
    end

    raw = T{:, idx};
    if isnumeric(raw) || islogical(raw)
        value = firstFiniteScalar(raw);
    end
end

function value = firstScalarFromStruct(S, wantedName)
    value = nan;
    names = string(fieldnames(S));

    idx = find(strcmpi(names, wantedName), 1);
    if isempty(idx)
        idx = find(contains(lower(names), lower(wantedName)), 1);
    end
    if isempty(idx)
        return;
    end

    raw = S.(char(names(idx)));
    if isnumeric(raw) || islogical(raw)
        value = firstFiniteScalar(raw);
    end
end

function value = firstFiniteScalar(raw)
    raw = raw(:);
    raw = raw(isfinite(raw));
    if isempty(raw)
        value = nan;
    else
        value = raw(1);
    end
end

function summaryStats = summarizeKpis(T, kpiNames)
    agents = unique(T.agent, 'stable');
    rows = table();

    for a = 1:numel(agents)
        agentName = agents(a);
        idxAgent = T.agent == agentName;

        for k = 1:numel(kpiNames)
            kpiName = kpiNames{k};
            x = T{idxAgent, kpiName};
            x = x(isfinite(x));

            if isempty(x)
                statsRow = table(agentName, string(kpiName), 0, nan, nan, nan, nan, nan, nan, nan, ...
                    'VariableNames', {'agent','kpi','n','mean','std','median','iqr','min','max','ci95_low'});
                statsRow.ci95_high = nan;
            else
                [ciLow, ciHigh] = bootstrapMeanCI(x, 5000, 0.05);
                statsRow = table(agentName, string(kpiName), numel(x), ...
                    mean(x, 'omitnan'), std(x, 0, 'omitnan'), median(x, 'omitnan'), ...
                    localIqr(x), min(x), max(x), ciLow, ciHigh, ...
                    'VariableNames', {'agent','kpi','n','mean','std','median','iqr','min','max','ci95_low','ci95_high'});
            end

            rows = [rows; statsRow]; %#ok<AGROW>
        end
    end

    summaryStats = rows;
end

function [ciLow, ciHigh] = bootstrapMeanCI(x, nBoot, alpha)
    x = x(:);
    x = x(isfinite(x));

    if numel(x) < 2
        ciLow = mean(x, 'omitnan');
        ciHigh = ciLow;
        return;
    end

    n = numel(x);
    bootMeans = nan(nBoot, 1);

    for b = 1:nBoot
        sampleIdx = randi(n, n, 1);
        bootMeans(b) = mean(x(sampleIdx), 'omitnan');
    end

    ciLow = localPercentile(bootMeans, 100 * alpha / 2);
    ciHigh = localPercentile(bootMeans, 100 * (1 - alpha / 2));
end

function spread = localIqr(x)
    spread = localPercentile(x, 75) - localPercentile(x, 25);
end

function q = localPercentile(x, p)
    x = sort(x(:));
    x = x(isfinite(x));

    if isempty(x)
        q = nan;
        return;
    end

    if numel(x) == 1
        q = x(1);
        return;
    end

    rank = 1 + (numel(x) - 1) * (p / 100);
    lo = floor(rank);
    hi = ceil(rank);
    if lo == hi
        q = x(lo);
    else
        weight = rank - lo;
        q = (1 - weight) * x(lo) + weight * x(hi);
    end
end

function plotKpiDistributions(T, kpiNames, outputDir)
    for k = 1:numel(kpiNames)
        kpiName = kpiNames{k};

        fig = figure('Color', 'w', 'Visible', 'off');
        if exist('boxchart', 'file') == 2
            boxchart(categorical(T.agent), T.(kpiName));
        elseif exist('boxplot', 'file') == 2
            boxplot(T.(kpiName), categorical(T.agent));
        else
            error('Neither boxchart nor boxplot is available.');
        end
        grid on;
        xlabel('Agent');
        ylabel(kpiName);
        title(sprintf('%s distribution across evaluation episodes', kpiName));
        exportgraphics(fig, fullfile(outputDir, sprintf('boxplot_%s.png', kpiName)), 'Resolution', 300);
        close(fig);
    end
end

function plotKpiConfidenceIntervals(summaryStats, kpiNames, outputDir)
    for k = 1:numel(kpiNames)
        kpiName = string(kpiNames{k});
        S = summaryStats(summaryStats.kpi == kpiName, :);

        fig = figure('Color', 'w', 'Visible', 'off');
        x = 1:height(S);
        y = S.mean;
        lo = S.mean - S.ci95_low;
        hi = S.ci95_high - S.mean;
        errorbar(x, y, lo, hi, 'o', 'LineWidth', 1.2, 'MarkerSize', 6);
        grid on;
        xlim([0.5, height(S) + 0.5]);
        xticks(x);
        xticklabels(S.agent);
        xlabel('Agent');
        ylabel(sprintf('%s mean with 95%% CI', kpiName));
        title(sprintf('%s mean and 95%% confidence interval', kpiName));
        exportgraphics(fig, fullfile(outputDir, sprintf('ci_%s.png', kpiName)), 'Resolution', 300);
        close(fig);
    end
end

function results = runSignificanceTests(T, kpiNames, baselineAgent, usePairedTests)
    results = table();
    agents = unique(T.agent, 'stable');

    if ~any(agents == baselineAgent)
        baselineAgent = agents(1);
        warning('Baseline agent not found. Using %s instead.', baselineAgent);
    end

    for k = 1:numel(kpiNames)
        kpiName = kpiNames{k};

        if usePairedTests
            [X, agentOrder] = buildPairedMatrix(T, kpiName);
            validRows = all(isfinite(X), 2);
            X = X(validRows, :);
            agentOrder = string(agentOrder);

            if size(X, 1) >= 2 && size(X, 2) >= 2 && exist('friedman', 'file') == 2
                pGlobal = friedman(X, 1, 'off');
                globalTest = "Friedman";
            else
                pGlobal = nan;
                globalTest = "Friedman unavailable";
            end

            baseIdx = find(agentOrder == baselineAgent, 1);
            if isempty(baseIdx)
                baseIdx = 1;
            end

            pRaw = nan(numel(agentOrder), 1);
            testName = strings(numel(agentOrder), 1);
            effect = nan(numel(agentOrder), 1);

            for a = 1:numel(agentOrder)
                if a == baseIdx
                    pRaw(a) = nan;
                    testName(a) = "baseline";
                    effect(a) = 0;
                    continue;
                end

                if exist('signrank', 'file') == 2
                    pRaw(a) = signrank(X(:, baseIdx), X(:, a));
                    testName(a) = "Wilcoxon signed-rank";
                else
                    pRaw(a) = nan;
                    testName(a) = "signrank unavailable";
                end
                effect(a) = median(X(:, a) - X(:, baseIdx), 'omitnan');
            end

            pAdj = holmAdjust(pRaw);
            for a = 1:numel(agentOrder)
                row = table(string(kpiName), globalTest, pGlobal, string(baselineAgent), ...
                    string(agentOrder(a)), testName(a), pRaw(a), pAdj(a), effect(a), ...
                    'VariableNames', {'kpi','global_test','global_p','baseline','comparison_agent','pairwise_test','p_raw','p_holm','effect_median_difference'});
                results = [results; row]; %#ok<AGROW>
            end

        else
            x = T.(kpiName);
            g = categorical(T.agent);

            if exist('kruskalwallis', 'file') == 2
                pGlobal = kruskalwallis(x, g, 'off');
                globalTest = "Kruskal-Wallis";
            else
                pGlobal = nan;
                globalTest = "Kruskal-Wallis unavailable";
            end

            baseValues = T{T.agent == baselineAgent, kpiName};
            pRaw = nan(numel(agents), 1);
            testName = strings(numel(agents), 1);
            effect = nan(numel(agents), 1);

            for a = 1:numel(agents)
                cmpValues = T{T.agent == agents(a), kpiName};
                if agents(a) == baselineAgent
                    pRaw(a) = nan;
                    testName(a) = "baseline";
                    effect(a) = 0;
                    continue;
                end

                if exist('ranksum', 'file') == 2
                    pRaw(a) = ranksum(baseValues, cmpValues);
                    testName(a) = "Mann-Whitney U";
                else
                    pRaw(a) = nan;
                    testName(a) = "ranksum unavailable";
                end
                effect(a) = cliffsDelta(baseValues, cmpValues);
            end

            pAdj = holmAdjust(pRaw);
            for a = 1:numel(agents)
                row = table(string(kpiName), globalTest, pGlobal, string(baselineAgent), ...
                    string(agents(a)), testName(a), pRaw(a), pAdj(a), effect(a), ...
                    'VariableNames', {'kpi','global_test','global_p','baseline','comparison_agent','pairwise_test','p_raw','p_holm','effect_median_difference'});
                results = [results; row]; %#ok<AGROW>
            end
        end
    end
end

function [X, agentOrder] = buildPairedMatrix(T, kpiName)
    agentOrder = unique(T.agent, 'stable');
    seeds = unique(T.seed, 'stable');
    X = nan(numel(seeds), numel(agentOrder));

    for s = 1:numel(seeds)
        for a = 1:numel(agentOrder)
            idx = T.seed == seeds(s) & T.agent == agentOrder(a);
            vals = T{idx, kpiName};
            if ~isempty(vals)
                X(s, a) = vals(1);
            end
        end
    end
end

function pAdj = holmAdjust(pRaw)
    pAdj = nan(size(pRaw));
    valid = find(isfinite(pRaw));
    if isempty(valid)
        return;
    end

    [pSorted, order] = sort(pRaw(valid), 'ascend');
    m = numel(pSorted);
    adjustedSorted = nan(size(pSorted));

    for i = 1:m
        adjustedSorted(i) = min(1, (m - i + 1) * pSorted(i));
    end
    adjustedSorted = cummax(adjustedSorted);

    pAdj(valid(order)) = adjustedSorted;
end

function delta = cliffsDelta(x, y)
    x = x(:);
    y = y(:);
    x = x(isfinite(x));
    y = y(isfinite(y));

    if isempty(x) || isempty(y)
        delta = nan;
        return;
    end

    greater = 0;
    less = 0;
    for i = 1:numel(x)
        greater = greater + sum(x(i) > y);
        less = less + sum(x(i) < y);
    end
    delta = (greater - less) / (numel(x) * numel(y));
end

function safeName = sanitizeFileName(name)
    safeName = regexprep(char(name), '[^a-zA-Z0-9_-]', '_');
end
