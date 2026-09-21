function R = analyzeBenchmark(campaign, opt)
% analyzeBenchmark  Auswertung einer Kampagne: Robustheit und Performance getrennt.
%
%   R = analyzeBenchmark("benchmark_v2")
%   R = analyzeBenchmark("benchmark_v2", Condition="noise")
%   R = analyzeBenchmark("twoseg", Baseline="TRPO_default")
%
%   Grundlage: results/<campaign>/eval/<Condition>.csv (evaluateCampaign) und
%   die Trainingsstatistik in results/<campaign>/agents/*.mat.
%
%   Ein Lauf ist ein Seed einer Konfiguration. Er gilt als ERFOLGREICH, wenn
%   keine seiner Auswertungsepisoden durch Kollision oder Gelenkgrenze
%   abbricht. Abbruchrate und KPIs werden getrennt berichtet, damit nicht
%   Robustheit und erreichte Guete vermischt werden (Absprache 20.09.2026):
%
%     Tabelle 1 (Robustheit, ALLE Laeufe): erfolgreiche Laeufe, Abbruchrate
%       ueber alle Auswertungsepisoden, Anteil der Abbrueche durch Kollision.
%     Tabelle 2 (Performance, NUR erfolgreiche Laeufe): je KPI Mittelwert und
%       Standardabweichung ueber die erfolgreichen Laeufe, dazu T1 und T2.
%       Die Werte sind also bedingt auf einen erfolgreichen Lauf.
%
%   Aggregation je Lauf: Mittel ueber die zufaelligen Startzustaende, K3 als
%   Maximum, K5 als Anteil der Episoden mit Kollision. T1 ist die
%   Standardabweichung des Episodenrewards in den letzten 100 Trainingsepisoden,
%   T2 die Trainingsdauer [min].
%
%   Tests (nur Konfigurationen mit mindestens MinSuccess erfolgreichen Laeufen):
%     - KPIs: zweiseitiger Mann-Whitney-U-Test auf Seed-Ebene gegen Baseline,
%       Holm-Korrektur ueber die Konfigurationen je KPI.
%     - Abbruchrate: exakter Test nach Fisher (erfolgreich/abgebrochen) gegen
%       Baseline, ebenfalls Holm-korrigiert.
%
%   Ausgabe in results/<campaign>/analysis/ (Praefix je Condition):
%     runs.csv, table1_robustness.csv/.tex, table2_performance.csv/.tex,
%     table2_full.csv, tests_kpi.csv, tests_abort.csv

    arguments
        campaign (1,1) string
        opt.Condition  (1,1) string = "nominal"
        opt.Baseline   (1,1) string = "TRPO_default"
        opt.MinSuccess (1,1) double {mustBeInteger, mustBePositive} = 4
        opt.Configs    (1,:) string = string.empty   % Auswahl, z. B. nur die Default-Agenten
        opt.Tag        (1,1) string = ""             % Zusatz im Dateinamen, z. B. "defaults"
        opt.OutRoot    (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    campDir = fullfile(opt.OutRoot, campaign);
    E = readtable(fullfile(campDir, 'eval', opt.Condition + ".csv"), 'TextType', 'string');
    E.config = E.agent + "_" + E.mode;
    if ~isempty(opt.Configs)
        % Die Holm-Korrektur umfasst nur die ausgewaehlten Konfigurationen
        E = E(ismember(E.config, opt.Configs), :);
    end
    kpis = "K" + (1:9);
    base = opt.Baseline;
    if ~contains(base, "_"), base = base + "_default"; end

    % ---------- Trainingsstatistik je Lauf ----------
    tr = trainingStats(campDir);

    % ---------- Ebene Lauf (config, seed) ----------
    runs = runLevel(E, kpis);
    runs = outerjoin(runs, tr, 'Keys', {'config','seed'}, 'MergeKeys', true, 'Type', 'left');
    configs = unique(runs.config, 'stable');

    % ---------- Tabelle 1: Robustheit ----------
    n = numel(configs);
    [nRuns, nSuccess, nEpisodes, nAbort] = deal(zeros(n,1));
    abortCollisionShare = zeros(n,1);
    for i = 1:n
        r = runs(runs.config == configs(i), :);
        nRuns(i)     = height(r);
        nSuccess(i)  = nnz(r.success);
        nEpisodes(i) = sum(r.nEpisodes);
        nAbort(i)    = sum(r.nAbort);
        abortCollisionShare(i) = sum(r.nAbortCollision) / max(1, sum(r.nAbort));
    end
    t1 = table(configs, nRuns, nSuccess, nSuccess ./ nRuns, nEpisodes, nAbort, ...
        nAbort ./ nEpisodes, abortCollisionShare, 'VariableNames', ...
        {'config','nRuns','nSuccess','successRate','nEpisodes','nAbort','abortRate','abortCollisionShare'});
    t1 = sortrows(t1, {'nSuccess','abortRate'}, {'descend','ascend'});

    % ---------- Tabelle 2: Performance der erfolgreichen Laeufe ----------
    cols = [kpis, "T1", "T2"];
    t2 = table(configs, zeros(n,1), 'VariableNames', {'config','nSuccess'});
    for c = cols
        [m, s] = deal(nan(n,1));
        for i = 1:n
            v = runs.(c)(runs.config == configs(i) & runs.success);
            t2.nSuccess(i) = numel(v);
            m(i) = meanOrNaN(v);
            s(i) = stdOrNaN(v);
        end
        t2.(c + "_mean") = m;
        t2.(c + "_std")  = s;
    end
    t2 = sortrows(t2, 'K2_mean');

    % ---------- Tests ----------
    tests  = kpiTests(runs, configs, base, kpis, opt.MinSuccess);
    aborts = abortTests(t1, base);

    % ---------- Ausgabe ----------
    outDir = fullfile(campDir, 'analysis');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    tag = opt.Condition;
    if strlength(opt.Tag) > 0, tag = tag + "_" + opt.Tag; end
    pre = @(name) fullfile(outDir, tag + "_" + name);
    writetable(runs, pre('runs.csv'));
    writetable(t1,   pre('table1_robustness.csv'));
    writetable(t2(:, {'config','nSuccess','K2_mean','K2_std','K4_mean','K4_std','T2_mean','T2_std'}), ...
               pre('table2_performance.csv'));
    writetable(t2,   pre('table2_full.csv'));
    writetable(tests,  pre('tests_kpi.csv'));
    writetable(aborts, pre('tests_abort.csv'));
    writeTable1Tex(pre('table1_robustness.tex'), t1, opt.Condition);
    writeTable2Tex(pre('table2_performance.tex'), t2, opt.Condition);
    writeTable2FullTex(pre('table2_full.tex'), t2, cols, opt.Condition);

    R = struct('runs', runs, 'table1', t1, 'table2', t2, 'tests', tests, 'abortTests', aborts, ...
               'baseline', base, 'condition', opt.Condition);
    fprintf('[analyzeBenchmark] %s / %s: %d Konfigurationen, %d Laeufe (%d erfolgreich) -> %s\n', ...
        campaign, opt.Condition, numel(configs), height(runs), nnz(runs.success), outDir);
end

% ================================ Ebene Lauf ================================

function runs = runLevel(E, kpis)
% Eine Zeile je Lauf (config, seed): Abbrueche ueber ALLE Episoden, KPIs ueber
% die zufaelligen Startzustaende.
    key = unique([E.config, string(E.seed)], 'rows', 'stable');
    runs = table();
    for i = 1:size(key, 1)
        cfg = key(i,1); sd = str2double(key(i,2));
        all_ = E(E.config == cfg & E.seed == sd, :);
        rnd  = all_(all_.init == "random", :);
        term = logical(all_.terminated);
        row = table(cfg, sd, height(all_), nnz(term), nnz(term & all_.K5 > 0), ~any(term), ...
            'VariableNames', {'config','seed','nEpisodes','nAbort','nAbortCollision','success'});
        for k = kpis
            switch k
                case "K3", row.(k) = max(rnd.(k));      % Spitzenfehler
                case "K5", row.(k) = mean(rnd.(k));     % Anteil Episoden mit Kollision
                otherwise, row.(k) = mean(rnd.(k));
            end
        end
        runs = [runs; row]; %#ok<AGROW>
    end
end

function tr = trainingStats(campDir)
% Trainingsstatistik aller Agentendateien (auch nicht ausgewaehlter, der
% Left-Join mit den Laeufen verwirft die ueberzaehligen).
% T1 (Streuung des Rewards in den letzten 100 Episoden) und T2 (Dauer [min]).
    files = dir(fullfile(campDir, 'agents', '*.mat'));
    tr = table(strings(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
               'VariableNames', {'config','seed','T1','T2'});
    for i = 1:numel(files)
        L = load(fullfile(files(i).folder, files(i).name), 'stats', 'wallclock', 'agentType', 'mode', 'seed');
        er = L.stats.EpisodeReward;
        tr = [tr; table(string(L.agentType) + "_" + string(L.mode), L.seed, ...
            std(er(max(1, end-99):end)), L.wallclock/60, ...
            'VariableNames', tr.Properties.VariableNames)]; %#ok<AGROW>
    end
end

% =================================== Tests ==================================

function T = kpiTests(runs, configs, base, kpis, minSuccess)
% Mann-Whitney-U auf Seed-Ebene, nur erfolgreiche Laeufe, Holm je KPI.
    T = table();
    sb = runs(runs.config == base & runs.success, :);
    others = configs(configs ~= base);
    for k = kpis
        p = nan(numel(others), 1); d = p; n2 = p;
        for j = 1:numel(others)
            so = runs(runs.config == others(j) & runs.success, :);
            n2(j) = height(so);
            d(j)  = medianOrNaN(so.(k)) - medianOrNaN(sb.(k));
            if height(sb) >= minSuccess && height(so) >= minSuccess
                p(j) = ranksum(sb.(k), so.(k));
            end
        end
        T = [T; table(repmat(k, numel(others), 1), repmat(string(base), numel(others), 1), others, ...
            repmat(height(sb), numel(others), 1), n2, d, p, holm(p), ...
            'VariableNames', {'kpi','baseline','config','n_baseline','n_config', ...
                              'median_diff_vs_baseline','p_raw','p_holm'})]; %#ok<AGROW>
    end
end

function T = abortTests(t1, base)
% Exakter Test nach Fisher: erfolgreiche gegen abgebrochene Laeufe.
    b = t1(t1.config == base, :);
    others = t1.config(t1.config ~= base);
    p = nan(numel(others), 1);
    for j = 1:numel(others)
        o = t1(t1.config == others(j), :);
        M = [b.nSuccess, b.nRuns - b.nSuccess; o.nSuccess, o.nRuns - o.nSuccess];
        try
            [~, p(j)] = fishertest(M);      % erste Ausgabe ist die Testentscheidung
        catch
            p(j) = NaN;
        end
    end
    T = table(repmat(string(base), numel(others), 1), others, p, holm(p), ...
        'VariableNames', {'baseline','config','p_raw','p_holm'});
end

function pAdj = holm(p)
% Holm-Korrektur, NaN bleibt NaN (Test nicht durchgefuehrt).
    pAdj = nan(size(p));
    ok = find(~isnan(p));
    if isempty(ok), return; end
    [ps, idx] = sort(p(ok));
    m = numel(ps);
    adj = min(1, cummax((m - (1:m)' + 1) .* ps(:)));
    pAdj(ok(idx)) = adj;
end

% ============================== LaTeX-Ausgabe ===============================

function writeTable1Tex(f, t1, cond)
    fid = fopen(f, 'w');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '%% analyzeBenchmark.m, Bedingung %s: Robustheit ueber alle Laeufe\n', cond);
    fprintf(fid, '\\begin{tabular}{lccc}\n\\toprule\n');
    fprintf(fid, 'Configuration & Successful runs & Abort rate & Aborts by collision \\\\\n\\midrule\n');
    for i = 1:height(t1)
        fprintf(fid, '%s & %d/%d & %.1f\\%% & %.0f\\%% \\\\\n', pretty(t1.config(i)), ...
            t1.nSuccess(i), t1.nRuns(i), 100*t1.abortRate(i), 100*t1.abortCollisionShare(i));
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end

function writeTable2Tex(f, t2, cond)
    fid = fopen(f, 'w');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '%% analyzeBenchmark.m, Bedingung %s: nur erfolgreiche Laeufe\n', cond);
    fprintf(fid, '\\begin{tabular}{lcccc}\n\\toprule\n');
    fprintf(fid, 'Configuration & Successful runs & $K_2$ & $K_4$ & $T_2$ [min] \\\\\n\\midrule\n');
    for i = 1:height(t2)
        if t2.nSuccess(i) == 0
            fprintf(fid, '%s & 0 & -- & -- & -- \\\\\n', pretty(t2.config(i)));
            continue;
        end
        fprintf(fid, '%s & %d & %s & %s & %.1f \\\\\n', pretty(t2.config(i)), t2.nSuccess(i), ...
            pm(t2.K2_mean(i), t2.K2_std(i), 4), pm(t2.K4_mean(i), t2.K4_std(i), 3), t2.T2_mean(i));
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end

function writeTable2FullTex(f, t2, cols, cond)
    fid = fopen(f, 'w');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '%% analyzeBenchmark.m, Bedingung %s: alle KPIs, nur erfolgreiche Laeufe\n', cond);
    fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\nMetric', repmat('c', 1, height(t2)));
    for i = 1:height(t2)
        fprintf(fid, ' & %s (%d)', pretty(t2.config(i)), t2.nSuccess(i));
    end
    fprintf(fid, ' \\\\\n\\midrule\n');
    for col = cols
        fprintf(fid, '%s', col);
        for i = 1:height(t2)
            fprintf(fid, ' & %s', pm(t2.(col + "_mean")(i), t2.(col + "_std")(i), 4));
        end
        fprintf(fid, ' \\\\\n');
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end

function s = pretty(config)
    s = strrep(config, '_', ' ');
end

function s = pm(m, sd, digits)
    if isnan(m)
        s = "--";
    elseif isnan(sd)
        s = string(sprintf('%.*g', digits, m));
    else
        s = string(sprintf('%.*g $\\pm$ %.*g', digits, m, max(1, digits-1), sd));
    end
end

function m = meanOrNaN(v)
    if isempty(v), m = NaN; else, m = mean(v); end
end

function s = stdOrNaN(v)
    if numel(v) < 2, s = NaN; else, s = std(v); end
end

function m = medianOrNaN(v)
    if isempty(v), m = NaN; else, m = median(v); end
end
