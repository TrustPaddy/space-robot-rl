function R = analyzeBenchmark(campaign, opt)
% analyzeBenchmark  Tabellen und Statistik zum Benchmark (ersetzt Tab. 5-7).
%
%   R = analyzeBenchmark("benchmark_v2")
%
%   Grundlage: results/<campaign>/eval/<Condition>.csv (evaluateCampaign) und
%   die Trainingsstatistik in results/<campaign>/agents/*.mat.
%
%   Aggregation (fuer alle Tabellen gleich):
%     - Pro Seed werden die KPIs ueber die zufaelligen Startzustaende
%       gemittelt (K3: Maximum, K5: Anteil Episoden mit Kollision).
%     - Tabellenwerte: Mittelwert +- Standardabweichung ueber die Seeds.
%     - Tests: je Startzustand Mittel ueber die Seeds -> gepaarte Stichprobe
%       ueber die Startzustaende. Friedman ueber alle Default-Agenten, dann
%       Wilcoxon signed-rank PPO gegen jeden anderen Agenten, Holm-korrigiert.
%       (Direkt ueber 5 Seeds getestet koennte Wilcoxon nie p < 0.05 erreichen.)
%     - T1: Standardabweichung des Episodenrewards in den letzten 100
%       Trainingsepisoden; T3: Trainingsdauer [min]. Je Seed, dann Mittel +- Std.
%
%   Ausgabe: R.summary, R.tests, R.ppo (Default vs. optimiert) und dieselben
%   Tabellen als CSV und LaTeX in results/<campaign>/analysis/.

    arguments
        campaign (1,1) string
        opt.Condition (1,1) string = "nominal"
        opt.Baseline  (1,1) string = "PPO"
        opt.OutRoot   (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    campDir = fullfile(opt.OutRoot, campaign);
    E = readtable(fullfile(campDir, 'eval', opt.Condition + ".csv"), 'TextType', 'string');
    E.config = E.agent + "_" + E.mode;
    Er = E(E.init == "random", :);
    kpis = "K" + (1:9);

    % ---------- Trainingsstatistik ----------
    files = dir(fullfile(campDir, 'agents', '*.mat'));
    tr = table();
    for i = 1:numel(files)
        L = load(fullfile(files(i).folder, files(i).name), 'stats', 'wallclock', 'agentType', 'mode', 'seed');
        er = L.stats.EpisodeReward;
        tr = [tr; table(string(L.agentType) + "_" + string(L.mode), L.seed, ...
            std(er(max(1, end-99):end)), L.wallclock/60, ...
            'VariableNames', {'config','seed','T1','T3'})]; %#ok<AGROW>
    end

    % ---------- Seed-Ebene ----------
    S = groupsummary(Er, {'config','seed'}, 'mean', setdiff(kpis, ["K3","K5"]));
    S = renamevars(S, "mean_" + setdiff(kpis, ["K3","K5"]), setdiff(kpis, ["K3","K5"]));
    S3 = groupsummary(Er, {'config','seed'}, 'max', "K3");
    S5 = groupsummary(Er, {'config','seed'}, 'mean', "K5");
    St = groupsummary(Er, {'config','seed'}, 'mean', "terminated");
    S.K3 = S3.max_K3;  S.K5 = S5.mean_K5;  S.term = St.mean_terminated;
    S = join(S, tr, 'Keys', {'config','seed'});

    % ---------- Tabelle: Mittel +- Std ueber Seeds ----------
    cols = [kpis, "term", "T1", "T3"];
    configs = unique(S.config, 'stable');
    summary = table(configs, 'VariableNames', {'config'});
    for c = cols
        m = zeros(numel(configs),1); s = m; n = m;
        for i = 1:numel(configs)
            v = S.(c)(S.config == configs(i));
            m(i) = mean(v); s(i) = std(v); n(i) = numel(v);
        end
        summary.(c + "_mean") = m;
        summary.(c + "_std")  = s;
    end
    summary.nSeeds = n;

    % ---------- Tests ueber Startzustaende (Mittel ueber Seeds) ----------
    P = groupsummary(Er, {'config','episode'}, 'mean', kpis);
    P = renamevars(P, "mean_" + kpis, kpis);
    defaults = configs(endsWith(configs, "_default"));
    base = opt.Baseline + "_default";
    tests = pairedTests(P, defaults, base, kpis);

    % ---------- Default vs. optimiertes PPO ----------
    ppo = table();
    if any(configs == opt.Baseline + "_optimized")
        ppo = pairedTests(P, [base; opt.Baseline + "_optimized"], base, kpis);
    end

    % ---------- Ausgabe ----------
    outDir = fullfile(campDir, 'analysis');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    writetable(S, fullfile(outDir, 'seed_level.csv'));
    writetable(summary, fullfile(outDir, 'summary.csv'));
    writetable(tests, fullfile(outDir, 'tests_vs_' + lower(opt.Baseline) + '.csv'));
    if ~isempty(ppo), writetable(ppo, fullfile(outDir, 'tests_ppo_default_vs_optimized.csv')); end
    writeLatexSummary(fullfile(outDir, 'summary.tex'), summary, cols);

    R = struct('seedLevel', S, 'summary', summary, 'tests', tests, 'ppo', ppo);
    fprintf('[analyzeBenchmark] %s: %d Konfigurationen, Tabellen in %s\n', campaign, numel(configs), outDir);
end

function T = pairedTests(P, configs, base, kpis)
% Friedman ueber configs (Bloecke = Startzustaende), dann Wilcoxon base vs. jede
% andere Konfiguration mit Holm-Korrektur. Delta = Median(config - base).
    others = configs(configs ~= base);
    T = table();
    for k = kpis
        X = zeros(numel(unique(P.episode)), numel(configs));
        for j = 1:numel(configs)
            Pj = sortrows(P(P.config == configs(j), :), 'episode');
            X(:, j) = Pj.(k);
        end
        if numel(configs) > 2 && all(range(X) > 0)
            pF = friedman(X, 1, 'off');
        else
            pF = NaN;
        end
        xb = X(:, configs == base);
        pRaw = zeros(numel(others), 1); d = pRaw;
        for j = 1:numel(others)
            xo = X(:, configs == others(j));
            d(j) = median(xo - xb);
            if all(xo == xb), pRaw(j) = 1; else, pRaw(j) = signrank(xo, xb); end
        end
        pHolm = holm(pRaw);
        T = [T; table(repmat(k, numel(others), 1), repmat(pF, numel(others), 1), others, d, pRaw, pHolm, ...
            'VariableNames', {'kpi','p_friedman','config','median_diff_vs_base','p_raw','p_holm'})]; %#ok<AGROW>
    end
end

function pAdj = holm(p)
    [ps, idx] = sort(p(:));
    m = numel(ps);
    adj = min(1, cummax((m - (1:m)' + 1) .* ps));
    pAdj = zeros(size(p));
    pAdj(idx) = adj;
end

function writeLatexSummary(f, summary, cols)
    fid = fopen(f, 'w');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '%% erzeugt von analyzeBenchmark.m -- Mittelwert $\\pm$ Std. ueber %d Seeds\n', summary.nSeeds(1));
    fprintf(fid, '\\begin{tabular}{l%s}\n\\toprule\nKPI', repmat('c', 1, height(summary)));
    fprintf(fid, ' & %s', strrep(summary.config, '_', '\_'));
    fprintf(fid, ' \\\\\n\\midrule\n');
    for col = cols
        fprintf(fid, '%s', col);
        for i = 1:height(summary)
            fprintf(fid, ' & %.4g $\\pm$ %.2g', summary.(col + "_mean")(i), summary.(col + "_std")(i));
        end
        fprintf(fid, ' \\\\\n');
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end
