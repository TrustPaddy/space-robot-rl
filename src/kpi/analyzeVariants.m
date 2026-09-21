function T = analyzeVariants(campaigns, labels, opt)
% analyzeVariants  Vergleicht mehrere Kampagnen (Varianten) mit einer Referenz.
%
%   T = analyzeVariants(["abl_wp0","abl_wv0"], ["W_p = 0","W_v = 0"])
%   T = analyzeVariants(camps, labels, Configs="PPO_optimized", Seeds=0:4, ...
%                       OutFile="results/abl_ppo_opt")
%
%   Fuer Ablation, Reward-Sensitivitaet und R2.4b: jede Variante ist eine eigene
%   Kampagne, die Referenz sind dieselben Konfigurationen aus benchmark_v2. Pro
%   Variante und Konfiguration werden dieselben Groessen berichtet wie in
%   analyzeBenchmark, also Abbruchrate ueber ALLE Laeufe und die KPIs nur ueber
%   die erfolgreichen Laeufe.
%
%   Seeds: die Varianten laufen mit 5 Seeds, benchmark_v2 mit 10. Mit
%   Seeds = 0:4 wird die Referenz auf dieselben Seeds eingeschraenkt.
%
%   Ergebnis T: eine Zeile je Variante und Konfiguration mit nRuns, nSuccess,
%   abortRate, Mittelwert, Standardabweichung und Median von K1, K2 und K4
%   sowie der mittleren Trainingsdauer T2. Mit OutFile zusaetzlich als CSV und
%   als LaTeX-Tabelle.

    arguments
        campaigns (1,:) string
        labels    (1,:) string = campaigns
        opt.Reference      (1,1) string = "benchmark_v2"
        opt.ReferenceLabel (1,1) string = "reference"
        opt.Configs        (1,:) string = string.empty
        opt.Seeds          (1,:) double = []
        opt.Condition      (1,1) string = "nominal"
        opt.OutFile        (1,1) string = ""
        opt.OutRoot        (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    if numel(labels) ~= numel(campaigns)
        error('analyzeVariants:labels', 'Es braucht genau ein Label je Kampagne.');
    end

    all_ = [opt.Reference, campaigns];
    lab  = [opt.ReferenceLabel, labels];
    runs = table();
    for i = 1:numel(all_)
        R = analyzeBenchmark(all_(i), Condition=opt.Condition, OutRoot=opt.OutRoot);
        r = R.runs;
        if ~isempty(opt.Configs), r = r(ismember(r.config, opt.Configs), :); end
        if ~isempty(opt.Seeds),   r = r(ismember(r.seed, opt.Seeds), :);     end
        r.variant = repmat(lab(i), height(r), 1);
        r.order   = repmat(i, height(r), 1);
        runs = [runs; r]; %#ok<AGROW>
    end

    key = unique(runs(:, {'order','variant','config'}), 'rows');
    key = sortrows(key, {'order','config'});
    T = table();
    for i = 1:height(key)
        r = runs(runs.variant == key.variant(i) & runs.config == key.config(i), :);
        s = r(r.success, :);
        row = table(key.variant(i), key.config(i), height(r), height(s), ...
            sum(r.nAbort) / sum(r.nEpisodes), ...
            'VariableNames', {'variant','config','nRuns','nSuccess','abortRate'});
        for k = ["K1","K2","K4"]
            row.(k + "_mean")   = meanOrNaN(s.(k));
            row.(k + "_std")    = stdOrNaN(s.(k));
            row.(k + "_median") = medianOrNaN(s.(k));
        end
        row.T2_mean = meanOrNaN(s.T2);
        T = [T; row]; %#ok<AGROW>
    end

    if strlength(opt.OutFile) > 0
        d = fileparts(opt.OutFile);
        if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
        writetable(T, opt.OutFile + ".csv");
        writeTex(opt.OutFile + ".tex", T, opt.Condition);
        fprintf('[analyzeVariants] %d Zeilen -> %s.csv/.tex\n', height(T), opt.OutFile);
    end
end

function writeTex(f, T, cond)
% Eine Zeile je Variante und Konfiguration: erfolgreiche Laeufe, Abbruchrate,
% K2 und K4 der erfolgreichen Laeufe.
    oneConfig = numel(unique(T.config)) == 1;
    fid = fopen(f, 'w');
    c = onCleanup(@() fclose(fid));
    fprintf(fid, '%% analyzeVariants.m, Bedingung %s: KPIs nur ueber erfolgreiche Laeufe\n', cond);
    if oneConfig
        fprintf(fid, '\\begin{tabular}{lccc}\n\\toprule\n');
        fprintf(fid, 'Variant & Successful runs & $K_2$ & $K_4$ \\\\\n\\midrule\n');
    else
        fprintf(fid, '\\begin{tabular}{llccc}\n\\toprule\n');
        fprintf(fid, 'Variant & Agent & Successful runs & $K_2$ & $K_4$ \\\\\n\\midrule\n');
    end
    for i = 1:height(T)
        if oneConfig
            fprintf(fid, '%s & %d/%d & %s & %s \\\\\n', T.variant(i), T.nSuccess(i), T.nRuns(i), ...
                pm(T.K2_mean(i), T.K2_std(i)), pm(T.K4_mean(i), T.K4_std(i)));
        else
            fprintf(fid, '%s & %s & %d/%d & %s & %s \\\\\n', T.variant(i), strrep(T.config(i), '_', ' '), ...
                T.nSuccess(i), T.nRuns(i), pm(T.K2_mean(i), T.K2_std(i)), pm(T.K4_mean(i), T.K4_std(i)));
        end
    end
    fprintf(fid, '\\bottomrule\n\\end{tabular}\n');
end

function s = pm(m, sd)
    if isnan(m)
        s = "--";
    elseif isnan(sd)
        s = string(sprintf('%.4g', m));
    else
        s = string(sprintf('%.4g $\\pm$ %.3g', m, sd));
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
