function makeFigures(opt)
% makeFigures  Erzeugt alle datengetriebenen Abbildungen des Papers neu.
%
%   makeFigures()                     % schreibt in den Figures-Ordner des Papers
%   makeFigures(OutDir="C:\tmp")      % woanders hin (zum Ansehen)
%   makeFigures(Only="boxplots")      % nur eine Gruppe
%
%   Gruppen und Dateinamen (gleiche Namen wie bisher, damit die .tex-Dateien
%   unveraendert bleiben):
%     boxplots   boxplot_K2/K4/K7/K9.png      Verteilung ueber die erfolgreichen Laeufe
%     ci         ci_K2/K4/K7/K9.png           95 %-Bootstrap-Konfidenzintervalle
%     training   trainingsverlauf_ppo_default/_optimized.png   Mittel +- Std ueber 10 Seeds
%     trainall   trainingsverlauf_defaults.png   Lernkurven aller sechs Default-Agenten,
%                erfolgreiche und gescheiterte Laeufe getrennt eingefaerbt
%     circle    soll_vs_ist_kreisbahn.png, soll_vs_ist_kreisbahn_opt.png,
%                basis_orientaion_ppo.png, basis_orientation_ppo_optimized.png
%     twoseg     rampe_1.png, rampe_opt_1.png, basis_ori_1.png, basis_ori_opt_1.png
%     momentum   impulserhaltung_scope.png    Gesamtimpuls waehrend einer Episode
%
%   Grundlage sind die Ergebnisse unter results/. Fuer die Episodenbilder wird
%   je Agent eine Episode aus der nominalen Startpose neu simuliert. Ausgewaehlt
%   wird der erfolgreiche Lauf mit dem mittleren K2 (Median), der Seed steht in
%   der Konsolenausgabe und gehoert in die Caption.

    arguments
        opt.OutDir  (1,1) string = "C:\Users\Deermste\Documents\FraUas\Mechatronikprojekt\Paper\space-robot-simulation-using-formal-methods\Figures"
        opt.Only    (1,:) string = ["boxplots","ci","training","trainall","circle","twoseg","momentum"]
        opt.OutRoot (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    if ~exist(opt.OutDir, 'dir'), mkdir(opt.OutDir); end
    set(groot, 'defaultAxesFontSize', 9, 'defaultTextFontSize', 9, 'defaultAxesFontName', 'Helvetica');

    R  = analyzeBenchmark("benchmark_v2", OutRoot=opt.OutRoot);
    Rt = analyzeBenchmark("twoseg",       OutRoot=opt.OutRoot);

    if any(opt.Only == "boxplots"), figBoxplots(R.runs, opt.OutDir); end
    if any(opt.Only == "ci"),       figCI(R.runs, opt.OutDir);       end
    if any(opt.Only == "training"), figTraining(opt.OutRoot, opt.OutDir); end
    if any(opt.Only == "trainall"), figTrainingAll(R.runs, opt.OutRoot, opt.OutDir); end

    if any(ismember(["circle","twoseg","momentum"], opt.Only))
        sel  = pickSeeds(R.runs,  ["PPO_default","PPO_optimized","TRPO_default"]);
        selT = pickSeeds(Rt.runs, ["PPO_default","PPO_optimized"]);
        L  = episodeLogs("benchmark_v2", sel,  opt.OutRoot);
        LT = episodeLogs("twoseg",       selT, opt.OutRoot);

        if any(opt.Only == "circle")
            figTrajectory(L.PPO_default,   fullfile(opt.OutDir, 'soll_vs_ist_kreisbahn.png'));
            figTrajectory(L.PPO_optimized, fullfile(opt.OutDir, 'soll_vs_ist_kreisbahn_opt.png'));
            figTrajectory(L.TRPO_default,  fullfile(opt.OutDir, 'soll_vs_ist_kreisbahn_trpo.png'));
            figQuaternion(L.PPO_default,   fullfile(opt.OutDir, 'basis_orientaion_ppo.png'));
            figQuaternion(L.PPO_optimized, fullfile(opt.OutDir, 'basis_orientation_ppo_optimized.png'));
        end
        if any(opt.Only == "twoseg")
            figTrajectory(LT.PPO_default,   fullfile(opt.OutDir, 'rampe_1.png'));
            figTrajectory(LT.PPO_optimized, fullfile(opt.OutDir, 'rampe_opt_1.png'));
            figQuaternion(LT.PPO_default,   fullfile(opt.OutDir, 'basis_ori_1.png'));
            figQuaternion(LT.PPO_optimized, fullfile(opt.OutDir, 'basis_ori_opt_1.png'));
        end
        if any(opt.Only == "momentum")
            figMomentum(L.PPO_optimized, fullfile(opt.OutDir, 'impulserhaltung_scope.png'));
        end
    end
    fprintf('[makeFigures] fertig -> %s\n', opt.OutDir);
end

% ============================== Verteilungen ================================

function figBoxplots(runs, outDir)
% Ein Bild je KPI: Box ueber die erfolgreichen Laeufe, dazu die einzelnen Laeufe.
    [cfg, lab] = configOrder(runs);
    for k = ["K2","K4","K7","K9"]
        f = newFig();
        hold on;
        data = cell(numel(cfg),1);
        for i = 1:numel(cfg)
            data{i} = runs.(k)(runs.config == cfg(i) & runs.success);
            if ~isempty(data{i})
                boxchart(repmat(i, numel(data{i}), 1), data{i}, 'BoxFaceColor', [0.30 0.45 0.70], ...
                    'MarkerStyle', 'none', 'BoxWidth', 0.5);
                scatter(i + 0.18*(rand(numel(data{i}),1)-0.5), data{i}, 10, [0.25 0.25 0.25], 'filled', ...
                    'MarkerFaceAlpha', 0.6);
            end
        end
        tickLabels(lab, cellfun(@numel, data)');
        ylabel(kpiLabel(k));
        v = vertcat(data{:});
        if ~isempty(v) && min(v) > 0 && max(v) / min(v) > 50
            set(gca, 'YScale', 'log');       % Spannweite ueber mehrere Groessenordnungen
        end
        grid on; box on;
        save1(f, fullfile(outDir, "boxplot_" + k + ".png"));
    end
end

function figCI(runs, outDir)
% Mittelwert je Konfiguration mit 95 %-Bootstrap-Konfidenzintervall.
    [cfg, lab] = configOrder(runs);
    rng(2026, 'twister');                       % reproduzierbares Bootstrap
    for k = ["K2","K4","K7","K9"]
        m = nan(numel(cfg),1); lo = m; hi = m; n = zeros(numel(cfg),1);
        for i = 1:numel(cfg)
            v = runs.(k)(runs.config == cfg(i) & runs.success);
            n(i) = numel(v);
            if isempty(v), continue; end
            m(i) = mean(v);
            if numel(v) > 1
                bs = mean(v(randi(numel(v), numel(v), 2000)), 1);
                lo(i) = prctile(bs, 2.5); hi(i) = prctile(bs, 97.5);
            else
                lo(i) = v; hi(i) = v;
            end
        end
        f = newFig();
        errorbar(1:numel(cfg), m, m - lo, hi - m, 'o', 'LineWidth', 1.1, ...
            'MarkerFaceColor', [0.30 0.45 0.70], 'Color', [0.20 0.30 0.50], 'CapSize', 6);
        xlim([0.5, numel(cfg)+0.5]);
        tickLabels(lab, n');
        ylabel(kpiLabel(k)); grid on; box on;
        if max(m) / max(1e-12, min(m)) > 50, set(gca, 'YScale', 'log'); end
        save1(f, fullfile(outDir, "ci_" + k + ".png"));
    end
end

function figTraining(outRoot, outDir)
% Lernkurven: gleitender Mittelwert je Seed, dazu Mittel +- Std ueber die Seeds.
% Beide Bilder stehen im Paper nebeneinander und bekommen dieselbe y-Achse.
    configs = ["PPO_default", "PPO_optimized"];
    Xs = cell(size(configs));
    for j = 1:numel(configs)
        files = dir(fullfile(outRoot, 'benchmark_v2', 'agents', configs(j) + "_s*.mat"));
        for i = 1:numel(files)
            L = load(fullfile(files(i).folder, files(i).name), 'stats');
            Xs{j}(:, i) = movmean(L.stats.EpisodeReward, 25);
        end
    end
    yLo = floor(min(cellfun(@(X) min(X, [], 'all'), Xs)) / 10) * 10;
    for j = 1:numel(configs)
        c = configs(j);
        X = Xs{j};
        mu = mean(X, 2); sd = std(X, 0, 2); ep = (1:size(X,1))';
        f = newFig();
        hold on;
        hBand = fill([ep; flipud(ep)], [mu-sd; flipud(mu+sd)], [0.30 0.45 0.70], ...
            'FaceAlpha', 0.25, 'EdgeColor', 'none');
        hSeed = plot(ep, X, 'Color', [0.6 0.6 0.6 0.35], 'LineWidth', 0.3);
        hMean = plot(ep, mu, 'Color', [0.15 0.25 0.45], 'LineWidth', 1.4);
        xlabel('Episode'); ylabel('Episode reward (moving average, 25)');
        ylim([yLo, 0]);
        grid on; box on;
        legend([hMean, hBand, hSeed(1)], {'mean over 10 seeds', 'mean $\pm$ std', 'individual seeds'}, ...
            'Interpreter', 'latex', 'Location', 'southeast', 'FontSize', 8);
        name = "trainingsverlauf_ppo_" + extractAfter(c, "PPO_") + ".png";
        save1(f, fullfile(outDir, name));
    end
end

function figTrainingAll(runs, outRoot, outDir)
% Lernkurven der sechs Default-Agenten als kleine Vielfache mit gleicher
% y-Achse. Jede Linie ist ein Seed (gleitender Mittelwert ueber 25 Episoden),
% blau erfolgreiche, orange gescheiterte Laeufe, schwarz das Mittel ueber alle
% zehn Seeds. Reihenfolge wie in den Tabellen des Papers.
    configs = ["TRPO_default","SAC_default","TD3_default","DDPG_default","PPO_default","PG_default"];
    cOk   = [0.184 0.427 0.710];      % #2F6DB5
    cFail = [0.784 0.439 0.118];      % #C8701E
    Xs = cell(size(configs)); ok = Xs;
    for j = 1:numel(configs)
        files = dir(fullfile(outRoot, 'benchmark_v2', 'agents', configs(j) + "_s*.mat"));
        for i = 1:numel(files)
            L = load(fullfile(files(i).folder, files(i).name), 'stats', 'seed');
            Xs{j}(:, i) = movmean(L.stats.EpisodeReward, 25);
            ok{j}(i) = runs.success(runs.config == configs(j) & runs.seed == L.seed);
        end
    end
    yLo = floor(min(cellfun(@(X) min(X, [], 'all'), Xs)) / 10) * 10;

    f = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [2 2 18 9], 'Color', 'w');
    t = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for j = 1:numel(configs)
        ax = nexttile(t);
        hold(ax, 'on');
        X = Xs{j}; ep = (1:size(X,1))';
        [hF, hS] = deal(gobjects(0));
        if any(~ok{j}), hF = plot(ax, ep, X(:, ~ok{j}), 'Color', [cFail 0.6], 'LineWidth', 0.5); end
        if any(ok{j}),  hS = plot(ax, ep, X(:,  ok{j}), 'Color', [cOk 0.6],   'LineWidth', 0.5); end
        hM = plot(ax, ep, mean(X, 2), 'Color', [0.1 0.1 0.1], 'LineWidth', 1.3);
        ylim(ax, [yLo, 0]); xlim(ax, [1, size(X,1)]);
        grid(ax, 'on'); box(ax, 'on');
        title(ax, sprintf('%s (%d/%d successful)', extractBefore(configs(j), "_"), ...
            nnz(ok{j}), numel(ok{j})), 'FontWeight', 'normal');
        if ~isempty(hS) && ~isempty(hF)
            hLeg = [hM, hS(1), hF(1)];      % Handles fuer die gemeinsame Legende
        end
    end
    % Eine Legende unter allen Feldern, damit sie keine Kurven verdeckt
    lg = legend(hLeg, {'mean over 10 seeds', 'successful run', 'failed run'}, ...
        'Orientation', 'horizontal', 'FontSize', 8);
    lg.Layout.Tile = 'south';
    xlabel(t, 'Episode', 'FontSize', 9);
    ylabel(t, 'Episode reward (moving average, 25)', 'FontSize', 9);
    save1(f, fullfile(outDir, 'trainingsverlauf_defaults.png'));
end

% ============================ Einzelne Episoden =============================

function sel = pickSeeds(runs, configs)
% Erfolgreicher Lauf mit dem mittleren K2 (Median) je Konfiguration.
    sel = struct();
    for c = configs
        r = runs(runs.config == c & runs.success, :);
        if isempty(r)
            warning('makeFigures:noRun', '%s hat keinen erfolgreichen Lauf.', c);
            continue;
        end
        [~, i] = min(abs(r.K2 - median(r.K2)));
        sel.(c) = r.seed(i);
        fprintf('[makeFigures] %s: Seed %d (K2 = %.4f)\n', c, r.seed(i), r.K2(i));
    end
end

function L = episodeLogs(campaign, sel, outRoot)
% Simuliert je Konfiguration eine Episode aus der nominalen Startpose neu.
    L = struct();
    cfgs = string(fieldnames(sel))';
    for c = cfgs
        agentFile = fullfile(outRoot, campaign, 'agents', sprintf('%s_s%d.mat', c, sel.(c)));
        tmp = [tempname '.mat'];
        evaluateAgent(string(agentFile), zeros(4,1), LogFile=string(tmp));
        S = load(tmp, 'logs', 'cfg');
        e.log = S.logs{1};
        e.cfg = S.cfg;
        e.seed = sel.(c);
        L.(c) = e;
        delete(tmp);
    end
end

function figTrajectory(e, file)
% Soll- und Ist-Bahn des Endeffektors in der XY-Ebene.
    p = sq(e.log.getElement('p_EE').Values.Data);
    EE_ref = referenceTrajectory(e.cfg, 0:e.cfg.Ts:e.cfg.T);
    ref = EE_ref.Data;
    f = newFig();
    plot(ref(:,1), ref(:,2), '--', 'Color', [0.75 0.30 0.20], 'LineWidth', 1.2); hold on;
    plot(p(:,1), p(:,2), '-', 'Color', [0.15 0.25 0.45], 'LineWidth', 1.2);
    xlabel('$x$ (m)', 'Interpreter', 'latex'); ylabel('$y$ (m)', 'Interpreter', 'latex');
    legend({'reference', 'actual'}, 'Location', 'best', 'FontSize', 8);
    axis equal; grid on; box on;
    save1(f, file);
end

function figQuaternion(e, file)
% Vektorteil des Quaternions der Basisorientierung ueber die Zeit. q_w liegt
% nahe 1 und wuerde die Skala bestimmen, sein Minimum steht in der Ausgabe.
    q = sq(e.log.getElement('basis_ori').Values.Data);
    t = e.log.getElement('basis_ori').Values.Time;
    f = newFig();
    plot(t, q(:, 2:4), 'LineWidth', 1.1);
    xlabel('Time (s)'); ylabel('Base quaternion, vector part'); xlim([0, t(end)]);
    legend({'$q_x$','$q_y$','$q_z$'}, 'Interpreter', 'latex', ...
        'Location', 'best', 'FontSize', 8, 'Orientation', 'horizontal');
    grid on; box on;
    save1(f, file);
    fprintf('[makeFigures]   min q_w = %.5f, max |q_xyz| = %.4f\n', min(q(:,1)), max(abs(q(:,2:4)), [], 'all'));
end

function figMomentum(e, file)
% Gesamtimpuls waehrend einer Episode (Impulsmonitor).
    p = sq(e.log.getElement('p_tot').Values.Data);
    t = e.log.getElement('p_tot').Values.Time;
    f = newFig();
    plot(t, p, 'LineWidth', 1.1);
    xlabel('Time (s)'); ylabel('Total linear momentum (N s)');
    xlim([0, t(end)]);
    legend({'$p_x$','$p_y$','$p_z$'}, 'Interpreter', 'latex', 'Location', 'southeast', ...
        'FontSize', 8, 'Orientation', 'horizontal');
    grid on; box on;
    save1(f, file);
    fprintf('[makeFigures] max |p_tot| = %.3g N s\n', max(abs(p), [], 'all'));
end

% ================================ Helfer ====================================

function [cfg, lab] = configOrder(runs)
    order = ["PG_default","PPO_default","TRPO_default","DDPG_default","TD3_default","SAC_default","PPO_optimized"];
    cfg = order(ismember(order, unique(runs.config)'));
    lab = strrep(cfg, "_default", "") ;
    lab = strrep(lab, "PPO_optimized", "PPO opt.");
end

function tickLabels(lab, n)
% Zweizeilige Achsenbeschriftung (Name, darunter n). Ein String mit newline
% zerlegt MATLAB in zwei Labels und verschiebt dadurch alle folgenden, deshalb
% \newline mit dem TeX-Interpreter.
    xticks(1:numel(lab));
    set(gca, 'TickLabelInterpreter', 'tex', 'XTickLabelRotation', 0);
    xticklabels(cellstr(lab + "\newlinen=" + string(n)));
    set(gcf, 'Position', [2 2 12 7]);     % breiter, damit sieben Labels waagerecht passen
end

function s = kpiLabel(k)
    switch k
        case "K2", s = 'K_2: mean squared EE position error (m^2)';
        case "K4", s = 'K_4: mean base orientation error (rad)';
        case "K7", s = 'K_7: control smoothness';
        case "K9", s = 'K_9: energy consumption';
        otherwise, s = char(k);
    end
end

function f = newFig()
    f = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [2 2 10 7], 'Color', 'w');
end

function save1(f, file)
    exportgraphics(f, file, 'Resolution', 300);
    close(f);
    fprintf('[makeFigures] %s\n', file);
end

function X = sq(D)
    X = squeeze(D);
    if size(X,1) < size(X,2), X = X.'; end
end
