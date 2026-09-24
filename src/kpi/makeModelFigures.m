function makeModelFigures(opt)
% makeModelFigures  Modellbilder und Kollisionstest fuer das Paper (Sec. III, Sec. V).
%
%   makeModelFigures()
%   makeModelFigures(Only="collision")
%
%   Gruppen:
%     diagrams   Spacerobot_slx.pdf, spacerobot_slx_robot.pdf, spacerobot_slx_reference.pdf,
%                impuls_monitor.pdf, spacerobot_slx_colissionmonitor.pdf
%                Vektorexport mit print -s, danach mit pdfcrop zugeschnitten (TeX Live).
%                Vorher werden alle Modellvariablen und ein Agent ('agent') in den
%                Base-Workspace geladen, sonst zeigt Simulink Bloecke als fehlerhaft.
%     collision  collision_test_dmin.png, collision_test_tau.png
%                (A) Start in einer kollidierenden Stellung, Moment null.
%                (B) Konstantes Moment tauB aus der Nullpose, bis der Arm die Basis
%                    beruehrt. d_min wird je Agentenschritt aus q mit derselben Funktion
%                    wie im Kollisionsmonitor berechnet (collisionCheckWrapper).
%
%   Das Modell wird nur geladen und ohne Speichern geschlossen.

    arguments
        opt.OutDir (1,1) string = "C:\Users\Deermste\Documents\FraUas\Mechatronikprojekt\Paper\space-robot-simulation-using-formal-methods\Figures"
        opt.Only   (1,:) string = ["diagrams","collision"]
        opt.tauB   (4,1) double = [0.2; 0.6; 0.4; 0.5]    % [N m], Arm faltet zur Basis
        opt.qA_deg (4,1) double = [0; 150; 150; 150]      % kollidierende Startstellung
    end

    if ~exist(opt.OutDir, 'dir'), mkdir(opt.OutDir); end
    set(groot, 'defaultAxesFontSize', 9, 'defaultTextFontSize', 9, 'defaultAxesFontName', 'Helvetica');

    S = setupSpaceRobotEnv();
    c = onCleanup(@() close_system(S.mdl, 0));

    if any(opt.Only == "diagrams"), exportDiagrams(S, opt.OutDir); end
    if any(opt.Only == "collision"), collisionTest(S, opt); end
end

% ================================ Diagramme ==================================

function exportDiagrams(S, outDir)
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    L = load(fullfile(root, 'results', 'benchmark_v2', 'agents', 'PPO_optimized_s0.mat'), 'agent');
    assignin('base', 'agent', L.agent);          % Variable des RL-Agent-Blocks

    mdl = S.mdl;
    sys = {mdl,                        'Spacerobot_slx'
           [mdl '/Robot'],             'spacerobot_slx_robot'
           [mdl '/Reference'],         'spacerobot_slx_reference'
           [mdl '/Robot/Impuls Monitor'], 'impuls_monitor'
           [mdl '/collision monitor'], 'spacerobot_slx_colissionmonitor'};
    for i = 1:size(sys, 1)
        tmp = [tempname '.pdf'];
        out = fullfile(outDir, [sys{i, 2} '.pdf']);
        print(['-s' sys{i, 1}], '-dpdf', tmp);
        [st, msg] = system(sprintf('pdfcrop --margins 2 "%s" "%s"', tmp, out));
        if st ~= 0
            warning('makeModelFigures:pdfcrop', 'pdfcrop fehlgeschlagen, ungeschnitten gespeichert: %s', msg);
            copyfile(tmp, out);
        end
        delete(tmp);
        fprintf('[makeModelFigures] %s -> %s\n', sys{i, 1}, out);
    end
end

% ============================== Kollisionstest ===============================

function collisionTest(S, opt)
    cfg = S.cfg;

    % (A) Start in einer kollidierenden Stellung
    qA = deg2rad(opt.qA_deg);
    [dA, cA] = collisionCheckWrapper(qA');
    lo = runConst(S, zeros(4, 1), qA);
    [tq, ~] = sig(lo, 'q');
    fprintf(['[makeModelFigures] (A) q0 = [%s] deg: Kontakt in q0 = %d (d_min der uebrigen Paare ' ...
             '%.3f m), Episode endet bei t = %.2f s\n'], num2str(opt.qA_deg'), cA, dA, tq(end));

    % (B) Konstantes Moment aus der Nullpose
    lo = runConst(S, opt.tauB, zeros(4, 1));
    [tq, q]   = sig(lo, 'q');
    [tc, ic]  = sig(lo, 'isCollision');
    [tt, tau] = sig(lo, 'tau');
    tAg = (0:cfg.Ts_agent:tq(end) + 1e-9)';
    dmin = nan(size(tAg)); coll = false(size(tAg));
    for j = 1:numel(tAg)
        [dmin(j), coll(j)] = collisionCheckWrapper(interp1(tq, q, min(tAg(j), tq(end)), 'previous'));
    end
    iW = find(dmin < cfg.d_safe & ~coll, 1);
    tW = tAg(iW);
    tC = tc(find(ic > 0.5, 1));
    tZ = tt(find(all(abs(tau) < 1e-12, 2) & tt >= tC - cfg.Ts_agent, 1));
    fprintf(['[makeModelFigures] (B) tau = [%s] N m: Warnung (d_min < d_safe) bei t = %.2f s, ' ...
             'Kontakt bei t = %.2f s, Moment null ab t = %.2f s, Episode endet bei t = %.2f s\n'], ...
             num2str(opt.tauB'), tW, tC, tZ, tq(end));

    % (a) Mindestabstand
    f = newFig();
    ok = ~coll;
    h1 = semilogy(tAg(ok), dmin(ok), '-o', 'Color', [0.15 0.25 0.45], 'MarkerSize', 2.5, ...
        'MarkerFaceColor', [0.15 0.25 0.45], 'LineWidth', 1.1); hold on;
    h2 = yline(cfg.d_safe, '--', 'Color', [0.75 0.30 0.20], 'LineWidth', 1);
    h3 = plot(tW, dmin(iW), 'o', 'MarkerSize', 7, 'Color', [0.85 0.55 0.10], 'LineWidth', 1.5);
    h4 = xline(tC, '-', 'Color', [0.40 0.40 0.40], 'LineWidth', 1);
    legend([h1 h2 h3 h4], {'$d_{\min}$', '$d_{\mathrm{safe}}$', 'warning', 'contact detected'}, ...
        'Interpreter', 'latex', 'Location', 'southwest', 'FontSize', 8);
    xlabel('Time (s)'); ylabel('Minimum distance (m)');
    xlim([0, cfg.T]); grid on; box on;
    save1(f, fullfile(opt.OutDir, 'collision_test_dmin.png'));

    % (b) Gelenkmomente
    f = newFig();
    h = stairs(tt, tau, 'LineWidth', 1.1); hold on;
    hc = xline(tC, '-', 'Color', [0.40 0.40 0.40], 'LineWidth', 1);
    legend([h; hc], [compose('$\\tau_%d$', 1:4), {'contact detected'}], 'Interpreter', 'latex', ...
        'Location', 'south', 'NumColumns', 3, 'FontSize', 8);
    xlabel('Time (s)'); ylabel('Joint torque (N m)');
    xlim([0, cfg.T]); ylim([min(0, min(opt.tauB)) - 0.45, max(opt.tauB) + 0.1]); grid on; box on;
    save1(f, fullfile(opt.OutDir, 'collision_test_tau.png'));
end

function lo = runConst(S, tauConst, q0)
% Agent mit konstanter Aktion: Netz ohne Gewichte, Bias = tauConst.
    nObs = S.obsInfo.Dimension(1);
    net = dlnetwork([featureInputLayer(nObs, 'Normalization', 'none', 'Name', 'obs')
                     fullyConnectedLayer(4, 'Name', 'fc', 'Weights', zeros(4, nObs), 'Bias', tauConst(:))]);
    actor = rlContinuousDeterministicActor(net, S.obsInfo, S.actInfo);
    agent = setActor(rlDDPGAgent(S.obsInfo, S.actInfo), actor);
    agent.UseExplorationPolicy = false;
    agent.AgentOptions.SampleTime = S.Ts_agent;   % sonst Standard 1 s
    env = S.env;
    env.ResetFcn = @(in) localResetFunction(in, struct('q0', q0, 'randomize', false, 'range_deg', 0));
    xp = sim(env, agent, rlSimulationOptions(MaxSteps = floor(S.cfg.T / S.cfg.Ts_agent)));
    lo = xp.SimulationInfo(1).logsout;
end

% ================================= Helfer ====================================

function [t, x] = sig(lo, name)
    v = lo.getElement(name).Values;
    t = v.Time(:);
    x = squeeze(v.Data);
    if size(x, 1) ~= numel(t), x = x.'; end
end

function f = newFig()
    f = figure('Visible', 'off', 'Units', 'centimeters', 'Position', [2 2 10 7], 'Color', 'w');
end

function save1(f, file)
    exportgraphics(f, file, 'Resolution', 300);
    close(f);
    fprintf('[makeModelFigures] %s\n', file);
end
