function parametrizeSpaceRobotModel(mdl)
% parametrizeSpaceRobotModel  Ersetzt fest eingetragene Werte in SpaceRobot.slx
% durch Workspace-Parameter aus benchmarkConfig (einmalige Migration, idempotent).
%
%   parametrizeSpaceRobotModel()            % migriert und speichert SpaceRobot.slx
%
%   Hintergrund: Masse der Basis, Gelenkdaempfung, Startwinkel, Momentengrenze
%   und Reward-Gewichte standen fest im Modell und wurden fuer Experimente von
%   Hand geaendert (z. B. W_ori 200 -> 2000, Basis 5 -> 25 kg). Dadurch liessen
%   sich alte Ergebnisse nicht mehr reproduzieren. Nach der Migration liest das
%   Modell diese Werte aus dem Workspace, gesetzt von setupSpaceRobotEnv:
%     robotP.*   Inertia-Bloecke (base_link, link1-4) und Gelenkdaempfung
%     q0, dq0    Positions-/Geschwindigkeits-Targets der vier Gelenke
%     tau_sat    Saturation vor der Strecke
%     rewardW.*  Gewichte und Konstanten im Reward-Block (Parameter-Daten)

    if nargin < 1, mdl = 'SpaceRobot'; end
    load_system(mdl);

    % ---- 1) Massen und Traegheiten ----
    set_param([mdl '/Robot/base_link/Inertia'], ...
        'Mass', 'robotP.m_base', 'MomentsOfInertia', 'robotP.I_base');
    for i = 1:4
        set_param(sprintf('%s/Robot/link%d/Inertia', mdl, i), ...
            'Mass', 'robotP.m_link', 'MomentsOfInertia', 'robotP.I_link');
    end

    % ---- 2) Gelenke: Daempfung und Startzustand ----
    for i = 1:4
        blk = sprintf('%s/Robot/joint%d', mdl, i);
        set_param(blk, ...
            'DampingCoefficient',       'robotP.joint_damping', ...
            'PositionTargetValue',      sprintf('q0(%d)', i), ...
            'PositionTargetValueUnits', 'rad', ...
            'VelocityTargetValue',      sprintf('dq0(%d)', i), ...
            'VelocityTargetValueUnits', 'rad/s');
        % Run-time-Parameter: q0 darf sich zwischen Episoden aendern, auch
        % wenn rlSimulinkEnv mit Fast Restart simuliert.
        trySet(blk, 'PositionTargetValue_conf', 'runtime');
        trySet(blk, 'VelocityTargetValue_conf', 'runtime');
    end

    % ---- 3) Momentensaettigung ----
    set_param([mdl '/Saturation'], 'UpperLimit', 'tau_sat', 'LowerLimit', '-tau_sat');

    % ---- 4) Reward-Funktion ----
    ch = findRewardChart(mdl);
    ch.Script = rewardScript();
    if isempty(ch.find('-isa', 'Stateflow.Data', 'Name', 'rewardW'))
        d = Stateflow.Data(ch);
        d.Name  = 'rewardW';
        d.Scope = 'Parameter';
    end

    % ---- 5) Gelenkreihenfolge in q und dq ----
    % Die Mux-Bloecke fuer q ('Mux') und dq ('Mux1') waren in umgekehrter
    % Reihenfolge verdrahtet ([joint4 ... joint1]), die Momente dagegen in
    % richtiger. Folge: Kollisionsmonitor pruefte eine falsche Armstellung,
    % K6 verwendete die Grenzen falscher Gelenke, K9 multiplizierte tau_i mit
    % dq_(5-i). Danach gilt q = [q1 q2 q3 q4], dq = [dq1 dq2 dq3 dq4].
    orderMuxByJoint([mdl '/Robot'], 'Mux');
    orderMuxByJoint([mdl '/Robot'], 'Mux1');

    save_system(mdl);
    fprintf('[parametrizeSpaceRobotModel] %s migriert und gespeichert.\n', mdl);
end

function orderMuxByJoint(sys, muxName)
% Verdrahtet die Eingaenge von sys/muxName so, dass Eingang k das Signal von
% joint k fuehrt. Jeder Eingang kommt von einem PS-Simulink-Konverter, der
% physikalisch mit genau einem Joint-Block verbunden ist.
    mux = [sys '/' muxName];
    ph  = get_param(mux, 'PortHandles');
    n   = numel(ph.Inport);
    srcPort = zeros(1, n);
    joint   = zeros(1, n);
    for k = 1:n
        ln = get_param(ph.Inport(k), 'Line');
        srcPort(k) = get_param(ln, 'SrcPortHandle');
        joint(k)   = jointOfConverter(get_param(srcPort(k), 'Parent'));
    end
    if isequal(joint, 1:n)
        return;                                   % bereits richtig
    end
    if ~isequal(sort(joint), 1:n)
        error('parametrizeSpaceRobotModel:mux', '%s: Joint-Zuordnung unklar (%s).', mux, mat2str(joint));
    end
    for k = 1:n
        delete_line(get_param(ph.Inport(k), 'Line'));
    end
    for k = 1:n
        add_line(sys, srcPort(joint == k), ph.Inport(k), 'autorouting', 'on');
    end
    fprintf('[parametrizeSpaceRobotModel] %s: Eingaenge von joint %s auf joint 1..%d umverdrahtet.\n', ...
        muxName, mat2str(joint), n);
end

function idx = jointOfConverter(conv)
    pc = get_param(conv, 'PortConnectivity');
    for p = 1:numel(pc)
        for d = pc(p).DstBlock(:)'
            if d == -1, continue; end
            tok = regexp(get_param(d, 'Name'), '^joint(\d)$', 'tokens', 'once');
            if ~isempty(tok), idx = str2double(tok{1}); return; end
        end
    end
    error('parametrizeSpaceRobotModel:mux', 'Kein Joint an %s gefunden.', conv);
end

function trySet(blk, name, value)
    try
        set_param(blk, name, value);
    catch ME
        warning('parametrizeSpaceRobotModel:conf', '%s: %s nicht gesetzt (%s)', blk, name, ME.message);
    end
end

function ch = findRewardChart(mdl)
    m = sfroot().find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
    charts = m.find('-isa', 'Stateflow.EMChart');
    hit = arrayfun(@(c) contains(c.Script, 'rewardFcn'), charts);
    if nnz(hit) ~= 1
        error('parametrizeSpaceRobotModel:reward', ...
            'Reward-Block nicht eindeutig gefunden (%d Treffer).', nnz(hit));
    end
    ch = charts(hit);
end

function s = rewardScript()
% Gleiche Berechnung wie bisher (Gl. 1-4 im Paper), Werte aus rewardW statt fest.
    L = {
    'function [reward,isDone] = rewardFcn(ep, ev, vbase, wbase, tau, tauPrev, e_ori)'
    '    % Gewichte und Konstanten: Parameter rewardW (benchmarkConfig.reward, Tab. 3)'
    '    persistent prev_dist;'
    '    if isempty(prev_dist), prev_dist = inf; end'
    ''
    '    % --- Fruehcheck auf Finite ---'
    '    if any(~isfinite([ep;ev;vbase;wbase;e_ori;tau;tauPrev]))'
    '        reward = rewardW.rfail; isDone = true; prev_dist = inf; return;'
    '    end'
    ''
    '    % ---- Fortschritt ----'
    '    dist = norm(ep);'
    '    prog = 0;'
    '    if isfinite(prev_dist)'
    '        d = prev_dist - dist;'
    '        if d>0, prog = rewardW.kp*d; end'
    '    end'
    '    prev_dist = dist;'
    ''
    '    % ---- Kosten ----'
    '    cost  = 0;'
    '    cost  = cost + rewardW.wp  * (ep.''*ep);'
    '    cost  = cost + rewardW.wv  * (ev.''*ev);'
    '    cost  = cost + rewardW.wori* (e_ori.''*e_ori);'
    '    cost  = cost + rewardW.wwb * (wbase.''*wbase);'
    '    cost  = cost + rewardW.wvb * (vbase.''*vbase);'
    '    cost  = cost + rewardW.wu  * (tau.''*tau);'
    '    cost  = cost + rewardW.wd  * ((tau - tauPrev).''*(tau - tauPrev));'
    ''
    '    % Bonus nahe Bahnpunkt'
    '    bonus = rewardW.kb * exp( - (dist/rewardW.sigma)^2 );'
    ''
    '    reward = (prog + bonus) - cost;'
    '    reward = reward / rewardW.C;'
    ''
    '    % Abbruch'
    '    isDone = false;'
    '    if isnan(reward) || dist > rewardW.dmax'
    '        isDone = true; reward = rewardW.rfail; prev_dist = inf;'
    '    end'
    'end'
    };
    s = strjoin(L, newline);
end
