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
%     q_lim      Gelenkgrenzen [rad] (4x2) fuer den Episodenabbruch

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
    % Stateflow gleicht beim Setzen des Codes die Daten nach POSITION ab (z. B.
    % wurde rewardW zu einem Parameter "isColl"). Deshalb danach Namen, Scope
    % und Portreihenfolge ausdruecklich auf den Sollzustand bringen.
    ch = findRewardChart(mdl);
    ch.Script = rewardScript();
    reconcileData(ch, ...
        {'ep','ev','vbase','wbase','tau','tauPrev','e_ori','isColl','q'}, ...
        {'reward','isDone'}, ...
        {'rewardW','q_lim'});

    % ---- 5) Gelenkreihenfolge in q und dq ----
    % Die Mux-Bloecke fuer q ('Mux') und dq ('Mux1') waren in umgekehrter
    % Reihenfolge verdrahtet ([joint4 ... joint1]), die Momente dagegen in
    % richtiger. Folge: Kollisionsmonitor pruefte eine falsche Armstellung,
    % K6 verwendete die Grenzen falscher Gelenke, K9 multiplizierte tau_i mit
    % dq_(5-i). Danach gilt q = [q1 q2 q3 q4], dq = [dq1 dq2 dq3 dq4].
    orderMuxByJoint([mdl '/Robot'], 'Mux');
    orderMuxByJoint([mdl '/Robot'], 'Mux1');

    % ---- 6) Episodenabbruch bei Kollision oder Gelenkgrenze ----
    % Vorher setzte eine Kollision nur die Momente auf null, und es gab keinen
    % Abbruch bei Gelenkgrenzen. Jetzt bekommt der Reward-Block isColl
    % (Kollisionsmonitor) und q (Robot) und beendet die Episode mit rfail.
    addRewardInput(mdl, 'isColl', 'collision monitor/1');
    addRewardInput(mdl, 'q',      'Robot/1');

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

function reconcileData(ch, inputs, outputs, params)
% Bringt die Stateflow-Daten der Funktion auf genau diese Liste: Eingaenge und
% Ausgaenge in der angegebenen Portreihenfolge, dazu die Parameter. Nicht
% erwartete Daten werden geloescht.
    expected = [inputs, outputs, params];
    for x = ch.find('-isa', 'Stateflow.Data')'
        if ~ismember(x.Name, expected), delete(x); end
    end
    setGroup(ch, inputs,  'Input',     true);
    setGroup(ch, outputs, 'Output',    true);
    setGroup(ch, params,  'Parameter', false);
end

function setGroup(ch, names, scope, withPort)
    for k = 1:numel(names)
        d = ch.find('-isa', 'Stateflow.Data', 'Name', names{k});
        if isempty(d)
            d = Stateflow.Data(ch);
            d.Name = names{k};
        end
        if ~strcmp(d.Scope, scope), d.Scope = scope; end
        if withPort && d.Port ~= k, d.Port = k; end
    end
end

function addRewardInput(mdl, name, topSrc)
% Legt im Reward-Subsystem einen Eingang 'name' an (Inport -> Rate Transition
% auf Ts_agent -> gleichnamiger Eingang der MATLAB Function) und verbindet
% ihn auf oberster Ebene mit topSrc ('Block/Port').
    rw = [mdl '/Reward'];
    if ~isempty(find_system(rw, 'SearchDepth', 1, 'BlockType', 'Inport', 'Name', name))
        return;                                   % bereits vorhanden
    end
    ref = get_param([rw '/ori_base'], 'Position');
    nIn = numel(find_system(rw, 'SearchDepth', 1, 'BlockType', 'Inport'));
    dy  = 40 * (nIn - 6);
    inBlk = add_block('simulink/Sources/In1', [rw '/' name], ...
        'Position', ref + [0 dy 0 dy]);
    rtName = sprintf('Rate Transition %s', name);
    rtRef  = get_param([rw '/Rate Transition7'], 'Position');
    add_block([rw '/Rate Transition7'], [rw '/' rtName], ...
        'Position', rtRef + [0 dy 0 dy]);
    fcnPort = fcnInputIndex(mdl, name);
    add_line(rw, [get_param(inBlk, 'Name') '/1'], [rtName '/1'], 'autorouting', 'on');
    add_line(rw, [rtName '/1'], sprintf('MATLAB Function/%d', fcnPort), 'autorouting', 'on');
    add_line(mdl, topSrc, sprintf('Reward/%s', get_param(inBlk, 'Port')), 'autorouting', 'on');
end

function idx = fcnInputIndex(mdl, name)
% Portnummer des Eingangs 'name' der Reward-Funktion (aus ihren Stateflow-Daten).
    d = findRewardChart(mdl).find('-isa', 'Stateflow.Data', 'Name', name);
    if numel(d) ~= 1 || ~strcmp(d.Scope, 'Input')
        error('parametrizeSpaceRobotModel:reward', 'Eingang "%s" der Reward-Funktion nicht gefunden.', name);
    end
    idx = d.Port;
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
    'function [reward,isDone] = rewardFcn(ep, ev, vbase, wbase, tau, tauPrev, e_ori, isColl, q)'
    '    % Gewichte und Konstanten: Parameter rewardW (benchmarkConfig.reward, Tab. 3)'
    '    % Gelenkgrenzen: Parameter q_lim [rad], Zeile j = [min max] von Gelenk j'
    '    persistent prev_dist k;'
    '    if isempty(prev_dist), prev_dist = inf; end'
    '    if isempty(k), k = 0; end'
    '    k = k + 1;                                   % Aufruf-Zaehler (1 je Agentenschritt)'
    ''
    '    % Terminalstrafe: rfail, bei fail_remaining = 1 fuer jeden verbleibenden'
    '    % Schritt (sonst lohnt sich ein frueher Abbruch bei negativen Rewards)'
    '    rfailTot = rewardW.rfail * (1 + rewardW.fail_remaining * max(0, rewardW.N - k));'
    ''
    '    % --- Fruehcheck auf Finite ---'
    '    if any(~isfinite([ep;ev;vbase;wbase;e_ori;tau;tauPrev]))'
    '        reward = rfailTot; isDone = true; prev_dist = inf; return;'
    '    end'
    ''
    '    % --- Sicherheitsverletzung: Kollision oder Gelenkgrenze -> Abbruch ---'
    '    qc = q(:);'
    '    if isColl || any(qc < q_lim(:,1)) || any(qc > q_lim(:,2))'
    '        reward = rfailTot; isDone = true; prev_dist = inf; return;'
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
    '        isDone = true; reward = rfailTot; prev_dist = inf;'
    '    end'
    'end'
    };
    s = strjoin(L, newline);
end
