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
%     robotP.*   Inertia- und Visual-Bloecke (base_link, link1-4), Gelenkdaempfung
%     q0, dq0    Positions-/Geschwindigkeits-Targets der vier Gelenke
%     tau_sat    Saturation vor der Strecke
%     rewardW.*  Gewichte und Konstanten im Reward-Block (Parameter-Daten)
%     q_lim      Gelenkgrenzen [rad] (4x2) fuer den Episodenabbruch
%     robotP.*   auch im Impulsmonitor (vorher fest 5 kg Basismasse, A27)
%     obs_noise  Messrauschen auf der Beobachtung (Block 'obs noise')
%     tauDist    aeusseres Gelenkmoment (Block 'joint disturbance')
%   Nominal (obs_noise = 0, tauDist.tau = 0) addieren die beiden neuen Bloecke
%   exakt 0, die Ergebnisse bleiben bitgleich zum Modell ohne sie.

    if nargin < 1, mdl = 'SpaceRobot'; end
    load_system(mdl);

    % ---- 1) Massen und Traegheiten ----
    % Auch die Solid-Bloecke 'Visual' aus dem URDF-Import tragen Masse (fest
    % 5 kg / [1 1 1] an der Basis, 1 kg / [0.1 0.1 0.1] je Glied). Die
    % simulierten Koerper hatten also 30 kg bzw. 2 kg, nicht 25 kg bzw. 1 kg,
    % und param_scale skalierte nur einen Teil. Jetzt kommen beide Bloecke aus
    % robotP (Aufteilung der Gesamtwerte in setupSpaceRobotEnv).
    set_param([mdl '/Robot/base_link/Inertia'], ...
        'Mass', 'robotP.m_base_body', 'MomentsOfInertia', 'robotP.I_base_body');
    set_param([mdl '/Robot/base_link/Visual'], 'InertiaType', 'Custom', ...
        'Mass', 'robotP.m_base_geo', 'MomentsOfInertia', 'robotP.I_base_geo');
    for i = 1:4
        set_param(sprintf('%s/Robot/link%d/Inertia', mdl, i), ...
            'Mass', 'robotP.m_link_body', 'MomentsOfInertia', 'robotP.I_link_body');
        set_param(sprintf('%s/Robot/link%d/Visual', mdl, i), 'InertiaType', 'Custom', ...
            'Mass', 'robotP.m_link_geo', 'MomentsOfInertia', 'robotP.I_link_geo');
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

    % ---- 7) Impulsmonitor: Massen aus robotP (A27) ----
    % totalMomentum rechnete mit fest eingetragenen Massen [5 1 1 1 1] kg
    % (simuliert: 30 kg Basis, 2 kg je Glied). Jetzt die Gesamtmassen aus
    % robotP. p_tot wird zusaetzlich geloggt (vorher nur Scope).
    ch = findChart(mdl, 'totalMomentum');
    newM = 'm = [robotP.m_base; robotP.m_link*ones(4,1)];   % Gesamtmasse je Koerper (robotP)';
    s = regexprep(ch.Script, 'm = \[[^\]\n]*\];[^\n]*', newM, 'once');
    if ~contains(s, newM)
        error('parametrizeSpaceRobotModel:momentum', 'Massenzeile im Impulsmonitor nicht gefunden.');
    end
    if ~strcmp(s, ch.Script), ch.Script = s; end
    in = cell(1, 15);
    for i = 1:5
        in(3*i-2:3*i) = {sprintf('v%d', i), sprintf('w%d', i), sprintf('q%d', i)};
    end
    reconcileData(ch, in, {'p_tot'}, {'robotP'});
    ph = get_param([mdl '/Robot/Impuls Monitor/MATLAB Function'], 'PortHandles');
    set_param(ph.Outport(1), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'p_tot');

    % ---- 8) Messrauschen auf der Beobachtung (Stresstest) ----
    % Zwischen 'Rate Transition1' (Agententakt) und RL_Agent: y = u + Zeile k
    % von obs_noise im k-ten Agentenschritt. Reward und KPIs bleiben ungestoert.
    % Erst Code setzen (legt die Ports an), dann verdrahten.
    blk = [mdl '/obs noise'];
    isNew = getSimulinkBlockHandle(blk) == -1;
    if isNew
        rt1 = [mdl '/Rate Transition1'];
        set_param(rt1, 'Position', get_param(rt1, 'Position') + [80 0 80 0]);
        add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
            'Position', [895 353 955 377], 'Orientation', get_param(rt1, 'Orientation'));
    end
    ch = chartOf(blk);
    ch.Script = obsNoiseScript();
    reconcileData(ch, {'u'}, {'y'}, {'obs_noise'});
    set_param(blk, 'SystemSampleTime', 'Ts_agent');
    if isNew
        delete_line(mdl, 'Rate Transition1/1', 'RL_Agent/1');
        add_line(mdl, 'Rate Transition1/1', 'obs noise/1', 'autorouting', 'on');
        add_line(mdl, 'obs noise/1', 'RL_Agent/1', 'autorouting', 'on');
    end

    % ---- 9) Aeusseres Gelenkmoment (Stresstest) ----
    % Zwischen 'Rate Transition' (Moment des Agenten, Takt Ts) und Robot:
    % tau_robot = tau + tauDist.tau fuer tauDist.t_on <= t < tauDist.t_off.
    % Reward, Unit Delay und das geloggte tau sehen weiter nur das Agentenmoment.
    blk = [mdl '/joint disturbance'];
    clk = [mdl '/Digital Clock'];
    isNew = getSimulinkBlockHandle(blk) == -1;
    if isNew
        add_block('simulink/Sources/Digital Clock', clk, ...
            'SampleTime', 'Ts', 'Position', [255 205 285 225]);
        add_block('simulink/User-Defined Functions/MATLAB Function', blk, ...
            'Position', [320 175 380 215]);
    end
    ch = chartOf(blk);
    ch.Script = jointDisturbanceScript();
    reconcileData(ch, {'tau', 't'}, {'tau_out'}, {'tauDist'});
    set_param(blk, 'SystemSampleTime', 'Ts');
    if isNew
        delete_line(mdl, 'Rate Transition/1', 'Robot/1');
        add_line(mdl, 'Rate Transition/1', 'joint disturbance/1', 'autorouting', 'on');
        add_line(mdl, 'Digital Clock/1',   'joint disturbance/2', 'autorouting', 'on');
        add_line(mdl, 'joint disturbance/1', 'Robot/1', 'autorouting', 'on');
    end

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
    ch = findChart(mdl, 'rewardFcn');
end

function ch = findChart(mdl, fcnName)
% MATLAB-Function-Block, dessen Code die Funktion fcnName enthaelt.
    m = sfroot().find('-isa', 'Simulink.BlockDiagram', 'Name', mdl);
    charts = m.find('-isa', 'Stateflow.EMChart');
    hit = arrayfun(@(c) contains(c.Script, fcnName), charts);
    if nnz(hit) ~= 1
        error('parametrizeSpaceRobotModel:chart', ...
            'Block mit "%s" nicht eindeutig gefunden (%d Treffer).', fcnName, nnz(hit));
    end
    ch = charts(hit);
end

function ch = chartOf(blk)
% Stateflow-Objekt des MATLAB-Function-Blocks blk (voller Pfad).
    ch = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', blk);
    if numel(ch) ~= 1
        error('parametrizeSpaceRobotModel:chart', 'Kein MATLAB-Function-Block %s.', blk);
    end
end

function s = obsNoiseScript()
    L = {
    'function y = addObsNoise(u)'
    '    % Messrauschen auf der Beobachtung (Stresstest). Im k-ten Agentenschritt'
    '    % wird Zeile k des Parameters obs_noise addiert (gesetzt von'
    '    % localResetFunction). Nominal ist obs_noise = 0, dann gilt y = u.'
    '    persistent k;'
    '    if isempty(k), k = 0; end'
    '    k = k + 1;'
    '    y = u;'
    '    y(:) = u(:) + obs_noise(min(k, size(obs_noise, 1)), :).'';'
    'end'
    };
    s = strjoin(L, newline);
end

function s = jointDisturbanceScript()
    L = {
    'function tau_out = addJointDisturbance(tau, t)'
    '    % Aeusseres Gelenkmoment (Stresstest): tauDist.tau [N*m] je Gelenk wird im'
    '    % Zeitfenster tauDist.t_on <= t < tauDist.t_off addiert. Nominal ist'
    '    % tauDist.tau = 0, dann gilt tau_out = tau.'
    '    tau_out = tau;'
    '    if t >= tauDist.t_on && t < tauDist.t_off'
    '        tau_out(:) = tau(:) + tauDist.tau(:);'
    '    end'
    'end'
    };
    s = strjoin(L, newline);
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
