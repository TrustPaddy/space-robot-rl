function S = setupSpaceRobotEnv(cfg)
% setupSpaceRobotEnv  Baut die SpaceRobot-RL-Umgebung worker-sicher auf.
%
%   S = setupSpaceRobotEnv()      % Nominalkonfiguration aus benchmarkConfig
%   S = setupSpaceRobotEnv(cfg)   % cfg: vollstaendig (benchmarkConfig) oder
%                                 % Struct mit Ueberschreibungen
%
%   Alle Werte kommen aus benchmarkConfig. Diese Funktion schreibt jede
%   Variable, die SpaceRobot.slx zur Simulationszeit liest, in den
%   (Worker-)Base-Workspace; das Modell wird geladen (nicht geoeffnet -> keine
%   GUI auf Workern). Identisch auf Client und Parallel-Workern.
%
%   Vom Modell gelesene Variablen:
%     Ts, Ts_agent                 Rate Transitions, Reward-Block
%     EE_ref, EE_vref              Referenz (From Workspace)
%     robot_rbt, d_safe            Kollisionsmonitor
%     tau_sat                      Saturation vor der Strecke
%     robotP                       Massen, Traegheiten, Gelenkdaempfung (Inertia- und
%                                  Visual-Bloecke, Impulsmonitor)
%     rewardW, q_lim               Reward-Gewichte und Gelenkgrenzen (Parameter
%                                  des Reward-Blocks; Abbruch bei Verletzung)
%     q0, dq0                      Gelenk-Startzustand (von der ResetFcn je Episode gesetzt)
%     obs_noise                    Messrauschen je Agentenschritt (Zeile) und
%                                  Beobachtung (Spalte), Block 'obs noise'; nominal 0,
%                                  bei noise.obs_std > 0 von der ResetFcn je Episode gesetzt
%     tauDist                      aeusseres Gelenkmoment (Block 'joint disturbance')
%
%   Rueckgabe S mit Feldern:
%     env, obsInfo, actInfo, Ts_agent, Ts, T, mdl, agentBlk, cfg, noise
%     (noise: Einstellungen fuer localResetFunction, siehe evaluateAgent)

    if nargin < 1 || isempty(cfg), cfg = struct(); end
    cfg = benchmarkConfig(cfg);

    T        = cfg.T;
    Ts       = cfg.Ts;
    Ts_agent = cfg.Ts_agent;
    mdl      = cfg.mdl;
    nJ       = 4;

    % ---- Sicherheits-/Spec-Parameter ----
    assignin('base','d_safe',  cfg.d_safe);
    assignin('base','tau_max', cfg.tau_max);
    assignin('base','tau_sat', cfg.tau_sat_scale * cfg.tau_max);
    assignin('base','dt_agent',0.05);                       % Legacy
    assignin('base','q1_lim',  deg2rad(cfg.q1_lim_deg));
    assignin('base','qi_lim',  deg2rad(cfg.qi_lim_deg));
    % Gelenkgrenzen fuer den Episodenabbruch im Reward-Block (4x2, [min max])
    assignin('base','q_lim',   deg2rad([cfg.q1_lim_deg; repmat(cfg.qi_lim_deg, 3, 1)]));

    % Schrittzeiten MUESSEN im Base-Workspace liegen: die Simulink-Bloecke
    % 'Rate Transition'/'Rate Transition1'/'collision monitor/Rate Transition'
    % (OutPortSampleTime) und 'Reward/MATLAB Function' (SystemSampleTime)
    % werten 'Ts' bzw. 'Ts_agent' zur Simulationszeit aus. Fehlen sie, bricht
    % JEDE Simulation beim Kompilieren ab -> im bayesopt-Lauf still als 1e6.
    assignin('base','Ts',Ts);
    assignin('base','Ts_agent',Ts_agent);

    % ---- Roboterparameter und Reward-Gewichte ----
    % m_*/I_* sind Gesamtwerte je Koerper (Impulsmonitor, Paper-Tabelle). Im
    % Modell ist jeder Koerper auf zwei starr verbundene Bloecke mit gleichem
    % Schwerpunkt verteilt, Inertia (*_body) und Solid 'Visual' (*_geo), im
    % Verhaeltnis des urspruenglichen Modells: Basis 5:1, Glieder 1:1. Das
    % aendert die Dynamik nicht, und die Werte sind exakt (30/6 = 5, 30-5 = 25,
    % 0.2/2 = 0.1), sodass die Ergebnisse bitgleich zu benchmark_v2 bleiben.
    s = cfg.robot.param_scale;
    robotP = struct( ...
        'm_base',        s * cfg.robot.m_base, ...
        'I_base',        s * cfg.robot.I_base, ...
        'm_link',        s * cfg.robot.m_link, ...
        'I_link',        s * cfg.robot.I_link, ...
        'joint_damping', s * cfg.robot.joint_damping);
    robotP.m_base_geo  = robotP.m_base / 6;
    robotP.I_base_geo  = robotP.I_base / 6;
    robotP.m_base_body = robotP.m_base - robotP.m_base_geo;
    robotP.I_base_body = robotP.I_base - robotP.I_base_geo;
    robotP.m_link_geo  = robotP.m_link / 2;
    robotP.I_link_geo  = robotP.I_link / 2;
    robotP.m_link_body = robotP.m_link - robotP.m_link_geo;
    robotP.I_link_body = robotP.I_link - robotP.I_link_geo;
    assignin('base','robotP',robotP);
    rewardW   = cfg.reward;
    rewardW.N = floor(T/Ts_agent);          % Agentenschritte je Episode (Horizont der Terminalstrafe)
    assignin('base','rewardW',rewardW);

    % ---- Startzustand (ResetFcn ueberschreibt q0 je Episode) ----
    assignin('base','q0',      cfg.init.q0(:));
    assignin('base','dq0',     zeros(nJ,1));
    assignin('base','base_v0', zeros(3,1));
    assignin('base','base_w0', zeros(3,1));
    assignin('base','phi0',    0);
    assignin('base','reward_init', 0);
    assignin('base','isdone_init', 0);

    % ---- Referenztrajektorie ----
    t = 0:Ts:T;
    [EE_ref, EE_vref] = referenceTrajectory(cfg, t);
    assignin('base','EE_ref',EE_ref);
    assignin('base','EE_vref',EE_vref);

    % ---- Roboter (Kollisionsmonitor) ----
    robot_rbt = importrobot('SpaceRobot.urdf');
    robot_rbt.DataFormat = 'row';
    assignin('base','q_des',zeros(numel(t), nJ));   % Legacy
    % Der 'collision monitor/MATLAB Function'-Block wertet 'robot_rbt' zur
    % Laufzeit aus -> muss ebenfalls im Base-Workspace liegen (sonst Sim-Abbruch).
    assignin('base','robot_rbt',robot_rbt);

    % ---- Modell laden + konfigurieren (kein open_system auf Workern) ----
    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end
    set_param(mdl, ...
        'StopTime',   num2str(T), ...
        'Solver',     'ode4', ...           % fester Integrator
        'FixedStep',  num2str(Ts), ...
        'SolverType', 'Fixed-step', ...
        'SimMechanicsOpenEditorOnUpdate', 'off');   % kein Mechanics Explorer
        % (bremst, und im -batch-Betrieb stuerzte MATLAB im Explorer-Playback ab)

    agentBlk = [mdl '/RL_Agent'];

    % ---- Observation & Action Definition ----
    % Reihenfolge wie am Observation-Mux im Modell:
    %   [e_p(3); e_v(3); v_base(3); w_base(3); q(4); dq(4); e_ori(3)] = 23
    ePLim=0.5; eVLim=1.0; qLim=pi; dqLim=3; vBLim=0.5; wBLim=1.0; eOriLim=pi;
    obsLow  = [-ePLim*ones(3,1); -eVLim*ones(3,1); -vBLim*ones(3,1); -wBLim*ones(3,1); ...
               -qLim*ones(nJ,1); -dqLim*ones(nJ,1); -eOriLim*ones(3,1)];
    obsHigh = -obsLow;
    obsInfo = rlNumericSpec([numel(obsLow) 1], LowerLimit=obsLow, UpperLimit=obsHigh, Name="obs");

    actInfo = rlNumericSpec([nJ 1], ...
        'Name','tau', ...
        'LowerLimit', -cfg.tau_max*ones(nJ,1), ...
        'UpperLimit',  cfg.tau_max*ones(nJ,1));

    % ---- Stoerungen (Stresstests, nominal aus) ----
    % Nominal addieren beide Bloecke exakt 0 -> Ergebnisse bitgleich zum Modell
    % ohne diese Bloecke. obs_noise hat eine Zeile je Aufruf des Blocks
    % (t = 0, Ts_agent, ..., T).
    noise = struct('obs_std', cfg.noise.obs_std, 'seed', cfg.noise.seed, ...
                   'size', [rewardW.N + 1, numel(obsLow)]);
    assignin('base','obs_noise', zeros(noise.size));
    assignin('base','tauDist', struct('tau', cfg.dist.tau(:), ...
        't_on', cfg.dist.t_on, 't_off', cfg.dist.t_off));

    % ---- RL-Umgebung ----
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    initCfg = cfg.init;
    env.ResetFcn = @(in) localResetFunction(in, initCfg, noise);

    % ---- Rueckgabe ----
    S = struct('env',env, 'obsInfo',obsInfo, 'actInfo',actInfo, ...
               'Ts_agent',Ts_agent, 'Ts',Ts, 'T',T, ...
               'mdl',mdl, 'agentBlk',agentBlk, 'cfg',cfg, 'noise',noise);
end
