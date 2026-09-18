function S = setupSpaceRobotEnv(cfg)
% setupSpaceRobotEnv  Baut die SpaceRobot-RL-Umgebung worker-sicher auf.
%
%   S = setupSpaceRobotEnv()      % Standard: Kreisbahn wie in SpaceRobotDynamic.m
%   S = setupSpaceRobotEnv(cfg)   % cfg-Struct zum Ueberschreiben von Defaults
%
%   Diese Funktion kapselt den kompletten Umgebungs-Aufbau aus
%   SpaceRobotDynamic.m in EINER Funktion, damit sie 1:1 identisch auf dem
%   Client UND auf jedem Parallel-Worker laeuft. Alle Variablen, die die
%   Simulink-"From Workspace"-Bloecke und die ResetFcn brauchen, werden in
%   den (Worker-)Base-Workspace geschrieben; das Modell wird geladen (nicht
%   geoeffnet -> keine GUI auf Workern).
%
%   Rueckgabe S mit Feldern:
%     env, obsInfo, actInfo, Ts_agent, Ts, T, mdl, agentBlk
%
%   Wird von trainAndEvaluate.m / bo.m ueber parallel.pool.Constant genutzt,
%   sodass jeder Worker die Env genau einmal baut und wiederverwendet.

    if nargin < 1 || isempty(cfg), cfg = struct(); end

    % ---- Defaults (identisch zu SpaceRobotDynamic.m) ----
    def.T        = 8.5;          % Episodendauer [s]
    def.r        = 0.5;          % Kreisradius [m]
    def.Ts       = 0.01;         % diskrete Schrittzeit RL-Schleife [s]
    def.Ts_agent = 0.1;          % Agent-Sample-Time [s]
    def.mdl      = 'SpaceRobot';
    cfg = setDefaults(cfg, def);

    T        = cfg.T;
    r        = cfg.r;
    Ts       = cfg.Ts;
    Ts_agent = cfg.Ts_agent;
    mdl      = cfg.mdl;

    % ---- Sicherheits-/Spec-Parameter (vom Simulink-Modell erwartet) ----
    d_safe   = 0.02;                 % Mindestabstand [m]
    tau_max  = 2.0;                  % Aktuatorgrenze [N*m]
    dt_agent = 0.05;                 % (Legacy, vom Modell referenziert)
    q1_lim   = deg2rad([ -85  85 ]);
    qi_lim   = deg2rad([ -170 170 ]);

    assignin('base','d_safe',d_safe);
    assignin('base','tau_max',tau_max);
    assignin('base','dt_agent',dt_agent);
    assignin('base','q1_lim',q1_lim);
    assignin('base','qi_lim',qi_lim);

    % Schrittzeiten MUESSEN im Base-Workspace liegen: die Simulink-Bloecke
    % 'Rate Transition'/'Rate Transition1'/'collision monitor/Rate Transition'
    % (OutPortSampleTime) und 'Reward/MATLAB Function' (SystemSampleTime)
    % werten 'Ts' bzw. 'Ts_agent' zur Simulationszeit aus. Fehlen sie, bricht
    % JEDE Simulation beim Kompilieren ab -> im bayesopt-Lauf still als 1e6.
    assignin('base','Ts',Ts);
    assignin('base','Ts_agent',Ts_agent);

    % ---- Soll-Kreisbahn ----
    omega  = pi/T;
    center = [4.5-r, 0.0, 0.0];
    t = 0:Ts:T;

    x = center(1) + r*cos(omega*t);
    y = center(2) + r*sin(omega*t);
    z = center(3) + 0*t;
    traj = [x' y' z'];               % gewuenschte EE-Punkte

    dt   = mean(diff(t));
    vref = [zeros(1,3); diff(traj)/dt];
    EE_ref  = timeseries(traj, t);   % Nx3 Soll-Position
    EE_vref = timeseries(vref, t);   % Nx3 Soll-Geschwindigkeit
    assignin('base','EE_ref',EE_ref);
    assignin('base','EE_vref',EE_vref);

    % ---- Roboter / IK (Start-Konfiguration) ----
    % q_des bleibt (wie im Originalskript) Null; die ResetFcn nutzt eine feste
    % Null-Startpose. Wir legen q_des dennoch an, falls das Modell es referenziert.
    robot_rbt = importrobot('SpaceRobot.urdf');
    robot_rbt.DataFormat = 'row';
    nJ = 4;
    q_des = zeros(numel(t), nJ);
    assignin('base','q_des',q_des);
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
        'SolverType', 'Fixed-step');

    agentBlk = [mdl '/RL_Agent'];

    % ---- Observation & Action Definition ----
    ePLim=0.5; eVLim=1.0; qLim=pi; dqLim=3; vBLim=0.5; wBLim=1.0; eOriLim=pi;
    obsLow  = [-ePLim*ones(3,1); -eVLim*ones(3,1); -qLim*ones(nJ,1); ...
               -dqLim*ones(nJ,1); -vBLim*ones(3,1); -wBLim*ones(3,1); -eOriLim*ones(3,1)];
    obsHigh = -obsLow;
    obsInfo = rlNumericSpec([numel(obsLow) 1], LowerLimit=obsLow, UpperLimit=obsHigh, Name="obs");

    actInfo = rlNumericSpec([nJ 1], ...
        'Name','tau', ...
        'LowerLimit', -tau_max*ones(nJ,1), ...
        'UpperLimit',  tau_max*ones(nJ,1));

    % ---- RL-Umgebung ----
    env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
    env.ResetFcn = @localResetFunction;

    % ---- Rueckgabe ----
    S = struct('env',env, 'obsInfo',obsInfo, 'actInfo',actInfo, ...
               'Ts_agent',Ts_agent, 'Ts',Ts, 'T',T, ...
               'mdl',mdl, 'agentBlk',agentBlk);
end

function cfg = setDefaults(cfg, def)
    f = fieldnames(def);
    for i = 1:numel(f)
        if ~isfield(cfg, f{i}) || isempty(cfg.(f{i}))
            cfg.(f{i}) = def.(f{i});
        end
    end
end
