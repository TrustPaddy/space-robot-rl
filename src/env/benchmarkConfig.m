function cfg = benchmarkConfig(varargin)
% benchmarkConfig  Zentrale Konfiguration aller Experimente (Benchmark v2).
%
%   cfg = benchmarkConfig()                          % Nominalkonfiguration
%   cfg = benchmarkConfig('traj',"linear", 'reward.wori',400)
%   cfg = benchmarkConfig(overrides)                 % Struct, auch verschachtelt
%   cfg = benchmarkConfig(trainCfg, overrides)       % mehrere Structs, der Reihe nach
%
%   JEDE Zahl, die ein Experiment beeinflusst, steht hier - nicht fest im
%   Simulink-Modell und nicht verstreut in Einzelskripten. setupSpaceRobotEnv
%   schreibt daraus alle Variablen in den Base-Workspace, die SpaceRobot.slx
%   liest. Ergebnisdateien speichern cfg mit ab, damit jede Zahl im Paper
%   einer Konfiguration zugeordnet werden kann.
%
%   Ueberschreiben: Name-Wert-Paare mit Punkt-Notation fuer Unterfelder
%   ('reward.wori', 400) oder ein Struct mit derselben Feldstruktur.

    % ---- Zeit ----
    cfg.T        = 8.5;       % Episodendauer [s]
    cfg.Ts       = 0.01;      % Solver-Schrittweite (fixed-step, ode4) [s]
    cfg.Ts_agent = 0.1;       % Entscheidungsintervall des Agenten [s]

    % ---- Referenztrajektorie ----
    % Startpunkt ist immer x = 4.5 m (gestreckter Arm bei q = 0), daher
    % Kreismittelpunkt = [4.5 - r, 0, 0].
    cfg.traj = "circle";      % "circle" | "linear" (Zwei-Segment-Bahn, Gl. 5)
    cfg.r    = 0.5;           % Radius [m]

    % ---- Aktorik / Sicherheit ----
    cfg.tau_max       = 2.0;  % Momentengrenze = Aktionsraum des Agenten [N*m]
    cfg.tau_sat_scale = 1.0;  % Saturation im Modell = tau_sat_scale*tau_max
                              % (Stresstest "reduced saturation": 0.75)
    cfg.d_safe        = 0.02; % Mindestabstand Kollisionsmonitor [m]
    cfg.q1_lim_deg    = [-85 85];     % Gelenk 1
    cfg.qi_lim_deg    = [-170 170];   % Gelenke 2-4

    % ---- Roboterparameter (Simscape-Modell, identisch zu SpaceRobot.urdf) ----
    cfg.robot.m_base        = 25;          % Basis [kg]
    cfg.robot.I_base        = [5 5 5];     % Basis-Haupttraegheitsmomente [kg*m^2]
    cfg.robot.m_link        = 1;           % je Armglied [kg]
    cfg.robot.I_link        = [0.1 0.1 0.1];
    cfg.robot.joint_damping = 1.5;         % viskose Gelenkdaempfung [N*m*s/rad]
    cfg.robot.param_scale   = 1.0;         % Stresstest "parameter uncertainty":
                                           % skaliert Massen, Traegheiten, Daempfung

    % ---- Reward (Gl. 1-4 und Tab. 3 im Paper) ----
    cfg.reward.wp    = 150;    % EE-Position
    cfg.reward.wv    = 25;     % EE-Geschwindigkeit
    cfg.reward.wori  = 200;    % Basis-Orientierung
    cfg.reward.wwb   = 8;      % Basis-Winkelgeschwindigkeit
    cfg.reward.wvb   = 2;      % Basis-Lineargeschwindigkeit
    cfg.reward.wu    = 0.02;   % Momentenbetrag
    cfg.reward.wd    = 0.06;   % Momentenaenderung
    cfg.reward.kp    = 10;     % Progress-Skalierung (0 -> r_prog aus)
    cfg.reward.kb    = 0.05;   % Proximity-Bonus    (0 -> r_bonus aus)
    cfg.reward.sigma = 0.02;   % Breite Proximity-Bonus [m]
    cfg.reward.C     = 500;    % Normierung
    cfg.reward.rfail = -1;     % Terminal-Strafe je Schritt
    % 1 -> rfail fuer den Abbruchschritt UND jeden verbleibenden Schritt bis zum
    % Horizont. Mit nur einmal -1 lohnt sich ein frueher Abbruch, weil alle
    % Schritt-Rewards negativ sind (Pilot: 0 % vollstaendige Episoden vs. 100 %).
    cfg.reward.fail_remaining = 1;
    cfg.reward.dmax  = 10;     % Abbruch, wenn EE-Fehler > dmax [m]

    % ---- Startzustand (Training) ----
    % Jede Trainingsepisode startet in q0 + U(-range, +range) je Gelenk. Die
    % Auswertung setzt ihre Startzustaende selbst (evaluateAgent).
    cfg.init.q0        = zeros(4,1);   % nominale Startpose [rad] (gestreckter Arm)
    cfg.init.randomize = true;
    cfg.init.range_deg = 1.0;          % halbe Breite der Gleichverteilung [deg]

    % ---- Training ----
    cfg.train.maxEpisodes = 1000;
    cfg.train.seed        = 0;

    cfg.mdl = 'SpaceRobot';

    % ---- Ueberschreibungen anwenden ----
    if nargin > 0 && all(cellfun(@isstruct, varargin))
        for k = 1:nargin                     % mehrere Structs: der Reihe nach
            cfg = mergeStruct(cfg, varargin{k});
        end
    elseif nargin > 0
        if mod(nargin, 2) ~= 0
            error('benchmarkConfig:args', 'Erwartet Name-Wert-Paare oder ein Struct.');
        end
        for k = 1:2:nargin
            cfg = setPath(cfg, char(varargin{k}), varargin{k+1});
        end
    end
end

function a = mergeStruct(a, b)
% Uebernimmt alle Felder aus b in a (rekursiv). Unbekannte Felder sind ein
% Fehler, damit Tippfehler nicht still ignoriert werden.
    f = fieldnames(b);
    for i = 1:numel(f)
        if ~isfield(a, f{i})
            error('benchmarkConfig:unknownField', 'Unbekanntes Feld "%s".', f{i});
        end
        if isstruct(a.(f{i})) && isstruct(b.(f{i}))
            a.(f{i}) = mergeStruct(a.(f{i}), b.(f{i}));
        else
            a.(f{i}) = b.(f{i});
        end
    end
end

function s = setPath(s, name, value)
    parts = strsplit(name, '.');
    probe = s;
    for i = 1:numel(parts)
        if ~isstruct(probe) || ~isfield(probe, parts{i})
            error('benchmarkConfig:unknownField', 'Unbekanntes Feld "%s".', name);
        end
        probe = probe.(parts{i});
    end
    s = setfield(s, parts{:}, value);
end
