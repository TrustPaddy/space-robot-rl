
%% bo.m  -  Launcher fuer die skriptbasierte Bayes-Hyperparameter-Optimierung
%
% Diese Datei ist nur der Einstiegspunkt. Die eigentliche Logik steckt in:
%   setupSpaceRobotEnv.m  - Umgebungs-Aufbau (worker-sicher)
%   getSearchSpace.m      - Suchraum je Agent
%   buildAgent.m          - Agent + korrekte Options je Agent
%   trainAndEvaluate.m    - bayesopt-Zielfunktion (Train + deterministische Eval)
%   optimizeAgent.m       - Treiber: Pool + parallele Trials + Speichern
%
% Strategie: PARALLELE TRIALS (bayesopt 'UseParallel' = true), je Trial
% sequentielles Training. Die Env wird pro Worker genau einmal gebaut.

clc;

% --- Projektpfade sicherstellen (falls "startup" noch nicht lief) ---
if exist('setupSpaceRobotEnv','file') ~= 2
    proot = fileparts(fileparts(fileparts(mfilename('fullpath'))));  % src/bo -> src -> Root
    run(fullfile(proot, 'startup.m'));
end

% --- Welchen Agenten optimieren? ---
agentType = "PPO" + ...
    "";     % "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"

% --- (Optional) Parallel-Pool vorab starten, sonst macht optimizeAgent das ---
% parpool(8);

% --- Optimierung starten (Defaults: 8 Worker, 60 Trials, 500 Episoden) ---
results = optimizeAgent(agentType, ...
    'MaxObjectiveEvaluations', 60, ...    % Anzahl HP-Kombinationen
    'MaxEpisodes',             500, ...   % Screening-Episoden pro Trial
    'EvalEpisodes',            5, ...      % deterministische Eval-Episoden
    'EvalFrequency',           20, ...     % Eval alle N Episoden
    'UseParallel',             true, ...   % parallele Trials
    'NumWorkers',              8, ...      % 8-Kern-CPU -> 8 Worker
    'MaxTime',                 3*3600);   % Sicherheitsnetz

% Ergebnis inspizieren:
%   bestPoint(results)
%   Bester Agent liegt in SavedAgents/Circular/BO/<Agent>/<timestamp>/bestAgent.mat
