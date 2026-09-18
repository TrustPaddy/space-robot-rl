%%  1) Trajektorie/IKrobuster machen
clc; clear; close all;
rng(0,'twister')

% --- Projektpfade sicherstellen (falls "startup" noch nicht lief) ---
if exist('setupSpaceRobotEnv','file') ~= 2
    proot = fileparts(fileparts(fileparts(mfilename('fullpath'))));  % src/training -> src -> Root
    run(fullfile(proot, 'startup.m'));
end

% Sicherheits-/Spec-Parameter
d_safe   = 0.02;        % Mindestabstand [m]
tau_max  = 2.0;         % Aktuatorgrenze [N*m]
dt_agent = 0.05;        % Agent rate [s]
q1_lim   = deg2rad([ -85  85 ]);
qi_lim   = deg2rad([ -170 170 ]);

assignin('base','d_safe',d_safe);
assignin('base','tau_max',tau_max);
assignin('base','dt_agent',dt_agent);
assignin('base','q1_lim',q1_lim);
assignin('base','qi_lim',qi_lim);

% Soll-Kreisbahn definieren
T = 8.5;                
omega = pi/T;
r = 0.5;               
center = [4.5-r, 0.0, 0.0];  

% % Punkt anstelle Kreisbahn  
% r = 0;               
% center = [3.7, 0.0, 0.0];  

Ts  = 0.01;        % Diskrete Schrittzeit der RL-Schleife
Ts_agent = 0.1;
t   = 0:Ts:T;      % Zeitvektor (0…10 s, inkl. Ende ok für From Workspace)

x = center(1) + r*cos(omega*t);
y = center(2) + r*sin(omega*t);
z = center(3) + 0*t;          

% % Eckpunkte aus dem ursprünglichen Kreis (gleichbleibend)
% P0 = [center(1)+r, center(2),   center(3)]; % Start (t=0)
% P1 = [center(1),   center(2)+r, center(3)]; % "Höhepunkt" (t=T/4)
% P2 = [center(1)-r, center(2),   center(3)];                                 % Ende (t=T) = Start
% 
% % Zeitaufteilung: zwei Segmente (P0->P1 und P1->P2)
% t1 = T/2;  % Wechsel bei der Hälfte der Periode
% 
% % Vorbelegen
% x = zeros(size(t));
% y = zeros(size(t));
% z = zeros(size(t));
% 
% % Segment 1: P0 -> P1 für t in [0, T/2]
% idx1 = (t <= t1);
% s1 = (t(idx1) - 0) / (t1 - 0);            % s in [0,1]
% x(idx1) = P0(1) + s1*(P1(1) - P0(1));
% y(idx1) = P0(2) + s1*(P1(2) - P0(2));
% z(idx1) = P0(3) + s1*(P1(3) - P0(3));
% 
% % Segment 2: P1 -> P2 für t in (T/2, T]
% idx2 = (t > t1);
% s2 = (t(idx2) - t1) / (T - t1);           % s in [0,1]
% x(idx2) = P1(1) + s2*(P2(1) - P1(1));
% y(idx2) = P1(2) + s2*(P2(2) - P1(2));
% z(idx2) = P1(3) + s2*(P2(3) - P1(3));

traj = [x' y' z'];   % gewünschte EE-Punkte

% Roboter laden
robot_rbt = importrobot('SpaceRobot.urdf');
robot_rbt.DataFormat = 'row';
eeBodyName = robot_rbt.BodyNames{end};

%robot_col = robot_rbt;

% Inverse Kinematik vorbereiten
ik = inverseKinematics('RigidBodyTree', robot_rbt); 

qseed = randomConfiguration(robot_rbt);
q_des = zeros(numel(t), numel(qseed));


% % Animation % figure; ax = axes; % plot3(ax, traj(:,1), traj(:,2), traj(:,3), 'r--','LineWidth',1.5); hold on; % grid(ax,'on'); axis(ax,'equal'); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); % title('Endeffektor-Kreisbahn mit Inverser Kinematik'); % % for k = 1:(0.1/Ts):numel(t) % show(robot_rbt, q_des(k,:), 'Parent', ax, 'PreservePlot', false); % view(2) % xlim([-1, 5]) % ylim([-2, 2]) % zlim([-2, 2]) % drawnow; % end

dt = mean(diff(t));
vref = [zeros(1,3); diff(traj)/dt];   % einfache Ableitung
EE_ref = timeseries(traj, t);         % Nx3
EE_vref = timeseries(vref, t);        % Nx3
assignin('base','EE_ref',EE_ref);
assignin('base','EE_vref',EE_vref);


mdl = 'SpaceRobot';
set_param(mdl, ...
    'StopTime', num2str(T), ...
    'Solver', 'ode4', ...        % fester Integrator
    'FixedStep', num2str(Ts), ...     
    'SolverType', 'Fixed-step' ...
);

open_system(mdl)
agentBlk = [mdl '/RL_Agent'];



%% 2) Observation & Action Definition
nJ = 4;                              % Anzahl der Gelenke 
obsDim = 3 + 3 + 2*nJ + 6 + 3;           % e_p,e_v,q,dq,v_base,w_base,ori_base -> = 23

ePLim=0.5; eVLim=1.0; qLim=pi; dqLim=3; vBLim=0.5; wBLim=1.0; eOriLim=pi; % rad
obsLow  = [-ePLim*ones(3,1); -eVLim*ones(3,1); -qLim*ones(nJ,1); -dqLim*ones(nJ,1); -vBLim*ones(3,1); -wBLim*ones(3,1); -eOriLim*ones(3,1)];
obsHigh = -obsLow;
obsInfo = rlNumericSpec([numel(obsLow) 1], LowerLimit=obsLow, UpperLimit=obsHigh, Name="obs");

actInfo = rlNumericSpec([nJ 1], ...
    'Name','tau', ...
    'LowerLimit', -2*ones(nJ,1), ...
    'UpperLimit', 2*ones(nJ,1));

% 3) RL-Umgebung mit Simulink verknüpfen
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @localResetFunction;   % Reset-Funktion (kommt unten)


% ===== ALT / REFERENZ: manuelle Netz-Definition (aktiv jetzt in Abschnitt 4a unten) =====
% %% 4) Actor (Policy) und Critic (Value) Netzwerke) ----
% obsPath = featureInputLayer(obsDim,'Normalization','none','Name','obs');
% 
% commonPath = [
%     fullyConnectedLayer(128,'Name','fc1')
%     reluLayer('Name','relu1')
%     fullyConnectedLayer(128,'Name','fc2')
%     reluLayer('Name','relu2')
% ];
% 
% meanPath = [
%     fullyConnectedLayer(nJ,'Name','mean')
% ];  
% 
% stdPath = [
%     fullyConnectedLayer(nJ,'Name','std')
%     softplusLayer('Name','softplus')
% ];
% 
% actorLG = layerGraph(obsPath);
% actorLG = addLayers(actorLG, commonPath);
% actorLG = addLayers(actorLG, meanPath);
% actorLG = addLayers(actorLG, stdPath);
% 
% actorLG = connectLayers(actorLG,'obs','fc1');
% actorLG = connectLayers(actorLG,'relu2','mean');
% actorLG = connectLayers(actorLG,'relu2','std');
% 
% actor = rlContinuousGaussianActor(actorLG, obsInfo, actInfo, ...
%     ActionMeanOutputNames="mean", ...
%     ActionStandardDeviationOutputNames="softplus" ...
% );
% 
% criticLG = layerGraph([
%     featureInputLayer(obsDim,'Normalization','none','Name','obs')
%     fullyConnectedLayer(128,'Name','fc1')
%     reluLayer('Name','relu1')
%     fullyConnectedLayer(128,'Name','fc2')
%     reluLayer('Name','relu2')
%     fullyConnectedLayer(1,'Name','Value') ...
% ]);
% 
% critic = rlValueFunction(criticLG, obsInfo, 'Observation','obs');
% 
% 
% %% 5) PPO-Agent definieren
% agentOpts = rlPPOAgentOptions(...
%     'SampleTime', Ts_agent, ...
%     'ExperienceHorizon', 1024, ...
%     'MiniBatchSize', 256, ...
%     'NumEpoch', 10, ...
%     'EntropyLossWeight', 1e-3 ...
%  );
% 
% agent = rlPPOAgent(actor, critic, agentOpts);
% 
% agent.AgentOptions.CriticOptimizerOptions.LearnRate = 5e-4;
% agent.AgentOptions.ActorOptimizerOptions.LearnRate = 1e-3;
% 
% 
% assignin('base','agent',agent);   

%% 4) Modus & Agent-Auswahl
mode      = "optimized";   % "optimized" (BO-HPs + explizite Netze aus 4a) | "default" (MATLAB-Default-HPs + Standardnetz)
agentType = "PPO";         % "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"
numHidden = 128;           % Default-Modus: 2 Hidden-Layer je numHidden Neuronen + ReLU
                           % (im Wunsch stand 182 -> hier bewusst 128, konsistent zum Projekt; bei Bedarf aendern)

agentType = upper(string(agentType));
onPolicy  = any(agentType == ["PG","PPO","TRPO"]);

%% 4a) Explizite Netze (nur "optimized" + On-Policy): Gaussian-Actor + Value-Critic
nets = [];
if mode == "optimized" && onPolicy
    % Gemeinsamer Feature-Pfad (2x128 + ReLU)
    actorNet = layerGraph([
        featureInputLayer(obsDim,'Normalization','none','Name','obs')
        fullyConnectedLayer(128,'Name','fc1')
        reluLayer('Name','relu1')
        fullyConnectedLayer(128,'Name','fc2')
        reluLayer('Name','relu2') ]);
    % Zwei Koepfe: Mittelwert + Streuung (softplus -> positiv)
    actorNet = addLayers(actorNet, fullyConnectedLayer(nJ,'Name','mean'));
    actorNet = addLayers(actorNet, [ fullyConnectedLayer(nJ,'Name','stdFc'); softplusLayer('Name','std') ]);
    actorNet = connectLayers(actorNet,'relu2','mean');
    actorNet = connectLayers(actorNet,'relu2','stdFc');
    actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
        ActionMeanOutputNames="mean", ActionStandardDeviationOutputNames="std");
    % State-Value-Critic (2x128 + ReLU)
    criticNet = [
        featureInputLayer(obsDim,'Normalization','none','Name','obs')
        fullyConnectedLayer(128,'Name','cfc1')
        reluLayer('Name','crelu1')
        fullyConnectedLayer(128,'Name','cfc2')
        reluLayer('Name','crelu2')
        fullyConnectedLayer(1,'Name','Value') ];
    critic = rlValueFunction(criticNet, obsInfo);
    nets = struct('actor', actor, 'critic', critic);
end

%% 5) Hyperparameter + Agent bauen
switch mode
    case "optimized"
        % Beste Hyperparameter je Agent (Best observed feasible point der Bayes-Opt).
        switch agentType
            case "TRPO"
                hp = struct('criticLR',0.0099218, 'EntropyLossWeight',0.0077964, ...
                            'GAEFactor',0.90407, 'DiscountFactor',0.96301, ...
                            'KLDivergenceLimit',0.027037);
            case "PPO"   % VORLAEUFIG: noch keine BO fuer die neuen HPs -> Platzhalter
                hp = struct('actorLR',0.00046351, 'criticLR',0.0050037, ...
                            'MiniBatchSize',71, 'ExperienceHorizon',464, ...
                            'EntropyLossWeight',0.021276);
            case "PG"
                hp = struct('actorLR',0.0014732, 'criticLR',0.0011844, ...
                            'EntropyLossWeight',0.00028766, 'DiscountFactor',0.97309);
            case "DDPG"
                hp = struct('actorLR',1.8361e-05, 'criticLR',0.00059354, ...
                            'MiniBatchSize',381, 'deviation',0.07003, ...
                            'TargetSmoothFactor',0.046295);
            case "TD3"
                hp = struct('actorLR',0.008932, 'criticLR',0.003068, ...
                            'MiniBatchSize',369, 'deviation',0.45212, ...
                            'TargetSmoothFactor',0.01053);
            case "SAC"
                hp = struct('actorLR',0.0080501, 'criticLR',0.00022378, ...
                            'MiniBatchSize',144, 'TargetSmoothFactor',0.0014283, ...
                            'entropyLR',0.0007604);
            otherwise
                error('Unbekannter agentType "%s".', agentType);
        end
        % On-Policy nutzt die expliziten Netze (nets); Off-Policy die Standardnetze.
        agent = buildAgent(agentType, hp, obsInfo, actInfo, Ts_agent, [], nets);

    case "default"
        % MATLAB-Default-Hyperparameter (kein Override) + Standardnetz (2x numHidden + ReLU).
        agent = buildAgent(agentType, struct(), obsInfo, actInfo, Ts_agent, numHidden);

    otherwise
        error('Unbekannter mode "%s". Erlaubt: "optimized" | "default".', mode);
end

assignin('base',"agent",agent);

%% --- Testlauf mit Reset ---
try   
    ai = getActionInfo(agent);        
    disp(ai)
catch ME
    warning(ME.identifier, '%s', ME.message);
end

% timestamp = datestr(now,'yyyymmdd_HHMMSS');
% saveDir   = fullfile("savedAgents_ppo1", "run_" + timestamp);


%% 7) Trainingsparameter
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', floor(T/Ts_agent), ...
    'StopTrainingCriteria','AverageReward', ...
    'StopTrainingValue', -2.2, ...
    'ScoreAveragingWindowLength', 25, ...
    'Plots','training-progress', ...
    'UseParallel', false ...                 % paralleles Training auf CPU-Workern
    );
%trainOpts.ParallelizationOptions.Mode = "sync";   % synchrones (deterministisches) Update
    % 'SaveAgentCriteria',      "AverageReward", ... % besten Agent zwischenspeichern
    % 'SaveAgentValue',         -inf, ...
    % 'SaveAgentDirectory',     saveDir, ...


%% 8) Training
% Bei parallelem Training brauchen ALLE Worker die Env-Variablen im Base-Workspace
% (u.a. robot_rbt fuer collisionCheckWrapper, das per evalin('base',...) zugreift).
% setupSpaceRobotEnv baut die identische Env pro Worker einmalig auf.
if trainOpts.UseParallel
    pool = gcp('nocreate');
    if isempty(pool), pool = parpool; end
    wait(parfevalOnAll(@() setupSpaceRobotEnv(), 0));
end

trainingStats = train(agent, env, trainOpts);

%% 9) Trainierten (letzten) Agenten speichern
% Ablage nach Modus: SavedAgents/Circular/<Optimized|Default>/<AGENT>.mat
modeFolder = "Optimized"; if mode == "default", modeFolder = "Default"; end
outDir  = fullfile('SavedAgents','Circular', char(modeFolder));
if ~exist(outDir,'dir'), mkdir(outDir); end
outFile = fullfile(outDir, char(agentType + ".mat"));
save(outFile, 'agent');
fprintf('Trainierter Agent gespeichert: %s\n', outFile);

% std(trainingStats.EpisodeReward(700:1000))
% std(diff(trainingStats.AverageReward))
[value, i] = max(trainingStats.AverageReward)