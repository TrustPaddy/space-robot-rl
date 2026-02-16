%%  1) Trajektorie/IKrobuster machen
clc; clear; close all;
rng(0,'twister')

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
r = 0.4;               
center = [4.5-r, 0.0, 0.0];  

% % Punkt anstelle Kreisbahn  
% r = 0;               
% center = [3.5, 0.0, 0.0];  

Ts  = 0.01;        % Diskrete Schrittzeit der RL-Schleife
Ts_agent = 0.1;
t   = 0:Ts:T;      % Zeitvektor (0…10 s, inkl. Ende ok für From Workspace)

x = center(1) + r*cos(omega*t);
y = center(2) + r*sin(omega*t);
z = center(3) + 0*t;          

traj = [x' y' z'];   % gewünschte EE-Punkte

% Roboter laden
robot_rbt = importrobot('SpaceRobot.urdf');
robot_rbt.DataFormat = 'row';
eeBodyName = robot_rbt.BodyNames{end};


% Inverse Kinematik vorbereiten
ik = inverseKinematics('RigidBodyTree', robot_rbt); 

qseed = randomConfiguration(robot_rbt);
q_des = zeros(numel(t), numel(qseed));

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
obsDim = 3 + 3 + 2*nJ + 6 + 3;           % e_p,e_v,q,dq,v_base,w_base -> = 20
% 
% obsInfo = rlNumericSpec([obsDim 1], ...
%     'Name','obs', ...
%     'Description','Robot observations');

ePLim=0.5; eVLim=1.0; qLim=pi; dqLim=3; vBLim=0.5; wBLim=1.0; eOriLim=pi; % rad
obsLow  = [-ePLim*ones(3,1); -eVLim*ones(3,1); -qLim*ones(nJ,1); -dqLim*ones(nJ,1); -vBLim*ones(3,1); -wBLim*ones(3,1); -eOriLim*ones(3,1)];
obsHigh = -obsLow;
obsInfo = rlNumericSpec([numel(obsLow) 1], LowerLimit=obsLow, UpperLimit=obsHigh, Name="obs");

actInfo = rlNumericSpec([nJ 1], ...
    'Name','tau', ...
    'LowerLimit', -2*ones(nJ,1), ...
    'UpperLimit', 2*ones(nJ,1));

%% 3) RL-Umgebung mit Simulink verknüpfen
env = rlSimulinkEnv(mdl, agentBlk, obsInfo, actInfo);
env.ResetFcn = @localResetFunction;   % Reset-Funktion (kommt unten)

%% 4) Actor (Policy) und Critic (Value) Netzwerke) ----
obsPath = featureInputLayer(obsDim,'Normalization','none','Name','obs');

commonPath = [
    fullyConnectedLayer(128,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(128,'Name','fc2')
    reluLayer('Name','relu2')
];

meanPath = [
    fullyConnectedLayer(nJ,'Name','mean')
];  

stdPath = [
    fullyConnectedLayer(nJ,'Name','std')
    softplusLayer('Name','softplus')
];

actorLG = layerGraph(obsPath);
actorLG = addLayers(actorLG, commonPath);
actorLG = addLayers(actorLG, meanPath);
actorLG = addLayers(actorLG, stdPath);

actorLG = connectLayers(actorLG,'obs','fc1');
actorLG = connectLayers(actorLG,'relu2','mean');
actorLG = connectLayers(actorLG,'relu2','std');

actor = rlContinuousGaussianActor(actorLG, obsInfo, actInfo, ...
    ActionMeanOutputNames="mean", ...
    ActionStandardDeviationOutputNames="softplus" ...
);

criticLG = layerGraph([
    featureInputLayer(obsDim,'Normalization','none','Name','obs')
    fullyConnectedLayer(128,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(128,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(1,'Name','Value') ...
]);

critic = rlValueFunction(criticLG, obsInfo, 'Observation','obs');

%% 4+5) TRPO-Agent (on-policy, Policy-Gradient)

initOpts = rlAgentInitializationOptions(NumHiddenUnit=128);

agent = rlTRPOAgent(obsInfo, actInfo, initOpts);

% --- TRPO-Optionen ---
agent.AgentOptions.SampleTime = Ts_agent;

assignin('base','agent',agent);

%% --- Testlauf mit Reset ---
try   
    ai = getActionInfo(agent);        
    disp(ai)
catch ME
    warning(ME.identifier, '%s', ME.message);
end

%% 7) Trainingsparameter
trainOpts = rlTrainingOptions(...
    'MaxEpisodes', 1000, ...
    'MaxStepsPerEpisode', floor(T/Ts_agent), ... 
    'StopTrainingCriteria','AverageReward', ...
    'StopTrainingValue', -2, ...
    'ScoreAveragingWindowLength', 25, ...
    'Plots','training-progress' ...
);

%% 8) Training
trainingStats = train(agent, env, trainOpts);

save('SpaceRobot_TRPO_agent.mat','agent');

% std(trainingStats.EpisodeReward(700:1000))
% 
% std(diff(trainingStats.AverageReward))
% 
% [value, i] = max(trainingStats.AverageReward)




