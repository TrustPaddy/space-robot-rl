
clc; clear; close all;

% ------------------------------------------------------------
% Soll-Kreisbahn definieren
% ------------------------------------------------------------
T = 8.5;                 % Umlaufzeit [s]
omega = pi/T;
r = 0.4;                % Radius [m]
center = [4.5-r, 0.0, 0.0];   % Kreiszentrum

Ts = 0.01;             % Diskrete Schrittzeit der RL-Schleife
t  = 0:Ts:T;            % Zeitvektor (0…10 s, inkl. Ende ok für From Workspace)

x = center(1) + r*cos(omega*t);
y = center(2) + r*sin(omega*t);
z = center(3) + 0*t;    % konstant in Z-Richtung

traj = [x' y' z'];      % gewünschte EE-Punkte

% ------------------------------------------------------------
% Roboter laden
% ------------------------------------------------------------
robot_rbt = importrobot('SpaceRobot.urdf');
robot_rbt.DataFormat = 'row';
eeBodyName = robot_rbt.BodyNames{end};

% ------------------------------------------------------------
% Inverse Kinematik vorbereiten
% ------------------------------------------------------------
ik = inverseKinematics('RigidBodyTree', robot_rbt);
weights = [1 1 1 0.1 0.1 0.1];   % Position stark, Orientierung schwach
% qseed = homeConfiguration(robot_rbt);
qseed = randomConfiguration(robot_rbt);

q_des = zeros(numel(t), numel(qseed));

for k = 1:numel(t)
    % gewünschte EE-Pose (Position + Orientierung = Identität)
    Tdes = trvec2tform(traj(k,:));  % homogene Transformationsmatrix
    [qsol, ~] = ik(eeBodyName, Tdes, weights, qseed);
    q_des(k,:) = qsol;
    qseed = qsol;  % als Startwert für nächsten Schritt
end

% Optional: Glättung der Gelenkwinkel
% q_des = smoothdata(q_des, 1, 'movmean', 5);

% ------------------------------------------------------------
% Animation (optional)
% ------------------------------------------------------------
figure; ax = axes;
plot3(ax, traj(:,1), traj(:,2), traj(:,3), 'r--','LineWidth',1.5); hold on;
grid(ax,'on'); axis(ax,'equal');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Endeffektor-Kreisbahn mit Inverser Kinematik');

for k = 1:(0.1/Ts):numel(t)
    show(robot_rbt, q_des(k,:), 'Parent', ax, 'PreservePlot', false);
    view(2);
    xlim([-1, 5]);
    ylim([-2, 2]);
    zlim([-2, 2]);
    drawnow;
end
