clc; clear; close all;

% Dein URDF
filename = 'kuka_iiwa.urdf';

% SPART-Import
[robot, robot_keys] = urdf2robot(filename);

% Startkonfiguration
qm = zeros(robot.n_q,1);   % Gelenkwinkel
u0 = zeros(6,1);           % Basisgeschwindigkeit (floating base)
um = zeros(robot.n_q,1);   % Gelenkgeschwindigkeit

T = 10; omega = 2*pi/T; r = 0.5;
center = [4.0, 0.0, 0.0];
N = 200; t = linspace(0,T,N);

x = center(1) + r*cos(omega*t);
y = center(2) + r*sin(omega*t);
z = center(3) + 0*t;
traj = [x' y' z'];

% Kinematik für Startpose
R0 = eye(3);   % Basis-Rotation
r0 = zeros(3,1); % Basis-Position

[RJ,RL,rJ,rL,e,g] = Kinematics(R0,r0,qm,robot);
[Bij,Bi0,P0,pm]   = DiffKinematics(R0,r0,rL,e,g,robot);
[t0,tm]           = Velocities(Bij,Bi0,P0,pm,u0,um,robot);

% Inertias
[I0,Im] = I_I(R0,RL,robot);
[M0_tilde,Mm_tilde] = MCB(I0,Im,Bij,Bi0,robot);
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

dt = t(2)-t(1);
q_log = zeros(N, robot.n_q);
rEE_log = zeros(N,3);

for k = 1:N
    % Endeffektor-Istposition
    [RJ,RL,rJ,rL,e,g] = Kinematics(R0,r0,qm,robot);
    pEE = rL(1:3,end);
    rEE_log(k,:) = pEE';
    
    % Sollposition
    pDes = traj(k,:)';
    
    % Fehler
    ePos = pDes - pEE;
    
    % EE-Jacobi
    [J0n, Jmn] = Jacob(rL(1:3,end),r0,rL,P0,pm,robot.n_links_joints,robot);
    
    % Gelenkraum-Kontrolle (simpler PD)
    Kp = 50; Kd = 10;
    Ftask = Kp*ePos;   % Task-Kraft (nur Position)
    tauqm = Jmn'*Ftask;
    
    % Externe Kräfte (im All meist 0)
    wF0 = zeros(6,1);
    wFm = zeros(6,robot.n_links_joints);
    
    % Dynamik mit floating base
    umdot = zeros(robot.n_q,1);  % Sollbeschl. Gelenke
    [taum_floating, u0dot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);
    
    % Basisbeschl. aus Floating-ID zurück
    u0 = u0 + u0dot_floating*dt;
    um = um + umdot*dt;
    
    % Gelenkwinkel updaten (Euler vorwärts)
    qm = qm + um*dt;
    
    q_log(k,:) = qm';
end

figure;
plot3(traj(:,1), traj(:,2), traj(:,3),'r--'); hold on;
plot3(rEE_log(:,1), rEE_log(:,2), rEE_log(:,3),'b-');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal;
legend('Soll-Kreis','Ist-Bahn');
title('SPART Floating-Base Kreisbahn');
