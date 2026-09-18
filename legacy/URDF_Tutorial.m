%SPART URDF Tutorial

%--- Clean and clear ---%
clc
close all
clear

%--- URDF filename ---%
filename='kuka_lwr.urdf';
%filename='kuka_iiwa.urdf';

%--- Create robot model ---%
[robot,robot_keys] = urdf2robot(filename);


%--- Parameters ---%
R0=eye(3);
r0=zeros(3,1);
qm=zeros(robot.n_q,1);
u0=zeros(6,1);
um=zeros(robot.n_q,1);

%--- Kinematics ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Diferential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);
%Jacobian of the last link
[J0n, Jmn]=Jacob(rL(1:3,end),r0,rL,P0,pm,robot.n_links_joints,robot);

%--- Dynamics ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Forward Dynamics ---%

%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=zeros(6,1);
wFm=zeros(6,robot.n_links_joints);
for i=1:robot.n_links_joints
    wFm(6,i)=-robot.links(i).mass*g;
end

%Joint torques
tauq0=zeros(6,1);
tauqm=zeros(robot.n_q,1);

%Forward Dynamics
[u0dot_FD,umdot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,robot);

%--- Inverse Dynamics - Flying ---%

%Accelerations
u0dot=zeros(6,1);
umdot=zeros(robot.n_q,1);

%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robot);

%Inverse Dynamics - Flying base
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%--- Forward Dynamics - Floating ---%

%Accelerations
umdot=zeros(robot.n_q,1);

%Inverse Dynamics - Floating Base
[taum_floating,u0dot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);