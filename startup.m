% startup.m  -  Projektpfade fuer das Space-Robot-RL-Projekt setzen.
%
% Einmal pro MATLAB-Session aus dem Projekt-Root ausfuehren:
%     >> startup
%
% Fuegt den Projekt-Root (damit SpaceRobot.slx / SpaceRobot.urdf per Namen
% gefunden werden) und alle Code-Ordner unter src/ zum MATLAB-Pfad hinzu.
% 'models/', 'legacy/' und 'backup/' bleiben BEWUSST draussen, damit z. B. das
% ungenutzte legacy/getRobotCol.m nichts ueberdeckt.
%
% bo.m und SpaceRobotDynamic.m rufen dieses Skript bei Bedarf automatisch auf.

proot = fileparts(mfilename('fullpath'));
addpath(proot);
addpath(genpath(fullfile(proot, 'src')));
fprintf('[startup] Projektpfade gesetzt (Root + src/).\n');
clear proot;
