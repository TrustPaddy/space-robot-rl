function robot = getRobotCol()
% Helper-Funktion für MATLAB-Function-Block
% Holt den RigidBodyTree aus dem Base-Workspace

robot = evalin('base','robot_col');

end
