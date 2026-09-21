function [EE_ref, EE_vref] = referenceTrajectory(cfg, t)
% referenceTrajectory  Soll-Position und -Geschwindigkeit des Endeffektors.
%
%   [EE_ref, EE_vref] = referenceTrajectory(benchmarkConfig(), 0:0.01:8.5)
%
%   Rueckgabe als timeseries (Nx3). Wird von setupSpaceRobotEnv in den
%   Base-Workspace geschrieben (Bloecke 'From Workspace' im Reference-Subsystem)
%   und von makeFigures zum Zeichnen der Sollbahn verwendet.
    r      = cfg.r;
    T      = cfg.T;
    center = [4.5 - r, 0.0, 0.0];   % Start bei x = 4.5 m (gestreckter Arm)

    switch string(cfg.traj)
        case "circle"
            omega = pi/T;
            traj  = [center(1) + r*cos(omega*t); ...
                     center(2) + r*sin(omega*t); ...
                     center(3) + 0*t]';

        case "linear"
            % Zwei Geraden durch drei Kreispunkte (Gl. 5 im Paper)
            P0 = [center(1)+r, center(2),   center(3)];
            P1 = [center(1),   center(2)+r, center(3)];
            P2 = [center(1)-r, center(2),   center(3)];
            t1 = T/2;
            traj = zeros(numel(t), 3);
            idx1 = (t <= t1);
            idx2 = ~idx1;
            traj(idx1,:) = P0 + (t(idx1)'/t1)              .* (P1 - P0);
            traj(idx2,:) = P1 + ((t(idx2)' - t1)/(T - t1)) .* (P2 - P1);

        otherwise
            error('setupSpaceRobotEnv:traj', 'Unbekannte Trajektorie "%s".', cfg.traj);
    end

    dt   = mean(diff(t));
    vref = [zeros(1,3); diff(traj)/dt];   % einfache Ableitung
    EE_ref  = timeseries(traj, t);
    EE_vref = timeseries(vref, t);
end
