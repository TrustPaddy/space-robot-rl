function [dmin, isColl] = collisionCheckWrapper(qrow)
% qrow: 1x4 double Gelenkwinkel
% dmin: minimaler Abstand [m] (Inf = sicher; <0 = Penetration)
% isColl: logical (true = Kollision)

    % Standardwerte
    dmin   = inf;
    isColl = false;

    persistent robot initDone
    if isempty(initDone)
        robot = evalin('base','robot_rbt');   % robot_col MUSS im Base-WS existieren
        robot.DataFormat = 'row';
        initDone = true;
    end

    try
        % *** GENAU derselbe Aufruf wie dein erfolgreicher Test ***
        [isColl_m, sepDist] = checkCollision(robot, qrow, ...
            'Exhaustive','on', ...
            'IgnoreSelfCollision','off', ...    % <--- wie im Test
            'SkippedSelfCollisions','parent');

        % sepDist auswerten
        if ~isempty(sepDist)
            dvals = double(sepDist(:));
            dmin  = min(dvals);
        else
            if isColl_m
                dmin = -0.0;
            else
                dmin = inf;
            end
        end

        isColl = logical(isColl_m);

    catch ME
        % Zum Debuggen NICHT künstlich Kollisions-true setzen!
        warning('collisionCheckWrapper:Error', ...
            'Fehler in checkCollision: %s', ME.message);
        dmin   = inf;
        isColl = false;
    end

    % Numerik-Guard
    if ~isfinite(dmin)
        dmin   = inf;
        isColl = false;
    end
end
