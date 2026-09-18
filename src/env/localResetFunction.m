function in = localResetFunction(in)
    % Reset-Funktion für RL-Training mit fester Startpose + fester Basis-Orientierung

    nJ = 4;

    % --- Startpose: feste Null-Pose ---
    % Bewusst KEIN evalin('base',...): das ist parallel-sicher (kein Zugriff auf
    % den Worker-Base-Workspace) und verhaltensgleich zum Original, da q_des in
    % SpaceRobotDynamic.m ohnehin nur Nullen enthaelt (IK-Schleife auskommentiert).
    q_start = zeros(nJ,1);

    % --- Anfangswerte ---
    q0        = q_start;
    dq0       = zeros(nJ,1);
    base_v0   = zeros(3,1);
    base_w0   = zeros(3,1);
    phi0      = 0;

    % --- In den Workspace/Model schreiben ---
    in = setVariable(in,'q0',q0);
    in = setVariable(in,'dq0',dq0);
    in = setVariable(in,'base_v0',base_v0);
    in = setVariable(in,'base_w0',base_w0);
    in = setVariable(in,'phi0',phi0);

    % in = setVariable(in,'resetFlag_init',true);   % nur zum Start 1

    % (falls du Memory-Blöcke nutzt)
    in = setVariable(in,'reward_init',0);
    in = setVariable(in,'isdone_init',0);
end
