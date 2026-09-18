function in = localResetFunction(in, init)
% localResetFunction  Setzt den Startzustand einer Episode.
%
%   in = localResetFunction(in)         % feste Null-Pose (gestreckter Arm)
%   in = localResetFunction(in, init)   % init = cfg.init aus benchmarkConfig
%
%   Mit init.randomize = true wird jede Gelenkstartposition gleichverteilt in
%   q0 +/- init.range_deg gezogen. Die Zufallszahlen kommen aus dem globalen
%   RNG, der vor dem Training mit rng(seed) gesetzt wird -> reproduzierbar.
%   Die Joint-Bloecke im Modell lesen q0/dq0 als Positions-/Geschwindigkeits-
%   Target. Basis startet immer in Ruhe in der Home-Orientierung.
%
%   Bewusst KEIN evalin('base',...): parallel-sicher (kein Zugriff auf den
%   Worker-Base-Workspace).

    nJ = 4;
    if nargin < 2 || isempty(init)
        init = struct('q0', zeros(nJ,1), 'randomize', false, 'range_deg', 0);
    end

    q0 = init.q0(:);
    if init.randomize
        q0 = q0 + deg2rad(init.range_deg(:)) .* (2*rand(nJ,1) - 1);
    end

    in = setVariable(in,'q0',q0);
    in = setVariable(in,'dq0',zeros(nJ,1));
    in = setVariable(in,'base_v0',zeros(3,1));
    in = setVariable(in,'base_w0',zeros(3,1));
    in = setVariable(in,'phi0',0);
    in = setVariable(in,'reward_init',0);
    in = setVariable(in,'isdone_init',0);
end
