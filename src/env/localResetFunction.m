function in = localResetFunction(in, init, noise, stream)
% localResetFunction  Setzt den Startzustand einer Episode.
%
%   in = localResetFunction(in)                       % feste Null-Pose (gestreckter Arm)
%   in = localResetFunction(in, init)                 % init = cfg.init aus benchmarkConfig
%   in = localResetFunction(in, init, noise)          % noise = S.noise aus setupSpaceRobotEnv
%   in = localResetFunction(in, init, noise, stream)  % Rauschen aus eigenem RandStream
%
%   Mit init.randomize = true wird jede Gelenkstartposition gleichverteilt in
%   q0 +/- init.range_deg gezogen. Die Zufallszahlen kommen aus dem globalen
%   RNG, der vor dem Training mit rng(seed) gesetzt wird -> reproduzierbar.
%   Die Joint-Bloecke im Modell lesen q0/dq0 als Positions-/Geschwindigkeits-
%   Target. Basis startet immer in Ruhe in der Home-Orientierung.
%
%   Messrauschen (Stresstest): bei noise.obs_std > 0 bekommt jede Episode eine
%   neue Rauschfolge obs_noise (noise.size), aus 'stream' oder, wenn keiner
%   uebergeben wird, aus dem globalen RNG. Bei obs_std = 0 bleibt obs_noise = 0
%   aus dem Base-Workspace, und es werden keine Zufallszahlen verbraucht.
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

    if nargin >= 3 && ~isempty(noise) && noise.obs_std > 0
        if nargin < 4 || isempty(stream)
            stream = RandStream.getGlobalStream;
        end
        in = setVariable(in,'obs_noise', noise.obs_std * randn(stream, noise.size));
    end
end
