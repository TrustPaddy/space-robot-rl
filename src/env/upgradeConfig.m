function cfg = upgradeConfig(cfg)
% upgradeConfig  Rechnet eine gespeicherte Konfiguration auf die aktuelle Bedeutung um.
%
%   cfg = upgradeConfig(L.cfg)    % z. B. aus einer Agentendatei von trainOne
%
%   Version 1 (ohne Feld version, bis 19.09.2026): robot.m_* und robot.I_*
%   galten nur fuer die Inertia-Bloecke. Die Solid-Bloecke 'Visual' trugen
%   zusaetzlich fest 5 kg / [1 1 1] kg*m^2 (Basis) und 1 kg / [0.1 0.1 0.1]
%   kg*m^2 (je Glied), die nicht mit param_scale skaliert wurden.
%   Version 2: robot.m_* und robot.I_* sind Gesamtwerte je Koerper.
%   Die Umrechnung ist exakt (25 + 5 = 30, 0.1 + 0.1 = 0.2), ein Agent aus
%   benchmark_v2 laeuft damit bitgleich wie beim Training.

    if isfield(cfg, 'version')
        v = cfg.version;
    else
        v = 1;
    end

    if v < 2
        if isfield(cfg.robot, 'param_scale') && cfg.robot.param_scale ~= 1
            error('upgradeConfig:scale', ['Version-1-Konfiguration mit param_scale ~= 1 ' ...
                'ist in Version 2 nicht darstellbar (Visual-Massen waren nicht skaliert).']);
        end
        cfg.robot.m_base = cfg.robot.m_base + 5;
        cfg.robot.I_base = cfg.robot.I_base + 1;
        cfg.robot.m_link = cfg.robot.m_link + 1;
        cfg.robot.I_link = cfg.robot.I_link + 0.1;
        cfg.version = 2;
    end
end
