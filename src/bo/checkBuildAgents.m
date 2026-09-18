function ok = checkBuildAgents()
% checkBuildAgents  Schneller Build-Check fuer alle 6 Agenttypen.
%
%   ok = checkBuildAgents()
%
%   Baut jeden Agenten mit den Mittelwerten seines Suchraums (getSearchSpace)
%   und den korrekten obs/act-Specs. Deckt den urspruenglichen Kernbug sofort
%   auf: falsche Option-Properties (z.B. NoiseOptions/ActorLR bei TRPO) fuehren
%   hier zu einem klaren FAIL, statt im bayesopt-Lauf still als 1e6 zu enden.
%
%   Laeuft OHNE Simulink/URDF (nur die Spezifikationen), daher sekundenschnell.

    % --- obs/act-Specs (gleiche Dimensionen wie in setupSpaceRobotEnv.m) ---
    nJ = 4;
    ePLim=0.5; eVLim=1.0; qLim=pi; dqLim=3; vBLim=0.5; wBLim=1.0; eOriLim=pi;
    obsLow  = [-ePLim*ones(3,1); -eVLim*ones(3,1); -qLim*ones(nJ,1); ...
               -dqLim*ones(nJ,1); -vBLim*ones(3,1); -wBLim*ones(3,1); -eOriLim*ones(3,1)];
    obsInfo = rlNumericSpec([numel(obsLow) 1], LowerLimit=obsLow, UpperLimit=-obsLow, Name="obs");
    actInfo = rlNumericSpec([nJ 1], 'Name','tau', ...
        'LowerLimit', -2*ones(nJ,1), 'UpperLimit', 2*ones(nJ,1));
    Ts_agent = 0.1;

    agents = ["PG","PPO","TRPO","DDPG","TD3","SAC"];
    ok = true;
    fprintf('--- checkBuildAgents ---\n');
    for i = 1:numel(agents)
        at = agents(i);
        p  = midParams(getSearchSpace(at));
        try
            agent = buildAgent(at, p, obsInfo, actInfo, Ts_agent);
            fprintf('OK    %-5s : %s gebaut\n', at, class(agent));
        catch ME
            ok = false;
            fprintf(2, 'FAIL  %-5s : %s\n', at, ME.message);
        end
    end
    if ok
        fprintf('--- Alle Agenten erfolgreich gebaut. ---\n');
    else
        fprintf(2, '--- Mindestens ein Agent fehlgeschlagen (siehe oben). ---\n');
    end
end

function p = midParams(vars)
% Mittelwert (bzw. geometrisches Mittel bei log) jedes Suchraum-Parameters.
    p = struct();
    for k = 1:numel(vars)
        v = vars(k);
        r = v.Range;
        if strcmpi(v.Type, 'integer')
            val = round(mean(r));
        elseif strcmpi(v.Transform, 'log')
            val = exp(mean(log(r)));
        else
            val = mean(r);
        end
        p.(v.Name) = val;
    end
end
