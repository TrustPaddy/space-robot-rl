function R = runAllOptimizations(agents, varargin)
% runAllOptimizations  Optimiert nacheinander mehrere Agenttypen.
%
%   R = runAllOptimizations()                      % alle 6 Agenten
%   R = runAllOptimizations(["PPO","TRPO"])        % Teilmenge
%   R = runAllOptimizations([], 'MaxObjectiveEvaluations', 30, ...)
%
%   Jeder Agenttyp bekommt einen eigenen bayesopt-Lauf (der selbst parallel
%   ueber die Trials laeuft). Zusaetzliche Name-Value-Optionen werden 1:1 an
%   optimizeAgent durchgereicht. R ist ein Struct mit einem Feld je Agent.
%
%   Tipp fuer Ueber-Nacht-Laeufe: Pool vorher einmal starten (parpool(N)),
%   dann laufen alle Agenten nacheinander im selben Pool.

    if nargin < 1 || isempty(agents)
        agents = ["PG","PPO","TRPO","DDPG","TD3","SAC"];
    end
    agents = string(agents);

    R = struct();
    for i = 1:numel(agents)
        at = upper(agents(i));
        fprintf('\n===== Optimiere %s (%d/%d) =====\n', at, i, numel(agents));
        try
            res = optimizeAgent(at, varargin{:});
            R.(matlab.lang.makeValidName(char(at))) = res;
        catch ME
            warning('runAllOptimizations:agentFailed', ...
                '%s fehlgeschlagen: %s', at, ME.message);
        end
    end
end
