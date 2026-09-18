function vars = getSearchSpace(agentType)
% getSearchSpace  Liefert den optimizableVariable-Suchraum je Agenttyp.
%
%   vars = getSearchSpace(agentType)
%
%   agentType : "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"
%
%   MAX. 5 Hyperparameter pro Agent (die wirkungsvollsten). Jeder Agent bekommt
%   NUR Hyperparameter, die er wirklich besitzt; die Property-Namen/-Strukturen
%   sind gegen die offizielle R2025b-Doku verifiziert. Die Variablennamen
%   entsprechen exakt den params-Feldern, die buildAgent.m auswertet. Bereiche
%   decken jeweils den MATLAB-Default und die bekannten guten Werte ab.

    agentType = upper(string(agentType));

    % Wiederverwendbare Bausteine
    actorLR = optimizableVariable('actorLR', [1e-5, 1e-2], 'Transform','log');
    criticLR= optimizableVariable('criticLR',[1e-5, 1e-2], 'Transform','log');
    miniB   = optimizableVariable('MiniBatchSize', [64, 512], 'Type','integer');
    expH    = optimizableVariable('ExperienceHorizon', [256, 2048], 'Type','integer', 'Transform','log');
    entW    = optimizableVariable('EntropyLossWeight', [1e-4, 5e-2], 'Transform','log');
    gae     = optimizableVariable('GAEFactor', [0.90, 0.99]);
    disc    = optimizableVariable('DiscountFactor', [0.95, 0.999]);
    kl      = optimizableVariable('KLDivergenceLimit', [5e-3, 5e-2]);
    dev     = optimizableVariable('deviation', [0.05, 0.5]);
    tsf     = optimizableVariable('TargetSmoothFactor', [1e-3, 1e-1], 'Transform','log');
    entLR   = optimizableVariable('entropyLR', [1e-4, 1e-2], 'Transform','log');

    switch agentType
        case "PG"    % 4 HP (PG hat kaum mehr sinnvolle Knoepfe)
            vars = [ actorLR; criticLR; entW; disc ];

        case "PPO"   % 5 HP: Actor/Critic-LR + Batch/Horizon + Entropie
            vars = [ actorLR; criticLR; miniB; expH; entW ];

        case "TRPO"  % 5 HP: kein actorLR (Trust-Region), dafuer KL-Limit
            vars = [ criticLR; entW; gae; disc; kl ];

        case "DDPG"  % 5 HP: LRs, Batch, Explorationsrauschen, Target-Smoothing
            vars = [ actorLR; criticLR; miniB; dev; tsf ];

        case "TD3"   % 5 HP: wie DDPG (deviation -> ExplorationModel in buildAgent)
            vars = [ actorLR; criticLR; miniB; dev; tsf ];

        case "SAC"   % 5 HP: LRs, Batch, Target-Smoothing, Entropie-LearnRate
            vars = [ actorLR; criticLR; miniB; tsf; entLR ];

        otherwise
            error('getSearchSpace:unknownAgent', ...
                'Unbekannter agentType "%s". Erlaubt: PG, PPO, TRPO, DDPG, TD3, SAC.', agentType);
    end
end
