function agent = buildAgent(agentType, params, obsInfo, actInfo, Ts_agent, numHidden, nets)
% buildAgent  Baut einen RL-Agenten mit den uebergebenen Hyperparametern.
%
%   agent = buildAgent(agentType, params, obsInfo, actInfo, Ts_agent)
%   agent = buildAgent(..., Ts_agent, numHidden)        % Netzbreite (Default 128)
%   agent = buildAgent(..., numHidden, nets)            % explizite On-Policy-Netze
%
%   agentType : "PG" | "PPO" | "TRPO" | "DDPG" | "TD3" | "SAC"
%   params    : Struct mit Hyperparameter-Feldern (nur vorhandene werden gesetzt)
%   numHidden : (optional) Neuronen je Hidden-Layer des Standardnetzes (Default 128)
%   nets      : (optional) struct('actor',..,'critic',..) fuer On-Policy-Agenten
%               (PG/PPO/TRPO). Wenn gesetzt, werden diese Netze statt der
%               InitOptions-Standardnetze verwendet. Fuer Off-Policy ignoriert.
%
%   Zentrale Stelle, an der die Hyperparameter auf die AGENT-SPEZIFISCH
%   KORREKTEN Options-Properties gemappt werden. Das behebt den Kernbug der
%   alten trainAndEvaluate.m, wo bei TRPO NoiseOptions/ActorLR gesetzt wurden,
%   die es dort gar nicht gibt (-> jeder Trial crashte).
%
%   Moegliche params-Felder (je nach Agent, max. 5 pro Agent):
%     actorLR, criticLR, MiniBatchSize, ClipFactor, EntropyLossWeight,
%     GAEFactor, DiscountFactor, KLDivergenceLimit, deviation,
%     TargetSmoothFactor, entropyLR
%   (Feld fehlt -> Property bleibt auf MATLAB-Default; setIf/isfield-Guards.)
%
%   HINWEIS: Die Property-Namen/-Strukturen sind gegen die offizielle R2025b-
%   Doku verifiziert. Wichtig: bei TD3/SAC ist CriticOptimizerOptions ein
%   VEKTOR aus 2 Optimierern (Twin-Critics) -> setCriticLR iteriert darueber.
%   Der Build-Check (checkBuildAgents.m) laesst jeden Agenten einmal bauen und
%   deckt etwaige Versions-Abweichungen sofort mit klarer Fehlermeldung auf.

    if nargin < 5 || isempty(Ts_agent), Ts_agent = 0.1; end
    if nargin < 6 || isempty(numHidden), numHidden = 128; end
    if nargin < 7, nets = []; end
    agentType = upper(string(agentType));

    initOpts = rlAgentInitializationOptions(NumHiddenUnit=numHidden);

    % Explizite Netze sind nur fuer On-Policy-Agenten sinnvoll (Gaussian-Actor +
    % State-Value-Critic). Fuer Off-Policy (DDPG/TD3/SAC) verworfen.
    if ~isempty(nets) && ~any(agentType == ["PG","PPO","TRPO"])
        warning('buildAgent:netsIgnored', ...
            'Explizite Netze werden nur fuer On-Policy (PG/PPO/TRPO) genutzt; fuer %s ignoriert.', agentType);
        nets = [];
    end

    switch agentType
        case "PG"
            agent = mkOnPolicy(@rlPGAgent, nets, obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            o = setIf(o, params, 'EntropyLossWeight', 'EntropyLossWeight');
            o = setIf(o, params, 'DiscountFactor',    'DiscountFactor');
            o = setActorLR(o, params);
            o = setCriticLR(o, params);   % UseBaseline ist per Default true -> Critic vorhanden
            agent.AgentOptions = o;

        case "PPO"
            agent = mkOnPolicy(@rlPPOAgent, nets, obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            o = setIf(o, params, 'MiniBatchSize',     'MiniBatchSize',     true);
            o = setIf(o, params, 'ExperienceHorizon', 'ExperienceHorizon', true);
            o = setIf(o, params, 'ClipFactor',        'ClipFactor');
            o = setIf(o, params, 'EntropyLossWeight', 'EntropyLossWeight');
            o = setIf(o, params, 'NumEpoch',          'NumEpoch',          true);
            o = setIf(o, params, 'GAEFactor',         'GAEFactor');
            o = setIf(o, params, 'DiscountFactor',    'DiscountFactor');
            o = setActorLR(o, params);
            o = setCriticLR(o, params);
            o.MiniBatchSize = min(o.MiniBatchSize, o.ExperienceHorizon);  % dok. Bedingung
            agent.AgentOptions = o;

        case "TRPO"
            agent = mkOnPolicy(@rlTRPOAgent, nets, obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            % TRPO hat KEINEN actorLR und KEIN NoiseOptions (Trust-Region-Update).
            o = setIf(o, params, 'MiniBatchSize',     'MiniBatchSize',     true);
            o = setIf(o, params, 'ExperienceHorizon', 'ExperienceHorizon', true);
            o = setIf(o, params, 'EntropyLossWeight', 'EntropyLossWeight');
            o = setIf(o, params, 'GAEFactor',         'GAEFactor');
            o = setIf(o, params, 'DiscountFactor',    'DiscountFactor');
            o = setIf(o, params, 'KLDivergenceLimit', 'KLDivergenceLimit');
            o = setCriticLR(o, params);
            o.MiniBatchSize = min(o.MiniBatchSize, o.ExperienceHorizon);  % dok. Bedingung
            agent.AgentOptions = o;

        case "DDPG"
            agent = rlDDPGAgent(obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            o = setIf(o, params, 'MiniBatchSize',      'MiniBatchSize', true);
            o = setIf(o, params, 'TargetSmoothFactor', 'TargetSmoothFactor');
            % DDPG: Ornstein-Uhlenbeck-Rauschen -> NoiseOptions
            if isfield(params,'deviation')
                o.NoiseOptions.StandardDeviation = params.deviation * ones(numel(actInfo.LowerLimit),1);
            end
            o = setActorLR(o, params);
            o = setCriticLR(o, params);
            agent.AgentOptions = o;

        case "TD3"
            agent = rlTD3Agent(obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            o = setIf(o, params, 'MiniBatchSize',      'MiniBatchSize', true);
            o = setIf(o, params, 'TargetSmoothFactor', 'TargetSmoothFactor');
            % TD3: Gauss'sches Explorationsrauschen -> ExplorationModel
            % (TargetPolicySmoothModel + PolicyUpdateFrequency bleiben auf Default)
            if isfield(params,'deviation')
                o.ExplorationModel.StandardDeviation = params.deviation * ones(numel(actInfo.LowerLimit),1);
            end
            o = setActorLR(o, params);
            o = setCriticLR(o, params);
            agent.AgentOptions = o;

        case "SAC"
            agent = rlSACAgent(obsInfo, actInfo, initOpts);
            o = agent.AgentOptions;
            o.SampleTime = Ts_agent;
            o = setIf(o, params, 'MiniBatchSize',      'MiniBatchSize', true);
            o = setIf(o, params, 'TargetSmoothFactor', 'TargetSmoothFactor');
            % SAC reguliert Exploration ueber die Entropie (kein manuelles deviation)
            if isfield(params,'entropyLR')
                o.EntropyWeightOptions.LearnRate = params.entropyLR;
            end
            o = setActorLR(o, params);
            o = setCriticLR(o, params);
            agent.AgentOptions = o;

        otherwise
            error('buildAgent:unknownAgent', ...
                'Unbekannter agentType "%s". Erlaubt: PG, PPO, TRPO, DDPG, TD3, SAC.', agentType);
    end
end

% ===== Hilfsfunktionen =====================================================

function agent = mkOnPolicy(ctor, nets, obsInfo, actInfo, initOpts)
% On-Policy-Agent bauen: mit expliziten Netzen (falls uebergeben), sonst per
% InitOptions-Standardnetzen.
    if ~isempty(nets)
        agent = ctor(nets.actor, nets.critic);
    else
        agent = ctor(obsInfo, actInfo, initOpts);
    end
end

function o = setIf(o, params, field, prop, isInt)
% Setzt o.(prop) = params.(field), falls das Feld existiert.
    if nargin < 5, isInt = false; end
    if isfield(params, field) && ~isempty(params.(field))
        v = params.(field);
        if isInt, v = round(double(v)); end
        o.(prop) = v;
    end
end

function o = setActorLR(o, params)
    if isfield(params,'actorLR') && ~isempty(params.actorLR)
        o.ActorOptimizerOptions.LearnRate         = params.actorLR;
        o.ActorOptimizerOptions.GradientThreshold = 1;   % Stabilitaet (wie MathWorks-Leitfaden)
    end
end

function o = setCriticLR(o, params)
    if isfield(params,'criticLR') && ~isempty(params.criticLR)
        % TD3/SAC haben ZWEI Critics -> CriticOptimizerOptions ist ein Vektor.
        % Deshalb ueber alle Elemente iterieren (bei DDPG/PPO/TRPO/PG numel==1).
        for j = 1:numel(o.CriticOptimizerOptions)
            o.CriticOptimizerOptions(j).LearnRate         = params.criticLR;
            o.CriticOptimizerOptions(j).GradientThreshold = 1;
        end
    end
end
