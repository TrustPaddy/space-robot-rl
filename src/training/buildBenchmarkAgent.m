function agent = buildBenchmarkAgent(agentType, mode, obsInfo, actInfo, Ts_agent)
% buildBenchmarkAgent  Agent fuer den Benchmark (Paper, Sec. VI/VII).
%
%   agent = buildBenchmarkAgent(agentType, "default",   obsInfo, actInfo, Ts_agent)
%   agent = buildBenchmarkAgent("PPO",     "optimized", obsInfo, actInfo, Ts_agent)
%
%   "default"   : MATLAB-Standardhyperparameter und Standardnetze mit zwei
%                 Hidden-Layern a 128 Neuronen; nur SampleTime wird gesetzt.
%                 Gilt fuer alle sechs Agenten (PG, PPO, TRPO, DDPG, TD3, SAC).
%   "optimized" : nur PPO. Konfiguration des 2025 trainierten optimierten PPO
%                 (SavedAgents/Circular/Optimized/PPO_1.mat): explizite
%                 Gauss-Policy und Value-Netz mit je 2x128 ReLU, Actor-LR 1e-3,
%                 Critic-LR 5e-4, Gradient-Threshold 1, gamma 0.995,
%                 Horizon 1024, Mini-Batch 256, 10 Epochen, Entropie 1e-3.
%
%   Der globale RNG muss vorher gesetzt sein (rng(seed)): er bestimmt die
%   Initialisierung der Netzgewichte.

    agentType = upper(string(agentType));
    mode      = lower(string(mode));

    switch mode
        case "default"
            agent = buildAgent(agentType, struct(), obsInfo, actInfo, Ts_agent, 128);

        case "optimized"
            if agentType ~= "PPO"
                error('buildBenchmarkAgent:mode', 'Modus "optimized" gibt es nur fuer PPO.');
            end
            agent = optimizedPPO(obsInfo, actInfo, Ts_agent);

        otherwise
            error('buildBenchmarkAgent:mode', 'Unbekannter Modus "%s".', mode);
    end
end

function agent = optimizedPPO(obsInfo, actInfo, Ts_agent)
    nObs = obsInfo.Dimension(1);
    nAct = actInfo.Dimension(1);

    % Gauss-Policy: gemeinsamer Rumpf, Kopf fuer Mittelwert und fuer Streuung
    actorNet = dlnetwork;
    actorNet = addLayers(actorNet, [
        featureInputLayer(nObs, 'Normalization', 'none', 'Name', 'obs')
        fullyConnectedLayer(128, 'Name', 'fc1')
        reluLayer('Name', 'relu1')
        fullyConnectedLayer(128, 'Name', 'fc2')
        reluLayer('Name', 'relu2')]);
    actorNet = addLayers(actorNet, fullyConnectedLayer(nAct, 'Name', 'mean'));
    actorNet = addLayers(actorNet, [
        fullyConnectedLayer(nAct, 'Name', 'stdFc')
        softplusLayer('Name', 'std')]);
    actorNet = connectLayers(actorNet, 'relu2', 'mean');
    actorNet = connectLayers(actorNet, 'relu2', 'stdFc');
    actorNet = initialize(actorNet);
    actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
        ActionMeanOutputNames = "mean", ActionStandardDeviationOutputNames = "std");

    criticNet = dlnetwork([
        featureInputLayer(nObs, 'Normalization', 'none', 'Name', 'obs')
        fullyConnectedLayer(128, 'Name', 'cfc1')
        reluLayer('Name', 'crelu1')
        fullyConnectedLayer(128, 'Name', 'cfc2')
        reluLayer('Name', 'crelu2')
        fullyConnectedLayer(1, 'Name', 'value')]);
    critic = rlValueFunction(criticNet, obsInfo);

    opts = rlPPOAgentOptions( ...
        SampleTime              = Ts_agent, ...
        ExperienceHorizon       = 1024, ...
        MiniBatchSize           = 256, ...
        NumEpoch                = 10, ...
        ClipFactor              = 0.2, ...
        EntropyLossWeight       = 1e-3, ...
        AdvantageEstimateMethod = "gae", ...
        GAEFactor               = 0.95, ...
        DiscountFactor          = 0.995);
    opts.ActorOptimizerOptions  = rlOptimizerOptions(LearnRate = 1e-3, GradientThreshold = 1);
    opts.CriticOptimizerOptions = rlOptimizerOptions(LearnRate = 5e-4, GradientThreshold = 1);

    agent = rlPPOAgent(actor, critic, opts);
end
