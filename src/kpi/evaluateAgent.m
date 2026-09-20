function T = evaluateAgent(agentFile, Q, opt)
% evaluateAgent  Deterministische Auswertung eines mit trainOne trainierten Agenten.
%
%   T = evaluateAgent(agentFile, Q)
%   T = evaluateAgent(agentFile, Q, Condition="reduced_sat", ...
%                     Config=struct('tau_sat_scale', 0.75))
%
%   Q : 4 x N Startwinkel [rad] (z. B. evalInitStates); Spalte k = Episode k.
%       Alle Agenten mit denselben Spalten auswerten -> gepaarte Tests.
%   Die Policy laeuft deterministisch (UseExplorationPolicy = false), auch bei
%   den stochastischen Agenten PPO, TRPO, PG und SAC. Grundlage ist die
%   Trainingskonfiguration aus der Agentendatei; Config ueberschreibt
%   einzelne Werte (Stresstests, andere Trajektorie).
%   Messrauschen (Config.noise.obs_std > 0): Episode k bekommt ihre Folge aus
%   RandStream('mrg32k3a', Seed = cfg.noise.seed) mit Substream k, also
%   dieselbe Folge fuer alle Agenten und unabhaengig vom globalen RNG.
%
%   T : eine Zeile je Episode mit agent, mode, seed, condition, episode,
%       q0 [deg], steps, terminated und K1..K9 (computeKPIsFromLogs; bei
%       vorzeitigem Abbruch ueber den ausgefuehrten Teil der Episode).
%   Prueft je Episode, dass die Simulation wirklich im vorgegebenen
%   Startzustand begonnen hat.
%
%   Simuliert wird ueber die RL-Umgebung, nicht mit sim('SpaceRobot'): nur die
%   Umgebung beendet eine Episode beim isDone-Signal (Kollision, Gelenkgrenze).

    arguments
        agentFile (1,1) string
        Q (4,:) double
        opt.Config struct = struct()
        opt.Condition (1,1) string = "nominal"
        opt.LogFile (1,1) string = ""      % optional: logsouts speichern (gross)
    end

    L = load(agentFile, 'agent', 'cfg', 'agentType', 'mode', 'seed');
    cfg = benchmarkConfig(upgradeConfig(L.cfg), opt.Config);
    S = setupSpaceRobotEnv(cfg);

    agent = L.agent;
    agent.UseExplorationPolicy = false;

    N = size(Q, 2);
    counter = containers.Map({'k'}, {0});          % Handle-Objekt: zaehlt Episoden
    S.env.ResetFcn = @(in) evalReset(in, Q, counter, S.noise);
    xp = sim(S.env, agent, rlSimulationOptions( ...
        MaxSteps = floor(cfg.T / cfg.Ts_agent), NumSimulations = N));

    params.tau_max = cfg.tau_max;
    qlim = deg2rad([cfg.q1_lim_deg; repmat(cfg.qi_lim_deg, 3, 1)]);
    params.q_min = qlim(:,1)';
    params.q_max = qlim(:,2)';

    rows = cell(N, 1);
    logs = cell(N, 1);
    for k = 1:N
        lo = xp(k).SimulationInfo(1).logsout;   % SimulationStorage -> SimulationOutput
        logs{k} = lo;
        q  = lo.getElement('q').Values;
        q0 = squeeze(q.Data); if size(q0,1) ~= 4, q0 = q0'; end
        if max(abs(q0(:,1) - Q(:,k))) > 1e-9
            error('evaluateAgent:init', 'Episode %d startete nicht im vorgegebenen Zustand.', k);
        end
        kpi = computeKPIsFromLogs({lo}, params);
        done = squeeze(xp(k).IsDone.Data);
        rows{k} = table(string(L.agentType), string(L.mode), L.seed, opt.Condition, k, ...
            rad2deg(Q(:,k)'), numel(done), logical(done(end)), ...
            kpi.K1, kpi.K2, kpi.K3, kpi.K4, kpi.K5, kpi.K6, kpi.K7, kpi.K8, kpi.K9, ...
            'VariableNames', {'agent','mode','seed','condition','episode','q0_deg','steps','terminated', ...
                              'K1','K2','K3','K4','K5','K6','K7','K8','K9'});
    end
    T = vertcat(rows{:});

    if strlength(opt.LogFile) > 0
        d = fileparts(opt.LogFile);
        if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
        save(opt.LogFile, 'logs', 'Q', 'cfg', '-v7.3');
    end
end

function in = evalReset(in, Q, counter, noise)
    k = counter('k') + 1;
    counter('k') = k;
    stream = [];
    if noise.obs_std > 0
        stream = RandStream('mrg32k3a', 'Seed', noise.seed);
        stream.Substream = k;
    end
    in = localResetFunction(in, struct('q0', Q(:, k), 'randomize', false, 'range_deg', 0), ...
                            noise, stream);
end
