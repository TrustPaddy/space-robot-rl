function [jobs, evals] = partCJobs(studies)
% partCJobs  Trainingslaeufe und Auswertungen fuer Teil C der Revision.
%
%   [jobs, evals] = partCJobs(["stress","twoseg","ablation","sensitivity","r24b"])
%
%   jobs  : table(agent, mode, seed, campaign, config) fuer runCampaign
%   evals : table(campaign, condition, config) fuer evaluateCampaign
%
%   Studien (Kampagnenordner unter results/):
%     r24b         R2.4b: PPO, TRPO, SAC default, W_ori und W_p je x0.5 und x2,
%                  Seeds 0-4 -> r24b_wori_x05, r24b_wori_x2, r24b_wp_x05, r24b_wp_x2.
%                  Referenz x1: benchmark_v2, Seeds 0-4.
%     ablation     PPO optimiert, je ein Term aus: W_p, W_v, W_ori, W_wb, W_vb,
%                  W_u, W_d = 0 sowie k_p = 0 und k_b = 0, Seeds 0-4 -> abl_<term>0.
%                  Referenz: benchmark_v2 PPO optimiert, Seeds 0-4.
%     sensitivity  PPO optimiert, W_p, W_v, W_ori, W_wb je x0.5 und x2, Seeds 0-4
%                  -> sens_<term>_x05, sens_<term>_x2. Referenz wie ablation.
%     twoseg       Zwei-Segment-Bahn (Gl. 5): PPO default, PPO optimiert, TRPO
%                  default, Seeds 0-9 -> twoseg.
%     stress       nur Auswertung der benchmark_v2-Agenten (70) unter: Messrauschen
%                  0.005, tau_sat x0.75 und x0.3, Stoermoment 2 N m an Gelenk 1
%                  und 2 (2.0-2.5 s), Parameter x1.5, alles zusammen (mit x0.75).
%                  -> results/benchmark_v2/eval/<condition>.csv
%
%   Seed-Zahl (Absprache 20.09.2026): Hauptbenchmark und Zwei-Segment-Bahn mit
%   10 Seeds, weil dort auf Seed-Ebene getestet wird. Die beschreibenden Studien
%   (r24b, ablation, sensitivity) mit 5 Seeds. Ausgewertet wird ueberall nach
%   demselben Schema: Abbruchrate ueber alle Laeufe, KPIs nur ueber die Laeufe
%   ohne Sicherheitsabbruch (analyzeBenchmark).
%
%   Reward-Varianten (r24b, ablation, sensitivity) werden mit den NOMINALEN
%   Gewichten ausgewertet (Condition "nominal"). Die Trajektorien und K2-K9
%   haengen nicht von den Gewichten ab, K1 ist dann zwischen den Varianten
%   vergleichbar. Die Trainingskurven (stats) enthalten den Reward der Variante.

    arguments
        studies (1,:) string
    end

    nomReward = benchmarkConfig().reward;
    evalNomReward = struct('reward', nomReward);

    jobs  = table(strings(0,1), strings(0,1), zeros(0,1), strings(0,1), cell(0,1), ...
                  'VariableNames', {'agent','mode','seed','campaign','config'});
    evals = table(strings(0,1), strings(0,1), cell(0,1), ...
                  'VariableNames', {'campaign','condition','config'});

    for study = studies
        switch study
            case "r24b"
                for term = ["wori","wp"]
                    for f = [0.5 2]
                        camp = sprintf('r24b_%s_x%s', term, factorTag(f));
                        cfg  = struct('reward', struct(term, f * nomReward.(term)));
                        jobs  = addJobs(jobs, ["SAC","TRPO","PPO"], "default", 0:4, camp, cfg);
                        evals = addEval(evals, camp, "nominal", evalNomReward);
                    end
                end

            case "ablation"
                for term = ["wp","wv","wori","wwb","wvb","wu","wd","kp","kb"]
                    camp = sprintf('abl_%s0', term);
                    cfg  = struct('reward', struct(term, 0));
                    jobs  = addJobs(jobs, "PPO", "optimized", 0:4, camp, cfg);
                    evals = addEval(evals, camp, "nominal", evalNomReward);
                end

            case "sensitivity"
                for term = ["wp","wv","wori","wwb"]
                    for f = [0.5 2]
                        camp = sprintf('sens_%s_x%s', term, factorTag(f));
                        cfg  = struct('reward', struct(term, f * nomReward.(term)));
                        jobs  = addJobs(jobs, "PPO", "optimized", 0:4, camp, cfg);
                        evals = addEval(evals, camp, "nominal", evalNomReward);
                    end
                end

            case "twoseg"
                cfg = struct('traj', "linear");
                jobs = addJobs(jobs, "TRPO", "default",   0:9, "twoseg", cfg);
                jobs = addJobs(jobs, "PPO",  "optimized", 0:9, "twoseg", cfg);
                jobs = addJobs(jobs, "PPO",  "default",   0:9, "twoseg", cfg);
                evals = addEval(evals, "twoseg", "nominal", struct());

            case "stress"
                noise = struct('obs_std', 0.005);
                dist  = struct('tau', [2; 2; 0; 0], 't_on', 2.0, 't_off', 2.5);
                evals = addEval(evals, "benchmark_v2", "noise",       struct('noise', noise));
                evals = addEval(evals, "benchmark_v2", "tau_sat_075", struct('tau_sat_scale', 0.75));
                % 0.75 (1.5 N m) greift bei TRPO default und PPO optimiert nie (max |tau|
                % nominal 0.76 bzw. 1.22 N m), daher zusaetzlich 0.3 (0.6 N m)
                evals = addEval(evals, "benchmark_v2", "tau_sat_03",  struct('tau_sat_scale', 0.3));
                evals = addEval(evals, "benchmark_v2", "disturbance", struct('dist', dist));
                evals = addEval(evals, "benchmark_v2", "param_15",    struct('robot', struct('param_scale', 1.5)));
                evals = addEval(evals, "benchmark_v2", "combined",    struct('noise', noise, ...
                    'tau_sat_scale', 0.75, 'dist', dist, 'robot', struct('param_scale', 1.5)));

            otherwise
                error('partCJobs:study', 'Unbekannte Studie "%s".', study);
        end
    end

    % Alle Configs pruefen (Tippfehler in Feldnamen -> Fehler jetzt, nicht auf dem Worker)
    cellfun(@(c) benchmarkConfig(c), [jobs.config; evals.config], 'UniformOutput', false);
end

function jobs = addJobs(jobs, agents, mode, seeds, camp, cfg)
    [a, s] = ndgrid(agents, seeds);
    n = numel(a);
    jobs = [jobs; table(a(:), repmat(string(mode), n, 1), s(:), repmat(string(camp), n, 1), ...
                        repmat({cfg}, n, 1), 'VariableNames', jobs.Properties.VariableNames)];
end

function evals = addEval(evals, camp, cond, cfg)
    evals = [evals; table(string(camp), string(cond), {cfg}, 'VariableNames', evals.Properties.VariableNames)];
end

function t = factorTag(f)
    t = strrep(sprintf('%g', f), '.', '');     % 0.5 -> "05", 2 -> "2"
end
