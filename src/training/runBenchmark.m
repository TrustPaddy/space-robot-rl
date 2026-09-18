% runBenchmark  Hauptbenchmark (Paper Sec. VI und VII-A): Training + Auswertung.
%
%   Vorher: alles committen. Jede Agentendatei speichert den Git-Commit und
%   ob es uncommittete Aenderungen gab (meta.gitDirty).
%
%   6 Agenten mit MATLAB-Standardhyperparametern + optimiertes PPO, je 5 Seeds
%   = 35 Trainings a 1000 Episoden. Fortsetzbar: bei Abbruch einfach erneut
%   starten, fertige Laeufe werden uebersprungen.
%
%   Ergebnisse: results/benchmark_v2/agents/*.mat, runs.csv, eval/nominal.csv

if exist('benchmarkConfig', 'file') ~= 2
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
end

campaign = "benchmark_v2";
seeds    = 0:4;
workers  = 6;          % Ryzen 7 7700 (8 Kerne), 32 GB RAM

agents = ["PG","PPO","TRPO","DDPG","TD3","SAC"];
[a, s] = ndgrid(agents, seeds);
jobs = table(a(:), repmat("default", numel(a), 1), s(:), 'VariableNames', {'agent','mode','seed'});
jobs = [jobs; table(repmat("PPO", numel(seeds), 1), repmat("optimized", numel(seeds), 1), seeds(:), ...
                    'VariableNames', {'agent','mode','seed'})];

% Laengste Laeufe zuerst starten (Off-Policy-Agenten sind am langsamsten),
% damit am Ende nicht ein einzelner Worker allein weiterrechnet
order = ["TD3","SAC","DDPG","TRPO","PPO","PG"];
[~, rank] = ismember(jobs.agent, order);
jobs = sortrows(addvars(jobs, rank), 'rank');
jobs.rank = [];

runCampaign(jobs, Campaign=campaign, Workers=workers);
evaluateCampaign(campaign, Condition="nominal", Workers=workers);
