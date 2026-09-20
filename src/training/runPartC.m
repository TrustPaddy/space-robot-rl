% runPartC  Kampagnen fuer Teil C der Revision: Training + Auswertung.
%
%   Vorher: alles committen. Jede Agentendatei speichert den Git-Commit und
%   ob es uncommittete Aenderungen gab (meta.gitDirty).
%
%   Start:   >> startup
%            >> runPartC
%   Stand:   in einer zweiten MATLAB-Sitzung (startup) z. B.
%            >> campaignStatus("r24b_wori_x2")
%            runCampaign gibt ausserdem alle 10 min den Stand aller Kampagnen aus.
%
%   Fortsetzbar: bei Abbruch einfach erneut starten. Fertige Trainingslaeufe
%   und vorhandene Auswertungsdateien (results/<Kampagne>/eval/<Bedingung>.csv)
%   werden uebersprungen.
%
%   Studien und Kampagnenordner: siehe partCJobs. Reihenfolge:
%     1) Stresstests (nur Auswertung der 35 benchmark_v2-Agenten, 6 Bedingungen, ca. 20 min)
%     2) alle Trainingslaeufe in einem Pool (102 Laeufe, grob 3,5 h mit 6 Workern)
%     3) Auswertung der neuen Kampagnen

if exist('benchmarkConfig', 'file') ~= 2
    run(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'startup.m'));
end

studies = ["stress", "twoseg", "ablation", "sensitivity", "r24b"];   % Auswahl
workers = 6;          % Ryzen 7 7700 (8 Kerne), 32 GB RAM

[jobsC, evalsC] = partCJobs(studies);
resultsDir = fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results');

% ---- 1) Stresstests zuerst (kurz, unabhaengig vom Training) ----
isStress = evalsC.campaign == "benchmark_v2";
for i = find(isStress)'
    if isfile(fullfile(resultsDir, evalsC.campaign(i), 'eval', evalsC.condition(i) + ".csv")), continue; end
    evaluateCampaign(evalsC.campaign(i), Condition=evalsC.condition(i), Config=evalsC.config{i}, Workers=workers);
end

% ---- 2) Training: laengste Laeufe zuerst (SAC >> TRPO > PPO) ----
if ~isempty(jobsC)
    [~, rankC] = ismember(jobsC.agent, ["SAC","TRPO","PPO"]);
    jobsC = sortrows(addvars(jobsC, rankC), 'rankC');
    jobsC.rankC = [];
    runCampaign(jobsC, Workers=workers);
end

% ---- 3) Auswertung der neuen Kampagnen (nur vollstaendige) ----
for i = find(~isStress)'
    campC = evalsC.campaign(i);
    if isfile(fullfile(resultsDir, campC, 'eval', evalsC.condition(i) + ".csv")), continue; end
    nJobs   = nnz(jobsC.campaign == campC);
    nAgents = numel(dir(fullfile(resultsDir, campC, 'agents', '*.mat')));
    if nAgents < nJobs
        warning('runPartC:incomplete', '%s: nur %d von %d Agenten fertig (errors.log), nicht ausgewertet.', ...
            campC, nAgents, nJobs);
        continue;
    end
    evaluateCampaign(campC, Condition=evalsC.condition(i), Config=evalsC.config{i}, Workers=workers);
end
fprintf('[runPartC] fertig.\n');
