function appendRunLog(csvFile, agentFile)
% appendRunLog  Haengt die Eckdaten eines Trainingslaufs an runs.csv an.
%
%   appendRunLog(csvFile, agentFile)   % agentFile: Ergebnis von trainOne

    L = load(agentFile, 'stats', 'meta', 'wallclock', 'agentType', 'mode', 'seed');
    er = L.stats.EpisodeReward;
    isNew = ~isfile(csvFile);
    fid = fopen(csvFile, 'a');
    if fid < 0
        warning('appendRunLog:open', '%s nicht beschreibbar.', csvFile);
        return;
    end
    c = onCleanup(@() fclose(fid));
    if isNew
        fprintf(fid, 'agent,mode,seed,episodes,mean_reward_last50,wallclock_min,git_commit,git_dirty,model_sha256,created\n');
    end
    fprintf(fid, '%s,%s,%d,%d,%.6f,%.2f,%s,%d,%s,%s\n', L.agentType, L.mode, L.seed, numel(er), ...
        mean(er(max(1, end-49):end)), L.wallclock/60, L.meta.gitCommit, L.meta.gitDirty, ...
        L.meta.modelSha256, L.meta.created);
end
