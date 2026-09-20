function logger = progressLogger(progFile, maxEpisodes)
% progressLogger  Logger fuer train(), der nach jeder Episode eine Zeile schreibt.
%
%   logger = progressLogger(progFile, maxEpisodes)
%   result = train(agent, env, trainOpts, Logger=logger)
%
%   progFile (CSV) bekommt je Trainingsepisode eine Zeile:
%     episode, max_episodes, reward, steps, elapsed_s, time
%   Die Datei wird beim Aufruf neu angelegt (ein wiederholter Lauf beginnt
%   von vorn). campaignStatus liest diese Dateien.
%
%   Der Callback gibt [] zurueck, der Logger speichert also selbst keine Daten
%   und veraendert das Training nicht (Regressionstest: bitgleiche Rewards mit
%   und ohne Logger). Sein Pflicht-Ordner (nur Metadaten) ist ein eigener
%   Ordner in tempdir, trainOne loescht ihn nach dem Training.

    d = fileparts(progFile);
    if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
    fid = fopen(progFile, 'w');
    if fid < 0
        error('progressLogger:open', '%s nicht beschreibbar.', progFile);
    end
    fprintf(fid, 'episode,max_episodes,reward,steps,elapsed_s,time\n');
    fclose(fid);

    logger = rlDataLogger();
    logger.LoggingOptions.LoggingDirectory = tempname;
    t0 = tic;
    logger.EpisodeFinishedFcn = @(data) writeLine(data, progFile, maxEpisodes, t0);
end

function out = writeLine(data, progFile, maxEpisodes, t0)
    out = [];
    fid = fopen(progFile, 'a');
    if fid < 0, return; end            % Fortschritt ist optional, Training laeuft weiter
    fprintf(fid, '%d,%d,%.6f,%d,%.1f,%s\n', data.EpisodeCount, maxEpisodes, ...
        data.EpisodeInfo.CumulativeReward, data.EpisodeInfo.StepsTaken, toc(t0), ...
        char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
    fclose(fid);
end
