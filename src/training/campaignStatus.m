function T = campaignStatus(campaign, opt)
% campaignStatus  Stand einer laufenden oder fertigen Kampagne.
%
%   campaignStatus("sens_r24b")             % Uebersicht im Command Window
%   campaignStatus("sens_r24b", All=true)   % auch fertige und wartende Laeufe einzeln
%   T = campaignStatus("sens_r24b");        % Tabelle, ohne Ausgabe
%
%   Quellen in results/<campaign>/:
%     campaign.mat    Jobliste (von runCampaign)
%     agents/*.mat    fertige Laeufe
%     progress/*.csv  eine Zeile je Trainingsepisode (trainOne, progressLogger)
%     runs.csv        Laufzeit fertiger Laeufe
%     errors.log      fehlgeschlagene Laeufe
%   Funktioniert auch in einer zweiten MATLAB-Sitzung, waehrend runCampaign
%   rechnet. runCampaign ruft es selbst regelmaessig auf.
%
%   Status: fertig | laeuft | haengt? (seit > 15 min keine Episode) | Fehler | wartet
%   Die Restzeit der Kampagne ist eine Schaetzung: Episodenrate je Agent/Modus
%   aus laufenden und fertigen Laeufen, verteilt auf die Worker.

    arguments
        campaign (1,1) string
        opt.All     (1,1) logical = false
        opt.OutRoot (1,1) string = string(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'results'))
    end

    campDir = fullfile(opt.OutRoot, campaign);
    if ~isfile(fullfile(campDir, 'campaign.mat'))
        error('campaignStatus:missing', 'Keine Kampagne %s (campaign.mat fehlt).', campDir);
    end
    C = load(fullfile(campDir, 'campaign.mat'), 'jobs', 'opt');
    jobs = C.jobs;
    workers = 6;
    campCfg = struct();
    if isfield(C, 'opt')
        if isfield(C.opt, 'Workers') && C.opt.Workers > 0, workers = C.opt.Workers; end
        if isfield(C.opt, 'Config'), campCfg = C.opt.Config; end
    end
    maxEpDefault = benchmarkConfig(campCfg).train.maxEpisodes;   % fuer noch nicht gestartete Laeufe

    n = height(jobs);
    run      = strings(n, 1);
    status   = strings(n, 1);
    episode  = zeros(n, 1);
    maxEp    = maxEpDefault * ones(n, 1);
    elapsedMin = nan(n, 1);
    remainMin  = nan(n, 1);
    reward25   = nan(n, 1);          % mittlerer Reward der letzten 25 Episoden
    lastUpdate = NaT(n, 1);

    failed = failedRuns(fullfile(campDir, 'errors.log'));
    R = readRuns(fullfile(campDir, 'runs.csv'));

    for i = 1:n
        run(i) = sprintf('%s_%s_s%d', upper(string(jobs.agent(i))), lower(string(jobs.mode(i))), jobs.seed(i));
        agentFile = fullfile(campDir, 'agents', run(i) + ".mat");
        progFile  = fullfile(campDir, 'progress', run(i) + ".csv");
        P = readProgress(progFile);

        if isfile(agentFile)
            status(i) = "fertig";
            episode(i) = maxEp(i);
            k = find(R.run == run(i), 1, 'last');
            if ~isempty(k)
                elapsedMin(i) = R.wallclock_min(k);
                episode(i) = R.episodes(k);
                maxEp(i) = R.episodes(k);
            end
            remainMin(i) = 0;
        elseif ~isempty(P)
            episode(i) = P.episode(end);
            maxEp(i)   = P.max_episodes(end);
            elapsedMin(i) = P.elapsed_s(end) / 60;
            remainMin(i)  = elapsedMin(i) / episode(i) * (maxEp(i) - episode(i));
            t = P.time(end);
            if ~isdatetime(t), t = datetime(t, 'InputFormat', 'yyyy-MM-dd HH:mm:ss'); end
            lastUpdate(i) = t;
            if datetime('now') - lastUpdate(i) <= minutes(15)
                status(i) = "laeuft";          % auch nach frueherem Fehler neu gestartet
            elseif ismember(run(i), failed)
                status(i) = "Fehler";
            else
                status(i) = "haengt?";
            end
        elseif ismember(run(i), failed)
            status(i) = "Fehler";
        else
            status(i) = "wartet";
        end
        if ~isempty(P)
            reward25(i) = mean(P.reward(max(1, end-24):end));
        end
    end

    % Restzeit wartender Laeufe: Minuten je Episode aus Laeufen desselben
    % Agenten/Modus, sonst aus allen Laeufen mit Daten
    rate = elapsedMin ./ max(episode, 1);
    rate(episode == 0) = NaN;
    key = upper(string(jobs.agent)) + "_" + lower(string(jobs.mode));
    for i = find(status == "wartet")'
        r = rate(key == key(i));
        if all(isnan(r)), r = rate; end
        remainMin(i) = mean(r, 'omitnan') * maxEp(i);
    end

    T = table(run, status, episode, maxEp, elapsedMin, remainMin, reward25, lastUpdate);
    if nargout > 0, return; end

    % ---- Ausgabe ----
    cnt = @(s) nnz(status == s);
    fprintf('[campaignStatus] %s  (%s): %d/%d fertig, %d laufen, %d warten', campaign, ...
        char(datetime('now', 'Format', 'HH:mm')), cnt("fertig"), n, cnt("laeuft"), cnt("wartet"));
    if cnt("haengt?") > 0, fprintf(', %d ohne Fortschritt seit > 15 min', cnt("haengt?")); end
    if cnt("Fehler") > 0,  fprintf(', %d Fehler (errors.log)', cnt("Fehler")); end
    fprintf('\n');

    show = status ~= "fertig" & status ~= "wartet";
    if opt.All, show = true(n, 1); end
    for i = find(show)'
        fprintf('  %-22s %-8s %4d/%-4d  %6.1f min', run(i), status(i), episode(i), maxEp(i), elapsedMin(i));
        if status(i) ~= "fertig" && ~isnan(remainMin(i))
            fprintf(', noch ca. %5.1f min', remainMin(i));
        end
        if ~isnan(reward25(i))
            fprintf(' | Reward (letzte 25) %8.2f', reward25(i));
        end
        fprintf('\n');
    end

    busy = status == "laeuft" | status == "haengt?";
    todo = remainMin(busy | status == "wartet");
    if ~isempty(todo) && ~all(isnan(todo))
        eta = max([remainMin(busy); sum(todo, 'omitnan') / workers]);
        fprintf('  Restzeit Kampagne: ca. %.0f min (Schaetzung, %d Worker) -> fertig ca. %s\n', ...
            eta, workers, char(datetime('now') + minutes(eta), 'HH:mm'));
    end
end

function P = readProgress(f)
    P = [];
    if ~isfile(f), return; end
    try
        P = readtable(f, 'TextType', 'string', 'Delimiter', ',');
    catch
        return;                          % Datei wird gerade geschrieben
    end
    if height(P) == 0, P = []; end
end

function R = readRuns(f)
    R = table(strings(0,1), zeros(0,1), zeros(0,1), 'VariableNames', {'run','episodes','wallclock_min'});
    if ~isfile(f), return; end
    X = readtable(f, 'TextType', 'string', 'Delimiter', ',');
    if height(X) == 0, return; end
    R = table(upper(X.agent) + "_" + lower(X.mode) + "_s" + string(X.seed), X.episodes, X.wallclock_min, ...
        'VariableNames', {'run','episodes','wallclock_min'});
end

function names = failedRuns(f)
% Laufnamen aus den Kopfzeilen "==== <Zeit> | AGENT mode sSEED ====" in errors.log
    names = strings(0, 1);
    if ~isfile(f), return; end
    tok = regexp(fileread(f), '==== [^|]*\| (\S+) (\S+) s(\d+) ====', 'tokens');
    for k = 1:numel(tok)
        names(end+1, 1) = sprintf('%s_%s_s%s', upper(tok{k}{1}), lower(tok{k}{2}), tok{k}{3}); %#ok<AGROW>
    end
end
