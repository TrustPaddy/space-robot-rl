function stop = saveBOState(results, ~, runDir)
% saveBOState  bayesopt-OutputFcn: Checkpoint + Fortschrittslog.
%   Laeuft nach jeder Evaluation auf dem Client (parallel-sicher).

    stop = false;

    % --- vollstaendigen Zustand als Checkpoint sichern ---
    try
        save(fullfile(runDir, 'bo_state.mat'), 'results');
    catch ME
        warning('saveBOState:save', 'Konnte bo_state.mat nicht speichern: %s', ME.message);
    end

    % --- Fortschritt (bester Objective-Wert bisher) mitschreiben ---
    try
        n   = numel(results.ObjectiveTrace);
        fid = fopen(fullfile(runDir, 'progress.log'), 'a');
        if fid >= 0
            fprintf(fid, '%s\titer=%d\tminObjective=%.6f\n', ...
                char(datetime('now')), n, results.MinObjective);
            fclose(fid);
        end
    catch
        % Logging ist unkritisch -> Fehler bewusst ignorieren
    end
end
