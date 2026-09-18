function diagnoseBOErrors(stateFile)
% diagnoseBOErrors  Zeigt die ECHTEN Fehlermeldungen eines fehlgeschlagenen
% bayesopt-Laufs an, die trainAndEvaluate.m im catch-Block als
% userData.error in results.UserDataTrace abgelegt hat.
%
%   diagnoseBOErrors()            % neueste bo_state.mat unter SavedAgents/Circular/BO
%   diagnoseBOErrors(stateFile)   % konkrete bo_state.mat / results.mat
%
% Hintergrund: Laufen die bayesopt-Trials parallel, erscheinen die
% warning()-Meldungen der Worker NICHT auf der Client-Konsole. Die Objective
% steht dann stumm auf 1e6. Die zugehoerige ME.message wurde aber pro Trial in
% userData.error gespeichert -> hier lesen wir sie ohne erneuten Lauf aus.

    if nargin < 1 || isempty(stateFile)
        stateFile = findNewestState();
    end
    if isempty(stateFile) || ~isfile(stateFile)
        error('diagnoseBOErrors:notFound', ...
            'Keine bo_state.mat/results.mat gefunden (%s).', string(stateFile));
    end
    fprintf('Lade: %s\n', stateFile);

    S = load(stateFile);
    results = pickResults(S);
    if isempty(results)
        error('diagnoseBOErrors:noResults', ...
            'Datei enthaelt kein BayesianOptimization-Objekt (Feld "results").');
    end

    obj = results.ObjectiveTrace(:);
    ud  = results.UserDataTrace(:);
    n   = numel(obj);
    fprintf('Trials gesamt        : %d\n', n);
    fprintf('Objective == 1e6     : %d\n', sum(obj >= 1e6 - 1));
    fprintf('Objective finite/gut : %d\n\n', sum(isfinite(obj) & obj < 1e6 - 1));

    % --- Fehlermeldungen einsammeln ---
    msgs = strings(0,1);
    for i = 1:numel(ud)
        e = extractError(ud{i});
        if strlength(e) > 0
            msgs(end+1,1) = e; %#ok<AGROW>
        end
    end

    if isempty(msgs)
        fprintf(['Keine userData.error-Eintraege gefunden. Entweder sind die Trials\n' ...
                 'nicht via catch gescheitert, oder UserDataTrace ist leer.\n']);
        return;
    end

    % --- Eindeutige Meldungen mit Haeufigkeit (haeufigste zuerst) ---
    [uniq, ~, idx] = unique(msgs, 'stable');
    counts = accumarray(idx, 1);
    [counts, order] = sort(counts, 'descend');
    uniq = uniq(order);

    fprintf('==== Eindeutige Fehlermeldungen (%d) ====\n\n', numel(uniq));
    for k = 1:numel(uniq)
        fprintf('[%d x] %s\n\n', counts(k), uniq(k));
    end
end

% ===== Hilfsfunktionen =====================================================

function f = findNewestState(root)
    if nargin < 1, root = fullfile('SavedAgents','Circular','BO'); end
    f = '';
    d = dir(fullfile(root, '**', 'bo_state.mat'));
    if isempty(d)
        d = dir(fullfile(root, '**', 'results.mat'));
    end
    if isempty(d), return; end
    [~, ix] = max([d.datenum]);
    f = fullfile(d(ix).folder, d(ix).name);
end

function results = pickResults(S)
    results = [];
    if isfield(S, 'results') && isa(S.results, 'BayesianOptimization')
        results = S.results; return;
    end
    % Fallback: erstes BayesianOptimization-Objekt in der Datei
    fn = fieldnames(S);
    for i = 1:numel(fn)
        if isa(S.(fn{i}), 'BayesianOptimization')
            results = S.(fn{i}); return;
        end
    end
end

function e = extractError(u)
    e = "";
    if isstruct(u) && isfield(u, 'error') && ~isempty(u.error)
        e = string(u.error);
        e = strtrim(e);
    end
end
