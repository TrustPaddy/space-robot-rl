%% ============================================================
%  checkUnits.m – Überprüft Einheitenkonsistenz in einem Simulink-Modell
%  Autor: ChatGPT (Weltraumroboter Projekt)
%  ============================================================

% Modellname anpassen
modelName = 'SpaceRobot';   % <== hier ggf. ändern

% Modell laden (öffnet es nicht im Editor)
load_system(modelName);

% Alle Blöcke im Modell finden
blocks = find_system(modelName, 'Type', 'Block');

% Ergebnisliste vorbereiten
results = [];

fprintf('🔍 Überprüfe Einheiten in Modell "%s"...\n\n', modelName);

for i = 1:length(blocks)
    blk = blocks{i};
    try
        % Kompilierte Port-Einheiten abrufen
        u = get_param(blk, 'CompiledPortUnits');
        % Eingänge und Ausgänge zusammenführen
        inUnits  = strjoin(string(u.Inport),  ', ');
        outUnits = strjoin(string(u.Outport), ', ');
        
        % Prüfen, ob unspecified vorkommt
        flag = "";
        if contains(inUnits, 'unspecified') || contains(outUnits, 'unspecified')
            flag = "⚠️  Unbestimmt";
        else
            flag = "✅ OK";
        end
        
        % Ergebnis speichern
        results = [results; {blk, inUnits, outUnits, flag}];
    catch
        % Falls Block keine Ports hat (z. B. Scope, Display)
        continue
    end
end

% Tabelle erstellen
T = cell2table(results, 'VariableNames', ...
    {'Block', 'InputUnits', 'OutputUnits', 'Status'});

% Tabelle sortieren: Unbestimmte zuerst
T = [T(T.Status ~= "✅ OK", :); T(T.Status == "✅ OK", :)];

% Ergebnisse anzeigen
disp(T);

% Optional: In Excel exportieren
writetable(T, [modelName '_UnitCheck.xlsx']);
fprintf('\n📄 Ergebnisse gespeichert in "%s_UnitCheck.xlsx".\n', modelName);

%% ============================================================
% Ende
% ============================================================
