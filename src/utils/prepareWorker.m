function prepareWorker(proot)
% prepareWorker  Richtet einen Parallel-Worker fuer SpaceRobot-Simulationen ein.
%
%   wait(parfevalOnAll(pool, @prepareWorker, 0, proot))
%
%   Setzt die Projektpfade und gibt jedem Worker einen eigenen Simulink-
%   Cache-Ordner, damit sich parallele Kompilierungen von SpaceRobot.slx
%   nicht gegenseitig ueberschreiben.

    addpath(proot);
    addpath(genpath(fullfile(proot, 'src')));
    t = getCurrentTask();
    cache = fullfile(tempdir, sprintf('spacerobot_slcache_w%d', t.ID));
    Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', cache, 'createDir', true);
end
