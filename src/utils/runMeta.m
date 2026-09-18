function meta = runMeta()
% runMeta  Herkunftsdaten fuer eine Ergebnisdatei.
%
%   meta = runMeta()
%
%   Felder: gitCommit, gitDirty (uncommittete Aenderungen?), modelSha256
%   (Pruefsumme von SpaceRobot.slx), matlab (Version), host, created.
%   Damit laesst sich jede Zahl im Paper einem Code- und Modellstand zuordnen.

    proot = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % src/utils -> Root
    meta.gitCommit = gitOut(proot, 'rev-parse HEAD');
    meta.gitDirty  = ~isempty(gitOut(proot, 'status --porcelain --untracked-files=no'));
    meta.modelSha256 = fileSha256(fullfile(proot, 'SpaceRobot.slx'));
    meta.matlab  = version;
    meta.host    = strtrim(getenv('COMPUTERNAME'));
    meta.created = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
end

function out = gitOut(proot, args)
    [status, out] = system(sprintf('git -C "%s" %s', proot, args));
    out = strtrim(out);
    if status ~= 0, out = ''; end
end

function h = fileSha256(f)
    fid = fopen(f, 'r');
    if fid < 0, h = ''; return; end
    bytes = fread(fid, inf, '*uint8');
    fclose(fid);
    md = java.security.MessageDigest.getInstance('SHA-256');
    md.update(bytes);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2)', 1, []));
end
