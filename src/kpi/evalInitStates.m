function Q = evalInitStates(n, range_deg, seed)
% evalInitStates  Feste Startzustaende fuer die Auswertung.
%
%   Q = evalInitStates(30, 1.0, 2026)   % 4 x 30 Startwinkel [rad]
%
%   Gleichverteilt in +/- range_deg je Gelenk um die nominale Null-Pose, aus
%   einem eigenen Zufallsstrom (der globale RNG bleibt unberuehrt). Alle
%   Agenten werden mit denselben Spalten ausgewertet -> gepaarte Tests.

    s = RandStream('twister', 'Seed', seed);
    Q = deg2rad(range_deg) * (2*rand(s, 4, n) - 1);
end
