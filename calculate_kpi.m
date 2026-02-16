clc; clear; close all;

% Sicherheits-/Spec-Parameter
d_safe   = 0.02;        % Mindestabstand [m]
tau_max  = 2.0;         % Aktuatorgrenze [N*m]
dt_agent = 0.05;        % Agent rate [s]
q1_lim   = deg2rad([ -85  85 ]);
qi_lim   = deg2rad([ -170 170 ]);

% --- Initialwerte für Reward & Done ---
reward_init = 0;      % z.B. Start-Reward = 0
isdone_init = 0;  % oder 0, wenn das Signal double ist

assignin('base','d_safe',d_safe);
assignin('base','tau_max',tau_max);
assignin('base','dt_agent',dt_agent);
assignin('base','q1_lim',q1_lim);
assignin('base','qi_lim',qi_lim);

% Soll-Kreisbahn definieren
T = 8.5;                
omega = pi/T;
r = 0.5;               
center = [4.5-r, 0.0, 0.0];  

Ts  = 0.01;        % Diskrete Schrittzeit der RL-Schleife
Ts_agent = 0.1;
t   = 0:Ts:T;      % Zeitvektor 

x = center(1) + r*cos(omega*t);
y = center(2) + r*sin(omega*t);
z = center(3) + 0*t;

% % Eckpunkte aus dem ursprünglichen Kreis (gleichbleibend)
% P0 = [center(1)+r, center(2),   center(3)]; % Start (t=0)
% P1 = [center(1),   center(2)+r, center(3)]; % "Höhepunkt" (t=T/4)
% P2 = [center(1)-r, center(2),   center(3)];                                 % Ende (t=T) = Start
% 
% % Zeitaufteilung: zwei Segmente (P0->P1 und P1->P2)
% t1 = T/2;  % Wechsel bei der Hälfte der Periode
% 
% % Vorbelegen
% x = zeros(size(t));
% y = zeros(size(t));
% z = zeros(size(t));
% 
% % Segment 1: P0 -> P1 für t in [0, T/2]
% idx1 = (t <= t1);
% s1 = (t(idx1) - 0) / (t1 - 0);            % s in [0,1]
% x(idx1) = P0(1) + s1*(P1(1) - P0(1));
% y(idx1) = P0(2) + s1*(P1(2) - P0(2));
% z(idx1) = P0(3) + s1*(P1(3) - P0(3));
% 
% % Segment 2: P1 -> P2 für t in (T/2, T]
% idx2 = (t > t1);
% s2 = (t(idx2) - t1) / (T - t1);           % s in [0,1]
% x(idx2) = P1(1) + s2*(P2(1) - P1(1));
% y(idx2) = P1(2) + s2*(P2(2) - P1(2));
% z(idx2) = P1(3) + s2*(P2(3) - P1(3));

traj = [x' y' z'];   % gewünschte EE-Punkte

% Roboter laden
robot_rbt = importrobot('SpaceRobot.urdf');
robot_rbt.DataFormat = 'row';
eeBodyName = robot_rbt.BodyNames{end};

% Inverse Kinematik vorbereiten
ik = inverseKinematics('RigidBodyTree', robot_rbt); 

qseed = randomConfiguration(robot_rbt);
q_des = zeros(numel(t), numel(qseed));

dt = mean(diff(t));
vref = [zeros(1,3); diff(traj)/dt];   % einfache Ableitung
EE_ref = timeseries(traj, t);         % Nx3
EE_vref = timeseries(vref, t);        % Nx3
assignin('base','EE_ref',EE_ref);
assignin('base','EE_vref',EE_vref);

% den Agenten Namen ändern
load('SpaceRobot_PPO_agent.mat','agent');   % legt 'agent' in base an
%% 

mdl = 'SpaceRobot';
set_param(mdl, ...
    'StopTime', num2str(T), ...
    'Solver', 'ode4', ...        % fester Integrator
    'FixedStep', num2str(Ts), ...     
    'SolverType', 'Fixed-step' ...
);

for i = 1:100
    simOut = sim("SpaceRobot");
    logsouts{i} = simOut.logsout;
end

params.tau_max = tau_max;          
params.q_min   = [q1_lim(1), qi_lim(1), qi_lim(1), qi_lim(1)];   
params.q_max   = [q1_lim(2), qi_lim(2), qi_lim(2), qi_lim(2)];

kpi = computeKPIsFromLogs(logsouts, params)
%% 


% %% ---------------------------------------------------------------
% %  Beispielplot: Soll- vs. Ist-Endeffektortrajektorie (XY) Episode 1
% % ---------------------------------------------------------------
% epIdx = 2;                          
% logsout = logsouts{epIdx};
% 
% % Fehler-Signal holen (EE_ref - EE_ist)
% EE_diff = logsout.getElement('p_EE').Values;
% t_sim   = EE_diff.Time;
% 
% % Daten auf N x 3 bringen
% EE_diff_data = reshape_time_series(EE_diff.Data);
% 
% % Solltrajektorie auf Simulationszeit interpolieren
% EE_ref_data = interp1(EE_ref.Time, EE_ref.Data, t_sim, 'linear', 'extrap');
% 
% % Isttrajektorie rekonstruieren (Annahme: diff = ref - ist)
% EE_ist = EE_diff_data;
% 
% % 2D-Plot der EE-Trajektorie nur x-y
% figure;
% plot(EE_ref_data(:,1), EE_ref_data(:,2), 'LineWidth', 1); hold on;
% plot(EE_ist(:,1),     EE_ist(:,2),     '--', 'LineWidth', 1);
% grid on; axis equal;
% xlabel('x [m]'); ylabel('y [m]');
% xlim([3.6 4.6]); ylim([-0.1 0.6]);
% legend('Soll-EE-Bahn', 'Ist-EE-Bahn', 'Location', 'best');
% title(sprintf('Endeffektortrajektorie (XY) – Episode 1', epIdx));
% axis equal;

% N = 100;
% 
% % ----- 1) Gemeinsamen Zeitvektor festlegen (z.B. von Episode 1) -----
% ep0 = 1;
% EE_diff0 = logsouts{ep0}.getElement('p_EE').Values;
% t_common = EE_diff0.Time(:);                 % gemeinsamer Zeitvektor (Nx1)
% 
% % Optional: Wenn du lieber auf gleichmäßigem Raster mitteln willst:
% % t_common = linspace(EE_diff0.Time(1), EE_diff0.Time(end), 1000)';
% 
% % ----- 2) Container für alle Episoden: (Nt x 3 x N) -----
% Nt = numel(t_common);
% EE_ref_all = nan(Nt, 3, N);
% EE_ist_all = nan(Nt, 3, N);
% 
% for ep = 1:N
%     logsout = logsouts{ep};
% 
%     % diff-Signal (ref - ist) holen
%     EE_diff = logsout.getElement('p_EE').Values;
%     t_ep    = EE_diff.Time(:);
%     diff_ep = reshape_time_series(EE_diff.Data);   % -> (Ne x 3)
% 
%     % Solltrajektorie auf Episodenzeit interpolieren
%     ref_ep = interp1(EE_ref.Time, EE_ref.Data, t_ep, 'linear', 'extrap');
% 
%     % Isttrajektorie rekonstruieren: ist = ref - diff
%     ist_ep = diff_ep;
% 
%     % Jetzt alles auf gemeinsamen Zeitvektor bringen
%     EE_ref_all(:,:,ep) = interp1(t_ep, ref_ep, t_common, 'linear', 'extrap');
%     EE_ist_all(:,:,ep) = interp1(t_ep, ist_ep, t_common, 'linear', 'extrap');
% end
% 
% % ----- 3) Mittelwert (und optional Std) über Episoden -----
% EE_ref_mean = mean(EE_ref_all, 3, 'omitnan');   % (Nt x 3)
% EE_ist_mean = mean(EE_ist_all, 3, 'omitnan');   % (Nt x 3)
% 
% EE_ref_std  = std(EE_ref_all, 0, 3, 'omitnan'); % optional
% EE_ist_std  = std(EE_ist_all, 0, 3, 'omitnan'); % optional
% 
% % ----- 4) Plot: gemittelte XY-Bahn -----
% figure;
% plot(EE_ref_mean(:,1), EE_ref_mean(:,2), 'LineWidth', 1.5); hold on;
% plot(EE_ist_mean(:,1), EE_ist_mean(:,2), '--', 'LineWidth', 1.5);
% grid on; axis equal;
% xlabel('x [m]'); ylabel('y [m]');
% xlim([3.4 4.6]); ylim([-0.1 0.6]);
% legend('Target EE Path', 'Average Actual EE Path', 'Location', 'best');
% title('End Effector Trajectory (XY) – Average over 30 Episodes');

N = 100;

% --- gemeinsamer Zeitvektor (z.B. Episode 1) ---
ep0 = 1;
q0_ts = logsouts{ep0}.getElement('basis_ori').Values;
t_common = q0_ts.Time(:);
Nt = numel(t_common);

% --- Container: (Nt x 4 x N) ---
Q_all = nan(Nt, 4, N);

for ep = 1:N
    logsout = logsouts{ep};
    q_ts = logsout.getElement('basis_ori').Values;

    t_ep = q_ts.Time(:);
    q_ep = reshape_time_series(q_ts.Data);   % -> (Ne x 4)  [w x y z]

    % Auf gemeinsamen Zeitvektor interpolieren
    Q_all(:,:,ep) = interp1(t_ep, q_ep, t_common, 'linear', 'extrap');
end

% --- Quaternionen sign-konsistent machen (q und -q sind gleich!) ---
% Referenz: Mittelwert der ersten Episode (oder q aus Episode 1)
q_ref = squeeze(Q_all(:,:,ep0));  % (Nt x 4)

for ep = 1:N
    q_ep = squeeze(Q_all(:,:,ep)); % (Nt x 4)

    % pro Zeitschritt: wenn Dot < 0 -> Vorzeichen flippen
    dots = sum(q_ep .* q_ref, 2);            % (Nt x 1)
    flip = dots < 0;
    q_ep(flip,:) = -q_ep(flip,:);

    Q_all(:,:,ep) = q_ep;
end

% --- Mittelwert bilden und pro Sample normalisieren ---
Q_mean = mean(Q_all, 3, 'omitnan');          % (Nt x 4)
Q_std  = std(Q_all, 0, 3, 'omitnan');        % optional

Q_mean = Q_mean ./ vecnorm(Q_mean, 2, 2);    % normalisieren je Zeile

% --- Plot: w,x,y,z über Zeit ---
figure;
plot(t_common, Q_mean(:,1), 'LineWidth', 1.5); hold on;
plot(t_common, Q_mean(:,2), 'LineWidth', 1.5);
plot(t_common, Q_mean(:,3), 'LineWidth', 1.5);
plot(t_common, Q_mean(:,4), 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Normalized Quaternion');
legend('w','x','y','z','Location','best');
xlim([0 9]); ylim([-0.2 1.2])
title('Base Orientation – Average over 30 Episodes');





function data2D = reshape_time_series(raw)

% Bringt ein timeseries.Data mit Form:
%   - chan x 1 x N
%   - 1 x chan x N
%   - 1 x 1 x N
%   - N x chan
% in die einheitliche Form:
%   N x chan

    sz = size(raw);

    if ndims(raw) == 3
        % Wir nehmen an: letzter Index = Zeit
        % typische Fälle: 3x1xN oder 4x1xN
        data2D = squeeze(permute(raw, [3 1 2]));  % N x chan
    elseif isvector(raw)
        % 1D: N oder 1xN -> N x 1
        data2D = raw(:);
    else
        % schon 2D: hoffen, dass es N x chan ist; sonst transponieren
        if sz(1) == 1 && sz(2) > 1
            data2D = raw.';                       % 1 x chan -> chan x 1
        else
            data2D = raw;
        end
    end
end