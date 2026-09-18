function kpi = computeKPIsFromLogs(logsoutIn, params)
% Rückgabe:
%   kpi struct mit Skalaren:
%     K1  Mean Episode Return
%     K2  EE-Position MSE
%     K3  Maximaler EE-Trackingfehler
%     K4  Basis-Orientierungsfehler (Mittelwert)
%     K5  Kollisionsrate (Episodenanteil)
%     K6  Gelenkgrenzen-Verletzungsrate
%     K7  Smoothness / Jerk-KPI
%     K8  Mittlere Basiswinkelgeschwindigkeit (||w_base||)
%     K9  Mittlerer Energieverbrauch

    % ===== logsoutIn in Cell-Array von Dataset normalisieren =====
    if isa(logsoutIn,'Simulink.SimulationData.Dataset')
        if numel(logsoutIn) == 1
            logsoutCell = {logsoutIn};
        else
            logsoutCell = cell(numel(logsoutIn),1);
            for i = 1:numel(logsoutIn)
                logsoutCell{i} = logsoutIn(i);
            end
        end

    elseif iscell(logsoutIn)
        logsoutCell = logsoutIn;

    elseif isstruct(logsoutIn) && isfield(logsoutIn,"logsout")
        % z.B. simOut(1:N)
        logsoutCell = cell(numel(logsoutIn),1);
        for i = 1:numel(logsoutIn)
            logsoutCell{i} = logsoutIn(i).logsout;
        end

    else
        error("computeKPIsFromLogs:UnsupportedType", ...
              "logsoutIn vom Typ %s wird nicht unterstützt.", class(logsoutIn));
    end

    NE = numel(logsoutCell);  % Anzahl Episoden

    % ===== Prealloc für episodenweise Größen =====
    epReturn          = zeros(NE,1);
    epTime            = zeros(NE,1);
    epMSE_EE          = zeros(NE,1);
    epMaxErr_EE       = zeros(NE,1);
    epHasCollision    = false(NE,1);
    epJointViol       = zeros(NE,1);
    epJerk            = zeros(NE,1);
    epOriMean         = zeros(NE,1);
    epBaseAngVelMean  = zeros(NE,1);   % mittlere Basiswinkelgeschwindigkeit
    epEnergy          = zeros(NE,1);   % Energie pro Episode

    for k = 1:NE
        logsout = logsoutCell{k};

        if ~isa(logsout,"Simulink.SimulationData.Dataset")
            error("Episode %d ist vom Typ %s, Dataset erwartet.", ...
                  k, class(logsout));
        end

        % ---------- Signale holen ----------
        reward   = logsout.getElement('reward').Values;
        EE_diff  = logsout.getElement('EE_pos_differenz').Values;
        tau      = logsout.getElement('tau').Values;
        isColl   = logsout.getElement('isCollision').Values;
        ori_err  = logsout.getElement('basis_orientation_error').Values;
        q        = logsout.getElement('q').Values;
        v_base   = logsout.getElement('v_base').Values; 
        w_base   = logsout.getElement('w_base').Values;   % Basiswinkelgeschwindigkeit
        dq       = logsout.getElement('dq').Values;       % Gelenkgeschwindigkeit

        % Zeitvektor
        t = reward.Time;

        % ---------- 1) Return (Episode, ohne Zeitgewicht) ----------
        epReturn(k) = sum(reward.Data);

        % ---------- 2) Episodenzeit ----------
        if numel(t) >= 2
            epTime(k) = t(end) - t(1);
        else
            epTime(k) = 0;
        end

        % ---------- 3) EE-Trackingfehler ----------
        EEdata = reshape_time_series(EE_diff.Data);    % N x dim
        if size(EEdata,2) == 1
            errNorm = abs(EEdata(:,1));
        else
            errNorm = vecnorm(EEdata,2,2);             % N x 1
        end
        epMSE_EE(k)    = mean(errNorm.^2);
        epMaxErr_EE(k) = max(errNorm);

        % ---------- 4) Kollision in Episode? ----------
        collData = reshape_time_series(isColl.Data);   % N x 1
        epHasCollision(k) = any(collData(:) > 0.5);

        % ---------- 5) Gelenkgrenzen-Verletzung ----------
        qData = reshape_time_series(q.Data);           % N x nCols
        nCols = size(qData,2);
        q_min = params.q_min(:)';                      % 1 x nJ_def
        q_max = params.q_max(:)';

        if numel(q_min) ~= nCols
            warning('computeKPIsFromLogs:qDimMismatch', ...
                'q dimension (%d) passt nicht zu q_min/q_max (%d). Grenzvektoren werden angepasst.', ...
                nCols, numel(q_min));

            if numel(q_min) > nCols
                q_min = q_min(1:nCols);
                q_max = q_max(1:nCols);
            else
                repFactor = ceil(nCols / numel(q_min));
                q_min = repmat(q_min, 1, repFactor);
                q_max = repmat(q_max, 1, repFactor);
                q_min = q_min(1:nCols);
                q_max = q_max(1:nCols);
            end
        end

        viol = (qData < q_min) | (qData > q_max);      % N x nCols
        epJointViol(k) = mean(viol(:));

        % ---------- 6) Smoothness / Jerk aus tau ----------
        tauData = reshape_time_series(tau.Data);       % N x nJ
        if size(tauData,1) >= 3
            tauDD    = diff(tauData,2,1);              % (N-2) x nJ
            jerkNorm = vecnorm(tauDD,2,2);             % (N-2) x 1
            epJerk(k)= mean(jerkNorm);
        else
            epJerk(k)= 0;
        end

        % ---------- 7) Basis-Orientierungsfehler ----------
        oriData = reshape_time_series(ori_err.Data);   % N x dim
        if size(oriData,2) == 1
            oriNorm = abs(oriData(:,1));
        else
            oriNorm = vecnorm(oriData,2,2);            % N x 1
        end
        epOriMean(k) = mean(oriNorm);

        % ---------- 8) Mittlere Basiswinkelgeschwindigkeit ||w_base|| ----------
        wBaseData = reshape_time_series(w_base.Data);  % N x dim (typisch 3)
        if size(wBaseData,1) >= 1
            angSpeed = vecnorm(wBaseData,2,2);         % N x 1
            epBaseAngVelMean(k) = mean(angSpeed);
        else
            epBaseAngVelMean(k) = 0;
        end

        % ---------- 9) Mittlerer Energieverbrauch ----------
        % P(t) = sum_j tau_j * dq_j;  E = ∫ P dt

        dqData  = reshape_time_series(dq.Data);        % N x nJ
        tauData = reshape_time_series(tau.Data);       % N x nJ

        N = min([size(dqData,1), size(tauData,1), numel(t)]);
        dqUse  = dqData(1:N,:);
        tauUse = tauData(1:N,:);
        tUse   = t(1:N);

        if numel(tUse) >= 2
            P           = sum(abs(tauUse .* dqUse), 2);     % Gesamtleistung
            epEnergy(k) = trapz(tUse, P);              % Energie pro Episode
        else
            epEnergy(k) = 0;
        end

    end

    % ======= Aggregation zu skalaren KPIs =======
    kpi.K1 = mean(epReturn);           % Mean Episode Return
    kpi.K2 = mean(epMSE_EE);           % EE-Position MSE
    kpi.K3 = max(epMaxErr_EE);         % Max EE-Trackingfehler
    kpi.K4 = mean(epOriMean);          % Basis-Orientierungsfehler
    kpi.K5 = mean(epHasCollision);     % Kollisionsrate (0..1)
    kpi.K6 = mean(epJointViol);        % Gelenkgrenzen-Verletzungsrate
    kpi.K7 = 1 - mean(epJerk);             % Smoothness / Jerk
    kpi.K8 = mean(epBaseAngVelMean);   % mittlere Basiswinkelgeschwindigkeit
    kpi.K9 = mean(epEnergy);           % mittlerer Energieverbrauch
end



% =====================================================================
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