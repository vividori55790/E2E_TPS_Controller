function params = config_dab()
    % --- [Hardware Spec] ---
    params.Vin_nom = 380;       % V
    params.Vout_nom = 96;      % V (Low Voltage Side Nominal)
    params.Vout_min = 70;      % V (Discharged Battery)
    params.Vout_max = 110;     % V (Fully Charged)
    params.n = 400/100;         % Turns Ratio
    params.L_nom = 90e-6;       % Nominal Inductance [H]
    params.Coss = 500e-12;      % MOSFET Output Capacitance [F] (Fixed unit error: uF -> pF)
    params.fs = 80e3;          % Switching Freq [Hz]
    params.P_rated = 1000;      % Rated Power Limit [W]
    
    % --- [Loss Parameters] ---
    params.Ron_pri = 15e-3;     % Primary Switch R_on
    params.Ron_sec = 15e-3;     % Secondary Switch R_on
    params.Rw = 50e-3;          % Winding Resistance
    
    % --- [Constraints & Safety] ---
    params.I_max = 50;          % Peak Current Limit [A]
    params.I_zvs_min = 2.0;     % Min current for ZVS [A]
    params.Deadtime = 200e-9;   % Deadtime [s]
    
    % --- [Robust Optimization: Aging Scenario] ---
    % 소자 노후화 시나리오 정의 (가중치 계산용)
    params.L_tol = 0.15;        % Inductance tolerance (±15%)
    params.R_aging = 0.30;      % Resistance increase (+30%)
    
    % --- [Solver Settings] ---
    params.w_cond = 1.0;        % Weight for Conduction Loss
    params.w_sw = 5.0;          % Weight for Switching Loss (ZVS 실패 시 치명적)
    params.w_robust = 0.3;      % Robustness Weight (노후화 고려 비중)
end
