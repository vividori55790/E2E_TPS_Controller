function [Loss, I_peak, I_rms, ZVS_flag, P_avg, P_loss_real] = physics_engine(x, Vin, Vout, L_val, params)
    % x = [D1, D2, Phi] (Normalized)
    D1 = x(1); D2 = x(2); Phi = x(3);
    
    % 1. Time Vector Setup
    N_res = 1000; % Resolution increased for low-power accuracy 
    t = linspace(0, 1, N_res+1); 
    
    % 2. Voltage Waveform Generation
    % Primary Voltage (vp) - Center Aligned
    vp = zeros(size(t));
    width_p = D1; 
    vp(t < width_p/2 | t > 1-width_p/2) = 1;      
    vp(t >= 0.5-width_p/2 & t <= 0.5+width_p/2) = -1; 
    vp = vp * Vin;
    
    % Secondary Voltage (vs) - Phase Shifted
    vs = zeros(size(t));
    t_s = mod(t - Phi, 1); 
    width_s = D2;
    % Vout is now variable input
    vs_mag = Vout * params.n;
    vs(t_s < width_s/2 | t_s > 1-width_s/2) = vs_mag;
    vs(t_s >= 0.5-width_s/2 & t_s <= 0.5+width_s/2) = -vs_mag;
    
    % 3. Inductor Current (Normalized Integration)
    v_L = vp - vs;
    i_L = cumtrapz(t, v_L) / (L_val * params.fs);
    i_L = i_L - mean(i_L); 
    
    % 4. Feature Extraction
    I_rms = rms(i_L);
    I_peak = max(abs(i_L));
    P_avg = mean(vp .* i_L); % Input Power (Positive = Pri to Sec, Negative = Sec to Pri)
    
    % 5. Bidirectional ZVS Judgment
    % ZVS Condition depends on Power Flow Direction
    
    % Primary ZVS Check (Relevant for Forward Power > 0)
    i_sw_p = i_L(1); % Current at t=0 (approx center of pos pulse... wait, logic check)
    % Based on previous logic: Turn-On is at t = 1 - D1/2.
    % Let's use robust index finding for Turn-On instant.
    t_on_p = mod(1 - D1/2, 1);
    [~, idx_p] = min(abs(t - t_on_p));
    i_on_p = i_L(idx_p);
    
    % Secondary ZVS Check (Relevant for Reverse Power < 0)
    % Sec Voltage Turn-On is at t_s = 1 - D2/2 => t = Phi + 1 - D2/2
    t_on_s = mod(Phi + (1 - D2/2), 1);
    [~, idx_s] = min(abs(t - t_on_s));
    i_on_s = -i_L(idx_s) / params.n; % Secondary Current (refer to sec side)
    % Note: i_L is defined as Pri current. Sec current i_sec = i_L * n (ideal).
    % Direction: If i_L flows into dot, i_sec flows out of dot.
    % Let's assume standard definitions.
    
    E_req_p = 0.5 * (2*params.Coss) * Vin^2;
    E_stored_p = 0.5 * L_val * i_on_p^2;
    
    % ZVS Flag Logic
    if P_avg >= 0
        % Forward Mode: Check Primary ZVS (Leading Leg) implies lower loss?
        % For DAB, usually Lagging Leg loses ZVS first at unity gain.
        % Just check Primary for safety as requested before.
        % Condition: Current must be negative to discharge Coss (for N-ch MOSFET).
        ZVS_flag = (E_stored_p > E_req_p) && (i_on_p < -params.I_zvs_min);
    else
        % Reverse Mode: Check Secondary ZVS
        % Simplified logic: Just Assume Symmetric Loss for now or use same P_sw penalty
        % If Reverse, we are sending power Sec -> Pri.
        % We want ZVS on Secondary side (which is now driving).
        ZVS_flag = (abs(i_on_s) > params.I_zvs_min); % Simplified
    end

    % 6. Loss Calculation
    P_cond = I_rms^2 * (params.Ron_pri + params.Ron_sec/params.n^2 + params.Rw);
    
    if ZVS_flag
        P_sw = 0; 
    else
        % Hard Switching Penalty (always applied if not ZVS)
        P_sw = 0.5 * params.Coss * Vin^2 * params.fs * 2; 
    end
    
    P_loss_real = P_cond + P_sw;
    Loss = params.w_cond * P_cond + params.w_sw * P_sw;
end
