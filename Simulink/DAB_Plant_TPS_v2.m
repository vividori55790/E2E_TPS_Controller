function [V_out_next, I_in_avg, I_leak_avg, P_transfer] = DAB_Plant_TPS_v2(V_in, V_out_prev, phi_ext_deg, Phi_int_deg, Load, V_batt, dt, params)
    % ... (comments) ...
    
    %% 1. 파라미터 추출
    n = params.n;               % 변압기 권선비 (N1/N2)
    L = params.L;               % 누설 인덕턴스 (H)
    fs = params.fs;             % 스위칭 주파수 (Hz)
    C_out = params.C_out;       % 출력 커패시터 (F)
    efficiency = params.efficiency; % 효율 (0~1)
    
    Ts = 1/fs;
    
    %% 2. 전력 전달 계산 (SPS 공식)
    % DAB 컨버터의 SPS 전력 전달 공식:
    % P = (V1 * V2') / (2*pi*fs*L) * phi * (1 - |phi|/pi)
    % 여기서 phi는 라디안, V2' = n*V2, |phi| <= pi/2
    
    % 2차측 전압을 1차측으로 환산
    V2_prime = n * V_out_prev;
    
    % 위상을 라디안으로 변환 (±90도 -> ±pi/2)
    phi_rad = phi_ext_deg * pi / 180;
    
    % SPS 전력 공식
    K = (V_in * V2_prime) / (2 * pi * fs * L);
    P_transfer = K * phi_rad * (1 - abs(phi_rad) / pi);
    
    % 효율 반영 (양방향에 따라 다르게)
    if P_transfer >= 0
        % 순방향: HV -> LV (충전)
        P_out = P_transfer * efficiency;
    else
        % 역방향: LV -> HV (방전)
        P_out = P_transfer / efficiency;
    end
    
    %% 3. 2차측(LV) 전력 균형 (Battery Model)
    % 기존 Resistive Load 대신 Battery Back-EMF 모델 적용
    % I_load는 컨버터 출력단에서 배터리로 흘러들어가는 전류
    % V_out_prev > V_batt 이면 충전(+), 반대면 방전(-)
    
    I_load = (V_out_prev - V_batt) / Load; % Load는 여기서 내부저항+선로저항 역할
    P_load = V_out_prev * I_load;
    
    % 커패시터 충방전 전력 (P_out은 DAB에서 들어오는 전력)
    P_cap = P_out - P_load;
    
    %% 4. 출력 전압 동역학
    % C * dV/dt = I_cap = P_cap / V
    % dV/dt = P_cap / (C * V)
    
    if abs(V_out_prev) < 1e-3
        dV_dt = 0;
    else
        dV_dt = P_cap / (C_out * V_out_prev);
    end
    
    V_out_next = V_out_prev + dV_dt * dt;
    V_out_next = max(0.1, V_out_next);  % 최소 0.1V (안정성)
    
    %% 5. 입력 전류 계산
    if abs(V_in) < 1e-3
        I_in_avg = 0;
    else
        I_in_avg = P_transfer / V_in;
    end
    
    %% 6. 인덕터 평균 전류 (AC link, 이상적으로 0)
    I_leak_avg = 0;
    
end
