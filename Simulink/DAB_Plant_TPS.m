function [V_out_next, I_in_avg, I_leak_avg] = DAB_Plant_TPS(V_in, V_out_prev, D1, D2, D3, Load, dt, params)
    % =========================================================================
    % DAB_Plant_TPS - TPS 제어 DAB 컨버터 Average Model (초고속 MIL용)
    % =========================================================================
    % 입력:
    %   V_in        : 1차측 입력 전압 (V)
    %   V_out_prev  : 이전 시간 출력 전압 (V) - 동적 계산용
    %   D1          : Phase Shift 1 (Global Primary-Secondary Shift, -0.5 ~ 0.5)
    %   D2          : Phase Shift 2 (Primary Inner Shift, 0 ~ 0.45)
    %   D3          : Phase Shift 3 (Secondary Inner Shift, 0 ~ 0.45)
    %   Load        : 부하 저항 (Ohm)
    %   dt          : 시뮬레이션 타임스텝 (sec)
    %   params      : 구조체 {n, L, fs, C_out, efficiency}
    %
    % 출력:
    %   V_out_next  : 다음 시간 출력 전압 (V)
    %   I_in_avg    : 평균 입력 전류 (A)
    %   I_leak_avg  : 누설 인덕터 평균 전류 (A) - 리플 무시, 보통 0
    %
    % 수식 기반: TPS Power Transfer Function (Average Model)
    %   P = (V1 * V2') / (8*fs*L) * f(D1, D2, D3)
    %   여기서 f(D1, D2, D3)는 TPS 위상에 따른 전력 전달 함수
    % =========================================================================
    
    %% 1. 파라미터 추출
    n = params.n;               % 변압기 권선비
    L = params.L;               % 누설 인덕턴스 (H)
    fs = params.fs;             % 스위칭 주파수 (Hz)
    C_out = params.C_out;       % 출력 커패시터 (F)
    efficiency = params.efficiency; % 손실 계수 (0.0 ~ 1.0, 보통 0.94~0.97)
    
    %% 2. 전력 전달 계산 (TPS Average Model)
    % V2'는 2차측 전압을 1차측으로 Referred한 값
    V2_prime = n * V_out_prev;
    
    % TPS 전력 전달 함수 계산 (정밀 수치 계산)
    P_transfer = calc_tps_power_numeric(V_in, V_out_prev, L, fs, n, D1, D2, D3);
    
    % 효율 반영 (손실 모델링)
    P_out = P_transfer * efficiency;
    
    %% 3. 출력 전류 계산 (부하로 흐르는 전류)
    I_load = V_out_prev / Load;
    
    %% 4. 출력 커패시터 충방전
    % 에너지 균형: P_out = P_load + P_capacitor
    % P_capacitor = C * V * (dV/dt)
    % dV/dt = (P_out - P_load) / (C * V_out)
    
    P_load = V_out_prev * I_load;
    P_cap = P_out - P_load;
    
    % 전압 변화율 계산
    if abs(V_out_prev) < 1e-3
        dV_dt = 0; % Zero Division 방지
    else
        dV_dt = P_cap / (C_out * V_out_prev);
    end
    
    % 다음 출력 전압 (Euler Integration)
    V_out_next = V_out_prev + dV_dt * dt;
    
    % 최소값 제한 (음수 방지)
    V_out_next = max(0, V_out_next);
    
    %% 5. 입력 전류 계산
    % 입력 전력: P_in = P_transfer (손실 전)
    % I_in_avg = P_in / V_in
    if abs(V_in) < 1e-3
        I_in_avg = 0;
    else
        I_in_avg = P_transfer / V_in;
    end
    
    %% 6. 누설 인덕터 전류 (Average Model에서는 DC 성분만 고려, 리플 무시)
    % 이상적인 DAB는 AC Link이므로 평균 전류는 0
    I_leak_avg = 0;
    
end


%% =========================================================================
%% Helper Function: TPS Power Calculation (정밀 수치 계산)
%% =========================================================================
function P_avg = calc_tps_power_numeric(V1, V2, L, fs, n, D1, D2, D3)
    % TPS 전력 전달 정밀 계산 (TPS_LUT.m의 calc_tps_power_numeric과 동일)
    % 1. 스위칭 이벤트 정의 (Normalized Time [0, 1])
    
    % Primary Voltage Switching (D2에 의한 내부 위상 변조)
    % +V1: [D2, 0.5]
    % -V1: [0.5+D2, 1.0]
    
    % Secondary Voltage Switching (D3에 의한 내부 위상 변조, D1만큼 Global Shift)
    % +V2': [D1+D3, D1+0.5] (mod 1)
    % -V2': [D1+0.5+D3, D1+1.0] (mod 1)
    
    % 2. 모든 스위칭 포인트 수집
    t_points = [0, D2, 0.5, 0.5+D2];
    
    s_start = mod(D1 + D3, 1);
    s_end   = mod(D1 + 0.5, 1);
    s_start_n = mod(D1 + 0.5 + D3, 1);
    s_end_n   = mod(D1 + 1.0, 1);
    
    t_points = [t_points, s_start, s_end, s_start_n, s_end_n];
    t_points = unique(sort(mod(t_points, 1)));
    
    if t_points(1) ~= 0, t_points = [0, t_points]; end
    if t_points(end) ~= 1, t_points = [t_points, 1]; end
    
    % 3. 각 구간별 전압 계산
    dt_vec = diff(t_points) * (1/fs);
    t_mids = t_points(1:end-1) + diff(t_points)/2;
    
    vp_vals = zeros(size(t_mids));
    vs_vals = zeros(size(t_mids));
    
    for k = 1:length(t_mids)
        tm = t_mids(k);
        
        % Primary Voltage
        if (tm >= D2) && (tm < 0.5)
            vp = V1;
        elseif (tm >= 0.5+D2) && (tm < 1.0)
            vp = -V1;
        else
            vp = 0;
        end
        
        % Secondary Voltage (Referred)
        tm_s = mod(tm - D1, 1);
        if (tm_s >= D3) && (tm_s < 0.5)
            vs = n * V2;
        elseif (tm_s >= 0.5+D3) && (tm_s < 1.0)
            vs = -n * V2;
        else
            vs = 0;
        end
        
        vp_vals(k) = vp;
        vs_vals(k) = vs;
    end
    
    % 4. 인덕터 전류 프로파일 계산
    slopes = (vp_vals - vs_vals) / L;
    d_currents = slopes .* dt_vec;
    i_profile = [0, cumsum(d_currents)];
    
    % DC 바이어스 제거 (AC coupling)
    area = 0;
    for k = 1:length(dt_vec)
        avg_h = (i_profile(k) + i_profile(k+1)) / 2;
        area = area + avg_h * dt_vec(k);
    end
    i_dc_bias = area / (1/fs);
    i_profile_corrected = i_profile - i_dc_bias;
    
    % 5. 평균 전력 계산
    % P = Average(V1 * i_L)
    p_sum = 0;
    for k = 1:length(dt_vec)
        v_seg = vp_vals(k);
        if v_seg == 0
            continue;
        end
        i_s = i_profile_corrected(k);
        i_e = i_profile_corrected(k+1);
        i_avg_seg = (i_s + i_e) / 2;
        p_sum = p_sum + v_seg * i_avg_seg * dt_vec(k);
    end
    
    P_avg = p_sum * fs;
end
