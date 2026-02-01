function [P_avg, I_rms, I_peak, ZVS_Metric] = calc_tps_performance(V1, V2, L, fs, n, D1, D2, D3)
    % =========================================================================
    % calc_tps_performance
    % TPS 제어 DAB 컨버터의 성능 지표 정밀 계산
    %
    % 입력:
    %   V1, V2     : 1차/2차 전압 (V)
    %   L, fs, n   : 등가 인덕턴스(H), 주파수(Hz), 권선비
    %   D1         : Global Phase Shift (-0.5 ~ 0.5)
    %   D2         : Pri Inner Shift (0 ~ 0.5, 0=Full Duty)
    %   D3         : Sec Inner Shift (0 ~ 0.5, 0=Full Duty)
    %
    % 출력:
    %   P_avg      : 평균 전력 (W)
    %   I_rms      : 1차측 전류 실효값 (A)
    %   I_peak     : 1차측 전류 피크값 (A)
    %   ZVS_Metric : ZVS 달성 여부 (1=Full ZVS, <1=Partial/Hard)
    %                (Soft Switching Margin을 간단히 평가)
    % =========================================================================

    %% 1. 스위칭 시점 정의 (수치해석용)
    % 시간 정규화 t_n = t * fs
    % Vp 스위칭: +V1 구간 [D2, 0.5], -V1 구간 [0.5+D2, 1.0]
    % Vs 스위칭: +nV2 구간 [D1+D3, D1+0.5], -nV2 구간 [D1+0.5+D3, D1+1.0] (modulo 1)
    
    % 주요 이벤트 포인트 수집
    events = [D2, 0.5, 0.5+D2];
    
    s_on  = mod(D1 + D3, 1);
    s_off = mod(D1 + 0.5, 1);
    s_on_n = mod(D1 + 0.5 + D3, 1);
    s_off_n = mod(D1 + 1.0, 1);
    
    events = [events, s_on, s_off, s_on_n, s_off_n];
    
    % 0과 1 포함, 중복 제거, 정렬
    t_points = unique(sort([0, 1, mod(events, 1)]));
    
    %% 2. 구간별 전압 및 전류 기울기 계산
    dt_vec = diff(t_points) * (1/fs); % 실제 시간 간격
    t_mids = t_points(1:end-1) + diff(t_points)/2;
    
    vp_vals = zeros(size(dt_vec));
    vs_vals = zeros(size(dt_vec));
    
    for k = 1:length(t_mids)
        tm = t_mids(k);
        
        % Primary Voltage Rule
        if (tm >= D2) && (tm < 0.5)
            vp = V1;
        elseif (tm >= 0.5+D2) && (tm < 1.0)
            vp = -V1;
        else
            vp = 0;
        end
        
        % Secondary Voltage Rule (Reflected)
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
    
    % 인덕터 전류 기울기 di/dt = (Vp - Vs)/L
    slopes = (vp_vals - vs_vals) / L;
    d_currents = slopes .* dt_vec;
    
    % 전류 프로파일 생성 (초기값 0 가정)
    i_raw = [0, cumsum(d_currents)];
    
    % DC Bias 제거 (Steady State 조건)
    % 평균 전류 = 0 이어야 함 (AC Link)
    i_avg_raw = 0;
    for k=1:length(dt_vec)
        seg_avg = (i_raw(k) + i_raw(k+1))/2;
        i_avg_raw = i_avg_raw + seg_avg * dt_vec(k);
    end
    i_offset = i_avg_raw / (sum(dt_vec)); % 즉 1/fs
    
    i_profile = i_raw - i_offset;
    
    %% 3. 성능 지표 계산
    
    % (1) P_avg: Average(Vp * i_L)
    p_sum = 0;
    
    % (2) I_rms^2: Integral(i^2 dt) / T
    i_sq_sum = 0;
    
    for k = 1:length(dt_vec)
        dt = dt_vec(k);
        i_k = i_profile(k);
        i_kp1 = i_profile(k+1);
        
        % Power Contribution
        % P_seg = Vp * I_avg_seg * dt
        v_seg = vp_vals(k);
        i_avg_seg = (i_k + i_kp1)/2;
        p_sum = p_sum + v_seg * i_avg_seg * dt;
        
        % RMS Contribution
        % Int( (a*t + b)^2 dt ) formula for linear segment
        % Simpson's rule or exact integration for line segment:
        % Int = dt/3 * (i_k^2 + i_k*i_kp1 + i_kp1^2)
        seg_sq_int = (dt/3) * (i_k^2 + i_k*i_kp1 + i_kp1^2);
        i_sq_sum = i_sq_sum + seg_sq_int;
    end
    
    P_avg = p_sum * fs;
    I_rms = sqrt(i_sq_sum * fs);
    I_peak = max(abs(i_profile));
    
    % (3) ZVS Metric (Soft Switching)
    % Primary Turning ON instants:
    % S1 (Leg A Top) turns ON at t=D2. Current should be NEGATIVE flowing out of node?
    % Convention: i_profile is current out of Pri Bridge into Inductor.
    % Leg A: Vp goes 0->V1 at t=D2. Current i_L should be < 0 for ZVS (discharge capacitance).
    % Leg B: Vp goes V1->0 at t=0.5 -> Hard/Soft?
    % Actually simpler: Check i_L sign at switching moments.
    %
    % SW1 (Rise edge of Vp): t=D2. Need i_L(D2) < 0.
    % SW2 (Fall edge of Vp): t=0.5. Need i_L(0.5) > 0.
    % SW3 (Neg Rise): t=0.5+D2. Need i_L(0.5+D2) > 0.
    % SW4 (Neg Fall): t=1.0. Need i_L(1.0) < 0.
    
    % Interpolate Current at specific times
    t_check = [D2, 0.5]; %, 0.5+D2, 1.0]; Symmetry makes checking half sufficient
    
    % Find indices closest to t_check (Exact match likely due to t_points construction)
    % Just use the calculated i_profile nodes.
    
    i_at_D2 = interp1(t_points, i_profile, D2, 'linear');
    i_at_05 = interp1(t_points, i_profile, 0.5, 'linear');
    
    is_zvs_1 = (i_at_D2 < 0); % Turn-on of diagonal pair
    is_zvs_2 = (i_at_05 > 0); % Turn-off of diagonal pair (Freewheeling)
    
    % Simple Score: 1.0 = All ZVS, 0.5 = Partial, 0.0 = Hard
    ZVS_Metric = (double(is_zvs_1) + double(is_zvs_2)) / 2;
    
end
