function metrics = calculatePerformanceMetrics(t, I_meas, I_ref, stepTime, phi, v_high, i_ind)
% calculatePerformanceMetrics: 시뮬레이션 결과로부터 상세 성능 지표 계산
%
% [Inputs]
%   t: 시간 벡터
%   I_meas: 측정 전류
%   I_ref: 지령 전류
%   stepTime: 스텝 발생 시각 (사용하지 않을 수도 있음, 레퍼런스로 자동 감지)
%   phi: 위상 제어 입력
%   v_high: 고전압측 전압 (옵션)
%   i_ind: 인덕터 전류 (옵션)
%
% [Outputs]
%   metrics: 계산된 성능 지표 구조체 (Updated: Phase Analysis & Smoothed Current Tracking)

    metrics = struct();
    if nargin < 5, phi = zeros(size(I_meas)); end
    if nargin < 6, v_high = zeros(size(I_meas)); end
    if nargin < 7, i_ind = zeros(size(I_meas)); end
    
    % [REQ] 평가 구간: 시뮬레이션 0.002초 이후 데이터만 사용
    valid_mask = t >= 0.002;
    t = t(valid_mask);
    I_meas = I_meas(valid_mask);
    I_ref = I_ref(valid_mask);
    if ~isempty(phi) && length(phi) >= length(valid_mask)
        phi = phi(valid_mask);
    end
    
    if nargin < 6 || isempty(v_high), v_high = zeros(size(t)); else, v_high = v_high(valid_mask); end
    if nargin < 7 || isempty(i_ind), i_ind = zeros(size(t)); else, i_ind = i_ind(valid_mask); end
    
    if isempty(t)
        % 데이터가 너무 짧음
        metrics.ISE=1e6; metrics.ITAE=1e6; metrics.IAE=1e6;
        metrics.Overshoot=100; metrics.InitValue=0; metrics.TargetValue=0; metrics.StepSize=0;
        metrics.SteadyStateError=100; metrics.RiseTime=0; metrics.SettlingTime=0; metrics.PeakTime=0; metrics.PeakValue=0;
        metrics.RippleStd=0; metrics.SaturationRatio=0; metrics.VoltageRobustness=0; metrics.ControlEffort=0;
        metrics.PhaseOvershoot=0; metrics.PhaseSettlingTime=0; metrics.PhaseOscillation=0;
        return;
    end
    
    % [REQ] Current Evaluation: "평균값이 지령치를 잘 따라가는지"
    % 심한 리플을 무시하기 위해 Moving Average 적용
    % 윈도우 크기는 데이터 포인트 수의 약 1% 정도로 설정 (가변 스텝 고려하여 적절히 선택)
    if length(I_meas) > 100
        windowSize = floor(length(I_meas) / 100); 
    else
        windowSize = 1;
    end
    if windowSize < 1, windowSize = 1; end
    
    I_meas_smooth = movmean(I_meas, windowSize);
    
    % Step 시점 검출 (레퍼런스의 급격한 변화 지점)
    [~, rel_idx] = max(abs(diff(I_ref)));
    t_step = t(rel_idx);
    
    % 스텝 이전과 이후 구간 분리
    idx_before = t < t_step;
    idx_after = t >= t_step;
    
    % 스텝 전후의 레퍼런스 값 (초기값과 최종값)
    if any(idx_before)
        I_init_ref = mean(I_ref(idx_before));    % 초기 레퍼런스
    else
        I_init_ref = I_ref(1); % 데이터가 없으면 첫 값
    end
    I_final_ref = mean(I_ref(idx_after));    % 최종 레퍼런스
    STEP_SIZE = abs(I_final_ref - I_init_ref);  % 스텝 크기
    
    metrics.InitValue = I_init_ref;
    metrics.TargetValue = I_final_ref;
    metrics.StepSize = STEP_SIZE;
    
    % ============================================================
    % [꼼수 방지 1] 스텝 직전 측정값 검증 - 미리 올라가 있으면 안됨
    % ============================================================
    if any(idx_before)
        I_before_meas = mean(I_meas_smooth(idx_before));
        init_deviation = abs(I_before_meas - I_init_ref) / (STEP_SIZE + 1e-6) * 100;
        metrics.InitDeviation = init_deviation;
        
        if init_deviation > 20
            metrics.CheatPenalty = init_deviation * 10;
        else
            metrics.CheatPenalty = 0;
        end
    else
        metrics.InitDeviation = 0;
        metrics.CheatPenalty = 0;
    end
    
    % 스텝 이후 데이터 추출
    t_after = t(idx_after) - t_step;  % 스텝 시점을 0으로 정규화
    I_after_smooth = I_meas_smooth(idx_after);
    I_after_raw = I_meas(idx_after);
    I_ref_after = I_ref(idx_after);
    
    if isempty(t_after) || length(t_after) < 5
        metrics.ISE = 1e6; metrics.ITAE = 1e6; metrics.IAE = 1e6;
        metrics.Overshoot = 100; metrics.Undershoot = 100;
        metrics.RiseTime = 1; metrics.SettlingTime = 1;
        metrics.SteadyStateError = 100;
        metrics.RippleStd = 0; metrics.SaturationRatio = 0; metrics.VoltageRobustness = 0; metrics.ControlEffort = 0;
        metrics.PhaseOvershoot=0; metrics.PhaseSettlingTime=0; metrics.PhaseOscillation=0;
        return;
    end
    
    % ============================================================
    % [핵심 1] Current Evaluation (Smoothed Mean Value Tracking)
    % ============================================================
    error_after = I_ref_after - I_after_smooth; % 오차 계산 시 Smoothed 사용
    norm_factor = max(STEP_SIZE, 0.1); 
    
    % 1. ISE, ITAE, IAE (Smoothed Error)
    metrics.ISE = trapz(t_after, (error_after / norm_factor).^2);
    metrics.ITAE = trapz(t_after, t_after .* abs(error_after / norm_factor));
    metrics.IAE = trapz(t_after, abs(error_after / norm_factor));
    
    % ============================================================
    % [핵심 2] Phase Evaluation (Stability & Dynamics)
    % ============================================================
    if ~isempty(phi)
        phi_after = phi(idx_after);
        
        % Steady State (Final 20%)
        n_p = length(phi_after);
        n_ss_p = max(1, round(n_p * 0.2));
        phi_ss_val = mean(phi_after(end-n_ss_p+1:end));
        
        % Phase Oscillation (Convergence Check)
        metrics.PhaseOscillation = std(phi_after(end-n_ss_p+1:end));
        
        % Phase Overshoot & Settling Time
        phi_start = phi_after(1);
        phi_step_mag = abs(phi_ss_val - phi_start);
        
        metrics.PhaseOvershoot = 0;
        metrics.PhaseSettlingTime = 0;
        
        if phi_step_mag > 1e-3 % 위상이 의미있게 변했을 때만 평가
            % Overshoot
            if phi_ss_val > phi_start
                phi_peak = max(phi_after);
                if phi_peak > phi_ss_val
                    metrics.PhaseOvershoot = (phi_peak - phi_ss_val) / phi_step_mag * 100;
                end
            else
                phi_peak = min(phi_after);
                if phi_peak < phi_ss_val
                    metrics.PhaseOvershoot = (phi_ss_val - phi_peak) / phi_step_mag * 100;
                end
            end
            
            % Settling Time (5% Band of Phase Step Magnitude)
            band = 0.05 * phi_step_mag;
            within_band = abs(phi_after - phi_ss_val) <= band;
            
            if ~all(within_band)
                last_out_idx = find(~within_band, 1, 'last');
                if ~isempty(last_out_idx) && last_out_idx < length(t_after)
                    metrics.PhaseSettlingTime = t_after(last_out_idx+1);
                else
                    metrics.PhaseSettlingTime = max(t_after);
                end
            end
        end
    else
        metrics.PhaseOvershoot = 0;
        metrics.PhaseSettlingTime = 0;
        metrics.PhaseOscillation = 0;
    end
    
    % ============================================================
    % Conventional Metrics (Based on Smoothed Current)
    % ============================================================
    
    metrics.Overshoot = 0;
    metrics.PeakValue = 0;
    metrics.PeakTime = 0;
    
    target = I_final_ref;
    
    if STEP_SIZE > 0.1
        [peak, peakIdx] = max(I_after_smooth);
        
        if I_final_ref > I_init_ref
             if peak > target
                  metrics.Overshoot = (peak - target) / STEP_SIZE * 100;
             end
             metrics.PeakValue = peak;
        else 
             [minVal, minIdx] = min(I_after_smooth);
             peak = minVal;
             peakIdx = minIdx;
             if minVal < target
                 metrics.Overshoot = (target - minVal) / STEP_SIZE * 100;
             end
             metrics.PeakValue = minVal;
        end
        metrics.PeakTime = t_after(peakIdx) + t_step;
    end
    
    % Undershoot
    metrics.Undershoot = 0;
    if STEP_SIZE > 0.1
        if I_final_ref > I_init_ref
            minVal = min(I_after_smooth);
            if minVal < I_init_ref
                metrics.Undershoot = (I_init_ref - minVal) / STEP_SIZE * 100;
            end
        else
            maxVal = max(I_after_smooth);
            if maxVal > I_init_ref
                metrics.Undershoot = (maxVal - I_init_ref) / STEP_SIZE * 100;
            end
        end
    end
    
    % Rise Time
    metrics.RiseTime = 0;
    if STEP_SIZE > 0.1
        val10 = I_init_ref + 0.1 * (I_final_ref - I_init_ref);
        val90 = I_init_ref + 0.9 * (I_final_ref - I_init_ref);
        
        if I_final_ref > I_init_ref
            idx10 = find(I_after_smooth >= val10, 1, 'first');
            idx90 = find(I_after_smooth >= val90, 1, 'first');
        else
            idx10 = find(I_after_smooth <= val10, 1, 'first');
            idx90 = find(I_after_smooth <= val90, 1, 'first');
        end
        
        if ~isempty(idx10) && ~isempty(idx90) 
            metrics.RiseTime = abs(t_after(idx90) - t_after(idx10));
        else
            metrics.RiseTime = max(t_after);
        end
    end
    
    % Settling Time (Current - Smoothed, 2%)
    metrics.SettlingTime = 0;
    if STEP_SIZE > 0.1
        tol = 0.02 * STEP_SIZE;
        within = abs(I_after_smooth - target) <= tol;
        
        if ~all(within)
            last_out = find(~within, 1, 'last');
            if ~isempty(last_out) && last_out < length(t_after)
                metrics.SettlingTime = t_after(last_out+1);
            else
                metrics.SettlingTime = max(t_after);
            end
        end
    end
    
    % Steady-State Error (Smoothed)
    metrics.SteadyStateError = 0;
    n_after = length(I_after_smooth);
    n_ss = max(1, round(n_after * 0.2));
    I_ss = mean(I_after_smooth(end-n_ss+1:end));
    
    if STEP_SIZE > 0.1
        metrics.SteadyStateError = abs(target - I_ss) / STEP_SIZE * 100;
    end
    
    % [꼼수 방지 2] Too fast?
    if metrics.RiseTime > 0 && metrics.RiseTime < 0.0001
        metrics.CheatPenalty = metrics.CheatPenalty + 100;
    end
    
    % Control Effort (Phase Smoothness)
    if ~isempty(phi)
        phi_after = phi(idx_after);
        dPhi = diff(phi_after);
        % [MODIFIED] Normalize by length (Mean Absolute Change per step)
        % This makes the metric independent of simulation duration
        if ~isempty(dPhi)
            metrics.ControlEffort = mean(abs(dPhi)); 
        else
            metrics.ControlEffort = 0;
        end
    else
        metrics.ControlEffort = 0;
    end
    
    % Current Ripple (Relative to Smooth Mean)
    metrics.RippleStd = std(I_after_raw - I_after_smooth);
    
    % Saturation Ratio
    if ~isempty(phi)
        sat_mask = abs(phi) >= 89.0;
        metrics.SaturationRatio = sum(sat_mask) / length(phi) * 100; 
    else
        metrics.SaturationRatio = 0;
    end

    % Voltage Robustness
    metrics.VoltageRobustness = 0;
    if ~isempty(v_high) && length(v_high) >= length(error_after)
        v_high_after = v_high(idx_after);
        dV = [0; diff(v_high_after)];
        transient_v = abs(dV) > 0.1;
        
        if any(transient_v)
            minLength = min(length(error_after), length(transient_v));
            err_v_transient = error_after(transient_v(1:minLength));
            if ~isempty(err_v_transient)
                metrics.VoltageRobustness = sqrt(mean(err_v_transient.^2));
            end
        end
    end
end
