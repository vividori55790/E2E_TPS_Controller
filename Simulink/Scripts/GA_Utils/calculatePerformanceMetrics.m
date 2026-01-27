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
%   metrics: 계산된 성능 지표 구조체

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
        return;
    end
    
    % Step 시점 검출 (레퍼런스의 급격한 변화 지점)
    % 데이터가 잘렸으므로 index 재검색
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
        I_before_meas = mean(I_meas(idx_before));
        init_deviation = abs(I_before_meas - I_init_ref) / (STEP_SIZE + 1e-6) * 100;  % % (+1e-6 div by zero 방지)
        metrics.InitDeviation = init_deviation;
        
        % 스텝 전에 이미 목표 방향으로 20% 이상 이동해 있으면 페널티
        if init_deviation > 20
            metrics.CheatPenalty = init_deviation * 10;  % 강한 페널티
        else
            metrics.CheatPenalty = 0;
        end
    else
        metrics.InitDeviation = 0;
        metrics.CheatPenalty = 0;
    end
    
    % 스텝 이후 데이터 추출
    t_after = t(idx_after) - t_step;  % 스텝 시점을 0으로 정규화
    I_after = I_meas(idx_after);
    I_ref_after = I_ref(idx_after);
    
    if isempty(t_after) || length(t_after) < 5
        % 데이터 부족 시 높은 페널티
        metrics.ISE = 1e6;
        metrics.ITAE = 1e6;
        metrics.IAE = 1e6;
        metrics.Overshoot = 100;
        metrics.Undershoot = 100;
        metrics.RiseTime = 1;
        metrics.SettlingTime = 1;
        metrics.PeakTime = 0;
        metrics.PeakValue = 0;
        metrics.SteadyStateError = 100;
        metrics.RippleStd = 0;
        metrics.SaturationRatio = 0;
        metrics.VoltageRobustness = 0;
        metrics.ControlEffort = 0;
        return;
    end
    
    % ============================================================
    % [핵심] 스텝 이후 구간만으로 성능 지표 계산
    % ============================================================
    error_after = I_ref_after - I_after;
    norm_factor = max(STEP_SIZE, 0.1); % 0으로 나누기 방지
    
    % 1. ISE (스텝 이후만, 스텝 크기로 정규화)
    metrics.ISE = trapz(t_after, (error_after / norm_factor).^2);
    
    % 2. ITAE (스텝 이후만, 시간은 스텝 시점 기준)
    metrics.ITAE = trapz(t_after, t_after .* abs(error_after / norm_factor));
    
    % 3. IAE (스텝 이후만)
    metrics.IAE = trapz(t_after, abs(error_after / norm_factor));
    
    % ============================================================
    % 4. Overshoot (스텝 크기 기준 백분율)
    % ============================================================
    metrics.Overshoot = 0;
    metrics.PeakValue = 0;
    metrics.PeakTime = 0;
    
    target = I_final_ref;
    
    if STEP_SIZE > 0.1
        [peak, peakIdx] = max(I_after);
        % 양수 방향 스텝일 때 Max, 음수 방향 스텝일 때 Min 고려해야 함
        % 여기서는 범용성을 위해 Abs Error 대신 부호 고려
        if I_final_ref > I_init_ref % Positive Step
            if peak > target
                 metrics.Overshoot = (peak - target) / STEP_SIZE * 100;
            end
            metrics.PeakValue = peak;
        else % Negative Step
             [minVal, minIdx] = min(I_after);
             peak = minVal;
             peakIdx = minIdx;
             if minVal < target
                 metrics.Overshoot = (target - minVal) / STEP_SIZE * 100;
             end
             metrics.PeakValue = minVal;
        end
        metrics.PeakTime = t_after(peakIdx) + t_step;  % 절대 시간
    end
    
    % ============================================================
    % 5. Undershoot
    % ============================================================
    metrics.Undershoot = 0;
    if STEP_SIZE > 0.1
        if I_final_ref > I_init_ref % Positive Step
            minVal = min(I_after);
            if minVal < I_init_ref
                metrics.Undershoot = (I_init_ref - minVal) / STEP_SIZE * 100;
            end
        else % Negative Step
            maxVal = max(I_after);
            if maxVal > I_init_ref
                metrics.Undershoot = (maxVal - I_init_ref) / STEP_SIZE * 100;
            end
        end
    end
    
    % ============================================================
    % 6. Rise Time (10%→90%, 스텝 시점 이후부터 계산)
    % ============================================================
    metrics.RiseTime = 0;
    if STEP_SIZE > 0.1
        val10 = I_init_ref + 0.1 * (I_final_ref - I_init_ref);
        val90 = I_init_ref + 0.9 * (I_final_ref - I_init_ref);
        
        if I_final_ref > I_init_ref
            idx10 = find(I_after >= val10, 1, 'first');
            idx90 = find(I_after >= val90, 1, 'first');
        else
            idx10 = find(I_after <= val10, 1, 'first');
            idx90 = find(I_after <= val90, 1, 'first');
        end
        
        if ~isempty(idx10) && ~isempty(idx90) 
            metrics.RiseTime = abs(t_after(idx90) - t_after(idx10));
        else
            metrics.RiseTime = max(t_after);
        end
    end
    
    % ============================================================
    % 7. Settling Time (2% band, 스텝 크기 기준)
    % ============================================================
    metrics.SettlingTime = 0;
    if STEP_SIZE > 0.1
        tol = 0.02 * STEP_SIZE;
        within = abs(I_after - target) <= tol;
        
        if ~all(within)
            last_out = find(~within, 1, 'last');
            if ~isempty(last_out) && last_out < length(t_after)
                metrics.SettlingTime = t_after(last_out+1);
            else
                metrics.SettlingTime = max(t_after);
            end
        end
    end
    
    % ============================================================
    % 8. Steady-State Error (마지막 20% 데이터 평균)
    % ============================================================
    metrics.SteadyStateError = 0;
    n_after = length(I_after);
    n_ss = max(1, round(n_after * 0.2));
    I_ss = mean(I_after(end-n_ss+1:end));
    
    if STEP_SIZE > 0.1
        metrics.SteadyStateError = abs(target - I_ss) / STEP_SIZE * 100;
    end
    
    % ============================================================
    % [꼼수 방지 2] 응답이 비정상적으로 빠른 경우 검증
    % ============================================================
    if metrics.RiseTime > 0 && metrics.RiseTime < 0.0001
        metrics.CheatPenalty = metrics.CheatPenalty + 100;
    end
    
    % ============================================================
    % 9. Control Effort (위상 변화량 RMS)
    % ============================================================
    if ~isempty(phi)
        % 여기서는 전체 phi가 아니라 해당 구간의 phi만 봐야할 수도 있음
        % phi는 이미 valid_mask 처리가 되었으므로 idx_after 구간만 봄
        phi_after = phi(idx_after);
        dPhi = diff(phi_after);
        metrics.ControlEffort = sum(abs(dPhi));
    else
        metrics.ControlEffort = 0;
    end
    
    % ============================================================
    % 10. Current Ripple (표준편차)
    % ============================================================
    ref_diff = [0; diff(I_ref)];  % I_ref 전체에 대한 미분
    steady_mask = abs(ref_diff) < 1e-3; % 전체 구간 마스크
    
    % [FIX] Array Size Mismatch Error 수정
    % error_after는 idx_after 구간이므로, steady_mask도 해당 구간만 추출
    steady_mask_after = steady_mask(idx_after);
    
    if any(steady_mask_after)
        % t_after > 0.005 조건을 걸어 스텝 직후 과도응답 제외
        ripple_mask = steady_mask_after & (t_after > 0.005);
        if any(ripple_mask)
            ripple_err = error_after(ripple_mask);
            metrics.RippleStd = std(ripple_err);
        else
            metrics.RippleStd = 0;
        end
    else
        metrics.RippleStd = 0;
    end
    
    % ============================================================
    % 11. Saturation Ratio (위상 포화)
    % ============================================================
    if ~isempty(phi)
        sat_mask = abs(phi) >= 89.0;
        metrics.SaturationRatio = sum(sat_mask) / length(phi) * 100; 
    else
        metrics.SaturationRatio = 0;
    end

    % ============================================================
    % 12. Voltage Robustness (전압 변동 시 전류 오차)
    % ============================================================
    metrics.VoltageRobustness = 0;
    if ~isempty(v_high) && length(v_high) >= length(error_after)
        % v_high, i_ind는 이미 valid_mask 처리됨.
        % 여기서 error_after에 매칭되는 idx_after 구간 추출 필요
        v_high_after = v_high(idx_after);
        
        dV = [0; diff(v_high_after)];
        transient_v = abs(dV) > 0.1;
        
        if any(transient_v)
            % 차원이 맞는지 확인 (error_after와 transient_v)
            minLength = min(length(error_after), length(transient_v));
            metrics.VoltageRobustness = sqrt(mean(error_after(1:minLength) .* transient_v(1:minLength)).^2); 
            % (위 식은 논리적으로 약간 이상함. transient_v가 1인 구간의 error mean square를 구해야 함)
            
            err_v_transient = error_after(transient_v(1:length(error_after)));
            if ~isempty(err_v_transient)
                metrics.VoltageRobustness = sqrt(mean(err_v_transient.^2));
            end
        end
    end
end
