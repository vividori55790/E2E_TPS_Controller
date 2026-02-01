function [cost, details] = Calculate_DAB_Cost(simOut)
    % CALCULATE_DAB_COST DAB 컨버터 제어기 튜닝 비용 함수 (With Debug Logging)
    %
    % [To Workspace 블록 설정 가이드]
    % 모델의 'Control_Logic' 서브시스템 출력 포트들은 다음과 같은 물리량을 의미합니다.
    % 최적화 및 디버깅을 위해 아래 변수들이 Workspace로 출력되어야 합니다.
    %
    % 1. out.I_Fil (필수): [물리량] Battery Side Current (Filtered) [A]
    %    - 제어기가 실제로 보고 있는 피드백 전류입니다. (이동평균 필터 적용됨)
    %    - 리플이 제거된 DC 성분이므로 제어 목표 추종 성능 평가의 핵심입니다.
    %
    % 2. out.Phi_Raw (필수): [물리량] Control Phase Angle (Lag) [Degree]
    %    - 제어기에 의해 계산된 위상차(Primary - Secondary)입니다.
    %    - 이 값의 변화율(Derivative)을 통해 제어기의 안정성과 부드러움을 평가합니다.
    %
    % 3. out.Phi_Ext_Out (권장): [물리량] Final Applied Phase Angle [Degree]
    %    - 리미터(Saturation)와 Rate Limiter를 거친 최종 적용 위상입니다.
    %    - Phi_Raw와 비교하여 포화(Saturation)가 얼마나 일어나는지 알 수 있습니다.
    %
    % 4. out.Err_Amp (디버깅용): [물리량] Tracking Error [A]
    %    - (지령치 - 측정치) 값입니다. 제어기 내부 오차 신호 확인용입니다.
    %
    % 5. out.Excess (디버깅용): [물리량] Anti-Windup Excess Amount
    %    - 적분기 누적 방지(Anti-windup)가 얼마나 작동했는지 보여줍니다.
    
    cost = 1e6;
    details = struct('Track', 1e6, 'Effort', 0, 'SS', 0, 'Overshoot_Pct', 0, 'Msg', 'Init');
    
    %% 1. 데이터 추출 (Data Extraction)
    try
        % 1.1 전류 데이터 (I_Fil) - 제어 목표
        if isprop(simOut, 'I_Fil') 
            sig_I = simOut.I_Fil;
        elseif isprop(simOut, 'yout') && ~isempty(simOut.yout.find('I_Fil'))
            sig_I = simOut.yout.get('I_Fil');
        else
            details.Msg = 'I_Fil not found in simOut';
            return;
        end
        
        % 1.2 위상 데이터 (Phi_Raw) - 제어 노력 (없으면 Phi_Ext_Out 대체 가능)
        if isprop(simOut, 'Phi_Raw')
            sig_Phi = simOut.Phi_Raw;
        elseif isprop(simOut, 'Phi_Ext_Out')
            sig_Phi = simOut.Phi_Ext_Out; % 대체재
        elseif isprop(simOut, 'yout')
            if ~isempty(simOut.yout.find('Phi_Raw'))
               sig_Phi = simOut.yout.get('Phi_Raw');
            elseif ~isempty(simOut.yout.find('Phi_Ext_Out'))
               sig_Phi = simOut.yout.get('Phi_Ext_Out');
            else
               sig_Phi = [];
            end
        else
            sig_Phi = [];
        end
        
        % Timeseries to Array
        if isa(sig_I, 'timeseries')
            T = sig_I.Time; Y_meas = sig_I.Data;
        else
            T = sig_I.time; Y_meas = sig_I.signals.values;
        end
        
        if ~isempty(sig_Phi)
             if isa(sig_Phi, 'timeseries'), U_ctrl = sig_Phi.Data;
             else, U_ctrl = sig_Phi.signals.values; end
        else
             U_ctrl = zeros(size(Y_meas)); 
        end
        
        % [FIX] Force Column Vectors to prevent implicit expansion (N x N explosion)
        T = T(:);
        Y_meas = Y_meas(:);
        U_ctrl = U_ctrl(:);

        % 발산 체크
        if any(isnan(Y_meas))
            details.Msg = 'NaN detected in Output';
            return;
        end
        
        if max(abs(Y_meas)) > 500
            details.Msg = 'Output Diverged (>500A)';
            return;
        end

        %% 2. 평가 구간 설정 (Step Response Analysis)
        % 시나리오: 0.02초에 10A Step
        T_step = 0.02;     
        Target_Current = 10;
        
        idx = find(T >= T_step, 1);
        if isempty(idx)
             details.Msg = 'Simulation too short (<0.02s)';
             return; 
        end
        
        T_eval = T(idx:end);
        Y_eval = Y_meas(idx:end);
        U_eval = U_ctrl(idx:end);

        %% 3. [J1] 참조 모델 추종 (Reference Tracking)
        % 물리적으로 이상적인 1차 시스템(Tau=2.5ms)을 얼마나 잘 따라가는가?
        Tau = 0.0025; 
        Y_ref = Target_Current * (1 - exp(-(T_eval - T_step)/Tau));
        Loss_Track = mean(log(cosh(Y_eval - Y_ref)));

        %% 4. [J2] 제어 노력 (Control Effort)
        % 위상각이 얼마나 부드러운가? (채터링 방지)
        if length(U_eval) > 1
            Loss_Effort = mean(diff(U_eval).^2);
        else
            Loss_Effort = 0;
        end

        %% 5. [J3] 정상 상태 오차 (Steady State Acc)
        % 최종 구간(마지막 10%) 오차가 2% 이내인가?
        idx_ss = floor(length(Y_eval) * 0.9);
        SS_Err = abs(mean(Y_eval(idx_ss:end)) - Target_Current);
        
        if SS_Err > (Target_Current * 0.02)
            Loss_SS = SS_Err * 10;
        else
            Loss_SS = 0;
        end
        
        %% 6. [Out] Overshoot (Optional for logging)
        Overshoot = max(Y_eval) - Target_Current;
        if Overshoot < 0, Overshoot = 0; end
        Overshoot_Pct = (Overshoot / Target_Current) * 100;
        
        %% 7. 최종 Cost
        % 가중치: Track(1.0) + Effort(0.05) + SS(1.0)
        % 위상각 값(수십도) 변화율은 숫자가 크므로 0.05 가중치 부여
        cost = Loss_Track + (0.05 * Loss_Effort) + Loss_SS;
        
        %% 8. Details Struct
        details.Track = Loss_Track;
        details.Effort = Loss_Effort;
        details.SS = Loss_SS;
        details.Overshoot_Pct = Overshoot_Pct;
        details.Msg = 'Success';

    catch e
        details.Msg = sprintf('Metric Calc Error: %s', e.message);
        cost = 1e6;
    end
end
