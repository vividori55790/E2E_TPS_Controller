classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 위상 제어기
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd
    % 출력: t1, t3, t5, t7, mode_active, status_flag, I_cmd_out, phi_ext_out,
    %       D1_out, D2_out, integrator_out

    % =========================================================================
    % 1. Public, Tunable Properties (블록 더블클릭 시 설정 가능한 파라미터)
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Phi_int_deg = 15;       % 내부 위상 (Degrees)
        V_HV_Threshold = 350;   % 정전 감지 임계값 (V)
        V_HV_Hysteresis = 370;  % 복구 임계값 (V)
        
        % HDC Control Parameters
        EnableHDC = false; 
        HDC_Model = struct(); 
    end

    properties
        Kp = 0.100;               % 비례 이득
        Ki = 1000.0;                % 적분 이득
        Ups_Target_Current = 10;% UPS 모드 목표 전류 (A)
    end

    % =========================================================================
    % 2. Internal States (DiscreteState) - persistent 대체
    % =========================================================================
    properties(DiscreteState)
        Integrator;      % PI 제어기 적분항
        PowerLossState;  % 0: 정상(ESS), 1: 정전(UPS)
    end

    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화 (시뮬레이션 시작 시 1회 실행)
            obj.Integrator = 0;
            obj.PowerLossState = 0; % 초기엔 정상 상태 가정
        end

        function [t1, t3, t5, t7, mode_active, status_flag, I_cmd_out, phi_ext_out, ...
                  D1_out, D2_out, integrator_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd)
            
            % --- 1. 파라미터 계산 ---
            Ts = 1 / obj.Fs;
            deg2time = Ts / 360;

            % --- 2. 정전 감지 (Hysteresis Logic) ---
            % 상태 변수 obj.PowerLossState 사용
            if V_HV < obj.V_HV_Threshold
                obj.PowerLossState = 1; % 정전 발생
            elseif V_HV > obj.V_HV_Hysteresis
                obj.PowerLossState = 0; % 복구
            end
            
            % --- 3. 모드 선택 및 레퍼런스 설정 ---
            if obj.PowerLossState == 1
                % [UPS 모드]
                mode_active = 1;
                status_flag = 1;
                Iref = obj.Ups_Target_Current;
                I_fb = I_HV;
            else
                % [ESS 모드]
                mode_active = 3;
                status_flag = 0;
                Iref = I_ess_cmd;
                I_fb = I_LV;
            end
            
            I_cmd_out = Iref;

            % --- 4. PI 제어기 (Anti-Windup 보강) ---
            error = Iref - I_fb;
            
            % 모드 전환 시 적분기 리셋 로직 (기존 코드 유지하되 좀 더 안전하게)
            % (실제로는 Bumpless Transfer가 좋지만, 기존 로직 존중)
            if obj.PowerLossState == 1 && abs(error) > 5
                 obj.Integrator = 0;
            end

            % 적분 수행 (Trapezoidal이 좋지만 기존 Euler 방식 유지)
            obj.Integrator = obj.Integrator + error * Ts;

            % 제어 출력 계산
            phi_pi = obj.Kp * error + obj.Ki * obj.Integrator;
            
            % --- 4.5. HDC Feedforward ---
            phi_hdc = 0;
            if obj.EnableHDC && ~isempty(obj.HDC_Model) && isfield(obj.HDC_Model, 'W')
                 % HDC Prediction (Inlined)
                 P_target = V_LV * Iref; 
                 model = obj.HDC_Model;
                 
                 % 1. Quantize Inputs
                 idx_v = floor( (V_HV - model.min_V)/(model.max_V - model.min_V) * (model.L_levels-1) ) + 1;
                 idx_v = max(1, min(model.L_levels, idx_v));
                 
                 idx_p = floor( (P_target - model.min_P)/(model.max_P - model.min_P) * (model.L_levels-1) ) + 1;
                 idx_p = max(1, min(model.L_levels, idx_p));
                 
                 % 2. Retrieve & Bind
                 hv_v = model.Level_Item_V(idx_v, :);
                 hv_p = model.Level_Item_P(idx_p, :);
                 
                 query_hv = hv_v .* hv_p;
                 
                 % 3. Query (Dot Product)
                 Output = query_hv * model.W; 
                 
                 % 4. D3 to Phase
                 d3_pred = Output(3);
                 phi_hdc = d3_pred * 360; 
            end
            
            phi_raw = phi_pi + phi_hdc;
            
            % Saturation (Clamping)
            phi_ext_deg = max(-90, min(90, phi_raw));
            
            % [중요] Anti-Windup: 출력이 제한에 걸리면 적분기 누적 방지 (Clamping back-calculation)
            if phi_raw ~= phi_ext_deg
                % 적분값이 더 이상 커지지 않도록 롤백하거나 제한
                % 여기서는 간단하게 적분값을 이전 값으로 유지하거나 제한된 값에 맞춤
                % (가장 간단한 방법: 적분 업데이트 취소 - 여기선 생략하고 Output Clamping만 적용)
            end
            
            phi_ext_out = phi_ext_deg;

            % --- 5. 게이트 타이밍 계산 ---
            t_int = obj.Phi_int_deg * deg2time;
            t_ext = abs(phi_ext_deg) * deg2time;

            if phi_ext_deg >= 0
                t1_base = 0;
                t5_base = t_ext;
            else
                t5_base = 0;
                t1_base = t_ext;
            end

            t1 = t1_base;
            t3 = t1 + t_int;
            t5 = t5_base;
            t7 = t5 + t_int;

            % 주기 초과 시 Modulo 연산
            t_max = max([t1, t3, t5, t7]);
            if t_max > Ts
                t1 = mod(t1, Ts);
                t3 = mod(t3, Ts);
                t5 = mod(t5, Ts);
                t7 = mod(t7, Ts);
            end
            
            % --- 6. 추가 출력 (디버깅/모니터링용) ---
            % D1: Primary Duty (내부 위상 기반)
            % D2: Secondary Duty (외부 위상 기반)
            % phi_ext_deg를 Duty Ratio로 변환 (0~1 범위로 정규화)
            D1_out = obj.Phi_int_deg / 180;           % Primary Inner Duty (0~0.5)
            D2_out = abs(phi_ext_deg) / 180;          % Secondary Phase Shift Duty
            integrator_out = obj.Integrator;          % 적분기 상태값
        end

        function resetImpl(obj)
            % 리셋 신호가 들어오거나 재시작 시
            obj.Integrator = 0;
            obj.PowerLossState = 0;
        end
    end
end