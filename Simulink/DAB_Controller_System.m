classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 DPS 위상 제어기
    %
    % 개선사항 (v2.0):
    %   - UPS 모드: Cascade 제어 (전압 외부루프 + 전류 내부루프)
    %   - DPS: 전류 크기에 따른 가변 내부위상 (5~25°)
    %   - 모드 전환 시 적분기 리셋 개선
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd
    % 출력: t1, t3, t5, t7, mode_active, status_flag, I_cmd_out, phi_ext_out,
    %       D1_out, D2_out, integrator_out

    % =========================================================================
    % 1. Public, Tunable Properties (블록 더블클릭 시 설정 가능한 파라미터)
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        
        % HV 전압 목표 및 밴드 범위
        V_HV_Ref = 380;         % UPS 모드 HV 전압 목표 (V)
        V_HV_Band = 1;          % ±1V 범위 (379~381V가 정상)
        
        % DPS 내부위상 범위
        Psi_Min = 5;            % 최소 내부 위상 (경부하)
        Psi_Max = 25;           % 최대 내부 위상 (대전력)
        I_Base = 10;            % 내부위상 계산용 기준 전류 (A)
        
        % HDC Control Parameters
        EnableHDC = false; 
        HDC_Model = struct(); 
    end

    properties
        % 전류 PI 이득 (외부 위상용, 공통)
        Kp_ext = 2.0;           % 비례 이득
        Ki_ext = 30.0;          % 적분 이득
        
        % 전압 PI 이득 (UPS 모드 전용, Cascade 외부루프)
        Kpv = -4.0;             % 전압 비례 이득 (음수: V↓ → 방전)
        Kiv = -400.0;           % 전압 적분 이득
        
        % 전류 지령 제한
        I_ESS_Max = 10;         % ESS 모드 최대 전류 (±10 A)
        I_UPS_Max = 30;         % UPS 모드 최대 전류 (±30 A, 대부하 대응)
    end

    % =========================================================================
    % 2. Internal States (DiscreteState) - persistent 대체
    % =========================================================================
    properties(DiscreteState)
        Integrator_ext;     % 전류 PI 적분항
        Integrator_v;       % 전압 PI 적분항 (UPS 모드)
        PowerLossState;     % 0: 정상(ESS), 1: 정전(UPS)
        PreviousMode;       % 이전 모드 (전환 감지용)
    end

    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화 (시뮬레이션 시작 시 1회 실행)
            obj.Integrator_ext = 0;
            obj.Integrator_v = 0;
            obj.PowerLossState = 0; % 초기엔 정상 상태 가정
            obj.PreviousMode = 0;
        end

        function [t1, t3, t5, t7, mode_active, status_flag, I_cmd_out, phi_ext_out, ...
                  D1_out, D2_out, integrator_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd)
            
            % --- 1. 파라미터 계산 ---
            Ts = 1 / obj.Fs;
            deg2time = Ts / 360;

            % --- 2. 정전 감지 (밴드 기반) ---
            % 379~381 V : 정상 → ESS
            % 그 외 : 정전/이상 → UPS
            if (V_HV < (obj.V_HV_Ref - obj.V_HV_Band)) || (V_HV > (obj.V_HV_Ref + obj.V_HV_Band))
                obj.PowerLossState = 1; % UPS 모드
            else
                obj.PowerLossState = 0; % ESS 모드
            end
            
            % --- 3. 모드 전환 감지 → 적분기 리셋 ---
            mode_changed = (obj.PowerLossState ~= obj.PreviousMode);
            
            if mode_changed && obj.PowerLossState == 1
                % ESS → UPS 전환: 모든 적분기 리셋 (부드러운 전환)
                obj.Integrator_ext = 0;
                obj.Integrator_v = 0;
            elseif mode_changed && obj.PowerLossState == 0
                % UPS → ESS 전환: 전류 적분기만 리셋
                obj.Integrator_ext = 0;
            end
            
            % --- 4. 모드별 기준 전류/피드백 선택 ---
            if obj.PowerLossState == 0
                %---------------------------
                % ESS 모드 (정상 상태)
                %---------------------------
                mode_active = 3;
                status_flag = 0;

                % ESS: I_ess_cmd 그대로 사용 (±I_ESS_Max 제한)
                Iref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, I_ess_cmd));
                I_fb = I_LV; % 배터리 전류 기준 제어

            else
                %---------------------------
                % UPS 모드 (정전/이상, 부하 공급)
                %---------------------------
                mode_active = 1;
                status_flag = 1;

                % *** Cascade 제어: 전압 외부루프 → 전류 지령 생성 ***
                % HV 전압을 V_HV_Ref (380V)로 정확히 유지
                % V_HV < 379 → err_v > 0 → I_ups_cmd < 0 (강한 방전)
                % V_HV > 381 → err_v < 0 → I_ups_cmd > 0 (강한 충전)
                err_v = obj.V_HV_Ref - V_HV;
                obj.Integrator_v = obj.Integrator_v + err_v * Ts;
                
                % 전압 PI 출력 = 전류 지령
                I_ups_cmd = obj.Kpv * err_v + obj.Kiv * obj.Integrator_v;

                % 양방향 허용 (방전/충전 모두, ±I_UPS_Max)
                I_ups_cmd = max(-obj.I_UPS_Max, min(obj.I_UPS_Max, I_ups_cmd));

                Iref = I_ups_cmd;
                I_fb = I_LV; % LV 전류 기준으로 제어
            end
            
            I_cmd_out = Iref;

            % --- 5. 전류 PI → 외부 위상(phi_ext_deg) ---
            err_ext = Iref - I_fb;
            obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts;
            
            % 과도 상태에서 적분 폭주 방지 (UPS 모드에서 큰 에러 시)
            if (obj.PowerLossState == 1) && (abs(err_ext) > 10)
                obj.Integrator_ext = 0;
            end

            phi_pi = obj.Kp_ext * err_ext + obj.Ki_ext * obj.Integrator_ext;
            
            % --- 5.5. HDC Feedforward (기존 로직 유지) ---
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
            
            phi_ext_out = phi_ext_deg;

            % --- 6. DPS: 전류 크기에 따른 내부 위상 조절 ---
            % 내부위상 범위: Psi_Min ~ Psi_Max (기본 5~25도)
            % 전류가 작을수록 Psi_Min (순환전류 최소)
            % 전류가 클수록 Psi_Max (소프트스위칭, ZVS 확보)
            
            I_mag = abs(I_fb); % 실제 흐르는 전류 크기
            I_norm = min(1, I_mag / obj.I_Base); % 정규화 (0~1)

            % 선형 매핑: I_norm = 0 → phi_int = Psi_Max (25도) - 역방향
            %            I_norm = 1 → phi_int = Psi_Min (5도)
            % → 수정: 경부하에서 작게, 대부하에서 크게
            phi_int_deg = obj.Psi_Min + (obj.Psi_Max - obj.Psi_Min) * I_norm;

            % --- 7. 내부/외부 위상 조합 → 게이트 타이밍 ---
            t_int = phi_int_deg * deg2time;
            t_ext = abs(phi_ext_deg) * deg2time;

            % 외부 위상의 부호로 전력 방향 결정
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
            
            % --- 8. 추가 출력 (디버깅/모니터링용) ---
            % D1: Primary Duty (가변 내부 위상 기반)
            % D2: Secondary Duty (외부 위상 기반)
            D1_out = phi_int_deg / 180;           % Primary Inner Duty (가변, 0~0.14)
            D2_out = abs(phi_ext_deg) / 180;      % Secondary Phase Shift Duty
            integrator_out = obj.Integrator_ext;  % 전류 적분기 상태값
            
            % --- 9. 이전 모드 저장 (다음 스텝용) ---
            obj.PreviousMode = obj.PowerLossState;
        end

        function resetImpl(obj)
            % 리셋 신호가 들어오거나 재시작 시
            obj.Integrator_ext = 0;
            obj.Integrator_v = 0;
            obj.PowerLossState = 0;
            obj.PreviousMode = 0;
        end
    end
end