classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 SPS 위상 제어기
    %
    % 개선사항 (v3.0 - SPS Mode):
    %   - SPS (Single Phase Shift) 제어만 사용
    %   - 위상 출력을 360도 각도로 출력 (시간 대신)
    %   - D1, D2는 SPS 고정값 (50%)으로 출력
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv
    % 출력: phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, 
    %       I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out
    %
    % 출력 위상 설명 (모두 0~360도 범위):
    %   - phi1_deg: Primary Bridge Leg A 기준 위상 (항상 0도)
    %   - phi3_deg: Primary Bridge Leg B 위상 (SPS: 180도 고정)
    %   - phi5_deg: Secondary Bridge Leg A 위상 (외부 위상에 의해 결정)
    %   - phi7_deg: Secondary Bridge Leg B 위상 (SPS: phi5 + 180도)

    % =========================================================================
    % 1. Public, Tunable Properties (블록 더블클릭 시 설정 가능한 파라미터)
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Ts_ctrl = 1e-7;         % 제어기 샘플 시간 (초) - Simulink 솔버와 동일하게 설정
        
        % HV 전압 목표 및 밴드 범위
        V_HV_Ref = 380;         % UPS 모드 HV 전압 목표 (V)
        V_HV_Band = 30;         % ±30V 범위 (350~410V가 정상)
        
        % [SPS 모드] 내부 위상 고정값 (TPS/DPS 사용 안함)
        % D1, D2 = 0.5 (50%) 고정 → 180도 위상차
        SPS_Inner_Phase = 180;  % Primary/Secondary 내부 위상 (도)
        
        % [주석 처리됨] DPS 내부위상 범위 - TPS/DPS 제어 시 사용
        % Psi_Min = 5;            % 최소 내부 위상 (경부하)
        % Psi_Max = 25;           % 최대 내부 위상 (대전력)
        % I_Base = 10;            % 내부위상 계산용 기준 전류 (A)
        
        % [주석 처리됨] HDC Control Parameters
        % TODO: 사용자가 복구시키라고 할 때까지 주석으로 유지할 것
        % EnableHDC = false; 
        % HDC_Model = struct(); 
    end

    properties
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

        function [phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, ...
                  I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv)
            
            % --- 1. 파라미터 계산 ---
            Ts = obj.Ts_ctrl;  % 제어기 샘플 시간 사용 (적분기용)

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
                % V_HV < 350V (저전압) → err_v > 0 → I_ups_cmd > 0 (양수 전류)
                %   → 양수 위상 → LV→HV 전력 전달 (배터리 방전으로 HV 전압 올림)
                % V_HV > 410V (과전압) → err_v < 0 → I_ups_cmd < 0 (음수 전류)
                %   → 음수 위상 → HV→LV 전력 전달 (배터리 충전으로 HV 전압 내림)
                err_v = obj.V_HV_Ref - V_HV;
                obj.Integrator_v = obj.Integrator_v + err_v * Ts;
                
                % 전압 PI 출력 = 전류 지령
                I_ups_cmd = Kpv * err_v + Kiv * obj.Integrator_v;

                % 양방향 허용 (방전/충전 모두, ±I_UPS_Max)
                I_ups_cmd = max(-obj.I_UPS_Max, min(obj.I_UPS_Max, I_ups_cmd));

                Iref = I_ups_cmd;
                I_fb = I_LV; % LV 전류 기준으로 제어
            end
            
            I_cmd_out = Iref;

            % --- 5. 전류 PI → 외부 위상(phi_ext_deg) ---
            % Positive Feedback(발산) 방지를 위해 오차 부호 반전
            err_ext = Iref - I_fb;
            obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts;
            
            % 과도 상태에서 적분 폭주 방지 (UPS 모드에서 큰 에러 시)
            if (obj.PowerLossState == 1) && (abs(err_ext) > 10)
                obj.Integrator_ext = 0;
            end

            phi_pi = Kp_ext * err_ext + Ki_ext * obj.Integrator_ext;
            
            % --- 5.5. HDC Feedforward ---
            % [주석 처리됨] TODO: 사용자가 복구시키라고 할 때까지 주석으로 유지할 것
            phi_hdc = 0;
            % if obj.EnableHDC && ~isempty(obj.HDC_Model) && isfield(obj.HDC_Model, 'W')
            %      % HDC Prediction (Inlined)
            %      P_target = V_LV * Iref; 
            %      model = obj.HDC_Model;
            %      
            %      % 1. Quantize Inputs
            %      idx_v = floor( (V_HV - model.min_V)/(model.max_V - model.min_V) * (model.L_levels-1) ) + 1;
            %      idx_v = max(1, min(model.L_levels, idx_v));
            %      
            %      idx_p = floor( (P_target - model.min_P)/(model.max_P - model.min_P) * (model.L_levels-1) ) + 1;
            %      idx_p = max(1, min(model.L_levels, idx_p));
            %      
            %      % 2. Retrieve & Bind
            %      hv_v = model.Level_Item_V(idx_v, :);
            %      hv_p = model.Level_Item_P(idx_p, :);
            %      
            %      query_hv = hv_v .* hv_p;
            %      
            %      % 3. Query (Dot Product)
            %      Output = query_hv * model.W; 
            %      
            %      % 4. D3 to Phase
            %      d3_pred = Output(3);
            %      phi_hdc = d3_pred * 360; 
            % end
            
            phi_raw = phi_pi + phi_hdc;
            
            % Saturation (Clamping): 외부 위상 ±90도 제한
            phi_ext_deg = max(-90, min(90, phi_raw));
            
            phi_ext_out = phi_ext_deg;

            % =====================================================================
            % --- 6. SPS 제어: 고정 내부 위상 사용 ---
            % =====================================================================
            % SPS에서는 D1 = D2 = 0.5 (50% 듀티)
            % 이는 Primary와 Secondary 각각 180도 위상차 고정
            
            % [주석 처리됨] TPS/DPS: 전류 크기에 따른 내부 위상 조절
            % I_mag = abs(I_fb); % 실제 흐르는 전류 크기
            % I_norm = min(1, I_mag / obj.I_Base); % 정규화 (0~1)
            % phi_int_deg = obj.Psi_Min + (obj.Psi_Max - obj.Psi_Min) * I_norm;
            
            % SPS 고정 내부 위상 (180도)
            phi_int_deg = obj.SPS_Inner_Phase;

            % =====================================================================
            % --- 7. 위상 계산 (360도 각도 출력) ---
            % =====================================================================
            % SPS 제어 기준:
            % - Primary Bridge: Leg A (S1) = 0도, Leg B (S3) = 180도 (고정)
            % - Secondary Bridge: Leg A (S5) = phi_ext, Leg B (S7) = phi_ext + 180도
            %
            % 전력 전달 방향:
            % - phi_ext > 0: Primary → Secondary (방전, LV에서 HV로)
            % - phi_ext < 0: Secondary → Primary (충전, HV에서 LV로)
            
            % Primary Bridge 위상 (고정)
            phi1_deg = 0;                       % S1 기준 (항상 0도)
            phi3_deg = phi_int_deg;             % S3 = 내부 위상 (SPS: 180도)
            
            % Secondary Bridge 위상 (외부 위상에 의해 결정)
            phi5_deg = phi_ext_deg;             % S5 = 외부 위상
            phi7_deg = phi_ext_deg + phi_int_deg;  % S7 = 외부 위상 + 내부 위상
            
            % 위상 출력: 음수 그대로 유지 (디버깅 편의)
            % phi1_deg: 항상 0도
            % phi3_deg: 180도 (SPS 고정)
            % phi5_deg: -90 ~ +90도 범위 (외부 위상)
            % phi7_deg: phi5 + 180도 (90 ~ 270도 범위)
            %
            % 참고: PWM Generator에서 음수 위상도 올바르게 처리함
            % 0~360 정규화가 필요하면 PWM Generator 내부에서 수행
            
            % --- 8. 추가 출력 (디버깅/모니터링용) ---
            % D1, D2: SPS에서는 50% 고정 (0.5)
            D1_out = 0.5;       % Primary Inner Duty (SPS 고정)
            D2_out = 0.5;       % Secondary Inner Duty (SPS 고정)
            integrator_out = obj.Integrator_ext;  % 전류 적분기 상태값
            
            % --- 9. 이전 모드 저장 (다음 스텝용) ---
            obj.PreviousMode = obj.PowerLossState;
            
            % 사용하지 않는 입력 경고 방지
            %#ok<*NASGU>
            if false
                dummy = I_HV; %#ok<UNRCH>
            end
        end

        function resetImpl(obj)
            % 리셋 신호가 들어오거나 재시작 시
            obj.Integrator_ext = 0;
            obj.Integrator_v = 0;
            obj.PowerLossState = 0;
            obj.PreviousMode = 0;
        end
        
        % --- 출력 개수 지정 ---
        function num = getNumOutputsImpl(~)
            num = 11;
        end
        
        % --- 출력 크기 지정 ---
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8,sz9,sz10,sz11] = getOutputSizeImpl(~)
            sz1 = [1 1]; sz2 = [1 1]; sz3 = [1 1]; sz4 = [1 1];
            sz5 = [1 1]; sz6 = [1 1]; sz7 = [1 1]; sz8 = [1 1];
            sz9 = [1 1]; sz10 = [1 1]; sz11 = [1 1];
        end
        
        % --- 출력 데이터 타입 지정 ---
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9,dt10,dt11] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double'; dt4 = 'double';
            dt5 = 'double'; dt6 = 'double'; dt7 = 'double'; dt8 = 'double';
            dt9 = 'double'; dt10 = 'double'; dt11 = 'double';
        end
        
        % --- 출력 복소수 여부 ---
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9,cp10,cp11] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false; cp4 = false;
            cp5 = false; cp6 = false; cp7 = false; cp8 = false;
            cp9 = false; cp10 = false; cp11 = false;
        end
        
        % --- 고정 크기 출력 ---
        function [fs1,fs2,fs3,fs4,fs5,fs6,fs7,fs8,fs9,fs10,fs11] = isOutputFixedSizeImpl(~)
            fs1 = true; fs2 = true; fs3 = true; fs4 = true;
            fs5 = true; fs6 = true; fs7 = true; fs8 = true;
            fs9 = true; fs10 = true; fs11 = true;
        end
        
        % --- DiscreteState 사양 지정 (Simulink 필수) ---
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            % 각 DiscreteState 속성의 크기, 타입, 복소수 여부 지정
            switch name
                case 'Integrator_ext'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Integrator_v'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'PowerLossState'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'PreviousMode'
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    error(['Unknown state: ', name]);
            end
        end

        % --- 샘플 시간 지정 (필수!) ---
        function sts = getSampleTimeImpl(obj)
            % 이 시스템은 이산 시간(Discrete)으로만 동작함을 선언
            % Type: Discrete, SampleTime: Ts_ctrl (1e-7), Offset: 0
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                                   'SampleTime', obj.Ts_ctrl, ...
                                   'OffsetTime', 0);
        end
    end
end