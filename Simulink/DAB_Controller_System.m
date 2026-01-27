classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 SPS 위상 제어기
    %
    % 개선사항 (v3.0 - SPS Mode):
    %   - SPS (Single Phase Shift) 제어만 사용
    %   - 위상 출력을 360도 각도로 출력 (시간 대신)
    %   - D1, D2는 SPS 고정값 (50%)으로 출력
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual
    % 출력: phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, 
    %       I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out, PWM_Enable
    %
    % 출력 위상 설명 (모두 0~360도 범위):
    % 출력 위상 설명 (모두 0~360도 범위):
    %   - phi1_deg: Primary Bridge Leg A 기준 위상 (항상 0도)
    %   - phi3_deg: Primary Bridge Leg B 위상 (SPS: 180도 고정)
    %   - phi5_deg: Secondary Bridge Leg A 위상 (외부 위상에 의해 결정)
    %   - phi7_deg: Secondary Bridge Leg B 위상 (SPS: phi5 + 180도)
    %
    % [부호 규약 (표준화)]
    %   - 전류(I): (+)=방전(LV->HV), (-)=충전(HV->LV)
    %   - 위상(Phi): (+)=전력 전달 Primary->Secondary (충전), (-)=Secondary->Primary (방전)
    %   ** 따라서 I_ref가 양수(방전)이면, 위상 Phi는 음수가 되어야 함. **

    % =========================================================================
    % 1. Public, Tunable Properties (블록 더블클릭 시 설정 가능한 파라미터)
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Ts_ctrl = 1/80e3;       % 제어기 샘플 시간 (초) - 1/Fs (12.5us)로 변경하여 현실적인 MCU 동작 모사
        
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
        
        % [NEW] Feedforward & System Params
        L_leakage = 90e-6;      % [MODIFIED] 누설 인덕턴스 90uH (사용자 설정 반영)
        EnableFF = true;       % [MODIFIED] 피드포워드 활성화 (기본 위상 제공으로 P게인 부담 경감)
        
        % [NEW] 지령치 슬루 레이트 제한 (A/s)
        % 예: 200A/s -> 0.0001초(100us)당 0.02A 변화
        % 부호 반전 시(-5 -> +5) 부드러운 전환 유도
        % 부호 반전 시(-5 -> +5) 부드러운 전환 유도
        MaxSlewRate = 2000;      % 기본값: 초당 2000A (매우 빠름, 필요 시 낮출 것)
        
        % [NEW] 전압 센싱 필터 시정수 (Disturbance Rejection)
        % 전압 급변 시 피드포워드가 너무 민감하게 반응하지 않도록 완충
        % [NEW] 전압 센싱 필터 시정수 (Disturbance Rejection)
        % 전압 급변 시 표드포워드 및 제어기가 너무 민감하게 반응하지 않도록 대폭 완화
        Tau_Voltage_Filter = 30e-3; % 30ms (피드포워드 진동 억제를 위해 시정수 강화)
        
        % [NEW] Dead Zone & Gain Scheduling Params
        DeadZone = 0.5;         % 불감대: 0.5A 이내의 오차는 0으로 처리 (진동 방지)
        GainSched_Threshold = 2.0; % 게인 스케줄링 임계값 (2A)
        GainSched_Factor = 0.4;    % 임계값 이내일 때 적용할 게인 비율 (40%)
        
        % [NEW] Soft Start Time
        SoftStartTime = 0.2;    % 0.2초 동안 서서히 피드포워드 증가
        
        % [NEW] Phase Change Rate Limiter (위상 변화율 제한)
        % 상용 제품의 Soft Change 모사 (DC 바이어스 및 과도 전류 방지)
        % 30,000 deg/s -> 30 deg/ms -> 90도 변화에 약 3ms 소요
        MaxPhaseRate = 30000;   % deg/s
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
        I_fb_filtered;      % 전류 피드백 필터링용 상태변수

        I_ref_ramped;       % [NEW] 슬루 레이트 적용된 내부 전류 지령치
        V_HV_Filtered;      % [NEW] 필터링된 HV 전압
        V_LV_Filtered;      % [NEW] 필터링된 LV 전압
        
        % [NEW] Feedforward Downsampling
        FeedforwardUpdateTimer; % 피드포워드 갱신 타이머
        Last_Phi_FF;            % 마지막으로 계산된 피드포워드 값 유지
        
        % [NEW] Phase Output Memory for Rate Limiting
        Prev_Phi_Output;        % 이전 스텝 위상 출력값
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
            obj.I_fb_filtered = 0;

            obj.I_ref_ramped = 0; % [NEW] 램프 지령치 초기화
            obj.V_HV_Filtered = 0;
            obj.V_LV_Filtered = 0;
            
            obj.FeedforwardUpdateTimer = 0;
            obj.Last_Phi_FF = 0;
            obj.Prev_Phi_Output = 0;
        end

    function [phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, ...
                  I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out, PWM_Enable, ...
                  err_out, phi_raw_out, excess_out, ifb_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual)
            
            % --- 1. 파라미터 계산 ---
            Ts = obj.Ts_ctrl;  % 제어기 샘플 시간 사용 (적분기용)
            
            % [MODIFIED] Startup Delay (초기 기동 대기)
            % 0.001초까지는 제어기 동작 중지 및 출력 0 고정
            % 시뮬레이션 초기 과도 상태 및 수렴 실패 방지
            t_current = getCurrentTime(obj);
            if t_current < 0.001
                % [출력 강제 0 할당]
                phi1_deg = 0; phi3_deg = 180; % SPS 기본 위상
                phi5_deg = 0; phi7_deg = 180; % 외부 위상 0
                mode_active = 0; % 0: 대기 모드
                status_flag = 0;
                I_cmd_out = 0;
                phi_ext_out = 0;
                % [중요] 스위칭 자체를 중단 (Gate Off)하여 변압기 포화/돌입전류 방지
                D1_out = 0; D2_out = 0; 
                integrator_out = 0;
                
                % [DEBUG] 초기화
                err_out = 0; phi_raw_out = 0; excess_out = 0; ifb_out = 0;
                
                % [NEW] PWM 비활성화 신호 출력
                PWM_Enable = 0; % Gate OFF 강제
                
                % 내부 상태변수 초기화 유지
                obj.I_ref_ramped = 0;
                obj.Integrator_ext = 0;
                obj.Integrator_v = 0;
                
                % 필터 상태변수는 현재 값으로 추적 (단절 방지)
                obj.V_HV_Filtered = V_HV;
                obj.V_LV_Filtered = V_LV;
                obj.I_fb_filtered = I_LV;
                return;
            end

            % --- 1.5. 전압/전류 센싱 LPF (강인성 확보) ---
            % 전압이 스텝으로 튈 때 제어기가 발작하지 않도록 부드럽게 만듦
            % alpha = Ts / (Tau + Ts)
            alpha_v = Ts / (obj.Tau_Voltage_Filter + Ts);
            
            % 초기값이 0이면 현재 값으로 초기화 (Startup Transient 방지)
            if obj.V_HV_Filtered == 0, obj.V_HV_Filtered = V_HV; end
            if obj.V_LV_Filtered == 0, obj.V_LV_Filtered = V_LV; end
            
            obj.V_HV_Filtered = (1 - alpha_v) * obj.V_HV_Filtered + alpha_v * V_HV;
            obj.V_LV_Filtered = (1 - alpha_v) * obj.V_LV_Filtered + alpha_v * V_LV;
            
            % [중요] 제어 로직에는 필터링된 "얌전한" 전압 사용
            V_HV_Control = obj.V_HV_Filtered;
            % V_LV_Control = obj.V_LV_Filtered; % 필요 시 사용

            % --- 2. 정전 감지 (밴드 기반) ---
            % 379~381 V : 정상 → ESS
            % 그 외 : 정전/이상 → UPS
            % 감지에는 즉각적인 반응을 위해 원본 V_HV 사용 가능, 
            % 하지만 노이즈 오작동 막으려면 V_HV_Control 사용 권장. 여기선 V_HV_Control 사용.
            if (V_HV_Control < (obj.V_HV_Ref - obj.V_HV_Band)) || (V_HV_Control > (obj.V_HV_Ref + obj.V_HV_Band))
                obj.PowerLossState = 1; % UPS 모드
            else
                obj.PowerLossState = 0; % ESS 모드
            end
            
            % --- 3. 모드 전환 감지 ---
            mode_changed = (obj.PowerLossState ~= obj.PreviousMode);
            
            % [Fixed] Bumpless Transfer
            % 모드 전환 시 적분기를 강제로 0으로 리셋하면 위상이 튀어 시스템이 발산함.
            % 따라서 리셋 코드를 제거하고, 기존 적분값을 유지하여 부드럽게 이어지도록 함.
            if mode_changed
                 % 모드 변경 로깅이나 플래그 처리만 수행 (필요 시)
            end
            
            % [NEW] Feedback LPF (Stronger)
            % 스위칭 노이즈 제거를 위해 필터 강화 (alpha: 0.5 -> 0.1)
            alpha_lpf = 0.04; % [Modified] 0.1 -> 0.04 (노이즈 억제 강화) 
            obj.I_fb_filtered = (1-alpha_lpf)*obj.I_fb_filtered + alpha_lpf*I_LV;
            I_fb_clean = obj.I_fb_filtered;
            
            % --- 4. 모드별 기준 전류/피드백 선택 ---
            if obj.PowerLossState == 0
                %---------------------------
                % ESS 모드 (정상 상태)
                %---------------------------
                mode_active = 3;
                status_flag = 0;

                % ESS: I_ess_cmd 그대로 사용 (±I_ESS_Max 제한)
                Iref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, I_ess_cmd));
                I_fb = I_fb_clean; % 필터링된 배터리 전류 사용

            else
                %---------------------------
                % UPS 모드 (정전/이상, 부하 공급)
                %---------------------------
                mode_active = 1;
                status_flag = 1;

                % *** Cascade 제어: 전압 외부루프 → 전류 지령 생성 ***
                % HV 전압을 V_HV_Ref (380V)로 정확히 유지
                % V_HV < 380V (부족) → 방전 필요 → I_ups_cmd > 0 (양수, 방전)
                % V_HV > 380V (과잉) → 충전 필요 → I_ups_cmd < 0 (음수, 충전)
                
                % [논리 확인]
                % err_v = Ref - Meas
                % 예: 380 - 370 = +10V (부족) -> Kp * (+10) -> +I_cmd (방전) -> OK
                err_v = obj.V_HV_Ref - V_HV_Control; % 안정된 전압 사용
                
                % [Voltage Loop Anti-Windup: Clamping]
                % 1. 가비지 적분값 및 예상 출력 계산
                temp_int_v = obj.Integrator_v + err_v * Ts;
                temp_I_cmd = Kpv * err_v + Kiv * temp_int_v;
                
                % 2. 포화 여부 확인 및 적분 업데이트
                if temp_I_cmd > obj.I_UPS_Max
                    % 상한 포화: 오차가 줄어드는 방향(음수)일 때만 적분 허용
                    if err_v < 0
                        obj.Integrator_v = temp_int_v;
                    end
                    I_ups_cmd = obj.I_UPS_Max; % 출력 제한
                elseif temp_I_cmd < -obj.I_UPS_Max
                    % 하한 포화: 오차가 줄어드는 방향(양수)일 때만 적분 허용
                    if err_v > 0
                        obj.Integrator_v = temp_int_v;
                    end
                    I_ups_cmd = -obj.I_UPS_Max; % 출력 제한
                else
                    % 비포화 구간: 정상 적분
                    obj.Integrator_v = temp_int_v;
                    I_ups_cmd = temp_I_cmd;
                end

                Iref = I_ups_cmd;
                I_fb = I_fb_clean; % 필터링된 LV 전류 기준으로 제어
            end
            
            % [MODIFIED] Slew Rate Limiter (지령치 완충)
            % 급격한 지령 변화(예: -5A -> +5A)를 잘게 쪼개서 추종
            I_target = Iref; 
            max_step = obj.MaxSlewRate * Ts;
            
            delta_I = I_target - obj.I_ref_ramped;
            
            if abs(delta_I) > max_step
                % 제한폭보다 크면 제한폭만큼만 이동
                obj.I_ref_ramped = obj.I_ref_ramped + sign(delta_I) * max_step;
            else
                % 제한폭 이내면 바로 도달
                obj.I_ref_ramped = I_target;
            end
            
            % 출력 및 제어에는 '부드러워진' 지령치 사용
            I_cmd_out = obj.I_ref_ramped;

            % --- 5. 전류 PI → 외부 위상(phi_ext_deg) ---
            % [부호 규약 적용]
            % 목표: I_ref > 0 (방전) --> Phi < 0 (Secondary -> Primary)
            % [MODIFIED] 1. Dead Zone (불감대) 적용
            raw_err = obj.I_ref_ramped - I_fb;
            if abs(raw_err) < obj.DeadZone
                err_ext = 0; % 오차가 작으면 무시 (리플에 의한 발진 차단)
            else
                % 오차의 연속성을 위해 데드존만큼 빼줌 (Smooth)
                err_ext = raw_err - sign(raw_err) * obj.DeadZone;
            end
            
            % [New] Feedforward Calculation (Downsampled to 5ms)
            % 매 스텝 계산하면 리플에 민감하므로, 0.005초마다 갱신
            FF_Update_Period = 0.005; 
            obj.FeedforwardUpdateTimer = obj.FeedforwardUpdateTimer + Ts;
            
            if obj.FeedforwardUpdateTimer >= FF_Update_Period
                % 타이머 만료 시에만 새로 계산
                if obj.EnableFF && abs(V_HV_Control) > 10
                    K_ff = (2 * obj.L_leakage * obj.Fs * 180) / V_HV_Control;
                    % [MODIFIED] 지령치가 아닌 'Ramped' 지령치를 사용하여 스텝 응답 시 FF 점프 방지
                    obj.Last_Phi_FF = -1 * obj.I_ref_ramped * K_ff; 
                else
                     obj.Last_Phi_FF = 0;
                end
                obj.FeedforwardUpdateTimer = 0; % 리셋
            end
            
            % [Modified] Soft Start Logic 적용
            % 시뮬레이션 초기(0 ~ 0.2초)에는 피드포워드 값을 0에서부터 서서히 올림
            % 급격한 위상 인가로 인한 LC 공진 및 돌입 전류 방지
            current_time = getCurrentTime(obj);
            soft_start_gain = 1.0;
            if current_time < obj.SoftStartTime
                soft_start_gain = current_time / obj.SoftStartTime;
            end
            
            phi_ff = obj.Last_Phi_FF * soft_start_gain; 
            
            % [Current Loop Anti-Windup: Incremental Back-Calculation]
            % 표준적인, 점진적 Anti-windup 방식 (노이즈에 강건함)
            
            % [MODIFIED] 2. Gain Scheduling (Active)
            Kp_current = Kp_ext; 
            if abs(err_ext) < obj.GainSched_Threshold
                Kp_current = Kp_ext * obj.GainSched_Factor; 
            end

            % 3. 순수 PI 출력 계산 (현재 적분값 기준)
            phi_pi_unbounded = -1 * (Kp_current * err_ext + Ki_ext * obj.Integrator_ext);
            
            % 2. 전체 출력 및 포화 계산
            phi_raw_unbounded = phi_pi_unbounded + phi_ff;
            PHI_LIMIT = 90;
            
            % [MODIFIED] 1. Saturation (Hard Limit)
            phi_saturated = max(-90, min(90, phi_raw_unbounded));
            
            % [MODIFIED] 2. Rate Limiter (Soft Change)
            % 급격한 위상 변화를 제한하여 Transformer/Switch 스트레스 저감
            delta_phi = phi_saturated - obj.Prev_Phi_Output;
            max_phi_step = obj.MaxPhaseRate * Ts;
            
            if abs(delta_phi) > max_phi_step
                % 제한폭보다 변화량이 크면 제한폭만큼만 이동
                phi_ext_deg = obj.Prev_Phi_Output + sign(delta_phi) * max_phi_step;
            else
                phi_ext_deg = phi_saturated;
            end
            
            % 상태 업데이트
            obj.Prev_Phi_Output = phi_ext_deg;
            
            % 3. Back-Calculation (적분기 업데이트)
            % 차이(Excess)만큼 적분기를 반대 방향으로 서서히 구동
            % Excess = Saturated - Unbounded
            % Eq: Int_new = Int + err*Ts - K_aw * (Excess) * Ts
            % 여기서 K_aw(Anti-windup Gain)는 보통 Ki/Kp 혹은 Ki*2 등을 사용
            % 이 시스템은 -1 게인이 출력단에 있으므로 부호가 반대로 작동해야 함을 고려하여 유도된 식:
            % obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts - K_aw * (phi_ext_deg - phi_raw_unbounded) * Ts;
            
            % Kaw 설정: 매우 중요한 튜닝 포인트
            % 기존(2*Ki)은 너무 커서 적분기를 급격히 0으로 만들어 시스템 발산 유발
            % 안전하게 1.0으로 설정하여 부드럽게(Slowly) 포화에서 빠져나오도록 함.
            K_aw = 1.0; 
            
            % 차이 계산 (포화되지 않았으면 0)
            excess = phi_ext_deg - phi_raw_unbounded;
            
            % 적분 상태 업데이트 (이전 스텝 값 기반 누적)
            % 유도: Unbounded = -1 * (Kp*err + Ki*Int) + FF
            % Unbounded > Limit (High) -> Int too Small (Negative large).
            % We need Int to Increase.
            % Excess = Limit - Unbound (Negative).
            % So we need Term to be Positive.
            % -1 * Excess * Kaw = Positive. Correct.
            
            obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts - (K_aw * excess * Ts);
            
            % 출력값은 이미 위에서 결정됨 (phi_ext_deg)
            phi_raw = phi_ext_deg; % (디버깅 변수용)
            
            % Final Safety Clamp (중복이지만 안전용)
            phi_ext_deg = max(-90, min(90, phi_ext_deg));
            
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
            
            % [Fixed] 이전 로직 잔재 삭제 (phi_pi 미정의 오류 해결)
            % HDC는 현재 0이므로 추가 연산 없이 위에서 계산된 값 사용
            
            % [NEW] Manual Phase Override Logic (User Request)
            % Mode_Select = 1 -> Use Phi_Manual directly
            % Mode_Select = 0 -> Use Calculated Controller Output
            if Mode_Select == 1
                phi_ext_deg = Phi_Manual;
            end
            
            % 최종 출력 할당
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
            % - phi_ext > 0: Primary → Secondary (충전, HV에서 LV로) [수정됨]
            % - phi_ext < 0: Secondary → Primary (방전, LV에서 HV로) [수정됨]
            
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
            
            % [NEW] 정상 동작 시 PWM 활성화
            PWM_Enable = 1;

            % [DEBUG] 내부 상태 출력
            err_out = err_ext;
            phi_raw_out = phi_raw_unbounded;
            excess_out = excess; 
            ifb_out = I_fb_clean;
            
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
            obj.I_fb_filtered = 0;
            obj.Prev_Phi_Output = 0;
        end
        
        % --- 출력 개수 지정 ---
        function num = getNumOutputsImpl(~)
            num = 16; % 디버깅용 포트 4개 추가
        end
        
        % --- 출력 크기 지정 ---
        % --- 출력 크기 지정 ---
        function [sz1,sz2,sz3,sz4,sz5,sz6,sz7,sz8,sz9,sz10,sz11,sz12,sz13,sz14,sz15,sz16] = getOutputSizeImpl(~)
            sz1 = [1 1]; sz2 = [1 1]; sz3 = [1 1]; sz4 = [1 1];
            sz5 = [1 1]; sz6 = [1 1]; sz7 = [1 1]; sz8 = [1 1];
            sz9 = [1 1]; sz10 = [1 1]; sz11 = [1 1]; sz12 = [1 1];
            sz13 = [1 1]; sz14 = [1 1]; sz15 = [1 1]; sz16 = [1 1];
        end
        
        % --- 출력 데이터 타입 지정 ---
        function [dt1,dt2,dt3,dt4,dt5,dt6,dt7,dt8,dt9,dt10,dt11,dt12,dt13,dt14,dt15,dt16] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double'; dt4 = 'double';
            dt5 = 'double'; dt6 = 'double'; dt7 = 'double'; dt8 = 'double';
            dt9 = 'double'; dt10 = 'double'; dt11 = 'double'; dt12 = 'double';
            dt13 = 'double'; dt14 = 'double'; dt15 = 'double'; dt16 = 'double';
        end
        
        % --- 출력 복소수 여부 ---
        function [cp1,cp2,cp3,cp4,cp5,cp6,cp7,cp8,cp9,cp10,cp11,cp12,cp13,cp14,cp15,cp16] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false; cp4 = false;
            cp5 = false; cp6 = false; cp7 = false; cp8 = false;
            cp9 = false; cp10 = false; cp11 = false; cp12 = false;
            cp13 = false; cp14 = false; cp15 = false; cp16 = false;
        end
        
        % --- 고정 크기 출력 ---
        function [fs1,fs2,fs3,fs4,fs5,fs6,fs7,fs8,fs9,fs10,fs11,fs12,fs13,fs14,fs15,fs16] = isOutputFixedSizeImpl(~)
            fs1 = true; fs2 = true; fs3 = true; fs4 = true;
            fs5 = true; fs6 = true; fs7 = true; fs8 = true;
            fs9 = true; fs10 = true; fs11 = true; fs12 = true;
            fs13 = true; fs14 = true; fs15 = true; fs16 = true;
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
                case 'I_fb_filtered'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'I_ref_ramped'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_HV_Filtered'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_LV_Filtered'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'FeedforwardUpdateTimer'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Last_Phi_FF'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Prev_Phi_Output'
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

        % --- [NEW] 출력 포트 이름 명시 (디버깅 편의성) ---
        function [name1, name2, name3, name4, name5, name6, name7, name8, name9, name10, name11, name12, name13, name14, name15, name16] = getOutputNamesImpl(~)
            name1 = 'phi1';
            name2 = 'phi3';
            name3 = 'phi5';
            name4 = 'phi7';
            name5 = 'Active_Mode';   % 1:UPS, 3:ESS
            name6 = 'Status_Flag';   % 1:Abnormal, 0:Normal
            name7 = 'I_Ref_Ramped';  % 실제 제어기가 추종하는 지령치
            name8 = 'Phi_Ext_Deg';
            name9 = 'D1_Out';
            name10 = 'D2_Out';
            name11 = 'Integ_State';
            name12 = 'PWM_Enable';   % 0:Disable(All OFF), 1:Enable
            name13 = 'Err_Amp';      % [DEBUG] 추종 오차 (지령-실제)
            name14 = 'Phi_Raw';      % [DEBUG] 포화 전 제어기 출력
            name15 = 'Excess';       % [DEBUG] 포화량 (Raw - Sat)
            name16 = 'I_Filt';       % [DEBUG] 필터링된 피드백 전류
        end
    end
end