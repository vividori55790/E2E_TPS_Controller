classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System  (v6.0 — Unified Linear Pipeline)
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 위상 제어기
    %
    % 아키텍처:
    %   1. Sensing & Filtering
    %   2. FSM (State Transition)
    %   3. Reference Generation & BMS Safety
    %   4. Control Strategy Selection (SPS Default → TPS Override)
    %   5. PI Feedback (항상 동작)
    %   6. Signal Synthesis & Limiting
    %   7. Flux Balancing
    %   8. Phase Assignment (통합 수식: D1, D2, Phi → phi1~phi7)
    %
    % Ctrl_Mode_Ext: 0=SPS, 1=EPS(Reserved), 2=DPS(Reserved), 3=TPS/HDC
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_L_pri, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual, Ctrl_Mode_Ext
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
    % 
    % =========================================================================
    % 1. Public, Tunable Properties (블록 더블클릭 시 설정 가능한 파라미터)
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Ts_ctrl = 1e-7;         % 제어 샘플링 시간 (Ts) [sec]
        
        % HV 전압 목표 및 밴드 범위
        V_HV_Ref = 380;         % [UPS] HV 목표 전압 (V)
        V_HV_Band = 30;         % [UPS] 정전 감지 오차 범위 (±V)
        
        % [SPS 모드] 내부 위상 고정값
        SPS_Inner_Phase = 180;  % [SPS] 내부 위상 고정값 (180도)
        
        % [주석 처리됨] DPS 내부위상 범위 - TPS/DPS 제어 시 사용
        % Psi_Min = 5;            % 최소 내부 위상 (경부하)
        % Psi_Max = 25;           % 최대 내부 위상 (대전력)
        % I_Base = 10;            % 내부위상 계산용 기준 전류 (A)
        
         % [TPS/HDC] Control Parameters
         % HDC 모델 기반 TPS 제어 — Ctrl_Mode_Ext=3 일 때 활성화
         % [TPS/HDC] Control Parameters
         % HDC 모델 구조체 (Code Gen 호환)
         % 사용법: MATLAB 워크스페이스에 로드된 HDC 구조체 변수명을 입력
         HDC_Data = struct('D',0);
        
        % [NEW] Feedforward & System Params
        L_leakage = 90e-6;      % 변압기 누설 인덕턴스 (H)
        EnableFF = true;       % 피드포워드(FF) 제어 활성화
        
        % [NEW] Reference Slew Rate
        MaxSlewRate = inf;      % 전류 지령 변화율 제한 (A/s) [inf: OFF]
        
        % [HDC Optimization] Execution Triggers (연산량 감소용)
        HDC_Update_V_Th = 2.0;    % HDC 재실행 전압 변화 임계값 (V)
        HDC_Update_P_Th = 50.0;   % HDC 재실행 전력 변화 임계값 (W)
        
        % [NEW] 전압 센싱 필터 (Sliding Window 사용)
        % Tau_Voltage_Filter = 30e-3; % [REMOVED] LPF 대신 이동평균 사용
        
        % [NEW] Dead Zone & Gain Scheduling
        DeadZone = 0;           % [Tune] 제어 불감대 (Deadzone) [A]
        GainSched_Threshold = 2.0; % [Tune] 게인 스케줄링 임계값 [A]
        GainSched_Factor = 1.0;    % [Tune] 게인 감소 비율 (0~1)
        
        % [NEW] Soft Start Time
        SoftStartTime = 0.2;    % 초기 기동 Soft Start 시간 (sec)
        
        % [NEW] Initialization Delay
        InitDelay = 0.001;      % FSM 초기화 지연 시간 (sec) - 센서 안정화 대기
        
        % [NEW] Phase Change Rate Limiter
        MaxPhaseRate = 30000;   % 위상 변화율 제한 (deg/s)
        
        % [NEW] Smart BMS Thresholds (26S Li-ion Spec)
        V_Pack_Max = 109.2;     % [BMS] 배터리 최대 전압 (Cell*4.2V)
        V_CV_Start = 108.0;     % [BMS] CV 충전 시작 전압
        V_ESS_Limit = 93.0;     % [BMS] 방전 중단 전압 (ESS Reserve)
        V_UPS_Limit = 87.0;     % [BMS] 출력 제한 시작 전압 (UPS Derating)
        V_Min_Limit = 84.0;     % [BMS] 셧다운 최저 전압 (System UVLO)
        
        K_CV = 5.0;             % [BMS] CV 제어 게인 (A/V)
                                
        % [NEW] Flux Balancing Parameters
        EnableFluxBalancing = true;  % Flux Balancing 활성화
        Flux_Kp = 0.005;        % [Flux] 비례 게인 (Kp)
        Flux_Ki = 0.05;         % [Flux] 적분 게인 (Ki)
        Flux_Limit = 0.02;      % [Flux] 최대 보정 Duty 제한 (0~0.5)
    end

    properties(Constant)
        % FSM State Definitions
        ST_INIT     = 1;
        ST_ESS_RUN  = 2;
        ST_UPS_RUN  = 3;
        ST_MANUAL   = 4;
        ST_FAULT    = 5;
    end

    properties
        % 전류 지령 제한
        I_ESS_Max = 10;         % [ESS] 최대 허용 전류 (A)
        I_UPS_Max = 30;         % [UPS] 최대 허용 전류 (A)
    end

    % =========================================================================
    % 1.5 Internal (Hidden) Properties — 외부 노출 불필요
    % =========================================================================
    properties(Access = private)
        HDC_Model_Data = struct();  % setupImpl에서 자동 로드되는 HDC 파라미터
    end

    % =========================================================================
    % 2. Internal States (DiscreteState) - persistent 대체
    % =========================================================================
    properties(DiscreteState)
        Integrator_ext;     % 전류 PI 적분항
        Integrator_v;       % 전압 PI 적분항 (UPS 모드)
        CurrentState;       % [FSM] 현재 상태
        PreviousMode;       % [FSM] 이전 상태 (Transition 감지용)


        I_ref_ramped;       % [NEW] 슬루 레이트 적용된 내부 전류 지령치
        V_HV_Filtered;      % [NEW] 필터링된 HV 전압 (이동평균)
        V_LV_Filtered;      % [NEW] 필터링된 LV 전압 (이동평균)
        
        % [NEW] Voltage Moving Average Buffers
        V_HV_Buffer;        % HV 전압 버퍼
        V_HV_Sum;           % HV 전압 합계
        V_LV_Buffer;        % LV 전압 버퍼
        V_LV_Sum;           % LV 전압 합계
        
        % [NEW] Feedforward Downsampling
        % FeedforwardUpdateTimer; % [REMOVED] 즉시 반영으로 변경됨
        Last_Phi_FF;            % 마지막으로 계산된 피드포워드 값 유지
        
        % [NEW] Phase Output Memory for Rate Limiting
        Prev_Phi_Output;        % 이전 스텝 위상 출력값
        
        % [NEW] High-Speed Moving Average Properties
        I_Buffer;               % [125x1] Circular Buffer (1 Switching Cycle @ 10MHz)
        BufferIdx;              % Current Index
        I_Sum;                  % Running Sum for O(1) Averaging
        ControlCheckDone;       % [NEW] Parameter check flag
        
        % [NEW] Flux Balancing States
        % [NEW] Flux Balancing States
        I_L_pri_Buffer;         % Inductor 전류 DC 검출용 버퍼
        I_L_pri_Sum;
        Flux_Integrator;        % Flux 제어기 적분항
        
        % [NEW] Time Counter (for Code Generation compatibility)
        TickCount;              % 시뮬레이션 틱 카운터 (t = TickCount * Ts)
        
        % [HDC Optimization] Caching States
        Last_Vin_HDC;
        Last_Vout_HDC;
        Last_Pref_HDC;
        Cached_D1;
        Cached_D2;
        Cached_Phi;
    end

    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)
        
        function setupImpl(obj)
            % 초기화 (시뮬레이션 시작 시 1회 실행)
            obj.Integrator_ext = 0;
            obj.Integrator_v = 0;
            obj.Flux_Integrator = 0; % [NEW]
            obj.CurrentState = obj.ST_INIT; % 초기 상태: INIT
            obj.PreviousMode = obj.ST_INIT;


            obj.I_ref_ramped = 0; % [NEW] 램프 지령치 초기화
            obj.V_HV_Filtered = 0;
            obj.V_LV_Filtered = 0;
            
            % obj.FeedforwardUpdateTimer = 0; % [REMOVED]
            obj.Last_Phi_FF = 0;
            obj.Prev_Phi_Output = 0;
            
            % Initialize Moving Average Filter (Window Size = 125)
            % 12.5us (1 Cycle) / 0.1us (Sample) = 125 Samples
            obj.I_Buffer = zeros(125,1);
            obj.BufferIdx = 1;
            obj.I_Sum = 0;
            
            % [NEW] Flux Balancing Buffer Init
            obj.I_L_pri_Buffer = zeros(125,1); % 동일한 윈도우 사이즈 사용
            obj.I_L_pri_Sum = 0;
            
            % [NEW] Voltage Buffers Init
            obj.V_HV_Buffer = zeros(125,1);
            obj.V_HV_Sum = 0;
            obj.V_LV_Buffer = zeros(125,1);
            obj.V_LV_Sum = 0;
            
            obj.ControlCheckDone = 0;
            obj.TickCount = 0; % [RESTORED]
            
            % [HDC Cache Init] Force run on first step
            obj.Last_Vin_HDC = -9999; 
            obj.Last_Vout_HDC = -9999;
            obj.Last_Pref_HDC = -9999;
            obj.Cached_D1 = 0.5;
            obj.Cached_D2 = 0.5;
            obj.Cached_Phi = 0;
            
            % =====================================================================
            % [HDC] Assign Model Data from Parameter (Nontunable Property)
            % =====================================================================
            obj.HDC_Model_Data = obj.HDC_Data;
            
            % Debug info (MATLAB only)
            if coder.target('MATLAB')
                if isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0
                     fprintf('>> [HDC] Model Parameter Loaded: Dim=%d, Bins=%d\n', ...
                        obj.HDC_Model_Data.D, obj.HDC_Model_Data.N_bins);
                elseif isfield(obj.HDC_Model_Data, 'D1_table')
                     fprintf('>> [HDC] LUT Table Loaded: Size=[%d x %d x %d]\n', ...
                        length(obj.HDC_Model_Data.Vin_bp), length(obj.HDC_Model_Data.Vout_bp), length(obj.HDC_Model_Data.P_bp));
                else
                     fprintf('>> [HDC] Empty Model Parameter detected. TPS Mode will be inactive.\n');
                end
            end
        end

    function [phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, ...
                  I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out, PWM_Enable, ...
                  err_out, phi_raw_out, excess_out, ifb_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L_pri, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual, Ctrl_Mode_Ext)
            
            Ts = obj.Ts_ctrl;
            t_current = double(obj.TickCount) * Ts;
            obj.TickCount = obj.TickCount + 1;
            
            % =================================================================
            % [STAGE 1] Sensing & Filtering (125-sample Moving Average)
            % =================================================================
            
            % Debug check (MATLAB only)
            if coder.target('MATLAB')
                if obj.ControlCheckDone == 0 && t_current >= 0.005
                     fprintf('>> [CTRL CHECK] Time=%.6f, Kp=%.4f, Ki=%.4f, Mode=%d\n', ...
                             t_current, Kp_ext, Ki_ext, Ctrl_Mode_Ext);
                     obj.ControlCheckDone = 1;
                end
            end
            
            % V_HV Filtering
            old_vhv = obj.V_HV_Buffer(obj.BufferIdx);
            obj.V_HV_Sum = obj.V_HV_Sum - old_vhv + V_HV;
            obj.V_HV_Buffer(obj.BufferIdx) = V_HV;
            obj.V_HV_Filtered = obj.V_HV_Sum / 125;
            
            % V_LV Filtering
            old_vlv = obj.V_LV_Buffer(obj.BufferIdx);
            obj.V_LV_Sum = obj.V_LV_Sum - old_vlv + V_LV;
            obj.V_LV_Buffer(obj.BufferIdx) = V_LV;
            obj.V_LV_Filtered = obj.V_LV_Sum / 125;
            
            V_HV_f = obj.V_HV_Filtered;
            V_LV_f = obj.V_LV_Filtered;

            % I_LV Current Filtering
            old_ilv = obj.I_Buffer(obj.BufferIdx);
            obj.I_Sum = obj.I_Sum - old_ilv + I_LV;
            obj.I_Buffer(obj.BufferIdx) = I_LV;
            
            % I_L_pri Filtering (Flux Balancing)
            old_pri = obj.I_L_pri_Buffer(obj.BufferIdx);
            obj.I_L_pri_Sum = obj.I_L_pri_Sum - old_pri + I_L_pri;
            obj.I_L_pri_Buffer(obj.BufferIdx) = I_L_pri;

            % Buffer Index (Circular)
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > 125, obj.BufferIdx = 1; end
            
            I_fb = obj.I_Sum / 125;
            I_L_pri_DC = obj.I_L_pri_Sum / 125;

            % =================================================================
            % [STAGE 2] FSM — State Transition Logic
            % =================================================================
            NextState = obj.CurrentState;
            
            if t_current < obj.InitDelay
                NextState = obj.ST_INIT;
            elseif Mode_Select == 1
                NextState = obj.ST_MANUAL;
            elseif V_LV_f < obj.V_Min_Limit
                NextState = obj.ST_FAULT;
            else
                switch obj.CurrentState
                    case obj.ST_INIT
                        if (V_HV_f < (obj.V_HV_Ref - obj.V_HV_Band)) || ...
                           (V_HV_f > (obj.V_HV_Ref + obj.V_HV_Band))
                            NextState = obj.ST_UPS_RUN;
                        else
                            NextState = obj.ST_ESS_RUN;
                        end
                    case obj.ST_ESS_RUN
                        if (V_HV_f < (obj.V_HV_Ref - obj.V_HV_Band)) || ...
                           (V_HV_f > (obj.V_HV_Ref + obj.V_HV_Band))
                            NextState = obj.ST_UPS_RUN;
                        end
                    case obj.ST_UPS_RUN
                        if (V_HV_f >= (obj.V_HV_Ref - obj.V_HV_Band)) && ...
                           (V_HV_f <= (obj.V_HV_Ref + obj.V_HV_Band))
                            NextState = obj.ST_ESS_RUN;
                        end
                    case obj.ST_MANUAL
                        if Mode_Select ~= 1
                            if (V_HV_f < (obj.V_HV_Ref - obj.V_HV_Band)) || ...
                               (V_HV_f > (obj.V_HV_Ref + obj.V_HV_Band))
                                NextState = obj.ST_UPS_RUN;
                            else
                                NextState = obj.ST_ESS_RUN;
                            end
                        end
                    case obj.ST_FAULT
                        if V_LV_f > (obj.V_Min_Limit + 5.0)
                            NextState = obj.ST_INIT;
                        end
                end
            end
            obj.CurrentState = NextState;
            
            % =================================================================
            % [STAGE 3] Reference Generation & BMS Safety
            % =================================================================
            PWM_Enable = 0;
            I_target = 0;
            mode_active = 0;
            
            switch obj.CurrentState
                case obj.ST_INIT
                    obj.Integrator_ext = 0;
                    obj.Integrator_v = 0;
                    
                case obj.ST_FAULT
                    % Hard Shutdown
                    
                case obj.ST_MANUAL
                    PWM_Enable = 1;
                    
                case {obj.ST_ESS_RUN, obj.ST_UPS_RUN}
                    PWM_Enable = 1;
                    
                    if obj.CurrentState == obj.ST_ESS_RUN
                        raw_ref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, I_ess_cmd));
                        mode_active = 3;
                    else
                        mode_active = 1;
                        err_v = obj.V_HV_Ref - V_HV_f;
                        temp_int_v = obj.Integrator_v + err_v * Ts;
                        limit_val = obj.I_UPS_Max;
                        temp_I_cmd = Kpv * err_v + Kiv * temp_int_v;
                        
                        if temp_I_cmd > limit_val
                            if err_v < 0, obj.Integrator_v = temp_int_v; end
                            raw_ref = limit_val;
                        elseif temp_I_cmd < -limit_val
                            if err_v > 0, obj.Integrator_v = temp_int_v; end
                            raw_ref = -limit_val;
                        else
                            obj.Integrator_v = temp_int_v;
                            raw_ref = temp_I_cmd;
                        end
                    end
                    
                    % --- BMS Safety ---
                    I_safe = raw_ref;
                    v_batt = V_LV_f;
                    
                    if raw_ref < 0  % Charging
                        if v_batt >= obj.V_Pack_Max
                            I_safe = 0;
                        elseif v_batt >= obj.V_CV_Start
                            I_limit = (obj.V_Pack_Max - v_batt) * obj.K_CV;
                            I_safe = max(raw_ref, -I_limit);
                        end
                        if abs(I_safe) < 0.1 && v_batt > obj.V_CV_Start
                            I_safe = 0;
                        end
                    elseif raw_ref > 0  % Discharging
                        if obj.CurrentState == obj.ST_ESS_RUN
                            if v_batt < obj.V_ESS_Limit, I_safe = 0; end
                        else
                            if v_batt < obj.V_UPS_Limit
                                factor = (v_batt - obj.V_Min_Limit) / (obj.V_UPS_Limit - obj.V_Min_Limit);
                                factor = max(0, min(1, factor));
                                I_safe = raw_ref * factor;
                            end
                        end
                    end
                    I_target = I_safe;
            end
            
            % Status flags
            status_flag = double((obj.CurrentState == obj.ST_UPS_RUN) || (obj.CurrentState == obj.ST_FAULT));
            if obj.CurrentState == obj.ST_INIT || obj.CurrentState == obj.ST_FAULT || obj.CurrentState == obj.ST_MANUAL
                mode_active = 0;
            end
            
            % =================================================================
            % [STAGE 4] Control Strategy Selection
            %   기본값: SPS (D1=0.5, D2=0.5, Phi_FF=0)
            %   TPS 모드이면 HDC 추론으로 덮어쓰기 (Override)
            % =================================================================
            D1_set = 0.5;        % Primary duty ratio (SPS default)
            D2_set = 0.5;        % Secondary duty ratio (SPS default)
            Phi_FF = 0;          % Phase feedforward [deg]
            is_hdc_active = false; %#ok<NASGU> % [CodeGen] 경로 보장
            is_hdc_active = false;
            
            switch Ctrl_Mode_Ext
                case 3  % ===== TPS: HDC 모델 기반 (또는 LUT) =====
                    
                    has_hdc = isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0;
                    has_lut = isfield(obj.HDC_Model_Data, 'D1_table');

                    % [CRITICAL ERROR CHECK]
                    if ~has_hdc && ~has_lut
                        error('>> [CTRL FAULT] TPS Mode(=3) Selected, BUT HDC/LUT Data is EMPTY! Check Sim Mode.');
                    end

                    if has_hdc
                        % =========================================================
                        % [Type A] HDC Model Inference (AI)
                        % =========================================================
                        
                        % [HDC Optimization] Caching Logic
                        % 입력 변화가 임계값보다 클 때만 HDC 모델 재실행
                        Vin_curr  = V_HV_f;
                        Vout_curr = V_LV_f;
                        Pref_curr = obj.I_ref_ramped * V_LV_f;
    
                        dV_in  = abs(Vin_curr - obj.Last_Vin_HDC);
                        dV_out = abs(Vout_curr - obj.Last_Vout_HDC);
                        dP_ref = abs(Pref_curr - obj.Last_Pref_HDC);
    
                        if dV_in > obj.HDC_Update_V_Th || ...
                           dV_out > obj.HDC_Update_V_Th || ...
                           dP_ref > obj.HDC_Update_P_Th
                            
                            % Run Inference
                            [d1_new, d2_new, phi_new] = hdc_inference_inline( ...
                                obj.HDC_Model_Data, Vin_curr, Vout_curr, Pref_curr);
                            
                            % Update Cache
                            obj.Cached_D1 = d1_new;
                            obj.Cached_D2 = d2_new;
                            obj.Cached_Phi = phi_new;
                            
                            obj.Last_Vin_HDC = Vin_curr;
                            obj.Last_Vout_HDC = Vout_curr;
                            obj.Last_Pref_HDC = Pref_curr;
                        end
                        
                        % Use Cached Values
                        d1_hdc = obj.Cached_D1;
                        d2_hdc = obj.Cached_D2;
                        phi_hdc = obj.Cached_Phi;
                        
                        is_hdc_active = true;
                        
                        % Safety Clamp & Override
                        D1_set = max(0.05, min(0.50, d1_hdc));
                        D2_set = max(0.05, min(0.50, d2_hdc));
                        phi_hdc = max(-0.48, min(0.48, phi_hdc));
                        
                        % HDC의 Phi를 피드포워드 위상으로 변환 (ratio → deg)
                        Phi_FF = phi_hdc * 360;

                    elseif has_lut
                        % =========================================================
                        % [Type B] LUT Interpolation (Table)
                        % =========================================================
                        
                        % 입력 클램핑 (LUT 범위 내)
                        Vin_in  = max(min(V_HV_f, max(obj.HDC_Model_Data.Vin_bp)), min(obj.HDC_Model_Data.Vin_bp));
                        Vout_in = max(min(V_LV_f, max(obj.HDC_Model_Data.Vout_bp)), min(obj.HDC_Model_Data.Vout_bp));
                        P_in    = max(min(obj.I_ref_ramped * V_LV_f, max(obj.HDC_Model_Data.P_bp)), min(obj.HDC_Model_Data.P_bp));
                        
                        % LUT 보간 실행
                        [d1_lut, d2_lut, phi_lut] = lut_inference_inline(obj.HDC_Model_Data, Vin_in, Vout_in, P_in);
                        
                        D1_set = d1_lut;
                        D2_set = d2_lut;
                        
                        % LUT Phi는 보통 ratio(0~0.5) 단위이므로 deg 변환
                        Phi_FF = phi_lut * 360;
                        
                        is_hdc_active = true;
                    end
                
                % case 1  % ===== EPS (미구현) =====
                    % TODO: D1 가변, D2 = 0.5 고정
                    % D1_set = ... ;
                    % Phi_FF = ... ;
                
                % case 2  % ===== DPS (미구현) =====
                    % TODO: D1 = D2 가변 (대칭)
                    % D1_set = ... ;
                    % D2_set = ... ;
                    % Phi_FF = ... ;
                
                otherwise  % ===== SPS (Default) =====
                    % D1_set = 0.5, D2_set = 0.5 유지
                    if obj.EnableFF && abs(V_HV_f) > 10
                        K_ff = (2 * obj.L_leakage * obj.Fs * 180) / V_HV_f;
                        Phi_FF = -1 * obj.I_ref_ramped * K_ff;
                    end
            end
            
            % Soft Start Gain (모든 모드 공통)
            if t_current < obj.SoftStartTime
                ss_gain = t_current / obj.SoftStartTime;
            else
                ss_gain = 1.0;
            end
            Phi_FF = Phi_FF * ss_gain;
            
            % =================================================================
            % [STAGE 5~6] PI Feedback + Signal Synthesis
            %   PI 제어기는 모드와 무관하게 항상 동작
            %   최종 위상 = PI + Feedforward
            % =================================================================
            err_ext = 0;
            phi_raw_unbounded = 0;
            excess = 0;
            phi_ext_deg = 0;
            
            if PWM_Enable && (obj.CurrentState ~= obj.ST_MANUAL)
                % Slew Rate Limiter
                if isinf(obj.MaxSlewRate)
                    obj.I_ref_ramped = I_target;
                else
                    max_step = obj.MaxSlewRate * Ts;
                    delta = I_target - obj.I_ref_ramped;
                    if abs(delta) > max_step
                        obj.I_ref_ramped = obj.I_ref_ramped + sign(delta) * max_step;
                    else
                        obj.I_ref_ramped = I_target;
                    end
                end
                
                % PI Controller
                raw_err = obj.I_ref_ramped - I_fb;
                
                % Deadzone
                if abs(raw_err) < obj.DeadZone
                    err_ext = 0;
                else
                    err_ext = raw_err - sign(raw_err) * obj.DeadZone;
                end
                
                % Gain Scheduling
                Kp_curr = Kp_ext;
                if abs(err_ext) < obj.GainSched_Threshold
                    Kp_curr = Kp_ext * obj.GainSched_Factor;
                end
                
                % PI Output + Feedforward
                Phi_PI = -1 * (Kp_curr * err_ext + Ki_ext * obj.Integrator_ext);
                phi_total_raw = Phi_PI + Phi_FF;
                
                % Saturation
                phi_saturated = max(-90, min(90, phi_total_raw));
                
                % Rate Limiter
                delta_phi = phi_saturated - obj.Prev_Phi_Output;
                max_phi_step = obj.MaxPhaseRate * Ts;
                if abs(delta_phi) > max_phi_step
                    phi_ext_deg = obj.Prev_Phi_Output + sign(delta_phi) * max_phi_step;
                else
                    phi_ext_deg = phi_saturated;
                end
                obj.Prev_Phi_Output = phi_ext_deg;
                
                % Integrator Update (Back-Calculation Anti-Windup)
                excess = phi_ext_deg - phi_total_raw;
                obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts - (excess * Ts);
                
                phi_raw_unbounded = phi_total_raw;
                
            elseif obj.CurrentState == obj.ST_MANUAL
                phi_ext_deg = Phi_Manual;
                D1_set = 0.5;  % Manual: SPS 고정
                D2_set = 0.5;
            else
                % PWM OFF: Reset
                obj.I_ref_ramped = 0;
                obj.Prev_Phi_Output = 0;
            end
            
            % =================================================================
            % [STAGE 7] Flux Balancing
            % =================================================================
            D_offset = 0;
            if PWM_Enable && obj.EnableFluxBalancing
                err_flux = -I_L_pri_DC;
                obj.Flux_Integrator = obj.Flux_Integrator + err_flux * Ts;
                max_integ = obj.Flux_Limit / obj.Flux_Ki;
                obj.Flux_Integrator = max(-max_integ, min(max_integ, obj.Flux_Integrator));
                D_raw = obj.Flux_Kp * err_flux + obj.Flux_Ki * obj.Flux_Integrator;
                D_offset = max(-obj.Flux_Limit, min(obj.Flux_Limit, D_raw));
            end
            
            % =================================================================
            % [STAGE 8] Phase Assignment — 통합 수식
            %   모든 모드에 대해 동일한 수식 사용:
            %     phi1 = 0                           (Primary Leg A, 기준)
            %     phi3 = D1 * 360                    (Primary Leg B)
            %     phi5 = phi_ext_deg                  (Secondary Leg A)
            %     phi7 = phi_ext_deg + D2 * 360       (Secondary Leg B)
            %
            %   SPS 일 때: D1=D2=0.5 → phi3=180, phi7=phi5+180 (기존과 동일)
            %   TPS 일 때: D1,D2 가변 → 각각 HDC 결정값 사용
            % =================================================================
            D1_final = D1_set + D_offset;    % Flux 보정 적용
            D2_final = D2_set;
            
            if PWM_Enable
                phi1_deg = 0;
                phi3_deg = D1_final * 360;
                phi5_deg = phi_ext_deg;
                phi7_deg = phi_ext_deg + D2_final * 360;
                
                D1_out = D1_final;
                D2_out = D2_final;
            else
                phi1_deg = 0;
                phi3_deg = 0;
                phi5_deg = 0;
                phi7_deg = 0;
                D1_out = 0;
                D2_out = 0;
            end
            
            % =================================================================
            % [STAGE 9] Output Assignment
            % =================================================================
            I_cmd_out = obj.I_ref_ramped;
            phi_ext_out = phi_ext_deg;
            integrator_out = obj.Integrator_ext;
            err_out = err_ext;
            phi_raw_out = phi_raw_unbounded;
            excess_out = excess;
            ifb_out = I_fb;
            
            obj.PreviousMode = obj.CurrentState;
        end

        function resetImpl(obj)
            % 리셋 신호가 들어오거나 재시작 시
            obj.Integrator_ext = 0;
            obj.Integrator_v = 0;
            obj.Flux_Integrator = 0; % [NEW]
            obj.CurrentState = obj.ST_INIT;
            obj.PreviousMode = obj.ST_INIT;

            obj.Prev_Phi_Output = 0;
            
            obj.I_Buffer = zeros(125,1);
            obj.BufferIdx = 1;
            obj.I_Sum = 0;
            
            % [NEW] Flux Balancing Reset
            obj.I_L_pri_Buffer = zeros(125,1);
            obj.I_L_pri_Sum = 0;
            
            % [NEW] Voltage Buffers Reset
            obj.V_HV_Buffer = zeros(125,1);
            obj.V_HV_Sum = 0;
            obj.V_LV_Buffer = zeros(125,1);
            obj.V_LV_Sum = 0;
            
            obj.ControlCheckDone = 0;
            
            % [NEW] Time Counter Reset
            obj.TickCount = 0;
            
            % [HDC Cache Reset]
            obj.Last_Vin_HDC = -9999; 
            obj.Last_Vout_HDC = -9999;
            obj.Last_Pref_HDC = -9999;
            obj.Cached_D1 = 0.5;
            obj.Cached_D2 = 0.5;
            obj.Cached_Phi = 0;
        end
        
        % --- 입력 개수 지정 ---
        function num = getNumInputsImpl(~)
            num = 13; % 12 기존 + Ctrl_Mode_Ext
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
                case 'CurrentState'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'PreviousMode'
                    sz = [1 1]; dt = 'double'; cp = false;

                case 'I_ref_ramped'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_HV_Filtered'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_LV_Filtered'
                    sz = [1 1]; dt = 'double'; cp = false;
                % case 'FeedforwardUpdateTimer'
                %    sz = [1 1]; dt = 'double'; cp = false;
                case 'Last_Phi_FF'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Prev_Phi_Output'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'I_Buffer'
                    sz = [125 1]; dt = 'double'; cp = false;
                case 'BufferIdx'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'I_Sum'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'ControlCheckDone'
                    sz = [1 1]; dt = 'double'; cp = false;
                % [NEW] Flux Balancing States
                case 'I_L_pri_Buffer'
                    sz = [125 1]; dt = 'double'; cp = false;
                case 'I_L_pri_Sum'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Flux_Integrator'
                    sz = [1 1]; dt = 'double'; cp = false;
                % [NEW] Voltage Moving Average States
                case 'V_HV_Buffer'
                    sz = [125 1]; dt = 'double'; cp = false;
                case 'V_HV_Sum'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'V_LV_Buffer'
                    sz = [125 1]; dt = 'double'; cp = false;
                case 'V_LV_Sum'
                    sz = [1 1]; dt = 'double'; cp = false;
                % [NEW] Time Counter
                case 'TickCount'
                    sz = [1 1]; dt = 'double'; cp = false;
                
                % [HDC Cache States]
                case 'Last_Vin_HDC'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Last_Vout_HDC'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Last_Pref_HDC'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'Cached_D1'
                     sz = [1 1]; dt = 'double'; cp = false;
                case 'Cached_D2'
                     sz = [1 1]; dt = 'double'; cp = false;
                case 'Cached_Phi'
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

        % --- [NEW] 입력 포트 이름 명시 ---
        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13] = getInputNamesImpl(~)
            n1 = 'V_HV';        n2 = 'I_HV';
            n3 = 'V_LV';        n4 = 'I_LV';
            n5 = 'I_L_pri';     n6 = 'I_ess_cmd';
            n7 = 'Kp_ext';      n8 = 'Ki_ext';
            n9 = 'Kpv';         n10 = 'Kiv';
            n11 = 'Mode_Select'; n12 = 'Phi_Manual';
            n13 = 'Ctrl_Mode';  % 0:SPS, 1:DPS(Reserved), 2:EPS(Reserved), 3:TPS/HDC
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

        methods (Static, Access = protected)
            % --- [NEW] Property Groups (UI Layout) ---
            function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Converter Controller (Full Bridge)', ...
                'Text', sprintf(['DAB 컨버터의 위상 천이(SPS/TPS) 및 모드 자동 전환 제어기입니다.\n', ...
                                 'ESS(정상)/UPS(정전) 모드 자동 전환, 배터리 보호(BMS), 자속 평형(Flux Balancing) 기능을 수행합니다.\n', ...
                                 'Ctrl_Mode_Ext=3 으로 HDC 기반 TPS 제어가 가능합니다.']));
        end

        function groups = getPropertyGroupsImpl
            % 1. 시스템 설정
            groupSys = matlab.system.display.Section(...
                'Title', '1. 시스템 기본 설정 (Hardware Spec)', ...
                'PropertyList', {'Fs', 'Ts_ctrl', 'L_leakage', 'SPS_Inner_Phase', 'InitDelay'});

            % 2. 운전 모드 및 전압/전류 제한
            groupMode = matlab.system.display.Section(...
                'Title', '2. 운전 모드 및 목표값 (Operation Targets)', ...
                'PropertyList', {'V_HV_Ref', 'V_HV_Band', 'I_ESS_Max', 'I_UPS_Max'});

            % 3. 배터리 관리 시스템 (BMS)
            groupBMS = matlab.system.display.Section(...
                'Title', '3. 배터리 보호 설정 (Smart BMS)', ...
                'PropertyList', {'V_Pack_Max', 'V_CV_Start', 'K_CV', 'V_ESS_Limit', 'V_UPS_Limit', 'V_Min_Limit'});

            % 4. 고급 제어 및 안정화
            groupAdv = matlab.system.display.Section(...
                'Title', '4. 고급 제어 기능 (Advanced Control)', ...
                'PropertyList', {'EnableFF', 'SoftStartTime', 'MaxSlewRate', 'MaxPhaseRate', ...
                                 'DeadZone', 'GainSched_Threshold', 'GainSched_Factor'});

            % 5. 자속 평형 제어
            groupFlux = matlab.system.display.Section(...
                'Title', '5. 변압기 자속 평형 (Flux Balancing)', ...
                'PropertyList', {'EnableFluxBalancing', 'Flux_Kp', 'Flux_Ki', 'Flux_Limit'});

             % 6. HDC/TPS 제어 설정
             groupHDC = matlab.system.display.Section(...
                 'Title', '6. HDC/TPS 제어 (AI Phase Shift Control)', ...
                 'PropertyList', {'HDC_Data', 'HDC_Update_V_Th', 'HDC_Update_P_Th'});

            groups = [groupSys, groupMode, groupBMS, groupAdv, groupFlux, groupHDC];
        end
    end
end

% =====================================================================
% HDC Model Auto-Loader (Called from setupImpl)
% =====================================================================
function HDC = load_hdc_model(model_path)
    % LOAD_HDC_MODEL — .mat 파일에서 HDC 모델을 로드하고 Simulink 호환 구조체로 변환
    %
    % 입력: model_path — 'HDC/Supervised/Supervised_Model_Results.mat' 등
    % 출력: HDC struct (flat matrices, no cell arrays)
    %
    % 파일이 없거나 로드 실패 시 빈 struct 반환 → Guard에 의해 SPS Fallback
    
    HDC = struct();  % Default: empty (SPS fallback)
    
    if ~exist(model_path, 'file')
        error('>> [HDC] Model file NOT FOUND at: %s\nCheck your path. Currently in: %s', model_path, pwd);
    end
    
    try
        loaded = load(model_path);
        
        % Supervised_Model_Results.mat structure: results.hdc_model
        if isfield(loaded, 'results') && isfield(loaded.results, 'hdc_model')
            src = loaded.results.hdc_model;
        elseif isfield(loaded, 'lut_data')
             % Phase 2 Generated LUT
             HDC = loaded.lut_data;
             return;
        elseif isfield(loaded, 'HDC')
            % Already deployed file (deploy_supervised_lut)
            HDC = loaded.HDC;
            return;
        else
            error('>> [HDC] Invalid model format! Missing results.hdc_model structure.');
        end
        
        % --- Cell Array → 3D Matrix 변환 (Simulink Coder 호환) ---
        HDC.D        = double(src.D);
        HDC.N_levels = double(src.N_levels);
        HDC.N_inputs = double(src.N_inputs);
        HDC.N_bins   = double(src.N_output_bins);
        
        % Base HVs [N_inputs x D]
        HDC.BaseHVs = double(src.base_hvs);
        
        % Level HVs: Cell {i}[N_levels x D] → 3D [N_inputs x N_levels x D]
        HDC.LevelHVs = zeros(HDC.N_inputs, HDC.N_levels, HDC.D);
        for i = 1:HDC.N_inputs
            HDC.LevelHVs(i, :, :) = double(src.level_hvs{i});
        end
        
        % Prototypes: Cell {k}[N_bins x D] → 3D [3 x N_bins x D]
        HDC.Prototypes = zeros(3, HDC.N_bins, HDC.D);
        for k = 1:3
            HDC.Prototypes(k, :, :) = double(src.output_prototypes{k});
        end
        
        % Permutation Indices
        HDC.PermIdx1 = double(src.perm_idx);
        HDC.PermIdx2 = double(src.perm_idx2);
        
        % Encoding Parameters
        HDC.Param_n       = double(src.encode_params.n);
        HDC.Param_Prated  = double(src.encode_params.P_rated);
        HDC.Param_VinNom  = double(src.encode_params.Vin_nom);
        HDC.Param_VoutNom = double(src.encode_params.Vout_nom);
        HDC.Param_Lnom    = double(src.encode_params.L_nom);
        HDC.Param_fs      = double(src.encode_params.fs);
        
        % Temperature
        if isfield(src, 'temperatures')
            HDC.Temps = double(src.temperatures);
        else
            HDC.Temps = [src.temperature, src.temperature, src.temperature];
        end
        
        % Output Ranges & Pre-calculated Bin Values
        HDC.OutputRanges = double(src.output_ranges);
        HDC.BinValues = zeros(3, HDC.N_bins);
        for k = 1:3
            HDC.BinValues(k, :) = linspace(HDC.OutputRanges(k,1), HDC.OutputRanges(k,2), HDC.N_bins);
        end
        
    catch ME
        error('>> [HDC] Exception while loading model: %s', ME.message);
    end
end
% =====================================================================
% HDC Inference Function (Inlined for Code Generation Compatibility)
% =====================================================================
function [D1, D2, Phi] = hdc_inference_inline(HDC, Vin, Vout, Pref)
    % HDC_INFERENCE_INLINE — HDC 모델 추론 (Simulink Coder 호환)
    %
    % 입력: Vin (V), Vout (V), Pref (W)
    % 출력: D1, D2 (duty ratio 0~0.5), Phi (phase ratio -0.5~0.5)
    %
    % HDC 구조체 필드 요구사항:
    %   HDC.D, HDC.N_levels, HDC.N_bins, HDC.N_inputs
    %   HDC.BaseHVs [N_inputs x D]
    %   HDC.LevelHVs [N_inputs x N_levels x D]
    %   HDC.Prototypes [3 x N_bins x D]
    %   HDC.PermIdx1 [1 x D], HDC.PermIdx2 [1 x D]
    %   HDC.Temps [1 x 3], HDC.BinValues [3 x N_bins]
    %   HDC.Param_n, Param_Prated, Param_VinNom, Param_Lnom, Param_fs
    
    % --- Guard: HDC 모델이 비어있으면 SPS 기본값 반환 ---
    if ~isfield(HDC, 'D') || HDC.D == 0
        D1 = 0.5; D2 = 0.5; Phi = 0;
        return;
    end
    
    D_dim = HDC.D;
    N_levels = HDC.N_levels;
    
    % --- Encoding Parameters ---
    n_tr    = HDC.Param_n;
    P_rated = HDC.Param_Prated;
    Vin_nom = HDC.Param_VinNom;
    L_nom   = HDC.Param_Lnom;
    fs      = HDC.Param_fs;
    
    % --- Feature Extraction (12 physics-informed features) ---
    M        = (n_tr * Vout) / max(Vin, 1);
    P_norm   = Pref / P_rated;
    V_ratio  = Vin / Vin_nom;
    P_abs_norm = abs(Pref) / P_rated;
    P_sign   = (sign(Pref) + 1) / 2;
    
    % Cross-Domain Features
    Z_L = 2 * pi * fs * L_nom;
    if abs(Pref) > 5
        Z_load = Vin^2 / abs(Pref);
        Z_norm = Z_L / Z_load;
        R_load = Vout^2 / abs(Pref);
        Load_Z = R_load / 100;
    else
        Z_norm = 0;
        Load_Z = 1.0;
    end
    Z_norm = max(0, min(1, Z_norm / 2.0));
    Load_Z = max(0, min(1, Load_Z));
    
    VP_cross = (M - 1.0) * P_norm;
    VP_cross = max(0, min(1, (VP_cross + 2) / 4));
    
    B_est = Vin / (4 * 20 * 3e-4 * fs);
    B_est = max(0, min(1, B_est / 0.35));
    
    % Feature Vector [12 x 1], all clamped to [0, 1]
    features = zeros(12, 1);
    features(1)  = max(0, min(1, (Vin - 180) / (460 - 180)));
    features(2)  = max(0, min(1, (Vout - 65) / (115 - 65)));
    features(3)  = max(0, min(1, (Pref + 1100) / 2200));
    features(4)  = max(0, min(1, (M - 0.3) / 2.5));
    features(5)  = max(0, min(1, (P_norm + 1) / 2));
    features(6)  = max(0, min(1, V_ratio / 1.5));
    features(7)  = max(0, min(1, P_abs_norm));
    features(8)  = P_sign;
    features(9)  = Z_norm;
    features(10) = VP_cross;
    features(11) = Load_Z;
    features(12) = B_est;
    
    % --- Quantize to level indices ---
    lvl_indices = zeros(12, 1);
    for i = 1:12
        idx_raw = floor(features(i) * (N_levels - 1)) + 1;
        if idx_raw < 1, idx_raw = 1; end
        if idx_raw > N_levels, idx_raw = N_levels; end
        lvl_indices(i) = idx_raw;
    end
    
    % --- Bind & Bundle ---
    hv = zeros(1, D_dim);
    for i = 1:12
        lvl_vec = reshape(HDC.LevelHVs(i, lvl_indices(i), :), 1, D_dim);
        base_vec = HDC.BaseHVs(i, :);
        hv = hv + (base_vec .* lvl_vec);
    end
    
    % --- N-gram Interactions ---
    % 1. Vin(1) * Vout(2)
    h1 = HDC.BaseHVs(1,:) .* reshape(HDC.LevelHVs(1, lvl_indices(1), :), 1, D_dim);
    h2 = HDC.BaseHVs(2,:) .* reshape(HDC.LevelHVs(2, lvl_indices(2), :), 1, D_dim);
    hv = hv + 0.5 * (h1 .* h2(HDC.PermIdx1));
    
    % 2. Pref(3) * M(4)
    h3 = HDC.BaseHVs(3,:) .* reshape(HDC.LevelHVs(3, lvl_indices(3), :), 1, D_dim);
    h4 = HDC.BaseHVs(4,:) .* reshape(HDC.LevelHVs(4, lvl_indices(4), :), 1, D_dim);
    hv = hv + 0.5 * (h3 .* h4(HDC.PermIdx1));
    
    % 3. Z(9) * Load(11) with PermIdx2
    h9  = HDC.BaseHVs(9,:)  .* reshape(HDC.LevelHVs(9,  lvl_indices(9),  :), 1, D_dim);
    h11 = HDC.BaseHVs(11,:) .* reshape(HDC.LevelHVs(11, lvl_indices(11), :), 1, D_dim);
    hv = hv + 0.3 * (h9 .* h11(HDC.PermIdx2));
    
    % --- Bipolarize ---
    hv = sign(hv);
    hv(hv == 0) = 1;
    
    % --- Readout: Softmax weighted average over bins ---
    D1  = readout_hdc_var(hv, 1, HDC);
    D2  = readout_hdc_var(hv, 2, HDC);
    Phi = readout_hdc_var(hv, 3, HDC);
end

function val = readout_hdc_var(hv, k, HDC)
    % READOUT_HDC_VAR — Single output variable readout via Softmax
    D_dim  = HDC.D;
    N_bins = HDC.N_bins;
    
    protos = reshape(HDC.Prototypes(k, :, :), N_bins, D_dim);
    sims = protos * hv';
    
    % Prototype norms
    norms = sqrt(sum(protos.^2, 2));
    norms(norms < 1e-10) = 1;
    sims = sims ./ norms;
    
    % Softmax with Temperature
    T = HDC.Temps(k);
    scaled = sims / T;
    max_s = max(scaled);
    exps = exp(scaled - max_s);
    weights = exps / (sum(exps) + 1e-10);
    
    % Weighted sum
    bins = HDC.BinValues(k, :)';
    val = sum(weights .* bins);
end

% =====================================================================
% LUT Inference Function (Trilinear Interpolation)
% =====================================================================
function [D1, D2, Phi] = lut_inference_inline(LUT, Vin, Vout, Pref)
    % interpn 지원 (Code Generation 호환성: 'linear', 'nearest' 등 가능)
    % LUT 구조체: Vin_bp, Vout_bp, P_bp (Vectors), D1_table (3D Array)
    
    % interpn(X1, X2, X3, V, x1q, x2q, x3q)
    
    D1  = interpn(LUT.Vin_bp, LUT.Vout_bp, LUT.P_bp, LUT.D1_table,  Vin, Vout, Pref, 'linear', 0.5);
    D2  = interpn(LUT.Vin_bp, LUT.Vout_bp, LUT.P_bp, LUT.D2_table,  Vin, Vout, Pref, 'linear', 0.5);
    Phi = interpn(LUT.Vin_bp, LUT.Vout_bp, LUT.P_bp, LUT.Phi_table, Vin, Vout, Pref, 'linear', 0);
    
    % Safety Clamp
    D1  = max(0.05, min(0.50, D1));
    D2  = max(0.05, min(0.50, D2));
    Phi = max(-0.48, min(0.48, Phi));
end