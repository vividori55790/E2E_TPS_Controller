classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System
    % DAB 컨버터 자동 모드 전환 (ESS <-> UPS) 및 SPS 위상 제어기
    %
    % 개선사항 (v3.0 - SPS Mode):
    %   - SPS (Single Phase Shift) 제어만 사용
    %   - 위상 출력을 360도 각도로 출력 (시간 대신)
    %   - D1, D2는 SPS 고정값 (50%)으로 출력
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_L_pri, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual
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
        
        % [주석 처리됨] HDC Control Parameters
        % TODO: 사용자가 복구시키라고 할 때까지 주석으로 유지할 것
        % EnableHDC = false; 
        % HDC_Model = struct(); 
        
        % [NEW] Feedforward & System Params
        L_leakage = 90e-6;      % 변압기 누설 인덕턴스 (H)
        EnableFF = true;       % 피드포워드(FF) 제어 활성화
        
        % [NEW] Reference Slew Rate
        MaxSlewRate = inf;      % 전류 지령 변화율 제한 (A/s) [inf: OFF]
        
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
            
            % [NEW] Time Counter Init
            obj.TickCount = 0;
        end

    function [phi1_deg, phi3_deg, phi5_deg, phi7_deg, mode_active, status_flag, ...
                  I_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out, PWM_Enable, ...
                  err_out, phi_raw_out, excess_out, ifb_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L_pri, I_ess_cmd, Kp_ext, Ki_ext, Kpv, Kiv, Mode_Select, Phi_Manual)
            
            % --- 1. 파라미터 계산 ---
            Ts = obj.Ts_ctrl;
            
            % [MODIFIED] Time calculation using TickCount (Code Generation compatible)
            t_current = double(obj.TickCount) * Ts;
            obj.TickCount = obj.TickCount + 1;  % Increment tick counter
            
            % =====================================================================
            % [STEP 1] Sensing & Filtering
            % - 125-sample Moving Average (runs in both MATLAB and Code Gen modes)
            % - Debug fprintf only in MATLAB mode
            % =====================================================================
            
            % [DEBUG] Parameter Check (Only in MATLAB mode - fprintf not supported in embedded)
            if coder.target('MATLAB')
                if obj.ControlCheckDone == 0 && t_current >= 0.005
                     fprintf('>> [CTRL CHECK] Time=%.6f, Kp_ext=%.4f, Ki_ext=%.4f\n', t_current, Kp_ext, Ki_ext);
                     obj.ControlCheckDone = 1;
                end
            end
            
            % 1.1 V_HV Filtering (Moving Average)
            old_vhv = obj.V_HV_Buffer(obj.BufferIdx);
            obj.V_HV_Sum = obj.V_HV_Sum - old_vhv + V_HV;
            obj.V_HV_Buffer(obj.BufferIdx) = V_HV;
            obj.V_HV_Filtered = obj.V_HV_Sum / 125;
            
            % 1.2 V_LV Filtering (Moving Average)
            old_vlv = obj.V_LV_Buffer(obj.BufferIdx);
            obj.V_LV_Sum = obj.V_LV_Sum - old_vlv + V_LV;
            obj.V_LV_Buffer(obj.BufferIdx) = V_LV;
            obj.V_LV_Filtered = obj.V_LV_Sum / 125;
            
            V_HV_Ctrl = obj.V_HV_Filtered;
            V_LV_Ctrl = obj.V_LV_Filtered;

            % 1.3 I_LV Current Filtering (Fast MA)
            old_ilv = obj.I_Buffer(obj.BufferIdx);
            obj.I_Sum = obj.I_Sum - old_ilv + I_LV;
            obj.I_Buffer(obj.BufferIdx) = I_LV;
            
            % 1.4 I_L_pri Filtering (Flux Balancing) - Same Index
            old_pri = obj.I_L_pri_Buffer(obj.BufferIdx);
            obj.I_L_pri_Sum = obj.I_L_pri_Sum - old_pri + I_L_pri;
            obj.I_L_pri_Buffer(obj.BufferIdx) = I_L_pri;

            % Buffer Index Update (Circular)
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > 125, obj.BufferIdx = 1; end
            
            I_fb = obj.I_Sum / 125;      % Feedback Current
            I_L_pri_DC = obj.I_L_pri_Sum / 125; % DC Offset Current

            % =====================================================================
            % [STEP 2] State Transition Logic
            % =====================================================================
            % Default: Stay in current state
            NextState = obj.CurrentState;
            
            % Global Transitions (Highest Priority)
            if t_current < obj.InitDelay
                NextState = obj.ST_INIT;
            elseif Mode_Select == 1
                NextState = obj.ST_MANUAL;
            elseif V_LV_Ctrl < obj.V_Min_Limit
                 % [Safety] Battery Undervoltage -> FAULT (Shutdown)
                 NextState = obj.ST_FAULT; 
            else
                % State-Specific Transitions
                switch obj.CurrentState
                    case obj.ST_INIT
                        % Init finished (t > 0.001) -> Decide Startup Mode
                        if (V_HV_Ctrl < (obj.V_HV_Ref - obj.V_HV_Band)) || (V_HV_Ctrl > (obj.V_HV_Ref + obj.V_HV_Band))
                            NextState = obj.ST_UPS_RUN;
                        else
                            NextState = obj.ST_ESS_RUN;
                        end
                        
                    case obj.ST_ESS_RUN
                        % ESS -> UPS (Grid Failure Logic)
                        if (V_HV_Ctrl < (obj.V_HV_Ref - obj.V_HV_Band)) || (V_HV_Ctrl > (obj.V_HV_Ref + obj.V_HV_Band))
                            NextState = obj.ST_UPS_RUN;
                        end
                        
                    case obj.ST_UPS_RUN
                        % UPS -> ESS (Grid Restoration)
                        % Hysteresis can be added here if needed.
                        if (V_HV_Ctrl >= (obj.V_HV_Ref - obj.V_HV_Band)) && (V_HV_Ctrl <= (obj.V_HV_Ref + obj.V_HV_Band))
                             NextState = obj.ST_ESS_RUN;
                        end
                        
                    case obj.ST_MANUAL
                        % Exiting Manual Mode -> Re-evaluate based on Grid
                         if Mode_Select ~= 1
                            if (V_HV_Ctrl < (obj.V_HV_Ref - obj.V_HV_Band)) || (V_HV_Ctrl > (obj.V_HV_Ref + obj.V_HV_Band))
                                NextState = obj.ST_UPS_RUN;
                            else
                                NextState = obj.ST_ESS_RUN;
                            end
                         end
                         
                    case obj.ST_FAULT
                         % Latch Fault. Reset required to exit (Not implemented yet, or use Reset block)
                         % If voltage recovers, maybe recover? For safety, stick to Fault.
                         if V_LV_Ctrl > (obj.V_Min_Limit + 5.0) % Hysteresis 5V
                             NextState = obj.ST_INIT; % Restart
                         end
                end
            end
            
            % Update State
            obj.CurrentState = NextState;
            
            % =====================================================================
            % [STEP 3] State Action & Reference Generation
            % =====================================================================
            PWM_Enable = 0;
            I_target = 0;
            phi_ext_deg = 0;
            mode_active = 0; % [FIX] Default Initialization
            status_flag = 0; % [FIX] Default Initialization
            
            switch obj.CurrentState
                case obj.ST_INIT
                    PWM_Enable = 0;
                    I_target = 0;
                    obj.Integrator_ext = 0;
                    obj.Integrator_v = 0;
                    
                case obj.ST_FAULT
                    PWM_Enable = 0; % Hard Shutdown
                    I_target = 0;
                    
                case obj.ST_MANUAL
                    PWM_Enable = 1;
                    phi_ext_deg = Phi_Manual;
                    % Freeze Integrators or Reset
                    % obj.Integrator_ext = 0; 
                    
                case {obj.ST_ESS_RUN, obj.ST_UPS_RUN}
                    PWM_Enable = 1;
                    
                    % --- Reference Generation ---
                    if obj.CurrentState == obj.ST_ESS_RUN
                        % ESS: Direct Current Command
                        raw_ref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, I_ess_cmd));
                        mode_active = 3; % ESS
                    else
                        % UPS: Voltage Loop -> Current Command
                        mode_active = 1; % UPS
                        err_v = obj.V_HV_Ref - V_HV_Ctrl;
                        temp_int_v = obj.Integrator_v + err_v * Ts;
                        
                        % Anti-Windup (Clamping)
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
                    
                    % --- BMS / Safety Limiter (Common) ---
                    I_safe = raw_ref;
                    v_batt = V_LV_Ctrl;
                    
                    if raw_ref < 0  % Charging
                        if v_batt >= obj.V_Pack_Max
                            I_safe = 0; % Overvoltage Protection
                        elseif v_batt >= obj.V_CV_Start
                            % CV Mode
                            I_limit = (obj.V_Pack_Max - v_batt) * obj.K_CV;
                            I_safe = max(raw_ref, -I_limit);
                        end
                        % Full Charge Cutoff
                        if abs(I_safe) < 0.1 && v_batt > obj.V_CV_Start
                             I_safe = 0; 
                        end
                        
                    elseif raw_ref > 0 % Discharging
                         if obj.CurrentState == obj.ST_ESS_RUN
                             % ESS Limit
                             if v_batt < obj.V_ESS_Limit
                                 I_safe = 0; 
                             end
                         else
                             % UPS Limit (Derating)
                             if v_batt < obj.V_UPS_Limit
                                 factor = (v_batt - obj.V_Min_Limit) / (obj.V_UPS_Limit - obj.V_Min_Limit);
                                 factor = max(0, min(1, factor));
                                 I_safe = raw_ref * factor;
                             end
                             % Note: < V_Min_Limit handled in Transition to FAULT
                         end
                    end
                    I_target = I_safe;
            end

            % =====================================================================
            % [STEP 4] Core Control Loop (Executed only if PWM_Enable & Not Manual)
            % =====================================================================
            
            % Default Outputs
            status_flag = double((obj.CurrentState == obj.ST_UPS_RUN) || (obj.CurrentState == obj.ST_FAULT));
            % Use saved mode for explicit 'mode_active' if not set in switch (init/fault)
            if obj.CurrentState == obj.ST_INIT || obj.CurrentState == obj.ST_FAULT || obj.CurrentState == obj.ST_MANUAL
                mode_active = 0; 
            end
            
            if PWM_Enable && (obj.CurrentState ~= obj.ST_MANUAL)
                % 4.1 Slew Rate Limiter
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
                
                % 4.2 Current PI Controller
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
                
                % PI Output
                phi_pi = -1 * (Kp_curr * err_ext + Ki_ext * obj.Integrator_ext);
                
                % 4.3 Feedforward (Soft Start)
                phi_ff = 0;
                if obj.EnableFF && abs(V_HV_Ctrl) > 10
                     K_ff = (2 * obj.L_leakage * obj.Fs * 180) / V_HV_Ctrl;
                     phi_ff_val = -1 * obj.I_ref_ramped * K_ff;
                     
                     % Soft Start
                     if t_current < obj.SoftStartTime
                         gain = t_current / obj.SoftStartTime;
                         phi_ff = phi_ff_val * gain;
                     else
                         phi_ff = phi_ff_val;
                     end
                end
                
                % 4.4 Saturation & Anti-Windup
                phi_total_raw = phi_pi + phi_ff;
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
                
                % Update Integrator (Back-Calculation)
                excess = phi_ext_deg - phi_total_raw;
                K_aw = 1.0; 
                obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts - (K_aw * excess * Ts);
                
                % Debug values
                phi_raw_unbounded = phi_total_raw;
                
            else
                % PWM Disabled or Manual: Reset internal ramps/vars if needed
                if obj.CurrentState ~= obj.ST_MANUAL
                    obj.I_ref_ramped = 0;
                    obj.Prev_Phi_Output = 0;
                end
                err_ext = 0;
                phi_raw_unbounded = 0;
                excess = 0;
            end
            
            % =====================================================================
            % [STEP 5] Flux Balancing & Final Output
            % =====================================================================
            D_offset = 0;
            if PWM_Enable && obj.EnableFluxBalancing
                 % Flux Control (regulate DC current to 0)
                 err_flux = -I_L_pri_DC; 
                 obj.Flux_Integrator = obj.Flux_Integrator + err_flux * Ts;
                 
                 % Flux Anti-windup
                 max_integ = obj.Flux_Limit / obj.Flux_Ki;
                 obj.Flux_Integrator = max(-max_integ, min(max_integ, obj.Flux_Integrator));
                 
                 D_raw = obj.Flux_Kp * err_flux + obj.Flux_Ki * obj.Flux_Integrator;
                 D_offset = max(-obj.Flux_Limit, min(obj.Flux_Limit, D_raw));
            end
            
            D1_out = 0.5 + D_offset;
            D2_out = 0.5;
            
            % Output assignment
            I_cmd_out = obj.I_ref_ramped;
            phi_ext_out = phi_ext_deg;
            integrator_out = obj.Integrator_ext;
            
            if PWM_Enable == 0
                D1_out = 0; D2_out = 0;
            end
            
            % Phase Calculation (SPS)
            phi_int_deg = obj.SPS_Inner_Phase;
            phi1_deg = 0;
            phi3_deg = phi_int_deg;
            phi5_deg = phi_ext_deg;
            phi7_deg = phi_ext_deg + phi_int_deg;
            
            % Debug Signals
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

        methods (Static, Access = protected)
            % --- [NEW] Property Groups (UI Layout) ---
            function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Converter Controller (Full Bridge)', ...
                'Text', sprintf(['DAB 컨버터의 위상 천이(SPS) 및 모드 자동 전환 제어기입니다.\n', ...
                                 'ESS(정상)/UPS(정전) 모드 자동 전환, 배터리 보호(BMS), 자속 평형(Flux Balancing) 기능을 수행합니다.']));
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

            groups = [groupSys, groupMode, groupBMS, groupAdv, groupFlux];
        end
    end
end