classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System  (v9.3 — Direct File Load HDC + PWM + RC Compensation)
    % DAB 컨버터 전력 추종 제어기 + PWM 신호 생성기 통합
    %
    % 아키텍처 (v9.3):
    %   1. Sensing & Filtering
    %   2. Control Strategy Selection (SPS/EPS/HDC)
    %   3. PI Feedback + Periodic Error Compensation(RC) + FF
    %      * 불감대(DeadZone)를 전력(전류) 지령치의 % 비율로 적용
    %      * 스위칭 주기에 따른 주기적 정상상태 오차 보정 알고리즘 선택적 적용
    %   4. Flux Balancing
    %   5. Phase Assignment
    %   6. PWM Generation
    %
    % =========================================================================
    % 1. Properties
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Ts_ctrl = 1e-7;         % 제어 샘플링 시간 (Ts) [sec]
        SPS_Inner_Phase = 180;  % [SPS] 내부 위상 고정값 (180도)
        L_leakage = 90e-6;      % 변압기 누설 인덕턴스 (H)
        n_transformer = 4;      % 변압기 턴비 (V_HV_nom / V_LV_nom = 380/95)
        EnableFF = true;        % 피드포워드(FF) 제어 활성화
        MaxSlewRate = 50000;    % 최종 목표 전력 지령 변화율 제한 [W/s]
        HDC_Update_V_Th = 2.0;  % HDC 재실행 전압 변화 임계값 (V)
        HDC_Update_P_Th = 50.0; % HDC 재실행 전력 변화 임계값 (W)
        
        % [추가] 주기적 정상상태 오차 보정 (Repetitive Control) 설정
        EnablePeriodicComp = false; % 주기적 오차 보정 알고리즘 활성화 여부
        
        GainSched_Threshold = 2.0;
        GainSched_Factor = 1.0;
        SoftStartTime = 0.2;    % 초기 기동 Soft Start 시간 (sec)
        MaxPhaseRate = 30000;   % 위상 변화율 제한 (deg/s)
        EnableFluxBalancing = true;
        Flux_Kp = 0.005;
        Flux_Ki = 0.05;
        Flux_Limit = 0.02;
        % [PWM] 게이트 드라이버 출력 전압 레벨
        V_Gate_ON = 18;         % ON 상태 전압 (V)
        V_Gate_OFF = -4;        % OFF 상태 전압 (V)
    end
    
    properties % Tunable properties
        DeadZone_Percent = 1.0;     % 전력 지령치 기준 제어 불감대 비율 [%] (ex: 1 = 1%)
        PeriodicComp_Gain = 0.05;   % 주기적 오차 보정 게인 (K_rc)
    end

    properties(Access = private)
        HDC_Model_Data = struct();
    end

    % =========================================================================
    % 2. Internal States
    % =========================================================================
    properties(DiscreteState)
        Integrator_ext;     I_ref_ramped;
        V_HV_Filtered;      V_LV_Filtered;
        V_HV_Buffer;        V_HV_Sum;
        V_LV_Buffer;        V_LV_Sum;
        Prev_Phi_Output;
        I_Buffer;            BufferIdx;           I_Sum;
        I_L_pri_Buffer;      I_L_pri_Sum;         Flux_Integrator;
        TickCount;
        PhaseCounter;
        
        % [추가] 주기적 오차 보정(Repetitive Control)용 버퍼
        RC_Buffer;
        RC_Idx;
        
        % HDC 추론 캐시 데이터
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
            obj.Integrator_ext = 0; obj.Flux_Integrator = 0;
            obj.I_ref_ramped = 0; obj.V_HV_Filtered = 0; obj.V_LV_Filtered = 0;
            obj.Prev_Phi_Output = 0;
            obj.I_Buffer = zeros(125,1); obj.BufferIdx = 1; obj.I_Sum = 0;
            obj.I_L_pri_Buffer = zeros(125,1); obj.I_L_pri_Sum = 0;
            obj.V_HV_Buffer = zeros(125,1); obj.V_HV_Sum = 0;
            obj.V_LV_Buffer = zeros(125,1); obj.V_LV_Sum = 0;
            obj.TickCount = 0;
            obj.PhaseCounter = 0;
            
            % 주기적 오차 보정 버퍼 (80kHz, 10MHz 샘플링 -> 1주기 125샘플)
            obj.RC_Buffer = zeros(125,1);
            obj.RC_Idx = 1;
            
            % HDC 상태 초기화
            obj.Last_Vin_HDC = 0;
            obj.Last_Vout_HDC = 0;
            obj.Last_Pref_HDC = 0;
            obj.Cached_D1 = 0.5;
            obj.Cached_D2 = 0.5;
            obj.Cached_Phi = 0;

            % ===============================================================
            % [핵심 변경점] Workspace를 거치지 않고 .mat 파일에서 직접 로드
            % ===============================================================
            model_file = 'Data/trained_smoothed_hdc_model_v4.mat';
            loaded_data = coder.load(model_file);
            obj.HDC_Model_Data = loaded_data.hdc_model;
            
            if coder.target('MATLAB')
                if isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0
                    fprintf('>> [HDC] Model Loaded Directly from File: Dim=%d, Bins=%d\n', ...
                        obj.HDC_Model_Data.D, obj.HDC_Model_Data.N_bins);
                else
                    fprintf('>> [HDC] Empty Model or Invalid Format. HDC Mode inactive.\n');
                end
            end
        end

        function [G1,G2,G3,G4,G5,G6,G7,G8, ...
                  Power_cmd_out, phi_ext_out, D1_out, D2_out, integrator_out, PWM_Enable_out, ...
                  err_out, phi_raw_out, excess_out, P_fb_out] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L_pri, Power_cmd_in, PWM_Enable_in, ...
                         Kp_ext, Ki_ext, Mode_Select, Phi_Manual, Ctrl_Mode_Ext, DeadTime_ns, ...
                         D1_ext_in, D2_ext_in)

            Ts = obj.Ts_ctrl;
            t_current = double(obj.TickCount) * Ts;
            obj.TickCount = obj.TickCount + 1;

            % =================================================================
            % [STAGE 1] Sensing & Filtering (125-sample Moving Average)
            % =================================================================
            idx = obj.BufferIdx;
            obj.V_HV_Sum = obj.V_HV_Sum - obj.V_HV_Buffer(idx) + V_HV;
            obj.V_HV_Buffer(idx) = V_HV;
            obj.V_HV_Filtered = obj.V_HV_Sum / 125;
            
            obj.V_LV_Sum = obj.V_LV_Sum - obj.V_LV_Buffer(idx) + V_LV;
            obj.V_LV_Buffer(idx) = V_LV;
            obj.V_LV_Filtered = obj.V_LV_Sum / 125;
            
            V_HV_f = obj.V_HV_Filtered;
            V_LV_f = obj.V_LV_Filtered;

            obj.I_Sum = obj.I_Sum - obj.I_Buffer(idx) + I_LV;
            obj.I_Buffer(idx) = I_LV;
            obj.I_L_pri_Sum = obj.I_L_pri_Sum - obj.I_L_pri_Buffer(idx) + I_L_pri;
            obj.I_L_pri_Buffer(idx) = I_L_pri;
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > 125, obj.BufferIdx = 1; end
            I_fb = obj.I_Sum / 125;
            I_L_pri_DC = obj.I_L_pri_Sum / 125;

            % =================================================================
            % [STAGE 2] Control Strategy Selection
            % =================================================================
            D1_set = 0.5; D2_set = 0.5; Phi_FF = 0;
            is_hdc_active = false;
            PWM_Enable = double(PWM_Enable_in > 0.5);

            switch Ctrl_Mode_Ext
                case 1  % ===== EPS: Extended Phase Shift =====
                    V_LV_safe = max(V_LV_f, 1);
                    k_eps = V_HV_f / (obj.n_transformer * V_LV_safe);
                    if k_eps > 1
                        D1_set = max(0.05, min(0.50, 1 / (2 * k_eps)));
                        D2_set = 0.5;
                    else
                        D1_set = 0.5;
                        D2_set = max(0.05, min(0.50, k_eps / 2));
                    end
                    if obj.EnableFF && abs(V_HV_f) > 10
                        V_LV_safe_ff = max(V_LV_f, 1);
                        I_equiv = obj.I_ref_ramped / V_LV_safe_ff;
                        Phi_FF = -1 * I_equiv * (2*obj.L_leakage*obj.Fs*180)/V_HV_f;
                    end

                case 3
                    % ===== HDC Mode =====
                    has_hdc = isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0;
                    has_lut = isfield(obj.HDC_Model_Data, 'D1_table');
                    
                    if has_hdc
                        Vin_curr = V_HV_f; Vout_curr = V_LV_f;
                        Pref_curr = obj.I_ref_ramped;
                        
                        if abs(Vin_curr-obj.Last_Vin_HDC) > obj.HDC_Update_V_Th || ...
                           abs(Vout_curr-obj.Last_Vout_HDC) > obj.HDC_Update_V_Th || ...
                           abs(Pref_curr-obj.Last_Pref_HDC) > obj.HDC_Update_P_Th
                            
                            [d1_n, d2_n, phi_n] = hdc_inference_inline(obj.HDC_Model_Data, Vin_curr, Vout_curr, Pref_curr);
                            obj.Cached_D1 = d1_n; 
                            obj.Cached_D2 = d2_n; 
                            obj.Cached_Phi = phi_n;
                            obj.Last_Vin_HDC = Vin_curr; 
                            obj.Last_Vout_HDC = Vout_curr; 
                            obj.Last_Pref_HDC = Pref_curr;
                        end
                        
                        D1_set = max(0.05, min(0.50, obj.Cached_D1));
                        D2_set = max(0.05, min(0.50, obj.Cached_D2));
                        Phi_FF = max(-0.48, min(0.48, obj.Cached_Phi)) * 360;
                        is_hdc_active = true;
                        
                    elseif has_lut
                        Vin_in = max(min(V_HV_f, max(obj.HDC_Model_Data.Vin_bp)), min(obj.HDC_Model_Data.Vin_bp));
                        Vout_in = max(min(V_LV_f, max(obj.HDC_Model_Data.Vout_bp)), min(obj.HDC_Model_Data.Vout_bp));
                        P_in = max(min(obj.I_ref_ramped, max(obj.HDC_Model_Data.P_bp)), min(obj.HDC_Model_Data.P_bp));
                        
                        [d1_l, d2_l, phi_l] = lut_inference_inline(obj.HDC_Model_Data, Vin_in, Vout_in, P_in);
                        D1_set = d1_l; D2_set = d2_l; Phi_FF = phi_l * 360; 
                        is_hdc_active = true;
                    else
                        D1_set = 0.5; D2_set = 0.5;
                        if obj.EnableFF && abs(V_HV_f) > 10
                            V_LV_safe_ff = max(V_LV_f, 1);
                            I_equiv = obj.I_ref_ramped / V_LV_safe_ff;
                            Phi_FF = -1 * I_equiv * (2*obj.L_leakage*obj.Fs*180)/V_HV_f;
                        end
                    end

                otherwise
                    % ===== 기본 SPS 모드 =====
                    if obj.EnableFF && abs(V_HV_f) > 10
                        V_LV_safe_ff = max(V_LV_f, 1);
                        I_equiv = obj.I_ref_ramped / V_LV_safe_ff;
                        Phi_FF = -1 * I_equiv * (2*obj.L_leakage*obj.Fs*180)/V_HV_f;
                    end
            end

            if t_current < obj.SoftStartTime
                Phi_FF = Phi_FF * (t_current / obj.SoftStartTime);
            end

            % =================================================================
            % [STAGE 3~4] PI Feedback + RC Comp + Signal Synthesis
            % =================================================================
            err_ext = 0; phi_raw_unbounded = 0; excess = 0; phi_ext_deg = 0;

            if PWM_Enable && (Mode_Select ~= 1)
                P_target = Power_cmd_in;
                if isinf(obj.MaxSlewRate)
                    obj.I_ref_ramped = P_target;
                else
                    max_step = obj.MaxSlewRate * Ts;
                    delta = P_target - obj.I_ref_ramped;
                    if abs(delta) > max_step
                        obj.I_ref_ramped = obj.I_ref_ramped + sign(delta) * max_step;
                    else
                        obj.I_ref_ramped = P_target;
                    end
                end
                
                % [전류 피드백 기반 제어: 외부 전력 지령을 내부 전류 지령으로 변환]
                if obj.I_ref_ramped <= 0
                    V_safe = max(abs(V_LV_f), 1);
                    I_cmd = obj.I_ref_ramped / V_safe;
                    I_fb_use = I_fb;
                    P_fb = V_LV_f * I_fb;
                else
                    V_safe = max(abs(V_HV_f), 1);
                    I_cmd = obj.I_ref_ramped / V_safe;
                    I_fb_use = I_HV;
                    P_fb = V_HV_f * I_HV;
                end
                
                % -------------------------------------------------------------
                % [수정] 전력(전류) 지령치에 비례한 백분율 불감대(DeadZone_Percent) 적용
                % -------------------------------------------------------------
                raw_err = I_cmd - I_fb_use;
                deadzone_limit = abs(I_cmd) * (obj.DeadZone_Percent / 100);
                
                if abs(raw_err) < deadzone_limit
                    err_ext = 0;
                else
                    err_ext = raw_err - sign(raw_err) * deadzone_limit;
                end
                
                Kp_curr = Kp_ext;
                if abs(err_ext) < obj.GainSched_Threshold
                    Kp_curr = Kp_ext * obj.GainSched_Factor; 
                end
                
                % -------------------------------------------------------------
                % [추가] 주기적 정상상태 오차 보정 (Repetitive Control)
                % -------------------------------------------------------------
                Phi_RC = 0;
                if obj.EnablePeriodicComp
                    rc_idx = obj.RC_Idx;
                    % 1주기(125 샘플) 전의 보상값 호출
                    past_rc_val = obj.RC_Buffer(rc_idx);
                    
                    % RC 업데이트 수식: U_rc(k) = Q * U_rc(k-N) + K_rc * e(k)
                    % 안정성을 위해 망각 계수(Q=0.99) 적용
                    new_rc_val = 0.99 * past_rc_val + obj.PeriodicComp_Gain * err_ext;
                    
                    % 발산 방지 Limit 설정 (전체 위상의 10도 이내로 보상 제한)
                    new_rc_val = max(-10, min(10, new_rc_val));
                    
                    obj.RC_Buffer(rc_idx) = new_rc_val;
                    Phi_RC = past_rc_val; % 현재 지령에는 과거 보상치 사용
                    
                    obj.RC_Idx = obj.RC_Idx + 1;
                    if obj.RC_Idx > 125, obj.RC_Idx = 1; end
                else
                    % 비활성화 시 버퍼 초기화
                    obj.RC_Buffer(:) = 0;
                end
                
                % PI 제어 + 주기적 보상(RC) + 피드포워드(FF) 결합
                % (오차가 양수면 Phi를 감소시켜야 하므로 부호 주의)
                Phi_PI = -1 * (Kp_curr * err_ext + Ki_ext * obj.Integrator_ext);
                phi_total_raw = Phi_PI - Phi_RC + Phi_FF;
                phi_saturated = max(-90, min(90, phi_total_raw));
                
                delta_phi = phi_saturated - obj.Prev_Phi_Output;
                max_phi_step = obj.MaxPhaseRate * Ts;
                if abs(delta_phi) > max_phi_step
                    phi_ext_deg = obj.Prev_Phi_Output + sign(delta_phi) * max_phi_step;
                else
                    phi_ext_deg = phi_saturated;
                end
                
                obj.Prev_Phi_Output = phi_ext_deg;
                excess = phi_ext_deg - phi_total_raw;
                obj.Integrator_ext = obj.Integrator_ext + err_ext * Ts - (excess * Ts);
                phi_raw_unbounded = phi_total_raw;
                
            elseif Mode_Select == 1
                % ===== 완전 매뉴얼 모드 =====
                phi_ext_deg = Phi_Manual; 
                D1_set = max(0.05, min(0.50, D1_ext_in)); 
                D2_set = max(0.05, min(0.50, D2_ext_in));
                P_fb = V_LV_f * I_fb;
            else
                obj.I_ref_ramped = 0; obj.Prev_Phi_Output = 0;
                P_fb = V_LV_f * I_fb;
            end

            % =================================================================
            % [STAGE 5] Flux Balancing
            % =================================================================
            D_offset = 0;
            if PWM_Enable && obj.EnableFluxBalancing && (Mode_Select ~= 1)
                err_flux = -I_L_pri_DC;
                obj.Flux_Integrator = obj.Flux_Integrator + err_flux * Ts;
                max_integ = obj.Flux_Limit / obj.Flux_Ki;
                obj.Flux_Integrator = max(-max_integ, min(max_integ, obj.Flux_Integrator));
                D_raw = obj.Flux_Kp * err_flux + obj.Flux_Ki * obj.Flux_Integrator;
                D_offset = max(-obj.Flux_Limit, min(obj.Flux_Limit, D_raw));
            end

            % =================================================================
            % [STAGE 6] Phase Assignment
            % =================================================================
            D1_final = D1_set + D_offset; D2_final = D2_set;
            if PWM_Enable
                phi1_deg = 0;
                phi3_deg = D1_final * 360;
                phi5_deg = phi_ext_deg;
                phi7_deg = phi_ext_deg + D2_final * 360;
                D1_out = D1_final; D2_out = D2_final;
            else
                phi1_deg = 0; phi3_deg = 0; phi5_deg = 0; phi7_deg = 0;
                D1_out = 0; D2_out = 0;
            end

            % =================================================================
            % [STAGE 7] PWM Generation
            % =================================================================
            Ts_sw = 1 / obj.Fs;
            phase_increment = (obj.Ts_ctrl / Ts_sw) * 360;
            obj.PhaseCounter = mod(obj.PhaseCounter + phase_increment, 360);

            V_ON = obj.V_Gate_ON; V_OFF = obj.V_Gate_OFF;

            if ~PWM_Enable
                G1=V_OFF; G2=V_OFF; G3=V_OFF; G4=V_OFF;
                G5=V_OFF; G6=V_OFF; G7=V_OFF; G8=V_OFF;
            else
                DT = (DeadTime_ns * 1e-9 / Ts_sw) * 360;
                cp = obj.PhaseCounter;

                p1 = mod(phi1_deg, 360); if p1<0, p1=p1+360; end
                p3 = mod(phi3_deg, 360); if p3<0, p3=p3+360; end
                p5 = mod(phi5_deg, 360); if p5<0, p5=p5+360; end
                p7 = mod(phi7_deg, 360); if p7<0, p7=p7+360; end

                rS1 = mod(cp - p1 + 360, 360);
                rS3 = mod(cp - p3 + 360, 360);
                rS5 = mod(cp - p5 + 360, 360);
                rS7 = mod(cp - p7 + 360, 360);

                if (rS1>=DT)&&(rS1<(180-DT)), G1=V_ON; else, G1=V_OFF; end
                if (rS1>=(180+DT))&&(rS1<(360-DT)), G2=V_ON; else, G2=V_OFF; end
                if (rS3>=DT)&&(rS3<(180-DT)), G3=V_ON; else, G3=V_OFF; end
                if (rS3>=(180+DT))&&(rS3<(360-DT)), G4=V_ON; else, G4=V_OFF; end
                if (rS5>=DT)&&(rS5<(180-DT)), G5=V_ON; else, G5=V_OFF; end
                if (rS5>=(180+DT))&&(rS5<(360-DT)), G6=V_ON; else, G6=V_OFF; end
                if (rS7>=DT)&&(rS7<(180-DT)), G7=V_ON; else, G7=V_OFF; end
                if (rS7>=(180+DT))&&(rS7<(360-DT)), G8=V_ON; else, G8=V_OFF; end
            end

            % =================================================================
            % [STAGE 8] Output Assignment
            % =================================================================
            Power_cmd_out = obj.I_ref_ramped;
            phi_ext_out = phi_ext_deg;
            integrator_out = obj.Integrator_ext;
            PWM_Enable_out = PWM_Enable;
            err_out = err_ext;
            phi_raw_out = phi_raw_unbounded;
            excess_out = excess;
            P_fb_out = P_fb;
        end

        function resetImpl(obj)
            obj.Integrator_ext=0; obj.Flux_Integrator=0; obj.Prev_Phi_Output=0;
            obj.I_Buffer=zeros(125,1); obj.BufferIdx=1; obj.I_Sum=0;
            obj.I_L_pri_Buffer=zeros(125,1); obj.I_L_pri_Sum=0;
            obj.V_HV_Buffer=zeros(125,1); obj.V_HV_Sum=0;
            obj.V_LV_Buffer=zeros(125,1); obj.V_LV_Sum=0;
            obj.TickCount=0; obj.PhaseCounter=0;
            
            obj.RC_Buffer=zeros(125,1); obj.RC_Idx=1;
            
            obj.Last_Vin_HDC = 0; obj.Last_Vout_HDC = 0; obj.Last_Pref_HDC = 0;
            obj.Cached_D1 = 0.5; obj.Cached_D2 = 0.5; obj.Cached_Phi = 0;
        end

        function num = getNumInputsImpl(~), num = 15; end
        function num = getNumOutputsImpl(~), num = 18; end

        function [s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12,s13,s14,s15,s16,s17,s18] = getOutputSizeImpl(~)
            s1=[1 1];s2=[1 1];s3=[1 1];s4=[1 1];s5=[1 1];s6=[1 1];s7=[1 1];s8=[1 1];
            s9=[1 1];s10=[1 1];s11=[1 1];s12=[1 1];s13=[1 1];s14=[1 1];s15=[1 1];s16=[1 1];s17=[1 1];s18=[1 1];
        end
        function [d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18] = getOutputDataTypeImpl(~)
            d1='double';d2='double';d3='double';d4='double';d5='double';d6='double';d7='double';d8='double';
            d9='double';d10='double';d11='double';d12='double';d13='double';d14='double';d15='double';d16='double';d17='double';d18='double';
        end
        function [c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18] = isOutputComplexImpl(~)
            c1=false;c2=false;c3=false;c4=false;c5=false;c6=false;c7=false;c8=false;
            c9=false;c10=false;c11=false;c12=false;c13=false;c14=false;c15=false;c16=false;c17=false;c18=false;
        end
        function [f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,f13,f14,f15,f16,f17,f18] = isOutputFixedSizeImpl(~)
            f1=true;f2=true;f3=true;f4=true;f5=true;f6=true;f7=true;f8=true;
            f9=true;f10=true;f11=true;f12=true;f13=true;f14=true;f15=true;f16=true;f17=true;f18=true;
        end

        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            switch name
                case {'Integrator_ext','I_ref_ramped','V_HV_Filtered','V_LV_Filtered', ...
                      'Prev_Phi_Output','BufferIdx','I_Sum','RC_Idx', ...
                      'I_L_pri_Sum','Flux_Integrator','TickCount','V_HV_Sum','V_LV_Sum', ...
                      'PhaseCounter', 'Last_Vin_HDC', 'Last_Vout_HDC', 'Last_Pref_HDC', ...
                      'Cached_D1', 'Cached_D2', 'Cached_Phi'}
                    sz = [1 1]; dt = 'double'; cp = false;
                case {'I_Buffer','I_L_pri_Buffer','V_HV_Buffer','V_LV_Buffer', 'RC_Buffer'}
                    sz = [125 1]; dt = 'double'; cp = false;
                otherwise
                    error(['Unknown state: ', name]);
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts_ctrl, 'OffsetTime', 0);
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10,n11,n12,n13,n14,n15] = getInputNamesImpl(~)
            n1='V_HV'; n2='I_HV'; n3='V_LV'; n4='I_LV'; n5='I_L_pri';
            n6='Power_cmd_in'; n7='PWM_En_in'; n8='Kp_ext'; n9='Ki_ext';
            n10='Mode_Select'; n11='Phi_Manual'; n12='Ctrl_Mode'; n13='DeadTime_ns';
            n14='D1_ext_in'; n15='D2_ext_in';
        end

        function [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11,o12,o13,o14,o15,o16,o17,o18] = getOutputNamesImpl(~)
            o1='G1'; o2='G2'; o3='G3'; o4='G4'; o5='G5'; o6='G6'; o7='G7'; o8='G8';
            o9='P_Ref'; o10='Phi_Ext'; o11='D1'; o12='D2';
            o13='Integ'; o14='PWM_En'; o15='Err'; o16='Phi_Raw'; o17='Excess'; o18='P_fb';
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Controller + PWM (HDC & RC Comp)', ...
                'Text', sprintf(['전류 추종 제어 + PWM 게이트 신호 생성 통합 블록입니다.\n', ...
                                 'HDC 모델 로드, 전력비례 불감대(%), 주기적 정상오차 보정(RC)을 지원합니다.']));
        end
        function groups = getPropertyGroupsImpl
            g1 = matlab.system.display.Section('Title','1. Hardware Spec', ...
                'PropertyList',{'Fs','Ts_ctrl','L_leakage','n_transformer','SPS_Inner_Phase'});
            g2 = matlab.system.display.Section('Title','2. Advanced Control & Error Comp', ...
                'PropertyList',{'DeadZone_Percent', 'EnablePeriodicComp', 'PeriodicComp_Gain', ...
                                'EnableFF','SoftStartTime','MaxSlewRate','MaxPhaseRate','GainSched_Threshold','GainSched_Factor'});
            g3 = matlab.system.display.Section('Title','3. Flux Balancing', ...
                'PropertyList',{'EnableFluxBalancing','Flux_Kp','Flux_Ki','Flux_Limit'});
            g4 = matlab.system.display.Section('Title','4. HDC Control Params', ...
                'PropertyList',{'HDC_Update_V_Th','HDC_Update_P_Th'});
            g5 = matlab.system.display.Section('Title','5. PWM Gate Driver', ...
                'PropertyList',{'V_Gate_ON','V_Gate_OFF'});
            groups = [g1, g2, g3, g4, g5];
        end
    end
end

% =====================================================================
% HDC Inference Function
% =====================================================================
function [D1, D2, Phi] = hdc_inference_inline(HDC, Vin, Vout, Pref)
    if ~isfield(HDC, 'M_int'), D1=0.5; D2=0.5; Phi=0; return; end
    
    X = [Pref; Vout]; 
    X_q = max(1, min(HDC.Q, round((X - HDC.min_X) ./ (HDC.max_X - HDC.min_X + 1e-9) * (HDC.Q - 1)) + 1));
    
    hv_sum = HDC.ID_vecs(1, :) .* HDC.L_vecs(X_q(1), :) + ...
             HDC.ID_vecs(2, :) .* HDC.L_vecs(X_q(2), :);
    
    if strcmp(HDC.mode, 'Multi_Interact')
        hv_sum = hv_sum + (HDC.ID_vecs(1, :) .* HDC.L_vecs(X_q(1), :)) .* (HDC.ID_vecs(2, :) .* HDC.L_vecs(X_q(2), :)) ...
                        + circshift(HDC.L_vecs(X_q(1), :), 1) .* circshift(HDC.L_vecs(X_q(2), :), -1);
    end
    
    S_curr_bin = sign(hv_sum)'; S_curr_bin(S_curr_bin == 0) = 1;
    
    sim = (HDC.C_bin * S_curr_bin) / HDC.D;
    max_sim_val = max(sim * HDC.gamma);
    delta_prime = exp(sim * HDC.gamma - max_sim_val) / sum(exp(sim * HDC.gamma - max_sim_val));
    
    K = size(HDC.M_int, 1);
    Y_hat_norm = zeros(2, 1);
    for k = 1:K
        Mk = reshape(HDC.M_int(k, :, :), 2, HDC.D);
        Y_hat_norm = Y_hat_norm + delta_prime(k) * (Mk * S_curr_bin / HDC.D);
    end
    
    Y_pred = HDC.min_Y + (Y_hat_norm + 1) .* (HDC.max_Y - HDC.min_Y) / 2;
    D1 = Y_pred(1); D2 = Y_pred(2); Phi = 0; 
end

% =====================================================================
% LUT Inference Function
% =====================================================================
function [D1, D2, Phi] = lut_inference_inline(LUT, Vin, Vout, Pref)
    D1=interpn(LUT.Vin_bp,LUT.Vout_bp,LUT.P_bp,LUT.D1_table,Vin,Vout,Pref,'linear',0.5);
    D2=interpn(LUT.Vin_bp,LUT.Vout_bp,LUT.P_bp,LUT.D2_table,Vin,Vout,Pref,'linear',0.5);
    Phi=interpn(LUT.Vin_bp,LUT.Vout_bp,LUT.P_bp,LUT.Phi_table,Vin,Vout,Pref,'linear',0);
    D1=max(0.05,min(0.50,D1)); D2=max(0.05,min(0.50,D2)); Phi=max(-0.48,min(0.48,Phi));
end