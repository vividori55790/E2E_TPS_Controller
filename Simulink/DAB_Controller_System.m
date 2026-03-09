classdef DAB_Controller_System < matlab.System
    % DAB_Controller_System  (v9.0 — Power Tracking + PWM Generation)
    % DAB 컨버터 전력 추종 제어기 + PWM 신호 생성기 통합
    %
    % 아키텍처 (v9.0):
    %   1. Sensing & Filtering
    %   2. Control Strategy Selection (SPS/EPS/TPS)
    %      Ctrl_Mode_Ext: 0=SPS, 1=EPS, 2=DPS(Reserved), 3=TPS
    %   3. PI Feedback + Signal Synthesis (Power-Based)
    %   4. Flux Balancing
    %   5. Phase Assignment (D1, D2, Phi → phi1~phi7)
    %   6. PWM Generation (Phase → Gate Signals, 데드타임 적용)
    %
    % 입력(15): V_HV, I_HV, V_LV, I_LV, I_L_pri, Power_cmd_in, PWM_Enable_in,
    %           Kp_ext, Ki_ext, Mode_Select, Phi_Manual, Ctrl_Mode_Ext, DeadTime_ns,
    %           D1_ext_in, D2_ext_in
    % 출력(18): G1~G8(8), Power_cmd_out, phi_ext_out, D1_out, D2_out,
    %           integrator_out, PWM_Enable, err_out, phi_raw_out, excess_out, P_fb_out
    %
    % [부호 규약]
    %   - 전력(P): (+)=방전(LV->HV), (-)=충전(HV->LV)
    %   - 위상(Phi): (+)=충전, (-)=방전
    %   - 전력 피드백: 받는 쪽 기준 (충전→배터리측, 방전→HV부하측)
    %
    % =========================================================================
    % 1. Properties
    % =========================================================================
    properties(Nontunable)
        Fs = 80e3;              % 스위칭 주파수 (Hz)
        Ts_ctrl = 1e-7;         % 제어 샘플링 시간 (Ts) [sec]
        SPS_Inner_Phase = 180;  % [SPS] 내부 위상 고정값 (180도)
        HDC_Data = struct('D',0);
        L_leakage = 90e-6;      % 변압기 누설 인덕턴스 (H)
        n_transformer = 4;      % 변압기 턴비 (V_HV_nom / V_LV_nom = 380/95)
        EnableFF = true;        % 피드포워드(FF) 제어 활성화
        MaxSlewRate = 50000;    % 최종 목표 전력 지령 변화율 제한 (Soft-start 등 충격 방지용) [W/s]
        HDC_Update_V_Th = 2.0;  % HDC 재실행 전압 변화 임계값 (V)
        HDC_Update_P_Th = 50.0; % HDC 재실행 전력 변화 임계값 (W)
        DeadZone = 0;           % 제어 불감대 [A]
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
        % (데이터셋 생성용 버전에서는 HDC/로그용 상태 생략하여 경고 방지)
        % [PWM] Phase Counter
        PhaseCounter;
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
            obj.HDC_Model_Data = obj.HDC_Data;
            if coder.target('MATLAB')
                if isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0
                    fprintf('>> [HDC] Model Loaded: Dim=%d, Bins=%d\n', obj.HDC_Model_Data.D, obj.HDC_Model_Data.N_bins);
                elseif isfield(obj.HDC_Model_Data, 'D1_table')
                    fprintf('>> [HDC] LUT Loaded: Size=[%d x %d x %d]\n', ...
                        length(obj.HDC_Model_Data.Vin_bp), length(obj.HDC_Model_Data.Vout_bp), length(obj.HDC_Model_Data.P_bp));
                else
                    fprintf('>> [HDC] Empty Model. TPS inactive.\n');
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
            % (Check log removed)

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
            is_hdc_active = false; %#ok<NASGU>
            is_hdc_active = false;
            PWM_Enable = double(PWM_Enable_in > 0.5);

            switch Ctrl_Mode_Ext
                case 1  % ===== EPS: Extended Phase Shift =====
                    % 전압비 기반 내부 위상 계산 → 순환전류 저감
                    V_LV_safe = max(V_LV_f, 1);
                    k_eps = V_HV_f / (obj.n_transformer * V_LV_safe);
                    if k_eps > 1
                        % V_HV 과다 → Primary duty 축소
                        D1_set = max(0.05, min(0.50, 1 / (2 * k_eps)));
                        D2_set = 0.5;
                    else
                        % V_LV 과다 → Secondary duty 축소
                        D1_set = 0.5;
                        D2_set = max(0.05, min(0.50, k_eps / 2));
                    end
                    % Feedforward (전력→위상 변환)
                    if obj.EnableFF && abs(V_HV_f) > 10
                        V_LV_safe_ff = max(V_LV_f, 1);
                        I_equiv = obj.I_ref_ramped / V_LV_safe_ff;
                        Phi_FF = -1 * I_equiv * (2*obj.L_leakage*obj.Fs*180)/V_HV_f;
                    end

                case 3
                    % ============================================================
                    % [TPS Mode — External D1/D2 Direct Injection]
                    % HDC/LUT 추론 비활성화: 데이터셋 수집 전용
                    % 외부에서 주입된 D1_ext_in, D2_ext_in을 직접 사용
                    % 외부 위상(Phi)만 PI 제어기가 조절
                    % ============================================================
                    D1_set = max(0.05, min(0.50, D1_ext_in));
                    D2_set = max(0.05, min(0.50, D2_ext_in));
                    % Feedforward: 전력→위상 변환 (외부 위상 PI만 사용)
                    if obj.EnableFF && abs(V_HV_f) > 10
                        V_LV_safe_ff = max(V_LV_f, 1);
                        I_equiv = obj.I_ref_ramped / V_LV_safe_ff;
                        Phi_FF = -1 * I_equiv * (2*obj.L_leakage*obj.Fs*180)/V_HV_f;
                    end
                    % --- [COMMENTED OUT] HDC/LUT Inference ---
                    % has_hdc = isfield(obj.HDC_Model_Data, 'D') && obj.HDC_Model_Data.D > 0;
                    % has_lut = isfield(obj.HDC_Model_Data, 'D1_table');
                    % if ~has_hdc && ~has_lut
                    %     error('>> [CTRL FAULT] TPS Mode(=3) but HDC/LUT EMPTY!');
                    % end
                    % if has_hdc
                    %     Vin_curr = V_HV_f; Vout_curr = V_LV_f;
                    %     Pref_curr = obj.I_ref_ramped * V_LV_f;
                    %     if abs(Vin_curr-obj.Last_Vin_HDC) > obj.HDC_Update_V_Th || ...
                    %        abs(Vout_curr-obj.Last_Vout_HDC) > obj.HDC_Update_V_Th || ...
                    %        abs(Pref_curr-obj.Last_Pref_HDC) > obj.HDC_Update_P_Th
                    %         [d1_n,d2_n,phi_n] = hdc_inference_inline(obj.HDC_Model_Data, Vin_curr, Vout_curr, Pref_curr);
                    %         obj.Cached_D1=d1_n; obj.Cached_D2=d2_n; obj.Cached_Phi=phi_n;
                    %         obj.Last_Vin_HDC=Vin_curr; obj.Last_Vout_HDC=Vout_curr; obj.Last_Pref_HDC=Pref_curr;
                    %     end
                    %     D1_set = max(0.05, min(0.50, obj.Cached_D1));
                    %     D2_set = max(0.05, min(0.50, obj.Cached_D2));
                    %     Phi_FF = max(-0.48, min(0.48, obj.Cached_Phi)) * 360;
                    %     is_hdc_active = true;
                    % elseif has_lut
                    %     Vin_in = max(min(V_HV_f, max(obj.HDC_Model_Data.Vin_bp)), min(obj.HDC_Model_Data.Vin_bp));
                    %     Vout_in = max(min(V_LV_f, max(obj.HDC_Model_Data.Vout_bp)), min(obj.HDC_Model_Data.Vout_bp));
                    %     P_in = max(min(obj.I_ref_ramped*V_LV_f, max(obj.HDC_Model_Data.P_bp)), min(obj.HDC_Model_Data.P_bp));
                    %     [d1_l,d2_l,phi_l] = lut_inference_inline(obj.HDC_Model_Data, Vin_in, Vout_in, P_in);
                    %     D1_set=d1_l; D2_set=d2_l; Phi_FF=phi_l*360; is_hdc_active=true;
                    % end
                otherwise
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
            % [STAGE 3~4] PI Feedback + Signal Synthesis
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
                %   충전(P<0): 배터리측 전류 = I_LV_f (여기선 I_fb)
                %   방전(P>0): HV부하측 전류 = I_HV
                if obj.I_ref_ramped <= 0
                    % 충전: 배터리가 받는 쪽
                    V_safe = max(abs(V_LV_f), 1);
                    I_cmd = obj.I_ref_ramped / V_safe;
                    I_fb_use = I_fb;
                    P_fb = V_LV_f * I_fb;
                else
                    % 방전(UPS): HV 부하가 받는 쪽
                    V_safe = max(abs(V_HV_f), 1);
                    I_cmd = obj.I_ref_ramped / V_safe;
                    I_fb_use = I_HV;
                    P_fb = V_HV_f * I_HV;
                end
                raw_err = I_cmd - I_fb_use;
                if abs(raw_err) < obj.DeadZone, err_ext = 0;
                else, err_ext = raw_err - sign(raw_err) * obj.DeadZone; end
                Kp_curr = Kp_ext;
                if abs(err_ext) < obj.GainSched_Threshold, Kp_curr = Kp_ext * obj.GainSched_Factor; end
                Phi_PI = -1 * (Kp_curr * err_ext + Ki_ext * obj.Integrator_ext);
                phi_total_raw = Phi_PI + Phi_FF;
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
                % [완전 매뉴얼 모드]: 외부 위상(D0) 및 D1, D2를 PI 제어 없이 직접 인가
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
            % Ctrl_Mode_Ext == 3 (TPS/Dataset Gen) 에서는 두 내부 위상(D1, D2)이 
            % 지령치를 기계적으로 따르도록 Flux Balancing을 비활성화합니다.
            if PWM_Enable && obj.EnableFluxBalancing && (Ctrl_Mode_Ext ~= 3)
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
            % [STAGE 7] PWM Generation (from DAB_PWM_Generator)
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

                % Normalize phases to 0~360
                p1 = mod(phi1_deg, 360); if p1<0, p1=p1+360; end
                p3 = mod(phi3_deg, 360); if p3<0, p3=p3+360; end
                p5 = mod(phi5_deg, 360); if p5<0, p5=p5+360; end
                p7 = mod(phi7_deg, 360); if p7<0, p7=p7+360; end

                % Relative phases
                rS1 = mod(cp - p1 + 360, 360);
                rS3 = mod(cp - p3 + 360, 360);
                rS5 = mod(cp - p5 + 360, 360);
                rS7 = mod(cp - p7 + 360, 360);

                % Gate signals with deadtime
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
                      'Prev_Phi_Output','BufferIdx','I_Sum', ...
                      'I_L_pri_Sum','Flux_Integrator','TickCount','V_HV_Sum','V_LV_Sum', ...
                      'PhaseCounter'}
                    sz = [1 1]; dt = 'double'; cp = false;
                case {'I_Buffer','I_L_pri_Buffer','V_HV_Buffer','V_LV_Buffer'}
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
                'Title', 'DAB Converter Controller + PWM (Unified)', ...
                'Text', sprintf(['전류 추종 제어 + PWM 게이트 신호 생성 통합 블록입니다.\n', ...
                                 '상위 EMS로부터 I_cmd/PWM_Enable을 수신하여 최종 G1~G8을 출력합니다.']));
        end
        function groups = getPropertyGroupsImpl
            g1 = matlab.system.display.Section('Title','1. Hardware Spec', ...
                'PropertyList',{'Fs','Ts_ctrl','L_leakage','n_transformer','SPS_Inner_Phase'});
            g2 = matlab.system.display.Section('Title','2. Advanced Control', ...
                'PropertyList',{'EnableFF','SoftStartTime','MaxSlewRate','MaxPhaseRate','DeadZone','GainSched_Threshold','GainSched_Factor'});
            g3 = matlab.system.display.Section('Title','3. Flux Balancing', ...
                'PropertyList',{'EnableFluxBalancing','Flux_Kp','Flux_Ki','Flux_Limit'});
            g4 = matlab.system.display.Section('Title','4. HDC/TPS Control', ...
                'PropertyList',{'HDC_Data','HDC_Update_V_Th','HDC_Update_P_Th'});
            g5 = matlab.system.display.Section('Title','5. PWM Gate Driver', ...
                'PropertyList',{'V_Gate_ON','V_Gate_OFF'});
            groups = [g1, g2, g3, g4, g5];
        end
    end
end

% =====================================================================
% HDC Model Auto-Loader
% =====================================================================
function HDC = load_hdc_model(model_path)
    HDC = struct();
    if ~exist(model_path, 'file')
        error('>> [HDC] Model file NOT FOUND: %s (pwd: %s)', model_path, pwd);
    end
    try
        loaded = load(model_path);
        if isfield(loaded,'results') && isfield(loaded.results,'hdc_model')
            src = loaded.results.hdc_model;
        elseif isfield(loaded,'lut_data'), HDC=loaded.lut_data; return;
        elseif isfield(loaded,'HDC'), HDC=loaded.HDC; return;
        else, error('>> [HDC] Invalid format!'); end
        HDC.D=double(src.D); HDC.N_levels=double(src.N_levels);
        HDC.N_inputs=double(src.N_inputs); HDC.N_bins=double(src.N_output_bins);
        HDC.BaseHVs=double(src.base_hvs);
        HDC.LevelHVs=zeros(HDC.N_inputs,HDC.N_levels,HDC.D);
        for i=1:HDC.N_inputs, HDC.LevelHVs(i,:,:)=double(src.level_hvs{i}); end
        HDC.Prototypes=zeros(3,HDC.N_bins,HDC.D);
        for k=1:3, HDC.Prototypes(k,:,:)=double(src.output_prototypes{k}); end
        HDC.PermIdx1=double(src.perm_idx); HDC.PermIdx2=double(src.perm_idx2);
        HDC.Param_n=double(src.encode_params.n);
        HDC.Param_Prated=double(src.encode_params.P_rated);
        HDC.Param_VinNom=double(src.encode_params.Vin_nom);
        HDC.Param_VoutNom=double(src.encode_params.Vout_nom);
        HDC.Param_Lnom=double(src.encode_params.L_nom);
        HDC.Param_fs=double(src.encode_params.fs);
        if isfield(src,'temperatures'), HDC.Temps=double(src.temperatures);
        else, HDC.Temps=[src.temperature,src.temperature,src.temperature]; end
        HDC.OutputRanges=double(src.output_ranges);
        HDC.BinValues=zeros(3,HDC.N_bins);
        for k=1:3, HDC.BinValues(k,:)=linspace(HDC.OutputRanges(k,1),HDC.OutputRanges(k,2),HDC.N_bins); end
    catch ME, error('>> [HDC] Load error: %s', ME.message); end
end

% =====================================================================
% HDC Inference Function
% =====================================================================
function [D1, D2, Phi] = hdc_inference_inline(HDC, Vin, Vout, Pref)
    if ~isfield(HDC,'D')||HDC.D==0, D1=0.5;D2=0.5;Phi=0; return; end
    D_dim=HDC.D; N_levels=HDC.N_levels;
    n_tr=HDC.Param_n; P_rated=HDC.Param_Prated; Vin_nom=HDC.Param_VinNom;
    L_nom=HDC.Param_Lnom; fs=HDC.Param_fs;
    M=(n_tr*Vout)/max(Vin,1); P_norm=Pref/P_rated; V_ratio=Vin/Vin_nom;
    P_abs_norm=abs(Pref)/P_rated; P_sign=(sign(Pref)+1)/2;
    Z_L=2*pi*fs*L_nom;
    if abs(Pref)>5
        Z_load=Vin^2/abs(Pref); Z_norm=Z_L/Z_load;
        R_load=Vout^2/abs(Pref); Load_Z=R_load/100;
    else, Z_norm=0; Load_Z=1.0; end
    Z_norm=max(0,min(1,Z_norm/2.0)); Load_Z=max(0,min(1,Load_Z));
    VP_cross=max(0,min(1,((M-1.0)*P_norm+2)/4));
    B_est=max(0,min(1,Vin/(4*20*3e-4*fs)/0.35));
    f=zeros(12,1);
    f(1)=max(0,min(1,(Vin-180)/(460-180))); f(2)=max(0,min(1,(Vout-65)/(115-65)));
    f(3)=max(0,min(1,(Pref+1100)/2200)); f(4)=max(0,min(1,(M-0.3)/2.5));
    f(5)=max(0,min(1,(P_norm+1)/2)); f(6)=max(0,min(1,V_ratio/1.5));
    f(7)=max(0,min(1,P_abs_norm)); f(8)=P_sign;
    f(9)=Z_norm; f(10)=VP_cross; f(11)=Load_Z; f(12)=B_est;
    lvl=zeros(12,1);
    for i=1:12
        ir=floor(f(i)*(N_levels-1))+1;
        if ir<1,ir=1; end; if ir>N_levels,ir=N_levels; end; lvl(i)=ir;
    end
    hv=zeros(1,D_dim);
    for i=1:12
        hv=hv+(HDC.BaseHVs(i,:).*reshape(HDC.LevelHVs(i,lvl(i),:),1,D_dim));
    end
    h1=HDC.BaseHVs(1,:).*reshape(HDC.LevelHVs(1,lvl(1),:),1,D_dim);
    h2=HDC.BaseHVs(2,:).*reshape(HDC.LevelHVs(2,lvl(2),:),1,D_dim);
    hv=hv+0.5*(h1.*h2(HDC.PermIdx1));
    h3=HDC.BaseHVs(3,:).*reshape(HDC.LevelHVs(3,lvl(3),:),1,D_dim);
    h4=HDC.BaseHVs(4,:).*reshape(HDC.LevelHVs(4,lvl(4),:),1,D_dim);
    hv=hv+0.5*(h3.*h4(HDC.PermIdx1));
    h9=HDC.BaseHVs(9,:).*reshape(HDC.LevelHVs(9,lvl(9),:),1,D_dim);
    h11=HDC.BaseHVs(11,:).*reshape(HDC.LevelHVs(11,lvl(11),:),1,D_dim);
    hv=hv+0.3*(h9.*h11(HDC.PermIdx2));
    hv=sign(hv); hv(hv==0)=1;
    D1=readout_hdc_var(hv,1,HDC); D2=readout_hdc_var(hv,2,HDC); Phi=readout_hdc_var(hv,3,HDC);
end

function val = readout_hdc_var(hv, k, HDC)
    D_dim=HDC.D; N_bins=HDC.N_bins;
    protos=reshape(HDC.Prototypes(k,:,:),N_bins,D_dim);
    sims=protos*hv';
    norms=sqrt(sum(protos.^2,2)); norms(norms<1e-10)=1; sims=sims./norms;
    T=HDC.Temps(k); scaled=sims/T; max_s=max(scaled);
    exps=exp(scaled-max_s); weights=exps/(sum(exps)+1e-10);
    val=sum(weights.*HDC.BinValues(k,:)');
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