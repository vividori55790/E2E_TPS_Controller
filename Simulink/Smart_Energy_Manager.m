classdef Smart_Energy_Manager < matlab.System
    % Smart_Energy_Manager  (v1.3 — Load-Following + SOC + Soft-Start)
    % DAB 변환기용 상위 에너지 관리 시스템
    %
    % v1.3 변경: EPS.m에서 다음 기능 이식
    %   - 부하추종(Peak-Shaving) 방전 (mid-peak ESS)
    %   - 전압 기반 SOC 추정 및 출력
    %   - ESS 지령 Soft-start 램프
    %   - 전력 제한 (P_DAB_max)
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kpv, Kiv, Mode_Select, Time_Hour
    % 출력: I_cmd, PWM_Enable, mode_active, status_flag, SOC_percent
    %
    % [부호 규약] I_cmd (+)=방전, (-)=충전
    %
    % =========================================================================
    properties(Nontunable)
        Ts_ems = 1e-7;          % EMS 샘플링 시간 [sec]
        V_HV_Ref = 380;         % [UPS] HV 목표 전압 (V)
        V_HV_Band = 30;         % [UPS] 정전 감지 범위 (±V)
        V_Ref_Ramp_Rate = 500;  % [UPS] 전압 복구 램프 속도 (V/s)

        % [BMS] 배터리 보호 임계값 (26S Li-ion)
        V_Pack_Max = 109.2;     % 만충 전압 (4.2V × 26)
        V_CV_Start = 108.0;     % CV 충전 시작 전압
        V_ESS_Limit = 93.0;     % ESS 방전 중단 전압
        V_UPS_Limit = 87.0;     % UPS Derating 시작 전압
        V_Min_Limit = 84.0;     % 시스템 셧다운 전압

        I_Charge_Limit = 30;    % BMS 최대 충전 전류 (A)
        I_Discharge_Limit = 50; % BMS 최대 방전 전류 (A)
        K_CV = 5.0;             % CV 제어 게인 (A/V)
        InitDelay = 0.001;      % FSM 초기화 지연 (sec)

        % [ESS] TOU 스케줄링
        ESS_Charge_Rate = 10;   % TOU 충전 전류 (A)
        ESS_Discharge_Rate = 10;% TOU 방전 전류 (A)
        TOU_OffPeak_Start = 23; % 경부하 시작 (밤 11시)
        TOU_OffPeak_End = 8;    % 경부하 종료 (아침 8시)
        TOU_OnPeak1_Start = 11; TOU_OnPeak1_End = 12;
        TOU_OnPeak2_Start = 13; TOU_OnPeak2_End = 17;
        SOC_Max_Threshold = 95; SOC_Min_Threshold = 20;

        % [NEW from EPS.m] 부하추종 & 전력 제한
        P_load_threshold = 500; % 부하추종 임계 전력 (W) — 이 이상이면 방전 개시
        P_DAB_max = 1000;       % DAB 최대 전력 제한 (W)
        n_transformer = 4;      % 변압기 턴비 (380/95)
        I_LV_min_limit = -15;   % 배터리 최저 방전 전류 하한 (A, 음수=방전)

        % [NEW from EPS.m] Soft-start
        Iref_ramp_rate = 500;   % ESS 전류 지령 변화율 제한 (A/s)
    end

    properties(Constant)
        ST_INIT=1; ST_ESS_RUN=2; ST_UPS_RUN=3; ST_MANUAL=4; ST_FAULT=5;
    end

    properties
        I_ESS_Max = 10; I_UPS_Max = 30;
    end

    properties(DiscreteState)
        CurrentState; PreviousMode; Integrator_v;
        V_HV_Buffer; V_HV_Sum; V_HV_Filtered;
        V_LV_Buffer; V_LV_Sum; V_LV_Filtered;
        I_HV_Buffer; I_HV_Sum; I_HV_Filtered;
        I_LV_Buffer; I_LV_Sum; I_LV_Filtered;
        BufferIdx; TickCount;
        Iref_soft;  % [NEW] Soft-start 내부 램프 상태
        V_Grid_Buffer; V_Grid_Sum; V_Grid_Filtered; % [New for UPS] 계통 전압 감지
        V_Ref_Ramp; % [NEW] UPS 전압 지령 램프 (Soft-Recovery)
    end

    methods(Access = protected)
        function setupImpl(obj)
            obj.CurrentState=obj.ST_INIT; obj.PreviousMode=obj.ST_INIT;
            obj.Integrator_v=0; obj.Iref_soft=0; obj.V_Ref_Ramp=0;
            obj.V_HV_Buffer=zeros(125,1); obj.V_HV_Sum=0; obj.V_HV_Filtered=0;
            obj.V_LV_Buffer=zeros(125,1); obj.V_LV_Sum=0; obj.V_LV_Filtered=0;
            obj.I_HV_Buffer=zeros(125,1); obj.I_HV_Sum=0; obj.I_HV_Filtered=0;
            obj.I_LV_Buffer=zeros(125,1); obj.I_LV_Sum=0; obj.I_LV_Filtered=0;
            obj.V_Grid_Buffer=zeros(125,1); obj.V_Grid_Sum=0; obj.V_Grid_Filtered=0;
            obj.BufferIdx=1; obj.TickCount=0;
        end

        function [I_cmd, PWM_Enable, mode_active, status_flag, SOC_percent, Grid_Connect, V_Ref_Debug, I_Int_Debug] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kpv, Kiv, Mode_Select, Time_Hour, V_Grid)

            Ts = obj.Ts_ems; t_current = double(obj.TickCount)*Ts;
            obj.TickCount = obj.TickCount+1;

            % =================================================================
            % [STAGE 1] Sensing & Filtering
            % =================================================================
            idx = obj.BufferIdx;
            obj.V_HV_Sum=obj.V_HV_Sum-obj.V_HV_Buffer(idx)+V_HV; obj.V_HV_Buffer(idx)=V_HV;
            obj.V_HV_Filtered=obj.V_HV_Sum/125;
            obj.V_LV_Sum=obj.V_LV_Sum-obj.V_LV_Buffer(idx)+V_LV; obj.V_LV_Buffer(idx)=V_LV;
            obj.V_LV_Filtered=obj.V_LV_Sum/125;
            obj.I_HV_Sum=obj.I_HV_Sum-obj.I_HV_Buffer(idx)+I_HV; obj.I_HV_Buffer(idx)=I_HV;
            obj.I_HV_Filtered=obj.I_HV_Sum/125;
            obj.I_LV_Sum=obj.I_LV_Sum-obj.I_LV_Buffer(idx)+I_LV; obj.I_LV_Buffer(idx)=I_LV;
            obj.I_LV_Filtered=obj.I_LV_Sum/125;
            obj.V_Grid_Sum=obj.V_Grid_Sum-obj.V_Grid_Buffer(idx)+V_Grid; obj.V_Grid_Buffer(idx)=V_Grid;
            obj.V_Grid_Filtered=obj.V_Grid_Sum/125;

            obj.BufferIdx=obj.BufferIdx+1; if obj.BufferIdx>125, obj.BufferIdx=1; end
            V_HV_f=obj.V_HV_Filtered; V_LV_f=obj.V_LV_Filtered;
            I_HV_f=obj.I_HV_Filtered; V_Grid_f=obj.V_Grid_Filtered;

            % =================================================================
            % [STAGE 1.5] SOC Estimation (Voltage-Based, from EPS.m)
            % =================================================================
            V_LV_safe = max(V_LV_f, 1);
            if V_LV_safe >= obj.V_Min_Limit && V_LV_safe <= obj.V_Pack_Max
                SOC_percent = 100 * (V_LV_safe - obj.V_Min_Limit) / (obj.V_Pack_Max - obj.V_Min_Limit);
            elseif V_LV_safe > obj.V_Pack_Max
                SOC_percent = 100;
            else
                SOC_percent = 0;
            end

            % =================================================================
            % [STAGE 2] FSM
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
                        % 초기화 중에는 계통 전압 확인
                        if (V_Grid_f<(obj.V_HV_Ref-obj.V_HV_Band))||(V_Grid_f>(obj.V_HV_Ref+obj.V_HV_Band))
                            NextState=obj.ST_UPS_RUN;
                        else, NextState=obj.ST_ESS_RUN; end
                    case obj.ST_ESS_RUN
                        % 계통 연결 운전 중: V_Grid 감시 (V_HV가 아님)
                        % 외부 계통 전압이 무너지면 즉시 분리 후 UPS 모드 진입
                        if (V_Grid_f<(obj.V_HV_Ref-obj.V_HV_Band))||(V_Grid_f>(obj.V_HV_Ref+obj.V_HV_Band))
                            NextState=obj.ST_UPS_RUN; end
                    case obj.ST_UPS_RUN
                        % UPS(단독) 운전 중: V_Grid 복구 감시
                        % 외부 계통이 정상 범위로 돌아오면 재연계 준비 (상태 복귀)
                        if (V_Grid_f>=(obj.V_HV_Ref-obj.V_HV_Band))&&(V_Grid_f<=(obj.V_HV_Ref+obj.V_HV_Band))
                            NextState=obj.ST_ESS_RUN; end
                    case obj.ST_MANUAL
                        if Mode_Select~=1
                            if (V_Grid_f<(obj.V_HV_Ref-obj.V_HV_Band))||(V_Grid_f>(obj.V_HV_Ref+obj.V_HV_Band))
                                NextState=obj.ST_UPS_RUN;
                            else, NextState=obj.ST_ESS_RUN; end
                        end
                    case obj.ST_FAULT
                        if V_LV_f>(obj.V_Min_Limit+5.0), NextState=obj.ST_INIT; end
                end
            end

            if NextState~=obj.CurrentState
                if NextState==obj.ST_UPS_RUN||NextState==obj.ST_INIT
                    obj.Integrator_v=0; obj.Iref_soft=0;
                    % [UPS 진입 초기화] 목표 전압 램프를 현재 전압(Instant V_HV)부터 시작
                    % 필터링된 값(V_HV_f)은 지연이 있을 수 있으므로 raw V_HV 사용
                    obj.V_Ref_Ramp = max(0, min(obj.V_HV_Ref, V_HV)); 
                end
                if NextState==obj.ST_ESS_RUN, obj.Iref_soft=0; end
            end
            obj.CurrentState = NextState;

            % =================================================================
            % [STAGE 3] Reference Generation
            % =================================================================
            PWM_Enable=0; I_target=0; mode_active=0; raw_ref=0;

            switch obj.CurrentState
                case obj.ST_INIT, obj.Integrator_v=0;
                case obj.ST_FAULT  % Shutdown
                case obj.ST_MANUAL
                    PWM_Enable=1; mode_active=4; I_target=0;

                case {obj.ST_ESS_RUN, obj.ST_UPS_RUN}
                    PWM_Enable = 1;

                    if obj.CurrentState == obj.ST_ESS_RUN
                        % =====================================================
                        % ESS Mode: TOU 스케줄링 + 부하추종 (from EPS.m)
                        % =====================================================
                        current_hour = mod(Time_Hour, 24);
                        is_off_peak = (current_hour>=obj.TOU_OffPeak_Start)||(current_hour<obj.TOU_OffPeak_End);
                        is_on_peak = (current_hour>=obj.TOU_OnPeak1_Start&&current_hour<obj.TOU_OnPeak1_End)|| ...
                                     (current_hour>=obj.TOU_OnPeak2_Start&&current_hour<obj.TOU_OnPeak2_End);

                        if is_off_peak
                            % --- 경부하: 충전 ---
                            target_ref = -obj.ESS_Charge_Rate;
                            mode_active = 3;
                        elseif is_on_peak
                            % --- 최대부하: 방전 ---
                            target_ref = obj.ESS_Discharge_Rate;
                            mode_active = 2;
                        else
                            % --- 중간부하: 부하추종 (from EPS.m) ---
                            % I_HV_f = 계통측 부하전류
                            V_HV_safe = max(V_HV_f, 1);
                            I_threshold = obj.P_load_threshold / V_HV_safe;
                            I_load = I_HV_f;

                            if I_load > I_threshold
                                % 부하 초과분만큼 방전 (Peak-Shaving)
                                I_dab_hv = I_load - I_threshold;
                                % 전력 제한 (from EPS.m P_DAB_max)
                                P_DAB = I_dab_hv * V_HV_safe;
                                P_DAB = min(P_DAB, obj.P_DAB_max);
                                I_dab_hv = P_DAB / V_HV_safe;
                                % HV→LV 전류 변환
                                target_ref = I_dab_hv * obj.n_transformer;
                                % SOC 보호: SOC 20% 이하 방전 금지
                                if SOC_percent <= obj.SOC_Min_Threshold
                                    target_ref = 0;
                                end
                                mode_active = 2;
                            else
                                % 부하 여유분으로 충전 가능
                                if abs(I_ess_cmd) > 0.1
                                    target_ref = I_ess_cmd;  % 외부 명령 우선
                                else
                                    target_ref = 0;  % 대기
                                end
                                mode_active = 3;
                            end
                        end

                        raw_ref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, target_ref));

                        % --- LV측 방전 전류 하한 보호 (from EPS.m) ---
                        if raw_ref > 0
                            I_LV_equiv = raw_ref;
                            if I_LV_equiv > abs(obj.I_LV_min_limit)
                                raw_ref = abs(obj.I_LV_min_limit);
                            end
                        end

                    else
                        % =====================================================
                        % UPS Mode: 전압 PI → 전류 지령
                        % =====================================================
                        mode_active = 1;

                        % [UPS Logic: Adaptive Soft Voltage Recovery]
                        % 목표 전압에 근접하면 감속하여 오버슈트 방지 및 부드러운 안착 유도
                        
                        dist = obj.V_HV_Ref - obj.V_Ref_Ramp;
                        Decel_Zone = 30; % 감속 시작 구간 (V)
                        Min_Rate = 10;   % 최저 속도 (V/s)
                        
                        if dist > 0
                            if dist < Decel_Zone
                                % 목표에 가까워지면 속도 줄임 (Linear Reduction)
                                current_rate = obj.V_Ref_Ramp_Rate * (dist / Decel_Zone);
                                current_rate = max(current_rate, Min_Rate);
                            else
                                % 일반 구간: 최대 속도
                                current_rate = obj.V_Ref_Ramp_Rate;
                            end
                            
                            obj.V_Ref_Ramp = obj.V_Ref_Ramp + current_rate * Ts;
                            
                            % Clamp to generic max
                            if obj.V_Ref_Ramp > obj.V_HV_Ref
                                obj.V_Ref_Ramp = obj.V_HV_Ref; 
                            end
                        else
                            % 이미 목표 도달
                            obj.V_Ref_Ramp = obj.V_HV_Ref;
                        end

                        % [UPS PI Control with Feedforward & Anti-Windup]
                        % 목표치(V_Ref_Ramp)와 현재값(V_HV_f)의 차이로 제어
                        err_v = obj.V_Ref_Ramp - V_HV_f;
                        
                        % Proportional Term
                        P_term = Kpv * err_v;
                        
                        % Integral Term (Previous State)
                        I_term = Kiv * obj.Integrator_v;
                        
                        % Feedforward Term (Load Current Compensation)
                        % 부하 전류(I_HV)를 미리 더해줌으로써 전압이 떨어지기 전에 대응
                        % I_HV가 양수(=부하 소비)일 때 방전(+)해야 하므로 더해줌
                        I_FF = I_HV_f; 
                        
                        % Total Command
                        temp_I_cmd = P_term + I_term + I_FF;
                        limit_val = obj.I_UPS_Max;
                        
                        % Output Limiting & Conditional Integration (Clamping)
                        if temp_I_cmd > limit_val
                            raw_ref = limit_val;
                            % Saturation Check: Only integrate if error reduces the output (unwinding)
                            if err_v < 0
                                obj.Integrator_v = obj.Integrator_v + err_v * Ts;
                            end
                        elseif temp_I_cmd < -limit_val
                            raw_ref = -limit_val;
                            % Saturation Check: Only integrate if error increases the output (unwinding)
                            if err_v > 0
                                obj.Integrator_v = obj.Integrator_v + err_v * Ts;
                            end
                        else
                            % Normal Operation: Integrate freely
                            raw_ref = temp_I_cmd;
                            obj.Integrator_v = obj.Integrator_v + err_v * Ts;
                        end
                    end

                    % =========================================================
                    % [STAGE 4] BMS Safety Limiter
                    % =========================================================
                    I_safe = raw_ref; v_batt = V_LV_f;
                    I_safe = max(-obj.I_Charge_Limit, min(obj.I_Discharge_Limit, I_safe));
                    if raw_ref < 0  % Charging
                        if v_batt >= obj.V_Pack_Max, I_safe=0;
                        elseif v_batt >= obj.V_CV_Start
                            I_limit = (obj.V_Pack_Max - v_batt)*obj.K_CV;
                            I_safe = max(raw_ref, -I_limit);
                        end
                        if abs(I_safe)<0.1 && v_batt>obj.V_CV_Start, I_safe=0; end
                    elseif raw_ref > 0  % Discharging
                        if obj.CurrentState==obj.ST_ESS_RUN
                            if v_batt<obj.V_ESS_Limit, I_safe=0; end
                        else
                            if v_batt<obj.V_UPS_Limit
                                factor=max(0,min(1,(v_batt-obj.V_Min_Limit)/(obj.V_UPS_Limit-obj.V_Min_Limit)));
                                I_safe=raw_ref*factor;
                            end
                        end
                    end

                    % =========================================================
                    % [STAGE 5] Soft-Start Ramp (from EPS.m)
                    %   ESS 지령의 급격한 변화를 방지
                    %   UPS는 비상 모드이므로 램프 미적용
                    % =========================================================
                    if obj.CurrentState == obj.ST_ESS_RUN
                        delta_i = I_safe - obj.Iref_soft;
                        max_step = obj.Iref_ramp_rate * Ts;
                        if abs(delta_i) > max_step
                            obj.Iref_soft = obj.Iref_soft + sign(delta_i)*max_step;
                        else
                            obj.Iref_soft = I_safe;
                        end
                        I_target = obj.Iref_soft;
                    else
                        % UPS: 즉시 반영 (비상 운전)
                        I_target = I_safe;
                        obj.Iref_soft = I_safe;
                    end
            end

            % Status
            status_flag = double((obj.CurrentState==obj.ST_UPS_RUN)||(obj.CurrentState==obj.ST_FAULT));
            if obj.CurrentState==obj.ST_INIT||obj.CurrentState==obj.ST_FAULT||obj.CurrentState==obj.ST_MANUAL
                mode_active=0;
            end
            
            % Grid Breaker Control
            % 1: Connected (ESS Mode), 0: Disconnected (UPS Mode, Fault, Init)
            if obj.CurrentState == obj.ST_ESS_RUN
                Grid_Connect = 1;
            else
                Grid_Connect = 0; % UPS 동작 시 계통 분리 (Islanding)
            end
            
            I_cmd = I_target;
            obj.PreviousMode = obj.CurrentState;
            
            % Debug Outputs
            V_Ref_Debug = obj.V_Ref_Ramp;
            I_Int_Debug = obj.Integrator_v;
        end

        function resetImpl(obj)
            obj.CurrentState=obj.ST_INIT; obj.PreviousMode=obj.ST_INIT;
            obj.Integrator_v=0; obj.Iref_soft=0;
            obj.V_HV_Buffer=zeros(125,1); obj.V_HV_Sum=0; obj.V_HV_Filtered=0;
            obj.V_LV_Buffer=zeros(125,1); obj.V_LV_Sum=0; obj.V_LV_Filtered=0;
            obj.I_HV_Buffer=zeros(125,1); obj.I_HV_Sum=0; obj.I_HV_Filtered=0;
            obj.I_LV_Buffer=zeros(125,1); obj.I_LV_Sum=0; obj.I_LV_Filtered=0;
            obj.BufferIdx=1; obj.TickCount=0;
        end

        function num=getNumInputsImpl(~), num=10; end % +V_Grid
        function num=getNumOutputsImpl(~), num=8; end  % +Debug Ports
        function [s1,s2,s3,s4,s5,s6,s7,s8]=getOutputSizeImpl(~), s1=[1 1];s2=[1 1];s3=[1 1];s4=[1 1];s5=[1 1];s6=[1 1];s7=[1 1];s8=[1 1]; end
        function [d1,d2,d3,d4,d5,d6,d7,d8]=getOutputDataTypeImpl(~), d1='double';d2='double';d3='double';d4='double';d5='double';d6='double';d7='double';d8='double'; end
        function [c1,c2,c3,c4,c5,c6,c7,c8]=isOutputComplexImpl(~), c1=false;c2=false;c3=false;c4=false;c5=false;c6=false;c7=false;c8=false; end
        function [f1,f2,f3,f4,f5,f6,f7,f8]=isOutputFixedSizeImpl(~), f1=true;f2=true;f3=true;f4=true;f5=true;f6=true;f7=true;f8=true; end
        function [sz,dt,cp]=getDiscreteStateSpecificationImpl(~,name)
            switch name
                case {'CurrentState','PreviousMode','Integrator_v','V_HV_Sum','V_LV_Sum', ...
                      'I_HV_Sum','I_LV_Sum','V_HV_Filtered','V_LV_Filtered', ...
                      'I_HV_Filtered','I_LV_Filtered','BufferIdx','TickCount','Iref_soft', ...
                      'V_Grid_Sum','V_Grid_Filtered','V_Ref_Ramp'}
                    sz=[1 1]; dt='double'; cp=false;
                case {'V_HV_Buffer','V_LV_Buffer','I_HV_Buffer','I_LV_Buffer','V_Grid_Buffer'}
                    sz=[125 1]; dt='double'; cp=false;
                otherwise, error(['Unknown state: ',name]);
            end
        end
        function sts=getSampleTimeImpl(obj), sts=createSampleTime(obj,'Type','Discrete','SampleTime',obj.Ts_ems,'OffsetTime',0); end
        function [n1,n2,n3,n4,n5,n6,n7,n8,n9,n10]=getInputNamesImpl(~)
            n1='V_HV';n2='I_HV';n3='V_LV';n4='I_LV';n5='I_ess_cmd';n6='Kpv';n7='Kiv';n8='Mode_Select';n9='Time_Hour';n10='V_Grid';
        end
        function [o1,o2,o3,o4,o5,o6,o7,o8]=getOutputNamesImpl(~)
            o1='I_cmd';o2='PWM_Enable';o3='Active_Mode';o4='Status_Flag';o5='SOC_pct';o6='Grid_SW';o7='V_Ref_Debug';o8='I_Int_Debug';
        end
    end

    methods (Static, Access = protected)
        function header=getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'),'Title','Smart Energy Manager (v1.3)', ...
                'Text','TOU + 부하추종 + SOC 출력 + Soft-start 기능이 포함된 EMS입니다.');
        end
        function groups=getPropertyGroupsImpl
            g1=matlab.system.display.Section('Title','1. System','PropertyList',{'Ts_ems','InitDelay'});
            g2=matlab.system.display.Section('Title','2. Operation','PropertyList',{'V_HV_Ref','V_HV_Band','V_Ref_Ramp_Rate','I_ESS_Max','I_UPS_Max'});
            g3=matlab.system.display.Section('Title','3. BMS','PropertyList',{'V_Pack_Max','V_CV_Start','K_CV','V_ESS_Limit','V_UPS_Limit','V_Min_Limit','I_Charge_Limit','I_Discharge_Limit'});
            g4=matlab.system.display.Section('Title','4. TOU Schedule','PropertyList',{'ESS_Charge_Rate','ESS_Discharge_Rate','TOU_OffPeak_Start','TOU_OffPeak_End','TOU_OnPeak1_Start','TOU_OnPeak1_End','TOU_OnPeak2_Start','TOU_OnPeak2_End','SOC_Max_Threshold','SOC_Min_Threshold'});
            g5=matlab.system.display.Section('Title','5. Load-Following (from EPS)','PropertyList',{'P_load_threshold','P_DAB_max','n_transformer','I_LV_min_limit','Iref_ramp_rate'});
            groups=[g1,g2,g3,g4,g5];
        end
    end
end
