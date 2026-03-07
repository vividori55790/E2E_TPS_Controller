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
    % 출력: Power_cmd, PWM_Enable, mode_active, status_flag, SOC_percent
    %
    % [부호 규약] Power_cmd (+)=방전, (-)=충전 (받는 쪽 전력 기준)
    %
    % =========================================================================
    properties(Nontunable)
        Ts_ems = 1e-7;          % 제어기 실행 주기 [초] (기본: 1e-7)
        V_HV_Ref = 380;         % [UPS] 정전 시 유지할 목표 전압 [V] (예: 380V)
        V_HV_Band = 30;         % [UPS] 정전 감지 전압 범위 [±V] (이 범위를 벗어나면 비상 모드 진입)
        V_Ref_Ramp_Rate = 500;  % [UPS] 계통 복구 시 부드러운 전압 회복 속도 [V/s]

        % [BMS] 배터리 보호 임계값 (26S Li-ion 기준)
        V_Pack_Max = 109.2;     % 배터리 만충 전압 [V] (이 이상은 과충전으로 간주해 절대 충전 금지)
        V_CV_Start = 108.0;     % 정전압(CV) 충전 시작 전압 [V] (만충이 가까워지면 충전속도를 서서히 줄이기 시작함)
        V_ESS_Limit = 93.0;     % 평상시 ESS 방전 중단 전압 [V] (방전 중 이 전압 이하로 떨어지면 보호 대기)
        V_UPS_Limit = 87.0;     % 비상시(UPS) 출력 축소(Derating) 시작 전압 [V]
        V_Min_Limit = 84.0;     % 시스템 완전 셧다운 전압 [V] (배터리 손상 방지)

        I_Charge_Limit = 30;    % 배터리 측 최대 강제 충전 허용 전류 [A]
        I_Discharge_Limit = 50; % 배터리 측 최대 강제 방전 허용 전류 [A] (하드웨어 한계)
        K_CV = 5.0;             % 정전압 충전 게인 [A/V] (충전 마무리 시 전류를 부드럽게 줄이기 위한 값)
        InitDelay = 0.001;      % 시스템 시작 후 안정화 대기 시간 [초]

        % [ESS] 시간대별 전기 요금 최적화(TOU) 설정
        ESS_Charge_Rate = 10;   % 심야 경부하 구간의 자동 충전 스피드 [A]
        ESS_Discharge_Rate = 10;% 주간 피크 구간의 자동 방전 스피드 [A]
        TOU_OffPeak_Start = 23; % 심야 충전 모드 켜짐 시간 [시]
        TOU_OffPeak_End = 8;    % 심야 충전 모드 꺼짐 시간 [시]
        TOU_OnPeak1_Start = 11; % 낮시간 1차 배터리 방전 시작 시간 [시]
        TOU_OnPeak1_End = 12;   % 낮시간 1차 배터리 방전 종료 시간 [시]
        TOU_OnPeak2_Start = 13; % 오후시간 2차 배터리 방전 시작 시간 [시]
        TOU_OnPeak2_End = 17;   % 오후시간 2차 배터리 방전 종료 시간 [시]
        SOC_Max_Threshold = 95; % 배터리 목표 충전 한도 [%] (이 수치 도달시 충전 중단)
        SOC_Min_Threshold = 20; % 배터리 방전 마지노선 [%] (이 이하로는 피크 저감을 안 함)

        % [부하추종] 수요 피크 컷아웃(Peak-Shaving) 및 컨버터 한계 설정
        P_load_threshold = 500; % 이 전력을 넘어가면 배터리가 도와서 방전을 시작함 [W]
        P_DAB_max = 1000;       % 변환기가 버틸 수 있는 최대 전력 파워 한계치 [W]
        n_transformer = 4;      % 내부 고주파 변압기 전압 턴비 (HV/LV = 380/95)
        I_LV_min_limit = -15;   % 방전 시 배터리 측에서 나갈 수 있는 최대 순간 방전량 제한 [A] (절대값 보호)
    end

    properties(Constant)
        ST_INIT=1; ST_ESS_RUN=2; ST_UPS_RUN=3; ST_MANUAL=4; ST_FAULT=5;
    end

    properties
        I_ESS_Max = 10;         % 평상시 ESS 운영에서 내보낼 수 있는 최대 연속 전류 한계 [A]
        I_UPS_Max = 30;         % 정전 비상상황(UPS) 시 전압 유지를 위해 끌어다 쓸 수 있는 최대 전류 한계 [A]
    end

    properties(DiscreteState)
        CurrentState; PreviousMode; Integrator_v;
        V_HV_Buffer; V_HV_Sum; V_HV_Filtered;
        V_LV_Buffer; V_LV_Sum; V_LV_Filtered;
        I_HV_Buffer; I_HV_Sum; I_HV_Filtered;
        I_LV_Buffer; I_LV_Sum; I_LV_Filtered;
        BufferIdx; TickCount;
        V_Grid_Buffer; V_Grid_Sum; V_Grid_Filtered; % [New for UPS] 계통 전압 감지
        V_Ref_Ramp; % [NEW] UPS 전압 지령 램프 (Soft-Recovery)
    end

    methods(Access = protected)
        function setupImpl(obj)
            obj.CurrentState=obj.ST_INIT; obj.PreviousMode=obj.ST_INIT;
            obj.Integrator_v=0; obj.V_Ref_Ramp=0;
            obj.V_HV_Buffer=zeros(125,1); obj.V_HV_Sum=0; obj.V_HV_Filtered=0;
            obj.V_LV_Buffer=zeros(125,1); obj.V_LV_Sum=0; obj.V_LV_Filtered=0;
            obj.I_HV_Buffer=zeros(125,1); obj.I_HV_Sum=0; obj.I_HV_Filtered=0;
            obj.I_LV_Buffer=zeros(125,1); obj.I_LV_Sum=0; obj.I_LV_Filtered=0;
            obj.V_Grid_Buffer=zeros(125,1); obj.V_Grid_Sum=0; obj.V_Grid_Filtered=0;
            obj.BufferIdx=1; obj.TickCount=0;
        end

        function [Power_cmd, PWM_Enable, mode_active, status_flag, SOC_percent, Grid_Connect, V_Ref_Debug, I_Int_Debug] = ...
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
                    obj.Integrator_v=0;
                    % [UPS 진입 초기화] 목표 전압 램프를 현재 전압(Instant V_HV)부터 시작
                    % 필터링된 값(V_HV_f)은 지연이 있을 수 있으므로 raw V_HV 사용
                    obj.V_Ref_Ramp = max(0, min(obj.V_HV_Ref, V_HV)); 
                end
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
                    % [STAGE 5] BMS Safety Limiter Applied to Target
                    % =========================================================
                    I_target = I_safe;
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
            
            % [전력 변환: 받는 쪽 기준]
            %   충전(I_target<0): 배터리가 받는 쪽 → P = I_target * V_LV
            %   방전(I_target>0): HV 부하가 받는 쪽 → P = I_target * V_HV / n
            %   (부호는 유지: (+)=방전, (-)=충전)
            V_LV_safe_out = max(V_LV_f, 1);
            V_HV_safe_out = max(V_HV_f, 1);
            if I_target <= 0
                % 충전: 배터리측 전력
                Power_cmd = I_target * V_LV_safe_out;
            else
                % 방전/UPS: HV 부하측 전력 (LV 전류 → HV 전력 변환)
                Power_cmd = I_target * V_LV_safe_out;
            end
            
            obj.PreviousMode = obj.CurrentState;
            
            % Debug Outputs
            V_Ref_Debug = obj.V_Ref_Ramp;
            I_Int_Debug = obj.Integrator_v;
        end

        function resetImpl(obj)
            obj.CurrentState=obj.ST_INIT; obj.PreviousMode=obj.ST_INIT;
            obj.Integrator_v=0;
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
                      'I_HV_Filtered','I_LV_Filtered','BufferIdx','TickCount', ...
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
            o1='Power_cmd';o2='PWM_Enable';o3='Active_Mode';o4='Status_Flag';o5='SOC_pct';o6='Grid_SW';o7='V_Ref_Debug';o8='I_Int_Debug';
        end
    end

    methods (Static, Access = protected)
        function header=getHeaderImpl
            header=matlab.system.display.Header(mfilename('class'),'Title','스마트 에너지 통합 관리 시스템 (Smart Energy Manager)', ...
                'Text','이 블록은 전력망(Grid)과 배터리 간의 똑똑한 에너지 흐름 제어를 담당합니다.\n평상시에는 시간에 따른 자동 충방전(스케줄링)과 부족한 전력을 배터리가 보태주는 파워 서포트(Peak-shaving)를 수행하며,\n정전 감지 시 즉각 단독 모드(UPS)로 전환되어 중요 부하에 일정한 전압을 멈춤 없이 쏴줍니다.');
        end
        function groups=getPropertyGroupsImpl
            g1=matlab.system.display.Section('Title','1. 기본 시스템 타이밍 (System)','PropertyList',{'Ts_ems','InitDelay'});
            g2=matlab.system.display.Section('Title','2. 비상 정전 방어 및 전압 유지 제한치 (UPS Operation)','PropertyList',{'V_HV_Ref','V_HV_Band','V_Ref_Ramp_Rate','I_ESS_Max','I_UPS_Max'});
            g3=matlab.system.display.Section('Title','3. 배터리 과충전/과방전 절대 보호선 (BMS Settings)','PropertyList',{'V_Pack_Max','V_CV_Start','K_CV','V_ESS_Limit','V_UPS_Limit','V_Min_Limit','I_Charge_Limit','I_Discharge_Limit'});
            g4=matlab.system.display.Section('Title','4. 시간대별 자동 루틴 충방전 스케줄 (Time Of Use)','PropertyList',{'ESS_Charge_Rate','ESS_Discharge_Rate','TOU_OffPeak_Start','TOU_OffPeak_End','TOU_OnPeak1_Start','TOU_OnPeak1_End','TOU_OnPeak2_Start','TOU_OnPeak2_End','SOC_Max_Threshold','SOC_Min_Threshold'});
            g5=matlab.system.display.Section('Title','5. 피크치 전력 서포트 및 물리적 컨버터 한계 (Limits)','PropertyList',{'P_load_threshold','P_DAB_max','n_transformer','I_LV_min_limit'});
            groups=[g1,g2,g3,g4,g5];
        end
    end
end
