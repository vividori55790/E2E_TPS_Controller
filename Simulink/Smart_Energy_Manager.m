classdef Smart_Energy_Manager < matlab.System
    % Smart_Energy_Manager  (v1.1 — Integrated Current Feedback)
    % DAB 변환기용 상위 에너지 관리 시스템
    %
    % 역할: "무엇을 할지" 결정하는 두뇌(Brain). 전류 지령(I_cmd)을 생성하여
    %       하위 제어기(DAB_Controller_System)에 전달.
    %
    % 아키텍처:
    %   1. 계통 전압 감시 및 정전 감지
    %   2. FSM (State Transition: INIT → ESS/UPS/FAULT)
    %   3. [UPS] 전압 제어 루프 (380V 유지 → 전류 지령 생성)
    %   4. [ESS] TOU 기반 경제 운전 스케줄링
    %   5. BMS 안전 리미터 (충/방전 한계 적용)
    %   6. 최종 I_cmd 출력
    %
    % 입력: V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kpv, Kiv, Mode_Select
    % 출력: I_cmd, PWM_Enable, mode_active, status_flag
    %
    % [부호 규약]
    %   - I_cmd (+) = 방전 (배터리→계통), (-) = 충전 (계통→배터리)
    %
    % =========================================================================
    % 1. Public, Nontunable Properties
    % =========================================================================
    properties(Nontunable)
        Ts_ems = 1e-7;          % EMS 샘플링 시간 (Ts) [sec] — 하위 제어기와 동일

        % HV 전압 목표 및 밴드 범위
        V_HV_Ref = 380;         % [UPS] HV 목표 전압 (V)
        V_HV_Band = 30;         % [UPS] 정전 감지 오차 범위 (±V)

        % [BMS] 배터리 보호 임계값 (26S Li-ion Spec)
        V_Pack_Max = 109.2;     % 배터리 최대 전압 (Cell*4.2V)
        V_CV_Start = 108.0;     % CV 충전 시작 전압
        V_ESS_Limit = 93.0;     % 방전 중단 전압 (ESS Reserve)
        V_UPS_Limit = 87.0;     % 출력 제한 시작 전압 (UPS Derating)
        V_Min_Limit = 84.0;     % 셧다운 최저 전압 (System UVLO)
        
        I_Charge_Limit = 30;    % BMS 최대 충전 전류 (A)
        I_Discharge_Limit = 50; % BMS 최대 방전 전류 (A)

        K_CV = 5.0;             % CV 제어 게인 (A/V)

        % [FSM] 초기화 지연
        InitDelay = 0.001;      % FSM 초기화 지연 시간 (sec) — 센서 안정화 대기

        % [ESS] TOU 스케줄링 설정
        ESS_Charge_Rate = 10;   % TOU 충전 시 정격 전류 (A)
        ESS_Discharge_Rate = 10;% TOU 방전 시 최대 전류 (A)

        % TOU 시간대 설정 (시 단위, 0~23)
        TOU_OffPeak_Start = 23; % 경부하 시작 시각
        TOU_OffPeak_End = 8;    % 경부하 종료 시각
        TOU_OnPeak1_Start = 11; % 첨두부하 구간 1 시작
        TOU_OnPeak1_End = 12;   % 첨두부하 구간 1 종료
        TOU_OnPeak2_Start = 13; % 첨두부하 구간 2 시작
        TOU_OnPeak2_End = 17;   % 첨두부하 구간 2 종료

        % [ESS] SOC 기반 보호 임계값
        SOC_Max_Threshold = 95; % 충전 중단 SOC (%)
        SOC_Min_Threshold = 20; % 방전 중단 SOC (%)
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
    % 2. Internal States (DiscreteState)
    % =========================================================================
    properties(DiscreteState)
        CurrentState;           % [FSM] 현재 상태
        PreviousMode;           % [FSM] 이전 상태 (Transition 감지용)
        Integrator_v;           % 전압 PI 적분항 (UPS 모드)

        % [Voltage/Current Filtering] 이동 평균 버퍼
        V_HV_Buffer; V_HV_Sum; V_HV_Filtered;
        V_LV_Buffer; V_LV_Sum; V_LV_Filtered;
        I_HV_Buffer; I_HV_Sum; I_HV_Filtered;
        I_LV_Buffer; I_LV_Sum; I_LV_Filtered;
        BufferIdx;

        % [Time] 틱 카운터
        TickCount;
    end

    % =========================================================================
    % 3. Methods
    % =========================================================================
    methods(Access = protected)

        function setupImpl(obj)
            obj.CurrentState = obj.ST_INIT;
            obj.PreviousMode = obj.ST_INIT;
            obj.Integrator_v = 0;

            obj.V_HV_Buffer = zeros(125,1); obj.V_HV_Sum = 0; obj.V_HV_Filtered = 0;
            obj.V_LV_Buffer = zeros(125,1); obj.V_LV_Sum = 0; obj.V_LV_Filtered = 0;
            obj.I_HV_Buffer = zeros(125,1); obj.I_HV_Sum = 0; obj.I_HV_Filtered = 0;
            obj.I_LV_Buffer = zeros(125,1); obj.I_LV_Sum = 0; obj.I_LV_Filtered = 0;
            obj.BufferIdx = 1;
            obj.TickCount = 0;
        end

        function [I_cmd, PWM_Enable, mode_active, status_flag] = ...
                stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Kpv, Kiv, Mode_Select)

            Ts = obj.Ts_ems;
            t_current = double(obj.TickCount) * Ts;
            obj.TickCount = obj.TickCount + 1;

            % =================================================================
            % [STAGE 1] Sensing & Filtering (125-sample Moving Average)
            % =================================================================
            idx = obj.BufferIdx;

            % HV Voltage
            obj.V_HV_Sum = obj.V_HV_Sum - obj.V_HV_Buffer(idx) + V_HV;
            obj.V_HV_Buffer(idx) = V_HV;
            obj.V_HV_Filtered = obj.V_HV_Sum / 125;

            % LV Voltage
            obj.V_LV_Sum = obj.V_LV_Sum - obj.V_LV_Buffer(idx) + V_LV;
            obj.V_LV_Buffer(idx) = V_LV;
            obj.V_LV_Filtered = obj.V_LV_Sum / 125;
            
            % HV Current
            obj.I_HV_Sum = obj.I_HV_Sum - obj.I_HV_Buffer(idx) + I_HV;
            obj.I_HV_Buffer(idx) = I_HV;
            obj.I_HV_Filtered = obj.I_HV_Sum / 125;

            % LV Current (Battery Current)
            obj.I_LV_Sum = obj.I_LV_Sum - obj.I_LV_Buffer(idx) + I_LV;
            obj.I_LV_Buffer(idx) = I_LV;
            obj.I_LV_Filtered = obj.I_LV_Sum / 125;

            % Buffer Index Update
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > 125, obj.BufferIdx = 1; end

            V_HV_f = obj.V_HV_Filtered;
            V_LV_f = obj.V_LV_Filtered;
            % I_HV_f = obj.I_HV_Filtered; % (Future Use: Load Estimation)
            % I_LV_f = obj.I_LV_Filtered; % (Used for BMS checks)

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

            % Transition detect: Reset integrator on mode change
            if NextState ~= obj.CurrentState
                if NextState == obj.ST_UPS_RUN || NextState == obj.ST_INIT
                    obj.Integrator_v = 0;
                end
            end
            obj.CurrentState = NextState;

            % =================================================================
            % [STAGE 3] Reference Generation
            %   UPS: 전압 PI 제어 → 전류 지령 생성
            %   ESS: 외부 I_ess_cmd를 수용 (또는 TOU 자동 스케줄링)
            % =================================================================
            PWM_Enable = 0;
            I_target = 0;
            mode_active = 0;

            switch obj.CurrentState
                case obj.ST_INIT
                    obj.Integrator_v = 0;

                case obj.ST_FAULT
                    % Hard Shutdown — I_cmd = 0, PWM OFF

                case obj.ST_MANUAL
                    PWM_Enable = 1;
                    mode_active = 4;
                    I_target = 0; % Manual은 하위 제어기에서 직접 처리

                case {obj.ST_ESS_RUN, obj.ST_UPS_RUN}
                    PWM_Enable = 1;

                    if obj.CurrentState == obj.ST_ESS_RUN
                        % --- ESS 모드: 외부 명령 또는 TOU 스케줄링 ---
                        raw_ref = max(-obj.I_ESS_Max, min(obj.I_ESS_Max, I_ess_cmd));
                        mode_active = 3;
                    else
                        % --- UPS 모드: 전압 PI 제어 → 전류 지령 생성 ---
                        mode_active = 1;
                        err_v = obj.V_HV_Ref - V_HV_f;
                        temp_int_v = obj.Integrator_v + err_v * Ts;
                        limit_val = obj.I_UPS_Max;
                        temp_I_cmd = Kpv * err_v + Kiv * temp_int_v;

                        % Anti-Windup (Conditional Integration)
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

                    % =============================================================
                    % [STAGE 4] BMS Safety Limiter
                    % =============================================================
                    I_safe = raw_ref;
                    v_batt = V_LV_f;

                    % 1) 하드웨어 전류 한계 (Absolute Maximum)
                    I_safe = max(-obj.I_Charge_Limit, min(obj.I_Discharge_Limit, I_safe));

                    % 2) CV (Constant Voltage) 충전 제어
                    if raw_ref < 0  % Charging
                        if v_batt >= obj.V_Pack_Max
                            I_safe = 0;
                        elseif v_batt >= obj.V_CV_Start
                            I_limit = (obj.V_Pack_Max - v_batt) * obj.K_CV;
                            I_safe = max(raw_ref, -I_limit);
                        end
                        % 전류가 너무 작아지면 충전 중단 (Cut-off)
                        if abs(I_safe) < 0.1 && v_batt > obj.V_CV_Start
                            I_safe = 0;
                        end
                        
                    elseif raw_ref > 0  % Discharging
                        if obj.CurrentState == obj.ST_ESS_RUN
                            if v_batt < obj.V_ESS_Limit, I_safe = 0; end
                        else
                            % UPS 모드에서는 가능한 한 방전하되, UVLO 직전 스로틀링
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
            % [OUTPUT]
            % =================================================================
            I_cmd = I_target;

            obj.PreviousMode = obj.CurrentState;
        end

        function resetImpl(obj)
            obj.CurrentState = obj.ST_INIT;
            obj.PreviousMode = obj.ST_INIT;
            obj.Integrator_v = 0;

            obj.V_HV_Buffer = zeros(125,1); obj.V_HV_Sum = 0; obj.V_HV_Filtered = 0;
            obj.V_LV_Buffer = zeros(125,1); obj.V_LV_Sum = 0; obj.V_LV_Filtered = 0;
            obj.I_HV_Buffer = zeros(125,1); obj.I_HV_Sum = 0; obj.I_HV_Filtered = 0;
            obj.I_LV_Buffer = zeros(125,1); obj.I_LV_Sum = 0; obj.I_LV_Filtered = 0;
            obj.BufferIdx = 1;
            obj.TickCount = 0;
        end

        function num = getNumInputsImpl(~)
            num = 8; % V_HV, I_HV(new), V_LV, I_LV(new), I_ess_cmd, Kpv, Kiv, Mode_Select
        end

        function num = getNumOutputsImpl(~)
            num = 4;
        end

        function [sz1,sz2,sz3,sz4] = getOutputSizeImpl(~)
            sz1 = [1 1]; sz2 = [1 1]; sz3 = [1 1]; sz4 = [1 1];
        end

        function [dt1,dt2,dt3,dt4] = getOutputDataTypeImpl(~)
            dt1 = 'double'; dt2 = 'double'; dt3 = 'double'; dt4 = 'double';
        end

        function [cp1,cp2,cp3,cp4] = isOutputComplexImpl(~)
            cp1 = false; cp2 = false; cp3 = false; cp4 = false;
        end

        function [fs1,fs2,fs3,fs4] = isOutputFixedSizeImpl(~)
            fs1 = true; fs2 = true; fs3 = true; fs4 = true;
        end

        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            switch name
                case {'CurrentState','PreviousMode','Integrator_v', ...
                      'V_HV_Sum','V_LV_Sum','I_HV_Sum','I_LV_Sum', ...
                      'V_HV_Filtered','V_LV_Filtered','I_HV_Filtered','I_LV_Filtered', ...
                      'BufferIdx','TickCount'}
                    sz = [1 1]; dt = 'double'; cp = false;
                case {'V_HV_Buffer','V_LV_Buffer','I_HV_Buffer','I_LV_Buffer'}
                    sz = [125 1]; dt = 'double'; cp = false;
                otherwise
                    error(['Unknown state: ', name]);
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts_ems, 'OffsetTime', 0);
        end

        function [n1,n2,n3,n4,n5,n6,n7,n8] = getInputNamesImpl(~)
            n1 = 'V_HV';
            n2 = 'I_HV';        % [NEW]
            n3 = 'V_LV';
            n4 = 'I_LV';        % [NEW]
            n5 = 'I_ess_cmd';
            n6 = 'Kpv';
            n7 = 'Kiv';
            n8 = 'Mode_Select';
        end

        function [name1, name2, name3, name4] = getOutputNamesImpl(~)
            name1 = 'I_cmd';        
            name2 = 'PWM_Enable';   
            name3 = 'Active_Mode';  
            name4 = 'Status_Flag';  
        end
    end

    methods (Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'Smart Energy Manager (v1.1 - Current Feedback Added)', ...
                'Text', sprintf('전류 피드백(I_HV, I_LV)이 추가된 상위 EMS입니다.'));
        end

        function groups = getPropertyGroupsImpl
            groupSys = matlab.system.display.Section(...
                'Title', '1. 시스템 기본 설정', ...
                'PropertyList', {'Ts_ems', 'InitDelay'});

            groupMode = matlab.system.display.Section(...
                'Title', '2. 운전 모드 목표값', ...
                'PropertyList', {'V_HV_Ref', 'V_HV_Band', 'I_ESS_Max', 'I_UPS_Max'});

            groupBMS = matlab.system.display.Section(...
                'Title', '3. 배터리 보호 (BMS)', ...
                'PropertyList', {'V_Pack_Max', 'V_CV_Start', 'K_CV', 'V_ESS_Limit', 'V_UPS_Limit', 'V_Min_Limit', 'I_Charge_Limit', 'I_Discharge_Limit'});

            groupTOU = matlab.system.display.Section(...
                'Title', '4. ESS TOU 스케줄링', ...
                'PropertyList', {'ESS_Charge_Rate', 'ESS_Discharge_Rate', ...
                                 'TOU_OffPeak_Start', 'TOU_OffPeak_End', ...
                                 'TOU_OnPeak1_Start', 'TOU_OnPeak1_End', ...
                                 'TOU_OnPeak2_Start', 'TOU_OnPeak2_End', ...
                                 'SOC_Max_Threshold', 'SOC_Min_Threshold'});

            groups = [groupSys, groupMode, groupBMS, groupTOU];
        end
    end
end
