classdef Direct_Efficiency_Analyzer < matlab.System
    % Direct_Efficiency_Analyzer (v3.4 - Exact 1-Cycle Moving Average & True ZVS)
    %
    % [Description]
    % Calculates the efficiency of the DAB converter using a precise
    % 1-cycle moving average filter. 
    %
    % [v3.4 Changes for 380V/96V 1kW DAB]
    % - HV측(380V)과 LV측(96V)의 전압/전류 스케일이 다르므로 ZVS 임계값을 분리함
    % - 기본값을 380V, 96V, 1kW, 80kHz 사양에 맞게 최적화
    
    properties(Nontunable)
        % Inductor AC Resistance [Ohm]
        R_L_ac = 0.05;              
        
        % HV Switches Count
        NumSwitchesHV = 4;          
        
        % LV Switches Count
        NumSwitchesLV = 4;
        
        % Switching Frequency [Hz]
        Fs = 80e3;
        
        % Sample Time [sec] (must match Simulink block sample time)
        Ts = 1e-7;
        
        % [HV Side (380V) ZVS Thresholds]
        % 380V의 약 10% 미만일 때 켜져야 ZVS로 인정
        ZVS_V_Threshold_HV = 30;
        % HV측 정격 평균 전류(약 2.6A) 대비 노이즈 마진
        ZVS_I_Threshold_HV = 0.5;
        
        % [LV Side (96V) ZVS Thresholds]
        % 96V의 약 10% 미만일 때 켜져야 ZVS로 인정
        ZVS_V_Threshold_LV = 10;
        % LV측 정격 평균 전류(약 10.4A) 대비 큰 리플/노이즈 마진
        ZVS_I_Threshold_LV = 2.0;
    end

    properties(Constant)
        MAX_BUFFER_SIZE = 500;
    end

    properties(DiscreteState)
        % Power Buffers
        Buffer_Pin
        Buffer_Pout
        Buffer_Ploss
        Buffer_PL_Cu
        Buffer_Psw_HV
        Buffer_Psw_LV
        Buffer_P_HV
        Buffer_P_LV
        
        % Hard Switching Flag Buffers (1 means Hard Switching occurred)
        Buffer_HardSw_HV
        Buffer_HardSw_LV
        
        % Running Sums
        Sum_Pin
        Sum_Pout
        Sum_Ploss
        Sum_PL_Cu
        Sum_Psw_HV
        Sum_Psw_LV
        Sum_P_HV
        Sum_P_LV
        
        % Hard Switching Running Sums
        Sum_HardSw_HV
        Sum_HardSw_LV
        
        % State Control
        BufferIdx
        SampleCount
        WindowSize
    end

    methods(Access = protected)
        
        function setupImpl(obj)
            M = obj.MAX_BUFFER_SIZE;
            obj.WindowSize = round(1 / (obj.Fs * obj.Ts));
            
            if obj.WindowSize < 1, obj.WindowSize = 1; end
            if obj.WindowSize > M, obj.WindowSize = M; end
            
            obj.Buffer_Pin    = zeros(M, 1);
            obj.Buffer_Pout   = zeros(M, 1);
            obj.Buffer_Ploss  = zeros(M, 1);
            obj.Buffer_PL_Cu  = zeros(M, 1);
            obj.Buffer_Psw_HV = zeros(M, 1);
            obj.Buffer_Psw_LV = zeros(M, 1);
            obj.Buffer_P_HV   = zeros(M, 1);
            obj.Buffer_P_LV   = zeros(M, 1);
            
            obj.Buffer_HardSw_HV = zeros(M, 1);
            obj.Buffer_HardSw_LV = zeros(M, 1);
            
            obj.Sum_Pin    = 0;
            obj.Sum_Pout   = 0;
            obj.Sum_Ploss  = 0;
            obj.Sum_PL_Cu  = 0;
            obj.Sum_Psw_HV = 0;
            obj.Sum_Psw_LV = 0;
            obj.Sum_P_HV   = 0;
            obj.Sum_P_LV   = 0;
            
            obj.Sum_HardSw_HV = 0;
            obj.Sum_HardSw_LV = 0;
            
            obj.BufferIdx   = 1;
            obj.SampleCount = 0;
        end

        function [Eff, P_Loss_Total, P_Copper_L, P_Semi_HV, P_Semi_LV, P_HV, P_LV, Is_ZVS] = stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L, V_sw_HV, I_sw_HV, V_sw_LV, I_sw_LV)
            
            % --- 1. Instantaneous Physics Calculation ---
            P_L_inst = (I_L ^ 2) * obj.R_L_ac;
            P_sw_HV_inst = (V_sw_HV * I_sw_HV) * obj.NumSwitchesHV;
            P_sw_LV_inst = (V_sw_LV * I_sw_LV) * obj.NumSwitchesLV;
            P_Loss_inst = P_L_inst + P_sw_HV_inst + P_sw_LV_inst;
            
            P_HV_raw = V_HV * I_HV;
            P_LV_raw = V_LV * I_LV;
            
            if P_HV_raw >= 0 
                P_In_inst  = P_HV_raw;
                P_Out_inst = abs(P_LV_raw);
            else
                P_In_inst  = abs(P_LV_raw);
                P_Out_inst = abs(P_HV_raw);
            end
            
            % --- True ZVS Overlap Detection (HV/LV Separated) ---
            % 전압이 V_Threshold 이상인데 전류도 I_Threshold 이상 흐르면 하드스위칭(1)
            is_Hard_HV = (abs(V_sw_HV) > obj.ZVS_V_Threshold_HV) && (abs(I_sw_HV) > obj.ZVS_I_Threshold_HV);
            is_Hard_LV = (abs(V_sw_LV) > obj.ZVS_V_Threshold_LV) && (abs(I_sw_LV) > obj.ZVS_I_Threshold_LV);
            
            % --- 2. Update Moving Average ---
            idx = obj.BufferIdx;
            N   = obj.WindowSize;
            
            % Remove Oldest
            obj.Sum_Pin    = obj.Sum_Pin    - obj.Buffer_Pin(idx)    + P_In_inst;
            obj.Sum_Pout   = obj.Sum_Pout   - obj.Buffer_Pout(idx)   + P_Out_inst;
            obj.Sum_Ploss  = obj.Sum_Ploss  - obj.Buffer_Ploss(idx)  + P_Loss_inst;
            obj.Sum_PL_Cu  = obj.Sum_PL_Cu  - obj.Buffer_PL_Cu(idx)  + P_L_inst;
            obj.Sum_Psw_HV = obj.Sum_Psw_HV - obj.Buffer_Psw_HV(idx) + P_sw_HV_inst;
            obj.Sum_Psw_LV = obj.Sum_Psw_LV - obj.Buffer_Psw_LV(idx) + P_sw_LV_inst;
            obj.Sum_P_HV   = obj.Sum_P_HV   - obj.Buffer_P_HV(idx)   + P_HV_raw;
            obj.Sum_P_LV   = obj.Sum_P_LV   - obj.Buffer_P_LV(idx)   + P_LV_raw;
            
            obj.Sum_HardSw_HV = obj.Sum_HardSw_HV - obj.Buffer_HardSw_HV(idx) + is_Hard_HV;
            obj.Sum_HardSw_LV = obj.Sum_HardSw_LV - obj.Buffer_HardSw_LV(idx) + is_Hard_LV;
            
            % Update Buffers
            obj.Buffer_Pin(idx)    = P_In_inst;
            obj.Buffer_Pout(idx)   = P_Out_inst;
            obj.Buffer_Ploss(idx)  = P_Loss_inst;
            obj.Buffer_PL_Cu(idx)  = P_L_inst;
            obj.Buffer_Psw_HV(idx) = P_sw_HV_inst;
            obj.Buffer_Psw_LV(idx) = P_sw_LV_inst;
            obj.Buffer_P_HV(idx)   = P_HV_raw;
            obj.Buffer_P_LV(idx)   = P_LV_raw;
            
            obj.Buffer_HardSw_HV(idx) = is_Hard_HV;
            obj.Buffer_HardSw_LV(idx) = is_Hard_LV;
            
            % Advance Index
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > N, obj.BufferIdx = 1; end
            if obj.SampleCount < N, obj.SampleCount = obj.SampleCount + 1; end
            
            % --- 3. Compute Averages & Efficiency ---
            P_Loss_Total = obj.Sum_Ploss / N;
            P_Copper_L   = obj.Sum_PL_Cu / N;
            P_Semi_HV    = obj.Sum_Psw_HV / N;
            P_Semi_LV    = obj.Sum_Psw_LV / N;
            P_HV         = obj.Sum_P_HV / N;
            P_LV         = obj.Sum_P_LV / N;
            
            P_in_avg  = obj.Sum_Pin / N;
            P_out_avg = obj.Sum_Pout / N;
            
            isBufferFull = (obj.SampleCount >= N);
            
            if (~isBufferFull) || (abs(P_in_avg) < 1e-3)
                Eff = 0;
            else
                Eff_IO   = (P_out_avg / P_in_avg) * 100;
                Eff_Loss = ((P_in_avg - P_Loss_Total) / P_in_avg) * 100;
                if Eff_IO > 100.0, Eff = Eff_Loss; else, Eff = Eff_IO; end
                if Eff > 100.0, Eff = 99.9; end
                if Eff < 0.0,   Eff = 0.0;  end
            end
            
            % --- 4. ZVS Decision ---
            % 1주기 내에 HV, LV 양쪽 모두 하드스위칭이 없어야 ZVS 성공
            if (~isBufferFull)
                Is_ZVS = 0;
            else
                if (obj.Sum_HardSw_HV == 0) && (obj.Sum_HardSw_LV == 0)
                    Is_ZVS = 1; % ZVS 달성
                else
                    Is_ZVS = 0; % 하드 스위칭 발생
                end
            end
        end
        
        function resetImpl(obj)
            M = obj.MAX_BUFFER_SIZE;
            obj.Buffer_Pin    = zeros(M, 1);
            obj.Buffer_Pout   = zeros(M, 1);
            obj.Buffer_Ploss  = zeros(M, 1);
            obj.Buffer_PL_Cu  = zeros(M, 1);
            obj.Buffer_Psw_HV = zeros(M, 1);
            obj.Buffer_Psw_LV = zeros(M, 1);
            obj.Buffer_P_HV   = zeros(M, 1);
            obj.Buffer_P_LV   = zeros(M, 1);
            obj.Buffer_HardSw_HV = zeros(M, 1);
            obj.Buffer_HardSw_LV = zeros(M, 1);
            
            obj.Sum_Pin = 0; obj.Sum_Pout = 0; obj.Sum_Ploss = 0; obj.Sum_PL_Cu = 0;
            obj.Sum_Psw_HV = 0; obj.Sum_Psw_LV = 0; obj.Sum_P_HV = 0; obj.Sum_P_LV = 0;
            obj.Sum_HardSw_HV = 0; obj.Sum_HardSw_LV = 0;
            
            obj.BufferIdx = 1;
            obj.SampleCount = 0;
        end
        
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts, 'OffsetTime', 0);
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            M = 500;
            switch name
                case {'BufferIdx', 'SampleCount', 'WindowSize', ...
                      'Sum_Pin', 'Sum_Pout', 'Sum_Ploss', 'Sum_PL_Cu', ...
                      'Sum_Psw_HV', 'Sum_Psw_LV', 'Sum_P_HV', 'Sum_P_LV', ...
                      'Sum_HardSw_HV', 'Sum_HardSw_LV'}
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    sz = [M 1]; dt = 'double'; cp = false;
            end
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputSizeImpl(~)
            out1=[1 1]; out2=[1 1]; out3=[1 1]; out4=[1 1]; out5=[1 1]; out6=[1 1]; out7=[1 1]; out8=[1 1];
        end
        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputDataTypeImpl(~)
            out1='double'; out2='double'; out3='double'; out4='double'; out5='double'; out6='double'; out7='double'; out8='double';
        end
        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputComplexImpl(~)
            out1=false; out2=false; out3=false; out4=false; out5=false; out6=false; out7=false; out8=false;
        end
        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputFixedSizeImpl(~)
            out1=true; out2=true; out3=true; out4=true; out5=true; out6=true; out7=true; out8=true;
        end
    end
    
    methods (Static, Access = protected)
        function groups = getPropertyGroupsImpl()
            groupComp = matlab.system.display.Section('Title', 'System & Thresholds', ...
                'PropertyList', {'R_L_ac', 'NumSwitchesHV', 'NumSwitchesLV', ...
                                 'ZVS_V_Threshold_HV', 'ZVS_I_Threshold_HV', ...
                                 'ZVS_V_Threshold_LV', 'ZVS_I_Threshold_LV'});
            groupTime = matlab.system.display.Section('Title', 'Timing Parameters', ...
                'PropertyList', {'Fs', 'Ts'});
            groups = [groupComp, groupTime];
        end
        
        function header = getHeaderImpl()
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Efficiency Analyzer (v3.4 - HV/LV Split ZVS)', ...
                'Text', sprintf(['DAB 컨버터 효율 및 ZVS 여부 측정 블록\\n', ...
                                 '[v3.4] 380V/96V 환경에 맞춰 HV/LV 임계값 독립 분리.']));
        end
    end
end