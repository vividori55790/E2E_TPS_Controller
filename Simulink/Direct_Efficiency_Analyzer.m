classdef Direct_Efficiency_Analyzer < matlab.System
    % Direct_Efficiency_Analyzer (v3.1 - Exact 1-Cycle Moving Average)
    %
    % [Description]
    % Calculates the efficiency of the DAB converter using a precise
    % 1-cycle moving average filter. The window size is computed from
    % the switching frequency and sample time to guarantee exact
    % 1-cycle averaging, eliminating residual ripple.
    %
    % [Key Features]
    % - Exact 1-cycle window: N = round(1 / (fs * Ts))
    % - O(1) circular buffer based moving average.
    % - fs and Ts are configurable Nontunable properties.
    % - Buffer is pre-allocated to MAX_BUFFER_SIZE for Code Generation.
    % - Code Generation Ready.
    %
    % [v3.1 Changes]
    % - fs, Ts as Nontunable properties (user-configurable).
    % - Exact window size calculation in setupImpl.
    % - Explicit getSampleTimeImpl for deterministic sample time.
    
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
    end

    properties(Constant)
        % Maximum buffer size for Code Generation (pre-allocation)
        % Must be >= max possible window size (1/(fs*Ts))
        % For 80kHz / 1e-7s = 125, but allow up to 500 for other configs
        MAX_BUFFER_SIZE = 500;
    end

    % Define states that change during execution
    properties(DiscreteState)
        % Circular Buffers (Data Storage) - Fixed Size MAX_BUFFER_SIZE
        Buffer_Pin
        Buffer_Pout
        Buffer_Ploss
        Buffer_PL_Cu
        Buffer_Psw_HV
        Buffer_Psw_LV
        
        % Running Sums (O(1) Accumulators)
        Sum_Pin
        Sum_Pout
        Sum_Ploss
        Sum_PL_Cu
        Sum_Psw_HV
        Sum_Psw_LV
        
        % State Control
        BufferIdx
        SampleCount     % Counts samples until buffer is full
        WindowSize      % Actual window size (computed in setup)
    end

    methods(Access = protected)
        
        function setupImpl(obj)
            % Initialize Discrete States
            M = obj.MAX_BUFFER_SIZE;
            
            % Compute exact window size: samples per 1 switching cycle
            obj.WindowSize = round(1 / (obj.Fs * obj.Ts));
            
            % Safety clamp
            if obj.WindowSize < 1
                obj.WindowSize = 1;
            end
            if obj.WindowSize > M
                obj.WindowSize = M;
            end
            
            % Initialize Buffers with MAX size (Code Gen requires fixed size)
            obj.Buffer_Pin    = zeros(M, 1);
            obj.Buffer_Pout   = zeros(M, 1);
            obj.Buffer_Ploss  = zeros(M, 1);
            obj.Buffer_PL_Cu  = zeros(M, 1);
            obj.Buffer_Psw_HV = zeros(M, 1);
            obj.Buffer_Psw_LV = zeros(M, 1);
            
            % Initialize Running Sums
            obj.Sum_Pin    = 0;
            obj.Sum_Pout   = 0;
            obj.Sum_Ploss  = 0;
            obj.Sum_PL_Cu  = 0;
            obj.Sum_Psw_HV = 0;
            obj.Sum_Psw_LV = 0;
            
            % Initialize Indices
            obj.BufferIdx   = 1;
            obj.SampleCount = 0;
        end

        function [Eff, P_Loss_Total, P_Copper_L, P_Semi_HV, P_Semi_LV] = stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L, V_sw_HV, I_sw_HV, V_sw_LV, I_sw_LV)
            
            % --- 1. Instantaneous Physics Calculation ---
            
            % Inductor Copper Loss (I^2 * R)
            P_L_inst = (I_L ^ 2) * obj.R_L_ac;
            
            % Switching/Conduction Loss
            P_sw_HV_inst = (V_sw_HV * I_sw_HV) * obj.NumSwitchesHV;
            P_sw_LV_inst = (V_sw_LV * I_sw_LV) * obj.NumSwitchesLV;
            
            P_Loss_inst = P_L_inst + P_sw_HV_inst + P_sw_LV_inst;
            
            % Power Flow Direction & Input/Output Power
            P_HV_raw = V_HV * I_HV;
            P_LV_raw = V_LV * I_LV;
            
            if P_HV_raw >= 0 
                % Charge Mode (HV -> LV)
                P_In_inst  = P_HV_raw;
                P_Out_inst = abs(P_LV_raw);
            else
                % Discharge Mode (LV -> HV)
                P_In_inst  = abs(P_LV_raw);
                P_Out_inst = abs(P_HV_raw);
            end
            
            % --- 2. Update Moving Average (O(1) Circular Buffer) ---
            
            idx = obj.BufferIdx;
            N   = obj.WindowSize;  % Exact 1-cycle window
            
            % Retrieve Oldest Values (to be removed from Sum)
            old_Pin    = obj.Buffer_Pin(idx);
            old_Pout   = obj.Buffer_Pout(idx);
            old_Ploss  = obj.Buffer_Ploss(idx);
            old_PL_Cu  = obj.Buffer_PL_Cu(idx);
            old_Psw_HV = obj.Buffer_Psw_HV(idx);
            old_Psw_LV = obj.Buffer_Psw_LV(idx);
            
            % Update Running Sums (Sum = Sum - Old + New)
            obj.Sum_Pin    = obj.Sum_Pin    - old_Pin    + P_In_inst;
            obj.Sum_Pout   = obj.Sum_Pout   - old_Pout   + P_Out_inst;
            obj.Sum_Ploss  = obj.Sum_Ploss  - old_Ploss  + P_Loss_inst;
            obj.Sum_PL_Cu  = obj.Sum_PL_Cu  - old_PL_Cu  + P_L_inst;
            obj.Sum_Psw_HV = obj.Sum_Psw_HV - old_Psw_HV + P_sw_HV_inst;
            obj.Sum_Psw_LV = obj.Sum_Psw_LV - old_Psw_LV + P_sw_LV_inst;
            
            % Update Buffers (Overwrite oldest slot with new value)
            obj.Buffer_Pin(idx)    = P_In_inst;
            obj.Buffer_Pout(idx)   = P_Out_inst;
            obj.Buffer_Ploss(idx)  = P_Loss_inst;
            obj.Buffer_PL_Cu(idx)  = P_L_inst;
            obj.Buffer_Psw_HV(idx) = P_sw_HV_inst;
            obj.Buffer_Psw_LV(idx) = P_sw_LV_inst;
            
            % Advance Index (Circular within WindowSize, NOT MAX_BUFFER_SIZE)
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > N
                obj.BufferIdx = 1;
            end
            
            % Track sample count for buffer-full detection
            if obj.SampleCount < N
                obj.SampleCount = obj.SampleCount + 1;
            end
            
            % --- 3. Compute Averages & Efficiency ---
            
            P_Loss_Total = obj.Sum_Ploss  / N;
            P_Copper_L   = obj.Sum_PL_Cu  / N;
            P_Semi_HV    = obj.Sum_Psw_HV / N;
            P_Semi_LV    = obj.Sum_Psw_LV / N;
            
            P_in_avg  = obj.Sum_Pin  / N;
            P_out_avg = obj.Sum_Pout / N;
            
            % Efficiency Logic
            isBufferFull = (obj.SampleCount >= N);
            
            if (~isBufferFull) || (abs(P_in_avg) < 1e-3)
                Eff = 0;
            else
                Eff_IO   = (P_out_avg / P_in_avg) * 100;
                Eff_Loss = ((P_in_avg - P_Loss_Total) / P_in_avg) * 100;
                
                % Select realistic efficiency
                if Eff_IO > 100.0
                    Eff = Eff_Loss;
                else
                    Eff = Eff_IO;
                end
                
                % Clamp
                if Eff > 100.0, Eff = 99.9; end
                if Eff < 0.0,   Eff = 0.0;  end
            end
        end
        
        function resetImpl(obj)
            % Reset all states
            M = obj.MAX_BUFFER_SIZE;
            
            obj.Buffer_Pin    = zeros(M, 1);
            obj.Buffer_Pout   = zeros(M, 1);
            obj.Buffer_Ploss  = zeros(M, 1);
            obj.Buffer_PL_Cu  = zeros(M, 1);
            obj.Buffer_Psw_HV = zeros(M, 1);
            obj.Buffer_Psw_LV = zeros(M, 1);
            
            obj.Sum_Pin    = 0;
            obj.Sum_Pout   = 0;
            obj.Sum_Ploss  = 0;
            obj.Sum_PL_Cu  = 0;
            obj.Sum_Psw_HV = 0;
            obj.Sum_Psw_LV = 0;
            
            obj.BufferIdx   = 1;
            obj.SampleCount = 0;
            % WindowSize is preserved (set in setupImpl, Nontunable)
        end
        
        % --- Sample Time Specification (Deterministic) ---
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                                   'SampleTime', obj.Ts, ...
                                   'OffsetTime', 0);
        end
        
        % Required for DiscreteState in Code Generation
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            M = 500;  % Must match MAX_BUFFER_SIZE
            
            switch name
                case 'BufferIdx'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'SampleCount'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'WindowSize'
                    sz = [1 1]; dt = 'double'; cp = false;
                case {'Sum_Pin', 'Sum_Pout', 'Sum_Ploss', 'Sum_PL_Cu', 'Sum_Psw_HV', 'Sum_Psw_LV'}
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    % All buffer states (Buffer_Pin, Buffer_Pout, etc.)
                    sz = [M 1];
                    dt = 'double';
                    cp = false;
            end
        end

        function [out1, out2, out3, out4, out5] = getOutputSizeImpl(~)
            out1=[1 1]; out2=[1 1]; out3=[1 1]; out4=[1 1]; out5=[1 1];
        end
        function [out1, out2, out3, out4, out5] = getOutputDataTypeImpl(~)
            out1='double'; out2='double'; out3='double'; out4='double'; out5='double';
        end
        function [out1, out2, out3, out4, out5] = isOutputComplexImpl(~)
            out1=false; out2=false; out3=false; out4=false; out5=false;
        end
        function [out1, out2, out3, out4, out5] = isOutputFixedSizeImpl(~)
            out1=true; out2=true; out3=true; out4=true; out5=true;
        end
        
        % --- Parameter Grouping & Display Labels ---
        function groups = getPropertyGroupsImpl(~)
            % Group: Component Parameters
            groupComp = matlab.system.display.Section(...
                'Title', 'Component Parameters (Loss Calculation)', ...
                'PropertyList', {'R_L_ac', 'NumSwitchesHV', 'NumSwitchesLV'});
            
            % Group: Timing Parameters
            groupTime = matlab.system.display.Section(...
                'Title', 'Timing Parameters (Moving Average Window)', ...
                'PropertyList', {'Fs', 'Ts'});
                
            groups = [groupComp, groupTime];
        end
        
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Efficiency Analyzer (1-Cycle Moving Average)', ...
                'Text', sprintf(['DAB 컨버터의 효율을 직접 측정하는 블록입니다.\n', ...
                                 '[v3.1] 정확한 1주기 이동평균 필터 모드\n', ...
                                 '윈도우 크기 = round(1 / (Fs * Ts))\n', ...
                                 '기본값: 80kHz, 1e-7s → 125 샘플']));
        end
    end
    
    methods (Static, Access = protected)
        % Static methods if needed in the future
    end
end
