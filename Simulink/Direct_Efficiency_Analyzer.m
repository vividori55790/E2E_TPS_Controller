classdef Direct_Efficiency_Analyzer < matlab.System
    % Direct_Efficiency_Analyzer (Optimized for CodeGen)
    %
    % [Description]
    % Calculates the efficiency of the DAB converter accurately by averaging
    % power over a user-defined number of switching cycles.
    % 
    % [Key Features]
    % - O(1) Running Sum Algorithm for fast execution.
    % - Parameterized Window Size to eliminate beating/ripple.
    % - Code Generation Ready (DiscreteState & Fixed-Size Buffers).
    
    properties(Nontunable)
        % Switching Frequency [Hz]
        SwitchingFreq = 80e3;       
        
        % Simulation Step Size [s]
        SampleTime = 1e-7;          
        
        % Averaging Window (Switching Cycles)
        NumCyclesToAverage = 20;    
        
        % Inductor AC Resistance [Ohm]
        R_L_ac = 0.05;              
        
        % HV Switches Count
        NumSwitchesHV = 4;          
        
        % LV Switches Count
        NumSwitchesLV = 4;          
    end

    % Internal Property to store the calculated window size for runtime use
    properties(Access = private)
        ComputedWindowSize
    end

    % Define states that change during execution
    properties(DiscreteState)
        % Circular Buffers (Data Storage)
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
        IsBufferFull
    end

    methods(Access = protected)
        
        function setupImpl(obj)
            % Initialize Discrete States
            
            % 1. Compute Window Size
            % Formula: Samples = (NumCycles * CyclePeriod) / SampleTime
            raw_size = obj.NumCyclesToAverage * (1 / obj.SwitchingFreq) / obj.SampleTime;
            val = round(raw_size);
            
            % Safety check
            if val < 1
                val = 1;
            end
            
            % Force fixed size for Code Generation
            ws = coder.const(val);
            
            obj.ComputedWindowSize = ws;
            
            N = ws;
            
            % 2. Initialize Buffers with Calculated Size
            obj.Buffer_Pin = zeros(N, 1);
            obj.Buffer_Pout = zeros(N, 1);
            obj.Buffer_Ploss = zeros(N, 1);
            obj.Buffer_PL_Cu = zeros(N, 1);
            obj.Buffer_Psw_HV = zeros(N, 1);
            obj.Buffer_Psw_LV = zeros(N, 1);
            
            % 3. Initialize Running Sums
            obj.Sum_Pin = 0;
            obj.Sum_Pout = 0;
            obj.Sum_Ploss = 0;
            obj.Sum_PL_Cu = 0;
            obj.Sum_Psw_HV = 0;
            obj.Sum_Psw_LV = 0;
            
            % 4. Initialize Indices
            obj.BufferIdx = 1;
            obj.IsBufferFull = 0; % 0=False, 1=True
        end

        function [Eff, P_Loss_Total, P_Copper_L, P_Semi_HV, P_Semi_LV] = stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L, V_sw_HV, I_sw_HV, V_sw_LV, I_sw_LV)
            
            % --- 1. Instantaneous Physics Calculation (순시값 계산) ---
            
            % Inductor Copper Loss (I^2 * R)
            P_L_inst = (I_L ^ 2) * obj.R_L_ac;
            
            % Switching/Conduction Loss Approximation
            P_sw_HV_inst = abs(V_sw_HV * I_sw_HV) * obj.NumSwitchesHV;
            P_sw_LV_inst = abs(V_sw_LV * I_sw_LV) * obj.NumSwitchesLV;
            
            P_Loss_inst = P_L_inst + P_sw_HV_inst + P_sw_LV_inst;
            
            % Power Flow Direction & Input/Output Power
            P_HV_raw = V_HV * I_HV;
            P_LV_raw = V_LV * I_LV;
            
            if P_HV_raw >= 0 
                % Charge Mode (HV -> LV)
                P_In_inst = P_HV_raw;
                P_Out_inst = abs(P_LV_raw);
            else
                % Discharge Mode (LV -> HV)
                P_In_inst = abs(P_LV_raw);
                P_Out_inst = abs(P_HV_raw);
            end
            
            % --- 2. Update Optimized Moving Average (O(1) Running Sum) ---
            
            idx = obj.BufferIdx;
            
            % Retrieve Oldest Values (to be removed from Sum)
            old_Pin     = obj.Buffer_Pin(idx);
            old_Pout    = obj.Buffer_Pout(idx);
            old_Ploss   = obj.Buffer_Ploss(idx);
            old_PL_Cu   = obj.Buffer_PL_Cu(idx);
            old_Psw_HV  = obj.Buffer_Psw_HV(idx);
            old_Psw_LV  = obj.Buffer_Psw_LV(idx);
            
            % Update Running Sums (Sum = Sum - Old + New)
            obj.Sum_Pin     = obj.Sum_Pin     - old_Pin     + P_In_inst;
            obj.Sum_Pout    = obj.Sum_Pout    - old_Pout    + P_Out_inst;
            obj.Sum_Ploss   = obj.Sum_Ploss   - old_Ploss   + P_Loss_inst;
            obj.Sum_PL_Cu   = obj.Sum_PL_Cu   - old_PL_Cu   + P_L_inst;
            obj.Sum_Psw_HV  = obj.Sum_Psw_HV  - old_Psw_HV  + P_sw_HV_inst;
            obj.Sum_Psw_LV  = obj.Sum_Psw_LV  - old_Psw_LV  + P_sw_LV_inst;

            % Update Buffers (Overwrite with New)
            obj.Buffer_Pin(idx)   = P_In_inst;
            obj.Buffer_Pout(idx)  = P_Out_inst;
            obj.Buffer_Ploss(idx) = P_Loss_inst;
            obj.Buffer_PL_Cu(idx) = P_L_inst;
            obj.Buffer_Psw_HV(idx)= P_sw_HV_inst;
            obj.Buffer_Psw_LV(idx)= P_sw_LV_inst;
            
            % Advance Index
            nextIdx = idx + 1;
            if nextIdx > obj.ComputedWindowSize
                nextIdx = 1;
                obj.IsBufferFull = 1; % Mark as full
            end
            obj.BufferIdx = nextIdx;
            
            % --- 3. Compute Averages & Efficiency ---
            
            N = obj.ComputedWindowSize;
            
            P_Loss_Total = obj.Sum_Ploss / N;
            P_Copper_L   = obj.Sum_PL_Cu / N;
            P_Semi_HV    = obj.Sum_Psw_HV / N;
            P_Semi_LV    = obj.Sum_Psw_LV / N;
            
            P_in_avg  = obj.Sum_Pin / N;
            P_out_avg = obj.Sum_Pout / N;
            
            % Efficiency Logic
            if (obj.IsBufferFull == 0) || (abs(P_in_avg) < 1e-3)
                Eff = 0;
            else
                Eff_IO = (P_out_avg / P_in_avg) * 100;
                Eff_Loss = ((P_in_avg - P_Loss_Total) / P_in_avg) * 100;
                
                % Select realistic efficiency
                if Eff_IO > 100.0
                    Eff = Eff_Loss;
                else
                    Eff = Eff_IO;
                end
                
                % Clamp
                if Eff > 100.0, Eff = 99.9; end
                if Eff < 0.0, Eff = 0.0; end
            end
            
        end
        
        % Required for DiscreteState in Code Generation
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(obj, name)
            % Calculates the window size at compile time
            raw_size = obj.NumCyclesToAverage * (1 / obj.SwitchingFreq) / obj.SampleTime;
            ws = round(raw_size);
            if ws < 1, ws = 1; end
            
            switch name
                case 'BufferIdx'
                    sz = [1 1]; dt = 'double'; cp = false;
                case 'IsBufferFull'
                    sz = [1 1]; dt = 'double'; cp = false;
                case {'Sum_Pin', 'Sum_Pout', 'Sum_Ploss', 'Sum_PL_Cu', 'Sum_Psw_HV', 'Sum_Psw_LV'}
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    sz = [ws 1];
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
            % Group 1: Simulation Settings
            groupSim = matlab.system.display.Section(...
                'Title', '1. Simulation Settings (Averaging)', ...
                'PropertyList', {'SwitchingFreq', 'SampleTime', 'NumCyclesToAverage'});
            
            % Group 2: Component Parameters
            groupComp = matlab.system.display.Section(...
                'Title', '2. Component Parameters (Loss Calculation)', ...
                'PropertyList', {'R_L_ac', 'NumSwitchesHV', 'NumSwitchesLV'});
                
            groups = [groupSim, groupComp];
        end
        % --- Clean Parameter Labels ---
        % (Simulink uses the comment above the property as the label if short, or PropertyName if not customized)
        % To force custom labels, we rely on the comments above properties in the 'properties' block.
    end
end
