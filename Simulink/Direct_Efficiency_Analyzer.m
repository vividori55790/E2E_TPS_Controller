classdef Direct_Efficiency_Analyzer < matlab.System
    % Direct_Efficiency_Analyzer (v2.1 - Instantaneous Power Mode)
    %
    % [Description]
    % Calculates the efficiency of the DAB converter using INSTANTANEOUS power.
    % Moving average filter is DISABLED (commented out) for debugging purposes.
    % 
    % [Key Features]
    % - Instantaneous power output (no filtering).
    % - Moving average code preserved as comments for future re-activation.
    % - Code Generation Ready.
    %
    % [v2.1 Changes]
    % - Disabled moving average filter (commented out).
    % - Direct instantaneous power output for debugging.
    
    properties(Nontunable)
        % Inductor AC Resistance [Ohm]
        R_L_ac = 0.05;              
        
        % HV Switches Count
        NumSwitchesHV = 4;          
        
        % LV Switches Count
        NumSwitchesLV = 4;          
    end

    % =========================================================================
    % [DISABLED] Moving Average Constants & Buffers
    % Uncomment to re-enable moving average filtering
    % =========================================================================
    % properties(Constant)
    %     % Fixed Window Size (125 samples = 1 cycle @ 80kHz / 1e-7s sample time)
    %     WINDOW_SIZE = 125;
    % end

    % Define states that change during execution
    properties(DiscreteState)
        % =====================================================================
        % [DISABLED] Moving Average Buffers - Commented out
        % =====================================================================
        % % Circular Buffers (Data Storage) - Fixed Size 125
        % Buffer_Pin
        % Buffer_Pout
        % Buffer_Ploss
        % Buffer_PL_Cu
        % Buffer_Psw_HV
        % Buffer_Psw_LV
        % 
        % % Running Sums (O(1) Accumulators)
        % Sum_Pin
        % Sum_Pout
        % Sum_Ploss
        % Sum_PL_Cu
        % Sum_Psw_HV
        % Sum_Psw_LV
        % 
        % % State Control (Shared Index)
        % BufferIdx
        % SampleCount     % Counts samples until buffer is full
        
        % [ACTIVE] Dummy state for Code Generation (at least one state required)
        DummyState
    end

    methods(Access = protected)
        
        function setupImpl(obj)
            % Initialize Discrete States
            
            % =====================================================================
            % [DISABLED] Moving Average Initialization
            % =====================================================================
            % N = obj.WINDOW_SIZE; % 125
            % 
            % % Initialize Buffers with Fixed Size
            % obj.Buffer_Pin = zeros(N, 1);
            % obj.Buffer_Pout = zeros(N, 1);
            % obj.Buffer_Ploss = zeros(N, 1);
            % obj.Buffer_PL_Cu = zeros(N, 1);
            % obj.Buffer_Psw_HV = zeros(N, 1);
            % obj.Buffer_Psw_LV = zeros(N, 1);
            % 
            % % Initialize Running Sums
            % obj.Sum_Pin = 0;
            % obj.Sum_Pout = 0;
            % obj.Sum_Ploss = 0;
            % obj.Sum_PL_Cu = 0;
            % obj.Sum_Psw_HV = 0;
            % obj.Sum_Psw_LV = 0;
            % 
            % % Initialize Indices
            % obj.BufferIdx = 1;
            % obj.SampleCount = 0;
            
            % [ACTIVE] Dummy state initialization
            obj.DummyState = 0;
        end

        function [Eff, P_Loss_Total, P_Copper_L, P_Semi_HV, P_Semi_LV] = stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_L, V_sw_HV, I_sw_HV, V_sw_LV, I_sw_LV)
            
            % --- 1. Instantaneous Physics Calculation (순시값 계산) ---
            
            % Inductor Copper Loss (I^2 * R)
            P_L_inst = (I_L ^ 2) * obj.R_L_ac;
            
            % Switching/Conduction Loss Calculation
            % Note: Using V*I primarily captures Conduction Loss.
            % Switching transients (overlap) may be missed if SampleTime > 10-20ns.
            % 'abs' is NOT used to correctly handle reactive power (C_oss charge/discharge) in ZVS.
            P_sw_HV_inst = (V_sw_HV * I_sw_HV) * obj.NumSwitchesHV;
            P_sw_LV_inst = (V_sw_LV * I_sw_LV) * obj.NumSwitchesLV;
            
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
            
            % =====================================================================
            % [DISABLED] Moving Average Update - Commented out
            % =====================================================================
            % % --- 2. Update Moving Average (Controller-Style O(1) Running Sum) ---
            % 
            % idx = obj.BufferIdx;
            % N = obj.WINDOW_SIZE;
            % 
            % % Retrieve Oldest Values (to be removed from Sum)
            % old_Pin     = obj.Buffer_Pin(idx);
            % old_Pout    = obj.Buffer_Pout(idx);
            % old_Ploss   = obj.Buffer_Ploss(idx);
            % old_PL_Cu   = obj.Buffer_PL_Cu(idx);
            % old_Psw_HV  = obj.Buffer_Psw_HV(idx);
            % old_Psw_LV  = obj.Buffer_Psw_LV(idx);
            % 
            % % Update Running Sums (Sum = Sum - Old + New)
            % obj.Sum_Pin     = obj.Sum_Pin     - old_Pin     + P_In_inst;
            % obj.Sum_Pout    = obj.Sum_Pout    - old_Pout    + P_Out_inst;
            % obj.Sum_Ploss   = obj.Sum_Ploss   - old_Ploss   + P_Loss_inst;
            % obj.Sum_PL_Cu   = obj.Sum_PL_Cu   - old_PL_Cu   + P_L_inst;
            % obj.Sum_Psw_HV  = obj.Sum_Psw_HV  - old_Psw_HV  + P_sw_HV_inst;
            % obj.Sum_Psw_LV  = obj.Sum_Psw_LV  - old_Psw_LV  + P_sw_LV_inst;
            % 
            % % Update Buffers (Overwrite with New)
            % obj.Buffer_Pin(idx)   = P_In_inst;
            % obj.Buffer_Pout(idx)  = P_Out_inst;
            % obj.Buffer_Ploss(idx) = P_Loss_inst;
            % obj.Buffer_PL_Cu(idx) = P_L_inst;
            % obj.Buffer_Psw_HV(idx)= P_sw_HV_inst;
            % obj.Buffer_Psw_LV(idx)= P_sw_LV_inst;
            % 
            % % Advance Index (Circular - Same as DAB_Controller_System)
            % obj.BufferIdx = obj.BufferIdx + 1;
            % if obj.BufferIdx > N
            %     obj.BufferIdx = 1;
            % end
            % 
            % % Track sample count for buffer-full detection
            % if obj.SampleCount < N
            %     obj.SampleCount = obj.SampleCount + 1;
            % end
            % 
            % % --- 3. Compute Averages & Efficiency ---
            % 
            % P_Loss_Total = obj.Sum_Ploss / N;
            % P_Copper_L   = obj.Sum_PL_Cu / N;
            % P_Semi_HV    = obj.Sum_Psw_HV / N;
            % P_Semi_LV    = obj.Sum_Psw_LV / N;
            % 
            % P_in_avg  = obj.Sum_Pin / N;
            % P_out_avg = obj.Sum_Pout / N;
            % 
            % % Efficiency Logic
            % isBufferFull = (obj.SampleCount >= N);
            % 
            % if (~isBufferFull) || (abs(P_in_avg) < 1e-3)
            %     Eff = 0;
            % else
            %     Eff_IO = (P_out_avg / P_in_avg) * 100;
            %     Eff_Loss = ((P_in_avg - P_Loss_Total) / P_in_avg) * 100;
            %     
            %     % Select realistic efficiency
            %     if Eff_IO > 100.0
            %         Eff = Eff_Loss;
            %     else
            %         Eff = Eff_IO;
            %     end
            %     
            %     % Clamp
            %     if Eff > 100.0, Eff = 99.9; end
            %     if Eff < 0.0, Eff = 0.0; end
            % end
            
            % =====================================================================
            % [ACTIVE] Direct Instantaneous Output (No Filtering)
            % =====================================================================
            
            % Direct output of instantaneous values
            P_Loss_Total = P_Loss_inst;
            P_Copper_L   = P_L_inst;
            P_Semi_HV    = P_sw_HV_inst;
            P_Semi_LV    = P_sw_LV_inst;
            
            % Instantaneous Efficiency Calculation
            if abs(P_In_inst) < 1e-3
                Eff = 0;
            else
                Eff_IO = (P_Out_inst / P_In_inst) * 100;
                Eff_Loss = ((P_In_inst - P_Loss_inst) / P_In_inst) * 100;
                
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
            
            % Keep dummy state alive (required for Code Gen)
            obj.DummyState = obj.DummyState;
        end
        
        function resetImpl(obj)
            % Reset all states
            
            % =====================================================================
            % [DISABLED] Moving Average Reset
            % =====================================================================
            % N = obj.WINDOW_SIZE;
            % 
            % obj.Buffer_Pin = zeros(N, 1);
            % obj.Buffer_Pout = zeros(N, 1);
            % obj.Buffer_Ploss = zeros(N, 1);
            % obj.Buffer_PL_Cu = zeros(N, 1);
            % obj.Buffer_Psw_HV = zeros(N, 1);
            % obj.Buffer_Psw_LV = zeros(N, 1);
            % 
            % obj.Sum_Pin = 0;
            % obj.Sum_Pout = 0;
            % obj.Sum_Ploss = 0;
            % obj.Sum_PL_Cu = 0;
            % obj.Sum_Psw_HV = 0;
            % obj.Sum_Psw_LV = 0;
            % 
            % obj.BufferIdx = 1;
            % obj.SampleCount = 0;
            
            obj.DummyState = 0;
        end
        
        % Required for DiscreteState in Code Generation
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, name)
            % =====================================================================
            % [DISABLED] Moving Average State Specs
            % =====================================================================
            % % Fixed window size = 125
            % N = 125;
            % 
            % switch name
            %     case 'BufferIdx'
            %         sz = [1 1]; dt = 'double'; cp = false;
            %     case 'SampleCount'
            %         sz = [1 1]; dt = 'double'; cp = false;
            %     case {'Sum_Pin', 'Sum_Pout', 'Sum_Ploss', 'Sum_PL_Cu', 'Sum_Psw_HV', 'Sum_Psw_LV'}
            %         sz = [1 1]; dt = 'double'; cp = false;
            %     otherwise
            %         % All buffer states
            %         sz = [N 1];
            %         dt = 'double';
            %         cp = false;
            % end
            
            % [ACTIVE] Only DummyState
            switch name
                case 'DummyState'
                    sz = [1 1]; dt = 'double'; cp = false;
                otherwise
                    sz = [1 1]; dt = 'double'; cp = false;
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
                
            groups = groupComp;
        end
        
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename('class'), ...
                'Title', 'DAB Efficiency Analyzer (Instantaneous Mode)', ...
                'Text', sprintf(['DAB 컨버터의 효율을 직접 측정하는 블록입니다.\n', ...
                                 '[v2.1] 이동평균 필터 비활성화 - 순시 전력 출력 모드\n', ...
                                 '참고: 필터링된 값이 필요하면 코드 내 주석을 해제하세요.']));
        end
    end
    
    methods (Static, Access = protected)
        % Static methods if needed in the future
    end
end
