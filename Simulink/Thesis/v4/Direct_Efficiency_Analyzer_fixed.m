classdef Direct_Efficiency_Analyzer_fixed < matlab.System
    % [V4 COPY, Simulink/Thesis/v4/, 2026-10-31] proposed fix of dataset_era/Direct_Efficiency_Analyzer.m; the user's file is not changed. NOT run inside the model:
    % only test_analyzer_fix.m, on constant inputs. In this model the sensors read P_HV > 0 and P_LV > 0 for LV -> HV (and both negative for HV -> LV, f1_eff.py), so the original
    % "P_HV >= 0: HV is the input" has input and output the wrong way round in both directions: Eff_IO comes out above 100 % and Eff falls through to Eff_Loss. Two lines swapped below.
    % Everything else as the original, cut down to what the efficiency needs (the ZVS and loss outputs are unchanged in the original and are not repeated here).
    properties(Nontunable)
        Fs = 80e3;
        Ts = 1e-7;
    end
    properties(Constant)
        MAX_BUFFER_SIZE = 500;
    end
    properties(Access = private)
        Buffer_Pin; Buffer_Pout; Sum_Pin; Sum_Pout; BufferIdx; SampleCount; WindowSize;
    end
    methods(Access = protected)
        function setupImpl(obj)
            M = obj.MAX_BUFFER_SIZE;
            obj.WindowSize = min(max(round(1 / (obj.Fs * obj.Ts)), 1), M);
            obj.Buffer_Pin = zeros(M, 1); obj.Buffer_Pout = zeros(M, 1);
            obj.Sum_Pin = 0; obj.Sum_Pout = 0; obj.BufferIdx = 1; obj.SampleCount = 0;
        end
        function Eff = stepImpl(obj, V_HV, I_HV, V_LV, I_LV)
            P_HV_raw = V_HV * I_HV;
            P_LV_raw = V_LV * I_LV;
            if P_HV_raw >= 0                 % [FIX] P_HV > 0 = LV -> HV: the LV side is the input (the original had these two lines the other way round)
                P_In_inst  = abs(P_LV_raw);
                P_Out_inst = abs(P_HV_raw);
            else                             % [FIX] P_HV < 0 = HV -> LV: the HV side is the input
                P_In_inst  = abs(P_HV_raw);
                P_Out_inst = abs(P_LV_raw);
            end
            idx = obj.BufferIdx;
            N = obj.WindowSize;
            obj.Sum_Pin  = obj.Sum_Pin  - obj.Buffer_Pin(idx)  + P_In_inst;
            obj.Sum_Pout = obj.Sum_Pout - obj.Buffer_Pout(idx) + P_Out_inst;
            obj.Buffer_Pin(idx) = P_In_inst;
            obj.Buffer_Pout(idx) = P_Out_inst;
            obj.BufferIdx = obj.BufferIdx + 1;
            if obj.BufferIdx > N, obj.BufferIdx = 1; end
            if obj.SampleCount < N, obj.SampleCount = obj.SampleCount + 1; end
            if obj.SampleCount < N || abs(obj.Sum_Pin / N) < 1e-3
                Eff = 0;
            else
                Eff = min(max(100 * obj.Sum_Pout / obj.Sum_Pin, 0), 99.9);
            end
        end
    end
end
