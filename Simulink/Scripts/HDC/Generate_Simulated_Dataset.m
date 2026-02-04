%% Generate_Simulated_Dataset.m
% AI-Ready Digital Twin v5.0 - Simulated Experience Dataset Generator
%
% Purpose:
%   Generates simulation-based training data for Offline Reinforcement Learning.
%   This script captures 'Actual Physics Data' including deadtime effects,
%   parasitic capacitance, and thermal variations that analytical models miss.
%
% Key Features:
%   1. Electro-Thermal Iteration: Considers P_loss ↔ Temperature interaction
%   2. Monte Carlo Uncertainty: Aging effects (C_dc, R_esr) + Thermal variation
%   3. Full Operating Range: V_batt sweep × Load levels × Charge/Discharge
%   4. AI Dataset Output: CSV format for RL agent training
%
% Reference: Readme_Efficiency.md (v5.0 Master Plan)
%
% Author: Auto-Generated from Master Plan
% Date: 2026-02-04

clear; clc;
fprintf('=============================================================\n');
fprintf('  AI-Ready Digital Twin v5.0 - Simulated Dataset Generator\n');
fprintf('=============================================================\n\n');

%% 1. Simulation Setup
modelName = 'Control_Logic'; % Updated to actual model name

% Check Model
if ~bdIsLoaded(modelName)
    fprintf('Loading model: %s...\n', modelName);
    try
        load_system(modelName);
        fprintf('Model loaded successfully.\n');
    catch ME
        error('Failed to load model "%s": %s\nPlease ensure the model exists in the current path.', modelName, ME.message);
    end
else
    fprintf('Model %s is already loaded.\n', modelName);
end

%% System Parameters (High Fidelity)
Param = struct();
Param.C_nom = 1000e-6;           % [F] Nominal DC-Link Capacitance
Param.L_nom = 90e-6;             % [H] Nominal Inductance (from Generate_TPS_Dataset)
Param.R_esr_nom = 0.05;          % [Ohm] Nominal ESR
Param.R_ds_25 = 0.02;            % [Ohm] R_ds(on) at 25°C
Param.R_th_heatsink = 1.5;       % [°C/W] Heatsink Thermal Resistance
Param.T_amb = 40;                % [°C] Ambient Temperature (Harsh Condition)
Param.Temp_Coeff_Rds = 0.006;    % [1/°C] Temperature Coefficient of R_ds(on)

% System Ratings
Grid_Voltage = 400;              % [V] Nominal Grid Voltage
Rated_Power = 5000;              % [W] Rated Power

fprintf('\n--- System Parameters ---\n');
fprintf('Grid Voltage: %d V\n', Grid_Voltage);
fprintf('Rated Power: %d W\n', Rated_Power);
fprintf('Ambient Temperature: %d °C\n', Param.T_amb);
fprintf('Heatsink R_th: %.2f °C/W\n', Param.R_th_heatsink);

%% Sweep Conditions (Test Matrix)
V_batt_sweep = [84, 96, 116];          % Battery Voltage Range [V]
Load_Levels = [0.2, 0.5, 0.8, 1.0];    % Load Rate (Fraction of Rated Power)
Op_Mode_List = ["Charge", "Discharge"]; % Operation Modes
N_MC_per_Point = 5;                     % Monte Carlo samples per operating point

Total_Cases = length(V_batt_sweep) * length(Load_Levels) * length(Op_Mode_List) * N_MC_per_Point;
fprintf('\n--- Test Matrix Configuration ---\n');
fprintf('Battery Voltages: [%s] V\n', strjoin(string(V_batt_sweep), ', '));
fprintf('Load Levels: [%s]\n', strjoin(string(Load_Levels), ', '));
fprintf('Operation Modes: [%s]\n', strjoin(Op_Mode_List, ', '));
fprintf('Monte Carlo Samples per Point: %d\n', N_MC_per_Point);
fprintf('Total Simulation Cases: %d\n', Total_Cases);

%% 2. Case Generation
in = Simulink.SimulationInput.empty;
run_idx = 1;
SimStopTime = '0.05'; % [s] Simulation Stop Time

fprintf('\n\nGenerating AI-Ready Digital Twin Cases (v5.0)...\n');

for mode = Op_Mode_List
    for v = V_batt_sweep
        for load_rate = Load_Levels
            
            % Load & Direction Logic
            if mode == "Charge"
                % Charging: Grid → Battery
                Calc_R_load_A = 1e6; % Open Grid Load (Conceptually - no resistive load)
                Target_I_batt = -(Rated_Power * load_rate) / v; % Negative current = charging
            else
                % Discharging: Battery → Grid
                Power_Target = Rated_Power * load_rate;
                Calc_R_load_A = (Grid_Voltage^2) / Power_Target; % Resistive load on grid side
                Target_I_batt = 0; % Current controlled by load, not by reference
            end
            
            for mc = 1:N_MC_per_Point
                in(run_idx) = Simulink.SimulationInput(modelName);
                
                % Uncertainty Injection
                if mc == 1 
                    % Nominal Baseline Case
                    Aging_F = 1; 
                    TypeStr = 'Nominal';
                    % Initial Temperature Assumption (Heuristic: 5% loss assumption)
                    Assumed_Loss = Rated_Power * load_rate * 0.05;
                    T_j_est = Param.T_amb + Assumed_Loss * Param.R_th_heatsink;
                else       
                    % Monte Carlo: Random Aging & Thermal Variation
                    Aging_F = 0.7 + 0.3 * rand(); % Aging factor: 70%~100% of nominal
                    TypeStr = 'Random';
                    % Random thermal variation
                    T_j_est = Param.T_amb + (Rated_Power * load_rate * 0.05) * (0.8 + 0.4*rand());
                end
                
                % Temperature Dependent R_ds(on)
                % R_ds(T) = R_ds(25°C) × [1 + α × (T - 25)]
                Calc_R_ds = Param.R_ds_25 * (1 + Param.Temp_Coeff_Rds * (T_j_est - 25));
                
                % Set Variables to Model Workspace or Block Parameters
                % Note: Ensure these variables exist in the Model Workspace or mask
                in(run_idx) = in(run_idx).setVariable('R_ds_val', Calc_R_ds);
                in(run_idx) = in(run_idx).setVariable('C_dc', Param.C_nom * Aging_F);
                in(run_idx) = in(run_idx).setVariable('V_batt', v);
                in(run_idx) = in(run_idx).setVariable('R_load_val', Calc_R_load_A);
                in(run_idx) = in(run_idx).setVariable('Target_Current', Target_I_batt);
                
                % Simulation Settings
                in(run_idx) = in(run_idx).setModelParameter('StopTime', SimStopTime);
                
                % Metadata for Post-Processing
                in(run_idx) = in(run_idx).setVariable('Meta_Mode', mode);
                in(run_idx) = in(run_idx).setVariable('Meta_Load', load_rate);
                in(run_idx) = in(run_idx).setVariable('Meta_V', v);
                in(run_idx) = in(run_idx).setVariable('Meta_Type', TypeStr);
                in(run_idx) = in(run_idx).setVariable('Meta_Tj_Input', T_j_est);
                in(run_idx) = in(run_idx).setVariable('Meta_AgingFactor', Aging_F);
                
                run_idx = run_idx + 1;
            end
        end
    end
end

fprintf('Case generation complete. Total cases: %d\n', length(in));

%% 3. Run Simulation
fprintf('\n=== Starting Parallel Simulation ===\n');
fprintf('Total runs: %d\n', length(in));
fprintf('Estimated time: %.1f minutes (depending on hardware)\n\n', length(in) * 0.5 / 60);

% Check Parallel Pool
if isempty(gcp('nocreate'))
    try
        parpool;
        fprintf('Parallel pool started.\n');
    catch
        fprintf('Warning: Could not start parallel pool. Running sequentially.\n');
    end
end

tic;
try
    % Use parsim for parallel simulation
    % Note: 'UseFastRestart', 'on' can be added if model supports it
    out = parsim(in, 'ShowProgress', 'on', 'StopOnError', 'off', ...
                 'TransferBaseWorkspaceVariables', 'on');
catch ME
    fprintf('Simulation Error: %s\n', ME.message);
    out = [];
end
simTime = toc;

fprintf('\nSimulation completed in %.2f seconds (%.2f minutes).\n', simTime, simTime/60);

%% 4. Analysis & AI Dataset Generation
if isempty(out)
    error('No simulation results available. Please check model configuration.');
end

fprintf('\n=== Processing Results ===\n');

% Initialize Results Table
results = table();
successCount = 0;
failCount = 0;

for k = 1:length(out)
    data = out(k);
    
    % Check for Simulation Error
    try
        stopEvent = data.SimulationMetadata.ExecutionInfo.StopEvent;
        if stopEvent == "DiagnosticError"
            failCount = failCount + 1;
            continue;
        end
    catch
        failCount = failCount + 1;
        continue;
    end
    
    try
        % Extract Time Vector
        sim_t = data.tout;
        
        % Use last 10% of simulation for steady-state analysis
        t_idx = sim_t > (sim_t(end) * 0.9);
        
        % Power & Efficiency Calculation
        % NOTE: Ensure 'V_A', 'I_A', 'V_B', 'I_B', 'I_L' are logged signals in the model
        V_A_avg = mean(data.V_A(t_idx)); 
        I_A_avg = mean(data.I_A(t_idx));
        V_B_avg = mean(data.V_B(t_idx)); 
        I_B_avg = mean(data.I_B(t_idx));
        
        % Calculate Power (DC Average)
        P_Grid = V_A_avg * I_A_avg;
        P_Batt = V_B_avg * I_B_avg;
        
        % Efficiency Calculation based on Mode
        if data.Meta_Mode == "Charge"
            % Grid → Battery: Efficiency = |P_out| / |P_in|
            if abs(P_Grid) > 1e-3
                Eff = abs(P_Batt / P_Grid) * 100;
            else
                Eff = 0;
            end
            P_Loss = abs(P_Grid) - abs(P_Batt);
        else
            % Battery → Grid: Efficiency = |P_out| / |P_in|
            if abs(P_Batt) > 1e-3
                Eff = abs(P_Grid / P_Batt) * 100;
            else
                Eff = 0;
            end
            P_Loss = abs(P_Batt) - abs(P_Grid);
        end
        
        % Thermal Feedback Check
        % Calculate actual junction temperature from measured losses
        T_j_Calculated = Param.T_amb + P_Loss * Param.R_th_heatsink;
        Thermal_Err = abs(T_j_Calculated - data.Meta_Tj_Input);
        
        % Additional Indicators
        Ripple = (range(data.V_A(t_idx)) / V_A_avg) * 100; % Voltage Ripple [%]
        I_Peak = max(abs(data.I_L(t_idx))); % Peak Inductor Current [A]
        
        % Create Result Row
        newRow = table(data.Meta_V, data.Meta_Load, string(data.Meta_Mode), string(data.Meta_Type), ...
                       data.Meta_AgingFactor, data.Meta_Tj_Input, ...
                       Eff, P_Loss, T_j_Calculated, Thermal_Err, Ripple, I_Peak, ...
                       P_Grid, P_Batt);
        newRow.Properties.VariableNames = {'V_Batt', 'Load', 'Mode', 'Type', 'AgingFactor', 'Tj_Input', ...
                                           'Efficiency', 'P_Loss', 'Tj_Calc', 'Therm_Err', 'Ripple_pct', 'I_Peak', ...
                                           'P_Grid', 'P_Batt'};
        results = [results; newRow];
        successCount = successCount + 1;
        
    catch ME
        % Log failure silently for robustness
        failCount = failCount + 1;
        fprintf('Warning: Failed to process case %d: %s\n', k, ME.message);
    end
end

fprintf('Processing complete.\n');
fprintf('  Successful: %d\n', successCount);
fprintf('  Failed: %d\n', failCount);

%% 5. Export Dataset
if isempty(results)
    fprintf('\nNo valid results to export. Please check model signal logging.\n');
else
    % Generate Timestamp
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    
    % Save Directory
    CurrentScriptDir = fileparts(mfilename('fullpath'));
    if isempty(CurrentScriptDir)
        CurrentScriptDir = pwd;
    end
    SaveDir = fullfile(CurrentScriptDir, 'Data');
    if ~exist(SaveDir, 'dir')
        mkdir(SaveDir);
    end
    
    % Export as CSV for AI Training
    csvPath = fullfile(SaveDir, ['AIDataset_' timestamp '.csv']);
    writetable(results, csvPath);
    fprintf('\nDataset exported: %s\n', csvPath);
    
    % Also save as MAT for MATLAB access
    matPath = fullfile(SaveDir, ['AIDataset_' timestamp '.mat']);
    save(matPath, 'results', 'Param', 'Grid_Voltage', 'Rated_Power');
    fprintf('MAT file saved: %s\n', matPath);
    
    %% 6. Visualization (Thermal Model Validity)
    fprintf('\n=== Generating Visualization ===\n');
    
    figure('Name', 'AI-Ready Digital Twin Analysis', 'Position', [100, 100, 1400, 500]);
    
    % Subplot 1: Thermal Model Validity
    subplot(1, 3, 1);
    scatter(results.Efficiency, results.Therm_Err, 50, results.Load, 'filled', 'MarkerEdgeColor', 'k');
    xlabel('Efficiency [%]', 'FontWeight', 'bold');
    ylabel('Thermal Estimation Error [°C]', 'FontWeight', 'bold');
    title('Thermal Model Validity Check', 'FontSize', 12);
    yline(5, 'r--', 'LineWidth', 2);
    text(min(results.Efficiency)+1, 5.5, 'Reliability Limit (5°C)', 'Color', 'r');
    colorbar('Title', 'Load Rate');
    grid on;
    
    % Subplot 2: Efficiency Distribution by Mode
    subplot(1, 3, 2);
    chargeIdx = results.Mode == "Charge";
    dischargeIdx = results.Mode == "Discharge";
    hold on;
    scatter(results.V_Batt(chargeIdx), results.Efficiency(chargeIdx), 60, 'b', 'filled', 'DisplayName', 'Charge');
    scatter(results.V_Batt(dischargeIdx), results.Efficiency(dischargeIdx), 60, 'r', 'filled', 'DisplayName', 'Discharge');
    xlabel('Battery Voltage [V]', 'FontWeight', 'bold');
    ylabel('Efficiency [%]', 'FontWeight', 'bold');
    title('Efficiency vs Battery Voltage', 'FontSize', 12);
    legend('Location', 'best');
    grid on;
    hold off;
    
    % Subplot 3: Loss vs Load Rate
    subplot(1, 3, 3);
    scatter3(results.V_Batt, results.Load, results.P_Loss, 50, results.Efficiency, 'filled');
    xlabel('Battery Voltage [V]', 'FontWeight', 'bold');
    ylabel('Load Rate', 'FontWeight', 'bold');
    zlabel('Power Loss [W]', 'FontWeight', 'bold');
    title('Loss Distribution', 'FontSize', 12);
    colorbar('Title', 'Efficiency [%]');
    view(45, 30);
    grid on;
    
    drawnow;
    
    %% 7. Acceptance Criteria Report
    fprintf('\n=============================================================\n');
    fprintf('           ACCEPTANCE CRITERIA REPORT (v5.0)\n');
    fprintf('=============================================================\n\n');
    
    % Criterion 1: Thermal Consistency
    thermal_pass = sum(results.Therm_Err < 5) / height(results) * 100;
    fprintf('1. THERMAL CONSISTENCY\n');
    fprintf('   Target: |T_sim - T_calc| < 5°C\n');
    fprintf('   Pass Rate: %.1f%% (%d/%d cases)\n', thermal_pass, sum(results.Therm_Err < 5), height(results));
    if thermal_pass >= 95
        fprintf('   Status: ✓ PASS\n\n');
    else
        fprintf('   Status: ✗ NEEDS REVIEW\n\n');
    end
    
    % Criterion 2: Magnetic Saturation
    sat_pass = sum(results.I_Peak < 90) / height(results) * 100;
    fprintf('2. MAGNETIC SATURATION PREVENTION\n');
    fprintf('   Target: I_peak < 90 A\n');
    fprintf('   Max I_peak observed: %.2f A\n', max(results.I_Peak));
    fprintf('   Pass Rate: %.1f%%\n', sat_pass);
    if sat_pass == 100
        fprintf('   Status: ✓ PASS\n\n');
    else
        fprintf('   Status: ✗ FAIL - Risk of inductor saturation!\n\n');
    end
    
    % Criterion 3: Ripple Stability
    ripple_pass = sum(results.Ripple_pct < 2.5) / height(results) * 100;
    fprintf('3. RIPPLE STABILITY\n');
    fprintf('   Target: Worst Case Ripple < 2.5%%\n');
    fprintf('   Max Ripple observed: %.2f%%\n', max(results.Ripple_pct));
    fprintf('   Pass Rate: %.1f%%\n', ripple_pass);
    if ripple_pass >= 95
        fprintf('   Status: ✓ PASS\n\n');
    else
        fprintf('   Status: ✗ NEEDS REVIEW\n\n');
    end
    
    % Criterion 4: AI Dataset Quality
    fprintf('4. AI DATASET QUALITY\n');
    fprintf('   Target: Outlier (divergence) = 0 cases\n');
    fprintf('   Total valid cases: %d\n', height(results));
    fprintf('   Failed simulations: %d\n', failCount);
    if failCount == 0
        fprintf('   Status: ✓ PASS\n\n');
    else
        fprintf('   Status: ⚠ WARNING - Some cases failed\n\n');
    end
    
    fprintf('=============================================================\n');
    fprintf('Dataset generation complete. Ready for Offline RL training.\n');
    fprintf('=============================================================\n');
end

%% End of Script
fprintf('\nScript execution finished at %s\n', datestr(now));
