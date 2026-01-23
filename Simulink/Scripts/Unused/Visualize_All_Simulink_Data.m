% Visualize_All_Simulink_Data.m
% Automatically finds and plots ALL timeseries/array data from Workspace

clear; clc; close all;
fprintf('=== Intelligent Visualizer (Workspace Scan) ===\n');

% 1. Load Data
if ~isfile('Comparison_Results.mat')
    error('Comparison_Results.mat not found. Run simulation first.');
end
load('Comparison_Results.mat');

% 2. Extract Signals from Workspace Variables
signals = containers.Map; % Name -> Data (struct with time, values)
vars = who;

ignored_vars = {'simOut', 'mil', 'ans', 'ME', 'Ref_VH', 'Ref_VL', 'Ref_Sw', 'Ref_I_ess_cmd', ...
                'V_pri_DC', 'V_batt_nom', 'P_rated', 'f_sw', 'Ts', 'N_pri', 'N_sec', ...
                'L_leakage', 'C_dc_link', 'R_wire', 'Ron_mosfet', 'R_snubber', 'C_snubber', ...
                'I_ess_cmd', 'kp_voltage', 'ki_voltage', 'TPS_Table', 'Load', 'params', ...
                'original_dir', 'sim_elapsed', 'mil_elapsed', 'simulink_ok', 'SIM_TIME'};

fprintf('Scanning Workspace Variables...\n');

for i = 1:length(vars)
    v_name = vars{i};
    if ismember(v_name, ignored_vars), continue; end
    
    val = eval(v_name);
    
    if isa(val, 'timeseries')
        signals(v_name) = struct('time', val.Time, 'values', val.Data, 'label', v_name);
        fprintf('Found Signal (Timeseries): %s\n', v_name);
    elseif isa(val, 'struct') && isfield(val, 'time') && isfield(val, 'signals')
        % Struct format (time, signals.values)
        signals(v_name) = struct('time', val.time, 'values', val.signals.values, 'label', v_name);
        fprintf('Found Signal (Struct): %s\n', v_name);
    elseif isa(val, 'double') && size(val, 2) > 1 && size(val, 1) > 10 % Heuristic for Array [time, data]
        % Probably Array format
        signals(v_name) = struct('time', val(:,1), 'values', val(:,2:end), 'label', v_name);
        fprintf('Found Signal (Array): %s\n', v_name);
    end
end

% 3. Extract from simOut (if exists and has logsout)
if exist('simOut', 'var') && isfield(struct(simOut), 'logsout')
     logs = simOut.logsout;
     if ~isempty(logs)
        for k = 1:logs.numElements
            elem = logs{k};
            name = elem.Name;
            if isempty(name), name = sprintf('LogSignal_%d', k); end
            ts = elem.Values;
            signals(name) = struct('time', ts.Time, 'values', ts.Data, 'label', name);
            fprintf('Found Signal (simOut.logsout): %s\n', name);
        end
     end
end

% 4. Intelligent Plotting
keys_all = keys(signals);
num_sigs = length(keys_all);

if num_sigs == 0
    fprintf('No meaningful signals found to visualize.\n');
    return;
end

fprintf('Plotting %d signals...\n', num_sigs);

% Create Figures
fig_elec = figure('Name', 'Electrical Signals', 'Position', [100 100 1200 900], 'Visible', 'off');
tiledlayout(fig_elec, 'flow');

fig_ctrl = figure('Name', 'Control Signals', 'Position', [150 150 1200 900], 'Visible', 'off');
tiledlayout(fig_ctrl, 'flow');

for k = 1:num_sigs
    name = keys_all{k};
    sig = signals(name);
    
    % Classification
    is_elec = contains(name, {'V', 'I', 'Volt', 'Curr', 'Amp', 'Power', 'Watt'}, 'IgnoreCase', true);
    is_ctrl = contains(name, {'Duty', 'D1', 'D2', 'Phi', 'Mode', 'State', 'Err', 'Active'}, 'IgnoreCase', true);
    
    if is_elec
        parent_fig = fig_elec;
    elseif is_ctrl
        parent_fig = fig_ctrl;
    else
        parent_fig = fig_elec; % Default
    end
    
    nexttile(parent_fig);
    plot(sig.time, sig.values, 'LineWidth', 1.5);
    title(name, 'Interpreter', 'none');
    grid on;
    xlabel('Time (s)');
    
    % Add Comparison with MIL Ref/Out if applicable
    if contains(name, 'I_') || contains(name, 'Current')
         hold on;
         if exist('mil', 'var') 
             if isfield(mil, 'I_ref'), plot(mil.time, mil.I_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'MIL Ref'); end
             if isfield(mil, 'I_out'), plot(mil.time, mil.I_out, 'r:', 'LineWidth', 1.5, 'DisplayName', 'MIL Out'); end
             legend('show'); 
         end
    end
end

saveas(fig_elec, '../../Simulink_Electrical.png');
saveas(fig_ctrl, '../../Simulink_Control.png');

fprintf('Saved plots to Simulink_Electrical.png and Simulink_Control.png\n');
