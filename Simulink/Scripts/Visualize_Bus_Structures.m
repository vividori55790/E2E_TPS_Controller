% Visualize_Bus_Structures.m
% Extracts and plots user-specified bus structures: Data_IO, Data_Control
% Accesses 'out' object directly.

clear; clc; close all;
fprintf('=== Intelligent Visualizer (Simulink Bus Structures) ===\n');

% 1. Load Data
if ~isfile('Comparison_Results.mat')
    error('Comparison_Results.mat not found. Run simulation first.');
end
load('Comparison_Results.mat');

if ~exist('out', 'var')
    error('Variable "out" not found inside Comparison_Results.mat');
end

% 2. Extract Signals from out object
signals_found = containers.Map; 

% Buses to look for
target_buses = {'Data_IO', 'Data_Control'};

for i = 1:length(target_buses)
    bus_name = target_buses{i};
    try
        % Try dot notation (works if created as top-level property)
        data_struct = out.(bus_name);
        fprintf('[Found Bus]: %s\n', bus_name);
        
        % Iterate through fields of the struct
        fields = fieldnames(data_struct);
        for k = 1:length(fields)
            f_name = fields{k};
            s_val = data_struct.(f_name);
            
            if isa(s_val, 'timeseries')
                % Store flattened: "Data_IO.V_HV" -> Key "V_HV" (or full name?)
                % User request: V_HV, I_HV etc.
                % Let's use simple name if unique, else full name.
                signals_found(f_name) = s_val;
                fprintf('  - Extracted: %s (%d samples)\n', f_name, length(s_val.Time));
            else
                fprintf('  - Skipped Field: %s (Type: %s)\n', f_name, class(s_val));
            end
        end
    catch
        fprintf('[Missing]: %s not found in "out" object.\n', bus_name);
    end
end

% 3. Plotting
if isempty(signals_found)
    fprintf('No signals extracted. Checking available properties in out:\n');
    disp(out);
    return;
end

fprintf('Plotting extracted signals...\n');

% Figure 1: Electrical
fig_elec = figure('Name', 'Electrical Response', 'Position', [100 100 1000 800], 'Visible', 'off');
tiledlayout(fig_elec, 2, 2);

elec_vars = {'V_HV', 'I_HV', 'V_LV', 'I_LV'};

for i = 1:length(elec_vars)
    v = elec_vars{i};
    nexttile;
    if isKey(signals_found, v)
        ts = signals_found(v);
        plot(ts.Time, ts.Data, 'LineWidth', 1.5);
        title(v, 'Interpreter', 'none');
        grid on;
        
        % Add Reference Comparison for Current
        if strcmp(v, 'I_LV') && exist('Ref_I_ess_cmd', 'var')
            hold on;
            plot(Ref_I_ess_cmd.Time, Ref_I_ess_cmd.Data, 'g--', 'LineWidth', 1.5);
            legend('Measured', 'Ref Command');
        elseif strcmp(v, 'V_HV') && exist('Ref_VH', 'var')
             hold on;
             plot(Ref_VH.Time, Ref_VH.Data, 'k--', 'LineWidth', 1.5);
             legend('Measured', 'Ref Voltage');
        end
    else
        text(0.5, 0.5, [v ' Not Found'], 'HorizontalAlignment', 'center');
    end
end
saveas(fig_elec, '../../Simulink_Electrical_Bus.png');

% Figure 2: Control
fig_ctrl = figure('Name', 'Control Response', 'Position', [150 150 1000 400], 'Visible', 'off');
tiledlayout(fig_ctrl, 1, 3);

% Heuristic naming match
ctrl_vars = {'Mode', 'Phi', 'D1'}; % Search keywords

keys_found = keys(signals_found);

for i = 1:length(ctrl_vars)
    keyword = ctrl_vars{i};
    nexttile;
    
    % Find matching key
    match = '';
    for k = 1:length(keys_found)
        if contains(keys_found{k}, keyword, 'IgnoreCase', true)
            match = keys_found{k};
            break;
        end
    end
    
    if ~isempty(match)
        ts = signals_found(match);
        plot(ts.Time, ts.Data, 'LineWidth', 1.5);
        title(match, 'Interpreter', 'none');
        grid on;
    else
        text(0.5, 0.5, [keyword ' Not Found'], 'HorizontalAlignment', 'center');
    end
end
saveas(fig_ctrl, '../../Simulink_Control_Bus.png');

fprintf('Saved plots to Simulink_Electrical_Bus.png and Simulink_Control_Bus.png\n');
