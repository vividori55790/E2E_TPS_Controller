% Visualize_Specific_Simulink_Signals.m
% Extracts and plots specific bus structures: Data_IO, Data_Control

clear; clc; close all;
fprintf('=== Intelligent Visualizer (Simulink Focus) ===\n');

% 1. Load Data
if ~isfile('Comparison_Results.mat')
    error('Comparison_Results.mat not found. Run simulation first.');
end
load('Comparison_Results.mat');

if ~exist('simOut', 'var')
    error('simOut not found inside Comparison_Results.mat');
end

% 2. Extract Signals from simOut
signals = containers.Map; 

% Helper to extract from bus
extract_bus = @(bus_name) extract_bus_signals(simOut, bus_name);

% Try to extract known buses
bus_IO = extract_bus('Data_IO');
bus_Ctrl = extract_bus('Data_Control');
bus_Phys = extract_bus('Data_Physical');

% Flatten Logic
all_signals = [bus_IO; bus_Ctrl; bus_Phys];

if isempty(all_signals)
    fprintf('No structured bus signals (Data_IO, Data_Control) found in simOut.\n');
    fprintf('Checking top-level logsout...\n');
    
    % Fallback: logsout inspection
    if isfield(struct(simOut), 'logsout')
        logs = simOut.logsout;
        for i = 1:logs.numElements
            elem = logs{i};
            s_name = elem.Name;
            if isempty(s_name), s_name = sprintf('Signal_%d',i); end
            
            % Check if it matches our targets
            if contains(s_name, {'V_HV', 'I_HV', 'V_LV', 'I_LV', 'D1', 'Phi', 'Mode'})
                 signals(s_name) = struct('time', elem.Values.Time, 'values', elem.Values.Data);
                 fprintf('Found fallback signal: %s\n', s_name);
            end
        end
    end
else
    % Map found bus signals
    for i = 1:length(all_signals)
        s = all_signals(i);
        signals(s.label) = s;
        fprintf('Found Bus Signal: %s\n', s.label);
    end
end

% 3. Plotting
if isempty(signals)
    fprintf('No signals found to plot.\n');
    return; 
end

fprintf('Plotting...\n');

% Figure 1: Electrical (V_HV, V_LV, I_HV, I_LV)
fig_elec = figure('Name', 'Electrical Response', 'Position', [100 100 1200 800], 'Visible', 'off');
tiledlayout(fig_elec, 2, 2);

targets_elec = {'V_HV', 'I_HV', 'V_LV', 'I_LV'};
for i = 1:length(targets_elec)
    name = targets_elec{i};
    nexttile;
    if isKey(signals, name)
        sig = signals(name);
        plot(sig.time, sig.values, 'LineWidth', 1.5);
        title(name, 'Interpreter', 'none');
        grid on;
        
        % Compare I_LV with Ref if available
        if strcmp(name, 'I_LV') && exist('Ref_I_ess_cmd', 'var')
             hold on;
             plot(Ref_I_ess_cmd.Time, Ref_I_ess_cmd.Data, 'g--', 'LineWidth', 1.5);
             legend('Measured', 'Reference');
        end
    else
        text(0.5, 0.5, [name ' Not Found'], 'HorizontalAlignment', 'center');
    end
end
saveas(fig_elec, '../../Simulink_Electrical_Bus.png');


% Figure 2: Control (Mode, Phase, Duty)
fig_ctrl = figure('Name', 'Control Response', 'Position', [150 150 1200 400], 'Visible', 'off');
tiledlayout(fig_ctrl, 1, 3);

targets_ctrl = {'Mode_Active', 'Phi_Ext', 'D1_out'}; 
% Adjust names if slightly different (e.g. Mode vs Mode_Active)
% Heuristic mapping
key_map = containers.Map;
keys_avail = keys(signals);
for k=1:length(keys_avail)
    k_name = keys_avail{k};
    if contains(k_name, 'Mode', 'IgnoreCase', true), key_map('Mode_Active') = k_name; end
    if contains(k_name, 'Phi', 'IgnoreCase', true), key_map('Phi_Ext') = k_name; end
    if contains(k_name, 'D1', 'IgnoreCase', true), key_map('D1_out') = k_name; end
end

for i = 1:length(targets_ctrl)
    t_name = targets_ctrl{i};
    nexttile;
    if isKey(key_map, t_name)
        real_name = key_map(t_name);
        sig = signals(real_name);
        plot(sig.time, sig.values, 'LineWidth', 1.5);
        title(real_name, 'Interpreter', 'none');
        grid on;
    else
         text(0.5, 0.5, [t_name ' Not Found'], 'HorizontalAlignment', 'center');
    end
end
saveas(fig_ctrl, '../../Simulink_Control_Bus.png');
fprintf('Saved plots to Simulink_Electrical_Bus.png and Simulink_Control_Bus.png\n');

% --- Helper ---
function extracted = extract_bus_signals(simOut, bus_name)
    extracted = [];
    % Check logsout first
    if isfield(struct(simOut), 'logsout')
        logs = simOut.logsout;
        idx = logs.find('Name', bus_name); % Find Element by Name? 
        % Actually logsout.find returns element.
        % If bus is inside, it might be a struct element
        
        % Try generic property search
        try
             data = simOut.get(bus_name); % Direct access if top-level logged
             % If it is a Struct with timeseries fields
             if isstruct(data)
                 f = fieldnames(data);
                 for k=1:length(f)
                     sub_d = data.(f{k});
                     if isa(sub_d, 'timeseries')
                         s.label = f{k}; % e.g. V_HV
                         s.time = sub_d.Time;
                         s.values = sub_d.Data;
                         extracted = [extracted; s];
                     end
                 end
             end
        catch
        end
    end
end
