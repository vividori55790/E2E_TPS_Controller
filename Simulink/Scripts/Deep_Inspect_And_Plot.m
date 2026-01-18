% Deep_Inspect_And_Plot.m
% Exhaustively searches 'out' object recursively for signals to plot.
% Handles Data_IO, Data_Control, and Data_Physical specifically.

clear; clc; close all;
fprintf('=== Deep Inspection & Plotting (Recursive) ===\n');

if ~isfile('Comparison_Results.mat')
    error('Comparison_Results.mat not found. Please run the simulation step first.');
end
load('Comparison_Results.mat');

if ~exist('out', 'var')
    error('"out" variable not found in workspace.');
end

% Container for signals to plot: Key = Full Path Name, Value = struct(time, values)
signals = containers.Map; 

% Helper function for recursion
    function extract_recursive(data, prefix)
        if isa(data, 'timeseries')
            signals(prefix) = struct('time', data.Time, 'values', data.Data);
            fprintf('  [Found Signal]: %s (%d samples)\n', prefix, length(data.Time));
            
        elseif isstruct(data)
            fields = fieldnames(data);
            for i = 1:length(fields)
                f_name = fields{i};
                new_prefix = [prefix, '.', f_name];
                if startsWith(new_prefix, '.'), new_prefix = f_name; end
                extract_recursive(data.(f_name), new_prefix);
            end
            
        elseif isa(data, 'Simulink.SimulationData.Dataset')
            for i = 1:data.numElements
                el = data.get(i);
                el_name = el.Name;
                if isempty(el_name), el_name = sprintf('Element_%d', i); end
                new_prefix = [prefix, '.', el_name];
                if startsWith(new_prefix, '.'), new_prefix = el_name; end
                
                extract_recursive(el.Values, new_prefix);
            end
        end
    end

% 1. Start Extraction
try
    % Check specific properties first
    targets = {'Data_IO', 'Data_Control', 'Data_Physical'};
    for i = 1:length(targets)
        t_name = targets{i};
        try
            val = out.get(t_name); % Access via get method
            extract_recursive(val, t_name);
        catch
            fprintf('  [Info]: %s not reachable via get(). Trying property.\n', t_name);
            if isprop(out, t_name)
                 extract_recursive(out.(t_name), t_name);
            end
        end
    end
    
    % Also check generic out properties if nothing found
    if isempty(signals)
        fprintf('  [Info]: No signals found yet. Scanning properties...\n');
        props = properties(out);
        for i = 1:length(props)
            extract_recursive(out.(props{i}), props{i});
        end
    end
    
catch ME
    fprintf('Error during extraction: %s\n', ME.message);
end

% 2. Plotting Logic
if isempty(signals)
    fprintf('No signals extracted. Cannot generate plots.\n');
    return;
end

fprintf('Generating Plots...\n');
all_keys = keys(signals);

% --- Figure 1: Electrical (Data_IO) ---
fig_elec = figure('Name', 'Electrical (Data_IO)', 'Position', [100 100 1200 800], 'Visible', 'off');
tiledlayout(fig_elec, 'flow');
has_elec = false;

for i = 1:length(all_keys)
    k = all_keys{i};
    if contains(k, 'Data_IO') || contains(k, 'V_') || contains(k, 'I_')
        nexttile;
        s = signals(k);
        plot(s.time, s.values, 'LineWidth', 1.5);
        title(k, 'Interpreter', 'none'); 
        grid on;
        has_elec = true;
        
        % Overlay Ref
        if contains(k, 'I_LV') && exist('Ref_I_ess_cmd', 'var')
            hold on; plot(Ref_I_ess_cmd.Time, Ref_I_ess_cmd.Data, 'g--', 'LineWidth', 1.5); legend('Meas', 'Ref');
        elseif contains(k, 'V_HV') && exist('Ref_VH', 'var')
            hold on; plot(Ref_VH.Time, Ref_VH.Data, 'k--', 'LineWidth', 1.5); legend('Meas', 'Ref');
        end
    end
end
if has_elec, saveas(fig_elec, '../../Simulink_Data_IO.png'); end


% --- Figure 2: Control (Data_Control) ---
fig_ctrl = figure('Name', 'Control (Data_Control)', 'Position', [150 150 1200 600], 'Visible', 'off');
tiledlayout(fig_ctrl, 'flow');
has_ctrl = false;

for i = 1:length(all_keys)
    k = all_keys{i};
    if contains(k, 'Data_Control') || contains(k, 'Mode') || contains(k, 'Phi') || contains(k, 'Duty')
        nexttile;
        s = signals(k);
        plot(s.time, s.values, 'LineWidth', 1.5);
        title(k, 'Interpreter', 'none'); 
        grid on;
        has_ctrl = true;
    end
end
if has_ctrl, saveas(fig_ctrl, '../../Simulink_Data_Control.png'); end


% --- Figure 3: Physical (Data_Physical) ---
fig_phys = figure('Name', 'Physical (Data_Physical)', 'Position', [200 200 1200 600], 'Visible', 'off');
tiledlayout(fig_phys, 'flow');
has_phys = false;

for i = 1:length(all_keys)
    k = all_keys{i};
    if contains(k, 'Data_Physical')
        nexttile;
        s = signals(k);
        plot(s.time, s.values, 'LineWidth', 1.5);
        title(k, 'Interpreter', 'none'); 
        grid on;
        has_phys = true;
    end
end
if has_phys, saveas(fig_phys, '../../Simulink_Data_Physical.png'); end


fprintf('Plots generated.\n');
