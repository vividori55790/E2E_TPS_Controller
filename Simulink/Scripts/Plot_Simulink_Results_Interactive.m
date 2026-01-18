% Plot_Simulink_Results_Interactive.m
% Extracts Data_IO, Data_Control, Data_Physical from 'out' object
% and plots them in MATLAB figures.

clear; clc; close all;

% 1. Load Data
if isfile('Comparison_Results.mat')
    load('Comparison_Results.mat');
    fprintf('Loaded Comparison_Results.mat\n');
else
    fprintf('Warning: Comparison_Results.mat not found. Assuming data is in workspace.\n');
end

if ~exist('out', 'var')
    error('Variable "out" not found.');
end

% 2. Extract Data Structures using get() method
try
    Data_IO = out.get('Data_IO');
    fprintf('Extracted Data_IO\n');
catch
    fprintf('Warning: Data_IO not found in out object.\n');
    Data_IO = [];
end

try
    Data_Control = out.get('Data_Control');
    fprintf('Extracted Data_Control\n');
catch
    fprintf('Warning: Data_Control not found in out object.\n');
    Data_Control = [];
end

try
    Data_Physical = out.get('Data_Physical');
    fprintf('Extracted Data_Physical\n');
catch
    fprintf('Warning: Data_Physical not found in out object.\n');
    Data_Physical = [];
end

% Helper function to extract timeseries from generic struct
    function results = extract_signals_from_struct(input_struct)
        results = containers.Map;
        if isfield(input_struct, 'signals') && isstruct(input_struct.signals)
            % Check if signals is a struct array or struct with fields
            sigs = input_struct.signals;
            
            % Case 1: Struct with fields (Bus)
            f = fieldnames(sigs);
            time_vec = [];
            if isfield(input_struct, 'time'), time_vec = input_struct.time; end
            
            for k = 1:length(f)
                fname = f{k};
                val = sigs.(fname);
                if isnumeric(val) || isa(val, 'timeseries')
                     % Create pseudo-timeseries struct
                     str.Time = time_vec; 
                     str.Data = val;
                     results(fname) = str;
                end
            end
            
            % Case 2: Signal field itself has values (Single signal)
            if isfield(sigs, 'values')
                 str.Time = time_vec;
                 str.Data = sigs.values;
                 if isfield(sigs, 'label') && ~isempty(sigs.label)
                     results(sigs.label) = str;
                 else
                     results('Signal') = str;
                 end
            end
            
        else
            % Flattened struct (fields are signals)
            f = fieldnames(input_struct);
            for k = 1:length(f)
                fname = f{k};
                val = input_struct.(fname);
                if isa(val, 'timeseries')
                    results(fname) = val;
                end
            end
        end
    end

% 3. Plot Data_IO (Electrical)
if ~isempty(Data_IO)
    figure('Name', 'Data_IO (Electrical)', 'Position', [100 100 1200 800]);
    
    extracted_IO = extract_signals_from_struct(Data_IO);
    io_keys = keys(extracted_IO);
    
    if isempty(io_keys)
        fprintf('  No valid signals found in Data_IO structure.\n');
        disp(Data_IO);
    else
        num_fields = length(io_keys);
        rows = ceil(sqrt(num_fields));
        cols = ceil(num_fields / rows);
        
        for i = 1:num_fields
            field_name = io_keys{i};
            ts = extracted_IO(field_name);
            
            subplot(rows, cols, i);
            % Robust plotting
            if isstruct(ts) && isfield(ts, 'Time') && isfield(ts, 'Data')
                 plot(ts.Time, ts.Data, 'LineWidth', 1.5); hold on;
                 t_len = length(ts.Time);
            elseif isa(ts, 'timeseries')
                 plot(ts.Time, ts.Data, 'LineWidth', 1.5); hold on;
                 t_len = length(ts.Time);
            else
                 continue;
            end
            
            % Overlay Reference
            if strcmp(field_name, 'I_LV') && exist('Ref_I_ess_cmd', 'var')
                plot(Ref_I_ess_cmd.Time, Ref_I_ess_cmd.Data, 'g--', 'LineWidth', 1.5);
                legend('Measured', 'Reference');
            elseif strcmp(field_name, 'V_HV') && exist('Ref_VH', 'var')
                plot(Ref_VH.Time, Ref_VH.Data, 'r--', 'LineWidth', 1.5);
                legend('Measured', 'Reference');
            end
            
            title(field_name, 'Interpreter', 'none');
            grid on;
            xlabel('Time (s)');
            ylabel('Amplitude');
        end
    end
end

% 4. Plot Data_Control
if ~isempty(Data_Control)
    figure('Name', 'Data_Control', 'Position', [150 150 1200 600]);
    
    extracted_Ctrl = extract_signals_from_struct(Data_Control);
    ctrl_keys = keys(extracted_Ctrl);
    
    if isempty(ctrl_keys)
         fprintf('  No valid signals found in Data_Control.\n');
    else
        num_fields = length(ctrl_keys);
        rows = ceil(sqrt(num_fields));
        cols = ceil(num_fields / rows);
        
        for i = 1:num_fields
            field_name = ctrl_keys{i};
            ts = extracted_Ctrl(field_name);
            subplot(rows, cols, i);
            if isstruct(ts), plot(ts.Time, ts.Data, 'LineWidth', 1.5); 
            elseif isa(ts, 'timeseries'), plot(ts.Time, ts.Data, 'LineWidth', 1.5); end
            title(field_name, 'Interpreter', 'none');
            grid on;
        end
    end
end

% 5. Plot Data_Physical
if ~isempty(Data_Physical)
    figure('Name', 'Data_Physical', 'Position', [200 200 1200 600]);
    
    extracted_Phys = extract_signals_from_struct(Data_Physical);
    phys_keys = keys(extracted_Phys);
    
    if isempty(phys_keys)
         fprintf('  No valid signals found in Data_Physical.\n');
    else
        num_fields = length(phys_keys);
        rows = ceil(sqrt(num_fields));
        cols = ceil(num_fields / rows);
        
        for i = 1:num_fields
            field_name = phys_keys{i};
            ts = extracted_Phys(field_name);
            subplot(rows, cols, i);
            if isstruct(ts), plot(ts.Time, ts.Data, 'LineWidth', 1.5); 
            elseif isa(ts, 'timeseries'), plot(ts.Time, ts.Data, 'LineWidth', 1.5); end
            title(field_name, 'Interpreter', 'none');
            grid on;
        end
    end
end

fprintf('Done. Check the generated figure windows.\n');
