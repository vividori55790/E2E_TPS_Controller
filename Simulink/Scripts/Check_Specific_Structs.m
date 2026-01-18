% Check_Specific_Structs.m
clear; clc;
if ~isfile('Comparison_Results.mat')
    error('Comparison_Results.mat not found.');
end
load('Comparison_Results.mat');

target_vars = {'Data_IO', 'Data_Control', 'Data_Physical'};
found_any = false;

fprintf('=== Checking for User-Specified Bus Structures ===\n');
for i = 1:length(target_vars)
    v_name = target_vars{i};
    if exist(v_name, 'var')
        found_any = true;
        val = eval(v_name);
        fprintf('\n[Found Structure: %s]\n', v_name);
        disp(val);
        
        % Check for timeseries inside fields
        fields = fieldnames(val);
        for k=1:length(fields)
            f_val = val.(fields{k});
            if isa(f_val, 'timeseries')
                fprintf('  - Field: %s (Timeseries, %d samples)\n', fields{k}, length(f_val.Time));
            else
                 fprintf('  - Field: %s (%s)\n', fields{k}, class(f_val));
            end
        end
    else
        fprintf('[Missing] %s not found in workspace.\n', v_name);
    end
end
