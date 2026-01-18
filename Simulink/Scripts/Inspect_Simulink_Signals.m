% Inspect_Simulink_Signals.m
clear; clc;
load('Comparison_Results.mat');

if exist('simOut', 'var')
    fprintf('=== Simulink Signal Inspection ===\n');
    
    % Check logsout
    if isfield(struct(simOut), 'logsout') 
        logs = simOut.logsout;
        num_sigs = logs.numElements;
        fprintf('Found %d signals in logsout:\n', num_sigs);
        for i = 1:num_sigs
            sig = logs{i};
            fprintf('  - %s (Block: %s)\n', sig.Name, sig.BlockPath.getBlock(1));
        end
    else
        fprintf('No logsout found in simOut.\n');
    end
    
    % Check separate timeseries in workspace (if 'To Workspace' blocks used)
    vars = who;
    fprintf('\n=== Workspace Variables ===\n');
    for i = 1:length(vars)
        if strcmp(vars{i}, 'simOut') || strcmp(vars{i}, 'mil'), continue; end
        val = eval(vars{i});
        if isa(val, 'timeseries')
            fprintf('  - %s (Timeseries, %d samples)\n', vars{i}, length(val.Time));
        end
    end
else
    fprintf('simOut variable not found in Comparison_Results.mat.\n');
end

% Check MIL Negative Current
min_I_mil = min(mil.I_out);
fprintf('\n=== MIL Current Check ===\n');
fprintf('Minimum MIL Current: %.3f A\n', min_I_mil);
if min_I_mil < -1
    fprintf('Verified: Negative current flow (Discharge) is active.\n');
else
    fprintf('Warning: Current did not go negative (Discharge failed).\n');
end
