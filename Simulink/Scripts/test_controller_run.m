% Test script for Controller.m
clear; clc;

% Dummy Inputs
V_HV = 400;
I_HV = 0;
V_LV = 100;
I_LV = 0;
I_ess_cmd = 10;

fprintf('Testing Controller execution...\n');
try
    [t1, t3, t5, t7, mode, status, I_cmd, phi] = Controller(V_HV, I_HV, V_LV, I_LV, I_ess_cmd);
    fprintf('Controller ran successfully.\n');
    fprintf('Outputs: t1=%.2e, t3=%.2e, t5=%.2e, t7=%.2e\n', t1, t3, t5, t7);
    fprintf('Phi_ext: %.2f\n', phi);
catch ME
    fprintf('Error running Controller: %s\n', ME.message);
    fprintf('Stack trace:\n');
    disp(ME.stack(1));
    % If missing file, create it
    if contains(ME.message, 'TPS_Table.mat')
       fprintf('TPS_Table.mat missing. Running TPS_LUT generation...\n');
       TPS_LUT;
       [t1, t3, t5, t7] = Controller(V_HV, I_HV, V_LV, I_LV, I_ess_cmd); 
    end
end
