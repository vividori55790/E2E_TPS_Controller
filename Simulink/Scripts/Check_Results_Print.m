% Check_Results.m
% Load results and print summary
load('Comparison_Results.mat');

fprintf('=== Simulation Verification Results ===\n');

% MIL Data
t_mil = mil.time;
I_mil = mil.I_out;
V_mil = mil.V_in;
Mode_mil = mil.mode;

% Check Points
points = {0.030, 'ESS Charge (10A)';
          0.050, 'ESS Discharge (-10A)';
          0.075, 'UPS Mode (V_in=330V)'};

fprintf('\n--- MIL Results ---\n');
for i = 1:size(points, 1)
    t_chk = points{i,1};
    label = points{i,2};
    
    idx = find(t_mil >= t_chk, 1);
    
    fprintf('[%s] @ t=%.3fs\n', label, t_chk);
    fprintf('  Input Voltage: %.1f V\n', V_mil(idx));
    fprintf('  Mode:          %d (0=ESS, 1=UPS)\n', Mode_mil(idx));
    fprintf('  Output Current: %.3f A\n', I_mil(idx));
    fprintf('--------------------------------\n');
end

% Simulink Data Check (if available in logsout)
if exist('simOut', 'var')
    fprintf('\n--- Simulink Results (Partial Check) ---\n');
    try
        logs = simOut.logsout;
        % Try to find signals. Assuming names like 'I_out', 'V_HV', 'Mode_Active' etc.
        % We need to know exact signal names in Simulink. 
        % Based on previous knowledge, maybe 'I_measured', 'I_cmd', etc.
        % Let's just inspect element names first.
        fprintf('Available Signals: ');
        for k=1:logs.numElements
             fprintf('%s, ', logs{k}.Name);
        end
        fprintf('\n');
    catch
        fprintf('Could not parse logsout.\n');
    end
end
