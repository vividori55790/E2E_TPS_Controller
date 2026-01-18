% Run Script for Main_Simulation.slx (Short Test)
clear; clc;

% --- Parameters ---
V_pri_DC = 400; V_batt_nom = 100; P_rated = 5000;
f_sw = 80e3; Ts = 1/f_sw; 
N_pri = 4; N_sec = 1; L_leakage = 50e-6;
C_dc_link = 500e-6; R_wire = 1e-3; Ron_mosfet = 5e-3;
R_snubber = 1e5; C_snubber = inf;
I_ess_cmd = 20;
kp_voltage = 5; ki_voltage = 5000;

if ~isfile('TPS_Table.mat')
    error('TPS_Table.mat missing.');
end

fprintf('Starting Short Simulation (5ms)...\n');
try
    % Override StopTime to 5ms for quick verification
    simOut = sim('Main_Simulation.slx', 'StopTime', '0.005');
    fprintf('Simulation Completed Successfully (5ms).\n');
catch ME
    fprintf('Error during simulation:\n%s\n', ME.message);
end
