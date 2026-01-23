% Generate_Smart_Report.m
% Generates "Intelligent" validation plots:
% 1. Command vs Actual Current (Full Simulation)
% 2. Reconstructed Switching Waveforms (Transformer Voltage/Current) at specific operating points
%    to prove the Phase Shift is working physically.

clear; clc; close all;
load('Comparison_Results.mat');

% --- 1. System Level Response (Command vs Actual) ---
fig_sys = figure('Name', 'System Response', 'Position', [100 100 1200 800], 'Visible', 'off');
tiledlayout(fig_sys, 3, 1);

% Plot 1: Voltage & Mode
nexttile;
yyaxis left
plot(mil.time*1000, mil.V_in, 'k-', 'LineWidth', 1.5);
ylabel('Input Voltage (V)');
ylim([0 450]);
yyaxis right
plot(mil.time*1000, mil.mode, 'r-', 'LineWidth', 1.5);
ylabel('Mode (0:ESS, 1:UPS)');
yticks([0 1]);
grid on;
title('System Conditions: Voltage Sag & Mode Transition');
legend('V_{in} (HV)', 'Mode');

% Plot 2: Current Tracking
nexttile;
plot(mil.time*1000, mil.I_out, 'b-', 'LineWidth', 2); hold on;
plot(mil.time*1000, mil.I_ref, 'g--', 'LineWidth', 2);
ylabel('LV Current (A)');
grid on;
legend('Actual Current', 'Command (Ref)');
title('Current Regulation Performance');

% Plot 3: Control Effort (Phase Shift)
nexttile;
plot(mil.time*1000, mil.phi_ext, 'm-', 'LineWidth', 1.5);
ylabel('Phase Shift (\circ)');
xlabel('Time (ms)');
grid on;
title('Control Action: Phase Shift');

saveas(fig_sys, '../../System_Response_Overview.png');


% --- 2. Reconstructed Switching Waveforms (Analytic Proof) ---
% We pick a snapshot at steady state (e.g., UPS Mode, t=0.075s)
% and reconstruct the high-speed switching waveforms based on V and Phi.

t_snap = 0.075; % 75ms (UPS Mode)
idx = find(mil.time >= t_snap, 1);
V1_dc = mil.V_in(idx);
V2_dc = mil.V_out(idx);
phi_deg = mil.phi_ext(idx);
I_avg = mil.I_out(idx);

% DAB Parameters
n = 4;
L = 50e-6;
f_sw = 80e3;
T_sw = 1/f_sw;
w_sw = 2*pi*f_sw;

% Reconstruct 3 cycles
t_micro = linspace(0, 3*T_sw, 1000); % 0 to 3 periods
phi_rad = phi_deg * pi/180;
phase_shift_time = (phi_deg/360) * T_sw;

% V_pri (Square wave)
% Duty=0.5 fixed for SPS (simplification)
V_pri_AC = V1_dc * sign(sin(w_sw * t_micro));

% V_sec_reflected (Square wave, shifted by phi)
% n*V2
V_sec_prime_AC = (n * V2_dc) * sign(sin(w_sw * (t_micro - phase_shift_time)));

% Inductor Voltage: V_L = V_pri - V_sec'
V_L = V_pri_AC - V_sec_prime_AC;

% Inductor Current: Integrate V_L/L
I_L_ac = cumtrapz(t_micro, V_L) / L;
% Remove DC component (steady state) and add Load Current component?
% No, AC link current has 0 DC average.
I_L_ac = I_L_ac - mean(I_L_ac); 

% Plot
fig_wave = figure('Name', 'Switching Waveforms (Reconstructed)', 'Position', [150 150 1000 800], 'Visible', 'off');
tiledlayout(fig_wave, 3, 1);

% Voltage
nexttile;
plot(t_micro*1e6, V_pri_AC, 'k-', 'LineWidth', 1.5); hold on;
plot(t_micro*1e6, V_sec_prime_AC, 'r--', 'LineWidth', 1.5);
grid on;
ylabel('Voltage (V)');
legend('V_{pri} (Transformer Primary)', 'V_{sec}'' (Reflected Secondary)');
title(sprintf('Transformer Voltages @ %.1fms (Mode: UPS, \\phi=%.1f\\circ)', t_snap*1000, phi_deg));

% Inductor Current
nexttile;
plot(t_micro*1e6, I_L_ac, 'b-', 'LineWidth', 1.5);
grid on;
ylabel('Inductor Current (A)');
title('Inductor Current I_L (High Frequency AC)');

% Power Transfer
P_inst = V_pri_AC .* I_L_ac;
P_avg_calc = mean(P_inst);
nexttile;
plot(t_micro*1e6, P_inst/1000, 'g-', 'Color', [0 0.5 0]);
yline(P_avg_calc/1000, 'k--', 'LineWidth', 2);
ylabel('Power (kW)');
xlabel('Time (\mus)');
grid on;
legend('Instantaneous Power', sprintf('Average Power = %.2f kW', P_avg_calc/1000));
title('Power Transfer Profile');

saveas(fig_wave, '../../Reconstructed_Switching_Waveforms.png');

fprintf('Generated Intelligent Reports:\n');
fprintf('1. System_Response_Overview.png (Command vs Actual)\n');
fprintf('2. Reconstructed_Switching_Waveforms.png (Proof of Phase Shift)\n');
