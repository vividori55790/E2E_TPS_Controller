% Plot_Results_To_File.m
clear; close all;
load('Comparison_Results.mat');

% Create invisible figure for saving
f = figure('Visible', 'off', 'Position', [0 0 1200 800]);

% Extract Data
t = mil.time * 1000; % ms
V_in = mil.V_in;
I_out = mil.I_out;
Link_Mode = mil.mode;

% Subplot 1: Input Voltage and Mode
subplot(3,1,1);
yyaxis left
plot(t, V_in, 'k-', 'LineWidth', 1.5);
ylabel('Input Voltage (V)');
ylim([0 450]);
yyaxis right
plot(t, Link_Mode, 'r-', 'LineWidth', 1.5);
ylabel('Mode (0:ESS, 1:UPS)');
ylim([-0.2 1.2]);
yticks([0 1]);
title('System State: Voltage Dip & Mode Transfer');
grid on;
legend('V_{in}', 'Mode');

% Subplot 2: Output Current
subplot(3,1,2);
plot(t, I_out, 'b-', 'LineWidth', 1.5); hold on;
if isfield(mil, 'I_ref')
    plot(t, mil.I_ref, 'g--', 'LineWidth', 1.5);
    legend('I_{out}', 'I_{ref}');
else
    legend('I_{out}');
end
ylabel('Output Current (A)');
grid on;
title('Output Current Regulation');

% Subplot 3: Phase Shift (Control Action)
subplot(3,1,3);
plot(t, mil.phi_ext, 'm-', 'LineWidth', 1.5);
ylabel('Phase Shift (deg)');
xlabel('Time (ms)');
grid on;
title('Control Effort (Phase Shift)');

% Save to file
saveas(f, '../../Simulation_Results.png');
close(f);
fprintf('Plot saved to Simulation_Results.png\n');
