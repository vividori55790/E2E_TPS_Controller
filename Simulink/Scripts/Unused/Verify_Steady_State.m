% =========================================================================
% Verify_Steady_State.m
% MIL 정상상태 분석 (긴 시뮬레이션)
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  MIL Steady State Analysis (Long Simulation)\n');
fprintf('=================================================================\n\n');

%% Parameters
params.n = 4.0;
params.L = 50e-6;
params.fs = 80e3;
params.C_out = 500e-6;
params.efficiency = 0.95;

V_in = 400;
V_out_init = 100;
Load = 10;  % 10 ohm

Phi_int_deg = 15;

%% Controller
Kp = 0.5;
Ki = 100;  % 더 높은 적분 이득으로 빠른 수렴
I_ref = 10;

%% Simulation
dt = 10e-6;
sim_time = 0.200;  % 200ms (충분히 길게)
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

fprintf('시뮬레이션 설정:\n');
fprintf('  - 시간: %.0f ms\n', sim_time*1000);
fprintf('  - Kp=%.2f, Ki=%.1f\n', Kp, Ki);
fprintf('  - 목표 전류: %.1f A\n', I_ref);

%% 이론값 계산
% P_required = V_out * I_out = 100 * 10 = 1000 W
% P = K * phi * (1 - |phi|/pi)
% K = (V1 * V2') / (2*pi*fs*L)
V2_prime = params.n * V_out_init;
K = (V_in * V2_prime) / (2 * pi * params.fs * params.L);
P_required = V_out_init * I_ref;

% phi 계산 (근사)
% 1000 = 6366 * phi * (1 - phi/pi)
% phi가 작다고 가정하면: 1000 ≈ 6366 * phi
phi_approx = P_required / K;
fprintf('  - 이론적 위상: %.2f rad (%.1f deg)\n', phi_approx, phi_approx*180/pi);
fprintf('  - K 계수: %.0f W/rad\n', K);
fprintf('  - 필요 전력: %.0f W\n\n', P_required);

%% Simulation
V_out = V_out_init;
integrator = 0;

results.time = time_vec;
results.V_out = zeros(n_steps, 1);
results.I_out = zeros(n_steps, 1);
results.P = zeros(n_steps, 1);
results.phi = zeros(n_steps, 1);

fprintf('시뮬레이션 실행 중...\n');
tic;

for k = 1:n_steps
    I_fb = V_out / Load;
    
    % Step input
    if time_vec(k) < 0.010
        I_ref_now = 0;
    else
        I_ref_now = I_ref;
    end
    
    % PI Controller
    error_sig = I_ref_now - I_fb;
    integrator = integrator + error_sig * dt;
    phi_ext_deg = Kp * error_sig + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % Plant v2
    [V_out_next, ~, ~, P] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                             Phi_int_deg, Load, dt, params);
    
    V_out = V_out_next;
    
    results.V_out(k) = V_out;
    results.I_out(k) = I_fb;
    results.P(k) = P;
    results.phi(k) = phi_ext_deg;
    
    if mod(k, floor(n_steps/10)) == 0
        fprintf('  %d%%', round(k/n_steps*100));
    end
end

elapsed = toc;
fprintf('\n완료! (%.2f sec)\n\n', elapsed);

%% Analysis
ss_idx = floor(0.9 * n_steps);  % 마지막 10%
I_ss = mean(results.I_out(ss_idx:end));
V_ss = mean(results.V_out(ss_idx:end));
phi_ss = mean(results.phi(ss_idx:end));
P_ss = mean(results.P(ss_idx:end));

fprintf('=================================================================\n');
fprintf('  정상상태 분석\n');
fprintf('=================================================================\n');
fprintf('  V_out:    %.2f V\n', V_ss);
fprintf('  I_out:    %.2f A (목표: %.1f A, 오차: %.1f%%)\n', I_ss, I_ref, abs(I_ss-I_ref)/I_ref*100);
fprintf('  phi_ext:  %.2f deg (%.3f rad)\n', phi_ss, phi_ss*pi/180);
fprintf('  P:        %.0f W\n', P_ss);
fprintf('=================================================================\n\n');

%% Plot
figure('Name', 'Steady State Analysis', 'Position', [100 100 1400 800]);

subplot(2,2,1);
plot(results.time*1000, results.V_out, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title(sprintf('출력 전압 (정상상태: %.1f V)', V_ss));

subplot(2,2,2);
plot(results.time*1000, results.I_out, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('출력 전류 (정상상태: %.2f A)', I_ss));
legend('실제', '목표', 'Location', 'best');

subplot(2,2,3);
plot(results.time*1000, results.P/1000, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('Power (kW)');
title(sprintf('전력 (정상상태: %.0f W)', P_ss));

subplot(2,2,4);
plot(results.time*1000, results.phi, 'm-', 'LineWidth', 1.5);
hold on;
yline(phi_approx * 180/pi, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (ms)');
ylabel('\phi_{ext} (deg)');
title(sprintf('위상 (정상상태: %.1f deg)', phi_ss));
legend('실제', '이론값', 'Location', 'best');

fprintf('분석 완료.\n');
