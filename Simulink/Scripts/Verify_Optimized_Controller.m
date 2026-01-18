% =========================================================================
% Verify_Optimized_Controller.m
% 최적화된 제어기 성능 검증 스크립트
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Optimized Controller Verification\n');
fprintf('=================================================================\n\n');

%% Plant Parameters
params.n = 4.0;
params.L = 50e-6;
params.fs = 80e3;
params.C_out = 1000e-6;
params.efficiency = 0.95;

%% Optimized Controller Parameters (Auto-Updated)
Kp = 0.100;  % 최적화 결과
Ki = 5.000;  % 최적화 결과
Phi_int_deg = 15;

fprintf('제어기 파라미터:\n');
fprintf('  Kp = %.3f\n', Kp);
fprintf('  Ki = %.3f\n\n', Ki);

%% Test Scenario
V_in = 400;
V_out_init = 100;
Load = 20;
I_ref = 10;

%% Simulation Setup
dt = 10e-6;
sim_time = 0.03; % 30ms (더 긴 시간으로 정착 확인)
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

%% Load TPS Table
if exist('TPS_Table.mat', 'file')
    load('TPS_Table.mat', 'TPS_Table');
    has_tps = true;
else
    has_tps = false;
end

%% State Variables
V_out = V_out_init;
integrator = 0;

%% Results Storage
results.time = time_vec;
results.V_out = zeros(n_steps, 1);
results.I_out = zeros(n_steps, 1);
results.I_in = zeros(n_steps, 1);
results.D1 = zeros(n_steps, 1);
results.phi_ext = zeros(n_steps, 1);

fprintf('시뮬레이션 실행 중 (총 %d 스텝)...\n', n_steps);
tic;

%% Main Loop
for k = 1:n_steps
    I_fb = V_out / Load;
    error = I_ref - I_fb;
    integrator = integrator + error * dt;
    phi_ext_deg = Kp * error + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    if has_tps
        P_ref = V_out * I_ref;
        V1_min = min(TPS_Table.V1_vec);
        V1_max = max(TPS_Table.V1_vec);
        P_min = min(TPS_Table.P_vec);
        P_max = max(TPS_Table.P_vec);
        V_in_clip = max(V1_min, min(V1_max, V_in));
        P_ref_clip = max(P_min, min(P_max, P_ref));
        D1 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D1, V_in_clip, P_ref_clip, 'linear', 0);
        D2 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D2, V_in_clip, P_ref_clip, 'linear', 0);
        D3 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D3, V_in_clip, P_ref_clip, 'linear', 0);
    else
        D1 = phi_ext_deg / 360;
        D2 = Phi_int_deg / 360;
        D3 = 0;
    end
    
    [V_out_next, I_in_avg, ~] = DAB_Plant_TPS(V_in, V_out, D1, D2, D3, Load, dt, params);
    V_out = V_out_next;
    
    results.V_out(k) = V_out;
    results.I_out(k) = I_fb;
    results.I_in(k) = I_in_avg;
    results.D1(k) = D1;
    results.phi_ext(k) = phi_ext_deg;
    
    if mod(k, floor(n_steps/10)) == 0
        fprintf('  %d%%', round(k/n_steps*100));
    end
end

elapsed = toc;
fprintf('\n시뮬레이션 완료! (%.2f초)\n\n', elapsed);

%% Performance Analysis
ss_idx = floor(0.7 * n_steps);
V_out_ss = mean(results.V_out(ss_idx:end));
I_out_ss = mean(results.I_out(ss_idx:end));

V_out_peak = max(results.V_out);
I_out_peak = max(results.I_out);

overshoot_V = (V_out_peak - V_out_ss) / V_out_ss * 100;
overshoot_I = (I_out_peak - I_out_ss) / I_out_ss * 100;

ss_error = abs(I_out_ss - I_ref) / I_ref * 100;

settling_threshold = 0.02 * I_ref;
for k = ss_idx:n_steps
    if all(abs(results.I_out(k:end) - I_out_ss) < settling_threshold)
        settling_time = time_vec(k);
        break;
    end
end
if ~exist('settling_time', 'var')
    settling_time = sim_time;
end

fprintf('=================================================================\n');
fprintf('  성능 지표 (최적화 후)\n');
fprintf('=================================================================\n');
fprintf('정상상태:\n');
fprintf('  V_out:   %.2f V\n', V_out_ss);
fprintf('  I_out:   %.2f A (목표: %.2f A)\n', I_out_ss, I_ref);
fprintf('  I_in:    %.2f A\n', mean(results.I_in(ss_idx:end)));
fprintf('\n과도응답:\n');
fprintf('  전압 오버슈트:  %.2f %% ', overshoot_V);
if overshoot_V < 5
    fprintf('✅\n');
else
    fprintf('❌ (목표: < 5%%)\n');
end
fprintf('  전류 오버슈트:  %.2f %% ', overshoot_I);
if overshoot_I < 5
    fprintf('✅\n');
else
    fprintf('❌ (목표: < 5%%)\n');
end
fprintf('  정상상태 오차:  %.2f %% ', ss_error);
if ss_error < 5
    fprintf('✅\n');
else
    fprintf('❌ (목표: < 5%%)\n');
end
fprintf('  정착시간 (2%%): %.4f sec ', settling_time);
if settling_time < 0.03
    fprintf('✅\n');
else
    fprintf('❌ (목표: < 0.03 sec)\n');
end
fprintf('=================================================================\n\n');

%% 최적화 전후 비교
fprintf('=================================================================\n');
fprintf('  최적화 전후 비교\n');
fprintf('=================================================================\n');
fprintf('제어기 파라미터:\n');
fprintf('  Kp:  0.500 → %.3f (%.0f%% 변화)\n', Kp, (Kp-0.5)/0.5*100);
fprintf('  Ki:  10.000 → %.3f (%.0f%% 변화)\n', Ki, (Ki-10)/10*100);
fprintf('\n성능 개선:\n');
fprintf('  오버슈트:   9.33%% → %.2f%% (%.1f%% p 개선)\n', overshoot_I, 9.33-overshoot_I);
fprintf('  정상오차:   12.9%% → %.2f%% (%.1f%% p 개선)\n', ss_error, 12.9-ss_error);
fprintf('  정착시간:   17.6ms → %.1fms\n', settling_time*1000);
fprintf('=================================================================\n\n');

%% Plotting
figure('Name', 'Optimized Controller Performance', 'Position', [100 100 1400 800]);

subplot(2,2,1);
plot(results.time*1000, results.V_out, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('시간 (ms)');
ylabel('출력 전압 (V)');
title(sprintf('출력 전압 (오버슈트: %.2f%%)', overshoot_V));

subplot(2,2,2);
plot(results.time*1000, results.I_out, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'k--', 'LineWidth', 1.5);
yline(I_out_ss + settling_threshold, 'g--', 'LineWidth', 0.8);
yline(I_out_ss - settling_threshold, 'g--', 'LineWidth', 0.8);
grid on;
xlabel('시간 (ms)');
ylabel('출력 전류 (A)');
title(sprintf('출력 전류 (오버슈트: %.2f%%, 정상오차: %.2f%%)', overshoot_I, ss_error));
legend('실제', '목표', '±2% 범위', '', 'Location', 'best');

subplot(2,2,3);
plot(results.time*1000, results.I_in, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('시간 (ms)');
ylabel('입력 전류 (A)');
title('입력 전류');

subplot(2,2,4);
plot(results.time*1000, results.phi_ext, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('시간 (ms)');
ylabel('위상 (deg)');
title('외부 위상 제어 (PI 출력)');

fprintf('플롯 생성 완료!\n');
fprintf('\n검증 완료.\n');
