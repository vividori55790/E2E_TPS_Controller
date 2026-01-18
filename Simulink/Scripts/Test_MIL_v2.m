% =========================================================================
% Test_MIL_v2.m
% 개선된 MIL 플랜트 모델 (v2) 테스트
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  MIL Plant v2 Test (SPS Formula)\n');
fprintf('=================================================================\n\n');

%% Parameters (run_simulation_v2.m과 동일)
params.n = 4.0;             % 변압기 권선비
params.L = 50e-6;           % 누설 인덕턴스 (H)
params.fs = 80e3;           % 스위칭 주파수 (Hz)
params.C_out = 500e-6;      % 출력 커패시터 (F)
params.efficiency = 0.95;   % 효율

V_in = 400;                 % 1차측(HV) 전압
V_out_init = 100;           % 초기 2차측(LV) 전압
Load = 10;                  % 부하 저항 (10 ohm -> 100V에서 10A)

Phi_int_deg = 15;           % 내부 위상 (고정)

%% Controller Parameters
Kp = 0.5;
Ki = 10;
I_ref = 10;                 % 목표 전류 (A)

%% Simulation Setup
dt = 10e-6;                 % 타임스텝
sim_time = 0.050;           % 50ms
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

fprintf('시뮬레이션 설정:\n');
fprintf('  - 시간: %.0f ms\n', sim_time*1000);
fprintf('  - 목표 전류: %.1f A\n', I_ref);
fprintf('  - 예상 부하전력: %.0f W\n\n', V_out_init * I_ref);

%% === Test 1: Open-Loop Power Characteristic ===
fprintf('=== Test 1: Open-Loop Power Characteristic ===\n');

phi_range = linspace(-60, 60, 121);  % -60 ~ +60 degrees
P_calc = zeros(size(phi_range));

for i = 1:length(phi_range)
    [~, ~, ~, P] = DAB_Plant_TPS_v2(V_in, V_out_init, phi_range(i), ...
                                    Phi_int_deg, Load, dt, params);
    P_calc(i) = P;
end

figure('Name', 'Power vs Phase Shift', 'Position', [100 100 800 400]);
plot(phi_range, P_calc, 'b-', 'LineWidth', 2);
grid on;
xlabel('Phase Shift \phi_{ext} (degrees)');
ylabel('Power (W)');
title('전력 전달 vs 위상 변동 (SPS 공식)');

[P_max, idx_max] = max(P_calc);
fprintf('  최대 전력: %.0f W at phi = %.1f deg\n', P_max, phi_range(idx_max));
fprintf('  phi=30 deg에서 전력: %.0f W\n', P_calc(find(phi_range>=30, 1)));

%% === Test 2: Closed-Loop Step Response ===
fprintf('\n=== Test 2: Closed-Loop Step Response ===\n');

V_out = V_out_init;
integrator = 0;

results.time = time_vec;
results.V_out = zeros(n_steps, 1);
results.I_out = zeros(n_steps, 1);
results.P = zeros(n_steps, 1);
results.phi = zeros(n_steps, 1);
results.error = zeros(n_steps, 1);

fprintf('시뮬레이션 실행 중...\n');
tic;

for k = 1:n_steps
    % Current feedback
    I_fb = V_out / Load;
    
    % Step input at t = 5ms
    if time_vec(k) < 0.005
        I_ref_now = 0;  % 초기 0A
    else
        I_ref_now = I_ref;  % 스텝: 10A
    end
    
    % PI Controller
    error_sig = I_ref_now - I_fb;
    integrator = integrator + error_sig * dt;
    phi_ext_deg = Kp * error_sig + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % Plant v2
    [V_out_next, I_in, ~, P] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                                 Phi_int_deg, Load, dt, params);
    
    % Update state
    V_out = V_out_next;
    
    % Save
    results.V_out(k) = V_out;
    results.I_out(k) = I_fb;
    results.P(k) = P;
    results.phi(k) = phi_ext_deg;
    results.error(k) = error_sig;
    
    if mod(k, floor(n_steps/10)) == 0
        fprintf('  %d%%', round(k/n_steps*100));
    end
end

elapsed = toc;
fprintf('\n시뮬레이션 완료! (%.2f sec)\n\n', elapsed);

%% Performance Analysis
step_idx = find(time_vec >= 0.005, 1);
ss_idx = floor(0.8 * n_steps);

I_ss = mean(results.I_out(ss_idx:end));
V_ss = mean(results.V_out(ss_idx:end));
I_peak = max(results.I_out(step_idx:end));

overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
ss_error = abs(I_ss - I_ref) / I_ref * 100;

% Settling time (2%)
settling_time = sim_time;
for sk = step_idx:n_steps
    if all(abs(results.I_out(sk:end) - I_ref) < 0.02*I_ref)
        settling_time = time_vec(sk) - time_vec(step_idx);
        break;
    end
end

fprintf('=================================================================\n');
fprintf('  성능 지표\n');
fprintf('=================================================================\n');
fprintf('정상상태:\n');
fprintf('  V_out: %.2f V\n', V_ss);
fprintf('  I_out: %.2f A (목표: %.1f A)\n', I_ss, I_ref);
fprintf('  P:     %.0f W\n', mean(results.P(ss_idx:end)));
fprintf('\n과도응답:\n');
fprintf('  오버슈트:    %.2f %%', overshoot);
if overshoot < 5, fprintf(' ✓\n'); else, fprintf(' ✗ (목표 < 5%%)\n'); end
fprintf('  정상오차:    %.2f %%', ss_error);
if ss_error < 5, fprintf(' ✓\n'); else, fprintf(' ✗ (목표 < 5%%)\n'); end
fprintf('  정착시간:    %.3f sec', settling_time);
if settling_time < 0.03, fprintf(' ✓\n'); else, fprintf(' ✗ (목표 < 0.03s)\n'); end
fprintf('=================================================================\n\n');

%% Plotting
figure('Name', 'MIL v2 Step Response', 'Position', [150 100 1400 800]);

subplot(2,2,1);
plot(results.time*1000, results.V_out, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title('출력 전압');

subplot(2,2,2);
plot(results.time*1000, results.I_out, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'k--', 'LineWidth', 1.5);
yline(I_ref*1.05, 'g--', 'LineWidth', 0.8);
yline(I_ref*0.95, 'g--', 'LineWidth', 0.8);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('출력 전류 (오버슈트: %.1f%%, 정상오차: %.1f%%)', overshoot, ss_error));
legend('실제', '목표', '±5%', '', 'Location', 'best');

subplot(2,2,3);
plot(results.time*1000, results.P/1000, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('Power (kW)');
title('전력 전달');

subplot(2,2,4);
plot(results.time*1000, results.phi, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('\phi_{ext} (deg)');
title('외부 위상 (PI 출력)');

%% === Test 3: Parameter Sweep for Optimization ===
fprintf('=== Test 3: Quick Parameter Sweep ===\n');

Kp_test = [0.3, 0.5, 0.8, 1.0, 1.5];
Ki_test = [10, 20, 30, 40, 50];

best_cost = inf;
best_Kp = 0;
best_Ki = 0;

for Kp = Kp_test
    for Ki = Ki_test
        V_out = V_out_init;
        integrator = 0;
        I_hist = zeros(n_steps, 1);
        
        for k = 1:n_steps
            I_fb = V_out / Load;
            if time_vec(k) < 0.005
                I_ref_now = 0;
            else
                I_ref_now = I_ref;
            end
            
            error_sig = I_ref_now - I_fb;
            integrator = integrator + error_sig * dt;
            phi_ext_deg = Kp * error_sig + Ki * integrator;
            phi_ext_deg = max(-90, min(90, phi_ext_deg));
            
            [V_out_next, ~, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                                     Phi_int_deg, Load, dt, params);
            V_out = V_out_next;
            I_hist(k) = I_fb;
        end
        
        I_ss = mean(I_hist(ss_idx:end));
        I_peak = max(I_hist(step_idx:end));
        overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
        ss_error = abs(I_ss - I_ref) / I_ref * 100;
        
        cost = 5 * overshoot + 3 * ss_error;
        
        if cost < best_cost
            best_cost = cost;
            best_Kp = Kp;
            best_Ki = Ki;
        end
    end
end

fprintf('최적 파라미터: Kp=%.2f, Ki=%.1f (Cost=%.1f)\n', best_Kp, best_Ki, best_cost);

fprintf('\nMIL v2 테스트 완료.\n');
