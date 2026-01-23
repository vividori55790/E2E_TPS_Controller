% =========================================================================
% Quick_MIL_Test.m
% 간단한 MIL 검증 스크립트 (Simulink 없이 단독 실행)
% 중앙 설정 파일 (DAB_System_Config.m) 사용
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Quick MIL Test (TPS DAB Virtual Plant)\n');
fprintf('=================================================================\n\n');

%% ==================== Load Configuration ====================
config = DAB_System_Config();
fprintf('설정 파일 로드 완료: DAB_System_Config.m\n');

%% ==================== Extract Parameters from Config ====================
% Plant Parameters
params.n = config.plant.n;
params.L = config.plant.L;
params.fs = config.plant.fs;
params.C_out = config.plant.C_out;
params.efficiency = config.plant.efficiency;

% Controller Parameters
Kp = config.controller.Kp;
Ki = config.controller.Ki;
Phi_int_deg = config.controller.Phi_int_deg;

% Test Scenario
V_in = config.hv.V_nom;
V_batt = config.battery.V_nom;
R_batt = config.battery.R_internal;
V_out_init = V_batt;
I_ref = config.test.I_ref;

% Simulation Setup
dt = config.sim.dt;
sim_time = 0.030;  % 30ms (Quick Test)
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

fprintf('시뮬레이션 설정:\n');
fprintf('  - 시간: %.3f sec\n', sim_time);
fprintf('  - 타임스텝: %.1f us\n', dt*1e6);
fprintf('  - 총 스텝: %d\n', n_steps);
fprintf('  - Kp=%.3f, Ki=%.1f\n\n', Kp, Ki);

%% ==================== Load TPS Table ====================
if isfile('TPS_Table.mat')
    load('TPS_Table.mat', 'TPS_Table');
    fprintf('TPS Look-Up Table 로드 완료\n\n');
    has_tps = true;
elseif isfile('../TPS_Table.mat')
    load('../TPS_Table.mat', 'TPS_Table');
    fprintf('TPS Look-Up Table 로드 완료 (상위 폴더)\n\n');
    has_tps = true;
else
    fprintf('TPS Look-Up Table 없음 (SPS 모드로 실행)\n\n');
    has_tps = false;
end

%% ==================== State Variables ====================
V_out = V_out_init;
integrator = 0;

%% ==================== Results Storage ====================
results.time = time_vec;
results.V_out = zeros(n_steps, 1);
results.I_out = zeros(n_steps, 1);
results.I_in = zeros(n_steps, 1);
results.D1 = zeros(n_steps, 1);
results.D2 = zeros(n_steps, 1);
results.D3 = zeros(n_steps, 1);
results.phi_ext = zeros(n_steps, 1);
results.error = zeros(n_steps, 1);

fprintf('시뮬레이션 실행 중...\n');
tic;

%% ==================== Main Loop ====================
for k = 1:n_steps
    % Current Feedback (배터리 모델)
    I_fb = (V_out - V_batt) / R_batt;
    
    % PI Controller
    error = I_ref - I_fb;
    integrator = integrator + error * dt;
    phi_ext_deg = Kp * error + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % TPS Control
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
    
    % Virtual Plant (SPS v2)
    [V_out_next, I_in_avg, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                                    Phi_int_deg, R_batt, V_batt, dt, params);
    
    % Update State
    V_out = V_out_next;
    
    % Save Results
    results.V_out(k) = V_out;
    results.I_out(k) = I_fb;
    results.I_in(k) = I_in_avg;
    results.D1(k) = D1;
    results.D2(k) = D2;
    results.D3(k) = D3;
    results.phi_ext(k) = phi_ext_deg;
    results.error(k) = error;
    
    % Progress
    if mod(k, floor(n_steps/10)) == 0
        fprintf('  진행률: %d%%\n', round(k/n_steps*100));
    end
end

elapsed = toc;
fprintf('\n시뮬레이션 완료!\n');
fprintf('  소요시간: %.3f 초\n', elapsed);
fprintf('  실시간 비율: %.1fx (%.3f sec 시뮬 / %.3f sec 실행)\n\n', sim_time/elapsed, sim_time, elapsed);

%% ==================== Performance Analysis ====================
fprintf('=================================================================\n');
fprintf('  성능 분석\n');
fprintf('=================================================================\n');

ss_idx = floor(0.8 * n_steps);
V_out_ss = mean(results.V_out(ss_idx:end));
I_out_ss = mean(results.I_out(ss_idx:end));
I_in_ss = mean(results.I_in(ss_idx:end));

V_out_peak = max(results.V_out);
I_out_peak = max(results.I_out);

overshoot_V = (V_out_peak - V_out_ss) / V_out_ss * 100;
overshoot_I = (I_out_peak - I_out_ss) / I_out_ss * 100;
ss_error = abs(I_out_ss - I_ref) / I_ref * 100;

settling_threshold_I = 0.02 * I_out_ss;
settling_idx = find(abs(results.I_out(ss_idx:end) - I_out_ss) < settling_threshold_I, 1);
if ~isempty(settling_idx)
    settling_time = time_vec(ss_idx + settling_idx - 1);
else
    settling_time = inf;
end

fprintf('정상상태 값:\n');
fprintf('  V_out: %.2f V\n', V_out_ss);
fprintf('  I_out: %.2f A (목표: %.2f A, 오차: %.2f%%)\n', I_out_ss, I_ref, ss_error);
fprintf('  I_in:  %.2f A\n', I_in_ss);
fprintf('\n과도응답:\n');
fprintf('  전압 오버슈트: %.2f%%\n', overshoot_V);
fprintf('  전류 오버슈트: %.2f%%\n', overshoot_I);
fprintf('  정착시간 (2%%): %.4f sec\n', settling_time);
fprintf('=================================================================\n\n');

%% ==================== Plotting ====================
fprintf('결과 플롯 생성 중...\n');

figure('Name', 'Quick MIL Test Results', 'Position', [100 100 1400 900]);

subplot(3,2,1);
plot(results.time*1000, results.V_out, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title('출력 전압');

subplot(3,2,2);
plot(results.time*1000, results.I_out, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title('출력 전류');
legend('실제', '목표', 'Location', 'best');

subplot(3,2,3);
plot(results.time*1000, results.I_in, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{in} (A)');
title('입력 전류');

subplot(3,2,4);
plot(results.time*1000, results.phi_ext, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('\phi_{ext} (deg)');
title('외부 위상 (PI 출력)');

subplot(3,2,5);
plot(results.time*1000, results.D1, 'r-', 'LineWidth', 1.5);
hold on;
plot(results.time*1000, results.D2, 'g-', 'LineWidth', 1.5);
plot(results.time*1000, results.D3, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('Duty (pu)');
title('TPS Duty Ratios');
legend('D1', 'D2', 'D3', 'Location', 'best');

subplot(3,2,6);
plot(results.time*1000, results.error, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('Error (A)');
title('제어 오차');

fprintf('플롯 완료!\n');
fprintf('\nQuick MIL Test 종료.\n');
