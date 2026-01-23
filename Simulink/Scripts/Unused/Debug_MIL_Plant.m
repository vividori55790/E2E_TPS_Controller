% =========================================================================
% Debug_MIL_Plant.m
% MIL 플랜트 모델 디버깅 및 분석
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  MIL Plant Model Debugging\n');
fprintf('=================================================================\n\n');

%% Parameters
params.n = 4.0;
params.L = 50e-6;
params.fs = 80e3;
params.C_out = 1000e-6;
params.efficiency = 0.95;

V_in = 400;
V_out_nom = 100;
Load = 20;  % 5A at 100V

%% Test 1: Open-Loop Power Calculation
fprintf('=== Test 1: Open-Loop Power vs Phase Shift ===\n');

phi_range = linspace(-0.25, 0.25, 51);  % D1: -0.25 ~ 0.25 (±90 deg)
P_calc = zeros(size(phi_range));

for i = 1:length(phi_range)
    D1 = phi_range(i);
    D2 = 0;  % No inner shift
    D3 = 0;
    
    % 전력 계산
    [~, I_in, ~] = DAB_Plant_TPS(V_in, V_out_nom, D1, D2, D3, Load, 1e-6, params);
    P_calc(i) = V_in * I_in;  % 입력 전력
end

figure('Name', 'Power vs Phase Shift', 'Position', [100 100 800 400]);
plot(phi_range*360, P_calc, 'b-', 'LineWidth', 2);
grid on;
xlabel('Phase Shift D1 (degrees)');
ylabel('Power (W)');
title('전력 전달 vs 위상 변동 (Open-Loop)');

fprintf('  max P = %.0f W at D1 = %.3f\n', max(abs(P_calc)), phi_range(find(abs(P_calc)==max(abs(P_calc)), 1)));
fprintf('  P at D1=0.1 (36 deg): %.0f W\n', P_calc(find(phi_range>=0.1, 1)));

%% Test 2: Voltage Dynamics
fprintf('\n=== Test 2: Voltage Dynamics (Fixed D1) ===\n');

dt = 10e-6;
sim_time = 0.020;
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

D1_fixed = 0.05;  % 18 deg
D2_fixed = 0;
D3_fixed = 0;

V_out = V_out_nom;
V_history = zeros(n_steps, 1);
I_history = zeros(n_steps, 1);
P_history = zeros(n_steps, 1);

for k = 1:n_steps
    [V_out_next, I_in, ~] = DAB_Plant_TPS(V_in, V_out, D1_fixed, D2_fixed, D3_fixed, Load, dt, params);
    
    V_history(k) = V_out;
    I_history(k) = V_out / Load;
    P_history(k) = V_in * I_in;
    
    V_out = V_out_next;
end

figure('Name', 'Voltage Dynamics', 'Position', [150 150 1200 600]);

subplot(2,2,1);
plot(time_vec*1000, V_history, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title(sprintf('출력 전압 (D1 = %.3f)', D1_fixed));

subplot(2,2,2);
plot(time_vec*1000, I_history, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title('출력 전류');

subplot(2,2,3);
plot(time_vec*1000, P_history, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('Power (W)');
title('전력 전달');

% V_out 변화율 분석
dV_dt = diff(V_history) / dt;
subplot(2,2,4);
plot(time_vec(1:end-1)*1000, dV_dt, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('dV/dt (V/s)');
title('전압 변화율');

fprintf('  V_out 최종값: %.2f V\n', V_history(end));
fprintf('  I_out 최종값: %.2f A\n', I_history(end));
fprintf('  평균 전력: %.0f W\n', mean(P_history));

%% Test 3: Closed-Loop with SPS Control (DAB_Controller_System 방식)
fprintf('\n=== Test 3: Closed-Loop SPS Control ===\n');

% DAB_Controller_System.m 방식 적용
% phi_ext_deg를 직접 D1으로 변환 (t_ext 계산)

Kp = 0.5;
Ki = 10;
I_ref = 5;  % 목표 전류

V_out = V_out_nom;
integrator = 0;

V_history2 = zeros(n_steps, 1);
I_history2 = zeros(n_steps, 1);
phi_history = zeros(n_steps, 1);

for k = 1:n_steps
    I_fb = V_out / Load;
    
    % PI Controller (DAB_Controller_System 방식)
    error_sig = I_ref - I_fb;
    integrator = integrator + error_sig * dt;
    phi_ext_deg = Kp * error_sig + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % Phase Shift 변환 (DAB_Controller_System 방식)
    % t_ext = abs(phi_ext_deg) * deg2time
    % D1 = phi_ext_deg / 360
    Phi_int_deg = 15;  % 내부 위상
    
    D1 = phi_ext_deg / 360;  % 외부 위상을 D1으로 변환
    D2 = Phi_int_deg / 360;  % Primary inner
    D3 = 0;                   % Secondary inner
    
    [V_out_next, ~, ~] = DAB_Plant_TPS(V_in, V_out, D1, D2, D3, Load, dt, params);
    
    V_history2(k) = V_out;
    I_history2(k) = I_fb;
    phi_history(k) = phi_ext_deg;
    
    V_out = V_out_next;
end

figure('Name', 'Closed-Loop SPS', 'Position', [200 200 1200 600]);

subplot(2,2,1);
plot(time_vec*1000, V_history2, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title('출력 전압 (Closed-Loop SPS)');

subplot(2,2,2);
plot(time_vec*1000, I_history2, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('출력 전류 (목표: %.1f A)', I_ref));
legend('실제', '목표', 'Location', 'best');

subplot(2,2,3);
plot(time_vec*1000, phi_history, 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('\phi_{ext} (deg)');
title('외부 위상');

subplot(2,2,4);
plot(time_vec*1000, phi_history/360, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('D1 (pu)');
title('D1 (위상 / 360)');

% 성능 지표
ss_idx = floor(0.7 * n_steps);
I_ss = mean(I_history2(ss_idx:end));
ss_error = abs(I_ss - I_ref) / I_ref * 100;

fprintf('  정상상태 전류: %.2f A (목표: %.1f A)\n', I_ss, I_ref);
fprintf('  정상상태 오차: %.2f %%\n', ss_error);

%% Test 4: 다양한 I_ref에서의 성능
fprintf('\n=== Test 4: Performance vs I_ref ===\n');

I_ref_range = [2, 5, 8, 10, 12];
results = struct();

for idx = 1:length(I_ref_range)
    I_ref_test = I_ref_range(idx);
    
    V_out = V_out_nom;
    integrator = 0;
    I_hist = zeros(n_steps, 1);
    
    for k = 1:n_steps
        I_fb = V_out / Load;
        error_sig = I_ref_test - I_fb;
        integrator = integrator + error_sig * dt;
        phi_ext_deg = Kp * error_sig + Ki * integrator;
        phi_ext_deg = max(-90, min(90, phi_ext_deg));
        
        D1 = phi_ext_deg / 360;
        D2 = 15 / 360;
        D3 = 0;
        
        [V_out_next, ~, ~] = DAB_Plant_TPS(V_in, V_out, D1, D2, D3, Load, dt, params);
        V_out = V_out_next;
        I_hist(k) = I_fb;
    end
    
    I_ss = mean(I_hist(ss_idx:end));
    ss_error = abs(I_ss - I_ref_test) / I_ref_test * 100;
    
    fprintf('  I_ref = %2d A → I_ss = %.2f A, Error = %.1f%%\n', ...
        I_ref_test, I_ss, ss_error);
end

fprintf('\n디버깅 완료.\n');
