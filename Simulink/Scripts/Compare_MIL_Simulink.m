% =========================================================================
% Compare_MIL_Simulink.m
% MIL 결과와 Simulink(Control_Logic) 결과 비교
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  MIL vs Simulink Comparison (Control_Logic.slx)\n');
fprintf('=================================================================\n\n');

%% Parameters (run_simulation_v2.m과 동일)
V_pri_DC = 400; 
V_batt_nom = 100; 
P_rated = 5000;
f_sw = 80e3; 
Ts = 1/f_sw; 
N_pri = 4; 
N_sec = 1; 
L_leakage = 50e-6;
C_dc_link = 500e-6; 
R_wire = 1e-3; 
Ron_mosfet = 5e-3;
R_snubber = 1e5; 
C_snubber = inf;
I_ess_cmd = 10;  % 테스트용 전류 명령
kp_voltage = 5; 
ki_voltage = 5000;

%% MIL Parameters
params.n = N_pri / N_sec;
params.L = L_leakage;
params.fs = f_sw;
params.C_out = C_dc_link;
params.efficiency = 0.95;

%% Load TPS Table
if ~exist('TPS_Table.mat', 'file')
    error('TPS_Table.mat missing. Run TPS_LUT.m first.');
end
load('TPS_Table.mat', 'TPS_Table');

%% Simulation Settings
SIM_TIME = 0.020; % 20ms
dt = 10e-6;

%% === STEP 1: Run Simulink (Control_Logic) ===
fprintf('STEP 1: Running Simulink (Control_Logic.slx)...\n');

try
    % Workspace로 파라미터 전송
    assignin('base', 'V_pri_DC', V_pri_DC);
    assignin('base', 'V_batt_nom', V_batt_nom);
    assignin('base', 'P_rated', P_rated);
    assignin('base', 'f_sw', f_sw);
    assignin('base', 'Ts', Ts);
    assignin('base', 'N_pri', N_pri);
    assignin('base', 'N_sec', N_sec);
    assignin('base', 'L_leakage', L_leakage);
    assignin('base', 'C_dc_link', C_dc_link);
    assignin('base', 'R_wire', R_wire);
    assignin('base', 'Ron_mosfet', Ron_mosfet);
    assignin('base', 'R_snubber', R_snubber);
    assignin('base', 'C_snubber', C_snubber);
    assignin('base', 'I_ess_cmd', I_ess_cmd);
    assignin('base', 'kp_voltage', kp_voltage);
    assignin('base', 'ki_voltage', ki_voltage);
    assignin('base', 'TPS_Table', TPS_Table);
    
    % Load and run
    model_path = 'Modules/Control_Logic';
    load_system(model_path);
    
    sim_start = tic;
    simOut = sim(model_path, 'StopTime', num2str(SIM_TIME));
    sim_elapsed = toc(sim_start);
    
    fprintf('  ✓ Simulink 완료 (%.2f sec 소요)\n', sim_elapsed);
    
    % 결과 추출 시도
    if exist('simOut.logsout', 'var') || isprop(simOut, 'logsout')
        fprintf('  - logsout 데이터 발견\n');
        simulink_ok = true;
    elseif exist('simOut.yout', 'var') || isprop(simOut, 'yout')
        fprintf('  - yout 데이터 발견\n');
        simulink_ok = true;
    else
        fprintf('  ! 경고: 출력 데이터 추출 불가\n');
        simulink_ok = false;
    end
    
catch ME
    fprintf('  ✗ Simulink 실행 실패: %s\n', ME.message);
    fprintf('    (Control_Logic.slx 모델 구조 확인 필요)\n');
    simulink_ok = false;
    sim_elapsed = 0;
end

%% === STEP 2: Run MIL ===
fprintf('\nSTEP 2: Running MIL (Virtual Plant)...\n');

time_vec = 0:dt:SIM_TIME;
n_steps = length(time_vec);

% Controller Parameters (DAB_Controller_System.m에서 로드)
Kp = 0.5;   % 기본값
Ki = 10;    % 기본값

% 초기 조건
V_out = V_batt_nom;
V_in = V_pri_DC;
integrator = 0;
Load = V_batt_nom / I_ess_cmd; % I = V/R 역산

% 결과 저장
mil_results.time = time_vec;
mil_results.V_out = zeros(n_steps, 1);
mil_results.I_out = zeros(n_steps, 1);
mil_results.D1 = zeros(n_steps, 1);

mil_start = tic;

for k = 1:n_steps
    I_fb = V_out / Load;
    error_sig = I_ess_cmd - I_fb;
    integrator = integrator + error_sig * dt;
    phi_ext_deg = Kp * error_sig + Ki * integrator;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % TPS Look-Up
    P_ref = V_out * I_ess_cmd;
    V1_min = min(TPS_Table.V1_vec);
    V1_max = max(TPS_Table.V1_vec);
    P_min = min(TPS_Table.P_vec);
    P_max = max(TPS_Table.P_vec);
    V_in_clip = max(V1_min, min(V1_max, V_in));
    P_ref_clip = max(P_min, min(P_max, P_ref));
    D1 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D1, V_in_clip, P_ref_clip, 'linear', 0);
    D2 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D2, V_in_clip, P_ref_clip, 'linear', 0);
    D3 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D3, V_in_clip, P_ref_clip, 'linear', 0);
    
    [V_out_next, ~, ~] = DAB_Plant_TPS(V_in, V_out, D1, D2, D3, Load, dt, params);
    V_out = V_out_next;
    
    mil_results.V_out(k) = V_out;
    mil_results.I_out(k) = I_fb;
    mil_results.D1(k) = D1;
end

mil_elapsed = toc(mil_start);
fprintf('  ✓ MIL 완료 (%.3f sec 소요)\n', mil_elapsed);

if sim_elapsed > 0
    fprintf('  → 속도 비교: MIL이 %.0fx 빠름\n', sim_elapsed / mil_elapsed);
end

%% === STEP 3: Comparison ===
fprintf('\nSTEP 3: Comparison...\n');

% MIL 성능 지표
ss_idx = floor(0.7 * n_steps);
I_ss = mean(mil_results.I_out(ss_idx:end));
I_peak = max(mil_results.I_out);
overshoot_mil = max(0, (I_peak - I_ess_cmd) / I_ess_cmd * 100);
ss_error_mil = abs(I_ss - I_ess_cmd) / I_ess_cmd * 100;

fprintf('\n[MIL 결과]\n');
fprintf('  - 정상상태 전류: %.2f A (목표: %.2f A)\n', I_ss, I_ess_cmd);
fprintf('  - 오버슈트: %.2f %%\n', overshoot_mil);
fprintf('  - 정상상태 오차: %.2f %%\n', ss_error_mil);

%% === STEP 4: Plot ===
fprintf('\nSTEP 4: Plotting...\n');

figure('Name', 'MIL vs Simulink', 'Position', [100 100 1200 600]);

subplot(2,1,1);
plot(mil_results.time*1000, mil_results.V_out, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title('출력 전압 (MIL)');

subplot(2,1,2);
plot(mil_results.time*1000, mil_results.I_out, 'r-', 'LineWidth', 1.5);
hold on;
yline(I_ess_cmd, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('출력 전류 (오버슈트: %.2f%%, 정상오차: %.2f%%)', overshoot_mil, ss_error_mil));
legend('MIL', '목표', 'Location', 'best');

fprintf('플롯 완료!\n');
fprintf('\n비교 스크립트 종료.\n');
