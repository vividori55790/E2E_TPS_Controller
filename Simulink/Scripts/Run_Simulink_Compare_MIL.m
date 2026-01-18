% =========================================================================
% Run_Simulink_Compare_MIL.m
% Control_Logic.slx 실행 후 MIL 결과와 비교 (다이나믹 시나리오 적용)
% 성능 최적화: Config 캐싱, assignin 최적화, 루프 외부 최적화
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Simulink vs MIL Comparison (Dynamic Scenario - Optimized)\n');
fprintf('=================================================================\n\n');

%% ==================== Load Configuration (Cached) ====================
config = DAB_System_Config();  % Persistent 캐싱됨 (2번째 호출부터 ~0.001초)

%% ==================== Create Scenario ====================
fprintf('=== STEP 0: 시나리오 생성 ===\n');
run('Create_Scenario.m');
fprintf('시나리오 로드 완료 (%d steps, %.3f sec)\n', length(Ref_VH.Data), SIM_TIME);

%% ==================== Setup Parameters (Optimized) ====================
% 한 번에 구조체로 할당 (개별 assignin보다 빠름)
sim_params = struct();

% Plant Parameters
sim_params.V_pri_DC = config.hv.V_nom;
sim_params.V_batt_nom = config.battery.V_nom;
sim_params.P_rated = config.plant.P_rated;
sim_params.f_sw = config.plant.fs;
sim_params.Ts = 1/config.plant.fs;
sim_params.N_pri = config.plant.n;
sim_params.N_sec = 1;
sim_params.L_leakage = config.plant.L;
sim_params.C_dc_link = config.plant.C_out;
sim_params.R_wire = 1e-3;
sim_params.Ron_mosfet = 5e-3;
sim_params.R_snubber = 1e5;
sim_params.C_snubber = inf;
sim_params.I_ess_cmd = config.test.I_ref;
sim_params.kp_voltage = 5;
sim_params.ki_voltage = 5000;

% MIL Parameters
params.n = config.plant.n;
params.L = config.plant.L;
params.fs = config.plant.fs;
params.C_out = config.plant.C_out;
params.efficiency = config.plant.efficiency;

% Controller Parameters
Kp = config.controller.Kp;
Ki = config.controller.Ki;
Phi_int_deg = config.controller.Phi_int_deg;
V_HV_Threshold = config.controller.V_HV_Threshold;
V_HV_Hysteresis = config.controller.V_HV_Hysteresis;
Ups_Target_Current = config.controller.Ups_Target_Current;

R_batt = config.battery.R_internal;
V_batt_nom = config.battery.V_nom;

%% ==================== Load TPS Table ====================
persistent TPS_Table_cached;
if isempty(TPS_Table_cached)
    if isfile('TPS_Table.mat')
        load('TPS_Table.mat', 'TPS_Table');
        TPS_Table_cached = TPS_Table;
        fprintf('TPS Table 로드 완료\n');
    elseif isfile('../TPS_Table.mat')
        load('../TPS_Table.mat', 'TPS_Table');
        TPS_Table_cached = TPS_Table;
        fprintf('TPS Table 로드 완료 (상위 폴더)\n');
    else
        error('TPS_Table.mat을 찾을 수 없습니다.');
    end
else
    TPS_Table = TPS_Table_cached;
end

sim_params.TPS_Table = TPS_Table;

%% ==================== Conditional addpath (한 번만) ====================
persistent paths_added;
if isempty(paths_added)
    addpath(pwd);
    addpath('..');
    addpath('../Data');
    paths_added = true;
end

%% ==================== Assign to Base Workspace (한 번에) ====================
field_names = fieldnames(sim_params);
for i = 1:length(field_names)
    assignin('base', field_names{i}, sim_params.(field_names{i}));
end
fprintf('파라미터 Base Workspace에 할당 완료\n');

%% ==================== Run Simulink ====================
fprintf('\n=== STEP 1: Simulink 실행 ===\n');

original_dir = pwd;
cd('../Modules');

% Load HDC Model (Cached)
persistent HDC_Model_Struct;
if isempty(HDC_Model_Struct)
    if isfile('../HDC_Model.mat')
        load('../HDC_Model.mat');
        HDC_Model_Struct = struct('W',W, 'Level_Item_V',Level_Item_V, 'Level_Item_P',Level_Item_P, ...
                                   'min_V',min_V, 'max_V',max_V, 'min_P',min_P, 'max_P',max_P, 'L_levels',L_levels);
        fprintf('HDC Model loaded.\n');
    else
        HDC_Model_Struct = struct();  % 빈 구조체
        fprintf('Warning: HDC_Model.mat not found.\n');
    end
end

if ~isempty(fieldnames(HDC_Model_Struct))
    assignin('base', 'HDC_Model_Struct', HDC_Model_Struct);
end

try
    fprintf('모델 로드 중: Control_Logic.slx\n');
    load_system('Control_Logic');

    sys_blocks = find_system('Control_Logic', 'MatchFilter', @Simulink.match.internal.filterOutInactiveVariantSubsystemChoices, 'BlockType', 'MATLABSystem');
    if ~isempty(sys_blocks)
        blk = sys_blocks{1};
        try
            set_param(blk, 'System', 'DAB_Controller_System');
            if ~isempty(fieldnames(HDC_Model_Struct))
                set_param(blk, 'EnableHDC', 'true');
                set_param(blk, 'HDC_Model', 'HDC_Model_Struct');
                fprintf('Enabled HDC on block: %s\n', blk);
            end
        catch ME
            fprintf('Could not set HDC parameters: %s\n', ME.message);
        end
    end

    tic;
    out = sim('Control_Logic', 'StopTime', num2str(SIM_TIME), 'ReturnWorkspaceOutputs', 'on');
    sim_elapsed = toc;
    fprintf('✓ Simulink 완료! (%.2f sec 소요)\n\n', sim_elapsed);

    cd(original_dir);
    simulink_ok = true;

catch ME
    cd(original_dir);
    fprintf('✗ Simulink 오류: %s\n', ME.message);
    simulink_ok = false;
    sim_elapsed = 0;
end

%% ==================== Run MIL (Optimized) ====================
fprintf('\n=== STEP 2: MIL 실행 (Optimized Loop) ===\n');

dt_mil = config.sim.dt;
time_vec = 0:dt_mil:SIM_TIME;
n_steps = length(time_vec);

% Scenario Data
if exist('Ref_VH', 'var')
    V_high_data = Ref_VH.Data;
    I_cmd_data = Ref_I_ess_cmd.Data;
else
    V_high_data = sim_params.V_pri_DC * ones(n_steps, 1);
    I_cmd_data = config.test.I_ref * ones(n_steps, 1);
    fprintf('Warning: Scenario data not found, using constants.\n');
end

% Pre-allocate results
mil.time = time_vec;
mil.V_out = zeros(n_steps, 1);
mil.I_out = zeros(n_steps, 1);
mil.phi_ext = zeros(n_steps, 1);
mil.mode = zeros(n_steps, 1);
mil.V_in = zeros(n_steps, 1);
mil.I_ref = zeros(n_steps, 1);

% HDC 사용 여부 루프 밖에서 체크 (매 스텝마다 exist() 호출 방지)
use_hdc = ~isempty(fieldnames(HDC_Model_Struct));

% Initialize states
PowerLossState = 0;
V_out = V_batt_nom;
integrator = 0;

fprintf('MIL 시뮬레이션 실행 중 (Kp=%.3f, Ki=%.1f)...\n', Kp, Ki);
tic;

% Main loop (최적화됨)
for k = 1:n_steps
    % 1. Get inputs
    idx = min(round(time_vec(k) / dt_mil) + 1, length(V_high_data));
    V_in = V_high_data(idx);
    I_cmd_user = I_cmd_data(idx);

    % 2. Current Feedback
    I_fb = (V_out - V_batt_nom) / R_batt;

    % 3. Mode Selection
    if V_in < V_HV_Threshold
        PowerLossState = 1;
    elseif V_in > V_HV_Hysteresis
        PowerLossState = 0;
    end

    Iref = (PowerLossState == 1) * Ups_Target_Current + (PowerLossState == 0) * I_cmd_user;

    % 4. PI Control
    error_sig = Iref - I_fb;

    if PowerLossState == 1 && abs(error_sig) > 5
        integrator = 0;
    end

    integrator = integrator + error_sig * dt_mil;
    phi_pi = Kp * error_sig + Ki * integrator;

    % 5. HDC Feedforward (최적화: 루프 외부에서 체크)
    phi_hdc = 0;
    if use_hdc
        try
            P_target = V_out * Iref;
            [~, ~, d3_pred] = HDC_Predict(V_in, P_target, HDC_Model_Struct);
            phi_hdc = d3_pred * 360;
        catch
            phi_hdc = 0;
        end
    end

    phi_ext_deg = phi_pi + phi_hdc;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));

    % 6. Plant
    [V_out_next, ~, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                             Phi_int_deg, R_batt, V_batt_nom, dt_mil, params);
    V_out = V_out_next;

    % 7. Save results
    mil.V_out(k) = V_out;
    mil.I_out(k) = (V_out - V_batt_nom) / R_batt;
    mil.phi_ext(k) = phi_ext_deg;
    mil.mode(k) = PowerLossState;
    mil.V_in(k) = V_in;
    mil.I_ref(k) = Iref;
end

mil_elapsed = toc;
fprintf('✓ MIL 완료! (%.3f sec 소요)\n', mil_elapsed);

%% ==================== Plot Results ====================
fprintf('\n=== STEP 3: 결과 비교 및 플롯 ===\n');

figure('Name', 'Simulink vs MIL (Dynamic Test)', 'Position', [50 50 1200 800]);

subplot(3,1,1);
yyaxis left
plot(mil.time*1000, mil.V_in, 'k-', 'LineWidth', 1.5);
ylabel('Input Voltage (V)');
ylim([0 450]);
yyaxis right
plot(mil.time*1000, mil.mode, 'rd-', 'MarkerSize', 2, 'LineWidth', 1);
ylabel('Mode (0:ESS, 1:UPS)');
ylim([-0.5 1.5]);
grid on;
title('Scenario: Input Voltage & Mode Transition');
legend('V_{in}', 'Mode');

subplot(3,1,2);
plot(mil.time*1000, mil.I_out, 'b-', 'LineWidth', 1.5); hold on;
plot(mil.time*1000, mil.I_ref, 'r--', 'LineWidth', 2);
grid on;
ylabel('Output Current (A)');
title(sprintf('Output Current (MIL) - Kp=%.3f, Ki=%.1f', Kp, Ki));
legend('I_{out}', 'I_{ref}');

subplot(3,1,3);
plot(mil.time*1000, mil.phi_ext, 'm-', 'LineWidth', 1.5);
grid on;
ylabel('Phase Shift (deg)');
xlabel('Time (ms)');
title('Control Action: Phase Shift (MIL)');

%% ==================== Performance Analysis ====================
fprintf('\n=== MIL 성능 분석 ===\n');
ss_idx = floor(0.85 * n_steps);
I_ss = mean(mil.I_out(ss_idx:end));
I_ref_final = mil.I_ref(end);
ss_error = abs(I_ss - I_ref_final) / I_ref_final * 100;

fprintf('정상상태 전류: %.2f A (목표: %.2f A)\n', I_ss, I_ref_final);
fprintf('정상상태 오차: %.2f %%\n', ss_error);

%% ==================== Info Message ====================
fprintf('\n=================================================================\n');
fprintf('  Simulink 결과는 Data Inspector에서 확인하세요.\n');
fprintf('  (Simulation Data Inspector 열기: Simulink > Analysis > Data Inspector)\n');
fprintf('=================================================================\n');
fprintf('\n비교 스크립트 종료.\n');
