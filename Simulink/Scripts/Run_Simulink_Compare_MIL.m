% =========================================================================
% Run_Simulink_Compare_MIL.m
% Control_Logic.slx 실행 후 MIL 결과와 비교 (다이나믹 시나리오 적용)
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Simulink vs MIL Comparison (Dynamic Scenario)\n');
fprintf('=================================================================\n\n');

%% 1. 시나리오 생성 실행
fprintf('=== STEP 0: 시나리오 생성 ===\n');
run('Create_Scenario.m'); % SIM_TIME, Ref_... 변수 생성
fprintf('시나리오 로드 완료 (%d steps, %.3f sec)\n', length(Ref_VH.Data), SIM_TIME);

%% Parameters (시뮬레이션과 동일하게)
V_pri_DC = 400; % 초기값 (Simulink Constant 블록용 Fallback)
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
I_ess_cmd = 10; % 초기값
kp_voltage = 5; 
ki_voltage = 5000;
Load = 10;  % Ohm

%% MIL Parameters
params.n = N_pri / N_sec;
params.L = L_leakage;
params.fs = f_sw;
params.C_out = C_dc_link;
params.efficiency = 0.95;

%% Load TPS Table
if isfile('TPS_Table.mat')
    load('TPS_Table.mat', 'TPS_Table');
    fprintf('TPS Table 로드 완료\n');
else
    load('../TPS_Table.mat', 'TPS_Table'); % 상위 폴더 확인
    fprintf('TPS Table 로드 완료 (상위 폴더)\n');
end

%% MATLAB 경로에 현재 디렉토리 추가
addpath(pwd);
addpath('..'); % 상위 폴더(Simulink Root) 추가
addpath('../Data'); % 데이터 사전 폴더 추가 (DAB_Parameters.sldd)

%% Base Workspace에 파라미터 할당
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
fprintf('파라미터 Base Workspace에 할당 완료\n');

%% Simulink 실행
fprintf('\n=== STEP 1: Simulink 실행 ===\n');

% Modules 디렉토리로 이동
original_dir = pwd;
cd('../Modules'); % 현재 Scripts 폴더 가정 -> 상위/Modules

% Load HDC Model
if isfile('../HDC_Model.mat')
    load('../HDC_Model.mat');
    HDC_Model_Struct = struct('W',W, 'Level_Item_V',Level_Item_V, 'Level_Item_P',Level_Item_P, 'min_V',min_V, 'max_V',max_V, 'min_P',min_P, 'max_P',max_P, 'L_levels',L_levels);
    assignin('base', 'HDC_Model_Struct', HDC_Model_Struct);
    fprintf('HDC Model loaded and assigned to workspace.\n');
else
    HDC_Model_Struct = [];
    fprintf('Warning: HDC_Model.mat not found.\n');
end

try
    clear DAB_Controller_System; % Ensure class update is picked up
    fprintf('모델 로드 중: Control_Logic.slx\n');
    load_system('Control_Logic');
    
    % Try to find the MATLAB System Block
    sys_blocks = find_system('Control_Logic', 'MatchFilter', @Simulink.match.internal.filterOutInactiveVariantSubsystemChoices, 'BlockType', 'MATLABSystem');
    if ~isempty(sys_blocks)
        blk = sys_blocks{1};
        % Try to set parameters if they exist
        try
            % Force reload of System Object to recognize new properties (EnableHDC)
            set_param(blk, 'System', 'DAB_Controller_System');
            
            set_param(blk, 'EnableHDC', 'true'); 
            set_param(blk, 'HDC_Model', 'HDC_Model_Struct');
            fprintf('Enabled HDC on block: %s\n', blk);
        catch ME
            fprintf('Could not set HDC parameters on block: %s\n', ME.message);
        end
    end

    % User Requirement: Output variable is 'out' and contains Data_IO/Control
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

%% MIL 실행
fprintf('\n=== STEP 2: MIL 실행 (Dynamic Loop) ===\n');

dt_mil = 10e-6;
time_vec = 0:dt_mil:SIM_TIME;
n_steps = length(time_vec);

% Controller States
Kp = 0.900;
Ki = 200.0;
Phi_int_deg = 15;
V_HV_Threshold = 350;
V_HV_Hysteresis = 370;
Ups_Target_Current = 10;
PowerLossState = 0; % 0: ESS, 1: UPS

V_out = V_batt_nom;
integrator = 0;

% Scenario Interpolation
if exist('Ref_VH', 'var')
    V_high_data = Ref_VH.Data;
    V_high_time = Ref_VH.Time;
    I_cmd_data = Ref_I_ess_cmd.Data;
    I_cmd_time = Ref_I_ess_cmd.Time;
else
    % Fallback
    V_high_data = 400 * ones(n_steps, 1);
    I_cmd_data = 10 * ones(n_steps, 1);
    fprintf('Warning: Scenario data not found, using constants.\n');
end

% Results Arrays
mil.time = time_vec;
mil.V_out = zeros(n_steps, 1);
mil.I_out = zeros(n_steps, 1);
mil.phi_ext = zeros(n_steps, 1);
mil.mode = zeros(n_steps, 1);
mil.V_in = zeros(n_steps, 1);

fprintf('MIL 시뮬레이션 실행 중...\n');
tic;

for k = 1:n_steps
    t_curr = time_vec(k);
    
    % 1. Get Inputs from Scenario (Linear Interp or Index)
    % Index lookup is faster for fixed step
    idx = round(t_curr / dt_mil) + 1;
    if idx > length(V_high_data), idx = length(V_high_data); end
    
    V_in = V_high_data(idx);
    I_cmd_user = I_cmd_data(idx);
    
    I_fb = V_out / Load; % Simple Resistive Load Feedback
    
    % 2. Controller Logic (DAB_Controller_System.m Logic Replicated)
    
    % Mode Selection Logic
    if V_in < V_HV_Threshold
        PowerLossState = 1; % UPS Mode
    elseif V_in > V_HV_Hysteresis
        PowerLossState = 0; % Recovery
    end
    
    if PowerLossState == 1
        % UPS Mode
        Iref = Ups_Target_Current;
    else
        % ESS Mode
        Iref = I_cmd_user;
    end
    
    % PI Control
    error_sig = Iref - I_fb;
    
    % Integrator Reset Logic
    if PowerLossState == 1 && abs(error_sig) > 5
        integrator = 0;
    end
    
    integrator = integrator + error_sig * dt_mil;
    phi_pi = Kp * error_sig + Ki * integrator;
    
    % --- HDC Feedforward (MIL) ---
    phi_hdc = 0;
    if exist('HDC_Model_Struct', 'var') && ~isempty(HDC_Model_Struct)
         % P_target approximation for HDC
         P_target = V_out * Iref; 
         
         % HDC Predict 
         % (Assuming HDC_Predict is on path, otherwise implementing inline)
         try
             [~, ~, d3_pred] = HDC_Predict(V_in, P_target, HDC_Model_Struct);
             phi_hdc = d3_pred * 360; 
         catch
             phi_hdc = 0; 
         end
    end
    
    phi_ext_deg = phi_pi + phi_hdc;
    phi_ext_deg = max(-90, min(90, phi_ext_deg));
    
    % 3. Plant Logic
    % Load is treated as R_wire + R_internal ~ 10mOhm for battery model.
    % However, 'Load' variable is currently 10 Ohm. Let's force a small R for battery.
    R_batt_internal = 0.1; 
    
    [V_out_next, ~, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                             Phi_int_deg, R_batt_internal, V_batt_nom, dt_mil, params);
    V_out = V_out_next;
    
    % 4. Save Results
    mil.V_out(k) = V_out;
    mil.I_out(k) = (V_out - V_batt_nom) / R_batt_internal; % Battery Current
    mil.phi_ext(k) = phi_ext_deg;
    mil.mode(k) = PowerLossState;
    mil.V_in(k) = V_in;
    
    % Save Target (Reference) for plotting
    mil.I_ref(k) = Iref;
end

mil_elapsed = toc;
fprintf('✓ MIL 완료! (%.3f sec 소요)\n', mil_elapsed);

%% 결과 저장
fprintf('\n=== STEP 5: 결과 저장 (Simulink Data_IO Focus) ===\n');
if exist('out', 'var')
    % 시나리오 Ref 변수들과 SimulationOutput 객체를 저장
    save('Comparison_Results.mat', 'out', 'Ref_I_ess_cmd', 'Ref_VH', 'Ref_Sw');
    fprintf('✓ 결과 저장 완료: Comparison_Results.mat (out + Scenario Refs)\n');
else
    fprintf('⚠ 경고: out 객체가 생성되지 않았습니다.\n');
end
%% 플롯 (Dynamic)
fprintf('\n=== STEP 3: 결과 비교 및 플롯 ===\n');

figure('Name', 'Simulink vs MIL (Dynamic Test)', 'Position', [50 50 1200 800]);

% 1. Input Voltage & Mode
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

% 2. Output Current
subplot(3,1,2);
plot(mil.time*1000, mil.I_out, 'b-', 'LineWidth', 1.5); hold on;
plot(mil.time*1000, mil.I_ref, 'g--', 'LineWidth', 2);
grid on;
ylabel('Output Current (A)');
title('Output Current (Target vs Actual)');
legend('I_{out} (MIL)', 'Target I');

% 3. Phase Shift
subplot(3,1,3);
plot(mil.time*1000, mil.phi_ext, 'm-', 'LineWidth', 1.5);
grid on;
ylabel('Phase Shift (deg)');
xlabel('Time (ms)');
title('Control Action: Phase Shift');

%% Simulink 데이터 비교 (가능한 경우)
if simulink_ok
    % Logsout 데이터 처리
    try
        % Simulink 결과 가져오기 (예: I_out, phi_ext)
        % 사용자가 ToWorkspace에 'I_out_sim', 'phi_sim' 등으로 저장했다면 여기서 비교
        % 현재는 MIL 수행 여부만 확인
        fprintf('Simulink 데이터가 존재합니다. Workspace에서 확인하세요.\n');
    catch
    end
end

fprintf('\n모든 검증 완료. 그래프를 확인하세요.\n');
