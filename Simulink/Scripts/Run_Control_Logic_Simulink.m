% =========================================================================
% Run_Control_Logic_Simulink.m
% Control_Logic.slx 직접 실행 및 결과 분석
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Running Simulink Control_Logic.slx\n');
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
I_ess_cmd = 10;  % 목표 전류
kp_voltage = 5; 
ki_voltage = 5000;

%% Load TPS Table
if exist('TPS_Table.mat', 'file')
    load('TPS_Table.mat', 'TPS_Table');
    fprintf('TPS Table 로드 완료\n');
else
    error('TPS_Table.mat 필요. TPS_LUT.m 먼저 실행하세요.');
end

%% Run Simulink
SIM_TIME = 0.050;  % 50ms
script_dir = fileparts(mfilename('fullpath'));
model_name = fullfile(script_dir, 'Modules', 'Control_Logic.slx');

fprintf('\nSimulink 모델 로드 중: %s\n', model_name);

try
    load_system(model_name);
    fprintf('모델 로드 완료\n');
    
    fprintf('시뮬레이션 시작 (%.0f ms)...\n', SIM_TIME*1000);
    tic;
    
    simOut = sim(model_name, 'StopTime', num2str(SIM_TIME));
    
    sim_time = toc;
    fprintf('시뮬레이션 완료! (%.2f sec 소요)\n\n', sim_time);
    
    % 결과 분석
    fprintf('=== 시뮬레이션 출력 분석 ===\n');
    fprintf('simOut 구조:\n');
    disp(simOut);
    
    % logsout 확인
    if isprop(simOut, 'logsout')
        logs = simOut.logsout;
        fprintf('\nlogsout 시그널 목록:\n');
        for i = 1:logs.numElements
            sig = logs.getElement(i);
            fprintf('  [%d] %s: size %s\n', i, sig.Name, mat2str(size(sig.Values.Data)));
        end
    elseif isprop(simOut, 'yout')
        fprintf('\nyout 사용 가능\n');
        disp(simOut.yout);
    else
        fprintf('\n경고: 출력 데이터을 찾을 수 없습니다.\n');
        fprintf('      모델에 Scope 또는 To Workspace 블록 추가 필요\n');
    end
    
catch ME
    fprintf('✗ 시뮬레이션 실패: %s\n', ME.message);
    fprintf('\n상세 오류:\n');
    disp(ME.stack);
end

fprintf('\n스크립트 종료.\n');
