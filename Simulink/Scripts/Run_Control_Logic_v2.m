% =========================================================================
% Run_Control_Logic_v2.m
% Control_Logic.slx 직접 실행 (디렉토리 변경 방식)
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Running Simulink Control_Logic.slx (v2)\n');
fprintf('=================================================================\n\n');

%% Parameters
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
I_ess_cmd = 10;
kp_voltage = 5; 
ki_voltage = 5000;

%% TPS Table
load('TPS_Table.mat', 'TPS_Table');
fprintf('TPS Table 로드 완료\n');

%% Modules 디렉토리로 이동
original_dir = pwd;
modules_dir = fullfile(pwd, 'Modules');
cd(modules_dir);
fprintf('작업 디렉토리: %s\n', pwd);

%% Run
SIM_TIME = 0.020;  % 20ms
model_name = 'Control_Logic';

fprintf('\n시뮬레이션 시작: %s (%.0f ms)\n', model_name, SIM_TIME*1000);

try
    load_system(model_name);
    fprintf('모델 로드 완료\n');
    
    tic;
    simOut = sim(model_name, 'StopTime', num2str(SIM_TIME));
    sim_time = toc;
    
    fprintf('시뮬레이션 완료! (%.2f sec)\n\n', sim_time);
    
    % 결과 확인
    fprintf('simOut 필드:\n');
    fn = fieldnames(simOut);
    for i = 1:length(fn)
        fprintf('  - %s\n', fn{i});
    end
    
    if isprop(simOut, 'logsout') && ~isempty(simOut.logsout)
        logs = simOut.logsout;
        fprintf('\nlogsout 시그널 (%d개):\n', logs.numElements);
        for i = 1:logs.numElements
            sig = logs.getElement(i);
            fprintf('  [%d] %s\n', i, sig.Name);
        end
    end
    
catch ME
    fprintf('✗ 오류: %s\n', ME.message);
end

%% 원래 디렉토리로 복귀
cd(original_dir);
fprintf('\n작업 디렉토리 복귀: %s\n', pwd);
fprintf('스크립트 종료.\n');
