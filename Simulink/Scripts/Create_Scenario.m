%% Create_Scenario.m
% DAB Converter Step Response Test Scenario
% Target: 0.1s Simulation, Step at 0.02s (0->10A)

% 1. 시뮬레이션 기본 설정
SIM_TIME = 0.05; 
assignin('base', 'SIM_TIME', SIM_TIME);

% 2. 제어 및 회로 파라미터 (초기값 - GA가 나중에 덮어씀)
assignin('base', 'Kp_ext', 1.0);
assignin('base', 'Ki_ext', 100.0);
assignin('base', 'Kpv', 0.1);  % UPS용 (에러 방지용 유지)
assignin('base', 'Kiv', 10.0); % UPS용 (에러 방지용 유지)

% 3. 전압원 설정
% HVDC (Primary) - 이상적 전압원 가정 시 필요
time_vec = [0, SIM_TIME];
val_vh = [380, 380]; 
Ref_VH = timeseries(val_vh, time_vec);
assignin('base', 'Ref_VH', Ref_VH);

% 4. ★ 중요: 전류 지령 (Step 입력)
% 0 ~ 0.02s: 0A
% 0.02s ~ 끝 : 10A (충전)
step_time = 0.02;
ref_current = 10;

t_cmd = [0, step_time-1e-6, step_time, SIM_TIME];
v_cmd = [0, 0,              ref_current, ref_current];

Ref_I_ess_cmd = timeseries(v_cmd, t_cmd);
assignin('base', 'Ref_I_ess_cmd', Ref_I_ess_cmd);

% fprintf('Create_Scenario: Clean Step Response (10A at %.2fs) Configured.\n', step_time);