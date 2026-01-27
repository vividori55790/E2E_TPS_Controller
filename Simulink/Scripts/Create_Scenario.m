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

% [NEW] 배터리 전압 및 초기조건 통일 (96V)
% 모델 내의 배터리 전압원과 커패시터 초기값이 이 변수들을 참조하도록 하세요.
V_batt = 96;
assignin('base', 'V_batt', V_batt);
assignin('base', 'V_init_LV', V_batt); % 커패시터 초기값용 (반드시 V_batt와 같아야 함)

% 2.5 배터리 전압 및 초기값 설정 (가장 중요)
V_batt = 96;
assignin('base', 'V_batt', V_batt); % Simulink 배터리 전압원 및 커패시터 초기전압용

% 3. 전압원 설정 (이상적인 전압원으로 고정 - 제어기 성능 격리 평가)
% HVDC (Primary) - 380V 고정
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

% 5. 회로 파라미터 보강 (전압 변동 억제용)
assignin('base', 'C_HV', 1000e-6); % 1000uF로 증량
assignin('base', 'R_ESR_HV', 0.05); % HV 커패시터용 저항 (50mOhm) - [복구] LC 공진 댐핑용 필수 저항

% LV(배터리)측 등가회로 안정화용 저항 (필수)
assignin('base', 'R_ESR_LV', 0.01); % LV 커패시터용 저항 (10mOhm)

% [NEW] 단순 저항 배터리 모델용 내부 저항
assignin('base', 'R_batt_int', 0.15); % 150mOhm (비교적 큰 내부저항 가정하여 댐핑 효과 증대)

% fprintf('Create_Scenario: Clean Step Response (10A at %.2fs) Configured.\n', step_time);