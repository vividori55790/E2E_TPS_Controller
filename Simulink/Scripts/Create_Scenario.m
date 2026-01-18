% Scripts/Create_Scenario.m

% 시뮬레이션 파라미터
SIM_TIME = 0.100; % 100ms로 연장 (모든 모드 테스트)
dt = 10e-6;
time = (0:dt:SIM_TIME)';
n_steps = length(time);

% 1. 기본값 설정
I_ess_cmd = zeros(n_steps, 1);
DC_link_Switch = zeros(n_steps, 1);
V_high = 400 * ones(n_steps, 1); % 기본 400V
V_low = 100 * ones(n_steps, 1);

% 2. 시나리오 정의

% [Event 1] 10ms: DC Link 스위치 ON
idx_switch_on = find(time >= 0.010, 1);
DC_link_Switch(idx_switch_on:end) = 1;

% [Event 2] 20ms~40ms: ESS 충전 모드 (10A)
idx_charge_start = find(time >= 0.020, 1);
idx_discharge_start = find(time >= 0.040, 1);
I_ess_cmd(idx_charge_start:idx_discharge_start-1) = 10;

% [Event 3] 40ms~60ms: ESS 방전 모드 (-10A)
idx_ups_start = find(time >= 0.060, 1);
I_ess_cmd(idx_discharge_start:idx_ups_start-1) = -10;

% [Event 4] 60ms~90ms: 정전 발생 (UPS 모드 테스트)
% 전압이 350V 이하로 떨어지면 UPS 모드 진입 (DAB_Controller_System 설정)
idx_ups_end = find(time >= 0.090, 1);
V_high(idx_ups_start:idx_ups_end-1) = 330; % 330V로 강하 (Threshold 350V)

% 90ms~: 전압 복구 (Hysteresis 370V 확인)
% 330 -> 400V 복구

% 3. 개별 변수로 생성 (Ref_Sw, Ref_I_ess_cmd, Ref_VL, Ref_VH)
Ref_I_ess_cmd = timeseries(I_ess_cmd, time);
Ref_Sw = timeseries(DC_link_Switch, time);
Ref_VH = timeseries(V_high, time);
Ref_VL = timeseries(V_low, time);

% Workspace로 내보내기
assignin('base', 'Ref_I_ess_cmd', Ref_I_ess_cmd);
assignin('base', 'Ref_Sw', Ref_Sw);
assignin('base', 'Ref_VH', Ref_VH);
assignin('base', 'Ref_VL', Ref_VL);
assignin('base', 'SIM_TIME', SIM_TIME); % SIM_TIME도 내보내기

fprintf('시나리오 데이터 생성 완료 (0.1s):\n');
fprintf('  - 20~40ms: ESS Charge (10A)\n');
fprintf('  - 40~60ms: ESS Discharge (-10A)\n');
fprintf('  - 60~90ms: UPS Mode (V_high = 330V)\n');
fprintf('  - 90ms~  : Recovery\n');