function config = DAB_System_Config()
% =========================================================================
% DAB_System_Config.m
% DAB 컨버터 시스템의 모든 파라미터를 중앙 집중 관리하는 설정 파일
% =========================================================================
% 사용 방법:
%   config = DAB_System_Config();
%   params = config.plant;
%   V_batt = config.battery.V_nom;
% =========================================================================

%% ==================== Plant Parameters ====================
% DAB 컨버터 하드웨어 사양
config.plant.n = 4.0;               % 변압기 권선비 (N_pri / N_sec)
config.plant.L = 50e-6;             % 누설 인덕턴스 (H)
config.plant.fs = 80e3;             % 스위칭 주파수 (Hz)
config.plant.C_out = 500e-6;        % 출력 커패시터 (F)
config.plant.efficiency = 0.95;     % 시스템 효율 (0~1)
config.plant.P_rated = 5000;        % 정격 전력 (W)

%% ==================== Battery Model ====================
% 배터리/에너지 저장 시스템 파라미터
config.battery.V_nom = 100;         % 공칭 전압 (V) - Back-EMF
config.battery.R_internal = 0.1;    % 내부 저항 (Ohm)

%% ==================== High Voltage Side ====================
% 1차측 (HV) 파라미터
config.hv.V_nom = 400;              % 공칭 전압 (V)
config.hv.V_min = 300;              % 최소 동작 전압 (V)
config.hv.V_max = 450;              % 최대 동작 전압 (V)

%% ==================== Controller Parameters ====================
% PI 제어기 게인 (최적화로 업데이트됨)
config.controller.Kp = 0.900;       % 비례 이득
config.controller.Ki = 200.0;       % 적분 이득
config.controller.Phi_int_deg = 15; % 내부 위상 (deg)

% 모드 전환 임계값
config.controller.V_HV_Threshold = 350;   % 정전 감지 임계값 (V)
config.controller.V_HV_Hysteresis = 370;  % 복구 임계값 (V)
config.controller.Ups_Target_Current = 10; % UPS 모드 목표 전류 (A)

%% ==================== Simulation Settings ====================
% MIL 시뮬레이션 설정
config.sim.dt = 10e-6;              % 시뮬레이션 타임스텝 (s)
config.sim.time = 0.100;            % 기본 시뮬레이션 시간 (s)
config.sim.step_start = 0.010;      % 스텝 입력 시작 시간 (s)

%% ==================== Optimization Settings ====================
% 최적화 탐색 범위
config.optim.Kp_range = [0.1, 3.0]; % Kp 범위 [min, max]
config.optim.Ki_range = [100, 1000]; % Ki 범위 [min, max]
config.optim.n_points = 10;          % 각 축 탐색 포인트 수

% 성능 목표
config.optim.target_overshoot = 5;   % 목표 오버슈트 (%)
config.optim.target_ss_error = 2;    % 목표 정상상태 오차 (%)
config.optim.target_settling = 0.030; % 목표 정착시간 (s)

% Cost Function 가중치
config.optim.weight_overshoot = 5.0;
config.optim.weight_ss_error = 15.0;
config.optim.weight_settling = 1.0;

%% ==================== Test Scenarios ====================
% 기본 테스트 조건
config.test.I_ref = 10;             % 기본 목표 전류 (A)
config.test.I_ref_init = 0;         % 스텝 전 전류 지령 (A)

%% ==================== Derived Parameters ====================
% 자동 계산되는 파라미터들
config.derived.Ts = 1 / config.plant.fs;  % 스위칭 주기 (s)
config.derived.n_steps = round(config.sim.time / config.sim.dt) + 1;

end
