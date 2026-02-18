function config = HDC_Config()
% =========================================================================
% HDC_Config.m
% DAB 컨버터 TPS 제어 프로젝트의 통합 파라미터 관리 파일
% =========================================================================
% 모든 Phase (1, 2, 3) 코드들이 이 설정 파일을 참조합니다.
% 파라미터를 변경하려면 이 파일 하나만 수정하면 됩니다.
% =========================================================================

%% ==================== 1. 시스템 하드웨어 (Hardware Spec) ====================
config.sys.n  = 4.0;         % 변압기 권선비 (N_pri / N_sec)
config.sys.L  = 90e-6;       % 누설 인덕턴스 (H) [사용자 지정 90uH]
config.sys.fs = 80e3;        % 스위칭 주파수 (Hz)
config.sys.P_rated = 1000;   % 정격 전력 (W) [사용자 지정 1000W]

% 커패시터 및 배터리
config.sys.C_HV = 1000e-6;   % HV측 DC 링크 커패시터 (F)
config.sys.C_LV = 500e-6;    % LV측 배터리 커패시터 (F)
config.sys.V_batt = 96;     % 배터리 공칭 전압 (V)

%% ==================== 2. Phase 1: LUT 최적화 설정 ====================
% 효율 맵(LUT) 생성 관련 파라미터
config.opt.GridSize = 12*6;            % 각 축(Vin, Vout) 그리드 해상도 (12x12)
config.opt.P_GridSize = 15*6;          % 전력 축 그리드 해상도 (더 촘촘하게)
config.opt.ZVS_Margin = 0.2;         % ZVS 최소 마진 (A) [0.2A로 완화하여 효율 개선]

% 전압/전력 탐색 범위
config.opt.Vin_Range  = [300, 450];  % HV 입력 전압 범위 (V)
config.opt.Vout_Range = [85, 110];   % LV 출력 전압 범위 (V)
config.opt.P_Range    = [-config.sys.P_rated, config.sys.P_rated]; % 전력 범위

% PSO 최적화 옵션 (하이브리드 탐색)
config.opt.PSO_SwarmSize = 50;       % 입자 수
config.opt.PSO_MaxIter   = 100;      % 최대 반복 횟수

%% ==================== 3. Phase 2: 제어기 설정 ====================
% PI 제어기 (전압 제어 + 위상 보정)
config.ctrl.Kp_phi = 0.5;   % 비례 이득 (deg/V)
config.ctrl.Ki_phi = 50.0;  % 적분 이득 (deg/s/V)

% 데드타임 보상
config.ctrl.DeadTime_ns = 200;       % 스위칭 데드타임 (ns)

% 시뮬레이션
config.sim.phase2_time = 0.2;        % Phase 2 시뮬레이션 시간 (s)

%% ==================== 4. Phase 3: 추정기 설정 ====================
% RLS (Recursive Least Squares) 파라미터
config.est.RLS_Lambda = 0.98;        % 망각 계수 (0.995 -> 0.98 반응성 향상)
config.est.EMA_Alpha  = 0.05;        % EMA 필터 계수
config.est.Interval   = 0.001;       % 추정 주기 (s) - 1ms

% 시뮬레이션
config.sim.phase3_time = 2.0;        % Phase 3 시뮬레이션 시간 (s)

%% ==================== 5. 공통 경로 설정 ====================
script_path = fileparts(mfilename('fullpath'));
if isempty(script_path), script_path = pwd; end

config.path.LUT_File  = fullfile(script_path, 'Phase1_LUT_Results.mat');
config.path.Ctrl_File = fullfile(script_path, 'Phase2_Controller_Data.mat');
config.path.Est_File  = fullfile(script_path, 'Phase3_Estimator_Results.mat');

end
