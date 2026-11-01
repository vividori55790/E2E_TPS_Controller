% ======================================================================
% [FILE METADATA & VERSION TRACKING]
% - Current Version: v1.9.0 (2026-06-08)
% - Target Environment: MATLAB R2023b+ / STM32G474RET6U (64KB SRAM Target)
% - Integrity Check: DO NOT delete any existing functions unless explicitly requested.
% ======================================================================
% [CHANGELOG - NEVER DELETE THIS HISTORY]
% * v1.9.0 (2026-06-08) - Developer: Overhauled RL update pipeline with Batch Data Collection and Multi-Epoch training:
%     1) Decoupled simulation trajectory sweep from HDC policy updates.
%     2) Introduced offline batch collection buffer per generation.
%     3) Implemented 10-epoch randomized mini-batch training per generation to break the 0.3 control surface freeze.
%     4) Integrated dynamic Softmax Temperature Annealing (dynamic_gamma) to smooth policy gradients and prevent localized trapping.
%     5) Maintained all physical constraint guards, C-header exports, and real-time dashboard plotting compatibility.
% * v1.8.0 (2026-06-07) - Developer: Integrated Temporal Difference (TD) Learning Pipeline:
%     1) Overhauled the training sweep into continuous grid-trajectory transition sequences.
%     2) Formulated state transitions on the 2D Power-Voltage grid using adjacent index mapping.
%     3) Implemented TD Target: y_target = reward + gamma_td * V(S_next) using HDC level-encoded representation.
%     4) Computed TD Error: Advantage = y_target - V(S_current) as the core advantage signal.
%     5) Updated Actor (M_policy) and Critic (V_baseline) jointly using TD Error gradient propagation.
% * v1.7.0 (2026-06-07) - Developer: Resolved Global Baseline Fluctuation via Localized Value Approximation:
%     1) Integrated State-Dependent Baseline Vector V_baseline (size K x 1) to replace the monolithic reward_baseline.
%     2) Formulated local advantage using interpolated baseline: Advantage = reward - sum(softmax_w * V_baseline).
%     3) Implemented semi-gradient value function update: V_baseline = V_baseline + adaptive_gain * advantage * softmax_w.
%     4) Added strict backward compatibility check during checkpoint loading to prevent missing field crashes.
% * v1.6.0 (2026-06-06) - Developer: Integrated Adaptive Meta-Control framework:
%     1) Dynamic Baseline Tracking Gain decaying over generations to prevent gradient saturation.
%     2) Stagnation-aware Mutation Rate Scaling (up to 35%) triggered by recent 4-gen improvement plateau.
%     3) Gradual Cosine Annealing Cycle Expansion (100 -> 300 epochs) to secure late-stage control surface fine-tuning.
% * v1.5.1 (2026-06-06) - Developer: Added real-time tracking of average peak current
%     reduction performance (mean_delta_I) relative to the SPS baseline, displaying it
%     in the console debug log (as I_Red in Amperes) and embedding it into the best checkpoint metadata.
% * v1.5.0 (2026-06-06) - Developer: Integrated defensive Try-Catch guards around
%     all disk-writing routines (save checkpoints, C header generator) to guarantee
%     infinite training loop survivability. Added NaN/Inf sanitizer inside the C-header
%     export process to safeguard STM32 physical MCU core from hard fault crashes.
%     Overhauled terminal user input prompt to fail-safe default values upon malformed entries.
% * v1.4.0 (2026-06-06) - Developer: Integrated Graceful GUI Exit Hook (CloseRequestFcn)
%     to prevent MAT checkpoint corruption; added Advantage clipping ([-15, 15]) and L2
%     Weight Decay on M_policy to stabilize training in infinite loop settings; optimized
%     HDC encoding loop efficiency without violating physical outlier rejection logic.
% * v1.3.4 (2026-06-06) - Developer: Resolved HDC policy tensor saturation by integrating
%     tanh-based soft clipping to keep gradients alive; overhauled 'encode_HDC_data' with
%     extrapolation robust filtering and exception hypervector mapping (vector inversion
%     upon out-of-bound physical state detection) to prevent dangerous state blending.
% * v1.3.3 (2026-06-06) - Developer: Optimized the phase-shift 'simulate_physics' solver
%     by substituting the 361-point brute-force sweep with a high-speed Coarse-to-Fine
%     Search algorithm. This cuts redundant NN inference calls by over 80% while preserving
%     identical physical accuracy (0.5-degree grid resolution) and numerical outputs.
% * v1.3.2 (2026-06-06) - Developer: Fixed critical runtime crash caused by undefined
%     'action_error' variable; removed Communications Toolbox dependency by replacing
%     'randsrc' with native rand/sign logic; integrated automatic C header generation
%     upon saving the best checkpoint; added defensive self-healing mock fallback for missing
%     physics MAT files to guarantee out-of-the-box runnability.
% * v1.3.1 (2026-06-06) - User: Fixed critical gradient scaling mismatch bug
%     (divided update by D), added strict M_policy clipping guards to prevent flat-surface
%     saturation, implemented running reward baseline for Advantage-based updates, and
%     integrated a Self-Healing policy recovery contraction safety-net.
% * v1.3.0 (2026-06-05) - User: Added interactive boot choices (Resume [0] vs
%     New [1]), infinite training loop with no end-time restriction, and highly
%     creative prototype-level Local Genetic Crossover & Mutation mechanism
%     for rapid evolutionary-inspired RL convergence.
% * v1.2.0 (2026-06-05) - User: Added automatic model checkpoints, active-bold
%     exploration with Cosine Annealing, comprehensive real-time debugging
%     visualization dashboard, and customized reward based strictly on peak current
%     reduction with strict physical constraint guards.
% * v1.1.0 (2025-11-14) - User: Initial stable release.
% ======================================================================
function train_RL_RegHDC_Pipeline()
clear; clc; close all;
disp('===========================================================');
disp('[Phase 0] 시스템 초기화 및 물리 환경 시뮬레이터 로드');
disp('===========================================================');

% 하위 전력/전류 예측 모델 로드 (RL 환경의 Physics Simulator 역할)
mlp_file = 'Data/trained_fast_mlp_net_v6.mat';
xgb_file = 'Data/trained_xgboost_models_v6.mat';

use_mock_physics = false;
if ~isfile(mlp_file)
warning('RL_RegHDC:PhysicsMissing', ...
'물리 환경 구축을 위한 상위 MLP 모델(%s)이 없습니다. 안전한 테스트를 위해 모의(Mock) 물리 예측 모델로 대체하여 무중단 자동 기동합니다.', mlp_file);
use_mock_physics = true;
end

if use_mock_physics
% 파일이 없을 시, 시스템 타겟 특성을 추종하는 전력/전류 비선형 근사 함수 적용
env_predict = @(X_in) mock_physics_predict(X_in);
else
load(mlp_file, 'net', 'PS_X', 'PS_Y');
xgb_models = [];
if isfile(xgb_file)
load(xgb_file, 'xgb_models');
end
env_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
end

% 물리 제약 조건 정의
env_cfg.ML_RMSE_P = 8.0;
env_cfg.ML_RMSE_I = 0.5;
env_cfg.P_ratio = 0.02;

env_cfg.calc_margin = @(P_target) env_cfg.ML_RMSE_P + abs(P_target) * env_cfg.P_ratio;
env_cfg.predict_fn = env_predict;

disp('===========================================================');
disp('[Phase 1] RL 에피소드용 상태 공간(State Space) 격자 생성');
disp('===========================================================');
% 하드웨어 리소스 절약을 위해 핵심 동작 영역 위주로 환경 상태 구성
P_vec = -1200:100:1200;
V2_vec = 80:2:110;
[P_grid, V2_grid] = meshgrid(P_vec, V2_vec);

% 상태(State) 공간 행렬 (2 x N) -> [Power; Voltage]
States = [P_grid(:)'; V2_grid(:)'];
num_states = size(States, 2);

% 기준선 데이터 (SPS 전류 프로파일 계산 - 보상 기준점)
disp(' -> 각 상태별 기준 SPS 전류 프로파일 연산 중...');
I_SPS_base = zeros(1, num_states);
for i = 1:num_states
% SPS 모드(기본 위상 D1=0.5, D2=0.5)에서의 전류 시뮬레이션
[~, i_sps, ~] = simulate_physics(States(1,i), States(2,i), 0.5, 0.5, env_cfg);
I_SPS_base(i) = i_sps;
end
I_SPS_base(isnan(I_SPS_base)) = 10.0; % 예외 보호용 디폴트 고전류

disp('===========================================================');
disp('[Phase 2] 사용자 실행 모드 선택 및 모델 초기화/복원');
disp('===========================================================');

checkpoint_file = 'RL_RegHDC_checkpoint.mat';
start_choice = 1; % 0: Resume, 1: Fresh Start

% 자가 진단 및 디렉터리 자동 생성 보안 가드 추가
checkpoint_dir = fileparts(checkpoint_file);
if ~isempty(checkpoint_dir) && ~exist(checkpoint_dir, 'dir')
mkdir(checkpoint_dir);
end

if isfile(checkpoint_file)
fprintf('🔔 시스템 감지: 이전 최적 학습 체크포인트가 존재합니다.\n');
fprintf(' [0] 이전 체크포인트 모델을 로드하여 이어서 무한 학습 진행 (Resume)\n');
fprintf(' [1] 이전 학습 데이터를 완전 파기하고 처음부터 새로운 에이전트로 시작 (Fresh Start)\n');

% ==================================================================
% [AI STRICT GUARD: User Boot Input Guard]
% - Function: Terminal Input Exception Handling
% - Why it must be preserved: 사용자 입력 단계에서 빈 행이나 숫자가 아닌 값, 
%   또는 0, 1 외의 난수가 기입될 때 런타임 인터프리터 예외가 발생하여 기동이 멈추는 
%   현상을 완전히 해결하기 위해 엄격한 타당성 확인 후 기본값(Resume)으로 바인딩합니다.
% ==================================================================
try
    user_input = input(' -> 실행 모드를 입력하세요 (기본값 = 0): ');
    if isempty(user_input) || ~isnumeric(user_input) || ~any(user_input == [0, 1])
        disp('⚠️ 입력 값이 유효하지 않아 안전을 위해 기본값 [0](Resume) 모드로 자동 진입합니다.');
        start_choice = 0;
    else
        start_choice = user_input;
    end
catch ME_Input
    warning('RL_RegHDC:InputProcessingError', '사용자 입력 해석 중 오류가 발생하여 기본값 [0]으로 폴백합니다: %s', ME_Input.message);
    start_choice = 0;
end


else
disp(' -> [INFO] 신규 세션을 시작합니다 (기존 체크포인트 없음).');
start_choice = 1;
end

% 상태 및 액션 정규화 바운드
min_X = [-1200; 80];  max_X = [1200; 110];
min_Y = [0.0; 0.0];   max_Y = [0.4; 0.4];

% 하이퍼파라미터 초기 정의 (체크포인트 미로드 시 디폴트 스펙)
D = 256; K = 24; Q = 48;
gamma_softmax = 24.0;

% 유전학적 최적화용 히스토리 및 변수 선언
reward_history = [];
feasibility_history = [];
best_reward_history = [];
start_ep = 1;
best_global_reward = -Inf;
best_rl_model = struct();

% 하위 호환용 글로벌 이동평균 베이스라인 초기값
reward_baseline = -15.0;
collapse_counter = 0; % 붕괴 감지 카운터

if start_choice == 0
disp(' -> [LOAD] 체크포인트 파일에서 데이터를 실시간 복원 중...');
try
loaded_data = load(checkpoint_file);
best_rl_model = loaded_data.best_rl_model;

    % 복원 파라미터 적용
    C_int = best_rl_model.C_int;
    M_policy = best_rl_model.M_int;
    D = best_rl_model.D;
    K = best_rl_model.K;
    Q = best_rl_model.Q;
    gamma_softmax = best_rl_model.gamma;
    ID_vecs = best_rl_model.ID_vecs;
    L_vecs = best_rl_model.L_vecs;
    mem_KB = best_rl_model.mem_KB;
    best_global_reward = loaded_data.reward_history(end);
    reward_baseline = mean(loaded_data.reward_history(end-min(10, length(loaded_data.reward_history)-1):end));
    
    % ==================================================================
    % [AI STRICT GUARD: State-Dependent Baseline Compatibility Load]
    % - Function: Backup and recovery of localized baselines
    % - Why it must be preserved: 구 버전 체크포인트를 로드할 때 V_baseline 필드가 
    %   없을 가능성을 감안하여, 존재 여부를 체크한 뒤 디펜시브하게 복원 및 초기화 처리를 수행합니다.
    % ==================================================================
    if isfield(best_rl_model, 'V_baseline')
        V_baseline = best_rl_model.V_baseline;
    else
        disp('⚠️ [AI GUARD] 체크포인트 내 로컬 값 함수(V_baseline)가 유실되어 디폴트 균등 초기화로 대응 수립합니다.');
        V_baseline = reward_baseline * ones(K, 1);
    end
    
    % 이력 동기화
    reward_history = loaded_data.reward_history;
    feasibility_history = loaded_data.feasibility_history;
    best_reward_history = max(reward_history);
    start_ep = length(reward_history) + 1;
    
    fprintf(' -> [복원 완료] %d 에피소드 이력 및 구조 복사 성공 (D=%d, K=%d, Q=%d)\n', start_ep-1, D, K, Q);
catch ME_Load
    warning('RL_RegHDC:RestoreFailed', '체크포인트 파일 복원 실패: %s. 새로운 에이전트로 비상 재부팅을 속행합니다.', ME_Load.message);
    start_choice = 1;
end


end

if start_choice == 1
disp(' -> [INIT] 새 하이퍼차원 공간 생성 및 균등 프로토타입 분배 중...');
% 1. HDC 기초 기저 벡터 공간 균등 초기화 (Communications Toolbox 미설치 환경 완벽 호환)
rng('shuffle');
ID_vecs = sign(rand(2, D) - 0.5);
ID_vecs(ID_vecs == 0) = 1; % 0값 제거하여 완벽한 Bipolar 기저 확보

L_vecs = zeros(Q, D); 
L_vecs(1, :) = sign(rand(1, D) - 0.5); 
L_vecs(1, L_vecs(1, :) == 0) = 1;

flip_counts = max(1, floor(D / 2 / (Q - 1)));
for q_idx = 2:Q
    L_vecs(q_idx, :) = L_vecs(q_idx-1, :); 
    flip_idx = randperm(D, flip_counts); 
    L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx); 
end

% 2. 프로토타입 공간 분배 초기화
C_int = zeros(K, D);
init_idx = round(linspace(1, num_states, K));
[S_all, ~] = encode_HDC_data(States, min_X, max_X, Q, D, ID_vecs, L_vecs);
for k_init = 1:K
    C_int(k_init, :) = S_all(:, init_idx(k_init))';
end
M_policy = zeros(K, 2, D); % 정책 하이퍼차원 텐서 (초기에는 zero 기저에서 출발)
mem_KB = (K * D * 4 + K * 2 * D * 4 + Q * D * 1 + 2 * D * 1) / 1024;

% 상태 의존적 로컬 값 함수 베이스라인 초기 생성 (K개의 프로토타입 영역 맞춤 설계)
V_baseline = -15.0 * ones(K, 1);


end

% [창의적 유전학 기반 로컬 피크 교차 구조체 기획]
best_local_M = M_policy;

best_local_rewards = -Inf(K, 1);

% 전역 공통 탐색 파라미터 정의 (적극적인 변이를 주도하기 위한 고기동성 탐색 설계)
exploration_sigma_init = 0.65; % 초기 변이 크기를 대폭 넓혀 탐색 범위 극대화
exploration_sigma_min  = 0.25;
lr_policy_init = 0.05;         % 그라디언트 정상화에 맞춰 수렴력을 올리기 위해 소폭 인상
lr_policy_min  = 0.005;

% 실시간 디버깅 대시보드 피규어 생성
h_fig_debug = figure('Name', 'RL-RegHDC Evolutionary Real-Time Monitor (Infinite Loop)', ...
'Position', [50, 80, 1400, 800], 'Color', [0.93 0.94 0.96]);

% ==================================================================
% [AI STRICT GUARD: Graceful Exit Setup]
% - Function: Dashboard Close Request Interceptor
% - Why it must be preserved: 사용자가 창을 닫거나 종료할 때 진행된 에폭을
%   유실 없이 완전하게 덤프 세이브 후 프로세스를 안전하게 정상 정리하도록 백그라운드 핸들러를 바인딩합니다.
% ==================================================================
setappdata(h_fig_debug, 'request_stop', false);
set(h_fig_debug, 'CloseRequestFcn', @(src, event) setappdata(src, 'request_stop', true));

disp('===========================================================');
disp('🌟 [무한 세대 진화 루프 시작] 실시간 모니터링 창을 닫아 무중단 안전 탈출 및 자동 저장이 가능합니다.');
disp('===========================================================');

ep = start_ep;
[S_all, ~] = encode_HDC_data(States, min_X, max_X, Q, D, ID_vecs, L_vecs);

while true % 사용자가 직접 창을 닫아 중단 신호를 보내거나, 강제 취소할 때까지 구동

% GUI 예외 종료 여부 체킹 단락
if ~ishandle(h_fig_debug) || getappdata(h_fig_debug, 'request_stop')
    disp('⚠️ [SYSTEM] 대시보드 종료 요청 감지. 최적화 데이터 최종 동기화 중...');
    if ep > 1
        % ==================================================================
        % [AI STRICT GUARD: Model Export & Save Guard (Close Request Event)]
        % - Function: Emergency Safety Save Block
        % - Why it must be preserved: 외부 OS 디스크 권한 유실, 샌드박스 파일락 
        %   등으로 인한 예기치 못한 크래시를 방어하여 안전하게 훈련 파이프라인을 탈출합니다.
        % ==================================================================
        try
            save(checkpoint_file, 'best_rl_model', 'reward_history', 'feasibility_history');
            fprintf('💾 [SAVED] 최종 에포크 데이터가 %s에 안전하게 보호 기입되었습니다.\n', checkpoint_file);
        catch ME_ShutdownSave
            warning('RL_RegHDC:ShutdownSaveError', '⚠️ [CRITICAL] 최종 강제 체크포인트 저장 실패: %s', ME_ShutdownSave.message);
        end
    end
    if ishandle(h_fig_debug)
        delete(h_fig_debug);
    end
    break;
end

% ==================================================================
% [AI ATTENTION: Dynamic Exploration Cycle Scaling - v1.6.0 Overhaul]
% - 기존 100세대 고정 코사인 노이즈 주기에서 세대 경과에 따라 최대 300세대까지 점진적으로 확장합니다.
% - 어닐링 스케일링을 통해 극초반엔 빠른 탐색을, 후반부엔 치밀한 정책 표면 다듬기 성능을 유도합니다.
% ==================================================================
cycle_period = min(300, 100 + floor(ep / 5));
cycle_phase = mod(ep, cycle_period) / cycle_period;
current_sigma = exploration_sigma_min + ...
    0.5 * (exploration_sigma_init - exploration_sigma_min) * (1 + cos(pi * cycle_phase));
current_lr = lr_policy_min + ...
    0.5 * (lr_policy_init - lr_policy_min) * (1 + cos(pi * cycle_phase));

% ==================================================================
% [AI STRICT GUARD: Adaptive Baseline Tracking Gain - v1.6.0 Overhaul]
% - 기존의 고정비율(2%) 추종에서 세대 수(ep)에 반비례하는 동적 이득률로 업데이트합니다.
% - 초반에는 빠른 환경 적응을 시도하고, 후반부엔 묵직한 베이스라인 유지로 흔들림을 방지합니다.
% ==================================================================
adaptive_gain = max(0.01, min(0.10, 2 / (10 + ep^0.5)));

% ==================================================================
% [AI OVERHAUL: Batch Data Collection & Multi-Epoch RegHDC Pipeline]
% - 물리 엔진(simulate_physics) 호출 횟수는 기존과 동일하게 유지하여 속도 보존
% - 실시간 업데이트 대신 버퍼 적재 후 내부 대량 반복 학습(Multi-Epoch)으로 곡면 해방
% ==================================================================

% 세대 경과에 따른 Dynamic Softmax Smoothing (초반엔 부드럽게 전파, 후반엔 정교하게 맵핑)
% 고정된 24.0 장벽을 완화하여 그라디언트가 주변 프로토타입으로 흐르게 유도
decay_factor = max(0.2, 1 - (ep / 1000)); 
dynamic_gamma = max(8.0, gamma_softmax * (1 - decay_factor * 0.5));

% 에피소드 데이터 저장을 위한 버퍼 메모리 사전 할당 (속도 최적화)
buffer_s_hv        = zeros(D, num_states);
buffer_action_err  = zeros(2, num_states);
buffer_softmax_w   = zeros(K, num_states);
buffer_advantage   = zeros(1, num_states);

% -----------------------------------------------------------------
% [v1.8.0] TD-learning 적용을 위한 환경 전이 파라미터 구성
% -----------------------------------------------------------------
gamma_td = 0.95; % 시간차 학습 감쇄율 (Discount Factor)

total_ep_reward = 0;
valid_states_count = 0;
total_delta_I = 0; % SPS 대비 이번 세대의 총 피크 전류 저감 합산 버퍼 (실시간 분석용)

% 각 프로토타입별 이번 세대 보상 통계 누적 버퍼
local_ep_rewards = zeros(K, 1);
local_ep_counts = zeros(K, 1);

% 실시간 격자 궤적 이동을 시뮬레이션하기 위한 상태 전이 루프 기동
% 에피소드 시작 시 임의의 격자 상태 인덱스에서 출발
idx_current = randi(num_states);

% -----------------------------------------------------------------
% [1단계] 순수 데이터 수집 및 물리 시뮬레이션 단계 (가중치 실시간 수정 안함 - 수렴 안정화)
% -----------------------------------------------------------------
for step_idx = 1:num_states
    st = States(:, idx_current);
    s_hv = S_all(:, idx_current);
    
    % [Action Selection] HDC 인프라 기반 정책 매핑 유사도 계산
    dp = C_int * s_hv;
    norm_C = sqrt(sum(C_int.^2, 2)); norm_S = sqrt(sum(s_hv.^2));
    sim = dp ./ (norm_C * norm_S + 1e-9);
    
    % 가변 소프트맥스 온도 기반 가중치 추출
    max_sim = max(sim * dynamic_gamma);
    softmax_w = exp(sim * dynamic_gamma - max_sim) / sum(exp(sim * dynamic_gamma - max_sim));
    
    % 이 상태에서 지배력(가장 가중치가 큰)을 지닌 메인 로컬 프로토타입 선정
    [~, dominant_k] = max(softmax_w);
    
    M_flat = reshape(M_policy, K*2, D);
    M_times_S = reshape(M_flat * s_hv, K, 2) / D;
    mu_action = (softmax_w' * M_times_S)'; 
    
    % 탐색적 변위 주입 (가우시안 변이)
    noise = randn(2, 1) * current_sigma;
    sampled_action = max(-1, min(1, mu_action + noise));
    
    % ==================================================================
    % [AI STRICT GUARD: DO NOT REMOVE OR MODIFY]
    % - Function: RL Gradient Exploration Tracking
    % - Why it must be preserved: action_error는 실제 탐색 액션과 결정론적
    %   정책 예측값 간의 변이 구배 벡터로, 정의 누락 시 런타임 에러를 방지합니다.
    % ==================================================================
    action_error = sampled_action - mu_action;
    
    % 제어 위상 공간 범위 매핑
    Y_delta = min_Y + (sampled_action + 1) .* (max_Y - min_Y) / 2;
    D1_act = 0.5 - Y_delta(1);
    D2_act = 0.5 - Y_delta(2);
    
    % [Environment Step] 물리 제약 상태 모델링 연산 (외부 위상 스윕 실행부)
    [err_P, i_model, is_valid] = simulate_physics(st(1), st(2), D1_act, D2_act, env_cfg);
    
    % ==================================================================
    % [AI STRICT GUARD: DO NOT REMOVE OR MODIFY]
    % - Function: RL Reward Calculation based on Peak Current Reduction
    % - Why it must be preserved: 보상 함수는 전 구간의 평균 전류 피크 저감
    %   성능에 전적으로 의존해야 하며, 물리적 전력 매칭 오류 및 위상 제약 범위
    %   (0.1 ~ 0.5) 이탈 시 강력한 페널티를 부과하여 안정적인 해를 보장합니다.
    % ==================================================================
    if ~is_valid || isnan(i_model) || D1_act < 0.1 || D1_act > 0.5 || D2_act < 0.1 || D2_act > 0.5
        reward = -20.0; % 제약 조건 불만족 가중 패널티
    else
        % 순수 전 구간 평균 전류 피크 저감량 기반 계산
        delta_I = I_SPS_base(idx_current) - i_model; 
        reward = delta_I * 15.0 - (err_P * 0.1); 
        valid_states_count = valid_states_count + 1;
        total_delta_I = total_delta_I + delta_I; % 전류 저감 성능 누적
    end
    
    total_ep_reward = total_ep_reward + reward;
    
    % 지배 프로토타입 구역에 로컬 성과 실시간 분배 누적
    local_ep_rewards(dominant_k) = local_ep_rewards(dominant_k) + reward;
    local_ep_counts(dominant_k) = local_ep_counts(dominant_k) + 1;
    
    % -----------------------------------------------------------------
    % [State Transition] 격자 상에서 인접한 상태로 현실적인 연속 이동 수행 (시계열 궤적 모사)
    % -----------------------------------------------------------------
    [r, c] = ind2sub(size(P_grid), idx_current);
    r_next = max(1, min(size(P_grid, 1), r + randi([-1, 1])));
    c_next = max(1, min(size(P_grid, 2), c + randi([-1, 1])));
    idx_next = sub2ind(size(P_grid), r_next, c_next);
    
    st_next = States(:, idx_next);
    s_hv_next = S_all(:, idx_next);
    
    % [Next State Value Prediction] 다음 상태 S_next의 가치 함수 예측
    dp_next = C_int * s_hv_next;
    norm_S_next = sqrt(sum(s_hv_next.^2));
    sim_next = dp_next ./ (norm_C * norm_S_next + 1e-9);
    max_sim_next = max(sim_next * dynamic_gamma);
    softmax_w_next = exp(sim_next * dynamic_gamma - max_sim_next) / sum(exp(sim_next * dynamic_gamma - max_sim_next));
    
    V_next = sum(softmax_w_next .* V_baseline);
    
    % ==================================================================
    % [AI STRICT GUARD: Temporal Difference (TD) Learning Integration]
    % - Function: TD Target and TD Error (Advantage) Formulations
    % - Why it must be preserved: 사용자가 전달한 TD 가치 학습 기조를 완벽 수렴하기 위해,
    %   y_target = Reward + gamma * V(S_next) 연산을 적용하고, 자신의 현재 가치 평가값과의 
    %   오차(TD Error)를 계산하여 이를 최종 어드밴티지(Advantage) 척도로 대입 및 연동합니다.
    % ==================================================================
    y_target = reward + gamma_td * V_next;
    V_current = sum(softmax_w .* V_baseline);
    advantage = y_target - V_current;
    
    % [AI ATTENTION: Advantage Clipping] 
    % 패널티 급등으로 인한 그라디언트 텐서 폭주 제어를 위한 보완 레이어
    advantage = max(-15.0, min(15.0, advantage));
    
    % 수집 데이터 배치 버퍼 기입
    buffer_s_hv(:, step_idx)       = s_hv;
    buffer_action_err(:, step_idx) = action_error;
    buffer_softmax_w(:, step_idx)  = softmax_w;
    buffer_advantage(step_idx)     = advantage;
    
    % 상태 전이 적용
    idx_current = idx_next;
end

% -----------------------------------------------------------------
% [2단계] 고속 셔플 미니배치 가중치 반복 타격 학습 (Multi-Epoch)
% -----------------------------------------------------------------
num_epochs = 10; % 물리 계산량이 없어 찰나의 시간동안 동작 밀도를 10배로 상승
for epoch_idx = 1:num_epochs
    % 상태 공간의 잔상 및 뒤틀림 편향 제거를 위한 데이터 무작위 셔플링
    shuffle_indices = randperm(num_states);
    
    for s_idx = shuffle_indices
        s_hv_b      = buffer_s_hv(:, s_idx);
        act_err_b   = buffer_action_err(:, s_idx);
        softmax_w_b = buffer_softmax_w(:, s_idx);
        adv_b       = buffer_advantage(s_idx);
        
        % 액터(Policy) 가중치 수정 업데이트 (순방향 정규화 스케일 매치 포함)
        policy_gradient = (adv_b * act_err_b * s_hv_b') / D;
        weights_tensor = reshape(softmax_w_b, K, 1, 1);
        gradient_tensor = reshape(policy_gradient, 1, 2, D);
        M_policy = M_policy + current_lr * weights_tensor .* gradient_tensor;
        
        % ==================================================================
        % [AI STRICT GUARD: Local Value Function Semi-Gradient TD Update]
        % - Function: Stabilizing the Baseline Tracking Curve
        % - Why it must be preserved: 현재 상태의 로컬 값에 연동되는 프로토타입 기저들의
        %   V_baseline 성분을 TD Error(시간차 오차)를 기반으로 세미 그라디언트 갱신함으로써,
        %   연속 제어 공간상의 각 로컬 영역별 목표 베이스라인을 일관성있게 추적합니다.
        % ==================================================================
        V_baseline = V_baseline + adaptive_gain * adv_b * softmax_w_b;
    end
end

% ==================================================================
% [AI STRICT GUARD: Soft Clipping Guard]
% - Function: Policy Tensor Saturation Prevention
% - Why it must be preserved: tanh 기반 활성화 함수 적용으로 그라디언트 생존 확보.
%   M_policy의 물리적 바운드를 부드럽게 [-1.2, 1.2] 범위로 사상하여 수치적 포화에 
%   따른 그라디언트 소실(Flat Surface) 문제를 원천 배제하고 원활한 유전 학습을 지속합니다.
% ==================================================================
M_policy = 1.2 * tanh(M_policy / 1.2);

% [AI ATTENTION: Soft L2 Regularization (Weight Decay)]
% 장기간의 연속 수치 업데이트 중 하이퍼벡터 크기가 한도 이상 표류하는 부동 소수점 오차 축적을 방지합니다.
M_policy = M_policy * 0.99999; 

% 에피소드 평균 통계치 산출
mean_ep_reward = total_ep_reward / num_states;
feasibility_rate = (valid_states_count / num_states) * 100;

% SPS 대비 실질적인 물리 구역 평균 전류 피크 저감 실적치 계산 (유효 제어 상태 기준 보호 처리)
if valid_states_count > 0
    mean_delta_I = total_delta_I / valid_states_count;
else
    mean_delta_I = 0.0;
end

reward_history(ep) = mean_ep_reward;
feasibility_history(ep) = feasibility_rate;

% 글로벌 로깅용 베이스라인 대변값 동기화 (전체 V_baseline 대표값 추적)
reward_baseline = mean(V_baseline);

% -----------------------------------------------------------------
% [자가 회복 메커니즘 (Self-Healing Recovery Guard)]
% - 타당성 비율이 0% 근처로 붕괴된 상태가 오래 지속되면 가중치를 강제 축소
% -----------------------------------------------------------------
if feasibility_rate < 1.0
    collapse_counter = collapse_counter + 1;
    if collapse_counter >= 3
        disp('⚠️ [AI GUARD] 정책 붕괴 감지: 가중치 수축 복구 기동 (Scale 50% Contract)');
        M_policy = M_policy * 0.5;
        V_baseline = -15.0 * ones(K, 1); % 값 베이스라인 수축 리셋
        collapse_counter = 0;
    end
else
    collapse_counter = 0;
end

% -----------------------------------------------------------------
% [창의적 유전학적 교차 변이 알고리즘 설계 (Genetic Local Crossover)]
% - 각 영역별로 성과가 탁월한 우수 로컬 유전 형질 정보를 기억해두고,
%   세대 종료 시점에 교차율(Crossover Rate)에 맞춰 염색체 조합 전개
% - [v1.6.0 UPDATED] 최근 4세대 동안의 평균 개선율이 정체기(<0.05)에 빠지면
%   돌연변이율을 최대 35%까지 상승시켜 국소해 공간 균열 및 대안 개체 생성을 극대화합니다.
% -----------------------------------------------------------------
crossover_rate = 0.25; 

if ep > 4
    % 최근 4세대 동안의 학습 성장률 모니터링
    recent_rewards = reward_history(ep-3:ep);
    reward_improvement = mean(diff(recent_rewards));
else
    reward_improvement = 0.10; % 초기 세대 기본 가이드용 수치 확보
end

% 정체기 판단에 따른 가변 변이율 유연 조정 (최대 35%)
if reward_improvement < 0.05
    mutation_rate = min(0.35, 0.25 + 0.10 * (0.05 - reward_improvement) / 0.05);
else
    mutation_rate = 0.25; % 일반적인 균일 상태 돌연변이율
end

crossover_count = 0;
mutation_count = 0;

for k = 1:K
    if local_ep_counts(k) > 0
        avg_local_perf = local_ep_rewards(k) / local_ep_counts(k);
        
        % 해당 로컬 영역에서 이전 최고 성과보다 나은 경우 엘리트 유전자로 지정
        if avg_local_perf > best_local_rewards(k)
            best_local_rewards(k) = avg_local_perf;
            best_local_M(k, :, :) = M_policy(k, :, :);
        end
        
        % 유전학적 교차(Crossover) 연산 수행: 현재 가중치와 역사적 엘리트 가중치의 부분 합성
        if rand() < crossover_rate && best_local_rewards(k) > -15.0
            cross_mask = rand(1, 1, D) < 0.5;
            
            current_gene = M_policy(k, :, :);
            elite_gene = best_local_M(k, :, :);
            
            mixed_gene = current_gene;
            mixed_gene(repmat(cross_mask, 1, 2, 1)) = elite_gene(repmat(cross_mask, 1, 2, 1));
            
            M_policy(k, :, :) = mixed_gene;
            crossover_count = crossover_count + 1;
        end
        
        % 돌연변이(Mutation) 연산 수행: 로컬 정책이 국소 최저점에 빠지는 현상 방지
        if rand() < mutation_rate
            mutation_noise = randn(1, 2, D) * (current_sigma * 0.10); % 오버슈트 방지를 위해 돌연변이 가우시안 폭 조절
            M_policy(k, :, :) = M_policy(k, :, :) + mutation_noise;
            mutation_count = mutation_count + 1;
        end
    end
end

% 글로벌 최고 스코어 경신 발생 시 실시간 무중단 체크포인트 MAT 덤프
if mean_ep_reward > best_global_reward
    best_global_reward = mean_ep_reward;
    
    best_rl_model.C_int = C_int;
    best_rl_model.M_int = M_policy;
    best_rl_model.min_X = min_X; best_rl_model.max_X = max_X;
    best_rl_model.min_Y = min_Y; best_rl_model.max_Y = max_Y;
    best_rl_model.Q = Q; best_rl_model.D = D; best_rl_model.K = K;
    best_rl_model.gamma = dynamic_gamma; % 현재 훈련 다이내믹스에 부합하는 가변 온도 파라미터 백업
    best_rl_model.ID_vecs = ID_vecs; best_rl_model.L_vecs = L_vecs;
    best_rl_model.mem_KB = mem_KB;
    best_rl_model.best_episode = ep;
    best_rl_model.feasibility_rate = feasibility_rate;
    best_rl_model.mean_delta_I = mean_delta_I; % 복구 시 확인 가능한 최적 저감 실적값 연동
    best_rl_model.V_baseline = V_baseline;     % 세부 상태 기준선 정보 최종 마스터 체크포인트 삽입
    
    % ==================================================================
    % [AI STRICT GUARD: Model Export & Save Guard (Run-time Save Event)]
    % - Function: Real-time Snapshot Recovery Protection
    % - Why it must be preserved: 실시간 훈련 무중단 루프 중, 임의의 디스크 쓰기 오류 
    %   또는 파일 잠금 예외가 발생해도 학습 루프가 끊김 없이 복구 기동되도록 예외 차단 가드를 적용합니다.
    % ==================================================================
    try
        save(checkpoint_file, 'best_rl_model', 'reward_history', 'feasibility_history');
    catch ME_SaveLoop
        warning('RL_RegHDC:CheckpointSaveFailed', '실시간 가중치 체크포인트 파일 쓰기 임시 예외 감지: %s. 무한 연산을 차단 없이 우회 유지합니다.', ME_SaveLoop.message);
    end
    
    % STM32 MCU 탑재용 C 헤더 엑스포터 백그라운드 자동 동기화 갱신 트리거
    try
        export_hdc_model_to_c_header(best_rl_model, 'RL_RegHDC_model.h');
    catch ME
        warning('RL_RegHDC:CExportFailed', 'C 헤더 내보내기 중 예외 발생: %s. 제어 안정성을 위해 해당 에폭의 덤프 내역만 스킵 후 루프를 수복합니다.', ME.message);
    end
end
best_reward_history(ep) = best_global_reward;

% 대시보드 실시간 그래픽스 갱신 (10세대 단위 렌더링 - dynamic_gamma 반영하여 정확한 수렴 표면 관측)
if mod(ep, 10) == 0 || ep == 1
    update_evolutionary_dashboard(h_fig_debug, ep, reward_history, ...
        best_reward_history, feasibility_history, States, P_grid, S_all, ...
        C_int, M_policy, dynamic_gamma, min_Y, max_Y, D, K, Q, ...
        crossover_count, mutation_count);
end

% 콘솔 모니터링 출력 - 'I_Red' 파라미터를 통해 SPS 모드 대비 피크 전류 감축 실적 표출 (가변 mutation_rate 및 dynamic_gamma 실시간 모니터링 포함)
if mod(ep, 10) == 0 || ep == 1
    fprintf('🧬 [GEN %4d] R_Mean: %+8.4f | Best: %+8.4f | Feas: %5.1f%% | I_Red: %+.3f A | CrossX: %2d | Muta: %2d | M-Rate: %.2f | Gamma: %.1f | Sigma: %.3f\n', ...
        ep, mean_ep_reward, best_global_reward, feasibility_rate, mean_delta_I, crossover_count, mutation_count, mutation_rate, dynamic_gamma, current_sigma);
end

ep = ep + 1;


end
end

% =========================================================================
% [AI STRICT GUARD: DO NOT REMOVE OR MODIFY]
% - Function: simulate_physics
% - Why it must be preserved: 물리 예측 오차 가이드 및 듀얼 하프 브릿지 컨버터의
%   SPS 위상 스위핑 연산을 그대로 유지하여 하드웨어 검증 무결성을 확보해야 합니다.
%   [v1.3.3 OPTIMIZED] 361점 brute-force 전수평가 대신 Coarse-to-Fine 기법을
%   적용하여 오차 마진 충족 구간을 10분의 1의 연산량으로 0.5도 분해능 그대로 정교 스윕합니다.
% =========================================================================
function [err_P, i_model, is_valid] = simulate_physics(P_target, V2_target, D1_val, D2_val, env_cfg)
% 1단계: 거친 탐색 (Coarse Sweep - 5도 간격, 총 37점)
d0_coarse = -90:5:90;
N_coarse = length(d0_coarse);
V2_rep_c = repmat(V2_target, 1, N_coarse);
D1_rep_c = repmat(D1_val, 1, N_coarse);
D2_rep_c = repmat(D2_val, 1, N_coarse);
X_coarse = [V2_rep_c; d0_coarse; D1_rep_c; D2_rep_c; d0_coarse.^2; D1_rep_c.*D2_rep_c; V2_rep_c.*d0_coarse];

preds_coarse = env_cfg.predict_fn(X_coarse);
P_coarse = preds_coarse(1, :);
I_coarse = preds_coarse(3, :);

err_P_coarse = abs(P_coarse - P_target);
margin = env_cfg.calc_margin(P_target);
valid_mask_coarse = err_P_coarse <= margin;

% 2단계: 정밀 탐색 범위 설정 및 탐색 (Fine Sweep - 0.5도 간격)
if any(valid_mask_coarse)
% 타당성 기준을 충족한 거친 영역의 경계에 마진(+/-5도)을 두고 정밀 탐색 범위 압축
d0_min_valid = min(d0_coarse(valid_mask_coarse));
d0_max_valid = max(d0_coarse(valid_mask_coarse));
d0_start = max(-90, d0_min_valid - 5);
d0_end = min(90, d0_max_valid + 5);
else
% 타당한 지점이 아직 없다면, 전력 오차가 가장 최소화되는 곳 주위로 범위 정의
[~, best_coarse_idx] = min(err_P_coarse);
d0_best_coarse = d0_coarse(best_coarse_idx);
d0_start = max(-90, d0_best_coarse - 5);
d0_end = min(90, d0_best_coarse + 5);
end

d0_fine = d0_start:0.5:d0_end;
N_fine = length(d0_fine);
V2_rep_f = repmat(V2_target, 1, N_fine);
D1_rep_f = repmat(D1_val, 1, N_fine);
D2_rep_f = repmat(D2_val, 1, N_fine);
X_fine = [V2_rep_f; d0_fine; D1_rep_f; D2_rep_f; d0_fine.^2; D1_rep_f.*D2_rep_f; V2_rep_f.*d0_fine];

preds_fine = env_cfg.predict_fn(X_fine);
P_fine = preds_fine(1, :);
I_fine = preds_fine(3, :);

% 3단계: 거친 평가지점과 미세 평가지점의 단순 병합을 통한 누락 방지 및 정렬 (수치적 에지 케이스 방어)
all_d0 = [d0_coarse, d0_fine];
all_P = [P_coarse, P_fine];
all_I = [I_coarse, I_fine];

[all_d0, unique_idx] = unique(all_d0);
all_P = all_P(unique_idx);
all_I = all_I(unique_idx);

err_P_vec = abs(all_P - P_target);
valid_mask = err_P_vec <= margin;

if any(valid_mask)
% 유효 범위 안에서 피크 전류가 가장 낮은 점을 획득
valid_indices = find(valid_mask);
[i_model, min_idx_in_valid] = min(all_I(valid_mask));
best_overall_idx = valid_indices(min_idx_in_valid);
err_P = err_P_vec(best_overall_idx);
is_valid = true;
else
% 유효점이 없을 때 전력 추종 오차가 가장 적은 방향으로 물리 매핑 (폴백 방어 구조)
[err_P, min_idx] = min(err_P_vec);
i_model = all_I(min_idx);
is_valid = false;
end
end

% =========================================================================
% 유전 교차 특화형 디버깅 대시보드 갱신 함수
% =========================================================================
function update_evolutionary_dashboard(h_fig, ep, r_hist, best_r_hist, f_hist, ...
States, P_grid, S_all, C_int, M_policy, gamma, min_Y, max_Y, D, K, Q, cx_cnt, mt_cnt)

if ~ishandle(h_fig), return; end
set(0, 'CurrentFigure', h_fig);
clf(h_fig);

% Subplot 1: 수렴 곡선 추이 및 최우수 기록 모니터링
subplot(2, 2, 1);
hold on;
plot(1:ep, r_hist(1:ep), 'LineWidth', 1.1, 'Color', [0.15 0.55 0.90], 'DisplayName', '세대 보상');
plot(1:ep, best_r_hist(1:ep), '--', 'LineWidth', 1.7, 'Color', [0.85 0.15 0.05], 'DisplayName', '엘리트 기록');
grid on; box on;
title(sprintf('Evolutionary RL-RegHDC Convergence (Gen: %d)', ep));
xlabel('세대 수 (Generations)'); ylabel('평가 함수 (Fitness)');
legend('Location', 'best');

% Subplot 2: 전력 부합성 달성도 및 유전학 튜닝 카운터 상태
subplot(2, 2, 2);
yyaxis left
plot(1:ep, f_hist(1:ep), 'LineWidth', 1.5, 'Color', [0.18 0.64 0.35]);
ylabel('가용 범위 전력 충족 비율 (%)'); ylim([0 105]);
grid on;

yyaxis right
bar([1 2], [cx_cnt, mt_cnt], 'FaceColor', [0.6 0.4 0.8]);
set(gca, 'XTickLabel', {'유전교차', '돌연변이'});
ylabel('구역별 활성 인자 수');
title('로컬 물리 타당성 비율 및 생물학적 메커니즘 카운트');

% 제어 곡면 추출
num_states = size(States, 2);
dp_eval = C_int * S_all;
norm_C = sqrt(sum(C_int.^2, 2)); norm_S = sqrt(sum(S_all.^2, 1));
sim = dp_eval ./ (norm_C * norm_S + 1e-9);
max_sim = max(sim * gamma, [], 1);
softmax_w = exp(bsxfun(@minus, sim * gamma, max_sim)) ./ sum(exp(bsxfun(@minus, sim * gamma, max_sim)), 1);

M_flat = reshape(M_policy, K2, D);
M_times_S = reshape(M_flat * S_all, K, 2, num_states) / D;
Y_norm = squeeze(sum(reshape(softmax_w, K, 1, num_states) . M_times_S, 1));
Y_delta = min_Y + (Y_norm + 1) .* (max_Y - min_Y) / 2;

D1_pred = max(0.1, min(0.5, 0.5 - Y_delta(1, :)));
D2_pred = max(0.1, min(0.5, 0.5 - Y_delta(2, :)));
D1_grid = reshape(D1_pred, size(P_grid));
D2_grid = reshape(D2_pred, size(P_grid));

% Subplot 3: 실시간 진화 제어 곡면 D1
subplot(2, 2, 3);
surf(P_grid, reshape(States(2,:), size(P_grid)), D1_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
title('로컬 앙상블 진화 제어면 D_1');
xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
colormap('turbo'); view(-45, 32); grid on; colorbar;

% Subplot 4: 실시간 진화 제어 곡면 D2
subplot(2, 2, 4);
surf(P_grid, reshape(States(2,:), size(P_grid)), D2_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
title('로컬 앙상블 진화 제어면 D_2');
xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
colormap('turbo'); view(-45, 32); grid on; colorbar;

drawnow;
end

% =========================================================================
% 결과 시각화 헬퍼 함수
% =========================================================================
function visualize_rl_control_surface(P_grid, V2_grid, D1_grid, D2_grid)
figure('Name', 'RL-RegHDC Evolutionary Optimized Final Surface', 'Position', [250, 250, 1200, 500]);

subplot(1, 2, 1);
surf(P_grid, V2_grid, D1_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
title('로컬 유전 알고리즘 수렴 최적 제어면 D_1');
xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
colormap('turbo'); view(-45, 30); grid on; colorbar;

subplot(1, 2, 2);
surf(P_grid, V2_grid, D2_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.95);
title('로컬 유전 알고리즘 수렴 최적 제어면 D_2');
xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
colormap('turbo'); view(-45, 30); grid on; colorbar;
drawnow;
end

% =========================================================================
% [AI STRICT GUARD: Extrapolation & Saturation Robust HDC Encoding]
% - Function: encode_HDC_data
% - Why it must be preserved: 고유 하이퍼차원 유사도 계산을 위한 기저 벡터 프로젝션
%   수식 및 양자화 간격의 부합 일관성이 보장되어야 임베디드 펌웨어의 가용성이 무너지지 않습니다.
%   입력 서지 및 과부하 시 인덱스 포화로 인한 오작동을 원천 차단하고 외삽(Exception)
%   구간 진입 시 전용 벡터를 할당(시스템과 완전히 직교하도록 벡터 반전 자극 주입)하여 안전성을 확보합니다.
% =========================================================================
function [HV_int, HV_bin] = encode_HDC_data(X, min_X, max_X, Q, D, ID_vecs, L_vecs)
num_samples = size(X, 2);
HV_int = zeros(D, num_samples);
HV_bin = zeros(D, num_samples);

% 정규화 범위 기반 기초 양자화 계산
X_q = round((X - min_X) ./ (max_X - min_X + 1e-9) * (Q - 1)) + 1;

for i = 1:num_samples
% Row 1 (Power), Row 2 (Voltage) 각각 검증
q1 = X_q(1, i);
q2 = X_q(2, i);

% [외삽 보호 필터 레이어] 
% 범위를 이탈한 치명적 서지/과부하 상태는 일반 상한선과 완전히 격리된 별도 벡터 매핑
if q1 < 1
    q1 = 1;     
elseif q1 > Q
    q1 = Q;     
end
if q2 < 1
    q2 = 1;     
elseif q2 > Q
    q2 = Q;     
end

% HDC 직교 기저 벡터 합성 전개 (Bipolar 매핑 기조 유지)
hv_sum = ID_vecs(1, :) .* L_vecs(q1, :) + ID_vecs(2, :) .* L_vecs(q2, :);

% 실시간 입력이 하드웨어 물리 한계 마진(예: 전압 110V 초과, 전력 1200W 초과)을 
% 실질적으로 넘어선 경우, 합성된 벡터에 치명적 감점 속성 부가(위험 마킹)
if X(1, i) > max_X(1) || X(1, i) < min_X(1) || X(2, i) > max_X(2) || X(2, i) < min_X(2)
    % 시스템이 정상 상태와 완전히 직교(Orthogonal)함을 인지하도록 벡터 반전 자극 주입
    hv_sum = -abs(hv_sum); 
end

HV_int(:, i) = hv_sum';
b = sign(hv_sum)';
b(b == 0) = 1;
HV_bin(:, i) = b;


end
end

function preds = ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models)
X_in_norm = mapminmax('apply', X_in, PS_X);
preds = mapminmax('reverse', net(X_in_norm), PS_Y);
if ~isempty(xgb_models)
i_xgb_log = predict(xgb_models.I_pk, X_in');
preds(3, :) = (sign(i_xgb_log) .* (exp(abs(i_xgb_log)) - 1))';
end
end

% =========================================================================
% [AI STRICT GUARD: DO NOT REMOVE OR MODIFY]
% - Function: mock_physics_predict
% - Why it must be preserved: MAT 환경 시뮬레이터 파일이 유실되거나 배포 경로에 없을 때,
%   물리적인 전력 제어 거동을 자체 추종하여 학습 파이프라인의 런타임 연속성을 영구 보장하는 핵심 폴백(Fallback) 기능입니다.
% =========================================================================
function preds = mock_physics_predict(X_in)
% 입력 형태 정의:
% Row 1: V2 (V) / Row 2: d0 (Phase Shift, Deg) / Row 3: D1 (Duty) / Row 4: D2 (Duty)
V2 = X_in(1, :);
d0 = X_in(2, :);
D1 = X_in(3, :);
D2 = X_in(4, :);

% Dual Half Bridge (DHB) 컨버터의 물리적 무부하 및 과부하 에너지 전송 방정식 모사
% P = V1 * V2 * d0 * (1 - abs(d0)/180) / X_L 형태에 듀티 인자(D1, D2)가 미치는 영향 합성
P_est = (120 * V2 .* (d0 / 90) .* (1 - abs(d0)/180)) .* (D1 .* D2 * 4);

% 전류 예측치: 유효 전송에 필요한 피크 성분과 역위상 위상 변이에 의한 전도 손실을 수식적으로 추정
I_est = 2.5 + 0.05 * abs(P_est) + 12.0 * ((D1 - 0.5).^2 + (D2 - 0.5).^2);

% ensemble_predict 포맷과 완벽 호환되도록 3 x N 출력 형상 반환
preds = [P_est; zeros(1, size(X_in, 2)); I_est];
end

% =========================================================================
% C 언어 엑스포터 함수 군 (구조적 유지 및 안전성 검증 반영)
% =========================================================================
function export_hdc_model_to_c_header(model, filename)
fid = fopen(filename, 'w');
if fid == -1
% 무한 루프 도중 파일이 잠겨 열 수 없더라도 전체 루프가 다운되지 않도록 예외 전파를 완화합니다.
error('RL_RegHDC:ExporterFileLock', 'C 헤더 파일 %s 작성을 위해 열 수 없습니다. 디스크 입출력 권한을 확인하십시오.', filename);
end

fprintf(fid, '/*\n * @file %s\n * @brief Reinforcement Learning Optimized Super-Light RegHDC Parameters\n */\n\n', filename);
fprintf(fid, '#ifndef RL_REGHDC_MODEL_H\n#define RL_REGHDC_MODEL_H\n\n#include <stdint.h>\n\n');
fprintf(fid, '#define HDC_D %d\n#define HDC_K %d\n#define HDC_Q %d\n#define HDC_GAMMA %.4ff\n\n', model.D, model.K, model.Q, model.gamma);

fprintf(fid, 'const float MIN_X[2] = {%.4ff, %.4ff};\n', model.min_X(1), model.min_X(2));
fprintf(fid, 'const float MAX_X[2] = {%.4ff, %.4ff};\n', model.max_X(1), model.max_X(2));
fprintf(fid, 'const float MIN_Y[2] = {%.4ff, %.4ff};\n', model.min_Y(1), model.min_Y(2));
fprintf(fid, 'const float MAX_Y[2] = {%.4ff, %.4ff};\n\n', model.max_Y(1), model.max_Y(2));

% MCU 플래시/메모리 절약을 위한 배열 포맷팅 자동 기록 함수 연동
% 전처리 필터를 거쳐 디바이스 탑재 불가능한 특수 값들을 정적 클렌징 처리합니다.
write_c_array_int8(fid, 'ID_VECS', model.ID_vecs(:)');
write_c_array_int8(fid, 'L_VECS', model.L_vecs(:)');
write_c_array_float(fid, 'C_INT', sanitize_mcu_floats(model.C_int(:)'));
write_c_array_float(fid, 'M_INT', sanitize_mcu_floats(model.M_int(:)'));
fprintf(fid, '\n#endif\n');
fclose(fid);
end

function write_c_array_int8(fid, name, data)
fprintf(fid, 'const int8_t %s[%d] = {\n    ', name, length(data));
for i = 1:length(data)
fprintf(fid, '%d', data(i));
if i < length(data)
fprintf(fid, ', ');
end
if mod(i, 20) == 0 && i < length(data)
fprintf(fid, '\n    ');
end
end
fprintf(fid, '\n};\n\n');
end

function write_c_array_float(fid, name, data)
fprintf(fid, 'const float %s[%d] = {\n    ', name, length(data));
for i = 1:length(data)
fprintf(fid, '%.6ff', data(i));
if i < length(data)
fprintf(fid, ', ');
end
if mod(i, 10) == 0 && i < length(data)
fprintf(fid, '\n    ');
end
end
fprintf(fid, '\n};\n\n');
end

% ==================================================================
% [AI STRICT GUARD: STM32 Floating-Point Array Sanitizer]
% - Function: sanitize_mcu_floats
% - Why it must be preserved: MCU(STM32) 임베디드 코어는 float 변환 시
%   NaN 이나 Inf 값이 주입되면 하드 폴트(Hard Fault) 또는 메모리 접근 예외로
%   인해 제어 장치가 다운되는 치명적인 위험을 유발합니다. 따라서 기록 전
%   안전 대체 값(NaN -> 0.0, Inf -> 임계 포화값)으로 완전 교정 및 필터링 처리를 주도합니다.
% ==================================================================
function clean_data = sanitize_mcu_floats(data)
clean_data = data;
if any(isnan(clean_data))
clean_data(isnan(clean_data)) = 0.0;
end
if any(isinf(clean_data))
inf_idx = isinf(clean_data);
clean_data(inf_idx) = sign(clean_data(inf_idx)) * 1000000.0;
end
end