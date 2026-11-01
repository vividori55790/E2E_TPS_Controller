% ======================================================================
% [DIAGNOSTIC PIPELINE - TARGET SURFACE IMITATION TEST]
% - 목적: 현 파라미터 구조(K=24, D=256)가 찢어진 곡면을 표현할 수 있는지 역검증
% ======================================================================
function train_RL_RegHDC_Diagnostic()
clear; clc; close all;
disp('===========================================================');
disp('[진단 모드] 임의의 가상 최적 곡면(저부하 0.1 ~ 고부하 0.5) 강제 학습');
disp('===========================================================');

% 기존 파라미터 구조 100% 동일 유지
D = 256; K = 24; Q = 48;
gamma_softmax = 24.0;

% 상태 공간 생성 (기존과 동일)
P_vec = -1200:100:1200;
V2_vec = 80:2:110;
[P_grid, V2_grid] = meshgrid(P_vec, V2_vec);
States = [P_grid(:)'; V2_grid(:)'];
num_states = size(States, 2);

min_X = [-1200; 80];  max_X = [1200; 110]; 
min_Y = [0.0; 0.0];   max_Y = [0.4; 0.4]; 

% HDC 기저 공간 초기화
rng(42); % 재현성을 위한 고정 seed
ID_vecs = sign(rand(2, D) - 0.5); ID_vecs(ID_vecs == 0) = 1;
L_vecs = zeros(Q, D); L_vecs(1, :) = sign(rand(1, D) - 0.5); L_vecs(1, L_vecs(1, :) == 0) = 1;
flip_counts = max(1, floor(D / 2 / (Q - 1)));
for q_idx = 2:Q
    L_vecs(q_idx, :) = L_vecs(q_idx-1, :); 
    flip_idx = randperm(D, flip_counts); 
    L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx); 
end

C_int = zeros(K, D);
init_idx = round(linspace(1, num_states, K));
[S_all, ~] = encode_HDC_data(States, min_X, max_X, Q, D, ID_vecs, L_vecs);
for k_init = 1:K
    C_int(k_init, :) = S_all(:, init_idx(k_init))';
end

% 기존과 동일하게 zeros로 출발하여 평평한 면을 깨고 나올 수 있는지 검증
M_policy = zeros(K, 2, D); 

% 학습 파라미터 (기존 로직 유지)
exploration_sigma_init = 0.65;
exploration_sigma_min  = 0.25;
lr_policy_init = 0.05;
lr_policy_min  = 0.005;

h_fig_debug = figure('Name', 'HDC Diagnostic Surface Imitation Monitor', 'Position', [100, 100, 1200, 600]);
[S_all, ~] = encode_HDC_data(States, min_X, max_X, Q, D, ID_vecs, L_vecs);

% -----------------------------------------------------------------
% [가상 타깃 곡면 함수 정의]
% - 에이전트가 학습해야 할 정답 (저부하에선 0.1, 고부하에선 0.5로 벌어지는 면)
% -----------------------------------------------------------------
% 부하(P)가 -1200일 때 0.1, +1200일 때 0.5가 되는 선형 방정식
target_duty_fn = @(p) 0.1 + 0.4 * ((p - (-1200)) / 2400);

for ep = 1:3000 % 빠른 진단을 위해 300세대 전개
    cycle_period = 100;
    cycle_phase = mod(ep, cycle_period) / cycle_period;
    current_sigma = exploration_sigma_min + 0.5 * (exploration_sigma_init - exploration_sigma_min) * (1 + cos(pi * cycle_phase));
    current_lr = lr_policy_min + 0.5 * (lr_policy_init - lr_policy_min) * (1 + cos(pi * cycle_phase));
    
    episode_indices = randperm(num_states);
    total_error = 0;
    
    for idx = episode_indices
        st = States(:, idx);
        s_hv = S_all(:, idx);
        
        % 정답 매핑 (우리가 강제로 보고 싶어하는 벌어진 곡면의 듀티값)
        D_target = target_duty_fn(st(1)); 
        
        % HDC 출력 연산
        dp = C_int * s_hv;
        norm_C = sqrt(sum(C_int.^2, 2)); norm_S = sqrt(sum(s_hv.^2));
        sim = dp ./ (norm_C * norm_S + 1e-9);
        max_sim = max(sim * gamma_softmax);
        softmax_w = exp(sim * gamma_softmax - max_sim) / sum(exp(sim * gamma_softmax - max_sim));
        
        M_flat = reshape(M_policy, K*2, D);
        M_times_S = reshape(M_flat * s_hv, K, 2) / D;
        mu_action = (softmax_w' * M_times_S)'; 
        
        % 기존 탐색 가우시안 노이즈 주입
        noise = randn(2, 1) * current_sigma;
        sampled_action = max(-1, min(1, mu_action + noise));
        action_error = sampled_action - mu_action;
        
        % 실제 스케일 변환
        Y_delta = min_Y + (sampled_action + 1) .* (max_Y - min_Y) / 2;
        D1_act = 0.5 - Y_delta(1);
        
        % -----------------------------------------------------------------
        % [진단용 가상 보상 함수]
        % - 기존 수식 구조(R = delta_I * 15)를 모사하기 위해, 
        % - 정답 듀티와의 오차를 기반으로 기존과 비슷한 크기의 보상 스케일(-20 ~ 0) 형성
        % -----------------------------------------------------------------
        duty_err = abs(D1_act - D_target);
        reward = -duty_err * 50.0; % 오차가 클수록 강력한 페널티
        total_error = total_error + duty_err;
        
        % 기존 가중치 업데이트 메커니즘 100% 동일 유지
        % baseline 역시 기존과 동일하게 -15 근처에서 수렴하도록 처리
        reward_baseline = -5.0; 
        advantage = reward - reward_baseline;
        advantage = max(-15.0, min(15.0, advantage)); % 기존 클리핑 가드 그대로
        
        policy_gradient = (advantage * action_error * s_hv') / D;
        weights_tensor = reshape(softmax_w, K, 1, 1);
        gradient_tensor = reshape(policy_gradient, 1, 2, D);
        M_policy = M_policy + current_lr * weights_tensor .* gradient_tensor;
    end
    
    % 기존 tanh soft clipping 가드 적용
    M_policy = 1.2 * tanh(M_policy / 1.2);
    
    % 10세대 단위 모니터링 출력 및 곡면 시각화
    if mod(ep, 10) == 0 || ep == 1
        mean_err = total_error / num_states;
        fprintf('🧪 [진단 세대 %3d] 정답 곡면과의 평균 듀티 오차: %.4f (Sigma: %.3f)\n', ep, mean_err, current_sigma);
        
        % 실시간 곡면 연산 및 플롯
        clf(h_fig_debug);
        dp_eval = C_int * S_all;
        sim_eval = dp_eval ./ (sqrt(sum(C_int.^2,2)) * sqrt(sum(S_all.^2,1)) + 1e-9);
        max_sim_e = max(sim_eval * gamma_softmax, [], 1);
        sw_e = exp(bsxfun(@minus, sim_eval * gamma_softmax, max_sim_e)) ./ sum(exp(bsxfun(@minus, sim_eval * gamma_softmax, max_sim_e)), 1);
        M_times_S_e = reshape(reshape(M_policy, K*2, D) * S_all, K, 2, num_states) / D;
        Y_norm = squeeze(sum(reshape(sw_e, K, 1, num_states) .* M_times_S_e, 1));
        Y_delta_e = min_Y + (Y_norm + 1) .* (max_Y - min_Y) / 2;
        D1_pred = max(0.1, min(0.5, 0.5 - Y_delta_e(1, :)));
        
        subplot(1,2,1);
        surf(P_grid, reshape(States(2,:), size(P_grid)), reshape(D1_pred, size(P_grid)), 'EdgeColor', 'none');
        title(sprintf('현재 에이전트가 구현한 곡면 (Gen: %d)', ep)); xlabel('Power'); ylabel('Voltage'); zlim([0.0 0.6]); colormap('turbo'); view(-45, 32); grid on;
        
        subplot(1,2,2);
        T_grid = arrayfun(target_duty_fn, P_grid);
        surf(P_grid, V2_grid, T_grid, 'EdgeColor', 'none', 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.5);
        title('우리가 강제로 주입한 정답 곡면 목표'); xlabel('Power'); ylabel('Voltage'); zlim([0.0 0.6]); view(-45, 32); grid on;
        drawnow;
    end
end
end

% 기존 인코딩 함수 (100% 동일)
function [HV_int, HV_bin] = encode_HDC_data(X, min_X, max_X, Q, D, ID_vecs, L_vecs)
num_samples = size(X, 2); HV_int = zeros(D, num_samples); HV_bin = zeros(D, num_samples);
X_q = round((X - min_X) ./ (max_X - min_X + 1e-9) * (Q - 1)) + 1;
for i = 1:num_samples
    q1 = X_q(1, i); q2 = X_q(2, i);
    if q1 < 1, q1 = 1; elseif q1 > Q, q1 = Q; end
    if q2 < 1, q2 = 1; elseif q2 > Q, q2 = Q; end
    hv_sum = ID_vecs(1, :) .* L_vecs(q1, :) + ID_vecs(2, :) .* L_vecs(q2, :);
    HV_int(:, i) = hv_sum'; b = sign(hv_sum)'; b(b == 0) = 1; HV_bin(:, i) = b;
end
end