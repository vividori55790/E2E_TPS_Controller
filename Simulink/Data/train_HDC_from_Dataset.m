% =========================================================================
% [데이터 정제 기반 초고속 HDC 파이프라인 v4]
% 핵심 1: Phase 3(능동 학습) 제거 -> "이상치 제거 및 스무딩된 데이터셋" 집중 학습
% 핵심 2: [초고속화] 중첩 for 루프를 다차원 행렬 연산(Vectorization)으로 전면 교체
% =========================================================================
function run_RegHDC_DataSmoothing_v4()
    disp('===========================================================');
    disp('[Phase 0] 시스템 초기화 및 모델 빌드');
    disp('===========================================================');
    
    mlp_file = 'trained_fast_mlp_net_v6.mat';
    xgb_file = 'trained_xgboost_models_v6.mat';
    if ~isfile(mlp_file), error('MLP 모델이 없습니다. 먼저 학습하세요.'); end
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    
    xgb_models = []; 
    if isfile(xgb_file), load(xgb_file, 'xgb_models'); end
    fast_ensemble_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
    
    p_margin = 5.0; 
    I_rmse_margin = 1.5; 

    % -------------------------------------------------------------
    % [Phase 1] 전역 최적해 데이터셋 추출
    % -------------------------------------------------------------
    global_dataset_file = 'Candidate_Dataset_Global_v3.csv'; 
    if ~isfile(global_dataset_file)
        disp('-> [신규 추출] 병렬 연산(parfor)으로 빠르게 데이터를 추출합니다.');
        dataset_tbl = extract_Global_Optima_Dataset(fast_ensemble_predict, p_margin, I_rmse_margin);
        writetable(dataset_tbl, global_dataset_file);
    else
        disp(['-> 기존 데이터셋 로드 완료: ', global_dataset_file]);
        dataset_tbl = readtable(global_dataset_file);
    end

    [unq_coords, ~, ic] = unique([dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V], 'rows');
    clean_D1 = accumarray(ic, dataset_tbl.Opt_D1, [], @mean);
    clean_D2 = accumarray(ic, dataset_tbl.Opt_D2, [], @mean);
    dataset_tbl = array2table([unq_coords(:,1), unq_coords(:,2), clean_D1, clean_D2], ...
        'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2'});

    % -------------------------------------------------------------
    % [Phase 1.5] 데이터 정제 (이상치 제거 및 스무딩)
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 1.5] 데이터 정제: 튀는 비정상 포인트 배제 및 곡면 보간');
    disp('===========================================================');
    
    smoothed_tbl = smooth_dataset_robust(dataset_tbl);
    
    fig_dataset = figure('Name', 'Data Cleansing Comparison (D1 & D2)', 'Position', [100, 100, 1400, 900]);
    P_grid = -1200:50:1200; V2_grid = 80:2:110;
    [P_mesh, V2_mesh] = meshgrid(P_grid, V2_grid);
    
    Raw_D1 = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D1, P_mesh, V2_mesh, 'natural');
    Smooth_D1 = griddata(smoothed_tbl.Target_P_W, smoothed_tbl.Target_V2_V, smoothed_tbl.Opt_D1, P_mesh, V2_mesh, 'natural');
    Raw_D2 = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D2, P_mesh, V2_mesh, 'natural');
    Smooth_D2 = griddata(smoothed_tbl.Target_P_W, smoothed_tbl.Target_V2_V, smoothed_tbl.Opt_D2, P_mesh, V2_mesh, 'natural');
    
    subplot(2, 2, 1); surf(P_mesh, V2_mesh, Raw_D1, 'EdgeColor', 'none'); 
    title('Before: D1 원본'); grid on; colormap(gca, 'parula'); view(45, 30); clim([0.1 0.5]);
    subplot(2, 2, 2); surf(P_mesh, V2_mesh, Smooth_D1, 'EdgeColor', 'none'); 
    title('After: D1 정제된 곡면'); grid on; colormap(gca, 'parula'); view(45, 30); clim([0.1 0.5]);
    subplot(2, 2, 3); surf(P_mesh, V2_mesh, Raw_D2, 'EdgeColor', 'none'); 
    title('Before: D2 원본'); grid on; colormap(gca, 'turbo'); view(45, 30); clim([0.1 0.5]);
    subplot(2, 2, 4); surf(P_mesh, V2_mesh, Smooth_D2, 'EdgeColor', 'none'); 
    title('After: D2 정제된 곡면'); grid on; colormap(gca, 'turbo'); view(45, 30); clim([0.1 0.5]);
    drawnow;

    % -------------------------------------------------------------
    % [Phase 2] 정제된 데이터 기반 집중 학습 (Passive Batch Learning)
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 2] 정제된 데이터셋으로 HDC 집중 학습 (루프 벡터화 초고속 적용)');
    disp('===========================================================');
    
    % [속도 최적화] 데이터가 이미 부드러우므로 과도한 차원수/클러스터수 다이어트
    D = 5000;      % 15000 -> 5000 (충분함)
    Q = 1000; 
    gamma = 1.2; 
    mode = 'Multi_Interact';
    K_init = 64;   % 128 -> 64 (메모리 및 연산 2배 감소)
    min_X = [-1200; 80]; max_X = [1200; 110]; 
    min_Y = [0.1; 0.1]; max_Y = [0.5; 0.5]; 
    
    rng(42);
    ID_vecs = randsrc(2, D, [-1, 1]);
    L_vecs = zeros(Q, D); L_vecs(1, :) = randsrc(1, D, [-1, 1]);
    flip_counts = max(1, floor(D / 2 / (Q - 1)));
    for q_idx = 2:Q
        L_vecs(q_idx, :) = L_vecs(q_idx-1, :);
        flip_idx = randperm(D, flip_counts);
        L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx);
    end
    
    X_base = [smoothed_tbl.Target_P_W'; smoothed_tbl.Target_V2_V'];
    Y_base_2d = [smoothed_tbl.Opt_D1'; smoothed_tbl.Opt_D2'];
    
    [S_hist_int, S_hist_bin] = encode_HDC_data(X_base, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
    Y_true_norm = 2 * ((Y_base_2d - min_Y) ./ (max_Y - min_Y)) - 1; 
    
    C_int = zeros(K_init, D); M_int = zeros(K_init, 2, D); 
    init_idx = randperm(size(X_base, 2), min(size(X_base, 2), K_init));
    for k = 1:length(init_idx), C_int(k, :) = S_hist_int(:, init_idx(k))'; end
    C_bin = sign(C_int); C_bin(C_bin == 0) = 1; 
    
    epochs_base = 1000; 
    fig_learn = figure('Name', 'HDC Learning Progress', 'Position', [100, 100, 800, 600]);
    
    tic; % 학습 시간 측정 시작
    for ep = 1:epochs_base
        lr_hdc = 0.05 * (1 + cos(pi * ep / epochs_base)) / 2 + 0.005; 
        
        shuffle_idx = randperm(size(X_base, 2));
        for b = 1:length(shuffle_idx)
            idx = shuffle_idx(b);
            S_curr_bin = S_hist_bin(:, idx); S_curr_int = S_hist_int(:, idx);
            Y_curr = Y_true_norm(:, idx);
            
            % 클러스터 유사도 계산
            sim = (C_bin * S_curr_bin) / D;
            [max_sim, k_star] = max(sim);
            C_int(k_star, :) = C_int(k_star, :) + (1 - max_sim) * S_curr_int';
            
            max_sim_val = max(sim * gamma);
            delta_prime = exp(sim * gamma - max_sim_val) / sum(exp(sim * gamma - max_sim_val)); % [K_init x 1]
            
            % ========================================================
            % [초고속 벡터화 1] 예측 과정 for 루프 제거 (행렬 곱셈 대체)
            % ========================================================
            M_flat = reshape(M_int, K_init*2, D);
            M_times_S = reshape(M_flat * S_curr_bin, K_init, 2) / D; % [K_init x 2] 행렬
            Y_hat_norm = (delta_prime' * M_times_S)'; % [2 x 1] 예측값 단번에 산출
            
            Error = Y_curr - Y_hat_norm; % [2 x 1] 오차
            
            % ========================================================
            % [초고속 벡터화 2] 모델 업데이트 for 루프 제거 (브로드캐스팅 대체)
            % ========================================================
            Err_S = Error * S_curr_int'; % [2 x D]
            weights = lr_hdc * delta_prime; % [K_init x 1]
            % 3차원 브로드캐스팅을 통해 128번의 루프를 단 1줄의 연산으로 처리
            M_int = M_int + reshape(weights, K_init, 1, 1) .* reshape(Err_S, 1, 2, D); 
        end
        
        C_bin = sign(C_int); C_bin(C_bin == 0) = 1; 
        
        if mod(ep, 20) == 0
            elapsed_time = toc;
            fprintf(' -> 집중 학습 %d/%d 완료 (현재 lr: %.4f) - 소요시간: %.1f초\n', ep, epochs_base, lr_hdc, elapsed_time);
            update_Realtime_UI(fig_learn, C_bin, M_int, min_X, max_X, min_Y, max_Y, Q, D, gamma, mode, K_init, ID_vecs, L_vecs, ep);
            tic; % 다음 구간 시간 측정
        end
    end
    
    hdc_model = struct('C_bin', C_bin, 'C_int', C_int, 'M_int', M_int, ...
                       'min_X', min_X, 'max_X', max_X, 'min_Y', min_Y, 'max_Y', max_Y, ...
                       'Q', Q, 'D', D, 'gamma', gamma, 'mode', mode, 'num_clusters', K_init, ...
                       'ID_vecs', ID_vecs, 'L_vecs', L_vecs);
    
    save('trained_smoothed_hdc_model_v4.mat', 'hdc_model');
    disp('===========================================================');
    disp(' -> 학습 완료 및 모델 저장: trained_smoothed_hdc_model_v4.mat');
    disp('===========================================================');
end

function smoothed_tbl = smooth_dataset_robust(tbl)
    smoothed_tbl = tbl;
    v2_list = unique(tbl.Target_V2_V);
    
    for i = 1:length(v2_list)
        idx = tbl.Target_V2_V == v2_list(i);
        sub_tbl = tbl(idx, :);
        [~, sort_idx] = sort(sub_tbl.Target_P_W);
        p_sorted = sub_tbl.Target_P_W(sort_idx);
        d1_sorted = sub_tbl.Opt_D1(sort_idx);
        d2_sorted = sub_tbl.Opt_D2(sort_idx);
        
        d1_clean = movmedian(d1_sorted, 5);
        d2_clean = movmedian(d2_sorted, 5);
        d1_smooth = movmean(d1_clean, 5);
        d2_smooth = movmean(d2_clean, 5);
        
        unsort_idx(sort_idx) = 1:length(sort_idx);
        smoothed_tbl.Opt_D1(idx) = d1_smooth(unsort_idx);
        smoothed_tbl.Opt_D2(idx) = d2_smooth(unsort_idx);
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

function dataset_tbl = extract_Global_Optima_Dataset(fast_predict, margin, rmse_margin)
    target_V2_list = 80:10:110; power_step = 100;            
    P_seq_pos = 0:power_step:1200; P_seq_neg = 0:-power_step:-1200;
    all_V2 = []; all_P = [];
    for v_idx = 1:length(target_V2_list)
        all_V2 = [all_V2, repmat(target_V2_list(v_idx), 1, length(P_seq_pos) + length(P_seq_neg))];
        all_P = [all_P, P_seq_pos, P_seq_neg];
    end
    total_pts = length(all_V2);
    D1_all = zeros(total_pts, 1); D2_all = zeros(total_pts, 1);
    parfor i = 1:total_pts
        [D1_all(i), D2_all(i), ~] = find_Grid_Global_Optimum(all_P(i), all_V2(i), fast_predict, margin);
    end
    dataset_tbl = array2table([all_P', all_V2', D1_all, D2_all], 'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2'});
    dataset_tbl = unique(dataset_tbl, 'rows');
end

function [opt_D1, opt_D2, opt_I] = find_Grid_Global_Optimum(P_t, V2_t, fast_predict, margin)
    lb_2d = [0.1, 0.1]; ub_2d = [0.5, 0.5];
    pso_opt = optimoptions('particleswarm', 'Display', 'none', 'SwarmSize', 100, 'MaxIterations', 30, 'UseVectorized', true);
    obj_fun = @(p2d) costFunction_Active_2D_Vec_Multi_D0(p2d, V2_t, P_t, fast_predict, margin);
    [Y_opt, opt_I] = particleswarm(obj_fun, 2, lb_2d, ub_2d, pso_opt);
    opt_D1 = Y_opt(1); opt_D2 = Y_opt(2);
end

function cost = costFunction_Active_2D_Vec_Multi_D0(p2d, V2, P_target, fast_predict, margin)
    N = size(p2d, 1); D1 = p2d(:, 1)'; D2 = p2d(:, 2)';
    if P_target > 0, d0_c = 0:10:90; else, d0_c = -90:10:0; end
    M_c = length(d0_c);
    V2_c = repmat(V2, M_c, N); D1_c = repmat(D1, M_c, 1); D2_c = repmat(D2, M_c, 1); D0_c = repmat(d0_c', 1, N);
    preds_c = fast_predict([V2_c(:), D0_c(:), D1_c(:), D2_c(:), D0_c(:).^2, D1_c(:).*D2_c(:), V2_c(:).*D0_c(:)]');
    P_preds_c = reshape(preds_c(1,:), M_c, N);
    I_pk_c = reshape(preds_c(3,:), M_c, N);
    err_c = abs(P_preds_c - P_target);
    valid_mask = err_c <= margin;
    valid_cost = I_pk_c + 4.0 * abs(D1_c - D2_c);
    valid_cost(~valid_mask) = inf; 
    [cost, ~] = min(valid_cost, [], 1); cost = cost(:);
    invalid = isinf(cost);
    if any(invalid)
        [min_err, min_err_idx] = min(err_c(:, invalid), [], 1);
        idx_list = find(invalid);
        for k = 1:length(idx_list)
            cost(idx_list(k)) = (min_err(k) - margin) * 5000 + I_pk_c(min_err_idx(k), idx_list(k));
        end
    end
end

function [HV_int, HV_bin] = encode_HDC_data(X, min_X, max_X, Q, D, ID_vecs, L_vecs, mode)
    HV_int = zeros(D, size(X, 2)); HV_bin = zeros(D, size(X, 2));
    X_q = max(1, min(Q, round((X - min_X) ./ (max_X - min_X + 1e-9) * (Q - 1)) + 1));
    for i = 1:size(X, 2)
        hv_sum = ID_vecs(1, :) .* L_vecs(X_q(1, i), :) + ID_vecs(2, :) .* L_vecs(X_q(2, i), :);
        if strcmp(mode, 'Multi_Interact')
            hv_sum = hv_sum + (ID_vecs(1, :) .* L_vecs(X_q(1, i), :)) .* (ID_vecs(2, :) .* L_vecs(X_q(2, i), :)) ...
                            + circshift(L_vecs(X_q(1, i), :), 1) .* circshift(L_vecs(X_q(2, i), :), -1);
        end
        HV_int(:, i) = hv_sum'; b = sign(hv_sum)'; b(b == 0) = 1; HV_bin(:, i) = b;
    end
end

function update_Realtime_UI(fig, C_bin, M_int, min_X, max_X, min_Y, max_Y, Q, D, gamma, mode, num_clusters, ID_vecs, L_vecs, ep)
    figure(fig); clf;
    [P_mesh, V2_mesh] = meshgrid(-1200:50:1200, 80:5:110); 
    X_test = [P_mesh(:)'; V2_mesh(:)'];
    
    [~, S_bin] = encode_HDC_data(X_test, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
    sim = (C_bin * S_bin) / D;
    max_sim_val = max(sim * gamma, [], 1);
    delta_prime = exp(bsxfun(@minus, sim * gamma, max_sim_val)) ./ sum(exp(bsxfun(@minus, sim * gamma, max_sim_val)), 1);
    
    % ========================================================
    % [초고속 벡터화 3] UI 렌더링 시에도 for 루프 제거 (즉시 출력)
    % ========================================================
    N_test = size(X_test, 2);
    M_flat = reshape(M_int, num_clusters*2, D);
    M_times_S = reshape(M_flat * S_bin, num_clusters, 2, N_test) / D; % [K, 2, N_test]
    
    % [K, N_test] 인 delta_prime을 브로드캐스팅하여 합산
    Y_hat_norm = squeeze(sum(reshape(delta_prime, num_clusters, 1, N_test) .* M_times_S, 1)); % [2, N_test]
    Y_pred = min_Y + (Y_hat_norm + 1) .* (max_Y - min_Y) / 2;
    
    subplot(1, 2, 1); surf(P_mesh, V2_mesh, reshape(Y_pred(1,:), size(P_mesh)), 'EdgeColor', 'none');
    title(sprintf('Learned D1 (Epoch %d)', ep)); colormap(gca, 'parula'); clim([0.1 0.5]); view(45, 30); grid on;
    subplot(1, 2, 2); surf(P_mesh, V2_mesh, reshape(Y_pred(2,:), size(P_mesh)), 'EdgeColor', 'none');
    title(sprintf('Learned D2 (Epoch %d)', ep)); colormap(gca, 'turbo'); clim([0.1 0.5]); view(45, 30); grid on;
    drawnow;
end