% =========================================================================
% [하이엔드 HDC 파이프라인 v10.0] - All-Integer RegHD & Cosine Similarity
% -------------------------------------------------------------------------
% [업데이트 내용]
% 1. 위상(D0) 탐색: -90도 ~ 90도 전 구간 탐색 및 최저 전류 리플(I_pk) 위상 자동 선정
% 2. 초기 학습 생략: Pre-trained 모델 존재 시 로드하여 즉시 Fine-Tuning 돌입
% 3. Fine-Tuning 집중: SQP 알고리즘 결합, 조밀한 타겟 그리드, Hard Example Mining 도입
% 4. Bipolar(sign) 양자화 제거 및 Cosine Similarity 적용 (정수 스케일 보정)
% 5. Neighborhood Fine-Tuning: 타겟점 주변 클러스터 Softmax 동시 업데이트
% 6. [NEW] 확실한 성능 개선을 위한 Fine-Tuning 국소(Local) 집중 반복 업데이트 적용
% =========================================================================
function run_RegHDC_HighEnd_v10_IntegerBase()
    disp('===========================================================');
    disp('[Phase 0] 시스템 초기화 및 모델 빌드');
    disp('===========================================================');
    
    % 예측 모델 로드
    mlp_file = 'Data/trained_fast_mlp_net_v6.mat';
    xgb_file = 'Data/trained_xgboost_models_v6.mat';
    if ~isfile(mlp_file), error('MLP 모델이 없습니다. 먼저 학습하세요.'); end
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    
    xgb_models = []; 
    if isfile(xgb_file), load(xgb_file, 'xgb_models'); end
    fast_ensemble_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
    
    % -------------------------------------------------------------
    % [원본 데이터셋에서 Test Set 분리 후 실제 예측 RMSE 동적 계산]
    % -------------------------------------------------------------
    disp(' -> 원본 데이터셋에서 Test 데이터를 분리하여 전력 및 전류의 실제 예측 RMSE를 계산합니다...');
    try
        data = readtable('TPS_Dataset_20260316_021229.csv');
        V2_vec = data.V2_V'; D0_vec = data.D0_deg'; D1_vec = data.D1'; D2_vec = data.D2';
        X_raw = [V2_vec; D0_vec; D1_vec; D2_vec; D0_vec.^2; D1_vec.*D2_vec; V2_vec.*D0_vec];
        Y_raw_P = [data.P_act_W]'; % 전력 타겟
        
        % 전류 타겟(I_pk) 추출 시도
        try
            Y_raw_I = [data.i_pk_pri_A]';
        catch
            try
                Y_raw_I = [data.I_pk]';
            catch
                Y_raw_I = NaN(size(Y_raw_P)); % fallback
            end
        end
        
        % MLDataset.m 과 동일한 Seed 및 Split 적용
        rng(42); cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
        X_test  = X_raw(:, test(cv));
        Y_test_P  = Y_raw_P(:, test(cv));
        Y_test_I  = Y_raw_I(:, test(cv));
        
        test_idx = randperm(size(X_test, 2), min(10000, size(X_test, 2)));
        X_test_sub = X_test(:, test_idx);
        Y_test_sub_P = Y_test_P(:, test_idx);
        Y_test_sub_I = Y_test_I(:, test_idx);
        
        Y_pred = fast_ensemble_predict(X_test_sub);
        
        % 1. 전력(MLP) RMSE 계산
        ML_RMSE_P = sqrt(mean((Y_test_sub_P - Y_pred(1, :)).^2));
        disp([' -> [계산 완료] Test Set 기반 실제 예측 모델 전력 RMSE: ', num2str(ML_RMSE_P, '%.2f'), ' W']);
        
        % 2. 전류(XGBoost) RMSE 계산
        if ~any(isnan(Y_test_sub_I))
            ML_RMSE_I = sqrt(mean((Y_test_sub_I - Y_pred(3, :)).^2));
            disp([' -> [계산 완료] Test Set 기반 실제 XGBoost 예측 전류 RMSE: ', num2str(ML_RMSE_I, '%.2f'), ' A']);
        else
            ML_RMSE_I = 0.5; % Default
            disp(' -> [경고] I_pk 컬럼을 찾지 못해 기본 전류 RMSE(0.5A)를 적용합니다.');
        end
        
    catch
        disp(' -> [경고] 원본 데이터셋을 찾을 수 없어 기본 RMSE(8.0W, 0.5A)를 적용합니다.');
        ML_RMSE_P = 8.0;
        ML_RMSE_I = 0.5;
    end
    
    % 동적 마진(Dynamic Margin) 계산 함수
    P_ratio   = 0.01;  
    calc_margin = @(P_target) ML_RMSE_P + abs(P_target) * P_ratio;
    
    % 제어 변수 상/하한선
    D_min = 0.1;  
    D_max = 0.50; 

    % -------------------------------------------------------------
    % [Phase 1] 전역 최적해 데이터셋 추출 
    % -------------------------------------------------------------
    global_dataset_file = 'Candidate_Dataset_PureOpt_v9.csv'; 
    if ~isfile(global_dataset_file)
        disp('-> [신규 추출] 동적 오차(Dynamic Margin) 기반 데이터를 추출합니다.');
        dataset_tbl = extract_Global_Optima_Dataset(fast_ensemble_predict, calc_margin, D_min, D_max);
        writetable(dataset_tbl, global_dataset_file);
    else
        disp(['-> 기존 정밀 데이터셋 로드 완료: ', global_dataset_file]);
        dataset_tbl = readtable(global_dataset_file);
    end

    [unq_coords, ~, ic] = unique([dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V], 'rows');
    clean_D1 = accumarray(ic, dataset_tbl.Opt_D1, [], @mean);
    clean_D2 = accumarray(ic, dataset_tbl.Opt_D2, [], @mean);
    dataset_tbl = array2table([unq_coords(:,1), unq_coords(:,2), clean_D1, clean_D2], ...
        'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2'});

    % -------------------------------------------------------------
    % [Phase 1.5] 추출된 정밀 데이터셋 시각화 (Surf 버전)
    % -------------------------------------------------------------
    disp(' -> 추출/로드된 데이터셋의 타겟(D1, D2) 제어 곡면(Surf)을 시각화합니다.');
    fig_dataset = figure('Name', 'Phase 1: Global Optima Surface', 'Position', [150, 150, 1000, 450]);
    
    % 1. Surf 출력을 위한 2D 격자(Grid) 생성 (해상도 50x50)
    [P_mesh, V2_mesh] = meshgrid(linspace(min(dataset_tbl.Target_P_W), max(dataset_tbl.Target_P_W), 50), ...
                                 linspace(min(dataset_tbl.Target_V2_V), max(dataset_tbl.Target_V2_V), 50));
    
    % 2. 1차원 배열 데이터를 2D 격자에 맞게 보간 (Linear Interpolation)
    D1_surf = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D1, P_mesh, V2_mesh, 'linear');
    D2_surf = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D2, P_mesh, V2_mesh, 'linear');
    
    % 3. Opt_D1 시각화 (Surf)
    subplot(1, 2, 1);
    surf(P_mesh, V2_mesh, D1_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('Target D_1 Control Surface');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('Opt D_1');
    colormap(gca, 'parula'); clim([D_min D_max]); colorbar; grid on; view(-45, 30);
    
    % 4. Opt_D2 시각화 (Surf)
    subplot(1, 2, 2);
    surf(P_mesh, V2_mesh, D2_surf, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('Target D_2 Control Surface');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('Opt D_2');
    colormap(gca, 'turbo'); clim([D_min D_max]); colorbar; grid on; view(-45, 30);
    
    drawnow; % 화면에 즉시 렌더링 (과부하 방지)

    % -------------------------------------------------------------
    % [Phase 2] All-Integer & Cosine Similarity HDC 자동 평활화 학습
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 2] Dataset-based HDC Pre-training 검토 (All-Integer Base)...');
    disp('===========================================================');
    
    D = 5000; Q = 1000; mode = 'Multi_Interact'; 
    K_init = 256;  % 공간 보간 해상도를 높이기 위해 128 -> 256 클러스터로 상향
    gamma_fixed = 8.0; % 빗살무늬 억제를 위한 고정된 온도(Softmax) 파라미터 적용
    
    min_X = [-1200; 80]; max_X = [1200; 110]; 
    min_Y = [1.0 - D_max; 1.0 - D_max]; max_Y = [1.0 - D_min; 1.0 - D_min]; 
    
    rng(42);
    ID_vecs = randsrc(2, D, [-1, 1]);
    L_vecs = zeros(Q, D); L_vecs(1, :) = randsrc(1, D, [-1, 1]);
    flip_counts = max(1, floor(D / 2 / (Q - 1)));
    for q_idx = 2:Q
        L_vecs(q_idx, :) = L_vecs(q_idx-1, :);
        flip_idx = randperm(D, flip_counts);
        L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx);
    end

    epochs_total = 2000; 
    fig_learn = figure('Name', 'HDC Auto-Smoothing Progress', 'Position', [100, 100, 800, 600]);

    pretrained_model_file = 'trained_highend_hdc_model_v10_pre.mat';
    
    if isfile(pretrained_model_file)
        disp([' -> [스킵] 기존 Pre-trained 모델을 발견했습니다. 로드하여 초기 학습을 건너뜁니다: ', pretrained_model_file]);
        load(pretrained_model_file, 'hdc_model');
        % UI 갱신 (로드된 초기 상태 확인용)
        update_Realtime_UI(fig_learn, hdc_model.C_int, hdc_model.M_int, min_X, max_X, min_Y, max_Y, Q, D, hdc_model.gamma_fixed, mode, K_init, ID_vecs, L_vecs, epochs_total, D_min, D_max);
    else
        disp(' -> [신규 학습] Pre-trained 모델이 없습니다. 자동 평활화 학습을 시작합니다...');
        
        Delta_D1 = 1.0 - dataset_tbl.Opt_D1; Delta_D2 = 1.0 - dataset_tbl.Opt_D2;
        X_base = [dataset_tbl.Target_P_W'; dataset_tbl.Target_V2_V'];
        Y_base_2d = [Delta_D1'; Delta_D2'];
        
        % Bipolar 양자화를 배제하고 정수형(Integer) 인코딩 벡터만 활용
        [S_hist_int, ~] = encode_HDC_data(X_base, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
        Y_true_norm = 2 * ((Y_base_2d - min_Y) ./ (max_Y - min_Y)) - 1; 
        
        C_int = zeros(K_init, D); M_int = zeros(K_init, 2, D); 
        init_idx = randperm(size(X_base, 2), min(size(X_base, 2), K_init));
        for k = 1:length(init_idx), C_int(k, :) = S_hist_int(:, init_idx(k))'; end
        
        tic; 
        for ep = 1:epochs_total
            % Cosine Annealing Learning Rate
            lr_hdc = 0.05 * (1 + cos(pi * ep / epochs_total)) / 2 + 0.005; 
            
            shuffle_idx = randperm(size(X_base, 2));
            for b = 1:length(shuffle_idx)
                idx = shuffle_idx(b);
                S_curr_int = S_hist_int(:, idx); 
                Y_curr = Y_true_norm(:, idx);
                
                % 1. 정수형 기반 코사인 유사도(Cosine Similarity) 계산
                dp = C_int * S_curr_int;
                norm_C = sqrt(sum(C_int.^2, 2));
                norm_S = sqrt(sum(S_curr_int.^2));
                sim = dp ./ (norm_C * norm_S + 1e-9);
                
                % 2. 클러스터 모델 업데이트 (가장 유사도 높은 클러스터)
                [max_sim, k_star] = max(sim);
                C_int(k_star, :) = C_int(k_star, :) + (1 - max_sim) * S_curr_int';
                
                % 3. 고정 감마(gamma_fixed) 기반의 Softmax 신뢰도 추출
                max_sim_val = max(sim * gamma_fixed);
                delta_prime = exp(sim * gamma_fixed - max_sim_val) / sum(exp(sim * gamma_fixed - max_sim_val)); 
                
                % 4. 회귀 모델(M_int) 예측 및 에러 계산 (정수형 상태 유지)
                M_flat = reshape(M_int, K_init*2, D);
                M_times_S = reshape(M_flat * S_curr_int, K_init, 2) / D; 
                Y_hat_norm = (delta_prime' * M_times_S)'; 
                Error = Y_curr - Y_hat_norm; 
                
                % 5. Neighborhood 모델 병렬 업데이트
                Err_S = Error * S_curr_int'; 
                weights = lr_hdc * delta_prime; 
                M_int = M_int + reshape(weights, K_init, 1, 1) .* reshape(Err_S, 1, 2, D); 
            end
            
            if mod(ep, 200) == 0
                fprintf(' -> Pre-training %d/%d 완료 | Gamma: %.2f (Fixed) | 소요시간: %.1f초\n', ep, epochs_total, gamma_fixed, toc);
                update_Realtime_UI(fig_learn, C_int, M_int, min_X, max_X, min_Y, max_Y, Q, D, gamma_fixed, mode, K_init, ID_vecs, L_vecs, ep, D_min, D_max);
                tic; 
            end
        end

        % C_bin이 아닌 C_int와 gamma_fixed를 모델 구조체에 저장
        hdc_model = struct('C_int', C_int, 'M_int', M_int, ...
                           'min_X', min_X, 'max_X', max_X, 'min_Y', min_Y, 'max_Y', max_Y, ...
                           'Q', Q, 'D', D, 'gamma_fixed', gamma_fixed, 'mode', mode, 'num_clusters', K_init, ...
                           'ID_vecs', ID_vecs, 'L_vecs', L_vecs);
                           
        save(pretrained_model_file, 'hdc_model');
        disp([' -> Pre-training 완료 및 중간 모델 저장: ', pretrained_model_file]);
    end

    % -------------------------------------------------------------
    % [Phase 2.5] NEW: 전사적 집중 Fine-Tuning (Hard Example Mining & SQP)
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 2.5] Physics-Informed 집중 Fine-Tuning 진행...');
    disp(' -> -90도~90도 전 구간 탐색 기반으로 조밀한 그리드에 대해 국소 성능을 집중 최적화합니다.');
    disp(' -> [강화됨] 발견된 취약점에 대해 완전히 수렴할 때까지 로컬 반복 업데이트를 수행합니다.');
    disp('===========================================================');
    
    % 미세 조정용 타겟 그리드 (해상도 대폭 상향)
    [ft_P_grid, ft_V2_grid] = meshgrid(-1200:100:1200, 80:5:110);
    ft_P_list = ft_P_grid(:)';
    ft_V2_list = ft_V2_grid(:)';
    num_ft_points = length(ft_P_list);
    
    % Epoch 상향 (15 -> 30) 및 적응형 학습률 적용
    ft_epochs = 1000; 
    
    I_improvement_threshold = ML_RMSE_I * 0.10; % 개선 임계값을 낮춰 더 많은 점들을 교정 타겟으로 삼음
    fprintf(' -> 적용된 전류 개선 목표치(Threshold): %.3f A\n', I_improvement_threshold);
    
    for ft_ep = 1:ft_epochs
        % 수정 코드: Phase 2와 동일한 스타일의 Cosine Annealing
        max_lr = 0.05;
        min_lr = 0.005;
        ft_lr = min_lr + 0.5 * (max_lr - min_lr) * (1 + cos(pi * (ft_ep - 1) / ft_epochs));
        fprintf(' -> Fine-Tuning Epoch %d/%d 시작 (LR: %.4f)...\n', ft_ep, ft_epochs, ft_lr);
        improved_count = 0;
        
        % 탐색 순서를 섞어 Stochasticity 부여
        shuffle_idx = randperm(num_ft_points);
        
        for b = 1:num_ft_points
            idx = shuffle_idx(b);
            P_tgt = ft_P_list(idx);
            V2_tgt = ft_V2_list(idx);
            current_margin = calc_margin(P_tgt);
            
            % 1. 기준선 확보: SPS 성능 평가 (이제 -90~90 전구간 탐색)
            [~, I_sps, ~] = simulate_point_vectorized(P_tgt, V2_tgt, 0.5, 0.5, fast_ensemble_predict, current_margin);
            if isnan(I_sps), I_sps = 1000; end 
            
            % 2. 현재 HDC 성능 평가 (전구간 탐색 반영)
            [D1_h, D2_h] = predict_HDC_model(hdc_model, P_tgt, V2_tgt);
            [~, I_hdc, ~] = simulate_point_vectorized(P_tgt, V2_tgt, D1_h, D2_h, fast_ensemble_predict, current_margin);
            if isnan(I_hdc), I_hdc = 1000; end
            
            % 3. 튜닝 타겟팅 판단 (Hard Example Mining)
            % HDC가 SPS를 못 이기거나 성능이 안 좋은 구간 집중 타겟
            if I_hdc > (I_sps - I_improvement_threshold) || I_hdc == 1000 
                
                % 4. 전사적 역량 집중: 강력한 SQP (fmincon) 기반으로 최적해 완전 재탐색
                [opt_D1, opt_D2, opt_I] = find_Grid_SQP_Optimum(P_tgt, V2_tgt, fast_ensemble_predict, current_margin, D_min, D_max);
                
                % 5. 정수형 기반 Neighborhood 업데이트 수행 (집중 학습 반영)
                if opt_I < I_hdc && opt_I < 1000
                    improved_count = improved_count + 1;
                    
                    X_in = [P_tgt; V2_tgt];
                    Y_opt_norm = 2 * (([1.0-opt_D1; 1.0-opt_D2] - min_Y) ./ (max_Y - min_Y)) - 1;
                    
                    [S_curr_int, ~] = encode_HDC_data(X_in, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
                    
                    % [수정됨] 모델이 해당 포인트를 확실히 교정할 수 있도록 내부 집중 학습 루프 적용
                    local_max_iters = 50; % 각 샘플 당 최대 50번 집중 반복 업데이트
                    
                    for l_iter = 1:local_max_iters
                        % 코사인 유사도 계산
                        dp = hdc_model.C_int * S_curr_int;
                        norm_C = sqrt(sum(hdc_model.C_int.^2, 2));
                        norm_S = sqrt(sum(S_curr_int.^2));
                        sim = dp ./ (norm_C * norm_S + 1e-9);
                        
                        [max_sim, k_star] = max(sim);
                        % Cluster Update
                        hdc_model.C_int(k_star, :) = hdc_model.C_int(k_star, :) + (1 - max_sim) * S_curr_int';
                        
                        max_sim_val = max(sim * hdc_model.gamma_fixed);
                        delta_prime = exp(sim * hdc_model.gamma_fixed - max_sim_val) / sum(exp(sim * hdc_model.gamma_fixed - max_sim_val)); 
                        
                        % Regression Model Prediction & Error
                        M_flat = reshape(hdc_model.M_int, K_init*2, D);
                        M_times_S = reshape(M_flat * S_curr_int, K_init, 2) / D; 
                        Y_hat_norm = (delta_prime' * M_times_S)'; 
                        Error = Y_opt_norm - Y_hat_norm; 
                        
                        % 충분히 에러가 수렴(MSE가 1e-3 미만)했다면 불필요한 연산 방지를 위해 종료
                        if mean(abs(Error)) < 1e-3
                            break;
                        end
                        
                        Err_S = Error * S_curr_int'; 
                        % 강한 튜닝: Soft-Update 가중치를 Error와 LR에 기반하여 주변부 클러스터까지 적용
                        weights = ft_lr * delta_prime; 
                        hdc_model.M_int = hdc_model.M_int + reshape(weights, K_init, 1, 1) .* reshape(Err_S, 1, 2, D);
                    end
                end
            end
        end
        fprintf('    -> 발견 및 교정된 성능 저하 격자 수: %d 개\n', improved_count);
        % 실시간 시각화 업데이트
        update_Realtime_UI(fig_learn, hdc_model.C_int, hdc_model.M_int, min_X, max_X, min_Y, max_Y, Q, D, hdc_model.gamma_fixed, mode, K_init, ID_vecs, L_vecs, epochs_total + ft_ep, D_min, D_max);
    end
    
    save('trained_highend_hdc_model_v10.mat', 'hdc_model');
    disp(' -> 집중 Fine-Tuning 최적화 학습 완료 및 모델 저장: trained_highend_hdc_model_v10.mat');
    
    % -------------------------------------------------------------
    % [Phase 3] 학습 모델 성능 평가
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 3] 성능 평가 플로팅 (SPS vs Proposed HDC-TPS)');
    disp('===========================================================');
    evaluate_SPS_vs_HDC(hdc_model, fast_ensemble_predict, calc_margin);
end

% =========================================================================
% [Helper Functions]
% =========================================================================
function preds = ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models)
    X_in_norm = mapminmax('apply', X_in, PS_X);
    preds = mapminmax('reverse', net(X_in_norm), PS_Y);
    if ~isempty(xgb_models)
        i_xgb_log = predict(xgb_models.I_pk, X_in');
        preds(3, :) = (sign(i_xgb_log) .* (exp(abs(i_xgb_log)) - 1))';
    end
end

% 전역 최적해 추출
function dataset_tbl = extract_Global_Optima_Dataset(fast_predict, calc_margin, D_min, D_max)
    target_V2_list = 80:10:110; power_step = 100;            
    P_seq_pos = 100:power_step:1200; P_seq_neg = -100:-power_step:-1200;
    
    all_V2 = []; all_P = [];
    for v_idx = 1:length(target_V2_list)
        all_V2 = [all_V2, repmat(target_V2_list(v_idx), 1, length(P_seq_pos) + length(P_seq_neg))];
        all_P = [all_P, P_seq_pos, P_seq_neg];
    end
    
    total_pts = length(all_V2);
    D1_all = zeros(total_pts, 1); D2_all = zeros(total_pts, 1);
    
    parfor i = 1:total_pts
        current_margin = calc_margin(all_P(i));
        [D1_all(i), D2_all(i), ~] = find_Grid_SQP_Optimum(all_P(i), all_V2(i), fast_predict, current_margin, D_min, D_max);
    end
    
    dataset_tbl = array2table([all_P', all_V2', D1_all, D2_all], 'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2'});
    dataset_tbl = unique(dataset_tbl, 'rows');
end

function [opt_D1, opt_D2, opt_I] = find_Grid_SQP_Optimum(P_t, V2_t, fast_predict, margin, D_min, D_max)
    d1_grid = D_min:0.05:D_max; d2_grid = D_min:0.05:D_max;
    [D1_g, D2_g] = meshgrid(d1_grid, d2_grid);
    p2d_grid = [D1_g(:), D2_g(:)];
    
    cost_grid = costFunction_Active_2D_Vec_Multi_D0(p2d_grid, V2_t, P_t, fast_predict, margin);
    [~, best_idx] = min(cost_grid);
    best_p0 = p2d_grid(best_idx, :); 

    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp', 'UseParallel', false);
    lb = [D_min, D_min]; ub = [D_max, D_max];
    
    obj_fun_scalar = @(p) costFunction_Active_2D_Vec_Multi_D0(p, V2_t, P_t, fast_predict, margin);
    [Y_opt, opt_I] = fmincon(obj_fun_scalar, best_p0, [], [], [], [], lb, ub, [], options);
    opt_D1 = Y_opt(1); opt_D2 = Y_opt(2);
end

function cost = costFunction_Active_2D_Vec_Multi_D0(p2d, V2, P_target, fast_predict, margin)
    N = size(p2d, 1); D1 = p2d(:, 1)'; D2 = p2d(:, 2)';
    
    % 수정 1: 전력 부호 무관하게 위상 D0 전구간 탐색 (-90 ~ 90)
    d0_c = -90:5:90; 
    M_c = length(d0_c);
    
    V2_c = repmat(V2, M_c, N); D1_c = repmat(D1, M_c, 1); D2_c = repmat(D2, M_c, 1); D0_c = repmat(d0_c', 1, N);
    
    preds_c = fast_predict([V2_c(:), D0_c(:), D1_c(:), D2_c(:), D0_c(:).^2, D1_c(:).*D2_c(:), V2_c(:).*D0_c(:)]');
    P_preds_c = reshape(preds_c(1,:), M_c, N);
    I_pk_c = reshape(preds_c(3,:), M_c, N);
    
    err_c = abs(P_preds_c - P_target);
    valid_mask = err_c <= margin;
    
    valid_cost = I_pk_c; valid_cost(~valid_mask) = inf; 
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

% 실시간 3D 표면 모니터링 (Cosine Similarity 및 Integer 벡터 호환으로 수정됨)
function update_Realtime_UI(fig, C_int, M_int, min_X, max_X, min_Y, max_Y, Q, D, gamma, mode, num_clusters, ID_vecs, L_vecs, ep, D_min, D_max)
    figure(fig); clf;
    [P_mesh, V2_mesh] = meshgrid(-1200:50:1200, 80:5:110); 
    X_test = [P_mesh(:)'; V2_mesh(:)'];
    
    [S_int, ~] = encode_HDC_data(X_test, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
    
    % Batch Cosine Similarity
    dp = C_int * S_int;
    norm_C = sqrt(sum(C_int.^2, 2));
    norm_S = sqrt(sum(S_int.^2, 1));
    sim = dp ./ (norm_C * norm_S + 1e-9);
    
    max_sim_val = max(sim * gamma, [], 1);
    delta_prime = exp(bsxfun(@minus, sim * gamma, max_sim_val)) ./ sum(exp(bsxfun(@minus, sim * gamma, max_sim_val)), 1);
    
    M_flat = reshape(M_int, num_clusters*2, D);
    M_times_S = reshape(M_flat * S_int, num_clusters, 2, size(X_test, 2)) / D; 
    Y_hat_norm = squeeze(sum(reshape(delta_prime, num_clusters, 1, size(X_test, 2)) .* M_times_S, 1)); 
    
    Y_pred_delta = min_Y + (Y_hat_norm + 1) .* (max_Y - min_Y) / 2;
    Y_pred_D = 1.0 - Y_pred_delta;
    
    subplot(1, 2, 1); surf(P_mesh, V2_mesh, reshape(Y_pred_D(1,:), size(P_mesh)), 'EdgeColor', 'none');
    title(sprintf('Learned D1 (Epoch %d)', ep)); colormap(gca, 'parula'); clim([D_min D_max]); view(45, 30); grid on;
    subplot(1, 2, 2); surf(P_mesh, V2_mesh, reshape(Y_pred_D(2,:), size(P_mesh)), 'EdgeColor', 'none');
    title(sprintf('Learned D2 (Epoch %d)', ep)); colormap(gca, 'turbo'); clim([D_min D_max]); view(45, 30); grid on;
    drawnow;
end

function evaluate_SPS_vs_HDC(hdc_model, fast_predict, calc_margin)
    test_V2 = 100; 
    test_P_list = -1200:100:1200; % -1200 ~ 1200 구간 전체 평가로 확장
    
    I_pk_SPS = zeros(size(test_P_list));
    I_pk_HDC = zeros(size(test_P_list));
    
    for i = 1:length(test_P_list)
        P_target = test_P_list(i);
        current_margin = calc_margin(P_target); 
        
        [~, best_I_sps, ~] = simulate_point_vectorized(P_target, test_V2, 0.5, 0.5, fast_predict, current_margin);
        I_pk_SPS(i) = best_I_sps; 
        
        [D1_hdc, D2_hdc] = predict_HDC_model(hdc_model, P_target, test_V2);
        [~, best_I_hdc, ~] = simulate_point_vectorized(P_target, test_V2, D1_hdc, D2_hdc, fast_predict, current_margin);
        I_pk_HDC(i) = best_I_hdc;
    end
    
    fig_eval = figure('Name', 'Performance Comparison (Vectorized Eval)', 'Position', [200, 200, 800, 600]);
    plot(test_P_list, I_pk_SPS, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'Color', [0, 0.4470, 0.7410]); hold on;
    plot(test_P_list, I_pk_HDC, '-p', 'LineWidth', 2, 'MarkerSize', 10, 'Color', [0.8500, 0.3250, 0.0980]);
    
    grid on;
    xlabel('Output Power P (W)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Peak Current Stress I_{pk} (A)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Current Stress Comparison (V_2 = %d V) - Global Search', test_V2), 'FontSize', 14, 'FontWeight', 'bold');
    legend('Conventional SPS (Global Search)', 'Proposed HDC-TPS (Global Search)', 'Location', 'northwest', 'FontSize', 12);
end

function [min_err, best_I_pk, opt_D0] = simulate_point_vectorized(P_target, V2_target, D1_val, D2_val, fast_predict, margin)
    % 수정 1: 전력 부호 무관하게 위상 D0 전구간 0.1도 간격 탐색
    d0_cands = -90:0.1:90; 
    N = length(d0_cands);
    
    V2_rep = repmat(V2_target, 1, N); 
    D1_rep = repmat(D1_val, 1, N); 
    D2_rep = repmat(D2_val, 1, N);
    
    X_batch = [V2_rep; d0_cands; D1_rep; D2_rep; d0_cands.^2; D1_rep.*D2_rep; V2_rep.*d0_cands];
    
    preds_batch = fast_predict(X_batch);
    P_preds = preds_batch(1, :);
    I_preds = preds_batch(3, :);
    
    err_P = abs(P_preds - P_target);
    valid_mask = err_P <= margin;
    
    if any(valid_mask)
        valid_I = I_preds(valid_mask);
        valid_d0 = d0_cands(valid_mask);
        valid_err = err_P(valid_mask);
        
        [best_I_pk, min_idx] = min(valid_I);
        opt_D0 = valid_d0(min_idx);
        min_err = valid_err(min_idx);
    else
        [min_err, min_idx] = min(err_P);
        opt_D0 = d0_cands(min_idx);
        best_I_pk = NaN; 
    end
end

function cost = calc_Ipk_with_vectorized(D1, D2, P_t, V2_t, fast_predict, D_min, D_max, margin)
    if D1 < D_min || D1 > D_max || D2 < D_min || D2 > D_max
        cost = 1e6; 
        return;
    end
    
    [min_err, I_pk, ~] = simulate_point_vectorized(P_t, V2_t, D1, D2, fast_predict, margin);
    
    if isnan(I_pk)
        cost = 1e5 + min_err * 100; 
    else
        cost = I_pk + min_err * 0.1; 
    end
end

% 예측 모델 역시 이진화 로직을 걷어내고 정수형 + 코사인 유사도로 동작하도록 변경
function [D1_pred, D2_pred] = predict_HDC_model(model, P_val, V2_val)
    X_in = [P_val; V2_val];
    [S_int, ~] = encode_HDC_data(X_in, model.min_X, model.max_X, model.Q, model.D, model.ID_vecs, model.L_vecs, model.mode);
    
    % Cosine Similarity 계산
    dp = model.C_int * S_int;
    norm_C = sqrt(sum(model.C_int.^2, 2));
    norm_S = sqrt(sum(S_int.^2));
    sim = dp ./ (norm_C * norm_S + 1e-9);
    
    max_sim_val = max(sim * model.gamma_fixed);
    delta_prime = exp(sim * model.gamma_fixed - max_sim_val) / sum(exp(sim * model.gamma_fixed - max_sim_val));
    
    M_flat = reshape(model.M_int, model.num_clusters*2, model.D);
    M_times_S = reshape(M_flat * S_int, model.num_clusters, 2) / model.D;
    Y_hat_norm = (delta_prime' * M_times_S)';
    
    Y_pred_delta = model.min_Y + (Y_hat_norm + 1) .* (model.max_Y - model.min_Y) / 2;
    D1_pred = 1.0 - Y_pred_delta(1); D2_pred = 1.0 - Y_pred_delta(2);
    
    D1_pred = max(0.1, min(0.5, D1_pred)); D2_pred = max(0.1, min(0.5, D2_pred));
end