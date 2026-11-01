% =========================================================================
% [RegHDC 순수 회귀 학습 및 평가 파이프라인] - Pure Regression & Capacity Scaling
% -------------------------------------------------------------------------
% [업데이트 내용]
% 1. 데이터셋 후처리(FSM 및 물리 방어 루프) 전면 제거:
%    - Test.m 에서 생성한 최종 정제 데이터셋(Physics_Dataset_Final_Regression_Ready.csv)을
%      어떠한 추가 필터링이나 왜곡 없이 모델 학습에 직접(Direct) 투입합니다.
%    - 중복되었던 대규모 시뮬레이션 기반 물리 제어면 디펜스 루프를 전면 생략하여
%      학습 진입 대기 시간을 비약적으로 단축시킵니다.
% 2. 2-Channel 순수 회귀 아키텍처 및 대형 모델(Capacity) 확장:
%    - 차원 수(D) 대역: 256, 384 / 프로토타입 수(K): 24, 32, 40 탐색 (SRAM ~96KB 최적화).
% 3. 모델 피팅 성능 투명 비교:
%    - 예측 데이터에도 FSM을 씌우지 않고 모델 본연의 피팅 결과를 그대로 시각화하여
%      Ground Truth 곡면과 모델 예측 곡면이 실제로 얼마나 닮았는지 사실적으로 분석합니다.
% 4. 물리 기반 가중치(Physics-informed Weight) 및 균등 초기화 반영:
%    - 전류 저감 민감도가 높은 고효율 제어 평면에 가중치를 가하여 집중 피팅합니다.
%    - 무작위 초기화 대신 공간 균등 추출 방식을 채택하여 수렴 복원력을 극대화했습니다.
% 5. 에포크 제한 해제 및 조기 종료 (Adaptive Early Stopping) 도입:
%    - 고정된 에포크 제한을 탈피하여 최대 2000 에포크 범위 내에서 오차가 개선되지 않을 때까지
%      스마트하게 자동 학습하며, 최적 검증 오차 시점의 가중치를 자동 복원합니다.
% =========================================================================
function train_evaluate_RegHDC_PostProcessed()
    disp('===========================================================');
    disp('[Phase 0] 시스템 초기화 및 전력/전류 예측 모델 로드');
    disp('===========================================================');
    
    % 예측 모델 로드 (전류 시뮬레이션 및 평가용)
    mlp_file = 'Data/trained_fast_mlp_net_v6.mat';
    xgb_file = 'Data/trained_xgboost_models_v6.mat';
    if ~isfile(mlp_file), error('MLP 모델이 없습니다. 먼저 Data 폴더에 모델을 준비하세요.'); end
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    
    xgb_models = []; 
    if isfile(xgb_file), load(xgb_file, 'xgb_models'); end
    fast_ensemble_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
    
    % 마진 기준
    ML_RMSE_P = 8.0; ML_RMSE_I = 0.5;
    P_ratio = 0.02;  
    calc_margin = @(P_target) ML_RMSE_P + abs(P_target) * P_ratio;

    disp('===========================================================');
    disp('[Phase 1] 회귀 학습용 물리 데이터셋 로드');
    disp('===========================================================');
    
    % Test.m 에서 생성한 부드러운 최종 시그모이드 블렌딩 데이터셋을 로드합니다.
    dataset_file = 'Physics_Dataset_Final_Regression_Ready.csv'; 
    if ~isfile(dataset_file)
        disp(' -> [알림] 시그모이드 블렌딩 데이터셋을 찾을 수 없어 기본 데이터셋 로드를 시도합니다.');
        dataset_file = 'HighRes_Physics_Dataset_Paper_v3_SmartSmoothed.csv';
        if ~isfile(dataset_file)
            error('학습에 사용할 데이터셋 CSV 파일을 찾을 수 없습니다.');
        end
    end
    
    disp([' -> 최종 사용 데이터셋: ', dataset_file]);
    dataset_tbl = readtable(dataset_file);
    
    % Grid 형태로 변환 (후처리 및 3D 플롯용)
    P_uniq = sort(unique(dataset_tbl.Target_P_W));
    V2_uniq = sort(unique(dataset_tbl.Target_V2_V));
    [P_grid, V2_grid] = meshgrid(P_uniq, V2_uniq);
    
    % [오류 수정] Phase 3 예측 연산 루프에 필요한 그리드 크기 정의 복원
    [num_V2, num_P] = size(P_grid);
    
    D1_grid_raw = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D1, P_grid, V2_grid, 'nearest');
    D2_grid_raw = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D2, P_grid, V2_grid, 'nearest');

    disp('===========================================================');
    disp('[Phase 1.5] 데이터셋 다이렉트 매핑 및 그리드 변환');
    disp('===========================================================');
    
    % [중요 변경] FSM 락인 및 물리 시뮬레이션 방어선 후처리를 전면 삭제합니다.
    % Test.m 에서 생성된 원본을 물리적 Ground Truth로 원형 그대로 신뢰합니다.
    D1_final = D1_grid_raw;
    D2_final = D2_grid_raw;
    
    % 기존에 연산되어 CSV에 저장된 전류 프로파일을 그대로 매핑
    I_SPS_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.I_SPS, P_grid, V2_grid, 'nearest');
    I_Opt_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.I_Opt, P_grid, V2_grid, 'nearest');

    % [물리 기반 가중치 설계]
    % 전류 저감 폭(SPS - Opt)이 크게 일어나는 핵심 제어 구간에 가중치를 가해 학습 정확도를 집중 향상시킵니다.
    Delta_I_all = I_SPS_grid(:) - I_Opt_grid(:);
    Delta_I_all(isnan(Delta_I_all)) = 0; % 예외 및 노이즈 보호
    max_delta_i = max(Delta_I_all);
    if max_delta_i <= 0, max_delta_i = 1.0; end
    % 전류 저감량 크기에 비례하여 1.0 ~ 4.0 범위의 학습 가중치 부여
    W_all = 1.0 + 3.0 * (max(0, Delta_I_all) / max_delta_i);

    % 검증용 데이터셋 구조화
    post_dataset_tbl = table(P_grid(:), V2_grid(:), D1_final(:), D2_final(:), I_SPS_grid(:), I_Opt_grid(:), ...
        'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2', 'I_SPS', 'I_Opt'});
        
    post_dataset_filename = 'RegHDC_PostProcessed_Dataset.csv';
    writetable(post_dataset_tbl, post_dataset_filename);
    fprintf('\n -> [연동 완료] 학습 대상 데이터셋 CSV 복사 완료: %s\n\n', post_dataset_filename);
    
    % 대시보드 출력 [1차 검증] - 데이터셋 기준
    evaluate_extracted_dataset_3D(post_dataset_tbl, ML_RMSE_I, '[1차 검증] 로드 완료 학습용 원본 데이터셋');
    
    % =========================================================================
    % [순수 회귀용 타겟 데이터 가공] - Target Y는 오직 2차원 (Delta_D1, Delta_D2)
    % =========================================================================
    X_all = [P_grid(:)'; V2_grid(:)']; 
    clean_D1 = D1_final(:);
    clean_D2 = D2_final(:);
    
    % Delta = 0.5 - D 구조 (D가 0.1~0.5 범위이므로 Delta는 0~0.4 범위)
    Delta_D1 = 0.5 - clean_D1; 
    Delta_D2 = 0.5 - clean_D2;
    
    Y_all = [Delta_D1'; Delta_D2']; % 순수 회귀 2채널 타겟 매트릭스 (2 x N)
    
    % Train / Test 분할 (80% / 20%)
    rng(42); 
    cv = cvpartition(size(X_all, 2), 'HoldOut', 0.2);
    X_train = X_all(:, training(cv)); Y_train = Y_all(:, training(cv)); W_train = W_all(training(cv));
    X_val   = X_all(:, test(cv));     Y_val   = Y_all(:, test(cv));     W_val   = W_all(test(cv));
    
    % Normalization Bound
    min_X = [-1200; 80]; max_X = [1200; 110]; 
    min_Y = [0.0; 0.0];  
    max_Y = [0.4; 0.4]; 
    
    % 정규화 범위: [-1, 1]
    Y_train_norm = 2 * ((Y_train - min_Y) ./ (max_Y - min_Y)) - 1; 
    
    disp(' -> 데이터 정규화 및 분할 완료:');
    fprintf('   - 훈련 데이터: %d 샘플\n', size(X_train, 2));
    fprintf('   - 검증 데이터: %d 샘플\n', size(X_val, 2));

    disp('===========================================================');
    disp('[Phase 2] Multi-Agent 확장 모델 탐색 및 하드웨어 경쟁 제약');
    disp('===========================================================');
    
    % 수용량 확장 탐색 후보군 (STM32G474 128KB SRAM 탑재 최적화 대역 지정)
    D_cands = [256, 384];     
    K_cands = [24, 32, 40];           
    Q_cands = [48, 64];           
    gamma_cands = [6.0, 10.0];    
    
    % 조기 종료(Early Stopping) 파라미터 제어 정의
    max_epochs = 100000; % 최대 에포크 제한을 2000까지 대폭 확장 (성능 미개선 시 조기 중단)
    patience = 50;     % 50 에포크 연속으로 검증 오차가 정체 또는 상승하면 강제 탈출
    
    agents_results = []; 
    agent_idx = 1;
    total_agents = length(D_cands) * length(K_cands) * length(Q_cands) * length(gamma_cands);
    
    fprintf(' -> 총 %d개의 대형 에이전트가 성능 극대화를 위해 경쟁합니다...\n', total_agents);
    
    for d = D_cands
        for q = Q_cands
            ID_vecs = randsrc(2, d, [-1, 1]); 
            L_vecs = zeros(q, d); L_vecs(1, :) = randsrc(1, d, [-1, 1]);
            flip_counts = max(1, floor(d / 2 / (q - 1)));
            for q_idx = 2:q
                L_vecs(q_idx, :) = L_vecs(q_idx-1, :); 
                flip_idx = randperm(d, flip_counts); 
                L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx); 
            end
            
            [S_train, ~] = encode_HDC_data(X_train, min_X, max_X, q, d, ID_vecs, L_vecs);
            [S_val, ~]   = encode_HDC_data(X_val,   min_X, max_X, q, d, ID_vecs, L_vecs);
            
            for k = K_cands
                for g = gamma_cands
                    % 메모리 사양 계산 (순수 회귀 2채널 반영)
                    mem_KB = (k * d * 4 + k * 2 * d * 4 + q * d * 1 + 2 * d * 1) / 1024; 
                    ops_kMAC = ((k * d) + (k * 2 * d)) / 1000;
                    
                    % STM32G474RET6 안전탑재 마진(96KB) 기준 하드웨어 패널티 제약 적용
                    hardware_penalty = 0;
                    if ops_kMAC > 20.0, hardware_penalty = (ops_kMAC - 20.0) * 10000; end
                    if mem_KB > 96.0, hardware_penalty = hardware_penalty + (mem_KB - 96.0) * 8000; end

                    C_int = zeros(k, d);  
                    M_int = zeros(k, 2, d); % 순수 2채널 가중치 텐서
                    
                    % [초기 프로토타입 개선] 공간상 고른 격자 분포(Grid Coverage)를 위해 균등 간격 인덱스로 초기화 추출
                    num_train = size(X_train, 2);
                    init_idx = round(linspace(1, num_train, min(num_train, k)));
                    for k_init = 1:length(init_idx)
                        C_int(k_init, :) = S_train(:, init_idx(k_init))'; 
                    end
                    
                    % 에포크 상태 추적 및 최적 오차 보존용 변수 초기화
                    C_best = C_int;
                    M_best = M_int;
                    best_epoch_rmse = Inf;
                    patience_counter = 0;
                    actual_epochs = 0;
                    
                    for ep = 1:max_epochs
                        lr_hdc = 0.05 * (1 + cos(pi * ep / max_epochs)) / 2 + 0.005; 
                        shuffle_idx = randperm(size(X_train, 2));
                        
                        for b = 1:length(shuffle_idx)
                            idx = shuffle_idx(b); 
                            S_curr = S_train(:, idx); 
                            Y_curr = Y_train_norm(:, idx);
                            W_sample = W_train(idx); % 현재 샘플의 물리 기반 중요도 가중치
                            
                            dp = C_int * S_curr; 
                            norm_C = sqrt(sum(C_int.^2, 2)); 
                            norm_S = sqrt(sum(S_curr.^2));
                            sim = dp ./ (norm_C * norm_S + 1e-9); 
                            
                            [max_sim, k_star] = max(sim); 
                            C_int(k_star, :) = C_int(k_star, :) + (1 - max_sim) * S_curr';
                            
                            max_sim_val_train = max(sim * g); 
                            delta_prime = exp(sim * g - max_sim_val_train) / sum(exp(sim * g - max_sim_val_train)); 
                            
                            % 순수 회귀 업데이트 (조건부 분류기 마스킹 불필요)
                            M_flat = reshape(M_int, k*2, d); 
                            M_times_S = reshape(M_flat * S_curr, k, 2) / d; 
                            Y_hat_norm = (delta_prime' * M_times_S)'; 
                            
                            Error = Y_curr - Y_hat_norm; 
                            Err_S = Error * S_curr'; 
                            
                            % 물리 가중치를 경사하강 가중폭에 적용하여 고성능 요구점의 오차 복원 극대화
                            weights = lr_hdc * delta_prime * W_sample; 
                            M_int = M_int + reshape(weights, k, 1, 1) .* reshape(Err_S, 1, 2, d); 
                        end
                        
                        % --- [에포크 종료 시점 마다 실시간 오차 및 조기종료 체크] ---
                        dp_val = C_int * S_val; 
                        norm_C_val = sqrt(sum(C_int.^2, 2)); norm_S_val = sqrt(sum(S_val.^2, 1));
                        sim_val = dp_val ./ (norm_C_val * norm_S_val + 1e-9); 
                        max_sim_val_vec = max(sim_val * g, [], 1);
                        delta_prime_val = exp(bsxfun(@minus, sim_val * g, max_sim_val_vec)) ./ sum(exp(bsxfun(@minus, sim_val * g, max_sim_val_vec)), 1);
                        
                        M_flat_val = reshape(M_int, k*2, d); 
                        M_times_S_val = reshape(M_flat_val * S_val, k, 2, size(X_val, 2)) / d; 
                        Y_hat_norm_val = squeeze(sum(reshape(delta_prime_val, k, 1, size(X_val, 2)) .* M_times_S_val, 1)); 
                        Y_pred_delta = min_Y + (Y_hat_norm_val + 1) .* (max_Y - min_Y) / 2;
                        
                        D1_val_true = 0.5 - Y_val(1, :); D2_val_true = 0.5 - Y_val(2, :);
                        D1_val_pred = 0.5 - Y_pred_delta(1, :); D2_val_pred = 0.5 - Y_pred_delta(2, :);
                        
                        rmse_epoch_val = sqrt(mean(([D1_val_true; D2_val_true] - [D1_val_pred; D2_val_pred]).^2, 'all'));
                        if isnan(rmse_epoch_val), rmse_epoch_val = 1000; end
                        
                        % 최적 성능 체크 및 조기종료 카운터 관리
                        if rmse_epoch_val < best_epoch_rmse - 1e-6
                            best_epoch_rmse = rmse_epoch_val;
                            C_best = C_int; % 최적 파라미터 백업
                            M_best = M_int;
                            patience_counter = 0;
                        else
                            patience_counter = patience_counter + 1;
                        end
                        
                        actual_epochs = ep;
                        if patience_counter >= patience
                            break; % 성능 향상이 정체되었으므로 학습 탈출
                        end
                    end
                    
                    % 에포크 종료 후 "역대 가장 피팅 정확도가 뛰어났던 시점"의 하이퍼차원 평면 복원
                    C_int = C_best;
                    M_int = M_best;
                    rmse_val = best_epoch_rmse;
                    
                    % 종합 에이전트 평가 스코어 연산
                    fitness_score = rmse_val * 10000 + mem_KB * 10.0 + ops_kMAC * 50.0 + hardware_penalty;
                    
                    if mod(agent_idx, 5) == 0 || agent_idx == 1
                        fprintf('   [Agent %3d/%3d] D=%3d, K=%2d, Q=%2d | Best RMSE: %.5f (Epoch %d/%d) | Score: %8.1f\n', ...
                            agent_idx, total_agents, d, k, q, rmse_val, actual_epochs - patience_counter, actual_epochs, fitness_score);
                    end
                    
                    temp_model = struct('C_int', C_int, 'M_int', M_int, 'min_X', min_X, 'max_X', max_X, ...
                        'min_Y', min_Y, 'max_Y', max_Y, 'Q', q, 'D', d, 'gamma', g, 'K', k, ...
                        'ID_vecs', ID_vecs, 'L_vecs', L_vecs);
                    
                    agents_results(agent_idx).model = temp_model; 
                    agents_results(agent_idx).D = d; agents_results(agent_idx).K = k; 
                    agents_results(agent_idx).Q = q; agents_results(agent_idx).Gamma = g; 
                    agents_results(agent_idx).RMSE = rmse_val; agents_results(agent_idx).Mem_KB = mem_KB; 
                    agents_results(agent_idx).kMAC = ops_kMAC; agents_results(agent_idx).Score = fitness_score;
                    agents_results(agent_idx).Epochs = actual_epochs;
                    
                    agent_idx = agent_idx + 1;
                end
            end
        end
    end
    
    scores = [agents_results.Score]; 
    [~, sort_idx] = sort(scores, 'ascend');
    best_agent = agents_results(sort_idx(1));
    best_model = best_agent.model;
    
    disp('-----------------------------------------------------------'); 
    disp('🏆 [HDC Agent Leaderboard - Top 3]');
    for i = 1:min(3, length(sort_idx))
        idx = sort_idx(i); 
        fprintf(' %d위: D=%3d, K=%2d, Q=%2d | 회귀 RMSE: %.5f (최적화 에포크: %d) | Score: %.1f\n', ...
            i, agents_results(idx).D, agents_results(idx).K, agents_results(idx).Q, agents_results(idx).RMSE, agents_results(idx).Epochs, agents_results(idx).Score);
    end

    disp('===========================================================');
    disp('[Phase 3] 최우수 모델 물리 성능 평가 및 예측 제어면 시각화');
    disp('===========================================================');
    
    % (1) 최적 모델을 통한 검증 세트의 정량 오차 연산
    [S_val_best, ~] = encode_HDC_data(X_val, min_X, max_X, best_model.Q, best_model.D, best_model.ID_vecs, best_model.L_vecs);
    dp_val = best_model.C_int * S_val_best; 
    norm_C_val = sqrt(sum(best_model.C_int.^2, 2)); norm_S_val = sqrt(sum(S_val_best.^2, 1));
    sim_val = dp_val ./ (norm_C_val * norm_S_val + 1e-9); 
    max_sim_val_vec = max(sim_val * best_model.gamma, [], 1);
    delta_prime_val = exp(bsxfun(@minus, sim_val * best_model.gamma, max_sim_val_vec)) ./ sum(exp(bsxfun(@minus, sim_val * best_model.gamma, max_sim_val_vec)), 1);
    M_flat_val = reshape(best_model.M_int, best_model.K*2, best_model.D); 
    M_times_S_val = reshape(M_flat_val * S_val_best, best_model.K, 2, size(X_val, 2)) / best_model.D; 
    Y_hat_norm_val = squeeze(sum(reshape(delta_prime_val, best_model.K, 1, size(X_val, 2)) .* M_times_S_val, 1)); 
    Y_pred_delta = min_Y + (Y_hat_norm_val + 1) .* (max_Y - min_Y) / 2;

    D1_val_true = 0.5 - Y_val(1, :); D2_val_true = 0.5 - Y_val(2, :);
    D1_val_pred = 0.5 - Y_pred_delta(1, :); D2_val_pred = 0.5 - Y_pred_delta(2, :);
    RMSE_Overall = sqrt(mean(([D1_val_true; D2_val_true] - [D1_val_pred; D2_val_pred]).^2, 'all'));
    
    disp('-----------------------------------------------------------');
    disp(' -> [모델 전체 평면 시뮬레이션] 훈련된 모델 예측값 기반 물리 제어면 복원 중...');
    
    [S_grid, ~] = encode_HDC_data(X_all, min_X, max_X, best_model.Q, best_model.D, best_model.ID_vecs, best_model.L_vecs);
    dp_grid = best_model.C_int * S_grid; 
    norm_C_grid = sqrt(sum(best_model.C_int.^2, 2)); norm_S_grid = sqrt(sum(S_grid.^2, 1));
    sim_grid = dp_grid ./ (norm_C_grid * norm_S_grid + 1e-9); 
    max_sim_grid = max(sim_grid * best_model.gamma, [], 1);
    delta_prime_grid = exp(bsxfun(@minus, sim_grid * best_model.gamma, max_sim_grid)) ./ sum(exp(bsxfun(@minus, sim_grid * best_model.gamma, max_sim_grid)), 1);
    
    M_times_S_grid = reshape(best_model.M_int, best_model.K*2, best_model.D) * S_grid;
    M_times_S_grid = reshape(M_times_S_grid, best_model.K, 2, size(X_all, 2)) / best_model.D; 
    Y_hat_norm_grid = squeeze(sum(reshape(delta_prime_grid, best_model.K, 1, size(X_all, 2)) .* M_times_S_grid, 1)); 
    Y_pred_grid_delta = min_Y + (Y_hat_norm_grid + 1) .* (max_Y - min_Y) / 2;
    
    % 회귀 출력 복원
    D1_pred_grid = 0.5 - Y_pred_grid_delta(1, :);
    D2_pred_grid = 0.5 - Y_pred_grid_delta(2, :);
    
    D1_pred_grid = max(0.1, min(0.5, D1_pred_grid)); 
    D2_pred_grid = max(0.1, min(0.5, D2_pred_grid));
    
    D1_pred_grid_2D = reshape(D1_pred_grid, size(P_grid)); 
    D2_pred_grid_2D = reshape(D2_pred_grid, size(P_grid));
    
    % [중요 변경] 모델 출력 예측 단계에서도 인위적인 2차 FSM Lock을 제거하여
    % 모델이 스스로 데이터셋 곡면을 원형 그대로 정밀 학습했는지 피팅 정확도를 투명하게 판정합니다.
    
    I_Model_grid_2D = zeros(size(P_grid));
    for v_idx = 1:num_V2
        for p_idx = 1:num_P
            [~, i_model, ~] = simulate_point_vectorized(P_grid(v_idx, p_idx), V2_grid(v_idx, p_idx), D1_pred_grid_2D(v_idx, p_idx), D2_pred_grid_2D(v_idx, p_idx), fast_ensemble_predict, calc_margin(P_grid(v_idx, p_idx)));
            if isnan(i_model), i_model = 1000; end
            I_Model_grid_2D(v_idx, p_idx) = i_model;
        end
    end
    
    model_dataset_tbl = table(P_grid(:), V2_grid(:), D1_pred_grid_2D(:), D2_pred_grid_2D(:), I_SPS_grid(:), I_Model_grid_2D(:), ...
        'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2', 'I_SPS', 'I_Opt'});
        
    evaluate_extracted_dataset_3D(model_dataset_tbl, ML_RMSE_I, '[2차 검증] 학습된 RegHDC 베스트 모델 예측');

    % =========================================================================
    % [Ground Truth 곡면 vs 학습 모델 예측 곡면 정밀 비교 시각화]
    % =========================================================================
    plot_model_fit_comparison(P_grid, V2_grid, D1_final, D1_pred_grid_2D, D2_final, D2_pred_grid_2D);

    % 정량성 리포트 연산
    Reduction_Model = I_SPS_grid(:) - I_Model_grid_2D(:);
    valid_mask_m = ~isnan(Reduction_Model);
    valid_reduction_m = Reduction_Model(valid_mask_m);
    margin_eval = ML_RMSE_I * 0.5;
    
    is_tps_grid = (D1_pred_grid_2D(:) < 0.49) | (D2_pred_grid_2D(:) < 0.49);
    valid_tps_mask = is_tps_grid & valid_mask_m;
    valid_reduction_tps = Reduction_Model(valid_tps_mask);
    
    improved_mask_m = valid_mask_m & (Reduction_Model > margin_eval);
    valid_reduction_improved = Reduction_Model(improved_mask_m);

    improved_pts_m = sum(improved_mask_m);
    max_red_m = max(valid_reduction_m);
    avg_red_all_m = mean(valid_reduction_m);
    avg_red_tps_m = mean(valid_reduction_tps);
    if isnan(avg_red_tps_m), avg_red_tps_m = 0; end
    avg_red_improved_m = mean(valid_reduction_improved);
    if isnan(avg_red_improved_m), avg_red_improved_m = 0; end

    fprintf('\n===========================================================\n');
    fprintf('🏆 [최종 정량적 성능 평가 리포트 (Final Quantitative Report)]\n');
    fprintf('===========================================================\n');
    fprintf('[1. 최적 모델 파라미터 (Hyperparameters)]\n');
    fprintf(' - Dimension (D)    : %d (고해상도 확장)\n', best_agent.D);
    fprintf(' - Clusters (K)     : %d\n', best_agent.K);
    fprintf(' - Quantization (Q) : %d\n', best_agent.Q);
    fprintf(' - Gamma (γ)        : %.1f\n', best_agent.Gamma);
    fprintf('\n[2. 하드웨어 리소스 점유율 (MCU 탑재 기준)]\n');
    fprintf(' - SRAM Memory      : %.1f KB (순수 회귀 축소 반영)\n', best_agent.Mem_KB);
    fprintf(' - Compute (MACs)   : %.2f kMAC / inference\n', best_agent.kMAC);
    fprintf('\n[3. 머신러닝 회귀 성능 지표 (Test Set)]\n');
    fprintf(' - 전체 위상 예측 RMSE   : %6.5f\n', RMSE_Overall);
    fprintf('\n[4. 물리적 전류 저감 성능 (SPS 대비)]\n');
    fprintf(' - 전체 구간 평균 전류 저감량 : %6.3f A\n', avg_red_all_m);
    fprintf(' - TPS 구간 평균 전류 저감량  : %6.3f A (실질적 제어 동작 구간)\n', avg_red_tps_m);
    fprintf(' - 개선 구간 평균 전류 저감량 : %6.3f A (저감 성공 지점 한정)\n', avg_red_improved_m);
    fprintf(' - 최대 전류 저감량 (Max ΔI)  : %6.3f A\n', max_red_m);
    fprintf(' - 유효 개선 포인트 비율      : %6.1f %% (전체 %d개 중 %d개)\n', (improved_pts_m / length(valid_reduction_m)) * 100, length(valid_reduction_m), improved_pts_m);
    fprintf('===========================================================\n');
    
    disp('===========================================================');
    disp('[Phase 4] 최우수 모델 파일 추출 (MATLAB & C Header)');
    disp('===========================================================');
    
    % 1. MATLAB 용 파일 저장
    mat_filename = 'RegHDC_BestModel.mat';
    save(mat_filename, 'best_model');
    fprintf(' -> [저장 완료] 추후 MATLAB 시뮬레이션용 모델: %s\n', mat_filename);
    
    % 2. STM32 MCU 용 C Header 파일 자동 생성 및 저장 (2채널 반영)
    h_filename = 'RegHDC_Model_STM32.h';
    export_hdc_model_to_c_header(best_model, h_filename);
    fprintf(' -> [저장 완료] STM32G474RET6 펌웨어 탑재용 C 헤더: %s\n', h_filename);

    disp('===========================================================');
    disp('모든 평가 및 추출이 성공적으로 종료되었습니다.');
    disp('===========================================================');
end

% =========================================================================
% 데이터셋 곡면 vs 모델 복원 곡면 대칭 시각화 함수
% =========================================================================
function plot_model_fit_comparison(P_grid, V2_grid, D1_true, D1_pred, D2_true, D2_pred)
    figure('Name', 'HDC Regression Model Fitting Analysis', 'Position', [150, 150, 1500, 850]);
    
    % D1 True
    ax1 = subplot(2, 2, 1);
    surf(P_grid, V2_grid, D1_true, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('Target Dataset: Control Surface D_1');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
    colormap(ax1, 'turbo'); clim(ax1, [0.1 0.5]); view(-45, 30); grid on;
    
    % D1 Pred
    ax2 = subplot(2, 2, 2);
    surf(P_grid, V2_grid, D1_pred, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('HDC Model Predicted: Control Surface D_1');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
    colormap(ax2, 'turbo'); clim(ax2, [0.1 0.5]); view(-45, 30); grid on;
    
    % D2 True
    ax3 = subplot(2, 2, 3);
    surf(P_grid, V2_grid, D2_true, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('Target Dataset: Control Surface D_2');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
    colormap(ax3, 'turbo'); clim(ax3, [0.1 0.5]); view(-45, 30); grid on;
    
    % D2 Pred
    ax4 = subplot(2, 2, 4);
    surf(P_grid, V2_grid, D2_pred, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('HDC Model Predicted: Control Surface D_2');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
    colormap(ax4, 'turbo'); clim(ax4, [0.1 0.5]); view(-45, 30); grid on;
    
    drawnow;
end

% =========================================================================
% 모델을 STM32용 C 언어 헤더 파일로 변환하는 함수 (2차원 출력용)
% =========================================================================
function export_hdc_model_to_c_header(model, filename)
    fid = fopen(filename, 'w');
    if fid == -1
        error('헤더 파일을 생성할 수 없습니다: %s', filename);
    end
    
    fprintf(fid, '/**\n');
    fprintf(fid, ' * @file %s\n', filename);
    fprintf(fid, ' * @brief RegHDC Trained Model Parameters for STM32 (2-Channel Pure Regression)\n');
    fprintf(fid, ' * @note Auto-generated by MATLAB\n');
    fprintf(fid, ' */\n\n');
    
    fprintf(fid, '#ifndef REGHDC_MODEL_H\n');
    fprintf(fid, '#define REGHDC_MODEL_H\n\n');
    fprintf(fid, '#include <stdint.h>\n\n');
    
    fprintf(fid, '// --- Model Hyperparameters ---\n');
    fprintf(fid, '#define HDC_D %d\n', model.D);
    fprintf(fid, '#define HDC_K %d\n', model.K);
    fprintf(fid, '#define HDC_Q %d\n', model.Q);
    fprintf(fid, '#define HDC_GAMMA %.4ff\n\n', model.gamma);
    
    fprintf(fid, '// --- Min-Max Normalization Parameters ---\n');
    fprintf(fid, 'const float MIN_X[2] = {%.4ff, %.4ff};\n', model.min_X(1), model.min_X(2));
    fprintf(fid, 'const float MAX_X[2] = {%.4ff, %.4ff};\n', model.max_X(1), model.max_X(2));
    fprintf(fid, 'const float MIN_Y[2] = {%.4ff, %.4ff};\n', model.min_Y(1), model.min_Y(2));
    fprintf(fid, 'const float MAX_Y[2] = {%.4ff, %.4ff};\n\n', model.max_Y(1), model.max_Y(2));
    
    fprintf(fid, '// --- Encoding Basis Vectors (int8_t to save SRAM) ---\n');
    write_c_array_int8(fid, 'ID_VECS', model.ID_vecs(:)');
    write_c_array_int8(fid, 'L_VECS', model.L_vecs(:)');
    
    fprintf(fid, '// --- Trained Regression Tensors (float) ---\n');
    write_c_array_float(fid, 'C_INT', model.C_int(:)');
    write_c_array_float(fid, 'M_INT', model.M_int(:)');
    
    fprintf(fid, '\n#endif // REGHDC_MODEL_H\n');
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

% =========================================================================
% HDC 인코딩 및 시뮬레이터 헬퍼 함수
% =========================================================================
function [HV_int, HV_bin] = encode_HDC_data(X, min_X, max_X, Q, D, ID_vecs, L_vecs)
    HV_int = zeros(D, size(X, 2)); HV_bin = zeros(D, size(X, 2));
    X_q = max(1, min(Q, round((X - min_X) ./ (max_X - min_X + 1e-9) * (Q - 1)) + 1));
    for i = 1:size(X, 2)
        hv_sum = ID_vecs(1, :) .* L_vecs(X_q(1, i), :) + ID_vecs(2, :) .* L_vecs(X_q(2, i), :);
        HV_int(:, i) = hv_sum'; b = sign(hv_sum)'; b(b == 0) = 1; HV_bin(:, i) = b;
    end
end

function preds = ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models)
    X_in_norm = mapminmax('apply', X_in, PS_X); preds = mapminmax('reverse', net(X_in_norm), PS_Y);
    if ~isempty(xgb_models)
        i_xgb_log = predict(xgb_models.I_pk, X_in'); preds(3, :) = (sign(i_xgb_log) .* (exp(abs(i_xgb_log)) - 1))';
    end
end

function [min_err, best_I_pk, opt_D0] = simulate_point_vectorized(P_target, V2_target, D1_val, D2_val, fast_predict, margin)
    d0_cands = -90:0.1:90; N = length(d0_cands);
    V2_rep = repmat(V2_target, 1, N); D1_rep = repmat(D1_val, 1, N); D2_rep = repmat(D2_val, 1, N);
    X_batch = [V2_rep; d0_cands; D1_rep; D2_rep; d0_cands.^2; D1_rep.*D2_rep; V2_rep.*d0_cands];
    preds_batch = fast_predict(X_batch); P_preds = preds_batch(1, :); I_preds = preds_batch(3, :);
    err_P = abs(P_preds - P_target); valid_mask = err_P <= margin;
    if any(valid_mask)
        valid_I = I_preds(valid_mask); valid_d0 = d0_cands(valid_mask); valid_err = err_P(valid_mask);
        [best_I_pk, min_idx] = min(valid_I); opt_D0 = valid_d0(min_idx); min_err = valid_err(min_idx);
    else
        [min_err, min_idx] = min(err_P); opt_D0 = d0_cands(min_idx); best_I_pk = NaN; 
    end
end

function evaluate_extracted_dataset_3D(dataset_tbl, ML_RMSE_I, title_prefix)
    P_eval_vec = unique(dataset_tbl.Target_P_W);
    V2_eval_vec = unique(dataset_tbl.Target_V2_V);
    [P_grid, V2_grid] = meshgrid(P_eval_vec, V2_eval_vec);
    
    Reduction_Data = dataset_tbl.I_SPS - dataset_tbl.I_Opt;
    Current_Reduction = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, Reduction_Data, P_grid, V2_grid);
    D1_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D1, P_grid, V2_grid);
    D2_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D2, P_grid, V2_grid);
    
    margin = ML_RMSE_I * 0.5; valid_reduction = Reduction_Data(~isnan(Reduction_Data));
    improved_pts = sum(valid_reduction > margin); neutral_pts  = sum(abs(valid_reduction) <= margin);
    worsened_pts = sum(valid_reduction < -margin); total_pts    = length(valid_reduction);
    max_red = max(valid_reduction);
    
    Performance_Map = NaN(size(P_grid));
    Performance_Map(Current_Reduction > margin) = 1; Performance_Map(abs(Current_Reduction) <= margin) = 0; Performance_Map(Current_Reduction < -margin) = -1;

    fprintf('\n===========================================================\n');
    fprintf('[성능 리포트] %s 최종 SPS 대비 전류 저감\n', title_prefix);
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - 분석 포인트 수 : %d 개\n', total_pts);
    fprintf(' - 유의미한 저감  : %d 개 (%5.1f%%)\n', improved_pts, improved_pts/total_pts*100);
    fprintf(' - 오차범위 내    : %d 개 (%5.1f%%)\n', neutral_pts, neutral_pts/total_pts*100);
    fprintf(' - 성능 저하 구간 : %d 개 (%5.1f%%)\n', worsened_pts, worsened_pts/total_pts*100);
    fprintf('===========================================================\n');

    figure('Name', sprintf('%s - Dashboard', title_prefix), 'Position', [100, 100, 1400, 900]);
    n_c = 128; div_cmap = [linspace(0.85, 1, n_c)', linspace(0.15, 1, n_c)', linspace(0.19, 1, n_c)'; linspace(1, 0.0, n_c)', linspace(1, 0.447, n_c)', linspace(1, 0.741, n_c)'];
    
    ax1 = subplot(2, 2, 1);
    surf(P_grid, V2_grid, Current_Reduction, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    hold on; surf(P_grid, V2_grid, zeros(size(P_grid)), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.4, 'EdgeColor', 'none'); hold off;
    title(sprintf('[%s]\nSPS 대비 전류 저감량 \\Delta I_{pk} (최대: %.2f A)', title_prefix, max_red));
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('\Delta I_{pk} (A)');
    clim_val = max(abs(min(valid_reduction)), max_red); if clim_val == 0 || isnan(clim_val), clim_val = 1; end
    clim(ax1, [-clim_val, clim_val]); colormap(ax1, div_cmap); colorbar(ax1); view(-45, 30); grid on;

    ax2 = subplot(2, 2, 2);
    scatter(ax2, P_grid(:), V2_grid(:), 180, Performance_Map(:), 'filled', 'square');
    axis(ax2, 'tight');
    title(sprintf('분류 히트맵 (개선: %d개, 타협수렴: %d개, 성능저하: %d개)', improved_pts, neutral_pts, worsened_pts));
    colormap(ax2, [0.8500 0.1500 0.1980; 0.8 0.8 0.8; 0.0000 0.4470 0.7410]); clim(ax2, [-1 1]);
    cb = colorbar(ax2); cb.Ticks = [-0.67, 0, 0.67]; cb.TickLabels = {'성능 저하', '타협 수렴', '성능 개선'};
    
    ax3 = subplot(2, 2, 3);
    surf(P_grid, V2_grid, D1_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('D_1 제어 위상 곡면 (전압/전력에 따른 변화)');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
    colormap(ax3, 'turbo'); clim(ax3, [0.1 0.5]); view(-45, 30); grid on;

    ax4 = subplot(2, 2, 4);
    surf(P_grid, V2_grid, D2_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('D_2 제어 위상 곡면 (전압/전력에 따른 변화)');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
    colormap(ax4, 'turbo'); clim(ax4, [0.1 0.5]); view(-45, 30); grid on;
    drawnow;
end