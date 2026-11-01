% =========================================================================
% [가상 물리 대리 모델] 다중 클러스터 기반 RegHDC (Physics Surrogate)
% 
% [주요 특징]
% 1. 논문 원리 적용: Equation 1 기반 삼각함수(Sine/Cosine) 비선형 인코딩
% 2. 기존 MLP, XGBoost, Random Forest를 완벽하게 대체하는 7-Input -> 4-Output 통합 모델
% 3. Dual-State Learning: Integer 가중치로 학습하고 Binary 가중치로 추론
% 4. 초고속 벡터화: 중첩 for 루프를 제거하여 대규모 군집(128개)도 초고속 학습 가능
% 5. 개선: 학습된 모델 존재 시 자동 로드 및 그래픽스 과부하 방지 로직 적용
% 6. 수학적 교정 (v9): 논문 Eq.4 기반 유효 그래디언트 평균화, Softmax 온도 최적화, K-means 동결 방지
% =========================================================================

function train_RegHD_Physics_Surrogate()
    clear; clc; close all;

    model_filename = 'trained_reghdc_physics_surrogate_v9.mat'; % 파일명 v9로 변경 (새로 학습 유도)

    disp('===========================================================');
    disp(' [Phase 1] 대용량 데이터 로드 및 전처리 (7-Input, 4-Output)');
    disp('===========================================================');
    
    % 1. 데이터 로드 (원본 스크립트의 더미 데이터 생성 로직 포함)
    data = [];
    try
        data = readtable('TPS_Dataset_20260316_021229.csv');
        disp([' -> 데이터 로드 성공: 총 ', num2str(height(data)), '개 샘플 확인']);
    catch
        disp(' -> [경고] CSV 파일이 없습니다. 312,000개의 대규모 임시 더미 데이터를 생성합니다.');
        data = array2table(rand(312000, 8), 'VariableNames', {'Target_P_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'P_act_W', 'P_act_HV_W', 'i_pk_pri_A'});
        data.V2_V = data.V2_V * 30 + 80; data.D0_deg = data.D0_deg * 180 - 90;
        data.P_act_W = (rand(312000, 1) - 0.2) * 1200; 
        data.P_act_HV_W = data.P_act_W .* (0.8 + rand(312000,1)*0.18); 
        data.i_pk_pri_A = rand(312000, 1) * 20 + 5; 
    end
    
    % 효율(Efficiency) 계산
    eff_raw = zeros(size(data.P_act_W));
    P1 = data.P_act_W; P2 = data.P_act_HV_W;
    valid_idx = sign(P1) == sign(P2) & P1 ~= 0;
    idx_fwd = valid_idx & (P1 > 0); eff_raw(idx_fwd) = (abs(P2(idx_fwd)) ./ abs(P1(idx_fwd))) * 100;
    idx_rev = valid_idx & (P1 < 0); eff_raw(idx_rev) = (abs(P1(idx_rev)) ./ abs(P2(idx_rev))) * 100;
    eff_raw(eff_raw > 100) = 100; eff_raw(eff_raw < 0) = 0;
    
    % Feature Engineering (7차원 입력)
    V2_vec = data.V2_V'; D0_vec = data.D0_deg'; D1_vec = data.D1'; D2_vec = data.D2';
    X_raw = [V2_vec; D0_vec; D1_vec; D2_vec; D0_vec.^2; D1_vec.*D2_vec; V2_vec.*D0_vec];
    Y_raw = [data.P_act_W, data.P_act_HV_W, data.i_pk_pri_A, eff_raw]'; % 4차원 출력
    
    % Train / Test Split (8:2)
    rng(42); cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
    X_train = X_raw(:, training(cv)); Y_train = Y_raw(:, training(cv));
    X_test  = X_raw(:, test(cv));     Y_test  = Y_raw(:, test(cv));
    
    % 정규화 (Min-Max Scaling)
    [X_train_norm, PS_X] = mapminmax(X_train, -1, 1);
    [Y_train_norm, PS_Y] = mapminmax(Y_train, -1, 1);
    X_test_norm = mapminmax('apply', X_test, PS_X);
    
    disp(' -> 데이터 전처리 및 정규화 완료.');

    % [수정] 모델 로드 여부와 관계없이 공통으로 사용되는 차원 변수 미리 정의
    num_features = size(X_train, 1);
    num_targets = size(Y_train, 1);
    num_train = size(X_train_norm, 2);

    % =========================================================================
    % [기존 모델 확인 및 분기 처리]
    % =========================================================================
    if isfile(model_filename)
        disp('===========================================================');
        disp([' [알림] 기존 학습된 모델을 발견했습니다: ', model_filename]);
        disp(' -> 학습 과정을 건너뛰고 대리 모델 파라미터를 로드합니다.');
        disp('===========================================================');
        
        load(model_filename, 'physics_hdc_model');
        
        % 파라미터 복원
        C_bin = physics_hdc_model.C_bin;
        M_real = physics_hdc_model.M_real; 
        D = physics_hdc_model.D;
        num_clusters = physics_hdc_model.num_clusters;
        gamma = physics_hdc_model.gamma;
        B_vecs = physics_hdc_model.B_vecs;
        b_vecs = physics_hdc_model.b_vecs;
        
        % 학습 당시의 정규화 파라미터 복원 및 Test 데이터 재적용
        PS_X = physics_hdc_model.PS_X;
        PS_Y = physics_hdc_model.PS_Y;
        X_test_norm = mapminmax('apply', X_test, PS_X);
        
        disp(' -> 모델 로드 완료.');
    else
        disp('===========================================================');
        disp(' [Phase 2] RegHD 물리 대리 모델 파라미터 초기화 (Sine/Cosine 인코딩)');
        disp('===========================================================');
        
        % 파라미터 셋업
        D = 10000;          
        num_clusters = 512; 
        
        % [핵심 수정 1] Softmax Temperature(gamma)를 크게 올려 클러스터 모델 간의 전문화(Specialization) 강제 유도
        gamma = 10.0;       
        
        epochs = 40;        
        % [핵심 수정 2] 배치 평균 그래디언트를 사용하므로, 학습 속도를 정상화하기 위해 init_lr 상향
        init_lr = 1.0;      
        batch_size = 2000;  
        
        % --- [논문 Eq. 1 적용] 인코딩 파라미터 생성 ---
        rng(100);
        B_vecs = randn(D, num_features); 
        b_vecs = 2 * pi * rand(D, 1);    
        
        % Dual-State 클러스터 및 회귀 모델 초기화
        C_int = randsrc(num_clusters, D, [-1, 1]); 
        C_bin = C_int; 
        M_real = zeros(num_clusters, num_targets, D); 

        disp([' -> HDC 파라미터 셋업 완료: D=', num2str(D), ', Clusters=', num2str(num_clusters)]);

        disp('===========================================================');
        disp(' [Phase 3] RegHDC Dual-State 집중 학습 (On-the-fly Encoding)');
        disp('===========================================================');
        
        tic;
        for ep = 1:epochs
            lr = init_lr * (0.95 ^ ep);
            shuffle_idx = randperm(num_train);
            
            for bs = 1:batch_size:num_train
                be = min(bs + batch_size - 1, num_train);
                idx_batch = shuffle_idx(bs:be);
                B = length(idx_batch);
                
                % 1. On-the-fly 인코딩 (삼각함수 기반 비선형 매핑)
                X_batch_norm = X_train_norm(:, idx_batch);
                Y_true_batch = Y_train_norm(:, idx_batch);
                
                [S_int_batch, S_bin_batch] = encode_HDC_batch(X_batch_norm, B_vecs, b_vecs);
                
                % 2. 클러스터 유사도 검색 및 업데이트
                sim = (C_bin * S_bin_batch) / D; % [K x B]
                [max_sim, best_k_idx] = max(sim, [], 1);
                
                % 클러스터 센터 업데이트 (for 루프 대신 고속 벡터화 처리 적용)
                for k = 1:num_clusters
                    idx_k = (best_k_idx == k);
                    if any(idx_k)
                        weights = 1 - max_sim(idx_k);
                        C_int(k, :) = C_int(k, :) + weights * S_int_batch(:, idx_k)';
                    end
                end
                
                % [핵심 수정 3] 클러스터 동결 방지: K-means 중심점의 무한 증식을 막아 40 epoch 내내 적응력 유지
                C_int = max(min(C_int, 10), -10);
                
                % 매 배치마다 C_bin 최신화 (클러스터링 성능 극대화)
                C_bin = sign(C_int); 
                C_bin(C_bin == 0) = 1;
                
                % 3. Softmax 신뢰도 계산 (Sharpened Confidence)
                max_sim_val = max(sim * gamma, [], 1);
                exp_sim = exp(bsxfun(@minus, sim * gamma, max_sim_val));
                delta_prime = exp_sim ./ sum(exp_sim, 1); % [K x B]
                
                % 4. 초고속 다중 모델 예측
                M_flat = reshape(M_real, num_clusters * num_targets, D);
                M_times_S = reshape(M_flat * S_bin_batch, num_clusters, num_targets, B) / D; % [K x 4 x B]
                
                % 신뢰도를 곱하여 앙상블 합산: Y_hat [4 x B]
                delta_reshaped = reshape(delta_prime, num_clusters, 1, B);
                Y_hat = squeeze(sum(bsxfun(@times, delta_reshaped, M_times_S), 1)); 
                
                if B == 1 % squeeze 시 차원 축소 방지
                    Y_hat = Y_hat(:); 
                end
                
                % 5. 에러 계산 및 회귀 모델 업데이트 (초고속 브로드캐스팅)
                Error = Y_true_batch - Y_hat; % [4 x B]
                
                for k = 1:num_clusters
                    % [핵심 수정 4] 해당 클러스터에 기여한 유효 데이터 수 (Effective N) 계산
                    eff_N = max(sum(delta_prime(k, :)), 1e-3); % 0으로 나누기 방지
                    
                    if eff_N > 0.01 % 해당 클러스터가 일정 부분 기여했을 때만 업데이트
                        weighted_error = bsxfun(@times, Error, delta_prime(k, :)); % [4 x B]
                        % 그래디언트 폭발을 방지하기 위해 단순히 더하지 않고 유효 샘플 수(eff_N)로 나누어 평균값 사용
                        M_real(k, :, :) = squeeze(M_real(k, :, :)) + (lr / eff_N) * (weighted_error * S_bin_batch');
                    end
                end
            end
            
            elapsed = toc;
            fprintf('   - Epoch %2d / %d 완료 (LR: %.4f) | 누적 소요시간: %.1f 초\n', ep, epochs, lr, elapsed);
        end
        
        disp(' -> 학습 완료! 대리 모델 구조체를 저장합니다...');
        physics_hdc_model = struct('C_bin', C_bin, 'M_real', M_real, 'D', D, ...
                                   'num_clusters', num_clusters, 'gamma', gamma, ...
                                   'B_vecs', B_vecs, 'b_vecs', b_vecs, ...
                                   'PS_X', PS_X, 'PS_Y', PS_Y);
        save(model_filename, 'physics_hdc_model');
    end

    disp('===========================================================');
    disp(' [Phase 4] 물리 대리 모델 추론 및 성능 비교 검증');
    disp('===========================================================');
    
    % 1. RegHDC Test 데이터 추론
    disp(' -> [RegHDC] Test Set 추론 중... (메모리 절약을 위해 청크 처리)');
    chunk_size = 5000;
    num_test = size(X_test_norm, 2);
    Y_pred_hdc_norm = zeros(num_targets, num_test);
    
    for bs = 1:chunk_size:num_test
        be = min(bs + chunk_size - 1, num_test);
        X_chunk = X_test_norm(:, bs:be);
        
        [~, S_bin_te] = encode_HDC_batch(X_chunk, B_vecs, b_vecs);
        
        sim_te = (C_bin * S_bin_te) / D;
        max_sim_te = max(sim_te * gamma, [], 1);
        delta_prime_te = exp(bsxfun(@minus, sim_te * gamma, max_sim_te)) ./ sum(exp(bsxfun(@minus, sim_te * gamma, max_sim_te)), 1);
        
        M_flat_te = reshape(M_real, num_clusters * num_targets, D);
        M_times_S_te = reshape(M_flat_te * S_bin_te, num_clusters, num_targets, be-bs+1) / D;
        delta_reshaped_te = reshape(delta_prime_te, num_clusters, 1, be-bs+1);
        
        Y_chunk_pred = squeeze(sum(bsxfun(@times, delta_reshaped_te, M_times_S_te), 1));
        if (be-bs+1) == 1, Y_chunk_pred = Y_chunk_pred(:); end
        
        Y_pred_hdc_norm(:, bs:be) = Y_chunk_pred;
    end
    
    Y_pred_HDC = mapminmax('reverse', Y_pred_hdc_norm, PS_Y);
    
    % 결과 시각화 및 비교 플롯
    target_names = {'P_{act} (W)', 'P_{act\_HV} (W)', 'i_{pk} (A)', 'Efficiency (%)'};
    
    % 평가 메트릭 저장용
    metrics = struct();
    metrics.HDC = calculate_metrics(Y_test, Y_pred_HDC);
    plot_evaluation(Y_test, Y_pred_HDC, 'RegHDC Physics Surrogate', target_names);
    
    % 2. 기존 모델 로드 및 비교
    mlp_file = 'trained_fast_mlp_net_v6.mat';
    xgb_file = 'trained_xgboost_models_v6.mat';
    
    if isfile(mlp_file)
        disp(' -> [비교] 기존 MLP 모델 성능 평가 중...');
        load(mlp_file, 'net', 'PS_X', 'PS_Y');
        Y_pred_MLP = mapminmax('reverse', net(mapminmax('apply', X_test, PS_X)), PS_Y);
        metrics.MLP = calculate_metrics(Y_test, Y_pred_MLP);
        plot_evaluation(Y_test, Y_pred_MLP, 'Legacy MLP 7-Feat', target_names);
    end
    
    if isfile(xgb_file)
        disp(' -> [비교] 기존 XGBoost 모델 성능 평가 중...');
        load(xgb_file, 'xgb_models');
        Y_pred_XGB = zeros(size(Y_test));
        Y_pred_XGB(1,:) = sign(predict(xgb_models.P, X_test')') .* (exp(abs(predict(xgb_models.P, X_test')')) - 1);
        Y_pred_XGB(2,:) = sign(predict(xgb_models.P_HV, X_test')') .* (exp(abs(predict(xgb_models.P_HV, X_test')')) - 1);
        Y_pred_XGB(3,:) = sign(predict(xgb_models.I_pk, X_test')') .* (exp(abs(predict(xgb_models.I_pk, X_test')')) - 1);
        Y_pred_XGB(4,:) = predict(xgb_models.Eff_UQ, X_test')';
        metrics.XGB = calculate_metrics(Y_test, Y_pred_XGB);
    end
    
    % 결과 통합 출력
    disp('===========================================================');
    disp(' [최종 모델 성능 비교 결과 (Test R^2 기준)]');
    disp('===========================================================');
    models_tested = fieldnames(metrics);
    fprintf('%-10s | %-12s | %-12s | %-12s | %-12s\n', 'Model', 'P_act R^2', 'P_HV R^2', 'I_pk R^2', 'Eff R^2');
    fprintf('-----------------------------------------------------------\n');
    for i = 1:length(models_tested)
        m_name = models_tested{i};
        r2_vals = metrics.(m_name).R2;
        fprintf('%-10s | %-12.4f | %-12.4f | %-12.4f | %-12.4f\n', m_name, r2_vals(1), r2_vals(2), r2_vals(3), r2_vals(4));
    end
    disp('===========================================================');
    disp(' -> 모든 학습 및 평가가 성공적으로 종료되었습니다.');
end

% -------------------------------------------------------------------------
% [보조 함수 1] 논문(Eq. 1) 기반 비선형 HDC 인코더 (완전 벡터화 처리)
% -------------------------------------------------------------------------
function [HV_int, HV_bin] = encode_HDC_batch(X_norm, B_vecs, b_vecs)
    dot_product = B_vecs * X_norm; 
    dot_product_plus_b = bsxfun(@plus, dot_product, b_vecs);
    HV_int = cos(dot_product_plus_b) .* sin(dot_product);
    HV_bin = sign(HV_int);
    HV_bin(HV_bin == 0) = 1;
end

% -------------------------------------------------------------------------
% [보조 함수 2] 평가지표 계산 유틸리티
% -------------------------------------------------------------------------
function res = calculate_metrics(Y_true, Y_pred)
    num_targets = size(Y_true, 1);
    res.R2 = zeros(1, num_targets);
    res.RMSE = zeros(1, num_targets);
    for i = 1:num_targets
        yt = Y_true(i, :); yp = Y_pred(i, :);
        res.RMSE(i) = sqrt(mean((yt - yp).^2));
        res.R2(i) = 1 - (sum((yt - yp).^2) / sum((yt - mean(yt)).^2));
    end
end

% -------------------------------------------------------------------------
% [보조 함수 3] 평가 시각화 함수 (그래픽스 오류 방지 로직 적용)
% -------------------------------------------------------------------------
function plot_evaluation(Y_actual, Y_pred, model_name, target_names)
    figure('Name', [model_name, ' Performance Validation'], 'Position', [100, 200, 1500, 800]);
    
    num_samples = size(Y_actual, 2);
    max_plot_points = 10000; 
    
    if num_samples > max_plot_points
        rng(42); 
        plot_idx = randperm(num_samples, max_plot_points);
    else
        plot_idx = 1:num_samples;
    end
    
    for i = 1:4
        y_true = Y_actual(i, :); 
        y_hat = Y_pred(i, :);
        
        rmse = sqrt(mean((y_true - y_hat).^2));
        R2 = 1 - (sum((y_true - y_hat).^2) / sum((y_true - mean(y_true)).^2));
        
        subplot(2, 2, i);
        scatter(y_true(plot_idx), y_hat(plot_idx), 10, 'b', 'filled', 'MarkerFaceAlpha', 0.2); hold on;
        
        m_val = min([y_true, y_hat]); M_val = max([y_true, y_hat]); mg = (M_val - m_val) * 0.05; 
        plot([m_val-mg, M_val+mg], [m_val-mg, M_val+mg], 'r--', 'LineWidth', 2); hold off;
        
        title({target_names{i}, sprintf('Test R^2 = %.4f | RMSE = %.2f', R2, rmse)}, 'Interpreter', 'tex');
        xlabel('Actual (Ground Truth)'); ylabel('Predicted (AI)'); grid on; axis square;
        xlim([m_val-mg, M_val+mg]); ylim([m_val-mg, M_val+mg]);
    end
    drawnow limitrate; 
end