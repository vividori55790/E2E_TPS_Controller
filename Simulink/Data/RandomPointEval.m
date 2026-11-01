function evaluate_RegHDC_random_points(num_points)
    % =========================================================================
    % [RegHDC 랜덤 포인트 평가 스크립트] - 2차 검증 (Point-wise FSM Lock-in 적용)
    % -------------------------------------------------------------------------
    % 훈련된 RegHDC 모델을 기반으로 무작위(Random) (Power, Voltage) 포인트를 생성하여,
    % 기존 SPS(0.5, 0.5) 제어 대비 시뮬레이션 환경에서의 전류(I_pk) 저감 성능을 평가합니다.
    % 훈련 파이프라인의 최종 검증과 동일하게 0.49 임계값 기반 Lock-in 방어를 수행합니다.
    % =========================================================================
    
    if nargin < 1 || isempty(num_points)
        num_points = 500; % 기본 랜덤 포인트 갯수
    end
    
    disp('===========================================================');
    disp('[Phase 1] 시스템 초기화 및 모델/시뮬레이터 로드');
    disp('===========================================================');
    
    % 최적 모델 로드
    best_model_file = 'RegHDC_BestModel.mat';
    if ~isfile(best_model_file)
        error(['모델 파일이 없습니다: ', best_model_file, '\n먼저 학습 파이프라인을 실행하여 모델을 추출하세요.']);
    end
    load(best_model_file, 'best_model');
    fprintf(' -> 최적 모델 로드 완료: %s\n', best_model_file);

    % 예측 모델 로드 (전류 시뮬레이션용)
    mlp_file = 'Data/trained_fast_mlp_net_v6.mat';
    xgb_file = 'Data/trained_xgboost_models_v6.mat';
    if ~isfile(mlp_file)
        error('MLP 시뮬레이터 모델이 없습니다. Data 폴더의 경로를 확인하세요.'); 
    end
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    
    xgb_models = []; 
    if isfile(xgb_file), load(xgb_file, 'xgb_models'); end
    fast_ensemble_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
    
    % 평가용 오차 마진 기준 (훈련 코드와 동일)
    ML_RMSE_P = 8.0; ML_RMSE_I = 0.5;
    P_ratio = 0.02;  
    calc_margin = @(P_target) ML_RMSE_P + abs(P_target) * P_ratio;
    margin_eval = ML_RMSE_I * 0.5; % 성능 개선 판단용 기준 마진
    
    disp('===========================================================');
    disp('[Phase 2] 무작위 작동 포인트(Random Operating Points) 생성');
    disp('===========================================================');
    
    % 학습 시 사용된 변수 범위를 기반으로 랜덤 데이터 생성
    min_P = best_model.min_X(1); max_P = best_model.max_X(1);
    min_V = best_model.min_X(2); max_V = best_model.max_X(2);
    
    rng('shuffle'); % 매 실행 시마다 다른 무작위 포인트 생성 보장
    P_rand = (max_P - min_P) .* rand(1, num_points) + min_P;
    V2_rand = (max_V - min_V) .* rand(1, num_points) + min_V;
    X_rand = [P_rand; V2_rand];
    
    fprintf(' -> %d 개의 무작위 (Power, Voltage) 포인트 생성 완료.\n', num_points);
    
    disp('===========================================================');
    disp('[Phase 3] RegHDC 모델 추론 및 Point-wise FSM Lock-in 적용');
    disp('===========================================================');
    
    % 1. HDC 인코딩
    [S_rand, ~] = encode_HDC_data(X_rand, best_model.min_X, best_model.max_X, ...
                                  best_model.Q, best_model.D, ...
                                  best_model.ID_vecs, best_model.L_vecs);
                              
    % 2. Attention 기반 유사도 연산
    dp = best_model.C_int * S_rand; 
    norm_C = sqrt(sum(best_model.C_int.^2, 2)); 
    norm_S = sqrt(sum(S_rand.^2, 1));
    sim = dp ./ (norm_C * norm_S + 1e-9); 
    max_sim = max(sim * best_model.gamma, [], 1);
    delta_prime = exp(bsxfun(@minus, sim * best_model.gamma, max_sim)) ./ ...
                  sum(exp(bsxfun(@minus, sim * best_model.gamma, max_sim)), 1);
              
    % 3. 회귀 디코딩 (가중합)
    M_times_S = reshape(best_model.M_int, best_model.K*3, best_model.D) * S_rand;
    M_times_S = reshape(M_times_S, best_model.K, 3, num_points) / best_model.D; 
    Y_hat_norm = squeeze(sum(reshape(delta_prime, best_model.K, 1, num_points) .* M_times_S, 1));
    if num_points == 1, Y_hat_norm = Y_hat_norm'; end % 1개 포인트일 경우 차원 축소 방지
    Y_pred_delta = best_model.min_Y + (Y_hat_norm + 1) .* (best_model.max_Y - best_model.min_Y) / 2;

    % 4. 모델 1차 분류 (SPS or TPS) 및 맵핑
    Pred_Class = Y_pred_delta(1, :) > 0;
    
    D1_pred = zeros(1, num_points); 
    D2_pred = zeros(1, num_points);
    
    D1_pred(Pred_Class) = 0.5; 
    D2_pred(Pred_Class) = 0.5;
    D1_pred(~Pred_Class) = 0.5 - Y_pred_delta(2, ~Pred_Class);
    D2_pred(~Pred_Class) = 0.5 - Y_pred_delta(3, ~Pred_Class);
    
    D1_pred = max(0.1, min(0.5, D1_pred)); 
    D2_pred = max(0.1, min(0.5, D2_pred));

    % --- [핵심] 랜덤 포인트용 FSM 방어 (Point-wise Lock-in) ---
    % 훈련 파이프라인의 2차 검증(snap_th_2 = 0.49)과 동일한 원리로,
    % 그리드 탐색이 아닌 개별 포인트 기반에서 임계값 초과 시 즉시 SPS로 락인하여 
    % 고부하 영역에서 오분류로 인한 계곡 부활을 원천 차단합니다.
    lock_mask = (D1_pred >= 0.49) | (D2_pred >= 0.49);
    D1_pred(lock_mask) = 0.5;
    D2_pred(lock_mask) = 0.5;
    
    fprintf(' -> 모델 추론 완료. TPS로 제어되는 포인트: %d / %d 개\n', sum(~lock_mask), num_points);

    disp('===========================================================');
    disp('[Phase 4] 물리적 시뮬레이션 기반 전류 성능 비교 (Model vs Baseline)');
    disp('===========================================================');
    
    I_SPS = zeros(1, num_points);
    I_Model = zeros(1, num_points);
    
    fprintf(' -> 시뮬레이션 평가 중... (0%%)');
    for i = 1:num_points
        if mod(i, max(1, floor(num_points/10))) == 0
            fprintf('\b\b\b\b\b%3d%%)', round(i/num_points*100));
        end
        
        P_curr = P_rand(i); V2_curr = V2_rand(i);
        margin = calc_margin(P_curr);
        
        % 1. Baseline (SPS: 무조건 0.5, 0.5 적용)
        [~, i_sps, ~] = simulate_point_vectorized(P_curr, V2_curr, 0.5, 0.5, fast_ensemble_predict, margin);
        I_SPS(i) = i_sps;
        
        % 2. RegHDC Model 예측치 적용
        [~, i_model, ~] = simulate_point_vectorized(P_curr, V2_curr, D1_pred(i), D2_pred(i), fast_ensemble_predict, margin);
        I_Model(i) = i_model;
    end
    fprintf('\n');
    
    disp('===========================================================');
    disp('[Phase 5] 정량적 성능 리포트 산출 및 시각화');
    disp('===========================================================');
    
    % 저감량 계산 (양수: 전류 저감 성공, 성능 향상)
    Reduction = I_SPS - I_Model;
    
    % 시뮬레이션 실패(NaN) 제거
    valid_mask = ~isnan(Reduction) & ~isnan(I_SPS);
    valid_red = Reduction(valid_mask);
    
    % 오차 마진(margin_eval) 기반 성능 클래스 분류
    improved_mask = valid_red > margin_eval;
    neutral_mask = abs(valid_red) <= margin_eval;
    worsened_mask = valid_red < -margin_eval;
    
    improved_pts = sum(improved_mask);
    neutral_pts = sum(neutral_mask);
    worsened_pts = sum(worsened_mask);
    total_valid = length(valid_red);
    
    % 실질적인 TPS 동작 구간(락인이 걸리지 않은 구간) 내에서의 성능 평균
    valid_tps_points = valid_mask & ~lock_mask;
    red_tps_only = Reduction(valid_tps_points);
    
    fprintf('\n🏆 [무작위 좌표 포인트(Random Points) 성능 평가 리포트]\n');
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - 총 생성 랜덤 포인트  : %d 개\n', num_points);
    fprintf(' - 유효 시뮬레이션      : %d 개\n', total_valid);
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - 🟢 성능 개선 (Improved) : %4d 개 (%5.1f%%) | 평균 저감: %5.3f A\n', improved_pts, (improved_pts/total_valid)*100, mean(valid_red(improved_mask)));
    fprintf(' - ⚪ 타협 수렴 (Neutral)  : %4d 개 (%5.1f%%)\n', neutral_pts, (neutral_pts/total_valid)*100);
    fprintf(' - 🔴 성능 저하 (Worsened) : %4d 개 (%5.1f%%) | 평균 증가: %5.3f A\n', worsened_pts, (worsened_pts/total_valid)*100, mean(abs(valid_red(worsened_mask))));
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - ⚡ 최대 전류 저감량 (Max ΔI) : %5.3f A\n', max(valid_red));
    fprintf(' - 📊 전체 유효 평균 저감량     : %5.3f A\n', mean(valid_red));
    if ~isempty(red_tps_only)
        fprintf(' - 🎯 TPS 제어 구간 평균 저감량 : %5.3f A (실질적 최적화 구간)\n', mean(red_tps_only));
    end
    fprintf('===========================================================\n');
    
    % --- 결과 3D 시각화 ---
    figure('Name', 'Random Points Performance Evaluation', 'Position', [200, 200, 1000, 600]);
    
    % 색상 매핑 설정
    colors = zeros(total_valid, 3);
    colors(improved_mask, :) = repmat([0.0000 0.4470 0.7410], improved_pts, 1); % Blue (개선)
    colors(neutral_mask, :) = repmat([0.8 0.8 0.8], neutral_pts, 1);            % Gray (동등/타협)
    colors(worsened_mask, :) = repmat([0.8500 0.1500 0.1980], worsened_pts, 1); % Red (저하)
    
    P_valid = P_rand(valid_mask);
    V2_valid = V2_rand(valid_mask);
    
    scatter3(P_valid, V2_valid, valid_red, 40, colors, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    % 0점(기준선) 및 마진(Margin) 평면 시각화
    [P_grid_plot, V2_grid_plot] = meshgrid(linspace(min_P, max_P, 10), linspace(min_V, max_V, 10));
    surf(P_grid_plot, V2_grid_plot, zeros(size(P_grid_plot)), 'FaceColor', 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    surf(P_grid_plot, V2_grid_plot, repmat(margin_eval, 10, 10), 'FaceColor', [0 0 1], 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    surf(P_grid_plot, V2_grid_plot, repmat(-margin_eval, 10, 10), 'FaceColor', [1 0 0], 'FaceAlpha', 0.05, 'EdgeColor', 'none');
    hold off;
    
    title(sprintf('Random Operation Points Evaluation (N=%d)\n\\Delta I_{pk} vs (Power, Voltage)', total_valid));
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('\Delta I_{pk} (A)');
    grid on; view(-45, 30);
    
    % 범례 추가 (더미 산점도 활용)
    h1 = scatter3(NaN,NaN,NaN, 50, [0.0000 0.4470 0.7410], 'filled', 'MarkerEdgeColor', 'k');
    h2 = scatter3(NaN,NaN,NaN, 50, [0.8 0.8 0.8], 'filled', 'MarkerEdgeColor', 'k');
    h3 = scatter3(NaN,NaN,NaN, 50, [0.8500 0.1500 0.1980], 'filled', 'MarkerEdgeColor', 'k');
    legend([h1, h2, h3], ...
        sprintf('Improved ( > %.2fA)', margin_eval), ...
        sprintf('Neutral (\\pm %.2fA)', margin_eval), ...
        sprintf('Worsened ( < -%.2fA)', margin_eval), ...
        'Location', 'best');
        
    disp('랜덤 포인트 평가가 성공적으로 완료되었습니다.');
    disp('===========================================================');
end

% =========================================================================
% 필수 Helper 함수 (훈련용 코드와 동일 구조 유지)
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