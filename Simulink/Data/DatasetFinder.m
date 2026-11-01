% =========================================================================
% [논문용 고품질 데이터셋 생성기] - Physics-based Smooth TPS Dataset Generator
% -------------------------------------------------------------------------
% [업데이트 내용: Constrained Surface Relaxation & Local Shaking 적용]
% 1. Global Optimization: 각 포인트별로 SPS와 무관하게 무조건 전류 최저점을 찾음.
% 2. Unlearnable Data Culling: 주변과 D1/D2가 급격히 튀는 포인트는 
%    신경망 학습을 방해하므로 Median Filter 기준으로 도려냄 (NaN 처리).
% 3. Constrained Smoothing & Shaking: 곡면을 부드럽게 이완시키되, 
%    해당 포인트가 SPS보다 성능이 떨어지게 되면(물리법칙 위배), 
%    평면을 흔들어(원래 최적점과 다듬어진 점 사이를 탐색) SPS 성능 우위를 
%    유지하는 가장 부드러운 타협점을 찾아냅니다.
% =========================================================================
function run_Physics_Dataset_Generator_Final()
    disp('===========================================================');
    disp('[Phase 0] 시스템 초기화 및 모델 로드');
    disp('===========================================================');
    
    % 예측 모델 로드
    mlp_file = 'Data/trained_fast_mlp_net_v6.mat';
    xgb_file = 'Data/trained_xgboost_models_v6.mat';
    if ~isfile(mlp_file), error('MLP 모델이 없습니다. 먼저 학습하세요.'); end
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    
    xgb_models = []; 
    if isfile(xgb_file), load(xgb_file, 'xgb_models'); end
    fast_ensemble_predict = @(X_in) ensemble_predict(X_in, net, PS_X, PS_Y, xgb_models);
    
    % [동적 마진 및 오차 기준 설정]
    disp(' -> 원본 데이터셋에서 Test Set의 실제 RMSE를 계산합니다...');
    try
        data = readtable('TPS_Dataset_20260316_021229.csv');
        V2_vec = data.V2_V'; D0_vec = data.D0_deg'; D1_vec = data.D1'; D2_vec = data.D2';
        X_raw = [V2_vec; D0_vec; D1_vec; D2_vec; D0_vec.^2; D1_vec.*D2_vec; V2_vec.*D0_vec];
        Y_raw_P = [data.P_act_W]'; 
        try Y_raw_I = [data.i_pk_pri_A]'; catch, try Y_raw_I = [data.I_pk]'; catch, Y_raw_I = NaN(size(Y_raw_P)); end, end
        
        rng(42); cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
        X_test  = X_raw(:, test(cv)); Y_test_P  = Y_raw_P(:, test(cv)); Y_test_I  = Y_raw_I(:, test(cv));
        
        test_idx = randperm(size(X_test, 2), min(10000, size(X_test, 2)));
        X_test_sub = X_test(:, test_idx); Y_test_sub_P = Y_test_P(:, test_idx); Y_test_sub_I = Y_test_I(:, test_idx);
        Y_pred = fast_ensemble_predict(X_test_sub);
        
        ML_RMSE_P = sqrt(mean((Y_test_sub_P - Y_pred(1, :)).^2));
        if ~any(isnan(Y_test_sub_I))
            ML_RMSE_I = sqrt(mean((Y_test_sub_I - Y_pred(3, :)).^2));
        else
            ML_RMSE_I = 0.5;
        end
    catch
        disp(' -> [경고] 원본 데이터셋을 찾을 수 없어 기본 오차(8.0W, 0.5A)를 적용합니다.');
        ML_RMSE_P = 8.0; ML_RMSE_I = 0.5;
    end
    fprintf(' -> [계산 완료] 전력 예측 오차(RMSE_P): %.2f W, 전류 예측 오차(RMSE_I): %.2f A\n', ML_RMSE_P, ML_RMSE_I);
    
    P_ratio = 0.02;  
    calc_margin = @(P_target) ML_RMSE_P + abs(P_target) * P_ratio;
    D_min = 0.1; D_max = 0.50; 

    % -------------------------------------------------------------
    % [Phase 1 & 1.5] 데이터 추출 및 곡면 흔들기(Constrained Shaking)
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 1] 논문용 고해상도 Physics-based Dataset 추출 시작');
    disp('===========================================================');
    
    global_dataset_file = 'HighRes_Physics_Dataset_Paper_v3_SmartSmoothed.csv'; 
    
    dataset_tbl = extract_PhysicsSmooth_Dataset(fast_ensemble_predict, calc_margin, D_min, D_max, ML_RMSE_I);
    
    writetable(dataset_tbl, global_dataset_file);
    disp(['-> [완료] 물리적 타당성과 매끄러움이 보장된 최종 데이터셋 저장 완료: ', global_dataset_file]);
    
    % -------------------------------------------------------------
    % [Phase 2] 추출된 데이터 기반 SPS vs TPS 성능 시각화
    % -------------------------------------------------------------
    disp('===========================================================');
    disp('[Phase 2] 추출 데이터 기반 성능 평가 및 3D 곡면 렌더링');
    disp('===========================================================');
    
    evaluate_extracted_dataset_3D(dataset_tbl, ML_RMSE_I);

    disp('===========================================================');
    disp('모든 파이프라인이 성공적으로 종료되었습니다.');
    disp('===========================================================');
end


% =========================================================================
% 핵심 알고리즘: 추출, 필터링, 그리고 곡면 흔들기(Shaking)
% =========================================================================
function dataset_tbl = extract_PhysicsSmooth_Dataset(fast_predict, calc_margin, D_min, D_max, RMSE_I)
    target_V2_list = 80:10:110; 
    P_seq_pos = 0:50:1200;    % 해상도를 조금 더 높임
    P_seq_neg = -50:-50:-1200; 
    P_all_seq = [P_seq_neg, P_seq_pos];
    P_all_seq = sort(unique(P_all_seq));
    
    [P_grid, V2_grid] = meshgrid(P_all_seq, target_V2_list);
    [num_V2, num_P] = size(P_grid);
    
    D1_grid_raw = 0.5 * ones(num_V2, num_P);
    D2_grid_raw = 0.5 * ones(num_V2, num_P);
    I_sps_grid  = zeros(num_V2, num_P);
    I_opt_raw   = zeros(num_V2, num_P);
    
    % --- [Step 1] 전역 탐색 (Global Search) ---
    fprintf(' -> [1/4] Grid 기반 전역 최적해(Global Optimum) 탐색 중...\n');
    for v_idx = 1:num_V2
        v2_curr = target_V2_list(v_idx);
        for p_idx = 1:num_P
            p_curr = P_grid(v_idx, p_idx);
            margin = calc_margin(p_curr);
            
            % SPS 성능 측정
            [~, i_sps, ~] = simulate_point_vectorized(p_curr, v2_curr, 0.5, 0.5, fast_predict, margin);
            if isnan(i_sps), i_sps = 1000; end
            I_sps_grid(v_idx, p_idx) = i_sps;
            
            % 초기 그리드 기반 1차 최적화 (SPS 한계 제약 없이 무조건 최소점)
            [d1_opt, d2_opt, i_opt] = find_global_optimum(p_curr, v2_curr, fast_predict, margin, D_min, D_max);
            
            D1_grid_raw(v_idx, p_idx) = d1_opt;
            D2_grid_raw(v_idx, p_idx) = d2_opt;
            I_opt_raw(v_idx, p_idx) = i_opt;
        end
        fprintf('    ... V2 = %d V 완료\n', v2_curr);
    end
    
    % --- [Step 2] 학습 불가 데이터 쳐내기 (Unlearnable Data Culling) ---
    fprintf(' -> [2/4] 학습을 방해하는 뾰족한 노이즈(Spike) 제거 중...\n');
    % Median Filter를 통해 주변 컨텍스트와 동떨어진 포인트를 찾습니다.
    D1_med = medfilt2(D1_grid_raw, [3 3], 'symmetric');
    D2_med = medfilt2(D2_grid_raw, [3 3], 'symmetric');
    
    spike_threshold = 0.08; % 주변 대비 0.08 이상 튀면 비정상
    bad_mask = (abs(D1_grid_raw - D1_med) > spike_threshold) | (abs(D2_grid_raw - D2_med) > spike_threshold);
    
    % 노이즈 지우기
    D1_clean = D1_grid_raw; D1_clean(bad_mask) = NaN;
    D2_clean = D2_grid_raw; D2_clean(bad_mask) = NaN;
    
    % NaN(구멍)을 주변 데이터 기반으로 자연스럽게 보간 (Inpainting)
    D1_filled = fillmissing(D1_clean, 'movmedian', 5, 2);
    D2_filled = fillmissing(D2_clean, 'movmedian', 5, 2);
    D1_filled = fillmissing(D1_filled, 'movmedian', 5, 1);
    D2_filled = fillmissing(D2_filled, 'movmedian', 5, 1);
    
    % --- [Step 3] 곡면 흔들기 및 물리 제약 기반 이완 (Constrained Surface Relaxation) ---
    fprintf(' -> [3/4] 부드러운 곡면 생성 및 물리 제약(SPS 성능) 타협점 탐색(Shaking)...\n');
    
    D1_smooth = D1_filled; D2_smooth = D2_filled;
    I_final   = zeros(num_V2, num_P);
    
    % 가우시안 커널로 표면을 다듬음 (초기화)
    h = fspecial('gaussian', 5, 2);
    
    % Iterative Relaxation (표면을 팽팽하게 당기는 과정)
    max_relax_iter = 3; 
    for iter = 1:max_relax_iter
        D1_smooth = imfilter(D1_smooth, h, 'replicate');
        D2_smooth = imfilter(D2_smooth, h, 'replicate');
        
        for v_idx = 1:num_V2
            for p_idx = 1:num_P
                p_curr = P_grid(v_idx, p_idx); v2_curr = V2_grid(v_idx, p_idx);
                margin = calc_margin(p_curr);
                i_sps_val = I_sps_grid(v_idx, p_idx);
                
                % 현재 평활화된 D1, D2로 전류 성능 확인
                d1_s = D1_smooth(v_idx, p_idx); d2_s = D2_smooth(v_idx, p_idx);
                [~, i_curr_smooth, ~] = simulate_point_vectorized(p_curr, v2_curr, d1_s, d2_s, fast_predict, margin);
                if isnan(i_curr_smooth), i_curr_smooth = 1000; end
                
                % 물리 제약 검사: 다듬은 점이 SPS보다 안 좋거나, 마진 확보가 안 된 경우 (학습 가치 하락)
                if i_curr_smooth > (i_sps_val - RMSE_I * 0.2) 
                    % [Shaking Algorithm 적용] 
                    % 다듬어진 부드러운 위치(d1_s, d2_s)와 원래의 최고 성능 위치(raw) 사이를 줄다리기
                    d1_raw = D1_grid_raw(v_idx, p_idx); d2_raw = D2_grid_raw(v_idx, p_idx);
                    
                    best_alpha = 0; best_i = i_curr_smooth;
                    found_valid = false;
                    
                    % 0(다듬은점) ~ 1(원본최적점) 사이를 선형 탐색하며 SPS를 이기는 가장 0에 가까운 점 찾기
                    for alpha = 0.1:0.1:1.0
                        d1_shake = d1_s * (1 - alpha) + d1_raw * alpha;
                        d2_shake = d2_s * (1 - alpha) + d2_raw * alpha;
                        
                        [~, i_shake, ~] = simulate_point_vectorized(p_curr, v2_curr, d1_shake, d2_shake, fast_predict, margin);
                        if ~isnan(i_shake) && i_shake < (i_sps_val - RMSE_I * 0.2)
                            best_alpha = alpha;
                            best_i = i_shake;
                            found_valid = true;
                            break; % 부드러움(0에 가까움)을 우선시하므로 찾으면 즉시 중단
                        end
                    end
                    
                    if found_valid
                        D1_smooth(v_idx, p_idx) = d1_s * (1 - best_alpha) + d1_raw * best_alpha;
                        D2_smooth(v_idx, p_idx) = d2_s * (1 - best_alpha) + d2_raw * best_alpha;
                        I_final(v_idx, p_idx) = best_i;
                    else
                        % 흔들어도 답이 없다면, SPS(0.5, 0.5)로 매끄럽게 흡수시킴
                        D1_smooth(v_idx, p_idx) = 0.5;
                        D2_smooth(v_idx, p_idx) = 0.5;
                        I_final(v_idx, p_idx) = i_sps_val;
                    end
                else
                    I_final(v_idx, p_idx) = i_curr_smooth;
                end
                
                % 안전 범위 클리핑
                D1_smooth(v_idx, p_idx) = max(D_min, min(D_max, D1_smooth(v_idx, p_idx)));
                D2_smooth(v_idx, p_idx) = max(D_min, min(D_max, D2_smooth(v_idx, p_idx)));
            end
        end
    end
    
    % --- [Step 4] 최종 데이터셋 조립 ---
    fprintf(' -> [4/4] 데이터셋 테이블 조립 중...\n');
    clean_P = P_grid(:); clean_V2 = V2_grid(:);
    clean_D1 = D1_smooth(:); clean_D2 = D2_smooth(:);
    clean_I_SPS = I_sps_grid(:); clean_I_Opt = I_final(:);
    
    dataset_tbl = table(clean_P, clean_V2, clean_D1, clean_D2, clean_I_SPS, clean_I_Opt, ...
        'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D1', 'Opt_D2', 'I_SPS', 'I_Opt'});
end

function [opt_d1, opt_d2, opt_i] = find_global_optimum(P_t, V2_t, fast_predict, margin, D_min, D_max)
    % 10x10의 러프한 2D 그리드를 생성하여 Global Minima의 대략적 위치를 찾음
    d_cands = linspace(D_min, D_max, 8);
    [D1_g, D2_g] = meshgrid(d_cands, d_cands);
    D1_g = D1_g(:); D2_g = D2_g(:);
    N_g = length(D1_g);
    
    best_grid_i = inf;
    best_grid_idx = 1;
    
    % 병렬화가 가능하게 Batch 형태로 계산 (빠른 계산)
    for k = 1:N_g
        [~, i_val, ~] = simulate_point_vectorized(P_t, V2_t, D1_g(k), D2_g(k), fast_predict, margin);
        if ~isnan(i_val) && i_val < best_grid_i
            best_grid_i = i_val;
            best_grid_idx = k;
        end
    end
    
    if isinf(best_grid_i)
        % 만족하는 해가 전혀 없으면 SPS 반환
        opt_d1 = 0.5; opt_d2 = 0.5; opt_i = 1000;
        return;
    end
    
    % 찾은 대략적 위치를 기반으로 fmincon 정밀 탐색 (Local Optimum 함정 회피)
    x0 = [D1_g(best_grid_idx), D2_g(best_grid_idx)];
    obj_fun = @(p) costFunction_Active_2D_Vec(p, V2_t, P_t, fast_predict, margin);
    
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp', ...
                           'StepTolerance', 1e-4, 'OptimalityTolerance', 1e-4);
    [Y_opt, final_i] = fmincon(obj_fun, x0, [], [], [], [], [D_min, D_min], [D_max, D_max], [], options);
    
    opt_d1 = Y_opt(1);
    opt_d2 = Y_opt(2);
    opt_i = final_i;
end

function cost = costFunction_Active_2D_Vec(p2d, V2, P_target, fast_predict, margin)
    [~, i_pk, ~] = simulate_point_vectorized(P_target, V2, p2d(1), p2d(2), fast_predict, margin);
    if isnan(i_pk)
        cost = 5000; % 유효하지 않은 물리점 패널티
    else
        cost = i_pk;
    end
end

% =========================================================================
% Evaluation & Visualization Function (Phase 2)
% =========================================================================
function evaluate_extracted_dataset_3D(dataset_tbl, ML_RMSE_I)
    P_eval_vec = unique(dataset_tbl.Target_P_W);
    V2_eval_vec = unique(dataset_tbl.Target_V2_V);
    [P_grid, V2_grid] = meshgrid(P_eval_vec, V2_eval_vec);
    
    Reduction_Data = dataset_tbl.I_SPS - dataset_tbl.I_Opt;
    
    Current_Reduction = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, Reduction_Data, P_grid, V2_grid);
    D1_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D1, P_grid, V2_grid);
    D2_grid = griddata(dataset_tbl.Target_P_W, dataset_tbl.Target_V2_V, dataset_tbl.Opt_D2, P_grid, V2_grid);
    
    margin = ML_RMSE_I * 0.5;
    
    valid_mask = ~isnan(Reduction_Data);
    valid_reduction = Reduction_Data(valid_mask);
    
    improved_pts = sum(valid_reduction > margin);
    neutral_pts  = sum(abs(valid_reduction) <= margin);
    worsened_pts = sum(valid_reduction < -margin);
    total_pts    = length(valid_reduction);
    
    max_red = max(valid_reduction);
    avg_red_all = mean(valid_reduction);
    avg_red_improved = mean(valid_reduction(valid_reduction > margin));
    if isnan(avg_red_improved), avg_red_improved = 0; end
    
    Performance_Map = NaN(size(P_grid));
    Performance_Map(Current_Reduction > margin) = 1;
    Performance_Map(abs(Current_Reduction) <= margin) = 0;
    Performance_Map(Current_Reduction < -margin) = -1;

    fprintf('\n===========================================================\n');
    fprintf('[성능 리포트] 이완 및 흔들기(Shaking) 기반 최종 SPS 대비 전류 저감\n');
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - 분석 포인트 수 : %d 개\n', total_pts);
    fprintf(' - 유의미한 저감  : %d 개 (%5.1f%%) [학습 가능하고 성능 우수]\n', improved_pts, improved_pts/total_pts*100);
    fprintf(' - 오차범위 내    : %d 개 (%5.1f%%) [SPS로 스무스하게 수렴]\n', neutral_pts, neutral_pts/total_pts*100);
    fprintf(' - 성능 저하 구간 : %d 개 (%5.1f%%) [알고리즘 방어율 검증]\n', worsened_pts, worsened_pts/total_pts*100);
    fprintf('-----------------------------------------------------------\n');
    fprintf(' - 최대 전류 저감 : %5.2f A\n', max_red);
    fprintf(' - 평균 저감(전체): %5.2f A\n', avg_red_all);
    fprintf(' - 평균 저감(개선): %5.2f A (개선된 구간 한정)\n', avg_red_improved);
    fprintf('===========================================================\n');

    figure('Name', 'Physics-Constrained Shaking Dataset Evaluation Dashboard', 'Position', [100, 100, 1400, 900]);

    n_c = 128;
    red_to_white = [linspace(0.85, 1, n_c)', linspace(0.15, 1, n_c)', linspace(0.19, 1, n_c)'];
    white_to_blue = [linspace(1, 0.0, n_c)', linspace(1, 0.447, n_c)', linspace(1, 0.741, n_c)'];
    div_cmap = [red_to_white; white_to_blue];

    ax1 = subplot(2, 2, 1);
    surf(P_grid, V2_grid, Current_Reduction, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    hold on; surf(P_grid, V2_grid, zeros(size(P_grid)), 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.4, 'EdgeColor', 'none'); hold off;
    title(sprintf('SPS 대비 전류 저감량 \\Delta I_{pk} (학습가능 곡면)\n(평균: %.2f A / 최대: %.2f A)', avg_red_all, max_red));
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('\Delta I_{pk} (A)');
    clim_val = max(abs(min(valid_reduction)), abs(max_red));
    if clim_val == 0 || isnan(clim_val), clim_val = 1; end
    clim(ax1, [-clim_val, clim_val]); colormap(ax1, div_cmap); colorbar(ax1); view(-45, 30); grid on;

    ax2 = subplot(2, 2, 2);
    scatter(ax2, P_grid(:), V2_grid(:), 180, Performance_Map(:), 'filled', 'square');
    axis(ax2, 'tight');
    title(sprintf('분류 히트맵 (개선: %d개, 타협수렴(SPS동등): %d개, 에러: %d개)', improved_pts, neutral_pts, worsened_pts));
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)');
    cmap_discrete = [0.8500 0.1500 0.1980; 0.8 0.8 0.8; 0.0000 0.4470 0.7410];
    colormap(ax2, cmap_discrete); clim(ax2, [-1 1]);
    cb = colorbar(ax2); cb.Ticks = [-0.67, 0, 0.67]; cb.TickLabels = {'성능 위배', '타협 수렴', '성능 개선'};
    
    ax3 = subplot(2, 2, 3);
    surf(P_grid, V2_grid, D1_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('최종 생성된 Dataset 상태: Target D_1 (부드러움 유지)');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_1');
    colormap(ax3, 'turbo'); clim(ax3, [0.1 0.5]); view(-45, 30); grid on;

    ax4 = subplot(2, 2, 4);
    surf(P_grid, V2_grid, D2_grid, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('최종 생성된 Dataset 상태: Target D_2 (부드러움 유지)');
    xlabel('Power P (W)'); ylabel('Voltage V_2 (V)'); zlabel('D_2');
    colormap(ax4, 'turbo'); clim(ax4, [0.1 0.5]); view(-45, 30); grid on;
    drawnow;
end

% =========================================================================
% Helper Functions 
% =========================================================================
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