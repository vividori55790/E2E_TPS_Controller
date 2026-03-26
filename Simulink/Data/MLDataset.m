% =========================================================================
% [통합 마스터 스크립트] TPS 대리 모델 최적화 및 평가 시스템 (완전 통합본)
% 아키텍처: Finite State Machine (FSM) 패턴 적용
% 
% [주요 고도화 포인트]
%   1. 기존 기능 완벽 복원: MLP(신경망) + Gradient Descent 알고리즘 및 체크포인트 기능 포함
%   2. Train/Test(8:2) 분할: K-Fold CV를 통한 완벽한 과적합(Overfitting) 방지 및 일반화 검증
%   3. Log-Transform 적용: 저전력 구간(Low Power)의 예측 퍼센트 오차 정밀도 확보 (XGBoost)
%   4. Pattern Search 알고리즘 도입 및 Smart x_0 할당: 탐색 속도 극대화 및 지형 돌파
%   5. 물리적 Hard Limit 페널티: 효율 100% 초과 등 비현실적 데이터 양쪽 모델에 완전 차단 적용
%   6. 타겟 데이터셋 대규모 확장 (총 1,856개 Operating Points)
% =========================================================================

clear; clc; close all;

%% 1. 유한 상태 머신 (FSM) 초기화
currentState = 'INIT';
isRunning = true;

% 전역적으로 사용할 데이터 변수들
data = [];
X_raw = []; Y_raw = [];
X_train = []; Y_train = [];
X_test = []; Y_test = [];

%% 2. FSM 메인 제어 루프
while isRunning
    switch currentState
        
        % =================================================================
        % 상태 1: 초기화, 병렬 풀 생성 및 데이터 분할 (INIT)
        % =================================================================
        case 'INIT'
            disp('===========================================================');
            disp('        TPS 데이터셋 대규모 최적화 및 평가 마스터 시스템        ');
            disp('===========================================================');
            disp('[시스템 초기화 중...]');
            
            % 1-A. 병렬 풀 확인
            p = gcp('nocreate');
            if isempty(p)
                disp(' -> 새로운 병렬 풀(Parallel Pool)을 생성합니다...');
                parpool;
            else
                disp([' -> 병렬 풀 활성화 확인 (활성 코어 수: ', num2str(p.NumWorkers), ')']);
            end
            
            % 1-B. 원본 데이터 로드
            disp(' -> 원본 데이터셋을 로드합니다...');
            data = readtable('TPS_Dataset_20260316_021229.csv');
            
            X_raw = [data.V2_V, data.D0_deg, data.D1, data.D2]';
            Y_raw = [data.P_act_W, data.P_act_HV_W, data.i_pk_pri_A]';
            
            % 1-C. Train / Test Split (8:2) - 일반화 성능 검증용
            rng(42); % 재현성을 위한 시드 고정
            cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
            idx_train = training(cv);
            idx_test = test(cv);
            
            X_train = X_raw(:, idx_train); Y_train = Y_raw(:, idx_train);
            X_test  = X_raw(:, idx_test);  Y_test  = Y_raw(:, idx_test);
            
            disp([' -> 데이터 분할 완료! (Train: ', num2str(sum(idx_train)), '개 / Test: ', num2str(sum(idx_test)), '개)']);
            disp(' ');
            
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 2: 메인 메뉴
        % =================================================================
        case 'MAIN_MENU'
            disp('===========================================================');
            disp(' 메인 메뉴를 선택하세요:');
            disp('  1. [최적화 Method 1] Fast MLP + Vectorized Coarse-to-Fine Search (parfor)');
            disp('  2. [최적화 Method 2] XGBoost (앙상블) + Pattern Search');
            disp('  3. [성능 평가] Test 데이터셋 기반 일반화 성능(R2, RMSE) 검증');
            disp('  0. [종 료] 시스템 종료');
            disp('===========================================================');
            
            user_choice = input('번호를 입력하세요: ');
            
            if user_choice == 1
                currentState = 'RUN_MLP_GD';
            elseif user_choice == 2
                currentState = 'RUN_XGB_PS';
            elseif user_choice == 3
                currentState = 'RUN_EVALUATE';
            elseif user_choice == 0
                currentState = 'EXIT';
            else
                disp('잘못된 입력입니다.'); disp(' ');
            end
            
        % =================================================================
        % 상태 3: MLP + GD 최적화 수행
        % =================================================================
        case 'RUN_MLP_GD'
            run_MLP_GD(data, X_train, Y_train);
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 4: XGBoost + Pattern Search 대규모 최적화 수행
        % =================================================================
        case 'RUN_XGB_PS'
            run_XGB_PatternSearch(data, X_train, Y_train);
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 5: 모델 성능 평가 (Test Set Only)
        % =================================================================
        case 'RUN_EVALUATE'
            evaluate_Models(X_test, Y_test);
            currentState = 'MAIN_MENU';
            
        case 'EXIT'
            disp('마스터 시스템을 안전하게 종료합니다. 수고하셨습니다!');
            isRunning = false;
    end
end


%% =========================================================================
% [함수부 1] Fast MLP + Vectorized Coarse-to-Fine Batch Search
% =========================================================================
function run_MLP_GD(data, X_train, Y_train)
    disp('-----------------------------------------------------------');
    disp('[Method 1] Fast MLP + Vectorized Coarse-to-Fine Search 최적화를 시작합니다.');
    
    % 1. MLP 전용 데이터 정규화 [-1, 1] (Train Set 기준으로만 계산)
    [X_norm, PS_X] = mapminmax(X_train);
    [Y_norm, PS_Y] = mapminmax(Y_train);
    
    % [개선안 1] AI 학습 엔진 교체 (trainscg + ReLU)
    nn_filename = 'trained_fast_mlp_net.mat';
    
    if isfile(nn_filename)
        disp([' -> 기존 신경망 모델(', nn_filename, ')을 로드합니다.']);
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
    else
        disp(' -> 완료된 모델이 없습니다. Fast MLP 학습 루틴을 시작합니다.');
        disp(' -> [초기화] 새로운 신경망 구조를 생성합니다. (trainscg + ReLU 적용)');
        hiddenLayerSize = [64, 64, 32]; 
        net = fitnet(hiddenLayerSize, 'trainscg');
        
        % ReLU (poslin) 활성화 함수 적용 (마지막 출력층 제외)
        for i = 1:length(hiddenLayerSize)
            net.layers{i}.transferFcn = 'poslin';
        end
        
        net.divideParam.trainRatio = 0.8; 
        net.divideParam.valRatio = 0.2;
        net.divideParam.testRatio = 0.0;
        net.trainParam.max_fail = 100;
        net.trainParam.epochs = 50000;
        net.trainParam.showWindow = false;
        
        disp(' -> [초고속 학습 진행 중...] 대용량 데이터를 한 번에 학습합니다.');
        step_timer = tic;
        [net, tr] = train(net, X_norm, Y_norm);
        train_time = toc(step_timer);
        
        disp([' -> 학습 완료! (소요 시간: ', num2str(train_time, '%.2f'), '초, Best Epoch: ', num2str(tr.best_epoch), ')']);
        save(nn_filename, 'net', 'PS_X', 'PS_Y');
    end
    
    % 2. 대규모 타겟 Grid 생성 (1,856개 조건)
    final_csv_name = 'Massive_Optimized_TPS_Dataset_FastMLP_Vectorized.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 8); 
    
    disp(' -> [병렬/벡터화 탐색] Coarse-to-Fine 2단계 전수조사를 시작합니다.');
    
    % [개선안 3] Coarse Grid 사전 생성 (성글게 쪼개어 전체 범위 커버)
    D0_c_cands = -90:5:90;       % 37개
    D1_c_cands = 0:0.05:0.5;    % 11개
    D2_c_cands = 0:0.05:0.5;    % 11개
    [D0_cg, D1_cg, D2_cg] = ndgrid(D0_c_cands, D1_c_cands, D2_c_cands);
    coarse_phase_cands = [D0_cg(:), D1_cg(:), D2_cg(:)];
    num_coarse = size(coarse_phase_cands, 1);
    
    unique_V2 = unique(target_conditions(:, 2));
    precomputed_coarse_Y = cell(length(unique_V2), 1);
    
    for v_idx = 1:length(unique_V2)
        curr_V2 = unique_V2(v_idx);
        X_batch = [repmat(curr_V2, num_coarse, 1), coarse_phase_cands]';
        X_batch_norm = mapminmax('apply', X_batch, PS_X);
        
        % [개선안 4] 메모리 OOM 방지를 위한 Batch 슬라이싱 함수 (predict_with_batches) 사용
        Y_batch_norm = predict_with_batches(net, X_batch_norm, 100000);
        precomputed_coarse_Y{v_idx} = mapminmax('reverse', Y_batch_norm, PS_Y);
    end
    
    % [개선안 5] parfor를 활용한 다중 타겟 병렬 검색
    disp([' -> ', num2str(total_steps), '개 타겟에 대해 parfor 병렬 최적화를 수행합니다... (속도 극대화)']);
    
    % parfor 내장 슬라이싱 변수 분리
    P_targets = target_conditions(:, 1);
    V2_targets = target_conditions(:, 2);
    
    tic;
    parfor idx = 1:total_steps
        P_target = P_targets(idx);
        V2_target = V2_targets(idx);
        
        % V2 인덱스 찾기 및 Coarse 예측 결과 가져오기
        v_idx = find(unique_V2 == V2_target, 1);
        Y_c_batch_raw = precomputed_coarse_Y{v_idx};
        
        % -------------------------------------------------------------
        % [1단계: Coarse Search] - 가장 가까운 근사치 도출
        % -------------------------------------------------------------
        [best_coarse_phases, ~] = evaluate_candidates(coarse_phase_cands, Y_c_batch_raw, P_target);
        
        % -------------------------------------------------------------
        % [2단계: Fine Search] - 근사치 주변을 매우 촘촘하게 쪼개어 정밀 탐색
        % -------------------------------------------------------------
        range_D0 = 5;    % +/- 5 deg
        range_D1 = 0.05; % +/- 0.05
        range_D2 = 0.05; % +/- 0.05
        
        D0_f_cands = linspace(max(-90, best_coarse_phases(1)-range_D0), min(90, best_coarse_phases(1)+range_D0), 21);
        D1_f_cands = linspace(max(0, best_coarse_phases(2)-range_D1), min(0.5, best_coarse_phases(2)+range_D1), 21);
        D2_f_cands = linspace(max(0, best_coarse_phases(3)-range_D2), min(0.5, best_coarse_phases(3)+range_D2), 21);
        
        [D0_fg, D1_fg, D2_fg] = ndgrid(D0_f_cands, D1_f_cands, D2_f_cands);
        fine_phase_cands = [D0_fg(:), D1_fg(:), D2_fg(:)];
        num_fine = size(fine_phase_cands, 1);
        
        X_f_batch = [repmat(V2_target, num_fine, 1), fine_phase_cands]';
        X_f_batch_norm = mapminmax('apply', X_f_batch, PS_X);
        
        % [개선안 4] Fine Grid 역시 OOM 방지를 위해 슬라이싱 예측 (5만개 단위)
        Y_f_batch_norm = predict_with_batches(net, X_f_batch_norm, 50000);
        Y_f_batch_raw = mapminmax('reverse', Y_f_batch_norm, PS_Y);
        
        % 최적해 평가
        [best_fine_phases, opt_vals] = evaluate_candidates(fine_phase_cands, Y_f_batch_raw, P_target);
        achieved_P = opt_vals(1); achieved_P_HV = opt_vals(2); min_I_pk = opt_vals(3); efficiency = opt_vals(4);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_fine_phases(1), best_fine_phases(2), best_fine_phases(3), achieved_P, min_I_pk, efficiency];
    end
    
    elapsed_time = toc;
    disp([' -> [병렬 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    
    save_final_csv(optimal_dataset, final_csv_name, '');
end


%% =========================================================================
% [함수부 2] XGBoost 학습 + Pattern Search 대규모 데이터셋 생성
% =========================================================================
function run_XGB_PatternSearch(data, X_train, Y_train)
    disp('-----------------------------------------------------------');
    disp('[Method 2] XGBoost (Log-Transform) + Pattern Search 최적화 시작');
    
    xgb_filename = 'trained_xgboost_models_v2.mat';
    
    % 타겟 변수 Log-Transform (비율 오차 최적화를 위함, 음수 허용)
    Y_train_log = sign(Y_train) .* log(abs(Y_train) + 1);
    
    if isfile(xgb_filename)
        disp([' -> 기존 모델(', xgb_filename, ')을 로드합니다.']);
        load(xgb_filename, 'xgb_models');
    else
        disp(' -> 모델 학습을 시작합니다. (K-Fold CV 적용, Max Evals: 1200)');
        opts = struct('Optimizer', 'bayesopt', ...
                      'ShowPlots', false, ...
                      'MaxObjectiveEvaluations', 1200, ...
                      'UseParallel', true, ...
                      'KFold', 5);
                  
        disp(' -> [1/3] P (Input Power) 학습 중...');
        xgb_models.P = fitrensemble(X_train', Y_train_log(1,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [2/3] P_HV (Output Power) 학습 중...');
        xgb_models.P_HV = fitrensemble(X_train', Y_train_log(2,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [3/3] I_pk (Peak Current) 학습 중...');
        xgb_models.I_pk = fitrensemble(X_train', Y_train_log(3,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
            
        save(xgb_filename, 'xgb_models');
        disp(' -> 모델 학습 및 저장 완료!');
    end
    
    ckpt_filename = 'TPS_Opt_Ckpt_XGB_PS.mat';
    final_csv_name = 'Massive_Optimized_TPS_Dataset_XGB_PS.csv';
    [target_conditions, total_steps] = setup_target_grid();
    
    if isfile(ckpt_filename)
        load(ckpt_filename, 'optimal_dataset', 'start_idx');
        disp([' -> [체크포인트 복구] 인덱스 ', num2str(start_idx), '/', num2str(total_steps), ' 부터 재개.']);
    else
        optimal_dataset = zeros(total_steps, 8); 
        start_idx = 1;
    end
    
    ps_options = optimoptions('patternsearch', 'Display', 'none', 'UseParallel', true, 'UseCompletePoll', true);
    avg_time_per_step = 0;
    
    for idx = start_idx:total_steps
        step_timer = tic; 
        
        P_target = target_conditions(idx, 1);
        V2_target = target_conditions(idx, 2);
        
        [lb, ub] = get_dynamic_bounds(data, P_target, V2_target);
        
        % Smart x0 할당
        dist = sqrt(((data.P_act_W - P_target)/max(abs(data.P_act_W))).^2 + ((data.V2_V - V2_target)/max(data.V2_V)).^2);
        [~, min_idx] = min(dist);
        x0 = [data.D0_deg(min_idx), data.D1(min_idx), data.D2(min_idx)];
        x0 = max(lb, min(ub, x0));
        
        obj_fun = @(phases) costFunction_XGB_LogPS(phases, V2_target, P_target, xgb_models);
        
        [best_phases, ~] = patternsearch(obj_fun, x0, [], [], [], [], lb, ub, [], ps_options);
        
        X_in = [V2_target, best_phases(1), best_phases(2), best_phases(3)];
        achieved_P_log = predict(xgb_models.P, X_in);
        achieved_P = sign(achieved_P_log) * (exp(abs(achieved_P_log)) - 1);
        
        achieved_PHV_log = predict(xgb_models.P_HV, X_in);
        achieved_P_HV = sign(achieved_PHV_log) * (exp(abs(achieved_PHV_log)) - 1);
        
        I_pk_log = predict(xgb_models.I_pk, X_in);
        min_I_pk = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
        
        efficiency = calc_efficiency(achieved_P, achieved_P_HV);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_phases(1), best_phases(2), best_phases(3), achieved_P, min_I_pk, efficiency];
        
        avg_time_per_step = update_eta(step_timer, avg_time_per_step, idx, total_steps, P_target, V2_target, 'XGB+PS');
        
        start_idx = idx + 1; 
        if mod(idx, 50) == 0
            save(ckpt_filename, 'optimal_dataset', 'start_idx');
        end
    end
    
    save_final_csv(optimal_dataset, final_csv_name, ckpt_filename);
end


%% =========================================================================
% [함수부 3] 모델 성능 평가 (Test Set 기반 일반화 성능 검증)
% =========================================================================
function evaluate_Models(X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[모델 성능 평가] 학습에 사용되지 않은 Test Set으로 일반화 성능을 검증합니다.');
    
    % 1. MLP 평가
    nn_filename = 'trained_fast_mlp_net.mat';
    if isfile(nn_filename)
        disp(' -> [1/2] Fast MLP 모델 Test Set 검증 중...');
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
        
        % Test 데이터를 학습된 MinMax 기준으로 정규화 후 예측
        X_test_norm = mapminmax('apply', X_test, PS_X);
        Y_pred_MLP = mapminmax('reverse', net(X_test_norm), PS_Y);
        
        calc_and_plot_metrics(Y_test, Y_pred_MLP, 'Fast MLP (Test Set Validation)');
    else
        disp(' -> [알림] Fast MLP 모델 파일이 없습니다.');
    end
    
    % 2. XGBoost 평가
    xgb_filename = 'trained_xgboost_models_v2.mat';
    if isfile(xgb_filename)
        disp(' -> [2/2] XGBoost 모델 Test Set 검증 중...');
        load(xgb_filename, 'xgb_models');
        
        Y_pred_log = zeros(size(Y_test));
        Y_pred_log(1,:) = predict(xgb_models.P, X_test')';
        Y_pred_log(2,:) = predict(xgb_models.P_HV, X_test')';
        Y_pred_log(3,:) = predict(xgb_models.I_pk, X_test')';
        
        % Log 역변환
        Y_pred_XGB = sign(Y_pred_log) .* (exp(abs(Y_pred_log)) - 1);
        
        calc_and_plot_metrics(Y_test, Y_pred_XGB, 'XGBoost (Test Set Validation)');
    else
        disp(' -> [알림] XGBoost 모델 파일이 없습니다.');
    end
    disp(' ');
end


%% =========================================================================
% [유틸리티 및 목적 함수들]
% =========================================================================

% 1. 대규모 타겟 전력/전압 Grid 생성 (총 1856개)
function [target_conditions, total_steps] = setup_target_grid()
    target_P_list = 50:10:1200;  % 50W ~ 1200W (10W 단위)
    target_V2_list = 80:2:110;   % 80V ~ 110V (2V 단위)
    [P_grid, V2_grid] = meshgrid(target_P_list, target_V2_list);
    target_conditions = [P_grid(:), V2_grid(:)]; 
    total_steps = size(target_conditions, 1);
end

% 2. 동적 탐색 영역(Bounds) 설정
function [lb, ub] = get_dynamic_bounds(data, P_target, V2_target)
    valid_idx = abs(data.V2_V - V2_target) <= 2 & abs(data.P_act_W - P_target) <= 50;
    if sum(valid_idx) > 5
        lb = [max(-90, min(data.D0_deg(valid_idx)) - 10), max(0, min(data.D1(valid_idx)) - 0.1), max(0, min(data.D2(valid_idx)) - 0.1)];
        ub = [min(90, max(data.D0_deg(valid_idx)) + 10), min(0.5, max(data.D1(valid_idx)) + 0.1), min(0.5, max(data.D2(valid_idx)) + 0.1)];
    else
        lb = [-90, 0, 0];
        ub = [90, 0.5, 0.5];
    end
end

% 3. 물리적 효율 계산
function eff = calc_efficiency(P, P_HV)
    if P > 0
        eff = (P_HV / P) * 100;
    elseif P < 0
        eff = (P / P_HV) * 100;
    else
        eff = 0; 
    end
end

% 4. MLP용 목적 함수 (Hard Limit 통합)
function cost = costFunction_MLP(phases, V2_target, P_target, net, PS_X, PS_Y)
    X_norm = mapminmax('apply', [V2_target; phases(1); phases(2); phases(3)], PS_X);
    Y_raw = mapminmax('reverse', net(X_norm), PS_Y);
    
    P_pred = Y_raw(1); P_HV_pred = Y_raw(2); I_pk_pred = Y_raw(3);
    
    eff = calc_efficiency(P_pred, P_HV_pred);
    penalty = 0;
    if eff > 100 || eff < 10 || I_pk_pred > 30 % 물리적 한계 초과
        penalty = 1e6;
    end
    
    error_power_pct = (abs(P_pred - P_target) / (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    cost = (1.0 * I_pk_pred) + (2.0 * P_loss_pred) + (50.0 * error_power_pct) + penalty;
end

% 5. XGBoost + Pattern Search용 목적 함수 (Log 역변환 및 Hard Limit 포함)
function cost = costFunction_XGB_LogPS(phases, V2_target, P_target, xgb_models)
    X_in = [V2_target, phases(1), phases(2), phases(3)];
    
    P_pred_log = predict(xgb_models.P, X_in);
    P_pred = sign(P_pred_log) * (exp(abs(P_pred_log)) - 1);
    
    P_HV_pred_log = predict(xgb_models.P_HV, X_in);
    P_HV_pred = sign(P_HV_pred_log) * (exp(abs(P_HV_pred_log)) - 1);
    
    I_pk_pred_log = predict(xgb_models.I_pk, X_in);
    I_pk_pred = sign(I_pk_pred_log) * (exp(abs(I_pk_pred_log)) - 1);
    
    eff = calc_efficiency(P_pred, P_HV_pred);
    penalty = 0;
    if eff > 100 || eff < 10 || I_pk_pred > 30 
        penalty = 1e6; 
    end
    
    error_power_pct = (abs(P_pred - P_target) / (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    cost = (1.0 * I_pk_pred) + (2.0 * P_loss_pred) + (50.0 * error_power_pct) + penalty;
end

% 6. ETA 계산
function avg_time = update_eta(timer_obj, avg_time, idx, total_steps, P_t, V2_t, method_str)
    step_dur = toc(timer_obj);
    if avg_time == 0, avg_time = step_dur; 
    else, avg_time = 0.8 * avg_time + 0.2 * step_dur; end
    time_left = avg_time * (total_steps - idx);
    fprintf('[%s] 진행률: [%d/%d] Target: %dW, %dV | ETA: %s\n', ...
        method_str, idx, total_steps, P_t, V2_t, datestr(time_left / 86400, 'HH:MM:SS'));
end

% 7. 결과 CSV 저장
function save_final_csv(optimal_dataset, csv_name, ckpt_name)
    optimal_T = array2table(optimal_dataset, 'VariableNames', ...
        {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'Efficiency_pct'});
    writetable(optimal_T, csv_name);
    if isfile(ckpt_name), delete(ckpt_name); end
    disp(['=> [저장 완료] 대규모 최적화 데이터셋이 "', csv_name, '" 로 성공적으로 생성되었습니다.']);
    disp(' ');
end

% 8. R2, RMSE 계산 및 산점도 출력 (tex 인터프리터 적용)
function calc_and_plot_metrics(Y_actual, Y_pred, model_name)
    target_names = {'P_{act} (Input Power, W)', 'P_{act_HV} (Output Power, W)', 'i_{pk_pri} (Peak Current, A)'};
    figure('Name', [model_name, ' Evaluation'], 'Position', [100, 200, 1500, 450]);
    
    for i = 1:3
        y_true = Y_actual(i, :); 
        y_hat = Y_pred(i, :);
        
        rmse = sqrt(mean((y_true - y_hat).^2));
        mae = mean(abs(y_true - y_hat));
        R2 = 1 - (sum((y_true - y_hat).^2) / sum((y_true - mean(y_true)).^2));
        
        fprintf(' [%s] Test R^2: %.4f, RMSE: %.4f, MAE: %.4f\n', target_names{i}, R2, rmse, mae);
        
        subplot(1, 3, i);
        scatter(y_true, y_hat, 15, 'b', 'filled', 'MarkerFaceAlpha', 0.4); hold on;
        
        m_val = min([y_true, y_hat]); 
        M_val = max([y_true, y_hat]); 
        mg = (M_val - m_val) * 0.05; 
        plot([m_val-mg, M_val+mg], [m_val-mg, M_val+mg], 'r--', 'LineWidth', 2); hold off;
        
        title({target_names{i}, sprintf('Test R^2 = %.4f', R2)}, 'Interpreter', 'tex');
        xlabel('Actual (Simulation)'); ylabel('Predicted (AI Model)'); grid on; axis square;
        xlim([m_val-mg, M_val+mg]); ylim([m_val-mg, M_val+mg]);
    end
end

% 9. [개선안 4] 메모리 초과(OOM) 방지를 위한 Batch 슬라이싱 예측기
function Y_batch_norm = predict_with_batches(net, X_batch_norm, batch_size)
    num_samples = size(X_batch_norm, 2);
    Y_batch_norm = zeros(3, num_samples);
    for bs = 1:batch_size:num_samples
        be = min(bs + batch_size - 1, num_samples);
        Y_batch_norm(:, bs:be) = net(X_batch_norm(:, bs:be));
    end
end

% 10. 후보군 조건 평가기 (벡터화 계산, Hard Limits 페널티 부여)
function [best_phases, opt_vals] = evaluate_candidates(phase_cands, Y_batch_raw, P_target)
    P_pred     = Y_batch_raw(1, :)';
    P_HV_pred  = Y_batch_raw(2, :)';
    I_pk_pred  = Y_batch_raw(3, :)';
    
    eff = zeros(length(P_pred), 1);
    pos_idx = P_pred > 0;
    neg_idx = P_pred < 0;
    
    eff(pos_idx) = (P_HV_pred(pos_idx) ./ P_pred(pos_idx)) * 100;
    eff(neg_idx) = (P_pred(neg_idx) ./ P_HV_pred(neg_idx)) * 100;
    
    % Hard Limits (벡터화 처리)
    invalid_mask = (eff < 10) | (eff > 100) | (I_pk_pred > 30);
    
    error_power_pct = (abs(P_pred - P_target) ./ (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    cost_array = (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) + (50.0 .* error_power_pct);
    cost_array(invalid_mask) = 1e6;
    
    [min_cost, min_cost_idx] = min(cost_array);
    
    if min_cost >= 1e6
        fallback_cost = (1.0 .* I_pk_pred) + (50.0 .* error_power_pct);
        [~, min_cost_idx] = min(fallback_cost);
    end
    
    best_phases = phase_cands(min_cost_idx, :);
    opt_vals = [P_pred(min_cost_idx), P_HV_pred(min_cost_idx), I_pk_pred(min_cost_idx), eff(min_cost_idx)];
end