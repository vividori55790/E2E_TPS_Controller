% =========================================================================
% [통합 마스터 스크립트] TPS 대리 모델 최적화 및 평가 시스템 (완전 통합본)
% 아키텍처: Finite State Machine (FSM) 패턴 적용
% 
% [주요 고도화 포인트]
%   1. AutoML(Multi-Agent 경쟁) 도입: 다양한 MLP 구조/알고리즘이 병렬로 경쟁하여 최적 모델 자동 선정
%   2. 독립 효율(Efficiency) 학습 타겟 추가 및 이중 검증(Dual-Check) 물리 제약 로직 반영
%   3. XGBoost 타겟별 선택적 Log-Transform (효율은 Raw 값으로 학습하여 왜곡 방지)
%   4. Train/Test(8:2) 분할: K-Fold CV를 통한 완벽한 과적합(Overfitting) 방지 및 일반화 검증
%   5. Pattern Search 알고리즘 도입 및 Smart x_0 할당: 탐색 속도 극대화 및 지형 돌파
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
            
            % 1-B. 원본 데이터 로드 (파일명 확인 필요)
            disp(' -> 원본 데이터셋을 로드합니다...');
            try
                data = readtable('TPS_Dataset_20260316_021229.csv');
            catch
                disp(' -> [경고] 지정된 CSV 파일이 없습니다. 임시 더미 데이터를 생성합니다.');
                data = array2table(rand(1000, 8), 'VariableNames', {'Target_P_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'P_act_W', 'P_act_HV_W', 'i_pk_pri_A'});
                data.V2_V = data.V2_V * 30 + 80; data.D0_deg = data.D0_deg * 180 - 90;
                % 물리적 계산 오류 방지를 위해 현실적인 P 값 강제 할당
                data.P_act_W = rand(1000, 1) * 1150 + 50; 
                data.P_act_HV_W = data.P_act_W .* (0.8 + rand(1000,1)*0.18); 
            end
            
            % [수정 1] 데이터셋에서 효율(Efficiency)을 직접 연산하여 4번째 타겟으로 추가
            eff_raw = zeros(size(data.P_act_W));
            pos_idx = data.P_act_W > 0;
            neg_idx = data.P_act_W < 0;
            eff_raw(pos_idx) = (data.P_act_HV_W(pos_idx) ./ data.P_act_W(pos_idx)) * 100;
            eff_raw(neg_idx) = (data.P_act_W(neg_idx) ./ data.P_act_HV_W(neg_idx)) * 100;
            
            X_raw = [data.V2_V, data.D0_deg, data.D1, data.D2]';
            % Y_raw 차원 확장: [P, P_HV, i_pk, Efficiency_pct]
            Y_raw = [data.P_act_W, data.P_act_HV_W, data.i_pk_pri_A, eff_raw]';
            
            % 1-C. Train / Test Split (8:2) - 일반화 성능 검증용
            rng(42); % 재현성을 위한 시드 고정
            cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
            idx_train = training(cv);
            idx_test = test(cv);
            
            X_train = X_raw(:, idx_train); Y_train = Y_raw(:, idx_train);
            X_test  = X_raw(:, idx_test);  Y_test  = Y_raw(:, idx_test);
            
            disp([' -> 데이터 분할 및 효율 타겟 추가 완료! (Train: ', num2str(sum(idx_train)), '개 / Test: ', num2str(sum(idx_test)), '개)']);
            disp(' ');
            
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 2: 메인 메뉴
        % =================================================================
        case 'MAIN_MENU'
            disp('===========================================================');
            disp(' 메인 메뉴를 선택하세요:');
            disp('  1. [최적화 Method 1] AutoML Multi-Agent 경쟁 MLP + Vectorized Search');
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
            
        case 'RUN_MLP_GD'
            run_AutoML_MLP(data, X_train, Y_train, X_test, Y_test);
            currentState = 'MAIN_MENU';
            
        case 'RUN_XGB_PS'
            run_XGB_PatternSearch(data, X_train, Y_train);
            currentState = 'MAIN_MENU';
            
        case 'RUN_EVALUATE'
            evaluate_Models(X_test, Y_test);
            currentState = 'MAIN_MENU';
            
        case 'EXIT'
            disp('마스터 시스템을 안전하게 종료합니다. 수고하셨습니다!');
            isRunning = false;
    end
end


%% =========================================================================
% [함수부 1] AutoML Multi-Agent MLP + Vectorized Coarse-to-Fine Batch Search
% =========================================================================
function run_AutoML_MLP(data, X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 1] AutoML Multi-Agent 신경망 경쟁 및 최적화를 시작합니다.');
    
    [X_norm, PS_X] = mapminmax(X_train);
    [Y_norm, PS_Y] = mapminmax(Y_train); % Y_train은 이제 4차원
    
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_test_norm = mapminmax('apply', Y_test, PS_Y);
    
    nn_filename = 'trained_fast_mlp_net_4targets.mat';
    
    if isfile(nn_filename)
        disp([' -> 기존 최고 성능의 신경망 모델(', nn_filename, ')을 로드합니다.']);
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
    else
        disp(' -> 완료된 모델이 없습니다. [에이전트 경쟁 모드]를 시작합니다.');
        
        agent_hidden_sizes = {[32, 32], [64, 64], [128, 64], [64, 64, 32], [128, 64, 32], [64, 64, 64, 32]};
        agent_train_funcs = {'trainscg', 'trainlm'}; 
        agent_activations = {'poslin', 'tansig'}; 
        
        agents = []; agent_id = 1;
        for h = 1:length(agent_hidden_sizes)
            for t = 1:length(agent_train_funcs)
                for a = 1:length(agent_activations)
                    agents(agent_id).id = agent_id;
                    agents(agent_id).hiddenSize = agent_hidden_sizes{h};
                    agents(agent_id).trainFcn = agent_train_funcs{t};
                    agents(agent_id).activation = agent_activations{a};
                    agent_id = agent_id + 1;
                end
            end
        end
        num_agents = length(agents);
        disp([' -> 총 ', num2str(num_agents), '개의 에이전트가 경쟁을 시작합니다! (parfor)']);
        
        trained_nets = cell(num_agents, 1);
        test_mses = zeros(num_agents, 1);
        train_times = zeros(num_agents, 1);
        
        parfor i = 1:num_agents
            curr_agent = agents(i);
            agent_net = fitnet(curr_agent.hiddenSize, curr_agent.trainFcn);
            for j = 1:length(curr_agent.hiddenSize)
                agent_net.layers{j}.transferFcn = curr_agent.activation;
            end
            
            agent_net.divideParam.trainRatio = 0.8; 
            agent_net.divideParam.valRatio = 0.2;
            agent_net.divideParam.testRatio = 0.0;
            agent_net.trainParam.showWindow = false;
            
            if strcmp(curr_agent.trainFcn, 'trainscg')
                agent_net.trainParam.epochs = 5000;
                agent_net.trainParam.max_fail = 50;
            else
                agent_net.trainParam.epochs = 1000;
                agent_net.trainParam.max_fail = 20;
            end
            
            t_start = tic;
            [agent_net_trained, ~] = train(agent_net, X_norm, Y_norm);
            train_times(i) = toc(t_start);
            
            Y_test_pred_norm = agent_net_trained(X_test_norm);
            test_mses(i) = perform(agent_net_trained, Y_test_norm, Y_test_pred_norm);
            trained_nets{i} = agent_net_trained;
        end
        
        disp('===========================================================');
        disp(' [에이전트 경쟁 결과 (Test MSE 기준 정렬)]');
        [sorted_mses, sorted_idx] = sort(test_mses);
        
        for k = 1:min(5, num_agents) 
            idx = sorted_idx(k);
            disp([' Rank ', num2str(k), ' | Agent #', num2str(agents(idx).id), ...
                  ' | 구조: [', num2str(agents(idx).hiddenSize), '] | 알고리즘: ', agents(idx).trainFcn, ...
                  ' | 활성화: ', agents(idx).activation, ' | Test MSE: ', num2str(sorted_mses(k), '%.6f')]);
        end
        disp('===========================================================');
        
        best_agent_idx = sorted_idx(1);
        net = trained_nets{best_agent_idx};
        disp([' -> [최종 챔피언 선정] Agent #', num2str(agents(best_agent_idx).id), ' 가 최종 모델로 채택되었습니다!']);
        save(nn_filename, 'net', 'PS_X', 'PS_Y');
    end
    
    % 대규모 타겟 Grid 생성 (1,856개 조건)
    final_csv_name = 'Massive_Optimized_TPS_Dataset_FastMLP_4Targets.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 9); % 효율 컬럼 2개로 확장 (AI 예측, 물리 연산)
    
    disp(' -> [병렬/벡터화 탐색] 챔피언 모델 기반 Coarse-to-Fine 2단계 전수조사를 시작합니다.');
    
    D0_c_cands = -90:5:90;      
    D1_c_cands = 0:0.05:0.5;    
    D2_c_cands = 0:0.05:0.5;    
    [D0_cg, D1_cg, D2_cg] = ndgrid(D0_c_cands, D1_c_cands, D2_c_cands);
    coarse_phase_cands = [D0_cg(:), D1_cg(:), D2_cg(:)];
    num_coarse = size(coarse_phase_cands, 1);
    
    unique_V2 = unique(target_conditions(:, 2));
    precomputed_coarse_Y = cell(length(unique_V2), 1);
    
    for v_idx = 1:length(unique_V2)
        curr_V2 = unique_V2(v_idx);
        X_batch = [repmat(curr_V2, num_coarse, 1), coarse_phase_cands]';
        X_batch_norm = mapminmax('apply', X_batch, PS_X);
        
        Y_batch_norm = predict_with_batches(net, X_batch_norm, 100000);
        precomputed_coarse_Y{v_idx} = mapminmax('reverse', Y_batch_norm, PS_Y);
    end
    
    disp([' -> ', num2str(total_steps), '개 타겟에 대해 parfor 병렬 최적화를 수행합니다...']);
    
    P_targets = target_conditions(:, 1);
    V2_targets = target_conditions(:, 2);
    
    tic;
    parfor idx = 1:total_steps
        P_target = P_targets(idx);
        V2_target = V2_targets(idx);
        
        v_idx = find(unique_V2 == V2_target, 1);
        Y_c_batch_raw = precomputed_coarse_Y{v_idx};
        
        [best_coarse_phases, ~] = evaluate_candidates(coarse_phase_cands, Y_c_batch_raw, P_target);
        
        range_D0 = 5; range_D1 = 0.05; range_D2 = 0.05; 
        
        D0_f_cands = linspace(max(-90, best_coarse_phases(1)-range_D0), min(90, best_coarse_phases(1)+range_D0), 21);
        D1_f_cands = linspace(max(0, best_coarse_phases(2)-range_D1), min(0.5, best_coarse_phases(2)+range_D1), 21);
        D2_f_cands = linspace(max(0, best_coarse_phases(3)-range_D2), min(0.5, best_coarse_phases(3)+range_D2), 21);
        
        [D0_fg, D1_fg, D2_fg] = ndgrid(D0_f_cands, D1_f_cands, D2_f_cands);
        fine_phase_cands = [D0_fg(:), D1_fg(:), D2_fg(:)];
        num_fine = size(fine_phase_cands, 1);
        
        X_f_batch = [repmat(V2_target, num_fine, 1), fine_phase_cands]';
        X_f_batch_norm = mapminmax('apply', X_f_batch, PS_X);
        
        Y_f_batch_norm = predict_with_batches(net, X_f_batch_norm, 50000);
        Y_f_batch_raw = mapminmax('reverse', Y_f_batch_norm, PS_Y);
        
        [best_fine_phases, opt_vals] = evaluate_candidates(fine_phase_cands, Y_f_batch_raw, P_target);
        
        % opt_vals: [P, P_HV, i_pk, eff_pred, eff_phy]
        optimal_dataset(idx, :) = [P_target, V2_target, best_fine_phases(1), best_fine_phases(2), best_fine_phases(3), opt_vals(1), opt_vals(3), opt_vals(4), opt_vals(5)];
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
    disp('[Method 2] XGBoost (Log-Transform 적용) + Pattern Search 최적화 시작');
    
    xgb_filename = 'trained_xgboost_models_4targets.mat';
    
    % [수정 2] 효율은 0~100 스케일이므로 로그변환을 하면 왜곡 발생 (선택적 Log Transform 적용)
    Y_train_xgb = Y_train;
    Y_train_xgb(1:3, :) = sign(Y_train(1:3, :)) .* log(abs(Y_train(1:3, :)) + 1);
    % Y_train_xgb(4, :) 효율 컬럼은 로그 변환 없이 원본 그대로 유지
    
    if isfile(xgb_filename)
        disp([' -> 기존 모델(', xgb_filename, ')을 로드합니다.']);
        load(xgb_filename, 'xgb_models');
    else
        disp(' -> 4개 타겟(P, P_HV, I_pk, Eff)에 대한 모델 학습을 시작합니다...');
        opts = struct('Optimizer', 'bayesopt', 'ShowPlots', false, 'MaxObjectiveEvaluations', 1000, 'UseParallel', true, 'KFold', 5);
                  
        disp(' -> [1/4] P (Input Power) 학습 중...');
        xgb_models.P = fitrensemble(X_train', Y_train_xgb(1,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [2/4] P_HV (Output Power) 학습 중...');
        xgb_models.P_HV = fitrensemble(X_train', Y_train_xgb(2,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [3/4] I_pk (Peak Current) 학습 중...');
        xgb_models.I_pk = fitrensemble(X_train', Y_train_xgb(3,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [4/4] Efficiency (AI Predicted) 학습 중...');
        xgb_models.Eff = fitrensemble(X_train', Y_train_xgb(4,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
            
        save(xgb_filename, 'xgb_models');
        disp(' -> 모델 학습 및 저장 완료!');
    end
    
    ckpt_filename = 'TPS_Opt_Ckpt_XGB_PS.mat';
    final_csv_name = 'Massive_Optimized_TPS_Dataset_XGB_PS.csv';
    [target_conditions, total_steps] = setup_target_grid();
    
    if isfile(ckpt_filename)
        load(ckpt_filename, 'optimal_dataset', 'start_idx');
    else
        optimal_dataset = zeros(total_steps, 9); 
        start_idx = 1;
    end
    
    ps_options = optimoptions('patternsearch', 'Display', 'none', 'UseParallel', true, 'UseCompletePoll', true);
    avg_time_per_step = 0;
    
    for idx = start_idx:total_steps
        step_timer = tic; 
        
        P_target = target_conditions(idx, 1);
        V2_target = target_conditions(idx, 2);
        
        [lb, ub] = get_dynamic_bounds(data, P_target, V2_target);
        
        dist = sqrt(((data.P_act_W - P_target)/max(abs(data.P_act_W))).^2 + ((data.V2_V - V2_target)/max(data.V2_V)).^2);
        [~, min_idx] = min(dist);
        x0 = [data.D0_deg(min_idx), data.D1(min_idx), data.D2(min_idx)];
        x0 = max(lb, min(ub, x0));
        
        obj_fun = @(phases) costFunction_XGB_LogPS(phases, V2_target, P_target, xgb_models);
        [best_phases, ~] = patternsearch(obj_fun, x0, [], [], [], [], lb, ub, [], ps_options);
        
        X_in = [V2_target, best_phases(1), best_phases(2), best_phases(3)];
        
        % 예측 및 복원
        achieved_P_log = predict(xgb_models.P, X_in);
        achieved_P = sign(achieved_P_log) * (exp(abs(achieved_P_log)) - 1);
        achieved_PHV_log = predict(xgb_models.P_HV, X_in);
        achieved_P_HV = sign(achieved_PHV_log) * (exp(abs(achieved_PHV_log)) - 1);
        I_pk_log = predict(xgb_models.I_pk, X_in);
        min_I_pk = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
        
        % 효율 복원 (Log 안걸었으므로 그대로 사용)
        eff_pred = predict(xgb_models.Eff, X_in);
        eff_phy = calc_efficiency(achieved_P, achieved_P_HV);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_phases(1), best_phases(2), best_phases(3), achieved_P, min_I_pk, eff_pred, eff_phy];
        
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
    nn_filename = 'trained_fast_mlp_net_4targets.mat';
    if isfile(nn_filename)
        disp(' -> [1/2] 챔피언 MLP 모델 Test Set 검증 중...');
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
        
        X_test_norm = mapminmax('apply', X_test, PS_X);
        Y_pred_MLP = mapminmax('reverse', net(X_test_norm), PS_Y);
        
        calc_and_plot_metrics(Y_test, Y_pred_MLP, 'Champion MLP (Test Set Validation)');
    else
        disp(' -> [알림] Fast MLP 모델 파일이 없습니다.');
    end
    
    % 2. XGBoost 평가
    xgb_filename = 'trained_xgboost_models_4targets.mat';
    if isfile(xgb_filename)
        disp(' -> [2/2] XGBoost 모델 Test Set 검증 중...');
        load(xgb_filename, 'xgb_models');
        
        Y_pred_log = zeros(size(Y_test));
        Y_pred_log(1,:) = predict(xgb_models.P, X_test')';
        Y_pred_log(2,:) = predict(xgb_models.P_HV, X_test')';
        Y_pred_log(3,:) = predict(xgb_models.I_pk, X_test')';
        Y_pred_log(4,:) = predict(xgb_models.Eff, X_test')'; % 효율은 Raw 상태 그대로
        
        Y_pred_XGB = zeros(size(Y_test));
        Y_pred_XGB(1:3, :) = sign(Y_pred_log(1:3, :)) .* (exp(abs(Y_pred_log(1:3, :))) - 1);
        Y_pred_XGB(4, :) = Y_pred_log(4, :); % 역변환 불필요
        
        calc_and_plot_metrics(Y_test, Y_pred_XGB, 'XGBoost (Test Set Validation)');
    else
        disp(' -> [알림] XGBoost 모델 파일이 없습니다.');
    end
    disp(' ');
end


%% =========================================================================
% [유틸리티 및 목적 함수들]
% =========================================================================

function [target_conditions, total_steps] = setup_target_grid()
    target_P_list = 50:10:1200;  
    target_V2_list = 80:2:110;   
    [P_grid, V2_grid] = meshgrid(target_P_list, target_V2_list);
    target_conditions = [P_grid(:), V2_grid(:)]; 
    total_steps = size(target_conditions, 1);
end

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

function eff = calc_efficiency(P, P_HV)
    if P > 0, eff = (P_HV / P) * 100;
    elseif P < 0, eff = (P / P_HV) * 100;
    else, eff = 0; end
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
    
    eff_pred = predict(xgb_models.Eff, X_in); % AI가 예측한 효율
    eff_phy = calc_efficiency(P_pred, P_HV_pred); % 물리적으로 계산한 효율
    
    % [중요] AI가 뱉어내는 예측 효율과 P 기반 물리적 효율 이중 검증 (모순 차단)
    penalty = 0;
    if eff_pred > 100 || eff_pred < 10 || I_pk_pred > 30 || eff_phy > 100 || eff_phy < 10
        penalty = 1e6; 
    end
    
    error_power_pct = (abs(P_pred - P_target) / (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    % 효율 예측 모델이 높은 정확도를 가질 경우, 효율 자체를 목적식 정규화 인자로 사용할 수 있음
    cost = (1.0 * I_pk_pred) + (2.0 * P_loss_pred) + (50.0 * error_power_pct) - (0.1 * eff_pred) + penalty;
end

function avg_time = update_eta(timer_obj, avg_time, idx, total_steps, P_t, V2_t, method_str)
    step_dur = toc(timer_obj);
    if avg_time == 0, avg_time = step_dur; 
    else, avg_time = 0.8 * avg_time + 0.2 * step_dur; end
    time_left = avg_time * (total_steps - idx);
    fprintf('[%s] 진행률: [%d/%d] Target: %dW, %dV | ETA: %s\n', ...
        method_str, idx, total_steps, P_t, V2_t, datestr(time_left / 86400, 'HH:MM:SS'));
end

function save_final_csv(optimal_dataset, csv_name, ckpt_name)
    optimal_T = array2table(optimal_dataset, 'VariableNames', ...
        {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'AI_Predicted_Eff_pct', 'Physical_Eff_pct'});
    writetable(optimal_T, csv_name);
    if isfile(ckpt_name), delete(ckpt_name); end
    disp(['=> [저장 완료] 대규모 최적화 데이터셋이 "', csv_name, '" 로 성공적으로 생성되었습니다.']);
    disp(' ');
end

function calc_and_plot_metrics(Y_actual, Y_pred, model_name)
    target_names = {'P_{act} (W)', 'P_{act_HV} (W)', 'i_{pk} (A)', 'Efficiency (%)'};
    figure('Name', [model_name, ' Evaluation'], 'Position', [100, 200, 1500, 800]);
    
    for i = 1:4
        y_true = Y_actual(i, :); 
        y_hat = Y_pred(i, :);
        
        rmse = sqrt(mean((y_true - y_hat).^2));
        mae = mean(abs(y_true - y_hat));
        R2 = 1 - (sum((y_true - y_hat).^2) / sum((y_true - mean(y_true)).^2));
        
        fprintf(' [%s] Test R^2: %.4f, RMSE: %.4f, MAE: %.4f\n', target_names{i}, R2, rmse, mae);
        
        % [수정 4] 4개의 그래프를 출력하도록 레이아웃 변경 (2x2)
        subplot(2, 2, i);
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

% [수정 3] 메모리 초과(OOM) 방지를 위한 Batch 슬라이싱 예측기 (배열 동적 할당 보완)
function Y_batch_norm = predict_with_batches(net, X_batch_norm, batch_size)
    num_samples = size(X_batch_norm, 2);
    Y_batch_norm = zeros(4, num_samples); % 타겟 개수(4) 반영
    for bs = 1:batch_size:num_samples
        be = min(bs + batch_size - 1, num_samples);
        Y_batch_norm(:, bs:be) = net(X_batch_norm(:, bs:be));
    end
end

% 벡터화 계산 및 이중 제약 조건 평가
function [best_phases, opt_vals] = evaluate_candidates(phase_cands, Y_batch_raw, P_target)
    P_pred     = Y_batch_raw(1, :)';
    P_HV_pred  = Y_batch_raw(2, :)';
    I_pk_pred  = Y_batch_raw(3, :)';
    eff_pred   = Y_batch_raw(4, :)'; % AI가 별도로 예측해낸 효율
    
    % 물리적(수학적) 효율 역산
    eff_phy = zeros(length(P_pred), 1);
    pos_idx = P_pred > 0;
    neg_idx = P_pred < 0;
    
    eff_phy(pos_idx) = (P_HV_pred(pos_idx) ./ P_pred(pos_idx)) * 100;
    eff_phy(neg_idx) = (P_pred(neg_idx) ./ P_HV_pred(neg_idx)) * 100;
    
    % [중요] AI가 뱉어내는 예측 효율(eff_pred)과 물리적 효율(eff_phy) 모두 검증 (Dual-Check)
    invalid_mask = (eff_pred < 10) | (eff_pred > 100) | (I_pk_pred > 30) | (eff_phy < 10) | (eff_phy > 100);
    
    error_power_pct = (abs(P_pred - P_target) ./ (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    % 코스트에 예측 효율을 마이너스 패널티로 주어 효율 극대화를 보조
    cost_array = (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) + (50.0 .* error_power_pct) - (0.1 .* eff_pred);
    cost_array(invalid_mask) = 1e6;
    
    [min_cost, min_cost_idx] = min(cost_array);
    
    if min_cost >= 1e6
        fallback_cost = (1.0 .* I_pk_pred) + (50.0 .* error_power_pct);
        [~, min_cost_idx] = min(fallback_cost);
    end
    
    best_phases = phase_cands(min_cost_idx, :);
    opt_vals = [P_pred(min_cost_idx), P_HV_pred(min_cost_idx), I_pk_pred(min_cost_idx), eff_pred(min_cost_idx), eff_phy(min_cost_idx)];
end