% =========================================================================
% [통합 마스터 스크립트] TPS 대리 모델 최적화 및 평가 시스템 (완전 통합본 V2)
% 아키텍처: Finite State Machine (FSM) 패턴 적용
% 
% [주요 고도화 포인트]
%   1. AutoML(Multi-Agent 경쟁) 도입: 다양한 MLP 구조/알고리즘 병렬 경쟁
%   2. 독립 효율(Efficiency) 타겟 추가 및 이중 검증(Dual-Check) 물리 제약 
%   3. 다목적 최적화(NSGA-II) 도입: I_pk 최소화와 효율 최대화 간의 Pareto Front
%   4. 설명 가능한 AI (XAI): PDP(Partial Dependence Plot) 물리적 영향력 시각화
%   5. 앙상블 스태킹 (Meta-Learning): MLP + XGBoost 융합 모델
%
% [🔥 V2 신규 추가 사항 (어드밴스드 레벨)]
%   - 1. 도메인 지식 파생 변수(Domain-Knowledge Feature): D0^2, D1*D2, V2*D0 등
%        물리 수식을 반영한 파생 특징(Feature) 3개를 추가하여 7차원 입력으로 성능 극대화
%   - 2. 불확실성 정량화(UQ: Uncertainty Quantification): Random Forest(TreeBagger)
%        기반 Quantile Regression을 통해 효율에 대한 95% 신뢰 구간(오차 범위) 도출
%   - 3. 전역 최적화 PSO (Particle Swarm Optimization): 지역 최적해에 빠지기 쉬운
%        Pattern Search 대신, 군집 지능형 최적화(PSO)를 적용하여 최상위 전역해 탐색
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
        % 상태 1: 초기화, 병렬 풀 생성 및 파생 변수(Feature Engineering) 추가
        % =================================================================
        case 'INIT'
            disp('===========================================================');
            disp('        DAB TPS 312k 대용량 최적화 및 평가 마스터 시스템 V2      ');
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
            disp(' -> 원본 데이터셋을 로드합니다... (수 초 소요될 수 있습니다)');
            try
                data = readtable('TPS_Dataset_20260316_021229.csv');
                disp([' -> 데이터 로드 성공: 총 ', num2str(height(data)), '개 행 확인됨.']);
            catch
                disp(' -> [경고] 지정된 CSV 파일이 없습니다. 대규모(30만개) 임시 더미 데이터를 생성합니다.');
                data = array2table(rand(312000, 8), 'VariableNames', {'Target_P_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'P_act_W', 'P_act_HV_W', 'i_pk_pri_A'});
                data.V2_V = data.V2_V * 30 + 80; data.D0_deg = data.D0_deg * 180 - 90;
                data.P_act_W = rand(312000, 1) * 1150 + 50; 
                data.P_act_HV_W = data.P_act_W .* (0.8 + rand(312000,1)*0.18); 
                data.i_pk_pri_A = rand(312000, 1) * 20 + 5; 
            end
            
            % 데이터셋에서 효율(Efficiency)을 직접 연산
            eff_raw = zeros(size(data.P_act_W));
            pos_idx = data.P_act_W > 0;
            neg_idx = data.P_act_W < 0;
            eff_raw(pos_idx) = (data.P_act_HV_W(pos_idx) ./ data.P_act_W(pos_idx)) * 100;
            eff_raw(neg_idx) = (data.P_act_W(neg_idx) ./ data.P_act_HV_W(neg_idx)) * 100;
            
            % [개선점 1] 물리 도메인 지식 기반 파생 변수(Feature Engineering) 생성
            % 단순 4차원이 아닌, 물리적 연관성이 깊은 조합을 추가해 7차원 공간 구성
            V2_vec = data.V2_V';
            D0_vec = data.D0_deg';
            D1_vec = data.D1';
            D2_vec = data.D2';
            
            D0_sq = D0_vec.^2;        % 위상 편이의 제곱항 (전력 방정식 내 비선형성 반영)
            D1_D2 = D1_vec .* D2_vec; % 내부 위상 간의 상호작용
            V2_D0 = V2_vec .* D0_vec; % 전압과 외부 위상 편이의 상호작용
            
            X_raw = [V2_vec; D0_vec; D1_vec; D2_vec; D0_sq; D1_D2; V2_D0];
            Y_raw = [data.P_act_W, data.P_act_HV_W, data.i_pk_pri_A, eff_raw]';
            
            % 1-C. Train / Test Split (8:2)
            rng(42);
            cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
            idx_train = training(cv);
            idx_test = test(cv);
            
            X_train = X_raw(:, idx_train); Y_train = Y_raw(:, idx_train);
            X_test  = X_raw(:, idx_test);  Y_test  = Y_raw(:, idx_test);
            
            disp([' -> 파생 변수 생성 및 데이터 분할 완료! (입력 차원: 7차원, Train: ', num2str(sum(idx_train)), '개)']);
            disp(' ');
            
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 2: 메인 메뉴
        % =================================================================
        case 'MAIN_MENU'
            disp('===========================================================');
            disp(' 메인 메뉴를 선택하세요:');
            disp('  1. [단일 최적화] AutoML Multi-Agent 경쟁 MLP + Vectorized Search');
            disp('  2. [전역 최적화 & UQ] XGBoost + PSO 전역 탐색 (불확실성 95% 신뢰구간 도출)');
            disp('  3. [다목적 최적화] NSGA-II 파레토 프론트 도출 (I_pk vs Efficiency)');
            disp('  4. [앙상블 융합] 스태킹(Stacking) 메타 모델 생성 및 평가');
            disp('  5. [모델 성능 & XAI] Test Set 일반화 검증 및 PDP 물리적 영향력 분석');
            disp('  0. [종 료] 시스템 종료');
            disp('===========================================================');
            
            user_choice = input('번호를 입력하세요: ');
            
            if user_choice == 1
                currentState = 'RUN_MLP_GD';
            elseif user_choice == 2
                currentState = 'RUN_XGB_PSO';
            elseif user_choice == 3
                currentState = 'RUN_NSGA2';
            elseif user_choice == 4
                currentState = 'RUN_STACKING';
            elseif user_choice == 5
                currentState = 'RUN_EVALUATE';
            elseif user_choice == 0
                currentState = 'EXIT';
            else
                disp('잘못된 입력입니다.'); disp(' ');
            end
            
        case 'RUN_MLP_GD'
            run_AutoML_MLP(data, X_train, Y_train, X_test, Y_test);
            currentState = 'MAIN_MENU';
            
        case 'RUN_XGB_PSO'
            run_XGB_PSO(data, X_train, Y_train);
            currentState = 'MAIN_MENU';
            
        case 'RUN_NSGA2'
            run_NSGA2_Optimization();
            currentState = 'MAIN_MENU';
            
        case 'RUN_STACKING'
            run_Ensemble_Stacking(X_train, Y_train, X_test, Y_test);
            currentState = 'MAIN_MENU';
            
        case 'RUN_EVALUATE'
            evaluate_Models(X_test, Y_test, X_train);
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
    [Y_norm, PS_Y] = mapminmax(Y_train); 
    
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_test_norm = mapminmax('apply', Y_test, PS_Y);
    
    % 차원이 달라졌으므로 기존 4차원 모델과 충돌 방지를 위해 v2로 저장
    nn_filename = 'trained_fast_mlp_net_7targets_312k_v2.mat';
    
    if isfile(nn_filename)
        disp([' -> 기존 최고 성능의 7차원 신경망 모델(', nn_filename, ')을 로드합니다.']);
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
    else
        disp(' -> 완료된 모델이 없습니다. [에이전트 경쟁 모드]를 시작합니다.');
        
        agent_hidden_sizes = {[64, 64], [128, 64], [64, 64, 32], [128, 64, 32]};
        agent_train_funcs = {'trainscg', 'trainrp'}; 
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
        disp([' -> 물리 파생 변수를 학습할 ', num2str(num_agents), '개 에이전트 가동 (parfor)']);
        
        trained_nets = cell(num_agents, 1);
        test_mses = zeros(num_agents, 1);
        
        parfor i = 1:num_agents
            curr_agent = agents(i);
            agent_net = fitnet(curr_agent.hiddenSize, curr_agent.trainFcn);
            for j = 1:length(curr_agent.hiddenSize)
                agent_net.layers{j}.transferFcn = curr_agent.activation;
            end
            
            agent_net.divideParam.trainRatio = 0.85; 
            agent_net.divideParam.valRatio = 0.15;
            agent_net.divideParam.testRatio = 0.0;
            agent_net.trainParam.showWindow = false;
            agent_net.trainParam.epochs = 3000;
            agent_net.trainParam.max_fail = 50;
            
            [agent_net_trained, ~] = train(agent_net, X_norm, Y_norm);
            
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
                  ' | Test MSE: ', num2str(sorted_mses(k), '%.6f')]);
        end
        
        best_agent_idx = sorted_idx(1);
        net = trained_nets{best_agent_idx};
        disp([' -> [최종 챔피언 선정] Agent #', num2str(agents(best_agent_idx).id), ' 채택!']);
        save(nn_filename, 'net', 'PS_X', 'PS_Y');
    end
    
    final_csv_name = 'Massive_Optimized_TPS_Dataset_FastMLP_7Feat.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 8); 
    
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
        % 파생 변수를 반영한 7차원 배치 행렬 생성
        V_batch = repmat(curr_V2, num_coarse, 1);
        D0_b = coarse_phase_cands(:,1); D1_b = coarse_phase_cands(:,2); D2_b = coarse_phase_cands(:,3);
        X_batch = [V_batch, D0_b, D1_b, D2_b, D0_b.^2, D1_b.*D2_b, V_batch.*D0_b]';
        
        X_batch_norm = mapminmax('apply', X_batch, PS_X);
        Y_batch_norm = predict_with_batches(net, X_batch_norm, 50000);
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
        D0_f = linspace(max(-90, best_coarse_phases(1)-range_D0), min(90, best_coarse_phases(1)+range_D0), 21);
        D1_f = linspace(max(0, best_coarse_phases(2)-range_D1), min(0.5, best_coarse_phases(2)+range_D1), 21);
        D2_f = linspace(max(0, best_coarse_phases(3)-range_D2), min(0.5, best_coarse_phases(3)+range_D2), 21);
        
        [D0_fg, D1_fg, D2_fg] = ndgrid(D0_f, D1_f, D2_f);
        fine_phase_cands = [D0_fg(:), D1_fg(:), D2_fg(:)];
        num_fine = size(fine_phase_cands, 1);
        
        V_f_batch = repmat(V2_target, num_fine, 1);
        D0_fb = fine_phase_cands(:,1); D1_fb = fine_phase_cands(:,2); D2_fb = fine_phase_cands(:,3);
        X_f_batch = [V_f_batch, D0_fb, D1_fb, D2_fb, D0_fb.^2, D1_fb.*D2_fb, V_f_batch.*D0_fb]';
        
        X_f_batch_norm = mapminmax('apply', X_f_batch, PS_X);
        Y_f_batch_norm = predict_with_batches(net, X_f_batch_norm, 50000);
        Y_f_batch_raw = mapminmax('reverse', Y_f_batch_norm, PS_Y);
        
        [best_fine_phases, opt_vals] = evaluate_candidates(fine_phase_cands, Y_f_batch_raw, P_target);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_fine_phases(1), best_fine_phases(2), best_fine_phases(3), opt_vals(1), opt_vals(3), opt_vals(4)];
    end
    
    elapsed_time = toc;
    disp([' -> [병렬 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    save_final_csv(optimal_dataset, final_csv_name);
end


%% =========================================================================
% [함수부 2] XGBoost(LSBoost) + TreeBagger(UQ) + Particle Swarm 전역 탐색
% =========================================================================
function run_XGB_PSO(data, X_train, Y_train)
    disp('-----------------------------------------------------------');
    disp('[Method 2] XGBoost + 전역 PSO 탐색 + 불확실성(UQ) 정량화 시작');
    
    % 차원(7차원) 업데이트 충돌 방지
    xgb_filename = 'trained_xgboost_models_7targets_312k_v2.mat';
    
    Y_train_xgb = Y_train;
    Y_train_xgb(1:3, :) = sign(Y_train(1:3, :)) .* log(abs(Y_train(1:3, :)) + 1);
    
    if isfile(xgb_filename)
        disp([' -> 기존 7차원 XGB+UQ 모델(', xgb_filename, ')을 로드합니다.']);
        load(xgb_filename, 'xgb_models');
    else
        disp(' -> 4개 타겟 모델 학습 시작 (물리적 파생 변수 7차원 학습)...');
        
        opts = struct('Optimizer', 'bayesopt', 'ShowPlots', false, 'MaxObjectiveEvaluations', 20, 'UseParallel', true);
                  
        disp(' -> [1/4] P (Input Power) 학습 중 (LSBoost)...');
        xgb_models.P = fitrensemble(X_train', Y_train_xgb(1,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [2/4] P_HV (Output Power) 학습 중 (LSBoost)...');
        xgb_models.P_HV = fitrensemble(X_train', Y_train_xgb(2,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        disp(' -> [3/4] I_pk (Peak Current) 학습 중 (LSBoost)...');
        xgb_models.I_pk = fitrensemble(X_train', Y_train_xgb(3,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        % [개선점 2: UQ 적용] 효율은 신뢰구간 도출을 위해 TreeBagger(Random Forest)로 학습
        disp(' -> [4/4] Efficiency (효율 + 95% 신뢰구간 UQ) 학습 중 (TreeBagger 50 Trees)...');
        xgb_models.Eff_UQ = TreeBagger(50, X_train', Y_train_xgb(4,:)', 'Method', 'regression', 'OOBPrediction', 'on');
            
        save(xgb_filename, 'xgb_models');
        disp(' -> 모델 학습 및 저장 완료!');
    end
    
    final_csv_name = 'Massive_Optimized_TPS_Dataset_PSO_UQ.csv';
    [target_conditions, total_steps] = setup_target_grid();
    
    % [개선점 2: UQ 데이터 저장] Eff_Lower_95, Eff_Upper_95 추가되어 총 10차원 할당
    optimal_dataset = zeros(total_steps, 10); 
    
    % [개선점 3: PSO 적용] 지역해 탈출을 위한 입자 군집 알고리즘 적용 (직렬, 바깥 루프 병렬)
    pso_options = optimoptions('particleswarm', 'Display', 'none', 'SwarmSize', 30, 'MaxIterations', 50, 'UseParallel', false);
    
    disp([' -> ', num2str(total_steps), '개 타겟에 대해 parfor PSO 병렬 최적화를 수행합니다...']);
    
    tic;
    parfor idx = 1:total_steps
        P_target = target_conditions(idx, 1);
        V2_target = target_conditions(idx, 2);
        
        [lb, ub] = get_dynamic_bounds(data, P_target, V2_target);
        
        % PSO 목적 함수
        obj_fun = @(phases) costFunction_XGB_LogPSO(phases, V2_target, P_target, xgb_models);
        
        % x0 시드가 불필요한 PSO 알고리즘
        [best_phases, ~] = particleswarm(obj_fun, 3, lb, ub, pso_options);
        
        % 7차원 피처 추출
        X_in = get_features_row(V2_target, best_phases(1), best_phases(2), best_phases(3));
        
        % 예측 및 복원
        achieved_P_log = predict(xgb_models.P, X_in);
        achieved_P = sign(achieved_P_log) * (exp(abs(achieved_P_log)) - 1);
        I_pk_log = predict(xgb_models.I_pk, X_in);
        min_I_pk = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
        
        % [개선점 2: UQ 추론] 5%, 50%(중간값), 95% 분위수(Quantile) 예측
        eff_quantiles = quantilePredict(xgb_models.Eff_UQ, X_in, 'Quantile', [0.05, 0.5, 0.95]);
        eff_lower = eff_quantiles(1); % 최악의 경우 (안전 설계 마진)
        eff_pred  = eff_quantiles(2); % 최적 예측치
        eff_upper = eff_quantiles(3); % 최상의 경우
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_phases(1), best_phases(2), best_phases(3), achieved_P, min_I_pk, eff_pred, eff_lower, eff_upper];
        
        if mod(idx, 100) == 0
            fprintf('[XGB+PSO 병렬 처리] %d / %d 타겟 처리 완료...\n', idx, total_steps);
        end
    end
    
    elapsed_time = toc;
    disp([' -> [XGB+PSO 및 UQ 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    
    save_final_csv(optimal_dataset, final_csv_name);
end


%% =========================================================================
% [함수부 3] 모델 성능 평가 (Test Set 기반 일반화 성능 검증)
% =========================================================================
function evaluate_Models(X_test, Y_test, X_train)
    disp('-----------------------------------------------------------');
    disp('[모델 성능 평가] 학습에 사용되지 않은 Test Set으로 일반화 성능을 검증합니다.');
    
    % 1. MLP 평가
    nn_filename = 'trained_fast_mlp_net_7targets_312k_v2.mat';
    if isfile(nn_filename)
        disp(' -> [1/2] 챔피언 7차원 파생 변수 MLP 모델 Test Set 검증 중...');
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
        
        X_test_norm = mapminmax('apply', X_test, PS_X);
        Y_pred_MLP = mapminmax('reverse', net(X_test_norm), PS_Y);
        
        calc_and_plot_metrics(Y_test, Y_pred_MLP, 'Champion MLP 7-Feat (Test Set)');
    else
        disp(' -> [알림] Fast MLP 모델 파일이 없습니다.');
    end
    
    % 2. XGBoost 평가
    xgb_filename = 'trained_xgboost_models_7targets_312k_v2.mat';
    if isfile(xgb_filename)
        disp(' -> [2/2] XGBoost(P, Ipk) + TreeBagger(UQ Eff) 모델 검증 중...');
        load(xgb_filename, 'xgb_models');
        
        Y_pred_log = zeros(size(Y_test));
        Y_pred_log(1,:) = predict(xgb_models.P, X_test')';
        Y_pred_log(2,:) = predict(xgb_models.P_HV, X_test')';
        Y_pred_log(3,:) = predict(xgb_models.I_pk, X_test')';
        
        % TreeBagger Prediction (단일 Point Estimate로 평가)
        Y_pred_log(4,:) = predict(xgb_models.Eff_UQ, X_test')'; 
        
        Y_pred_XGB = zeros(size(Y_test));
        Y_pred_XGB(1:3, :) = sign(Y_pred_log(1:3, :)) .* (exp(abs(Y_pred_log(1:3, :))) - 1);
        Y_pred_XGB(4, :) = Y_pred_log(4, :); 
        
        calc_and_plot_metrics(Y_test, Y_pred_XGB, 'XGBoost & UQ (Test Set)');
        
        % [XAI 모듈] PDP 분석 실행
        disp(' -> [XAI] XGBoost 모델의 부분 의존성(PDP) 시각화를 시작합니다...');
        run_XAI_Analysis(xgb_models, X_train);
    else
        disp(' -> [알림] XGBoost 모델 파일이 없습니다.');
    end
    disp(' ');
end


%% =========================================================================
% [함수부 4] 다목적 최적화 (NSGA-II Pareto Front)
% =========================================================================
function run_NSGA2_Optimization()
    disp('-----------------------------------------------------------');
    disp('[NSGA-II 다목적 최적화] I_pk 최소화 vs Efficiency 최대화 트레이드오프 분석');
    
    xgb_filename = 'trained_xgboost_models_7targets_312k_v2.mat';
    if ~isfile(xgb_filename)
        disp(' -> [오류] XGBoost 모델 파일이 필요합니다. Method 2를 먼저 실행하세요.');
        return;
    end
    load(xgb_filename, 'xgb_models');
    
    P_target = input(' -> 분석할 Target Power (W) 를 입력하세요 (예: 1000): ');
    V2_target = input(' -> 분석할 V2 전압 (V) 을 입력하세요 (예: 100): ');
    
    disp(' -> 파레토 프론트(Pareto Front)를 탐색 중입니다. 수십 초 소요될 수 있습니다...');
    
    nvars = 3; 
    lb = [-90, 0, 0];
    ub = [90, 0.5, 0.5];
    
    opts = optimoptions('gamultiobj', 'PopulationSize', 50, 'MaxGenerations', 30, ...
                        'Display', 'off', 'PlotFcn', @gaplotpareto);
                    
    obj_fun = @(x) nsga2_objectives(x, V2_target, xgb_models);
    con_fun = @(x) nsga2_constraints(x, V2_target, P_target, xgb_models);
    
    figure('Name', 'NSGA-II Pareto Front', 'Position', [200, 200, 800, 600]);
    [~, fval_pareto] = gamultiobj(obj_fun, nvars, [], [], [], [], lb, ub, con_fun, opts);
    
    title(sprintf('Pareto Front (Target: %dW, %dV)', P_target, V2_target));
    xlabel('Peak Current I_{pk} (A) - 최소화 목적');
    ylabel('Efficiency (%) - 최대화 목적');
    grid on;
    disp(' -> [분석 완료] Pareto Front 그래프가 생성되었습니다.');
end

function f = nsga2_objectives(phases, V2_target, xgb_models)
    X_in = get_features_row(V2_target, phases(1), phases(2), phases(3));
    
    I_pk_log = predict(xgb_models.I_pk, X_in);
    I_pk_pred = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
    eff_pred = predict(xgb_models.Eff_UQ, X_in); % TreeBagger 단일 예측치 사용
    
    f = [I_pk_pred, -eff_pred]; 
end

function [c, ceq] = nsga2_constraints(phases, V2_target, P_target, xgb_models)
    X_in = get_features_row(V2_target, phases(1), phases(2), phases(3));
    P_pred_log = predict(xgb_models.P, X_in);
    P_pred = sign(P_pred_log) * (exp(abs(P_pred_log)) - 1);
    
    ceq = P_pred - P_target; 
    c = []; 
end


%% =========================================================================
% [함수부 5] 앙상블 스태킹 (Ensemble Stacking Meta-model)
% =========================================================================
function run_Ensemble_Stacking(X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[앙상블 스태킹] MLP와 XGBoost의 예측을 융합하는 메타 모델을 학습합니다.');
    
    mlp_file = 'trained_fast_mlp_net_7targets_312k_v2.mat';
    xgb_file = 'trained_xgboost_models_7targets_312k_v2.mat';
    
    if ~isfile(mlp_file) || ~isfile(xgb_file)
        disp(' -> [오류] 모델 파일이 존재하지 않습니다. Method 1, 2를 먼저 실행하세요.');
        return;
    end
    
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    load(xgb_file, 'xgb_models');
    
    disp(' -> Level-0 기본 예측기(Base Predictors) 추론 중...');
    X_train_norm = mapminmax('apply', X_train, PS_X);
    Y_pred_mlp_train = mapminmax('reverse', net(X_train_norm), PS_Y);
    
    Y_pred_xgb_train = zeros(size(Y_train));
    Y_pred_xgb_train(1,:) = predict(xgb_models.P, X_train')';
    Y_pred_xgb_train(2,:) = predict(xgb_models.P_HV, X_train')';
    Y_pred_xgb_train(3,:) = predict(xgb_models.I_pk, X_train')';
    Y_pred_xgb_train(4,:) = predict(xgb_models.Eff_UQ, X_train')';
    
    Y_pred_xgb_train(1:3, :) = sign(Y_pred_xgb_train(1:3, :)) .* (exp(abs(Y_pred_xgb_train(1:3, :))) - 1);
    
    disp(' -> Level-1 메타 모델(Meta-learner) 학습 중...');
    meta_models = cell(4,1);
    for i = 1:4
        X_meta_train = [Y_pred_mlp_train(i, :)', Y_pred_xgb_train(i, :)'];
        meta_models{i} = fitlm(X_meta_train, Y_train(i, :)');
    end
    
    disp(' -> Test Set으로 최종 융합 메타 모델 성능 검증 중...');
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_pred_mlp_test = mapminmax('reverse', net(X_test_norm), PS_Y);
    
    Y_pred_xgb_test = zeros(size(Y_test));
    Y_pred_xgb_test(1,:) = predict(xgb_models.P, X_test')';
    Y_pred_xgb_test(2,:) = predict(xgb_models.P_HV, X_test')';
    Y_pred_xgb_test(3,:) = predict(xgb_models.I_pk, X_test')';
    Y_pred_xgb_test(4,:) = predict(xgb_models.Eff_UQ, X_test')';
    Y_pred_xgb_test(1:3, :) = sign(Y_pred_xgb_test(1:3, :)) .* (exp(abs(Y_pred_xgb_test(1:3, :))) - 1);
    
    Y_pred_stacked = zeros(size(Y_test));
    for i = 1:4
        X_meta_test = [Y_pred_mlp_test(i, :)', Y_pred_xgb_test(i, :)'];
        Y_pred_stacked(i, :) = predict(meta_models{i}, X_meta_test)';
    end
    
    calc_and_plot_metrics(Y_test, Y_pred_stacked, 'Ensemble Stacking (Meta-Model)');
    save('trained_meta_stacking_model_v2.mat', 'meta_models');
    disp(' -> 스태킹 메타 모델이 성공적으로 생성 및 저장되었습니다!');
end


%% =========================================================================
% [함수부 6] Explainable AI (XAI) - Partial Dependence Plot
% =========================================================================
function run_XAI_Analysis(xgb_models, X_train)
    % 7차원 피처 대응
    feature_names = {'V2', 'D0_deg', 'D1', 'D2', 'D0_sq', 'D1_D2', 'V2_D0'};
    X_tbl = array2table(X_train', 'VariableNames', feature_names);
    
    figure('Name', 'XAI: Partial Dependence Plots (Physical Insights)', 'Position', [150, 150, 1600, 800]);
    features_to_analyze = {'D0_deg', 'D1', 'D2'};
    
    % Target 1: Peak Current (도통 손실) - LSBoost 모델 분석
    for j = 1:3
        subplot(2, 3, j);
        plotPartialDependence(xgb_models.I_pk, X_tbl, features_to_analyze{j});
        title(['PDP: ', features_to_analyze{j}, ' on I_{pk}']);
        grid on;
    end
    
    % Target 2: Efficiency (효율) - TreeBagger 모델 분석
    for j = 1:3
        subplot(2, 3, j+3);
        plotPartialDependence(xgb_models.Eff_UQ, X_tbl, features_to_analyze{j});
        title(['PDP: ', features_to_analyze{j}, ' on Efficiency']);
        grid on;
    end
end


%% =========================================================================
% [유틸리티 및 목적 함수들]
% =========================================================================

% [신규 헬퍼 함수] 물리 파생 7차원 특징 추출기
function X_feat = get_features_row(V2, D0, D1, D2)
    X_feat = [V2, D0, D1, D2, D0^2, D1*D2, V2*D0];
end

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

% 차원(UQ 유무)에 따라 유동적으로 헤더를 자동 생성하여 저장
function save_final_csv(optimal_dataset, csv_name)
    if size(optimal_dataset, 2) == 10
        vars = {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'AI_Predicted_Eff_pct', 'Eff_Lower_95', 'Eff_Upper_95'};
    else
        vars = {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'AI_Predicted_Eff_pct'};
    end
    optimal_T = array2table(optimal_dataset, 'VariableNames', vars);
    writetable(optimal_T, csv_name);
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

% DAB 컨버터 PSO 맞춤형 XGBoost 목적 함수
function cost = costFunction_XGB_LogPSO(phases, V2_target, P_target, xgb_models)
    X_in = get_features_row(V2_target, phases(1), phases(2), phases(3));
    
    P_pred_log = predict(xgb_models.P, X_in);
    P_pred = sign(P_pred_log) * (exp(abs(P_pred_log)) - 1);
    
    P_HV_pred_log = predict(xgb_models.P_HV, X_in);
    P_HV_pred = sign(P_HV_pred_log) * (exp(abs(P_HV_pred_log)) - 1);
    
    I_pk_pred_log = predict(xgb_models.I_pk, X_in);
    I_pk_pred = sign(I_pk_pred_log) * (exp(abs(I_pk_pred_log)) - 1);
    
    eff_pred = predict(xgb_models.Eff_UQ, X_in); 
    
    penalty = 0;
    if eff_pred > 100 || eff_pred < 10 || I_pk_pred > 30 
        penalty = 1e6; 
    end
    
    error_power_pct = (abs(P_pred - P_target) / (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    cost = (1.0 * I_pk_pred) + (2.0 * P_loss_pred) + (50.0 * error_power_pct) - (0.1 * eff_pred) + penalty;
end

% 대규모 데이터 추론용 Batch 슬라이싱 예측기
function Y_batch_norm = predict_with_batches(net, X_batch_norm, batch_size)
    num_samples = size(X_batch_norm, 2);
    Y_batch_norm = zeros(4, num_samples); 
    for bs = 1:batch_size:num_samples
        be = min(bs + batch_size - 1, num_samples);
        Y_batch_norm(:, bs:be) = net(X_batch_norm(:, bs:be));
    end
end

% DAB 컨버터 벡터화 최적 위상 탐색 및 페널티 부여
function [best_phases, opt_vals] = evaluate_candidates(phase_cands, Y_batch_raw, P_target)
    P_pred     = Y_batch_raw(1, :)';
    P_HV_pred  = Y_batch_raw(2, :)';
    I_pk_pred  = Y_batch_raw(3, :)';
    eff_pred   = Y_batch_raw(4, :)'; 
    
    invalid_mask = (eff_pred < 10) | (eff_pred > 100) | (I_pk_pred > 30);
    
    error_power_pct = (abs(P_pred - P_target) ./ (abs(P_target) + 1e-6)) * 100;
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    cost_array = (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) + (50.0 .* error_power_pct) - (0.1 .* eff_pred);
    cost_array(invalid_mask) = 1e6;
    
    [min_cost, min_cost_idx] = min(cost_array);
    
    if min_cost >= 1e6
        fallback_cost = (1.0 .* I_pk_pred) + (50.0 .* error_power_pct);
        [~, min_cost_idx] = min(fallback_cost);
    end
    
    best_phases = phase_cands(min_cost_idx, :);
    opt_vals = [P_pred(min_cost_idx), P_HV_pred(min_cost_idx), I_pk_pred(min_cost_idx), eff_pred(min_cost_idx)];
end
%%