% =========================================================================
% [통합 마스터 스크립트] TPS 대리 모델 최적화 및 평가 시스템 (안전 통합본 V6)
% 
% [V6 핵심 변경사항]
%   1. 인코딩 오류 전면 수정 및 한국어화 완료
%   2. [메뉴 5] PSO 다중 해 수집기 적용 (Candidate_Dataset.csv 생성)
%   3. [메뉴 7] K-Means 클러스터링 기반 Multi-Model 양자화 RegHDC 구축
%   4. [메뉴 10] 다중 클러스터 호환 연속-이진화 강화학습(RL) 적용
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
            disp('        DAB TPS 312k 대용량 최적화 마스터 시스템 V6 [최종 통합버전] ');
            disp('===========================================================');
            disp('[시스템 초기화 중...]');
            
            % [V5/V6] 기존 학습 모델 스마트 활성화 (Auto-Copy & Upgrade)
            v4_files = {'trained_fast_mlp_net_v5.mat', 'trained_xgboost_models_v5.mat', 'trained_rf_models_v5.mat', 'trained_hdc_controller_model_v5.mat', 'trained_meta_stacking_model_v5.mat'};
            v6_files = strrep(v4_files, '_v5', '_v6');
            for vi = 1:length(v4_files)
                if isfile(v4_files{vi}) && ~isfile(v6_files{vi})
                    disp([' -> [Auto-Upgrade] 이전 모델 발견! V6로 자동 업그레이드 복사: ', v6_files{vi}]);
                    copyfile(v4_files{vi}, v6_files{vi});
                end
            end
            
            % 병렬 풀 초기화
            p = gcp('nocreate');
            if isempty(p)
                disp(' -> 새로운 병렬 풀(Parallel Pool)을 생성합니다...');
                parpool;
            else
                disp([' -> 병렬 풀 활성화 확인 (활성 코어 수: ', num2str(p.NumWorkers), ')']);
            end
            
            disp(' -> 원본 데이터셋을 로드합니다... (수 초 소요될 수 있습니다)');
            try
                data = readtable('TPS_Dataset_20260316_021229.csv');
                disp([' -> 데이터 로드 성공: 총 ', num2str(height(data)), '개 샘플 확인완료']);
            catch
                disp(' -> [경고] 지정된 CSV 파일이 없습니다. 대규모 임시 더미 데이터를 생성합니다.');
                data = array2table(rand(312000, 8), 'VariableNames', {'Target_P_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'P_act_W', 'P_act_HV_W', 'i_pk_pri_A'});
                data.V2_V = data.V2_V * 30 + 80; data.D0_deg = data.D0_deg * 180 - 90;
                data.P_act_W = (rand(312000, 1) - 0.2) * 1200; % 음수 전력 포함 더미
                data.P_act_HV_W = data.P_act_W .* (0.8 + rand(312000,1)*0.18); 
                data.i_pk_pri_A = rand(312000, 1) * 20 + 5; 
            end
            
            % =============================================================
            % 물리적 효율(Efficiency) 계산 로직 전면 보수
            % =============================================================
            eff_raw = zeros(size(data.P_act_W));
            P1 = data.P_act_W;
            P2 = data.P_act_HV_W;
            
            % 부호가 같고 0이 아닌 경우만 유효 (부호가 다르면 무효전력이 지배적이므로 무시)
            valid_idx = sign(P1) == sign(P2) & P1 ~= 0;
            
            % 순방향 (P1 -> P2, P1이 양수)
            idx_fwd = valid_idx & (P1 > 0);
            eff_raw(idx_fwd) = (abs(P2(idx_fwd)) ./ abs(P1(idx_fwd))) * 100;
            
            % 역방향 (P2 -> P1, P1이 음수)
            idx_rev = valid_idx & (P1 < 0);
            eff_raw(idx_rev) = (abs(P1(idx_rev)) ./ abs(P2(idx_rev))) * 100;
            
            % 비정상적인 전압 스파이크 등으로 효율이 100%를 초과하는 소수값은 100으로 클리핑
            eff_raw(eff_raw > 100) = 100;
            eff_raw(eff_raw < 0) = 0;
            % =============================================================
            
            % 도메인 지식 파생 변수 3개 추가 (총 7차원)
            V2_vec = data.V2_V'; D0_vec = data.D0_deg'; D1_vec = data.D1'; D2_vec = data.D2';
            X_raw = [V2_vec; D0_vec; D1_vec; D2_vec; D0_vec.^2; D1_vec.*D2_vec; V2_vec.*D0_vec];
            Y_raw = [data.P_act_W, data.P_act_HV_W, data.i_pk_pri_A, eff_raw]';
            
            % Train / Test Split (8:2)
            rng(42); cv = cvpartition(size(X_raw, 2), 'HoldOut', 0.2);
            X_train = X_raw(:, training(cv)); Y_train = Y_raw(:, training(cv));
            X_test  = X_raw(:, test(cv));     Y_test  = Y_raw(:, test(cv));
            
            disp(' -> 파생 변수 생성 및 데이터 분할 완료! (입력 차원: 7차원)'); disp(' ');
            currentState = 'MAIN_MENU';
            
        % =================================================================
        % 상태 2: 메인 메뉴
        % =================================================================
        case 'MAIN_MENU'
            disp('===========================================================');
            disp(' 메인 메뉴를 선택하세요.');
            disp('  1. [단일 최적화] AutoML Multi-Agent 경쟁 MLP + Vectorized Search');
            disp('  2. [전역 최적화] XGBoost + PSO 전역 탐색 (불확실성 UQ 포함)');
            disp('  3. [신규 최적화] Random Forest 기반 안정적 최적화 + PSO 탐색');
            disp('  4. [다목적 최적화] NSGA-II 파레토 프론트 도출 (I_pk vs Efficiency)');
            disp('  5. [V6: 다중 해 수집] Best-in-Class PSO 기반 다중 위상 아카이브 구축 (Candidate_Dataset)');
            disp('  6. [모델 검증 & XAI] Test Set 일반화 검증 및 PDP 물리 영향도 분석');
            disp('  7. [V6: 클러스터 HDC] 초경량 Multi-Model 양자화 RegHDC (RegHD 논문 이식)');
            disp('  8. [신규 할당] 효율 전용 스페셜리스트 모델 다중 알고리즘 경쟁 학습');
            disp('  9. [3D 시각화] Target P, V2 에 따른 최적 위상(D0, D1, D2) 3D 분포 플롯');
            disp(' 10. [V6: 클러스터 RL] 선택된 클러스터 모드의 HDC 모델 강화학습 (Soft Penalty)');
            disp(' 11. [데이터 증강] 앵커 데이터 기반 대리 모델 학습 + 10만건 연속 데이터 증강');
            disp('  0. [종료] 시스템 종료');
            disp('===========================================================');
            
            user_choice = input('번호를 입력하세요: ');
            
            if user_choice == 1, currentState = 'RUN_MLP';
            elseif user_choice == 2, currentState = 'RUN_XGB';
            elseif user_choice == 3, currentState = 'RUN_RF';
            elseif user_choice == 4, currentState = 'RUN_NSGA2';
            elseif user_choice == 5, currentState = 'RUN_STACKING';
            elseif user_choice == 6, currentState = 'RUN_EVALUATE';
            elseif user_choice == 7, currentState = 'RUN_HDC';
            elseif user_choice == 8, currentState = 'RUN_EFF_SPEC';
            elseif user_choice == 9, currentState = 'RUN_PLOT_3D';
            elseif user_choice == 10, currentState = 'RUN_HDC_RL';
            elseif user_choice == 11, currentState = 'RUN_SURROGATE_AUG';
            elseif user_choice == 0, currentState = 'EXIT';
            else, disp('잘못된 입력입니다.'); disp(' '); end
            
        case 'RUN_MLP', run_AutoML_MLP(data, X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_XGB', run_XGB_PSO(data, X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_RF', run_RandomForest(data, X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_NSGA2', run_NSGA2_Optimization(); currentState = 'MAIN_MENU';
        case 'RUN_STACKING', run_Active_RegHDC_Learning(data, X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_EVALUATE', evaluate_Models(X_test, Y_test, X_train); currentState = 'MAIN_MENU';
        case 'RUN_HDC', run_Regression_HDC(); currentState = 'MAIN_MENU';
        case 'RUN_EFF_SPEC', run_Efficiency_Specialist(X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_PLOT_3D', run_3D_Plot_Phases(); currentState = 'MAIN_MENU';
        case 'RUN_HDC_RL', run_HDC_Reinforcement_Learning(X_test, Y_test); currentState = 'MAIN_MENU';
        case 'RUN_SURROGATE_AUG', run_Surrogate_Augmentation(X_train, Y_train, X_test, Y_test); currentState = 'MAIN_MENU';
            
        case 'EXIT'
            disp('마스터 시스템을 안전하게 종료합니다. 수고하셨습니다!');
            isRunning = false;
    end
end


%% =========================================================================
% [함수 1] AutoML Multi-Agent MLP + Vectorized Search
% =========================================================================
function run_AutoML_MLP(data, X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 1] AutoML Multi-Agent 신경망 모델 로드 및 학습');
    
    [X_norm, PS_X] = mapminmax(X_train);
    [Y_norm, PS_Y] = mapminmax(Y_train); 
    
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_test_norm = mapminmax('apply', Y_test, PS_Y);
    
    nn_filename = 'trained_fast_mlp_net_v6.mat';
    needs_training = true;
    
    % 자가 진단 및 자동 재학습 로직
    if isfile(nn_filename)
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
        disp(' -> 기존 모델 발견. 성능 자가 진단을 실시합니다...');
        
        test_idx = randperm(size(X_test, 2), min(1000, size(X_test, 2)));
        Y_pred = mapminmax('reverse', net(mapminmax('apply', X_test(:, test_idx), PS_X)), PS_Y);
        R2 = 1 - (sum((Y_test(4, test_idx) - Y_pred(4, :)).^2) / sum((Y_test(4, test_idx) - mean(Y_test(4, test_idx))).^2));
        
        if R2 > 0.6
            disp([' -> 모델 상태 양호 (효율 R^2 = ', num2str(R2, '%.2f'), '). 재학습을 생략합니다.']);
            needs_training = false;
        else
            disp([' -> [경고] 기존 모델 성능 불량 (효율 R^2 = ', num2str(R2, '%.2f'), '). 폐기하고 재학습합니다.']);
            delete(nn_filename);
        end
    end
    
    if needs_training
        disp(' -> 준비된 모델이 없습니다. [에이전트 경쟁 모드]를 시작합니다.');
        
        agent_hidden_sizes = {[64, 64], [128, 64], [64, 64, 32], [128, 64, 32], [256, 64], [512, 128, 16]};
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
        disp([' -> 물리 파생 변수를 학습할 ', num2str(num_agents), '개 에이전트 가동 (parfor 병렬학습)']);
        
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
    
    % 병렬 벡터화 탐색 (Coarse-to-Fine)
    disp(' -> [병렬/벡터화 탐색] 챔피언 모델 기반 Coarse-to-Fine 2단계 전수조사를 시작합니다.');
    final_csv_name = 'Massive_Optimized_TPS_Dataset_FastMLP_V6.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 8); 
    
    D0_c_cands = -90:5:90; D1_c_cands = 0:0.05:0.5; D2_c_cands = 0:0.05:0.5;    
    [D0_cg, D1_cg, D2_cg] = ndgrid(D0_c_cands, D1_c_cands, D2_c_cands);
    coarse_phase_cands = [D0_cg(:), D1_cg(:), D2_cg(:)];
    num_coarse = size(coarse_phase_cands, 1);
    
    unique_V2 = unique(target_conditions(:, 2));
    precomputed_coarse_Y = cell(length(unique_V2), 1);
    
    for v_idx = 1:length(unique_V2)
        curr_V2 = unique_V2(v_idx);
        V_batch = repmat(curr_V2, num_coarse, 1);
        D0_b = coarse_phase_cands(:,1); D1_b = coarse_phase_cands(:,2); D2_b = coarse_phase_cands(:,3);
        X_batch = [V_batch, D0_b, D1_b, D2_b, D0_b.^2, D1_b.*D2_b, V_batch.*D0_b]';
        
        X_batch_norm = mapminmax('apply', X_batch, PS_X);
        Y_batch_norm = predict_with_batches(net, X_batch_norm, 50000);
        precomputed_coarse_Y{v_idx} = mapminmax('reverse', Y_batch_norm, PS_Y);
    end
    
    disp([' -> ', num2str(total_steps), '개 조건에 대해 parfor 병렬 최적화를 수행합니다...']);
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
        
        if mod(idx, 100) == 0
            fprintf('[MLP 병렬 탐색] %d / %d 조건 처리 완료...\n', idx, total_steps);
        end
    end
    
    elapsed_time = toc;
    disp([' -> [병렬 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    save_final_csv(optimal_dataset, final_csv_name);
end

%% =========================================================================
% [함수 2] XGBoost + TreeBagger 전역 탐색 및 PSO 최적화
% =========================================================================
function run_XGB_PSO(data, X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 2] XGBoost 기반 대리 모델 학습 및 로드');
    
    xgb_filename = 'trained_xgboost_models_v6.mat';
    needs_training = true;
    
    Y_train_xgb = Y_train;
    Y_train_xgb(1:3, :) = sign(Y_train(1:3, :)) .* log(abs(Y_train(1:3, :)) + 1);
    
    if isfile(xgb_filename)
        load(xgb_filename, 'xgb_models');
        disp(' -> 기존 모델 발견. 성능 자가 진단을 실시합니다...');
        
        test_idx = randperm(size(X_test, 2), min(1000, size(X_test, 2)));
        eff_pred = predict(xgb_models.Eff_UQ, X_test(:, test_idx)');
        R2 = 1 - (sum((Y_test(4, test_idx)' - eff_pred).^2) / sum((Y_test(4, test_idx)' - mean(Y_test(4, test_idx)')).^2));
        
        if R2 > 0.6
            disp([' -> 모델 상태 양호 (효율 R^2 = ', num2str(R2, '%.2f'), '). 재학습 생략.']);
            needs_training = false;
        else
            disp([' -> [경고] 기존 모델 성능 불량 (효율 R^2 = ', num2str(R2, '%.2f'), '). 재학습 진행.']);
            delete(xgb_filename);
        end
    end
    
    if needs_training
        disp(' -> 4개 대상 모델 학습 시작 (수 분 소요)...');
        opts = struct('Optimizer', 'bayesopt', 'ShowPlots', false, 'MaxObjectiveEvaluations', 10, 'UseParallel', true);
        
        disp(' -> [1/4] P 학습 중...'); xgb_models.P = fitrensemble(X_train', Y_train_xgb(1,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        disp(' -> [2/4] P_HV 학습 중...'); xgb_models.P_HV = fitrensemble(X_train', Y_train_xgb(2,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        disp(' -> [3/4] I_pk 학습 중...'); xgb_models.I_pk = fitrensemble(X_train', Y_train_xgb(3,:)', 'Method', 'LSBoost', 'OptimizeHyperparameters', 'auto', 'HyperparameterOptimizationOptions', opts);
        
        % 불확실성(UQ) 정량화를 위해 OOBPrediction = 'on' 설정
        disp(' -> [4/4] Efficiency (TreeBagger UQ) 학습 중...'); xgb_models.Eff_UQ = TreeBagger(50, X_train', Y_train(4,:)', 'Method', 'regression', 'OOBPrediction', 'on');
        
        save(xgb_filename, 'xgb_models');
        disp(' -> XGBoost 모델 학습 완료!');
    end
    
    final_csv_name = 'Massive_Optimized_TPS_Dataset_PSO_UQ_V6.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 10); % 10차원 (효율 신뢰구간 포함)
    
    pso_options = optimoptions('particleswarm', 'Display', 'none', 'SwarmSize', 30, 'MaxIterations', 50, 'UseParallel', false, 'UseVectorized', true);
    disp([' -> ', num2str(total_steps), '개 조건에 대해 parfor PSO 병렬 최적화를 수행합니다...']);
    
    tic;
    parfor idx = 1:total_steps
        P_target = target_conditions(idx, 1);
        V2_target = target_conditions(idx, 2);
        
        [lb, ub] = get_dynamic_bounds(data, P_target, V2_target);
        obj_fun = @(phases) costFunction_XGB_LogPSO(phases, V2_target, P_target, xgb_models);
        
        [best_phases, ~] = particleswarm(obj_fun, 3, lb, ub, pso_options);
        
        X_in = get_features_row(V2_target, best_phases(1), best_phases(2), best_phases(3));
        achieved_P_log = predict(xgb_models.P, X_in);
        achieved_P = sign(achieved_P_log) * (exp(abs(achieved_P_log)) - 1);
        I_pk_log = predict(xgb_models.I_pk, X_in);
        min_I_pk = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
        
        % UQ 예측 (5%, 50%, 95% 분위수)
        eff_quantiles = quantilePredict(xgb_models.Eff_UQ, X_in, 'Quantile', [0.05, 0.5, 0.95]);
        eff_lower = eff_quantiles(1); eff_pred  = eff_quantiles(2); eff_upper = eff_quantiles(3);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_phases(1), best_phases(2), best_phases(3), achieved_P, min_I_pk, eff_pred, eff_lower, eff_upper];
        
        if mod(idx, 100) == 0
            fprintf('[XGB+PSO 병렬 처리] %d / %d 조건 처리 완료...\n', idx, total_steps);
        end
    end
    
    elapsed_time = toc;
    disp([' -> [XGB+PSO 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    save_final_csv(optimal_dataset, final_csv_name);
end

%% =========================================================================
% [함수 3] Random Forest 전용 모델 및 PSO 최적화
% =========================================================================
function run_RandomForest(data, X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 3] Random Forest 기반 모델 (안정적 최적화)');
    
    rf_filename = 'trained_rf_models_v6.mat';
    needs_training = true;
    
    if isfile(rf_filename)
        load(rf_filename, 'rf_models');
        disp(' -> 기존 RF 모델 발견. 자가 진단 중...');
        test_idx = randperm(size(X_test, 2), min(1000, size(X_test, 2)));
        eff_pred = predict(rf_models.Eff, X_test(:, test_idx)');
        R2 = 1 - (sum((Y_test(4, test_idx)' - eff_pred).^2) / sum((Y_test(4, test_idx)' - mean(Y_test(4, test_idx)')).^2));
        
        if R2 > 0.6
            disp([' -> 모델 상태 양호 (R^2 = ', num2str(R2, '%.2f'), ').']);
            needs_training = false;
        else
            delete(rf_filename);
        end
    end
    
    if needs_training
        disp(' -> Random Forest (50 Trees) 4개 대상 학습을 시작합니다...');
        rf_models.P = TreeBagger(50, X_train', Y_train(1,:)', 'Method', 'regression');
        rf_models.P_HV = TreeBagger(50, X_train', Y_train(2,:)', 'Method', 'regression');
        rf_models.I_pk = TreeBagger(50, X_train', Y_train(3,:)', 'Method', 'regression');
        rf_models.Eff = TreeBagger(50, X_train', Y_train(4,:)', 'Method', 'regression');
        save(rf_filename, 'rf_models');
        disp(' -> Random Forest 학습 및 저장 완료!');
    end
    
    final_csv_name = 'Massive_Optimized_TPS_Dataset_RF_PSO_V6.csv';
    [target_conditions, total_steps] = setup_target_grid();
    optimal_dataset = zeros(total_steps, 8); 
    
    pso_options = optimoptions('particleswarm', 'Display', 'none', 'SwarmSize', 30, 'MaxIterations', 50, 'UseParallel', false, 'UseVectorized', true);
    disp([' -> ', num2str(total_steps), '개 조건에 대해 parfor RF PSO 병렬 최적화를 수행합니다...']);
    
    tic;
    parfor idx = 1:total_steps
        P_target = target_conditions(idx, 1);
        V2_target = target_conditions(idx, 2);
        
        [lb, ub] = get_dynamic_bounds(data, P_target, V2_target);
        obj_fun = @(phases) costFunction_RF_PSO(phases, V2_target, P_target, rf_models);
        
        [best_phases, ~] = particleswarm(obj_fun, 3, lb, ub, pso_options);
        
        X_in = get_features_row(V2_target, best_phases(1), best_phases(2), best_phases(3));
        achieved_P = predict(rf_models.P, X_in);
        min_I_pk = predict(rf_models.I_pk, X_in);
        eff_pred = predict(rf_models.Eff, X_in);
        
        optimal_dataset(idx, :) = [P_target, V2_target, best_phases(1), best_phases(2), best_phases(3), achieved_P, min_I_pk, eff_pred];
        
        if mod(idx, 100) == 0
            fprintf('[RF+PSO 병렬 처리] %d / %d 조건 처리 완료...\n', idx, total_steps);
        end
    end
    
    elapsed_time = toc;
    disp([' -> [RF+PSO 탐색 완료] 소요 시간: ', num2str(elapsed_time, '%.2f'), ' 초']);
    save_final_csv(optimal_dataset, final_csv_name);
end

%% =========================================================================
% [함수 4] 다목적 최적화 (NSGA-II)
% =========================================================================
function run_NSGA2_Optimization()
    disp('-----------------------------------------------------------');
    disp('[NSGA-II 다목적 최적화] I_pk 최소화 vs Efficiency 최대화');
    
    xgb_filename = 'trained_xgboost_models_v6.mat';
    if ~isfile(xgb_filename)
        disp(' -> [오류] XGBoost 모델이 없습니다. 메뉴 2번을 먼저 실행하세요.'); return;
    end
    load(xgb_filename, 'xgb_models');
    
    P_target = input(' -> Target Power (W) 입력 (예 1000): ');
    V2_target = input(' -> Target V2 전압 (V) 입력 (예 100): ');
    
    disp(' -> 파레토 프론트(Pareto Front) 탐색 중...');
    
    nvars = 3; lb = [-90, 0, 0]; ub = [90, 0.5, 0.5];
    opts = optimoptions('gamultiobj', 'PopulationSize', 50, 'MaxGenerations', 30, 'Display', 'off', 'PlotFcn', @gaplotpareto);
                    
    obj_fun = @(x) nsga2_objectives(x, V2_target, xgb_models);
    con_fun = @(x) nsga2_constraints(x, V2_target, P_target, xgb_models);
    
    figure('Name', 'NSGA-II Pareto Front', 'Position', [200, 200, 800, 600]);
    gamultiobj(obj_fun, nvars, [], [], [], [], lb, ub, con_fun, opts);
    
    title(sprintf('Pareto Front (Target: %dW, %dV)', P_target, V2_target));
    xlabel('Peak Current I_{pk} (A) - 최소화'); ylabel('Efficiency (%) - 최대화');
    grid on;
end

% NSGA-II 목적 함수
function f = nsga2_objectives(phases, V2_target, xgb_models)
    phases = phases(:)'; 
    X_in = [V2_target, phases(1), phases(2), phases(3), phases(1)^2, phases(2)*phases(3), V2_target*phases(1)];
    
    I_pk_log = predict(xgb_models.I_pk, X_in);
    I_pk_pred = sign(I_pk_log) * (exp(abs(I_pk_log)) - 1);
    eff_pred = predict(xgb_models.Eff_UQ, X_in); 
    
    f = [I_pk_pred, -eff_pred]; 
end

function [c, ceq] = nsga2_constraints(phases, V2_target, P_target, xgb_models)
    phases = phases(:)'; 
    X_in = [V2_target, phases(1), phases(2), phases(3), phases(1)^2, phases(2)*phases(3), V2_target*phases(1)];
    
    P_pred_log = predict(xgb_models.P, X_in);
    P_pred = sign(P_pred_log) * (exp(abs(P_pred_log)) - 1);
    
    ceq = P_pred - P_target; 
    c = []; 
end

%% =========================================================================
% [함수 5 - V7.5 물리 대칭성 기반 정밀 최적화판] 능동적 곡면 추출 및 RegHDC 동시 학습
% (특징: HDC 자체 학습, P=0 기준 양방향 순차 딥서치, 내부 위상 단조 증가(벌어짐) 강제)
% =========================================================================
function run_Active_RegHDC_Learning(data, X_train, Y_train, X_test, Y_test)
    disp('===========================================================');
    disp('[통합 Phase] 능동적 곡면 추출 및 RegHDC 동시 학습 (V7.5 딥서치 최적화)');
    disp(' -> 1. 딥서치 적용: PSO 입자 수 및 반복 횟수 15배 상향 (정밀 탐색)');
    disp(' -> 2. 단조 증가 강제: 전력이 커질수록 D1, D2가 줄어들지 않고 0.5로 향하도록 하한선 고정');
    disp(' -> 3. 순차 탐색 보장: 0W에서 시작하여 ±1200W 방향으로 각각 독립적 순차 탐색');
    disp(' -> 4. 학습 방식: 확보된 쌍봉낙타형 고품질 전 구간 데이터를 통한 HDC 곡면 학습');
    disp('===========================================================');
    
    % -------------------------------------------------------------
    % 0. 가상 물리 환경 로드 및 모델 정확도(RMSE) 분석
    % -------------------------------------------------------------
    mlp_file = 'trained_fast_mlp_net_v6.mat';
    xgb_file = 'trained_xgboost_models_v6.mat';
    rf_file  = 'trained_rf_models_v6.mat';
    
    if ~isfile(mlp_file) || ~isfile(xgb_file) || ~isfile(rf_file)
        disp(' -> [오류] 대리 모델 파일이 부족합니다. 1, 2, 3번을 먼저 실행하세요.'); return;
    end
    
    load(mlp_file, 'net', 'PS_X', 'PS_Y'); 
    load(xgb_file, 'xgb_models'); 
    load(rf_file, 'rf_models');
    
    % Test Set 기반 각 모델 성능 평가
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_pred_mlp = mapminmax('reverse', net(X_test_norm), PS_Y);
    Y_pred_xgb_p = sign(predict(xgb_models.P, X_test')') .* (exp(abs(predict(xgb_models.P, X_test')')) - 1);
    Y_pred_rf_p  = predict(rf_models.P, X_test')';
    
    % 최적 전력 모델 선정 및 RMSE 추출
    mses_p = [mean((Y_test(1,:) - Y_pred_mlp(1,:)).^2), mean((Y_test(1,:) - Y_pred_xgb_p).^2), mean((Y_test(1,:) - Y_pred_rf_p).^2)];
    [~, best_p] = min(mses_p);
    rmse_p = sqrt(mses_p(best_p));
    
    % 최적 전류 모델 선정
    Y_pred_mlp_i = Y_pred_mlp(3,:);
    Y_pred_xgb_i = sign(predict(xgb_models.I_pk, X_test')') .* (exp(abs(predict(xgb_models.I_pk, X_test')')) - 1);
    Y_pred_rf_i  = predict(rf_models.I_pk, X_test')';
    mses_i = [mean((Y_test(3,:) - Y_pred_mlp_i).^2), mean((Y_test(3,:) - Y_pred_xgb_i).^2), mean((Y_test(3,:) - Y_pred_rf_i).^2)];
    [~, best_i_pk] = min(mses_i);

    % 오차 마진 설정
    p_margin = max(3.0, rmse_p * 1.5); 
    fprintf(' -> [분석 완료] 전력 모델 RMSE: %.2fW | 설정 마진: %.2fW\n', rmse_p, p_margin);
    
    % 검증용 메타 모델 (무거운 연산용)
    const_net = parallel.pool.Constant(net); 
    const_xgb = parallel.pool.Constant(xgb_models); 
    const_rf  = parallel.pool.Constant(rf_models);
    bic_predict = @(X_in) predict_best_in_class_lite(X_in, best_p, best_i_pk, const_net.Value, PS_X, PS_Y, const_xgb.Value, const_rf.Value);
    
    % 탐색용 초고속 예측기 (PSO 목적 함수용)
    fast_mlp_predict = @(X_in) mapminmax('reverse', net(mapminmax('apply', X_in', PS_X)), PS_Y);
    
    % -------------------------------------------------------------
    % 1. HDC 환경 설정 (RegHD 2차원 출력 아키텍처)
    % -------------------------------------------------------------
    D = 10000; Q = 500; gamma = 15.0; mode = 'Multi_Interact';
    K_init = 32; K_final = 8;
    min_X = [-1200; 80]; max_X = [1200; 110]; 
    min_Y = [0; 0]; max_Y = [0.5; 0.5]; % D1, D2 2차원만 학습
    
    rng(100);
    ID_vecs = randsrc(2, D, [-1, 1]);
    L_vecs = zeros(Q, D); L_vecs(1, :) = randsrc(1, D, [-1, 1]);
    flip_counts = max(1, floor(D / 2 / (Q - 1)));
    for q_idx = 2:Q
        L_vecs(q_idx, :) = L_vecs(q_idx-1, :);
        flip_idx = randperm(D, flip_counts);
        L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx);
    end
    
    % -------------------------------------------------------------
    % Phase 1: 0도 기준 양방향 앵커 초기화
    % -------------------------------------------------------------
    disp(' -> [Phase 1] 0W 근처 극저부하 앵커 추출 중... (단조 증가의 시작점)');
    anchors_X = []; anchors_Y = [];
    v2_seeds = linspace(80, 110, 8); 
    tic;
    for vi = 1:length(v2_seeds)
        % 0에서 최대한 가까운 25W에서 D1, D2가 거의 0인 상태를 출발점으로 잡습니다.
        [a_pos_X, a_pos_Y] = find_initial_anchor(v2_seeds(vi), 25, fast_mlp_predict, p_margin);
        [a_neg_X, a_neg_Y] = find_initial_anchor(v2_seeds(vi), -25, fast_mlp_predict, p_margin);
        anchors_X = [anchors_X; a_pos_X; a_neg_X];
        anchors_Y = [anchors_Y; a_pos_Y; a_neg_Y]; % [D0, D1, D2] 저장
    end
    disp([' -> 앵커 추출 완료! 소요 시간: ', num2str(toc, '%.2f'), '초']);
    
    % -------------------------------------------------------------
    % Phase 2: 정밀 딥서치 능동 곡면 탐색 (스킵 제거, 단조 증가 강제)
    % -------------------------------------------------------------
    disp(' -> [Phase 2] 전 구간(±1200W) 딥서치 탐색 시작 (PSO 비용 15배 상향)...');
    target_V2_list = 80:5:110; power_step = 25;
    % 0부터 양방향으로 순차적으로 뻗어나가도록 스텝 구성
    P_seq_pos = 25:power_step:1200; 
    P_seq_neg = -25:-power_step:-1200;
    
    all_trajectories = []; 
    saved_X_history = []; 
    saved_Y_history_2d = []; 
    
    tic;
    for v_idx = 1:length(target_V2_list)
        V2_t = target_V2_list(v_idx);
        fprintf('   [V2 = %.1f V] 정밀 양방향 딥서치 진행 중...\n', V2_t);
        
        for direction = 1:2
            if direction == 1, P_seq = P_seq_pos; else, P_seq = P_seq_neg; end
            
            % 시작 앵커점 매칭
            [~, closest_idx] = min(abs(anchors_X(:,1) - P_seq(1)) + abs(anchors_X(:,2) - V2_t));
            prev_Y = anchors_Y(closest_idx, :); 
            
            for p_idx = 1:length(P_seq)
                P_t = P_seq(p_idx); X_curr = [P_t; V2_t];
                
                % [핵심 수정 1] 휴리스틱 스킵 제거 -> 무조건 정밀 탐색 수행
                % [핵심 수정 2] 단조 증가(벌어짐) 강제: 이전 단계의 D1, D2를 하한선으로 설정
                % (오차가 누적되어 갇히는 것을 방지하기 위해 아주 미세한 여유만 줌)
                relax = 0.005; 
                lb_2d = [max(0, prev_Y(2) - relax), max(0, prev_Y(3) - relax)];
                ub_2d = [0.5, 0.5]; % 상한은 항상 최대치(0.5)로 열어둠
                
                % PSO 파라미터 대폭 상향 (입자수 150, 반복 50)
                pso_opt = optimoptions('particleswarm', 'Display', 'none', ...
                                       'SwarmSize', 150, 'MaxIterations', 50, ...
                                       'UseVectorized', true, 'FunctionTolerance', 1e-4);
                
                % 초기 입자 위치에 이전 최적값 포함
                init_swarm = repmat(prev_Y(2:3), 150, 1) + (rand(150, 2) * 0.05);
                init_swarm = max(lb_2d, min(ub_2d, init_swarm));
                init_swarm(1,:) = prev_Y(2:3);
                pso_opt.InitialSwarmMatrix = init_swarm;
                
                obj_fun = @(p2d) costFunction_Active_2D_Vec_Multi_D0(p2d, V2_t, P_t, fast_mlp_predict, p_margin);
                [Y_opt_2d, ~] = particleswarm(obj_fun, 2, lb_2d, ub_2d, pso_opt);
                
                [D0_opt, P_pred, I_pk_pred] = find_best_D0_Multi(P_t, V2_t, Y_opt_2d(1), Y_opt_2d(2), bic_predict, p_margin);
                
                % [파단 방지] 만약 단조 증가 제약 때문에 전력 마진을 못 맞추는 구간이 발생하면,
                % 제약을 살짝 풀고 와이드 서치를 한 번 더 수행
                if abs(P_pred - P_t) > p_margin * 3
                    pso_opt_wide = optimoptions('particleswarm', 'Display', 'none', 'SwarmSize', 200, 'MaxIterations', 80, 'UseVectorized', true);
                    [Y_opt_2d_wide, ~] = particleswarm(obj_fun, 2, [0, 0], [0.5, 0.5], pso_opt_wide);
                    [D0_opt_wide, P_pred_wide, I_pk_pred_wide] = find_best_D0_Multi(P_t, V2_t, Y_opt_2d_wide(1), Y_opt_2d_wide(2), bic_predict, p_margin);
                    
                    if abs(P_pred_wide - P_t) < abs(P_pred - P_t)
                        Y_opt_2d = Y_opt_2d_wide; D0_opt = D0_opt_wide; 
                        P_pred = P_pred_wide; I_pk_pred = I_pk_pred_wide;
                    end
                end
                
                % 데이터 저장
                all_trajectories = [all_trajectories; P_t, V2_t, D0_opt, Y_opt_2d(1), Y_opt_2d(2), P_pred, I_pk_pred];
                saved_X_history = [saved_X_history, X_curr];
                saved_Y_history_2d = [saved_Y_history_2d, Y_opt_2d']; 
                
                % 다음 P 스텝을 위한 기준점 갱신 (계속 벌어지도록)
                prev_Y = [D0_opt, Y_opt_2d(1), Y_opt_2d(2)]; 
            end
        end
    end
    elapsed_search = toc;
    disp([' -> 전 구간 딥서치 및 데이터 수집 완료! (소요 시간: ', num2str(elapsed_search, '%.2f'), '초)']);
    fprintf(' -> 확보된 총 데이터 포인트: %d 개\n', size(saved_X_history, 2));

    % -------------------------------------------------------------
    % Phase 3: HDC 군집화 및 학습 (전 구간 데이터를 통한 학습 강화)
    % -------------------------------------------------------------
    disp(' -> [Phase 3] 정밀하게 확보된 쌍봉낙타 곡면 데이터를 이용한 HDC 학습 시작...');
    
    C_int = zeros(K_init, D); M_int = zeros(K_init, 2, D); 
    
    % HDC 데이터 인코딩
    [S_hist_int, S_hist_bin] = encode_HDC_data(saved_X_history, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
    Y_true_norm = 2 * ((saved_Y_history_2d - min_Y) ./ (max_Y - min_Y)) - 1; 
    
    num_train = size(S_hist_bin, 2);
    init_idx = randperm(num_train, K_init);
    for k = 1:K_init, C_int(k, :) = S_hist_int(:, init_idx(k))'; end
    C_bin = sign(C_int); C_bin(C_bin == 0) = 1;
    M_bin = sign(M_int); M_bin(M_bin == 0) = 1;

    epochs = 20; 
    init_lr_hdc = 0.1;
    beta = 0.8; 
    
    tic;
    for ep = 1:epochs
        lr_hdc = init_lr_hdc * (0.95 ^ ep);
        shuffle_idx = randperm(num_train);
        
        for b = 1:num_train
            idx = shuffle_idx(b);
            S_curr_bin = S_hist_bin(:, idx);
            S_curr_int = S_hist_int(:, idx);
            Y_curr = Y_true_norm(:, idx);
            
            sim = (C_bin * S_curr_bin) / D;
            [max_sim, k_star] = max(sim);
            
            C_int(k_star, :) = C_int(k_star, :) + (1 - max_sim) * S_curr_int';
            
            delta_prime = exp(sim * gamma) / sum(exp(sim * gamma));
            Y_hat_norm = zeros(2, 1);
            for k = 1:K_init
                Y_hat_norm = Y_hat_norm + delta_prime(k) * ((squeeze(M_bin(k, :, :)) * S_curr_bin) / D);
            end
            
            Error = Y_curr - Y_hat_norm;
            for k = 1:K_init
                Update_M = lr_hdc * delta_prime(k) * Error * S_curr_int';
                M_int(k, :, :) = squeeze(M_int(k, :, :)) * beta + Update_M * (1 - beta);
            end
            
            if mod(b, 200) == 0
                C_bin = sign(C_int); C_bin(C_bin == 0) = 1;
                M_bin = sign(M_int); M_bin(M_bin == 0) = 1;
            end
        end
        C_bin = sign(C_int); C_bin(C_bin == 0) = 1; M_bin = sign(M_int); M_bin(M_bin == 0) = 1;
    end
    elapsed_train = toc;
    disp([' -> HDC 곡면 학습 완료! (소요 시간: ', num2str(elapsed_train, '%.2f'), '초)']);

    % -------------------------------------------------------------
    % Phase 4: 가지치기 및 데이터 저장 (Pruning)
    % -------------------------------------------------------------
    usage_counts = zeros(K_init, 1);
    sim_hist = (C_bin * S_hist_bin) / D;
    [~, max_k_indices] = max(sim_hist, [], 1);
    for i = 1:length(max_k_indices), usage_counts(max_k_indices(i)) = usage_counts(max_k_indices(i)) + 1; end
    [~, sorted_k] = sort(usage_counts, 'descend');
    top_k_idx = sorted_k(1:K_final);
    
    hdc_model = struct();
    hdc_model.C_bin = sign(C_int(top_k_idx, :)); hdc_model.M_bin = sign(M_int(top_k_idx, :, :));
    hdc_model.min_X = min_X; hdc_model.max_X = max_X; hdc_model.min_Y = min_Y; hdc_model.max_Y = max_Y;
    hdc_model.Q = Q; hdc_model.D = D; hdc_model.gamma = gamma; hdc_model.mode = mode; hdc_model.num_clusters = K_final;
    hdc_model.ID_vecs = ID_vecs; hdc_model.L_vecs = L_vecs;
    
    save('trained_hdc_controller_model_v6.mat', 'hdc_model');
    opt_T = array2table(all_trajectories, 'VariableNames', {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A'});
    writetable(opt_T, 'Candidate_Dataset_V6.csv');
    
    plot_HDC_Manifold_2D(hdc_model);
end

% -------------------------------------------------------------------------
% [보조 함수 1] 초고속 다중 D0 평가 벡터화 목적함수
% -------------------------------------------------------------------------
function cost = costFunction_Active_2D_Vec_Multi_D0(p2d, V2, P_target, fast_predict, margin)
    N = size(p2d, 1);
    D1 = p2d(:, 1); D2 = p2d(:, 2);
    
    if P_target > 0, d0_c = 0:10:90; else, d0_c = -90:10:0; end
    M_c = length(d0_c);
    
    V2_c = repmat(V2, M_c, N); D1_c = repmat(D1', M_c, 1); D2_c = repmat(D2', M_c, 1); D0_c = repmat(d0_c', 1, N);
    X_in_c = [V2_c(:), D0_c(:), D1_c(:), D2_c(:), D0_c(:).^2, D1_c(:).*D2_c(:), V2_c(:).*D0_c(:)];
    preds_c = fast_predict(X_in_c);
    P_preds_c = reshape(preds_c(1,:), M_c, N);
    
    [~, min_idx] = min(abs(P_preds_c - P_target));
    best_d0_c = d0_c(min_idx)'; 
    
    offsets = -5:0.5:5; M_f = length(offsets);
    D0_f = max(-90, min(90, repmat(best_d0_c', M_f, 1) + repmat(offsets', 1, N)));
    V2_f = repmat(V2, M_f, N); D1_f = repmat(D1', M_f, 1); D2_f = repmat(D2', M_f, 1);
    
    X_in_f = [V2_f(:), D0_f(:), D1_f(:), D2_f(:), D0_f(:).^2, D1_f(:).*D2_f(:), V2_f(:).*D0_f(:)];
    preds_f = fast_predict(X_in_f);
    
    P_preds_f = reshape(preds_f(1,:), M_f, N);
    I_pk_f = reshape(preds_f(3,:), M_f, N);
    err_f = abs(P_preds_f - P_target);
    cost = zeros(N, 1);
    
    for i = 1:N
        valid_idx = err_f(:, i) <= margin;
        if any(valid_idx)
            cost(i) = min(I_pk_f(valid_idx, i)); 
        else
            [min_err, min_err_idx] = min(err_f(:, i));
            cost(i) = (min_err - margin) * 5000 + I_pk_f(min_err_idx, i);
        end
    end
end

% -------------------------------------------------------------------------
% [보조 함수 2] 단일 좌표의 최종 물리 D0 역산
% -------------------------------------------------------------------------
function [best_D0, p_pred, i_pk] = find_best_D0_Multi(P_t, V2, D1, D2, bic_predict, margin)
    if P_t > 0, d0_c = 0:5:90; else, d0_c = -90:5:0; end
    n_c = length(d0_c);
    X_c = [repmat(V2, n_c, 1), d0_c(:), repmat(D1, n_c, 1), repmat(D2, n_c, 1)];
    X_in_c = [X_c, X_c(:,2).^2, X_c(:,3).*X_c(:,4), X_c(:,1).*X_c(:,2)];
    preds = bic_predict(X_in_c);
    [~, idx] = min(abs(preds(1,:) - P_t));
    
    d0_f = max(-90, d0_c(idx)-5) : 0.5 : min(90, d0_c(idx)+5);
    n_f = length(d0_f);
    X_f = [repmat(V2, n_f, 1), d0_f(:), repmat(D1, n_f, 1), repmat(D2, n_f, 1)];
    X_in_f = [X_f, X_f(:,2).^2, X_f(:,3).*X_f(:,4), X_f(:,1).*X_f(:,2)];
    preds_f = bic_predict(X_in_f);
    
    P_f = preds_f(1, :); I_f = preds_f(3, :); err_f = abs(P_f - P_t);
    valid_idx = err_f <= margin;
    
    if any(valid_idx)
        [min_I, v_idx] = min(I_f(valid_idx));
        valid_D0 = d0_f(valid_idx); valid_P = P_f(valid_idx);
        best_D0 = valid_D0(v_idx); p_pred = valid_P(v_idx); i_pk = min_I;
    else
        [~, min_err_idx] = min(err_f);
        best_D0 = d0_f(min_err_idx); p_pred = P_f(min_err_idx); i_pk = I_f(min_err_idx);
    end
end

% -------------------------------------------------------------------------
% [보조 함수 3] 극저부하 기준 앵커 추출 (D1, D2를 0에 가깝게 유도)
% -------------------------------------------------------------------------
function [anchors_X, anchors_Y] = find_initial_anchor(V2, P_t, fast_predict, margin)
    % 쌍봉낙타의 최하단부(가장 좁은 곳)를 형성하기 위해 탐색 범위를 0에 매우 가깝게 설정
    d1_c = 0:0.01:0.05; d2_c = 0:0.01:0.05;
    [D1_grid, D2_grid] = meshgrid(d1_c, d2_c);
    D1_vec = D1_grid(:); D2_vec = D2_grid(:);
    N_s = length(D1_vec);
    
    best_cur = []; min_i = inf;
    for i = 1:N_s
        [d0, p_p, i_p] = find_best_D0_Multi(P_t, V2, D1_vec(i), D2_vec(i), fast_predict, margin);
        if abs(p_p - P_t) <= margin * 1.5
            if i_p < min_i
                min_i = i_p;
                best_cur = [d0, D1_vec(i), D2_vec(i)];
            end
        end
    end
    
    if isempty(best_cur)
        if P_t > 0, d0_def = 2; else, d0_def = -2; end
        best_cur = [d0_def, 0.01, 0.01]; 
    end
    
    anchors_Y = best_cur;
    anchors_X = [P_t, V2];
end

% -------------------------------------------------------------------------
% [보조 함수 4] HDC 인코딩
% -------------------------------------------------------------------------
function [HV_int, HV_bin] = encode_HDC_data(X, min_X, max_X, Q, D, ID_vecs, L_vecs, mode)
    num_samples = size(X, 2); num_features = size(X, 1);
    HV_int = zeros(D, num_samples); HV_bin = zeros(D, num_samples);
    X_norm = (X - min_X) ./ (max_X - min_X + 1e-9);
    X_q = max(1, min(Q, round(X_norm * (Q - 1)) + 1));
    for i = 1:num_samples
        hv_sum = zeros(1, D);
        for f = 1:num_features, hv_sum = hv_sum + ID_vecs(f, :) .* L_vecs(X_q(f, i), :); end
        if strcmp(mode, 'Multi_Interact')
            hv_cross1 = (ID_vecs(1, :) .* L_vecs(X_q(1, i), :)) .* (ID_vecs(2, :) .* L_vecs(X_q(2, i), :));
            hv_cross2 = circshift(L_vecs(X_q(1, i), :), 1) .* circshift(L_vecs(X_q(2, i), :), -1);
            hv_sum = hv_sum + hv_cross1 + hv_cross2;
        end
        HV_int(:, i) = hv_sum'; bin_vec = sign(hv_sum)'; bin_vec(bin_vec == 0) = 1; HV_bin(:, i) = bin_vec;
    end
end

function preds = predict_best_in_class_lite(X_in, best_p, best_i_pk, net, PS_X, PS_Y, xgb_models, rf_models)
    N = size(X_in, 1); preds = zeros(4, N);
    X_in_norm = mapminmax('apply', X_in', PS_X);
    mlp_preds = mapminmax('reverse', net(X_in_norm), PS_Y);
    if best_p == 1, preds(1,:) = mlp_preds(1,:); elseif best_p == 2, preds(1,:) = (sign(predict(xgb_models.P, X_in)) .* (exp(abs(predict(xgb_models.P, X_in))) - 1))'; else, preds(1,:) = predict(rf_models.P, X_in)'; end
    if best_i_pk == 1, preds(3,:) = mlp_preds(3,:); elseif best_i_pk == 2, preds(3,:) = (sign(predict(xgb_models.I_pk, X_in)) .* (exp(abs(predict(xgb_models.I_pk, X_in))) - 1))'; else, preds(3,:) = predict(rf_models.I_pk, X_in)'; end
end

% -------------------------------------------------------------------------
% [보조 함수 5] 곡면 시각화 (업그레이드: 초고밀도 연속 Surf 출력)
% -------------------------------------------------------------------------
function plot_HDC_Manifold_2D(hdc_model)
    disp(' -> [디버깅] 학습된 2차원(D1, D2) HDC 제어기 초고밀도 시각화 생성 중...');
    
    P_grid = -1200:10:1200;  
    P_grid(abs(P_grid) < 5) = []; 
    V2_grid = 80:1:110;      
    
    [P_mesh, V2_mesh] = meshgrid(P_grid, V2_grid); 
    X_test = [P_mesh(:)'; V2_mesh(:)'];
    
    [~, S_bin] = encode_HDC_data(X_test, hdc_model.min_X, hdc_model.max_X, hdc_model.Q, hdc_model.D, hdc_model.ID_vecs, hdc_model.L_vecs, hdc_model.mode);
    sim = (hdc_model.C_bin * S_bin) / hdc_model.D;
    delta_prime = exp(sim * hdc_model.gamma) ./ sum(exp(sim * hdc_model.gamma), 1);
    
    Y_hat_norm = zeros(2, size(X_test, 2)); 
    for k = 1:hdc_model.num_clusters
        Y_hat_norm = Y_hat_norm + bsxfun(@times, delta_prime(k, :), (squeeze(hdc_model.M_bin(k, :, :)) * S_bin) / hdc_model.D); 
    end
    Y_pred = hdc_model.min_Y + (Y_hat_norm + 1) .* (hdc_model.max_Y - hdc_model.min_Y) / 2;
    
    Y_pred_D1 = reshape(Y_pred(1,:), size(P_mesh));
    Y_pred_D2 = reshape(Y_pred(2,:), size(P_mesh));
    
    figure('Name', 'HDC 2D Active Learned Manifold (Dense Surface)', 'Position', [200, 200, 1200, 500]);
    
    subplot(1, 2, 1); 
    surf(P_mesh, V2_mesh, Y_pred_D1, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('HDC Inferred D1 (Ratio)'); 
    xlabel('Target Power (W)'); ylabel('Target V2 (V)'); zlabel('D1 (Ratio)');
    grid on; colormap(gca, 'parula'); colorbar; view(45, 30);
    
    subplot(1, 2, 2); 
    surf(P_mesh, V2_mesh, Y_pred_D2, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    title('HDC Inferred D2 (Ratio)'); 
    xlabel('Target Power (W)'); ylabel('Target V2 (V)'); zlabel('D2 (Ratio)');
    grid on; colormap(gca, 'turbo'); colorbar; view(45, 30);
end

%% =========================================================================
% [함수 6] 모델 성능 평가 & XAI
% =========================================================================
function evaluate_Models(X_test, Y_test, X_train)
    disp('-----------------------------------------------------------');
    disp('[모델 성능 평가] 학습에 사용하지 않은 Test Set으로 일반화 성능 검증');
    % (생략 없이 원본 유지)
    nn_filename = 'trained_fast_mlp_net_v6.mat';
    if isfile(nn_filename)
        disp(' -> [1/3] MLP 7-Feat 모델 검증 중...');
        load(nn_filename, 'net', 'PS_X', 'PS_Y');
        Y_pred_MLP = mapminmax('reverse', net(mapminmax('apply', X_test, PS_X)), PS_Y);
        calc_and_plot_metrics(Y_test, Y_pred_MLP, 'MLP 7-Feat');
    end
    
    xgb_filename = 'trained_xgboost_models_v6.mat';
    if isfile(xgb_filename)
        disp(' -> [2/3] XGBoost 모델 검증 중...');
        load(xgb_filename, 'xgb_models');
        Y_pred_XGB = zeros(size(Y_test));
        Y_pred_XGB(1,:) = sign(predict(xgb_models.P, X_test')') .* (exp(abs(predict(xgb_models.P, X_test')')) - 1);
        Y_pred_XGB(2,:) = sign(predict(xgb_models.P_HV, X_test')') .* (exp(abs(predict(xgb_models.P_HV, X_test')')) - 1);
        Y_pred_XGB(3,:) = sign(predict(xgb_models.I_pk, X_test')') .* (exp(abs(predict(xgb_models.I_pk, X_test')')) - 1);
        Y_pred_XGB(4,:) = predict(xgb_models.Eff_UQ, X_test')'; 
        calc_and_plot_metrics(Y_test, Y_pred_XGB, 'XGBoost');
        disp(' -> [XAI] XGBoost 모델의 PDP 시각화를 시작합니다...');
        run_XAI_Analysis(xgb_models, X_train);
    end
    
    rf_filename = 'trained_rf_models_v6.mat';
    if isfile(rf_filename)
        disp(' -> [3/3] Random Forest 모델 검증 중...');
        load(rf_filename, 'rf_models');
        Y_pred_RF = zeros(size(Y_test));
        Y_pred_RF(1,:) = predict(rf_models.P, X_test')';
        Y_pred_RF(2,:) = predict(rf_models.P_HV, X_test')';
        Y_pred_RF(3,:) = predict(rf_models.I_pk, X_test')';
        Y_pred_RF(4,:) = predict(rf_models.Eff, X_test')';
        calc_and_plot_metrics(Y_test, Y_pred_RF, 'Random Forest');
    end
    disp(' ');
end

function calc_and_plot_metrics(Y_actual, Y_pred, model_name)
    target_names = {'P_{act} (W)', 'P_{act_HV} (W)', 'i_{pk} (A)', 'Efficiency (%)'};
    figure('Name', [model_name, ' Evaluation'], 'Position', [100, 200, 1500, 800]);
    
    for i = 1:4
        y_true = Y_actual(i, :); y_hat = Y_pred(i, :);
        rmse = sqrt(mean((y_true - y_hat).^2));
        mae = mean(abs(y_true - y_hat));
        R2 = 1 - (sum((y_true - y_hat).^2) / sum((y_true - mean(y_true)).^2));
        
        fprintf(' [%s] Test R^2: %.4f, RMSE: %.4f, MAE: %.4f\n', target_names{i}, R2, rmse, mae);
        
        subplot(2, 2, i);
        scatter(y_true, y_hat, 15, 'b', 'filled', 'MarkerFaceAlpha', 0.4); hold on;
        m_val = min([y_true, y_hat]); M_val = max([y_true, y_hat]); mg = (M_val - m_val) * 0.05; 
        plot([m_val-mg, M_val+mg], [m_val-mg, M_val+mg], 'r--', 'LineWidth', 2); hold off;
        
        title({target_names{i}, sprintf('Test R^2 = %.4f', R2)}, 'Interpreter', 'tex');
        xlabel('Actual'); ylabel('Predicted'); grid on; axis square;
        xlim([m_val-mg, M_val+mg]); ylim([m_val-mg, M_val+mg]);
    end
end

function run_XAI_Analysis(xgb_models, X_train)
    figure('Name', 'XAI: Partial Dependence Plots', 'Position', [150, 150, 1600, 800]);
    feature_indices = [2, 3, 4]; 
    feature_titles = {'D0_{deg}', 'D1', 'D2'};
    
    num_total = size(X_train, 2);
    sub_idx = randperm(num_total, min(5000, num_total));
    X_sub = X_train(:, sub_idx)'; 
    
    for j = 1:3
        subplot(2, 3, j);
        try
            plotPartialDependence(xgb_models.I_pk, feature_indices(j), X_sub);
        catch
            plotPartialDependence(xgb_models.I_pk, X_sub, feature_indices(j));
        end
        title(['PDP: ', feature_titles{j}, ' on I_{pk}']); grid on;
    end
    
    for j = 1:3
        subplot(2, 3, j+3);
        try
            plotPartialDependence(xgb_models.Eff_UQ, feature_indices(j), X_sub);
        catch
            plotPartialDependence(xgb_models.Eff_UQ, X_sub, feature_indices(j));
        end
        title(['PDP: ', feature_titles{j}, ' on Efficiency']); grid on;
    end
end

%% =========================================================================
% [함수 7] RegHD 논문 기반: 실시간 군집화 + 이중 상태(Dual-State) 양자화 RegHDC
% =========================================================================
function run_Regression_HDC()
    disp('-----------------------------------------------------------');
    disp('[Method 7] RegHD: 실시간 군집화 및 이중 상태(Integer/Binary) 양자화 제어기 학습');
    disp(' -> 논문(RegHD)의 방식을 적용하여 K-Means를 제거하고 HD 공간에서 런타임 클러스터링과');
    disp(' -> 이진/정수형 모델 교차 업데이트(Iterative Learning)를 수행합니다.');

    % 다중 후보군 파일 로드
    loaded_file = 'Candidate_Dataset_V6.csv';
    if ~isfile(loaded_file)
        disp(' -> [경고] Candidate_Dataset_V6.csv가 없습니다. 임시 데이터를 로드합니다.');
        loaded_file = 'Massive_Optimized_TPS_Dataset_PSO_UQ_V6.csv';
        if ~isfile(loaded_file), return; end
    end

    disp([' -> 학습 데이터 로드 완료: ', loaded_file]);
    opt_data = readtable(loaded_file);

    % 입력(Target P, V2) / 출력(D0, D1, D2)
    X_opt = [opt_data.Target_P_W, opt_data.Target_V2_V]';
    Y_opt = [opt_data.Opt_D0_deg, opt_data.Opt_D1, opt_data.Opt_D2]';

    % 데이터 분할 (80% Train, 20% Test)
    rng(42);
    cv = cvpartition(size(X_opt, 2), 'HoldOut', 0.2);
    X_tr = X_opt(:, training(cv)); Y_tr = Y_opt(:, training(cv));
    X_te = X_opt(:, test(cv));     Y_te = Y_opt(:, test(cv));

    % HDC 파라미터 설정
    D = 10000;
    Q = 500;
    mode = 'Multi_Interact';
    num_clusters = 3; % K=3 모드
    gamma = 10.0; % Softmax Temperature
    
    % 양자화 바운더리 매핑
    min_X = min(X_tr, [], 2);
    max_X = max(X_tr, [], 2);
    [Y_tr_norm, PS_Y_opt] = mapminmax(Y_tr, -1, 1);
    
    % Base Hypervectors 생성
    rng(100);
    ID_vecs = randsrc(size(X_tr,1), D, [-1, 1]);
    L_vecs = zeros(Q, D);
    L_vecs(1, :) = randsrc(1, D, [-1, 1]);
    flip_counts = max(1, floor(D / 2 / (Q - 1)));
    
    for q_idx = 2:Q
        L_vecs(q_idx, :) = L_vecs(q_idx-1, :);
        flip_idx = randperm(D, flip_counts);
        L_vecs(q_idx, flip_idx) = -L_vecs(q_idx, flip_idx);
    end
    
    % [논문 핵심] 정수형(Integer)과 이진형(Binary) 이중 인코딩
    disp(' -> 데이터를 HDC 하이퍼공간(D=10000)으로 이중(Dual) 인코딩 중...');
    [HV_tr_int, HV_tr_bin] = encode_HDC_dual(X_tr, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
    [HV_te_int, HV_te_bin] = encode_HDC_dual(X_te, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);

    % -------------------------------------------------------------
    % [Step 1] Dual-State 클러스터 및 모델 초기화
    % -------------------------------------------------------------
    % 클러스터 모델: Integer는 무작위 이진값, Binary는 Integer의 Sign값
    C_int = randsrc(num_clusters, D, [-1, 1]); 
    C_bin = C_int; 
    
    % 회귀 모델: Integer는 0으로 초기화, Binary는 1로 초기화
    M_int = zeros(num_clusters, 3, D);
    M_bin = ones(num_clusters, 3, D);

    % -------------------------------------------------------------
    % [Step 2] 실시간 군집화 및 반복적 에러 업데이트 (Training)
    % -------------------------------------------------------------
    epochs = 30;
    init_lr = 0.05;
    batch_size = 500;
    num_train = size(HV_tr_int, 2);
    
    disp(' -> [Training] 실시간 군집화 및 이진화 교차 업데이트 시작...');
    
    for ep = 1:epochs
        lr = init_lr * (0.95 ^ ep); % Learning rate decay
        
        % 미니 배치 셔플링
        shuffle_idx = randperm(num_train);
        
        for bs = 1:batch_size:num_train
            be = min(bs + batch_size - 1, num_train);
            idx_batch = shuffle_idx(bs:be);
            B = length(idx_batch);
            
            S_int_batch = HV_tr_int(:, idx_batch); % [D x B]
            S_bin_batch = HV_tr_bin(:, idx_batch); % [D x B]
            Y_true_batch = Y_tr_norm(:, idx_batch); % [3 x B]
            
            % 1. 클러스터 유사도 검색 (Binary Hamming 기반 Cosine Similarity)
            % C_bin [K x D] * S_bin [D x B] -> [K x B]
            sim = (C_bin * S_bin_batch) / D; 
            [max_sim, best_k_idx] = max(sim, [], 1); % 가장 가까운 모드 탐색
            
            % 2. 클러스터 센터 업데이트 (Integer 모델 사용)
            for b = 1:B
                k = best_k_idx(b);
                % 논문 수식: C_l = C_l + (1 - \delta_l) * S
                C_int(k, :) = C_int(k, :) + (1 - max_sim(b)) * S_int_batch(:, b)';
            end
            
            % 3. Softmax 신뢰도 계산 (Confidence)
            % 논문: \delta' = Softmax(\delta)
            exp_sim = exp(sim * gamma);
            delta_prime = exp_sim ./ sum(exp_sim, 1); % [K x B]
            
            % 4. 다중 모델 가중 합산 예측 (Binary 모델 사용)
            Y_hat = zeros(3, B);
            for k = 1:num_clusters
                M_k_bin = squeeze(M_bin(k, :, :)); % [3 x D]
                % Binary 모델과 Binary 입력 내적
                pred_k = (M_k_bin * S_bin_batch) / D; % [3 x B]
                
                % Softmax 신뢰도(\delta')를 가중치로 합산
                Y_hat = Y_hat + bsxfun(@times, pred_k, delta_prime(k, :));
            end
            
            % 5. 회귀 모델 업데이트 (Integer 모델 사용)
            % 논문 수식: M_i = M_i + \alpha * (y - \hat{y}) * S
            error = Y_true_batch - Y_hat; % [3 x B]
            
            for k = 1:num_clusters
                % 클러스터별 신뢰도 기반 에러 가중 적용
                weighted_error = bsxfun(@times, error, delta_prime(k, :)); % [3 x B]
                
                % Integer 모델 누적
                M_int(k, :, :) = squeeze(M_int(k, :, :)) + lr * (weighted_error * S_int_batch');
            end
            
            % 6. 모델 이진화 (Binarization) - 매 배치 종료 후 최신화
            C_bin = sign(C_int); C_bin(C_bin == 0) = 1;
            M_bin = sign(M_int); M_bin(M_bin == 0) = 1;
        end
        
        if mod(ep, 10) == 0
            fprintf('   - Epoch %d / %d 완료 (Learning Rate: %.4f)\n', ep, epochs, lr);
        end
    end

    % -------------------------------------------------------------
    % [Step 3] Test Set 평가 (Inference - Weighted Accumulation)
    % -------------------------------------------------------------
    disp(' -> Test Set 다중 모델(Multi-Model) 앙상블 추론 검증 시작...');
    
    % Binary 연산으로 유사도 검색
    sim_te = (C_bin * HV_te_bin) / D; 
    delta_prime_te = exp(sim_te * gamma) ./ sum(exp(sim_te * gamma), 1);
    
    Y_pred_te_norm = zeros(3, size(HV_te_bin, 2));
    for k = 1:num_clusters
        M_k_bin = squeeze(M_bin(k, :, :));
        pred_k = (M_k_bin * HV_te_bin) / D;
        
        % 모든 모델의 출력을 신뢰도(Confidence)에 따라 가중 합산
        Y_pred_te_norm = Y_pred_te_norm + bsxfun(@times, pred_k, delta_prime_te(k, :));
    end
    
    % 정규화 역변환
    Y_pred_te = mapminmax('reverse', Y_pred_te_norm, PS_Y_opt);
    
    % 성능 평가
    R2_sum = 0;
    for c = 1:3
        y_true_c = Y_te(c, :); y_hat_c = Y_pred_te(c, :);
        R2_sum = R2_sum + (1 - (sum((y_true_c - y_hat_c).^2) / sum((y_true_c - mean(y_true_c)).^2)));
    end
    fprintf(' -> 다중 모델 이진 양자화 RegHDC 평균 Test R^2: %.4f\n', R2_sum / 3);

    % 제어기 탑재용 모델 저장 (추론 시 Binary 모델만 사용)
    hdc_model = struct();
    hdc_model.C_bin = C_bin;     % 하드웨어 탑재용 Binary 클러스터 센터
    hdc_model.M_bin = M_bin;     % 하드웨어 탑재용 Binary 회귀 가중치
    hdc_model.C_int = C_int;     % 추가 RL 튜닝을 위한 Integer 백업
    hdc_model.M_int = M_int;     % 추가 RL 튜닝을 위한 Integer 백업
    hdc_model.ID_vecs = ID_vecs;
    hdc_model.L_vecs = L_vecs;
    hdc_model.min_X = min_X;
    hdc_model.max_X = max_X;
    hdc_model.Q = Q;
    hdc_model.D = D;
    hdc_model.gamma = gamma;
    hdc_model.PS_Y_opt = PS_Y_opt;
    hdc_model.mode = mode;
    hdc_model.num_clusters = num_clusters;
    
    save('trained_hdc_controller_model_v6.mat', 'hdc_model');
    disp(' -> [성공] RegHD 알고리즘 최적화 Multi-Model HDC 제어기 저장 완료');
    
    calc_and_plot_metrics(Y_te, Y_pred_te, 'RegHD Multi-Model Quantized');
end


%% -------------------------------------------------------------------------
% [보조 함수] RegHD 논문형 이중 출력 인코더 (Integer & Binary 모두 반환)
% -------------------------------------------------------------------------
function [HV_int, HV_bin] = encode_HDC_dual(X, min_X, max_X, Q, D, ID_vecs, L_vecs, mode)
    num_samples = size(X, 2);
    num_features = size(X, 1);
    
    HV_int = zeros(D, num_samples);
    HV_bin = zeros(D, num_samples);

    X_norm = (X - min_X) ./ (max_X - min_X + 1e-9);
    X_q = round(X_norm * (Q - 1)) + 1;
    X_q = max(1, min(Q, X_q));

    for i = 1:num_samples
        hv_sum = zeros(1, D);
        for f = 1:num_features
            hv_sum = hv_sum + ID_vecs(f, :) .* L_vecs(X_q(f, i), :);
        end
        
        if num_features >= 2
            if strcmp(mode, 'Interact')
                hv_cross = (ID_vecs(1, :) .* L_vecs(X_q(1, i), :)) .* (ID_vecs(2, :) .* L_vecs(X_q(2, i), :));
                hv_sum = hv_sum + hv_cross; 
            elseif strcmp(mode, 'Multi_Interact')
                hv_cross1 = (ID_vecs(1, :) .* L_vecs(X_q(1, i), :)) .* (ID_vecs(2, :) .* L_vecs(X_q(2, i), :));
                hv_cross2 = circshift(L_vecs(X_q(1, i), :), 1) .* circshift(L_vecs(X_q(2, i), :), -1);
                hv_sum = hv_sum + hv_cross1 + hv_cross2;
            elseif strcmp(mode, 'Multi_Interact_Shift')
                hv_cross1 = (ID_vecs(1, :) .* L_vecs(X_q(1, i), :)) .* (ID_vecs(2, :) .* L_vecs(X_q(2, i), :));
                hv_cross2 = (ID_vecs(1, :) .* circshift(L_vecs(X_q(2, i), :), 5)) .* L_vecs(X_q(1, i), :);
                hv_cross3 = circshift(L_vecs(X_q(1, i), :), 2) .* circshift(L_vecs(X_q(2, i), :), -2);
                hv_sum = hv_sum + hv_cross1 + hv_cross2 + hv_cross3;
            end
        end
        
        % [논문 원리] 업데이트용 정수(Integer)와 추론/검색용 이진(Binary) 분리
        HV_int(:, i) = hv_sum';
        bin_vec = sign(hv_sum)';
        bin_vec(bin_vec == 0) = 1; % 0은 1로 치환하여 완벽한 Binary(Bipolar) 구성
        HV_bin(:, i) = bin_vec;
    end
end

%% =========================================================================
% [유틸리티 및 목적 함수들]
% =========================================================================

function X_feat = get_features_row(V2, D0, D1, D2)
    X_feat = [V2, D0, D1, D2, D0^2, D1*D2, V2*D0];
end

function [target_conditions, total_steps] = setup_target_grid(is_massive)
    if nargin < 1, is_massive = false; end
    if is_massive
        target_P_list = [-1200:5:-50, 50:5:1200];  
        target_V2_list = 80:0.5:110;  
    else
        target_P_list = [-1200:50:-50, 50:50:1200];  
        target_V2_list = 80:2:110;   
    end
    [P_grid, V2_grid] = meshgrid(target_P_list, target_V2_list);
    target_conditions = [P_grid(:), V2_grid(:)]; 
    total_steps = size(target_conditions, 1);
end

function [lb, ub] = get_dynamic_bounds(data, P_target, V2_target)
    dist = sqrt(((data.V2_V - V2_target) / 30).^2 + ((data.P_act_W - P_target) / 1200).^2);
    [~, sorted_idx] = sort(dist);
    k_nn = 30; 
    valid_idx = sorted_idx(1:min(k_nn, length(sorted_idx)));
    
    margin_D0 = 15;
    margin_D = 0.1;
    
    lb = [max(-90, min(data.D0_deg(valid_idx)) - margin_D0), ...
          max(0, min(data.D1(valid_idx)) - margin_D), ...
          max(0, min(data.D2(valid_idx)) - margin_D)];
          
    ub = [min(90, max(data.D0_deg(valid_idx)) + margin_D0), ...
          min(0.5, max(data.D1(valid_idx)) + margin_D), ...
          min(0.5, max(data.D2(valid_idx)) + margin_D)];
end

function save_final_csv(optimal_dataset, csv_name)
    if size(optimal_dataset, 2) == 10
        vars = {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'AI_Predicted_Eff_pct', 'Eff_Lower_95', 'Eff_Upper_95'};
    else
        vars = {'Target_P_W', 'Target_V2_V', 'Opt_D0_deg', 'Opt_D1', 'Opt_D2', 'Simulated_P_W', 'Min_i_pk_A', 'AI_Predicted_Eff_pct'};
    end
    optimal_T = array2table(optimal_dataset, 'VariableNames', vars);
    writetable(optimal_T, csv_name);
    disp(['=> [저장 완료] 대규모 최적화 데이터셋 "', csv_name, '" 이 성공적으로 생성되었습니다.']);
    disp(' ');
end

function cost = costFunction_XGB_LogPSO(phases, V2_target, P_target, xgb_models)
    N = size(phases, 1);
    V2_vec = repmat(V2_target, N, 1);
    D0_vec = phases(:, 1); D1_vec = phases(:, 2); D2_vec = phases(:, 3);
    X_in = [V2_vec, D0_vec, D1_vec, D2_vec, D0_vec.^2, D1_vec.*D2_vec, V2_vec.*D0_vec];
    
    P_pred_log = predict(xgb_models.P, X_in);
    P_pred = sign(P_pred_log) .* (exp(abs(P_pred_log)) - 1);
    P_HV_pred_log = predict(xgb_models.P_HV, X_in);
    P_HV_pred = sign(P_HV_pred_log) .* (exp(abs(P_HV_pred_log)) - 1);
    I_pk_pred_log = predict(xgb_models.I_pk, X_in);
    I_pk_pred = sign(I_pk_pred_log) .* (exp(abs(I_pk_pred_log)) - 1);
    eff_pred = predict(xgb_models.Eff_UQ, X_in); 
    
    error_power_W = abs(P_pred - P_target);
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    penalty = zeros(N, 1);
    penalty = penalty + max(0, I_pk_pred - 30) * 1000;
    penalty = penalty + max(0, eff_pred - 100) * 1000 + max(0, -eff_pred) * 1000;
    penalty = penalty + max(0, error_power_W - max(10, abs(P_target)*0.02)) * 5000;
    
    tie_breaker_sps = 1e-3 * ((0.5 - D1_vec).^2 + (0.5 - D2_vec).^2);
    cost = (100.0 .* error_power_W) + (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) - (1.0 .* eff_pred) + penalty + tie_breaker_sps;
end

function cost = costFunction_RF_PSO(phases, V2_target, P_target, rf_models)
    N = size(phases, 1);
    V2_vec = repmat(V2_target, N, 1);
    D0_vec = phases(:, 1); D1_vec = phases(:, 2); D2_vec = phases(:, 3);
    X_in = [V2_vec, D0_vec, D1_vec, D2_vec, D0_vec.^2, D1_vec.*D2_vec, V2_vec.*D0_vec];
    
    P_pred = predict(rf_models.P, X_in);
    P_HV_pred = predict(rf_models.P_HV, X_in);
    I_pk_pred = predict(rf_models.I_pk, X_in);
    eff_pred = predict(rf_models.Eff, X_in);
    
    error_power_W = abs(P_pred - P_target);
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    penalty = zeros(N, 1);
    penalty = penalty + max(0, I_pk_pred - 30) * 1000;
    penalty = penalty + max(0, eff_pred - 100) * 1000 + max(0, -eff_pred) * 1000;
    penalty = penalty + max(0, error_power_W - max(10, abs(P_target)*0.02)) * 5000;
    
    tie_breaker_sps = 1e-3 * ((0.5 - D1_vec).^2 + (0.5 - D2_vec).^2);
    cost = (100.0 .* error_power_W) + (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) - (1.0 .* eff_pred) + penalty + tie_breaker_sps;
end

function Y_batch_norm = predict_with_batches(net, X_batch_norm, batch_size)
    num_samples = size(X_batch_norm, 2);
    Y_batch_norm = zeros(4, num_samples); 
    for bs = 1:batch_size:num_samples
        be = min(bs + batch_size - 1, num_samples);
        Y_batch_norm(:, bs:be) = net(X_batch_norm(:, bs:be));
    end
end

function [best_phases, opt_vals] = evaluate_candidates(phase_cands, Y_batch_raw, P_target)
    P_pred     = Y_batch_raw(1, :)';
    P_HV_pred  = Y_batch_raw(2, :)';
    I_pk_pred  = Y_batch_raw(3, :)';
    eff_pred   = Y_batch_raw(4, :)'; 
    
    error_power_W = abs(P_pred - P_target);
    P_loss_pred = abs(P_pred - P_HV_pred);
    
    penalty = zeros(size(P_pred));
    penalty = penalty + max(0, I_pk_pred - 30) .* 1000;
    penalty = penalty + max(0, eff_pred - 100) .* 1000 + max(0, -eff_pred) .* 1000;
    penalty = penalty + max(0, error_power_W - max(10, abs(P_target)*0.02)) .* 5000;
    
    D1_vec = phase_cands(:, 2);
    D2_vec = phase_cands(:, 3);
    tie_breaker_sps = 1e-3 .* ((0.5 - D1_vec).^2 + (0.5 - D2_vec).^2);
    
    cost_array = (100.0 .* error_power_W) + (1.0 .* I_pk_pred) + (2.0 .* P_loss_pred) - (1.0 .* eff_pred) + penalty + tie_breaker_sps;
    
    [min_cost, min_cost_idx] = min(cost_array);
    
    if min_cost >= 1e5
        fallback_cost = (100.0 .* error_power_W) + (1.0 .* I_pk_pred);
        [~, min_cost_idx] = min(fallback_cost);
    end
    
    best_phases = phase_cands(min_cost_idx, :);
    opt_vals = [P_pred(min_cost_idx), P_HV_pred(min_cost_idx), I_pk_pred(min_cost_idx), eff_pred(min_cost_idx)];
end

%% =========================================================================
% [함수 8] 효율 전용 스페셜리스트 모델 설정 알고리즘 경쟁 학습
% =========================================================================
function run_Efficiency_Specialist(X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 8] 효율(Efficiency) 특화 대리 모델 에이전트 경쟁 학습 (AutoML)');
    
    [X_norm, PS_X] = mapminmax(X_train);
    
    Y_eff_train = Y_train(4, :);
    Y_eff_test = Y_test(4, :);
    [Y_eff_norm, PS_Y_eff] = mapminmax(Y_eff_train);
    X_test_norm = mapminmax('apply', X_test, PS_X);
    
    eff_spec_file = 'trained_eff_specialist_v6.mat';
    needs_training = true;
    
    if isfile(eff_spec_file)
        load(eff_spec_file, 'eff_specialist', 'eff_spec_type', 'PS_X_eff', 'PS_Y_eff_s');
        disp(' -> 기존 효율 전용 모델을 발견했습니다. 성능 자가 진단 중...');
        if strcmp(eff_spec_type, 'NN')
            Y_test_eff_norm = mapminmax('apply', X_test, PS_X_eff);
            Y_eff_pred = mapminmax('reverse', eff_specialist(Y_test_eff_norm), PS_Y_eff_s);
        else
            Y_eff_pred = predict(eff_specialist, X_test')';
        end
        R2 = 1 - (sum((Y_eff_test - Y_eff_pred).^2) / sum((Y_eff_test - mean(Y_eff_test)).^2));
        fprintf(' -> 현재 스페셜리스트의 모델 Test R^2 = %.4f\n', R2);
        
        user_ans = input(' -> 다시 경쟁시켜 더 나은 모델을 찾으시겠습니까? (Y/N): ', 's');
        if strcmpi(user_ans, 'N') || isempty(user_ans)
            needs_training = false;
        end
    end
    
    if needs_training
        disp(' -> 효율 예측 정확도를 극대화하기 위한 가상 경쟁(AutoML)을 시작합니다.');
        agent_hidden_sizes = {[64, 64], [128, 64, 32], [256, 128, 64], [256, 256, 128, 64]};
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
        disp([' -> 총 ', num2str(num_agents), '개의 튜닝 신경망 에이전트를 생성하여 병렬(parfor) 경쟁 학습을 실시합니다...']);
        
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
            agent_net.trainParam.epochs = 5000;
            agent_net.trainParam.max_fail = 100;
            
            [agent_net_trained, ~] = train(agent_net, X_norm, Y_eff_norm);
            
            Y_test_pred_norm = agent_net_trained(X_test_norm);
            Y_test_pred = mapminmax('reverse', Y_test_pred_norm, PS_Y_eff);
            
            test_mses(i) = mean((Y_eff_test - Y_test_pred).^2);
            trained_nets{i} = agent_net_trained;
        end
        
        disp(' -> 비선형 트리 앙상블(Random Forest, XGBoost) 에이전트 학습 추가 진행 중(단일 코어)...');
        mdl_rf = TreeBagger(100, X_train', Y_eff_train', 'Method', 'regression', 'MinLeafSize', 5);
        pred_rf = predict(mdl_rf, X_test')';
        mse_rf = mean((Y_eff_test - pred_rf).^2);
        
        mdl_lsb = fitrensemble(X_train', Y_eff_train', 'Method', 'LSBoost', 'NumLearningCycles', 100, 'LearnRate', 0.1);
        pred_lsb = predict(mdl_lsb, X_test')';
        mse_lsb = mean((Y_eff_test - pred_lsb).^2);
        
        disp('===========================================================');
        disp(' [효율 스페셜리스트 에이전트 경쟁 결과 (Test MSE 기준 정렬)]');
        [sorted_mses, sorted_idx] = sort(test_mses);
        
        best_nn_mse = sorted_mses(1);
        best_nn_net = trained_nets{sorted_idx(1)};
        best_nn_agent = agents(sorted_idx(1));
        
        all_mses = [best_nn_mse, mse_rf, mse_lsb];
        model_names = {['MLP 구조: ', num2str(best_nn_agent.hiddenSize), ' (', best_nn_agent.trainFcn, ')'], ...
                       'Random Forest (100 Trees)', ...
                       'XGBoost (LSBoost, 100 Cycles)'};
        
        [sorted_all, rank_idx] = sort(all_mses);
        for k = 1:3
            disp([' Rank ', num2str(k), ' | ', model_names{rank_idx(k)}, ' | Test MSE: ', num2str(sorted_all(k), '%.4f')]);
        end
        
        disp('-----------------------------------------------------------');
        if rank_idx(1) == 1
            disp([' -> [최종 챔피언] 신경망 에이전트 채택!']);
            eff_specialist = best_nn_net;
            eff_spec_type = 'NN';
            PS_X_eff = PS_X;
            PS_Y_eff_s = PS_Y_eff;
        elseif rank_idx(1) == 2
            disp([' -> [최종 챔피언] Random Forest 앙상블 채택!']);
            eff_specialist = mdl_rf;
            eff_spec_type = 'RF';
            PS_X_eff = []; PS_Y_eff_s = [];
        else
            disp([' -> [최종 챔피언] XGBoost(LSBoost) 앙상블 채택!']);
            eff_specialist = mdl_lsb;
            eff_spec_type = 'LSB';
            PS_X_eff = []; PS_Y_eff_s = [];
        end
        
        save(eff_spec_file, 'eff_specialist', 'eff_spec_type', 'PS_X_eff', 'PS_Y_eff_s');
        disp([' -> 학습된 전용 모델 저장 완료: ', eff_spec_file]);
    end
end

%% =========================================================================
% [함수 9] 최적 위상 3D Surface 시각화
% =========================================================================
function run_3D_Plot_Phases(target_file)
    disp('-----------------------------------------------------------');
    disp('[Method 9] Target P, V2 에 대한 최적 위상(D0, D1, D2) 3D 플롯');
    
    if nargin >= 1 && isfile(target_file)
        loaded_file = target_file;
    else
        possible_files = {'Candidate_Dataset_V6.csv', ...
                          'Massive_Optimized_TPS_Dataset_BestInClass_V6.csv', ...
                          'Massive_Optimized_TPS_Dataset_RF_PSO_V6.csv', ...
                          'Massive_Optimized_TPS_Dataset_PSO_UQ_V6.csv', ...
                          'Massive_Optimized_TPS_Dataset_FastMLP_V6.csv'};
        loaded_file = '';
        for i = 1:length(possible_files)
            if isfile(possible_files{i})
                loaded_file = possible_files{i};
                break;
            end
        end
    end

    if isempty(loaded_file)
        disp(' -> [오류] 시각화할 최적 데이터셋이 없습니다. 5번 메뉴 등을 먼저 실행하세요.');
        return;
    end

    disp([' -> 최적 데이터셋 로드 완료: ', loaded_file]);
    opt_data = readtable(loaded_file);

    P = opt_data.Target_P_W;
    V2 = opt_data.Target_V2_V;
    D0 = opt_data.Opt_D0_deg;
    D1 = opt_data.Opt_D1;
    D2 = opt_data.Opt_D2;

    valid_idx = (D2 >= 0) & (D2 <= 1.0) & (abs(D0) <= 90);
    P = P(valid_idx); V2 = V2(valid_idx); D0 = D0(valid_idx); D1 = D1(valid_idx); D2 = D2(valid_idx);
    
    has_eff = ismember('AI_Predicted_Eff_pct', opt_data.Properties.VariableNames);
    if has_eff
        Eff = opt_data.AI_Predicted_Eff_pct(valid_idx);
    end

    figure('Name', 'Optimized Phases 3D Visualization', 'Position', [100, 150, 1600, 500]);

    subplot(1, 3, 1);
    scatter3(P, V2, D0, 15, D0, 'filled', 'MarkerFaceAlpha', 0.8);
    title('Target P & V2 vs Optimized D0');
    xlabel('Target Power (W)'); ylabel('Target V2 (V)'); zlabel('D0 (deg)');
    grid on; colorbar; colormap(gca, 'jet'); view(45, 30);

    subplot(1, 3, 2);
    scatter3(P, V2, D1, 15, D1, 'filled', 'MarkerFaceAlpha', 0.8);
    title('Target P & V2 vs Optimized D1');
    xlabel('Target Power (W)'); ylabel('Target V2 (V)'); zlabel('D1 (Ratio)');
    grid on; colorbar; colormap(gca, 'parula'); view(45, 30);

    subplot(1, 3, 3);
    if has_eff
        scatter3(P, V2, D2, 15, Eff, 'filled', 'MarkerFaceAlpha', 0.8);
        title('Optimized D2 (Color: Efficiency %)');
        c = colorbar; c.Label.String = 'Efficiency (%)';
    else
        scatter3(P, V2, D2, 15, D2, 'filled', 'MarkerFaceAlpha', 0.8);
        title('Target P & V2 vs Optimized D2');
        colorbar;
    end
    xlabel('Target Power (W)'); ylabel('Target V2 (V)'); zlabel('D2 (Ratio)');
    grid on; colormap(gca, 'turbo'); view(45, 30);
    
    disp(' -> 3D 시각화가 완료되었습니다. 창을 확인해주세요!');
end

%% =========================================================================
% [함수 11] 앵커 데이터 기반 대리 모델 학습 + 10만건 연속 데이터 증강
% =========================================================================
function run_Surrogate_Augmentation(X_train, Y_train, X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 11] 앵커 데이터 기반 역회귀(Inverse Regression) 대리 모델 학습 & 10만건 증강');
    
    anchor_file = 'Candidate_Dataset_V6.csv';
    if ~isfile(anchor_file)
        disp(' -> [오류] 앵커 데이터셋이 없습니다. 5번 메뉴를 먼저 실행하세요.');
        return;
    end
    
    disp([' -> 앵커 데이터셋 로드: ', anchor_file]);
    anchor_data = readtable(anchor_file);
    
    disp(' ');
    disp(' -> [1단계] 앵커 데이터를 이용하여 연속형(Continuous) 매핑 역회귀 대리 모델을 학습합니다.');
    
    X_med = [anchor_data.Target_P_W, anchor_data.Target_V2_V]';
    Y_med = [anchor_data.Opt_D0_deg, anchor_data.Opt_D1, anchor_data.Opt_D2]';
    [X_med_norm, PS_X_med] = mapminmax(X_med);
    [Y_med_norm, PS_Y_med] = mapminmax(Y_med);
    
    phase_surrogate_net = fitnet([64, 64]);
    phase_surrogate_net.trainParam.showWindow = false;
    phase_surrogate_net.trainParam.epochs = 2000;
    phase_surrogate_net = train(phase_surrogate_net, X_med_norm, Y_med_norm);
    
    save('trained_phase_surrogate_v6.mat', 'phase_surrogate_net', 'PS_X_med', 'PS_Y_med');
    disp(' -> 역회귀 대리 모델 학습 및 저장 완료: trained_phase_surrogate_v6.mat');
    
    disp(' ');
    disp(' -> [2단계] 물리 검증용 Best-in-Class 메타 모델을 로드합니다...');
    
    mlp_file = 'trained_fast_mlp_net_v6.mat';
    xgb_file = 'trained_xgboost_models_v6.mat';
    rf_file  = 'trained_rf_models_v6.mat';
    eff_spec_file = 'trained_eff_specialist_v6.mat';
    
    if ~isfile(mlp_file) || ~isfile(xgb_file) || ~isfile(rf_file) || ~isfile(eff_spec_file)
        disp(' -> [오류] 1, 2, 3, 8번 모델이 모두 필요합니다. 먼저 실행하세요.');
        return;
    end
    
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    load(xgb_file, 'xgb_models');
    load(rf_file, 'rf_models');
    load(eff_spec_file, 'eff_specialist', 'eff_spec_type', 'PS_X_eff', 'PS_Y_eff_s');
    
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_pred_mlp = mapminmax('reverse', net(X_test_norm), PS_Y);
    Y_pred_xgb = zeros(size(Y_test));
    Y_pred_xgb(1,:) = sign(predict(xgb_models.P, X_test')') .* (exp(abs(predict(xgb_models.P, X_test')')) - 1);
    Y_pred_xgb(2,:) = sign(predict(xgb_models.P_HV, X_test')') .* (exp(abs(predict(xgb_models.P_HV, X_test')')) - 1);
    Y_pred_xgb(3,:) = sign(predict(xgb_models.I_pk, X_test')') .* (exp(abs(predict(xgb_models.I_pk, X_test')')) - 1);
    Y_pred_xgb(4,:) = predict(xgb_models.Eff_UQ, X_test')';
    Y_pred_rf = zeros(size(Y_test));
    Y_pred_rf(1,:) = predict(rf_models.P, X_test')';
    Y_pred_rf(2,:) = predict(rf_models.P_HV, X_test')';
    Y_pred_rf(3,:) = predict(rf_models.I_pk, X_test')';
    Y_pred_rf(4,:) = predict(rf_models.Eff, X_test')';
    
    [~, best_p] = min([mean((Y_test(1,:)-Y_pred_mlp(1,:)).^2), mean((Y_test(1,:)-Y_pred_xgb(1,:)).^2), mean((Y_test(1,:)-Y_pred_rf(1,:)).^2)]);
    [~, best_p_hv] = min([mean((Y_test(2,:)-Y_pred_mlp(2,:)).^2), mean((Y_test(2,:)-Y_pred_xgb(2,:)).^2), mean((Y_test(2,:)-Y_pred_rf(2,:)).^2)]);
    [~, best_i_pk] = min([mean((Y_test(3,:)-Y_pred_mlp(3,:)).^2), mean((Y_test(3,:)-Y_pred_xgb(3,:)).^2), mean((Y_test(3,:)-Y_pred_rf(3,:)).^2)]);
    if strcmp(eff_spec_type, 'NN')
        X_te_eff_n = mapminmax('apply', X_test, PS_X_eff);
        Y_pred_eff_spec = mapminmax('reverse', eff_specialist(X_te_eff_n), PS_Y_eff_s);
    else
        Y_pred_eff_spec = predict(eff_specialist, X_test')';
    end
    [~, best_eff] = min([mean((Y_test(4,:)-Y_pred_mlp(4,:)).^2), mean((Y_test(4,:)-Y_pred_xgb(4,:)).^2), mean((Y_test(4,:)-Y_pred_rf(4,:)).^2), mean((Y_test(4,:)-Y_pred_eff_spec).^2)]);
    
    % 1. 무거운 모델들을 워커간 공유 가능한 Constant 메모리로 변환
    const_net = parallel.pool.Constant(net);
    const_xgb = parallel.pool.Constant(xgb_models);
    const_rf  = parallel.pool.Constant(rf_models);
    const_eff = parallel.pool.Constant(eff_specialist);

    % 2. 익명 함수 내부에서 .Value 로 접근하도록 수정
    bic_predict = @(X_in) predict_best_in_class(X_in, best_p, best_p_hv, best_i_pk, best_eff, ...
        const_net.Value, PS_X, PS_Y, const_xgb.Value, const_rf.Value, const_eff.Value, ...
        eff_spec_type, PS_X_eff, PS_Y_eff_s);
    disp(' -> Best-in-Class 메타 모델 조립 완료.');
    
    disp(' ');
    disp(' -> [3단계] 10만 개의 대규모 연속 데이터셋 증강(Augmentation)을 시작합니다.');
    
    num_massive = 100000;
    rand_P = (rand(1, num_massive) * 2400) - 1200; 
    rand_P((rand_P > -40) & (rand_P < 40)) = sign(rand_P((rand_P > -40) & (rand_P < 40)) + 1e-5) * 40;
    rand_V2 = (rand(1, num_massive) * 30) + 80;
    
    X_massive = [rand_P; rand_V2];
    X_mas_norm = mapminmax('apply', X_massive, PS_X_med);
    Y_mas_norm = phase_surrogate_net(X_mas_norm);
    Y_massive_pred = mapminmax('reverse', Y_mas_norm, PS_Y_med);
    
    Y_massive_pred(1, :) = max(-90, min(90, Y_massive_pred(1, :))); % D0
    Y_massive_pred(2, :) = max(0, min(0.5, Y_massive_pred(2, :)));  % D1
    Y_massive_pred(3, :) = max(0, min(0.5, Y_massive_pred(3, :)));  % D2
    
    disp(' -> 증강된 대규모 데이터 물리 모델 피드백 검증 중...');
    X_massive_in = [rand_V2', Y_massive_pred(1,:)', Y_massive_pred(2,:)', Y_massive_pred(3,:)', Y_massive_pred(1,:)'.^2, Y_massive_pred(2,:)'.*Y_massive_pred(3,:)', rand_V2'.*Y_massive_pred(1,:)'];
    
    massive_preds = zeros(4, num_massive);
    batch_size = 10000;
    for bs = 1:batch_size:num_massive
        be = min(bs + batch_size - 1, num_massive);
        massive_preds(:, bs:be) = bic_predict(X_massive_in(bs:be, :));
    end
    
    massive_dataset = zeros(num_massive, 8);
    massive_dataset(:, 1) = X_massive(1, :)';
    massive_dataset(:, 2) = X_massive(2, :)';
    massive_dataset(:, 3) = Y_massive_pred(1, :)';
    massive_dataset(:, 4) = Y_massive_pred(2, :)';
    massive_dataset(:, 5) = Y_massive_pred(3, :)';
    massive_dataset(:, 6) = massive_preds(1, :)'; % Predicted P_act
    massive_dataset(:, 7) = massive_preds(3, :)'; % Min I_pk
    massive_dataset(:, 8) = massive_preds(4, :)'; % Expected Efficiency
    
    disp(' -> [성공] 10만 건 대규모 연속 증강 데이터셋 저장 중...');
    save_final_csv(massive_dataset, 'Massive_Optimized_TPS_Dataset_BestInClass_V6.csv');
    
    disp(' ');
    disp(' -> [자동 연계] 생성된 증강 데이터셋에 대한 3D 시각화를 자동 실행합니다...');
    run_3D_Plot_Phases('Massive_Optimized_TPS_Dataset_BestInClass_V6.csv');
end

%% =========================================================================
% [함수 10] 클러스터 기반 HDC 강화학습 V6 (연속 가중치 튜닝 후 양자화 적용)
% =========================================================================
function run_HDC_Reinforcement_Learning(X_test, Y_test)
    disp('-----------------------------------------------------------');
    disp('[Method 10] V6 클러스터 기반 HDC 강화학습 (Multi-Model RL)');
    disp(' -> 7번 메뉴에서 만든 여러 모드의 연속(Continuous) 가중치를 RL로');
    disp(' -> 미세 조정한 뒤, 마지막에 다시 이진화하여 성능을 극대화합니다.');
    
    hdc_file = 'trained_hdc_controller_model_v6.mat';
    mlp_file = 'trained_fast_mlp_net_v6.mat';
    xgb_file = 'trained_xgboost_models_v6.mat';
    rf_file  = 'trained_rf_models_v6.mat';
    eff_spec_file = 'trained_eff_specialist_v6.mat';
    
    if ~isfile(hdc_file) || ~isfile(mlp_file) || ~isfile(xgb_file) || ~isfile(rf_file) || ~isfile(eff_spec_file)
        disp(' -> [오류] 1, 2, 3, 7, 8번 모델이 모두 필요합니다. 먼저 실행하세요.');
        return;
    end
    
    load(hdc_file, 'hdc_model');
    disp(' -> [가상 물리 환경 구축] Best-in-Class 메타 앙상블 시스템을 조립 중입니다...');
    load(mlp_file, 'net', 'PS_X', 'PS_Y');
    load(xgb_file, 'xgb_models');
    load(rf_file, 'rf_models');
    load(eff_spec_file, 'eff_specialist', 'eff_spec_type', 'PS_X_eff', 'PS_Y_eff_s');
    
    % Best-in-Class 모델 선별
    X_test_norm = mapminmax('apply', X_test, PS_X);
    Y_pred_mlp = mapminmax('reverse', net(X_test_norm), PS_Y);
    Y_pred_xgb = zeros(size(Y_test));
    Y_pred_xgb(1,:) = sign(predict(xgb_models.P, X_test')') .* (exp(abs(predict(xgb_models.P, X_test')')) - 1);
    Y_pred_xgb(2,:) = sign(predict(xgb_models.P_HV, X_test')') .* (exp(abs(predict(xgb_models.P_HV, X_test')')) - 1);
    Y_pred_xgb(3,:) = sign(predict(xgb_models.I_pk, X_test')') .* (exp(abs(predict(xgb_models.I_pk, X_test')')) - 1);
    Y_pred_xgb(4,:) = predict(xgb_models.Eff_UQ, X_test')';
    Y_pred_rf = zeros(size(Y_test));
    Y_pred_rf(1,:) = predict(rf_models.P, X_test')';
    Y_pred_rf(2,:) = predict(rf_models.P_HV, X_test')';
    Y_pred_rf(3,:) = predict(rf_models.I_pk, X_test')';
    Y_pred_rf(4,:) = predict(rf_models.Eff, X_test')';
    
    [~, best_p] = min([mean((Y_test(1,:)-Y_pred_mlp(1,:)).^2), mean((Y_test(1,:)-Y_pred_xgb(1,:)).^2), mean((Y_test(1,:)-Y_pred_rf(1,:)).^2)]);
    [~, best_p_hv] = min([mean((Y_test(2,:)-Y_pred_mlp(2,:)).^2), mean((Y_test(2,:)-Y_pred_xgb(2,:)).^2), mean((Y_test(2,:)-Y_pred_rf(2,:)).^2)]);
    [~, best_i_pk] = min([mean((Y_test(3,:)-Y_pred_mlp(3,:)).^2), mean((Y_test(3,:)-Y_pred_xgb(3,:)).^2), mean((Y_test(3,:)-Y_pred_rf(3,:)).^2)]);
    if strcmp(eff_spec_type, 'NN')
        X_te_eff_n = mapminmax('apply', X_test, PS_X_eff);
        Y_pred_eff_spec = mapminmax('reverse', eff_specialist(X_te_eff_n), PS_Y_eff_s);
    else
        Y_pred_eff_spec = predict(eff_specialist, X_test')';
    end
    [~, best_eff] = min([mean((Y_test(4,:)-Y_pred_mlp(4,:)).^2), mean((Y_test(4,:)-Y_pred_xgb(4,:)).^2), mean((Y_test(4,:)-Y_pred_rf(4,:)).^2), mean((Y_test(4,:)-Y_pred_eff_spec).^2)]);
    
    bic_predict = @(X_in) predict_best_in_class(X_in, best_p, best_p_hv, best_i_pk, best_eff, net, PS_X, PS_Y, xgb_models, rf_models, eff_specialist, eff_spec_type, PS_X_eff, PS_Y_eff_s);
    disp(' -> Best-in-Class 가상 물리 환경(Virtual Physics Environment) 준비 완료.');

    % 다중 클러스터 모델 파라미터 로드
    C_clusters = hdc_model.C_clusters;
    W_continuous = hdc_model.W_continuous;
    num_clusters = hdc_model.num_clusters;
    ID_vecs = hdc_model.ID_vecs; L_vecs = hdc_model.L_vecs;
    min_X = hdc_model.min_X; max_X = hdc_model.max_X;
    Q = hdc_model.Q; D = hdc_model.D;
    PS_Y_opt = hdc_model.PS_Y_opt;
    mode = hdc_model.mode;
    
    % 사용자에게 튜닝할 클러스터 선택 받기
    disp([' -> 현재 HDC 모델은 ', num2str(num_clusters), '개의 클러스터 모드를 가지고 있습니다.']);
    target_cluster = input(['튜닝할 클러스터 번호를 선택하세요 (1 ~ ', num2str(num_clusters), '): ']);
    
    if isempty(target_cluster) || target_cluster < 1 || target_cluster > num_clusters
        disp(' -> 잘못된 입력입니다. 1번 클러스터를 기본으로 튜닝합니다.');
        target_cluster = 1;
    end
    
    % 타겟 클러스터의 연속형 가중치 추출 (RL은 연속 공간에서 이루어짐)
    W_full = W_continuous{target_cluster}; 
    W_D0_frozen = W_full(1, :);    % D0 행은 동결
    W = W_full(2:3, :);            % D1, D2 행만 RL 대상
    
    PS_Y_D1D2 = PS_Y_opt;
    PS_Y_D1D2.xrows = 2;
    nc_full = PS_Y_opt.no_change;
    nc_d1d2 = nc_full(nc_full >= 2) - 1;
    PS_Y_D1D2.no_change = nc_d1d2;
    PS_Y_D1D2.xoffset = PS_Y_opt.xoffset(2:3);
    PS_Y_D1D2.gain = PS_Y_opt.gain(2:3);
    
    max_epochs = 10000; % 클러스터별 튜닝이므로 Epoch를 짧게 설정
    batch_size = 500;
    
    initial_lr = 0.01;
    min_lr = 0.0001;
    initial_noise = 0.05;
    min_noise = 0.005;
    decay_rate = 0.9995;
    weight_decay = 1e-5;
    
    cost_target = 3.0;           
    improvement_threshold = 0.005;
    patience = 200;              
    checkpoint_interval = 200;
    
    disp([' -> 클러스터 #', num2str(target_cluster), ' 강화학습을 시작합니다... (Soft Penalty 적용)']);
    
    % ---------------------------------------------------------------------
    % RL 메인 루프 (연속 가중치 W 업데이트)
    % ---------------------------------------------------------------------
    stagnation_count = 0; 
    best_cost = Inf;
    W_best = W;
    
    tic;
    for ep = 1:max_epochs
        % 1. 무작위 목표 전력/전압 샘플링
        rand_P = (rand(1, batch_size) * 2400) - 1200;
        rand_P((rand_P > -40) & (rand_P < 40)) = sign(rand_P((rand_P > -40) & (rand_P < 40)) + 1e-5) * 40; 
        rand_V2 = (rand(1, batch_size) * 30) + 80;
        X_state = [rand_P; rand_V2];
        
        HV_state = encode_HDC_data_compete(X_state, min_X, max_X, Q, D, ID_vecs, L_vecs, mode);
        
        % 연속형 W를 이용한 예측
        Y_base_norm = W * HV_state;  
        
        current_lr = max(min_lr, initial_lr * (decay_rate^ep));
        current_noise = max(min_noise, initial_noise * (decay_rate^ep));
        
        noise = randn(2, batch_size) * current_noise;
        Y_exp_norm_raw = Y_base_norm + noise;
        
        Y_base_clipped = max(-1, min(1, Y_base_norm));
        Y_exp_clipped  = max(-1, min(1, Y_exp_norm_raw));
        Y_base = mapminmax('reverse', Y_base_clipped, PS_Y_D1D2);
        Y_exp  = mapminmax('reverse', Y_exp_clipped, PS_Y_D1D2);
        
        D1_base = max(0, min(0.5, Y_base(1,:)));
        D2_base = max(0, min(0.5, Y_base(2,:)));
        D1_exp  = max(0, min(0.5, Y_exp(1,:)));
        D2_exp  = max(0, min(0.5, Y_exp(2,:)));
        
        % D0 도출 (Vectorized)
        D0_base = find_matching_D0_vectorized(rand_P, rand_V2, D1_base, D2_base, bic_predict);
        D0_exp  = find_matching_D0_vectorized(rand_P, rand_V2, D1_exp, D2_exp, bic_predict);
        
        X_in_base = [rand_V2', D0_base', D1_base', D2_base', D0_base'.^2, D1_base'.*D2_base', rand_V2'.*D0_base'];
        phys_base = bic_predict(X_in_base);
        X_in_exp = [rand_V2', D0_exp', D1_exp', D2_exp', D0_exp'.^2, D1_exp'.*D2_exp', rand_V2'.*D0_exp'];
        phys_exp = bic_predict(X_in_exp);
        
        p_error_base = abs(phys_base(1,:) - rand_P);
        p_error_exp  = abs(phys_exp(1,:) - rand_P);
        tolerance = max(15, abs(rand_P) * 0.03); 
        
        penalty_base = max(0, p_error_base - tolerance) .* 50;
        penalty_exp  = max(0, p_error_exp  - tolerance) .* 50;
        
        cost_base = phys_base(3,:) + penalty_base;
        cost_exp  = phys_exp(3,:)  + penalty_exp;
        
        advantage = cost_base - cost_exp;
        better_mask = advantage > 0;
        num_better = sum(better_mask);
        
        if num_better > 0
            adv_vals = advantage(better_mask);
            adv_weights = adv_vals / (mean(adv_vals) + 1e-5);
            effective_noise = Y_exp_clipped(:, better_mask) - Y_base_clipped(:, better_mask);
            weighted_noise = effective_noise .* adv_weights;
            HV_better = HV_state(:, better_mask);
            dW = (weighted_noise * HV_better') / D;
            W = W * (1 - weight_decay) + current_lr * dW;
        end
        
        val_cost = mean(cost_base);
        if val_cost < (best_cost - improvement_threshold)
            best_cost = val_cost;
            W_best = W;
            stagnation_count = 0;
        else
            stagnation_count = stagnation_count + 1;
        end
        
        if mod(ep, 50) == 0
            fprintf(' [Epoch %d] Val Cost: %.3f (Best: %.3f) | LR: %.5f | 정체: %d/%d\n', ...
                    ep, val_cost, best_cost, current_lr, stagnation_count, patience);
        end
        
        if val_cost <= cost_target
            fprintf('\n -> [목표 달성] Val Cost %.3f 에 도달하여 조기 종료합니다.\n', val_cost);
            break;
        end
        if stagnation_count >= patience
            fprintf('\n -> [수렴 완료] %d 회 이상 개선이 없어 RL을 종료합니다.\n', patience);
            break;
        end
    end
    
    elapsed = toc;
    disp('===========================================================');
    fprintf(' -> 클러스터 #%d RL 튜닝 완료! 소요 시간: %.2f초\n', target_cluster, elapsed);
    
    % 튜닝된 최고 성능의 연속형 가중치를 저장
    W_tuned_continuous = [W_D0_frozen; W_best];
    hdc_model.W_continuous{target_cluster} = W_tuned_continuous;
    
    % 이진화(Binarization) 재적용
    W_bin = sign(W_tuned_continuous);
    W_bin(W_bin == 0) = 1;
    hdc_model.W_clusters{target_cluster} = W_bin;
    
    save(hdc_file, 'hdc_model');
    disp(' -> [성공] 튜닝된 연속 가중치를 이진화(Binarize)하여 최종 클러스터 모델 업데이트를 완료했습니다.');
end

function D0_opt = find_matching_D0_vectorized(P_target, V2, D1, D2, bic_predict)
    N = length(P_target);
    
    % [Stage 1] Coarse Search
    D0_cands_c = -90:5:90; 
    M1 = length(D0_cands_c);

    V2_rep_c = repmat(V2(:)', M1, 1);
    D1_rep_c = repmat(D1(:)', M1, 1);
    D2_rep_c = repmat(D2(:)', M1, 1);
    D0_rep_c = repmat(D0_cands_c(:), 1, N);

    X_in_c = [V2_rep_c(:), D0_rep_c(:), D1_rep_c(:), D2_rep_c(:), D0_rep_c(:).^2, D1_rep_c(:).*D2_rep_c(:), V2_rep_c(:).*D0_rep_c(:)];
    
    preds_c = bic_predict(X_in_c);
    P_preds_c = reshape(preds_c(1,:), M1, N);
    
    P_targets_rep_c = repmat(P_target(:)', M1, 1);
    [~, min_idx_c] = min(abs(P_preds_c - P_targets_rep_c));
    
    best_coarse = D0_cands_c(min_idx_c); 

    % [Stage 2] Fine Search
    offsets = (-2.5:0.1:2.5)'; 
    M2 = length(offsets);
    
    D0_rep_f = repmat(best_coarse, M2, 1) + repmat(offsets, 1, N); 
    D0_rep_f = max(-90, min(90, D0_rep_f));

    V2_rep_f = repmat(V2(:)', M2, 1);
    D1_rep_f = repmat(D1(:)', M2, 1);
    D2_rep_f = repmat(D2(:)', M2, 1);

    X_in_f = [V2_rep_f(:), D0_rep_f(:), D1_rep_f(:), D2_rep_f(:), D0_rep_f(:).^2, D1_rep_f(:).*D2_rep_f(:), V2_rep_f(:).*D0_rep_f(:)];
    
    preds_f = bic_predict(X_in_f);
    P_preds_f = reshape(preds_f(1,:), M2, N);
    
    P_targets_rep_f = repmat(P_target(:)', M2, 1);
    [~, min_idx_f] = min(abs(P_preds_f - P_targets_rep_f));
    
    linear_indices = sub2ind([M2, N], min_idx_f, 1:N);
    D0_opt = D0_rep_f(linear_indices);
end