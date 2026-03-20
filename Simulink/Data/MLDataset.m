% =========================================================================
% TPS 데이터셋 기반 다중 머신러닝 모델 학습 및 최적 모델 자동 채택 스크립트
% 포함된 모델: ANN, Random Forest, SVM, HDC, Deep BPNN, Gradient Boosting
% 특징: 병렬 컴퓨팅(Parallel Computing) 적용, 중간 저장 기능 포함
% =========================================================================

clc; clear; close all;

%% 0. 병렬 풀(Parallel Pool) 초기화
fprintf('병렬 컴퓨팅 환경을 설정하는 중입니다...\n');
poolobj = gcp('nocreate'); % 현재 실행 중인 병렬 풀 확인
if isempty(poolobj)
    fprintf('[안내] 새로운 병렬 풀(Parallel Pool)을 시작합니다. (최초 실행 시 시간이 다소 소요될 수 있습니다.)\n');
    parpool; % 병렬 워커(Worker) 실행
else
    fprintf('[안내] 이미 실행 중인 병렬 풀을 사용합니다. (워커 수: %d)\n', poolobj.NumWorkers);
end
fprintf('\n');

%% 1. 데이터 로드 및 전처리
filename = 'TPS_Dataset_20260313_090038.csv';
fprintf('데이터를 불러오는 중: %s...\n', filename);
data = readtable(filename);

% 결측치(NaN) 제거
data = rmmissing(data);

% 입력 변수 (X): 내부 위상(D1, D2), 전압(V2_V), 목표 전력(P_act_W)
X = [data.D1, data.D2, data.V2_V, data.P_act_W]';

% 출력 변수 (Y): 1차 인덕터 전류, 외부 위상(D0_deg) (2차 인덕터 전류는 제외)
Y = [data.i_pk_pri_A, data.D0_deg]';

N_samples = size(X, 2);
fprintf('총 %d개의 샘플 데이터가 준비되었습니다.\n\n', N_samples);

%% 2. 데이터 분할 (Train / Test)
rng(42); % 재현성을 위한 난수 시드 고정
cv = cvpartition(N_samples, 'HoldOut', 0.2);
idx_train = training(cv);
idx_test = test(cv);

X_train = X(:, idx_train);
Y_train = Y(:, idx_train);
X_test  = X(:, idx_test);
Y_test  = Y(:, idx_test);

% 머신러닝 모델들을 위한 전치(Transpose) 데이터 (N x Features)
X_train_T = X_train';
X_test_T  = X_test';

%% 3. 다중 머신러닝 모델 학습 (병렬 연산 적용)

% 중간 저장을 위한 파일명 설정
checkpoint_file = 'ml_models_checkpoint.mat';
fprintf('[안내] 학습 과정이 ''%s'' 파일에 중간 저장됩니다.\n\n', checkpoint_file);
save(checkpoint_file, 'X_train', 'Y_train', 'X_test', 'Y_test'); 

% 성능 저장을 위한 구조체 초기화
models_info = struct('name', {}, 'rmse', {});
num_targets = size(Y, 1);

% -------------------------------------------------------------------------
% [모델 1] 인공신경망 (ANN) - 데이터 병렬 학습
fprintf('--- [1/6] 인공신경망(ANN) 학습 시작 (병렬 연산) ---\n');
tic; 
net_ANN = fitnet([20 20], 'trainlm');
net_ANN.trainParam.showWindow = false; 
% 'useParallel', 'yes' 옵션을 통해 데이터 분산 병렬 학습 수행
net_ANN = train(net_ANN, X_train, Y_train, 'useParallel', 'yes');

Y_pred_ANN = net_ANN(X_test);
rmse_ANN = sqrt(mean((Y_test - Y_pred_ANN).^2, 2)); 
total_rmse_ANN = sum(rmse_ANN);
models_info(end+1) = struct('name', 'ANN', 'rmse', total_rmse_ANN);

t_elapsed = toc; 
fprintf('  -> ANN 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_ANN);
save(checkpoint_file, 'net_ANN', 'models_info', '-append'); 

% -------------------------------------------------------------------------
% [모델 2] 랜덤 포레스트 (Random Forest) - 타겟별 parfor 병렬 학습
fprintf('\n--- [2/6] 랜덤 포레스트(Random Forest) 학습 시작 (parfor 병렬) ---\n');
tic;
mdl_RF = cell(num_targets, 1);
Y_pred_RF = zeros(num_targets, size(Y_test, 2));

% parfor를 이용해 여러 타겟(전류, 위상) 모델을 다중 코어에서 동시 학습
parfor i = 1:num_targets
    fprintf('  - 타겟 변수 %d 학습 진행 중... (Worker 분산)\n', i);
    y_tr = Y_train(i,:)'; % parfor 슬라이싱을 위한 임시 변수
    mdl_RF{i} = fitrensemble(X_train_T, y_tr, 'Method', 'Bag');
    Y_pred_RF(i,:) = predict(mdl_RF{i}, X_test_T)';
end
rmse_RF = sqrt(mean((Y_test - Y_pred_RF).^2, 2));
total_rmse_RF = sum(rmse_RF);
models_info(end+1) = struct('name', 'Random Forest', 'rmse', total_rmse_RF);

t_elapsed = toc;
fprintf('  -> Random Forest 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_RF);
save(checkpoint_file, 'mdl_RF', 'models_info', '-append'); 

% -------------------------------------------------------------------------
% [모델 3] 서포트 벡터 머신 (SVM) - 타겟별 parfor 병렬 학습
fprintf('\n--- [3/6] 서포트 벡터 머신(SVM) 학습 시작 (parfor 병렬) ---\n');
tic;
mdl_SVM = cell(num_targets, 1);
Y_pred_SVM = zeros(num_targets, size(Y_test, 2));

parfor i = 1:num_targets
    fprintf('  - 타겟 변수 %d 학습 진행 중... (Worker 분산)\n', i);
    y_tr = Y_train(i,:)';
    mdl_SVM{i} = fitrsvm(X_train_T, y_tr, 'Standardize', true, 'KernelFunction', 'gaussian');
    Y_pred_SVM(i,:) = predict(mdl_SVM{i}, X_test_T)';
end
rmse_SVM = sqrt(mean((Y_test - Y_pred_SVM).^2, 2));
total_rmse_SVM = sum(rmse_SVM);
models_info(end+1) = struct('name', 'SVM', 'rmse', total_rmse_SVM);

t_elapsed = toc;
fprintf('  -> SVM 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_SVM);
save(checkpoint_file, 'mdl_SVM', 'models_info', '-append'); 

% -------------------------------------------------------------------------
% [모델 4] 초고차원 컴퓨팅 (HDC) - 내장 행렬 가속
fprintf('\n--- [4/6] HDC(Hyperdimensional Computing) 학습 시작 ---\n');
tic;
D_dim = 5000; 
W_hdc = randn(D_dim, size(X_train, 1));
b_hdc = randn(D_dim, 1) * 2 * pi;

fprintf('  - 고차원 매핑 및 가중치 계산 중... (행렬 연산 최적화)\n');
H_train = cos(W_hdc * X_train + b_hdc);

lambda_hdc = 1e-4;
beta_hdc = Y_train * H_train' * pinv(H_train * H_train' + lambda_hdc * eye(D_dim));

H_test = cos(W_hdc * X_test + b_hdc);
Y_pred_HDC = beta_hdc * H_test;

rmse_HDC = sqrt(mean((Y_test - Y_pred_HDC).^2, 2));
total_rmse_HDC = sum(rmse_HDC);
models_info(end+1) = struct('name', 'HDC', 'rmse', total_rmse_HDC);

t_elapsed = toc;
fprintf('  -> HDC 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_HDC);
save(checkpoint_file, 'W_hdc', 'b_hdc', 'beta_hdc', 'models_info', '-append'); 

% -------------------------------------------------------------------------
% [모델 5] 논문 기반 심층 역전파 신경망 (Deep BPNN) - 데이터 병렬 학습
fprintf('\n--- [5/6] 논문 기반 심층 역전파 신경망(Deep BPNN) 학습 시작 (병렬 연산) ---\n');
tic;
net_BPNN = fitnet([32 16 8], 'trainlm'); 
net_BPNN.trainParam.showWindow = false; 
net_BPNN.trainParam.epochs = 1000; 
net_BPNN.trainParam.goal = 1e-6;   
net_BPNN = train(net_BPNN, X_train, Y_train, 'useParallel', 'yes'); % 병렬화

Y_pred_BPNN = net_BPNN(X_test);
rmse_BPNN = sqrt(mean((Y_test - Y_pred_BPNN).^2, 2));
total_rmse_BPNN = sum(rmse_BPNN);
models_info(end+1) = struct('name', 'Deep BPNN', 'rmse', total_rmse_BPNN);

t_elapsed = toc;
fprintf('  -> Deep BPNN 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_BPNN);
save(checkpoint_file, 'net_BPNN', 'models_info', '-append'); 

% -------------------------------------------------------------------------
% [모델 6] 그래디언트 부스팅 회귀 (Gradient Boosting) - 타겟별 parfor 병렬 학습
fprintf('\n--- [6/6] 그래디언트 부스팅(Gradient Boosting) 학습 시작 (parfor 병렬) ---\n');
tic;
mdl_GBR = cell(num_targets, 1);
Y_pred_GBR = zeros(num_targets, size(Y_test, 2));

parfor i = 1:num_targets
    fprintf('  - 타겟 변수 %d 학습 진행 중... (Worker 분산)\n', i);
    y_tr = Y_train(i,:)';
    mdl_GBR{i} = fitrensemble(X_train_T, y_tr, 'Method', 'LSBoost', 'NumLearningCycles', 100);
    Y_pred_GBR(i,:) = predict(mdl_GBR{i}, X_test_T)';
end
rmse_GBR = sqrt(mean((Y_test - Y_pred_GBR).^2, 2));
total_rmse_GBR = sum(rmse_GBR);
models_info(end+1) = struct('name', 'Gradient Boosting', 'rmse', total_rmse_GBR);

t_elapsed = toc;
fprintf('  -> Gradient Boosting 학습 완료 (소요 시간: %.2f초, 총 RMSE: %.4f)\n', t_elapsed, total_rmse_GBR);
save(checkpoint_file, 'mdl_GBR', 'models_info', '-append'); 

%% 4. 성능 비교 및 최적의 모델 채택
fprintf('\n======================================================\n');
fprintf('모델 학습 및 평가 완료\n');
fprintf('======================================================\n');

% 모델 비교 시각화
model_names = {models_info.name};
rmse_values = [models_info.rmse];

figure;
bar(rmse_values, 'FaceColor', [0.2 0.6 0.8]);
set(gca, 'xticklabel', model_names);
ylabel('총 RMSE (낮을수록 좋음)');
title('머신러닝 모델 성능 비교 (Test 데이터 기준)');
grid on;

% 가장 RMSE가 낮은 모델 찾기
[best_rmse, best_idx] = min(rmse_values);
best_model_name = models_info(best_idx).name;

fprintf('비교 결과, 가장 성능이 우수한 모델은 [%s] 입니다. (RMSE: %.4f)\n', best_model_name, best_rmse);
fprintf('이후의 시뮬레이션 및 예측은 [%s] 모델을 채택하여 진행합니다.\n', best_model_name);

%% 5. 채택된 최적 모델의 회귀 분석 플롯
switch best_model_name
    case 'ANN'
        Y_best_pred = Y_pred_ANN;
    case 'Random Forest'
        Y_best_pred = Y_pred_RF;
    case 'SVM'
        Y_best_pred = Y_pred_SVM;
    case 'HDC'
        Y_best_pred = Y_pred_HDC;
    case 'Deep BPNN'
        Y_best_pred = Y_pred_BPNN;
    case 'Gradient Boosting'
        Y_best_pred = Y_pred_GBR;
end

figure;
plotregression(Y_test(:), Y_best_pred(:), sprintf('[%s] Test Data Regression', best_model_name));
title(sprintf('최적 채택 모델: %s 의 실제값과 예측값 상관관계', best_model_name));

%% 6. 연속적인 결과 예측 테스트 (시뮬레이터 활용 예시)
fprintf('\n=== [%s] 모델을 활용한 새로운 값 연속 예측 테스트 ===\n', best_model_name);
test_D1 = 0.3;
test_D2 = 0.3;
test_V2 = 84;
test_P_act = linspace(-150, -50, 100); 

X_new = [repmat(test_D1, 1, 100); 
         repmat(test_D2, 1, 100); 
         repmat(test_V2, 1, 100); 
         test_P_act];

switch best_model_name
    case 'ANN'
        Y_new_pred = net_ANN(X_new);
        
    case 'Random Forest'
        Y_new_pred = zeros(num_targets, size(X_new, 2));
        for i = 1:num_targets
            Y_new_pred(i,:) = predict(mdl_RF{i}, X_new')';
        end
        
    case 'SVM'
        Y_new_pred = zeros(num_targets, size(X_new, 2));
        for i = 1:num_targets
            Y_new_pred(i,:) = predict(mdl_SVM{i}, X_new')';
        end
        
    case 'HDC'
        H_new = cos(W_hdc * X_new + b_hdc);
        Y_new_pred = beta_hdc * H_new;
        
    case 'Deep BPNN'
        Y_new_pred = net_BPNN(X_new);
        
    case 'Gradient Boosting'
        Y_new_pred = zeros(num_targets, size(X_new, 2));
        for i = 1:num_targets
            Y_new_pred(i,:) = predict(mdl_GBR{i}, X_new')';
        end
end

figure;
subplot(2,1,1);
plot(test_P_act, Y_new_pred(1,:), 'b-', 'LineWidth', 2);
title(sprintf('[%s] 목표 전력(P_{act})에 따른 1차 인덕터 전류 예측', best_model_name));
xlabel('P_{act} [W]'); ylabel('i_{pk,pri} [A]');
grid on;

subplot(2,1,2);
plot(test_P_act, Y_new_pred(2,:), 'k-', 'LineWidth', 2);
title('목표 전력(P_{act})에 따른 외부 위상(D0) 예측');
xlabel('P_{act} [W]'); ylabel('D0 [deg]');
grid on;

fprintf('연속 예측 테스트 시각화가 완료되었습니다.\n');