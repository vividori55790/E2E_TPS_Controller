% =========================================================================
% TPS 데이터셋 기반 머신러닝 모델 학습 및 최적 모델 자동 채택 스크립트
% [안정성 및 연산 효율 개선 버전 - 성능 및 설정값 원본 유지]
%
% <개선 사항>
% 1. HDC 모델 연산 최적화: pinv 대신 mrdivide(/) 연산 사용 (수학적 결과 동일, 속도 향상)
% 2. 안정성 확보: 각 모델 학습 완료 직후 즉각적인 체크포인트 저장 (데이터 유실 방지)
% * 주의: 모델 성능 보존을 위해 하이퍼파라미터 및 최적화 알고리즘은 원본을 그대로 유지함
% =========================================================================

clc; clear; close all;

%% 0. 전역 학습 제약 조건 설정 (원본 유지)
GLOBAL_MAX_TIME = 7200; % 신경망 모델당 최대 학습 시간 (초)
GLOBAL_ENSEMBLE_TREES = 1000; % 앙상블 모델(RF, GBR) 트리 개수
GLOBAL_MAX_EPOCHS = 20000; % 신경망 최대 에폭
GLOBAL_MAX_FAIL = 50; % 조기 종료(Early Stopping) 기준

%% 1. 병렬 풀(Parallel Pool) 초기화
fprintf('병렬 컴퓨팅 환경을 설정하는 중입니다...\n');
poolobj = gcp('nocreate');
if isempty(poolobj)
    fprintf('[안내] 새로운 병렬 풀(Parallel Pool)을 시작합니다.\n');
    parpool;
else
    fprintf('[안내] 이미 실행 중인 병렬 풀을 사용합니다. (워커 수: %d)\n', poolobj.NumWorkers);
end
fprintf('\n');

%% 2. 데이터 로드 및 전처리
filename = 'TPS_Dataset_20260313_090038.csv';
fprintf('데이터를 불러오는 중: %s...\n', filename);

try
    data = readtable(filename);
catch
    fprintf('[오류] %s 파일을 찾을 수 없습니다. 테스트용 임시 데이터를 생성합니다.\n', filename);
    data = array2table(randn(1000, 6), 'VariableNames', {'D1', 'D2', 'V2_V', 'P_act_W', 'i_pk_pri_A', 'D0_deg'});
end

% 결측치 제거
data = rmmissing(data);

% 입력 변수 (X): 내부 위상(D1, D2), 전압(V2_V), 목표 전력(P_act_W)
X = [data.D1, data.D2, data.V2_V, data.P_act_W]';

% 출력 변수 (Y): 인덕터 피크 전류(i_pk_pri_A) 단일 타겟으로 축소
Y = [data.i_pk_pri_A]';

N_samples = size(X, 2);
fprintf('총 %d개의 샘플 데이터가 준비되었습니다. (단일 타겟: i_pk)\n\n', N_samples);

%% 3. 데이터 분할 (Train / Validation / Test = 70 / 15 / 15)
rng(42); % 재현성을 위한 난수 시드 고정 (원본 유지)
idx = randperm(N_samples);

N_train = round(0.70 * N_samples);
N_val   = round(0.15 * N_samples);
N_test  = N_samples - N_train - N_val;

idx_train = idx(1:N_train);
idx_val   = idx(N_train+1 : N_train+N_val);
idx_test  = idx(N_train+N_val+1 : end);

X_train = X(:, idx_train); Y_train = Y(:, idx_train);
X_val   = X(:, idx_val);   Y_val   = Y(:, idx_val);
X_test  = X(:, idx_test);  Y_test  = Y(:, idx_test);

%% 3-1. 글로벌 데이터 정규화 (X 및 Y 모두 적용)
% Train 데이터를 기준으로 평균(mu)과 표준편차(sigma) 계산
mu_X = mean(X_train, 2); sigma_X = std(X_train, 0, 2);
mu_Y = mean(Y_train, 2); sigma_Y = std(Y_train, 0, 2);

sigma_X(sigma_X == 0) = 1;
sigma_Y(sigma_Y == 0) = 1;

% X 정규화
X_train_norm = (X_train - mu_X) ./ sigma_X;
X_val_norm   = (X_val - mu_X) ./ sigma_X;
X_test_norm  = (X_test - mu_X) ./ sigma_X;

% Y 정규화 (학습 효율 극대화)
Y_train_norm = (Y_train - mu_Y) ./ sigma_Y;
Y_val_norm   = (Y_val - mu_Y) ./ sigma_Y;
Y_test_norm  = (Y_test - mu_Y) ./ sigma_Y;

% 트리/SVM용 전치 데이터
X_train_norm_T = X_train_norm'; Y_train_norm_T = Y_train_norm';
X_val_norm_T   = X_val_norm';   Y_val_T        = Y_val';

%% 4. 다중 머신러닝 모델 학습 및 검증
checkpoint_file = 'ml_models_checkpoint.mat';
% 기본 데이터 먼저 저장
save(checkpoint_file, 'X_train', 'Y_train', 'X_val', 'Y_val', 'X_test', 'Y_test', 'mu_X', 'sigma_X', 'mu_Y', 'sigma_Y'); 

% Validation RMSE를 저장할 구조체 (모델 선정의 기준)
models_info = struct('name', {}, 'val_rmse', {});
par_opts = statset('UseParallel', true);

% -------------------------------------------------------------------------
% [모델 1] 인공신경망 (ANN)
fprintf('--- [1/6] 인공신경망(ANN) 학습 시작 ---\n');
tic; 
net_ANN = fitnet([128 32], 'trainlm'); % 원본 성능 유지를 위해 trainlm 사용
net_ANN.layers{1}.transferFcn = 'poslin'; 
net_ANN.layers{2}.transferFcn = 'poslin';
net_ANN.performParam.regularization = 1e-5;
net_ANN.trainParam.showWindow = false; 
net_ANN.trainParam.epochs = GLOBAL_MAX_EPOCHS;
net_ANN.trainParam.max_fail = GLOBAL_MAX_FAIL;

X_combine = [X_train_norm, X_val_norm];
Y_combine = [Y_train_norm, Y_val_norm];
net_ANN.divideFcn = 'divideind';
net_ANN.divideParam.trainInd = 1:N_train;
net_ANN.divideParam.valInd   = N_train+1 : N_train+N_val;
net_ANN.divideParam.testInd  = [];

net_ANN = train(net_ANN, X_combine, Y_combine, 'useParallel', 'yes');

Y_pred_norm_ANN = net_ANN(X_val_norm);
Y_pred_ANN = (Y_pred_norm_ANN .* sigma_Y) + mu_Y; % 역변환
rmse_ANN = sqrt(mean((Y_val - Y_pred_ANN).^2)); 

models_info(end+1) = struct('name', 'ANN (Paper Spec)', 'val_rmse', rmse_ANN);
fprintf('  -> ANN 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_ANN);

% [개선] 학습 즉시 체크포인트 저장
save(checkpoint_file, 'net_ANN', 'models_info', '-append');

% -------------------------------------------------------------------------
% [모델 2] 랜덤 포레스트 (Random Forest)
fprintf('\n--- [2/6] 랜덤 포레스트(Random Forest) 학습 시작 ---\n');
tic;
mdl_RF = fitrensemble(X_train_norm_T, Y_train_norm_T, 'Method', 'Bag', ...
    'NumLearningCycles', GLOBAL_ENSEMBLE_TREES, 'MinLeafSize', 5, 'Options', par_opts);

Y_pred_norm_RF = predict(mdl_RF, X_val_norm_T)';
Y_pred_RF = (Y_pred_norm_RF .* sigma_Y) + mu_Y;
rmse_RF = sqrt(mean((Y_val - Y_pred_RF).^2));

models_info(end+1) = struct('name', 'Random Forest', 'val_rmse', rmse_RF);
fprintf('  -> RF 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_RF);

save(checkpoint_file, 'mdl_RF', 'models_info', '-append');

% -------------------------------------------------------------------------
% [모델 3] 서포트 벡터 머신 (SVM)
fprintf('\n--- [3/6] 서포트 벡터 머신(SVM) 학습 시작 ---\n');
tic;
mdl_SVM = fitrsvm(X_train_norm_T, Y_train_norm_T, 'KernelFunction', 'gaussian', 'IterationLimit', 1e6);

Y_pred_norm_SVM = predict(mdl_SVM, X_val_norm_T)';
Y_pred_SVM = (Y_pred_norm_SVM .* sigma_Y) + mu_Y;
rmse_SVM = sqrt(mean((Y_val - Y_pred_SVM).^2));

models_info(end+1) = struct('name', 'SVM', 'val_rmse', rmse_SVM);
fprintf('  -> SVM 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_SVM);

save(checkpoint_file, 'mdl_SVM', 'models_info', '-append');

% -------------------------------------------------------------------------
% [모델 4] 초고차원 컴퓨팅 (HDC)
fprintf('\n--- [4/6] HDC(Hyperdimensional Computing) 학습 시작 ---\n');
tic;
D_dim = 10000; 
W_hdc = randn(D_dim, size(X_train, 1));
b_hdc = randn(D_dim, 1) * 2 * pi;

H_train = cos(W_hdc * X_train_norm + b_hdc);
lambda_hdc = 1e-4;

% [개선] pinv 대신 빠르고 안정한 행렬 나눗셈(mrdivide, /) 연산 사용 (수학적 결과 동일)
beta_hdc = (Y_train_norm * H_train') / (H_train * H_train' + lambda_hdc * eye(D_dim));

H_val = cos(W_hdc * X_val_norm + b_hdc);
Y_pred_norm_HDC = beta_hdc * H_val;
Y_pred_HDC = (Y_pred_norm_HDC .* sigma_Y) + mu_Y;
rmse_HDC = sqrt(mean((Y_val - Y_pred_HDC).^2));

models_info(end+1) = struct('name', 'HDC', 'val_rmse', rmse_HDC);
fprintf('  -> HDC 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_HDC);

save(checkpoint_file, 'W_hdc', 'b_hdc', 'beta_hdc', 'models_info', '-append');

% -------------------------------------------------------------------------
% [모델 5] 심층 역전파 신경망 (Deep BPNN)
fprintf('\n--- [5/6] 심층 역전파 신경망(Deep BPNN) 학습 시작 ---\n');
tic;
net_BPNN = fitnet([256 128 64 32], 'trainlm'); % 원본 성능 유지를 위해 trainlm 사용
for i = 1:4
    net_BPNN.layers{i}.transferFcn = 'poslin'; 
end
net_BPNN.performParam.regularization = 1e-4; 
net_BPNN.trainParam.showWindow = false; 
net_BPNN.trainParam.epochs = GLOBAL_MAX_EPOCHS; 
net_BPNN.trainParam.max_fail = GLOBAL_MAX_FAIL; 

net_BPNN.divideFcn = 'divideind';
net_BPNN.divideParam.trainInd = 1:N_train;
net_BPNN.divideParam.valInd   = N_train+1 : N_train+N_val;
net_BPNN.divideParam.testInd  = [];

net_BPNN = train(net_BPNN, X_combine, Y_combine, 'useParallel', 'yes');

Y_pred_norm_BPNN = net_BPNN(X_val_norm);
Y_pred_BPNN = (Y_pred_norm_BPNN .* sigma_Y) + mu_Y;
rmse_BPNN = sqrt(mean((Y_val - Y_pred_BPNN).^2));

models_info(end+1) = struct('name', 'Deep BPNN', 'val_rmse', rmse_BPNN);
fprintf('  -> Deep BPNN 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_BPNN);

save(checkpoint_file, 'net_BPNN', 'models_info', '-append');

% -------------------------------------------------------------------------
% [모델 6] 그래디언트 부스팅 회귀 (Gradient Boosting)
fprintf('\n--- [6/6] 그래디언트 부스팅(Gradient Boosting) 학습 시작 ---\n');
tic;
mdl_GBR = fitrensemble(X_train_norm_T, Y_train_norm_T, 'Method', 'LSBoost', ...
    'NumLearningCycles', GLOBAL_ENSEMBLE_TREES, 'LearnRate', 0.1, 'MinLeafSize', 5, 'Options', par_opts);

Y_pred_norm_GBR = predict(mdl_GBR, X_val_norm_T)';
Y_pred_GBR = (Y_pred_norm_GBR .* sigma_Y) + mu_Y;
rmse_GBR = sqrt(mean((Y_val - Y_pred_GBR).^2));

models_info(end+1) = struct('name', 'Gradient Boosting', 'val_rmse', rmse_GBR);
fprintf('  -> GBR 완료 (%.2f초) | Val RMSE: %.4f [A]\n', toc, rmse_GBR);

save(checkpoint_file, 'mdl_GBR', 'models_info', '-append');

%% 5. 성능 비교 및 최적의 모델 채택 (Validation 데이터 기준)
fprintf('\n======================================================\n');
fprintf('모델 학습 완료 (Validation 데이터를 기준으로 최적 모델 평가)\n');
fprintf('======================================================\n');

model_names = {models_info.name};
val_rmse_values = [models_info.val_rmse];

figure;
bar(val_rmse_values, 'FaceColor', [0.2 0.6 0.8]);
set(gca, 'xticklabel', model_names, 'TickLabelInterpreter', 'none');
xtickangle(45);
ylabel('Validation RMSE [A] (낮을수록 좋음)');
title('최적화 평가함수용 머신러닝 모델 성능 비교 (Validation Data)');
grid on;

% Validation RMSE가 가장 낮은 모델 찾기
[best_val_rmse, best_idx] = min(val_rmse_values);
best_model_name = models_info(best_idx).name;

fprintf('비교 결과, 가장 성능이 우수한 모델은 [%s] 입니다. (Val RMSE: %.4f A)\n', best_model_name, best_val_rmse);

switch best_model_name
    case 'ANN (Paper Spec)'
        best_model = net_ANN;
    case 'Random Forest'
        best_model = mdl_RF;
    case 'SVM'
        best_model = mdl_SVM;
    case 'HDC'
        best_model = struct('W', W_hdc, 'b', b_hdc, 'beta', beta_hdc);
    case 'Deep BPNN'
        best_model = net_BPNN;
    case 'Gradient Boosting'
        best_model = mdl_GBR;
end

% 최종 최적 모델(best_model)과 모델 이름(best_model_name)을 체크포인트 파일에 추가 저장
save(checkpoint_file, 'best_model', 'best_model_name', '-append');
fprintf('=> 최적 모델 객체 및 이름이 ''%s''에 추가 저장되었습니다.\n', checkpoint_file);

%% 6. 채택된 최적 모델의 최종 Test Data 평가 (Data Leakage 없는 순수 평가)
switch best_model_name
    case 'ANN (Paper Spec)'
        Y_test_pred_norm = net_ANN(X_test_norm);
    case 'Random Forest'
        Y_test_pred_norm = predict(mdl_RF, X_test_norm')';
    case 'SVM'
        Y_test_pred_norm = predict(mdl_SVM, X_test_norm')';
    case 'HDC'
        H_test = cos(W_hdc * X_test_norm + b_hdc);
        Y_test_pred_norm = beta_hdc * H_test;
    case 'Deep BPNN'
        Y_test_pred_norm = net_BPNN(X_test_norm);
    case 'Gradient Boosting'
        Y_test_pred_norm = predict(mdl_GBR, X_test_norm')';
end

% 예측값 역변환 (Un-normalize)
Y_test_pred = (Y_test_pred_norm .* sigma_Y) + mu_Y;
test_rmse = sqrt(mean((Y_test - Y_test_pred).^2));

fprintf('=> [%s] 모델의 최종 Unseen Test Data RMSE: %.4f [A]\n', best_model_name, test_rmse);

figure;
plotregression(Y_test, Y_test_pred, sprintf('[%s] Final Test Data (i_{pk})', best_model_name));
title(sprintf('최적 채택 모델: %s 의 실제 i_{pk}와 예측값 상관관계', best_model_name), 'Interpreter', 'none');

%% 7. 최적화 평가용 연속 예측 테스트 (PSO에서 활용될 로직)
fprintf('\n=== 최적화 알고리즘(PSO 등)에서의 평가 함수(Eval Fcn) 호출 시뮬레이션 ===\n');
test_D1 = 0.3;
test_D2 = 0.3;
test_V2 = 84;
test_P_act = linspace(-150, -50, 100); 

X_new = [repmat(test_D1, 1, 100); 
         repmat(test_D2, 1, 100); 
         repmat(test_V2, 1, 100); 
         test_P_act];

% 1. X 정규화
X_new_norm = (X_new - mu_X) ./ sigma_X;

% 2. 예측 수행 (정규화된 타겟값 반환)
switch best_model_name
    case 'ANN (Paper Spec)'
        Y_new_pred_norm = net_ANN(X_new_norm);
    case 'Random Forest'
        Y_new_pred_norm = predict(mdl_RF, X_new_norm')';
    case 'SVM'
        Y_new_pred_norm = predict(mdl_SVM, X_new_norm')';
    case 'HDC'
        H_new = cos(W_hdc * X_new_norm + b_hdc);
        Y_new_pred_norm = beta_hdc * H_new;
    case 'Deep BPNN'
        Y_new_pred_norm = net_BPNN(X_new_norm);
    case 'Gradient Boosting'
        Y_new_pred_norm = predict(mdl_GBR, X_new_norm')';
end

% 3. Y 역변환 (최적화 알고리즘에 전달될 실제 전류값)
Y_new_pred_real = (Y_new_pred_norm .* sigma_Y) + mu_Y;

figure;
plot(test_P_act, Y_new_pred_real, 'b-', 'LineWidth', 2);
title(sprintf('[%s] 목표 전력(P_{act})에 따른 1차 인덕터 피크 전류(i_{pk}) 예측', best_model_name), 'Interpreter', 'none');
xlabel('P_{act} [W]'); ylabel('i_{pk,pri} [A]');
grid on;

fprintf('평가 함수 시뮬레이션 테스트가 완료되었습니다.\n');