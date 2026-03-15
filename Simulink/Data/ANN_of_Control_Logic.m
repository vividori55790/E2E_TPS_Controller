% =========================================================================
% TPS 데이터셋 기반 신경망 학습 스크립트 (Surrogate Model)
% 입력: D1, D2, V2_V, P_act_W
% 출력: i_pk_pri_A, i_pk_sec_A, D0_deg
% =========================================================================

clc; clear; close all;

%% 1. 데이터 로드 및 전처리
filename = 'TPS_Dataset_20260313_090038.csv';
fprintf('데이터를 불러오는 중: %s...\n', filename);
data = readtable(filename);

% 결측치(NaN) 제거 (데이터 안정성을 위함)
data = rmmissing(data);

% 입력 변수 (X): 내부 위상(D1, D2), 전압(V2_V), 목표 전력(P_act_W)
% 참고: MATLAB의 fitnet은 열(Column)을 하나의 샘플로 취급하므로 전치(')가 필요합니다.
X = [data.D1, data.D2, data.V2_V, data.P_act_W]';

% 출력 변수 (Y): 1차 인덕터 전류, 2차 인덕터 전류, 외부 위상(D0_deg)
Y = [data.i_pk_pri_A, data.i_pk_sec_A, data.D0_deg]';

fprintf('총 %d개의 샘플 데이터가 준비되었습니다.\n', size(X, 2));

%% 2. 인공신경망(ANN) 구조 설정
% 은닉층(Hidden Layer) 설정: 노드 개수 [32, 16] 형태의 2개 층 사용
% 비선형성이 강한 경우 노드 수를 [64, 32] 등으로 늘릴 수 있습니다.
hiddenLayerSize = [64, 64, 32]; 

% 최적의 학습 알고리즘 선택: 'trainlm' (Levenberg-Marquardt)
% 과적합(Overfitting) 방지가 더 필요하다면 'trainbr' (Bayesian Regularization) 사용 권장
trainFcn = 'trainbr'; 

% 피드포워드 신경망 객체 생성 (데이터 정규화는 fitnet 내부에서 자동으로 수행됨)
net = fitnet(hiddenLayerSize, trainFcn);

%% 3. 데이터 분할 및 학습 조건 설정
% 데이터를 학습(Train), 검증(Validation), 테스트(Test) 용도로 무작위 분할
net.divideParam.trainRatio = 70/100; % 70% 학습
net.divideParam.valRatio   = 15/100; % 15% 검증 (과적합 방지용)
net.divideParam.testRatio  = 15/100; % 15% 최종 성능 평가용

% 하이퍼파라미터 미세 조정 (선택 사항)
net.trainParam.epochs = 1000;    % 최대 반복 횟수
net.trainParam.goal = 1e-6;      % 목표 오차율(MSE)
net.trainParam.min_grad = 1e-7;  % 최소 기울기

%% 4. 신경망 학습 진행
fprintf('\n신경망 학습을 시작합니다. (Training Tool 창이 팝업됩니다)...\n');
[net, tr] = train(net, X, Y);

%% 5. 학습 결과 평가 및 시각화
% 전체 데이터에 대한 예측값 계산
Y_pred = net(X);

% 오차(Error) 계산
errors = gsubtract(Y, Y_pred);

% 성능 지표(MSE: Mean Squared Error) 계산
trainPerformance = perform(net, Y(:,tr.trainInd), Y_pred(:,tr.trainInd));
valPerformance = perform(net, Y(:,tr.valInd), Y_pred(:,tr.valInd));
testPerformance = perform(net, Y(:,tr.testInd), Y_pred(:,tr.testInd));

fprintf('\n=== 학습 성능 (MSE) ===\n');
fprintf('학습 데이터 MSE: %e\n', trainPerformance);
fprintf('검증 데이터 MSE: %e\n', valPerformance);
fprintf('테스트 데이터 MSE: %e\n', testPerformance);

% 회귀 분석 플롯 (실제값 vs 예측값)
figure;
plotregression(Y(:,tr.trainInd), Y_pred(:,tr.trainInd), 'Training', ...
               Y(:,tr.valInd), Y_pred(:,tr.valInd), 'Validation', ...
               Y(:,tr.testInd), Y_pred(:,tr.testInd), 'Testing', ...
               Y, Y_pred, 'All Data');
title('실제값과 예측값의 상관관계 분석');

%% 6. 연속적인 결과 예측 테스트 (시뮬레이터 활용 예시)
fprintf('\n=== 임의의 새로운 값으로 연속 예측 테스트 ===\n');
% 예: D1=0.3, D2=0.3, V2=84V, P_act = -100W일 때 연속적인 변화량 예측
test_D1 = 0.3;
test_D2 = 0.3;
test_V2 = 84;
test_P_act = linspace(-150, -50, 100); % 전력을 -150W에서 -50W까지 100단계로 연속 변화

% 새로운 입력 행렬 구성
X_new = [repmat(test_D1, 1, 100); 
         repmat(test_D2, 1, 100); 
         repmat(test_V2, 1, 100); 
         test_P_act];

% 학습된 신경망을 통한 고속 예측
Y_new_pred = net(X_new);

% 결과 시각화
figure;
subplot(2,1,1);
plot(test_P_act, Y_new_pred(1,:), 'r-', 'LineWidth', 2); hold on;
plot(test_P_act, Y_new_pred(2,:), 'b--', 'LineWidth', 2);
xlabel('Target Power (P_{act}) [W]');
ylabel('Inductor Current [A]');
legend('i_{pk\_pri} (1차측)', 'i_{pk\_sec} (2차측)');
title('입력 전력 변화에 따른 인덕터 전류 연속 예측');
grid on;

subplot(2,1,2);
plot(test_P_act, Y_new_pred(3,:), 'k-', 'LineWidth', 2);
xlabel('Target Power (P_{act}) [W]');
ylabel('External Phase (D0) [deg]');
title('입력 전력 변화에 따른 외부 위상 연속 예측');
grid on;

%% 7. 모델 저장
% 차후 시뮬레이터 로직이나 제어기 설계에 활용하기 위해 모델을 저장합니다.
save('TPS_Surrogate_Model_BR.mat', 'net');
fprintf('\n학습된 모델이 TPS_Surrogate_Model.mat 파일로 저장되었습니다.\n');