% main.m - HDC 데이터셋 생성 및 검증
clc; clear; close all;

% 1. 초기 설정
fprintf('--- [HDC] Robust DAB Control Optimization Start ---\n');
params = config_dab();

% 2. 데이터 포인트 생성 (Day 2: 50개, Day 3: 10,000개 이상)
N_samples = 500; % 검증용
% Vin 범위: 300V ~ 450V
Vin_vec = 300 + (450-300) * rand(N_samples, 1);
% Vout 범위: Vout_min ~ Vout_max (배터리 전압 변동)
Vout_vec = params.Vout_min + (params.Vout_max - params.Vout_min) * rand(N_samples, 1);
% Pref 범위: -P_rated ~ P_rated (양방향)
% P_rated가 없는 경우를 대비한 안전장치 (config_dab 업데이트 가정)
if isfield(params, 'P_rated')
    P_max = params.P_rated;
else
    P_max = 5000; % Default
end
% Pref 범위: -P_rated ~ P_rated (양방향)
% 0 주변의 Deadzone(±5%)은 회피하는 것이 학습에 유리함
rand_sign = sign(rand(N_samples, 1) - 0.5);
Pref_vec = rand_sign .* ((0.05 * P_max) + (P_max * 0.95) * rand(N_samples, 1));

% 결과 저장용 배열
Results = zeros(N_samples, 7); % [Vin, Vout, Pref, D1, D2, Phi, Efficiency]

% 3. 최적화 루프 실행
% parfor i = 1:N_samples 
for i = 1:N_samples
    Vin = Vin_vec(i);
    Vout = Vout_vec(i);
    Pref = Pref_vec(i);
    
    try
        [opt_x, min_loss] = dab_optimizer(Vin, Vout, Pref, params);
        
        % 후처리: 효율 계산
        [Cost, ~, ~, ~, P_in, P_loss_real] = physics_engine(opt_x, Vin, Vout, params.L_nom, params);
        
        % Efficiency Calculation (Bidirectional)
        % P_loss_real은 항상 양수(손실)
        % P_in은 양수(순방향) 또는 음수(역방향)
        
        P_abs = abs(P_in);
        
        if P_abs > P_loss_real && P_abs > 1e-6
            Efficiency = (P_abs - P_loss_real) / P_abs;
        else
            Efficiency = 0; 
        end
        
        Results(i, :) = [Vin, Vout, Pref, opt_x(1), opt_x(2), opt_x(3), Efficiency];
        
        if mod(i, 10) == 0
            fprintf('Progress: %d / %d (Eff: %.2f%%)\n', i, N_samples, Efficiency*100);
        end
    catch ME
        fprintf('Error at sample %d: %s\n', i, ME.message);
        Results(i, :) = [Vin, Vout, Pref, NaN, NaN, NaN, NaN];
    end
end

% 4. 데이터 저장 및 시각화
valid_idx = ~isnan(Results(:,7));
Data = Results(valid_idx, :);

save('HDC_Dataset.mat', 'Data');
writematrix(Data, 'HDC_Dataset.csv'); % Save as CSV
fprintf('데이터 저장 완료: HDC_Dataset.mat, HDC_Dataset.csv\n');

% 시각화 (X축: Voltage Gain M, Y축: Power, Z축: Eff)
if ~isempty(Data)
    Vin = Data(:,1); Vout = Data(:,2);
    Gain_M = (params.n * Vout) ./ Vin; % Voltage Conversion Ratio
    
    figure;
    scatter3(Gain_M, Data(:,3), Data(:,7) * 100, 50, Data(:,7), 'filled');
    xlabel('Voltage Gain M (nVout/Vin)');
    ylabel('Power Command [W]');
    zlabel('Efficiency [%]');
    title('Robust DAB Optimization (Variable Vout)');
    colorbar; grid on;
    view(45, 30);
end
