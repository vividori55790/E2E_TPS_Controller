% =========================================================================
% Generate_TPS_Dataset.m
% DAB 컨버터 TPS 제어 최적 동작점(Optimal Operating Point) 데이터셋 생성기
%
% 목적:
%   다양한 전압 이득(d)와 부하 조건(P_pu)에 대해
%   RMS 전류(손실)를 최소화하는 TPS 제어 변수 (D1, D2, D3)를 찾는다.
%   결과물은 HDC(Hyperdimensional Computing) 학습용 데이터로 사용된다.
%
% 출력 파일: TPS_Optimal_Dataset.mat
% =========================================================================

clear; clc;
addpath(genpath('../')); % 상위 폴더 및 Scripts 포함

%% 1. 시스템 파라미터 설정
% (DAB_System_Config.m 값 참조)
sys_params.L  = 90e-6;    % 누설 인덕턴스 (H)
sys_params.fs = 80e3;     % 스위칭 주파수 (Hz)
sys_params.n  = 1;        % 권선비 (1:1 가정)
sys_params.V1_nom = 380;  % 기준 1차측 전압 (V)

% 계산용 최대 전력 (Normalization Base)
% P_max_theoretical = (V1 * nV2) / (8*fs*L)
P_max_base = (sys_params.V1_nom^2) / (8 * sys_params.fs * sys_params.L);
fprintf('Base Max Power (for Pu=1): %.2f kW\n', P_max_base/1000);

%% 2. 탐색 범위 설정 (Sweep Range)
d_gain_vec = 0.5 : 0.05 : 1.5;   % 전압 이득 (V2'/V1)
p_pu_vec   = -0.8 : 0.05 : 0.8;  % 전력 지령 (Pu)

% 결과 저장용 변수 pre-allocation
num_d = length(d_gain_vec);
num_p = length(p_pu_vec);

res_D1 = zeros(num_d, num_p);
res_D2 = zeros(num_d, num_p);
res_D3 = zeros(num_d, num_p);
res_Irms = zeros(num_d, num_p);
res_ZVS = zeros(num_d, num_p);
res_P_err = zeros(num_d, num_p);

%% 3. 최적화 실행 (Parallel Computing 권장)
% parpool('local'); % 필요 시 활성화

fprintf('Starting TPS Optimization Sweep...\n');
fprintf('Total Points: %d x %d = %d\n', num_d, num_p, num_d*num_p);

options = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 100, ...
    'OptimalityTolerance', 1e-6);

tic;

for i = 1:num_d
    d = d_gain_vec(i);
    V1 = sys_params.V1_nom;
    V2 = d * (V1 / sys_params.n); % V2' = d * V1
    
    for j = 1:num_p
        p_target_pu = p_pu_vec(j);
        P_req = p_target_pu * P_max_base;
        
        % 경부하(0 근처) 예외 처리: 그냥 모든 변수 0
        if abs(p_target_pu) < 0.01
            res_D1(i, j) = 0; res_D2(i, j) = 0.5; res_D3(i, j) = 0.5;
            res_Irms(i, j) = 0; res_ZVS(i, j) = 1;
            continue;
        end
        
        % --- Optimization Problem Setup ---
        % x = [D1, D2, D3]
        % Bounds:
        % D1: -0.5 ~ 0.5 (Phase)
        % D2, D3: 0 ~ 0.45 (Duty modulation, 0.45 limit for deadtime safety)
        
        lb = [-0.5, 0, 0];
        ub = [ 0.5, 0.45, 0.45];
        
        % Initial Guess (Start from SPS)
        % SPS requires phase shift roughly proportional to Power
        % P ~ d * phi * (1 - |phi|)
        % Simple linear approx: phi ~ P_pu / 2 (Normalized)
        phi_init = p_target_pu * 0.5; 
        x0 = [phi_init, 0, 0]; 
        
        % Objective Function: Minimize RMS Current
        obj_fun = @(x) get_rms_cost(x, V1, V2, sys_params);
        
        % Constraint Function: Power Matching
        nonl_con = @(x) power_constraint(x, V1, V2, sys_params, P_req);
        
        try
            [x_opt, f_val, exitflag] = fmincon(obj_fun, x0, [], [], [], [], lb, ub, nonl_con, options);
            
            % 결과 저장
            res_D1(i, j) = x_opt(1);
            res_D2(i, j) = x_opt(2);
            res_D3(i, j) = x_opt(3);
            
            % 상세 메트릭 재계산
            [P_act, I_rms, ~, ZVS] = calc_tps_performance(V1, V2, sys_params.L, sys_params.fs, sys_params.n, x_opt(1), x_opt(2), x_opt(3));
            
            res_Irms(i, j) = I_rms;
            res_ZVS(i, j) = ZVS;
            res_P_err(i, j) = P_act - P_req;
            
        catch ME
            fprintf('Error at d=%.2f, P_pu=%.2f: %s\n', d, p_target_pu, ME.message);
        end
    end
    
    % 진행상황 표시
    elapsed = toc;
    progress = (i / num_d) * 100;
    fprintf('Progress: %.1f%% (d=%.2f done) - Elapsed: %.1fs\n', progress, d, elapsed);
end

%% 4. 결과 저장 및 시각화
DatasetName = 'TPS_Optimal_Dataset.mat';
save(DatasetName, 'd_gain_vec', 'p_pu_vec', 'res_D1', 'res_D2', 'res_D3', 'res_Irms', 'res_ZVS', 'sys_params');
fprintf('Optimization Complete. Dataset saved to %s\n', DatasetName);

% 간단한 컨투어 플롯 (D1 Map)
figure;
contourf(p_pu_vec, d_gain_vec, res_D1, 20, 'LineColor', 'none');
colorbar;
xlabel('Power (pu)');
ylabel('Voltage Gain (d)');
title('Optimal Phase Shift (D1) Map');
grid on;


%% --- Local Helper Functions ---

function I_rms = get_rms_cost(x, V1, V2, p)
    D1 = x(1); D2 = x(2); D3 = x(3);
    [~, I_rms, ~, ~] = calc_tps_performance(V1, V2, p.L, p.fs, p.n, D1, D2, D3);
end

function [c, ceq] = power_constraint(x, V1, V2, p, P_target)
    D1 = x(1); D2 = x(2); D3 = x(3);
    [P_act, ~, ~, ~] = calc_tps_performance(V1, V2, p.L, p.fs, p.n, D1, D2, D3);
    
    c = []; % No inequality constraints
    ceq = P_act - P_target; % P_act must equal P_target
end
