% =========================================================================
% Optimize_Controller_MIL.m
% PI 제어기 자동 최적화 (MIL v2 - 배터리 모델 사용)
% =========================================================================
% 이 스크립트는 DAB_Plant_TPS_v2 (SPS 평균화 모델)를 사용하여
% PI 제어기 게인 (Kp, Ki)을 Grid Search로 최적화합니다.
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  PI Controller Optimization (MIL v2 - Battery Model)\n');
fprintf('=================================================================\n\n');

%% ==================== Load Configuration ====================
config = DAB_System_Config();

% Plant Parameters
params.n = config.plant.n;
params.L = config.plant.L;
params.fs = config.plant.fs;
params.C_out = config.plant.C_out;
params.efficiency = config.plant.efficiency;

% Operating Conditions
V_in = config.hv.V_nom;
V_batt = config.battery.V_nom;
R_batt = config.battery.R_internal;
V_out_init = V_batt;
Phi_int_deg = config.controller.Phi_int_deg;

% Test Conditions
I_ref = config.test.I_ref;
I_ref_init = config.test.I_ref_init;

% Simulation Settings
dt = config.sim.dt;
sim_time = config.sim.time;
step_start = config.sim.step_start;

% Optimization Settings
Kp_range = linspace(config.optim.Kp_range(1), config.optim.Kp_range(2), config.optim.n_points);
Ki_range = linspace(config.optim.Ki_range(1), config.optim.Ki_range(2), config.optim.n_points);

% Cost Function Weights
w_os = config.optim.weight_overshoot;
w_ss = config.optim.weight_ss_error;
w_st = config.optim.weight_settling;

% Targets
target_os = config.optim.target_overshoot;
target_ss = config.optim.target_ss_error;

fprintf('설정 파일 로드 완료: DAB_System_Config.m\n');
fprintf('탐색 공간: Kp [%.2f ~ %.2f], Ki [%.0f ~ %.0f]\n', ...
    min(Kp_range), max(Kp_range), min(Ki_range), max(Ki_range));
fprintf('총 조합: %d개\n\n', length(Kp_range) * length(Ki_range));

%% ==================== Simulation Setup ====================
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

%% ==================== Initialize Result Matrices ====================
cost_matrix = zeros(length(Kp_range), length(Ki_range));
overshoot_matrix = zeros(length(Kp_range), length(Ki_range));
sserror_matrix = zeros(length(Kp_range), length(Ki_range));
settling_matrix = zeros(length(Kp_range), length(Ki_range));

best_cost = inf;
best_Kp = 0;
best_Ki = 0;
best_results = struct();

fprintf('최적화 시작...\n');
tic;
total_iter = length(Kp_range) * length(Ki_range);
iter_count = 0;

%% ==================== Grid Search ====================
for i = 1:length(Kp_range)
    for j = 1:length(Ki_range)
        iter_count = iter_count + 1;
        Kp = Kp_range(i);
        Ki = Ki_range(j);
        
        % --- Initialize Simulation ---
        V_out = V_out_init;
        integrator = 0;
        I_hist = zeros(n_steps, 1);
        V_hist = zeros(n_steps, 1);
        phi_hist = zeros(n_steps, 1);
        
        % --- Main Simulation Loop ---
        for k = 1:n_steps
            % 1. 전류 피드백 계산 (배터리 모델)
            I_fb = (V_out - V_batt) / R_batt;
            
            % 2. 스텝 지령
            if time_vec(k) < step_start
                I_ref_now = I_ref_init;
            else
                I_ref_now = I_ref;
            end
            
            % 3. PI 제어기
            error_sig = I_ref_now - I_fb;
            integrator = integrator + error_sig * dt;
            phi_ext_deg = Kp * error_sig + Ki * integrator;
            
            % Saturation
            phi_ext_deg = max(-90, min(90, phi_ext_deg));
            
            % 4. Plant v2 호출 (배터리 모델)
            [V_out_next, ~, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                                     Phi_int_deg, R_batt, V_batt, dt, params);
            V_out = V_out_next;
            
            % 5. 저장
            I_hist(k) = I_fb;
            V_hist(k) = V_out;
            phi_hist(k) = phi_ext_deg;
        end
        
        % --- Performance Metrics (스텝 이후) ---
        step_idx = find(time_vec >= step_start, 1);
        ss_idx = floor(0.85 * n_steps);
        
        I_ss = mean(I_hist(ss_idx:end));
        I_peak = max(I_hist(step_idx:end));
        
        overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
        ss_error = abs(I_ss - I_ref) / I_ref * 100;
        
        % Settling Time (2% 기준)
        settling_time = sim_time - step_start;
        for sk = step_idx:n_steps
            if all(abs(I_hist(sk:end) - I_ref) < 0.02 * I_ref)
                settling_time = time_vec(sk) - step_start;
                break;
            end
        end
        
        % --- Cost Function ---
        penalty = 0;
        if overshoot > target_os
            penalty = penalty + (overshoot - target_os)^2;
        end
        if ss_error > target_ss
            penalty = penalty + (ss_error - target_ss)^2 * 10;
        end
        
        cost = w_os * overshoot + w_ss * ss_error + w_st * (settling_time * 100) + penalty;
        
        % --- Save Results ---
        cost_matrix(i, j) = cost;
        overshoot_matrix(i, j) = overshoot;
        sserror_matrix(i, j) = ss_error;
        settling_matrix(i, j) = settling_time;
        
        if cost < best_cost
            best_cost = cost;
            best_Kp = Kp;
            best_Ki = Ki;
            best_results.I_hist = I_hist;
            best_results.V_hist = V_hist;
            best_results.phi_hist = phi_hist;
            best_results.overshoot = overshoot;
            best_results.ss_error = ss_error;
            best_results.settling_time = settling_time;
        end
        
        % Progress
        if mod(iter_count, 10) == 0
            fprintf('  진행률: %d/%d (%.0f%%)\n', iter_count, total_iter, iter_count/total_iter*100);
        end
    end
end

elapsed = toc;
fprintf('\n최적화 완료! (%.1f초)\n\n', elapsed);

%% ==================== Results Summary ====================
fprintf('=================================================================\n');
fprintf('  최적화 결과\n');
fprintf('=================================================================\n');
fprintf('최적 파라미터:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.1f\n', best_Ki);
fprintf('\n성능 지표:\n');
fprintf('  오버슈트:      %.2f %%', best_results.overshoot);
if best_results.overshoot < target_os, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  정상상태 오차: %.2f %%', best_results.ss_error);
if best_results.ss_error < target_ss, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  정착시간:      %.3f sec', best_results.settling_time);
if best_results.settling_time < config.optim.target_settling, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  Cost:          %.1f\n', best_cost);
fprintf('=================================================================\n\n');

% Top 5
fprintf('Top 5 파라미터 조합:\n');
[sorted_costs, sort_idx] = sort(cost_matrix(:));
for rank = 1:min(5, length(sorted_costs))
    [ri, ci] = ind2sub(size(cost_matrix), sort_idx(rank));
    fprintf('  %d. Kp=%.2f, Ki=%.0f → OS=%.1f%%, SS=%.1f%%, ST=%.3fs\n', ...
        rank, Kp_range(ri), Ki_range(ci), ...
        overshoot_matrix(ri,ci), sserror_matrix(ri,ci), settling_matrix(ri,ci));
end
fprintf('\n');

%% ==================== Plotting ====================
figure('Name', 'PI Controller Optimization Results', 'Position', [100 100 1600 900]);

% 1. Cost Surface
subplot(2,3,1);
[Kp_grid, Ki_grid] = meshgrid(Kp_range, Ki_range);
surf(Kp_grid, Ki_grid, cost_matrix');
hold on;
plot3(best_Kp, best_Ki, best_cost, 'r*', 'MarkerSize', 20, 'LineWidth', 3);
xlabel('Kp'); ylabel('Ki'); zlabel('Cost');
title('Cost Surface');
colorbar; grid on;

% 2. Overshoot Contour
subplot(2,3,2);
contourf(Kp_grid, Ki_grid, overshoot_matrix', 15);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Overshoot (%)');
colorbar; grid on;

% 3. SS Error Contour
subplot(2,3,3);
contourf(Kp_grid, Ki_grid, sserror_matrix', 15);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Steady-State Error (%)');
colorbar; grid on;

% 4. Best Response (Current)
subplot(2,3,4);
plot(time_vec*1000, best_results.I_hist, 'b-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'r--', 'LineWidth', 1.5);
yline(I_ref * 1.05, 'g:', 'LineWidth', 1);
yline(I_ref * 0.95, 'g:', 'LineWidth', 1);
xline(step_start*1000, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('전류 응답 (Kp=%.2f, Ki=%.0f)', best_Kp, best_Ki));
legend('실제', '목표', '+5%', '-5%', '스텝', 'Location', 'best');

% 5. Best Response (Voltage)
subplot(2,3,5);
plot(time_vec*1000, best_results.V_hist, 'b-', 'LineWidth', 1.5);
hold on;
yline(V_batt, 'k--', 'LineWidth', 1);
xline(step_start*1000, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (ms)');
ylabel('V_{out} (V)');
title('출력 전압');
legend('V_{out}', 'V_{batt}', '스텝', 'Location', 'best');

% 6. Best Response (Phase)
subplot(2,3,6);
plot(time_vec*1000, best_results.phi_hist, 'm-', 'LineWidth', 1.5);
hold on;
xline(step_start*1000, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (ms)');
ylabel('\phi_{ext} (deg)');
title('위상 제어 출력');
legend('\phi_{ext}', '스텝', 'Location', 'best');

%% ==================== Update Controller File ====================
fprintf('제어기 파일 업데이트 중...\n');

controller_file = fullfile('..', 'DAB_Controller_System.m');

try
    fid = fopen(controller_file, 'r');
    if fid == -1
        error('파일을 열 수 없습니다: %s', controller_file);
    end
    content = fread(fid, '*char')';
    fclose(fid);
    
    content = regexprep(content, '(Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', best_Kp));
    content = regexprep(content, '(Ki\s*=\s*)[\d.]+', sprintf('$1%.1f', best_Ki));
    
    fid = fopen(controller_file, 'w');
    fwrite(fid, content, 'char');
    fclose(fid);
    
    fprintf('  ✓ %s 업데이트 완료\n', controller_file);
    fprintf('    Kp = %.3f, Ki = %.1f\n', best_Kp, best_Ki);
catch ME
    fprintf('  ✗ 업데이트 실패: %s\n', ME.message);
end

%% ==================== Update Config File ====================
fprintf('\n설정 파일 업데이트 중...\n');

config_file = fullfile('..', 'DAB_System_Config.m');

try
    fid = fopen(config_file, 'r');
    if fid == -1
        error('파일을 열 수 없습니다: %s', config_file);
    end
    content = fread(fid, '*char')';
    fclose(fid);
    
    content = regexprep(content, '(config\.controller\.Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', best_Kp));
    content = regexprep(content, '(config\.controller\.Ki\s*=\s*)[\d.]+', sprintf('$1%.1f', best_Ki));
    
    fid = fopen(config_file, 'w');
    fwrite(fid, content, 'char');
    fclose(fid);
    
    fprintf('  ✓ %s 업데이트 완료\n', config_file);
catch ME
    fprintf('  ✗ 업데이트 실패: %s\n', ME.message);
end

fprintf('\n최적화 프로세스 종료.\n');
