% =========================================================================
% Final_Optimize_Controller.m
% 최종 PI 제어기 최적화 (MIL v2 사용, Ki 확장 범위)
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Final PI Controller Optimization (MIL v2)\n');
fprintf('=================================================================\n\n');

%% Parameters
params.n = 4.0;
params.L = 50e-6;
params.fs = 80e3;
params.C_out = 500e-6;
params.efficiency = 0.95;

V_in = 400;
V_out_init = 100;
Load = 10;  % 10 ohm (10A at 100V)
Phi_int_deg = 15;
I_ref = 10;

%% Tuning Range (Ki 범위 확대)
Kp_range = linspace(0.3, 1.5, 7);
Ki_range = linspace(50, 200, 7);  % Ki 50~200

fprintf('탐색 공간: Kp [%.2f ~ %.2f], Ki [%.0f ~ %.0f]\n', ...
    min(Kp_range), max(Kp_range), min(Ki_range), max(Ki_range));
fprintf('총 조합: %d개\n\n', length(Kp_range) * length(Ki_range));

%% Simulation Setup
dt = 10e-6;
sim_time = 0.100;  % 100ms
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);
step_start = 0.010;  % 10ms에서 스텝

best_cost = inf;
best_Kp = 0;
best_Ki = 0;
best_results = [];

cost_matrix = zeros(length(Kp_range), length(Ki_range));
overshoot_matrix = zeros(length(Kp_range), length(Ki_range));
sserror_matrix = zeros(length(Kp_range), length(Ki_range));
settling_matrix = zeros(length(Kp_range), length(Ki_range));

fprintf('최적화 시작...\n');
tic;
total_iter = length(Kp_range) * length(Ki_range);
iter_count = 0;

for i = 1:length(Kp_range)
    for j = 1:length(Ki_range)
        iter_count = iter_count + 1;
        Kp = Kp_range(i);
        Ki = Ki_range(j);
        
        % Simulation
        V_out = V_out_init;
        integrator = 0;
        I_hist = zeros(n_steps, 1);
        V_hist = zeros(n_steps, 1);
        
        for k = 1:n_steps
            I_fb = V_out / Load;
            
            if time_vec(k) < step_start
                I_ref_now = 0;
            else
                I_ref_now = I_ref;
            end
            
            error_sig = I_ref_now - I_fb;
            integrator = integrator + error_sig * dt;
            phi_ext_deg = Kp * error_sig + Ki * integrator;
            phi_ext_deg = max(-90, min(90, phi_ext_deg));
            
            [V_out_next, ~, ~, ~] = DAB_Plant_TPS_v2(V_in, V_out, phi_ext_deg, ...
                                                     Phi_int_deg, Load, dt, params);
            V_out = V_out_next;
            I_hist(k) = I_fb;
            V_hist(k) = V_out;
        end
        
        % Performance Metrics (스텝 이후 분석)
        step_idx = find(time_vec >= step_start, 1);
        ss_idx = floor(0.85 * n_steps);
        
        I_ss = mean(I_hist(ss_idx:end));
        I_peak = max(I_hist(step_idx:end));
        
        overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
        ss_error = abs(I_ss - I_ref) / I_ref * 100;
        
        % Settling Time (2%)
        settling_time = sim_time - step_start;
        for sk = step_idx:n_steps
            if all(abs(I_hist(sk:end) - I_ref) < 0.02*I_ref)
                settling_time = time_vec(sk) - step_start;
                break;
            end
        end
        
        % Save
        overshoot_matrix(i,j) = overshoot;
        sserror_matrix(i,j) = ss_error;
        settling_matrix(i,j) = settling_time;
        
        % Cost Function
        % 목표: 오버슈트 < 5%, 정상오차 < 2%, 정착시간 최소화
        w_os = 5.0;
        w_ss = 10.0;  % 정상상태 오차 매우 중요
        w_st = 1.0;
        
        penalty = 0;
        if overshoot > 5, penalty = penalty + (overshoot - 5)^2; end
        if ss_error > 2, penalty = penalty + (ss_error - 2)^2 * 10; end
        
        cost = w_os * overshoot + w_ss * ss_error + w_st * (settling_time * 100) + penalty;
        cost_matrix(i,j) = cost;
        
        if cost < best_cost
            best_cost = cost;
            best_Kp = Kp;
            best_Ki = Ki;
            best_results.I_hist = I_hist;
            best_results.V_hist = V_hist;
            best_results.overshoot = overshoot;
            best_results.ss_error = ss_error;
            best_results.settling_time = settling_time;
        end
        
        if mod(iter_count, 7) == 0
            fprintf('  진행률: %d/%d (%.0f%%)\n', iter_count, total_iter, iter_count/total_iter*100);
        end
    end
end

elapsed = toc;
fprintf('\n최적화 완료! (%.1f초)\n\n', elapsed);

%% Results
fprintf('=================================================================\n');
fprintf('  최적화 결과\n');
fprintf('=================================================================\n');
fprintf('최적 파라미터:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.1f\n', best_Ki);
fprintf('\n성능 지표:\n');
fprintf('  오버슈트:    %.2f %%', best_results.overshoot);
if best_results.overshoot < 5, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  정상상태 오차: %.2f %%', best_results.ss_error);
if best_results.ss_error < 2, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  정착시간:    %.3f sec', best_results.settling_time);
if best_results.settling_time < 0.03, fprintf(' ✓\n'); else, fprintf(' ✗\n'); end
fprintf('  Cost:        %.1f\n', best_cost);
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

%% Plotting
figure('Name', 'Final Optimization Results', 'Position', [100 100 1600 900]);

subplot(2,3,1);
[Kp_grid, Ki_grid] = meshgrid(Kp_range, Ki_range);
surf(Kp_grid, Ki_grid, cost_matrix');
hold on;
plot3(best_Kp, best_Ki, best_cost, 'r*', 'MarkerSize', 20, 'LineWidth', 3);
xlabel('Kp'); ylabel('Ki'); zlabel('Cost');
title('Cost Surface');
colorbar; grid on;

subplot(2,3,2);
contourf(Kp_grid, Ki_grid, overshoot_matrix', 15);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Overshoot (%)');
colorbar; grid on;

subplot(2,3,3);
contourf(Kp_grid, Ki_grid, sserror_matrix', 15);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Steady-State Error (%)');
colorbar; grid on;

subplot(2,3,[4,5,6]);
plot(time_vec*1000, best_results.I_hist, 'b-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'r--', 'LineWidth', 1.5);
yline(I_ref * 1.05, 'g:', 'LineWidth', 1);
yline(I_ref * 0.95, 'g:', 'LineWidth', 1);
xline(step_start*1000, 'k--', 'LineWidth', 1);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('최적 응답 (Kp=%.3f, Ki=%.0f): OS=%.1f%%, SS_Err=%.1f%%', ...
    best_Kp, best_Ki, best_results.overshoot, best_results.ss_error));
legend('실제', '목표', '+5%', '-5%', '스텝 시작', 'Location', 'best');

%% Update Controller File
fprintf('제어기 파일 업데이트 중...\n');
controller_file = 'DAB_Controller_System.m';

try
    fid = fopen(controller_file, 'r');
    content = fread(fid, '*char')';
    fclose(fid);
    
    content = regexprep(content, '(Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', best_Kp));
    content = regexprep(content, '(Ki\s*=\s*)[\d.]+', sprintf('$1%.1f', best_Ki));
    
    copyfile(controller_file, 'DAB_Controller_System.m.backup_final');
    
    fid = fopen(controller_file, 'w');
    fwrite(fid, content, 'char');
    fclose(fid);
    
    fprintf('  ✓ %s 업데이트 완료\n', controller_file);
    fprintf('    Kp = %.3f, Ki = %.1f\n', best_Kp, best_Ki);
catch ME
    fprintf('  ✗ 업데이트 실패: %s\n', ME.message);
end

fprintf('\n최종 최적화 프로세스 종료.\n');
