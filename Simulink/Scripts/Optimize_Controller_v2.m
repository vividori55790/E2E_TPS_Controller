% =========================================================================
% Optimize_Controller_v2.m
% PI 제어기 파라미터 자동 최적화 - 오버슈트 중심 버전
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  PI Controller Auto-Optimization V2 (Overshoot-Focused)\n');
fprintf('=================================================================\n\n');

%% Plant Parameters (고정)
params.n = 4.0;
params.L = 50e-6;
params.fs = 80e3;
params.C_out = 1000e-6;
params.efficiency = 0.95;

%% Test Scenario (고정)
V_in = 400;
V_out_init = 100;
Load = 20;
I_ref = 10;

%% Tuning Range (확장)
Kp_range = linspace(0.3, 2.0, 10);  % 10 points
Ki_range = linspace(8, 50, 10);     % 10 points (더 높은 값까지)
Phi_int_deg = 15;

fprintf('탐색 공간: Kp [%.2f ~ %.2f], Ki [%.2f ~ %.2f]\n', ...
    min(Kp_range), max(Kp_range), min(Ki_range), max(Ki_range));
fprintf('총 조합: %d개\n\n', length(Kp_range) * length(Ki_range));

%% Load TPS Table
if exist('TPS_Table.mat', 'file')
    load('TPS_Table.mat', 'TPS_Table');
    has_tps = true;
    fprintf('TPS Table 로드 완료\n');
else
    has_tps = false;
    fprintf('TPS Table 없음 (SPS 모드)\n');
end

%% Optimization Setup
dt = 10e-6; % 10us (통일)
sim_time = 0.030; % 30ms
time_vec = 0:dt:sim_time;
n_steps = length(time_vec);

best_cost = inf;
best_Kp = 0;
best_Ki = 0;
best_results = [];

cost_matrix = zeros(length(Kp_range), length(Ki_range));
overshoot_matrix = zeros(length(Kp_range), length(Ki_range));
sserror_matrix = zeros(length(Kp_range), length(Ki_range));

fprintf('\n최적화 시작...\n');
total_iter = length(Kp_range) * length(Ki_range);
iter_count = 0;
tic;

%% Grid Search
for i = 1:length(Kp_range)
    for j = 1:length(Ki_range)
        iter_count = iter_count + 1;
        
        Kp = Kp_range(i);
        Ki = Ki_range(j);
        
        % Run Simulation
        V_out = V_out_init;
        integrator = 0;
        
        I_out_hist = zeros(n_steps, 1);
        V_out_hist = zeros(n_steps, 1);
        
        for k = 1:n_steps
            I_fb = V_out / Load;
            error_sig = I_ref - I_fb;
            integrator = integrator + error_sig * dt;
            phi_ext_deg = Kp * error_sig + Ki * integrator;
            phi_ext_deg = max(-90, min(90, phi_ext_deg));
            
            if has_tps
                P_ref = V_out * I_ref;
                V1_min = min(TPS_Table.V1_vec);
                V1_max = max(TPS_Table.V1_vec);
                P_min = min(TPS_Table.P_vec);
                P_max = max(TPS_Table.P_vec);
                V_in_clip = max(V1_min, min(V1_max, V_in));
                P_ref_clip = max(P_min, min(P_max, P_ref));
                D1 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D1, V_in_clip, P_ref_clip, 'linear', 0);
                D2 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D2, V_in_clip, P_ref_clip, 'linear', 0);
                D3 = interp2(TPS_Table.V1_vec, TPS_Table.P_vec, TPS_Table.D3, V_in_clip, P_ref_clip, 'linear', 0);
            else
                D1 = phi_ext_deg / 360;
                D2 = Phi_int_deg / 360;
                D3 = 0;
            end
            
            [V_out_next, ~, ~] = DAB_Plant_TPS(V_in, V_out, D1, D2, D3, Load, dt, params);
            V_out = V_out_next;
            
            I_out_hist(k) = I_fb;
            V_out_hist(k) = V_out;
        end
        
        % Performance Metrics
        ss_idx = floor(0.7 * n_steps);
        I_ss = mean(I_out_hist(ss_idx:end));
        V_ss = mean(V_out_hist(ss_idx:end));
        
        I_peak = max(I_out_hist);
        V_peak = max(V_out_hist);
        
        % 오버슈트 계산 (목표값 기준)
        overshoot_I = max(0, (I_peak - I_ref) / I_ref * 100);
        
        % 정상상태 오차 (목표값 기준)
        ss_error = abs(I_ss - I_ref) / I_ref * 100;
        
        % Settling Time (2%)
        settling_threshold = 0.02 * I_ref;
        settling_time = sim_time;
        for sk = ss_idx:n_steps
            if all(abs(I_out_hist(sk:end) - I_ss) < settling_threshold)
                settling_time = time_vec(sk);
                break;
            end
        end
        
        % 저장
        overshoot_matrix(i, j) = overshoot_I;
        sserror_matrix(i, j) = ss_error;
        
        % Cost Function (오버슈트 중심)
        % 목표: 오버슈트 < 5%, 정착시간 최소화
        w_overshoot = 5.0;   % 오버슈트 가중치 (증가)
        w_ss_error = 2.0;    % 정상상태 오차
        w_settling = 0.5;    % 정착시간 (감소)
        
        % Penalty: 오버슈트가 5% 초과 시 추가 패널티
        overshoot_penalty = 0;
        if overshoot_I > 5
            overshoot_penalty = (overshoot_I - 5)^2;
        end
        
        cost = w_overshoot * overshoot_I + w_ss_error * ss_error + ...
               w_settling * (settling_time * 100) + overshoot_penalty;
        cost_matrix(i, j) = cost;
        
        if cost < best_cost
            best_cost = cost;
            best_Kp = Kp;
            best_Ki = Ki;
            best_results.I_out = I_out_hist;
            best_results.V_out = V_out_hist;
            best_results.overshoot_I = overshoot_I;
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
fprintf('\n최적화 완료! (%.2f초 소요)\n\n', elapsed);

%% Results
fprintf('=================================================================\n');
fprintf('  최적화 결과\n');
fprintf('=================================================================\n');
fprintf('최적 파라미터:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.3f\n', best_Ki);
fprintf('\n성능 지표:\n');
fprintf('  전류 오버슈트:  %.2f %%', best_results.overshoot_I);
if best_results.overshoot_I < 5
    fprintf(' ✓ 목표 달성\n');
else
    fprintf(' ✗ 목표 미달 (< 5%%)\n');
end
fprintf('  정상상태 오차:  %.2f %%\n', best_results.ss_error);
fprintf('  정착시간:       %.4f sec\n', best_results.settling_time);
fprintf('  Cost:           %.2f\n', best_cost);
fprintf('=================================================================\n\n');

%% 최적 파라미터 범위 내 Top 5 출력
fprintf('Top 5 파라미터 조합:\n');
[sorted_costs, sort_idx] = sort(cost_matrix(:));
for rank = 1:min(5, length(sorted_costs))
    [ri, ci] = ind2sub(size(cost_matrix), sort_idx(rank));
    fprintf('  %d. Kp=%.2f, Ki=%.1f → Cost=%.1f, OS=%.1f%%, SS_Err=%.1f%%\n', ...
        rank, Kp_range(ri), Ki_range(ci), sorted_costs(rank), ...
        overshoot_matrix(ri, ci), sserror_matrix(ri, ci));
end
fprintf('\n');

%% Plot
figure('Name', 'Optimization Results V2', 'Position', [100 100 1600 900]);

subplot(2,3,1);
[Kp_grid, Ki_grid] = meshgrid(Kp_range, Ki_range);
surf(Kp_grid, Ki_grid, cost_matrix');
hold on;
plot3(best_Kp, best_Ki, best_cost, 'r*', 'MarkerSize', 20, 'LineWidth', 3);
xlabel('Kp'); ylabel('Ki'); zlabel('Cost');
title('Cost Function Surface');
colorbar; grid on;

subplot(2,3,2);
contourf(Kp_grid, Ki_grid, overshoot_matrix', 20);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Overshoot (%)');
colorbar; grid on;

subplot(2,3,3);
contourf(Kp_grid, Ki_grid, sserror_matrix', 20);
hold on;
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('Kp'); ylabel('Ki');
title('Steady-State Error (%)');
colorbar; grid on;

subplot(2,3,[4,5,6]);
plot(time_vec*1000, best_results.I_out, 'b-', 'LineWidth', 1.5);
hold on;
yline(I_ref, 'r--', 'LineWidth', 1.5);
yline(I_ref * 1.05, 'g--', 'LineWidth', 1, 'Label', '+5%');
yline(I_ref * 0.95, 'g--', 'LineWidth', 1, 'Label', '-5%');
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title(sprintf('최적 응답 (Kp=%.3f, Ki=%.3f): 오버슈트 %.2f%%, 정상오차 %.2f%%', ...
    best_Kp, best_Ki, best_results.overshoot_I, best_results.ss_error));
legend('실제', '목표', '+5% 허용', '-5% 허용', 'Location', 'best');

fprintf('플롯 완료!\n');

%% Update Controller File
fprintf('\n제어기 파일 업데이트 중...\n');
controller_file = 'DAB_Controller_System.m';

try
    fid = fopen(controller_file, 'r');
    if fid == -1
        error('파일을 열 수 없습니다.');
    end
    file_content = fread(fid, '*char')';
    fclose(fid);
    
    file_content = regexprep(file_content, '(Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', best_Kp));
    file_content = regexprep(file_content, '(Ki\s*=\s*)[\d.]+', sprintf('$1%.3f', best_Ki));
    
    backup_file = 'DAB_Controller_System.m.backup2';
    copyfile(controller_file, backup_file);
    fprintf('  - 백업 생성: %s\n', backup_file);
    
    fid = fopen(controller_file, 'w');
    fwrite(fid, file_content, 'char');
    fclose(fid);
    
    fprintf('  ✓ %s 업데이트 완료\n', controller_file);
    fprintf('    Kp: %.3f, Ki: %.3f\n', best_Kp, best_Ki);
catch ME
    fprintf('  ✗ 업데이트 실패: %s\n', ME.message);
end

fprintf('\n최적화 V2 프로세스 종료.\n');
