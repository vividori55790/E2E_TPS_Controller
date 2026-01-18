% =========================================================================
% Auto_Optimize_And_Verify.m
% 최적화 실행 후 여러 시나리오에서 자동 검증하는 통합 스크립트
% =========================================================================
% 개선 사항:
%   - 2단계 Grid Search (Coarse → Fine) 적용
%   - 기존 대비 시뮬레이션 횟수 50% 감소, 정밀도 향상
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Auto Optimize & Verify - DAB Controller\n');
fprintf('  (2-Stage Grid Search Algorithm)\n');
fprintf('=================================================================\n\n');

%% ==================== Load Configuration ====================
config = DAB_System_Config();
fprintf('설정 파일 로드 완료: DAB_System_Config.m\n\n');

%% ==================== STEP 1: Run Optimization (2-Stage) ====================
fprintf('=================================================================\n');
fprintf('  STEP 1: PI 제어기 2단계 최적화\n');
fprintf('=================================================================\n\n');

% 2단계 최적화 실행
[best_Kp, best_Ki, optim_results, optim_stats] = run_two_stage_optimization(config);

fprintf('\n최적화 결과:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.1f\n', best_Ki);
fprintf('  오버슈트: %.2f%%\n', optim_results.overshoot);
fprintf('  정상상태 오차: %.2f%%\n', optim_results.ss_error);
fprintf('  정착시간: %.3f sec\n', optim_results.settling_time);
fprintf('\n최적화 통계:\n');
fprintf('  총 시뮬레이션 횟수: %d (기존 %d → %.0f%% 감소)\n', ...
        optim_stats.total_sims, optim_stats.baseline_sims, ...
        (1 - optim_stats.total_sims/optim_stats.baseline_sims)*100);
fprintf('  소요 시간: %.2f sec\n\n', optim_stats.elapsed_time);

%% ==================== STEP 2: Define Test Scenarios ====================
fprintf('=================================================================\n');
fprintf('  STEP 2: 테스트 시나리오 정의\n');
fprintf('=================================================================\n\n');

scenarios = {
    struct('name', 'Nominal',       'V_in', 400, 'I_ref', 10,  'V_batt', 100);
    struct('name', 'Low Voltage',   'V_in', 350, 'I_ref', 10,  'V_batt', 100);
    struct('name', 'High Voltage',  'V_in', 450, 'I_ref', 10,  'V_batt', 100);
    struct('name', 'Low Current',   'V_in', 400, 'I_ref', 5,   'V_batt', 100);
    struct('name', 'High Current',  'V_in', 400, 'I_ref', 15,  'V_batt', 100);
    struct('name', 'Low Battery',   'V_in', 400, 'I_ref', 10,  'V_batt', 90);
    struct('name', 'High Battery',  'V_in', 400, 'I_ref', 10,  'V_batt', 110);
};

fprintf('정의된 시나리오: %d개\n', length(scenarios));
for i = 1:length(scenarios)
    s = scenarios{i};
    fprintf('  %d. %s (V_in=%dV, I_ref=%dA, V_batt=%dV)\n', ...
            i, s.name, s.V_in, s.I_ref, s.V_batt);
end
fprintf('\n');

%% ==================== STEP 3: Run Verification ====================
fprintf('=================================================================\n');
fprintf('  STEP 3: 시나리오별 검증 실행\n');
fprintf('=================================================================\n\n');

targets.overshoot = config.optim.target_overshoot;
targets.ss_error = config.optim.target_ss_error;
targets.settling = config.optim.target_settling;

verify_results = cell(length(scenarios), 1);
pass_count = 0;

for i = 1:length(scenarios)
    scenario = scenarios{i};
    fprintf('[%d/%d] %s 검증 중... ', i, length(scenarios), scenario.name);
    
    result = verify_scenario(scenario, best_Kp, best_Ki, config);
    
    result.pass = (result.overshoot < targets.overshoot) && ...
                  (result.ss_error < targets.ss_error) && ...
                  (result.settling < targets.settling);
    
    verify_results{i} = result;
    
    if result.pass
        fprintf('✅ PASS\n');
        pass_count = pass_count + 1;
    else
        fprintf('❌ FAIL (OS=%.1f%%, SS=%.1f%%, ST=%.3fs)\n', ...
                result.overshoot, result.ss_error, result.settling);
    end
end

fprintf('\n');

%% ==================== STEP 4: Generate Report ====================
fprintf('=================================================================\n');
fprintf('  STEP 4: 검증 리포트\n');
fprintf('=================================================================\n\n');

fprintf('┌────────────────────┬──────────┬──────────┬──────────┬────────┐\n');
fprintf('│ Scenario           │ Overshoot│ SS Error │ Settling │ Result │\n');
fprintf('├────────────────────┼──────────┼──────────┼──────────┼────────┤\n');

for i = 1:length(scenarios)
    name = scenarios{i}.name;
    r = verify_results{i};
    
    if r.pass
        status = '✅ PASS';
    else
        status = '❌ FAIL';
    end
    
    fprintf('│ %-18s │ %6.2f %% │ %6.2f %% │ %6.3f s │ %s │\n', ...
            name, r.overshoot, r.ss_error, r.settling, status);
end

fprintf('├────────────────────┴──────────┴──────────┴──────────┴────────┤\n');
fprintf('│ 목표: Overshoot < %.0f%%, SS Error < %.0f%%, Settling < %.2fs       │\n', ...
        targets.overshoot, targets.ss_error, targets.settling);
fprintf('└──────────────────────────────────────────────────────────────┘\n');

fprintf('\n총 %d/%d 시나리오 통과 (%.0f%%)\n\n', pass_count, length(scenarios), ...
        pass_count/length(scenarios)*100);

%% ==================== STEP 5: Visualization ====================
fprintf('=================================================================\n');
fprintf('  STEP 5: 결과 시각화\n');
fprintf('=================================================================\n\n');

figure('Name', 'Verification Results', 'Position', [50 50 1400 900]);

% 1. 2단계 최적화 결과 시각화
subplot(2,3,1);
% Coarse grid
scatter(optim_stats.coarse_Kp(:), optim_stats.coarse_Ki(:), 30, optim_stats.coarse_cost(:), 'filled');
hold on;
% Fine grid (크게 표시)
scatter(optim_stats.fine_Kp(:), optim_stats.fine_Ki(:), 80, optim_stats.fine_cost(:), 's', 'filled');
% Best point
plot(best_Kp, best_Ki, 'r*', 'MarkerSize', 20, 'LineWidth', 3);
colorbar;
xlabel('Kp'); ylabel('Ki');
title('2-Stage Grid Search');
legend('Coarse', 'Fine', 'Best', 'Location', 'best');
grid on;

% 2. 오버슈트 비교
subplot(2,3,2);
scenario_names = cellfun(@(s) s.name, scenarios, 'UniformOutput', false);
overshoots = cellfun(@(r) r.overshoot, verify_results);
bar(overshoots);
hold on;
yline(targets.overshoot, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('Overshoot (%)');
title('오버슈트 비교');
grid on;

% 3. 정상상태 오차 비교
subplot(2,3,3);
ss_errors = cellfun(@(r) r.ss_error, verify_results);
bar(ss_errors);
hold on;
yline(targets.ss_error, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('SS Error (%)');
title('정상상태 오차 비교');
grid on;

% 4. 정착시간 비교
subplot(2,3,4);
settlings = cellfun(@(r) r.settling, verify_results);
bar(settlings * 1000);
hold on;
yline(targets.settling * 1000, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('Settling Time (ms)');
title('정착시간 비교');
grid on;

% 5. 전류 응답 비교 (주요 시나리오)
subplot(2,3,5);
colors = lines(3);
plot(verify_results{1}.time * 1000, verify_results{1}.I_hist, 'Color', colors(1,:), 'LineWidth', 1.5);
hold on;
plot(verify_results{2}.time * 1000, verify_results{2}.I_hist, 'Color', colors(2,:), 'LineWidth', 1.5);
plot(verify_results{3}.time * 1000, verify_results{3}.I_hist, 'Color', colors(3,:), 'LineWidth', 1.5);
yline(10, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title('전류 응답 비교');
legend('Nominal', 'Low V', 'High V', 'Target', 'Location', 'best');

% 6. 최적화 수렴 과정
subplot(2,3,6);
plot([optim_stats.coarse_best_cost, optim_stats.fine_best_cost], 'bo-', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
scatter(1, optim_stats.coarse_best_cost, 100, 'b', 'filled');
scatter(2, optim_stats.fine_best_cost, 100, 'r', 'filled');
set(gca, 'XTick', [1, 2], 'XTickLabel', {'Coarse', 'Fine'});
ylabel('Best Cost');
title('최적화 수렴 과정');
grid on;
text(1.1, optim_stats.coarse_best_cost, sprintf('%.1f', optim_stats.coarse_best_cost));
text(2.1, optim_stats.fine_best_cost, sprintf('%.1f', optim_stats.fine_best_cost));

%% ==================== STEP 6: Final Summary ====================
fprintf('=================================================================\n');
fprintf('  최종 요약\n');
fprintf('=================================================================\n');

if pass_count == length(scenarios)
    fprintf('🎉 모든 시나리오 통과! 제어기가 범용적으로 동작합니다.\n');
else
    fprintf('⚠️ %d개 시나리오 실패. 추가 튜닝 필요.\n', length(scenarios) - pass_count);
    fprintf('\n실패한 시나리오:\n');
    for i = 1:length(scenarios)
        if ~verify_results{i}.pass
            fprintf('  - %s\n', scenarios{i}.name);
        end
    end
end

fprintf('\n최종 파라미터:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.1f\n', best_Ki);
fprintf('\n효율성:\n');
fprintf('  기존 Grid Search: %d회 시뮬레이션\n', optim_stats.baseline_sims);
fprintf('  2-Stage Search:   %d회 시뮬레이션 (%.0f%% 절약)\n', ...
        optim_stats.total_sims, (1 - optim_stats.total_sims/optim_stats.baseline_sims)*100);

fprintf('=================================================================\n');
fprintf('  검증 완료\n');
fprintf('=================================================================\n');

%% ========================================================================
%% Helper Functions
%% ========================================================================

function [best_Kp, best_Ki, best_results, stats] = run_two_stage_optimization(config)
    % 2단계 Grid Search 최적화
    
    fprintf('--- Stage 1: Coarse Grid Search ---\n');
    
    tic;
    
    % Parameters
    params.n = config.plant.n;
    params.L = config.plant.L;
    params.fs = config.plant.fs;
    params.C_out = config.plant.C_out;
    params.efficiency = config.plant.efficiency;
    
    V_in = config.hv.V_nom;
    V_batt = config.battery.V_nom;
    R_batt = config.battery.R_internal;
    V_out_init = V_batt;
    Phi_int_deg = config.controller.Phi_int_deg;
    I_ref = config.test.I_ref;
    
    dt = config.sim.dt;
    sim_time = config.sim.time;
    step_start = config.sim.step_start;
    time_vec = 0:dt:sim_time;
    n_steps = length(time_vec);
    
    % ===== Stage 1: Coarse Grid (5x5 = 25 sims) =====
    n_coarse = 5;
    Kp_coarse = linspace(config.optim.Kp_range(1), config.optim.Kp_range(2), n_coarse);
    Ki_coarse = linspace(config.optim.Ki_range(1), config.optim.Ki_range(2), n_coarse);
    
    [Kp_grid_c, Ki_grid_c] = meshgrid(Kp_coarse, Ki_coarse);
    cost_grid_c = zeros(size(Kp_grid_c));
    
    coarse_best_cost = inf;
    coarse_best_Kp = 0;
    coarse_best_Ki = 0;
    
    sim_count = 0;
    
    for i = 1:n_coarse
        for j = 1:n_coarse
            Kp = Kp_coarse(i);
            Ki = Ki_coarse(j);
            
            [cost, ~, ~, ~] = simulate_controller(Kp, Ki, V_in, V_batt, R_batt, V_out_init, ...
                                                   Phi_int_deg, I_ref, dt, time_vec, n_steps, ...
                                                   step_start, sim_time, params, config);
            cost_grid_c(j, i) = cost;
            sim_count = sim_count + 1;
            
            if cost < coarse_best_cost
                coarse_best_cost = cost;
                coarse_best_Kp = Kp;
                coarse_best_Ki = Ki;
            end
        end
    end
    
    fprintf('  Coarse 최적점: Kp=%.2f, Ki=%.0f (Cost=%.1f)\n', ...
            coarse_best_Kp, coarse_best_Ki, coarse_best_cost);
    
    % ===== Stage 2: Fine Grid around best (5x5 = 25 sims) =====
    fprintf('--- Stage 2: Fine Grid Search ---\n');
    
    n_fine = 5;
    % ±20% 범위로 미세 탐색
    Kp_fine_range = [max(config.optim.Kp_range(1), coarse_best_Kp * 0.8), ...
                     min(config.optim.Kp_range(2), coarse_best_Kp * 1.2)];
    Ki_fine_range = [max(config.optim.Ki_range(1), coarse_best_Ki * 0.8), ...
                     min(config.optim.Ki_range(2), coarse_best_Ki * 1.2)];
    
    Kp_fine = linspace(Kp_fine_range(1), Kp_fine_range(2), n_fine);
    Ki_fine = linspace(Ki_fine_range(1), Ki_fine_range(2), n_fine);
    
    [Kp_grid_f, Ki_grid_f] = meshgrid(Kp_fine, Ki_fine);
    cost_grid_f = zeros(size(Kp_grid_f));
    
    best_cost = inf;
    best_Kp = 0;
    best_Ki = 0;
    best_results = struct();
    
    for i = 1:n_fine
        for j = 1:n_fine
            Kp = Kp_fine(i);
            Ki = Ki_fine(j);
            
            [cost, overshoot, ss_error, settling, I_hist] = ...
                simulate_controller(Kp, Ki, V_in, V_batt, R_batt, V_out_init, ...
                                    Phi_int_deg, I_ref, dt, time_vec, n_steps, ...
                                    step_start, sim_time, params, config);
            cost_grid_f(j, i) = cost;
            sim_count = sim_count + 1;
            
            if cost < best_cost
                best_cost = cost;
                best_Kp = Kp;
                best_Ki = Ki;
                best_results.overshoot = overshoot;
                best_results.ss_error = ss_error;
                best_results.settling_time = settling;
                best_results.I_hist = I_hist;
            end
        end
    end
    
    fprintf('  Fine 최적점: Kp=%.3f, Ki=%.1f (Cost=%.1f)\n', best_Kp, best_Ki, best_cost);
    
    elapsed_time = toc;
    
    % Statistics
    baseline_n = config.optim.n_points;
    stats.baseline_sims = baseline_n * baseline_n;  % 기존: 10x10 = 100
    stats.total_sims = sim_count;  % 2-stage: 5x5 + 5x5 = 50
    stats.elapsed_time = elapsed_time;
    stats.coarse_best_cost = coarse_best_cost;
    stats.fine_best_cost = best_cost;
    
    % For visualization
    stats.coarse_Kp = Kp_grid_c;
    stats.coarse_Ki = Ki_grid_c;
    stats.coarse_cost = cost_grid_c;
    stats.fine_Kp = Kp_grid_f;
    stats.fine_Ki = Ki_grid_f;
    stats.fine_cost = cost_grid_f;
    
    % Update files
    update_controller_file(best_Kp, best_Ki);
    update_config_file(best_Kp, best_Ki);
end

function [cost, overshoot, ss_error, settling, I_hist] = simulate_controller(Kp, Ki, V_in, V_batt, R_batt, V_out_init, Phi_int_deg, I_ref, dt, time_vec, n_steps, step_start, sim_time, params, config)
    % 단일 시뮬레이션 실행
    
    V_out = V_out_init;
    integrator = 0;
    I_hist = zeros(n_steps, 1);
    
    for k = 1:n_steps
        I_fb = (V_out - V_batt) / R_batt;
        
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
                                                 Phi_int_deg, R_batt, V_batt, dt, params);
        V_out = V_out_next;
        I_hist(k) = I_fb;
    end
    
    % Performance
    step_idx = find(time_vec >= step_start, 1);
    ss_idx = floor(0.85 * n_steps);
    
    I_ss = mean(I_hist(ss_idx:end));
    I_peak = max(I_hist(step_idx:end));
    
    overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
    ss_error = abs(I_ss - I_ref) / I_ref * 100;
    
    settling = sim_time - step_start;
    for sk = step_idx:n_steps
        if all(abs(I_hist(sk:end) - I_ref) < 0.02 * I_ref)
            settling = time_vec(sk) - step_start;
            break;
        end
    end
    
    % Cost
    w_os = config.optim.weight_overshoot;
    w_ss = config.optim.weight_ss_error;
    w_st = config.optim.weight_settling;
    
    penalty = 0;
    if overshoot > config.optim.target_overshoot
        penalty = penalty + (overshoot - config.optim.target_overshoot)^2;
    end
    if ss_error > config.optim.target_ss_error
        penalty = penalty + (ss_error - config.optim.target_ss_error)^2 * 10;
    end
    
    cost = w_os * overshoot + w_ss * ss_error + w_st * (settling * 100) + penalty;
end

function result = verify_scenario(scenario, Kp, Ki, config)
    % 특정 시나리오에서 제어기 검증
    
    params.n = config.plant.n;
    params.L = config.plant.L;
    params.fs = config.plant.fs;
    params.C_out = config.plant.C_out;
    params.efficiency = config.plant.efficiency;
    
    V_in = scenario.V_in;
    V_batt = scenario.V_batt;
    I_ref = scenario.I_ref;
    R_batt = config.battery.R_internal;
    Phi_int_deg = config.controller.Phi_int_deg;
    
    dt = config.sim.dt;
    sim_time = config.sim.time;
    step_start = config.sim.step_start;
    time_vec = 0:dt:sim_time;
    n_steps = length(time_vec);
    
    V_out = V_batt;
    integrator = 0;
    I_hist = zeros(n_steps, 1);
    
    for k = 1:n_steps
        I_fb = (V_out - V_batt) / R_batt;
        
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
                                                 Phi_int_deg, R_batt, V_batt, dt, params);
        V_out = V_out_next;
        I_hist(k) = I_fb;
    end
    
    % Performance
    step_idx = find(time_vec >= step_start, 1);
    ss_idx = floor(0.85 * n_steps);
    
    I_ss = mean(I_hist(ss_idx:end));
    I_peak = max(I_hist(step_idx:end));
    
    result.overshoot = max(0, (I_peak - I_ref) / I_ref * 100);
    result.ss_error = abs(I_ss - I_ref) / I_ref * 100;
    
    result.settling = sim_time - step_start;
    for sk = step_idx:n_steps
        if all(abs(I_hist(sk:end) - I_ref) < 0.02 * I_ref)
            result.settling = time_vec(sk) - step_start;
            break;
        end
    end
    
    result.time = time_vec;
    result.I_hist = I_hist;
end

function update_controller_file(Kp, Ki)
    controller_file = fullfile('..', 'DAB_Controller_System.m');
    
    try
        fid = fopen(controller_file, 'r');
        content = fread(fid, '*char')';
        fclose(fid);
        
        content = regexprep(content, '(Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', Kp));
        content = regexprep(content, '(Ki\s*=\s*)[\d.]+', sprintf('$1%.1f', Ki));
        
        fid = fopen(controller_file, 'w');
        fwrite(fid, content, 'char');
        fclose(fid);
    catch
    end
end

function update_config_file(Kp, Ki)
    config_file = fullfile('..', 'DAB_System_Config.m');
    
    try
        fid = fopen(config_file, 'r');
        content = fread(fid, '*char')';
        fclose(fid);
        
        content = regexprep(content, '(config\.controller\.Kp\s*=\s*)[\d.]+', sprintf('$1%.3f', Kp));
        content = regexprep(content, '(config\.controller\.Ki\s*=\s*)[\d.]+', sprintf('$1%.1f', Ki));
        
        fid = fopen(config_file, 'w');
        fwrite(fid, content, 'char');
        fclose(fid);
    catch
    end
end
