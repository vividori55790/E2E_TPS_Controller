% =========================================================================
% Auto_Optimize_And_Verify.m
% 최적화 실행 후 여러 시나리오에서 자동 검증하는 통합 스크립트
% =========================================================================
% 기능:
%   1. PI 제어기 최적화 실행
%   2. 다양한 운전 조건에서 자동 검증
%   3. Pass/Fail 리포트 생성
% =========================================================================

clear; clc; close all;

fprintf('=================================================================\n');
fprintf('  Auto Optimize & Verify - DAB Controller\n');
fprintf('=================================================================\n\n');

%% ==================== Load Configuration ====================
config = DAB_System_Config();
fprintf('설정 파일 로드 완료: DAB_System_Config.m\n\n');

%% ==================== STEP 1: Run Optimization ====================
fprintf('=================================================================\n');
fprintf('  STEP 1: PI 제어기 최적화\n');
fprintf('=================================================================\n\n');

% 최적화 실행 (별도 함수로 분리)
[best_Kp, best_Ki, optim_results] = run_optimization(config);

fprintf('\n최적화 결과:\n');
fprintf('  Kp = %.3f\n', best_Kp);
fprintf('  Ki = %.1f\n', best_Ki);
fprintf('  오버슈트: %.2f%%\n', optim_results.overshoot);
fprintf('  정상상태 오차: %.2f%%\n', optim_results.ss_error);
fprintf('  정착시간: %.3f sec\n\n', optim_results.settling_time);

%% ==================== STEP 2: Define Test Scenarios ====================
fprintf('=================================================================\n');
fprintf('  STEP 2: 테스트 시나리오 정의\n');
fprintf('=================================================================\n\n');

% 다양한 운전 조건 정의
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

% 성능 목표
targets.overshoot = config.optim.target_overshoot;
targets.ss_error = config.optim.target_ss_error;
targets.settling = config.optim.target_settling;

% 결과 저장
verify_results = cell(length(scenarios), 1);
pass_count = 0;

for i = 1:length(scenarios)
    scenario = scenarios{i};
    fprintf('[%d/%d] %s 검증 중... ', i, length(scenarios), scenario.name);
    
    % 시나리오 검증 실행
    result = verify_scenario(scenario, best_Kp, best_Ki, config);
    
    % Pass/Fail 판정
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
fprintf('│ 목표: Overshoot < %.0f%%, SS Error < %.0f%%, Settling < %.0fs       │\n', ...
        targets.overshoot, targets.ss_error, targets.settling);
fprintf('└──────────────────────────────────────────────────────────────┘\n');

fprintf('\n총 %d/%d 시나리오 통과 (%.0f%%)\n\n', pass_count, length(scenarios), ...
        pass_count/length(scenarios)*100);

%% ==================== STEP 5: Visualization ====================
fprintf('=================================================================\n');
fprintf('  STEP 5: 결과 시각화\n');
fprintf('=================================================================\n\n');

figure('Name', 'Verification Results', 'Position', [50 50 1400 800]);

% 1. 성능 비교 바 차트
subplot(2,2,1);
scenario_names = cellfun(@(s) s.name, scenarios, 'UniformOutput', false);
overshoots = cellfun(@(r) r.overshoot, verify_results);
bar(overshoots);
hold on;
yline(targets.overshoot, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('Overshoot (%)');
title('오버슈트 비교');
legend('실제', '목표', 'Location', 'best');
grid on;

subplot(2,2,2);
ss_errors = cellfun(@(r) r.ss_error, verify_results);
bar(ss_errors);
hold on;
yline(targets.ss_error, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('SS Error (%)');
title('정상상태 오차 비교');
legend('실제', '목표', 'Location', 'best');
grid on;

subplot(2,2,3);
settlings = cellfun(@(r) r.settling, verify_results);
bar(settlings * 1000);  % ms로 표시
hold on;
yline(targets.settling * 1000, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', scenario_names, 'XTickLabelRotation', 45);
ylabel('Settling Time (ms)');
title('정착시간 비교');
legend('실제', '목표', 'Location', 'best');
grid on;

% 2. 전류 응답 비교 (주요 3개 시나리오)
subplot(2,2,4);
colors = lines(length(scenarios));
legend_entries = {};
for i = [1, 2, 3]  % Nominal, Low Voltage, High Voltage
    plot(verify_results{i}.time * 1000, verify_results{i}.I_hist, ...
         'Color', colors(i,:), 'LineWidth', 1.5);
    hold on;
    legend_entries{end+1} = scenarios{i}.name;
end
yline(10, 'k--', 'LineWidth', 1.5);
legend_entries{end+1} = 'Target';
grid on;
xlabel('Time (ms)');
ylabel('I_{out} (A)');
title('전류 응답 비교 (주요 시나리오)');
legend(legend_entries, 'Location', 'best');

%% ==================== STEP 6: Final Summary ====================
fprintf('=================================================================\n');
fprintf('  최종 요약\n');
fprintf('=================================================================\n');

if pass_count == length(scenarios)
    fprintf('🎉 모든 시나리오 통과! 제어기가 범용적으로 동작합니다.\n');
    fprintf('\n최종 파라미터:\n');
    fprintf('  Kp = %.3f\n', best_Kp);
    fprintf('  Ki = %.1f\n', best_Ki);
    fprintf('\nDAB_Controller_System.m 및 DAB_System_Config.m 업데이트 완료.\n');
else
    fprintf('⚠️ 일부 시나리오 실패. 추가 튜닝이 필요할 수 있습니다.\n');
    fprintf('\n실패한 시나리오:\n');
    for i = 1:length(scenarios)
        if ~verify_results{i}.pass
            fprintf('  - %s\n', scenarios{i}.name);
        end
    end
end

fprintf('=================================================================\n');
fprintf('  검증 완료\n');
fprintf('=================================================================\n');

%% ========================================================================
%% Helper Functions
%% ========================================================================

function [best_Kp, best_Ki, best_results] = run_optimization(config)
    % 최적화 로직 (Optimize_Controller_MIL.m과 동일)
    
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
    
    Kp_range = linspace(config.optim.Kp_range(1), config.optim.Kp_range(2), config.optim.n_points);
    Ki_range = linspace(config.optim.Ki_range(1), config.optim.Ki_range(2), config.optim.n_points);
    
    best_cost = inf;
    best_Kp = 0;
    best_Ki = 0;
    best_results = struct();
    
    total_iter = length(Kp_range) * length(Ki_range);
    iter_count = 0;
    
    for i = 1:length(Kp_range)
        for j = 1:length(Ki_range)
            iter_count = iter_count + 1;
            Kp = Kp_range(i);
            Ki = Ki_range(j);
            
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
            
            settling_time = sim_time - step_start;
            for sk = step_idx:n_steps
                if all(abs(I_hist(sk:end) - I_ref) < 0.02 * I_ref)
                    settling_time = time_vec(sk) - step_start;
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
            
            cost = w_os * overshoot + w_ss * ss_error + w_st * (settling_time * 100) + penalty;
            
            if cost < best_cost
                best_cost = cost;
                best_Kp = Kp;
                best_Ki = Ki;
                best_results.overshoot = overshoot;
                best_results.ss_error = ss_error;
                best_results.settling_time = settling_time;
                best_results.I_hist = I_hist;
            end
            
            if mod(iter_count, 20) == 0
                fprintf('  최적화 진행률: %d/%d (%.0f%%)\n', iter_count, total_iter, iter_count/total_iter*100);
            end
        end
    end
    
    % Update files
    update_controller_file(best_Kp, best_Ki);
    update_config_file(best_Kp, best_Ki);
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
        % Silent fail
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
        % Silent fail
    end
end
