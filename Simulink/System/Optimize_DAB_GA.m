% DAB_GA_Optimization.m
% DAB 컨버터 제어 파라미터 유전 알고리즘 최적화 스크립트
% 최적화 대상: Kp_ext, Ki_ext (전류 제어), Kpv, Kiv (전압 제어)

clear; clc; close all;

% 1. 시나리오 초기화 (시간 벡터 및 기본 변수 생성)
run('Scripts/Create_Scenario.m');

% 2. GA 설정 (주석 처리됨 - 데이터 확인용)
% pop_size = 10;
% max_gen = 5;
% num_vars = 4;
% lb = [0.1,  1.0,  -20.0, -2000.0];
% ub = [20.0, 500.0, -0.1,  -10.0];
% options = optimoptions('ga', 'PopulationSize', pop_size, 'MaxGenerations', max_gen, 'Display', 'iter', 'PlotFcn', {@gaplotbestf}, 'UseParallel', false);

fprintf('>>> 시뮬레이션 데이터 검증 모드 시작...\n');
fprintf('    Model: Control_Logic.slx\n');
fprintf('    Time: 0.5s\n');

% 3. GA 실행 (주석 처리됨)
objective = @(x) dab_cost_function(x);
% [x_opt, fval] = ga(objective, num_vars, [], [], [], [], lb, ub, [], options);

% 테스트용 기본 파라미터 적용
default_params = [2.0, 30.0, -4.0, -400.0];
update_workspace_params(default_params);

% 단일 시뮬레이션 실행 확인
try
    fprintf('    Simulating with default parameters...\n');
    cost = dab_cost_function(default_params);
    fprintf('    Simulation Successful! Cost: %.4f\n', cost);
    
    % 결과 플롯 (simOut을 얻기 위해 다시 실행하거나 cost function 내부 수정 필요하지만, 
    % 여기서는 간단히 cost function이 에러 없이 도는지 확인하고, 
    % 실제 데이터를 보려면 sim 명령어를 직접 호출)
    simOut = sim('Control_Logic', 'ReturnWorkspaceOutputs', 'on');
    plot_results(simOut);
    
catch e
    fprintf('!!! Simulation Failed: %s\n', e.message);
end

% 4. 최적 결과 출력 및 적용 (주석 처리됨)
% fprintf('\n>>> 최적화 완료! \n');
% fprintf('    Best Cost: %.4f\n', fval);
% update_workspace_params(x_opt);
% sim_out = sim('Control_Logic');
% plot_results(sim_out);


% =========================================================================
% Helper Functions
% =========================================================================

function cost = dab_cost_function(params)
    % 파라미터 Workspace 업데이트
    update_workspace_params(params);
    
    try
        % 시뮬레이션 실행
        model = 'Control_Logic';
        simOut = sim(model, 'ReturnWorkspaceOutputs', 'on');
        
        % 데이터 추출
        % 전류: ESS 모드 추종 성능
        data_I = simOut.Current_Low; 
        t = data_I.Time;
        y_I = data_I.Data;
        
        % 전압: UPS 모드 유지 성능
        data_V = simOut.Voltage_High;
        y_V = data_V.Data;
        
        % --- Cost 1: Current Control (ESS Charge) ---
        % Target: 10A, Interval: 0.12s ~ 0.19s (Steady State)
        idx_c_ss = t >= 0.12 & t <= 0.19; 
        idx_c_tr = t >= 0.10 & t <= 0.12; % Transient
        
        % Steady State Error (MSE)
        err_c_ss = y_I(idx_c_ss) - 10;
        mse_c = mean(err_c_ss.^2);
        
        % Overshoot (Max excursion above 10A + margin)
        % 과도 응답 구간 포함해서 최대값 체크
        idx_c_all = t >= 0.10 & t <= 0.20;
        max_I_charge = max(y_I(idx_c_all));
        overshoot_c = max(0, max_I_charge - 10.5); % 0.5A 마진 초과분 페널티
        
        % --- Cost 2: Current Control (ESS Discharge) ---
        % Target: -10A, Interval: 0.22s ~ 0.29s (Steady State)
        idx_d_ss = t >= 0.22 & t <= 0.29;
        
        err_d_ss = y_I(idx_d_ss) - (-10);
        mse_d = mean(err_d_ss.^2);
        
        % Overshoot (Max excursion below -10A - margin)
        idx_d_all = t >= 0.20 & t <= 0.30;
        min_I_discharge = min(y_I(idx_d_all));
        overshoot_d = max(0, (-10.5) - min_I_discharge); % -10.5A 미만 페널티
        
        % --- Cost 3: Voltage Control (UPS Mode) ---
        % Target: 380V, Interval: 0.35s ~ 0.44s (Steady State)
        % 중요: 전압 제어는 매우 정밀해야 함
        idx_u_ss = t >= 0.35 & t <= 0.44;
        
        err_u_ss = y_V(idx_u_ss) - 380;
        mse_u = mean(err_u_ss.^2);
        
        % Overshoot/Undershoot (Voltage Dip/Swell)
        % UPS 진입/탈출 과도구간(0.30~0.35)에서의 전압 변동폭 억제
        idx_u_tr = t >= 0.30 & t <= 0.35;
        v_tr = y_V(idx_u_tr);
        % 목표 380V 기준 ±5% (361~399V) 벗어나면 페널티
        undershoot_v = max(0, 370 - min(v_tr)); % 370V 이하로 떨어지는 것 방지 (Dip 최소화)
        overshoot_v  = max(0, max(v_tr) - 390); % 390V 이상 튀는 것 방지
        
        % --- Total Cost Calculation ---
        % 가중치 설정 (전압 제어 중요도 높음)
        w_i_mse = 1.0;
        w_i_os  = 10.0; % 전류 오버슈트 페널티
        
        w_v_mse = 20.0; % 전압 정밀도 매우 중요
        w_v_os  = 5.0;  % 전압 Dip/Swell 페널티
        
        cost_current = w_i_mse * (mse_c + mse_d) + w_i_os * (overshoot_c + overshoot_d);
        cost_voltage = w_v_mse * mse_u + w_v_os * (undershoot_v + overshoot_v);
        
        cost = cost_current + cost_voltage;
        
        % NaN 또는 Inf 체크
        if isnan(cost) || isinf(cost)
            cost = 1e9;
        end
        
        % 상세 로깅 (디버깅용, iter가 많으면 주석 처리)
        % fprintf('Cost: %.4f (I_MSE: %.4f, I_OS: %.4f, V_MSE: %.4f, V_OS: %.4f)\n', ...
        %     cost, (mse_c+mse_d), (overshoot_c+overshoot_d), mse_u, (undershoot_v+overshoot_v));
        
    catch e
        fprintf('Error in simulation: %s\n', e.message);
        cost = 1e9;
    end
end

function update_workspace_params(params)
    % Base Workspace의 제어 변수(timeseries) 업데이트
    % params: [Kp_ext, Ki_ext, Kpv, Kiv]
    
    % 기존 시간 벡터 가져오기
    t = evalin('base', 'time'); 
    n = length(t);
    
    Kp_val = params(1);
    Ki_val = params(2);
    Kpv_val = params(3);
    Kiv_val = params(4);
    
    % 상수 벡터 생성
    Kp_vec = Kp_val * ones(n, 1);
    Ki_vec = Ki_val * ones(n, 1);
    Kpv_vec = Kpv_val * ones(n, 1);
    Kiv_vec = Kiv_val * ones(n, 1);
    
    % Timeseries 생성
    ts_Kp = timeseries(Kp_vec, t);
    ts_Ki = timeseries(Ki_vec, t);
    ts_Kpv = timeseries(Kpv_vec, t);
    ts_Kiv = timeseries(Kiv_vec, t);
    
    % Workspace 할당
    assignin('base', 'Ref_Kp_ext', ts_Kp);
    assignin('base', 'Ref_Ki_ext', ts_Ki);
    assignin('base', 'Ref_Kpv', ts_Kpv);
    assignin('base', 'Ref_Kiv', ts_Kiv);
end

function plot_results(simOut)
    figure('Name', 'Optimized Simulation Results', 'Color', 'w');
    
    t = simOut.tout;
    
    subplot(2,1,1);
    plot(t, simOut.Current_Low.Data, 'LineWidth', 1.5);
    grid on;
    title('Low Voltage Current (ESS Mode)');
    ylabel('Current [A]');
    xlabel('Time [s]');
    xline([0.1, 0.2, 0.3], '--k');
    legend('I_{LV}', 'Events');
    
    subplot(2,1,2);
    plot(t, simOut.Voltage_High.Data, 'LineWidth', 1.5);
    grid on;
    title('High Voltage (UPS Mode)');
    ylabel('Voltage [V]');
    xlabel('Time [s]');
    yline(380, 'r--', 'Target 380V');
    xline([0.3, 0.45], '--k');
end
