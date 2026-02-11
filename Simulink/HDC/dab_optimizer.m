function [opt_x, min_cost] = dab_optimizer(Vin, Vout, Pref, params)
    % 1. Robust Objective Function 정의
    % 세 가지 시나리오의 가중 합 (Nominal 70%, L-Drift 15%, Aging 15%)
    objective = @(x) ...
        0.70 * calculate_cost(x, Vin, Vout, Pref, params.L_nom, params) + ...
        0.15 * calculate_cost(x, Vin, Vout, Pref, params.L_nom*(1-params.L_tol), params) + ...
        0.15 * calculate_cost(x, Vin, Vout, Pref, params.L_nom*(1+params.L_tol), params);

    % 2. Constraints (Power Balance & Safety)
    % Optimized to use the efficient physics_engine with power return
    nonlcon = @(x) dab_constraints(x, Vin, Vout, Pref, params);
    
    % 3. MultiStart Optimization
    problem = createOptimProblem('fmincon', ...
        'objective', objective, ...
        'x0', [0.2, 0.2, 0.1], ...
        'lb', [0, 0, -0.5], 'ub', [0.5, 0.5, 0.5], ...
        'nonlcon', nonlcon, ...
        'options', optimoptions('fmincon','Display','off','Algorithm','sqp', ...
        'FiniteDifferenceStepSize', 0.02, 'StepTolerance', 1e-4));
        
    ms = MultiStart('Display','off');
    % 핵심: StartPoints를 늘려(10회) 전역 탐색 능력 강화
    [opt_x, min_cost] = run(ms, problem, 10); 
end

function J = calculate_cost(x, Vin, Vout, Pref, L_val, params)
    [Loss, ~, ~, ~, ~] = physics_engine(x, Vin, Vout, L_val, params);
    J = Loss; 
end

function [c, ceq] = dab_constraints(x, Vin, Vout, Pref, params)
    % Power Balance & Peak Current
    % Efficiently get all data from physics_engine
    [~, I_peak, ~, ~, P_calc] = physics_engine(x, Vin, Vout, params.L_nom, params);
    
    ceq = P_calc - Pref;
    c = I_peak - params.I_max;
end
