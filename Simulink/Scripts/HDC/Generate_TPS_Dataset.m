
% Generate_TPS_Dataset.m
% Generates an optimal control dataset for a DAB converter using TPS (Triple Phase Shift).
% Objective: Minimize TOTAL LOSS (Conduction + Switching).
%
% Algorithm: EXACT ANALYTICAL + WARM START SEQUENTIAL OPTIMIZATION (PARALLELIZED)
% - Uses Warm Start to ensure continuity of control variables (Smooth Maps) for each Voltage Pair.
% - Uses Parallel Processing (parfor) to compute different Voltage Pairs simultaneously.
% - Exact Analytical solution for zero numerical error.
%
% Generates:
%   Inputs: V_in, V_out, Power_Ref
%   Outputs: D1, D2, Phi, Loss, ZVS_Flag, k_ratio, P_pu

clear; clc;

% =========================================================================
% 1. System Parameters (High Fidelity)
% =========================================================================
L = 35e-6;       % 35uH (Optimized for 1kW Power Transfer, Max ~2kW)
fs = 80e3;        
n = 1;           
R_total = 0.2;        
t_fall  = 100e-9;     
I_min_zvs = 1.0;      

% Operating Ranges
V_in_Range  = linspace(320, 420, 10);   
V_out_Range = linspace(80, 120, 10);    
P_Steps = 0:50:1200; % Optimized for 1kW Target (Steps of 50W)

P_Range_Pos = P_Steps;
P_Range_Neg = -fliplr(P_Steps(2:end)); % -5000 ... -250

% Compute Grid for Parallel Execution
[V1_Grid, V2_Grid] = ndgrid(V_in_Range, V_out_Range);
Tasks_V1 = V1_Grid(:);
Tasks_V2 = V2_Grid(:);
Num_Tasks = length(Tasks_V1);
Num_Points_Per_Task = length(P_Range_Pos) + length(P_Range_Neg);
Total_Scenarios = Num_Tasks * Num_Points_Per_Task;

fprintf('Generating Continuous TPS Optimization Dataset (Parallel + Warm Start)...\n');
fprintf('Total Operating Voltage Pairs: %d\n', Num_Tasks);
fprintf('Total Scenarios: %d\n', Total_Scenarios);

% Prepare Output Container
Results_Cell = cell(Num_Tasks, 1);

tic;

% Check for Parallel Pool
if isempty(gcp('nocreate'))
    try
        parpool; 
    catch
        fprintf('Warning: Could not start parallel pool. Running sequentially.\n');
    end
end

% Parallel Loop over Voltage Conditions
parfor i = 1:Num_Tasks
    v1_val = Tasks_V1(i);
    v2_val = Tasks_V2(i);
    
    % Solver Options (Local to worker)
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp', ...
        'StepTolerance', 1e-6, 'ConstraintTolerance', 1e-6);
    
    % --- Forward Sweep (Positive Power) ---
    res_pos = run_power_sweep(v1_val, v2_val, P_Range_Pos, options, L, fs, R_total, t_fall, n, 1);
    
    % --- Backward Sweep (Negative Power) ---
    res_neg = run_power_sweep(v1_val, v2_val, P_Range_Neg, options, L, fs, R_total, t_fall, n, -1);
    
    % Combine Results for this Voltage Pair
    Results_Cell{i} = [res_pos; res_neg];
    
    % Simple Progress Indicator (Note: Order is not guaranteed in console)
    if mod(i, 5) == 0
       fprintf('Completed Task %d / %d (V1=%.1f, V2=%.1f)\n', i, Num_Tasks, v1_val, v2_val);
    end
end

% Aggregate Data
Dataset = vertcat(Results_Cell{:});
Dataset = sortrows(Dataset, [1, 2, 3]); % Sort by Vin, Vout, Power

elapsed = toc;
fprintf('Dataset Generation Complete in %.2f seconds.\n', elapsed);

% Save Data
% Gets the directory where this script is located
CurrentScriptDir = fileparts(mfilename('fullpath'));
if isempty(CurrentScriptDir)
    % Fallback if running line-by-line without saving (unlikely)
    CurrentScriptDir = pwd;
end

SaveDir = fullfile(CurrentScriptDir, 'Data');
if ~exist(SaveDir, 'dir')
    [status, msg, msgID] = mkdir(SaveDir);
    if status == 0
        error('Error creating directory "%s": %s', SaveDir, msg);
    end
end

savePath = fullfile(SaveDir, 'TPS_Optimal_Dataset.mat');
save(savePath, 'Dataset', 'L', 'fs', 'R_total', 't_fall');
fprintf('Saved to: %s\n', savePath);

% =========================================================================
% Visualization
% =========================================================================
if ~isempty(Dataset)
    figure(101); clf;
    % Plot Loss Map
    subplot(1,2,1);
    scatter3(Dataset(:,3), Dataset(:,2), Dataset(:,7), 15, Dataset(:,7), 'filled');
    xlabel('Power (W)'); ylabel('Vout (V)'); zlabel('Loss (W)');
    title('TPS Optimization: Loss Map');
    colorbar; view(45, 30); grid on;
    
    % Plot D1 Control Map (Example)
    subplot(1,2,2);
    scatter3(Dataset(:,3), Dataset(:,2), Dataset(:,4), 15, Dataset(:,4), 'filled');
    xlabel('Power (W)'); ylabel('Vout (V)'); zlabel('D1 (Primary Duty)');
    title('Control Map: D1');
    view(45, 30); grid on;
    
    drawnow;
end

% =========================================================================
% Local Functions
% =========================================================================

function data_chunk = run_power_sweep(v1, v2, P_range, options, L, fs, R, t_fall, n, direction)
    % Performs a sequential power sweep with Warm Start
    % direction: 1 for forward (positive), -1 for backward (negative start)
    
    num_pts = length(P_range);
    data_chunk = zeros(num_pts, 10);
    
    % Theoretical Max Power (Updated based on user feedback: 4x standard SPS estimate)
    % P_max = (4 * n * V1 * V2) / (8 * fs * L) = (n * V1 * V2) / (2 * fs * L)
    P_max_theoretical = (n*v1*v2) / (2*fs*L);
    
    lb = [0, 0, -0.25];
    ub = [0.5, 0.5, 0.25];
    
    % Initial Guess
    if direction > 0
        x_init = [0.01, 0.01, 0.01];
    else
        x_init = [0.01, 0.01, -0.01];
    end
    
    for k = 1:num_pts
        p_val = P_range(k);
        
        % Check theoretical limit
        if abs(p_val) > P_max_theoretical * 1.15
             data_chunk(k, :) = [v1, v2, p_val, nan, nan, nan, nan, 0, v1/(n*v2), p_val/P_max_theoretical];
             continue;
        end
        
        % Zero Power Handling
        if abs(p_val) < 1e-3
            data_chunk(k, :) = [v1, v2, p_val, 0, 0, 0, 0, 1, v1/(n*v2), 0];
            % Nudge for next step
            if direction > 0, x_init = [0.02, 0.02, 0.01]; else, x_init = [0.02, 0.02, -0.01]; end
            continue;
        end
        
        % Define Objective and Constraints
        ObjFcn = @(x) calculate_TotalLoss_Analytical(x, v1, v2, L, fs, R, t_fall);
        ConFcn = @(x) constraints_Analytical(x, v1, v2, L, fs, p_val);
        
        % Use Warm Start x_init
        x0 = max(lb, min(ub, x_init)); 
        
        try
            [x_opt, min_loss] = fmincon(ObjFcn, x0, [], [], [], [], lb, ub, ConFcn, options);
            [~, ~, zvs_flags] = calculate_Metrics_Exact(x_opt, v1, v2, L, fs);
            
            data_chunk(k, :) = [v1, v2, p_val, x_opt(1), x_opt(2), x_opt(3), min_loss, all(zvs_flags), v1/(n*v2), p_val/P_max_theoretical];
            
            % Update Warm Start
            x_init = x_opt; 
        catch
            % Optimization Failed
            data_chunk(k, :) = [v1, v2, p_val, nan, nan, nan, nan, 0, 0, 0];
        end
    end
end

function TotalCost = calculate_TotalLoss_Analytical(x, V1, V2, L, fs, R_path, t_fall)
    [I_rms, I_off_p, I_off_s, I_on_p, I_on_s] = calculate_Metrics_Exact(x, V1, V2, L, fs);
    P_cond = (I_rms^2) * R_path;
    P_off_pri = 4 * (0.5 * V1 * abs(I_off_p) * t_fall * fs);
    P_off_sec = 4 * (0.5 * V2 * abs(I_off_s) * t_fall * fs);
    
    P_hard_on = 0;
    % Soft Penalty for Hard Switching (High weight)
    if I_on_p > 0, P_hard_on = P_hard_on + 0.5 * 1e-9 * V1^2 * fs * 1000; end
    if I_on_s < 0, P_hard_on = P_hard_on + 0.5 * 1e-9 * V2^2 * fs * 1000; end
    
    TotalCost = P_cond + P_off_pri + P_off_sec + P_hard_on;
end

function [I_rms, I_off_p, I_off_s, I_on_p, I_on_s, zvs_flags] = calculate_Metrics_Exact(x, V1, V2, L, fs)
    d1 = x(1); d2 = x(2); phi = x(3);
    T = 1/fs;
    half_w1 = (0.5 - d1)/2 * T;
    half_w2 = (0.5 - d2)/2 * T;
    shift_s = phi * T;
    
    t_ev = unique(sort([0; 0.25*T - half_w1; 0.25*T + half_w1; ... 
            0.75*T - half_w1; 0.75*T + half_w1; ...
            mod_t(0.25*T - half_w2 + shift_s, T); mod_t(0.25*T + half_w2 + shift_s, T); ...
            mod_t(0.75*T - half_w2 + shift_s, T); mod_t(0.75*T + half_w2 + shift_s, T); T]));
             
    tol = 1e-10; t_ev([false; diff(t_ev) < tol]) = [];
    
    % Analytical Solving
    t_mid = (t_ev(1:end-1) + t_ev(2:end)) / 2;
    vp_vec = get_Volt_Levels(t_mid, d1, 0, V1, fs);
    vs_vec = get_Volt_Levels(t_mid, d2, phi, V2, fs);
    slopes = (vp_vec - vs_vec) / L;
    dt_vec = diff(t_ev);
    i_shape = [0; cumsum(slopes .* dt_vec)];
    i_mid = (i_shape(1:end-1) + i_shape(2:end)) / 2;
    i0 = -sum(i_mid .* dt_vec) / T;
    i_vals = i_shape + i0;
    
    sum_sq = 0;
    for k = 1:length(dt_vec)
        sum_sq = sum_sq + (dt_vec(k)/3) * (i_vals(k)^2 + i_vals(k)*i_vals(k+1) + i_vals(k+1)^2);
    end
    I_rms = sqrt(sum_sq / T);
    
    t_p_on  = 0.25*T - half_w1; t_p_off = 0.25*T + half_w1;
    t_s_on  = mod_t(0.25*T - half_w2 + shift_s, T); t_s_off = mod_t(0.25*T + half_w2 + shift_s, T);
    
    I_on_p  = interp_circular(t_ev, i_vals, t_p_on, T);
    I_off_p = interp_circular(t_ev, i_vals, t_p_off, T);
    I_on_s  = interp_circular(t_ev, i_vals, t_s_on, T);
    I_off_s = interp_circular(t_ev, i_vals, t_s_off, T);
    
    zvs_flags = [(I_on_p < -0.5), (I_on_s > 0.5)];
end

function [c, ceq] = constraints_Analytical(x, V1, V2, L, fs, P_ref)
    d1 = x(1); d2 = x(2); phi = x(3); T=1/fs;
    % Identical vertex logic as above for consistency
    half_w1 = (0.5 - d1)/2 * T; half_w2 = (0.5 - d2)/2 * T; shift_s = phi * T;
    t_ev = unique(sort([0; 0.25*T - half_w1; 0.25*T + half_w1; 0.75*T - half_w1; 0.75*T + half_w1; ...
            mod_t(0.25*T - half_w2 + shift_s, T); mod_t(0.25*T + half_w2 + shift_s, T); ...
            mod_t(0.75*T - half_w2 + shift_s, T); mod_t(0.75*T + half_w2 + shift_s, T); T]));
    t_ev([false; diff(t_ev) < 1e-10]) = [];
    
    t_mid = (t_ev(1:end-1) + t_ev(2:end)) / 2;
    vp_vec = get_Volt_Levels(t_mid, d1, 0, V1, fs);
    vs_vec = get_Volt_Levels(t_mid, d2, phi, V2, fs);
    slopes = (vp_vec - vs_vec) / L; dt_vec = diff(t_ev);
    i_shape = [0; cumsum(slopes .* dt_vec)];
    i0 = -sum( ((i_shape(1:end-1)+i_shape(2:end))/2) .* dt_vec ) / T;
    
    E_sum = 0;
    for k = 1:length(dt_vec)
        i_avg = (i_shape(k) + i_shape(k+1))/2 + i0;
        E_sum = E_sum + vp_vec(k) * i_avg * dt_vec(k);
    end
    ceq = (E_sum * fs - P_ref)/1000; c=[];
end

function val_vec = get_Volt_Levels(t_vec, D, phi, V_dc, fs)
    T = 1/fs; phase = mod(t_vec/T - phi, 1); half_active = (0.5 - D)/2;
    val_vec = zeros(size(t_vec));
    mask_p = (phase >= (0.25 - half_active)) & (phase < (0.25 + half_active));
    val_vec(mask_p) = V_dc;
    mask_n = (phase >= (0.75 - half_active)) & (phase < (0.75 + half_active));
    val_vec(mask_n) = -V_dc;
end
function t = mod_t(t_in, T), t = mod(t_in, T); t(t < 0) = t + T; end
function val = interp_circular(t_node, i_node, t_query, T)
    t_query = mod(t_query, T);
    idx = find(t_node <= t_query + 1e-12, 1, 'last');
    if isempty(idx), idx=1; end; if idx==length(t_node), idx=length(t_node)-1; end
    mu = (t_query - t_node(idx)) / (t_node(idx+1) - t_node(idx));
    val = i_node(idx) + mu*(i_node(idx+1) - i_node(idx));
end
