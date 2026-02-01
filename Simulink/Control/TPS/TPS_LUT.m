% =========================================================
% [Modified] DAB TPS Optimization Lookup Table Generator
% Saves to 'TPS_Table.mat' for Controller usage.
% Uses rigorous Steady-State Time-Domain Analysis for Power Loop.
% =========================================================

clear; clc;

% --- 1. Parameters ---
n = 4.0;
L = 50e-6;
fs = 80e3; 
Ts = 1/fs;
V1_nom = 400; 
V2_nom = 100;

V_base = V1_nom;
P_base = 5000;

% V1 Range: 300 to 450
V1_range = [300, 350, 380, 400, 420, 450]; 
V2_fixed = 100; 
% Power Range: -5kW to +5kW
P_range = linspace(-5000, 5000, 21);

% Prepare Data Arrays
[V1_grid, P_grid] = meshgrid(V1_range, P_range);
D1_map = zeros(size(V1_grid));
D2_map = zeros(size(V1_grid));
D3_map = zeros(size(V1_grid));

fprintf('Generating High-Fidelity TPS LookUp Table (n=%.1f)...\n', n);

options = optimoptions('fmincon','Display','off', 'Algorithm', 'sqp', ...
    'DiffMinChange', 1e-4, 'TolFun', 1e-4);

for i = 1:length(P_range)
    fprintf('Processing Power Level %d / %d...\n', i, length(P_range));
    for j = 1:length(V1_range)
        V1_val = V1_range(j);
        P_val = P_range(i);
        
        if abs(P_val) < 100
             D1_map(i,j) = 0; D2_map(i,j) = 0; D3_map(i,j) = 0;
             continue;
        end
        
        % Initial Guess: SPS Equivalent
        % P approx K * D1 * (1-D1). Solve quadratic for D1 guess.
        V2_prime = n*V2_fixed;
        K_factor = (V1_val * V2_prime) / (2*fs*L);
        % P/K = D - D^2 -> D^2 - D + P/K = 0
        % D = (1 - sqrt(1 - 4P/K))/2 (approx small D)
        discriminant = 1 - 4*abs(P_val)/K_factor;
        if discriminant < 0, discriminant = 0; end
        D_guess = (1 - sqrt(discriminant))/2 * sign(P_val);
        
        x0 = [D_guess, 0, 0]; 
        
        % Limits: Global Shift -0.5..0.5, Inner Shifts 0..0.45
        % Limit Inner Shift < 0.5 to strictly maintain some pulse width
        lb = [-0.5, 0.0, 0.0]; 
        ub = [ 0.5, 0.45, 0.45];
        
        % Objective: Minimize RMS Current Proxy (Simple Sum of Squares works well for efficiency)
        obj_fun = @(x) x(1)^2 + x(2)^2 + x(3)^2; 
        
        % Constraint: Precise Power Transfer
        nonl_con = @(x) power_constraint_precise(x, V1_val, V2_fixed, L, fs, n, P_val);
        
        try
            x_opt = fmincon(obj_fun, x0, [],[],[],[], lb, ub, nonl_con, options);
            D1_map(i,j) = x_opt(1);
            D2_map(i,j) = x_opt(2);
            D3_map(i,j) = x_opt(3);
        catch
            D1_map(i,j) = 0;
        end
    end
end

TPS_Table.V1_vec = V1_range;
TPS_Table.P_vec  = P_range;
TPS_Table.D1 = D1_map;
TPS_Table.D2 = D2_map;
TPS_Table.D3 = D3_map;

save('TPS_Table.mat', 'TPS_Table');
fprintf('TPS_Table.mat generated successfully.\n');


% --- Helper Functions ---
function [c, ceq] = power_constraint_precise(x, V1, V2, L, fs, n, P_ref)
    % Calculates steady-state power transfer for given Phase Shifts
    D1 = x(1); % Global Shift (Pri to Sec)
    D2 = x(2); % Pri Inner
    D3 = x(3); % Sec Inner
    
    P_calc = calc_tps_power_numeric(V1, V2, L, fs, n, D1, D2, D3);
    
    c = [];
    ceq = P_calc - P_ref;
end

function P_avg = calc_tps_power_numeric(V1, V2, L, fs, n, D1, D2, D3)
    % 1. Define Switching Events in [0, 1] (Normalized Time)
    % Pri Voltage Edges: 
    % Rising: D2 (Start of +V1 pulse? No, see Controller.m)
    % Controller: t1=0 (Q1 on), t3=0.5+D2 (Q3 on).
    % Overlap Q1&Q4 (Voltage +V1): [D2, 0.5] if D2 in 0..0.5
    % Overlap Q2&Q3 (Voltage -V1): [0.5+D2, 1.0]
    
    % Let's map Controller Logic to Voltage Pattern:
    % Pri +V1: [D2, 0.5]  (Duration 0.5 - D2)
    % Pri 0V : [0.5, 0.5+D2] (Freewheel)
    % Pri -V1: [0.5+D2, 1.0]
    % Pri 0V : [0.0, D2] (Freewheel) -> Loop wrap
    
    % Sec Edges (Shifted by D1):
    % Reference Frame: Global Time
    % Sec +V2': [D1 + D3, D1 + 0.5]
    % Sec -V2': [D1 + 0.5 + D3, D1 + 1.0]
    % (Modulo 1.0)
    
    % 2. Collect all unique time points
    t_points = [0, D2, 0.5, 0.5+D2]; % Pri Points
    
    s_start = mod(D1 + D3, 1);
    s_end   = mod(D1 + 0.5, 1);
    s_start_n = mod(D1 + 0.5 + D3, 1);
    s_end_n   = mod(D1 + 1.0, 1);
    
    t_points = [t_points, s_start, s_end, s_start_n, s_end_n];
    t_points = unique(sort(mod(t_points, 1)));
    
    % Ensure 0 and 1 are included
    if t_points(1) ~= 0, t_points = [0, t_points]; end
    if t_points(end) ~= 1, t_points = [t_points, 1]; end
    
    % 3. Iterate Segments
    i_val = 0; % Start with arbitrary current
    % We need to find steady state i(0).
    % i(t) integrates slope (vp - vs)/L.
    % i(1) = i(0) + sum(slopes * dt).
    % For periodic steady state, sum(slopes * dt) must be 0? 
    % Actually i(1) must equal i(0). But (vp-vs) average must be 0 for flux balance?
    % Let's assume ideal DC steady state implies integral VL = 0.
    % So we just compute slopes, integrate from i(0)=0 to get profile, then offset average?
    % No, i_avg is load dependent? 
    % For DAB, i_avg_inductor is 0 (AC Link). 
    % Correct. The inductor current has 0 DC bias in steady state.
    
    % Re-calculate slopes
    dt_vec = diff(t_points) * (1/fs); % Time durations
    t_mids = t_points(1:end-1) + diff(t_points)/2;
    
    vp_vals = zeros(size(t_mids));
    vs_vals = zeros(size(t_mids));
    
    for k = 1:length(t_mids)
        tm = t_mids(k); % Normalized time 0..1
        
        % Primary Voltage State
        % [+V1]: D2 <= t < 0.5
        % [-V1]: 0.5+D2 <= t < 1.0
        % else 0
        if (tm >= D2) && (tm < 0.5)
            vp = V1;
        elseif (tm >= 0.5+D2) && (tm < 1.0)
            vp = -V1;
        else
            vp = 0;
        end
        
        % Secondary Voltage State (Referred)
        % Reference window shifted by D1
        % Effective Window: [D1+D3, D1+0.5] (Mod 1) is +V2'
        % [D1+0.5+D3, D1+1] is -V2'
        
        % Normalize tm relative to Sec Frame
        tm_s = mod(tm - D1, 1); 
        if (tm_s >= D3) && (tm_s < 0.5)
            vs = n*V2;
        elseif (tm_s >= 0.5+D3) && (tm_s < 1.0)
            vs = -n*V2;
        else
            vs = 0;
        end
        
        vp_vals(k) = vp;
        vs_vals(k) = vs;
    end
    
    slopes = (vp_vals - vs_vals) / L;
    d_currents = slopes .* dt_vec;
    
    % Construct Current Waveform (relative to i(0)=0)
    i_profile = [0, cumsum(d_currents)];
    % Remove DC offset to force zero average (AC coupling of transformer/inductor)
    % Integrate i_profile to find average
    % Area (Trapezoidal)
    area = 0;
    for k = 1:length(dt_vec)
        avg_h = (i_profile(k) + i_profile(k+1))/2;
        area = area + avg_h * dt_vec(k);
    end
    i_dc_bias = area / (1/fs);
    
    i_profile_corrected = i_profile - i_dc_bias;
    
    % 4. Calculate Power
    % P = Avg(V1 * iL)
    % integrate V1(t) * i(t) dt
    p_sum = 0;
    for k = 1:length(dt_vec)
        v_seg = vp_vals(k);
        if v_seg == 0
            continue;
        end
        % Current is linear in segment: i_start to i_end
        i_s = i_profile_corrected(k);
        i_e = i_profile_corrected(k+1);
        % integral (v * (slope*t + c)) = v * avg_i * dt
        i_avg_seg = (i_s + i_e) / 2;
        p_sum = p_sum + v_seg * i_avg_seg * dt_vec(k);
    end
    
    P_avg = p_sum * fs; % Divide by Period (1/fs)
end