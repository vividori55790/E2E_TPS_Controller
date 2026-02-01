%% Visualize_Optimization_Results.m
% Parses Optimization_Log.txt and generates detailed visualizations of the optimization process.
% Place this script in the same directory as 'Optimization_Log.txt'.

clear; clc; close all;

%% 1. File Setup
currentPath = fileparts(mfilename('fullpath'));
logFile = fullfile(currentPath, 'Optimization_Log.txt');

if ~exist(logFile, 'file')
    error('Log file not found: %s', logFile);
end

fprintf('Reading Log File: %s\n', logFile);

%% 2. Parse Log File
fid = fopen(logFile, 'r');
data = struct('Iteration', {}, 'TimeStr', {}, 'Kp', {}, 'Ki', {}, 'Cost', {}, ...
              'Track', {}, 'Effort', {}, 'SS', {}, 'Over', {});

lineIdx = 0;
rawLines = 0;

while ~feof(fid)
    tline = fgetl(fid);
    rawLines = rawLines + 1;
    
    if ischar(tline) && ~isempty(tline)
        % Format in Run_BayesOpt.m:
        % [%s] Kp=%.4f, Ki=%.4f | Cost=%.4f | Trk=%.3f, Eff=%.3f, SS=%.3f, Ov=%.1f%% | %s
        
        % Regex pattern to extract values (handling potential spaces)
        % Example: [00:40:52] Kp=3.5746, Ki=562.3115 | Cost=7.9708 | Trk=7.806, Eff=3.298, SS=0.000, Ov=623.1% | Success
        pat = '\[(.*?)\] Kp=([\d\.]+),\s*Ki=([\d\.]+)\s*\|\s*Cost=([\d\.]+)\s*\|\s*Trk=([\d\.]+),\s*Eff=([\d\.]+),\s*SS=([\d\.]+),\s*Ov=([\d\.]+)%';
        
        tokens = regexp(tline, pat, 'tokens');
        
        if ~isempty(tokens)
            tokens = tokens{1};
            lineIdx = lineIdx + 1;
            
            data(lineIdx).Iteration = lineIdx;
            data(lineIdx).TimeStr = tokens{1};
            data(lineIdx).Kp = str2double(tokens{2});
            data(lineIdx).Ki = str2double(tokens{3});
            data(lineIdx).Cost = str2double(tokens{4});
            data(lineIdx).Track = str2double(tokens{5});
            data(lineIdx).Effort = str2double(tokens{6});
            data(lineIdx).SS = str2double(tokens{7});
            data(lineIdx).Over = str2double(tokens{8});
        end
    end
end
fclose(fid);

fprintf('Processed %d lines. Found %d valid log entries.\n', rawLines, lineIdx);

if lineIdx == 0
    warning('No valid data points found. Check log file format.');
    return;
end

%% 3. Convert to Table & Filter
T_all = struct2table(data);

% Filter out failed runs (Cost >= 999999 usually indicates failure/penalty)
validMask = T_all.Cost < 1e5;
T = T_all(validMask, :);

fprintf('Filtered out %d failed/invalid runs. Plotting %d valid runs.\n', ...
    height(T_all) - height(T), height(T));

%% 4. Visualizations

% --- Figure 1: Cost Convergence ---
fig1 = figure('Name', 'Cost Convergence', 'Color', 'w', 'Position', [100 100 1000 600]);

% Subplot 1: Raw Cost & Min Cost
subplot(2, 1, 1);
plot(T.Iteration, T.Cost, 'o-', 'Color', [0.7 0.7 0.7], 'MarkerSize', 3, 'DisplayName', 'Iteration Cost');
hold on;
min_costs = cummin(T.Cost);
plot(T.Iteration, min_costs, 'r-', 'LineWidth', 2, 'DisplayName', 'Best Cost Found');
title('Optimization Convergence History');
xlabel('Iteration'); ylabel('Cost Function Value');
grid on; legend('Location', 'best');
xlim([1, max(T.Iteration)]);

% Subplot 2: Zoomed in on lower costs (cutoff top 10% for clarity if high variance)
subplot(2, 1, 2);
filtered_costs = T.Cost(T.Cost < prctile(T.Cost, 90)); % Show bottom 90%
histogram(filtered_costs, 30, 'FaceColor', 'b', 'FaceAlpha', 0.6);
title('Cost Distribution (Bottom 90%)');
xlabel('Cost'); ylabel('Frequency');
grid on;


% --- Figure 2: Parameter Space Exploration (3D) ---
fig2 = figure('Name', 'Parameter Space Search', 'Color', 'w', 'Position', [150 150 1000 800]);

scatter3(T.Kp, T.Ki, T.Cost, 50, T.Cost, 'filled');
colormap(jet);
c = colorbar;
c.Label.String = 'Cost';
xlabel('Kp (Proportional Gain)');
ylabel('Ki (Integral Gain)');
zlabel('Cost');
title('3D Parameter Space Exploration');

% Log scale is often better for PID gains search visualization
set(gca, 'XScale', 'log', 'YScale', 'log');
grid on; view(45, 30);

% Highlight Best Point
[minC, idx] = min(T.Cost);
hold on;
plot3(T.Kp(idx), T.Ki(idx), minC, 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
text(T.Kp(idx), T.Ki(idx), minC, sprintf('  Best: Kp=%.3f, Ki=%.3f', T.Kp(idx), T.Ki(idx)), ...
    'BackgroundColor', 'w', 'EdgeColor', 'k');


% --- Figure 3: Parameter Evolution Over Time ---
fig3 = figure('Name', 'Parameter Evolution', 'Color', 'w', 'Position', [200 200 1000 600]);

subplot(2, 1, 1);
plot(T.Iteration, T.Kp, 'b.-', 'LineWidth', 1);
ylabel('Kp'); title('Kp Evolution'); 
grid on; xlim([1, max(T.Iteration)]);

subplot(2, 1, 2);
plot(T.Iteration, T.Ki, 'r.-', 'LineWidth', 1);
ylabel('Ki'); title('Ki Evolution'); xlabel('Iteration');
grid on; xlim([1, max(T.Iteration)]);


% --- Figure 4: Objective Components Breakdown ---
fig4 = figure('Name', 'Objective Components', 'Color', 'w', 'Position', [250 250 1200 800]);

subplot(2, 2, 1);
scatter(T.Iteration, T.Track, 20, T.Cost, 'filled');
title('Tracking Error (RMSE/ISE)'); xlabel('Iteration'); ylabel('Value'); grid on; colorbar;

subplot(2, 2, 2);
scatter(T.Iteration, T.Effort, 20, T.Cost, 'filled');
title('Control Effort'); xlabel('Iteration'); ylabel('Value'); grid on; colorbar;

subplot(2, 2, 3);
scatter(T.Iteration, T.SS, 20, T.Cost, 'filled');
title('Steady State Error'); xlabel('Iteration'); ylabel('Value'); grid on; colorbar;

subplot(2, 2, 4);
scatter(T.Iteration, T.Over, 20, T.Cost, 'filled');
title('Overshoot (%)'); xlabel('Iteration'); ylabel('Value (%)'); grid on; colorbar;


%% 5. Print Summary
fprintf('\n=== Optimization Summary ===\n');
fprintf('Total Iterations Parsed: %d\n', height(T));
fprintf('Best Cost: %.4f\n', minC);
fprintf('Best Parameters: Kp = %.5f, Ki = %.5f\n', T.Kp(idx), T.Ki(idx));
fprintf('Details at Best:\n');
fprintf('  - Tracking Error: %.4f\n', T.Track(idx));
fprintf('  - Control Effort: %.4f\n', T.Effort(idx));
fprintf('  - SS Error:       %.4f\n', T.SS(idx));
fprintf('  - Overshoot:      %.2f%%\n', T.Over(idx));
fprintf('============================\n');

% --- Figure 5: Approximate Cost Landscape (Interpolated) ---
% Note: This approximates the "Cost Model" surface using available data points.
fig5 = figure('Name', 'Approximate Cost Landscape', 'Color', 'w', 'Position', [300 300 1000 800]);

% Create a grid in log-space (since parameters are log-scaled)
valid_Kp = T.Kp(T.Kp > 0);
valid_Ki = T.Ki(T.Ki > 0);

if ~isempty(valid_Kp) && ~isempty(valid_Ki)
    x_log = log10(valid_Kp);
    y_log = log10(valid_Ki);
    z = T.Cost;

    % Create interpolant (fitting a smooth surface through points)
    F = scatteredInterpolant(x_log, y_log, z, 'natural', 'none');

    % Grid for visualization
    [xq_log, yq_log] = meshgrid(...
        linspace(min(x_log), max(x_log), 100), ...
        linspace(min(y_log), max(y_log), 100));
    
    zq = F(xq_log, yq_log);

    % Convert back to linear space for plotting
    xq = 10.^xq_log;
    yq = 10.^yq_log;

    % Surface Plot
    surf(xq, yq, zq, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
    hold on;
    
    % Add contour for easier reading of minima
    contour3(xq, yq, zq, 20, 'k', 'LineWidth', 0.5);
    
    % Overlay actual data points
    scatter3(valid_Kp, valid_Ki, z, 30, 'k', 'filled', 'MarkerEdgeColor', 'w');
    
    % Highlight Minimum
    plot3(T.Kp(idx), T.Ki(idx), minC, 'rp', 'MarkerSize', 20, 'MarkerFaceColor', 'r');

    set(gca, 'XScale', 'log', 'YScale', 'log', 'ZScale', 'linear');
    xlabel('Kp (Log Scale)'); ylabel('Ki (Log Scale)'); zlabel('Cost');
    title('Approximate Cost Landscape (Interpolated Surface)');
    colorbar; colormap(parula);
    view(45, 30); grid on;
else
    title('Not enough data for landscape interpolation');
end

