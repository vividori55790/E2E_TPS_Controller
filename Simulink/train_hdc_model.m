% =========================================================
% HDC Model Trainer for DAB Converter TPS Control
% Generates a C-header file for STM32 implementation.
% =========================================================

clear; clc;

% --- 1. Load Data ---
if ~isfile('TPS_Table.mat')
    error('TPS_Table.mat not found. Run TPS_LUT.m first.');
end
load('TPS_Table.mat');

% Flatten Grid Data
V_grid = TPS_Table.V1_vec;
P_grid = TPS_Table.P_vec;
[V_mesh, P_mesh] = meshgrid(V_grid, P_grid);

X_v = V_mesh(:);
X_p = P_mesh(:);
Y_d1 = TPS_Table.D1(:);
Y_d2 = TPS_Table.D2(:);
Y_d3 = TPS_Table.D3(:);

% Filter constraints (Remove invalid 0s if necessary, but 0 is valid for 0 power)
% We keep all to learn the full landscape.

% --- 2. HDC Hyperparameters ---
D = 1024;                % Dimensions (Multiple of 32 for efficient MCU handling)
L_levels = 64;           % Quantization Levels for Inputs

% Normalization parameters
min_V = 300; max_V = 450;
min_P = -5000; max_P = 5000;

rng(42); % Reproducibility

% --- 3. Generate Level Hypervectors (Bit-flip progression) ---
% Algorithm: Create Base, then flip bits progressively to create correlated levels.
% Level 1 and Level L should be orthogonal (flip D/2 bits total).
% Store as bipolar (-1, 1) for math, convert to bits for storage later.

Level_Memory = zeros(L_levels, D);
base = randi([0, 1], 1, D) * 2 - 1; % -1 or 1

% Indices to flip (Random permutation of 1..D)
flip_sequence = randperm(D);
flip_count_per_level = floor( (D/2) / (L_levels-1) );

current_vec = base;
for i = 1:L_levels
    Level_Memory(i, :) = current_vec;
    
    % Flip next batch of bits
    if i < L_levels
        start_idx = (i-1)*flip_count_per_level + 1;
        end_idx = start_idx + flip_count_per_level - 1;
        bits_to_flip = flip_sequence(start_idx:end_idx);
        current_vec(bits_to_flip) = -current_vec(bits_to_flip);
    end
end

fprintf('HDC Setup: D=%d, Levels=%d\n', D, L_levels);

% --- 4. Encoding & Training ---
% Feature X = Bind(Level(V), Level(P)) => XOR in binary, Element-wise Mult in Bipolar
% Since we allow reuse, let's use separate Level Memories for V and P? 
% Or same memory mapped differently? Using separate random Level Memories is better for orthogonality.

% Let's create two separate Level Memories to avoid crosstalk.
Level_Item_V = Level_Memory; % Use the one generated
Level_Item_P = Level_Memory; % Actually, regenerate for P to ensure independence
% Regenerate P levels
base_p = randi([0, 1], 1, D) * 2 - 1;
flip_seq_p = randperm(D);
current_vec = base_p;
for i = 1:L_levels
    Level_Item_P(i, :) = current_vec;
    if i < L_levels
        start_idx = (i-1)*flip_count_per_level + 1;
        end_idx = start_idx + flip_count_per_level - 1;
        bits_to_flip = flip_seq_p(start_idx:end_idx);
        current_vec(bits_to_flip) = -current_vec(bits_to_flip);
    end
end

% Encode Training Matrix H
N_samples = length(X_v);
H_train = zeros(N_samples, D);

fprintf('Encoding %d samples...\n', N_samples);

for k = 1:N_samples
    v_val = X_v(k);
    p_val = X_p(k);
    
    % Quantize
    idx_v = floor( (v_val - min_V)/(max_V - min_V) * (L_levels-1) ) + 1;
    idx_p = floor( (p_val - min_P)/(max_P - min_P) * (L_levels-1) ) + 1;
    
    idx_v = max(1, min(L_levels, idx_v));
    idx_p = max(1, min(L_levels, idx_p));
    
    % Bind (Element-wise Multiply for Bipolar)
    hv_v = Level_Item_V(idx_v, :);
    hv_p = Level_Item_P(idx_p, :);
    
    H_train(k, :) = hv_v .* hv_p;
end

% Train Readout Weights (Ridge Regression)
% W = (H'H + lambda*I)^-1 * H' * Y
lambda = 1.0; % Regularization
Y_target = [Y_d1, Y_d2, Y_d3];

fprintf('Training Regression Model...\n');
% Solve: (H'H + lam*I) W_t = H'Y => W_t = ... => Output = H * W_t
% Note: W should be (D x 3).
% Normal Equation
A = H_train' * H_train + lambda * eye(D);
B = H_train' * Y_target;
W = A \ B; % D x 3 Matrix

% --- 5. Validation ---
Y_pred = H_train * W;
mse = mean(mean((Y_pred - Y_target).^2));
fprintf('Training MSE: %.6f\n', mse);

% --- 6. Export to C Header (STM32) ---
% We need to export:
% 1. Level_Item_V (Compressed to binary bits)
% 2. Level_Item_P (Compressed to binary bits)
% 3. Weights W (Floats)

fileID = fopen('hdc_model.h','w');
fprintf(fileID, '#ifndef HDC_MODEL_H\n#define HDC_MODEL_H\n\n');
fprintf(fileID, '#include <stdint.h>\n\n');
fprintf(fileID, '// HDC Dimensions\n');
fprintf(fileID, '#define HDC_D %d\n', D);
fprintf(fileID, '#define HDC_D_WORDS (HDC_D / 32)\n');
fprintf(fileID, '#define HDC_LEVELS %d\n\n', L_levels);

fprintf(fileID, '// Normalization Constants\n');
fprintf(fileID, '#define MIN_V %.2ff\n', min_V);
fprintf(fileID, '#define MAX_V %.2ff\n', max_V);
fprintf(fileID, '#define MIN_P %.2ff\n', min_P);
fprintf(fileID, '#define MAX_P %.2ff\n', max_P);

% Helper to write bipolar matrix as bits
write_binary_matrix(fileID, 'HDC_LEVEL_V', Level_Item_V);
write_binary_matrix(fileID, 'HDC_LEVEL_P', Level_Item_P);

% Write Weights (Floats)
fprintf(fileID, '// Regression Weights (D x 3) => Flattened [D][3]\n');
fprintf(fileID, '// Layout: { w_d1_0, w_d2_0, w_d3_0, w_d1_1, ... }\n');
fprintf(fileID, 'static const float HDC_WEIGHTS[%d] = {\n', D*3);
for r = 1:D
    fprintf(fileID, '    %.6ff, %.6ff, %.6ff', W(r,1), W(r,2), W(r,3));
    if r < D, fprintf(fileID, ','); end
    fprintf(fileID, '\n');
end
fprintf(fileID, '};\n\n');

fprintf(fileID, '#endif // HDC_MODEL_H\n');
fclose(fileID);

fprintf('HDC Model exported to hdc_model.h\n');

% --- 7. Save MATLAB Model for Simulation ---
save('HDC_Model.mat', 'Level_Item_V', 'Level_Item_P', 'W', 'min_V', 'max_V', 'min_P', 'max_P', 'D', 'L_levels');
fprintf('HDC Model saved to HDC_Model.mat\n');

% --- Helper Function ---
function write_binary_matrix(fid, name, mat)
    [rows, cols] = size(mat);
    fprintf(fid, 'static const uint32_t %s[%d][%d] = {\n', name, rows, cols/32);
    for r = 1:rows
        fprintf(fid, '    {');
        % Process 32 bits at a time
        for c = 1:32:cols
            chunk = mat(r, c:min(c+31, cols));
            % Convert bipolar (-1, 1) to bits (0, 1) -> 1 is 1, -1 is 0
            bits = chunk > 0; 
            % Pack into uint32
            val_int = 0;
            for b = 1:length(bits)
                if bits(b), val_int = bitset(val_int, b); end
            end
            fprintf(fid, '0x%08X', val_int);
            if c+31 < cols, fprintf(fid, ', '); end
        end
        fprintf(fid, '}, // Level %d\n', r);
    end
    fprintf(fid, '};\n\n');
end
