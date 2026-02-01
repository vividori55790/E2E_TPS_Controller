
% Train_HDC_Model.m
% Trains an HDC (Hyperdimensional Computing) model for DAB TPS Control.
%
% Approach: Associative Memory (Nearest Neighbor Regression)
% 1. Encode Inputs (Vin, Vout, Power) into a Hypervector (Query Key).
% 2. Store Key-Value pairs in memory. 
%    - Key: Encoded State Hypervector
%    - Value: Optimal Control Variables [D1, D2, Phi]
% 3. During inference, the query is compared to stored keys to find the closest match.
%
% Input: Data/TPS_Optimal_Dataset.mat
% Output: Data/HDC_Model_TPS.mat

clear; clc;

% =========================================================================
% 1. Configuration
% =========================================================================
HDC_Dim = 10000;         % Hypervector Dimension
Num_Levels = 100;        % Number of quantization levels for continuous inputs
Valid_Ratio = 0.2;       % Validation Split Ratio (for testing accuracy)

fprintf('Loading TPS Dataset...\n');
% Resolve Path
CurrentDir = fileparts(mfilename('fullpath'));
DataPath = fullfile(CurrentDir, 'Data', 'TPS_Optimal_Dataset.mat');

if ~exist(DataPath, 'file')
    error('Dataset not found at: %s. Run Generate_TPS_Dataset.m first.', DataPath);
end

load(DataPath, 'Dataset', 'L', 'fs');

% =========================================================================
% 2. Data Preprocessing
% =========================================================================
% Dataset Cols: [Vin, Vout, Power, D1, D2, Phi, Loss, ZVS, k, P_pu]
Raw_Inputs = Dataset(:, 1:3);
Raw_Outputs = Dataset(:, 4:6);

% Remove Failed Scenarios (NaNs)
valid_mask = ~isnan(Raw_Outputs(:,1));
Clean_Inputs = Raw_Inputs(valid_mask, :);
Clean_Outputs = Raw_Outputs(valid_mask, :);

Num_Samples = size(Clean_Inputs, 1);
fprintf('Valid Samples: %d\n', Num_Samples);

% Normalize Inputs for Level Projection (Min-Max Scaling)
Min_Vals = min(Clean_Inputs);
Max_Vals = max(Clean_Inputs);

% Create Ranges for quantization
Vin_Range = linspace(Min_Vals(1), Max_Vals(1), Num_Levels);
Vout_Range = linspace(Min_Vals(2), Max_Vals(2), Num_Levels);
Power_Range = linspace(Min_Vals(3), Max_Vals(3), Num_Levels);

% =========================================================================
% 3. Initialize HDC Basis Vectors
% =========================================================================
fprintf('Initializing HDC Basis Vectors (D=%d)...\n', HDC_Dim);
rng(42); % Fixed Seed for Reproducibility

% ID Vectors (Orthogonal) - one for each field
% Fields: Vin, Vout, Power
ID_Vin = 2 * randi([0, 1], HDC_Dim, 1) - 1;   % Bipolar {-1, +1}
ID_Vout = 2 * randi([0, 1], HDC_Dim, 1) - 1;
ID_Power = 2 * randi([0, 1], HDC_Dim, 1) - 1;

% Level Vectors (Continuous transition)
% We generate a base vector and flip bits progressively
Level_Base = 2 * randi([0, 1], HDC_Dim, 1) - 1;
Level_Mat = zeros(HDC_Dim, Num_Levels);
Level_Mat(:, 1) = Level_Base;

bits_to_flip = floor(HDC_Dim / 2 / Num_Levels); % Total flip 50% across levels
current_vec = Level_Base;
perm_idx = randperm(HDC_Dim); % Random order of bits to flip

for i = 2:Num_Levels
    start_idx = (i-2) * bits_to_flip + 1;
    end_idx = (i-1) * bits_to_flip;
    
    flip_indices = perm_idx(start_idx:min(end_idx, HDC_Dim));
    current_vec(flip_indices) = -current_vec(flip_indices);
    Level_Mat(:, i) = current_vec;
end

% Function to map scalar to Level Index
quantize = @(val, range) sum(val >= range); 

% =========================================================================
% 4. Encode Memory (Training)
% =========================================================================
fprintf('Encoding Memory...\n');

% Split Train/Validation
cv = cvpartition(Num_Samples, 'HoldOut', Valid_Ratio);
Train_Idx = training(cv);
Test_Idx = test(cv);

Train_Inputs = Clean_Inputs(Train_Idx, :);
Train_Outputs = Clean_Outputs(Train_Idx, :);
Test_Inputs = Clean_Inputs(Test_Idx, :);
Test_Outputs = Clean_Outputs(Test_Idx, :);

Num_Train = size(Train_Inputs, 1);
Memory_Keys = zeros(HDC_Dim, Num_Train, 'int8'); % Store as int8 to save space (-1, 1)

tic;
% Vectorized Encoding is harder with levels, doing loop (fast enough for 4k points)
parfor i = 1:Num_Train
    % Get Level Indices
    row = Train_Inputs(i,:);
    idx_v1 = sum(row(1) >= Vin_Range);
    idx_v2 = sum(row(2) >= Vout_Range);
    idx_p  = sum(row(3) >= Power_Range);
    
    % Clamp indices
    idx_v1 = max(1, min(Num_Levels, idx_v1));
    idx_v2 = max(1, min(Num_Levels, idx_v2));
    idx_p  = max(1, min(Num_Levels, idx_p));
    
    % Bind: ID * Level (Element-wise multiply for bipolar)
    H_v1 = ID_Vin .* Level_Mat(:, idx_v1);
    H_v2 = ID_Vout .* Level_Mat(:, idx_v2);
    H_p  = ID_Power .* Level_Mat(:, idx_p);
    
    % Bundle: H_total = H_v1 + H_v2 + H_p (Majority Vote / Binarize)
    % For "Record binding", simple addition is common. 
    % H_enc = sign(H_v1 + H_v2 + H_p)
    H_sum = H_v1 + H_v2 + H_p;
    H_enc = int8(sign(H_sum));
    H_enc(H_enc == 0) = 1; % Handle ties
    
    Memory_Keys(:, i) = H_enc;
end
elapsed = toc;
fprintf('Encoding Complete in %.2f seconds.\n', elapsed);

% Memory Values are just the Train_Outputs
Memory_Values = Train_Outputs;

% =========================================================================
% 5. Validation (Inference Test)
% =========================================================================
fprintf('Validating Model...\n');
Num_Test = size(Test_Inputs, 1);
Predictions = zeros(Num_Test, 3);
Errors = zeros(Num_Test, 3);

% Turn off parfor for validation if debugging, but here it's fine
% Pre-cast for broadcasting
Mem_Keys_Double = double(Memory_Keys); % D x N_Train

for i = 1:Num_Test
    % Encode Probe
    row = Test_Inputs(i,:);
    idx_v1 = max(1, min(Num_Levels, sum(row(1) >= Vin_Range)));
    idx_v2 = max(1, min(Num_Levels, sum(row(2) >= Vout_Range)));
    idx_p  = max(1, min(Num_Levels, sum(row(3) >= Power_Range)));
    
    H_v1 = ID_Vin .* Level_Mat(:, idx_v1);
    H_v2 = ID_Vout .* Level_Mat(:, idx_v2);
    H_p  = ID_Power .* Level_Mat(:, idx_p);
    
    Probe = sign(H_v1 + H_v2 + H_p);
    Probe(Probe==0) = 1;
    
    % Similarity Search (Dot Product)
    % Sim = Probe' * Memory_Keys
    % Since vectors are bipolar {-1, 1}, Dot Product is Hamming Distance proxy
    Scores = Probe' * Mem_Keys_Double; 
    
    % Find Top-K Nearest Neighbors
    K = 3;
    [~, sorted_idx] = sort(Scores, 'descend');
    top_indices = sorted_idx(1:K);
    
    % Regressive Interaction: Mean of Top K
    match_vals = Memory_Values(top_indices, :);
    Predictions(i, :) = mean(match_vals, 1);
    
    Errors(i, :) = abs(Predictions(i, :) - Test_Outputs(i, :));
end

Mean_MAE = mean(Errors);
fprintf('Validation MAE -> D1: %.4f, D2: %.4f, Phi: %.4f\n', Mean_MAE(1), Mean_MAE(2), Mean_MAE(3));

% =========================================================================
% 6. Save Model
% =========================================================================
ModelPath = fullfile(CurrentDir, 'Data', 'HDC_Model_TPS.mat');
save(ModelPath, 'ID_Vin', 'ID_Vout', 'ID_Power', 'Level_Mat', ...
    'Memory_Keys', 'Memory_Values', 'Min_Vals', 'Max_Vals', ...
    'Vin_Range', 'Vout_Range', 'Power_Range', 'HDC_Dim');

fprintf('Model Saved to: %s\n', ModelPath);

% =========================================================================
% 7. Visualization
% =========================================================================
figure(201); clf;
subplot(3,1,1);
plot(Test_Outputs(:,1), 'k', 'LineWidth', 1.5); hold on;
plot(Predictions(:,1), 'r--');
title('Prediction: D1'); legend('True', 'HDC Est'); grid on;

subplot(3,1,2);
plot(Test_Outputs(:,2), 'k', 'LineWidth', 1.5); hold on;
plot(Predictions(:,2), 'b--');
title('Prediction: D2'); grid on;

subplot(3,1,3);
plot(Test_Outputs(:,3), 'k', 'LineWidth', 1.5); hold on;
plot(Predictions(:,3), 'm--');
title('Prediction: Phi'); grid on;
xlabel('Test Sample Index');
