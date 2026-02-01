function [D1, D2, D3] = HDC_Predict(V_in, P_ref, Model)
    % HDC_Predict: Predicts Triple Phase Shift (TPS) parameters using trained HDC model
    % Inputs:
    %   V_in: Input Voltage (Scalar)
    %   P_ref: Target Power (Scalar)
    %   Model: Struct containing HDC parameters (W, Levels, etc.)
    % Outputs:
    %   D1, D2, D3: Predicted duty cycles/phase shifts
    
    % 1. Quantize Inputs
    % Voltage
    idx_v = floor( (V_in - Model.min_V)/(Model.max_V - Model.min_V) * (Model.L_levels-1) ) + 1;
    idx_v = max(1, min(Model.L_levels, idx_v));
    
    % Power
    idx_p = floor( (P_ref - Model.min_P)/(Model.max_P - Model.min_P) * (Model.L_levels-1) ) + 1;
    idx_p = max(1, min(Model.L_levels, idx_p));
    
    % 2. Retrieve Level Hypervectors
    hv_v = Model.Level_Item_V(idx_v, :);
    hv_p = Model.Level_Item_P(idx_p, :);
    
    % 3. Bind (XOR/Mult)
    % Assuming Bipolar {-1, 1} representation
    query_hv = hv_v .* hv_p;
    
    % 4. Query (Matrix Multiply)
    % Output = Query * Weights
    % Query: (1 x D), Weights: (D x 3) -> Output: (1 x 3)
    Output = query_hv * Model.W;
    
    D1 = Output(1);
    D2 = Output(2);
    D3 = Output(3);
end
