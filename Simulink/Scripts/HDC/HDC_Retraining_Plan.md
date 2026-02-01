# HDC Iterative Retraining Plan

## 1. Objective

To enhance the accuracy of the DAB Converter control model by refining the initial "Bundled" model through iterative retraining. The initial model, created by simple superposition, often suffers from inter-class interference, especially when high-dimensional space is crowded or features are noisy.

## 2. Methodology: Perceptron-based Retraining

We will implement a retraining mechanism inspired by frameworks like **OnlineHD** or **QuantHD**. The core idea is to penalize the model for incorrect predictions and reward it for correct ones, progressively shifting the class hypervectors ($C_k$) to separate the classes in the hyperspace.

### Algorithm Steps

1.  **Load Initial Model**: Start with the `Class_Vectors` ($M$) generated from the Initial Training phase.
2.  **Hyperparameters**:
    - `Epochs`: Number of passes through the dataset (e.g., 20-50).
    - `Learning Rate (alpha)`: Weight of the update (constant or decaying).
3.  **Training Loop**:
    - For each Epoch:
      - Shuffle the training dataset.
      - For each sample $(Feature, Label_{true})$:
        1.  **Encode**: Convert features to Query Hypervector $Q$ using the _same_ ID and Level matrices.
        2.  **Predict**: Calculate Cosine Similarity (or Dot Product) between $Q$ and all Class Vectors $C_k$.
            - $\text{Scores}_k = Q \cdot C_k$
            - $Label_{pred} = \arg\max_k (\text{Scores}_k)$
        3.  **Update Rule** (if $Label_{pred} \neq Label_{true}$):
            - **Strengthen Correct Class**: $C_{true} = C_{true} + \alpha \cdot Q$
            - **Weaken Wrong Class**: $C_{pred} = C_{pred} - \alpha \cdot Q$
            - _Note_: This effectively pushes the class center of $C_{true}$ towards $Q$ and $C_{pred}$ away from $Q$.

## 3. MATLAB Implementation Plan

### function `Train_HDC_Retrain.m`

**Inputs**:

- `HDC_Initial_Model.mat` (Contains $C$, $ID$, $L$, etc.)
- `HDC_TrainingData.mat`

**Process**:

1.  **Data Preparation**:
    - Load and normalize features exactly as in Initial Training.
    - Pre-compute all Sample Hypervectors ($Q_{all}$) to save time inside the loop? (Memory trade-off: $N \times 10000$ might be large. If $N=5000$, it's 50MB-400MB, acceptable).
2.  **QuantHD / BinHD Considerations**:
    - **Quantization**: To ensure the model remains "Quantized" (hardware friendly), we can clamp the values of $C_k$ to a specific range (e.g., int8 or int16) after updates.
    - **Binarization**: If targeting BinHD (1-bit weights), we would take $sign(C_k)$ after every update or batch. However, maintaining integer precision (Accumulator) during training usually yields better convergence. We will maintain **Integer Weights** during training and quantized only for final inference.

3.  **Evaluation Metric**:
    - Track Accuracy (Exact Match) and **Mean Absolute Error (MAE)** of the Phase Angle.
    - Since Phase is continuous, a prediction of $85^\circ$ when true is $86^\circ$ is better than prediction $-20^\circ$.
    - _Extension_: Consider "Soft Labels" or updating neighbors ($C_{true \pm 1}$) slightly to preserve topological order, though standard Retraining treats classes as independent.

4.  **Output**:
    - `HDC_Retrained_Model.mat`: Updated Class Vectors.
    - Training Log: Accuracy vs Epoch plot.

## 4. Next Steps

1.  Verify `Collect_HDC_Data.m` runs and produces valid data.
2.  Verify `Train_HDC_Model.m` produces a baseline model.
3.  Implement `Train_HDC_Retrain.m` following this plan.
