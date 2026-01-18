# Simulink Scripts Documentation

This directory contains MATLAB scripts for simulating, optimizing, and verifying the DAB Converter control logic.

## 0. Configuration System (신규)

모든 스크립트는 중앙 설정 파일을 사용합니다.

- **`../DAB_System_Config.m`**:
  - 모든 시스템 파라미터를 중앙 집중 관리
  - Plant, Battery, Controller, Simulation 설정 포함
  - 파라미터 변경 시 이 파일만 수정하면 됨

  ```matlab
  config = DAB_System_Config();
  params = config.plant;
  V_batt = config.battery.V_nom;
  ```

## 1. Controller Optimization (MIL)

Scripts for tuning PI controller parameters using Model-in-the-Loop (MIL) simulation.

- **`Auto_Optimize_And_Verify.m`** ⭐ (권장):
  - 최적화 + 다중 시나리오 검증을 한 번에 실행
  - 7개 시나리오 (전압/전류/배터리 변동) 자동 검증
  - Pass/Fail 리포트 및 시각화 제공
- **`Optimize_Controller_MIL.m`**:
  - 단독 최적화 스크립트 (검증 없음)
  - Uses `DAB_Plant_TPS_v2.m` (SPS Analytic Model) for fast iteration.
  - Minimizes a cost function based on Overshoot, Steady-State Error, and Settling Time.

- **`Verify_Optimized_Controller.m`**:
  - Verifies the tuned parameters on a longer simulation or different conditions using the full TPS Plant model.

## 2. Simulink Execution & Control

Scripts that interact directly with `Control_Logic.slx` or other Simulink models.

- **`Run_Control_Logic.m`**:
  - Sets up parameters and runs the `Control_Logic.slx` model.
  - Handles directory switching to `Modules/` to ensure relative paths work.

- **`Run_Simulink_Compare_MIL.m`**:
  - Runs Simulink simulation (Dynamic Scenario) and compares the results against the MIL (MATLAB-only) model.
  - Useful for validating that the Simulink implementation matches the theoretical MIL model.

## 3. Visualization & Inspection

Tools for plotting and debugging simulation data.

- **`Deep_Inspect_And_Plot.m`**:
  - Detailed inspection of simulation results. Plots Volts, Amps, and Control signals.
  - **Active Development**: Currently the primary tool for visualization.
- **`Visualize_Bus_Structures.m`**:
  - Specialized for handling Simulink Bus objects (`Data_IO`, `Data_Control`).
- **`Inspect_Simulink_Signals.m`**:
  - Quick inspection of available signals in `simOut`.

## 4. Verification & Testing

- **`Verify_Steady_State.m`**: Checks if the converter reaches steady state correctly.
- **`Quick_MIL_Test.m`**: A fast sanity check for the TPS MATLAB Plant model.

## 5. Helpers

- **`Create_Scenario.m`**: Generates test scenarios (Voltage steps, Load changes).
- **`Generate_Smart_Report.m`**: Auto-generates a text/PDF report of simulation stats.
- **`Plot_Results_To_File.m`**: Saves plots to disk.

---

**Note**: large data files (`*.mat`) are excluded from Git, except for `TPS_Table.mat` and `HDC_Model.mat`.
**Plant Models**:

- `DAB_Plant_TPS.m`: Full Triple Phase Shift (TPS) numerical model.
- `DAB_Plant_TPS_v2.m`: Simplified Single Phase Shift (SPS) analytical model for fast optimization.
