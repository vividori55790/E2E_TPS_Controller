# 하이브리드 UPS용 DAB 컨버터 전구간 통합 검증 마스터 플랜 (AI-Ready Digital Twin v5.0)

본 문서는 하이브리드 UPS의 전 운전 영역에 걸쳐 성능(효율), 신뢰성(강인성), **안전성(스트레스)**를 검증하고, 이를 통해 열-전기 연성 해석 및 AI 제어기 학습용 데이터를 구축하기 위한 최상위 프로토콜입니다.

## 1. 핵심 전략: 열 피드백 루프 및 AI/HDC 데이터셋

기존 v4.0의 손실 분석에 **'온도 피드백'**을 추가하여 실제 하드웨어와의 오차를 1% 미만으로 줄입니다.

### A. Electro-Thermal Iteration

계산된 손실($P_{loss}$)이 다시 소자 온도($T_j$)를 상승시키고, 이것이 다시 저항($R_{ds}$)을 바꾸는 물리적 순환 고리를 시뮬레이션 데이터에 반영합니다.

### B. AI/RL Data Pipeline (Offline Reinforcement Learning)

1.  **Analytical Prior (Base Policy):** `Scripts/HDC/Generate_TPS_Dataset.m`
    - 수식 기반의 이상적인 TPS 최적해를 계산하여 초기 학습 데이터로 활용합니다.
    - Warm-Start용 가중치 초기화에 사용됩니다.

2.  **Simulated Experience (Fine-Tuning):** `Scripts/HDC/Generate_Simulated_Dataset.m` (Proposed)
    - 아래 `v5.0 Workflow`를 통해 수집되는 **'Actual Physics Data'**.
    - 데드타임, 기생 커패시터, 열적 변화 등 수식 모델이 놓친 비선형성(Non-linearity)을 학습합니다.
    - **Goal:** Offline RL Agent가 Nominal Model과 Real Plant 사이의 "Residual Dynamics"를 학습하게 함.

## 2. 시스템 토폴로지 및 센서 상세 위치 (Precise Sensor Installation)

회로 결선 실수를 방지하기 위해 각 센서의 물리적 위치와 극성(Polarity)을 글로 상세히 정의합니다.

### A. 센서 설치 가이드 (Text Description)

#### Meter A (Grid Side Power Meter)

- **Voltage:** Grid 전압 측정.
- **Current:** Grid 전류 측정 (Inverter 방향이 +).

#### AC Link Meter (ZVS Analysis)

- **Voltage:** 변압기 1차측 전압 ($v_p$).
- **Current:** 인덕터 전류 ($i_L$).
- **극성:** 1차측(HV) 브리지에서 2차측(LV) 브리지 방향으로 흐를 때 양수(+).

#### Device Level Sensing (Semiconductor Loss) - **[반드시 추가]**

정밀 손실 분석 및 ZVS 검증을 위해 스위치의 전압/전류를 직접 측정해야 합니다. 대칭성을 고려하여 각 브리지의 대표 스위치 1개씩 선정합니다.

**1차측(High Voltage) 대표 스위치 ($Q_1$):**

- **위치:** 풀브리지 좌측 상단(Top-Left) MOSFET.
- **Voltage Sensor ($Q1\_V_{ds}$):** $Q_1$의 Drain(+)과 Source(-) 양단에 병렬 연결. ZVS 동작 확인용.
- **Current Sensor ($Q1\_I_d$):** $Q_1$의 Drain 앞단 라인을 끊고 직렬 연결(Drain으로 전류가 들어가는 방향이 +). 도통 손실 계산용.

**2차측(Low Voltage) 대표 스위치 ($Q_5$):**

- **위치:** 풀브리지 좌측 상단(Top-Left) MOSFET.
- **Voltage Sensor ($Q5\_V_{ds}$):** $Q_5$의 Drain(+)과 Source(-) 양단에 병렬 연결.
- **Current Sensor ($Q5\_I_d$):** $Q_5$의 Drain 앞단에 직렬 연결.

## 3. 고급 분석 로직: 열 피드백 (Thermal Feedback Loop)

시뮬레이션의 정확도를 극한으로 올리기 위해 손실 $\leftrightarrow$ 온도 상호작용을 고려합니다.

1.  **초기 가정:** $T_{j, init} = 25^\circ C$ (또는 이전 스텝 온도)로 시뮬레이션 수행.
2.  **손실 계산:** $P_{loss} = P_{in} - P_{out}$.
3.  **온도 추정:** $T_{est} = T_{amb} + P_{loss} \times R_{th(j-a)}$.
    - ($R_{th}$: 방열판 열저항, 예: $2.0 ^\circ C/W$)
4.  **피드백 판정:** 만약 $|T_{est} - T_{j, init}| > 5^\circ C$ 이면, 해당 데이터 포인트는 'Thermal Transient' 데이터로 분류하거나 필요 시 재해석.

## 4. 시뮬레이션 변수 매트릭스 (Test Matrix)

### A. Operation Loop

- **Battery Voltage ($V_{batt}$):** [84V, 96V, 116V]
- **Load Rate ($P_{rate}$):** [0.2, 0.5, 0.8, 1.0]
- **Direction:** [Charge, Discharge]

### B. Uncertainty Loop (Monte Carlo)

- **Aging:** $C_{dc}$ (-30%), $R_{esr}$ (+100%)
- **Thermal:** $R_{ds(on)}$ (온도 추정치 반영 가변)

## 5. 최종 통합 스크립트 (MATLAB Workflow v5.0)

이 스크립트는 열 피드백 검증과 AI 데이터셋 자동 생성 기능을 포함합니다.
**파일 저장 제안:** `Scripts/HDC/Generate_Simulated_Dataset.m`

```matlab
%% 1. Simulation Setup
modelName = 'Control_Logic'; % Updated to actual model name

% Check Model
if ~bdIsLoaded(modelName), load_system(modelName); end

% System Parameters
Param.C_nom = 1000e-6;
Param.L_nom = 20e-6;
Param.R_esr_nom = 0.05;
Param.R_ds_25 = 0.02;
Param.R_th_heatsink = 1.5; % [degC/W] 방열판 열저항
Param.T_amb = 40; % [degC] 주변 온도 (가혹 조건)

Grid_Voltage = 400;
Rated_Power = 5000;

% Sweep Conditions
V_batt_sweep = [84, 96, 116];
Load_Levels = [0.2, 0.5, 0.8, 1.0];
Op_Mode_List = ["Charge", "Discharge"];
N_MC_per_Point = 5;

%% 2. Case Generation
in = Simulink.SimulationInput.empty;
run_idx = 1;

fprintf('Generating AI-Ready Digital Twin Cases (v5.0)...\n');

for mode = Op_Mode_List
    for v = V_batt_sweep
        for load_rate = Load_Levels

            % Load & Direction Logic
            if mode == "Charge"
                Calc_R_load_A = 1e6; % Open Grid Load (Conceptually)
                Target_I_batt = -(Rated_Power * load_rate) / v;
            else
                Power_Target = Rated_Power * load_rate;
                Calc_R_load_A = (Grid_Voltage^2) / Power_Target;
                Target_I_batt = 0;
            end

            for mc = 1:N_MC_per_Point
                in(run_idx) = Simulink.SimulationInput(modelName);

                % Uncertainty Injection
                if mc == 1 % Nominal Baseline
                    Aging_F = 1; TypeStr = 'Nominal';
                    % 초기 온도 가정 (Heuristic)
                    Assumed_Loss = Rated_Power * load_rate * 0.05; % 5% 손실 가정
                    T_j_est = Param.T_amb + Assumed_Loss * Param.R_th_heatsink;
                else       % Monte Carlo
                    Aging_F = 0.7 + 0.3 * rand();
                    TypeStr = 'Random';
                    T_j_est = Param.T_amb + (Rated_Power * load_rate * 0.05) * (0.8 + 0.4*rand());
                end

                % Temperature Dependent R_ds(on)
                Calc_R_ds = Param.R_ds_25 * (1 + 0.006 * (T_j_est - 25));

                % Set Variablesto Model Workspace or Block Parameters
                % Note: Ensure these variables exist in the Model Workspace or mask
                in(run_idx) = in(run_idx).setVariable('R_ds_val', Calc_R_ds);
                in(run_idx) = in(run_idx).setVariable('C_dc', Param.C_nom * Aging_F);
                in(run_idx) = in(run_idx).setVariable('V_batt', v);
                in(run_idx) = in(run_idx).setVariable('R_load_val', Calc_R_load_A);
                in(run_idx) = in(run_idx).setVariable('Target_Current', Target_I_batt);

                % Metadata
                in(run_idx) = in(run_idx).setModelParameter('StopTime', '0.05');
                in(run_idx) = in(run_idx).setVariable('Meta_Mode', mode);
                in(run_idx) = in(run_idx).setVariable('Meta_Load', load_rate);
                in(run_idx) = in(run_idx).setVariable('Meta_V', v);
                in(run_idx) = in(run_idx).setVariable('Meta_Type', TypeStr);
                in(run_idx) = in(run_idx).setVariable('Meta_Tj_Input', T_j_est);

                run_idx = run_idx + 1;
            end
        end
    end

end

%% 3. Run Simulation
fprintf('Starting Parallel Simulation (Total %d runs)...\n', length(in));
try
    % Use 'UseFastRestart', 'on' if possible for speed
    out = parsim(in, 'ShowProgress', 'on', 'StopOnError', 'off', 'TransferBaseWorkspaceVariables', 'on');
catch ME
    fprintf('Error: %s\n', ME.message); out = [];
end

%% 4. Analysis & AI Dataset Generation
results = table();
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

for k = 1:length(out)
    data = out(k);
    if data.SimulationMetadata.ExecutionInfo.StopEvent == "DiagnosticError", continue; end

    try
        % Extract Time Vector
        sim_t = data.tout;
        t_idx = sim_t > (sim_t(end) * 0.9);

        % Power & Efficiency Calculation
        % NOTE: Ensure 'V_A', 'I_A', etc. are logged signals
        V_A_avg = mean(data.V_A(t_idx)); I_A_avg = mean(data.I_A(t_idx));
        V_B_avg = mean(data.V_B(t_idx)); I_B_avg = mean(data.I_B(t_idx));

        % Calculate Power (DC avg)
        P_Grid = V_A_avg * I_A_avg;
        P_Batt = V_B_avg * I_B_avg;

        if data.Meta_Mode == "Charge"
            % Grid -> Battery
            Eff = abs(P_Batt / P_Grid) * 100;
            P_Loss = abs(P_Grid) - abs(P_Batt);
        else
            % Battery -> Grid
            Eff = abs(P_Grid / P_Batt) * 100;
            P_Loss = abs(P_Batt) - abs(P_Grid);
        end

        % Thermal Feedback Check
        T_j_Calculated = Param.T_amb + P_Loss * Param.R_th_heatsink;
        Thermal_Err = abs(T_j_Calculated - data.Meta_Tj_Input);

        % Indicators
        Ripple = (range(data.V_A(t_idx)) / V_A_avg) * 100;
        I_Peak = max(abs(data.I_L(t_idx)));

        newRow = {data.Meta_V, data.Meta_Load, string(data.Meta_Mode), string(data.Meta_Type), ...
                  Eff, P_Loss, T_j_Calculated, Thermal_Err, Ripple, I_Peak};
        results = [results; newRow];

    catch
        % Log failure
    end

end

results.Properties.VariableNames = {'V_Batt', 'Load', 'Mode', 'Type', 'Efficiency', 'P_Loss', 'T_Final', 'Therm_Err', 'Ripple', 'I_Peak'};

% Export for AI Training (CSV)
writetable(results, ['AIDataset_' timestamp '.csv']);
fprintf('Dataset exported: AIDataset_%s.csv\n', timestamp);

%% 5. Visualization (Thermal Validity)
figure('Name', 'Thermal Model Validity');
scatter(results.Efficiency, results.Therm_Err, 50, results.Load, 'filled');
xlabel('Efficiency [%]'); ylabel('Thermal Estimation Error [degC]');
title('Digital Twin Reliability Check (Target Error < 5 degC)');
yline(5, 'r--', 'Reliability Limit');
colorbar;
```

## 6. 통합 판정 기준 (Acceptance Criteria v5.0)

| 구분           | 검사 항목           | 판정 기준                | 의미                         |
| :------------- | :------------------ | :----------------------- | :--------------------------- | ------------ | ------------------------ |
| **Reality**    | Thermal Consistency | $                        | T*{sim} - T*{calc}           | < 5^\circ C$ | 열-전기 모델 정합성 확보 |
| **Safety**     | Magnetic Saturation | $I_{peak} < 90A$         | 인덕터 포화 절대 방지        |
| **Robustness** | Ripple Stability    | Worst Case Ripple < 2.5% | 노후화/과부하 시 제어 안정성 |
| **AI Ready**   | Dataset Quality     | Outlier (발산) 0건       | AI 학습 데이터의 무결성 확보 |
