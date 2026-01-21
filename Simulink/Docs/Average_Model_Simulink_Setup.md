# 평균화 모델 Simulink 회로 구성 가이드

## Control_Logic_Avg.slx 생성 방법

---

## 1. 전체 블록 다이어그램

```
┌────────────────────────────────────────────────────────────────────────────┐
│                        Control_Logic_Avg.slx                               │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      INPUT SECTION                                  │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                                                                     │   │
│  │  [From Workspace]      [From Workspace]      [From Workspace]       │   │
│  │  Ref_VH (V_source)     Ref_I_ess_cmd         Load_Change            │   │
│  │       ↓                      ↓                     ↓                │   │
│  │       │                      │                     │                │   │
│  │  ┌────────────────────────────────────────────────────────────┐     │   │
│  │  │              Control Parameters (From Workspace)          │     │   │
│  │  │  [Kp_ext]  [Ki_ext]  [Kpv]  [Kiv]                        │     │   │
│  │  │     ↓         ↓        ↓      ↓                          │     │   │
│  │  └────────────────────────────────────────────────────────────┘     │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      CONTROLLER SECTION                             │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                                                                     │   │
│  │          ┌─────────────────────────────────────┐                    │   │
│  │          │      DAB_Controller_System          │                    │   │
│  │          │      (MATLAB System Block)          │                    │   │
│  │          ├─────────────────────────────────────┤                    │   │
│  │  V_HV ──>│ 1. V_HV                             │                    │   │
│  │  I_HV ──>│ 2. I_HV      t1, t3, t5, t7 ───────>│ (미사용, 평균화)   │   │
│  │  V_LV ──>│ 3. V_LV      mode_active ──────────>│ To Workspace       │   │
│  │  I_LV ──>│ 4. I_LV      status_flag ──────────>│ To Workspace       │   │
│  │  I_cmd ─>│ 5. I_ess_cmd I_cmd_out ────────────>│ To Workspace       │   │
│  │          │              phi_ext_out ──────────>│ → Average Model    │   │
│  │  Kp_ext ─>│ 6. Kp_ext    D1_out (phi_int) ────>│ → Average Model    │   │
│  │  Ki_ext ─>│ 7. Ki_ext    D2_out ──────────────>│ To Workspace       │   │
│  │  Kpv ────>│ 8. Kpv       integrator_out ──────>│ To Workspace       │   │
│  │  Kiv ────>│ 9. Kiv                             │                    │   │
│  │          └─────────────────────────────────────┘                    │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      PLANT SECTION (Average Model)                  │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │                                                                     │   │
│  │  V_source ──────────┐                                               │   │
│  │  phi_ext_out ───────┼──> ┌───────────────────────┐                  │   │
│  │  D1*180 (phi_int) ──┼──> │   DAB_Average_Model   │                  │   │
│  │  I_load_HV ─────────┘    │   (MATLAB System)     │                  │   │
│  │                          ├───────────────────────┤                  │   │
│  │                          │ V_HV ────────> Feedback ──> Controller   │   │
│  │                          │ V_LV ────────> Feedback ──> Controller   │   │
│  │                          │ I_HV ────────> To Workspace              │   │
│  │                          │ I_LV ────────> Feedback ──> Controller   │   │
│  │                          │ P_transfer ──> To Workspace              │   │
│  │                          └───────────────────────┘                  │   │
│  │                                                                     │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                      OUTPUT SECTION (To Workspace)                  │   │
│  ├─────────────────────────────────────────────────────────────────────┤   │
│  │  out.Voltage_High     out.Voltage_Low      out.Current_Low          │   │
│  │  out.I_cmd_out        out.Phi_ext_out      out.Integrator_out       │   │
│  │  out.P_transfer       out.Status_flag                               │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                            │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 단계별 Simulink 모델 생성

### Step 1: 새 모델 생성

```
File > New > Simulink Model
Save as: Control_Logic_Avg.slx
```

### Step 2: From Workspace 블록 추가 (입력)

| 블록             | Variable Name   | 용도                    |
| ---------------- | --------------- | ----------------------- |
| From Workspace 1 | `Ref_VH`        | HV 전원 전압 (시나리오) |
| From Workspace 2 | `Ref_I_ess_cmd` | 전류 지령 (시나리오)    |
| From Workspace 3 | `Load_Change`   | 부하 전류 (시나리오)    |
| From Workspace 4 | `Kp_ext`        | 전류 PI 비례 이득       |
| From Workspace 5 | `Ki_ext`        | 전류 PI 적분 이득       |
| From Workspace 6 | `Kpv`           | 전압 PI 비례 이득       |
| From Workspace 7 | `Kiv`           | 전압 PI 적분 이득       |

**From Workspace 설정:**

- Sample time: -1 (inherited)
- Interpolation: Zero-order hold

### Step 3: DAB_Controller_System 블록 추가

```
Simulink Library > User-Defined Functions > MATLAB System
```

**설정:**

- System object name: `DAB_Controller_System`
- UseExternalParams: ☑ true

**입력 연결 (9개):**

1. V_HV ← (Average Model 출력에서 피드백)
2. I_HV ← (Average Model 출력에서 피드백)
3. V_LV ← (Average Model 출력에서 피드백)
4. I_LV ← (Average Model 출력에서 피드백)
5. I_ess_cmd ← From Workspace (Ref_I_ess_cmd)
6. Kp_ext ← From Workspace
7. Ki_ext ← From Workspace
8. Kpv ← From Workspace
9. Kiv ← From Workspace

### Step 4: DAB_Average_Model 블록 추가

```
Simulink Library > User-Defined Functions > MATLAB System
```

**설정:**

- System object name: `DAB_Average_Model`

**입력 연결 (4개):**

1. V_source ← From Workspace (Ref_VH)
2. phi_ext ← Controller 출력 (phi_ext_out)
3. phi_int ← Controller 출력 (D1_out \* 180)
4. I_load_HV ← From Workspace (Load_Change)

### Step 5: 피드백 루프 연결

```
┌────────────────┐              ┌────────────────┐
│   Controller   │ ──phi_ext──> │ Average Model  │
│                │ ──phi_int──> │                │
│   V_HV <─────────────────────── V_HV           │
│   V_LV <─────────────────────── V_LV           │
│   I_LV <─────────────────────── I_LV           │
└────────────────┘              └────────────────┘
```

**주의: 대수 루프 방지**

- 피드백 경로에 **Memory** 또는 **Unit Delay** 블록 추가
- 또는 Average Model의 InitialCondition 설정 활용

### Step 6: To Workspace 블록 추가 (출력)

| Variable Name    | 연결 소스                 | 설명        |
| ---------------- | ------------------------- | ----------- |
| `Voltage_High`   | Average Model V_HV        | HV 전압     |
| `Voltage_Low`    | Average Model V_LV        | LV 전압     |
| `Current_Low`    | Average Model I_LV        | LV 전류     |
| `I_cmd_out`      | Controller I_cmd_out      | 전류 지령   |
| `Phi_ext_out`    | Controller phi_ext_out    | 외부 위상   |
| `Integrator_out` | Controller integrator_out | 적분기 상태 |
| `P_transfer`     | Average Model P_transfer  | 전달 전력   |
| `Status_flag`    | Controller status_flag    | 동작 모드   |

**To Workspace 설정:**

- Save format: Timeseries
- Variable name: 위 표 참조
- Limit data points to last: inf
- Structure with Time 체크 해제

---

## 3. 상세 연결도

### 3.1 제어기 입력

```
From Workspace (Ref_VH) ──────────────────────────────────────┐
                                                              │
                         ┌───────────────────┐                │
Average Model V_HV ─────>│ 1. V_HV           │                │
Constant (0) ───────────>│ 2. I_HV           │                │
Average Model V_LV ─────>│ 3. V_LV           │                │
Average Model I_LV ─────>│ 4. I_LV           │                │
From Workspace (Ref) ───>│ 5. I_ess_cmd      │                │
From Workspace (Kp) ────>│ 6. Kp_ext         │                │
From Workspace (Ki) ────>│ 7. Ki_ext         │                │
From Workspace (Kpv) ───>│ 8. Kpv            │                │
From Workspace (Kiv) ───>│ 9. Kiv            │                │
                         │                   │                │
                         │ DAB_Controller_   │                │
                         │ System            │                │
                         │                   │──> phi_ext_out │
                         │                   │──> D1_out ─────┤
                         │                   │──> ... (other) │
                         └───────────────────┘                │
                                                              │
                         ┌───────────────────┐                │
 ────────────────────────│ 1. V_source <─────┘                │
Controller phi_ext ─────>│ 2. phi_ext                         │
                         │                                    │
Gain (180) ──────────────│ 3. phi_int  <── Controller D1_out  │
    └──────────────────────── (D1 * 180 = phi_int_deg)        │
                         │                                    │
From Workspace (Load) ──>│ 4. I_load_HV                       │
                         │                                    │
                         │ DAB_Average_Model │                │
                         │                   │──> V_HV ───────┤ (피드백)
                         │                   │──> V_LV ───────┤ (피드백)
                         │                   │──> I_LV ───────┤ (피드백)
                         │                   │──> ...         │
                         └───────────────────┘                │
```

### 3.2 D1_out → phi_int 변환

Controller의 `D1_out`은 0~0.14 범위의 듀티비입니다.
이를 각도로 변환:

```
phi_int_deg = D1_out * 180
```

**Simulink에서:**

- **Gain** 블록 추가
- Gain 값: 180
- D1_out → Gain → Average Model phi_int 입력

---

## 4. Model Configuration

### Solver 설정

```
Simulation > Model Configuration Parameters
```

- **Type**: Fixed-step
- **Solver**: discrete (no continuous states)
- **Fixed-step size**: 10e-6 (10µs) 또는 1/80e3 (스위칭 주기)

### Data Import/Export

```
Simulation > Model Configuration Parameters > Data Import/Export
```

- ☑ Save simulation output as single object
- Variable name: `out`
- Format: Dataset

---

## 5. Subsystem 구조 (권장)

복잡도를 줄이기 위해 Subsystem으로 그룹화:

```
Control_Logic_Avg.slx
├── Inputs (Subsystem)
│   ├── From Workspace: Ref_VH
│   ├── From Workspace: Ref_I_ess_cmd
│   ├── From Workspace: Load_Change
│   └── Control Parameters
│       ├── From Workspace: Kp_ext
│       ├── From Workspace: Ki_ext
│       ├── From Workspace: Kpv
│       └── From Workspace: Kiv
│
├── Controller (Subsystem)
│   └── DAB_Controller_System (MATLAB System)
│
├── Plant (Subsystem)
│   ├── DAB_Average_Model (MATLAB System)
│   └── Gain (D1 → phi_int 변환)
│
└── Outputs (Subsystem)
    └── To Workspace 블록들
```

---

## 6. 초기화 스크립트

모델 시작 전 Base Workspace 설정:

```matlab
%% Setup_Average_Model.m
% Control_Logic_Avg.slx 실행 전 호출

% 시뮬레이션 시간
SIM_TIME = 1.0;

% 시나리오 생성
run('Create_Scenario.m');

% 제어 파라미터 (From Workspace용)
Kp_ext = timeseries(2.0, [0; SIM_TIME]);
Ki_ext = timeseries(30.0, [0; SIM_TIME]);
Kpv = timeseries(-4.0, [0; SIM_TIME]);
Kiv = timeseries(-400.0, [0; SIM_TIME]);

% Base Workspace에 할당
assignin('base', 'Kp_ext', Kp_ext);
assignin('base', 'Ki_ext', Ki_ext);
assignin('base', 'Kpv', Kpv);
assignin('base', 'Kiv', Kiv);
assignin('base', 'SIM_TIME', SIM_TIME);

fprintf('✓ Average Model 초기화 완료\n');
```

---

## 7. 비교: 스위칭 vs 평균화

| 항목            | Control_Logic.slx (스위칭) | Control_Logic_Avg.slx (평균화) |
| --------------- | -------------------------- | ------------------------------ |
| 플랜트          | PLECS/Simscape 스위칭 회로 | DAB_Average_Model              |
| 시뮬레이션 속도 | 느림 (1초 → 5~30초)        | 빠름 (1초 → 0.1~1초)           |
| 리플            | 있음 (현실적)              | 없음 (평균값)                  |
| GA 최적화       | 시간 소요 큼               | 적합                           |
| 용도            | 최종 검증                  | 초기 튜닝, 파라미터 탐색       |

---

_Last updated: 2026-01-21_
