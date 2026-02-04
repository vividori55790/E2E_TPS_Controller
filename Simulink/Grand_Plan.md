# 🚀 Hybrid DAB Converter Project: Grand Master Plan v3.2

이 문서는 단순 SPS 제어를 넘어, **HDC(Hyperdimensional Computing) 기반의 지능형 TPS(Triple Phase Shift) 제어**가 적용된 고성능 하이브리드 UPS 시스템을 구축하기 위한 구체적인 실행 계획입니다.

**[v3.2 주요 변경점]**

- **Paradigm Shift**: TPS 제어의 핵심 구현 방법론을 **HDC(Hyperdimensional Computing)**로 변경.
- **Restructuring**: 기존 Phase 6(AI/Future Work)에 있던 HDC 내용을 Phase 4의 메인 솔루션으로 격상.
- **Workflow**: `데이터 수집 (Offline) -> HDC 학습 -> 실시간 추론 제어` 프로세스 구체화.

---

## 📅 Phase 1: 기반 모델의 현실화 (Deep Modeling)

**목표**: 이상적인 전압원을 제거하고, 실제 배터리 특성(SOC, 과도응답)과 하드웨어 손실을 시뮬레이션에 반영합니다.

### 1-1. 배터리 등가 회로 (Thevenin Model) 상세 구현

- **구현 방법**: Simulink 기본 블록만 사용하여 투명하게 구현.
- **회로 구성**: OCV(Lookup Table) + R_series + RC Pair (Transient).

### 1-2. 실시간 효율 및 손실 측정기 (Efficiency Meter) [진행 중]

- **목표**: 시뮬레이션 실행 중 실시간으로 효율(%)을 스코프로 확인.
- **구현 방법**:
  1.  **입력/출력 전력 측정**: $P_{in} = V_{in} \cdot I_{in}$, $P_{out} = V_{out} \cdot I_{out}$.
  2.  **효율 계산**: $\eta = (P_{out} / P_{in}) \times 100 [\%]$.

### 1-3. 디지털 제어 지연 (Digital Control Delay)

- **목표**: 실제 MCU(DSP)의 샘플링 및 연산 지연 반영.
- **구현**: 제어 입력 루프에 Unit Delay(1~2 Sample) 추가.

---

## 🧠 Phase 2: 지능형 에너지 관리 (Smart BMS Logic)

**목표**: **Finite State Machine (FSM)** 기반의 강건한 상태 관리 및 배터리 보호 로직.

### 2-1. 스마트 충전/방전 FSM

- **충전**: CC(정전류) -> CV(정전압) -> Cut-off.
- **방전**: ESS Reserve (20%) / UPS Deep Discharge (5%) 관리.
- **구현 상태**: `DAB_Controller_System.m` 내부에 FSM 구조 이미 반영됨.

---

## 🛡️ Phase 3: 상용급 강건 설계 (Robustness & Protection)

**목표**: 실제 하드웨어 보호를 위한 안전장치 구축.

### 3-1. 변압기 포화 방지 (Active Flux Balancing) [완료]

- 1차측 전류 DC 오프셋을 0으로 만드는 Duty 보정 루프.

### 3-2. 데드타임 보상 (Dead-time Compensation)

- 실제 하드웨어의 데드타임(Dead-time)으로 인한 전압 손실 및 왜곡 보상.

---

## ⚡ Phase 4: HDC 기반 초고효율 TPS 제어 (Core Innovation) **[Current Priority]**

**목표**: 복잡한 연산이 필요한 TPS 최적해를 **HDC(Hyperdimensional Computing)**의 연관 메모리 추론을 통해 실시간으로 도출합니다.

### 4-1. TPS 최적 데이터셋 구축 (Offline - Scientific)

- **Input (Context)**: 전압비 ($k = V_{in}/V_{out}$), 부하 전류 ($I_{req}$).
- **Output (Label)**: 효율을 극대화(무효전력 최소화/ZVS 달성)하는 최적의 제어 변수 $(D_1, D_2, \phi)$.
- **방법**: MATLAB 최적화 툴박스를 이용해 다양한 동작 조건에 대한 등고선(Look-up Table 수준) 데이터 생성.

### 4-2. Reality Validation & AI Data Generation (Offline - Empirical)

> **Refer to:** `Simulink/Readme_Efficiency.md`

- **목표**: 수식 모델의 한계를 극복하기 위해 실제 시뮬레이션(물리적 지연, 데드타임, 열적 변화 반영) 기반의 데이터를 수집.
- **Protocol**: `Readme_Efficiency.md`의 v5.0 Master Verification Plan에 따라 Electro-Thermal Loop가 적용된 고정밀 데이터를 확보.
- **활용**: HDC/RL Agent의 Fine-Tuning 및 Reality Gap 최소화.

### 4-3. HDC 모델 학습 (Encoding & Training)

- **Encoding**: 연속적인 상태 변수($k, I_{req}$)를 고차원 벡터(Hypervector)로 변환 (예: RBF Projection, Cyclic Encoding).
- **Training**: 최적 제어 변수($D_1, D_2, \phi$)와 연관된 상태 벡터들을 결합(Bundling)하여 **Associative Memory** 구축.
- **장점**: LUT 대비 메모리 효율적이며, 노이즈에 강건하고 연산 속도가 매우 빠름.

### 4-4. 실시간 추론 제어기 구현 (Runtime Inference)

- **Simulink Block**: 학습된 HDC 메모리를 탑재한 MATLAB Function 블록 구현.
- **Process**: `Sensing -> Encoding -> Similarity Search (Query) -> Decoding -> Apply TPS Control`.

### 4-5. Seamless Transition (Hybrid Mode)

- 경부하/특정 조건에서는 HDC 추론 결과가 불안정할 수 있으므로, 기존 SPS 제어와의 **Hybrid 운용** 전략 수립.
- 모드 전환 시 Hysteresis 적용하여 과도 응답 최소화.

---

## 🎯 Phase 5: 제어 루프 최적화 (Post-Tuning)

**목표**: HDC가 제공하는 최적 동작점 위에서 미세 조정(Fine-tuning)을 수행합니다.

### 5-1. 베이지안 최적화 (Bayesian Optimization)

- HDC는 '최적의 경로(Trajectory)'를 제시하고, PI 제어기는 그 경로를 잘 따라가도록 $K_p, K_i$ 이득을 튜닝.
- **Reference Tracking** 성능 극대화.

---

## 실행 순서 (Updated Action Plan)

1.  **[완료]**: Phase 4-1 TPS 수식적 최적해 도출 (`Generate_TPS_Dataset.m`).
2.  **[진행 중]**: Phase 4-2 Reality Validation 및 학습 데이터 생성 (`Readme_Efficiency.md` 참조).
3.  **[다음]**: Phase 4-3/4-4 HDC 인코더 및 추론 엔진 구현.
4.  **[통합]**: HDC 제어기를 `DAB_Controller_System`에 통합 및 검증.
5.  **[마무리]**: Phase 5 베이지안 최적화로 과도 응답 성능 튜닝.
