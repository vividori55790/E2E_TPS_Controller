# 🚀 Hybrid DAB Converter Project: Grand Master Plan v3.3

이 문서는 단순 SPS 제어를 넘어, **AI(Artificial Intelligence) 기반의 지능형 TPS(Triple Phase Shift) 제어**가 적용된 고성능 하이브리드 UPS 시스템을 구축하기 위한 구체적인 실행 계획입니다.

**[v3.3 주요 변경점]**

-   **Deep Learning Adoption**: HDC 개념을 확장하여 **Neural Network(Regression Model)** 기반의 고정밀 제어 도입.
-   **Robust Dataset**: 물리 엔진 기반의 **100만 샘플 대규모 데이터셋** 구축 계획 확정.
-   **Workflow Refinement**: `Data Gen -> AI Training -> Validation -> Deployment` 4단계 파이프라인 정립.

---

## 📅 Phase 1: 기반 모델의 현실화 (Deep Modeling)

**목표**: 이상적인 전압원을 제거하고, 실제 배터리 특성(SOC, 과도응답)과 하드웨어 손실을 시뮬레이션에 반영합니다.

### 1-1. 배터리 등가 회로 (Thevenin Model) 상세 구현

-   **구현 방법**: Simulink 기본 블록만 사용하여 투명하게 구현.
-   **회로 구성**: OCV(Lookup Table) + R_series + RC Pair (Transient).

### 1-2. 실시간 효율 및 손실 측정기 (Efficiency Meter) [진행 중]

-   **목표**: 시뮬레이션 실행 중 실시간으로 효율(%)을 스코프로 확인.
-   **구현 방법**:
    1.  **입력/출력 전력 측정**: $P_{in} = V_{in} \cdot I_{in}$, $P_{out} = V_{out} \cdot I_{out}$.
    2.  **효율 계산**: $\eta = (P_{out} / P_{in}) \times 100 [\%]$.

### 1-3. 디지털 제어 지연 (Digital Control Delay)

-   **목표**: 실제 MCU(DSP)의 샘플링 및 연산 지연 반영.
-   **구현**: 제어 입력 루프에 Unit Delay(1~2 Sample) 추가.

---

## 🧠 Phase 2: 지능형 에너지 관리 (Smart BMS Logic)

**목표**: **Finite State Machine (FSM)** 기반의 강건한 상태 관리 및 배터리 보호 로직.

### 2-1. 스마트 충전/방전 FSM

-   **충전**: CC(정전류) -> CV(정전압) -> Cut-off.
-   **방전**: ESS Reserve (20%) / UPS Deep Discharge (5%) 관리.
-   **구현 상태**: `DAB_Controller_System.m` 내부에 FSM 구조 이미 반영됨.

---

## 🛡️ Phase 3: 상용급 강건 설계 (Robustness & Protection)

**목표**: 실제 하드웨어 보호를 위한 안전장치 구축.

### 3-1. 변압기 포화 방지 (Active Flux Balancing) [완료]

-   **1차측 전류 DC 오프셋을 0으로 만드는 Duty 보정 루프.**

### 3-2. 데드타임 보상 (Dead-time Compensation)

-   **실제 하드웨어의 데드타임(Dead-time)으로 인한 전압 손실 및 왜곡 보상.**

---

## ⚡ Phase 4: AI 기반 초고효율 TPS 제어 (Core Innovation) **[Current Priority]**

**목표**: 복잡한 비선형 TPS 최적해를 **AI(Neural Network)** 추론을 통해 실시간으로 도출하여 전 구간 효율을 극대화합니다.

### 4-1. TPS 최적 데이터셋 구축 (Big Data Generation) **[진행 중]**

-   **Input**: $V_{in}, V_{out}, P_{ref}$ (전압 및 양방향 전력 지령).
-   **Output**: 최적의 TPS 변수 $(D_1, D_2, \phi)$ 및 예상 효율.
-   **Physics Engine v3.0**:
    -   정확한 2차측 전도 손실($I^2 R \cdot n^2$), 코어 손실(Steinmetz), 스위칭/데드타임 손실 반영.
    -   순방향/역방향 효율 수식 분리 적용.
-   **Scale**: 100만 샘플 (Stratified Sampling으로 전 영역 커버).
-   **File**: `generate_training_data.m`

### 4-2. AI 제어 모델 학습 (Neural Network Training) **[Next Step]**

-   **Architecture**: Feedforward Neural Network (Hidden Layers: 64-32).
-   **Training**: Levenberg-Marquardt 알고리즘 사용 (`trainlm`).
-   **Goal**: MSE < 1e-6, R-value > 0.999.
-   **File**: `train_hdc_network.m`

### 4-3. 실시간 추론 제어기 검증 (Validation)

-   **Process**: 학습된 모델(`trained_hdc_model.mat`)을 Simulink 내 MATLAB Function 또는 System Object로 탑재.
-   **Verification**:
    1.  Step Response: 급격한 부하 변동 시 과형/정착 시간 확인.
    2.  Efficiency Map: 시뮬레이션 결과가 예측된 고효율(97%+)을 달성하는지 검증.

### 4-4. 배포 및 하드웨어 구현 (Deployment)

-   **Deployment**: MATLAB Coder를 이용해 학습된 신경망을 C 코드로 변환하거나, Look-up Table로 경량화하여 DSP 탑재.

---

## 🎯 Phase 5: 제어 루프 최적화 (Post-Tuning)

**목표**: AI가 제공하는 최적 동작점(Feedforward) 위에서 피드백 제어기의 미세 조정(Fine-tuning)을 수행합니다.

### 5-1. 하이브리드 제어 구조

-   **Feedforward**: AI 모델이 $(D_1, D_2, \phi)$의 기준값을 생성.
-   **Feedback**: PI 제어기가 미세 오차를 보정하여 강건성 확보.

---

## 실행 순서 (Action Plan - Updated)

1.  **[진행 중]**: `generate_training_data.m` 실행하여 **100만 개 고품질 데이터셋** 생성 (물리 기반 검증 포함).
2.  **[대기]**: `train_hdc_network.m` 실행하여 **AI 제어 모델 학습**.
3.  **[예정]**: 학습된 모델을 Simulink(`Main_Simulation.slx`)에 탑재하여 **효율 및 동특성 검증**.
4.  **[마무리]**: 최종 보고서 작성 및 성능 지표(효율 맵) 도출.
