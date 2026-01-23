# 🚀 Hybrid DAB Converter Project: Grand Master Plan v2.0

이 문서는 단순 SPS 제어를 넘어, **AI 기반(HDC) TPS 제어 및 유전 알고리즘(GA) 최적화가 적용된 고효율 하이브리드 UPS 시스템**을 구축하기 위한 구체적인 실행 계획입니다.

---

## 📅 Phase 1: 기반 모델의 현실화 (Deep Modeling)

**목표**: 이상적인 전압원을 제거하고, 실제 배터리 특성(SOC, 과도응답)과 하드웨어 손실을 시뮬레이션에 반영합니다.

### 1-1. 배터리 등가 회로 (Thevenin Model) 상세 구현

- **구현 방법**: Simulink 기본 블록만 사용하여 투명하게 구현 (Simscape Battery 블록 미사용 권장 - 커스텀 용이성 위함)
- **회로 구성**:
  - **OCV(Open Circuit Voltage)**: `1-D Lookup Table` (Input: SOC → Output: Voltage)
  - **R_series (내부 저항)**: 충/방전 시 전압 강하 유발
  - **RC Pair (과도 응답)**: 1차 RC 회로 ($R_{transient} || C_{transient}$) 직렬 연결
- **SOC 계산 로직**:
  ```matlab
  % Simulink 블록 수식
  SOC(t) = SOC_init - (1 / (Capacity_Ah * 3600)) * Integral(I_batt)
  ```
- **데이터셋 (Li-ion 96V pack 예시)**:
  - **SOC**: `[0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]`
  - **OCV**: `[84.0, 90.0, 93.0, 94.4, 95.6, 96.6, 98.0, 100.2, 103.0, 106.4, 109.2]`
- **파라미터 추천 (96V 시스템)**:
  - Capacity: 50 Ah
  - $R_{series}$: 0.04 $\Omega$
  - $R_{transient}$: 0.02 $\Omega$, $C_{transient}$: 1000 F
- **⚡ 0.5초 시뮬레이션용 팁 (Fast Dynamics)**:
  - 짧은 시간에 충방전 변화를 보려면 **용량과 C값을 대폭 축소**해야 함
  - Capacity: **0.02 Ah** (Gain 블록 수정: `-1/(0.02*3600)`)
  - $C_{transient}$: **2 F** (시정수 0.04초)

### 1-2. 실시간 효율 및 손실 측정기 (Efficiency Meter)

- **목표**: 시뮬레이션 실행 중 실시간으로 효율(%)을 스코프로 확인
- **구현 방법**:
  1.  **입력 전력($P_{in}$)**: $V_{in} \times I_{in}$ → `Mean (Running Average)` 블록 통과
  2.  **출력 전력($P_{out}$)**: $V_{out} \times I_{out}$ → `Mean` 블록 통과
  3.  **손실 모델링 (옵션)**:
      - 스위칭 손실 ($P_{sw}$): $f_{sw} \times (E_{on} + E_{off})$ (Look-up Table로 근사)
      - 도통 손실 ($P_{cond}$): $I_{rms}^2 \times R_{ds(on)}$
  4.  **효율 계산**: $\eta = (P_{out} / P_{in}) \times 100 [\%]$

### 1-3. 소프트 스타트 (Soft-Start) 및 돌입 전류 방지

- **현상**: DC Link의 $R-L$ 임피던스로 인해 초기 기동 시 과도한 리플과 돌입 전류 발생
- **해결책 1 (회로)**: DC Link 커패시터의 `Initial Voltage`를 입력 전압과 동일하게 설정 (시뮬레이션 초기화 팁)
- **해결책 2 (제어)**: **Soft-Start 로직** 구현
  - 시뮬레이션 시작 후 0~0.05초 동안은 위상 제한값($\phi_{limit}$)을 0에서 90도까지 서서히 증가시킴
  - 또는 전류 지령 $I_{ref}$를 **Ramp 함수**를 통해 서서히 증가시킴
  - `DAB_Controller_System.m` 내부에 `SoftStartTimer` 상태 변수 추가 예정

---

## 🧠 Phase 2: 지능형 에너지 관리 (Smart BMS Logic)

**목표**: 충/방전 시나리오와 비상 상황(UPS)을 관리하는 상위 두뇌를 만듭니다.

### 2-1. CC-CV 충전 제어기 상세

- **구조**: `Switch` 블록을 사용하여 제어 모드 변경
- **알고리즘**:
  1.  **CC 모드**: 배터리 전압 < $V_{set\_limit}$ (54V)일 때 → 전류 제어(SPS) 수행
  2.  **CV 모드**: 배터리 전압 $\ge V_{set\_limit}$일 때 → 전압 제어(PI)로 전환하여 전류를 서서히 줄임
  3.  **Cut-off**: 전류 < 0.5A 도달 시 → PWM Off (Floating)

### 2-2. 하이브리드 운영 로직 (Reserve Strategy)

- **구현 도구**: `Stateflow` Chart 또는 `MATLAB Function`
- **핵심 로직**:
  ```matlab
  if (Mode == ESS_DISCHARGE)
      if (SOC < 0.2) % 20% 미만이면
          Stop_Discharge(); % 방전 중단 (UPS용 에너지 보존)
          Alert("Battery Low - Reserved for UPS");
      end
  elseif (Mode == UPS_MODE)
      Allow_Discharge_Until(SOC > 0.05); % 비상시엔 5%까지 쥐어짜기
  end
  ```

---

## ⚙️ Phase 3: 제어 최적화 (Optimization with GA)

**목표**: 사람의 직관이 아닌, 데이터 기반의 최적 PI 계수($K_p, K_i$) 도출.

### 3-1. MATLAB 유전 알고리즘 (GA) 파이프라인

- **도구**: Global Optimization Toolbox (`ga` 함수)
- **스크립트 구조 (`optimize_dab.m`)**:
  1.  **변수 설정**: 최적화할 변수 `x = [Kp_ext, Ki_ext]`
  2.  **비용 함수 (Cost Function)**:
      - Simulink를 `sim()` 명령어로 0.1초간 실행
      - 에러 데이터(`Error_Signal`) 수집
      - $$Cost = \int (t \times |Error|) dt + (Overshoot \times Weight)$$ (ITAE 기준)
  3.  **실행**: `ga(@cost_func, 2, ...)` 실행하여 비용이 최소화되는 계수 탐색

### 3-2. 최적화 후 검증

- 도출된 파라미터를 적용하여 Step Response 테스트 (2A → 8A 부하 변동) 수행
- 최적화 전/후의 정착 시간(Settling Time) 및 오버슈트 비교 리포트 작성

---

## 🚀 Phase 4: 차세대 제어 기법 (AI-TPS Control)

**목표**: 모든 동작점에서 ZVS(Zero Voltage Switching)를 달성하고 무효 전력을 최소화하는 TPS 제어.

### 4-1. TPS (Triple Phase Shift) 제어기 구조 변경

- **현재**: $D_1=0.5, D_2=0.5$ (고정)
- **변경**: $D_1, D_2, \phi$ 세 가지 변수를 모두 조작
- **구현**: `DAB_Controller_System.m`에 입력 포트 추가 및 내부 로직 수정
  - 입력: `Optimal_D1`, `Optimal_D2`, `Optimal_Phi`

### 4-2. HDC (HyperDimensional Computing) 학습 및 적용

- **데이터셋 생성 (`gen_tps_data.m`)**:
  - 전압비($k = V_{in}/nV_{out}$)와 부하($P$)를 조금씩 바꿔가며 10,000개 포인트 생성
  - 각 포인트마다 무효전력이 최소가 되는 $(D_1, D_2, \phi)$를 수치해석(KKT 조건 등)으로 계산
- **HDC 학습**:
  - 입력($k, P$)을 초고차원 벡터(Hypervector)로 인코딩
  - Regression(회귀) 학습을 통해 연관 메모리(Associative Memory) 구축
- **임베디드 적용**:
  - 학습된 가중치 행렬(Matrix)만 있으면 단순 행렬 곱셈으로 최적의 $D_1, D_2$ 실시간 추론 가능 (연산량 매우 적음)

---

## � 실행 순서 (Action Plan)

1.  **[오늘]**: Phase 1-1 배터리 등가 회로 구현 및 모델링 완료
2.  **[내일]**: Phase 3-1, 3-2 GA 최적화 스크립트 작성 및 **SPS 제어기 튜닝 완료**
3.  **[모레]**: Phase 2 BMS 로직(CC-CV, Reserve) 추가
4.  **[이후]**: Phase 4 TPS/HDC 연구 및 구현 시작
