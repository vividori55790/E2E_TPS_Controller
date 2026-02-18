DAB 컨버터 TPS 제어: 효율 최적화 및 노후화 대응 로드맵

이 문서는 기존의 '과도한 Robust' 방식에서 벗어나, **최대 효율(Max Efficiency)**을 달성하면서도 현실적인 **파라미터 변화(Aging)**에 대응하기 위한 단계별 가이드입니다.

Phase 1. [Offline] 최적 효율 LUT 재생성 (The Golden Map)

가장 먼저 할 일은 시뮬레이션 상에서 SPS보다 확실히 효율이 높은 $D_1, D_2, \phi$ 조합을 찾는 것입니다.

1.1 목적 함수(Objective Function) 수정

기존의 복잡한 Cost Function을 버리고, 전류 실효값($I_{rms}$) 최소화에 집중합니다.

변경 전: $Cost = Loss + PowerErr + ZVS_{penalty} + Robust_{score}$

변경 후: 

$$Minimize \ J(D_1, D_2, \phi) = I_{rms}(D_1, D_2, \phi) + P_{ZVS\_Penalty}$$

$P_{out} = P_{ref}$ (등식 제약 조건: 전력은 반드시 맞춰야 함)

$ZVS$ (부등식 제약 조건: Soft Switching 영역 내에 있어야 함)

1.2 최적화 알고리즘 교체 (Global Search)

단순 랜덤 탐색(Perturbation) 대신, 전체 영역을 훑는 알고리즘을 사용해야 SPS가 놓치는 고효율 지점(Triangular/Trapezoidal Current Mode)을 찾을 수 있습니다.

추천 알고리즘:

Particle Swarm Optimization (PSO): DAB 최적화 논문에서 가장 많이 사용. 구현이 쉽고 수렴이 빠름.

Grid Search + fmincon: 전체 $D_1, D_2$ 범위를 0.05 단위로 쪼개서 전수 조사(Grid) 후, 가장 좋은 지점에서 미세 조정(Local Search).

1.3 ZVS 마진(Safety Margin) 확보

노후화를 대비해 LUT 생성 단계에서 최소한의 안전 마진을 둡니다.

이론적 ZVS 경계가 $I_{min} > 0$ 이라면, LUT 생성 시 $I_{min} > I_{margin}$ (예: 0.5A)로 설정합니다.

효과: 인덕턴스($L$)가 5~10% 변해도 ZVS가 깨지지 않는 "방어적인 최적점"을 찾게 됩니다. 효율 손실은 0.1% 미만입니다.

Phase 2. [Runtime] 제어기 구조 개선 (Step 1 & 2)

생성된 LUT를 MCU에 탑재할 때, 단순히 값을 불러오는 것을 넘어 피드백 제어와 결합해야 합니다.

2.1 Feedforward + Feedback 구조 (필수)

LUT 값만 믿고 제어하면 실제 소자 오차 때문에 원하는 전력이 나오지 않습니다.

LUT (Feedforward): 현재 $V_{in}, V_{out}, P_{ref}$에 맞는 최적의 비율($D_1, D_2, \phi_{base}$)을 출력.

PI Controller (Feedback): 실제 전압($V_{out}$)과 목표 전압($V_{ref}$)의 오차를 보상하는 추가 위상차($\Delta \phi$) 생성.

$$\phi_{final} = \phi_{LUT}(P_{ref}) + \Delta \phi_{PI}$$

동작 원리: $L$이 노후화되어 전력이 부족하면, PI 제어기가 자동으로 $\phi$를 늘려서 전력을 맞춥니다. $D_1, D_2$는 LUT가 정해준 "최적 효율 비율"을 유지하므로 고효율 운전이 가능합니다.

2.2 데드타임 보상 (Dead-time Compensation)

실제 하드웨어에서는 데드타임 때문에 전압 강하가 발생하여 전력 전달량이 줄어듭니다.

대응: $\phi$ 지령치에 데드타임만큼의 위상각을 미리 더해주거나, PI 제어기의 이득(Gain)을 충분히 확보하여 보상합니다.

Phase 3. [Advanced] 온라인 파라미터 추정 (Step 3)

고신뢰성 시스템(전기차 충전기, 위성 등)을 위해 소자 노후화를 실시간으로 감지하는 단계입니다.

3.1 인덕턴스($L$) 실시간 추정 원리

DAB의 전력 전달 공식은 $P = \frac{V_1 V_2}{2 \pi f L} \phi (1 - \frac{|\phi|}{\pi})$ 형태입니다.

나머지 변수($V_1, V_2, f, \phi, P_{meas}$)는 센서로 알 수 있습니다.

유일한 미지수는 $L$ (누설 인덕턴스)입니다.

3.2 구현 알고리즘 (Recursive Least Squares - RLS)

MCU 내부에서 1초 또는 1분 주기로 $L$ 값을 역산합니다.

# 개념 코드 (Pythonic Pseudo-code)
def estimate_inductance(V_in, V_out, Phi_curr, P_measured):
    # 이론적 전력 수식 (L 제외)
    K = (V_in * V_out) / (2 * pi * f) * Phi_curr * (1 - abs(Phi_curr)/pi)
    
    # L_estimated = K / P_measured
    # 노이즈를 피하기 위해 이동 평균 필터(Moving Average) 적용
    L_new = K / P_measured
    L_estimated = 0.99 * L_estimated + 0.01 * L_new 
    
    return L_estimated
