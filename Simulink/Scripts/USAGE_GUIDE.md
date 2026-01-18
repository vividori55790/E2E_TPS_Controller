# 🚀 MATLAB 스크립트 사용 가이드

이 문서는 최근 추가된 MATLAB 스크립트들의 사용 방법을 설명합니다.

---

## 📂 파일 구조

```
Simulink/
├── DAB_System_Config.m           # 중앙 설정 파일
├── DAB_Controller_System.m        # 제어기 클래스
├── DAB_Plant_TPS.m               # Full TPS Plant
├── DAB_Plant_TPS_v2.m            # SPS Plant (빠름)
└── Scripts/
    ├── Auto_Optimize_And_Verify.m    ⭐ 추천
    ├── Optimize_Controller_MIL.m
    ├── Quick_MIL_Test.m
    ├── Run_Simulink_Compare_MIL.m
    └── Verify_Optimized_Controller.m
```

---

## 🎯 주요 스크립트 사용법

### 1. **Auto_Optimize_And_Verify.m** ⭐ (가장 추천)

**기능:** 최적화 + 검증을 한 번에 수행

**실행 방법:**

```matlab
cd Simulink/Scripts
Auto_Optimize_And_Verify
```

**수행 작업:**

1. **2단계 Grid Search**로 최적 Kp/Ki 탐색 (50회 시뮬레이션)
2. 7개 시나리오에서 자동 검증
   - Nominal (400V, 10A)
   - Low Voltage (350V, 10A)
   - High Voltage (450V, 10A)
   - Low Current (400V, 5A)
   - High Current (400V, 15A)
   - Low Battery (400V, 10A, V_batt=90V)
   - High Battery (400V, 10A, V_batt=110V)
3. Pass/Fail 리포트 출력
4. 결과 시각화 (6개 플롯)

**출력 예시:**

```
=== STEP 1: PI 제어기 2단계 최적화 ===
--- Stage 1: Coarse Grid Search ---
  Coarse 최적점: Kp=1.25, Ki=450 (Cost=28.5)
--- Stage 2: Fine Grid Search ---
  Fine 최적점: Kp=1.183, Ki=472.5 (Cost=21.3)

최적화 통계:
  총 시뮬레이션 횟수: 50 (기존 100 → 50% 감소)
  소요 시간: 4.82 sec

=== STEP 3: 시나리오별 검증 실행 ===
[1/7] Nominal 검증 중... ✅ PASS
[2/7] Low Voltage 검증 중... ✅ PASS
...

총 7/7 시나리오 통과 (100%)
```

**소요 시간:** ~30초

---

### 2. **Optimize_Controller_MIL.m**

**기능:** PI 제어기 최적화만 수행 (검증 없음)

**실행 방법:**

```matlab
cd Simulink/Scripts
Optimize_Controller_MIL
```

**수행 작업:**

- 10×10 Grid Search (100회 시뮬레이션)
- 최적 Kp/Ki 탐색
- `DAB_Controller_System.m` 및 `DAB_System_Config.m` 자동 업데이트

**소요 시간:** ~10초

---

### 3. **Quick_MIL_Test.m**

**기능:** MIL Plant 모델 빠른 검증

**실행 방법:**

```matlab
cd Simulink/Scripts
Quick_MIL_Test
```

**수행 작업:**

- 30ms 단순 시뮬레이션
- 현재 Kp/Ki로 성능 확인
- 6개 플롯 생성

**소요 시간:** ~2초

---

### 4. **Run_Simulink_Compare_MIL.m**

**기능:** Simulink vs MIL 비교

**실행 방법:**

```matlab
cd Simulink/Scripts
Run_Simulink_Compare_MIL
```

**수행 작업:**

1. 동적 시나리오 생성 (100ms)
2. Simulink 시뮬레이션 실행
3. MIL 시뮬레이션 실행
4. 결과 비교 플롯

**소요 시간:** ~5초 (최적화 후)

**주의사항:**

- Simulink 모델 경로가 올바른지 확인
- `Control_Logic.slx`가 `../Modules/` 폴더에 있어야 함

---

### 5. **Verify_Optimized_Controller.m**

**기능:** 최적화된 제어기 검증 (긴 시뮬레이션)

**실행 방법:**

```matlab
cd Simulink/Scripts
Verify_Optimized_Controller
```

**수행 작업:**

- 50ms 시뮬레이션
- 성능 지표 계산 (오버슈트, 정상상태 오차, 정착시간)
- 4개 플롯 생성

**소요 시간:** ~3초

---

## 🔧 파라미터 변경 방법

모든 파라미터는 `DAB_System_Config.m`에서 중앙 관리됩니다.

### 예시: 배터리 전압 변경

```matlab
% 파일 열기
edit DAB_System_Config.m

% Line 28 찾기:
config.battery.V_nom = 100;  % 공칭 전압 (V)

% 값 변경:
config.battery.V_nom = 120;  % 120V로 변경

% 저장 (Ctrl+S)
```

**변경 후 MATLAB 재시작 필요:**

```matlab
clear all  % 캐시 초기화
```

### 주요 변경 가능 파라미터:

| 파라미터        | 위치                        | 기본값      |
| --------------- | --------------------------- | ----------- |
| 배터리 전압     | `config.battery.V_nom`      | 100V        |
| 배터리 내부저항 | `config.battery.R_internal` | 0.1Ω        |
| HV 전압         | `config.hv.V_nom`           | 400V        |
| 스위칭 주파수   | `config.plant.fs`           | 80kHz       |
| Kp 탐색 범위    | `config.optim.Kp_range`     | [0.1, 3.0]  |
| Ki 탐색 범위    | `config.optim.Ki_range`     | [100, 1000] |

---

## 🎨 출력 플롯 해석

### Auto_Optimize_And_Verify.m

1. **2-Stage Grid Search** (subplot 1)
   - 동그라미: Coarse 탐색 포인트
   - 사각형: Fine 탐색 포인트
   - 빨간 별: 최종 최적점

2. **오버슈트 비교** (subplot 2)
   - 막대: 각 시나리오별 오버슈트
   - 빨간 선: 목표값 (5%)

3. **정상상태 오차** (subplot 3)
   - 막대: 각 시나리오별 정상상태 오차
   - 빨간 선: 목표값 (2%)

---

## ⚡ 성능 최적화 내역

### 최적화 전 (기존)

- Config 로드: ~1초
- Simulink+MIL: ~15초
- **총 ~16초**

### 최적화 후

- Config 로드: ~0.001초 (Persistent 캐싱)
- Simulink+MIL: ~5초 (루프 최적화)
- **총 ~5초** (**3배 빠름**)

**주요 최적화 기법:**

1. ✅ Persistent 캐싱 (Config, TPS Table, HDC Model)
2. ✅ 조건부 `addpath()` (한 번만 실행)
3. ✅ 구조체 배치 할당 (`assignin()` 최적화)
4. ✅ 루프 외부 최적화 (`exist()` 체크 제거)
5. ✅ 벡터화 연산

---

## 🆘 문제 해결

### "TPS_Table.mat not found"

```matlab
cd Simulink
TPS_LUT  % TPS Table 생성
```

### "Control_Logic.slx not found"

```matlab
% Simulink 모델 경로 확인
cd Simulink/Modules
dir Control_Logic.slx
```

### 캐시 초기화 필요 시

```matlab
clear all
close all
clc
```

---

## 📝 권장 워크플로우

### 초기 설정

```matlab
1. cd Simulink
2. TPS_LUT  % TPS Table 생성 (첫 1회만)
3. train_hdc_model  % HDC Model 생성 (선택사항)
```

### 일반 사용

```matlab
1. cd Scripts
2. Auto_Optimize_And_Verify  % 최적화 + 검증
   또는
   Quick_MIL_Test  % 빠른 테스트
```

### 파라미터 튜닝

```matlab
1. edit ../DAB_System_Config.m  % 파라미터 변경
2. clear all  % 캐시 초기화
3. Auto_Optimize_And_Verify  % 재최적화
```

---

이 가이드로 모든 스크립트를 효율적으로 사용할 수 있습니다! 🎉
