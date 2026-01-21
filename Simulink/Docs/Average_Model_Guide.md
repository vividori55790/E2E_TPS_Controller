# DAB 평균화 모델 설계 가이드

## Average Model for DAB Converter

---

## 1. 개요

DAB (Dual Active Bridge) 컨버터의 **평균화 모델**은 스위칭 동작을 생략하고,
**평균 전력 전달 방정식**만으로 동작하는 간소화된 모델입니다.

### 1.1 장점

- **시뮬레이션 속도 향상**: 스위칭 모델 대비 10~100배 빠름
- **GA/Optimization 적합**: 수천 회 반복 시뮬레이션에 적합
- **설계 초기 단계**: 제어기 튜닝, 파라미터 스윕에 유용
- **시스템 레벨 분석**: 전체 시스템 동작 검증

### 1.2 단점

- **스위칭 리플 없음**: ZVS/ZCS 분석 불가
- **고주파 특성 무시**: EMI, 고주파 노이즈 미반영
- **데드타임 영향 무시**: 실제 구현과 차이

---

## 2. 핵심 전력 전달 방정식

### 2.1 Single Phase Shift (SPS)

가장 기본적인 DAB 위상 제어 방식:

```
P = (V₁ · V₂') / (ω · L) · φ · (1 - |φ|/π)
```

- **V₁**: HV측 DC 전압 (V)
- **V₂'**: LV측 DC 전압을 HV측으로 환산 (V₂' = V₂ · n)
- **ω**: 각주파수 (ω = 2π·fs)
- **L**: 직렬 인덕턴스 (H)
- **φ**: 외부 위상 (rad, -π/2 ~ π/2)

### 2.2 Dual Phase Shift (DPS)

내부 위상(D)을 추가한 확장 모델:

```
P = (V₁ · V₂') / (ω · L) · D · φ_ext · (1 - D - |φ_ext|/π)
```

- **D**: 내부 위상 듀티 (D = |φ_int|/π, 0 ~ 0.5)
- **φ_ext**: 외부 위상 (rad)
- **φ_int**: 내부 위상 (rad)

---

## 3. 스위칭 모델과 동치 조건

평균화 모델이 스위칭 모델과 **동일한 DC 평균 출력**을 내기 위한 조건:

### 3.1 필수 파라미터 일치

| 파라미터           | 스위칭 모델 | 평균화 모델 |
| ------------------ | ----------- | ----------- |
| 변압기 권선비 (n)  | 4.0         | 4.0         |
| 직렬 인덕턴스 (L)  | 50 µH       | 50 µH       |
| 스위칭 주파수 (fs) | 80 kHz      | 80 kHz      |
| HV측 전압 (V_HV)   | 380 V       | 380 V       |
| LV측 전압 (V_LV)   | 100 V       | 100 V       |

### 3.2 위상각 변환

```matlab
% 스위칭 모델: 시간 기반 (t1~t7)
t_ext = |phi_ext_deg| * (Ts / 360);  % 외부 위상 → 시간

% 평균화 모델: 라디안 기반
phi_ext_rad = phi_ext_deg * pi / 180;
```

### 3.3 전류 방향 정의

| 조건      | 전력 방향      | I_HV | I_LV |
| --------- | -------------- | ---- | ---- |
| φ_ext > 0 | HV → LV (방전) | > 0  | < 0  |
| φ_ext < 0 | LV → HV (충전) | < 0  | > 0  |

---

## 4. Simulink 평균화 모델 회로 구성

### 4.1 블록 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                    Average DAB Model                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────┐    ┌──────────────────┐    ┌─────────────┐    │
│  │ V_HV    │───>│                  │───>│ I_HV        │    │
│  │ (DC)    │    │                  │    │ (Controlled)│    │
│  └─────────┘    │  DAB_Average_    │    └─────────────┘    │
│                 │  Model           │                        │
│  ┌─────────┐    │  (MATLAB System) │    ┌─────────────┐    │
│  │ V_LV    │───>│                  │───>│ I_LV        │    │
│  │ (Battery)│   │                  │    │ (Controlled)│    │
│  └─────────┘    │                  │    └─────────────┘    │
│                 │                  │                        │
│  ┌─────────┐    │                  │    ┌─────────────┐    │
│  │ φ_ext   │───>│                  │───>│ P_transfer  │    │
│  │ (Control)│   │                  │    │ (Power)     │    │
│  └─────────┘    │                  │    └─────────────┘    │
│                 │                  │                        │
│  ┌─────────┐    │                  │                        │
│  │ φ_int   │───>│                  │                        │
│  │ (DPS)   │    └──────────────────┘                        │
│  └─────────┘                                                │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 4.2 전원 모델링

#### HV측 (Grid/AC-DC)

```
┌──────────┐     ┌────────┐
│ V_HV_ref │────>│ Voltage│───> V_HV (Capacitor 전압)
│ (380 V)  │     │ Source │
└──────────┘     └────────┘
                      │
                      │ I_load (부하 전류)
                      ▼
                 ┌────────┐
                 │ C_HV   │ (부하 커패시터)
                 └────────┘
```

#### LV측 (Battery)

```
┌──────────┐     ┌────────┐
│ V_batt   │────>│ Battery│───> V_LV (배터리 전압)
│ (100 V)  │     │ Model  │
└──────────┘     └────────┘
       │              │
       │    E_oc      │ I_LV (DAB 전류)
       │    ┌─────────┤
       └───>│ R_int   │
            └─────────┘
```

---

## 5. 제어기 연동

### 5.1 From Workspace를 통한 파라미터 전달

GA 최적화 시 제어 파라미터를 외부에서 주입:

```matlab
% GA 스크립트에서
Ctrl_Params = timeseries([Kp, Ki, Kpv, Kiv], [0; SIM_TIME]);
assignin('base', 'Ctrl_Params', Ctrl_Params);

% Simulink 모델에서 From Workspace 블록 설정:
%   - Variable Name: Ctrl_Params
%   - Output data type: double
%   - Interpolation: Zero-order hold
```

### 5.2 제어기 ↔ 평균화 모델 연결

```
┌─────────────────┐     ┌──────────────────┐
│ DAB_Controller_ │     │ DAB_Average_     │
│ System          │     │ Model            │
├─────────────────┤     ├──────────────────┤
│                 │     │                  │
│ V_HV ────────────────>│ V_HV             │
│ V_LV ────────────────>│ V_LV             │
│                 │     │                  │
│ Ctrl_Params ────┤     │                  │
│                 │     │                  │
│ phi_ext_out ──────────>│ phi_ext         │
│ D1 (φ_int/180) ───────>│ phi_int         │
│                 │     │                  │
│ I_cmd_out       │<────── I_LV            │
│                 │     │                  │
└─────────────────┘     └──────────────────┘
```

---

## 6. 검증 방법

### 6.1 정적 동작점 비교

동일한 DC 전압, 위상각에서 평균 전류 비교:

```matlab
% 스위칭 모델 (1초 시뮬레이션 후 평균)
I_LV_avg_sw = mean(out_switching.I_LV.Data(end-1000:end));

% 평균화 모델
I_LV_avg = out_average.I_LV.Data(end);

% 오차 계산
error_percent = abs(I_LV_avg_sw - I_LV_avg) / abs(I_LV_avg_sw) * 100;
fprintf('Average vs Switching Error: %.2f%%\n', error_percent);
```

### 6.2 동적 응답 비교

스텝 입력에 대한 응답 비교 (리플 제외):

```matlab
% 스위칭 모델 결과를 이동평균 필터로 평활화
window_size = round(fs / 10);  % 10 kHz 대역폭
I_LV_smoothed = movmean(out_switching.I_LV.Data, window_size);

% 평균화 모델과 비교
plot(t, I_LV_smoothed, 'b-', t, out_average.I_LV.Data, 'r--');
legend('Switching (Smoothed)', 'Average');
```

---

## 7. 파일 구조

```
DAB-Converter-PSIM/Simulink/
├── DAB_Controller_System.m      # 제어기 (v3.0, 외부 파라미터 지원)
├── DAB_Average_Model.m          # 평균화 모델 (NEW)
├── DAB_System_Config.m          # 시스템 설정
│
├── Modules/
│   ├── Control_Logic.slx        # 스위칭 모델 (기존)
│   └── Control_Logic_Avg.slx    # 평균화 모델 (생성 필요)
│
└── Scripts/
    ├── GA_Optimize_Controller.m # GA 최적화 (From Workspace 지원)
    └── Compare_Sw_Avg.m         # 스위칭 vs 평균화 비교 스크립트 (TBD)
```

---

## 8. 빠른 시작 가이드

### Step 1: 평균화 모델 Simulink 생성

1. `Control_Logic.slx` 복사 → `Control_Logic_Avg.slx`
2. 스위칭 회로 삭제
3. `DAB_Average_Model` MATLAB System 블록 추가
4. 입출력 연결

### Step 2: From Workspace 블록 추가

1. Simulink Library → Sinks → From Workspace
2. Variable name: `Ctrl_Params`
3. Sample time: -1 (inherited)
4. 제어기 입력에 연결

### Step 3: GA 최적화 실행

```matlab
% MATLAB 명령창에서
cd Scripts
GA_Optimize_Controller  % 평균화 모델 사용 시 훨씬 빠름
```

---

## 9. 참고 문헌

1. S. Inoue and H. Akagi, "A Bidirectional DC-DC Converter for an Energy Storage System With Galvanic Isolation," IEEE Trans. Power Electron., 2007
2. B. Zhao et al., "Overview of Dual-Active-Bridge Isolated Bidirectional DC-DC Converter for High-Frequency-Link Power-Conversion System," IEEE Trans. Power Electron., 2014
3. R. De Doncker et al., "A Three-Phase Soft-Switched High-Power-Density DC/DC Converter for High-Power Applications," IEEE Trans. Ind. Appl., 1991

---

_Last updated: 2026-01-21_
