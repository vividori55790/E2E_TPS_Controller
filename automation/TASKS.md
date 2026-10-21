# 작업 큐 (상태: [ ] 미착수, [~] 진행 중, [x] 완료, [!] 막힘)

마감 2026-10-28. 필수 = T1–T9. 선택 = T10+.

## 완료
- [x] T0a 해석 모델 `Simulink/Thesis/tps_model.py` (정확한 구간 선형 정상상태; 자체검증 `test_tps_model.py` 전부 PASS:
  무차별 샘플링·교과서 SPS·대칭성).
- [x] T0b 최적 곡면 v1 `Simulink/Thesis/optimal_surface.py --tag v1` → `out/surface_v1.npz`, 로그 `out/optimal_surface_v1.log`.
  격자 |P| 0–1200 W(25 W) × V2 84–109.2 V(1.2 V), D 0.1–0.5(0.005), PI 첫 교차점.
  점별 최적: SPS 대비 평균 −0.099 A, 최대 −1.578 A, 열화 0점. 매끄러운 곡면(ICM, ε=max(0.01 A, 0.5 %)):
  최적 대비 손실 평균 0.0018 / 최대 0.0155 A, 그러나 **인접 격자 최대 변화 0.245가 남음** → T1.
- [x] T0c 루프 러너 `automation/` (loop_runner.py, job.py, start_loop.ps1).
- [x] T1 곡면 v2 (`smooth_surface_v2.py`, 1차 Dirichlet 벌점 + 연성 gap + SPS 하드 제약, 3×3 색칠 ICM, λ 스윕).
  선택 λ=5e-3: 최적 대비 손실 평균 1.37 mA(v1 1.77), 인접 최대 변화 0.09(v1 0.245), max|∂D/∂u| 1.9(v1 11.8),
  격자점 열화 0, 조밀(10 W×0.4 V, 쌍선형) 5 mA 초과 열화 0점(v1 4점). 거칠기(>0.04)는 전부 |P|≤175 W,
  큰 것(>0.1)은 V2 92.4–97.2 V, |P|≤75 W (k≈1 특이 모서리). 전 λ 곡면 `out/surface_v2_sweep.npz`(T3에서 학습 용이성으로 목표 선택 가능), 2차(thin-plate)
  비교 `surface_v2o2_*`(이득 없음, 큰 λ에서 ICM 정체). 그림 `out/fig_surface_v1_vs_v2.png`, `_v2_pareto.png`, `_v2_dense_dI.png`.
- [x] T2 `test_controllers.py` 19항목 ALL PASS(`out/test_controllers.log`). 수정: SmoothRegHDCQ의 cos 표를
  최근접→선형보간(CMSIS 방식)으로(최근접은 α 작을 때 ΔD 0.033 오차).

## 필수
- [ ] T3 `train_e2e.py`: 제안 SmoothRegHDC
  (a) 곡면 지도학습(ridge; D, σ, α 탐색 — 적합도는 D RMSE가 아니라 **무작위 점의 Ipk 손실**로 선택),
  (b) 물리 내장 E2E 미세조정: 균일 무작위 (|P|, V2) 배치에서 PI 정상상태 Ipk/Ipk_SPS를 직접 최소화
  + SPS 열화 벌점 + 불가능(전력 미달) 벌점 + 야코비안 매끄러움 벌점. 기울기: (D1, D2)에 대한 유한차분(φ 재해석 포함)
  × 읽기층 M에 대한 연쇄법칙, Adam. (b) 전후를 모두 저장.
  모델 저장 `out/model_smooth_reghdc*.json`.
  메모(T1/T2에서): 목표 곡면은 `surface_v2_sweep.npz`의 λ들 중 선택(기본 v2=5e-3). ridge α가 너무 작으면(1e-6)
  Σ|M|≈500 → 양자화 민감; α≥1e-4 권장(적합 RMSE 0.0076). D 128, σ 0.25에서 격자 RMSE(D) 0.006.
  평가는 격자점이 아니라 무작위/조밀 점에서(격자 사이 쌍선형도 SPS보다 나빠질 수 있음: v1 −7.8 mA).
- [ ] T4 기준 모델: (i) SPS, (ii) ICROS2026 학습 목표 곡면 재생(`Simulink/RegHDC_PostProcessed_Dataset.csv`, 부호 있는 P,
  V2 80–110 → GridLUT; 음/양 전력 각각 평가), (iii) LegacyRegHDC(D=32, K=8, Q=32, γ=8; 논문 설정)를 새 곡면에 학습,
  (iv) 같은 메모리의 LUT(쌍선형), (v) 제안 모델 (a), (b), (b)의 int16 버전.
- [ ] T5 `evaluate.py`: 공통 평가. 조밀 격자(|P| 10 W, V2 0.4 V) + 무작위 1만 점(논문의 random point evaluation 대응).
  지표: SPS 대비 ΔIpk(전체 평균, TPS 이득 구간 평균, 최대), 열화 점 비율(ΔI < −0.005 A), 점별 최적 대비 손실,
  불가능 점 수, 매끄러움(인접 최대 변화, 총변동, |∂D/∂P| 최대), 메모리, 연산량, Irms 변화(부가).
  결과 표 `out/eval_table.csv`, 그림: D1/D2 곡면 비교(기존 vs 제안), ΔIpk 지도, 열화 지도. 논문 Fig. 2/3 스타일.
- [!] T6 Simulink 정상상태 검증(0단계, 판정용). **막힘(2026-10-21 17:10): 데스크톱 로그온 사용자 없음**
  (explorer.exe 없음) → --interactive 작업이 실행되지 않음(결과 0x41303 "아직 실행 안 됨"). 스모크 잡
  `matlab_smoke`는 로그온 트리거로 등록돼 있어 사용자가 로그온하면 자동 실행됨 → `job.py status/diag matlab_smoke`로 확인.
  그동안 verify_steady_state.m은 미리 작성해 둘 수 있음(실행은 로그온 후). 먼저 MATLAB 스모크 잡(--interactive). 동작하면
  `Simulink/Thesis/verify_steady_state.m` 작성: `Modules/Control_Logic.slx`를 `Scripts/Generate_TPS_Dataset.m`과 같은 변수로
  수동 위상 모드(Mode_Select=1, Ctrl_Mode_Ext=3)에서 **상수 D0**로 ≥ 25 ms 실행, 마지막 1 ms에서 Ipk, i_L 평균(DC),
  P_LV, P_HV, V_LV, V_HV 측정. 점: SPS(V2 84/95/109.2 × D0 ±8, ±20, ±35) + TPS 대표점(제안 곡면의 몇 점, 기존 데이터에서
  k≈1 이득이 컸던 점 포함). 해석 모델(측정된 V_LV, V_HV 사용)과 비교 → 표·그림. parsim 사용 가능하면 병렬.
  판정: (a) k≈1 TPS 이득 실재 여부, (b) 해석–Simulink 잔차 크기·구조.
- [ ] T7 (T6 결과에 따라) 잔차가 의미 있으면 회색상자: 해석 모델 + 잔차 보정(소형 회귀)으로 T1–T5 재실행. 무시할 만하면
  그 근거를 RESULTS.md에 기록.
- [ ] T8 최종 폐루프 검증: 제안 제어기를 Simulink에 넣은 버전(복사본 모델 또는 MATLAB Function, D0는 기존 PI)으로
  대표 운전점 정상상태 Ipk를 SPS와 비교, 전력 계단 응답 1–2개. MATLAB 추론 함수 `Simulink/Thesis/smooth_reghdc_infer.m`.
- [ ] T9 MCU 배포물: `Simulink/Thesis/export_mcu.py` → `out/SmoothRegHDC_STM32.h` + `smooth_reghdc.c`(int16/float 둘 다,
  cos는 테이블 또는 CORDIC 주석), PC에서 C 참조 구현과 Python 출력 일치 테스트(gcc 있으면). 메모리/연산량 표.
- [ ] T10 `Simulink/Thesis/RESULTS.md`(한국어): 방법, 표, 그림, 한계(이상 모델 가정, k≈1), 논문 장 구성 제안,
  ICROS2026 대비 개선 요약. 3.392 A(논문 최대 저감) 재검증 결과 포함.

## 선택
- [ ] T11 ZVS 영역 분석(스위칭 순간 전류 부호) — 부가 지표.
- [ ] T12 V2 범위 확장 민감도(72–120 V) — 이득이 커지는지.
