# 최종 보고서 근거표

`FINAL_REPORT.md`의 수치별 출처다(5쪽 밖). 경로는 `Simulink/Thesis/` 기준이다. 판정 규칙은 각 스크립트의 머리말에 실행 전에 등록했다. 표시: [S] Simulink 실행 값, [참고] 해석 모델·Python 플랜트, [미완료] 재지 않음. 공통 조건(따로 적지 않으면): 데이터셋 시기 모델 사본 `dataset_era/Control_Logic_dataset_era.slx`, HV→LV, V1 380 V, 25 ns, 상수 D0 두 계단 전력 보간, 20 ms, 마지막 1 ms. 최대 = 80주기 피크의 최댓값, 중앙값 = 80주기 피크의 중앙값.

## 1. 결론과 3. 얻는 것: 정상상태 피크

| 수치 | 조건(공통과 다른 것) | 출처 | 표시 |
|---|---|---|---|
| 84, 109.2 V 100 W 1.747 → 1.252 A, 2.269 → 1.410 A | 2.88° 계단 | `out/r13_points_med.csv` 열 `max_SPS`, `max_MB` | [S] |
| 표 1의 100, 300, 1000 W | 2.88° 계단 | 같은 파일 | [S] |
| 표 1의 25, 1200 W, 1.2 kW 일곱 V2 평균 −0.9 %(0.045 A), 25 W 평균 | 0.72° 계단 | `out/edge25_check.csv`(`set` ≠ C), `out/edge25_verdict.md` | [S] |
| 그림 1 | 위 두 파일. `final_fig.py` → `out/fig_final_map.png`, `out/fig_final.log` | 같음 | [S] |
| k ≈ 1(92.4, 96.6 V) 평균 −0.008, +0.002 A | 2.88° 계단, 판정 가능 점 | `out/r13_weighted.log` | [S] |
| 기본 52점 평균 +0.115 A(+5.4 %), 중앙값 기준 +0.127 A(+5.9 %) | 판정 가능 52점, 점마다 같은 가중 | `out/r13_verdict.md`, `out/r13_weighted.log` | [S] |
| 전력 균등 가중 +0.063 A(+2.5 %), +0.076 A(+3.0 %) | V2마다 100–1000 W 사다리꼴(판정 아닌 계산) | `out/r13_weighted.log` | [S] |
| 다른 V2·전력 44점 +0.069 A(+3.1 %), +0.072 A(+3.2 %) | 48점 중 깨끗한 44점, 학습에 쓰지 않음 | `out/md_check.log`, `out/md_check.csv` | [S] |
| 100 ns 100 W 저감 23.9 %, 23.2 % | 원본 100 ns, 30 ms | `out/simulink_pi_compare.csv` | [S] |
| 109.2 V 100 W 1.476 A(−35 %), 중앙값 1.371 A(−40 %) | 0.72° 계단 | `out/edge25_check.csv`(`set` = C) | [S] |
| 대표 6점 +0.328 → +0.334 A(중앙값 +0.351 → +0.351 A) | 84, 109.2 V × 100, 300, 1000 W, 12.5 ns | `out/g12_verdict.md`, `out/g12_check.log` | [S] |
| 최저 대비 저감 91 %, 83 % | 잰 후보(곡면 둘레 + 먼 듀티 155쌍) 중 최저 대비, 52점 평균 +0.127 / +0.140 A(중앙값), +0.115 / +0.139 A(최대) | `opt25.py`, `out/opt25_check.log`, `out/opt25_verdict.md`, REPORT 11.5 | [S] |
| 재학습 곡면 이득 −0.004 A, +0.004 A | 앵커 밖 48점 중 47점, 문턱 +0.01 A | `out/md_verdict.md` | [S] |

## 2. 무엇이 바뀌었나, 5. MCU 자원

| 수치 | 조건 | 출처 | 표시 |
|---|---|---|---|
| 기존 데이터셋 목표 제어기 16점(15점), 96.6 V 150 W 0.665 → 1.507 A, 평균 차 0.134 A(0.139 A) | 기본 56점, 판정 가능 53점 | `out/tgt25_check.csv`, `out/tgt25_verdict.md` | [S] |
| ICROS 재구현 대비 평균 0.415 A(0.422 A) | 재구현, 분류기는 데이터셋 라벨 | 같음 | [S] |
| 미세조정 몫 −0.018 A, −0.012 A | 실행이 다른 16점, 105 V 400 W 한 점이 대부분 | 같음 | [S] |
| LUT 49×22, D = 64, LUT 25×11, LUT 13×6 | 상수 위상 25 ns, 실행이 다른 점 | `out/tgt25_verdict.md` | [S] |
| 4,618 B, 5,128 B, 256차원 × 곱셈 4회, 코사인 표 1025개 | 내보낸 C 코드 | `out/export_mcu.log`, `out/SmoothRegHDC_STM32.h` | 코드 |
| 20,253점 비트 단위 일치 | LCC-win64, x86-64 PC. ARM이 아님 | `out/c_compile_check.log` | 코드 |

## 3. 전류 튐, 스위칭

| 수치 | 조건 | 출처 | 표시 |
|---|---|---|---|
| 튐 3.18, 1.03 A 대 0.24, 0.18 A, SPS 0.23, 0.18 A | 폐루프 램프 100 → 900 W, 40 ms, 제어기 사본, 25 ns, 위상 래치, Kp 1, Ki 24500, 듀티는 외부 입력 | `out/r12_verdict_r2.md`, `out/r12_check_r2.csv` | [S] |
| 처음 이득에서 1.39 A | Kp 2, Ki 49000 | `out/r12_verdict.md` | [S] |
| 계단 4.89 → 2.83 A, 3.58 → 2.42 A, 7/8, +0.18 A(+1.3 %), 정착 3.44 대 1.35 ms, 13.09·13.31·13.49 A | 폐루프, 이득 ½, 계단 직후 2 ms 최대 | `out/r11_check.log`, `out/r11_check.csv` | [S] |
| V·I 22,619 → 4,599 VA(−80 %) | 기본 52점 합, 오프셋 이동 점검 −76 … −83 %, Coss 0 모델 | `zvs_check.py map25`, `out/zvs_map25.log`, `out/zvs_map25.csv` | [S] |

## 3. 효율

공통: `f1_eff.py`(규칙 10-31 14:41, 실행 전), 120회, `job_errors.py f1eff` 오류 0, 상수 D0 20 ms, 마지막 1 ms, V1 380 V. 손실 L = P_LV − P_HV − 버스 저장 에너지 변화, V·I 국소 스텝 직접 평균. 제안 제어기는 듀티를 25 ns 칸으로 올린 B*.

| 수치 | 조건 | 출처 | 표시 |
|---|---|---|---|
| 판정: 절대 손실 미수렴, 효율 이득 미확정, 25 ns 값 수렴값 아님, 분석 블록 아님, 솔버 스텝, 수지 안 맞음, 효율 블록 쓸 수 없음, 비례 | 등록 규칙 Q1–Q7 | `out/f1eff_verdict.md`, `out/f1eff_check.log` | [S] |
| 표 2의 손실과 효율 차 | \|P\|에서 두 계단 보간, 2.88° | `out/f1eff_eta.csv` 열 `L_SPS_*`, `L_B_*`, `de_*`, `L0_*`, `de_0` | [S], 외삽은 측정 아님 |
| 12.5 → 6.25 ns 중앙값 13 %, 최대 34 % | 같은 입력 24행, L(12.5) > 0.5 W | `out/f1eff_check.csv` 열 `L12.5`, `L6.25`, 로그 Q1 | [S] |
| 0.18 pp | LV→HV 109.2 V 100 W, 문턱 0.10 pp | `out/f1eff_eta.csv` 열 `de_12.5`, `de_6.25` | [S] |
| 0.008 W, 8회 | 제어기 25 ns, 솔버 6.25 ns 대 6.25 ns | `out/f1eff_check.log` Q4 줄 | [S] |
| 차이 0.002 W | \|L_a − L\| 중앙값, 고운 세 격자 | `out/f1eff_check.log` Q3 줄 | [S] |
| 기본 52점 평균 +0.33 pp | 25 ns, 수렴값 아님 | `out/r13_verdict.md`, REPORT 11.5 | [S] |
| 131–181 W, 최대 0.2 pp | 6.25 ns 두 계단의 폭, 손실 ∝ P²이면 생기는 직선 보간의 어림, 판정 아님 | `f1eff_side.py` → `out/f1eff_side.log` "interpolation width" | 어림 |
| 23회 중 16회 | 6.25 ns, L > 0.5 W, 34 × 52 mΩ × i² + 다이오드, 벗어난 7회는 제안 제어기 100 W, 1.13–1.59배 | `out/f1eff_check.log` Q5 줄 | [S] |
| 효율 블록 72회 중 71회, +3.25, +0.19 pp | 실제 흐름 방향으로 계산, 판정 아님. 규칙 답은 HV→LV 33/36, LV→HV 32/36, 효율로 못 씀 20/36, 21/36 | `f1eff_side.py` → `out/f1eff_side.log`, `out/f1eff_sweep_r*.csv` 열 `Eff_out`, `Loss_out`, `dataset_era/Direct_Efficiency_Analyzer.m` 128–134, 192–195행 | 코드, [S] |

## 4. 나빠지는 조건

| 수치 | 조건 | 출처 | 표시 |
|---|---|---|---|
| \|1 − k\| ≤ 0.04 33점 −0.004, −0.006 A, 낮음 0, 높음 3 | 기본 + 다른 V2 점(판정 아닌 정리) | `out/md_k.log` | [S] |
| 95 V 100 W 0.297 대 0.360 A | 12.5 ns에서 0.295 대 0.359 A | `out/g12_check.log` | [S] |
| V1 ±5 % 12/18, 0.371 → 0.641 A, 중앙값 +0.247 A, 정규화 12 → 0, 판정 가능 27/30 | 100–300 W, V2는 k가 1을 지나는 값 | `out/v1x_verdict.md`, `out/v1x_check.csv` | [S] |
| ±1 %, ±2 % 0.476 → 0.527 A, 0.352 → 0.534 A | 100, 200 W, V2 여섯 값, 12경우씩 | `out/v1e_verdict.md`, `out/v1e_check.csv` | [S] |
| LV→HV 28점 +0.115 A(+0.123 A), HV→LV +0.116 A(+0.125 A) | 7 V2 × 100, 300, 600, 1000 W | `out/rev25_verdict.md`, `out/rev25_side.log`, `out/rev25_check.csv` | [S] |
| 100.8 V 600 W +0.069 A(+0.030 A) → −0.004 A(−0.012 A); 둘레 다섯 점 0.003–0.054 A | LV→HV, 6.25 ns, 0.72° 계단 | `out/rev6_verdict.md`, `out/rev6_check.csv` | [S] |
| SPS 방향 차 최대 0.489 A | LV→HV − HV→LV, 109.2 V 1 kW | `out/rev25_side.log` | [S] |
| HV→LV 위반 네 점 → 6.25 ns 0.000 … +0.098 A, 다섯 점, 나머지 91점, 판정 불가 8점 | 6.25 ns, 0.72° 계단 | `out/h12_verdict.md`, `out/h12_check.csv`, `out/r13_verdict.md`, `out/md_check.csv` | [S] |
| 96.6 V 25, 75 W +0.053, +0.142 A(+0.025, +0.055 A) | 0.72° 계단, 실행 비주기 | `out/edge25_check.csv` | [S] |
| 98.7 V 200 W +0.032 A(−0.014 A) | 0.72° 계단, 기준선 | `out/v1e_check.csv` | [S] |
| V2 엣지 +0.80 A(300 W), +0.29 A(1 kW) | 84 → 95 V, 0.5 ms, 폐루프, 이득 ½ | `out/r11_v2.csv` | [S] |

## 5. 적용 방법, 6. 신뢰도

| 수치 | 조건 | 출처 | 표시 |
|---|---|---|---|
| 회귀를 제어기 안에서: 규칙 "같음", 최대 피크 차 +0.036, +0.102 A(듀티 차 0.0027, 0.0028은 본문 밖) | 폐루프 램프, 제어기 사본, 25 ns, 이득 ½, 입력은 측정 전압 이동평균, 90, 109.2 V × 1, 8주기, 문턱 0.106 A | `out/f1v3_verdict.md`, `out/f1v3_check.csv` | [S] |
| 기동 0–6 ms 피크 21.9, 19.9 A → 27.1, 26.3 A | 같은 실행, 규칙 밖, 첫 듀티 (0.4473, 0.5000) | `out/r12/r2_MB_*_ramp_periods.csv`, `out/f1v3/v3_MB_*_h1_periods.csv`, 열 `Ipk`, `D1`, `D2` | [S] |
| 84 V 100 W 1.73 A → 3.1 A | 원본 100 ns, Kp 2, Ki 49000, 35 ms | `out/simulink_pi_pilot*.csv`, REPORT 4절 | [S] |
| −0.079 … +0.054 A(36구간), 위상 p2p 0.61° | 폐루프 이득 ½ − 상수 D0 25 ns, 주기 피크 중앙값 | `out/r11_check.log` | [S] |
| 약 4 kHz 한계 주기 | Kp 2, Ki 49000 | `out/r11_check.log` | [S] |
| 부호 전환 "문제 없음", 95 V 약 1 ms 정지 | ±300 W, 50 W/ms, 폐루프, 이득 ½ | `out/flip25_verdict.md`, `out/flip25_check.csv` | [S] |
| L ±10 % 72–136 %, 데드타임 200 ns 96–102 % | 84, 109.2 V × 100, 300, 1000 W | `out/rob25_verdict.md`, `out/rob25_check.csv` | [S] |
| 중간 V2 54경우 위반 없음 | Python 플랜트 25 ns | `out/rob_interior_plant.log` | [참고] |
| 238회 약 3시간 | 잡 `r13a` 시작·끝, 6 병렬, SPS·제안·보정 곡면 | `automation/state/jobs/r13a.json` | 기록 |
| 미세조정 약 18분 | 잡 `e2e_b3`, 2000 스텝 | `automation/state/jobs/e2e_b3.log` | 기록 |
| 709회 중 76회, 72회 | `r13b`까지 | `out/r13_map.log`, `out/zvs_map25.log` | [S] |
| 27구성 중앙값 0.003 A, 최대 0.024 A | 25, 12.5 ns 모두 주기적 | `out/g12_check.log`, `out/g12_verdict.md` | [S] |
| 10/12, 9/12, TT 0 비트 단위 같음 | 10 ns 진단 | `out/g12_verdict.md`, `out/g12_check.log` | [S] |
| 195행 비트 단위, 1.35 = 1.15 + 0.21 − 0.01 A | 데이터셋 생성 실험 재실행 | `out/staircase_compare.log`, `out/ipk_decompose.log` | [S] |
| 3.392 A "도달 불가", SPS 2.505, 2.633 A, 최대 저감 0.759, 0.632 A(0.785, 0.642 A), 21쌍 | V2 110 V, 150, 200 W, 2.88° 계단, 깨끗하지 않은 구성은 200 W 제안 제어기 하나, 67회 오류 0, 운전점은 추정 | `f1_icros.py`, `out/f1icros_verdict.md`, `out/f1icros_check.csv`, `out/f1icros_check.log`, REPORT 6.7 | [S] |
| 가장 큰 저감 1.376 A | 109.2 V 25 W, 기본 56점 + 25, 50, 75, 1200 W 안에서 | `out/edge25_check.csv`(`set` = E), `out/r13_points_med.csv` | [S] |
| 실행 총수, 오류 141회의 내역과 재실행 | 러너 기록 줄과 폐루프 요약 파일의 `error` 칸, 고정 듀티 계단은 잡 로그의 조합 수. 스모크, 195행 재실행, 첫 시뮬레이션 전에 멈춘 잡은 세지 않음 | `final_counts.py`, `out/final_counts.log`, `automation/job_errors.py` | 기록 |
| 대표 행 재현 | 84, 109.2 V × 100, 300, 1000 W, SPS와 제안 제어기의 두 계단, 새 MATLAB 세션, 같은 러너 | `repro6.py`, `out/repro6_check.log` | [S] |
