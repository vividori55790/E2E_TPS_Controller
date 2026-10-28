# 작업 큐 (상태: [ ] 미착수, [~] 진행 중, [x] 완료, [!] 막힘)

## ★★ 2026-10-27 추가 (최우선) — 사용자: "전류 피크 저감이 그 정도일 리가 없다", 자기 데이터셋으로 확인하라
배경(대화형 세션 10-27 확인, PROGRESS 10-27 항목): `Simulink/Data/TPS_Checkpoint_done_20260316_021229.mat` = CSV와 행 단위로 동일
(`inspect_checkpoint.m` → `out/checkpoint_20260316_021229.csv`). 데이터셋은 같은 전력에서 TPS가 SPS보다 0.5–3.4 A 낮다고 말하지만,
정상상태(R.4) 대비 분해하면 그 차이의 대부분(평균 1.15 / 1.21 A)이 **데이터셋 SPS 행의 과대값**이다(TPS 행은 정상상태와 거의 같음).
데이터셋 생성 모델(35d5a78, `dataset_era/`)의 D0 From Workspace 블록(`From Workspace11`)은 **Interpolate data = on(기본값)** —
스크립트가 timeseries에 ZOH를 지정해도 블록이 선형보간한다. 사용자는 "고정 외부 위상(수동 위상)으로, 정상상태까지 돌렸다"고 함.
- **진행 기록(10-27 19:30, 반복 6)**: (a) **완료** — interp=1 재실행 195행이 체크포인트와 **비트 단위로 같음**(max|ΔIpk| = |ΔP| = 0,
  5조합 모두) → 데이터셋 = 이 모델 + 이 스크립트 + D0 블록 선형보간 확정. interp=0은 첫 시도에서 Simulink 설정 오류(보간 끔이면
  OutputAfterFinalValue='Holding final value' 필요)로 실패 → `staircase_check.m` 수정(3번째 인수 interps). 재실행 `stair_i0`와
  R6 스모크 `R6_smoke`가 **CPU 0 %로 멈춤**: 19:11:20에 MathWorksServiceHost가 재시작(CrashReporter 동반)된 뒤 MATLAB마다
  MathWorksServiceHostWindow 프로세스가 떠 있음 → 온라인 라이선싱 로그인 창 대기로 추정(사용자 데스크톱에서 MathWorks 로그인 필요;
  Session 0에서는 창을 볼 수 없음). 대응: R5(b)/(c)·R6을 한 세션에서 순서대로 하는 `r5r6_chain.m`(각 단계 출력이 있으면 건너뜀)을
  잡 `r5r6`으로 제출(로그인되면 자동 진행). 그동안 Python 플랜트 보조 재생 `staircase_replay.py`(linear/zoh/정상상태). 분석
  `staircase_compare.py`(a/b/c 표 + `fig_staircase_compare.png`), R7 분석 `dataset_fixed_analyze.py`(원 데이터셋에서 시험 완료).
  멈춘 `stair_i0`/`R6_smoke`는 kill 권한이 없어 그대로 둔다(로그인 후 풀리면 같은 결과를 중복 계산할 뿐, 출력 파일 충돌 없음).
- [~] R5 **판정 실험 `staircase_check.m`**(잡 `staircase_check`, 10-27 19:01 시작, 대화형 MATLAB): 사용자 생성기와 똑같은 44 ms·39계단·
  0.1 ms 창 실험을 5조합(`out/staircase_combos.csv`: 데이터셋에서 저감이 가장 컸던 TPS 3 + 같은 V2 SPS 2) × {Interpolate 켬(원래),
  끔}으로 재실행 → `out/staircase_check.csv`(tag, interp, V2, D1, D2, step, D0_label, Ipk, P_LV, P_HV). 할 일: `job.py status`로 끝났는지
  보고, 분석 스크립트 `staircase_compare.py`를 써서 (a) interp=1 행이 체크포인트 CSV의 같은 (V2, D1, D2, D0 라벨) 행을 재현하는지
  (Ipk, P 오차), (b) interp=0 행이 정상상태(t6 `out/simulink_steady.csv`, R.4 `out/simulink_pi_compare.csv`, 해석/플랜트)와 맞는지,
  (c) **같은 측정 전력에서 TPS 조합 vs SPS의 Ipk 차이**가 interp=1과 interp=0에서 각각 얼마인지(사용자의 질문에 직접 답하는 표).
  잡이 실패하면 log로 원인 파악 후 고쳐 재제출. 결과를 RESULTS.md 새 절 "R.8 데이터셋 생성 방식 재현 실험"에 표로.
- [~] R6 **(10-27 19:30: `gen_dataset_fixed.m` 작성, `r5r6` 체인의 3단계로 대기 중 — MATLAB 로그인 필요. 격자는 명세의
  linspace(0.2, 0.5, 11) 대신 데이터셋 격자의 부분집합 Dg([1:2:19, 20])로 했다(원 데이터셋과 행 단위 비교 가능). V2 = V2_vec(9) =
  94.61 V(95 V에 가장 가까운 격자값; 명세의 96.63은 격자값이 아님). 실행 순서는 SPS → 6×6 → 나머지라 부분 결과로도 R7이 가능.
  출력 `out/dataset_fixed_part_NNN.csv`. 스모크(R6_smoke)는 로그인 대기로 미실행 → 체인 3단계가 첫 실행이다. 오류가 나면 로그를
  보고 고쳐 `r5r6`를 다시 제출하면 된다(1·2단계는 출력이 있으면 건너뜀).)** (원 명세) (R5에서 interp=0이 정상상태와 맞고 데이터셋 SPS
  과대가 interp=1의 산물로 확인되면) **사용자 방법 그대로의 수정 데이터셋**:
  `Simulink/Thesis/gen_dataset_fixed.m` = `Scripts/Generate_TPS_Dataset.m`의 실험을 그대로 따르되 D0 블록 Interpolate 'off'
  (`setBlockParameter`, 모델 저장 금지, dataset_era 모델). 마감(10-28) 안에 끝나도록 격자 축소: V2 {84, 95, 109.2}(데이터셋 격자와
  같은 값으로는 84/109.2 + 가장 가까운 96.6316) × D1, D2 ∈ linspace(0.2, 0.5, 11) = 363 시뮬, parsim 6워커(10-26 측정 ≈ 30 ms당
  워커 1회 ~180 s → 44 ms ≈ 4–5 h). **재시작 안전하게**: 배치(12개)마다 `out/dataset_fixed_part_XXX.csv`를 쓰고 이미 있는 조합은
  건너뛴다(재부팅·잡 종료 후 재제출 시 이어서). 잡은 `--interactive`(로그온 필요; 로그온이 없으면 대기 — PROGRESS에 기록).
  먼저 2조합 스모크로 시간·신호를 확인한 뒤 전체 제출.
- [~] R7 **(10-28 12:45, 반복 10: MATLAB 17시간째 정지. (1) `r5r6_chain.m`이 먼저 stair_i0/R6_smoke의
  `automation/state/jobs/*.done`을 최대 40분 기다린다(세 MATLAB이 같은 로그인에서 함께 풀리면 풀 2개가 6코어·31 GB를 나눠 쓰고
  1단계가 중복되므로; r5r6 MATLAB은 시작 단계에서 멈춰 아직 스크립트를 읽지 않았다). (2) **판정 규칙 사전 고정** `r8_verdict.py`
  (+ `test_r8_verdict.py` 5경우 ALL PASS) → r8_finalize 5단계가 `out/r8_verdict.md`(Q1 진짜 계단 저감 비 r ≥ 0.75 사용자 쪽 /
  ≤ 0.5 보간 산물, Q2 D − B_st, Q3 K1_ 정상상태 이득 0.05/0.15 A, Q4 수정 데이터셋 r과 M-B 차 0.1 A, 0절용 문장)를 쓴다.
  로그인 후 할 일: r8_finalize 한 번 → r8_verdict.md의 판정·문장을 RESULTS 0절·R.8에 옮긴다(규칙이 결론 수정을 말하면 고친다).)**
  **(10-28 08:30, 반복 9: MATLAB 13시간째 정지(세 MATLAB dcpu 0, 산출물 변화 없음). 마감 정리: `ipk_decompose.py`에 그림
  `out/fig_ipk_decompose.png`((a) A/B/C/D 점, (b) A−B 분해 막대; r8_finalize가 다시 그림) 추가, RESULTS 0절에 "마감 상태" 항목
  (확정 = R.1–R.7, R.8 a/c/d; 예비 = R.8 ii, R6/R7). 로그인 후 할 일은 아래 반복 8과 같다(r8_finalize.py 한 번).)**
  **(10-28 04:40, 반복 8: MATLAB 여전히 정지(9시간째, 세 MATLAB dcpu 0). Simulink 수치만으로 분해 `ipk_decompose.py` →
  RESULTS R.8 (d): k≠1 7점 데이터셋 저감 1.35 A = SPS 과대 1.15 A(85 %) + M-B 정상상태 저감 0.21 + (M-B − 데이터셋 최저 TPS) −0.01 A.
  D − B > 0.05 A인 109.2 V 100 W(+0.29)·84 V 300 W(+0.10)·84 V 100 W의 데이터셋 최저 조합 3개 정상상태 23점(BT_)을
  `stair_steady_points.csv`에 추가(`add_best_tps_steady_points.py`, 106점; Python 플랜트로 감쌈 확인). **out/에 stair_steady_part*.csv를
  손으로 만들지 말 것**(r5r6가 2단계를 건너뜀). 로그인 후 할 일 = `python -I -X utf8 Simulink/Thesis/r8_finalize.py` 한 번
  (R5 b/c, 분해의 BT_ 정상상태, k1, R6/R7을 입력 있는 것만) → `out/r8_finalize.log`로 R.8·0절 수치 교체.)**
  **(10-28 00:10, 반복 7: MATLAB 여전히 로그인 대기(세 MATLAB CPU 0 %, `matlab_stall_probe.py`). Python 점검 완료 →
  RESULTS R.8 "R7 예비의 점검": (1) k≈1 100 W +0.46 A = 계단 이력 산물(D0 0° 공존 상태 0.75 vs 0.44 A, 2.40° 느린 과도;
  `k1_history_probe.py`), 정상상태 이득 ≤ 0.09 A, td 0이면 SPS 최선(`k1_light_probe.py`). (2) 같은 플랜트·같은 정상상태에서
  M-B = 격자 최저 TPS 저감의 94 %(격자 없음 18점 +0.231 vs +0.246 A, 최대 차 0.088 A @109.2 V 100 W; 2.88° 격자 플랜트 0.040/0.167 A;
  `mb_vs_grid_probe.py`). `stair_steady_points.csv`에 94.61 V K1_ 18점 추가(`add_k1_steady_points.py`) → r5r6 2단계가 함께 측정.
  Simulink 결과가 생기면 `k1_simulink_compare.py`(준비·시험 완료)도 돌려 R.8 (1)을 Simulink로 확인한다.)**
  **(10-27 19:35: `dataset_fixed_analyze.py` 완료·시험. Python 플랜트 예비판(`replay_grid.py`, zoh/linear)을 RESULTS R.8에
  넣었다. R6 Simulink 파트가 생기면 `python -I -X utf8 Simulink/Thesis/dataset_fixed_analyze.py`(기본 prefix out/dataset_fixed) →
  `out/dataset_fixed_compare.{log,csv}`, `fig_dataset_fixed.png`. R.8의 예비 표를 Simulink 수치로 바꾸고 0절 결론을 갱신한다.
  특히 경부하 격자 최저 TPS > M-B인지, k ≈ 1 100 W의 +0.46 A(Python)가 Simulink에서도 나오는지 판정한다.)** (원 명세) 수정 데이터셋
  분석(사용자 파이프라인의 "가상물리모델 → 최적화"를 ML 없이 직접): 각 조합의 (D0 라벨→측정 P, Ipk)를 첫 분기에서
  보간해 (V2, |P|)별 최소 Ipk TPS vs SPS를 구한다(원 데이터셋에도 같은 방법을 적용한 표와 나란히). 결과를 M-B(R.4 Simulink PI 정상상태)와
  비교. RESULTS.md 0절 요약과 R.8 갱신, 사용자 질문("저감이 그 정도일 리 없다")에 대한 답을 수치로 한 단락. 원 데이터셋에서 쓰던
  대리모델(MLP/XGBoost) 재학습은 하지 않는다(범위 밖, 시간 부족).
- 주의: 루프 시작 시 `staircase_check`가 아직 돌고 있으면 그 사이 R6 스크립트·스모크를 준비하고 `wait_for_jobs`로 턴을 끝낸다.
  MATLAB 잡을 kill해도 MATLAB.exe가 남을 수 있다(메모리 참고) — 중복 실행 전에 확인.

## ★ 범위 변경 (사용자 결정 2026-10-24 ~ 10-26) — 이 절이 아래 모든 항목보다 우선한다
사용자 지시: "기존 SPS/TPS 분류기만 없애서 내부 위상을 완전히 회귀 모델의 E2E로 출력하는 것까지만 진행"
+ "외부 위상(D0)은 연산 병목을 없애려고 PI 제어기로 설계한다고 가정하는 프로젝트" + 학습 목표는 **"둘 다 비교"**.
마감 2026-10-28. 이 범위 밖의 새 작업은 하지 않는다(이미 끝난 결과는 RESULTS.md 부록으로만 유지).
- [x] R1 (2026-10-26 14:00) `train_targets.py` → `out/model_smooth_reghdc_MA_{neg,pos}.json`(M-A, a와 같은 부호화·α, 방향별:
  데이터셋 ±P 라벨이 최대 0.09/0.14 다름, 본문 = HV→LV = 데이터셋 P<0), M-B0 = (a), M-B = b3. Legacy M-A는 evaluate.build
  (`model_legacy_final.npz`). 해석 1만 점: M-A −0.0096 A / 27.3 % 열화 / 최악 −828 mA, M-B +0.0815 / 0 % / −2.8 mA.
  (원 명세) **두 학습 목표의 E2E 회귀 모델** (분류기·FSM 없음, 출력 (D1, D2), D0는 PI). **같은 모델 구조·같은 학습 절차**로 목표만 바꾼다.
  - M-A "기존 목표": 사용자의 기존 파이프라인 데이터셋 `Simulink/Physics_Dataset_Final_Regression_Ready.csv`
    (사용자의 최신 순수 회귀 trainer `Simulink/Data/RegHDCTrainer.m`이 읽는 파일, 부호 있는 P, V2 80–110 V).
    필요하면 `Simulink/RegHDC_PostProcessed_Dataset.csv`(논문 그림 2 우측)도 같은 방식으로 한 행 더.
  - M-B0 "정상상태 목표": `out/surface_v2`(정상상태 최적 매끄러운 곡면)를 같은 절차로 지도학습.
  - M-B "정상상태 + 물리 E2E 미세조정": 기존 b3(또는 같은 절차로 재현).
  - 모델 구조: SmoothRegHDC(제안)를 기본으로, 표에 LegacyRegHDC(D=32, K=8, Q=32, γ=8; 논문 설정)도 같은 두 목표로 한 줄씩.
- [x] R2 **완료(2026-10-26 17:10)**. (1) 해석 1만 점(RESULTS R.2). (2) 플랜트(수정 해법, variant B, 1200점; RESULTS R.3):
  M-A −0.0178 A / 27.7 % 열화 / 최악 −885 mA, M-B0 +0.0789 / 1.25 % / −45 mA, M-B +0.0788 / 1.0 % / −19.5 mA(20 mA 초과 0 %,
  열화는 전부 V2 92–100 V·|P| ≤ 401 W = 다중 정상상태 영역). (3) Simulink(RESULTS R.4): `simulink_sweep.py` 4차 150회 →
  `out/simulink_pi_compare.{csv,log}`, `fig_simulink_pi_compare.png`. M-A 6/12점 열화(95 V 전부, 최악 −0.49 A = −50 %),
  M-B 95 V = SPS, k≠1 경부하 −0.41…−0.53 A(23–24 %), 평균 +0.22 A(+7.1 %), 열화 1점 −11 mA(보간 불확도 안). 듀티 0.5 정확값은
  위상 카운터 경계(knife edge) 산물이라 D ≤ 0.4999999로 재실행. LV→HV는 미실행(최소 요건 한 방향 충족).
  (이전 기록 14:40) (1) 완료(`evaluate.py --b-tags 1,2,3,4,5_sps,6,7_D64 --final 3`, 그림 `fig_targets_*.png`).
  (2) 플랜트 해법 결함 수정 후 잡 `pvR2_MA_MB`, `pvR2_MB0`(variant B, 1200점) 실행 중 → `out/plant_verdict_B_R2_B_*.{log,npz}`.
  (3) **진짜 폐루프는 판정에 못 씀**: 제어기 PI 모드가 2.88° 위상 격자에서 디더해 SPS Ipk +1.2–3 A(이득 무관, RESULTS R.7) →
  `simulink_sweep.py`: 목표 전력을 감싸는 두 D0 계단의 상수 D0 정상상태(verify_steady_state.m) 보간. 1차 92회 잡 `sw1b`
  → `out/simulink_sweep_r1.csv`(취소한 sw1의 고아 MATLAB도 같은 계산 → `dataset_era/out/simulink_sweep_r1.csv`, 먼저 끝나는 쪽 사용).
  다음: `simulink_sweep.py next`(감싸지 못한 구성 2차) → `compare` → RESULTS R.3/R.4 표·그림 채우기.
  (원 명세) **평가 (같은 잣대)**: (1) 정상상태 해석 모델(무작위 1만 점 + 조밀 격자; 기존 evaluate.py 지표), (2) 비이상 플랜트
  `plant_full.py` PI 정상상태(변형 Ron40 무droop), (3) **Simulink PI 정상상태(결정적 판정)**: V2 {84, 95, 109.2} × |P|
  {100, 300, 600, 1000} W(최소 한 방향, 가능하면 양방향 일부) × 제어기 {SPS, M-A, M-B0, M-B}.
  D0는 PI로 닫는다: 가능하면 `dataset_era/`의 DAB_Controller_System.m **복사본**에 "PI + 외부 D1/D2" 모드를 넣어 진짜 폐루프로
  (프로젝트 가정과 동일), 어려우면 plant_full로 예측한 D0 주변 3점 정상상태 스윕 + 보간(= PI 정상상태)으로. 같은 측정 전력에서
  Ipk를 SPS와 비교. MATLAB은 `job.py submit NAME --interactive` (사용자가 MATLAB에 로그인해 둠, 10-24 19:12 스모크 성공,
  t6_verify 52점 1537 s 성공). 결과 `out/simulink_pi_compare.csv` + 표·그림.
- [x] R3 (2026-10-26 17:15) RESULTS 0절 요약(결론·세 잣대 수치), R.1–R.7(R.3 플랜트 표, R.4 Simulink 표·그림, R.5 Simulink 대
  Simulink `dataset_vs_simulink.py`, R.6 상한 `tps_limit.py`, R.7 PI 폐루프 파일럿), 8절 경위·(4) 무효 표시, 9절 부록 A 처리,
  10절 장 구성, 한계 갱신.
  (원 명세) RESULTS.md 마무리(한국어): 맨 앞 요약(초록용 수치), M-A vs M-B 비교표(해석/플랜트/Simulink), D1·D2 곡면 그림 비교,
  ΔIpk 지도. **물리 설명 포함**: TPS가 줄일 수 있는 것은 SPS 피크 중 전압 불일치 순환 성분 V1·|1−k|/(4·fs·L)뿐
  (이 설계 k = 0.88–1.15 → 84 V 1.53 A, 109.2 V 1.97 A, 95 V 0) → 저감은 경부하 30–40 %, 1 kW 2–3 %, k≈1 ≈ 0.
  사용자는 "400 V·1 kW인데 저감이 너무 작다"고 의심함 → 절대값보다 **상대 저감(%)과 |1−k| 의존성**으로 제시하고, 무제약 전수탐색
  (D 0.01–0.5, D0 전 분기) 결과도 같다는 것을 근거로 적는다(2026-10-26 대화형 세션 확인: 84 V 100 W −29 %, 1000 W −1.9 %;
  109.2 V 100 W −39 %, 1000 W −2.8 %; 95 V ≈ 0). 논문 장 구성 초안 갱신.
- [x] R4 (2026-10-26) 최종 모델 M-B = b3 그대로 → 재생성 불필요(`out/SmoothRegHDC_STM32.h`, `smooth_reghdc_params.m`는 b3 기준).
  C 컴파일 시험은 선택(컴파일러 없음).
  (원 명세) 최종 모델(M-B)의 MCU/MATLAB 내보내기 재생성(모델이 b3에서 바뀐 경우만). C 컴파일 시험은 선택.
- 중단(범위 밖): T7 나머지(b8–b11 선택, pv_* 후속), T8의 별도 계단 응답, T12. 이미 얻은 결과는 부록으로만.

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
- [x] T3 `train_e2e.py` 완료(2026-10-21 18:00). (a) 120 구성 → λ 2e-3, D 256, σ 0.1, α 1e-3. (b) 6변형
  (b1 β4 γ1e-3, b2 D128, b3 β20, b4 γ1e-2, b5 SPS 초기화 3000 step, b6 lr 6e-4) → b* = b3(검증 점수, 힌지 가중 20).
  test: 평균 ΔI 0.0815 A(최적 0.0819), 최악 −2.8 mA, 5 mA 초과 열화 0. b5(곡면 없이 E2E만) 0.0807 A.
  모델 `out/model_smooth_reghdc_{a,b1..b6}.json`, 추이 `out/train_e2e_b*_hist.csv`, 기울기 검증 `test_train_e2e.py` ALL PASS.
- (원래 T3 명세)
  (a) 곡면 지도학습(ridge; D, σ, α 탐색 — 적합도는 D RMSE가 아니라 **무작위 점의 Ipk 손실**로 선택),
  (b) 물리 내장 E2E 미세조정: 균일 무작위 (|P|, V2) 배치에서 PI 정상상태 Ipk/Ipk_SPS를 직접 최소화
  + SPS 열화 벌점 + 불가능(전력 미달) 벌점 + 야코비안 매끄러움 벌점. 기울기: (D1, D2)에 대한 유한차분(φ 재해석 포함)
  × 읽기층 M에 대한 연쇄법칙, Adam. (b) 전후를 모두 저장.
  모델 저장 `out/model_smooth_reghdc*.json`.
  메모(T1/T2에서): 목표 곡면은 `surface_v2_sweep.npz`의 λ들 중 선택(기본 v2=5e-3). ridge α가 너무 작으면(1e-6)
  Σ|M|≈500 → 양자화 민감; α≥1e-4 권장(적합 RMSE 0.0076). D 128, σ 0.25에서 격자 RMSE(D) 0.006.
  평가는 격자점이 아니라 무작위/조밀 점에서(격자 사이 쌍선형도 SPS보다 나빠질 수 있음: v1 −7.8 mA).
- [x] T4 기준 모델 — `evaluate.build()`에 구현(LUT16, Signed 어댑터, Legacy 캐시 `out/model_legacy_{icros,v2}.npz`).
  원 명세: (i) SPS, (ii) ICROS2026 학습 목표 곡면 재생(`Simulink/RegHDC_PostProcessed_Dataset.csv`, 부호 있는 P,
  V2 80–110 → GridLUT; 음/양 전력 각각 평가), (iii) LegacyRegHDC(D=32, K=8, Q=32, γ=8; 논문 설정)를 새 곡면에 학습,
  (iv) 같은 메모리의 LUT(쌍선형), (v) 제안 모델 (a), (b), (b)의 int16 버전.
- [x] T5 `evaluate.py` 완료(해석 모델 기준): `out/eval_table.csv`, `out/evaluate.log`, `out/eval_icros_recheck.csv`,
  `out/fig_eval_maps.png`, `out/fig_eval_slices.png`. 기준 최적 `pointwise_random.py` → `out/pointwise_{val,test,dense}.npz`.
  T6/T7에서 플랜트가 바뀌면(회색상자) 다시 돌리면 됨. 원 명세: 공통 평가. 조밀 격자(|P| 10 W, V2 0.4 V) + 무작위 1만 점(논문의 random point evaluation 대응).
  지표: SPS 대비 ΔIpk(전체 평균, TPS 이득 구간 평균, 최대), 열화 점 비율(ΔI < −0.005 A), 점별 최적 대비 손실,
  불가능 점 수, 매끄러움(인접 최대 변화, 총변동, |∂D/∂P| 최대), 메모리, 연산량, Irms 변화(부가).
  결과 표 `out/eval_table.csv`, 그림: D1/D2 곡면 비교(기존 vs 제안), ΔIpk 지도, 열화 지도. 논문 Fig. 2/3 스타일.
- [x] T6 **완료(2026-10-26)**: Simulink 자체 실행 — t6_verify 52점(10-24 19:40, `out/simulink_steady.csv`), Python 플랜트 대
  Simulink 비교(`plant_vs_simulink.py`), R.4 PI 정상상태 150회. 아래는 이전 기록.
  Simulink 정상상태 검증. **대체 경로 판정 완료(2026-10-24 18:00, RESULTS 8절)**: validate90(8000 정상 행) 플랜트
  Ron40 무droop bias +0.008 / RMSE 0.177 A(이상 −0.133/0.250); replay2000 선형보간 RMSE 0.525 A vs ZOH 1.40 → 라벨 오류 확정;
  verdictA: k≈1(93.6–96.4 V) 플랜트 TPS 이득 0–16 mA = 사실상 없음; verdictB: b3 플랜트 평균 0.0793 A, 그러나 1.25 % 점이
  SPS보다 5 mA 넘게 나쁨(최악 −23.8 mA, V2 90–100 V, |P| < 430 W), LUT −135 mA. 원인 = 데드타임(t7_probe: td0이면 1 mA 이내,
  부호가 Ron/droop에 따라 바뀜) → T7. **남은 것: Simulink 자체 실행(사용자 로그온 시 t6_verify 자동)**.
  (이전 기록) **원인 확정(2026-10-24)**: MATLAB R2025b가 **온라인 라이선싱**
  (`C:\Program Files\MATLAB\R2025b\licenses\license_info.xml`: `<licmode>onlinelicensing</licmode>`, .lic 없음) →
  비대화형 Session 0에서는 MathWorks 계정 로그인/ServiceHost에 닿지 못해 로그 한 줄 없이 멈춤(matlab_probe2: -logfile
  비어 있음, 2.5분 후 종료). 해결은 사용자만 가능: (1) 데스크톱 로그온(대화형 잡 자동 실행), (2) MathWorks batch licensing
  토큰(MLM_LICENSE_TOKEN), (3) 라이선스 파일 방식 활성화. **대체 경로(진행 중, RESULTS 8절)**:
  - `out/plant_params.md`: .slx XML에서 데이터셋 시기(35d5a78) 플랜트 복원. **HEAD 모델은 입력이 상수로 주석 처리돼
    T6에 못 씀** → verify_steady_state.m을 `dataset_era/` 복사본(make_dataset_era.py, git 제외)으로 전환.
  - `plant_full.py`(전체 주기 슈팅: Ron, 다이오드, 데드타임 100 ns, 2.88° 엣지 올림, 직렬 C, 전원 처짐; test_plant_full ALL PASS).
    `plant_sim.py`는 반주기 대칭 간이판.
  - `validate_plant.py`(D0 = +90° 정상 행): 예비 300점에서 이상 모델 Ipk RMSE 0.238 A, 플랜트 0.150 A. 잡 `validate90`(8000점).
  - `transient_replay.py`: 생성 과정 재생. **선형보간 가설 채택(예비 40조합: RMSE 0.52 A, 전력 지문 0.9986 vs 0.9994;
    ZOH는 2000조합 RMSE 1.40 A, 0.952)** → 데이터셋 D0 라벨이 실제와 다름. 잡 `replay2000`(linear 진행 중).
  - `plant_verdict.py` A(k≈1 재최적화)/B(b3·LUT가 비이상 플랜트에서 SPS보다 나쁜지) 잡 `verdictA`, `verdictB`.
  다음: 잡 결과로 RESULTS 8절 확정, T7 판단(잔차가 유의하면 비이상 플랜트 기준 재학습 = T7).
  (이전 기록) **막힘(2026-10-21 17:10): 데스크톱 로그온 사용자 없음**
  (explorer.exe 없음) → --interactive 작업이 실행되지 않음(결과 0x41303 "아직 실행 안 됨"). 스모크 잡
  `matlab_smoke`는 로그온 트리거로 등록돼 있어 사용자가 로그온하면 자동 실행됨 → `job.py status/diag matlab_smoke`로 확인.
  그동안 verify_steady_state.m은 미리 작성해 둘 수 있음(실행은 로그온 후).
  **2026-10-21 17:45 준비 완료**: `make_verify_points.py` → `out/verify_points.csv`(52점: SPS 18, TPS v2 24(양방향),
  ICROS 10), `verify_steady_state.m`(상수 D0 30 ms, 마지막 1 ms + 10/15/20/25 ms 정착 확인), `compare_simulink.py`(측정
  V_LV/V_HV로 해석 재평가, k≈1 판정 출력). 대화형 잡 `t6_verify` 등록됨 → 로그온 시 자동 실행. 결과가 생기면
  `python -I -X utf8 Simulink/Thesis/compare_simulink.py`만 돌리면 됨. (미검증: MATLAB 쪽 신호 이름·변수는 Generate_TPS_Dataset.m 복사.) 먼저 MATLAB 스모크 잡(--interactive). 동작하면
  `Simulink/Thesis/verify_steady_state.m` 작성: `Modules/Control_Logic.slx`를 `Scripts/Generate_TPS_Dataset.m`과 같은 변수로
  수동 위상 모드(Mode_Select=1, Ctrl_Mode_Ext=3)에서 **상수 D0**로 ≥ 25 ms 실행, 마지막 1 ms에서 Ipk, i_L 평균(DC),
  P_LV, P_HV, V_LV, V_HV 측정. 점: SPS(V2 84/95/109.2 × D0 ±8, ±20, ±35) + TPS 대표점(제안 곡면의 몇 점, 기존 데이터에서
  k≈1 이득이 컸던 점 포함). 해석 모델(측정된 V_LV, V_HV 사용)과 비교 → 표·그림. parsim 사용 가능하면 병렬.
  판정: (a) k≈1 TPS 이득 실재 여부, (b) 해석–Simulink 잔차 크기·구조.
- [x] T7 **중단·종결(2026-10-26 범위 변경)**: 플랜트 평가 수치는 옛 해법 오류로 무효(RESULTS 9절 = 부록 A), 최종 모델은 b3.
  ±20 mA는 다중 정상상태라 판정 불가(R.3). 아래는 이전 기록.
  (2026-10-24 18:20) 잔차 보정 대신 **데드타임 견고 학습**: 플랜트 잔차의 문제 부분(k≈1 경부하 ±20 mA)은 부호조차
  플랜트 세부에 따라 바뀌어 보정 대상이 아님 → E2E에 SPS 이탈 비용 m(1−exp(−ΣΔ/δ0)) 추가(`train_e2e.departure`, --margin;
  LUT도 지원, test ALL PASS). 학습: b8(25 mA) 0.0791 / b9(40 mA) 0.0755 / b10(δ0 0.01) 0.0799 / L49m 0.0841(LUT 거의 불변),
  b3 0.0857(검증). 잡: pv_{A,B,C}_{all,k1}(플랜트 변형 3 × 영역 2, 모델 SPS,b3,b8,b9,b10,L49s3,L49m), e2e_b11(b8 이어 2000 step).
  다음: 결과로 최종 모델 선택(플랜트 3변형 모두에서 5 mA 초과 열화 0 + 평균 최대) → evaluate.py 최종 b* 지정(현재 vscore가 이상
  점수로 b3를 고름 → T7 모델을 명시적으로 선택하도록 수정), export_mcu/export_matlab 재생성, RESULTS 9절 확정.
  원 명세: (T6 결과에 따라) 잔차가 의미 있으면 회색상자: 해석 모델 + 잔차 보정(소형 회귀)으로 T1–T5 재실행. 무시할 만하면
  그 근거를 RESULTS.md에 기록.
- [x] T8 **완료(2026-10-26)**: R.4 = 제어기의 (D1, D2)를 Simulink에 넣은 PI 정상상태 판정. 기존 제어기의 PI 모드 실폐루프는
  2.88° 위상 격자 디더로 피크가 부풀어 판정에 못 씀(RESULTS R.7, 파일럿 기록). `test_smooth_reghdc_infer.m` MATLAB 실행 PASS
  (208 벡터, max|ΔD| 4.4e-16). 계단 응답은 범위 밖으로 중단. 아래는 이전 기록.
  최종 폐루프 검증: 제안 제어기를 Simulink에 넣은 버전(복사본 모델 또는 MATLAB Function, D0는 기존 PI)으로
  대표 운전점 정상상태 Ipk를 SPS와 비교, 전력 계단 응답 1–2개. MATLAB 추론 함수 `Simulink/Thesis/smooth_reghdc_infer.m`.
  **2026-10-24 준비**: `smooth_reghdc_infer.m`(P*, V2[, V1] → D1, D2; V1 물리 정규화, codegen용 persistent),
  `smooth_reghdc_params.m`(export_matlab.py 생성, 가중치 리터럴), `test_smooth_reghdc_infer.m`(시험벡터 208개,
  `out/smooth_reghdc_testvec.csv`). Python 왕복 검사 `test_export_matlab.py` PASS(5e-16). MATLAB 실행은 미검증.
- [x] T9 **완료(C 컴파일은 선택, 2026-10-26 범위 변경 R4)**: 내보내기와 비트 단위 Python 에뮬레이션까지. 컴파일러가 없어 C 컴파일
  시험은 하지 않음(한계로 RESULTS에 기록). 아래는 이전 기록.
  (2026-10-24: V1 등가 입력 함수 `smooth_reghdc_v1_equiv()` 추가, 수치 불변) `export_mcu.py` 작성: float32 + int16(위상을 2^-k 회전 단위 uint32로 → 랩어라운드 = mod 2π, Q15 cos 표 선형보간,
  int64 MAC) C 코드와 비트 단위 Python 에뮬레이션(`SmoothRegHDCFixed`, evaluate의 int16 행). b3 기준으로 내보냄:
  int16 max|ΔD| 2.7e-4(성능 동일, 4618 B), float32 5.6e-7(5128 B). **남은 것: C 컴파일 시험** — gcc/clang/cl 없음.
  선택지: (1) 사용자가 컴파일러 설치, (2) `pip install ziglang`(zig cc)로 PC 시험 — 시스템 Python에 패키지를 설치하는 일이라 사용자 확인 후.
  원 명세: MCU 배포물: `Simulink/Thesis/export_mcu.py` → `out/SmoothRegHDC_STM32.h` + `smooth_reghdc.c`(int16/float 둘 다,
  cos는 테이블 또는 CORDIC 주석), PC에서 C 참조 구현과 Python 출력 일치 테스트(gcc 있으면). 메모리/연산량 표.
- [x] T10 **완료(2026-10-26, R3와 함께)**: RESULTS 0절 요약·R절 본론·10절 장 구성·한계. 아래는 이전 기록.
  `Simulink/Thesis/RESULTS.md`(한국어): 방법, 표, 그림, 한계(이상 모델 가정, k≈1), 논문 장 구성 제안,
  ICROS2026 대비 개선 요약. 3.392 A(논문 최대 저감) 재검증 결과 포함.
  2026-10-24: 3.392 A 재검증 완료(RESULTS 4절, `icros_max_check.py`: 그 점의 정상상태 최적 1.686 A > 필요 1.344 A → 도달 불가;
  출처는 RegHDCTrainer.m 콘솔 출력, 데이터셋 I_SPS − 대리모델, 원 코드 덮어써짐). 8절 확정. 남은 것: 9절(T7) 확정, 장 구성,
  ICROS2026 대비 요약, 맨 앞 요약(초록용 수치).

## 선택
- [x] T11 ZVS 영역 분석(`zvs.py` → `out/zvs.log`, `zvs_table.csv`; 시험 1만 점, 이상 모델). 전 레그 ZVS(I_min 0):
  SPS 74.6 %, ICROS 곡면 50.9 %, LUT49 81.5 %, b3 78.8 %, 점별 최적 87.9 %. |P|<300 W: SPS 23 % → b3 40 %.
  I_min 0.5 A에서는 모두 62–65 %(TPS는 스위칭 순간 전류가 작아 ZVS 여유가 얇음 — 정직하게 기록).
- [ ] T12 V2 범위 확장 민감도(72–120 V) — 이득이 커지는지.
- [x] T13 완료(2026-10-24, RESULTS 5절): `train_lut_e2e.py`(+test ALL PASS). 시험 1만 점: LUT49 E2E 0.0815 A/−0.9 mA,
  SPS 시작 L49s3 0.0815/−0.2 mA(최악점 최선), L25 1116 B 0.0813/−5.6 mA, L13 328 B 0.0804/−16.9 mA; RegHDC D64(b7, 1288 B)
  0.0812/−6.1 mA. → 성능을 좌우하는 것은 물리 E2E 학습, 파라미터화는 무관(정직하게 기록). evaluate.py에 행 추가.
  (원 메모) 2입력에서는 같은 메모리 LUT(T1 곡면 49×22, 4.3 kB)가 제안 (a)와 사실상 같은 성능
  (test 평균 ΔI 0.0812 vs 0.0814 A, 열화 0 vs 0.08 %). 공정 비교로 **LUT에도 같은 E2E 미세조정**(쌍선형 가중치 연쇄법칙,
  train_e2e의 loss_grad 재사용)을 적용해 비교. 논문 주장(통합·매끄러움·물리 E2E)이 LUT 대비 무엇을 더 주는지 정직하게 정리.
- [x] T14 (축소: 3입력은 이상 모델에서 소거됨) `v1_invariance.py` → `out/v1_invariance.log`. 이상 DAB 스케일링
  i = V1/(ωL)·F(k, D), P = V1²/(ωL)·G(k, D) (수치 확인 7e-15 A)이므로 V1 입력은 (V2' = V2·380/V1, |P|' = |P|·(380/V1)²)
  정규화로 2입력에 흡수된다 → 3입력 HDC로 메모리 장점을 보이는 실험은 이상 모델에서는 의미 없음(비이상성만 남김).
  대신 강건성: V1 361–399 V에서 b3 원 입력은 9.45 % 점이 SPS보다 5 mA 넘게 나쁨(최악 −302 mA, k 0.92–1.07),
  정규화 입력은 0 %(최악 −2.4 mA), 평균 0.0802 → 0.0908 A. → smooth_reghdc_infer.m에 V1 정규화 반영(C 내보내기는 아직).
