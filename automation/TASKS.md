# 작업 큐 (상태: [ ] 미착수, [~] 진행 중, [x] 완료, [!] 막힘)

## ★★★★★ 2026-10-29 19:10 사용자 지시 — 모든 것보다 먼저: "저렇게 생긴 게 정말 최적 곡면이 맞는지부터 검증해"
대상: 제안 곡면 M-B(`out/model_smooth_reghdc_b3.json`, 그림 `out/fig_surface3d.png`). **판정은 Simulink 수치로만**(해석 모델·Python 플랜트는
운전점·D0 계단 선택에만 쓴다). 사용자는 레퍼런스 [1]처럼 k = 1 중앙이 깊게 파여야 한다고 의심한다 — 그 가설도 Simulink로 직접 시험한다.
- **진행(반복 22, 10-29 21:00)**: OPT0 완료(커밋 fd95a30, 결과 전). `surface_opt_check.py`(make/next/compare; 묶음 mb = OPT3, k1 = OPT2,
  grid = OPT1)와 `opt_chain.m`(m_static_check ALL PASS) 작성. mb 118회 + k1 71회를 잡 **`opt1`**(`--after r5r6`, job.py에 새로 넣은 대기열
  옵션)로 제출 → r5r6 끝나면 MATLAB이 바로 이어 돈다(mb 약 75분 → k1 약 60분 → grid). **r5r6가 끝난 턴에 맨 먼저**
  `python -I -X utf8 Simulink/Thesis/surface_opt_check.py make --fams grid`를 돌려 `out/opt_points_grid_r1.csv`를 만든다(opt_chain이
  grid 차례(r5r6 종료 약 2.2 h 뒤)에 파일을 확인한다; 늦으면 `opt_chain(1)`을 `opt1b`로 다시 제출 — 끝난 묶음은 건너뜀). 그다음
  `r8_finalize.py`(R6 전체로 Q4 확정). opt1이 끝나면 `surface_opt_check.py next`(감싸지 못한 구성 → `opt_chain(2)`) → `compare` →
  OPT4(그림, `out/surface_opt_verdict.md`, REPORT 11절·RESULTS R.10). 2단계 증거: 109.2 V 100 W에서 데이터셋 조합 (0.247, 0.200)이
  M-B보다 0.20 A 낮음(R.8 Q2 "실재하는 차") → 규칙 (i)이 깨질 가능성이 큼 → 그러면 OPT4 재학습 경로(Simulink 격자 최적 목표)를 바로 준비.
- **진행(반복 23, 10-30 01:10–01:50)**: r5r6 3단계 묶음 20/21(02:05쯤 끝). 그사이 MATLAB 없이 할 일을 했다.
  (1) **Q2 창 민감도**(`flicker_check.py` → `out/flicker_check.{log,csv}`): 109.2 V 100 W의 "실재하는 차" 0.20 A는 끝 창에서만 나온다
  (15 / 20 / 25 ms / 창 최대: +0.13 / −0.13 / −0.08 / −0.07 A). 비교한 두 실행이 1 ms 창마다 피크가 오간다(폭 0.42–0.47 A, 진동 판정 아래).
  규칙은 그대로 두고 RESULTS 0절·R.8 (d)·R.10과 REPORT 0·6.6·8·11절에 "미확립"으로 고쳤다. **규칙 (i)이 그 점에서 깨진다고 미리 가정하지 말 것.**
  (2) `surface_opt_check.py compare`가 이제 `out/surface_opt_verdict.md`(규칙 (i)–(iii) 표, 측정 후보 수, 창 민감도: 25 ms 창·15 ms–끝 최대
  — 판정 아님)와 `out/fig_surface_opt_check.png`도 쓴다(부분 데이터로 시험 후 출력 삭제; 반복 22 시험의 `out/surface_opt_check.csv`가
  커밋돼 있어 이번에 지웠다). (3) `verify_steady_state.m`: 매 1 ms 창(15 ms–끝) 피크 열 추가 + i_L 마지막 한 주기 400점을
  `out/iL_wave/<결과 파일 이름>.csv`로 저장(R9용, try/catch로 본 출력에 영향 없음). m_static_check ALL PASS. **opt1이 첫 실행이다** —
  opt1 첫 묶음 로그에 'i_L waveform not written'이나 오류가 있으면 확인. (4) R9 준비 완료(아래 R9 줄).
- **진행(반복 24, 10-30 01:52–02:50)**: r5r6 끝(01:50, R6 363/363). opt1이 01:50부터 돈다 — 24회 묶음당 931 s, mb 5묶음 → k1 3묶음(40 ms)
  → grid 5묶음, **05:30쯤 끝**. 첫 묶음에서 수정된 verify_steady_state.m 확인: 오류 0, 매 ms 창 열(Ipk_at_15…30ms), `out/iL_wave/` 파형
  (24 × 400, NaN 0). **`out/opt_points_grid_r1.csv` 만들었다**(80구성 101회, 22구성은 기존 실행으로 이미 감쌈; R6 84/109.2 V 121/121).
  계단 상위 조합은 대부분 M-B와 같은 쪽(84 V D1 ≤ D2, 109.2 V D2 < D1). r8_finalize → Q4 판정(아래 R7 줄). 다음: opt1 끝나면
  `surface_opt_check.py next` → (감싸지 못한 구성이 있으면 `opt_chain(2)`를 `--after rob1`이 아니라 **가장 먼저** 돌도록 제출 — 사용자
  최우선) → `compare` → OPT4.
- **진행(반복 25, 10-30 05:05–05:40)**: opt1 끝(05:03, 290회 오류 0). `surface_opt_check.py compare` 1라운드(잠정, 46/187구성 미포착):
  (i) "최적 아님" 9점 중 4점(84 V 100/150/300 W +0.067/+0.070/+0.089, 109.2 V 100 W +0.250 A; 최적 = 84 V 200/600/1000, 109.2 V 300/1000 W),
  (ii) "k = 1 골 실재"(95 V 100 W D = 0.40 −0.149 A; 같은 계단 약 70 W에서 D 0.35–0.45 0.415 vs SPS 0.738 A; 300·600 W는 SPS 최선),
  (iii) "불일치"(같은 쪽 100 %, 84 V ρ 0.798 < 0.8 하나). 판정 아닌 점검 셋을 compare에 넣었다: 곡면 거칠기 σ ≈ 0.067 A(경부하) →
  최솟값 선택 편향 기댓값(84 V 100·300 W 0.110·0.107 A > 차), 95 V 같은 계단 표, SPS 4.32° 실행은 마지막 주기 0.41 A인데 1 ms 최대 0.738 A
  (간헐 스파이크). `pwm_counter_check.py`: 제어기 위상 카운터 게이트 명령은 40 ms 내내 주기적 → 원인은 플랜트(데드타임·Coss 0).
  **MATLAB 순서 바꿈**: zvs/robust 점 파일을 `*_r1q.csv`로 옮겨 대기 중인 zvs1·rob1이 시작하자마자 실패하게 했다(kill 아님). → eff1 → optR
  → **k1map**(`k1_map.py`, 79회 40 ms, V2 90–100 V × 첫 네 계단 × {SPS, D 0.475/0.45/0.40} + 95 V 핵심 3실행 재실행(주기별 피크)) → zvs2 → rob2(같은 출력 이름).
  `verify_steady_state.m`이 이제 마지막 1 ms 80주기의 주기별 최대·최소를 `out/iL_pp/`에 쓴다(m_static_check OK; optR 첫 실행).
  주기별 피크 기록은 optR 첫 묶음(05:43)에서 확인했다(80주기 최대 = Ipk_sim, 오류 0).
  **다음 턴**: optR가 끝나면(약 07:00) `surface_opt_check.py compare`(2라운드) → RESULTS R.10·REPORT 11.1 갱신(1라운드 표를 2라운드로).
  `efficiency_sweep.py next` → eff2(`--after rob2`). k1map이 끝나면(약 08:05) `k1_map.py compare` → 앵커 폭을 정해 `train_corrected.py`
  → M-S 검증 점 생성·제출(OPT4 줄).
- **진행(반복 26, 10-30 06:47–07:30)**: optR 끝(06:46, 2–4라운드 98회 오류 0). **2라운드 판정(184/187구성)**: (i) "최적 아님"(판정 가능 11점 중
  5점, 모두 ≤ 300 W; 넷은 거칠기 기댓값 안), (ii) "골 실재"(그대로), (iii) **"일치"**(84 V ρ 0.798 → 0.805, 84 V 150 W G2가 잡혀서; `opt5`의
  84 V SPS 400 W가 G2 2.495 A보다 낮으면 ρ 0.748로 뒤집힘). **핵심 발견: 규칙이 판정하지 않는 점의 차가 더 크다**(compare에 `undecided_gaps`
  추가, 판정 아님): 109.2 V 600·800 W +0.441·+0.330 A(SPS 진동), 200 W +0.408 A(M-B 자신이 진동), 84 V 400·800 W +0.113·+0.117 A(SPS 미포착).
  해석 모델로는 이 점들의 M-B와 Simulink 최저 후보가 0.06 A 안으로 같다 → 기생 효과가 SPS 무릎에서 최적 듀티를 옮김(109.2 V D2 −0.03…−0.05,
  84 V D1 +0.04). **1라운드의 "중·고부하는 최적"은 틀렸다**(RESULTS 0절·R.10, REPORT 0·8·9·11절 고침). 거칠기 점검 보강: M-B 중심 d2가
  중부하 이상 +0.276(1 kW 두 점에서 M-B가 V자 바닥), 경부하 −0.058 → 기댓값은 두 σ 중 작은 쪽. 재실행 결정성 21쌍 Ipk 차 ≤ 1e-10 A.
  **플래너 결함 수정**: 목표 전력이 비인접 계단 사이 빈칸이면 `bracket()`이 반대 방향(음의 D0)으로만 늘렸다(84 V SPS 400/800 W, 95 V D 0.20이
  4라운드 내내 미포착) → `plan()` 빈칸 채우기 → `out/opt_points_{k1,grid}_r5.csv` 5회. **대기열 재배치**: `out/robust_points_r1q.csv` →
  `out/robust_points_r1q2.csv`로 옮겨 rob2가 zvs2 뒤 곧바로 실패 → **opt5**(`--after rob2`: `opt_chain(5, {'k1','grid'})` 후
  `opt_chain(1, {'ms','msk1'})` — 그때 M-S 검증 점 파일이 있으면 함께 돈다) → **rob3**(`--after opt5`, r1q2). rob3 끝나면 r1q2를 r1로 되돌려도 됨.
  **OPT4 준비**: 앵커 7개 `out/opt4_anchors_r2.csv`(k = 1 앵커는 k1map 뒤 추가), `train_corrected.py --delta`(M-B 판독층 + 보정분만 릿지;
  시험: 앵커 적중 0.001–0.016, 기울기 최대 2.75 vs M-B 2.82, M-B 최적점에서 변화 ≤ 0.009(84 V 1 kW D1 −0.003) — 링잉), `surface_opt_check.py`에
  ms/msk1 묶음(`configs_ms`, MS_TAG "MS", M-S가 M-B와 0.003 넘게 다른 점만; 새 운전점에는 M-B·SPS도; ms = 84/109.2 V × 100–1000 W 11단계 +
  89/104 V × 5단계 30 ms, msk1 = 93–97 V × 70–300 W 40 ms). 시험 모델로 ms 50구성, msk1 18구성(시험 산출물은 지움). k1map 첫 묶음(93·94 V)은
  같은 계단에서 SPS와 D 0.40–0.475가 비슷 → k = 1 골이 95 V 근처에 좁을 수 있음.
  **다음 턴(k1map 끝, 약 07:55)**: `k1_map.py compare` → k = 1 앵커 결정(폭이 ±1–2 V로 좁으면 "인코더 σ 0.1 ≈ 2.5 V로 표현 불가 + Coss 0 모델
  산물 가능성"으로 앵커에서 빼는 것도 선택지 — 근거를 적을 것) → 앵커 CSV 확정 → `train_corrected.py --anchors out/opt4_anchors_r2.csv --tag MS
  --delta` → `surface_opt_check.py make --fams ms,msk1`(Python 플랜트로 계단 예측, 수 분) → **opt5 시작(약 08:40) 전에** `out/opt_points_ms_r1.csv`·
  `out/opt_points_msk1_r1.csv`가 있어야 함. 그다음 M-S 판정 도구(`surface_opt_check.py`에 ms_compare: M-S − M-B, M-S − SPS, M-S − 그 점 최저 후보,
  규칙 (i) 같은 허용오차)를 결과 전에 써 둔다.
- **진행(반복 27, 10-30 07:53–08:25)**: k1map 끝(07:52, 79회 오류 0). `k1_map.py compare`(그림에 주기 피크 중앙값 표시 추가): SPS 간헐 스파이크
  띠 = V2 95–97 V(k 1.00–1.02) × 65–180 W(80주기 중 2–12), 93·94·100 V SPS 주기적, 93·94 V SPS = TPS ±0.07 A. 100 W 보간 SPS − 최선 D1 = D2:
  95 V +0.20, 96 V +0.48, 97 V +0.77 A. 첫 계단(1.44°) 전력이 93 V +91 W → 97 V −86 W(데드타임 위상 이동). TPS도 93–95 V 240–440 W에서 튐.
  **k = 1 앵커 = cap 2개**(96 V 100 W·97 V 200 W, D ≤ 0.475, wV 1.2, wP 70/80; `train_corrected.target()`에 mode 열 'cap' 추가 — M-B가 이미
  0.40–0.47인 97–98 V 100 W를 되돌리지 않도록). **95 V는 못 고침**(M-B 사전활성 −0.03…−0.04 포화, σ 0.1 ≈ 2.5 V; `fit_delta(sample_w)` 가중
  20–50이면 95 V 0.49까지지만 109.2 V 400/1000 W D2 +0.024 → 기각). → `out/opt4_anchors_r3.csv` → **M-S 학습**(`out/model_smooth_reghdc_MS.json`,
  앵커 밖 100–1000 W 최대 0.011, M-B 최적 6점 ≤ 0.008, 기울기 2.93). `surface_opt_check.py make --fams ms,msk1`(MSK1_V2에 98 V 추가) →
  `out/opt_points_ms_r1.csv` 84회, `out/opt_points_msk1_r1.csv` 86회(08:05, opt5 시작 전). **M-S 판정 규칙 결과 전 등록**: `ms_verdict.py` +
  `test_ms_verdict.py` 18항목 ALL PASS(08:15; git add/commit은 이 세션에서 승인 불가 → 러너 턴 끝 커밋이 등록, 파일 시각이 증거).
  `surface_opt_check.py ms_compare`(→ `out/ms_check.{csv,log}`, `out/ms_verdict.md`, `out/fig_ms_check.png`; M-S 실행 없이 시험만 하고 출력 삭제).
  **대기열 재배치**: `out/robust_points_r1q2.csv` → `r1q3.csv`(rob3가 opt5 뒤 곧바로 실패) → 새 잡 **msR**(`--after rob3`,
  `opt_rounds(2, 4, py, {'ms','msk1'})` — `opt_rounds.m`에 묶음 인자 추가, m_static_check OK) → **rob4**(`--after msR`, r1q3).
  **다음 턴**: zvs2 끝(약 08:40)이면 `zvs_check.py compare` → R9 표(RESULTS·REPORT). opt5 끝(약 11:10)이면 `surface_opt_check.py compare`
  (5라운드: (iii) 84 V SPS 400 W, 84 V 400/800 W 규칙 (i)) + `ms_compare`(1라운드 잠정) → RESULTS R.10·REPORT 11.3. msR 끝나면 ms_compare 확정.
- [x] OPT0 **판정 규칙 먼저 등록**(결과를 보기 전에): `Simulink/Thesis/surface_opt_verdict.py`에 규칙과 가짜 입력 시험(`test_surface_opt_verdict.py`).
  (반복 22: 19항목 ALL PASS. 후보 = 2.88° 두 계단 정상상태만, 진동(|mean i_L| > 20 mA) 후보 제외, M-B/SPS 진동이면 판정 불가,
  (ii)는 min(D1, D2) < 0.45, (iii)은 같은 쪽 = s(D2*−D1*) ≥ 0 비율 ≥ 75 %·SPS만 최적이면 불일치, 상승 = Spearman ≥ 0.8 & 최대 전력 d* ≥ 0.45.)
  (i) 한 점에서 "M-B가 Simulink 최적": Ipk(M-B) − min(그 점의 모든 Simulink 후보 Ipk) ≤ max(0.03 A, Ipk_SPS의 2 %). (ii) "k = 1 중앙의 깊은 골이
  실재": V2 = 95 V에서 D < 0.45인 후보 중 하나라도 같은 전력의 SPS보다 0.05 A 넘게 낮다(100/300/600 W 중 어디서든). (iii) 곡면 모양 판정:
  Simulink 격자 최적 (D1*, D2*)가 M-B와 같은 쪽(84 V는 D1 < D2, 109.2 V는 D2 < D1)이고 |P|가 커질수록 0.5로 오르는지. 규칙은 이후 바꾸지 않는다.
- [x] OPT1–OPT3 2라운드 완료(10-30 06:47, 184/187구성, 위 반복 26). 남은 3구성 = 잡 `opt5`(플래너 결함 수정 뒤 5회). 끝나면 compare 다시
  → (iii) 확정(84 V SPS 400 W), 84 V 400/800 W 규칙 (i) 판정 → 판정문·RESULTS R.10·REPORT 11.1 숫자 갱신.
- [x] OPT1 **Simulink 격자 최적(= r5r6 3단계 R6 결과 활용)**: R6 = 사용자 생성기 실험 그대로 + D0 보간 끔, V2 {84.0, 94.61, 109.2} × D1, D2 ∈ 11값(0.2–0.5).
  r5r6 2단계(계단 D0 라벨에서의 30 ms 정상상태)로 **1 ms 계단 값이 정상상태와 같은지 먼저 확인**(같지 않으면 그 차를 표로 보고하고 격자 판정은
  정상상태 보정 후에). 각 (V2, |P| ∈ {100, 150, 200, 300, 400, 600, 800, 1000} W, 두 방향)에서 조합마다 첫 분기에서 목표 전력을 감싸는 인접 D0
  계단 사이를 측정 전력으로 보간한 Ipk → 조합 중 최소 = Simulink 격자 최적. 허용오차(규칙 (i)) 안에 드는 조합 수도 센다(최적이 평평한지).
- [x] OPT2 **k = 1 중앙 시험(새 Simulink 실행)**: V2 = 95.0 V, |P| {100, 300, 600} W, (D1, D2) ∈ {D1 = D2 ∈ 0.50, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20} ∪
  {(0.45, 0.5), (0.5, 0.45), (0.40, 0.5), (0.5, 0.40)}, `simulink_sweep.py` 방식(상수 D0 두 계단으로 목표 전력을 감싸고 30 ms, 마지막 1 ms; 듀티는
  D_SAFE 상한). 감싸지 못한 점은 다음 라운드로.
- [x] OPT3 **M-B 국소 최적성(새 Simulink 실행)**: (84, 109.2 V) × (100, 300, 1000 W)에서 M-B의 (D1, D2)와 ±0.02, ±0.04 십자(9 후보/점) → 같은 방식으로
  같은 전력 Ipk. M-B가 국소 최소(규칙 (i))인지.
- [~] OPT4 정리(판정문·그림·REPORT 11절·RESULTS R.10은 2라운드로 완료, 10-30 07:20; k = 1 지도 REPORT 11.2, M-S 학습·규칙 등록 REPORT 11.3
  완료 10-30 08:20, **M-S Simulink 검증 대기**(opt5 → msR → `ms_compare`) — 위 반복 27): `surface_opt_check.py` → `out/surface_opt_check.{csv,log}`, 그림 `out/fig_surface_opt_check.png`(V2별 |P|–D1/D2 단면: M-B 곡선, Simulink
  격자 최적 점, 허용오차 안 후보 범위 막대; 아래 행 Ipk(M-B) − Ipk(최적)). 판정문 `out/surface_opt_verdict.md`. REPORT.md·RESULTS.md에 절 추가.
  규칙 (i)이 깨지는 점이 있으면 곡면을 고치는 방법(Simulink 격자 최적을 목표로 재학습 = R13 앞당김)을 제안하고 바로 진행.
  (반복 25 설계안, 1라운드 근거): 보정은 경부하 두 곳만. (a) k ≈ 1 경부하 → D1 = D2 ≈ 0.45(범위는 k1map: V2 폭, 어느 계단까지 SPS보다
  나은지; 300 W 이상은 SPS 유지). (b) 109.2 V 100 W 부근 → 흔들리지 않는 후보 쪽((0.263, 0.200) 또는 (0.199, 0.127); 2라운드로 150/200 W 확인).
  84 V 경부하는 거칠기 범위 안이라 M-B 그대로. 방법: 목표 = M-B 곡면 + 매끄러운 보정 (ΔD1, ΔD2)(|P|, V2)(가우스 범프, 폭은 측정 간격),
  같은 Smooth RegHDC로 지도학습(해석 모델 E2E 미세조정은 하지 않음 — 보정을 되돌림) → M-S. Simulink 검증 = R.4 12점 + OPT 점들에서
  같은 규칙. k = 1 골은 Coss 0 모델의 데드타임 현상일 수 있다는 단서를 함께 적는다.
  **도구 완료: `train_corrected.py`**(--anchors CSV[V2, P, D1, D2, wV, wP, source], --alpha, --anchor-weight, --tag) → model json, log, 그림.
  임시 앵커(`out/opt4_anchors_r1test.csv`, PROVISIONAL)로 시험: M-B 인코더(σ 0.1)로는 ±1.5 V·±45 W 폭의 k = 1 앵커 0.45가 0.484까지만
  표현된다(α 1e-3). α 1e-5 + 앵커 가중 50이면 0.467이지만, 앵커 밖 최대 변화가 0.033(기준 0.007)으로 커지고 야코비안 최대가 3.64(M-B 2.82)다.
  109.2 V 앵커 (0.263, 0.200)은 (0.238, 0.186) → (0.254, 0.195)다. → **k1map으로 k = 1 효과의 V2 폭을 먼저 확인**한다. ±3 V 이상이면 지금
  인코더로 되고, 더 좁으면 인코더(σ)를 바꿔야 한다. 그러면 구조가 바뀌므로 그 사실을 보고서에 적는다.
  (반복 22 메모: Python 플랜트를 E2E 학습 루프에 넣는 길은 막혔다 — `plant_full.operating_point` 512점이 10분을 넘겨 중단(배치 4096 ×
  1200 step 불가). 남는 길은 Simulink 데이터 기반이다. OPT1 격자 최적 + OPT3 국소 탐색으로 M-B 곡면에 매끄러운 보정 (ΔD1, ΔD2)(|P|, V2)를
  맞추거나, R13(V2 7값) 데이터로 Simulink 정상상태 최적 곡면을 직접 만든다. V2 84/95/109.2 세 값만으로는 V2 방향 보간 근거가 약하다.)
  **(반복 26, 2라운드 근거로 설계 변경)**: 보정은 경부하 두 곳이 아니라 세 곳이다. (1) SPS 무릎: 84 V 400 W → (0.453, 0.500), 800 W → (0.484, 0.500);
  109.2 V 600 W → (0.500, 0.389), 800 W → (0.484, 0.389) (wP 90 W). (2) 109.2 V 경부하 능선: 100 W → (0.263, 0.200), 150·200 W → (0.295, 0.232)
  (wP 40 W). (3) k = 1 경부하(k1map 뒤). 모두 wV 5 V → 중간 V2(89/104 V)도 검증. `out/opt4_anchors_r2.csv`. 학습은 `--delta`로.
  84 V 경부하·109.2 V 150 W(거칠기 범위 안)와 M-B 최적점(84 V 200/600/1000, 109.2 V 300/400/1000 W)은 M-B 그대로 두되, 링잉으로 0.003–0.009
  움직이므로 ms 묶음이 그 점들도 다시 잰다.
- MATLAB 순서(반복 27, 10-30 08:30): zvs2(실행 중, 08:40쯤 끝) → rob2(곧바로 실패 = 의도) → **opt5**(5라운드 5회 + M-S 검증 ms 84회 30 ms·
  msk1 86회 40 ms, 약 2.3 h → 11:10쯤) → rob3(입력을 r1q3로 옮겨 둬서 곧바로 실패 = 의도) → **msR**(M-S 2–4라운드) → **rob4**(R10 111회, r1q3)
  → **eff2**(R8 나머지 10회, `out/eff_points_r2.csv` → `out/eff_sweep_r2.csv`). eff2의 84 V SPS L3/L4/L7 3회는 opt5 grid r5와 같다 →
  **opt5가 끝난 턴에 `out/eff_points_r2.csv`를 지우고 `efficiency_sweep.py next`를 다시 돌려**(r2로 다시 씀) 중복을 뺄 것(eff2는 시작할 때 읽음).
- MATLAB 순서(반복 26, 10-30 06:53): k1map(실행 중, 07:55쯤 끝) → zvs2(R9 70회, 약 46분) → rob2(입력을 r1q2로 옮겨 둬서 곧바로 실패 = 의도)
  → **opt5**(5라운드 5회 + `opt_points_ms_r1.csv`·`opt_points_msk1_r1.csv`가 그때 있으면 M-S 검증) → **rob3**(R10 111회, r1q2).
- MATLAB 순서(옛 기록): 지금 `r5r6`(10-29 19:03 재시작; 19:02 시스템 충돌 — 버그체크 0xa0000001, 그래픽/가상 디스플레이 드라이버 계열로 추정) → OPT2+OPT3를
  한 잡으로(약 200 실행, 1–2 h) → 아래 R8 이후. 충돌로 잡이 죽으면(`job.py`가 died 표시) 같은 이름으로 다시 제출하고 이미 있는 출력은 건너뛰게 한다.
  (반복 23: 대기열 = r5r6 → opt1 → eff1 → zvs1(70회, 약 45분).)
  (반복 25: 대기열 = eff1(실행 중, 05:30쯤 끝) → zvs1·rob1(점 파일을 옮겨 둬서 MATLAB 시작 직후 실패 = 의도) → optR(2–4라운드, 약 1–2 h)
  → k1map(약 1 h) → zvs2(R9 70회, 약 46분) → rob2(R10 111회, 약 75분). zvs2/rob2는 `out/zvs_points_r1q.csv`/`out/robust_points_r1q.csv`를
  읽고 출력 이름은 원래와 같다(zvs_sweep_r1, robust_sweep_r1). 대기 잡 순서를 바꾸는 방법 = 그 잡이 읽을 입력 파일을 치워 곧바로 실패시키고
  새 이름으로 다시 내기(kill은 대화형 대기 잡을 못 멈춤).)
  (반복 24: 대기열 = opt1(05:30쯤 끝) → eff1(약 25분) → zvs1(약 46분) → **rob1**(R10 111회, 약 75분) → **optR**(`opt_rounds(2, 4)`:
  `surface_opt_check.py next` → `opt_chain(rnd)`를 감쌀 것이 없을 때까지 MATLAB 안에서 반복). OPT 2라운드를 eff1 앞에 넣으려면 대기
  잡을 죽여야 하는데 **`job.py kill`은 대화형 대기 잡을 멈추지 못한다**. rob1로 시험해 보니 작업 스케줄러 작업만 지워지고, 세션 1의
  wscript → cmd → `python job.py wait`가 살아남았다(.done = −9만 써서 뒤에 줄 선 잡이 바로 풀릴 뻔했다). taskkill은 사용자 승인이
  필요해 루프에서 못 쓴다. 그래서 rob1.json을 running으로 되돌리고 .done을 지웠다. rob1은 원래 자리(zvs1 뒤)에서 돈다.
  **대기 중인 대화형 잡을 kill하지 말 것.** 순서를 바꾸려면 사용자가 데스크톱에서 해당 cmd.exe를 끝내야 한다.)
  (반복 22: `opt1` = OPT3+OPT2(+grid)로 r5r6 뒤 대기 중. 재부팅 주의: 두 잡의 로그온 트리거가 남아 로그온 때 둘 다 다시 뜰 수 있다 —
  r5r6가 died(.done −2)면 opt1의 대기가 바로 풀려 MATLAB 둘이 함께 돈다. 재부팅 뒤에는 `job.py status`로 보고 하나를 kill/재제출.)

## ★★★★ 2026-10-29 19:00 사용자 지시 — 최우선 (위에서부터 순서대로)
검증 원칙: LOOP_PROMPT "검증 원칙" — Simulink로 검증한 수치만 결론에 쓴다(해석 모델·Python 플랜트 = 참고, 표시 필수).
MATLAB은 한 번에 잡 하나(병렬 풀 6워커)만. 지금 `r5r6` 실행 중(10-29 18:42 시작, 약 5 h) — 그동안 MATLAB 없이 되는 준비를 먼저 한다.

- [x] (반복 21, 10-29 19:27) R0 초판 작성 완료 — `Simulink/Thesis/REPORT.md` 0–10절. 표시 [S]/[참고]/[대기]. 7절의 [1] 비교는
  `ref1_compare.py`(해석 모델, 참고)로 계산했다. 루프는 [1] 원문을 보지 못했다(저장소에 PDF 없음; 사양·그림 설명은 이 항목의 사용자 메모).
  이상 모델에서는 k = 1·100 W에 골이 없다. 그래서 Fig. 14(a)의 얕은 골은 원문 확인이 필요하다고 적었다. **새 Simulink 결과마다 갱신할 것.**
- R0 **사람이 읽을 수 있는 보고서** `Simulink/Thesis/REPORT.md` (한국어). 독자 = 사용자(DAB·TPS·RegHDC 전문, 이 보고서를 읽으며
  하나씩 검증하고 이 내용으로 졸업논문을 쓴다). RESULTS.md(루프 누적 기록)와 별개로, 처음부터 끝까지 읽히는 문서로 쓴다.
  내부 약어(T#, R#, b3, M-B0 등)는 쓰지 않거나 처음 한 번 정의한다. 모든 수치 옆에 (출처 파일, **Simulink 검증 / 참고(미검증)**)를 단다.
  구성: 0 한 쪽 요약(무엇을 했나, Simulink 검증 핵심 수치, ICROS2026 대비 무엇이 바뀌었나, 남은 것) / 1 문제와 설계(사양, PWM 규약,
  D0 = PI 가정, 목표 1–3) / 2 제어기 구조(분류기 없는 단일 연속 회귀, 부호화·출력·MCU 크기, `out/fig_surface3d.png`) / 3 두 학습 목표
  (기존 데이터셋 vs 정상상태 최적)와 만든 방법 / 4 검증 방법(데이터셋 시기 Simulink 모델, 상수 D0 두 계단 보간 = PI 정상상태, 진짜 PI
  폐루프를 판정에 안 쓴 이유) / 5 Simulink 결과(12점 표, 11점 평균 +0.13 A — 진동 점 포함 +0.22 A는 인용 금지, 95 V에서 기존 목표의
  열화) / 6 기존 데이터셋 재검증(비트 단위 재현, From Workspace 보간 기본값, SPS 과대, **최솟값 선택 편향**으로 TPS가 맞아 보인 이유,
  Simulink만의 분해 1.35 = 1.15 + 0.21 − 0.01 A, 3.392 A) / 7 저감 크기의 물리적 상한과 레퍼런스 [1](Li et al., JESTPE 2023:
  V1 200 V, V2 160–230 V, n 1, fs 20 kHz, L 140 µH) 비교 — 상대 저감은 같은 범위, [1] Fig. 15 곡면도 V2 가장자리에서 깊고 k = 1은 SPS
  근처, [1] Fig. 14(a) 200 V 100 W의 얕은 골(D 0.84–0.85 / 1) / 8 한계와 미검증 목록(상태 표) / 9 재현 방법(명령, 파일) /
  10 논문 장 구성 제안. 그림은 out/의 PNG를 상대 경로로 넣는다. 새 Simulink 결과가 나올 때마다 이 문서를 갱신한다.
- [~] (반복 21) R8 부분 완료: `efficiency_sweep.py compare`가 기존 R.4 실행을 재사용해 43/63구성의 η·손실을 낸다
  (`out/efficiency_compare.log`, RESULTS R.9, REPORT 5절). 남은 20구성 38회는 `out/eff_points_r1.csv`에 있다. **r5r6가 끝나면**
  `python automation/job.py submit eff1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_batch('out/eff_points_r1.csv', 'out/eff_sweep_r1.csv', 24, 0.030)"`
  (약 25 min)를 실행한다(반복 22: OPT가 최우선이므로 `opt1` 뒤로 — `--after opt1`). 그 뒤 `efficiency_sweep.py next`로 감싸지지 않은 구성이 있으면 r2를 돌리고, `compare`와 그림(Fig. 26–28 형식, 아직 없음)을 만든다.
  (반복 25: eff1 끝(05:27, 38회 오류 0) → compare 52/63, M-B 깨끗한 13구성 손실 −2.52 W, Δη +0.34 pp. `result_files()`가 opt_sweep_mb/grid
  30 ms 실행도 읽게 했다. **optR가 끝난 뒤** `efficiency_sweep.py next` → eff_points_r2 → 잡 eff2를 대기열 끝(`--after rob2`)에 낸다.)
  (반복 27: next에 빈칸 채우기 추가 → 57/63 감쌈, `out/eff_points_r2.csv` 10회 → 잡 **eff2**(`--after rob4`). opt5 뒤 점 파일 재생성. eff2가 끝나면
  `efficiency_sweep.py compare` → R.9·REPORT 5절, 그다음 Fig. 26–28 형식 그림.)
  (r5r6_chain.m에 2b단계로도 넣었지만, 실행 중인 r5r6은 수정 전 파일을 읽었다.)
- R8 (개선 1) **효율·손실 곡선**: 기존 Simulink 정상상태 결과(R.4 스윕 출력, t6)의 P_LV·P_HV로 η = |P_LV|/|P_HV|(HV→LV)를
  같은 전력 보간으로 계산. 부족한 점은 추가 실행(SPS, M-A, M-B × V2 84/95/109.2 × |P| 100/200/300/400/600/800/1000 W). 그림: 레퍼런스
  Fig. 26–28 형식(ipk, η 대 P, V2별). 전력 손실도 표로.
- [~] (반복 23) R9 준비 완료: `zvs_check.py`(make / compare / selftest). 파형 = `verify_steady_state.m`이 10-30부터 저장하는 i_L 한 주기.
  판정은 Coss = 0 모델에 맞춰 "데드타임 100 ns 동안 들어오는 소자의 바디 다이오드가 전류를 나르는가"로 한다(끄는 순간·켜는 순간
  부호가 둘 다 맞으면 ZVS, 하나만이면 marginal). 변압기 방향·i_L 부호·시간 오프셋은 모든 실행에 공통인 하나로 di/dt에 맞춘다.
  selftest(Python 플랜트 파형, 참고)에서는 공통 프레임 σ = ρ = +1을 찾았고, 교과서 SPS 패턴(84 V 경부하 2차 hard, 109.2 V 경부하 1차 hard,
  84 V 1 kW 전부 ZVS)이 나와 PASS다. 잡 **`zvs1`**(`--after eff1`) = `out/zvs_points_r1.csv` 70회(R.4 괄호 계단을 그대로 재실행해
  파형을 얻는다. 결정적이므로 `flicker_check.py` 3절이 재현성도 확인한다). 나머지 20구성은 eff1이 파형과 함께 돈다.
  zvs1이 끝나면 `python -I -X utf8 Simulink/Thesis/zvs_check.py compare` → `out/zvs_check.{csv,log}` → RESULTS·REPORT에 표를 넣는다.
  (반복 25: zvs1은 의도적으로 실패시키고 같은 점을 **zvs2**(`--after k1map`, `out/zvs_points_r1q.csv`)로 다시 냈다. 끝나는 것은 zvs2다.)
- [ ] R9 (개선 6) **ZVS 범위(Simulink)**: 정상상태 실행의 i_L 파형에서 각 레그의 스위칭 순간 전류 부호로 ZVS 여부 판정(엣지 시각은 D0/D1/D2와
  제어기 PWM 타이밍에서 계산, 데드타임 고려). SPS, M-A, M-B 비교 표.
- [~] (반복 24) R10 준비 완료. 규칙을 결과 전에 `robust_check.py` 머리말에 등록했다(selftest ALL PASS). 변형은 L 81/99 µH, td 200 ns,
  V1 361/399 V(하나씩)이고, 점은 V2 84/109.2 × |P| 100/300/1000 W × {SPS, M-B(R.4 듀티)}다. 공칭은 R.4다. 점마다 허용오차 max(0.03 A, SPS 2 %)를
  넘게 나쁘면 "열화"다. 진동(|mean i_L| > 20 mA)이나 감싸지 못한 경우는 "판정 불가"다. 75 % 이상 판정 가능하고 열화가 없으면 "견고"다.
  MATLAB 쪽은 `verify_steady_param.m`이다(verify_steady_state의 복사본 + 열 L_uH/td_ns/V1 → setBlockParameter Inductor3 'l', UCC0–3
  tdLH/tdHL, Ref_VH; 묶음·재시작 안전, m_static_check OK). opt1이 원본을 쓰는 중이라 복사본으로 만들었다 — 대기열이 빈 뒤에 합칠 것.
  `make` → `out/robust_points_r1.csv`, 잡 **`rob1`**(`--after zvs1`). 끝나면 `robust_check.py next`(감싸지 못한 구성이 있으면 r2) → `compare` →
  `out/robust_verdict.md` → RESULTS·REPORT. **첫 실행이다 — 로그에서 setBlockParameter 오류가 있는지 먼저 확인**(Simscape compiletime 파라미터).
  (반복 25: rob1은 의도적으로 실패시키고 **rob2**(`--after zvs2`, `out/robust_points_r1q.csv`)로 다시 냈다.)
- [ ] R10 (개선 5) **파라미터 견고성(Simulink)**: L ±10 %, 데드타임 100→200 ns, V1 ±5 %(블록/변수 값을 시뮬레이션 입력으로만 바꿈, 모델 저장 금지)
  × 대표 6점 × {SPS, M-B}(가능하면 M-A) — 상수 D0 두 계단 보간. M-B가 어디서도 SPS보다 의미 있게 나빠지지 않는지.
- [ ] R11 (개선 4) **동특성: PI 폐루프 계단 응답**: `dataset_era/`의 DAB_Controller_System.m **복사본**에서 위상 분해능 문제(100 ns, 2.88°)를
  해결(예: 위상 갱신을 주기 동기로, 분해능 향상 또는 엣지 시각 보간)한 뒤, D0 = PI, D1/D2 = 제어기 출력으로 전력 계단(1000→500→1000 W)과
  V2 계단(예 84↔95 V) 응답. 정착 시간, 오버슈트, 과도 중 Ipk. SPS vs M-B. 모델 원본은 수정하지 않는다.
- [ ] R12 (개선 2) **분류기 제거 효과 시연**: R11의 폐루프로, SPS/TPS 경계를 지나가는 느린 전력 램프(예: V2 90 V 또는 110 V, 0→800 W)에서
  기존 하이브리드(ICROS 분류기 + FSM 고정 곡면 = `Simulink/RegHDC_PostProcessed_Dataset.csv`를 경계 그대로 재생)와 M-B를 비교.
  D1/D2 점프, 전류 스파이크, 전력 추종 오차. 목표 2("곡면을 매끄럽게")의 직접 증거.
- [ ] R13 (개선 3) **수정 데이터셋으로 재학습**: R6(보간 끈 사용자 방식 데이터셋)을 V2 7값(84, 88.2, 92.4, 96.6, 100.8, 105, 109.2) × D1, D2 11×11로
  넓혀(재시작 안전, 배치 저장) Simulink 정상상태 최적 곡면을 직접 만들고, 같은 E2E 회귀로 학습(M-S). Simulink(R.4 점 + 무작위 일부)로 평가해
  M-B와 비교. 오래 걸리므로(수 시간) R8–R12의 짧은 MATLAB 잡 뒤에 밤새 돌린다.
- (메일 알림은 사용자 결정으로 쓰지 않는다, 10-29.)

## ★★★ 2026-10-29 18:30 재부팅 (사용자) — 재부팅 후 첫 턴에 할 일
- 사용자가 PC를 재부팅했다. 로그인 대기로 멈춰 있던 MATLAB 잡 `stair_i0`, `R6_smoke`, `r5r6`는 재부팅으로 죽었고(job.py가 부팅 이전 시작
  잡을 `died`로 표시하고 .done = −2를 쓴다), 그 셋의 로그온 트리거 작업(DABJob-*)은 로그온 때 세 MATLAB이 한꺼번에 뜨지 않도록
  대화형 세션에서 지웠다(DABJob-matlab_smoke도).
- **`r5r6` 하나만 다시 제출**한다(3단계가 stair_i0·R6_smoke를 포함한다; `waitForJobs`는 그 둘의 .done(−2)을 보고 바로 통과):
  `python automation/job.py submit r5r6 --interactive -- matlab -batch "<이전 r5r6과 같은 명령; automation/state/jobs/r5r6.json의 cmd>"`.
  제출 전에 `job.py status`로 셋이 died인지, MATLAB.exe가 남아 있지 않은지 확인. 사용자는 재부팅 후 Windows 로그온과 MATLAB 로그인을 한다.
  로그인 전이면 대화형 작업은 로그온 트리거로 대기한다 — 그동안 4시간마다 확인만 하고 사용량을 아낀다.
- 끝나면 `python -I -X utf8 Simulink/Thesis/r8_finalize.py` → `out/r8_verdict.md`(판정 규칙은 10-28 사전 등록, 바꾸지 말 것).
- [~] (반복 18) 확인 완료(셋 died/failed, MATLAB.exe 0개) → **`r5r6` 10-29 18:32:55 재제출**, 18:33 현재 로그온 트리거 대기.
- [~] (반복 21) r5r6는 18:42에 시작해 1단계(진짜 계단, `out/staircase_check_interp0.csv`)를 마쳤다. 19:05 PC 재부팅으로 2단계 18/36회에서
  죽었다(died). 19:09:56에 재제출했고, 19:16 로그온 뒤 19:21부터 2단계를 다시 돌고 있다. 1단계 판정(Q1)은 r8_verdict.md에 이미 반영됐다(r = 0.33,
  "대부분 보간 산물"). `verify_steady_batch.m`은 이제 있는 part 파일을 건너뛰고, 체인 2단계는 항상 호출한다(재부팅 중단 대비).

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
- [x] (반복 24, 10-30) **R5·R6·R7 Simulink 완료.** R6 363/363(`out/dataset_fixed_part_001…021.csv`), `r8_finalize.py` → `out/r8_verdict.md`
  Q4: 규칙 1 r = 0.51 "일부만 보간 산물", 규칙 2 "수정 데이터셋 최선이 M-B보다 평균 0.40 A 더 줄인다"(최대 +1.88 A, 109.2 V 300 W).
  분해 `q4_decompose.py`(판정 아님): 0.40 = 진짜 계단 SPS 과대 0.35(87 %, 거의 109.2 V 300 W 4.62 대 2.82 A) + 같은 전력 Ipk 차 0.05 A
  (최대 0.30 A @ 109.2 V 100 W). RESULTS 0절·R.8 "R7 Simulink" 표, REPORT 0절 표·6.4b절·8절 반영. 최저 조합의 정상상태 = OPT1 격자 묶음.
- [x] R7 **(10-29 01:05, 반복 13, 마감 후: MATLAB 30시간째 정지(dcpu ≈ 0.03 배경 틱, 산출물 변화 없음). 로그인 후 경로 자체 시험
  재확인 ALL PASS. RESULTS의 정지 시각은 01:03으로 고정하고 "이후 확인은 PROGRESS에만"으로 바꿨다 → **대기 턴은 PROGRESS 한 항목만
  쓰고 짧게 끝낸다**(RESULTS·TASKS 수정 불필요). 로그인 후 할 일은 아래와 같다.)**
  **(10-28 21:00, 반복 12: MATLAB 25시간째 정지(세 MATLAB dcpu 0, 산출물 변화 없음). 이번 턴은 막히지 않은 T9 C 컴파일
  검증을 끝냈다. 로그인 후 할 일은 반복 11과 같다(r8_finalize 한 번 → r8_verdict.md를 0절·R.8에).)**
  **(10-28 16:50, 반복 11: MATLAB 21시간째 정지. `r5r6_chain.m`의 세 단계를 `runStep`(try/catch)으로 격리 — 1단계가
  예외를 던져도 2단계 정상상태 106점(Q2·Q3 근거)과 R6가 돈다. 실패 단계가 있으면 마지막에 error → job status에 드러남.
  MATLAB 없는 구문 점검 `m_static_check.py`(+ `test_m_static_check.py` ALL PASS) — **.m을 고치면 제출 전에 반드시 돌릴 것**.
  RESULTS 0절의 R.4 평균을 진동 점 제외 11점 +0.13 A(+5.7 %)로 정정(12점 +0.22 A는 109.2 V 600 W 진동 점이 부풀림).)**
  **(10-28 12:45, 반복 10: MATLAB 17시간째 정지. (1) `r5r6_chain.m`이 먼저 stair_i0/R6_smoke의
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
  C 컴파일 시험은 10-28에 MATLAB 동봉 LCC로 완료(T9).
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
- [x] T9 **C 컴파일 시험 완료(2026-10-28 21:00, 반복 12)**: MATLAB에 딸린 LCC-win64(`C:\Program Files\MATLAB\R2025b\sys\lcc64`,
  MATLAB·라이선스 불필요)를 찾아 `c_compile_check.py`(+ `test_c_compile_check.py` ALL PASS: 연산 하나 바꾼 변이체를 int64/y 검사가 잡음)
  → `out/c_compile_check.{log,csv}` ALL PASS. 20,253점에서 int64 누산값·float32 y 비트가 SmoothRegHDCFixed와 전부 같음. 첫 컴파일에서
  C89식 비상수 집합체 초기화 2곳(오류)과 죽은 분기 음수 시프트(경고)를 `export_mcu.py`에서 고쳐 .c만 재생성(헤더 SHA1 불변, -A 경고 0).
  LCC libc에 `log1pf` 구현이 없어 하네스가 채움. ARM 툴체인·실기는 미검증. (이전: 2026-10-26 범위 변경 R4로 선택 처리)
  아래는 이전 기록.
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
- [ ] T12 **중단(2026-10-24 범위 변경, 위 "★ 범위 변경" 절의 중단 목록 — 하지 않는다)** V2 범위 확장 민감도(72–120 V) — 이득이 커지는지.
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
