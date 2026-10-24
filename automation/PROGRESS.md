# 턴 일지 (최신이 위)

<!-- 새 항목은 이 줄 바로 아래에 -->

## 2026-10-24 17:01–17:58 (반복 3) — 러너 크래시 수정 / MATLAB 원인 확정 / T11·T13·T14 완료 / T6 대체 경로(Python 플랜트) / T8 준비
- **러너가 10-21 17:54부터 3일 멈춰 있었음**: 턴 2 요약의 '−'(U+2212)를 cp949 콘솔에 print하다 UnicodeEncodeError로 종료
  (`runner_stdout.err`). `loop_runner.log()`를 파일 먼저 쓰고 콘솔은 errors=replace로 출력하게 수정(cp949 자가검사 통과).
  또 `running_jobs()`가 시작 못 한 대화형 잡(로그 없음: matlab_smoke, t6_verify)을 세지 않게 수정(헛대기 240분 방지).
  **수정은 러너 재시작 후 적용**: 현재 러너(pid 8652)는 옛 코드이므로 이번 턴 summary는 ASCII/한글만 썼다.
  10-24 16:54 턴(중단됨)이 train_lut_e2e.py/test를 남겼고 matlab_probe2(-logfile, 비대화형)는 로그 없이 멈춰 rc −9.
- **MATLAB 원인 확정**: R2025b `licenses/license_info.xml` = onlinelicensing(.lic 없음) → Session 0에서 로그인 불가로 정지.
  여전히 데스크톱 로그온 없음(explorer.exe 없음). 멈춘 MATLAB.exe는 이제 없음(재부팅된 듯).
- **회로 추출(하위 에이전트)** `out/plant_params.md`: 데이터셋 시기(35d5a78) 플랜트. HEAD Control_Logic.slx는 From Workspace
  전부 주석 처리(상수 Phi 70°, SPS, 96 V) + 신호 이름 변경 + DAB_Controller_System.m +333/−242 → `make_dataset_era.py`로
  옛 모델/제어기를 `Thesis/dataset_era/`(git 제외)에 꺼내고 verify_steady_state.m을 그쪽으로(경로 가림, parsim 워커 포함).
- **Python 플랜트**: `plant_sim.py`(반주기, 간이) → `plant_full.py`(전체 주기 Newton 슈팅 + 직렬 C 상태, 스텝 내 상태 비율 적분;
  스텝 중앙 샘플링은 15 mA/2 W 오차였음). 1000 µF 직렬 C 자체 효과 평균 +3.4 mA. `test_plant_full.py` ALL PASS.
  PI 정상상태 `FullPlant.operating_point`: 2.88° 양자화를 켜면 전력이 D0의 계단이라 PI가 못 맞춤(50 W 목표에 155 W) →
  판정용은 quant=False(100 ns는 Simulink 제어기 산물, HRTIM 184 ps). 데드타임 때문에 SPS V2 84 V에서 D0 = 0이 150 W를
  보내 φ ≥ 0 브래킷 제한 제거.
- **검증(예비, 300점, D0 = +90° 정상 행)**: 이상 모델 Ipk bias −0.135 / RMSE 0.238 A; 플랜트 Ron52 +0.052 / 0.150 A;
  noclamp(채널만 역도통)는 HV 손실을 맞추나(+4.8 W) Ipk +0.145 A. Simulink 손실 11.8 %를 플랜트가 덜 잡음(미해결, 작음).
- **재생(transient_replay.py)**: ZOH 2000조합 RMSE 1.40 A(라벨 이상 정상상태 1.48 A와 같은 수준), 전력 지문 0.952;
  선형보간 40조합 RMSE 0.52 A, bias +0.08 A, 지문 0.9986(데이터셋 0.9994) → **데이터셋은 D0 램프 중 측정, 라벨 ≠ 실제 D0**.
  1 ms 과도 DC 오프셋 가설은 기각(τ ≈ 0.1 ms).
- **T13** 완료(RESULTS 5절): LUT에 같은 E2E → LUT49 0.0815 A(b3와 동일), SPS 시작 L49s3 최악 −0.2 mA, L25 1116 B 0.0813,
  RegHDC D64 1288 B 0.0812. 파라미터화는 성능에 무관, E2E 학습이 핵심. evaluate.py에 행 추가, eval_table 갱신(32 s).
- **T14**(축소, RESULTS 6절): 이상 DAB 스케일링으로 V1 소거(7e-15 A). V1 ±5 %에서 b3 원 입력 9.45 % 열화(최악 −302 mA),
  물리 정규화 0 %(−2.4 mA), 평균 0.080 → 0.091 A. smooth_reghdc_infer.m(V1 선택 인수)과 C(`smooth_reghdc_v1_equiv`)에 반영.
- **T11**(RESULTS 7절): 전 레그 ZVS SPS 74.6 / ICROS 50.9 / b3 78.8 / 최적 87.9 %, 경부하 23 → 40 %; I_min 0.5 A면 모두 62–65 %.
- **T8 준비**: export_matlab.py → smooth_reghdc_params.m(리터럴), smooth_reghdc_infer.m, test_smooth_reghdc_infer.m,
  out/smooth_reghdc_testvec.csv; Python 왕복 test_export_matlab.py PASS(5e-16). MATLAB 실행은 미검증.
- **실행 중 잡**: replay2000(linear 단계), validate90(8000점, 변형 11개), verdictA(k≈1 재최적화, 2430 운전점),
  verdictB(1200점 × SPS/b3/LUT L49s3/L49, 비이상 플랜트 PI 정상상태).
- 다음: 잡 결과로 RESULTS 8절 확정(재생 RMSE, 검증 최적 변형, k≈1 판정, b3의 비이상 플랜트 열화 여부) → T7 결정
  (열화가 있으면 plant_full을 E2E 손실에 넣는 재학습; 기울기는 FD 또는 해석 모델 기울기 + 잔차 보정). 이후 T10 정리.

## 2026-10-21 17:20–18:05 (반복 2) — T3, T4, T5 완료 / T6 준비 완료(로그온 대기) / T9 내보내기(C 미컴파일) / job.py 출력 버그 수정
- **T3 (b) 결과**(검증 4000점, `train_e2e_b*_hist.csv`): 모두 (a)에서 시작해 단조 개선. b1 β4: 평균 0.08571, 최악 −4.6 mA, 열화 0;
  b3 β20: 0.08567, −2.3 mA; b4 γ1e-2: 0.0841, −2.0 mA, 인접 변화 0.087(가장 매끄러움); b2 D128: 0.08562, 열화 4점;
  b6 lr 6e-4 ≈ b1; **b5 SPS 초기화(곡면·데이터 없이 E2E만)**: 0.0849, 인접 0.086. 6잡 동시 실행 약 17분(2000 step).
- **b* = b3**: evaluate.build의 공통 검증 점수에서 힌지 가중을 4 → 20으로 바꿔(목표 3 우선) b1/b6 대신 b3 선택. test 결과는 RESULTS.md 3절.
  test 1만 점: 0.0815 A(최적 0.0819, 손실 0.47 mA), 최악 −2.8 mA, 5 mA 초과 열화 0, 조밀 최악 −2.0 mA. int16 C 에뮬 동일(4618 B).
- 결정 근거 기록: 평균 이득에서는 LUT 49×22(0.0812)와 사실상 동급 → RESULTS.md에 그대로 적음, 장점은 E2E 학습 가능성·매끄러움·
  차원 확장성(T13/T14 선택 작업으로 검증 제안).
- **job.py 버그 3**: 분리(detached) 잡의 프로그램 출력이 전부 사라짐. 원인: 콘솔 없는 cmd.exe가 띄운 콘솔 프로그램(python)은
  새 숨은 콘솔을 받아 거기에 쓰므로 `.cmd` 안의 `>> log`는 cmd 내장 echo에만 적용됨(probe로 확인: Python은 실행되지만 로그는
  start/exit 줄뿐). 수정: 분리 잡은 `job.py _run NAME` 감독 프로세스가 CREATE_NO_WINDOW + 로그 핸들로 직접 실행, .done 기록.
  대화형(스케줄 작업) 경로는 그대로. 새 하위명령 `job.py wait NAME...`(완료 순서대로 한 줄씩). 수정 전에 띄운 e2e_b1–b6는
  로그가 비어 있음(결과 JSON/hist CSV는 정상 기록).
- **T3 (a)** `train_e2e.py a`: 목표 곡면 λ{1e-3..2e-2} × D{64,128,256} × σ{0.1..0.35} × α{1e-4,1e-3}, 점수 = 검증 4000점
  평균 ΔI − 4·mean(relu(−ΔI)) − 1e-3·mean‖∂Δ/∂x‖². 선택: λ 2e-3, D 256, σ 0.1, α 1e-3 (Σ|M| 77) → 검증 평균 ΔI 0.0856 A,
  최소 −7.8 mA, 5 mA 초과 열화 6/4000. 상위 10개 점수 차 < 2e-4 A(구성에 둔감). α=1e-5는 같은 점수에 Σ|M|≈900이라 제외.
- **T3 (b)** 경로: 암시 함수 정리로 dIpk/dD = ∂Ipk/∂D − (∂Ipk/∂φ)(∂P/∂D)/(∂P/∂φ) (근 재탐색 없이 직접 평가 6회),
  `test_train_e2e.py`: φ 재해석 유한차분과 일치(중앙 오차 3e-5 = 기준의 이분법 잡음), dL/dM 상대오차 8e-4(γ=0).
  처음 hD=1e-4 후진차분은 O(h·f'') 편향 3e-3 → 1e-6으로. lr 2e-3은 발산(검증 최소 −421 mA) → 2e-4 + 워밍업 50.
- **점별 최적(무작위 점)** `pointwise_random.py`: 다중 시작(SPS, 주변 4 격자 최적, 쌍선형) 벡터화 패턴 탐색. 60점 무차별
  (격자 0.005 + Nelder–Mead) 대조: 평균 −0.004 mA, 최대 +0.000 mA(절대 나쁘지 않음). val 평균 이득 0.0861 A, test(1만, seed 2026)
  0.0819 A, dense 저장. → 제안 모델 평가의 상한.
- **T4/T5** `evaluate.py`: SPS, ICROS2026 곡면 LUT(±P), ICROS2026 RegHDC 재학습(±P), Legacy RegHDC(v2 곡면), LUT v2 49×22 /
  25×11(int16), 제안 (a)/(b*)/(b*) int16(C 에뮬). (a)까지 시험 실행 결과(test 1만 점):
  ICROS 곡면 평균 ΔI −0.053/−0.058 A(28–32 % 열화), ICROS RegHDC −0.27/−0.30 A(59–74 % 열화, 23점 실행불가),
  Legacy(v2) 0.048 A(17 % 열화, 입력 양자화 계단), LUT 49×22 0.0812 A(열화 0, 4328 B), LUT 25×11 0.0809 A(0.23 %, 1116 B),
  제안 (a) 0.0814 A(점별 최적 대비 손실 0.52 mA, 열화 0.08 %, 최소 −7.4 mA, 5128 B / int16 4618 B).
  **발견: 2입력에서는 같은 메모리 LUT가 (a)와 사실상 동급** → T13/T14(선택) 추가.
- **ICROS2026 재검증**(RESULTS.md 4절): 주장 평균 0.406 A → 정상상태 0.048 A, V2 90/100 V는 SPS보다 나쁨, 최대 주장 3.024 A
  (P −150 W, V2 110 V)는 기준 SPS가 4.736 A로 부풀려진 탓(정상상태 2.429 A) → 0.221 A.
- **T6 준비**: make_verify_points.py(52점), verify_steady_state.m, compare_simulink.py, 대화형 잡 `t6_verify` 등록(로그온 시 실행).
- **T9** export_mcu.py: b3 내보냄, int16 C 에뮬 max|ΔD| 2.7e-4, float32 5.6e-7. C 컴파일러 없음(미컴파일).
- 다음: T6은 로그온 대기(잡 자동 실행). 그동안 T13(LUT에도 E2E — 공정 비교), T8 준비(smooth_reghdc_infer.m, 폐루프 모델 복사본
  설계), T10 정리. 남은 MATLAB.exe 4쌍(Session 0, 약 2.3 GB)은 여전히 사용자 정리 대상.

## 2026-10-21 17:01–17:50 (반복 1) — T1, T2 완료 / MATLAB 막힘 원인 확인
- **job.py 버그 2건 수정**: (1) `submit NAME --interactive -- CMD`에서 argparse REMAINDER가 `--interactive`를 명령으로
  삼킴(rc 9009) → NAME 뒤 옵션을 직접 파싱. (2) `--interactive` 작업에 로그온 트리거 추가(아무도 로그온 안 했으면
  Start-ScheduledTask가 조용히 아무것도 안 함). 새 하위명령 `job.py diag NAME`(작업 상태, explorer.exe, MATLAB 프로세스).
  루프에서는 schtasks/PowerShell 직접 호출이 승인 필요로 막히므로 diag를 쓸 것.
- **MATLAB**: 데스크톱 로그온 사용자 없음(explorer.exe 없음) → 대화형 작업 미실행(마지막 결과 267011 = 0x41303).
  `matlab_smoke`는 로그온 시 자동 실행되도록 등록됨. Session 0에서 멈춘 MATLAB.exe 4쌍(각 ≈575 MB)이 여전히 남아 있음
  (PID 9332/10968/12332/12156 + 런처 4개). 사용자 정리 대상으로 둠. T6 [!].
- **T1** `Simulink/Thesis/smooth_surface_v2.py` (로그 `out/smooth_surface_v2*.log`, 표 `out/surface_v2_sweep.csv`).
  - 진단: v1에서 인접 변화 > 0.04인 142개 중 119개가 |P| < 100 W, 전부 |P| ≤ 175 W(V2 84–102 V); > 0.1은 전부
    V2 92.4–97.2 V, |P| ≤ 75 W. 최대 0.245는 V2 94.8 V, P 0→25 W.
    원인: k≠1이면 경부하 최적 D → D_min, k=1이면 모든 P>0에서 SPS 최적 → (P=0, k=1)이 특이점. 하드 허용집합(gap ≤ ε)이 따라가게 강제.
  - 방법: E = Σ gap(A) + μ·Irms/Irms_SPS + λ·Σ|ΔD|²/h²(모델 정규화 좌표), Ipk ≤ Ipk_SPS 하드, 0.005 격자 위 3×3 색칠 ICM,
    λ 오름차순 연속 + v1 초기값 중 낮은 에너지. 모든 결과는 pi_steady로 정확 재평가 + 조밀 격자 쌍선형 평가.
  - λ 스윕(1차): 1e-3 → gap 0.34 mA/step 0.15; 5e-3 → 1.37 mA/0.09; 2e-2 → 4.39 mA/0.065(단, P≤50 W 전 V2에서 이득 최대
    250 mA 손실 — 자연스러운 경부하 기울기까지 눌러버림). **선택 λ=5e-3**: v1보다 두 축 모두 우월(gap 1.37 vs 1.77 mA,
    step 0.09 vs 0.245, max|∂D/∂u| 1.92 vs 11.76, 조밀 최소 ΔI −2.4 vs −7.8 mA, 5 mA 초과 열화 0 vs 4점). 격자 평균 저감 0.0977 A.
  - 2차(thin-plate) 벌점도 시험(`v2o2`): 같은 gap에서 step이 더 크고, λ ≥ 1e-4에서 ICM이 격자 양자화 때문에 정체. 채택 안 함.
  - 남은 V 방향 꺾임(0.09–0.15)은 물리적: V2<95에서는 D1을, V2>95에서는 D2를 줄임 → V2=95 선에 V자 주름.
- **T2** `Simulink/Thesis/test_controllers.py` 19항목 ALL PASS(`out/test_controllers.log`): SPS 근 찾기 = 닫힌형(1e-10),
  첫 교차점 = 스칼라 brentq(400점 중 실행가능성 불일치 0), 전력 정확(5e-8 W), GridLUT 노드/쌍선형/클램프, LUT 폐루프 =
  저장 지표, LegacyRegHDC(D32 K8 Q32) 40 ep val RMSE 0.051(상수 0.092), 메모리 4160 B, |P| 0–1200에서 코드 16개(양자화
  계단), SmoothRegHDC ridge RMSE 0.006, 야코비안 = 유한차분(4e-9).
  - 발견·수정: SmoothRegHDCQ 최근접 cos 표 오차 ΔD 0.033 (α=1e-6에서 Σ|M|≈500이 ±π/1024 위상오차 증폭) → 선형보간(CMSIS
    arm_cos 방식)으로 바꿔 1.8e-3. α=1e-4면 Σ|M|≈75.
- 다음: T3 train_e2e.py (목표 곡면 λ 선택은 무작위 점 Ipk 손실로).

## 2026-10-21 (대화형 세션, 루프 시작 전) — 출발점
- 논문 분석: 기존은 SPS/TPS 분류 + TPS 회귀(FSM 고정 후처리), 전체 평균 저감 0.337 A, TPS 구간 1.222 A, 최대 3.392 A.
- 해석 모델 vs 기존 Simulink 데이터(2만 점): 전력 R² 0.989(부호 반대 규약), Ipk R² 0.858, RMSE 1.48 A, 평균 +0.88 A
  (Simulink가 큼), |D0|가 클수록 증가. SPS 행 780개: 평균 +1.44 A, 최대 +6.1 A, D0에 비단조(V2≈95 V: −31.6° 6.66 A,
  −28.1° 4.46 A) → 1 ms 계단 과도 측정의 DC 오프셋 의심.
- 기존 최적 데이터셋(Physics_Dataset_Before_Smoothing.csv) V2=90/100, |P|=1200 W: 데이터셋 SPS 6.2–6.7 A vs 정상상태 SPS
  4.2–4.4 A, "최적 TPS" 4.40 A = 정상상태 SPS와 동일 → k≈1 이득은 부풀려진 SPS 기준 때문일 가능성(정상상태 Simulink로 판정 예정).
- 이상 모델 이론 한계(전수탐색): V2=84 V 50 W −0.81 A … 1.2 kW −0.10 A; V2=109.2 V 50 W −1.18 A … 1.2 kW −0.13 A; V2=95 V ≈0.
- MATLAB `-batch`가 ssh(Session 0)에서 멈춤(5–9 s CPU 후 대기). 멈춘 스모크 프로세스 4쌍이 남아 있을 수 있음(사용자가 정리).
- 구현: tps_model.py(+테스트 PASS), optimal_surface.py(v1 실행 완료), pi_steady.py, controllers.py(미검증), 루프 러너.
