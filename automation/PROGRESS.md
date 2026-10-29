# 턴 일지 (최신이 위)

<!-- 새 항목은 이 줄 바로 아래에 -->

## 2026-10-30 01:12–01:55 (반복 23) — Q2 0.20 A는 측정 창 산물일 수 있음 / OPT4 판정문·그림 도구 / R9 ZVS 준비·zvs1 제출
- **러너가 240분 대기 만료로 깨웠다**(r5r6는 아직 실행 중). r5r6 3단계 R6는 묶음 17/21이 끝났고(01:00), 20번째 묶음이 01:32에 시작했다.
  02:05쯤 끝난다. 그동안 MATLAB 없이 할 수 있는 일을 했다.
- **발견: R.8 Q2 "실재하는 차"(109.2 V 100 W, M-B − 데이터셋 조합 +0.197 A)는 측정 창에 민감하다.** `flicker_check.py`
  → `out/flicker_check.{log,csv}`. M-B의 위 계단 실행(D0 15.84°, 124 W)은 `iL_mean`이 7.5 mA로 진동 판정 아래다. 그런데 1 ms 창 피크가
  15/20/25/30 ms에서 2.130/2.072/1.662/2.130 A로 오간다. 조합의 15.55° 실행도 1.262/1.681/1.262/1.262 A로 오간다. 피크만 다른 창에서
  읽으면 D − B_st는 +0.129/−0.126/−0.080/**+0.197**(끝, 판정값)/−0.069(창 최대) A다. 판정 규칙(r8_verdict.py)은 바꾸지 않았다.
  대신 "M-B가 109.2 V 100 W에서 최적이 아니다"를 "미확립"으로 고쳤다(RESULTS 0절·R.8 (d) 표·R.10, REPORT 0·6.6·8·11절).
  이렇게 오가는 실행은 진동 없는 정상상태 231개 중 5개(2.2 %)다. 109.2 V 경부하에 3개, 94.61 V TPS 0.453에 2개다. Q3은 영향이 없다
  (TPS 실행의 끝 창이 낮은 쪽이라 다른 창이면 TPS가 더 나쁘다). 84 V 두 점은 다섯 창 모두 같다. 원인은 미상이다.
- **`surface_opt_check.py compare` 확장**: 판정문 `out/surface_opt_verdict.md`(규칙 (i) 표에 측정 후보 수, (ii)·(iii) 표, OPT1 계단 대
  정상상태, 창 민감도 = 같은 두 계단에서 25 ms 창·15 ms–끝 최대로 다시 적용해 바뀌는 점 — 판정 아님)와 그림 `out/fig_surface_opt_check.png`
  (V2별 |P|–듀티: M-B 곡선, 점마다 최저 후보, 허용오차 안 범위; 아래 행 차·허용오차; 오른쪽 95 V 골)를 낸다. `measure(col=…)`, `win_cols()`.
  `--partial`로 시험했고 출력은 지웠다. 반복 22 시험 때 남아 커밋된 `out/surface_opt_check.csv`도 지웠다(부분 데이터였다).
- **`verify_steady_state.m`**(opt1부터 적용): 창 끝을 `[0.010, 0.015:0.001:simTime]`로 바꿔 매 ms 피크 열을 더했다(기존 열 이름은 그대로).
  또 i_L 마지막 한 주기 400점을 `out/iL_wave/<결과 파일 이름>.csv`에 쓴다(try/catch라 본 결과에 영향 없음, 하위 폴더라 결과 glob에 안 걸림).
  m_static_check ALL PASS. MATLAB으로는 아직 실행하지 않았다 → **opt1 첫 묶음 로그를 확인할 것**.
- **R9 `zvs_check.py`**: Coss = 0 모델의 ZVS 판정 = 드라이버 데드타임 100 ns 동안 들어오는 소자의 바디 다이오드가 전류를 나르는가
  (끄는 순간·켜는 순간 노드 전류 부호). 엣지 = 100 ns 격자 올림(plant_params.md 4절). 변압기 방향 σ와 i_L 부호 ρ는 모든 실행 합의 di/dt
  상관으로 정하고, 오프셋 c는 실행별 최적의 중앙값으로 정한다. 실행마다 따로 맞추면 D0 ≈ 0에서 σ, ρ가 뒤집혔다(109.2 V 100 W SPS).
  selftest(Python 플랜트 9점, 참고) PASS: σ = ρ = +1, 84 V 100 W SPS = ZZHH, 109.2 V 100 W SPS = HHZZ, 84 V 1 kW = ZZZZ. 플랜트에서 109.2 V
  1 kW SPS의 1차는 marginal이다(i_L(0) −0.84 A가 데드타임 안에 0.9 A 오른다). `make` → `out/zvs_points_r1.csv` 70회(R.4 괄호 계단 재실행 —
  결정적이라 재현성 확인도 됨, `flicker_check.py` 3절; 가짜 재실행 파일로 시험 후 삭제). **잡 `zvs1` 제출(`--after eff1`)**.
- 대기열: r5r6(02:05쯤) → opt1(mb 약 75분 → k1 약 60분 → grid) → eff1(약 25분) → zvs1(약 45분).
- 다음 턴(r5r6 종료): **맨 먼저** `surface_opt_check.py make --fams grid`(opt1이 grid 차례 전에 파일을 봐야 함) → opt1 첫 묶음 로그·
  `out/iL_wave/` 확인 → `r8_finalize.py`(R6 전체 Q4) → RESULTS R.8 R7 표·REPORT 6절.

## 2026-10-29 20:42–21:15 (반복 22) — r5r6 2단계 완료 → Q2/Q3 판정 / OPT0 규칙 등록 / OPT 잡 대기열
- **r5r6**: 2단계(같은 라벨 정상상태 106점)가 20:23에 끝났다(`out/stair_steady_part{001,037,073}.csv`). 3단계 R6는 20:24부터 돈다.
  묶음 1/21(18조합)은 982 s, 묶음당 약 16분이라 10-30 02시쯤 끝난다.
- `r8_finalize.py` → `out/r8_verdict.md`. 판정 결과는 다음과 같다. **Q2 "실재하는 차"**: 109.2 V 100 W에서 데이터셋 조합 (0.247, 0.200)의
  정상상태가 1.568 A이고 M-B는 1.764 A다(D − B_st +0.197 A; 84 V 두 점은 +0.053, +0.039). **Q3 "이득 없음"**: 94.61 V 100–200 W에서 TPS
  0.453 / 0.326이 SPS보다 0.011–0.468 A 나쁘다. 30 vs 20 ms 차는 최대 0.20 A다. Q1은 그대로다(r = 0.33). 같은 라벨에서 1 ms 창과
  정상상태의 차는 진짜 계단 평균 0.08 A, 최대 1.17 A다. **Q4는 R6 18/363조합으로 낸 값이라 판정으로 쓰지 않는다**(격자 최저가 D2 = 0.2 몇
  조합뿐). B_st의 불확도: 라벨 간격이 넓은 보간(15.6° → 74 W, 18.5° → 146 W)이고, 15.6° 실행은 30 ms에서도 정착 중이었다(20 ms 1.68 → 30 ms 1.26 A).
- **OPT0 완료(커밋 fd95a30, OPT 결과 전)**: `surface_opt_verdict.py` + `test_surface_opt_verdict.py` 19항목 ALL PASS. 세부 결정은 TASKS
  OPT0 줄에 있다(후보 = 2.88° 두 계단 정상상태만, 진동 제외, (ii) min(D) < 0.45, (iii) 같은 쪽 ≥ 75 %·Spearman ≥ 0.8·끝 ≥ 0.45).
- **`surface_opt_check.py`**(make/next/compare; mb = OPT3 십자 + 두 듀티 동시 +0.04/+0.08 + BT, k1 = OPT2 40 ms, grid = OPT1 R6 상위 3 + SPS
  + M-B, R6가 V2마다 121조합이 다 있어야 만든다) + **`opt_chain.m`**(묶음별 try/catch, 점 파일이 없으면 건너뜀; m_static_check ALL PASS).
  `make --fams mb,k1` → `out/opt_points_mb_r1.csv` 118회(M-B·SPS 12구성은 R.4 재사용), `out/opt_points_k1_r1.csv` 71회. 이상 모델로
  닿지 못하는 구성은 없었다. compare는 기존 실행으로 연기 시험만 했고, 그 출력은 지웠다.
- **job.py `--after JOB[,JOB]`**(대기열): interactive는 .cmd에서 `job.py wait`, detached는 `_run`에서 기다린다. qtest_a/b로 시험했다(b가 a
  종료 30 s 뒤 실행). 시험 파일은 지웠다. **잡 `opt1`**(`opt_chain(1)`, `--after r5r6`), **`eff1`**(R8 38회, `--after opt1`)을 제출했다.
  셋 다 running이고, 두 잡은 "waiting for" 상태다. 재부팅 위험은 TASKS MATLAB 순서 줄에 적었다(로그온 트리거가 둘 다 다시 뜰 수 있음).
- 다음 턴(r5r6 종료 = 러너가 잡 하나 끝나면 깨움): **맨 먼저** `surface_opt_check.py make --fams grid`를 돌린다(opt_chain이 약 2.2 h 뒤 grid 차례에
  파일을 확인). 그다음 `r8_finalize.py`(R6 전체 → Q4 확정), RESULTS R.8 R7 표·REPORT 6절 갱신. opt1이 끝나면 next → compare → OPT4.
- Python 플랜트 E2E 재학습은 막혔다(`plant_full.operating_point` 512점이 10분을 넘겨 중단). 수정 경로는 Simulink 데이터 기반으로 한다(TASKS OPT4 메모).
- RESULTS: 머리말, 0절(Q2·Q3·OPT 항목), R.8 (d) 표·k ≈ 1 표·같은 라벨 정상상태 문단, 새 R.10. REPORT: 0절 표 두 줄·남은 것, 6.4·6.6절
  [S] 결과, 8절 표, 9절 명령, 새 11절.

## 2026-10-29 19:08–19:27 (반복 21) — r5r6 재부팅 사망 → 재제출(19:21부터 실행 중) / REPORT.md 초판 / R8 효율 부분 결과
- **r5r6 경위**: 18:38 로그온으로 시작해 1단계 interp=0(5회, 358 s) → `out/staircase_check_interp0.csv` 195행(18:49). 2단계 part001
  18/36회(19:00:32)에서 PC가 재부팅됐다(GetTickCount 부팅 19:05:39). job.py가 died(.done −2) 처리. 옛 로그는
  `automation/state/jobs/r5r6_run1_1029_1838.log.bak`(state/, gitignore). **19:09:56 재제출** → 19:16:46 로그온으로 작업 시작,
  `r5r6_chain: start 19:20:59`, 1단계 건너뜀, 2단계 19:22:32 시뮬레이션 시작. 남은 시간 약 1 h(2단계) + 4–5 h(3단계).
- 앞선 반복 21 시도(18:58, 19:04 러너 재시작으로 끊김; PROGRESS 항목 없음)가 `staircase_compare.py` + `r8_verdict.py`(18:59)와
  `selection_bias.py`(19:00)를 돌려 두었다. 확인 결과 Q1 판정은 r = 0.33(13칸 평균 +1.42 → +0.47 A)이고, 규칙상 "대부분 보간 산물"이다.
  진짜 계단에서도 SPS가 튀는 칸이 있다: 109.2 V HV→LV 300 W 4.62 A(정상 2.82 A), 84 V LV→HV 300 W 4.16 A. 2단계에서 확인한다(규칙은 안 바꿈).
- **R0 `Simulink/Thesis/REPORT.md` 초판**(0–10절, 모든 수치에 [S]/[참고]/[대기] + 출처). RESULTS 머리말·0절·R.8 (ii)를 오늘 결과로 고쳤다.
- **`ref1_compare.py`** → `out/ref1_compare.{log,csv}`(해석 모델, 참고). [1] 사양(200 V, 160–230 V, n 1, 20 kHz, 140 µH)의 상대 저감은
  가장자리 경부하 31–35 %, 1 kW 0.5–2.4 %, k = 1에서 0이다. 이 설계 값은 tps_limit.log와 같다(29.3 %, 39.0 %). k = 1·100 W에서는
  D1 = D2 = 0.42–0.425(문헌 0.84–0.85)가 SPS보다 17–19 % 높아 이상 모델에 골이 없다. → Fig. 14(a)의 골은 원문 확인 필요.
- **`efficiency_sweep.py`**(R8): R.4 실행을 재사용해 compare → 43/63구성. M-B는 진동 없는 12구성에서 손실 −2.70 W, Δη +0.36 pp,
  최악 84 V 100 W −1.31 pp다(Ipk·RMS가 낮은데 손실 +1.44 W, 원인 미상). M-A는 95 V 100 W −4.0 pp. `next` → `out/eff_points_r1.csv` 38회.
- **MATLAB 파일 수정**(m_static_check ALL PASS): `verify_steady_batch.m`은 part 출력이 있으면 그 묶음을 건너뛴다. `r5r6_chain.m`은 2단계를
  항상 호출하고(이전: part 하나라도 있으면 2단계 전체 건너뜀 → 재부팅 시 점 누락 위험), 2b단계(eff_points_r1)를 3단계 앞에 넣었다.
  다만 실행 중인 MATLAB은 19:20:59에 수정 전 체인을 읽었으므로 이번 실행에서 2b는 돌지 않는다 → r5r6 뒤 잡 `eff1`로 따로 돌린다.
- 다음 턴: r5r6 진행 확인(2단계 part001/037/073). 끝나면 r8_finalize.py → REPORT/RESULTS 갱신, 그다음 eff1 제출.
  기다리는 동안 MATLAB 없이 할 일은 R9 ZVS 판정 코드 준비다(verify_steady_state의 i_L 파형 저장 여부 확인 필요).

## 2026-10-29 19:15 (대화형 세션) — 시스템 충돌 2회, MATLAB 소프트웨어 렌더링, 최적성 검증 최우선
- **19:02, 19:05 시스템 버그체크 0xa0000001 두 번**(Kernel-Power 41, 미니덤프 `C:\Windows\Minidump\102926-*.dmp`).
  직전 19:01:07 가상 디스플레이 드라이버 `MttVDD.dll` 런타임 오류(10121) → 장치 오프라인, `C:\Windows\LiveKernelReports\WATCHDOG\`
  덤프가 19:01–19:05 매분. 두 번째는 r5r6 MATLAB 시작(19:03:27) 약 2분 뒤. 그래픽/가상 디스플레이 스택 문제로 추정(사용자에게 HDMI 더미
  플러그·AMD 드라이버 재설치 권고). 그 사이 r5r6가 2단계에서 두 번 끊김.
- 대응: `job.py`가 MATLAB 잡에 `-softwareopengl`을 자동으로 붙인다(GPU 렌더링 회피). 이미 제출된 `state/jobs/r5r6.cmd`도 같은 옵션으로 고침.
  MATLAB이 이 옵션을 거부하면(로그에 오류) 옵션을 빼고 다시 제출하고 기록할 것.
- **사용자 지시: 제안 곡면이 정말 최적인지 Simulink로 먼저 검증** → TASKS.md "★★★★★" OPT0–OPT4가 최우선.

## 2026-10-29 18:36–18:41 (반복 20) — `r5r6` 여전히 로그온 대기 / 1분 간격 턴 반복 멈춤
- `job.py diag r5r6`(18:37): 작업 "준비", 마지막 결과 267011(한 번도 실행 안 됨), explorer.exe 0개, MATLAB.exe 0개 — 반복 19와 같음.
- 반복 18–20이 1분 간격으로 열린 원인: `loop_runner.running_jobs()`(355–368행)는 `.log`가 없는 --interactive 잡을 "시작 안 됨"으로
  세지 않는다 → `wait_for_jobs:true`를 보고해도 대기 없이 `min_gap_s`(60 s) 뒤 다음 턴(턴당 $0.4–0.8).
- 조치: `automation/state/jobs/r5r6.log`에 "placeholder, NOT job output" 세 줄을 미리 썼다(state/는 gitignore). 잡 .cmd는 `>>`로
  덧붙이므로 실제 출력은 그 아래 "==== job r5r6 start"부터 이어진다. interactive 잡의 사망 판정은 부팅 시각만 보므로(job.py 163행)
  이 파일로 died가 되지 않는다. 이제 러너는 최대 240분 기다린다. 러너 코드는 바꾸지 않았다(실행 중인 프로세스는 다시 읽지 않음).
- 다음 턴: 로그에서 placeholder 아래 줄부터 본다. "==== job r5r6 start"가 없으면 아직 로그온 전.
  나머지 절차는 반복 18 항목과 같다.

## 2026-10-29 18:35 (반복 19) — `r5r6` 로그온 대기 확인만
- `job.py status`: r5r6 running(18:32:55 제출), 로그 없음. `job.py diag r5r6`(18:35:09): 작업 \DABJob-r5r6 상태 "준비",
  로그온 모드 "대화형만", 마지막 결과 267011(한 번도 실행 안 됨), explorer.exe 0개(= 아직 아무도 로그온 안 함), MATLAB.exe 0개.
- 사용자 Windows 로그온 → 작업 시작 → MATLAB 로그인 순서로 풀린다. 코드·RESULTS·TASKS 수정 없음. 다음 절차는 반복 18 항목과 같다.

## 2026-10-29 18:31–18:34 (반복 18, 재부팅 후 첫 턴) — 멈춘 MATLAB 정리됨 / `r5r6` 재제출
- 사용자 재부팅(TASKS ★★★): `job.py status` — stair_i0 died(.done −2), R6_smoke·r5r6 failed(rc 1073807364 = 0x40010004,
  18:26:39 강제 종료; 로그 "MATLAB error Exit Status: 0x40010004"). `matlab_stall_probe.py`(18:31, 18:33): MATLAB.exe 0개,
  새 winlogon(18:29:39)만 있음. 산출물은 그대로(staircase_check.csv interp=1 195행, stair_steady_points.csv 10-28 04:08).
- 스모크 잡 `smoke1029`(disp(version))을 냈다가 TASKS의 "r5r6 하나만" 지시를 보고 시작 전에 kill(rc −9, MATLAB 미기동).
- **`r5r6` 재제출 18:32:55**(같은 명령 `matlab -batch "cd('Simulink/Thesis'); r5r6_chain"`, --interactive). job.py가 이전
  r5r6.done/.log를 지웠다. stair_i0.done·R6_smoke.done이 있으므로 `waitForJobs`는 바로 통과한다. 18:33 현재 로그 없음
  = Windows 로그온 트리거 대기(사용자 로그온 전). 코드·RESULTS 수정 없음.
- 다음: 체인이 끝나면 `python -I -X utf8 Simulink/Thesis/r8_finalize.py` 한 번 → `out/r8_verdict.md`를 RESULTS 0절·R.8에(문턱 재조정 금지).
  로그가 "r5r6_chain: start" 뒤 오래 진전이 없으면 다시 로그인 대기(matlab_stall_probe로 dcpu 확인).

## 2026-10-29 17:10–17:14 (반복 17, 마감 다음 날) — MATLAB 46시간째 정지 / 대기 턴 짧게
- MATLAB: `matlab_stall_probe.py`(17:11) — 세 MATLAB.exe 누적 CPU 301.6/62.5/58.6 s(4시간 동안 +24.4/+5.1/+5.2 s = 배경 틱),
  dcpu5s 0/0/0, ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐; staircase_check_interp0·
  stair_steady_part·dataset_fixed_part 없음). `automation/state/jobs/`에 stair_i0/R6_smoke/r5r6의 .done 없음, 로그 파일은
  10-27 19:11/19:16/19:23 그대로(log idle 2747–2759분).
- Thesis·automation 추적 파일 변화 없음(이 항목 제외). 코드·RESULTS·TASKS 수정 없음. 로그인 후 절차는 반복 13 항목과 같다.

## 2026-10-29 13:08–13:12 (반복 16, 마감 다음 날) — MATLAB 42시간째 정지 / 대기 턴 짧게
- MATLAB: `matlab_stall_probe.py`(13:09) — 세 MATLAB.exe 누적 CPU 277.2/57.4/53.4 s(4시간 동안 +22.7/+5.5/+5.5 s = 배경 틱),
  dcpu5s 0.03/0/0, ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐; staircase_check_interp0·
  stair_steady_part·dataset_fixed_part 없음; stair_i0/R6_smoke/r5r6의 .done 없음). 잡 세 개 log idle 2505–2518분.
- Thesis·automation 추적 파일 변화 없음(이 항목 제외). 코드·RESULTS·TASKS 수정 없음. 로그인 후 절차는 반복 13 항목과 같다.

## 2026-10-29 09:06–09:10 (반복 15, 마감 다음 날) — MATLAB 38시간째 정지 / 대기 턴 짧게
- MATLAB: `matlab_stall_probe.py`(09:07) — 세 MATLAB.exe 누적 CPU 254.5/51.9/47.9 s(4시간 동안 +25/+6/+5 s = 배경 틱),
  dcpu5s 0.03/0/0, ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐; staircase_check_interp0·
  stair_steady_part·dataset_fixed_part 없음; stair_i0/R6_smoke/r5r6의 .done 없음). 잡 세 개 log idle 2263–2275분.
- 막히지 않은 작업이 남았는지 다시 봤다: R5–R7은 MATLAB 대기, 미착수로 남은 T12는 10-24 범위 변경에서 중단된 항목(TASKS 123행)
  → 할 것 없음. 다음 턴이 다시 고르지 않도록 TASKS의 T12 줄에 "중단"을 적었다. 코드·RESULTS 수정 없음. 로그인 후 절차는 반복 13 항목과 같다.

## 2026-10-29 05:05–05:08 (반복 14, 마감 다음 날) — MATLAB 34시간째 정지 / 대기 턴 짧게
- MATLAB: `matlab_stall_probe.py`(05:06) — 세 MATLAB.exe 누적 CPU 229.5/46.0/42.8 s(4시간 동안 +24/+6/+5 s = 배경 틱),
  dcpu5s 0/0/0, ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐, stair_steady_part·dataset_fixed_part 없음,
  stair_steady_points.csv는 10-28 04:08 Python 산출물 그대로). 잡 세 개 log idle 2022–2034분.
- 코드·RESULTS·TASKS 수정 없음(현재 RESULTS가 최종본). 로그인 후 절차는 반복 13 항목과 같다(`r8_finalize.py` 한 번 → 판정 이식).

## 2026-10-29 01:02–01:08 (반복 13, 마감 다음 날) — MATLAB 30시간째 정지 / 대기 턴 짧게
- MATLAB: `matlab_stall_probe.py`(01:02) — 세 MATLAB.exe 누적 CPU 205.2/40.2/37.5 s, dcpu5s 0.03/0/0(배경 틱), ServiceHostWindow 21개,
  산출물 변화 없음(staircase_check.csv interp=1 195행뿐, stair_steady_part·dataset_fixed_part 없음). 잡 세 개 log idle 1779–1791분.
- 로그인 후 경로 회귀 확인: `test_r8_verdict.py` ALL PASS, `test_ipk_decompose.py` PASS, `m_static_check.py r5r6_chain.m` OK.
  r8_finalize가 부르는 스크립트들은 반복 11–12에서 바뀐 파일(simulink_sweep, export_mcu, c_compile_check)을 import하지 않는다.
  시험 후 추적 파일 변화 없음.
- RESULTS: 정지 시각 세 곳을 10-29 01:03·30시간째로 바꾸고, 맨 위 상자에 "이후의 정지 확인은 PROGRESS 맨 위에만"을 넣었다.
  한계 항목 제목을 "마감까지 없을 수 있다" → "마감(10-28)까지 나오지 않았다"로 사실에 맞췄다. 이제 대기 턴은 이 파일만 쓰면 된다.
- 러너의 잡 대기 상한은 240분(`loop_runner.CFG.job_wait_max_min`)이라 로그인이 없으면 4시간마다 턴이 열린다. 사용자가 정한 설정이라
  바꾸지 않았다.

## 2026-10-28 20:47–21:10 (반복 12, 마감일 저녁) — MATLAB 25시간째 정지 / T9 C 컴파일 검증 완료(MATLAB 동봉 LCC-win64)
- MATLAB: `matlab_stall_probe.py`(20:47, 20:58) — 세 MATLAB.exe 누적 CPU 182/35.7/32.7 s, dcpu5s 0, ServiceHostWindow 21개,
  산출물 변화 없음(staircase_check.csv interp=1 195행뿐). 로그인 대기 그대로.
- **막히지 않은 남은 한계 하나를 닫았다: "C 코드 미컴파일".** 이 PC에 gcc/clang/cl은 없지만 MATLAB에 LCC-win64가 딸려 있다
  (`C:\Program Files\MATLAB\R2025b\sys\lcc64\lcc64\bin\lcc64.exe`, lcclnk64; MATLAB을 띄우지 않고 라이선스도 쓰지 않는다).
  `c_compile_check.py`: 전달물 3파일을 임시 폴더에 복사해 (A) 그대로, (B) MAC 루프 뒤에 acc0/acc1/y 저장 한 줄만 끼운 탐침 사본으로
  빌드한다. 입출력은 float32 비트 패턴(16진)이라 십진 반올림이 끼지 않는다.
  - 첫 컴파일 결과(원래 .c): **오류 4** — `float y[2] = {SRHDC_C[0], …}`, `float y[2] = {(float)acc0 * s + …}`의 비상수 집합체
    초기화(C89 규칙; C99 이상 ARM 컴파일러는 받음), **경고 1** — 죽은 분기의 `frac << (15 - SH)`(SH = 17 → 음수 시프트). `-O`는 LCC
    최적화기가 softplus 줄에서 잘못된 SSE 어셈블리를 내는 컴파일러 버그(쓰지 않음; MATLAB의 lcc64 mex 옵션도 -O 없음).
  - `export_mcu.py` C_SOURCE 수정: 원소별 대입 2곳, `#if SRHDC_WQ_SHIFT >= 10`으로 분기 선택. `export_mcu.py --model
    Simulink/Thesis/out/model_smooth_reghdc_b3.json`로 재생성 → `SmoothRegHDC_STM32.h`(614035cf554a)·`smooth_reghdc.h`(921ea673d552)
    SHA1 불변, `smooth_reghdc.c`만 ee53da196d7c → 3930c9fd7131, export_mcu.log 수치 동일.
  - 링크: LCC libc64.lib에 `log1pf` 구현이 없다(math.h 선언만) → 하네스에 `(float)log1p((double)x)` 보충(전달물 불변).
  - **결과 ALL PASS**(`out/c_compile_check.{log,csv}`): 20,253점(무작위 2만 seed 2026 + 경계 45 + MATLAB 시험 벡터 208) 전부에서
    int64 누산값과 float32 y 비트가 `SmoothRegHDCFixed`(evaluate의 int16 행)와 같음. -A 경고 0. 플랫폼: int/long 4 B, int64 8 B,
    (int32)−3 >> 1 = −2(산술). Q15 입력 변환(C float32 lrintf vs Python float64 round) 54점 1 LSB 차 → max|ΔD| 3.9e-5, 같은 입력이면
    softplus float32/64 차 2.0e-8. float64 모델 대비 C q15 2.6e-4, C f32 5.0e-7(내보내기 로그와 같음).
  - `test_c_compile_check.py` ALL PASS: 깨끗한 빌드 통과, `>> 15 → >> 14` 변이체는 컴파일되지만 int64·y 검사가 FAIL, out/ 파일 불변.
- RESULTS: 3절 T9 단락에 "C 컴파일 검증" 문단, 0절 구조·한계, 10절 4장·6장(6장은 결론 수치를 Simulink 우선 23–24 %/0.8–3.4 %로 고치고
  R.8 (d) 85 %, 데드타임 미학습 몫, D0≈0 0.3 A, Simulink 대기 항목 추가), 미해결/한계 마지막 항목, 정지 시각 20:58·25시간째.

## 2026-10-28 16:35–17:05 (반복 11, 마감일) — MATLAB 21시간째 정지 / 체인 단계 격리, MATLAB 없는 구문 점검, 0절 평균 정정
- MATLAB: `matlab_stall_probe.py` — 세 MATLAB.exe 누적 CPU 157.5/30.8/27.8 s(4시간 동안 stair_i0만 +24 s, dcpu 0.05 = 배경 틱),
  ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐). r5r6 로그는 시작 줄뿐 = 아직 스크립트를 읽지 않음.
  러너는 마감 후에도 STOP/goal_met 전까지 돌고, 잡 하나가 끝나면 바로 턴을 연다(loop_runner.wait_for_jobs) → 로그인 시각과 무관하게
  결과가 처리된다.
- **체인 정적 재검토**: staircase_check / verify_steady_batch / verify_steady_state / gen_dataset_fixed를 다시 읽었다. D0 보간 끄기는
  실제 위상을 구동하는 블록에 걸린다(`out/plant_params.md` 0절: Phi_Manual 등은 데이터셋 시기 모델에서 미사용, 수동 모드 phi = D0
  From Workspace). 실행 오류 요인은 찾지 못했다. 다만 `r5r6_chain.m`은 1단계가 예외를 던지면(예: interp=0 설정이 또 거부돼 5회 모두
  실패 → `cell2table({})` 오류) 2단계(상수 D0 정상상태 106점 = Q2 BT_, Q3 K1_ 근거)와 3단계까지 통째로 중단되는 구조였다.
  → `runStep`(try/catch, getReport 출력)으로 단계를 격리하고, 실패한 단계가 있으면 마지막에 error로 끝내 `job.py status`에 드러나게 했다.
- **`m_static_check.py`**(새): MATLAB 없이 .m의 블록 키워드–`end` 짝과 괄호 균형을 본다(주석, 문자열/전치 구분, %{ %}, `...` 처리,
  괄호 안 `end`는 인덱스). `test_m_static_check.py`: MATLAB에서 실제로 돈 7개 파일 통과, r5r6_chain 망가뜨린 사본 4종(`end` 누락·추가,
  `)` 누락, 닫히지 않은 문자열) 모두 검출, 까다로운 구문(`case 'if'`, `x(end)`, `a'`, `[1 2\n3 4]`) 오탐 없음 → ALL PASS. 수정 후
  Thesis의 .m 10개 모두 OK.
- **RESULTS 0절 정정**: R.4 M-B 평균 "+0.22 A(+7.1 %)"는 12점 평균이라 SPS 기준 자체가 진동한 109.2 V 600 W(+1.18 A, 본문이 "크기를
  해석하지 않는다"고 한 점)를 포함했다. 인용값을 진동 점을 뺀 11점 평균 +0.134 A(+5.7 %)로 바꾸고(`out/simulink_pi_compare.log`로
  재계산 확인: (2.655 − 1.181)/11), R.4 요약표의 굵은 표시를 그 행으로 옮겼다. 맨 위에 "사용자 확인 필요"(로그인 → 자동 진행 →
  r8_finalize) 상자, 0절·R.8 (ii) 정지 시각 21시간째, (ii)에 단계 격리 한 문장.
- **그림 `out/fig_simulink_pi_compare.png` 수정**(`simulink_sweep.py fig`, 저장된 CSV에서 다시 그림 → CSV·log md5 불변): SPS 기준 실행이
  진동한 (V2, P)(= 109.2 V 600 W)의 Simulink ΔI를 비우고 세로 점선 + "SPS reference oscillates: no Simulink value"로 표시했다. 전에는
  그 점(M-B0 +1.61, M-B +1.18 A)이 109.2 V 칸의 축을 차지해 그림에서 가장 큰 "저감"으로 보였다. 속 빈 점 설명은 "제어기 실행의 진동"으로
  좁혔다. 팔레트는 그대로(evaluate.py BLUE/ORANGE/AQUA; node가 없어 dataviz 검증 스크립트는 못 돌림).

## 2026-10-28 12:20–12:50 (반복 10, 마감일) — MATLAB 17시간째 정지 / 로그인 직후 체인 안전장치, 판정 규칙 사전 고정
- MATLAB: `matlab_stall_probe.py` — stair_i0/R6_smoke/r5r6의 MATLAB.exe 세 개 모두 5초 CPU 증가 ≈ 0(누적 133/25.5/23 s),
  ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐, stair_steady_part·dataset_fixed_part 없음).
  PushNotification으로 사용자에게 로그인 요청을 보내려 했으나 "terminal active"라 전송되지 않았다(루프에서는 호출 수단이 없다).
- **로그인 직후의 충돌 방지**(`r5r6_chain.m`): 멈춘 세 MATLAB은 같은 로그인에서 함께 풀린다. stair_i0는 직렬 실행 중이었다
  (로그 cp949 해독: 풀이 시작 직후 종료됨). 그 출력 `staircase_check_interp0.csv`는 r5r6 1단계와 같은 파일이다. R6_smoke는 자체
  6워커 풀을 연다. 그대로 두면 6워커 풀 2개와 클라이언트 3개가 6코어·31 GB를 나눠 쓰고 1단계를 중복 계산한다. → `waitForJobs`:
  두 잡의 `automation/state/jobs/<이름>.done`(cmd가 MATLAB 종료 시 성공·실패와 관계없이 씀)을 최대 40분 기다린 뒤 풀을 연다.
  r5r6 MATLAB은 시작 단계에서 멈춰 있어(로그에 첫 출력 없음) 아직 스크립트를 읽지 않았으므로 수정이 반영된다. MATLAB이 없어
  실행 시험은 못 했고 정적으로만 확인했다. 함께 정적 점검한 것: verify_steady_state.m은 106점 파일(label/V2/D0/D1/D2/step)을 그대로
  읽는다. R6_smoke 접두어 `dataset_fixed_smoke_part_*`는 R6 재개 dir 패턴과 분석 glob(`dataset_fixed_part_*`)에 걸리지 않는다.
- **판정 규칙 사전 고정** `r8_verdict.py` → `out/r8_verdict.md`(한국어). Simulink 결과가 하나도 없는 12:30에 정했다.
  Q1(R5 b/c): 데이터셋 실행에서 같은 전력 SPS − TPS ≥ 0.5 A인 칸들에서 r = 평균(진짜 계단)/평균(데이터셋). r ≥ 0.75면 사용자
  쪽(M-B 결론 수정), r ≤ 0.5면 보간 산물, 그 사이는 일부. SPS 기준선(진짜 계단 SPS 행 − R.4 정상상태)과 같은 라벨 |ΔIpk|도 적는다.
  Q2(BT_): D − B_st 최대 ≤ 0.05 A면 같음, ≤ 0.15 A면 작은 차, 그 이상이면 실재. Q3(K1_): 정상상태 SPS − TPS 최대(100–300 W)에
  같은 문턱을 쓰고, 30/20 ms 정착 점검을 붙인다. Q4(R6/R7): 수정/원(같은 격자)의 r에 Q1 규칙, 수정 − M-B 평균 ≤ 0.1 A면 같은
  수준이다. 끝에 0절용 문장을 조립하고, 결론 수정이 필요한 판정이 있으면 경고 줄을 넣는다. 입력이 없으면 "아직 없음"이다.
  `test_r8_verdict.py`: 임시 디렉터리의 가짜 입력 5경우(none / artefact 30 % / partial 60 % / user 90 % / mb_match)에서 모든 갈래의
  판정 코드가 맞고 보고서에 nan이 없다(ALL PASS). `r8_finalize.py` 5단계로 넣었다. 현재 실제 `out/r8_verdict.md`는 네 질문 모두
  "아직 없음"이다. `test_ipk_decompose.py` PASS(회귀 없음).
- RESULTS: 0절 정지 시각 갱신(17시간째, "마감까지 없다" → "아직 없다"), "마감 상태"에 사전 규칙 요약, R.8 (ii)에 대기와 판정 단계,
  한계에 한 줄.

## 2026-10-28 08:13–08:35 (반복 9, 마감일) — MATLAB 13시간째 정지 / 분해 그림 추가, RESULTS 마감본 표시
- MATLAB: `matlab_stall_probe.py` — stair_i0/R6_smoke/r5r6의 MATLAB.exe 세 개 모두 5초 CPU 증가 0(누적 110/21/18 s),
  ServiceHostWindow 21개, 산출물 변화 없음(staircase_check.csv interp=1 195행뿐, stair_steady_part 없음).
- **그림 `out/fig_ipk_decompose.png`**(`ipk_decompose.py`의 `fig()`, main 끝에서 호출 → r8_finalize가 다시 그림): k≠1 깨끗한 7점.
  (a) 같은 측정 전력의 Ipk — 주황 원 = SPS(속 빈 A 데이터셋 행 / 찬 C 정상상태), 파랑 사각 = TPS(속 빈 B 데이터셋 최저 /
  찬 D M-B 정상상태). SPS는 크게 벌어지고(최대 109.2 V 100 W +2.33 A) TPS는 거의 겹친다. (b) A−B, A−C, C−D 막대 + 평균
  (+1.35 / +1.15 / +0.21 A), 제목에 평균 D−B −0.014 A. 수치는 반복 8의 log와 같다. `test_ipk_decompose.py` PASS,
  `r8_finalize.py` 정상(Simulink 입력이 없는 두 단계는 건너뜀).
- RESULTS: 0절의 깨진 줄바꿈 고침, 그림 경로를 0절·R.8 (d)·10절 3장에 넣음, R.8 (d)에 그림 읽는 법 두 문장, 0절에 "마감 상태"
  항목(확정/예비 구분, 로그인 후 절차, 바뀔 수 있는 칸 = 109.2 V 100 W D−B와 R7 진짜 계단 열).

## 2026-10-28 04:04–04:50 (반복 8, 마감일) — MATLAB 9시간째 정지 / Simulink만의 분해로 사용자 질문에 답 / 로그인 후 한 번에 마무리할 준비
- MATLAB: `matlab_stall_probe.py` — stair_i0/R6_smoke/r5r6의 MATLAB.exe 세 개 모두 5초 CPU 증가 0(누적 87/16/14 s),
  ServiceHostWindow 21개, 잡 로그 변화 없음(r5r6는 시작 줄뿐 = r5r6_chain 아직 호출 전). 산출물 변화 없음.
- **`ipk_decompose.py` → `out/ipk_decompose.{log,csv}`(Simulink 수치만)**: 같은 측정 전력(HV→LV, 첫 상향 교차)에서 A = 데이터셋
  SPS 행, B = 데이터셋 20×20 최저 TPS 행(`dataset_fixed_compare_py_zoh.csv`의 orig_full), C/D = R.4 SPS/M-B 정상상태.
  k≠1 7점(진동 점 109.2 V 600 W 제외) 평균 A−B +1.348 = A−C +1.151(85 %) + C−D +0.211 + D−B −0.014 A. 즉 M-B의 Simulink 정상상태
  피크 = 데이터셋 최저 TPS 행(평균). D−B 최대 109.2 V 100 W +0.287, 84 V 300 W +0.095. k≈1(94.61 vs 95 V)은 따로 표시.
  10-27 대화형 세션의 분해(1.21 = 1.15 + 0.21 − 0.15)와 같은 결론을 스크립트로 재현.
- **BT_ 정상상태 23점**(`add_best_tps_steady_points.py`): D−B > 0 세 점의 데이터셋 최저 조합 (0.247, 0.200)@109.2 V, (0.374, 0.389)@84 V,
  (0.216, 0.232)@84 V를 D0 라벨 −4.87°…(데이터셋이 1.5×목표에 닿은 라벨 + 1)로 `out/stair_steady_points.csv`에 추가(83 → 106점;
  R6 격자에는 이 조합 없음). Python 플랜트로 감쌈 확인(84 V 두 조합은 D0 = 0에서 이미 −124/−220 W라 음의 라벨 2개가 필요했음).
  Python 예측 정상상태 B = 1.499 / 2.142 / 1.310 A, 같은 플랜트 M-B 1.609 / 2.159 / 1.259 → 같은 플랜트 차 +0.11 / +0.02 / −0.05 A.
  `test_ipk_decompose.py`(임시 디렉터리의 가짜 part로 Simulink 경로 시험) PASS. out/에 가짜 part를 만들지 않았다.
- **`r8_finalize.py`**: 로그인 후 한 번 실행하면 staircase_compare → ipk_decompose --py → k1_simulink_compare → dataset_fixed_analyze를
  입력이 있는 것만 돌려 `out/r8_finalize.log`에 모음(현재는 ipk_decompose만 실행됨, 시험 완료).
- RESULTS: 0절 R.8 항목을 Simulink만의 분해 중심으로 다시 씀, R.8 (d) 표·답 단락 추가, (ii) 대기 단락에 106점·r8_finalize, 10절 3장,
  한계에 "MATLAB 정지로 R.8 Simulink 확인 일부가 마감까지 없을 수 있음" 항목.

## 2026-10-27 23:35 – 10-28 00:15 (반복 7) — MATLAB 여전히 정지 / R7 예비의 두 의문점을 Python으로 점검
- MATLAB: `matlab_stall_probe.py`(ctypes, 읽기 전용) — stair_i0/R6_smoke/r5r6의 MATLAB.exe 세 개 모두 5초 동안 CPU 증가 0,
  누적 61.8/10.5/9.2 s, MathWorksServiceHostWindow 21개. 4시간째 로그인 대기. 산출물 변화 없음(staircase_check.csv는 interp=1 195행뿐).
- **(1) k≈1 100 W의 +0.46 A(R7 예비, Python 계단 재생)는 정상상태 이득이 아니다.** `k1_history_probe.py`: 94.61 V SPS 계단 재생 행이
  D0 0°(14 W 0.755 A)와 2.40°(145 W 0.955 A)에서 정상상태(159 W 0.444 A, 157 W 0.439 A)와 다르다. 2.40°는 느린 과도(3 ms에 0.439 A로
  수렴), 0°는 공존 상태(이력을 따라오면 0.75 A가 21 ms 유지, 0에서 출발하면 0.444 A). `k1_light_probe.py`(정상상태, D0 0.05°):
  격자 최저 TPS 이득 100 W +0.02–0.07, 200 W +0.02–0.09 A, 150 W·300 W 이상은 SPS 최선. td 0이면 모든 전력에서 SPS 최선(이상 모델과 같음).
  1250스텝 버전은 10분 넘게 걸려 중단(k1_probe, rc −9)하고 125스텝으로 다시 돌렸다(k1_probe2).
- **(2) 같은 플랜트·같은 정상상태 측정에서 M-B 대 격자 최저 TPS**(`mb_vs_grid_probe.py`, R6 격자 11×11, 3 V2 × 6 전력):
  격자 없는 플랜트 M-B +0.231 vs 최저 +0.246 A(94 %, 차 평균 0.014 / 최대 0.088 A @109.2 V 100 W), 2.88° 격자 플랜트 +0.240 vs +0.280
  (차 최대 0.167 A @109.2 V 100–150 W). R7 예비표의 84 V 차이(+0.63 vs +0.41)는 Python 대 Simulink 플랜트 차이(SPS 1.88 vs 1.73 A)다.
  109.2 V 경부하에서 남는 0.09–0.16 A는 데드타임을 모르는 이상 모델로 학습한 몫이다(최적이 D1 큰 쪽 (0.263, 0.200)으로 이동).
- Simulink 준비: `add_k1_steady_points.py`로 `out/stair_steady_points.csv`에 94.61 V K1_ 18점(SPS, (0.453,0.453), (0.326,0.326) ×
  D0 −2.40…10.03°)을 더했다(r5r6 2단계가 아직 읽지 않았으므로 반영됨, 83점 = 36개 배치 3개). 결과 비교 `k1_simulink_compare.py`
  (대용 데이터로 시험 완료).
- RESULTS: R.8에 "R7 예비의 점검" (1)(2)와 표, 0절 R.8 항목에 두 문장, 한계에 두 항목(D0≈0 공존 상태 0.3 A, 데드타임 미학습 몫).

## 2026-10-27 19:07–19:45 (반복 6) — R5(a) 확정: 데이터셋 = 생성 실험 + D0 선형보간(비트 단위 재현) / MATLAB 로그인 대기로 R5(b)·R6 보류
- 시작 상태: 대화형 세션이 같은 `staircase_check`를 두 번 제출했다(`staircase_check` 19:00, `stair_check` 19:01; 같은 결과를 같은
  파일에 씀). 둘 다 정상 종료.
- **R5(a)**: interp=1 195행(5조합 × 39계단)이 체크포인트의 같은 (V2, D1, D2, D0 라벨) 행과 **비트 단위로 같다**(max|ΔIpk| =
  |ΔP_LV| = |ΔP_HV| = 0). → 데이터셋 = 이 모델(35d5a78) + 이 스크립트 + D0 From Workspace 블록 Interpolate on(기본값) 확정.
- interp=0 5회는 모두 실패했다("보간이 활성화되지 않았기 때문에 최종 작업 공간 데이터 값 이후에 출력값을 외삽할 수 없습니다").
  → OutputAfterFinalValue='Holding final value'가 필요하다. `staircase_check.m`을 수정했다(3번째 인수 interps, 블록 파라미터 2개).
- **같은 측정 전력 SPS − TPS (Simulink interp=1 = 데이터셋)**, `staircase_compare.py` → `out/staircase_compare.log`:
  109.2 V HV→LV 100/150/300 W +2.64/+2.79/+1.65 A, 84 V (0.263, 0.311) 100/150 W +0.72/+0.73, 84 V (0.468, 0.484) 1 kW +1.70,
  LV→HV 84 V 100 W +1.79, 109.2 V 100 W +1.34. 같은 전력 SPS 정상상태(R.4) 109.2 V 100 W 2.30 A, 84 V 1 kW 4.38 A. 데이터셋 SPS는
  각각 4.63 A(100 W)와 6.62 A다.
- **MATLAB 정지(차단 요인)**: `stair_i0`(19:10)는 parpool이 곧바로 닫혀 직렬 실행으로 넘어갔고, 19:11:14 "시뮬레이션 실행 중"에서
  멈췄다(CPU 누적 37 s, 이후 0 %). 19:11:20에 MathWorksServiceHost 새 인스턴스가 떴다(부모 프로세스는 이미 종료, CrashReporter·Monitor
  동반). 그 뒤 `R6_smoke`(19:16)와 `r5r6`(19:23)도 MATLAB 시작 단계에서 CPU 0 %로 멈췄다(누적 5 s, 첫 출력 없음). MATLAB마다
  MathWorksServiceHostWindow 프로세스 7개가 붙어 있다 → 온라인 라이선싱 로그인 창 대기로 추정한다(Session 0에서는 창을 볼 수 없음).
  mathworks 호스트 443 연결은 정상이다. 프로세스를 끝낼 권한이 없어(taskkill 승인 필요) 그대로 뒀다. 진단은 Python ctypes(프로세스
  스냅샷 + GetProcessTimes)로 했다.
- 대응: `r5r6_chain.m`(잡 `r5r6`)이 한 세션에서 순서대로 실행한다. (1) interp=0 5회, (2) 같은 D0 라벨 65점의 정상상태(30 ms,
  verify_steady_batch 36개씩, `out/stair_steady_points.csv`), (3) `gen_dataset_fixed` 363조합(SPS → 6×6 거친 격자 → 나머지 순서,
  배치 18, 이어하기). 각 단계는 출력이 있으면 건너뛴다. 로그인되면 자동으로 진행되고, 예상 시간은 약 0.5 h + 4.5 h다.
- 새 스크립트: `gen_dataset_fixed.m`(R6; MATLAB 미검증 — 스모크가 멈춤), `staircase_compare.py`(a/b/c 표 + 그림
  `fig_staircase_compare.png`), `dataset_fixed_analyze.py`(R7; 원 데이터셋으로 시험: 전 격자 최저 TPS, 109.2 V HV→LV 150 W
  +3.43 A, 84 V 1 kW +2.04 A), `staircase_replay.py`, `replay_grid.py`.
- **보조(Python 플랜트, 판정 근거 아님)**: `staircase_replay.py` — linear 재생 대 Simulink interp=1 Ipk RMSE 0.31–0.88 A,
  zoh 대 플랜트 정상상태 평균 ±0.03 A(|D0| ≤ 32°). 같은 전력 SPS − TPS(linear → zoh → 정상상태): 109.2 V HV→LV 100 W
  +1.70 → +0.73 → +0.75, 150 W +1.70 → +0.51 → +0.59; 84 V 100 W +1.24 → +0.53 → +0.53; 1 kW +0.88 → +0.08 → +0.09.
- **R7 예비(Python)**: `replay_grid.py`로 R6 격자 363조합을 zoh/linear로 재생했고(각 64 s), `dataset_fixed_analyze.py`로 분석했다.
  HV→LV 8점 평균 ΔI(격자 최저 TPS, 같은 전력): 원 데이터셋 84 V +0.82 A(+25 %) / 109.2 V +1.95 A(+40 %) / 94.61 V +0.31 A,
  진짜 계단(zoh) +0.23 / +0.46 / +0.18 A, linear 재생 +0.68 / +1.20 / +0.23 A. zoh의 SPS Ipk는 R.4 Simulink 정상상태 SPS와 0.17 A
  안에서 같다(진동점 제외). 남은 질문(R6 Simulink로 판정)은 둘이다. 경부하 격자 최저 TPS가 M-B보다 더 줄이는지(84 V 100 W +0.63 vs
  +0.41, 109.2 V +0.85 vs +0.53), 그리고 k ≈ 1 100 W에서 +0.46 A가 나오는지(M-B는 SPS 유지).
- RESULTS: R.8절을 새로 쓰고((a) 확정, (c) 데이터셋 방식 표, Python 보조, R7 예비 표, Simulink (ii)·R6 대기), 0절 요약에 한 항목을
  넣었다.

## 2026-10-27 18:45–19:05 (대화형 세션) — 사용자 데이터셋 재확인, 생성 방식 재현 실험 착수, 루프 재개
- 사용자: "Simulink 기존 파일로 돌린 게 맞나, 저감이 너무 낮다", "내 데이터셋(TPS_Checkpoint_done_20260316_021229)을 확인하라",
  "데이터셋도 정상상태까지 돌렸다(고정 외부 위상)".
- 모델 확인: R.4/T6의 `dataset_era/Control_Logic_dataset_era.slx` = git 35d5a78 `Control_Logic.slx`와 SHA1 동일(7dc93ab…).
  HEAD 모델과의 회로 파라미터 차이는 커패시터 3개(Capacitor2 47→470 µF, Capacitor3/4 1000→100 µF)뿐.
- `inspect_checkpoint.m`(잡 ckpt_inspect): 312,000행 전부 valid, 8000 시뮬 오류 0, CSV와 7열 행 단위 동일.
- 데이터셋만으로 본 저감(첫 분기, 같은 측정 전력): 84 V 150/300/600/1000 W +1.42/+0.76/+0.84/+1.15 A, 109.2 V +3.42/+0.66/+0.89/+0.50 A.
  HV→LV 정상 7점 분해: 데이터셋 저감 1.21 A = 데이터셋 SPS 과대 1.15 A + 정상상태 저감 0.21 A − 0.15 A.
  예: 84 V 1 kW 데이터셋 SPS 6.62 A vs 정상상태 4.38 A, 데이터셋 최저 TPS 4.40 A ≈ 정상상태 SPS.
- 모델 XML: D0 From Workspace(`From Workspace11`) Interpolate 기본값(on). → R5 `staircase_check.m`(잡 실행 중) 작성.
- 그림 `fig_surface3d.py` → `out/fig_surface3d.png`(M-B vs 기존 목표 D1/D2 3D, 논문 그림 2/3 형식).
  M-B: V2 84 V |P| 0/100/300/600/1000 W에서 D1 0.100/0.212/0.358/0.442/0.451, D2 0.113/0.244/0.407/0.500/0.500;
  95 V는 100 W부터 SPS; 109.2 V D1 0.127/0.199/0.350/0.487/0.500, D2 0.100/0.167/0.296/0.421/0.443.
- 사용자 지시: 루프 러너로 자동화해 원격이 꺼져도 계속 → TASKS.md "★★ 2026-10-27" R5–R7 추가, STOP 제거 후 러너 재시작.

## 2026-10-26 13:54–14:50 (반복 5, 사용 한도로 끊긴 10-24 턴의 이어서) — 범위 변경 R1 완료 / 플랜트 해법 결함·다중 정상상태 / Simulink PI 판정 착수
- **R1** `train_targets.py`: M-A = 사용자 데이터셋(`Physics_Dataset_Final_Regression_Ready.csv`)을 (a)와 같은 부호화·α로 ridge
  (방향별; ±P 라벨이 최대 0.09/0.14 다름). 해석 1만 점(`evaluate.py`, 평가표에 M-A 곡면·Smooth·Legacy 행 추가): M-A(P<0)
  −0.0096 A, 5 mA 초과 열화 27.3 %, 최악 −828 mA(V2 ≈ 95 V 경부하에서 D ≈ 0.15–0.2의 깊은 TPS); M-B(b3) +0.0815 A, 0 %, −2.8 mA.
  그림 `fig_targets_maps.png`, `fig_targets_slices.png`.
- **플랜트 해법 결함(10-24 턴 끝에서 발견, 이번에 수정)**: `t7_outliers.py`/`plant_conv_probe.py` — 경부하·k≈1에서 8회 Newton이
  수렴하지 않아(잔차 최대 0.4 A) Ipk 오차 최대 0.4 A. 수정(`plant_full.py`): 감쇠 Newton(`_polish`), 주기 사상 스펙트럼 반경 검사,
  1주기 해가 없거나 불안정하면 시간적분(600주기 + 80주기 창, `_relax`), `operating_point` = 따뜻한 시작 D0 스캔으로 첫 상향
  교차점 → 구간 이분법(옛 동작은 `legacy=True`). `plant_period_probe.py`: 미수렴 점의 일부는 1주기 해가 있는데 Newton이 놓친 것,
  일부는 지속 저주파 진동. **`plant_attractor_probe.py`: 안정한 1주기 정상상태가 둘 이상 공존**(Ipk ~17–20 mA, P ~3 W 차이, 이력이
  결정) → T7의 ±20 mA는 해법 잡음만이 아니라 실제 불확정성. test_plant_full 항목 5(정상성)·6(첫 교차점) 추가(잡 test_plant_full4).
  → 10-24 verdictB·pv_*·t7_select 수치는 무효(RESULTS 8절 (4)·9절에 표시), T7 선택(b10)은 채택 안 함.
- **Simulink 대 Python 플랜트**(`plant_vs_simulink.py`, t6 52점 상수 D0): 플랜트(양자화 포함) Ipk RMSE 0.39 A(ICROS 경부하 TPS 0.76 A),
  P_LV RMSE 110 W(편향 −70 W); 해석 모델 0.41 A / 144 W. 경부하에서는 플랜트도 Simulink 전력을 잘 못 맞춤 → Simulink 자체로 판정.
- **R2(3) Simulink PI**: 기존 제어기의 "외부 D1/D2 + 위상 PI" 모드(Ctrl_Mode 3, Mode_Select≠1)를 찾음. 데이터셋 시기 모델은
  Mode_Select/Kp/Ki가 Constant 블록(1, 20, 490000; `slx_trace_inputs.py`) → `setBlockParameter`로 시뮬레이션별 변경
  (`verify_pi_closed_loop.m`, 모델 미저장). 파일럿 4회: 모델 이득은 채터링(Ipk 16–18 A), 낮춘 이득들도 SPS Ipk +1.2–3 A·평균 전류
  0.1–0.5 A로 이득과 무관 → 원인은 100 ns(2.88°) 위상 격자에서의 PI 디더(시뮬레이션 제어기 산물, HRTIM엔 없음). → 디더 없는 PI
  정상상태 = 목표 전력을 감싸는 두 D0 계단의 상수 D0 정상상태 보간(`simulink_sweep.py make/next/compare`). 1차 92회 잡 `sw1b`.
- **R3 근거 재현 스크립트**(사용자 10-26 세션 수치를 파일로): `dataset_vs_simulink.py`(데이터셋 SPS − Simulink 정상상태, 같은 전력:
  84 V +0.10…+1.45 / −0.15…+2.34 A, 109.2 V −0.02…+2.54 / −0.36…+1.79 A), `tps_limit.py`(상한 V1|1−k|/(4fsL) 1.53/0/1.97 A,
  84 V 100 W −29 % … 1 kW −2.0 %, 109.2 V −39 % … −2.9 %, 95 V 0; 무제약 전수탐색 ≈ PI 첫 교차 최적).
  RESULTS 0절 요약·R.1–R.7·10절·한계 작성.
- 버그·인프라: verify_steady_state.m/verify_pi_closed_loop.m가 cd 후 상대 경로로 writetable → 절대 경로화. **job.py kill은 대화형
  MATLAB 프로세스를 못 끝냄**(schtasks /End만) → 취소한 sw1의 MATLAB+워커 6개가 남아 같은 92회를 계산 중(taskkill은 승인 필요라
  못 함; `dataset_era/out/`을 만들어 그 결과도 쓰이게 함). CPU를 나눠 써서 sw1b 약 1시간 예상.
- (15:00–16:40) **Simulink R2(3) 완료**. sw1b는 parsim 결과 역직렬화 중 메모리 부족(결과 1개 94 MB × 92)으로 순차 실행으로 떨어졌고
  고아 sw1은 끝까지 감. 둘 다 멈출 권한이 없어 `dataset_era/DAB_Controller_System.m`(git 제외 복사본) setupImpl에 잠시
  `error(...)`를 넣어 새 시뮬레이션만 실패시킴 → 고아는 88/92 정상 결과를 쓰고 종료, sw1b 종료 → 원본 복원(cmp 확인).
  이후 `verify_steady_batch.m`(24회씩 묶음)으로 2–4차. **위상 카운터 knife edge**: 엣지 위상이 정확히 2.88° 격자점이면 부동소수점
  누적 오차로 펄스가 62/63샘플로 흔들려 저주파 진동·Ipk 증가(95 V 600 W SPS 1.659 vs D2 0.49984 1.256 A) → 모든 듀티 ≤ 0.4999999
  (`D_SAFE`)로 재실행, D = 0.5 정확값 실행 제외. 결과(RESULTS R.4): M-A 6/12점 열화(95 V 4점 모두, 최악 −0.493 A = −50 %),
  M-B0 0/12, M-B 1/12(84 V 300 W −11 mA, 계단 보간 불확도 안), M-B 평균 +0.221 A / +7.1 %, k≠1 경부하 +0.41…+0.53 A(23–24 %),
  95 V는 SPS와 같음. 109.2 V 600 W는 SPS 기준 자체가 진동(5.15 A)해 해석 제외.
- 플랜트(수정 해법, variant B, 1200점): M-B0 +0.0789 A(이상 +0.0764), 1.25 % 열화, 최악 −45 mA, 전부 V2 92–100 V·|P| ≤ 401 W.
- (17:08) 플랜트 M-A −0.0178 A / 27.7 % 열화(최악 −885 mA), M-B(b3) +0.0788 A / 1.0 % 열화 / 최악 −19.5 mA / 20 mA 초과 0 %
  → RESULTS R.3 표·0절 결론("세 잣대가 같은 순서: 목표가 결과를 가른다"). (17:10) MATLAB `test_smooth_reghdc_infer.m` PASS
  (208 벡터, 4.4e-16) → T8 MATLAB 미검증 해소. RESULTS 8절 경위·한계 갱신(MATLAB 사용 가능, 잡 운영 주의 2가지).
- 상태: R1–R4 완료, T6/T8/T9/T10 완료로 정리, T7 중단·종결. 남은 선택 과제: LV→HV Simulink, 109.2 V 600 W SPS 진동 원인,
  C 컴파일(컴파일러 필요), 주기 동기 위상 갱신을 넣은 진짜 PI 폐루프.

## 2026-10-26 13:50–14:10 (대화형 세션) — 범위 축소, Simulink 대 Simulink 판정, 루프 재개
- **범위 축소(사용자)**: 분류기 제거 + (D1, D2) E2E 회귀, D0는 PI(연산 병목 제거가 프로젝트 가정), 학습 목표는 "둘 다 비교"
  → TASKS.md 맨 앞 "★ 범위 변경" 절(R1–R4)이 최우선. T7 나머지·T12 중단.
- **t6_verify 완료(10-24 19:40, 52점, 0 오류, 1537 s)** → `out/simulink_steady.csv`. 20→30 ms Ipk 변화 ≤ 0.14 A(정착).
  SPS 18점: Simulink − 해석 Ipk 평균 +0.064 A, RMSE 0.308 A. −1200 W, V2 90/100 V(ICROS 곡면 = SPS): Simulink 4.10/4.32 A
  (데이터셋 SPS 6.68/6.23 A). 경부하 ICROS 점(V2 110 V)은 **고정 D0라 Simulink 전력이 목표와 다름**(데드타임 이동, 예 −150 W 목표
  → −16 W) → 같은 전력 비교는 R2의 PI 정상상태로 해야 함.
- **Simulink 대 Simulink(해석 모델·D0 라벨 미사용)**: 데이터셋 SPS 행 vs t6 Simulink 정상상태 SPS를 **측정 전력이 같은 지점**에서
  보간 비교. 데이터셋 − 정상상태: 84 V P>0 +0.04…+1.40 A, 84 V P<0 −0.16…+2.34 A(1354 W 8.12 vs 5.78 A),
  109.2 V P>0 −0.04…+2.47 A, 109.2 V P<0 −0.34…+1.79 A. → 같은 Simulink 플랜트(기생성분 포함)에서도 데이터셋 Ipk는 정상상태가
  아님. **사용자의 반론("해석 모델은 기생성분을 못 담는다")에 대한 핵심 근거이므로 RESULTS 맨 앞 요약과 8절에 표로 넣을 것.**
  한계로 명기: Simulink 모델에도 Coss·자화 인덕턴스 없음.
- 같은 세션의 무제약 전수탐색(D 0.01–0.5, D0 전 분기, 전력 ±0.5 %): 84 V 100/200/600/1000 W 저감 29/13/2.8/1.9 %,
  109.2 V 39/22/4.1/2.8 %, 95 V ≈ 0. 상한 = 전압 불일치 순환 성분 V1·|1−k|/(4·fs·L)(84 V 1.53 A, 109.2 V 1.97 A).
- 인프라: register_watchdog.ps1의 PowerShell 배열 버그(쉼표가 +보다 먼저 묶임)로 `_detached.cmd`가 깨져 10-24 19:44 재시작이
  멈췄음 → 수정·재생성. job.py: submit 전 _refresh(오래된 running 기록이 재제출을 막던 문제), status가 모든 잡 갱신,
  대화형 잡은 wscript로 창 없이 실행(10-24 18:35 t6_verify가 콘솔 창 닫힘으로 ^C 종료된 것으로 추정).

## 2026-10-24 18:45 (대화형 세션, 인프라만) — 감시 작업 / 러너 견고화 / 재시작 예약
- 사용자: 중간에 컴퓨터가 종료됨(부팅 16:59, AtStartup으로 러너 17:01 자동 재개 확인).
- 3일 정지(10-21 17:54–10-24 17:01)의 재시작 실패 원인: `_detached.cmd`가 마지막 `echo`의 0을 반환 → 작업 스케줄러가 성공으로 보고
  restart-on-failure 미작동. 수정: 래퍼가 러너 종료코드를 반환(`register_watchdog.ps1`이 재작성, `start_loop.ps1`도 동일).
- **감시 작업 `DABThesisLoopWatchdog`**(S4U, 10분마다 + 부팅 5분 후): 러너가 없고 STOP이 없으면 DABThesisLoop 시작.
  STOP+RESTART가 함께 있으면 둘 다 지우고 재시작(옛 러너는 STOP만 알아서). 로그 `automation/state/watchdog.log`.
- loop_runner.py: 30초 하트비트 스레드(턴 중에도 갱신), 반복 단위 예외 방어(한 예외로 러너가 죽지 않음), goal_met 시 STOP 기록
  (감시 작업이 끝난 캠페인을 다시 띄우지 않게), RESTART 파일(잡이 없을 때 종료 → 감시 작업이 새 코드로 재시작),
  boost_effort 기본 "max"(주간이 균등 일정보다 3 % 이상 뒤처지고 5h < 50 %일 때만; 거부되면 자동 제거).
- job.py: 부팅 이전에 시작했거나 감독 프로세스(pid)가 사라진 잡은 `died`(.done = −2)로 표시 → 헛대기 방지.
- 18:43 STOP+RESTART 기록: 지금 러너(pid 8652, 옛 코드)는 턴 5 후 종료, 감시 작업이 10분 안에 새 코드로 재시작한다.
  (주간 사용량 2 % vs 일정 29 % — 3일 정지분만큼 뒤처짐.)

## 2026-10-24 17:52–18:30 (반복 4) — T6 대체 판정 확정 / 데드타임 원인 규명 / T7 견고 학습 착수 / 3.392 A 재검증
- **잡 결과(RESULTS 8절 확정)**:
  - validate90(8000 정상 행, 11 변형, 651 s): 이상 bias −0.133 / RMSE 0.250 A; 최적 플랜트 Ron40 무droop +0.008 / 0.177;
    Ron52 droop +0.051 / 0.177; 데드타임 0 +0.044 / 0.184; noclamp +0.152 / 0.251. 행이 모두 고전력(552–2720 W)이라 경부하 미검증.
  - replay2000(7.8만 행): 선형보간 bias +0.060 / RMSE 0.525 A / 전력 지문 0.9999(데이터셋 0.9994), ZOH −0.811 / 1.401 / 0.952
    → 라벨 오류 확정.
  - verdictA(354 s): k≈1(V2 93.6/95/96.4) 플랜트 최대 TPS 이득 0–16 mA, 이상 0–7 mA → **k≈1 이득 없음** 판정. 가장자리
    (84/109.2 V)는 플랜트와 이상이 ±0.07 A 안에서 같음.
  - verdictB(757 s, 1200점): b3 플랜트 평균 0.0793 A(이상 0.0771), **5 mA 초과 열화 1.25 %, 최악 −23.8 mA**(V2 90.4–99.8 V,
    |P| 37–427 W); LUT L49s3 −135 mA / 1.75 %, L49 −137 mA / 1.67 %.
- **원인(t7_diag.py, t7_probe.py, 5분 47초)**: 열화 점은 0.5 − min D = 1e-4–0.05(거의 SPS). 플랜트에서 SPS를 Δ 5e-4–0.06
  벗어나면 Ipk가 비단조로 ±20 mA 뜀; td = 0이면 이상과 1 mA 이내 → 데드타임(스위칭 전류 ≈ 0에서 데드타임 전압 극성 반전, PI D0
  분기 이동). 부호가 Ron/droop에 따라 반대(37 W/94.4 V: Ron52 droop +13…+24 mA, Ron0 무droop −15 mA) → 보정할 수 없는 충실도 한계.
- **T7 설계**: 잔차 보정 대신 SPS 이탈 비용 m(1 − exp(−(Δ1+Δ2)/δ0))를 E2E 손실·힌지·검증 점수에 추가(`train_e2e.departure`,
  `--margin/--d0`, LUT `--init-from`). 기본 0 = 기존 동작. test_train_e2e ALL PASS(이탈 비용 포함 dL/dM 상대오차 1.2e-4).
  학습(b3에서 1200 step, 각 약 8.5분, 4개 동시): b8 m25 → 검증 0.0791 A(b3 0.0857), 이탈 2064 중 25 mA 미만 210;
  b9 m40 0.0755 / 132; b10 δ0 0.01 0.0799 / 268; L49m 0.0841 / 484(LUT는 거의 불변 — 쌍선형 칸에서 급전이 불가). 모두 best step = 마지막
  → 미수렴. e2e_b11(b8 이어 2000 step) 실행 중.
- **플랜트 견고성 평가 잡 6개**: `plant_verdict.py --part B --variant {A,B,C} --region {all,k1} --models SPS,b3,b8,b9,b10,L49s3,L49m
  --tag _X_Y` (A Ron52 droop, B Ron40 무droop, C Ron0 무droop; all 1200점 seed 606 = verdictB와 같은 점, k1 600점 V2 88–102,
  |P| 25–500). 스모크(C, k1, 6점)에서 b3 최악 −60 mA → 변형 C가 가장 가혹.
- **3.392 A 재검증(T10, RESULTS 4절)**: 하위 에이전트 조사로 저장된 출처 없음, RegHDCTrainer.m:375–412 콘솔 출력(데이터셋 I_SPS −
  대리모델) 추정, 원 trainer 덮어써짐. `icros_max_check.py`: P 150 W / V2 110 V에서 필요한 모델 전류 1.344 A < 정상상태 전수 최적
  1.686 A(SPS 2.429) → **도달 불가**. 최대 가능 저감 1.2–1.6 A(경부하 가장자리).
- evaluate.py `--final TAG`(플랜트 기준 최종 b* 지정; 기본은 기존 vscore). RESULTS 9절(T7 진행 중), 10절(장 구성 초안) 추가.
- 다음: pv_* 결과 → 최종 모델 선택(3 변형 × 2 영역 모두 5 mA 초과 열화 0이 목표, 그중 평균 최대). 부족하면 b11/m 상향 또는
  초기값 개선. 이후 `evaluate.py --final`, `export_mcu.py --model`, `export_matlab.py --model` 재생성, RESULTS 9절·요약·ICROS 대비 표.

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
