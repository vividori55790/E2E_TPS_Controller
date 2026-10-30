# 격자 세분 점검: 100 ns 격자의 0.1 A 차가 25 ns(10 ns)에서도 남는가? (자동 생성)

생성 2026-10-30 15:08, `fine_grid_check.py compare`. 규칙은 이 파일 머리말(10-30, 세분 격자 실행 전 등록)이며 바꾸지 않았다. 모두 Simulink(데이터셋 시기 모델) 정상상태, 상수 D0, **20 ms 창**(100 ns 실행은 Ipk_at_20ms)으로 같게 비교했다.

- R1(듀티 선 스캔 거칠기): **undecidable** (25 ns / 100 ns 거칠기 중앙값 비 nan, 두 격자 모두 판정 가능한 스캔 2개)
- R2 84 V 100 W: 'M-B lower than SPS' 0.414 A(100 ns) → +0.495 A(25 ns): **kept**
- R2 84 V 150 W: 'M-B lower than SPS' 0.168 A(100 ns) → +0.346 A(25 ns): **kept**
- R2 84 V 150 W: 'M-S lower than M-B' 0.132 A(100 ns) → +0.000 A(25 ns): **gone**
- R2 84 V 150 W: 'M-S lower than SPS' 0.299 A(100 ns) → +0.346 A(25 ns): **kept**
- R2 84 V 400 W: 'SPS lower than M-B' 0.076 A(100 ns) → -0.074 A(25 ns): **reversed**
- R2 84 V 400 W: 'M-S lower than M-B' 0.138 A(100 ns) → +0.002 A(25 ns): **gone**
- R2 84 V 400 W: 'M-S lower than SPS' 0.062 A(100 ns) → +0.076 A(25 ns): **kept**
- R2 84 V 800 W: 'SPS lower than M-B' 0.172 A(100 ns) → -0.055 A(25 ns): **gone**
- R2 84 V 800 W: 'M-S lower than M-B' 0.206 A(100 ns) → -0.018 A(25 ns): **gone**
- R2 109.2 V 100 W: 'M-B lower than SPS' 0.591 A(100 ns) → +0.859 A(25 ns): **kept**
- R2 109.2 V 100 W: 'M-S lower than M-B' 0.183 A(100 ns) → +0.025 A(25 ns): **gone**
- R2 109.2 V 100 W: 'M-S lower than SPS' 0.774 A(100 ns) → +0.884 A(25 ns): **kept**
- R2 109.2 V 300 W: 'M-B lower than SPS' 0.299 A(100 ns) → +0.288 A(25 ns): **kept**
- R2 109.2 V 600 W: 'M-S lower than M-B' 0.387 A(100 ns) → +0.023 A(25 ns): **gone**
- R2 109.2 V 1000 W: 'M-B lower than SPS' 0.165 A(100 ns) → +0.158 A(25 ns): **kept**
- R3(R.4 대표 6점, 둘 다 깨끗): SPS − M-B 평균 +0.249 A(100 ns) → +0.328 A(25 ns)

표 원본 `out/fine_grid_check.csv`, 로그 `out/fine_grid_check.log`.
