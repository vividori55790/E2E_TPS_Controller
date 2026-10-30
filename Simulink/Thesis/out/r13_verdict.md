# R13 1단계: 25 ns 격자 Simulink 정상상태 지도 (자동 생성)

생성 2026-10-30 15:30, `r13_map.py compare`. 규칙은 `r13_map.py` 머리말(10-30 15:20, R13 실행 전 등록)이며 바꾸지 않았다. 모두 Simulink(데이터셋 시기 모델, 제어기·로컬 솔버 25 ns) 정상상태, 상수 D0 두 계단 전력 보간, 20 ms 창.

- 점 56개(V2 7값 × |P| 8값) 중 Q1 판정 가능 0개 — **잠정**(75 % 미만)
- Q1 M-S 국소 최적성: 최적 0, 최적 아님 0 → 2단계(재학습): **안 함**
- Q2 목표 3: M-S가 SPS보다 나쁜 점 0, M-B 0
- Q3 M-S: 7점 평균 SPS − M-S +0.294 A(중앙값 +0.175), SPS 대비 +13.7 %
- Q3 M-B: 10점 평균 SPS − M-B +0.259 A(중앙값 +0.155), SPS 대비 +11.3 %
- Q4 M-S 대 M-B(31점): 개선 0, 같음 5, 열화 0 → **M-S holds**
- Q5 93 V 100 W: 'SPS lower than M-B' +0.190 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 93 V 100 W: 'SPS lower than M-S' +0.050 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 93 V 100 W: 'M-S lower than M-B' +0.140 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 93 V 150 W: 'SPS lower than M-B' +0.105 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 84 V 700 W: 'SPS lower than M-S' +0.095 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 84 V 700 W: 'M-B lower than M-S' +0.072 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 109.2 V 700 W: 'M-S lower than M-B' +1.003 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 95 V 70 W: 'D0.40 lower than SPS' +0.286 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 95 V 100 W: 'D0.40 lower than SPS' +0.149 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 150 W: 'M-S lower than SPS' +0.581 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 150 W: 'M-S lower than M-B' +0.581 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 200 W: 'M-S lower than SPS' +0.518 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 200 W: 'M-S lower than M-B' +0.518 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 300 W: 'M-S lower than SPS' +0.270 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 96 V 300 W: 'M-S lower than M-B' +0.270 A(100 ns) → +nan A(25 ns): **undecidable**
- Q5 97 V 150 W: 'M-B lower than M-S' +0.259 A(100 ns) → +nan A(25 ns): **undecidable**

**아직 감싸지 못한 후보 값 333개 — 잠정**(`r13_map.py next`).

| V2 | |P| | SPS | M-B | M-S | Q1 | 차(최선) | Q2 M-S | Q2 M-B |
|---|---|---|---|---|---|---|---|---|
| 84 | 100 |  1.747 |  1.252 |  1.252 | 판정 불가 | +nan () | better | better |
| 84 | 150 |  1.872 |  1.527 |  1.526 | 판정 불가 | +nan () | better | better |
| 84 | 200 |  2.001 |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 84 | 300 |  2.260 |  2.138 |      - | 판정 불가 | +nan () | undecidable | better |
| 84 | 400 |  2.528 |  2.453 |  2.451 | 판정 불가 | +nan () | better | better |
| 84 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 84 | 800 |  3.693 |  3.638 |  3.656 | 판정 불가 | +nan () | same | same |
| 84 | 1000 |  4.345 |  4.300 |  4.300 | 판정 불가 | +nan () | same | same |
| 88.2 | 100 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 150 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 200 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 300 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 400 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 800 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 88.2 | 1000 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 100 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 150 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 200 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 300 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 400 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 800 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 92.4 | 1000 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 100 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 150 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 200 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 300 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 400 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 800 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 96.6 | 1000 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 100 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 150 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 200 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 300 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 400 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 800 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 100.8 | 1000 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 100 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 150 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 200 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 300 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 400 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 600 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 800 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 105 | 1000 |      - |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 109.2 | 100 |  2.269 |  1.410 |  1.386 | 판정 불가 | +nan () | better | better |
| 109.2 | 150 |  2.395 |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 109.2 | 200 |  2.524 |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 109.2 | 300 |  2.783 |  2.495 |      - | 판정 불가 | +nan () | undecidable | better |
| 109.2 | 400 |  3.048 |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 109.2 | 600 |  3.592 |  3.439 |  3.417 | 판정 불가 | +nan () | better | better |
| 109.2 | 800 |  4.165 |      - |      - | 판정 불가 | +nan () | undecidable | undecidable |
| 109.2 | 1000 |  4.758 |  4.599 |      - | 판정 불가 | +nan () | undecidable | better |

표 원본 `out/r13_map.csv`(후보별), `out/r13_points.csv`(점별), 로그 `out/r13_map.log`, 그림 `out/fig_r13_map.png`.
