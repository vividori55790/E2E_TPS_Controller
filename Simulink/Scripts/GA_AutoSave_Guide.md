# GA 최적화 자동 저장 기능 가이드

## 📋 개요

GA (Genetic Algorithm) 최적화 스크립트가 개선되어 다음 기능들이 추가되었습니다:

✅ **자동 저장**: 각 세대(generation)마다 결과를 `.mat` 파일로 자동 저장  
✅ **로그 기록**: 진행 상황을 텍스트 파일로 실시간 기록  
✅ **재개 기능**: 중단된 최적화를 이어서 실행 가능  
✅ **결과 비교**: 여러 실행 결과를 쉽게 비교

---

## 📁 파일 구조

### 메인 스크립트

- **`GA_Optimize_Controller.m`**: 개선된 GA 최적화 스크립트 (자동 저장 기능)
- **`GA_Resume_Optimization.m`**: 중단된 최적화를 재개하는 스크립트
- **`View_GA_Results.m`**: 저장된 결과를 확인하고 비교하는 스크립트

### 저장 폴더

- **`GA_Results/`**: 모든 결과가 저장되는 폴더 (자동 생성)
  - `GA_Progress_YYYY-MM-DD_HH-MM-SS.mat`: 각 세대마다 업데이트되는 진행 파일
  - `GA_Log_YYYY-MM-DD_HH-MM-SS.txt`: 실시간 로그 파일
  - `GA_Final_YYYY-MM-DD_HH-MM-SS.mat`: 최종 결과 파일
  - `GA_FinalPlot_YYYY-MM-DD_HH-MM-SS.fig`: 최종 결과 플롯

---

## 🚀 사용법

### 1. 새로운 GA 최적화 실행

```matlab
% MATLAB 커맨드 윈도우에서
GA_Optimize_Controller()
```

**자동으로 수행되는 작업:**

- `GA_Results/` 폴더 생성
- 각 세대마다 `GA_Progress_*.mat` 파일 업데이트
- 진행 상황을 `GA_Log_*.txt`에 실시간 기록
- 최종 결과를 `GA_Final_*.mat`로 저장
- 최종 플롯을 `.fig` 파일로 저장

**중간에 꺼지더라도 걱정 없음!**

- Progress 파일에 마지막 세대까지의 결과가 저장되어 있습니다.

---

### 2. 중단된 최적화 재개

MATLAB이 꺼졌거나 최적화가 중단되었을 때:

```matlab
% 마지막 저장 지점부터 재개
GA_Resume_Optimization()
```

**동작 방식:**

1. `GA_Results/` 폴더에서 가장 최근 `GA_Progress_*.mat` 파일을 자동으로 찾음
2. 저장된 개체군(population)과 점수를 복원
3. 중단된 세대부터 최적화를 계속 진행
4. 새로운 로그 파일(`GA_Log_Resumed_*.txt`)로 기록

---

### 3. 저장된 결과 확인

```matlab
% 모든 저장된 결과 보기
View_GA_Results()
```

**표시되는 정보:**

- 모든 실행 결과 목록
- 각 결과의 Kp, Ki, Cost, 실행 시간
- 최적 결과 (Cost가 가장 낮은 것)
- 결과 비교 플롯

**워크스페이스에 저장되는 변수:**

- `GA_AllResults`: 모든 결과의 배열
- `GA_BestResult`: 최적 결과 구조체

---

## 📊 저장되는 데이터

### Progress 파일 (`GA_Progress_*.mat`)

```matlab
saveData = struct();
saveData.Generation      % 현재 세대 번호
saveData.BestFval        % 현재까지 최적 비용
saveData.BestIndividual  % 최적 개체
saveData.Population      % 전체 개체군
saveData.Score           % 각 개체의 점수
saveData.Timestamp       % 저장 시각
```

### Final 파일 (`GA_Final_*.mat`)

```matlab
finalResult = struct();
finalResult.Kp            % 최적 Kp 값
finalResult.Ki            % 최적 Ki 값
finalResult.Cost          % 최종 비용
finalResult.Elapsed       % 소요 시간 (초)
finalResult.Generations   % 총 세대 수
finalResult.ExitFlag      % GA 종료 플래그
finalResult.Timestamp     % 실행 시작 시각
finalResult.RefCurrent    % 사용된 참조 전류
finalResult.StepTime      % Step 시점
finalResult.SimTime       % 시뮬레이션 시간
```

### Log 파일 (`GA_Log_*.txt`)

```
[10:05:23] Initializing optimization...
[10:05:24] Scenario loaded successfully
[10:05:25] Model "Control_Logic" loaded
[10:05:26] Starting GA optimization...
[10:05:30] Gen   1: Best Cost = 12.345678
[10:05:34] Gen   2: Best Cost = 10.234567
...
```

---

## 💡 시나리오별 사용 팁

### 시나리오 1: 밤새 실행 후 PC가 꺼진 경우

```matlab
% 1. 저장된 결과 확인
View_GA_Results()

% 2. 로그 파일 확인 (어디까지 실행되었는지)
type('GA_Results\GA_Log_2026-01-23_05-00-00.txt')

% 3. 필요하면 재개
GA_Resume_Optimization()
```

### 시나리오 2: 결과가 마음에 안 들어서 다시 실행

```matlab
% 새로 실행 (이전 결과는 자동으로 보존됨)
GA_Optimize_Controller()

% 나중에 모든 결과를 비교
View_GA_Results()
```

### 시나리오 3: 최적 파라미터를 바로 사용하고 싶을 때

```matlab
% 1. 결과 로드
View_GA_Results()

% 2. 워크스페이스에서 사용
Kp_optimized = GA_BestResult.Kp;
Ki_optimized = GA_BestResult.Ki;

% 3. Controller에 적용
assignin('base', 'Kp_ext', Kp_optimized);
assignin('base', 'Ki_ext', Ki_optimized);
```

---

## ⚙️ 설정 변경

`GA_Optimize_Controller.m`에서 설정을 변경할 수 있습니다:

```matlab
% Line 66-67: GA 파라미터
'PopulationSize', 10,      % 개체군 크기 (클수록 느리지만 정확)
'MaxGenerations', 20,      % 최대 세대 수

% Line 62-63: 탐색 범위
lb = [0.01, 1.0];          % [Kp_min, Ki_min]
ub = [50.0, 5000.0];       % [Kp_max, Ki_max]

% Line 49-51: 시뮬레이션 조건
STEP_TIME = 0.02;          % Step 발생 시점 [s]
REF_CURRENT = 10.0;        % 목표 전류 [A]
SIM_TIME = 0.05;           % 시뮬레이션 시간 [s]
```

---

## 🔍 문제 해결

### Q: "GA_Results 폴더가 존재하지 않습니다" 오류

**A:** `GA_Optimize_Controller()`를 먼저 실행하세요. 폴더가 자동으로 생성됩니다.

### Q: Resume 했는데 처음부터 시작되는 것 같아요

**A:** `GA_Resume_Optimization.m` 내부에서 `InitialPopulationMatrix`를 확인하세요. Progress 파일이 올바르게 로드되었는지 확인하세요.

### Q: 로그 파일이 너무 많아요

**A:** 오래된 파일은 수동으로 삭제해도 됩니다. 또는 `GA_Results/Archive/` 폴더를 만들어 이동시키세요.

### Q: 어느 결과가 최종 결과인가요?

**A:** `View_GA_Results()`를 실행하면 자동으로 최적 결과를 찾아줍니다.

---

## 📝 버전 정보

- **버전**: 2.0
- **최종 수정일**: 2026-01-23
- **주요 개선 사항**:
  - 자동 저장 기능 추가
  - 로그 파일 생성
  - 재개 기능 구현
  - 결과 비교 도구 추가

---

## 📞 추가 정보

- 비용 함수(Cost Function)는 `evaluateCost()` 함수에서 정의됩니다
- ISE, ITAE, Overshoot, Settling Time을 조합하여 최적화합니다
- 가중치 조정이 필요하면 Line 248-251을 수정하세요

```matlab
% Line 248-251: 비용 함수 가중치
cost = 2.0 * (ISE / 50) + ...      % ISE 기여도
       1.0 * (ITAE / 0.5) + ...    % ITAE 기여도
       3.0 * (OS / 100) + ...      % Overshoot 기여도
       1.5 * (Ts / 0.03);          % Settling Time 기여도
```

---

**즐거운 최적화 되세요! 🚀**
