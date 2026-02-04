# 병렬 시뮬레이션 (Parallel Simulation) 및 베이지언 최적화 주의사항

베이지언 최적화(`bayesopt`)나 `parfor`를 사용하여 Simulink 모델을 병렬로 시뮬레이션할 때 발생할 수 있는 문제와 해결 방법을 정리합니다.

## 1. 변수 전달 (Variable Passing)

병렬 워커(Worker)는 메인 MATLAB 워크스페이스(Base Workspace)의 변수를 공유하지 않습니다. 단순히 스크립트에서 변수를 선언했다고 해서 워커가 이를 볼 수 있는 것이 아닙니다.

**잘못된 방법:**

```matlab
Kp = 0.5;
% 워커 내부에서 sim을 호출할 때 베이스 워크스페이스의 Kp를 참조하려 하면 에러 발생 또는 섀도잉 문제 발생
out = sim('MyModel');
```

**올바른 방법 (`Simulink.SimulationInput` 사용):**
`Simulink.SimulationInput` 객체를 사용하여 명시적으로 변수를 주입해야 합니다.

```matlab
in = Simulink.SimulationInput('MyModel');
in = in.setVariable('Kp', 0.5);
in = in.setVariable('Ki', 100);
out = sim(in);
```

## 2. 파일 I/O 및 동시 접근 충돌 (Race Conditions)

여러 워커가 동시에 **동일한 파일**에 쓰기를 시도하면 충돌이 발생하여 시뮬레이션이 멈추거나 에러가 발생합니다.

- **로그 파일:** `Optimization_Log.txt`와 같은 로그 파일을 작성할 때, 동시 접근을 막거나 각 워커별로 별도의 파일을 생성해야 합니다. 하지만 `bayesopt`의 경우 메인 함수(OutputFcn)에서만 로그를 남기는 것이 안전합니다.
- **To File 블록:** Simulink 모델 내부의 'To File' 블록이 고정된 파일명(예: `data.mat`)을 가지고 있다면, 병렬 실행 시 모든 워커가 이 파일을 덮어쓰려 하므로 에러가 납니다. 'To Workspace'를 사용하고 `sim` 함수의 출력(`out`)으로 데이터를 받는 것이 좋습니다.

## 3. 에러 핸들링 (Error Handling via try-catch)

병렬 풀에서 에러가 발생하면 디버깅이 매우 어렵고, 하나의 워커가 죽으면 전체 최적화 프로세스가 중단될 수 있습니다. **반드시 `try-catch` 구문을 사용하여 에러를 포착하고, 최적화가 계속될 수 있도록 해야 합니다.**

**코드 예시:**

```matlab
function Objective = objective_function(params)
    try
        % ... 시뮬레이션 설정 ...
        out = sim(in);

        % ... 결과 처리 및 Cost 계산 ...
        Objective = cost_value;

    catch ME
        % 에러 발생 시 로그 출력
        fprintf('Error in simulation: %s\n', ME.message);

        % 최적화가 멈추지 않도록 매우 큰 Penalty 값 반환
        Objective = 1e6;
    end
end
```

## 4. 모델 로드 및 경로 (Path & Model Loading)

워커들이 모델 파일을 찾지 못하는 경우가 있습니다.

- **경로 추가:** 필요한 경로(라이브러리 등)가 `addpath`로 추가되어 있는지 확인해야 합니다.
- **`load_system`:** 시뮬레이션 시작 전에 `load_system('MyModel')`을 호출하여 메모리에 올리는 것이 안전할 수 있습니다. (보통 `sim`이 처리하지만 명시적인 것이 좋을 때가 있음)

## 5. 진단 및 디버깅 팁

- **`ShowProgress`:** `bayesopt` 옵션에서 `Verbose` 값을 조절하여 진행 상황을 확인합니다.
- **Command Window Output:** 병렬 워커의 `fprintf` 출력은 메인 커맨드 윈도우에 즉시 나타나지 않을 수 있습니다. 중요한 디버그 정보는 파일로 남기거나(이름 충돌 주의), 에러 발생 시 `ME.message`를 확인하는 것이 중요합니다.
- **Worker ID:** 필요하다면 `task = getCurrentTask(); id = task.ID;`를 통해 현재 워커의 ID를 알아내어 로그 파일명에 붙일 수 있습니다 (`Log_Worker1.txt`).

6. 자동화 스크립트 (Automation Script: Model Workspace Cleanup)

모델 워크스페이스(Model Workspace)에 이미 `Kp`, `Ki` 같은 변수가 정의되어 있으면, 시뮬레이션 시 Base Workspace나 `Simulink.SimulationInput`의 값이 제대로 적용되지 않거나 충돌이 발생할 수 있습니다. 이를 해결하기 위해 모델 워크스페이스의 변수를 자동으로 삭제하고 Base Workspace로 옮기는 스크립트를 사용하세요.

**스크립트 위치:** `Simulink/Script/Cleanup_Model_Workspace.m`

**사용법:**

```matlab
% 모델명과 정리할 변수 목록을 입력
Cleanup_Model_Workspace('Main_Simulation', {'Kp', 'Ki'});
```

이 함수는 모델 워크스페이스에 해당 변수가 있으면 값을 Base Workspace로 대피시키고, 모델 워크스페이스에서는 삭제하여 섀도잉 문제를 해결합니다.

## 요약

1.  **`Simulink.SimulationInput`**으로 변수 전달 (필수).
2.  **`try-catch`**로 에러 방어 및 페널티 부여 (필수).
3.  **파일 쓰기 충돌** 주의 (To File 블록 제거 또는 고유 이름 사용).
4.  **모델 워크스페이스 정리**로 변수 충돌 예방.
