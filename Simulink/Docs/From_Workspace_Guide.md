# From Workspace를 통한 제어 파라미터 전달 가이드

## 개요

GA 최적화 시 시뮬레이션마다 다른 제어 파라미터(Kp, Ki, Kpv, Kiv)를
DAB_Controller_System 블록에 전달하는 방법을 설명합니다.

---

## 1. Simulink 모델 설정 방법

### 1.1 From Workspace 블록 추가

**절차:**

1. `Control_Logic.slx` 모델 열기
2. Simulink Library Browser → Sources → **From Workspace** 블록 드래그
3. 블록 더블클릭하여 설정:

```
Parameter               | Value
------------------------|------------------
Data (variable name)    | Ctrl_Params
Sample time             | -1 (inherited)
Interpolation method    | Zero-order hold
Output after final data | Holding final value
```

### 1.2 DAB_Controller_System 입력 연결

**기존 연결:**

```
[V_HV] ──┬──> DAB_Controller_System (입력 1)
[I_HV] ──┼──> DAB_Controller_System (입력 2)
[V_LV] ──┼──> DAB_Controller_System (입력 3)
[I_LV] ──┼──> DAB_Controller_System (입력 4)
[I_ess_cmd] ─> DAB_Controller_System (입력 5)
```

**새로운 연결 (입력 6번 추가):**

```
[From Workspace: Ctrl_Params] ──> DAB_Controller_System (입력 6)
```

### 1.3 MATLAB System 블록 설정

DAB_Controller_System 블록을 더블클릭하여:

```
Property              | Value
----------------------|------------------
UseExternalParams     | ☑ true (체크)
```

---

## 2. GA 스크립트에서의 파라미터 전달

### 2.1 timeseries 생성

```matlab
% 최적화할 파라미터
Kp_ext = 2.5;
Ki_ext = 150.0;
Kpv = -5.0;
Kiv = -500.0;

% 시뮬레이션 시간
SIM_TIME = 1.0;  % 1초

% 시간 벡터 (시작, 끝)
t_params = [0; SIM_TIME];

% 파라미터 값 (시간에 따라 일정)
% 각 행: [Kp, Ki, Kpv, Kiv]
params_data = repmat([Kp_ext, Ki_ext, Kpv, Kiv], 2, 1);

% timeseries 객체 생성
Ctrl_Params = timeseries(params_data, t_params);
Ctrl_Params.Name = 'Ctrl_Params';
```

### 2.2 Base Workspace에 할당

```matlab
% MATLAB Base Workspace에 변수 할당
% (Simulink From Workspace 블록이 이 변수를 읽음)
assignin('base', 'Ctrl_Params', Ctrl_Params);

% UseExternalParams 플래그도 함께 설정
assignin('base', 'UseExternalParams', true);
```

### 2.3 시뮬레이션 실행

```matlab
% 시뮬레이션 실행
sim('Control_Logic', 'StopTime', num2str(SIM_TIME));

% 결과 가져오기
out = evalin('base', 'out');
```

---

## 3. 제어기 내부 처리 로직

### 3.1 DAB_Controller_System.m 코드

```matlab
function [t1, t3, t5, t7, ...] = stepImpl(obj, V_HV, I_HV, V_LV, I_LV, I_ess_cmd, Ctrl_Params)

    % --- 0. 제어 파라미터 추출 ---
    % Ctrl_Params = [Kp_ext, Ki_ext, Kpv, Kiv] (From Workspace에서 입력)

    if obj.UseExternalParams && numel(Ctrl_Params) >= 4 && any(Ctrl_Params ~= 0)
        % 외부 파라미터 사용
        Kp_ext = Ctrl_Params(1);
        Ki_ext = Ctrl_Params(2);
        Kpv = Ctrl_Params(3);
        Kiv = Ctrl_Params(4);
    else
        % 기본값 사용
        Kp_ext = obj.Kp_ext_default;
        Ki_ext = obj.Ki_ext_default;
        Kpv = obj.Kpv_default;
        Kiv = obj.Kiv_default;
    end

    % 이후 Kp_ext, Ki_ext, Kpv, Kiv를 PI 제어에 사용
    ...
end
```

### 3.2 동작 원리

```
┌─────────────────────────────────────────────────────────────┐
│                    시뮬레이션 시작                           │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. GA 스크립트가 Ctrl_Params timeseries 생성               │
│     params_data = [Kp, Ki, Kpv, Kiv]                        │
│                         │                                   │
│                         ▼                                   │
│  2. assignin('base', 'Ctrl_Params', Ctrl_Params)            │
│     → Base Workspace에 저장                                 │
│                         │                                   │
│                         ▼                                   │
│  3. sim('Control_Logic') 실행                               │
│                         │                                   │
│                         ▼                                   │
│  4. From Workspace 블록이 Ctrl_Params 읽음                  │
│     → 매 타임스텝마다 [Kp, Ki, Kpv, Kiv] 벡터 출력          │
│                         │                                   │
│                         ▼                                   │
│  5. DAB_Controller_System.stepImpl()                        │
│     → Ctrl_Params 입력 받아 로컬 변수로 추출                │
│     → PI 제어에 사용                                        │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. 주의사항

### 4.1 데이터 타입

```matlab
% From Workspace 출력은 기본적으로 double
% Ctrl_Params 크기: [1 x 4] 또는 [4 x 1]
% stepImpl에서 numel(Ctrl_Params) >= 4로 검증
```

### 4.2 시간 정렬

```matlab
% timeseries 시간 벡터는 시뮬레이션 시간과 일치해야 함
% 최소 2개 시점 필요 (시작, 끝)
t_params = [0; SIM_TIME];
```

### 4.3 Zero-Order Hold

```
From Workspace 설정에서 'Zero-order hold' 선택 시:
    t = 0.0:  [Kp, Ki, Kpv, Kiv]
    t = 0.1:  [Kp, Ki, Kpv, Kiv]  (동일값 유지)
    t = 0.5:  [Kp, Ki, Kpv, Kiv]  (동일값 유지)
    t = 1.0:  [Kp, Ki, Kpv, Kiv]  (동일값 유지)
```

### 4.4 UseExternalParams 플래그

```matlab
% MATLAB System 블록에서 UseExternalParams = true 설정 필수!
% false이면 기본값(Kp_ext_default 등) 사용
```

---

## 5. 디버깅

### 5.1 파라미터 전달 확인

```matlab
% Base Workspace에서 확인
Ctrl_Params = evalin('base', 'Ctrl_Params');
disp(Ctrl_Params.Data);  % [Kp, Ki, Kpv, Kiv] 출력

% 시뮬레이션 후 To Workspace로 확인
% Simulink 모델에 To Workspace 블록 추가하여 Ctrl_Params 값 기록
```

### 5.2 일반적인 오류

| 오류                             | 원인                        | 해결                         |
| -------------------------------- | --------------------------- | ---------------------------- |
| "Variable Ctrl_Params not found" | Base Workspace에 없음       | assignin() 호출 확인         |
| 파라미터가 기본값으로 동작       | UseExternalParams = false   | 블록 설정에서 true로 변경    |
| 크기 불일치                      | timeseries 데이터 형태 오류 | params_data를 [2 x 4] 행렬로 |

---

## 6. 빠른 테스트

```matlab
%% From Workspace 테스트 스크립트
clear; clc;

% 1. 파라미터 설정
Kp_ext = 3.0;
Ki_ext = 200.0;
Kpv = -6.0;
Kiv = -600.0;
SIM_TIME = 0.5;

% 2. timeseries 생성
t_params = [0; SIM_TIME];
params_data = repmat([Kp_ext, Ki_ext, Kpv, Kiv], 2, 1);
Ctrl_Params = timeseries(params_data, t_params);
Ctrl_Params.Name = 'Ctrl_Params';

% 3. Base Workspace에 할당
assignin('base', 'Ctrl_Params', Ctrl_Params);
assignin('base', 'UseExternalParams', true);
assignin('base', 'SIM_TIME', SIM_TIME);

% 4. 모델 로드 및 시뮬레이션
cd('Modules');  % 모델이 있는 폴더로 이동
load_system('Control_Logic');
sim('Control_Logic', 'StopTime', num2str(SIM_TIME));

% 5. 결과 확인
out = evalin('base', 'out');
fprintf('파라미터 전달 테스트 완료!\n');
fprintf('  Kp=%.2f, Ki=%.2f, Kpv=%.2f, Kiv=%.2f\n', Kp_ext, Ki_ext, Kpv, Kiv);
```

---

_Last updated: 2026-01-21_
