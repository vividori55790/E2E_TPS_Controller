% Apply_Low_Memory_Settings.m
% Simulink 모델에 저메모리 설정 적용
%
% 실행 방법:
%   1. 이 스크립트를 실행
%   2. 시뮬레이션 실행 (약간 느려지지만 메모리 사용량 대폭 감소)
%
% 적용되는 설정:
%   - Signal Logging 비활성화
%   - 데이터 포인트 제한
%   - 시뮬레이션 스텝 크기 증가 (선택)
%   - Accelerator 모드 비활성화

fprintf('=================================================================\n');
fprintf('  Low Memory Settings for Simulink\n');
fprintf('=================================================================\n\n');

%% ==================== 모델 로드 ====================
model_name = 'Control_Logic';
model_path = fullfile(pwd, '..', 'Modules', [model_name '.slx']);

if ~exist(model_path, 'file')
    error('모델을 찾을 수 없습니다: %s', model_path);
end

fprintf('[1/6] 모델 로드: %s\n', model_name);
load_system(model_name);

%% ==================== 시뮬레이션 모드 변경 ====================
fprintf('[2/6] 시뮬레이션 모드 → Normal (메모리 절약)\n');
set_param(model_name, 'SimulationMode', 'normal');

%% ==================== Signal Logging 비활성화 ====================
fprintf('[3/6] Signal Logging 비활성화\n');
try
    set_param(model_name, 'SignalLogging', 'off');
catch
    fprintf('    (SignalLogging 파라미터 없음 - 무시)\n');
end

%% ==================== Data Import/Export 설정 ====================
fprintf('[4/6] 데이터 포인트 제한 (최대 10,000)\n');
try
    % 데이터 포인트 제한
    set_param(model_name, 'LimitDataPoints', 'on');
    set_param(model_name, 'MaxDataPoints', '10000');
    
    % 상태/출력 저장 비활성화 (필요시)
    % set_param(model_name, 'SaveState', 'off');
    % set_param(model_name, 'SaveOutput', 'off');
    % set_param(model_name, 'SaveTime', 'off');
catch ME
    fprintf('    경고: %s\n', ME.message);
end

%% ==================== Solver 설정 (선택적) ====================
fprintf('[5/6] Solver 설정 확인\n');

current_step = get_param(model_name, 'FixedStep');
fprintf('    현재 Fixed Step: %s\n', current_step);

% 스텝 크기 증가 옵션 (주석 해제하여 사용)
% 주의: 제어 정확도에 영향을 줄 수 있음
%{
new_step = '50e-6';  % 10us → 50us (5배 빠름, 메모리 5배 절약)
fprintf('    Fixed Step 변경: %s → %s\n', current_step, new_step);
set_param(model_name, 'FixedStep', new_step);
%}

%% ==================== Inline Parameters 비활성화 ====================
fprintf('[6/6] Inline Parameters 비활성화 (컴파일 메모리 절약)\n');
try
    set_param(model_name, 'InlineParams', 'off');
catch
    fprintf('    (InlineParams 파라미터 없음 - 무시)\n');
end

%% ==================== 추가 최적화 설정 ====================
fprintf('\n추가 최적화 설정 적용 중...\n');

try
    % Block Reduction 끄기 (컴파일 시간 단축)
    set_param(model_name, 'BlockReduction', 'off');
    fprintf('    Block Reduction: off\n');
catch
end

try
    % 조건부 입력 분기 최적화 끄기
    set_param(model_name, 'ConditionallyExecuteInputs', 'off');
    fprintf('    ConditionallyExecuteInputs: off\n');
catch
end

try
    % 불필요한 데이터 타입 변환 최적화 끄기
    set_param(model_name, 'OptimizeBlockIOStorage', 'off');
    fprintf('    OptimizeBlockIOStorage: off\n');
catch
end

%% ==================== 모델 저장 ====================
fprintf('\n모델 설정 저장 중...\n');
save_system(model_name);
fprintf('✓ 저장 완료\n');

%% ==================== 요약 ====================
fprintf('\n=================================================================\n');
fprintf('  저메모리 설정 적용 완료!\n');
fprintf('=================================================================\n');
fprintf('  적용된 설정:\n');
fprintf('    ✓ SimulationMode: Normal\n');
fprintf('    ✓ SignalLogging: Off\n');
fprintf('    ✓ MaxDataPoints: 10,000\n');
fprintf('    ✓ InlineParams: Off\n');
fprintf('    ✓ 여러 최적화 옵션 비활성화\n');
fprintf('\n');
fprintf('  [참고] 추가 메모리 절약이 필요하면:\n');
fprintf('    1. 이 스크립트에서 Solver 설정 섹션 주석 해제\n');
fprintf('       (Fixed Step: 10us → 50us로 변경)\n');
fprintf('    2. To Workspace 블록의 Decimation 값 증가\n');
fprintf('       (예: Decimation = 10 → 데이터 1/10만 저장)\n');
fprintf('=================================================================\n');
