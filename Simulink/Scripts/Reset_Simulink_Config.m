% Reset_Simulink_Config.m
% Simulink 모델의 하드웨어 설정을 "Generic Real-Time (PC Simulation)"으로 강제 초기화합니다.
% GA 최적화 시 발생하는 STM32/PIL 관련 오류를 해결하기 위함입니다.

modelName = 'Control_Logic'; % 메인 모델 이름
subModelName = 'DAB_Controller_System'; % (만약 서브 모델이 별도 파일이라면)

try
    disp(['>>> Opening Model: ', modelName]);
    load_system(modelName);
    
    cs = getActiveConfigSet(modelName);
    
    % 1. 하드웨어 보드 설정 초기화 (STM32 -> None)
    disp('>>> Resetting Hardware Board to "None" (Generic)...');
    set_param(cs, 'HardwareBoard', 'None');
    
    % 2. 솔버 설정 (Fixed-step, Auto - let Simulink decide)
    disp('>>> Configuring Solver for PC Simulation (Auto)...');
    set_param(cs, 'SolverType', 'Fixed-step');
    set_param(cs, 'Solver', 'FixedStepAuto'); % Simulink가 알아서 결정하도록 복구
    set_param(cs, 'FixedStep', '1e-7'); % 샘플 타임은 유지
    
    % 3. 코드 생성 타겟을 "grt.tlc" (Generic Real-Time)로 변경
    disp('>>> Setting System Target File to "grt.tlc"...');
    set_param(cs, 'SystemTargetFile', 'grt.tlc');
    
    % 4. 시뮬레이션 모드를 "Normal"로 강제 설정
    disp('>>> Setting Simulation Mode to "Normal"...');
    set_param(modelName, 'SimulationMode', 'normal');
    
    % 5. 서브시스템/모델 참조 블록도 Normal 모드로
    % (모델 내 모든 ModelReference 블록을 찾아서 Normal로 변경)
    refBlocks = find_system(modelName, 'BlockType', 'ModelReference');
    for i = 1:length(refBlocks)
        disp(['    Found ModelRef: ', refBlocks{i}]);
        set_param(refBlocks{i}, 'SimulationMode', 'Normal');
    end

    % 6. 캐시 및 아티팩트 정리 (Crucial for erasing PIL leftovers)
    disp('>>> Clearing "slprj", "slruns" and cache folders...');
    if exist('slprj', 'dir'), rmdir('slprj', 's'); end
    if exist('slruns', 'dir'), rmdir('slruns', 's'); end
    Simulink.sdi.clear;
    
    % 7. 저장
    save_system(modelName);
    disp('>>> [SUCCESS] All configurations reset to Generic MIL Mode & Cache Cleared.');
    disp('>>> You can now restart MATLAB and run GA optimization.');

catch ME
    disp('>>> [ERROR] Failed to reset configuration.');
    disp(ME.message);
end
