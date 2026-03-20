%% GA_Optimize_Controller.m
% DAB Converter Controller Optimization using Genetic Algorithm (GA)
% Target: Optimize Kp_ext, Ki_ext for ESS Current Control
% Updated: 2026-01-23 1170, 70 Would work
%
% [주요 기능]
% - 매 세대마다 터미널에 상세 성능 지표 출력
% - 매 세대마다 전류 그래프 실시간 플롯
% - 모든 성능 지표를 CSV/MAT 파일로 저장
% - 자동 저장 및 재개 기능

function GA_Optimize_Controller()
    %% 0. 환경 설정
    clear; clc; close all;
    clear functions; % [NEW] persistent 변수 초기화 (evalCount 등)
    
    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath), scriptPath = pwd; end
    addpath(scriptPath); 
    addpath(fullfile(scriptPath, '..'));
    addpath(fullfile(scriptPath, '..', 'Modules'));
    addpath(fullfile(scriptPath, '..', 'Data'));
    addpath(fullfile(scriptPath, 'GA_Utils')); % [NEW] Utils Path Added
    
    % 결과 저장 폴더 생성
    resultDir = fullfile(scriptPath, 'GA_Results');
    if ~exist(resultDir, 'dir')
        mkdir(resultDir);
    end
    
    % 타임스탬프로 파일명 생성
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    saveFile = fullfile(resultDir, sprintf('GA_Progress_%s.mat', timestamp));
    logFile = fullfile(resultDir, sprintf('GA_Log_%s.txt', timestamp));
    metricsFile = fullfile(resultDir, sprintf('GA_Metrics_%s.csv', timestamp));
    
    % 캐시 파일 설정 (결과 재사용 - DISABLE_LOAD: 항상 새로 시작)
    cacheFile = fullfile(resultDir, 'GA_History_Cache.mat');
    
    % [RESET] 이전 기록 참조 안 함. 항상 빈 캐시로 시작.
    GA_Cache_Map = containers.Map('KeyType', 'char', 'ValueType', 'any');
    assignin('base', 'GA_Cache_Map', GA_Cache_Map);
    
    % 로그 파일 초기화
    logID = fopen(logFile, 'w');
    fprintf(logID, '===========================================\n');
    fprintf(logID, 'DAB Controller GA Optimization Log\n');
    fprintf(logID, 'Started: %s\n', datestr(now));
    fprintf(logID, '===========================================\n\n');
    fclose(logID);
    
    % CSV 헤더 작성
    csvID = fopen(metricsFile, 'w');
    fprintf(csvID, 'EvalNum,Kp,Ki,Cost,ISE,ITAE,IAE,Overshoot_pct,Undershoot_pct,');
    fprintf(csvID, 'RiseTime_ms,SettlingTime_ms,PeakTime_ms,SteadyStateError_pct,');
    fprintf(csvID, 'PeakValue,TargetValue,Valid,Cached\n'); % [Added Cached column]
    fclose(csvID);
    
    %% 1. 시나리오 및 모델 초기화
    disp('===========================================');
    disp('  DAB Controller GA Optimization');
    disp('  [Enhanced with Caching & Real-time Monitoring]');
    disp('===========================================');
    writeLog(logFile, 'Initializing optimization...');
    
    % Create_Scenario 실행
    try
        run(fullfile(scriptPath, 'Create_Scenario.m'));
        disp('>>> Scenario Loaded.');
        writeLog(logFile, 'Scenario loaded successfully');
    catch ME
        msg = sprintf('Create_Scenario.m 실행 실패: %s', ME.message);
        writeLog(logFile, msg);
        error(msg);
    end
    
    % 모델 확인 및 로드
    modelName = 'Control_Logic';
    if ~bdIsLoaded(modelName)
        try
            load_system(modelName);
            disp(['>>> Model "' modelName '" loaded.']);
            writeLog(logFile, sprintf('Model "%s" loaded', modelName));
        catch
            msg = sprintf('Model "%s" not found!', modelName);
            writeLog(logFile, msg);
            error(msg);
        end
    end
    
    % [CLEAN FIX] Disable Simulink Cache to prevent parallel file access errors (EOF)
    try
        set_param(modelName, 'SimulinkCacheManifest', 'off');
        % Delete existing cache if it exists
        cacheFileSLXC = [modelName '.slxc'];
        if exist(cacheFileSLXC, 'file')
            delete(cacheFileSLXC);
            disp('>>> Simulink Cache (.slxc) cleared for parallel safety.');
        end
    catch
        % Parameter might not exist in older versions or if model is locked
    end
    %% 2. ===== 하드코딩된 지령치 (DO NOT DELETE) =====
    %% 2. ===== 하드코딩된 지령치 (Updated: Bidirectional Step) =====
    % 사용자 요구사항: +10A -> -10A -> +10A (Zero Crossing & Radical Change)
    SIM_TIME = 0.10;       % 시뮬레이션 시간 [s]
    
    CMD_LEVEL_HIGH = 10.0;
    CMD_LEVEL_LOW = -10.0;
    
    % Step Timings
    T_STEP_1 = 0.03; % 10A -> -10A
    T_STEP_2 = 0.06; % -10A -> 10A
    
    % Reference Time Series
    t_cmd = [0, T_STEP_1 - 1e-6, T_STEP_1, T_STEP_2 - 1e-6, T_STEP_2, SIM_TIME];
    v_cmd = [CMD_LEVEL_HIGH, CMD_LEVEL_HIGH, CMD_LEVEL_LOW, CMD_LEVEL_LOW, CMD_LEVEL_HIGH, CMD_LEVEL_HIGH];
    
    RefCurrentTS = timeseries(v_cmd(:), t_cmd(:));
    
    % Step Info for Metrics (Use the first major drop as primary characteristic)
    STEP_TIME = T_STEP_1; 
    STEP_MAGNITUDE = abs(CMD_LEVEL_LOW - CMD_LEVEL_HIGH); % 20A
    
    fprintf('>>> Reference: Bidirectional Step (+10A -> -10A -> +10A)\n');
    writeLog(logFile, 'Reference: Bidirectional Step (+10A -> -10A -> +10A)');
    
    %% 3. 실시간 플롯 Figure 생성
    figHandle = figure('Name', 'GA Real-time Monitor', 'Position', [50, 50, 1600, 500]);
    
    % 왼쪽: 전류 응답 플롯
    ax1 = subplot(1,3,1);
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlabel(ax1, 'Time [ms]');
    ylabel(ax1, 'Current [A]');
    title(ax1, 'Current Response (Best Individual)');
    xlim(ax1, [0, SIM_TIME*1000]);
    
    % 중앙: Cost 히스토리 플롯
    ax2 = subplot(1,3,2);
    hold(ax2, 'on');
    grid(ax2, 'on');
    xlabel(ax2, 'Evaluation #');
    ylabel(ax2, 'Cost');
    title(ax2, 'Cost History');

    % 오른쪽: 파라미터 탐색 공간 (Scatter Plot)
    ax3 = subplot(1,3,3);
    hold(ax3, 'on');
    grid(ax3, 'on');
    xlabel(ax3, 'Kp');
    ylabel(ax3, 'Ki');
    title(ax3, 'Parameter Search Space');
    set(ax3, 'XScale', 'log', 'YScale', 'log'); % 광범위한 탐색이므로 로그 스케일 권장
    
    drawnow;
    
    % [NEW] 안전 종료를 위한 STOP 버튼 생성
    uicontrol('Parent', figHandle, 'Style', 'pushbutton', 'String', '🛑 STOP (즉시 종료)', ...
              'Units', 'normalized', 'Position', [0.4, 0.01, 0.2, 0.05], ...
              'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', [1, 0.7, 0.7], ...
              'Callback', 'set(gcbf, ''Tag'', ''STOP_NOW''); error(''User requested STOP'');');

    %% 3.5. 병렬 처리 준비 (Parallel Pool)
    % [REQ] 병렬 풀 활성화
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        disp('>>> Starting Parallel Pool for faster optimization...');
        parpool; % 기본 프로필로 시작
    else
        disp(['>>> Parallel Pool active: ' num2str(poolobj.NumWorkers) ' workers.']);
    end

    %% 4. GA 설정
    nVars = 2;  % [Kp, Ki]
    % 파라미터 범위 수정 (Saturation 방지)
    % 기존 [100, 10000]은 너무 커서 제어기가 포화(90도)되고, 결과가 똑같아짐(Cost Stagnation 원인)
    lb = [0.001, 0.1];       % 하한
    ub = [5.0, 500.0];       % 상한 (현실적인 범위로 축소)
    
    % 전역 변수로 평가 카운터 및 데이터 저장
    evalData = struct();
    evalData.count = 0;
    evalData.bestCost = inf;
    evalData.bestParams = [0, 0];
    evalData.costHistory = [];
    evalData.allMetrics = [];
    
    % GA 옵션: 탐색 성능 강화 (Global Search)
    popSize = 50;   % [Increased] 20 -> 50
    maxGen = 50;    % [Increased] 30 -> 50
    
    options = optimoptions('ga', ...
        'PopulationSize', popSize, ...
        'MaxGenerations', maxGen, ...
        'MaxStallGenerations', 10, ...
        'FunctionTolerance', 1e-6, ... 
        'InitialPopulationRange', [0.01, 10; 2.0, 100], ... % [MODIFIED] 초기 탐색을 선형 영역으로 유도
        'CrossoverFraction', 0.8, ...
        'SelectionFcn', @selectionroulette, ... 
        'Display', 'off', ... 
        'UseParallel', true, ... 
        'PlotFcn', [], ...
        'OutputFcn', @(options, state, flag) gaOutputFcn(options, state, flag, ...
            saveFile, logFile, modelName, RefCurrentTS, SIM_TIME, STEP_TIME, STEP_MAGNITUDE, ...
            figHandle, ax1, ax2, ax3, metricsFile, maxGen, cacheFile));
    
    writeLog(logFile, sprintf('GA Settings: PopSize=%d, MaxGen=%d, UseParallel=TRUE', popSize, maxGen));
    writeLog(logFile, 'Selection: Roulette Wheel (Exploration Focused)');

    writeLog(logFile, sprintf('Search bounds: Kp=[%.4f, %.2f], Ki=[%.2f, %.2f]', lb(1), ub(1), lb(2), ub(2)));
    writeLog(logFile, sprintf('Step Input: Bidirectional (+%.1fA -> %.1fA -> +%.1fA)', CMD_LEVEL_HIGH, CMD_LEVEL_LOW, CMD_LEVEL_HIGH));
    
    %% 5. GA 실행 (with Pre-flight Check)
    disp(' ');
    disp('>>> 4.5. Pre-flight Check (Dry Run)...');
    
    % [NEW] Parallel DataQueue 설정 (Centralized Logging)
    % Worker들이 데이터를 보내면 Master가 받아서 CSV에 씁니다.
    DQueue = parallel.pool.DataQueue;
    afterEach(DQueue, @(data) logWorkerData(metricsFile, data));
    
    % 비용 함수 정의 (DataQueue 전달)
    costFcn = @(x) evaluateCostWithMetrics(x, modelName, RefCurrentTS, SIM_TIME, STEP_TIME, metricsFile, figHandle, DQueue);

    try
        % Dry Run: 안전한 파라미터로 1회 실행하여 모든 기능(Simulink, 파라미터, CSV 등) 점검
        dryRunParams = [1.0, 100.0];
        fprintf('    Running single simulation to verify system integrity (Kp=1, Ki=100)...\n');
        
        % Dry Run에서는 Queue가 있어도 동작하지만, 단일 실행이므로 직접 리턴값 확인
        dryCost = evaluateCostWithMetrics(dryRunParams, modelName, RefCurrentTS, SIM_TIME, STEP_TIME, metricsFile, figHandle, DQueue);
        fprintf('    >>> [SUCCESS] Pre-flight Check Passed! (Cost: %.4f)\n', dryCost);
        fprintf('    >>> Now starting massive parallel optimization...\n');
    catch ME
        fprintf('\n>>> [FAIL] Pre-flight Check Failed!\n');
        fprintf('    Error: %s\n', ME.message);
        fprintf('    Check Simulink model, paths, or parameters.\n');
        error('Pre-flight check failed.');
    end

    disp(' ');
    disp('>>> Starting GA Optimization...');
    disp('>>> Generation 1 is calculating... (Please wait)');
    disp(' ');

    writeLog(logFile, 'Starting GA optimization...');
    
    t_start = tic;
    try
        [x_best, fval, exitflag, output] = ga(costFcn, nVars, [], [], [], [], lb, ub, [], options);
    catch ME
        if strcmp(get(figHandle, 'Tag'), 'STOP_NOW')
            disp('>>> 🛑 GA Optimization stopped by user (Immediate Stop).');
            % 현재까지 찾은 최적해 복구 시도 (global 변수 활용 필요하지만 여기선 persistent 변수라 복구 불가)
            % 하지만 GA 로직상 마지막 x_best는 없어도 로그는 남음
            x_best = [1, 100]; % 임시 값
            fval = inf;
            exitflag = -1;
            output = struct('generations', 0, 'funccount', 0);
        else
            rethrow(ME);
        end
    end
    elapsed = toc(t_start);
    
    %% 6. 최종 결과 출력
    fprintf('\n');
    disp('===========================================================');
    disp('                    OPTIMIZATION COMPLETE!                  ');
    disp('===========================================================');
    fprintf('  Total Time:     %.2f sec (%.2f min)\n', elapsed, elapsed/60);
    fprintf('  Generations:    %d\n', output.generations);
    fprintf('  Function Evals: %d\n', output.funccount);
    fprintf('-----------------------------------------------------------\n');
    fprintf('  BEST PARAMETERS:\n');
    fprintf('    Kp = %.6f\n', x_best(1));
    fprintf('    Ki = %.6f\n', x_best(2));
    fprintf('  BEST COST: %.6f\n', fval);
    disp('===========================================================');
    
    writeLog(logFile, '');
    writeLog(logFile, '===========================================');
    writeLog(logFile, 'Optimization Complete!');
    writeLog(logFile, sprintf('Time: %.2f sec, Generations: %d', elapsed, output.generations));
    writeLog(logFile, sprintf('Best Kp: %.6f, Best Ki: %.6f', x_best(1), x_best(2)));
    writeLog(logFile, sprintf('Best Cost: %.6f', fval));
    
    % 캐시 저장 (Final)
    try
        GA_Cache_Map = evalin('base', 'GA_Cache_Map');
        save(cacheFile, 'GA_Cache_Map');
        fprintf('>>> Simulation cache saved to: %s (%d entries)\n', cacheFile, GA_Cache_Map.Count);
    catch
    end
    
    %% 7. 최종 검증 및 상세 성능 분석
    disp(' ');
    disp('>>> Running final verification with detailed analysis...');
    
    [finalCost, finalMetrics, t_sim, I_meas, I_ref, Phi_meas] = evaluateWithFullMetrics(...
        x_best, modelName, RefCurrentTS, SIM_TIME, STEP_TIME);
    
    % 최종 성능 지표 출력
    disp('-----------------------------------------------------------');
    disp('              FINAL PERFORMANCE METRICS                    ');
    disp('          (Step-Normalized, Anti-Cheat Version)            ');
    disp('-----------------------------------------------------------');
    fprintf('  Step Size:                       %.2f A (%.1f → %.1f A)\n', finalMetrics.StepSize, finalMetrics.InitValue, finalMetrics.TargetValue);
    fprintf('  ISE (Integral Squared Error):    %.6f\n', finalMetrics.ISE);
    fprintf('  ITAE (Integral Time Abs Error):  %.6f\n', finalMetrics.ITAE);
    fprintf('  IAE (Integral Abs Error):        %.6f\n', finalMetrics.IAE);
    fprintf('  Overshoot:                       %.2f %%\n', finalMetrics.Overshoot);
    fprintf('  Undershoot:                      %.2f %%\n', finalMetrics.Undershoot);
    fprintf('  Rise Time (10%%-90%%):            %.3f ms\n', finalMetrics.RiseTime * 1000);
    fprintf('  Settling Time (2%%):              %.3f ms\n', finalMetrics.SettlingTime * 1000);
    fprintf('  Peak Time:                       %.3f ms\n', finalMetrics.PeakTime * 1000);
    fprintf('  Steady-State Error:              %.2f %%\n', finalMetrics.SteadyStateError);
    fprintf('  Peak Value:                      %.3f A\n', finalMetrics.PeakValue);
    fprintf('  Target Value:                    %.3f A\n', finalMetrics.TargetValue);
    if isfield(finalMetrics, 'InitDeviation')
        fprintf('  Init Deviation:                  %.2f %% (Cheat check)\n', finalMetrics.InitDeviation);
    end
    if isfield(finalMetrics, 'CheatPenalty') && finalMetrics.CheatPenalty > 0
        fprintf('  Cheat Penalty Applied:           %.2f\n', finalMetrics.CheatPenalty);
    end
    fprintf('  Control Effort (Smoothness):     %.2f (Lower is better)\n', finalMetrics.ControlEffort);
    fprintf('  Saturation Ratio (>89deg):       %.2f %% (Commercial Safety)\n', finalMetrics.SaturationRatio);
    disp('-----------------------------------------------------------');
    
    writeLog(logFile, sprintf('Final ISE: %.6f, ITAE: %.6f', finalMetrics.ISE, finalMetrics.ITAE));
    writeLog(logFile, sprintf('Overshoot: %.2f%%, Settling: %.3fms', finalMetrics.Overshoot, finalMetrics.SettlingTime*1000));
    
    %% 8. 최종 플롯 생성
    figure('Name', 'Final Optimized Response', 'Position', [100, 100, 1000, 700]);
    
    % 상단: 전류 응답
    subplot(3,1,1);
    plot(t_sim*1000, I_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(t_sim*1000, I_meas, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
    
    % 오버슈트/정착 시간 표시
    if finalMetrics.Overshoot > 0
        plot(finalMetrics.PeakTime*1000, finalMetrics.PeakValue, 'r*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Peak');
    end
    
    grid on;
    xlabel('Time [ms]');
    ylabel('Current [A]');
    title(sprintf('Step Response (Kp=%.4f, Ki=%.4f)', x_best(1), x_best(2)));
    legend('Location', 'southeast');
    xlim([0, SIM_TIME*1000]);
    
    % 중단: 오차
    subplot(3,1,2);
    error_sig = I_ref - I_meas;
    plot(t_sim*1000, error_sig, 'r-', 'LineWidth', 1.5);
    hold on;
    yline(0, 'k--');
    grid on;
    xlabel('Time [ms]');
    ylabel('Error [A]');
    title('Tracking Error');
    xlim([0, SIM_TIME*1000]);
    
    % 하단: 성능 지표 텍스트
    subplot(3,1,3);
    axis off;
    text(0.1, 0.9, sprintf('Performance Summary (Kp=%.4f, Ki=%.4f)', x_best(1), x_best(2)), ...
        'FontSize', 14, 'FontWeight', 'bold');
    text(0.1, 0.7, sprintf('ISE: %.6f    ITAE: %.6f    IAE: %.6f', ...
        finalMetrics.ISE, finalMetrics.ITAE, finalMetrics.IAE), 'FontSize', 11);
    text(0.1, 0.5, sprintf('Overshoot: %.2f%%    Undershoot: %.2f%%', ...
        finalMetrics.Overshoot, finalMetrics.Undershoot), 'FontSize', 11);
    text(0.1, 0.3, sprintf('Rise Time: %.3f ms    Settling Time: %.3f ms', ...
        finalMetrics.RiseTime*1000, finalMetrics.SettlingTime*1000), 'FontSize', 11);
    text(0.1, 0.1, sprintf('Steady-State Error: %.2f%%    Peak: %.3f A', ...
        finalMetrics.SteadyStateError, finalMetrics.PeakValue), 'FontSize', 11);
    
    drawnow;
    
    % 플롯 저장
    plotFile = fullfile(resultDir, sprintf('GA_FinalPlot_%s.fig', timestamp));
    savefig(gcf, plotFile);
    
    pngFile = fullfile(resultDir, sprintf('GA_FinalPlot_%s.png', timestamp));
    saveas(gcf, pngFile);
    
    %% 9. 최종 결과 저장
    finalResult = struct();
    finalResult.Kp = x_best(1);
    finalResult.Ki = x_best(2);
    finalResult.Cost = fval;
    finalResult.Elapsed = elapsed;
    finalResult.Generations = output.generations;
    finalResult.FuncCount = output.funccount;
    finalResult.ExitFlag = exitflag;
    finalResult.Timestamp = timestamp;
    finalResult.RefCurrent = CMD_LEVEL_HIGH;
    finalResult.InitCurrent = CMD_LEVEL_LOW;
    finalResult.StepMagnitude = STEP_MAGNITUDE;
    finalResult.StepTime = STEP_TIME;
    finalResult.SimTime = SIM_TIME;
    finalResult.Metrics = finalMetrics;
    
    % .mat 파일로 저장
    finalFile = fullfile(resultDir, sprintf('GA_Final_%s.mat', timestamp));
    % .mat 파일로 저장
    finalFile = fullfile(resultDir, sprintf('GA_Final_%s.mat', timestamp));
    save(finalFile, 'finalResult', 'x_best', 'fval', 'exitflag', 'output', 't_sim', 'I_meas', 'I_ref', 'Phi_meas');
    
    %% 9-B. [REQ] 주파수 응답 분석 (Bode Plot & Margins)
    disp('>>> Performing Frequency Response Analysis (Linearization)...');
    try
        % 1. Simulink Linearization
        % 모델 입출력 포트를 기준으로 선형화 시도
        % (주의: DAB 같은 스위칭 컨버터는 평균 모델이 아니면 선형화가 부정확할 수 있음)
        % 여기서는 시스템이 동작점(Steady State)에 도달했다고 가정하고 선형화
        
        % 파라미터 업데이트
        assignin('base', 'Kp_ext', x_best(1));
        assignin('base', 'Ki_ext', x_best(2));
        
        io = getlinio(modelName); 
        if isempty(io)
            % IO가 명시되지 않은 경우, 주요 신호로 추정 시도
            % (사용자가 모델에 Linear Analysis Point를 설정했기를 기대)
            fprintf('Warning: No Linear Analysis Points found in model. Using default snapshot linearization.\n');
        end
        
        % Snapshot time: 시뮬레이션 끝 부분(Steady State)
        op = findop(modelName, SIM_TIME); 
        sys_ol = linearize(modelName, op); 
        
        % 2. Bode Plot
        bodeFig = figure('Name', 'Frequency Response Analysis', 'Position', [150, 150, 800, 600]);
        margin(sys_ol);
        grid on; 
        title('Open-Loop Bode Plot (Optimized Controller)');
        
        % 3. Margins
        [Gm, Pm, Wcg, Wcp] = margin(sys_ol);
        Gm_dB = 20*log10(Gm);
        
        fprintf('\n[Stability Analysis]\n');
        fprintf('  Gain Margin: %.2f dB (at %.2f rad/s)\n', Gm_dB, Wcg);
        fprintf('  Phase Margin: %.2f deg (at %.2f rad/s)\n', Pm, Wcp);
        
        if Pm >= 45
            fprintf('  >>> Result: STABLE (Phase Margin >= 45 deg)\n');
        else
            fprintf('  >>> Result: UNSTABLE (Phase Margin < 45 deg) - CHECK GAINS\n');
        end
        
        savefig(bodeFig, fullfile(resultDir, sprintf('GA_Bode_%s.fig', timestamp)));
    catch ME
        fprintf('  [Skipped] Linearization failed: %s\n', ME.message);
    end
    
    assignin('base', 'GA_Result', finalResult);
    
    fprintf('\n>>> Results saved to:\n');
    fprintf('    - Workspace:  GA_Result\n');
    fprintf('    - MAT File:   %s\n', finalFile);
    fprintf('    - Metrics:    %s\n', metricsFile);
    fprintf('    - Log:        %s\n', logFile);
    fprintf('    - Plot:       %s\n', pngFile);
    
    writeLog(logFile, sprintf('All results saved. Completed: %s', datestr(now)));
    
    disp(' ');
    disp('>>> Optimization complete! All results and metrics saved.');
end

%% =========================================================================
%  비용 함수 (성능 지표 계산 + CSV 저장)
%  =========================================================================
function cost = evaluateCostWithMetrics(params, modelName, RefCurrentTS, simTime, stepTime, metricsFile, figHandle, DQueue)
    % [CHECK STOP] - 안전 종료 (GUI 접근은 Client에서만 수행)
    if ~isempty(figHandle) && isempty(getCurrentTask())
        if isvalid(figHandle)
            drawnow limitrate;
            if strcmp(get(figHandle, 'Tag'), 'STOP_NOW')
                error('UserStop:Request', 'Optimization stopped by user.');
            end
        end
    end
    
    % [PARALLEL STABILITY] Worker별 독립 빌드 폴더 설정 (충돌 방지)
    persistent isWorkerConfigured
    if isempty(isWorkerConfigured)
        task = getCurrentTask();
        if ~isempty(task)
            myTempDir = fullfile(tempdir, sprintf('GADAB_Worker_%d', task.ID));
            if ~exist(myTempDir, 'dir'), mkdir(myTempDir); end
            try
                Simulink.fileGenControl('set', 'CacheFolder', myTempDir, 'CodeGenFolder', myTempDir, 'createDir', true);
            catch
            end
        end
        isWorkerConfigured = true;
    end

    persistent evalCount
    if isempty(evalCount)
        evalCount = 0;
    end
    evalCount = evalCount + 1;
    
    Kp = params(1);
    Ki = params(2);
    
    % [debug] Worker Log (First few only)
    % [debug] Worker Log (First few only)
    task = getCurrentTask();
    workerID = 0;
    if ~isempty(task), workerID = task.ID; end
    
    % Always print for debugging the "Identical Cost" issue
    fprintf('[Worker %d] Simulating: Kp=%.4f, Ki=%.4f (Eval #%d)\n', workerID, Kp, Ki, evalCount);

    % [CACHE CHECK]
    cacheKey = sprintf('%.4f_%.4f', Kp, Ki);
    cachedResult = [];
    isCached = false;
    
    try
        GA_Cache_Map = evalin('base', 'GA_Cache_Map');
        if isKey(GA_Cache_Map, cacheKey)
            cachedResult = GA_Cache_Map(cacheKey);
            cost = cachedResult.Cost;
            metrics = cachedResult.Metrics;
            isCached = true;
            
            % [DATA QUEUE] 캐시된 데이터도 로깅을 위해 Master로 전송
            if nargin >= 8 && ~isempty(DQueue)
                logData = struct('EvalNum', evalCount, 'Kp', Kp, 'Ki', Ki, 'Cost', cost, ...
                                 'Metrics', metrics, 'IsCached', true, 'IsValid', true);
                send(DQueue, logData);
            end
            return;
        end
    catch
        % Worker workspace handling
    end
    
    % [CRITICAL FIX] Ensure Model is Loaded
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end

    % [FUNDAMENTAL FIX] Variable Injection Strategy (SimulationInput ONLY)
    % Parallel workers run in their own isolated MATLAB sessions. 
    % Relying on 'assignin' to Base/Model workspace is flaky.
    % The only 100% reliable method is to pack EVERYTHING upon the SimulationInput object.
    
    % Extend time range
    t_gain = [-1, simTime + 1.0];
    Ref_Kp_ext_Val = timeseries([Kp, Kp], t_gain);
    Ref_Ki_ext_Val = timeseries([Ki, Ki], t_gain);
    Ref_VH_Val = timeseries([380, 380], t_gain); 

    try
        % 1. Create Simulation Input Object
        in = Simulink.SimulationInput(modelName);
        
        % 2. Set Model Configuration
        in = in.setModelParameter('StopTime', num2str(simTime));
        in = in.setModelParameter('SimulationMode', 'normal'); 
        in = in.setModelParameter('FastRestart', 'off'); 
        
        % 3. Mask & Workspace Variables (Crucial Step)
        % These variables are injected into the model's "Global Workspace" overlay
        in = in.setVariable('Ref_Kp_ext', Ref_Kp_ext_Val);
        in = in.setVariable('Ref_Ki_ext', Ref_Ki_ext_Val);
        in = in.setVariable('Ref_I_ess_cmd', RefCurrentTS);
        in = in.setVariable('Ref_VH', Ref_VH_Val);
        
        % [CRITICAL] Control Gains (Optimized + Fixed)
        in = in.setVariable('Kp_ext', Kp);
        in = in.setVariable('Ki_ext', Ki);
        in = in.setVariable('Kpv', 0.1);  % UPS Voltage P Gain
        in = in.setVariable('Kiv', 10.0); % UPS Voltage I Gain
        
        % [CRITICAL] Circuit Parameters (Must match Create_Scenario.m)
        in = in.setVariable('V_init_LV', 96);
        in = in.setVariable('V_batt', 96);
        in = in.setVariable('C_HV', 1000e-6);
        in = in.setVariable('R_ESR_HV', 0.05);
        in = in.setVariable('R_ESR_LV', 0.01);
        in = in.setVariable('R_batt_int', 0.15);
        
        % [FALLBACK] Direct Base Workspace Injection for Workers
        % Some Simulink masks/callbacks might peek at 'base' directly.
        assignin('base', 'Kp_ext', Kp);
        assignin('base', 'Ki_ext', Ki);
        assignin('base', 'Kpv', 0.1);
        assignin('base', 'Kiv', 10.0);
        assignin('base', 'V_batt', 96);
        assignin('base', 'C_HV', 1000e-6);
        assignin('base', 'R_ESR_HV', 0.05);
        
        % 4. Run Simulation
        % 'parsim' is better, but inside 'parfor' or GA worker, 'sim' on the input object is correct.
        simOut = sim(in);
        
        % 5. Check Output
        if isempty(simOut) || (isprop(simOut,'ErrorMessage') && ~isempty(simOut.ErrorMessage))
             error('Simulation returned empty or error.');
        end
        
        [t, I_meas, phi, v_high, i_ind] = extractSimulationSignals(simOut);
        I_ref = resampleReference(RefCurrentTS, t);
        
    catch ME
        errMsg = ME.message;
        if isprop(ME, 'cause') && ~isempty(ME.cause)
             errMsg = [errMsg ' Cause: ' ME.cause{1}.message];
        end
        fprintf('>>> Simulation Failed (Kp=%.2f, Ki=%.2f): %s\n', Kp, Ki, errMsg);
        cost = 1e6;
        
        % 실패 로그 전송
        if nargin >= 8 && ~isempty(DQueue)
             % 빈 메트릭
             dummyMetrics = struct('ISE',1e6, 'ITAE',1e6, 'IAE',1e6, 'Overshoot',0, 'Undershoot',0, ...
                 'RiseTime',0, 'SettlingTime',0, 'PeakTime',0, 'SteadyStateError',0, ...
                 'PeakValue',0, 'TargetValue',0, 'RippleStd',0, 'SaturationRatio',0, ...
                 'VoltageRobustness',0, 'ControlEffort',0, 'PhaseOvershoot',0, ...
                 'PhaseSettlingTime',0, 'PhaseOscillation',0, 'InitValue',0, 'StepSize',0);
             
             logData = struct('EvalNum', evalCount, 'Kp', Kp, 'Ki', Ki, 'Cost', cost, ...
                             'Metrics', dummyMetrics, 'IsCached', false, 'IsValid', false);
             send(DQueue, logData);
        end
        return;
    end
    
    % 데이터 유효성 검사
    if isempty(t) || length(t) < 10
        cost = 1e6;
        return;
    end
    
    % 크기 맞추기
    n = min(length(t), min(length(I_meas), length(I_ref)));
    t = t(1:n); I_meas = I_meas(1:n); I_ref = I_ref(1:n);
    if length(phi) >= n, phi = phi(1:n); else, phi = [phi; zeros(n-length(phi), 1)]; end
    
    % 성능 지표 계산
    metrics = calculatePerformanceMetrics(t, I_meas, I_ref, stepTime, phi, v_high, i_ind);
    
    % 비용 계산 (복사본 유지)
    if metrics.PhaseOvershoot > 5.0
        c_ph_ovs = 5.0 * ((metrics.PhaseOvershoot - 5.0)^2);
    else
        c_ph_ovs = 1.0 * metrics.PhaseOvershoot;
    end

    baseCost = 2.0 * (metrics.ISE / 0.5) + ...
               1.5 * (metrics.ITAE / 0.05) + ...
               5.0 * (metrics.ControlEffort / 0.1) + ...   
               10.0 * (metrics.PhaseOscillation / 0.1) + ... 
               2.0 * (metrics.PhaseSettlingTime / 0.05) + ...
               c_ph_ovs + ...
               3.0 * (metrics.SaturationRatio / 5) + ...
               1.0 * metrics.VoltageRobustness + ...
               1.5 * (metrics.SteadyStateError / 5);

    TRACKING_ERR_THRESHOLD = 15.0; 
    cost_mandatory = 0;
    if metrics.SteadyStateError > TRACKING_ERR_THRESHOLD
        cost_mandatory = 50000 + (metrics.SteadyStateError - TRACKING_ERR_THRESHOLD)^2 * 100;
    end

    if isfield(metrics, 'CheatPenalty')
        cost = baseCost + metrics.CheatPenalty + cost_mandatory;
    else
        cost = baseCost + cost_mandatory;
    end
    
    if isnan(cost) || isinf(cost), cost = 1e6; end
    
    % [DEBUG] Worker First Run Summary (Verification)
    if evalCount == 1
        task = getCurrentTask();
        workerID = 0;
        if ~isempty(task), workerID = task.ID; end
        
        fprintf('\n[Worker %d First Run Summary]\n', workerID);
        fprintf('  Params: Kp=%.4f, Ki=%.4f\n', Kp, Ki);
        fprintf('  Cost:   %.6f\n', cost);
        fprintf('  Metrics: ISE=%.4f, Overshoot=%.2f%%, Settling=%.4fs, SS_Err=%.2f%%, Sat=%.1f%%\n', ...
            metrics.ISE, metrics.Overshoot, metrics.SettlingTime, metrics.SteadyStateError, metrics.SaturationRatio);
        fprintf('  ------------------------------------------------------------\n');
    end
    
    % [CACHE UPDATE]
    try
        resultStruct = struct();
        resultStruct.Cost = cost;
        resultStruct.Metrics = metrics;
        resultStruct.Timestamp = now;
        
        GA_Cache_Map = evalin('base', 'GA_Cache_Map');
        GA_Cache_Map(cacheKey) = resultStruct;
        assignin('base', 'GA_Cache_Map', GA_Cache_Map);
    catch
    end
    
    % [DATA QUEUE] 결과 전송 (Master Logging)
    if nargin >= 8 && ~isempty(DQueue)
        logData = struct('EvalNum', evalCount, 'Kp', Kp, 'Ki', Ki, 'Cost', cost, ...
                         'Metrics', metrics, 'IsCached', isCached, 'IsValid', true);
        send(DQueue, logData);
    end
    
    % [LEGACY] Single Core Logging (fallback)
    if isempty(DQueue) && isempty(getCurrentTask())
        saveMetricsToCSV(metricsFile, evalCount, Kp, Ki, cost, metrics, true, isCached);
    end
end


%  [MOVED] calculateAllMetrics -> calculatePerformanceMetrics.m (in GA_Utils)

%% =========================================================================
%  전체 성능 평가 (최종 검증용)
%  =========================================================================
function [cost, metrics, t, I_meas, I_ref, phi] = evaluateWithFullMetrics(params, modelName, RefCurrentTS, simTime, stepTime)
    Kp = params(1);
    Ki = params(2);
    
    
    % [FUNDAMENTAL FIX] Variable Injection for Final Verification
    t_gain = [0, simTime];
    Ref_Kp_ext = timeseries([Kp, Kp], t_gain);
    Ref_Ki_ext = timeseries([Ki, Ki], t_gain);
    Ref_VH_TS = timeseries([380, 380], [0, simTime]);

    assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
    assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
    assignin('base', 'Ref_I_ess_cmd', RefCurrentTS);
    assignin('base', 'Ref_VH', Ref_VH_TS);
    assignin('base', 'Kp_ext', Kp);
    assignin('base', 'Ki_ext', Ki);
    assignin('base', 'V_init_LV', 96);
    assignin('base', 'V_batt', 96);

    in = Simulink.SimulationInput(modelName);
    in = in.setModelParameter('StopTime', num2str(simTime));
    in = in.setModelParameter('SimulationMode', 'accelerator');
    
    in = in.setVariable('Ref_Kp_ext', Ref_Kp_ext);
    in = in.setVariable('Ref_Ki_ext', Ref_Ki_ext);
    in = in.setVariable('Ref_I_ess_cmd', RefCurrentTS);
    in = in.setVariable('Ref_VH', Ref_VH_TS);
    
    % [CRITICAL] Control Gains
    in = in.setVariable('Kp_ext', Kp);
    in = in.setVariable('Ki_ext', Ki);
    in = in.setVariable('Kpv', 0.1);
    in = in.setVariable('Kiv', 10.0);
    
    % [CRITICAL] Circuit Parameters
    in = in.setVariable('V_init_LV', 96);
    in = in.setVariable('V_batt', 96);
    in = in.setVariable('C_HV', 1000e-6);
    in = in.setVariable('R_ESR_HV', 0.05);
    in = in.setVariable('R_ESR_LV', 0.01);
    in = in.setVariable('R_batt_int', 0.15);
    
    simOut = sim(in);
    
    [t, I_meas, phi, v_high, i_ind] = extractSimulationSignals(simOut);
    I_ref = resampleReference(RefCurrentTS, t);
    
    n = min(length(t), min(length(I_meas), length(I_ref)));
    t = t(1:n); I_meas = I_meas(1:n); I_ref = I_ref(1:n);
    
    % [FIX] Ensure all vectors are columns and match length n
    t = t(:); I_meas = I_meas(:); I_ref = I_ref(:);
    phi = phi(:); v_high = v_high(:); i_ind = i_ind(:);
    
    % Pad or Crop Auxiliary Signals
    if length(phi) >= n, phi = phi(1:n); else, phi = [phi; zeros(n-length(phi), 1)]; end
    if length(v_high) >= n, v_high = v_high(1:n); else, v_high = [v_high; zeros(n-length(v_high), 1)]; end
    if length(i_ind) >= n, i_ind = i_ind(1:n); else, i_ind = [i_ind; zeros(n-length(i_ind), 1)]; end
    
    metrics = calculatePerformanceMetrics(t, I_meas, I_ref, stepTime, phi, v_high, i_ind);
    
    % 비용 계산 (evaluateCostWithMetrics와 동일)
    % 비용 계산 (Reflected New Logic)
    % Quadratic Overshoot for Phase
    if metrics.PhaseOvershoot > 5.0
        c_ph_ovs = 5.0 * ((metrics.PhaseOvershoot - 5.0)^2);
    else
        c_ph_ovs = 1.0 * metrics.PhaseOvershoot;
    end

    baseCost = 2.0 * (metrics.ISE / 0.5) + ...
               1.5 * (metrics.ITAE / 0.05) + ...
               5.0 * (metrics.ControlEffort / 0.1) + ...   % Smoothness
               10.0 * (metrics.PhaseOscillation / 0.1) + ... % Stability
               2.0 * (metrics.PhaseSettlingTime / 0.05) + ...
               c_ph_ovs + ...
               3.0 * (metrics.SaturationRatio / 5) + ...
               1.5 * (metrics.SteadyStateError / 5);


    
     % [MANDATORY CONDITION] Tracking Requirement (동일 로직 적용)
    TRACKING_ERR_THRESHOLD = 15.0; 
    cost_mandatory = 0;
    if metrics.SteadyStateError > TRACKING_ERR_THRESHOLD
        cost_mandatory = 50000 + (metrics.SteadyStateError - TRACKING_ERR_THRESHOLD)^2 * 100;
        fprintf('  [Warning] Final Result failed mandatory tracking check (Error: %.2f%%)\n', metrics.SteadyStateError);
    end

    if isfield(metrics, 'CheatPenalty')
        cost = baseCost + metrics.CheatPenalty + cost_mandatory;
    else
        cost = baseCost + cost_mandatory;
    end
end

%  [MOVED] saveMetricsToCSV -> saveMetricsToCSV.m (in GA_Utils)

%% =========================================================================
%  GA OutputFcn: 각 세대마다 플롯 및 터미널 출력
%  =========================================================================
function [state, options, optchanged] = gaOutputFcn(options, state, flag, ...
    saveFile, logFile, modelName, RefCurrentTS, simTime, stepTime, stepMagnitude, figHandle, ax1, ax2, ax3, metricsFile, maxGen, cacheFile)
    
    optchanged = false;
    
    persistent costHistory bestCostEver bestParamsEver
    
    if strcmp(flag, 'init')
        costHistory = [];
        bestCostEver = inf;
        bestParamsEver = [0, 0];
        set(figHandle, 'UserData', ''); % Stop 플래그 초기화
    end
    
    % [Check Stop Button] 버튼 눌림 확인
    if isvalid(figHandle) && strcmp(get(figHandle, 'UserData'), 'STOP_REQUESTED')
        state.StopFlag = 'y'; % GA 종료 시그널
        fprintf('\n>>> 🛑 사용자에 의해 STOP 버튼이 눌렸습니다. 최적화를 종료합니다...\n');
        writeLog(logFile, 'Optimization terminated by user (STOP button).');
    end
    
    if strcmp(flag, 'iter')
        gen = state.Generation;
        
        % 현재 세대의 최적 개체
        [minScore, minIdx] = min(state.Score);
        bestParams = state.Population(minIdx, :);
        
        % 전체 최적 갱신
        if minScore < bestCostEver
            bestCostEver = minScore;
            bestParamsEver = bestParams;
        end
        
        costHistory = [costHistory; minScore];
        
        % 터미널에 상세 정보 출력
        fprintf('\n');
        fprintf('┌─────────────────────────────────────────────────────────────┐\n');
        fprintf('│  Generation %3d                                             │\n', gen);
        fprintf('├─────────────────────────────────────────────────────────────┤\n');
        fprintf('│  Best in Gen:  Kp = %8.4f,  Ki = %8.2f                │\n', bestParams(1), bestParams(2));
        fprintf('│  Gen Cost:     %.6f                                     │\n', minScore);
        fprintf('│  Best Overall: Kp = %8.4f,  Ki = %8.2f  (Cost: %.4f) │\n', ...
            bestParamsEver(1), bestParamsEver(2), bestCostEver);
        fprintf('└─────────────────────────────────────────────────────────────┘\n');
        
        % [VISUAL] 터미널에는 텍스트 출력, 그래프 창에는 점만 찍기 (빠르게)
        % (OutputFcn에서 시뮬레이션을 다시 돌려서 그래프 그리는 건 느리므로 생략)
        % 대신 Cost Trend와 Parameter Space 점은 업데이트
        
        if isvalid(ax2)
            plot(ax2, 0:length(costHistory)-1, costHistory, 'b-o', 'LineWidth', 1.5);
            title(ax2, sprintf('Cost History (Gen %d)', gen));
            grid(ax2, 'on');
        end
        
        if isvalid(ax3)
            plot(ax3, state.Population(:,1), state.Population(:,2), 'k.', 'MarkerSize', 8); % 전체 개체군
            plot(ax3, bestParams(1), bestParams(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Best
            title(ax3, sprintf('Search Space (Red: Gen Best)'));
        end
        
        drawnow limitrate; % 화면 갱신 강제

        
        % 로그에 기록
        writeLog(logFile, sprintf('Gen %d: Kp=%.4f, Ki=%.2f, Cost=%.6f', gen, bestParams(1), bestParams(2), minScore));
        
        % Progress 파일 저장
        saveData = struct();
        saveData.Generation = gen;
        saveData.BestFval = bestCostEver;
        saveData.BestParams = bestParamsEver;
        saveData.Population = state.Population;
        saveData.Score = state.Score;
        saveData.CostHistory = costHistory;
        saveData.Timestamp = datestr(now);
        save(saveFile, 'saveData');
        
        % [CACHE SAVE] 매 세대마다 캐시 저장
        try
            GA_Cache_Map = evalin('base', 'GA_Cache_Map');
            save(cacheFile, 'GA_Cache_Map');
        catch
        end
    end
    
    if strcmp(flag, 'done')
        writeLog(logFile, 'GA optimization finished.');
        fprintf('\n>>> GA optimization loop finished.\n');
        
        try
            GA_Cache_Map = evalin('base', 'GA_Cache_Map');
            save(cacheFile, 'GA_Cache_Map');
        catch
        end
    end
end

%  [MOVED] extractSignals, resampleReference, writeLog, calculateAllMetrics, saveMetricsToCSV
%  These functions have been moved to the 'GA_Utils' folder to clean up the code.
%  See: calculatePerformanceMetrics.m, extractSimulationSignals.m, etc.
%  =========================================================================
