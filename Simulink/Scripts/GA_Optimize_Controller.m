%% GA_Optimize_Controller.m
% DAB Converter Controller Optimization using Genetic Algorithm (GA)
% Target: Optimize Kp_ext, Ki_ext for ESS Current Control
% Updated: 2026-01-23
%
% [주요 기능]
% - 매 세대마다 터미널에 상세 성능 지표 출력
% - 매 세대마다 전류 그래프 실시간 플롯
% - 모든 성능 지표를 CSV/MAT 파일로 저장
% - 자동 저장 및 재개 기능

function GA_Optimize_Controller()
    %% 0. 환경 설정
    clear; clc; close all;
    
    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath), scriptPath = pwd; end
    addpath(scriptPath); 
    addpath(fullfile(scriptPath, '..'));
    addpath(fullfile(scriptPath, '..', 'Modules'));
    addpath(fullfile(scriptPath, '..', 'Data'));
    
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
    fprintf(csvID, 'PeakValue,TargetValue,Valid\n');
    fclose(csvID);
    
    %% 1. 시나리오 및 모델 초기화
    disp('===========================================');
    disp('  DAB Controller GA Optimization');
    disp('  [Enhanced with Real-time Monitoring]');
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
    
    %% 2. ===== 하드코딩된 지령치 (DO NOT DELETE) =====
    STEP_TIME = 0.02;      % Step 발생 시점 [s]
    REF_CURRENT = 10.0;    % 목표 전류 [A]
    SIM_TIME = 0.05;       % 시뮬레이션 시간 [s]
    
    % 지령치 타임시리즈 생성
    t_cmd = [0, STEP_TIME - 1e-6, STEP_TIME, SIM_TIME];
    v_cmd = [0, 0, REF_CURRENT, REF_CURRENT];
    RefCurrentTS = timeseries(v_cmd(:), t_cmd(:));
    
    fprintf('>>> Reference: Step to %.1fA at t=%.3fs\n', REF_CURRENT, STEP_TIME);
    writeLog(logFile, sprintf('Reference: Step to %.1fA at t=%.3fs', REF_CURRENT, STEP_TIME));
    
    %% 3. 실시간 플롯 Figure 생성
    figHandle = figure('Name', 'GA Real-time Monitor', 'Position', [50, 50, 1200, 600]);
    
    % 왼쪽: 전류 응답 플롯
    ax1 = subplot(1,2,1);
    hold(ax1, 'on');
    grid(ax1, 'on');
    xlabel(ax1, 'Time [ms]');
    ylabel(ax1, 'Current [A]');
    title(ax1, 'Current Response (Best Individual)');
    xlim(ax1, [0, SIM_TIME*1000]);
    
    % 오른쪽: Cost 히스토리 플롯
    ax2 = subplot(1,2,2);
    hold(ax2, 'on');
    grid(ax2, 'on');
    xlabel(ax2, 'Evaluation #');
    ylabel(ax2, 'Cost');
    title(ax2, 'Cost History');
    
    drawnow;
    
    %% 4. GA 설정
    nVars = 2;  % [Kp, Ki]
    lb = [0.01, 1.0];
    ub = [50.0, 5000.0];
    
    % 전역 변수로 평가 카운터 및 데이터 저장
    evalData = struct();
    evalData.count = 0;
    evalData.bestCost = inf;
    evalData.bestParams = [0, 0];
    evalData.costHistory = [];
    evalData.allMetrics = [];
    
    options = optimoptions('ga', ...
        'PopulationSize', 10, ...
        'MaxGenerations', 20, ...
        'Display', 'iter', ...
        'UseParallel', false, ...
        'PlotFcn', [], ...  % 기본 플롯 비활성화 (우리 플롯 사용)
        'OutputFcn', @(options, state, flag) gaOutputFcn(options, state, flag, ...
            saveFile, logFile, modelName, RefCurrentTS, SIM_TIME, STEP_TIME, ...
            figHandle, ax1, ax2, metricsFile));
    
    writeLog(logFile, sprintf('GA Settings: PopSize=%d, MaxGen=%d', 10, 20));
    writeLog(logFile, sprintf('Search bounds: Kp=[%.2f, %.2f], Ki=[%.2f, %.2f]', lb(1), ub(1), lb(2), ub(2)));
    
    %% 5. GA 실행
    disp(' ');
    disp('>>> Starting GA Optimization...');
    disp('>>> 매 세대마다 Best Individual의 성능 지표가 출력됩니다.');
    disp(' ');
    writeLog(logFile, 'Starting GA optimization...');
    
    % 비용 함수
    costFcn = @(x) evaluateCostWithMetrics(x, modelName, RefCurrentTS, SIM_TIME, STEP_TIME, metricsFile);
    
    t_start = tic;
    [x_best, fval, exitflag, output] = ga(costFcn, nVars, [], [], [], [], lb, ub, [], options);
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
    
    %% 7. 최종 검증 및 상세 성능 분석
    disp(' ');
    disp('>>> Running final verification with detailed analysis...');
    
    [finalCost, finalMetrics, t_sim, I_meas, I_ref] = evaluateWithFullMetrics(...
        x_best, modelName, RefCurrentTS, SIM_TIME, STEP_TIME);
    
    % 최종 성능 지표 출력
    disp('-----------------------------------------------------------');
    disp('              FINAL PERFORMANCE METRICS                    ');
    disp('-----------------------------------------------------------');
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
    finalResult.RefCurrent = REF_CURRENT;
    finalResult.StepTime = STEP_TIME;
    finalResult.SimTime = SIM_TIME;
    finalResult.Metrics = finalMetrics;
    
    % .mat 파일로 저장
    finalFile = fullfile(resultDir, sprintf('GA_Final_%s.mat', timestamp));
    save(finalFile, 'finalResult', 'x_best', 'fval', 'exitflag', 'output', 't_sim', 'I_meas', 'I_ref');
    
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
function cost = evaluateCostWithMetrics(params, modelName, RefCurrentTS, simTime, stepTime, metricsFile)
    persistent evalCount
    if isempty(evalCount)
        evalCount = 0;
    end
    evalCount = evalCount + 1;
    
    Kp = params(1);
    Ki = params(2);
    
    % 파라미터 할당
    t_gain = [0, simTime];
    Ref_Kp_ext = timeseries([Kp, Kp], t_gain);
    Ref_Ki_ext = timeseries([Ki, Ki], t_gain);
    
    assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
    assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
    assignin('base', 'Kp_ext', Kp);
    assignin('base', 'Ki_ext', Ki);
    
    % 시뮬레이션 실행
    try
        simOut = sim(modelName, 'StopTime', num2str(simTime), ...
                     'SimulationMode', 'accelerator', ...
                     'FastRestart', 'off');
        [t, I_meas] = extractCurrentSignal(simOut);
        I_ref = resampleReference(RefCurrentTS, t);
        validSim = true;
    catch
        cost = 1e6;
        saveMetricsToCSV(metricsFile, evalCount, Kp, Ki, cost, [], false);
        return;
    end
    
    % 데이터 유효성 검사
    if isempty(t) || length(t) < 10
        cost = 1e6;
        saveMetricsToCSV(metricsFile, evalCount, Kp, Ki, cost, [], false);
        return;
    end
    
    % 크기 맞추기
    n = min(length(t), min(length(I_meas), length(I_ref)));
    t = t(1:n); I_meas = I_meas(1:n); I_ref = I_ref(1:n);
    t = t(:); I_meas = I_meas(:); I_ref = I_ref(:);
    
    % 성능 지표 계산
    metrics = calculateAllMetrics(t, I_meas, I_ref, stepTime);
    
    % 비용 계산 (가중합)
    cost = 2.0 * (metrics.ISE / 50) + ...
           1.0 * (metrics.ITAE / 0.5) + ...
           3.0 * (metrics.Overshoot / 100) + ...
           1.5 * (metrics.SettlingTime / 0.03);
    
    if isnan(cost) || isinf(cost)
        cost = 1e6;
    end
    
    % === 첫 번째 개체 평가 시에만 시험 출력 ===
    if evalCount == 1
        fprintf('\n');
        fprintf('┌─────────────────────────────────────────────────────────────┐\n');
        fprintf('│           [시험 출력] 첫 번째 개체 평가 결과                │\n');
        fprintf('├─────────────────────────────────────────────────────────────┤\n');
        fprintf('│  Parameters:  Kp = %8.4f,  Ki = %8.2f                  │\n', Kp, Ki);
        fprintf('│  Cost:        %.6f                                       │\n', cost);
        fprintf('├─────────────────────────────────────────────────────────────┤\n');
        fprintf('│  Performance Metrics:                                       │\n');
        fprintf('│    ISE:             %.6f                                  │\n', metrics.ISE);
        fprintf('│    ITAE:            %.6f                                  │\n', metrics.ITAE);
        fprintf('│    IAE:             %.6f                                  │\n', metrics.IAE);
        fprintf('│    Overshoot:       %.2f %%                                 │\n', metrics.Overshoot);
        fprintf('│    Undershoot:      %.2f %%                                 │\n', metrics.Undershoot);
        fprintf('│    Rise Time:       %.3f ms                                │\n', metrics.RiseTime * 1000);
        fprintf('│    Settling Time:   %.3f ms                                │\n', metrics.SettlingTime * 1000);
        fprintf('│    Peak Time:       %.3f ms                                │\n', metrics.PeakTime * 1000);
        fprintf('│    SS Error:        %.2f %%                                 │\n', metrics.SteadyStateError);
        fprintf('│    Peak Value:      %.3f A                                 │\n', metrics.PeakValue);
        fprintf('│    Target Value:    %.3f A                                 │\n', metrics.TargetValue);
        fprintf('└─────────────────────────────────────────────────────────────┘\n');
        fprintf('>>> 이후부터는 세대(Generation)별로 Best Individual만 출력됩니다.\n\n');
    end
    
    % CSV에 저장
    saveMetricsToCSV(metricsFile, evalCount, Kp, Ki, cost, metrics, true);
end

%% =========================================================================
%  상세 성능 지표 계산 함수
%  =========================================================================
function metrics = calculateAllMetrics(t, I_meas, I_ref, stepTime)
    metrics = struct();
    
    error_sig = I_ref - I_meas;
    
    % Step 시점 검출
    [~, step_idx] = max(abs(diff(I_ref)));
    t_step = t(step_idx);
    idx_after = t > t_step;
    t_after = t(idx_after);
    I_after = I_meas(idx_after);
    
    % Target 값
    if any(idx_after)
        target = mean(I_ref(idx_after));
    else
        target = max(I_ref);
    end
    metrics.TargetValue = target;
    
    % 1. ISE (Integral Squared Error)
    metrics.ISE = trapz(t, error_sig.^2);
    
    % 2. ITAE (Integral Time Absolute Error)
    metrics.ITAE = trapz(t, t .* abs(error_sig));
    
    % 3. IAE (Integral Absolute Error)
    metrics.IAE = trapz(t, abs(error_sig));
    
    % 4. Overshoot
    metrics.Overshoot = 0;
    metrics.PeakValue = 0;
    metrics.PeakTime = 0;
    if abs(target) > 0.1 && any(idx_after)
        [peak, peakIdx] = max(I_after);
        metrics.PeakValue = peak;
        metrics.PeakTime = t_after(peakIdx);
        if peak > target
            metrics.Overshoot = (peak - target) / target * 100;
        end
    end
    
    % 5. Undershoot (최소값이 0 이하로 내려가는 경우)
    metrics.Undershoot = 0;
    if any(idx_after)
        minVal = min(I_after);
        if minVal < 0
            metrics.Undershoot = abs(minVal) / target * 100;
        end
    end
    
    % 6. Rise Time (10% ~ 90%)
    metrics.RiseTime = 0;
    if abs(target) > 0.1 && any(idx_after)
        val10 = 0.1 * target;
        val90 = 0.9 * target;
        
        idx10 = find(I_meas >= val10, 1, 'first');
        idx90 = find(I_meas >= val90, 1, 'first');
        
        if ~isempty(idx10) && ~isempty(idx90) && idx90 > idx10
            metrics.RiseTime = t(idx90) - t(idx10);
        end
    end
    
    % 7. Settling Time (2% band)
    metrics.SettlingTime = 0;
    if abs(target) > 0.1 && any(idx_after)
        tol = 0.02 * abs(target);
        within = abs(I_after - target) <= tol;
        if ~all(within)
            last_out = find(~within, 1, 'last');
            metrics.SettlingTime = t_after(last_out) - t_step;
        end
    end
    
    % 8. Steady-State Error (마지막 10% 데이터 평균)
    metrics.SteadyStateError = 0;
    if any(idx_after)
        n_after = length(I_after);
        n_ss = max(1, round(n_after * 0.1));
        I_ss = mean(I_after(end-n_ss+1:end));
        if abs(target) > 0.1
            metrics.SteadyStateError = abs(target - I_ss) / target * 100;
        end
    end
end

%% =========================================================================
%  전체 성능 평가 (최종 검증용)
%  =========================================================================
function [cost, metrics, t, I_meas, I_ref] = evaluateWithFullMetrics(params, modelName, RefCurrentTS, simTime, stepTime)
    Kp = params(1);
    Ki = params(2);
    
    t_gain = [0, simTime];
    Ref_Kp_ext = timeseries([Kp, Kp], t_gain);
    Ref_Ki_ext = timeseries([Ki, Ki], t_gain);
    
    assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
    assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
    assignin('base', 'Kp_ext', Kp);
    assignin('base', 'Ki_ext', Ki);
    
    simOut = sim(modelName, 'StopTime', num2str(simTime), 'SimulationMode', 'accelerator');
    [t, I_meas] = extractCurrentSignal(simOut);
    I_ref = resampleReference(RefCurrentTS, t);
    
    n = min(length(t), min(length(I_meas), length(I_ref)));
    t = t(1:n); I_meas = I_meas(1:n); I_ref = I_ref(1:n);
    t = t(:); I_meas = I_meas(:); I_ref = I_ref(:);
    
    metrics = calculateAllMetrics(t, I_meas, I_ref, stepTime);
    
    cost = 2.0 * (metrics.ISE / 50) + ...
           1.0 * (metrics.ITAE / 0.5) + ...
           3.0 * (metrics.Overshoot / 100) + ...
           1.5 * (metrics.SettlingTime / 0.03);
end

%% =========================================================================
%  CSV 저장 함수
%  =========================================================================
function saveMetricsToCSV(metricsFile, evalNum, Kp, Ki, cost, metrics, valid)
    csvID = fopen(metricsFile, 'a');
    if csvID > 0
        if valid && ~isempty(metrics)
            fprintf(csvID, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n', ...
                evalNum, Kp, Ki, cost, ...
                metrics.ISE, metrics.ITAE, metrics.IAE, ...
                metrics.Overshoot, metrics.Undershoot, ...
                metrics.RiseTime*1000, metrics.SettlingTime*1000, metrics.PeakTime*1000, ...
                metrics.SteadyStateError, metrics.PeakValue, metrics.TargetValue, 1);
        else
            fprintf(csvID, '%d,%.6f,%.6f,%.6f,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,0\n', ...
                evalNum, Kp, Ki, cost);
        end
        fclose(csvID);
    end
end

%% =========================================================================
%  GA OutputFcn: 각 세대마다 플롯 및 터미널 출력
%  =========================================================================
function [state, options, optchanged] = gaOutputFcn(options, state, flag, ...
    saveFile, logFile, modelName, RefCurrentTS, simTime, stepTime, figHandle, ax1, ax2, metricsFile)
    
    optchanged = false;
    
    persistent costHistory bestCostEver bestParamsEver
    
    if strcmp(flag, 'init')
        costHistory = [];
        bestCostEver = inf;
        bestParamsEver = [0, 0];
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
        
        % 최적 개체로 시뮬레이션하여 플롯
        try
            t_gain = [0, simTime];
            Ref_Kp_ext = timeseries([bestParamsEver(1), bestParamsEver(1)], t_gain);
            Ref_Ki_ext = timeseries([bestParamsEver(2), bestParamsEver(2)], t_gain);
            assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
            assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
            
            simOut = sim(modelName, 'StopTime', num2str(simTime), 'SimulationMode', 'accelerator');
            [t, I_meas] = extractCurrentSignal(simOut);
            I_ref = resampleReference(RefCurrentTS, t);
            
            % 성능 지표 계산
            n = min(length(t), min(length(I_meas), length(I_ref)));
            t = t(1:n); I_meas = I_meas(1:n); I_ref = I_ref(1:n);
            metrics = calculateAllMetrics(t(:), I_meas(:), I_ref(:), stepTime);
            
            % 터미널에 성능 지표 추가 출력
            fprintf('   ➤ Overshoot: %.2f%%  |  Settling: %.2f ms  |  SSE: %.2f%%\n', ...
                metrics.Overshoot, metrics.SettlingTime*1000, metrics.SteadyStateError);
            
            % 플롯 업데이트
            if isvalid(figHandle)
                % 전류 응답 플롯 업데이트
                cla(ax1);
                plot(ax1, t*1000, I_ref, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
                plot(ax1, t*1000, I_meas, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured');
                title(ax1, sprintf('Gen %d: Kp=%.3f, Ki=%.1f (OS=%.1f%%, Ts=%.1fms)', ...
                    gen, bestParamsEver(1), bestParamsEver(2), metrics.Overshoot, metrics.SettlingTime*1000));
                legend(ax1, 'Location', 'southeast');
                
                % Cost 히스토리 플롯
                cla(ax2);
                plot(ax2, 1:length(costHistory), costHistory, 'b-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
                xlabel(ax2, 'Generation');
                ylabel(ax2, 'Best Cost');
                title(ax2, sprintf('Cost History (Current: %.4f)', minScore));
                
                drawnow;
            end
            
        catch ME
            fprintf('   (Plot update failed: %s)\n', ME.message);
        end
        
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
    end
    
    if strcmp(flag, 'done')
        writeLog(logFile, 'GA optimization finished.');
        fprintf('\n>>> GA optimization loop finished.\n');
    end
end

%% =========================================================================
%  신호 추출 함수
%  =========================================================================
function [t, y] = extractCurrentSignal(simOut)
    names = {'Current_Low', 'I_LV', 'I_out', 'I_ess'};
    raw = [];
    
    for i = 1:length(names)
        if isprop(simOut, names{i})
            raw = simOut.(names{i});
            break;
        end
    end
    
    if isempty(raw) && isprop(simOut, 'yout')
        try
            raw = simOut.yout.get('Current_Low');
        catch
        end
    end
    
    if isempty(raw)
        error('Current signal not found in simulation output.');
    end
    
    if isa(raw, 'timeseries')
        y = squeeze(raw.Data);
        t = raw.Time;
    elseif isstruct(raw)
        y = squeeze(raw.signals.values);
        t = raw.time;
    else
        y = squeeze(raw);
        t = simOut.tout;
    end
    
    y = y(:);
    t = t(:);
end

%% =========================================================================
%  Reference 리샘플링 함수
%  =========================================================================
function I_ref = resampleReference(RefTS, t_query)
    t_query = t_query(:);
    resampled = resample(RefTS, t_query);
    I_ref = squeeze(resampled.Data);
    I_ref = I_ref(:);
end

%% =========================================================================
%  로그 파일 작성 헬퍼 함수
%  =========================================================================
function writeLog(logFile, message)
    logID = fopen(logFile, 'a');
    if logID > 0
        fprintf(logID, '[%s] %s\n', datestr(now, 'HH:MM:SS'), message);
        fclose(logID);
    end
end
