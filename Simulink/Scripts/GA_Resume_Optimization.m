%% GA_Resume_Optimization.m
% Resume a previously interrupted GA optimization
% This script allows you to continue from the last saved checkpoint
%
% Usage:
%   1. Run this script
%   2. Select the GA_Progress_*.mat file you want to resume from
%   3. The optimization will continue from that generation

function GA_Resume_Optimization()
    %% 0. 환경 설정
    clear; clc; close all;
    
    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath), scriptPath = pwd; end
    addpath(scriptPath); 
    addpath(fullfile(scriptPath, '..'));
    addpath(fullfile(scriptPath, '..', 'Modules'));
    addpath(fullfile(scriptPath, '..', 'Data'));
    
    resultDir = fullfile(scriptPath, 'GA_Results');
    
    %% 1. 이전 결과 파일 선택
    if ~exist(resultDir, 'dir')
        error('GA_Results 폴더가 존재하지 않습니다. 이전 실행 결과가 없습니다.');
    end
    
    % 사용 가능한 Progress 파일 찾기
    progressFiles = dir(fullfile(resultDir, 'GA_Progress_*.mat'));
    
    if isempty(progressFiles)
        error('저장된 GA Progress 파일을 찾을 수 없습니다.');
    end
    
    % 가장 최근 파일 자동 선택
    [~, idx] = max([progressFiles.datenum]);
    selectedFile = fullfile(resultDir, progressFiles(idx).name);
    
    fprintf('===========================================\n');
    fprintf('  Resume GA Optimization\n');
    fprintf('===========================================\n');
    fprintf('Loading: %s\n', progressFiles(idx).name);
    fprintf('Last modified: %s\n', progressFiles(idx).date);
    fprintf('===========================================\n\n');
    
    % 데이터 로드
    load(selectedFile, 'saveData');
    
    fprintf('Saved state:\n');
    fprintf('  Generation: %d\n', saveData.Generation);
    fprintf('  Best Cost:  %.6f\n', saveData.BestFval);
    fprintf('  Timestamp:  %s\n', saveData.Timestamp);
    fprintf('\n');
    
    %% 2. 새로운 세션 시작
    timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
    newSaveFile = fullfile(resultDir, sprintf('GA_Progress_Resumed_%s.mat', timestamp));
    newLogFile = fullfile(resultDir, sprintf('GA_Log_Resumed_%s.txt', timestamp));
    
    % 로그 파일 초기화
    logID = fopen(newLogFile, 'w');
    fprintf(logID, '===========================================\n');
    fprintf(logID, 'DAB Controller GA Optimization (RESUMED)\n');
    fprintf(logID, 'Resumed from Generation: %d\n', saveData.Generation);
    fprintf(logID, 'Started: %s\n', datestr(now));
    fprintf(logID, '===========================================\n\n');
    fclose(logID);
    
    %% 3. 시나리오 및 모델 초기화
    try
        run(fullfile(scriptPath, 'Create_Scenario.m'));
        disp('>>> Scenario Loaded.');
        writeLog(newLogFile, 'Scenario loaded successfully');
    catch ME
        msg = sprintf('Create_Scenario.m 실행 실패: %s', ME.message);
        writeLog(newLogFile, msg);
        error(msg);
    end
    
    modelName = 'Control_Logic';
    if ~bdIsLoaded(modelName)
        try
            load_system(modelName);
            disp(['>>> Model "' modelName '" loaded.']);
            writeLog(newLogFile, sprintf('Model "%s" loaded', modelName));
        catch
            msg = sprintf('Model "%s" not found!', modelName);
            writeLog(newLogFile, msg);
            error(msg);
        end
    end
    
    %% 4. 하드코딩된 지령치
    STEP_TIME = 0.02;
    REF_CURRENT = 10.0;
    SIM_TIME = 0.05;
    
    t_cmd = [0, STEP_TIME - 1e-6, STEP_TIME, SIM_TIME];
    v_cmd = [0, 0, REF_CURRENT, REF_CURRENT];
    RefCurrentTS = timeseries(v_cmd(:), t_cmd(:));
    
    fprintf('>>> Reference: Step to %.1fA at t=%.3fs\n', REF_CURRENT, STEP_TIME);
    
    %% 5. GA 재시작 설정
    nVars = 2;
    lb = [0.01, 1.0];
    ub = [50.0, 5000.0];
    
    % 남은 세대 계산
    maxGen = 20;
    startGen = saveData.Generation + 1;
    remainingGen = maxGen - saveData.Generation;
    
    fprintf('>>> Remaining generations: %d (from Gen %d to %d)\n', remainingGen, startGen, maxGen);
    writeLog(newLogFile, sprintf('Resuming from generation %d, %d generations remaining', startGen, remainingGen));
    
    % GA 옵션 (초기 개체군을 복원)
    options = optimoptions('ga', ...
        'PopulationSize', 10, ...
        'MaxGenerations', remainingGen, ...
        'InitialPopulationMatrix', saveData.Population, ...
        'InitialScoresMatrix', saveData.Score, ...
        'Display', 'iter', ...
        'UseParallel', false, ...
        'PlotFcn', @gaplotbestf, ...
        'OutputFcn', @(options, state, flag) saveProgress(options, state, flag, newSaveFile, newLogFile, startGen));
    
    %% 6. GA 재시작
    disp('>>> Resuming GA Optimization...');
    writeLog(newLogFile, 'Resuming GA optimization...');
    
    costFcn = @(x) evaluateCost(x, modelName, RefCurrentTS, SIM_TIME);
    
    t_start = tic;
    [x_best, fval, exitflag, output] = ga(costFcn, nVars, [], [], [], [], lb, ub, [], options);
    elapsed = toc(t_start);
    
    %% 7. 최종 결과
    totalGen = saveData.Generation + output.generations;
    
    fprintf('\n===========================================\n');
    fprintf('  Optimization Complete!\n');
    fprintf('===========================================\n');
    fprintf('  Time Elapsed: %.2f sec\n', elapsed);
    fprintf('  Total Generations:  %d\n', totalGen);
    fprintf('  Best Kp:      %.4f\n', x_best(1));
    fprintf('  Best Ki:      %.4f\n', x_best(2));
    fprintf('  Final Cost:   %.6f\n', fval);
    fprintf('===========================================\n');
    
    writeLog(newLogFile, '');
    writeLog(newLogFile, '===========================================');
    writeLog(newLogFile, 'Optimization Complete!');
    writeLog(newLogFile, sprintf('Total Generations: %d', totalGen));
    writeLog(newLogFile, sprintf('Best Kp: %.4f', x_best(1)));
    writeLog(newLogFile, sprintf('Best Ki: %.4f', x_best(2)));
    writeLog(newLogFile, sprintf('Final Cost: %.6f', fval));
    writeLog(newLogFile, '===========================================');
    
    %% 8. 최종 결과 저장
    finalResult = struct();
    finalResult.Kp = x_best(1);
    finalResult.Ki = x_best(2);
    finalResult.Cost = fval;
    finalResult.Elapsed = elapsed;
    finalResult.TotalGenerations = totalGen;
    finalResult.ExitFlag = exitflag;
    finalResult.Timestamp = timestamp;
    finalResult.ResumedFrom = saveData.Timestamp;
    
    finalFile = fullfile(resultDir, sprintf('GA_Final_Resumed_%s.mat', timestamp));
    save(finalFile, 'finalResult', 'x_best', 'fval', 'exitflag', 'output');
    
    assignin('base', 'GA_Result', finalResult);
    
    fprintf('>>> Results saved to:\n');
    fprintf('    - File: %s\n', finalFile);
    fprintf('    - Log: %s\n', newLogFile);
    
    writeLog(newLogFile, sprintf('Final results saved to: %s', finalFile));
    writeLog(newLogFile, sprintf('Completed: %s', datestr(now)));
end

%% =========================================================================
%  Cost Function (비용 함수)
%  =========================================================================
function cost = evaluateCost(params, modelName, RefCurrentTS, simTime)
    Kp = params(1);
    Ki = params(2);
    
    t_gain = [0, simTime];
    Ref_Kp_ext = timeseries([Kp, Kp], t_gain);
    Ref_Ki_ext = timeseries([Ki, Ki], t_gain);
    
    assignin('base', 'Ref_Kp_ext', Ref_Kp_ext);
    assignin('base', 'Ref_Ki_ext', Ref_Ki_ext);
    assignin('base', 'Kp_ext', Kp);
    assignin('base', 'Ki_ext', Ki);
    
    try
        simOut = sim(modelName, 'StopTime', num2str(simTime), ...
                     'SimulationMode', 'accelerator', ...
                     'FastRestart', 'off');
    catch
        cost = 1e6;
        return;
    end
    
    try
        [t, I_meas] = extractCurrentSignal(simOut);
        I_ref = resampleReference(RefCurrentTS, t);
    catch
        cost = 1e6;
        return;
    end
    
    if isempty(t) || length(t) < 10
        cost = 1e6;
        return;
    end
    
    n = min(length(t), min(length(I_meas), length(I_ref)));
    t = t(1:n);
    I_meas = I_meas(1:n);
    I_ref = I_ref(1:n);
    
    t = t(:);
    I_meas = I_meas(:);
    I_ref = I_ref(:);
    
    error_sig = I_ref - I_meas;
    
    [~, step_idx] = max(abs(diff(I_ref)));
    t_step = t(step_idx);
    idx_after = t > t_step;
    
    if any(idx_after)
        target = mean(I_ref(idx_after));
    else
        target = max(I_ref);
    end
    
    ISE = trapz(t, error_sig.^2);
    ITAE = trapz(t, t .* abs(error_sig));
    
    OS = 0;
    if abs(target) > 0.1 && any(idx_after)
        peak = max(I_meas(idx_after));
        if peak > target
            OS = (peak - target) / target * 100;
        end
    end
    
    Ts = 0;
    if abs(target) > 0.1 && any(idx_after)
        tol = 0.02 * abs(target);
        within = abs(I_meas(idx_after) - target) <= tol;
        if ~all(within)
            last_out = find(~within, 1, 'last');
            t_after = t(idx_after);
            Ts = t_after(last_out) - t_step;
        end
    end
    
    cost = 2.0 * (ISE / 50) + ...
           1.0 * (ITAE / 0.5) + ...
           3.0 * (OS / 100) + ...
           1.5 * (Ts / 0.03);
    
    if isnan(cost) || isinf(cost)
        cost = 1e6;
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
%  OutputFcn: 각 세대마다 진행 상황 저장
%  =========================================================================
function [state, options, optchanged] = saveProgress(options, state, flag, saveFile, logFile, offset)
    optchanged = false;
    
    if strcmp(flag, 'iter')
        gen = state.Generation + offset - 1;
        bestFval = state.Best(end);
        
        msg = sprintf('Gen %3d: Best Cost = %.6f', gen, bestFval);
        writeLog(logFile, msg);
        
        saveData = struct();
        saveData.Generation = gen;
        saveData.BestFval = bestFval;
        saveData.BestIndividual = state.Best;
        saveData.Population = state.Population;
        saveData.Score = state.Score;
        saveData.Timestamp = datestr(now);
        
        save(saveFile, 'saveData');
    end
    
    if strcmp(flag, 'done')
        writeLog(logFile, 'GA optimization finished.');
    end
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
