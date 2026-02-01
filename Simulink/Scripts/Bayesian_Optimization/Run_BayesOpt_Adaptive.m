%% Run_BayesOpt_Adaptive.m
% [Auto-Expansion] Automatically expands search space if optimal point is near boundaries.
% [자동 확장] 최적점이 경계 근처에 있을 경우 탐색 범위를 자동으로 확장하여 재실행합니다.

clear; clc; close all;

currentPath = fileparts(mfilename('fullpath'));
addpath(currentPath);
addpath(fullfile(currentPath, '..')); 
addpath(fullfile(currentPath, '..', '..')); 

%% [Log Setup] 로그 파일 설정
logFile = fullfile(currentPath, 'Optimization_Log_Adaptive.txt');
fid = fopen(logFile, 'w');
if fid ~= -1
    fprintf(fid, '=== DAB Adaptive Optimization Log Started at %s ===\n', datestr(now));
    fprintf(fid, 'Format: [Time] Params | Cost | Details(Track, Effort, SS, Over) | Msg\n');
    fprintf(fid, '--------------------------------------------------------------------------------\n');
    fclose(fid);
end

% DataQueue 생성 (비동기 로깅용)
q = parallel.pool.DataQueue;
afterEach(q, @(data) logQueueData(data, logFile));

%% [Settings] 초기 설정
% 초기 탐색 범위 (Initial Bounds)
current_Kp_range = [0.01, 50];
current_Ki_range = [0.1, 500000];

% 스테이지 설정 (Stage Settings)
% 각 스테이지마다 설정된 횟수만큼 최적화를 수행하고, 경계 확장 여부를 판단합니다.
evals_per_stage = 20; 
max_stages = 10;

results = []; % 이전 결과 저장용
accumulated_X = table();
accumulated_Obj = [];

fprintf('>> [Adaptive BayesOpt] Start...\n');

%% [Main Loop] 최적화 반복
for stage = 1:max_stages
    fprintf('\n================================================================\n');
    fprintf('>> [Stage %d/%d] Kp: [%g, %g], Ki: [%g, %g]\n', ...
        stage, max_stages, current_Kp_range(1), current_Kp_range(2), current_Ki_range(1), current_Ki_range(2));
    
    % 변수 정의 (현재 경계 사용)
    vars = [
        optimizableVariable('Kp', current_Kp_range, 'Transform', 'log')
        optimizableVariable('Ki', current_Ki_range, 'Transform', 'log')
    ];
    
    % 목적 함수 (DataQueue 포함)
    objFcn = @(params) DAB_Objective_Function(params, q);
    
    % 옵션 설정
    opts = {
        'Verbose', 1, ...
        'MaxObjectiveEvaluations', evals_per_stage, ...
        'UseParallel', true, ... % 병렬 처리 사용
        'AcquisitionFunctionName', 'expected-improvement-plus', ...
        'IsObjectiveDeterministic', false, ...
        'PlotFcn', {} % 플롯 끄기
    };
    
    % 이전 데이터가 있으면 InitialX로 주입 (지식 전이)
    if ~isempty(accumulated_X)
        % 현재 범위 밖의 점이 InitialX에 있으면 에러가 날 수 있으므로, 
        % 이론상 확장만 하므로 모두 포함되지만, 혹시 모르니 체크할 수도 있음.
        % 여기서는 확장이 보장되므로 그대로 전달.
        opts = [opts, {'InitialX', accumulated_X, 'InitialObjective', accumulated_Obj}];
        fprintf('>> [Info] Resuming with %d previous points.\n', height(accumulated_X));
    end
    
    % 최적화 실행
    results = bayesopt(objFcn, vars, opts{:});
    
    % 결과 누적 (다음 스테이지용)
    accumulated_X = results.XTrace;
    accumulated_Obj = results.ObjectiveTrace;
    
    %% [Check Expansion] 경계 확장 필요성 검사
    bestParams = results.XAtMinObjective;
    minCost = results.MinObjective;
    bestKp = bestParams.Kp;
    bestKi = bestParams.Ki;
    
    fprintf('>> [Stage %d Result] Best Kp: %.5f, Ki: %.5f (Cost: %.5f)\n', stage, bestKp, bestKi, minCost);
    
    needs_expansion = false;
    
    % Rule: 경계의 2배(로그 스케일 고려) 이내에 최적점이 있으면 확장
    % Kp Check
    if bestKp < current_Kp_range(1) * 2
        new_lower = current_Kp_range(1) / 10;
        fprintf('>> [Expand] Kp is near LOWER bound. Expanding %.5f -> %.5f\n', current_Kp_range(1), new_lower);
        current_Kp_range(1) = new_lower;
        needs_expansion = true;
    end
    if bestKp > current_Kp_range(2) / 2
        new_upper = current_Kp_range(2) * 10;
        fprintf('>> [Expand] Kp is near UPPER bound. Expanding %.5f -> %.5f\n', current_Kp_range(2), new_upper);
        current_Kp_range(2) = new_upper;
        needs_expansion = true;
    end
    
    % Ki Check
    if bestKi < current_Ki_range(1) * 2
        new_lower = current_Ki_range(1) / 10;
        fprintf('>> [Expand] Ki is near LOWER bound. Expanding %.5f -> %.5f\n', current_Ki_range(1), new_lower);
        current_Ki_range(1) = new_lower;
        needs_expansion = true;
    end
    if bestKi > current_Ki_range(2) / 2
        new_upper = current_Ki_range(2) * 10;
        fprintf('>> [Expand] Ki is near UPPER bound. Expanding %.5f -> %.5f\n', current_Ki_range(2), new_upper);
        current_Ki_range(2) = new_upper;
        needs_expansion = true;
    end
    
    % 확장이 필요 없으면 종료
    if ~needs_expansion
        fprintf('>> [Success] Optimal point is stable within bounds. Stopping.\n');
        break;
    else
        fprintf('>> [Continue] Bounds expanded. Starting next stage...\n');
    end
end

%% 결과 저장
resDir = fullfile(currentPath, 'Results');
if ~exist(resDir, 'dir'), mkdir(resDir); end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(fullfile(resDir, sprintf('BayesOpt_Adaptive_Result_%s.mat', timestamp)), 'results', 'accumulated_X', 'accumulated_Obj');

fprintf('\n>> [Done] All stages completed.\n');


%% [Local Function] DataQueue 리스너 (동일)
function logQueueData(data, logFile)
    fid = fopen(logFile, 'a');
    if fid == -1, return; end
    try
        switch data.Type
            case 'RESULT'
                if isfield(data.Details, 'Msg'), msgStr = data.Details.Msg; else, msgStr = 'N/A'; end
                fprintf(fid, '[%s] Kp=%.4f, Ki=%.4f | Cost=%.4f | Trk=%.3f, Eff=%.3f, SS=%.3f, Ov=%.1f%% | %s\n', ...
                    data.Timestamp, data.Kp, data.Ki, data.Cost, ...
                    data.Details.Track, data.Details.Effort, data.Details.SS, data.Details.Overshoot_Pct, msgStr);
                
                % Console Output (Simple)
                if data.Cost >= 999999
                     fprintf('[Res] FAIL: %s (Kp=%.2f, Ki=%.2f)\n', msgStr, data.Kp, data.Ki);
                else
                     fprintf('[Res] Cost=%.4f (Kp=%.2f, Ki=%.2f)\n', data.Cost, data.Kp, data.Ki);
                end
                
            case 'ERROR'
                fprintf(fid, '[ERROR] %s\n', data.Msg);
                fprintf(2, '[Queue Error] %s\n', data.Msg);
                
            case 'DEBUG'
                fprintf(fid, '[DEBUG] %s\n', data.Msg);
        end
    catch
    end
    fclose(fid);
end
