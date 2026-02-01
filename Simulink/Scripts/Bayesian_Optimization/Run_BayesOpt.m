%% Run_BayesOpt.m (With Log Cleanup)

clear; clc; close all;

currentPath = fileparts(mfilename('fullpath'));
addpath(currentPath);
addpath(fullfile(currentPath, '..')); 
addpath(fullfile(currentPath, '..', '..')); 

%% [NEW] 이전 로그 파일 초기화 & DataQueue 설정
logFile = fullfile(currentPath, 'Optimization_Log.txt');
fid = fopen(logFile, 'w');
if fid ~= -1
    fprintf(fid, '=== DAB Optimization Log Started at %s ===\n', datestr(now));
    fprintf(fid, 'Format: [Time] Params | Cost | Details(Track, Effort, SS, Over) | Msg\n');
    fprintf(fid, '--------------------------------------------------------------------------------\n');
    fclose(fid);
end

% DataQueue 생성 (비동기 로깅용)
q = parallel.pool.DataQueue;
afterEach(q, @(data) logQueueData(data, logFile));

fprintf('>> [BayesOpt] DAB Converter Controller Tuning Start...\n');

%% 설정
    optimizableVariable('Kp', [0.01, 50], 'Transform', 'log')
    optimizableVariable('Ki', [0.1, 500000], 'Transform', 'log')
];

max_evals = 300; 
use_parallel = true; 

%% 실행
% DataQueue를 인자로 넘기는 익명 함수 생성
objFcn = @(params) DAB_Objective_Function(params, q);

fprintf('>> [BayesOpt] Running Optimization (%d iterations)...\n', max_evals);
fprintf('>> [Tip] Check "Optimization_Log.txt" for real-time details.\n');

% Pass options directly as Name-Value pairs
results = bayesopt(objFcn, vars, ...
    'Verbose', 1, ...
    'MaxObjectiveEvaluations', max_evals, ...
    'UseParallel', use_parallel, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'IsObjectiveDeterministic', false, ...
    'PlotFcn', {}); % [FIX] Plot 오류 방지 (콘솔 로그 확인 권장)

%% 결과 저장
bestParams = results.XAtMinObjective;
minCost = results.MinObjective;

fprintf('\n>> Best Kp: %.5f, Ki: %.5f (Cost: %.5f)\n', bestParams.Kp, bestParams.Ki, minCost);

resDir = fullfile(currentPath, 'Results');
if ~exist(resDir, 'dir'), mkdir(resDir); end
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
save(fullfile(resDir, sprintf('BayesOpt_Result_%s.mat', timestamp)), 'results', 'bestParams', 'minCost');

%% [Local Function] DataQueue 리스너
function logQueueData(data, logFile)
    % Log types: RESULT, DEBUG, ERROR
    
    fid = fopen(logFile, 'a');
    if fid == -1, return; end
    
    try
        switch data.Type
            case 'RESULT'
                % 결과 로깅
                % Msg 필드가 없는 경우를 대비해 안전하게 처리
                if isfield(data.Details, 'Msg')
                    msgStr = data.Details.Msg;
                else
                    msgStr = 'N/A';
                end
                
                fprintf(fid, '[%s] Kp=%.4f, Ki=%.4f | Cost=%.4f | Trk=%.3f, Eff=%.3f, SS=%.3f, Ov=%.1f%% | %s\n', ...
                    data.Timestamp, data.Kp, data.Ki, data.Cost, ...
                    data.Details.Track, data.Details.Effort, data.Details.SS, data.Details.Overshoot_Pct, msgStr);
                
                % 콘솔에도 간단히 출력
                if data.Cost >= 999999
                     fprintf('[Res] FAIL: %s (Kp=%.2f, Ki=%.2f)\n', msgStr, data.Kp, data.Ki);
                else
                     fprintf('[Res] Cost=%.4f (Kp=%.2f, Ki=%.2f)\n', data.Cost, data.Kp, data.Ki);
                end
                
            case 'ERROR'
                % 에러 로깅
                fprintf(fid, '[ERROR] %s\n', data.Msg);
                fprintf(2, '[Queue Error] %s\n', data.Msg);
                
            case 'DEBUG'
                % 디버그 로깅
                fprintf(fid, '[DEBUG] %s\n', data.Msg);
        end
    catch
        % 로깅 실패 시 무시
    end
    fclose(fid);
end
