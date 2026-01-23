%% View_GA_Results.m
% View and compare saved GA optimization results
% This script helps you explore all saved GA results

function View_GA_Results()
    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath), scriptPath = pwd; end
    
    resultDir = fullfile(scriptPath, 'GA_Results');
    
    if ~exist(resultDir, 'dir')
        error('GA_Results 폴더가 존재하지 않습니다.');
    end
    
    %% 저장된 Final 결과 파일 찾기
    finalFiles = dir(fullfile(resultDir, 'GA_Final_*.mat'));
    
    if isempty(finalFiles)
        fprintf('저장된 GA 최종 결과가 없습니다.\n');
        
        % Progress 파일 확인
        progressFiles = dir(fullfile(resultDir, 'GA_Progress_*.mat'));
        if ~isempty(progressFiles)
            fprintf('\n중간 저장 파일이 %d개 있습니다.\n', length(progressFiles));
            fprintf('GA_Resume_Optimization.m을 실행하여 최적화를 재개할 수 있습니다.\n');
            
            % 가장 최근 progress 파일 표시
            [~, idx] = max([progressFiles.datenum]);
            fprintf('\n가장 최근 저장 파일:\n');
            fprintf('  파일명: %s\n', progressFiles(idx).name);
            fprintf('  날짜: %s\n', progressFiles(idx).date);
            
            load(fullfile(resultDir, progressFiles(idx).name), 'saveData');
            fprintf('  세대: %d\n', saveData.Generation);
            fprintf('  최적 비용: %.6f\n', saveData.BestFval);
        end
        return;
    end
    
    %% 모든 결과 로드 및 표시
    fprintf('===========================================\n');
    fprintf('  GA Optimization Results\n');
    fprintf('===========================================\n');
    fprintf('발견된 결과 파일: %d개\n\n', length(finalFiles));
    
    results = cell(length(finalFiles), 1);
    
    for i = 1:length(finalFiles)
        filePath = fullfile(resultDir, finalFiles(i).name);
        data = load(filePath);
        
        if isfield(data, 'finalResult')
            results{i} = data.finalResult;
        else
            % 구버전 형식
            results{i} = struct();
            if isfield(data, 'x_best')
                results{i}.Kp = data.x_best(1);
                results{i}.Ki = data.x_best(2);
            end
            if isfield(data, 'fval')
                results{i}.Cost = data.fval;
            end
            results{i}.Timestamp = finalFiles(i).date;
        end
        
        fprintf('--- 결과 #%d ---\n', i);
        fprintf('파일: %s\n', finalFiles(i).name);
        fprintf('날짜: %s\n', finalFiles(i).date);
        
        if isfield(results{i}, 'Kp')
            fprintf('Kp:   %.4f\n', results{i}.Kp);
        end
        if isfield(results{i}, 'Ki')
            fprintf('Ki:   %.4f\n', results{i}.Ki);
        end
        if isfield(results{i}, 'Cost')
            fprintf('Cost: %.6f\n', results{i}.Cost);
        end
        if isfield(results{i}, 'Elapsed')
            fprintf('Time: %.2f sec (%.2f min)\n', results{i}.Elapsed, results{i}.Elapsed/60);
        end
        if isfield(results{i}, 'Generations')
            fprintf('Generations: %d\n', results{i}.Generations);
        elseif isfield(results{i}, 'TotalGenerations')
            fprintf('Total Generations: %d (resumed)\n', results{i}.TotalGenerations);
        end
        fprintf('\n');
    end
    
    %% 최적 결과 찾기
    costs = zeros(length(results), 1);
    for i = 1:length(results)
        if isfield(results{i}, 'Cost')
            costs(i) = results{i}.Cost;
        else
            costs(i) = inf;
        end
    end
    
    [minCost, bestIdx] = min(costs);
    
    fprintf('===========================================\n');
    fprintf('  최적 결과 (가장 낮은 Cost)\n');
    fprintf('===========================================\n');
    fprintf('파일: %s\n', finalFiles(bestIdx).name);
    fprintf('Kp:   %.4f\n', results{bestIdx}.Kp);
    fprintf('Ki:   %.4f\n', results{bestIdx}.Ki);
    fprintf('Cost: %.6f\n', minCost);
    fprintf('===========================================\n\n');
    
    %% 워크스페이스에 저장
    assignin('base', 'GA_AllResults', results);
    assignin('base', 'GA_BestResult', results{bestIdx});
    
    fprintf('결과가 워크스페이스에 저장되었습니다:\n');
    fprintf('  - GA_AllResults: 모든 결과\n');
    fprintf('  - GA_BestResult: 최적 결과\n\n');
    
    %% 로그 파일 보기 옵션
    fprintf('관련 로그 파일:\n');
    logFiles = dir(fullfile(resultDir, 'GA_Log_*.txt'));
    for i = 1:min(5, length(logFiles))
        fprintf('  %d. %s\n', i, logFiles(i).name);
    end
    
    if ~isempty(logFiles)
        fprintf('\n로그 파일을 보려면:\n');
        fprintf('  type(''%s'')\n', fullfile(resultDir, logFiles(end).name));
    end
    
    %% 비교 플롯
    if length(results) > 1
        figure('Name', 'GA Results Comparison', 'Position', [100, 100, 1000, 600]);
        
        % Cost 비교
        subplot(2,2,1);
        bar(costs);
        ylabel('Cost');
        xlabel('Result #');
        title('Cost Comparison');
        grid on;
        
        % Kp 비교
        subplot(2,2,2);
        Kps = arrayfun(@(i) results{i}.Kp, 1:length(results));
        bar(Kps);
        ylabel('Kp');
        xlabel('Result #');
        title('Kp Values');
        grid on;
        
        % Ki 비교
        subplot(2,2,3);
        Kis = arrayfun(@(i) results{i}.Ki, 1:length(results));
        bar(Kis);
        ylabel('Ki');
        xlabel('Result #');
        title('Ki Values');
        grid on;
        
        % Kp vs Ki 산점도
        subplot(2,2,4);
        scatter(Kps, Kis, 100, costs, 'filled');
        xlabel('Kp');
        ylabel('Ki');
        title('Kp vs Ki (colored by Cost)');
        colorbar;
        grid on;
        
        % 최적점 표시
        hold on;
        plot(results{bestIdx}.Kp, results{bestIdx}.Ki, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
        legend('All Results', 'Best Result');
    end
    
    fprintf('\n완료!\n');
end
