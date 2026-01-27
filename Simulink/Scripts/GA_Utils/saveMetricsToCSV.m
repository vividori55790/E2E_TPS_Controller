function saveMetricsToCSV(metricsFile, evalNum, Kp, Ki, cost, metrics, valid, cached)
% saveMetricsToCSV: 성능 지표를 CSV 파일에 저장
    if nargin < 8, cached = false; end
    
    % [PARALLEL SAFETY] 파일 접근 재시도 로직 (최대 5회)
    maxRetries = 5;
    csvID = -1;
    for i = 1:maxRetries
        try
            csvID = fopen(metricsFile, 'a');
            if csvID > 0
                break; % 성공 시 루프 탈출
            end
        catch
            % 파일 열기 실패 시 대기
        end
        pause(0.01 + rand() * 0.05); % 10ms ~ 60ms 랜덤 대기 (충돌 분산)
    end
    
    if csvID > 0
        if valid && ~isempty(metrics)
            % metrics 구조체 필드 체크 (없으면 NaN)
            if ~isfield(metrics, 'RippleStd'), metrics.RippleStd = NaN; end
            
            fprintf(csvID, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d\n', ...
                evalNum, Kp, Ki, cost, ...
                metrics.ISE, metrics.ITAE, metrics.IAE, ...
                metrics.Overshoot, metrics.Undershoot, ...
                metrics.RiseTime*1000, metrics.SettlingTime*1000, metrics.PeakTime*1000, ...
                metrics.SteadyStateError, metrics.PeakValue, metrics.TargetValue, 1, cached);
        else
            fprintf(csvID, '%d,%.6f,%.6f,%.6f,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,0,%d\n', ...
                evalNum, Kp, Ki, cost, cached);
        end
        fclose(csvID);
    end
end
