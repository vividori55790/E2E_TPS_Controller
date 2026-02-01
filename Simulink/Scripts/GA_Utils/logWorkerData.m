function logWorkerData(metricsFile, data)
% logWorkerData: DataQueue callback to write metrics to CSV in Master
%
% [Inputs]
%   metricsFile: Absolute path to CSV file
%   data: Structure received from worker (EvalNum, Kp, Ki, Cost, Metrics, ...)

    % 1. Command Window Output (Verbose for User Check)
    % "Terminal Input" check: Print result to terminal so user can see it
    if mod(data.EvalNum, 1) == 0 % Print every step
        fprintf('  [Eval %03d] Kp=%.4f, Ki=%.4f => Cost=%.6f\n', ...
            data.EvalNum, data.Kp, data.Ki, data.Cost);
    end
    
    % Disabled 'dot' printing to avoid clutter
    % fprintf('.');
    
    % 2. Write to CSV (Master Only - No file conflict)
    try
        % Re-use saveMetricsToCSV logic or write directly
        % Since saveMetricsToCSV is robust, let's use it if available in path.
        % However, passing 'metrics' struct is cleaner.
        
        m = data.Metrics;
        
        fileID = fopen(metricsFile, 'a');
        if fileID ~= -1
            fprintf(fileID, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%d,%d\n', ...
                data.EvalNum, data.Kp, data.Ki, data.Cost, ...
                m.ISE, m.ITAE, m.IAE, ...
                m.Overshoot, m.Undershoot, ...
                m.RiseTime*1000, m.SettlingTime*1000, m.PeakTime*1000, ...
                m.SteadyStateError, m.PeakValue, m.TargetValue, ...
                data.IsValid, data.IsCached);
            fclose(fileID);
        end
    catch
        % File might be locked briefly by OS or Antivirus
    end
end
