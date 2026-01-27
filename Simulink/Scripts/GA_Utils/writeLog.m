function writeLog(logFile, message)
% writeLog: 로그 파일에 메시지 기록 (타임스탬프 포함)
    logID = fopen(logFile, 'a');
    if logID > 0
        fprintf(logID, '[%s] %s\n', datestr(now, 'HH:MM:SS'), message);
        fclose(logID);
    end
end
