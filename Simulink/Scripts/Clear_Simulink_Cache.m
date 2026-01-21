% Clear_Simulink_Cache.m
% Simulink 컴파일 캐시 완전 삭제 (메모리 문제 해결용)
%
% 사용법: 
%   이 스크립트를 Scripts 폴더에서 실행하세요.
%   시뮬레이션 전에 한 번 실행하면 캐시가 초기화됩니다.

fprintf('=================================================================\n');
fprintf('  Simulink Cache Cleaner\n');
fprintf('=================================================================\n\n');

% 1. 열려있는 모든 모델 닫기
fprintf('[1/5] 열린 Simulink 모델 닫는 중...\n');
bdclose('all');

% 2. 메모리 클리어
fprintf('[2/5] MATLAB 메모리 클리어...\n');
clear mex;
clear functions;

% 3. Simulink 파일 생성 제어 리셋
fprintf('[3/5] Simulink 파일 생성 제어 리셋...\n');
try
    Simulink.fileGenControl('reset');
catch
    fprintf('    (Simulink.fileGenControl 실패 - 무시)\n');
end

% 4. slprj 폴더 삭제
fprintf('[4/5] slprj 캐시 폴더 삭제...\n');

cache_dirs = {
    fullfile(pwd, 'slprj')
    fullfile(pwd, '..', 'slprj')
    fullfile(pwd, '..', 'Modules', 'slprj')
};

for i = 1:length(cache_dirs)
    cache_dir = cache_dirs{i};
    if isfolder(cache_dir)
        try
            rmdir(cache_dir, 's');
            fprintf('    삭제: %s\n', cache_dir);
        catch ME
            fprintf('    삭제 실패: %s (%s)\n', cache_dir, ME.message);
        end
    end
end

% 5. 기타 캐시 파일 삭제
fprintf('[5/5] 추가 캐시 파일 삭제...\n');

cache_patterns = {'*.slxc', '*.mexa64', '*.mexw64'};
search_dirs = {pwd, fullfile(pwd, '..'), fullfile(pwd, '..', 'Modules')};

for d = 1:length(search_dirs)
    for p = 1:length(cache_patterns)
        files = dir(fullfile(search_dirs{d}, cache_patterns{p}));
        for f = 1:length(files)
            filepath = fullfile(files(f).folder, files(f).name);
            try
                delete(filepath);
                fprintf('    삭제: %s\n', files(f).name);
            catch
                % 무시
            end
        end
    end
end

% 6. 툴박스 캐시 리프레시
fprintf('\n[완료] 툴박스 캐시 리프레시...\n');
rehash toolboxcache;

fprintf('\n=================================================================\n');
fprintf('  캐시 클리어 완료!\n');
fprintf('  이제 시뮬레이션을 다시 실행해 보세요.\n');
fprintf('  (첫 실행 시 컴파일이 새로 수행되어 시간이 더 걸릴 수 있습니다)\n');
fprintf('=================================================================\n');
