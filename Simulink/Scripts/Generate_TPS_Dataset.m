%% Generate_TPS_Dataset.m
% =========================================================================
% DAB Converter TPS Current Stress Optimization Dataset Generator
% (AI-TPSM Stage I: Inductor Peak Current Data Collection)
%
% 목적: 다양한 동작 조건(P, V2, D1, D2)에서 Simulink 시뮬레이션을 수행하여
%       인덕터 피크 전류(i_pk)를 수집, 신경망 학습 데이터셋 구축
%
% 모델: Control_Logic.slx
% 입력: From Workspace (Ref_Power, D1, D2, Ref_VH 등)
% 출력: To Workspace (구조체 형식, out.xxx)
% 제어: 전력 기반 — Power_cmd = -P (충전방향, 배터리측 기준)
%       외부 위상(Phi/D0)만 PI 제어, D1/D2는 외부 고정
%
% =========================================================================

function Generate_TPS_Dataset()
    %% =====================================================================
    %  0. 환경 설정
    % =====================================================================
    clear; clc; close all;
    
    fprintf('=========================================================================\n');
    fprintf('  DAB TPS Current Stress Dataset Generator\n');
    fprintf('  (AI-TPSM Stage I: i_pk Data Collection via In-Worker Simulation)\n');
    fprintf('=========================================================================\n\n');
    
    % 작업 디렉토리 설정
    scriptDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);  % Simulink 폴더
    dataDir = fullfile(projectDir, 'Data');
    if ~exist(dataDir, 'dir'), mkdir(dataDir); end
    
    % 타임스탬프
    timestamp = datestr(now, 'yyyymmdd_HHMMss');
    resultFile = fullfile(dataDir, sprintf('TPS_Dataset_%s.mat', timestamp));
    csvFile = fullfile(dataDir, sprintf('TPS_Dataset_%s.csv', timestamp));
    logFile = fullfile(dataDir, sprintf('TPS_Dataset_%s.log', timestamp));
    
    % 로그 초기화
    logID = fopen(logFile, 'w');
    fprintf(logID, 'TPS Dataset Generation Log\nStarted: %s\n', datestr(now));
    fprintf(logID, '=========================================\n\n');
    fclose(logID);
    
    %% =====================================================================
    %  1. 시스템 사양 정의
    % =====================================================================
    spec = struct();
    spec.P_rated   = 1000;      % W
    spec.V_in      = 380;       % V (고정)
    spec.V_out_nom = 96;        % V (공칭)
    spec.n         = 4;         % 변압기 권선비
    spec.L         = 150e-6;    % H (총 인덕턴스)
    spec.fs        = 80e3;      % Hz (스위칭 주파수)
    
    % 보호 한계
    spec.I_pri_max = 10;        % A (1차측 피크 전류 한계)
    spec.I_sec_max = 40;        % A (2차측 피크 전류 한계)
    
    % 시뮬레이션 설정
    SIM_TIME = 0.06;            % 60ms (과도응답 포함)
    SS_START = 0.04;            % 정상상태 시작 (40ms)
    
    % 제어기 파라미터
    Kp_val = 5.0;
    Ki_val = 5000;
    DeadTime_ns_val = 200;      % ns
    Kpv_val = 5.0;              % 전압 PI (비사용, 상수)
    Kiv_val = 5000;
    
    fprintf('>> System Specs:\n');
    fprintf('   P_rated=%dW, V_in=%dV, n=%d, L=%.0fuH, fs=%.0fkHz\n', ...
        spec.P_rated, spec.V_in, spec.n, spec.L*1e6, spec.fs/1e3);
    fprintf('   Protection: I_pri_max=%.0fA, I_sec_max=%.0fA\n\n', ...
        spec.I_pri_max, spec.I_sec_max);
    
    %% =====================================================================
    %  2. 파라미터 그리드 생성 (160,000 조합)
    % =====================================================================
    N_steps = 20;
    
    P_vec  = linspace(100, 1000, N_steps);       % 출력 전력 (W)
    V2_vec = linspace(84, 109.2, N_steps);       % 출력 전압 (V)
    D1_vec = linspace(0.05, 0.5, N_steps);       % 1차 듀티
    D2_vec = linspace(0.05, 0.5, N_steps);       % 2차 듀티
    
    [P_grid, V2_grid, D1_grid, D2_grid] = ndgrid(P_vec, V2_vec, D1_vec, D2_vec);
    P_all  = P_grid(:);
    V2_all = V2_grid(:);
    D1_all = D1_grid(:);
    D2_all = D2_grid(:);
    N_total = length(P_all);
    
    fprintf('>> Parameter Grid: %d steps → Total %d combinations\n', N_steps, N_total);
    fprintf('   P:  [%.0f, %.0f] W\n', min(P_all), max(P_all));
    fprintf('   V2: [%.1f, %.1f] V\n', min(V2_all), max(V2_all));
    fprintf('   D1: [%.2f, %.2f]\n', min(D1_all), max(D1_all));
    fprintf('   D2: [%.2f, %.2f]\n\n', min(D2_all), max(D2_all));
    
    %% =====================================================================
    %  3. Simulink 경로 설정
    % =====================================================================
    fprintf('>> Setting up paths and parallel pool...\n');
    addpath(projectDir);
    addpath(fullfile(projectDir, 'Modules'));
    addpath(fullfile(projectDir, 'Data'));
    
    modelName = 'Control_Logic';
    modelDir = fullfile(projectDir, 'Modules');
    originalDir = pwd;
    
    %% =====================================================================
    %  4. 병렬 풀(Parallel Pool) 및 시뮬레이션 환경 초기화
    % =====================================================================
    % 사전에 막대한 SimulationInput 배열을 만들지 않고 Base 객체 1개만 만듭니다.
    tic;
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool;
    end
    
    % 각 워커별로 독립된 파일 캐시 폴더 할당 (Simulink FileExistsError 캐시 충돌 방지)
    fprintf('   Configuring worker-specific Simulink cache folders...\n');
    spmd
        worker_cache = fullfile(tempdir, sprintf('sl_cache_worker_%d', spmdIndex));
        if ~exist(worker_cache, 'dir'), mkdir(worker_cache); end
        Simulink.fileGenControl('set', 'CacheFolder', worker_cache, 'CodeGenFolder', worker_cache);
    end
    
    t_input = [0; SIM_TIME];
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    baseIn = baseIn.setModelParameter('SimulationMode', 'normal');
    
    % 공통(상수) 파라미터 미리 셋팅
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kp_ext', timeseries(Kp_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Ki_ext', timeseries(Ki_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kpv', timeseries(Kpv_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kiv', timeseries(Kiv_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_I_ess_cmd', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('load_R', timeseries(1000 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('timestamp', timeseries(12 * ones(2,1), t_input));
    
    setup_elapsed = toc;
    fprintf('   Parallel Setup Complete (%.1f sec).\n\n', setup_elapsed);
    
    %% =====================================================================
    %  5. 병렬 시뮬레이션 및 데이터 동시 추출 (parfor 적용)
    % =====================================================================
    fprintf('>> Running simulations (%d combinations) via Workers...\n', N_total);
    fprintf('   This avoids memory explosion by allocating directly in workers.\n\n');
    
    % 결과를 저장할 벡터 사전 할당 (시뮬레이션 객체 보존 안 함)
    i_pk_pri = NaN(N_total, 1);
    i_pk_sec = NaN(N_total, 1);
    D0_converged = NaN(N_total, 1);
    valid_flag = false(N_total, 1);
    error_msg = cell(N_total, 1);
    
    % 진행 상태 피드백용 DataQueue (로그 파일에 동시 기록)
    DQueue = parallel.pool.DataQueue;
    afterEach(DQueue, @(msg) logStatusMsg(msg, logFile));
    
    cd(modelDir);
    tic;
    
    % 핵심: parfor 루프 안에서 즉석으로 객체 만들고 지우기
    parfor idx = 1:N_total
        
        simIn = baseIn; % Worker마다 base 설정 복사
        
        P_curr  = P_all(idx);
        D1_curr = D1_all(idx);
        D2_curr = D2_all(idx);
        
        % 가변 변수 주입 (충전 방향 기준 전력)
        Power_cmd_val = -P_curr;
        
        ts_P = timeseries(Power_cmd_val * ones(2,1), t_input);
        ts_D1 = timeseries(D1_curr * ones(2,1), t_input);
        ts_D2 = timeseries(D2_curr * ones(2,1), t_input);
        
        simIn = simIn.setVariable('Ref_Power', ts_P);
        simIn = simIn.setVariable('D1', ts_D1);
        simIn = simIn.setVariable('D2', ts_D2);
        
        % 워커별로 모델 로드 확인 (parfor 내에서 필수)
        if ~bdIsLoaded(modelName)
            load_system(modelName);
        end
        
        try
            simOut = sim(simIn); % 개별 시뮬레이션 실행 (결과만 즉시 받음)
            
            % 에러 체크
            if ~isempty(simOut.ErrorMessage)
                error_msg{idx} = simOut.ErrorMessage;
                continue; % 다음 idx로 건너뜀 (NaN 유지)
            end
            
            out_data = simOut;
            I_L_data = [];
            t_sim = [];
            
            % 1. 인덕터 피크 추출 (getSignalData_local 함수 활용)
            if isprop(out_data, 'Inductor_Current')
                raw_I = out_data.Inductor_Current;
                I_L_data = getSignalData_local(raw_I);
                t_sim = getSignalTime_local(raw_I, out_data);
            elseif isprop(out_data, 'I_L')
                raw_I = out_data.I_L;
                I_L_data = getSignalData_local(raw_I);
                t_sim = getSignalTime_local(raw_I, out_data);
            elseif isprop(out_data, 'yout')
                try
                    raw_I = out_data.yout.get('Inductor_Current');
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                catch, end
            end
            
            if isempty(I_L_data) || isempty(t_sim)
                error_msg{idx} = 'Current data (Inductor_Current) not found.';
                continue;
            end
            
            % 정상상태 구간 슬라이싱
            ss_mask = t_sim >= SS_START;
            if sum(ss_mask) < 10
                error_msg{idx} = 'Insufficient steady-state data.';
                continue;
            end
            
            I_L_ss = I_L_data(ss_mask);
            T_cycle = 1 / spec.fs;
            dt_sim = mean(diff(t_sim(ss_mask)));
            samples_per_cycle = max(2, round(T_cycle / dt_sim));
            
            if length(I_L_ss) >= samples_per_cycle
                last_cycle = I_L_ss(end-samples_per_cycle+1:end);
            else
                last_cycle = I_L_ss;
            end
            
            i_pk_val = max(abs(last_cycle));
            
            % 클렌징 1: 한계치 이상 과전류
            if i_pk_val > spec.I_pri_max * 1.5 || i_pk_val * spec.n > spec.I_sec_max * 1.5
                error_msg{idx} = sprintf('Overcurrent (i_pk=%.1fA)', i_pk_val);
                continue;
            end
            
            % 클렌징 2: 발산/진동 감지 (Std dev가 너무 큰 경우)
            mean_I = mean(abs(I_L_ss));
            std_I = std(I_L_ss);
            if std_I > 5 * mean_I && mean_I > 0.1
                error_msg{idx} = 'PI divergence.';
                continue;
            end
            
            % 2. 외부 위상 추출
            phi_data = [];
            if isprop(out_data, 'Phi_ext_out')
                phi_data = getSignalData_local(out_data.Phi_ext_out);
            elseif isprop(out_data, 'phi_ext_out')
                phi_data = getSignalData_local(out_data.phi_ext_out);
            elseif isprop(out_data, 'Phi_ext')
                phi_data = getSignalData_local(out_data.Phi_ext);
            end
            
            if ~isempty(phi_data)
                D0_converged(idx) = phi_data(end) / 360;
            end
            
            % 안전한 값으로 저장
            i_pk_pri(idx) = i_pk_val;
            i_pk_sec(idx) = i_pk_val * spec.n;
            valid_flag(idx) = true;
            
        catch ME_inner
            error_msg{idx} = sprintf('Error: %s', ME_inner.message);
        end
        
        % 예외 규정: 가장 처음 시뮬레이션(idx==1)은 결과 요약하여 전송
        if idx == 1 && valid_flag(idx)
            send(DQueue, sprintf('   [FIRST PREVIEW] idx=1: P=%.0fW, V2=%.1fV -> i_pk_pri=%.2fA, D0_conv=%.3f', ...
                P_curr, V2_curr, i_pk_val, D0_converged(idx)));
        elseif valid_flag(idx) == false && mod(idx, 10) == 0
            % 실패한 케이스도 10번에 한번 모니터링
            send(DQueue, sprintf('   [%d / %d] Failed. (P=%.0fW) Error: %s', idx, N_total, P_curr, error_msg{idx}));
        elseif mod(idx, 10) == 0
            send(DQueue, sprintf('   [%d / %d] Simulations reached. (P=%.0fW, V2=%.1fV)', idx, N_total, P_curr, V2_curr));
        end
    end
    
    parsim_elapsed = toc;
    cd(originalDir);
    
    %% =====================================================================
    %  6. 추출 통계 계산
    % =====================================================================
    n_valid = sum(valid_flag);
    n_overcurrent = sum(~cellfun('isempty', strfind(error_msg, 'Overcurrent')));
    n_diverged = sum(~cellfun('isempty', strfind(error_msg, 'divergence')));
    n_sim_error = sum(~cellfun('isempty', error_msg)) - n_overcurrent - n_diverged;
    
    fprintf('\n>> Simulation & Extraction complete!\n');
    fprintf('   Parfor Runtime: %.1f sec (%.1f min)\n', parsim_elapsed, parsim_elapsed/60);
    fprintf('   Valid:       %d / %d (%.1f%%)\n', n_valid, N_total, 100*n_valid/N_total);
    fprintf('   Overcurrent: %d\n', n_overcurrent);
    fprintf('   Diverged:    %d\n', n_diverged);
    fprintf('   Sim Error:   %d\n\n', n_sim_error);
    
    % 통계결과를 로그 파일에도 기록
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '\n>> Simulation & Extraction complete!\n');
        fprintf(fid, '   Parfor Runtime: %.1f sec (%.1f min)\n', parsim_elapsed, parsim_elapsed/60);
        fprintf(fid, '   Valid:       %d / %d (%.1f%%)\n', n_valid, N_total, 100*n_valid/N_total);
        fprintf(fid, '   Overcurrent: %d\n', n_overcurrent);
        fprintf(fid, '   Diverged:    %d\n', n_diverged);
        fprintf(fid, '   Sim Error:   %d\n\n', n_sim_error);
        fclose(fid);
    end

    %% =====================================================================
    %  7. 저장
    % =====================================================================
    fprintf('>> Saving to file...\n');
    
    I_cmd_all = P_all ./ V2_all;  % 참고용 전류 환산값
    
    dataset = table(P_all, V2_all, D1_all, D2_all, I_cmd_all, ...
                    i_pk_pri, i_pk_sec, D0_converged, valid_flag, ...
                    'VariableNames', {'P_W', 'V2_V', 'D1', 'D2', 'I_cmd_ref_A', ...
                                      'i_pk_pri_A', 'i_pk_sec_A', 'D0_norm', 'valid'});
    dataset_valid = dataset(valid_flag, :);
    
    meta = struct();
    meta.spec = spec;
    meta.grid = struct('N_steps', N_steps, 'P_vec', P_vec, 'V2_vec', V2_vec, ...
                       'D1_vec', D1_vec, 'D2_vec', D2_vec);
    meta.sim = struct('SIM_TIME', SIM_TIME, 'SS_START', SS_START, ...
                      'Kp', Kp_val, 'Ki', Ki_val, 'DeadTime_ns', DeadTime_ns_val);
    meta.stats = struct('N_total', N_total, 'N_valid', n_valid, ...
                        'N_overcurrent', n_overcurrent, 'N_diverged', n_diverged, ...
                        'N_sim_error', n_sim_error);
    meta.timestamp = timestamp;
    meta.elapsed_sec = parsim_elapsed + setup_elapsed;
    
    save(resultFile, 'dataset', 'dataset_valid', 'meta', 'error_msg', '-v7.3');
    fprintf('   MAT: %s\n', resultFile);
    
    writetable(dataset_valid, csvFile);
    fprintf('   CSV: %s\n', csvFile);
    
    %% =====================================================================
    %  8. 시각화
    % =====================================================================
    if n_valid > 0
        figure('Name', 'TPS Dataset Overview', 'Position', [50 50 1400 900]);
        
        subplot(2,3,1);
        scatter(dataset_valid.P_W, dataset_valid.i_pk_pri_A, 2, dataset_valid.D1, 'filled');
        colorbar; xlabel('P (W)'); ylabel('i_{pk,pri} (A)');
        title('i_{pk} vs Power (color:D1)'); grid on;
        
        subplot(2,3,2);
        scatter(dataset_valid.V2_V, dataset_valid.i_pk_pri_A, 2, dataset_valid.D2, 'filled');
        colorbar; xlabel('V_2 (V)'); ylabel('i_{pk,pri} (A)');
        title('i_{pk} vs Voltage (color:D2)'); grid on;
        
        subplot(2,3,3);
        histogram(dataset_valid.i_pk_pri_A, 50, 'FaceColor', [0.2 0.6 1.0]);
        xlabel('i_{pk,pri} (A)'); ylabel('Count');
        title(sprintf('Distribution (N=%d)', n_valid)); grid on;
        
        subplot(2,3,4);
        P_mid = P_vec(round(N_steps/2)); V2_mid = V2_vec(round(N_steps/2));
        mask_mid = abs(dataset_valid.P_W - P_mid) < 50 & abs(dataset_valid.V2_V - V2_mid) < 2;
        if sum(mask_mid) > 10
            scatter(dataset_valid.D1(mask_mid), dataset_valid.D2(mask_mid), ...
                    20, dataset_valid.i_pk_pri_A(mask_mid), 'filled');
            colorbar; xlabel('D_1'); ylabel('D_2');
            title(sprintf('i_{pk} (P≈%.0fW, V2≈%.0fV)', P_mid, V2_mid)); grid on;
        end
        
        subplot(2,3,5);
        pie([n_valid, n_overcurrent, n_diverged, n_sim_error], ...
            {sprintf('Valid(%d)',n_valid), sprintf('OC(%d)',n_overcurrent), ...
             sprintf('Div(%d)',n_diverged), sprintf('Err(%d)',n_sim_error)});
        title('Data Validity');
        
        subplot(2,3,6);
        scatter(dataset_valid.P_W, dataset_valid.V2_V, 2, dataset_valid.i_pk_pri_A, 'filled');
        colorbar; xlabel('P (W)'); ylabel('V_2 (V)');
        title('Grid Coverage'); grid on;
        
        sgtitle(sprintf('TPS Dataset: %d/%d valid (%.1f%%)', n_valid, N_total, 100*n_valid/N_total));
        savefig(gcf, fullfile(dataDir, sprintf('TPS_Dataset_%s.fig', timestamp)));
    end
    
    fprintf('\n=========================================================================\n');
    fprintf('  COMPLETE: %d/%d valid (%.1f%%), %.1f min\n', ...
        n_valid, N_total, 100*n_valid/N_total, meta.elapsed_sec/60);
    fprintf('=========================================================================\n');
end

%% Helper: 신호 데이터 추출 (프로젝트 표준 방식 적용)
function data = getSignalData_local(raw)
    if isa(raw, 'timeseries')
        data = squeeze(raw.Data);
    elseif isstruct(raw) && isfield(raw, 'signals')
        data = squeeze(raw.signals.values);
    elseif isa(raw, 'Simulink.SimulationData.Dataset')
        data = squeeze(raw.getElement(1).Values.Data);
    else
        data = squeeze(raw);
    end
end

function t = getSignalTime_local(raw, simOut)
    if isa(raw, 'timeseries')
        t = raw.Time;
    elseif isstruct(raw) && isfield(raw, 'time')
        t = raw.time;
    elseif isa(raw, 'Simulink.SimulationData.Dataset')
        t = raw.getElement(1).Values.Time;
    else
        if isprop(simOut, 'tout')
            t = simOut.tout;
        else
            t = [];
        end
    end
end

%% Helper: 메시지 터미널 출력 및 로그 파일 병렬 안전 기록
function logStatusMsg(msg, logFile)
    disp(msg);
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '[%s] %s\n', datestr(now, 'HH:MM:SS'), msg);
        fclose(fid);
    end
end
