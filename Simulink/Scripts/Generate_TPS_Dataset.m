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
    
    % 체크포인트 파일 (고정 이름 — 재개 시 동일 파일 사용)
    checkpointFile = fullfile(dataDir, 'TPS_Checkpoint.mat');
    BATCH_SIZE = 100;  % 배치 크기: 이 단위마다 중간 저장
    
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
    
    % 제어기 파라미터 (Power-based 제어이므로 극단적 포화 방지를 위해 스케일 조정)
    Kp_val = 0.05;
    Ki_val = 50.0;
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
    D1_vec = linspace(0.2, 0.5, N_steps);       % 1차 듀티
    D2_vec = linspace(0.2, 0.5, N_steps);       % 2차 듀티
    
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
    %  2.5 체크포인트 로드 (이전 진행분 복원)
    % =====================================================================
    % 결과를 저장할 벡터 사전 할당
    i_pk_pri = NaN(N_total, 1);
    i_pk_sec = NaN(N_total, 1);
    D0_converged = NaN(N_total, 1);
    P_actual_all = NaN(N_total, 1);
    valid_flag = false(N_total, 1);
    error_msg = cell(N_total, 1);
    completed_mask = false(N_total, 1);  % 시뮬레이션 시도 완료 여부 (성공/실패 모두)
    is_resume = false; % 체크포인트 로드 여부 플래그
    
    if exist(checkpointFile, 'file')
        fprintf('>> [RESUME] Checkpoint found: %s\n', checkpointFile);
        cp = load(checkpointFile);
        
        % 그리드 일관성 확인 (N_total이 같아야 안전하게 복원 가능)
        if isfield(cp, 'N_total_cp') && cp.N_total_cp == N_total
            i_pk_pri      = cp.i_pk_pri;
            i_pk_sec      = cp.i_pk_sec;
            D0_converged  = cp.D0_converged;
            P_actual_all  = cp.P_actual_all;
            valid_flag    = cp.valid_flag;
            error_msg     = cp.error_msg;
            completed_mask = cp.completed_mask;
            
            % 이전 타임스탬프/로그 복원
            if isfield(cp, 'timestamp_cp')
                timestamp  = cp.timestamp_cp;
                resultFile = fullfile(dataDir, sprintf('TPS_Dataset_%s.mat', timestamp));
                csvFile    = fullfile(dataDir, sprintf('TPS_Dataset_%s.csv', timestamp));
                logFile    = fullfile(dataDir, sprintf('TPS_Dataset_%s.log', timestamp));
            end
            
            n_done = sum(completed_mask);
            fprintf('   Restored %d / %d completed results (%.1f%%)\n', n_done, N_total, 100*n_done/N_total);
            fprintf('   Remaining: %d simulations\n\n', N_total - n_done);
            
            is_resume = true; % 복원 성공 시 플래그 설정
        else
            fprintf('   [WARNING] Checkpoint grid size mismatch (cp=%d vs current=%d). Starting fresh.\n\n', ...
                cp.N_total_cp, N_total);
            completed_mask = false(N_total, 1);
        end
    else
        fprintf('>> No checkpoint found. Starting fresh.\n\n');
    end
    
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
    % 캐시 충돌 방지를 위한 Simulink 캐시 끄기 및 삭제
    try
        if bdIsLoaded(modelName)
            set_param(modelName, 'SimulinkCacheManifest', 'off');
        end
        cacheFileSLXC = fullfile(modelDir, [modelName '.slxc']);
        if exist(cacheFileSLXC, 'file'), delete(cacheFileSLXC); end
        fprintf('   Simulink Cache (.slxc) cleared for parallel safety.\n');
    catch
    end

    % 사전에 막대한 SimulationInput 배열을 만들지 않고 Base 객체 1개만 만듭니다.
    tic;
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool;
    end
    
    t_input = [0; SIM_TIME];
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    baseIn = baseIn.setModelParameter('SimulationMode', 'normal');
    
    % 공통(상수) 파라미터 미리 셋팅
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_VL', timeseries(spec.V_out_nom * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kp_ext', timeseries(Kp_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Ki_ext', timeseries(Ki_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kpv', timeseries(Kpv_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Kiv', timeseries(Kiv_val * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_I_ess_cmd', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('load_R', timeseries(1000 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('timestamp', timeseries(12 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('V_Grid', timeseries(spec.V_in * ones(2,1), t_input)); % [CRITICAL FIX] 계통 전압 추가
    
    % [CRITICAL FIX] 제어 모드 강제 주입 (TPS 생성 전용)
    % Ctrl_Mode_Ext = 3 (TPS), Mode_Select = 2 (Auto Control)
    baseIn = baseIn.setVariable('Mode_Select', timeseries(2 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Phi_Manual', timeseries(0 * ones(2,1), t_input));
    
    % [CRITICAL FIX] 회로 파라미터 주입 (병렬 워커 빈 메모리 에러 해결)
    % V_init_LV와 V_batt는 각 시뮬레이션 케이스마다 V2_curr 비율로 동적 할당되므로, 여기서 초기 96 제거 (이후 testIn/simIn에서 주입)
    baseIn = baseIn.setVariable('C_HV', 1000e-6);
    baseIn = baseIn.setVariable('R_ESR_HV', 0.05);
    baseIn = baseIn.setVariable('R_ESR_LV', 0.01);
    baseIn = baseIn.setVariable('R_batt_int', 0.15);
    baseIn = baseIn.setVariable('Kp_ext', Kp_val);
    baseIn = baseIn.setVariable('Ki_ext', Ki_val);
    baseIn = baseIn.setVariable('Kpv', Kpv_val);
    baseIn = baseIn.setVariable('Kiv', Kiv_val);
    
    setup_elapsed = toc;
    fprintf('   Parallel Setup Complete (%.1f sec).\n\n', setup_elapsed);
    
    %% =====================================================================
    %  4.5 단일 테스트 시뮬레이션 (병렬 실행 전 검증)
    % =====================================================================
    if ~is_resume
        fprintf('>> Running Pre-Flight Single Simulation Test (2 cases)...\n');
        test_indices = [1, round(N_total/2)]; % 2번의 테스트로 변경
        
        for k_test = 1:length(test_indices)
        test_idx = test_indices(k_test);
        P_curr  = P_all(test_idx);
        V2_curr = V2_all(test_idx);
        D1_curr = D1_all(test_idx);
        D2_curr = D2_all(test_idx);
        
        fprintf('   --- Test Case %d (idx=%d) ---\n', k_test, test_idx);
        
        testIn = baseIn;
        testIn = testIn.setVariable('Ref_Power', timeseries(-P_curr * ones(2,1), t_input));
        I_ess_cmd_val = -P_curr / V2_curr;
        testIn = testIn.setVariable('Ref_I_ess_cmd', timeseries(I_ess_cmd_val * ones(2,1), t_input));
        testIn = testIn.setVariable('Ref_VL', timeseries(V2_curr * ones(2,1), t_input));
        testIn = testIn.setVariable('D1', timeseries(D1_curr * ones(2,1), t_input));
        testIn = testIn.setVariable('D2', timeseries(D2_curr * ones(2,1), t_input));
        testIn = testIn.setVariable('V_init_LV', V2_curr);
        testIn = testIn.setVariable('V_batt', V2_curr);
        
        % [FALLBACK] 마스크 에러 방지용으로 메인 작업공간에 변수 할당
        assignin('base', 'V_init_LV', V2_curr);  assignin('base', 'V_batt', V2_curr);
        assignin('base', 'C_HV', 1000e-6);       assignin('base', 'R_ESR_HV', 0.05);
        assignin('base', 'R_ESR_LV', 0.01);      assignin('base', 'R_batt_int', 0.15);
        assignin('base', 'Kp_ext', Kp_val);      assignin('base', 'Ki_ext', Ki_val);
        assignin('base', 'Kpv', Kpv_val);        assignin('base', 'Kiv', Kiv_val);
    
        if ~bdIsLoaded(modelName), load_system(modelName); end
        
        try
            testOut = sim(testIn);
            if ~isempty(testOut.ErrorMessage)
                logStatusMsg(sprintf('   [TEST FAILED] Error: %s', testOut.ErrorMessage), logFile);
            else
                out_data = testOut;
                I_L_data = []; t_sim = [];
                if isprop(out_data, 'Inductor_Current1')
                    raw_I = out_data.Inductor_Current1;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'Inductor_Current')
                    raw_I = out_data.Inductor_Current;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'yout') && isprop(out_data.yout, 'getElement')
                    try
                        raw_I = out_data.yout.getElement('Inductor_Current1');
                        I_L_data = getSignalData_local(raw_I);
                        t_sim = getSignalTime_local(raw_I, out_data);
                    catch
                        try
                            raw_I = out_data.yout.getElement('Inductor_Current');
                            I_L_data = getSignalData_local(raw_I);
                            t_sim = getSignalTime_local(raw_I, out_data);
                        catch, end
                    end
                end
                
                if isempty(I_L_data)
                    logStatusMsg('   [TEST FAILED] Inductor_Current1 data not found in simOut.', logFile);
                else
                    ss_mask_test = t_sim >= SS_START;
                    I_L_ss_test = I_L_data(ss_mask_test);
                    T_cycle_test = 1 / spec.fs;
                    dt_sim_test = mean(diff(t_sim(ss_mask_test)));
                    samples_per_cycle_test = max(2, round(T_cycle_test / dt_sim_test));
                    
                    if length(I_L_ss_test) >= samples_per_cycle_test
                        last_cycle_test = I_L_ss_test(end-samples_per_cycle_test+1:end);
                    else
                        last_cycle_test = I_L_ss_test;
                    end
                    i_pk_val_test = max(abs(last_cycle_test));
                    
                    phi_data_test = [];
                    if isprop(out_data, 'Phi_ext_out1')
                        phi_data_test = getSignalData_local(out_data.Phi_ext_out1);
                    elseif isprop(out_data, 'Phi_ext_out')
                        phi_data_test = getSignalData_local(out_data.Phi_ext_out);
                    end
                    phi_val_test = NaN;
                    if ~isempty(phi_data_test)
                        phi_val_test = phi_data_test(end) / 360;
                    end
                    
                    % 3. 실제 전력 직접 계산 (V_LV * I_LV)
                    p_actual_test = NaN;
                    v_lv_data_t = []; i_lv_data_t = [];
                    
                    if isprop(out_data, 'V_LV1'), v_lv_data_t = getSignalData_local(out_data.V_LV1);
                    elseif isprop(out_data, 'V_LV'), v_lv_data_t = getSignalData_local(out_data.V_LV); 
                    elseif isprop(out_data, 'yout')
                        try v_lv_data_t = getSignalData_local(out_data.yout.get('V_LV1')); catch
                            try v_lv_data_t = getSignalData_local(out_data.yout.get('V_LV')); catch, end
                        end
                    end
                    
                    if isprop(out_data, 'I_LV1'), i_lv_data_t = getSignalData_local(out_data.I_LV1);
                    elseif isprop(out_data, 'I_LV'), i_lv_data_t = getSignalData_local(out_data.I_LV); 
                    elseif isprop(out_data, 'yout')
                        try i_lv_data_t = getSignalData_local(out_data.yout.get('I_LV1')); catch
                            try i_lv_data_t = getSignalData_local(out_data.yout.get('I_LV')); catch, end
                        end
                    end
                    
                    if ~isempty(v_lv_data_t) && ~isempty(i_lv_data_t)
                        p_calc_data_t = v_lv_data_t .* i_lv_data_t;
                        p_calc_ss_t = p_calc_data_t(ss_mask_test);
                        p_actual_test = mean(p_calc_ss_t);
                    else
                        p_actual_test = P_curr; % Fallback
                    end
                    
                    testMsg = sprintf('   [TEST SUCCESS] idx(%d) P_cmd=%.0fW, P_act=%.1fW, V2=%.1fV -> i_pk_pri=%.2fA, D0_conv=%.3f', ...
                        test_idx, P_curr, p_actual_test, V2_curr, i_pk_val_test, phi_val_test);
                    logStatusMsg(testMsg, logFile);
                end
            end
        catch ME_test
            logStatusMsg(sprintf('   [TEST FATAL] %s', ME_test.message), logFile);
        end
    end
    fprintf('\n');
    else
        fprintf('>> Pre-Flight Test skipped (Resuming from checkpoint).\n\n');
    end

    %% =====================================================================
    %  5. 병렬 시뮬레이션 및 데이터 동시 추출 (배치 parsim + 체크포인트)
    % =====================================================================
    
    % 남은 인덱스만 추출
    remaining_indices = find(~completed_mask);
    N_remaining = length(remaining_indices);
    
    fprintf('>> Running simulations (%d remaining / %d total) via parsim...\n', N_remaining, N_total);
    fprintf('   Batch size: %d (checkpoint saved after each batch)\n\n', BATCH_SIZE);
    
    cd(modelDir);
    tic;
    
    % 배치 단위로 parsim 실행 + 각 배치 후 체크포인트 저장
    N_batches = ceil(N_remaining / BATCH_SIZE);
    
    for batchIdx = 1:N_batches
        % 이번 배치에 해당하는 remaining_indices 범위 계산
        batchStart = (batchIdx - 1) * BATCH_SIZE + 1;
        batchEnd   = min(batchIdx * BATCH_SIZE, N_remaining);
        batch_indices = remaining_indices(batchStart:batchEnd);
        n_batch = length(batch_indices);
        
        n_done_so_far = sum(completed_mask);
        fprintf('>> [Batch %d/%d] Processing indices %d~%d (%d sims) | Overall: %d/%d done (%.1f%%)\n', ...
            batchIdx, N_batches, batch_indices(1), batch_indices(end), n_batch, ...
            n_done_so_far, N_total, 100*n_done_so_far/N_total);
        
        % === parsim용 SimulationInput 배열 생성 ===
        simIn_array(1:n_batch) = Simulink.SimulationInput(modelName);
        
        for k = 1:n_batch
            P_curr  = P_all(batch_indices(k));
            V2_curr = V2_all(batch_indices(k));
            D1_curr = D1_all(batch_indices(k));
            D2_curr = D2_all(batch_indices(k));
            
            % baseIn의 모든 설정을 복사
            simIn_array(k) = baseIn;
            
            % 가변 변수 주입 (충전 방향 기준 전력)
            Power_cmd_val = -P_curr;
            I_ess_cmd_val = Power_cmd_val / V2_curr;
            
            simIn_array(k) = simIn_array(k).setVariable('Ref_Power', timeseries(Power_cmd_val * ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('Ref_I_ess_cmd', timeseries(I_ess_cmd_val * ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('Ref_VL', timeseries(V2_curr * ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D1', timeseries(D1_curr * ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D2', timeseries(D2_curr * ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('V_init_LV', V2_curr);
            simIn_array(k) = simIn_array(k).setVariable('V_batt', V2_curr);
        end
        
        % === parsim 실행 (안전장벽 적용) ===
        parsim_success = false;
        try
            simOut_array = parsim(simIn_array, ...
                'UseFastRestart', 'off', ...
                'ShowProgress', 'on', ...
                'TransferBaseWorkspaceVariables', 'on', ...
                'ShowSimulationManager', 'off', ...
                'SetupFcn', @()workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val));
            parsim_success = true;
        catch ME_parsim
            logStatusMsg(sprintf('   [BATCH %d FATAL] parsim crashed: %s', batchIdx, ME_parsim.message), logFile);
            for k = 1:n_batch
                global_idx = batch_indices(k);
                error_msg{global_idx} = sprintf('parsim fatal crash: %s', ME_parsim.message);
                completed_mask(global_idx) = true;
            end
        end
        
        % === parsim 결과 후처리 (기존 추출 로직 동일) ===
        if parsim_success
            for k = 1:n_batch
                global_idx = batch_indices(k);
            P_curr  = P_all(global_idx);
            V2_curr = V2_all(global_idx);
            D1_curr = D1_all(global_idx);
            D2_curr = D2_all(global_idx);
            
            try
                simOut = simOut_array(k);
                
                % parsim 에러 확인
                if ~isempty(simOut.ErrorMessage)
                    error_msg{global_idx} = simOut.ErrorMessage;
                    completed_mask(global_idx) = true;
                    logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                        global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, simOut.ErrorMessage), logFile);
                    continue;
                end
                
                out_data = simOut;
                I_L_data = [];
                t_sim = [];
                
                % 1. 인덕터 피크 추출
                if isprop(out_data, 'Inductor_Current1')
                    raw_I = out_data.Inductor_Current1;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'Inductor_Current')
                    raw_I = out_data.Inductor_Current;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'I_L1')
                    raw_I = out_data.I_L1;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'I_L')
                    raw_I = out_data.I_L;
                    I_L_data = getSignalData_local(raw_I);
                    t_sim = getSignalTime_local(raw_I, out_data);
                elseif isprop(out_data, 'yout')
                    try
                        raw_I = out_data.yout.get('Inductor_Current1');
                        I_L_data = getSignalData_local(raw_I);
                        t_sim = getSignalTime_local(raw_I, out_data);
                    catch
                        try
                            raw_I = out_data.yout.get('Inductor_Current');
                            I_L_data = getSignalData_local(raw_I);
                            t_sim = getSignalTime_local(raw_I, out_data);
                        catch, end
                    end
                end
                
                if isempty(I_L_data) || isempty(t_sim)
                    error_msg{global_idx} = 'Current data (Inductor_Current1) not found.';
                    completed_mask(global_idx) = true;
                    logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                        global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, error_msg{global_idx}), logFile);
                    continue;
                end
                
                % 정상상태 구간 슬라이싱
                ss_mask = t_sim >= SS_START;
                if sum(ss_mask) < 10
                    error_msg{global_idx} = 'Insufficient steady-state data.';
                    completed_mask(global_idx) = true;
                    logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                        global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, error_msg{global_idx}), logFile);
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
                    error_msg{global_idx} = sprintf('Overcurrent (i_pk=%.1fA)', i_pk_val);
                    completed_mask(global_idx) = true;
                    logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                        global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, error_msg{global_idx}), logFile);
                    continue;
                end
                
                % 클렌징 2: 발산/진동 감지
                mean_I = mean(abs(I_L_ss));
                std_I = std(I_L_ss);
                if std_I > 5 * mean_I && mean_I > 0.1
                    error_msg{global_idx} = 'PI divergence.';
                    completed_mask(global_idx) = true;
                    logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                        global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, error_msg{global_idx}), logFile);
                    continue;
                end
                
                % 2. 외부 위상 추출
                phi_data = [];
                if isprop(out_data, 'Phi_ext_out1')
                    phi_data = getSignalData_local(out_data.Phi_ext_out1);
                elseif isprop(out_data, 'Phi_ext_out')
                    phi_data = getSignalData_local(out_data.Phi_ext_out);
                elseif isprop(out_data, 'phi_ext_out1')
                    phi_data = getSignalData_local(out_data.phi_ext_out1);
                elseif isprop(out_data, 'phi_ext_out')
                    phi_data = getSignalData_local(out_data.phi_ext_out);
                elseif isprop(out_data, 'Phi_ext1')
                    phi_data = getSignalData_local(out_data.Phi_ext1);
                elseif isprop(out_data, 'Phi_ext')
                    phi_data = getSignalData_local(out_data.Phi_ext);
                end
                
                if ~isempty(phi_data)
                    D0_converged(global_idx) = phi_data(end) / 360;
                end
                
                % 3. 실제 전력 직접 계산 (V_LV * I_LV)
                p_actual_val = NaN;
                v_lv_data = []; i_lv_data = [];
                
                if isprop(out_data, 'V_LV1'), v_lv_data = getSignalData_local(out_data.V_LV1);
                elseif isprop(out_data, 'V_LV'), v_lv_data = getSignalData_local(out_data.V_LV); 
                elseif isprop(out_data, 'yout')
                    try v_lv_data = getSignalData_local(out_data.yout.get('V_LV1')); catch
                        try v_lv_data = getSignalData_local(out_data.yout.get('V_LV')); catch, end
                    end
                end
                
                if isprop(out_data, 'I_LV1'), i_lv_data = getSignalData_local(out_data.I_LV1);
                elseif isprop(out_data, 'I_LV'), i_lv_data = getSignalData_local(out_data.I_LV); 
                elseif isprop(out_data, 'yout')
                    try i_lv_data = getSignalData_local(out_data.yout.get('I_LV1')); catch
                        try i_lv_data = getSignalData_local(out_data.yout.get('I_LV')); catch, end
                    end
                end
                
                if ~isempty(v_lv_data) && ~isempty(i_lv_data)
                    p_calc_data = v_lv_data .* i_lv_data;
                    p_calc_ss = p_calc_data(ss_mask);
                    p_actual_val = mean(p_calc_ss);
                else
                    p_actual_val = P_curr; % Fallback
                end
                
                P_actual_all(global_idx) = p_actual_val;
                
                % 안전한 값으로 저장
                i_pk_pri(global_idx) = i_pk_val;
                i_pk_sec(global_idx) = i_pk_val * spec.n;
                valid_flag(global_idx) = true;
                completed_mask(global_idx) = true;
                
                logStatusMsg(sprintf('   [%d / %d] SUCCESS. P_cmd=%.0fW, P_act=%.1fW, V2=%.1fV, D1=%.2f, D2=%.2f -> i_pk_pri=%.2fA, D0_conv=%.3f', ...
                    global_idx, N_total, P_curr, p_actual_val, V2_curr, D1_curr, D2_curr, i_pk_val, D0_converged(global_idx)), logFile);
                
            catch ME_inner
                error_msg{global_idx} = sprintf('Error: %s', ME_inner.message);
                completed_mask(global_idx) = true;
                logStatusMsg(sprintf('   [%d / %d] FAILED. P_cmd=%.0fW, V2=%.1fV, D1=%.2f, D2=%.2f | Error: %s', ...
                    global_idx, N_total, P_curr, V2_curr, D1_curr, D2_curr, error_msg{global_idx}), logFile);
            end
        end  % 후처리 for loop end
        end  % if parsim_success end
        
        % 메모리 해제
        if parsim_success
            clear simOut_array;
        end
        clear simIn_array;
        
        % === 체크포인트 저장 (Safe Save 방식 적용) ===
        N_total_cp    = N_total;  %#ok<NASGU>
        timestamp_cp  = timestamp;  %#ok<NASGU>
        
        tempCheckpoint = fullfile(dataDir, 'TPS_Checkpoint_temp.mat');
        try
            save(tempCheckpoint, 'i_pk_pri', 'i_pk_sec', 'D0_converged', 'P_actual_all', ...
                 'valid_flag', 'error_msg', 'completed_mask', 'N_total_cp', 'timestamp_cp', '-v7.3');
            movefile(tempCheckpoint, checkpointFile, 'f');
            
            n_done_now = sum(completed_mask);
            fprintf('   [Checkpoint Saved (Safe)] %d/%d done (%.1f%%) -> %s\n\n', ...
                n_done_now, N_total, 100*n_done_now/N_total, checkpointFile);
        catch ME_save
            logStatusMsg(sprintf('   [SAVE ERROR] Disk full or write error: %s\n', ME_save.message), logFile);
        end
        
        % 메모리 누수 방지용: 10 배치마다 Parallel Pool 리셋 (가비지 컬렉션)
        if mod(batchIdx, 10) == 0
            fprintf('   [Memory Flush] Restarting parallel pool to prevent memory leaks...\n');
            delete(gcp('nocreate'));
            parpool;
        end
        
    end  % batch loop end
    
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
    fprintf('   Parsim Runtime: %.1f sec (%.1f min)\n', parsim_elapsed, parsim_elapsed/60);
    fprintf('   Valid:       %d / %d (%.1f%%)\n', n_valid, N_total, 100*n_valid/N_total);
    fprintf('   Overcurrent: %d\n', n_overcurrent);
    fprintf('   Diverged:    %d\n', n_diverged);
    fprintf('   Sim Error:   %d\n\n', n_sim_error);
    
    % 통계결과를 로그 파일에도 기록
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '\n>> Simulation & Extraction complete!\n');
        fprintf(fid, '   Parsim Runtime: %.1f sec (%.1f min)\n', parsim_elapsed, parsim_elapsed/60);
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
    
    dataset = table(P_actual_all, P_all, V2_all, D1_all, D2_all, I_cmd_all, ...
                    i_pk_pri, i_pk_sec, D0_converged, valid_flag, ...
                    'VariableNames', {'P_W', 'P_cmd_W', 'V2_V', 'D1', 'D2', 'I_cmd_ref_A', ...
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
    
    % 체크포인트 파일 정리 (완료됨으로 이름 변경)
    if exist(checkpointFile, 'file')
        doneCheckpoint = fullfile(dataDir, sprintf('TPS_Checkpoint_done_%s.mat', timestamp));
        movefile(checkpointFile, doneCheckpoint);
        fprintf('   Checkpoint archived: %s\n', doneCheckpoint);
    end
    
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
        mask_mid = abs(dataset_valid.P_W - P_mid) < 100 & abs(dataset_valid.V2_V - V2_mid) < 2;
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

%% Helper: parsim SetupFcn — 워커 초기화 시 base workspace에 회로 파라미터 주입
function workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val)
    assignin('base', 'C_HV', 1000e-6);
    assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01);
    assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp_val);
    assignin('base', 'Ki_ext', Ki_val);
    assignin('base', 'Kpv', Kpv_val);
    assignin('base', 'Kiv', Kiv_val);
end
