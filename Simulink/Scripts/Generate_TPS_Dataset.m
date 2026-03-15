%% Generate_TPS_Dataset.m
% =========================================================================
% DAB Converter TPS Dataset Generator - Stage I
% (V2, D0, D1, D2 Grid via Time-Series Staircase)
% =========================================================================

function Generate_TPS_Dataset()
    %% 0. 환경 설정
    clear; clc; close all;
    
    fprintf('=========================================================================\n');
    fprintf('  DAB TPS Dataset Generator (Time-Series Staircase Mode)\n');
    fprintf('  Target: V2, D0, D1, D2 -> i_pk_pri, Power_LV, Power_HV\n');
    fprintf('=========================================================================\n\n');
    
    scriptDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    dataDir = fullfile(projectDir, 'Data');
    if ~exist(dataDir, 'dir'), mkdir(dataDir); end
    
    checkpointFile = fullfile(dataDir, 'TPS_Checkpoint_Stairs.mat');
    BATCH_SIZE = 12;  % 배치 사이즈 (파라미터 조합 묶음)
    
    timestamp = datestr(now, 'yyyymmdd_HHMMss');
    resultFile = fullfile(dataDir, sprintf('TPS_Dataset_%s.mat', timestamp));
    csvFile    = fullfile(dataDir, sprintf('TPS_Dataset_%s.csv', timestamp));
    logFile    = fullfile(dataDir, sprintf('TPS_Dataset_%s.log', timestamp));
    
    logID = fopen(logFile, 'w');
    fprintf(logID, 'TPS Dataset Generation Log (Staircase Mode)\nStarted: %s\n', datestr(now));
    fprintf(logID, '=========================================\n\n');
    fclose(logID);
    
    %% 1. 시스템 사양 정의
    spec = struct();
    spec.V_in      = 380;       % V (고정)
    spec.n         = 4;
    spec.fs        = 80e3;
    spec.I_pri_max = 10;
    spec.I_sec_max = 40;
    
    % 시뮬레이션 설정 (다단 스텝)
    SIM_TIME = 0.044;           % 44ms (위상 -90~90도 확장에 따른 시간 증가)
    T_STARTUP = 0.005;          % 5ms 초기 안정화 시간
    T_STEP    = 0.001;          % 매 스텝간 1ms
    N_D0_STEPS = 39;            % 39계단 (-90도 ~ 90도)
    
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    %% 2. 파라미터 그리드 생성 (V2, D1, D2 중심)
    N_V2 = 20; N_D1 = 20; N_D2 = 20;
    
    V2_vec = linspace(84, 109.2, N_V2);
    D1_vec = linspace(0.2, 0.5, N_D1);
    D2_vec = linspace(0.2, 0.5, N_D2);
    
    % 전력 변화가 0도 근처에서 극심하고 +/-90도에서 둔감한 DAB의 특성 반영
    % 전력 결과값이 선형적인 간격으로 찍히도록 위상 배분을 1 - sqrt(1 - |x|) 형태로 비선형 인가
    norm_x = linspace(-1, 1, N_D0_STEPS);
    D0_vec = sign(norm_x) .* 90 .* (1 - sqrt(1 - abs(norm_x)));
    
    [V2_grid, D1_grid, D2_grid] = ndgrid(V2_vec, D1_vec, D2_vec);
    V2_all = V2_grid(:);
    D1_all = D1_grid(:);
    D2_all = D2_grid(:);
    N_sims  = length(V2_all);
    N_total = N_sims * N_D0_STEPS;
    
    fprintf('>> Parameter Grid:\n');
    fprintf('   V2: %d steps, D1: %d steps, D2: %d steps => %d combinations\n', N_V2, N_D1, N_D2, N_sims);
    fprintf('   D0: %d steps inside each simulation (%.1f ~ %.1f deg)\n', N_D0_STEPS, D0_vec(1), D0_vec(end));
    fprintf('   Total Dataset Rows: %d\n\n', N_total);
    
    %% 2.5 D0 Timeseries (Zero-Order Hold 계단형) 생성
    % 첫 구간(0 ~ 0.006초)은 초기 안정화를 위해 D0_vec(1)을 인가
    % 이후 매 0.001초 간격으로 다음 위상으로 갱신
    T_STARTUP = 0.006;          
    T_STEP    = 0.001;          
    
    t_ts = [0];
    d_ts = [D0_vec(1)];
    for k = 1:(N_D0_STEPS-1)
        % k=1: 0.006초 시점에 D0_vec(2)로 변경됨
        t_change = T_STARTUP + (k-1) * T_STEP; 
        t_ts(end+1) = t_change;
        d_ts(end+1) = D0_vec(k+1); 
    end
    ts_D0 = timeseries(d_ts', t_ts');
    ts_D0.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');
    
    %% 3. 체크포인트 및 변수 할당
    dataset_V2 = NaN(N_total, 1);
    dataset_D0 = NaN(N_total, 1);
    dataset_D1 = NaN(N_total, 1);
    dataset_D2 = NaN(N_total, 1);
    dataset_I_pk = NaN(N_total, 1);
    dataset_P_act = NaN(N_total, 1);     % Power_LV
    dataset_P_act_HV = NaN(N_total, 1);  % Power_HV 추가
    dataset_valid = false(N_total, 1);
    
    completed_sims = false(N_sims, 1);
    error_msg = cell(N_sims, 1);
    
    if exist(checkpointFile, 'file')
        fprintf('>> [RESUME] Checkpoint found: %s\n', checkpointFile);
        try
            cp = load(checkpointFile);
            if isfield(cp, 'N_sims_cp') && cp.N_sims_cp == N_sims
                dataset_V2 = cp.dataset_V2; dataset_D0 = cp.dataset_D0;
                dataset_D1 = cp.dataset_D1; dataset_D2 = cp.dataset_D2;
                dataset_I_pk = cp.dataset_I_pk; 
                dataset_P_act = cp.dataset_P_act;
                if isfield(cp, 'dataset_P_act_HV')
                    dataset_P_act_HV = cp.dataset_P_act_HV;
                end
                dataset_valid = cp.dataset_valid; completed_sims = cp.completed_sims;
                error_msg = cp.error_msg;
                n_done = sum(completed_sims);
                fprintf('   Restored %d / %d completed simulations.\n\n', n_done, N_sims);
            else
                fprintf('   Grid size mismatch! Starting fresh.\n\n');
                completed_sims = false(N_sims, 1);
            end
        catch
            fprintf('   Failed to load checkpoint file. Starting fresh.\n\n');
            completed_sims = false(N_sims, 1);
        end
    end
    
    %% 4. 시뮬레이션 풀 셋업
    addpath(projectDir); addpath(fullfile(projectDir, 'Modules'));
    modelName = 'Control_Logic';
    modelDir = fullfile(projectDir, 'Modules');
    originalDir = pwd;
    
    poolobj = gcp('nocreate');
    if isempty(poolobj), parpool; end
    
    t_input = [0; SIM_TIME];
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    
    % 공통 파라미터 (상수)
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('V_Grid', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Power', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_I_ess_cmd', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('Mode_Select', timeseries(1 * ones(2,1), t_input)); % Mode=1 (완전 매뉴얼)
    baseIn = baseIn.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2,1), t_input)); 
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2,1), t_input));
    
    % D0 (위상)는 ZOH Timeseries로 인가하여 매 시뮬레이션 내에서 실시간 변동 적용 
    baseIn = baseIn.setVariable('D0', ts_D0);
    baseIn = baseIn.setVariable('Phi_Manual', ts_D0);  
    
    baseIn = baseIn.setVariable('C_HV', 1000e-6);
    baseIn = baseIn.setVariable('R_ESR_HV', 0.05);
    baseIn = baseIn.setVariable('R_ESR_LV', 0.01);
    baseIn = baseIn.setVariable('R_batt_int', 0.15);
    baseIn = baseIn.setVariable('Kp_ext', Kp_val);
    baseIn = baseIn.setVariable('Ki_ext', Ki_val);
    baseIn = baseIn.setVariable('Kpv', Kpv_val);
    baseIn = baseIn.setVariable('Kiv', Kiv_val);
    
    %% 5. 시뮬레이션 실행 (배치)
    remaining_indices = find(~completed_sims);
    N_remaining = length(remaining_indices);
    fprintf('>> Running simulations (%d remaining) via parsim...\n', N_remaining);
    
    cd(modelDir);
    tic;
    
    N_batches = ceil(N_remaining / BATCH_SIZE);
    
    for batchIdx = 1:N_batches
        batchStart = (batchIdx - 1) * BATCH_SIZE + 1;
        batchEnd   = min(batchIdx * BATCH_SIZE, N_remaining);
        batch_indices = remaining_indices(batchStart:batchEnd);
        n_batch = length(batch_indices);
        
        simIn_array(1:n_batch) = Simulink.SimulationInput(modelName);
        for k = 1:n_batch
            idx = batch_indices(k);
            simIn_array(k) = baseIn;
            % V2, D1, D2는 해당 시뮬레이션 내에서는 고정값 유지
            simIn_array(k) = simIn_array(k).setVariable('V_init_LV', V2_all(idx));
            simIn_array(k) = simIn_array(k).setVariable('V_batt', V2_all(idx));
            simIn_array(k) = simIn_array(k).setVariable('Ref_VL', timeseries(V2_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D1', timeseries(D1_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D2', timeseries(D2_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D1_ext_in', timeseries(D1_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D2_ext_in', timeseries(D2_all(idx)*ones(2,1), t_input));
            
            % [메모리 개선] 워커(Worker) 단에서 데이터를 바로 처리하고 무거운 파형을 버리도록 PostSimFcn 등록
            simIn_array(k) = simIn_array(k).setPostSimFcn(@(out) processSimResult(out, T_STARTUP, T_STEP, N_D0_STEPS));
        end
        
        parsim_success = false;
        try
            simOut_array = parsim(simIn_array, 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                 'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', ...
                 'SetupFcn', @()workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val));
            parsim_success = true;
        catch ME
            logStatusMsg(sprintf('[BATCH %d] Parsim crash: %s', batchIdx, ME.message), logFile);
            for k = 1:n_batch, completed_sims(batch_indices(k)) = true; end
        end
        
        if parsim_success
            for k = 1:n_batch
                sim_idx = batch_indices(k);
                simOut = simOut_array(k);
                
                % 워커에서 처리되어 반환된 요약 데이터 호출
                res = simOut.getUserData();
                
                if ~isempty(simOut.ErrorMessage) || (~isempty(res) && ~isempty(res.error_msg))
                    if ~isempty(simOut.ErrorMessage)
                        error_msg{sim_idx} = simOut.ErrorMessage;
                    else
                        error_msg{sim_idx} = res.error_msg;
                    end
                    completed_sims(sim_idx) = true;
                    continue;
                end
                
                % 요약 데이터가 성공적으로 있는 경우 병합
                if ~isempty(res)
                    for d_idx = 1:N_D0_STEPS
                        if res.valid(d_idx)
                            row_idx = (sim_idx - 1) * N_D0_STEPS + d_idx;
                            
                            dataset_V2(row_idx) = V2_all(sim_idx);
                            dataset_D1(row_idx) = D1_all(sim_idx);
                            dataset_D2(row_idx) = D2_all(sim_idx);
                            dataset_D0(row_idx) = D0_vec(d_idx);
                            dataset_I_pk(row_idx) = res.i_pk(d_idx);
                            dataset_P_act(row_idx) = res.p_act(d_idx);
                            dataset_P_act_HV(row_idx) = res.p_act_hv(d_idx);
                            dataset_valid(row_idx) = true;
                            
                            % 명령창 및 로그 파일에 성공 상태 출력
                            logStatusMsg(sprintf('   [Sim %d / %d] SUCCESS. V2=%.1fV, D1=%.2f, D2=%.2f, D0=%.1fdeg -> i_pk_pri=%.2fA, P_act_LV=%.1fW, P_act_HV=%.1fW', ...
                                sim_idx, N_sims, V2_all(sim_idx), D1_all(sim_idx), D2_all(sim_idx), D0_vec(d_idx), res.i_pk(d_idx), res.p_act(d_idx), res.p_act_hv(d_idx)), logFile);
                        end
                    end
                end
                completed_sims(sim_idx) = true;
            end
        end
        
        % 체크포인트 단기 저장 (디스크 오버로딩 방지)
        clear simIn_array simOut_array;
        N_sims_cp = N_sims;
        save('TPS_temp.mat', 'dataset_V2','dataset_D1','dataset_D2','dataset_D0','dataset_I_pk','dataset_P_act','dataset_P_act_HV','dataset_valid','completed_sims','error_msg','N_sims_cp', '-v7.3');
        movefile('TPS_temp.mat', checkpointFile, 'f');
        fprintf('   [Batch %d/%d] Saved %d/%d sims.\n', batchIdx, N_batches, sum(completed_sims), N_sims);
        
        % [메모리 누수 방지] 일정 배치(예: 50배치 = 600개) 주기로 parpool 강제 재시작 (메모리 완전 반환)
        if mod(batchIdx, 50) == 0 && batchIdx < N_batches
            fprintf('   [Memory Flush] Restarting parallel pool to free memory...\n');
            logStatusMsg(sprintf('[Memory Flush] Restarting parpool at Batch %d', batchIdx), logFile);
            delete(gcp('nocreate'));
            parpool;
        end
    end
    
    cd(originalDir);
    
    %% 6. 정리 및 파일 저장
    valid_mask = dataset_valid;
    n_valid = sum(valid_mask);
    
    % 데이터 셋 테이블 구성 시 Power_HV 추가
    dataset = table(dataset_P_act(valid_mask), dataset_P_act_HV(valid_mask), dataset_V2(valid_mask), ...
                    dataset_D0(valid_mask), dataset_D1(valid_mask), dataset_D2(valid_mask), ...
                    dataset_I_pk(valid_mask), dataset_I_pk(valid_mask)*spec.n, ...
                    'VariableNames', {'P_act_W', 'P_act_HV_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'i_pk_pri_A', 'i_pk_sec_A'});
    
    save(resultFile, 'dataset', 'error_msg', '-v7.3');
    writetable(dataset, csvFile);
    
    fprintf('>> Generated Dataset: %d valid rows out of %d. Saved to:\n   %s\n', n_valid, N_total, resultFile);
    
    % 체크포인트 정리
    if n_valid > 0
        doneCheckpoint = fullfile(dataDir, sprintf('TPS_Checkpoint_done_%s.mat', timestamp));
        movefile(checkpointFile, doneCheckpoint, 'f');
    end
end

%% Helpers
function [data, t] = extract_signal(simOut, names)
    data = []; t = [];
    for k = 1:length(names)
        if isprop(simOut, names{k})
            raw = simOut.(names{k});
            if isa(raw, 'timeseries')
                data = squeeze(raw.Data); t = raw.Time; return;
            end
        elseif isprop(simOut, 'yout')
            try
                raw = simOut.yout.getElement(names{k});
                if ~isempty(raw)
                    data = squeeze(raw.Values.Data); t = raw.Values.Time; return;
                end
            catch, end
            try
                raw = simOut.yout.get(names{k});
                if ~isempty(raw)
                    data = squeeze(raw.Values.Data); t = raw.Values.Time; return;
                end
            catch, end
        end
    end
end

function logStatusMsg(msg, logFile)
    disp(msg);
    fid = fopen(logFile, 'a');
    if fid ~= -1
        fprintf(fid, '[%s] %s\n', datestr(now, 'HH:MM:SS'), msg);
        fclose(fid);
    end
end

function workerSetupFcn(Kp, Ki, Kpv, Kiv)
    assignin('base', 'C_HV', 1000e-6); assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01); assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp); assignin('base', 'Ki_ext', Ki);
    assignin('base', 'Kpv', Kpv); assignin('base', 'Kiv', Kiv);
end

% === 메모리 절약을 위한 워커 노드 후처리 함수 ===
function newSimOut = processSimResult(simOut, T_STARTUP, T_STEP, N_D0_STEPS)
    % 워커에서 시뮬레이션 직후 실행. 무거운 파형 데이터를 메모리에서 버리고 필요한 결과만 추출
    res = struct();
    res.error_msg = '';
    res.p_act = NaN(N_D0_STEPS, 1);
    res.p_act_hv = NaN(N_D0_STEPS, 1);
    res.i_pk = NaN(N_D0_STEPS, 1);
    res.valid = false(N_D0_STEPS, 1);
    
    if ~isempty(simOut.ErrorMessage)
        res.error_msg = simOut.ErrorMessage;
        newSimOut = Simulink.SimulationOutput;
        newSimOut = newSimOut.setUserData(res);
        return;
    end
    
    try
        [I_L_data, t_sim] = extract_signal(simOut, {'Inductor_Current1','Inductor_Current','I_L1','I_L'});
        [P_lv_data, t_plv] = extract_signal(simOut, {'Power_LV'});
        [P_hv_data, t_phv] = extract_signal(simOut, {'Power_HV'}); 
        
        if isempty(P_lv_data)
            [v_lv,  t_v] = extract_signal(simOut, {'V_LV1', 'V_LV'});
            [i_lv,  ~]   = extract_signal(simOut, {'I_LV1', 'I_LV'});
            if ~isempty(v_lv) && ~isempty(i_lv), P_lv_data = v_lv .* i_lv; t_plv = t_v; end
        end
        if isempty(P_hv_data)
            [v_hv,  t_v_hv] = extract_signal(simOut, {'V_HV1', 'V_HV'});
            [i_hv,  ~]   = extract_signal(simOut, {'I_HV1', 'I_HV'});
            if ~isempty(v_hv) && ~isempty(i_hv), P_hv_data = v_hv .* i_hv; t_phv = t_v_hv; end
        end
        
        if isempty(I_L_data) || isempty(P_lv_data) || isempty(P_hv_data)
            res.error_msg = 'Required signals not found.';
        else
            for d_idx = 1:N_D0_STEPS
                if d_idx == 1
                    t_end_step = T_STARTUP;
                else
                    t_end_step = T_STARTUP + (d_idx - 1) * T_STEP;
                end
                
                % [요청 반영] 인덕터 피크, 전력 평균 모두 "스텝 직전 0.1ms"로 통일
                t_start_step = t_end_step - 0.0001; 
                
                mask_i = (t_sim >= t_start_step) & (t_sim < t_end_step - 1e-6);
                mask_p_lv = (t_plv >= t_start_step) & (t_plv < t_end_step - 1e-6);
                mask_p_hv = (t_phv >= t_start_step) & (t_phv < t_end_step - 1e-6);
                
                if sum(mask_i) > 5
                    res.i_pk(d_idx) = max(abs(I_L_data(mask_i)));
                    res.p_act(d_idx) = mean(P_lv_data(mask_p_lv));
                    res.p_act_hv(d_idx) = mean(P_hv_data(mask_p_hv));
                    res.valid(d_idx) = true;
                end
            end
        end
    catch ME
        res.error_msg = ME.message;
    end
    
    % 가벼운 객체 생성 후 메인 스레드로 반환 (이전 무거운 파형 데이터들은 메모리 해제됨)
    newSimOut = Simulink.SimulationOutput;
    newSimOut = newSimOut.setUserData(res);
end