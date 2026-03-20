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
    BATCH_SIZE = 6;  % 배치 사이즈 (파라미터 조합 묶음)
    
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
    T_WINDOW  = 0.0001;         % [수동 설정] 데이터 추출 구간 (0.1ms = 0.0001, 0.5ms = 0.0005)
    N_D0_STEPS = 39;            % 39계단 (-90도 ~ 90도)
    
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    %% 2. 파라미터 그리드 생성 (V2, D1, D2 중심)
    N_V2 = 20; N_D1 = 20; N_D2 = 20;
    
    V2_vec = linspace(84, 109.2, N_V2);
    D1_vec = linspace(0.2, 0.5, N_D1);
    D2_vec = linspace(0.2, 0.5, N_D2);
    
    % 전력 변화가 0도 근처에서 극심하고 +/-90도에서 둔감한 DAB의 특성 반영
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
    T_STARTUP = 0.006;          
    T_STEP    = 0.001;          
    
    t_ts = [0];
    d_ts = [D0_vec(1)];
    for k = 1:(N_D0_STEPS-1)
        t_change = T_STARTUP + (k-1) * T_STEP; 
        t_ts(end+1) = t_change;
        d_ts(end+1) = D0_vec(k+1); 
    end
    
    % 시뮬레이션 종료 시간에 대한 값을 명시적으로 추가하여 시간 초과 에러 방지
    if t_ts(end) < SIM_TIME
        t_ts(end+1) = SIM_TIME;
        d_ts(end+1) = d_ts(end);
    end
    
    ts_D0 = timeseries(d_ts', t_ts');
    ts_D0.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');
    
    %% 3. 체크포인트 및 변수 할당
    dataset_V2 = NaN(N_total, 1);
    dataset_D0 = NaN(N_total, 1);
    dataset_D1 = NaN(N_total, 1);
    dataset_D2 = NaN(N_total, 1);
    dataset_I_pk = NaN(N_total, 1);
    dataset_P_act = NaN(N_total, 1);     
    dataset_P_act_HV = NaN(N_total, 1);  
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
    baseIn = baseIn.setVariable('Mode_Select', timeseries(1 * ones(2,1), t_input)); 
    baseIn = baseIn.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2,1), t_input)); 
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2,1), t_input));
    
    baseIn = baseIn.setVariable('D0', ts_D0);
    baseIn = baseIn.setVariable('Phi_Manual', ts_D0);  
    
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
            
            % Workspace를 명확히 'base' 또는 모델로 지정
            simIn_array(k) = simIn_array(k).setVariable('V_init_LV', V2_all(idx), 'Workspace', 'Control_Logic');
            simIn_array(k) = simIn_array(k).setVariable('V_batt', V2_all(idx), 'Workspace', 'Control_Logic');
            simIn_array(k) = simIn_array(k).setVariable('Ref_VL', timeseries(V2_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D1', timeseries(D1_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D2', timeseries(D2_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D1_ext_in', timeseries(D1_all(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('D2_ext_in', timeseries(D2_all(idx)*ones(2,1), t_input));
            
            % [수정] 오류를 유발하던 PostSimFcn 완전히 제거
        end
        
        parsim_success = false;
        try
            simOut_array = parsim(simIn_array, 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                 'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', ...
                 'SetupFcn', @()workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val, spec));
            parsim_success = true;
        catch ME
            logStatusMsg(sprintf('[BATCH %d] Parsim crash: %s', batchIdx, ME.message), logFile);
            for k = 1:n_batch, completed_sims(batch_indices(k)) = true; end
        end
        
        if parsim_success
            for k = 1:n_batch
                sim_idx = batch_indices(k);
                simOut = simOut_array(k);
                
                % 1. 시뮬레이션 컴파일/초기화 에러 확인
                if ~isempty(simOut.ErrorMessage)
                    error_msg{sim_idx} = simOut.ErrorMessage;
                    completed_sims(sim_idx) = true;
                    logStatusMsg(sprintf('   [Sim %d / %d] FAILED. Error: %s', sim_idx, N_sims, simOut.ErrorMessage), logFile);
                    continue;
                end
                
                % 2. 메인 스레드에서 직접 분석 수행 (안전한 기존 방식, T_WINDOW 전달)
                res = analyze_sim_data(simOut, T_STARTUP, T_STEP, N_D0_STEPS, T_WINDOW);
                
                if ~isempty(res.error_msg)
                    error_msg{sim_idx} = res.error_msg;
                    completed_sims(sim_idx) = true;
                    logStatusMsg(sprintf('   [Sim %d / %d] FAILED. Error: %s', sim_idx, N_sims, res.error_msg), logFile);
                    continue;
                end
                
                % 3. 정상적으로 데이터가 추출된 경우 할당 및 출력
                data_found = false;
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
                        data_found = true;
                        
                        % 명령창 출력 부분 복원!
                        logStatusMsg(sprintf('   [Sim %d / %d] SUCCESS. V2=%.1fV, D1=%.2f, D2=%.2f, D0=%.1fdeg -> i_pk_pri=%.2fA, P_act_LV=%.1fW, P_act_HV=%.1fW', ...
                            sim_idx, N_sims, V2_all(sim_idx), D1_all(sim_idx), D2_all(sim_idx), D0_vec(d_idx), res.i_pk(d_idx), res.p_act(d_idx), res.p_act_hv(d_idx)), logFile);
                    end
                end
                
                if ~data_found
                    logStatusMsg(sprintf('   [Sim %d / %d] WARNING: Sim succeeded, but no data points found. Check step size.', sim_idx, N_sims), logFile);
                end
                
                completed_sims(sim_idx) = true;
            end
        end
        
        % 파형 데이터가 담긴 137MB 객체들을 Batch마다 완전히 날려서 메모리 확보
        clear simIn_array simOut_array simOut;
        N_sims_cp = N_sims;
        save('TPS_temp.mat', 'dataset_V2','dataset_D1','dataset_D2','dataset_D0','dataset_I_pk','dataset_P_act','dataset_P_act_HV','dataset_valid','completed_sims','error_msg','N_sims_cp', '-v7.3');
        movefile('TPS_temp.mat', checkpointFile, 'f');
        fprintf('   [Batch %d/%d] Saved %d/%d sims.\n', batchIdx, N_batches, sum(completed_sims), N_sims);
        
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
    
    dataset = table(dataset_P_act(valid_mask), dataset_P_act_HV(valid_mask), dataset_V2(valid_mask), ...
                    dataset_D0(valid_mask), dataset_D1(valid_mask), dataset_D2(valid_mask), ...
                    dataset_I_pk(valid_mask), dataset_I_pk(valid_mask)*spec.n, ...
                    'VariableNames', {'P_act_W', 'P_act_HV_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'i_pk_pri_A', 'i_pk_sec_A'});
    
    save(resultFile, 'dataset', 'error_msg', '-v7.3');
    writetable(dataset, csvFile);
    
    fprintf('>> Generated Dataset: %d valid rows out of %d. Saved to:\n   %s\n', n_valid, N_total, resultFile);
    
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
        elseif isprop(simOut, 'logsout')
            try
                raw = simOut.logsout.get(names{k});
                if ~isempty(raw) && isa(raw.Values, 'timeseries')
                    data = squeeze(raw.Values.Data); t = raw.Values.Time; return;
                end
            catch, end
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
        msg = strrep(msg, newline, ' | '); 
        fprintf(fid, '[%s] %s\n', datestr(now, 'HH:MM:SS'), msg);
        fclose(fid);
    end
end

function workerSetupFcn(Kp, Ki, Kpv, Kiv, spec)
    assignin('base', 'C_HV', 1000e-6); 
    assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01); 
    assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp); 
    assignin('base', 'Ki_ext', Ki);
    assignin('base', 'Kpv', Kpv); 
    assignin('base', 'Kiv', Kiv);
    
    assignin('base', 'spec', spec);
    assignin('base', 'fs', spec.fs);
    assignin('base', 'n', spec.n);
    assignin('base', 'V_in', spec.V_in);
end

% [수정] simOut을 변형하지 않고, 분석된 결과(struct)만 반환하도록 재작성
function res = analyze_sim_data(simOut, T_STARTUP, T_STEP, N_D0_STEPS, T_WINDOW)
    res = struct();
    res.error_msg = '';
    res.p_act = NaN(N_D0_STEPS, 1);
    res.p_act_hv = NaN(N_D0_STEPS, 1);
    res.i_pk = NaN(N_D0_STEPS, 1);
    res.valid = false(N_D0_STEPS, 1);
    
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
            res.error_msg = 'Required signals not found. (Check To Workspace or Data Logging blocks)';
        else
            for d_idx = 1:N_D0_STEPS
                if d_idx == 1
                    t_end_step = T_STARTUP;
                else
                    t_end_step = T_STARTUP + (d_idx - 1) * T_STEP;
                end
                
                % 위에서 설정한 T_WINDOW 변수를 사용하여 구간 계산
                t_start_step = t_end_step - T_WINDOW; 
                
                % [매우 중요] 다음 스텝으로 넘어가는 순간의 Transient Spike가 
                % 포함되지 않도록 등호(=)를 빼고 마진(-1e-6)을 주어 오버랩을 완벽히 차단합니다.
                mask_i = (t_sim >= t_start_step) & (t_sim < t_end_step - 1e-6);
                mask_p_lv = (t_plv >= t_start_step) & (t_plv < t_end_step - 1e-6);
                mask_p_hv = (t_phv >= t_start_step) & (t_phv < t_end_step - 1e-6);
                
                % 단, 데이터가 1개라도 있으면 인정하도록 하여 에러 방지
                if sum(mask_i) > 0 && sum(mask_p_lv) > 0 && sum(mask_p_hv) > 0
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
end