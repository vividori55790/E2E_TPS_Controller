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
    fprintf('  Target: V2, D0, D1, D2 -> i_pk_pri, Power_LV\n');
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
    SIM_TIME = 0.025;           % 25ms
    T_STARTUP = 0.005;          % 5ms 초기 안정화 시간
    T_STEP    = 0.001;          % 매 스텝간 1ms (총 20 step)
    N_D0_STEPS = 20;            % 20계단
    
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    %% 2. 파라미터 그리드 생성 (V2, D1, D2 중심)
    N_V2 = 20; N_D1 = 20; N_D2 = 20;
    
    V2_vec = linspace(84, 109.2, N_V2);
    D1_vec = linspace(0.2, 0.5, N_D1);
    D2_vec = linspace(0.2, 0.5, N_D2);
    
    % 전력 변화가 0도 근처에서 극심하고 90도에서 둔감한 DAB의 특성( P ∝ D0*(1-D0) )을 반영
    % 전력 결과값이 선형적인 간격으로 찍히도록 위상 배분을 1 - sqrt(1 - x) 형태로 비선형 인가
    norm_x = linspace(0, 1, N_D0_STEPS);
    D0_vec = 90 * (1 - sqrt(1 - norm_x));
    
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
    dataset_P_act = NaN(N_total, 1);
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
                dataset_I_pk = cp.dataset_I_pk; dataset_P_act = cp.dataset_P_act;
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
                
                if ~isempty(simOut.ErrorMessage)
                    error_msg{sim_idx} = simOut.ErrorMessage;
                    completed_sims(sim_idx) = true;
                    continue;
                end
                
                % 신호 추출
                try
                    [I_L_data, t_sim] = extract_signal(simOut, {'Inductor_Current1','Inductor_Current','I_L1','I_L'});
                    [P_lv_data, t_plv] = extract_signal(simOut, {'Power_LV'});
                    
                    % Power_LV To Workspace 블록이 없을 시 차선책 (V * I)
                    if isempty(P_lv_data)
                        [v_lv,  t_v] = extract_signal(simOut, {'V_LV1', 'V_LV'});
                        [i_lv,  ~]   = extract_signal(simOut, {'I_LV1', 'I_LV'});
                        if ~isempty(v_lv) && ~isempty(i_lv)
                            P_lv_data = v_lv .* i_lv;
                            t_plv = t_v; 
                        end
                    end
                    
                    if isempty(I_L_data) || isempty(P_lv_data)
                        error_msg{sim_idx} = 'Required signals not found.';
                        completed_sims(sim_idx) = true; continue;
                    end
                    
                    % ========== 다중 결과 추출 (20스텝 커팅) ==========
                    for d_idx = 1:N_D0_STEPS
                        row_idx = (sim_idx - 1) * N_D0_STEPS + d_idx;
                        
                        % 각 D0 위상별 끝나는 시점 계산
                        if d_idx == 1
                            t_end_step = T_STARTUP; % 0.006
                        else
                            t_end_step = T_STARTUP + (d_idx - 1) * T_STEP; % 0.007, 0.008 ...
                        end
                        
                        % 안정화(정상상태)를 가정하고, 스텝 종료 직전 0.5ms (절반) 구간의 데이터만 추출
                        t_start_step = t_end_step - 0.0005; 
                        
                        % [중요] 위상이 변하는 시점(edge)이 포함되지 않도록, t_end_step보다 아주 살짝 앞까지만 자름
                        mask = (t_sim >= t_start_step) & (t_sim < t_end_step - 1e-6);
                        if sum(mask) > 5 % 데이터가 충분히 확보된 구간 한정
                            try
                                i_pk = max(abs(I_L_data(mask)));
                                
                                mask_p = (t_plv >= t_start_step) & (t_plv <= t_end_step);
                                p_act = mean(P_lv_data(mask_p));
                                
                                dataset_V2(row_idx) = V2_all(sim_idx);
                                dataset_D1(row_idx) = D1_all(sim_idx);
                                dataset_D2(row_idx) = D2_all(sim_idx);
                                dataset_D0(row_idx) = D0_vec(d_idx);
                                dataset_I_pk(row_idx) = i_pk;
                                dataset_P_act(row_idx) = p_act;
                                dataset_valid(row_idx) = true;
                                
                                % 명령창 및 로그 파일에 성공 상태 출력
                                logStatusMsg(sprintf('   [Sim %d / %d] SUCCESS. V2=%.1fV, D1=%.2f, D2=%.2f, D0=%.1fdeg -> i_pk_pri=%.2fA, P_act=%.1fW', ...
                                    sim_idx, N_sims, V2_all(sim_idx), D1_all(sim_idx), D2_all(sim_idx), D0_vec(d_idx), i_pk, p_act), logFile);
                            catch
                            end
                        end
                    end
                    completed_sims(sim_idx) = true;
                    
                catch extME
                    error_msg{sim_idx} = extME.message;
                    completed_sims(sim_idx) = true;
                end
            end
        end
        
        % 체크포인트 단기 저장 (디스크 오버로딩 방지)
        clear simIn_array simOut_array;
        N_sims_cp = N_sims;
        save('TPS_temp.mat', 'dataset_V2','dataset_D1','dataset_D2','dataset_D0','dataset_I_pk','dataset_P_act','dataset_valid','completed_sims','error_msg','N_sims_cp', '-v7.3');
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
    
    dataset = table(dataset_P_act(valid_mask), dataset_V2(valid_mask), ...
                    dataset_D0(valid_mask), dataset_D1(valid_mask), dataset_D2(valid_mask), ...
                    dataset_I_pk(valid_mask), dataset_I_pk(valid_mask)*spec.n, ...
                    'VariableNames', {'P_act_W', 'V2_V', 'D0_deg', 'D1', 'D2', 'i_pk_pri_A', 'i_pk_sec_A'});
    
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
