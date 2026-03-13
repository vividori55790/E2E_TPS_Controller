%% Regenerate_Bad_Batches.m
% =========================================================================
% 불량 배치(D0 변화에도 전류/전력이 동일) 재생성 스크립트
% 기존 CSV에서 나쁜 행을 제거한 뒤, 해당 (V2, D1, D2) 조합만 재시뮬레이션
% =========================================================================

function Regenerate_Bad_Batches()
    %% 0. 환경 설정
    clc; close all;
    
    fprintf('=========================================================================\n');
    fprintf('  DAB TPS Bad Batch Regenerator\n');
    fprintf('=========================================================================\n\n');
    
    scriptDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    dataDir = fullfile(projectDir, 'Data');
    
    % 원본 CSV 파일 경로
    originalCSV = fullfile(dataDir, 'TPS_Dataset_20260313_090038.csv');
    
    if ~exist(originalCSV, 'file')
        error('Original CSV not found: %s', originalCSV);
    end
    
    BATCH_SIZE = 12;  % parsim 배치 사이즈
    
    %% 1. 시스템 사양 (Generate_TPS_Dataset.m과 동일)
    spec = struct();
    spec.V_in      = 380;
    spec.n         = 4;
    spec.fs        = 80e3;
    spec.I_pri_max = 10;
    spec.I_sec_max = 40;
    
    SIM_TIME = 0.025;
    T_STARTUP = 0.006;
    T_STEP    = 0.001;
    N_D0_STEPS = 20;
    
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    %% 2. 파라미터 그리드 (원본과 동일)
    N_V2 = 20; N_D1 = 20; N_D2 = 20;
    V2_vec = linspace(84, 109.2, N_V2);
    D1_vec = linspace(0.2, 0.5, N_D1);
    D2_vec = linspace(0.2, 0.5, N_D2);
    
    norm_x = linspace(0, 1, N_D0_STEPS);
    D0_vec = 90 * (1 - sqrt(1 - norm_x));
    
    [V2_grid, D1_grid, D2_grid] = ndgrid(V2_vec, D1_vec, D2_vec);
    V2_all = V2_grid(:);
    D1_all = D1_grid(:);
    D2_all = D2_grid(:);
    N_sims = length(V2_all);
    
    %% 3. D0 Timeseries 생성 (원본과 동일)
    t_ts = [0];
    d_ts = [D0_vec(1)];
    for k = 1:(N_D0_STEPS-1)
        t_change = T_STARTUP + (k-1) * T_STEP;
        t_ts(end+1) = t_change;
        d_ts(end+1) = D0_vec(k+1);
    end
    ts_D0 = timeseries(d_ts', t_ts');
    ts_D0.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');
    
    %% 4. CSV 읽기 및 불량 배치 식별
    fprintf('>> Reading original CSV...\n');
    T = readtable(originalCSV);
    fprintf('   Total rows: %d\n', height(T));
    
    % 매 20행 단위로 배치를 확인
    num_batches = floor(height(T) / N_D0_STEPS);
    bad_batch_indices = [];  % 0-based batch index
    
    for b = 0:(num_batches-1)
        row_start = b * N_D0_STEPS + 1;
        row_end   = row_start + N_D0_STEPS - 1;
        
        batch_data = T(row_start:row_end, :);
        d0_vals = batch_data.D0_deg;
        p_vals  = batch_data.P_act_W;
        i_vals  = batch_data.i_pk_pri_A;
        
        d0_range = max(d0_vals) - min(d0_vals);
        p_range  = max(p_vals) - min(p_vals);
        i_range  = max(i_vals) - min(i_vals);
        
        if d0_range > 10 && p_range < 1.0 && i_range < 0.1
            bad_batch_indices(end+1) = b;  %#ok<AGROW>
        end
    end
    
    N_bad = length(bad_batch_indices);
    fprintf('>> Found %d bad batches\n', N_bad);
    
    if N_bad == 0
        fprintf('>> No bad batches found. Nothing to do.\n');
        return;
    end
    
    fprintf('   Batch range: %d to %d (0-based)\n', bad_batch_indices(1), bad_batch_indices(end));
    fprintf('   CSV row range: %d to %d\n', bad_batch_indices(1)*N_D0_STEPS+1, (bad_batch_indices(end)+1)*N_D0_STEPS);
    
    %% 5. 해당 배치의 (V2, D1, D2) 조합에 대응하는 sim_idx 찾기
    % CSV의 각 배치에서 V2, D1, D2 값을 추출하여 V2_all/D1_all/D2_all에서 매칭
    bad_sim_indices = zeros(N_bad, 1);
    for i = 1:N_bad
        b = bad_batch_indices(i);
        row_start = b * N_D0_STEPS + 1;
        v2_val = T.V2_V(row_start);
        d1_val = T.D1(row_start);
        d2_val = T.D2(row_start);
        
        % V2_all, D1_all, D2_all에서 매칭되는 인덱스 찾기
        match = find(abs(V2_all - v2_val) < 1e-6 & abs(D1_all - d1_val) < 1e-6 & abs(D2_all - d2_val) < 1e-6);
        if ~isempty(match)
            bad_sim_indices(i) = match(1);
        else
            warning('Could not match batch %d: V2=%.6f, D1=%.6f, D2=%.6f', b, v2_val, d1_val, d2_val);
            bad_sim_indices(i) = -1;
        end
    end
    
    % 매칭 안 된 것 제외
    valid_mask = bad_sim_indices > 0;
    bad_sim_indices = bad_sim_indices(valid_mask);
    bad_batch_indices_valid = bad_batch_indices(valid_mask);
    N_regen = length(bad_sim_indices);
    fprintf('>> Will regenerate %d simulations\n\n', N_regen);
    
    %% 6. CSV에서 나쁜 행 제거
    fprintf('>> Removing bad rows from CSV...\n');
    bad_rows = [];
    for i = 1:length(bad_batch_indices)
        b = bad_batch_indices(i);
        row_start = b * N_D0_STEPS + 1;
        row_end   = row_start + N_D0_STEPS - 1;
        bad_rows = [bad_rows, row_start:row_end]; %#ok<AGROW>
    end
    T_clean = T;
    T_clean(bad_rows, :) = [];
    fprintf('   Removed %d rows. Remaining: %d rows\n', length(bad_rows), height(T_clean));
    
    %% 7. 시뮬레이션 재실행
    fprintf('>> Setting up simulation...\n');
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
    
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('V_Grid', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_Power', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('Ref_I_ess_cmd', timeseries(zeros(2,1), t_input));
    baseIn = baseIn.setVariable('Mode_Select', timeseries(1 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2,1), t_input));
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
    
    %% 8. 재시뮬레이션 및 결과 수집
    regen_results = table('Size', [N_regen * N_D0_STEPS, 7], ...
        'VariableTypes', {'double','double','double','double','double','double','double'}, ...
        'VariableNames', {'P_act_W','V2_V','D0_deg','D1','D2','i_pk_pri_A','i_pk_sec_A'});
    
    regen_valid = false(N_regen * N_D0_STEPS, 1);
    
    cd(modelDir);
    
    N_batches = ceil(N_regen / BATCH_SIZE);
    fprintf('>> Running %d simulations in %d batches...\n', N_regen, N_batches);
    tic;
    
    for batchIdx = 1:N_batches
        batchStart = (batchIdx - 1) * BATCH_SIZE + 1;
        batchEnd   = min(batchIdx * BATCH_SIZE, N_regen);
        n_batch = batchEnd - batchStart + 1;
        
        simIn_array(1:n_batch) = Simulink.SimulationInput(modelName);
        for k = 1:n_batch
            regen_k = batchStart + k - 1;
            idx = bad_sim_indices(regen_k);
            simIn_array(k) = baseIn;
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
            fprintf('[BATCH %d] Parsim crash: %s\n', batchIdx, ME.message);
        end
        
        if parsim_success
            for k = 1:n_batch
                regen_k = batchStart + k - 1;
                idx = bad_sim_indices(regen_k);
                simOut = simOut_array(k);
                
                if ~isempty(simOut.ErrorMessage)
                    fprintf('   [Regen %d] Error: %s\n', regen_k, simOut.ErrorMessage);
                    continue;
                end
                
                try
                    [I_L_data, t_sim] = extract_signal(simOut, {'Inductor_Current1','Inductor_Current','I_L1','I_L'});
                    [P_lv_data, t_plv] = extract_signal(simOut, {'Power_LV'});
                    
                    if isempty(P_lv_data)
                        [v_lv,  t_v] = extract_signal(simOut, {'V_LV1', 'V_LV'});
                        [i_lv,  ~]   = extract_signal(simOut, {'I_LV1', 'I_LV'});
                        if ~isempty(v_lv) && ~isempty(i_lv)
                            P_lv_data = v_lv .* i_lv;
                            t_plv = t_v;
                        end
                    end
                    
                    if isempty(I_L_data) || isempty(P_lv_data)
                        fprintf('   [Regen %d] Required signals not found.\n', regen_k);
                        continue;
                    end
                    
                    for d_idx = 1:N_D0_STEPS
                        row_idx = (regen_k - 1) * N_D0_STEPS + d_idx;
                        
                        if d_idx == 1
                            t_end_step = T_STARTUP;
                        else
                            t_end_step = T_STARTUP + (d_idx - 1) * T_STEP;
                        end
                        t_start_step = t_end_step - 0.0005;
                        
                        mask = (t_sim >= t_start_step) & (t_sim < t_end_step - 1e-6);
                        if sum(mask) > 5
                            try
                                i_pk = max(abs(I_L_data(mask)));
                                mask_p = (t_plv >= t_start_step) & (t_plv <= t_end_step);
                                p_act = mean(P_lv_data(mask_p));
                                
                                regen_results.P_act_W(row_idx) = p_act;
                                regen_results.V2_V(row_idx) = V2_all(idx);
                                regen_results.D0_deg(row_idx) = D0_vec(d_idx);
                                regen_results.D1(row_idx) = D1_all(idx);
                                regen_results.D2(row_idx) = D2_all(idx);
                                regen_results.i_pk_pri_A(row_idx) = i_pk;
                                regen_results.i_pk_sec_A(row_idx) = i_pk * spec.n;
                                regen_valid(row_idx) = true;
                                
                                fprintf('   [Regen %d/%d] V2=%.1f, D0=%.1f -> i_pk=%.2f, P=%.1f\n', ...
                                    regen_k, N_regen, V2_all(idx), D0_vec(d_idx), i_pk, p_act);
                            catch
                            end
                        end
                    end
                    
                catch extME
                    fprintf('   [Regen %d] Error: %s\n', regen_k, extME.message);
                end
            end
        end
        
        clear simIn_array simOut_array;
        fprintf('   [Batch %d/%d] Done.\n', batchIdx, N_batches);
        
        % 메모리 누수 방지
        if mod(batchIdx, 50) == 0 && batchIdx < N_batches
            fprintf('   [Memory Flush] Restarting parallel pool...\n');
            delete(gcp('nocreate'));
            parpool;
        end
    end
    
    elapsed = toc;
    cd(originalDir);
    
    %% 9. 결과 합치기
    regen_valid_results = regen_results(regen_valid, :);
    fprintf('\n>> Regenerated %d valid rows (out of %d expected)\n', height(regen_valid_results), N_regen * N_D0_STEPS);
    
    % 정리된 원본 + 재생성 결과 합치기
    T_final = [T_clean; regen_valid_results];
    
    % V2, D1, D2, D0 기준으로 정렬 (원본 순서 복원)
    T_final = sortrows(T_final, {'D1', 'D2', 'V2_V', 'D0_deg'});
    
    fprintf('>> Final dataset: %d rows\n', height(T_final));
    
    %% 10. 저장
    % 원본 백업
    backupCSV = fullfile(dataDir, 'TPS_Dataset_20260313_090038_backup.csv');
    if ~exist(backupCSV, 'file')
        copyfile(originalCSV, backupCSV);
        fprintf('>> Original CSV backed up to: %s\n', backupCSV);
    end
    
    % 덮어쓰기
    writetable(T_final, originalCSV);
    fprintf('>> Updated CSV saved: %s\n', originalCSV);
    fprintf('>> Elapsed time: %.1f seconds (%.1f minutes)\n', elapsed, elapsed/60);
    fprintf('=========================================================================\n');
    fprintf('  DONE!\n');
    fprintf('=========================================================================\n');
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

function workerSetupFcn(Kp, Ki, Kpv, Kiv)
    assignin('base', 'C_HV', 1000e-6); assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01); assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp); assignin('base', 'Ki_ext', Ki);
    assignin('base', 'Kpv', Kpv); assignin('base', 'Kiv', Kiv);
end
