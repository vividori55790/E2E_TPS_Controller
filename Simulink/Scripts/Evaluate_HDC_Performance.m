%% Evaluate_HDC_Performance.m
% =========================================================================
% HDC Model vs SPS Performance Evaluation Dataset Generator
% (Random Sampling on V_LV & P_ref Grid, Steady-State Evaluation)
% =========================================================================

function Evaluate_HDC_Performance()
    %% 0. 환경 설정
    clear; clc; close all;
    
    fprintf('=========================================================================\n');
    fprintf('  HDC Model Performance Evaluator (Random Grid Sampling)\n');
    fprintf('  Target: Random (V_LV, P_ref) -> Efficiency, Inductor Peak Current\n');
    fprintf('=========================================================================\n\n');
    
    scriptDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    dataDir = fullfile(projectDir, 'Data');
    if ~exist(dataDir, 'dir'), mkdir(dataDir); end
    
    checkpointFile = fullfile(dataDir, 'HDC_Eval_Checkpoint.mat');
    BATCH_SIZE = 3;  % [수정됨] 워커 수를 3으로 제한함에 따라 배치 사이즈도 3으로 조정
    
    timestamp = datestr(now, 'yyyymmdd_HHMMss');
    resultFile = fullfile(dataDir, sprintf('HDC_Eval_Result_%s.mat', timestamp));
    csvFile    = fullfile(dataDir, sprintf('HDC_Eval_Result_%s.csv', timestamp));
    
    %% 1. 시스템 사양 및 시뮬레이션 설정
    spec = struct();
    spec.V_in = 380;       % HV측 고정 전압 [V]
    spec.n = 4;
    spec.fs = 80e3;
    
    SIM_TIME = 0.15;       % 폐루프 안정화를 위한 충분한 시뮬레이션 시간
    T_WINDOW = 0.02;       % 정상상태 분석 구간 (마지막 20ms 데이터 추출)
    
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    %% 2. 파라미터 그리드 생성 및 랜덤 샘플링
    % V_LV 및 P_ref 범위 설정
    V_LV_vec = linspace(84, 110, 15);      % 배터리 전압 범위 (15 포인트)
    P_ref_vec = linspace(-1000, 1000, 21);  % 충방전 전력 지령치 범위 (21 포인트)
    
    [V_grid, P_grid] = ndgrid(V_LV_vec, P_ref_vec);
    V_all_points = V_grid(:);
    P_all_points = P_grid(:);
    Total_Grid_Points = length(V_all_points);
    
    % 랜덤 추출할 샘플 개수 정의
    N_SAMPLES = 100; % 평가를 위해 무작위로 추출할 개수 (필요시 수정)
    
    rng(42); % 재현성을 위한 시드 고정
    sample_indices = randsample(Total_Grid_Points, N_SAMPLES);
    
    V_eval = V_all_points(sample_indices);
    P_eval = P_all_points(sample_indices);
    N_sims = length(V_eval);
    
    fprintf('>> Parameter Grid & Sampling:\n');
    fprintf('   V_LV range: %.1fV ~ %.1fV\n', min(V_LV_vec), max(V_LV_vec));
    fprintf('   P_ref range: %.1fW ~ %.1fW\n', min(P_ref_vec), max(P_ref_vec));
    fprintf('   Randomly selected %d points out of %d total grid points.\n\n', N_sims, Total_Grid_Points);
    
    %% 3. 체크포인트 및 변수 할당
    dataset_V_LV = NaN(N_sims, 1);
    dataset_P_ref = NaN(N_sims, 1);
    dataset_Eff = NaN(N_sims, 1);
    dataset_I_peak = NaN(N_sims, 1);
    dataset_P_LV_act = NaN(N_sims, 1);
    dataset_P_HV_act = NaN(N_sims, 1);
    dataset_valid = false(N_sims, 1);
    completed_sims = false(N_sims, 1);
    error_msg = cell(N_sims, 1);
    
    if exist(checkpointFile, 'file')
        fprintf('>> [RESUME] Checkpoint found. Resuming...\n');
        cp = load(checkpointFile);
        dataset_V_LV = cp.dataset_V_LV; dataset_P_ref = cp.dataset_P_ref;
        dataset_Eff = cp.dataset_Eff; dataset_I_peak = cp.dataset_I_peak;
        dataset_P_LV_act = cp.dataset_P_LV_act; dataset_P_HV_act = cp.dataset_P_HV_act;
        dataset_valid = cp.dataset_valid; completed_sims = cp.completed_sims;
        error_msg = cp.error_msg;
    end
    
    %% 4. 시뮬레이션 풀 및 베이스 입력 셋업
    addpath(projectDir); addpath(fullfile(projectDir, 'Modules'));
    modelName = 'Control_Logic';
    modelDir = fullfile(projectDir, 'Modules');
    originalDir = pwd;
    
    % [수정됨] 워커 수를 3개로 제한하는 로직 추가
    poolobj = gcp('nocreate');
    if isempty(poolobj)
        parpool(3); 
    elseif poolobj.NumWorkers ~= 3
        delete(poolobj);
        parpool(3);
    end
    
    t_input = [0; SIM_TIME];
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    
    % 제어기 입력 공통 변수 (Timeseries 형식)
    % Ctrl_Mode: 3 (HDC Mode), Mode_Select: 0 (Closed-loop)
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2,1), t_input));
    baseIn = baseIn.setVariable('Ctrl_Mode', timeseries(3 * ones(2,1), t_input)); 
    baseIn = baseIn.setVariable('Mode_Select', timeseries(0 * ones(2,1), t_input)); 
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2,1), t_input));
    
    % 디버깅 매뉴얼 변수 (Mode_Select=0 이므로 로직에 영향 없음)
    baseIn = baseIn.setVariable('Phi_ext', timeseries(zeros(2,1), t_input));  
    baseIn = baseIn.setVariable('D1_ext', timeseries(0.5 * ones(2,1), t_input));
    baseIn = baseIn.setVariable('D2_ext', timeseries(0.5 * ones(2,1), t_input));
    
    %% 5. 시뮬레이션 실행 (배치)
    remaining_indices = find(~completed_sims);
    N_remaining = length(remaining_indices);
    fprintf('>> Running simulations (%d remaining) via parsim...\n', N_remaining);
    
    cd(modelDir);
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
            
            % 각 샘플별 동작점(V_batt, Ref_Power) 매핑
            simIn_array(k) = simIn_array(k).setVariable('V_batt_in', timeseries(V_eval(idx)*ones(2,1), t_input));
            simIn_array(k) = simIn_array(k).setVariable('Ref_Power', timeseries(P_eval(idx)*ones(2,1), t_input));
        end
        
        parsim_success = false;
        try
            simOut_array = parsim(simIn_array, 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                 'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', ...
                 'SetupFcn', @()workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val, spec));
            parsim_success = true;
        catch ME
            fprintf('[BATCH %d] Parsim crash: %s\n', batchIdx, ME.message);
            for k = 1:n_batch, completed_sims(batch_indices(k)) = true; end
        end
        
        if parsim_success
            for k = 1:n_batch
                sim_idx = batch_indices(k);
                simOut = simOut_array(k);
                
                if ~isempty(simOut.ErrorMessage)
                    error_msg{sim_idx} = simOut.ErrorMessage;
                    completed_sims(sim_idx) = true;
                    fprintf('   [Sim %d] FAILED. Err: %s\n', sim_idx, simOut.ErrorMessage);
                    continue;
                end
                
                % [수정됨] P_ref 값도 인자로 넘겨 전력 추종 정상 도달 여부를 검증
                res = analyze_steady_state(simOut, SIM_TIME, T_WINDOW, P_eval(sim_idx));
                
                if ~isempty(res.error_msg)
                    error_msg{sim_idx} = res.error_msg;
                    % 전력 도달 실패 및 기타 에러 시 기록 남김
                    fprintf('   [Sim %d] FAILED/INVALID. Err: %s\n', sim_idx, res.error_msg);
                else
                    dataset_V_LV(sim_idx) = V_eval(sim_idx);
                    dataset_P_ref(sim_idx) = P_eval(sim_idx);
                    dataset_Eff(sim_idx) = res.efficiency;
                    dataset_I_peak(sim_idx) = res.i_pk;
                    dataset_P_LV_act(sim_idx) = res.p_lv;
                    dataset_P_HV_act(sim_idx) = res.p_hv;
                    dataset_valid(sim_idx) = true;
                    
                    fprintf('   [Sim %d] V=%.1fV, Pref=%.1fW | P_act=%.1fW, Eff=%.2f%%, I_pk=%.2fA\n', ...
                        sim_idx, V_eval(sim_idx), P_eval(sim_idx), res.p_lv, res.efficiency*100, res.i_pk);
                end
                completed_sims(sim_idx) = true;
            end
        end
        
        clear simIn_array simOut_array simOut;
        save('HDC_temp.mat', 'dataset_V_LV','dataset_P_ref','dataset_Eff','dataset_I_peak',...
             'dataset_P_LV_act','dataset_P_HV_act','dataset_valid','completed_sims','error_msg', '-v7.3');
        movefile('HDC_temp.mat', checkpointFile, 'f');
    end
    
    cd(originalDir);
    
    %% 6. 결과 저장
    valid_mask = dataset_valid;
    n_valid = sum(valid_mask);
    
    dataset = table(dataset_V_LV(valid_mask), dataset_P_ref(valid_mask), ...
                    dataset_P_LV_act(valid_mask), dataset_P_HV_act(valid_mask), ...
                    dataset_Eff(valid_mask) * 100, dataset_I_peak(valid_mask), ...
                    'VariableNames', {'V_LV_V', 'P_Ref_W', 'P_LV_act_W', 'P_HV_act_W', 'Efficiency_Percent', 'I_L_peak_A'});
    
    save(resultFile, 'dataset', 'error_msg', '-v7.3');
    writetable(dataset, csvFile);
    
    fprintf('>> Evaluation Done! %d valid points saved to:\n   %s\n', n_valid, resultFile);
    delete(checkpointFile);
end

%% Helpers
function workerSetupFcn(Kp, Ki, Kpv, Kiv, spec)
    % 모델이 필요로 하는 파라미터 변수들만 명확하게 할당
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
        end
    end
end

% 정상 상태 윈도우 추출 및 효율, 피크 전류 계산 (+ 전력 도달 검증)
function res = analyze_steady_state(simOut, SIM_TIME, T_WINDOW, P_ref)
    res = struct();
    res.error_msg = '';
    res.efficiency = NaN; res.i_pk = NaN;
    res.p_lv = NaN; res.p_hv = NaN;
    
    try
        % To Workspace 블록의 변수명으로 데이터 추출
        [I_L_data, t_sim] = extract_signal(simOut, {'out_I_L'});
        [P_lv_data, t_plv] = extract_signal(simOut, {'out_P_LV'});
        [P_hv_data, t_phv] = extract_signal(simOut, {'out_P_HV'});
        
        if isempty(I_L_data) || isempty(P_lv_data) || isempty(P_hv_data)
            res.error_msg = 'Required output signals (out_I_L, out_P_LV, out_P_HV) not found.';
            return;
        end
        
        % 정상 상태 시간 마스크 생성
        t_start = SIM_TIME - T_WINDOW;
        
        mask_i = (t_sim >= t_start);
        mask_p_lv = (t_plv >= t_start);
        mask_p_hv = (t_phv >= t_start);
        
        if sum(mask_i) == 0 || sum(mask_p_lv) == 0 || sum(mask_p_hv) == 0
            res.error_msg = 'No data found within the steady-state window.';
            return;
        end
        
        % 결과 산출
        I_steady = I_L_data(mask_i);
        res.i_pk = max(abs(I_steady));
        
        P_lv_steady = mean(P_lv_data(mask_p_lv));
        P_hv_steady = mean(P_hv_data(mask_p_hv));
        
        res.p_lv = P_lv_steady;
        res.p_hv = P_hv_steady;
        
        % =========================================================================
        % [추가됨] 전력 추종 성공 여부 검증 (오차 5% 또는 10W 허용 범위)
        % 충/방전 방향 및 효율에 따라 LV 혹은 HV측 전력 중 하나라도 제어 목표에 도달했는지 확인
        % =========================================================================
        power_tol = max(abs(P_ref) * 0.05, 10); % P_ref의 5% 혹은 최소 10W 오차 마진
        
        if (abs(P_lv_steady - P_ref) > power_tol) && (abs(P_hv_steady - P_ref) > power_tol)
            res.error_msg = sprintf('Power tracking failed: Target P_ref=%.1fW, Actual P_LV=%.1fW, P_HV=%.1fW', ...
                                     P_ref, P_lv_steady, P_hv_steady);
            return;
        end
        
        % 효율(Efficiency) 계산
        % 전력이 매우 작을 경우 오차 발생을 막기 위한 방어코드
        if abs(P_lv_steady) < 5 || abs(P_hv_steady) < 5
            res.efficiency = 0; % 전력 전달이 없으므로 효율 0 처리
        elseif P_lv_steady > 0 && P_hv_steady > 0
            % 방전 모드 (LV -> HV) : 입력 LV, 출력 HV
            res.efficiency = abs(P_hv_steady) / abs(P_lv_steady);
        elseif P_lv_steady < 0 && P_hv_steady < 0
            % 충전 모드 (HV -> LV) : 입력 HV, 출력 LV
            res.efficiency = abs(P_lv_steady) / abs(P_hv_steady);
        else
            res.efficiency = 0; % 부호가 엇갈리는 비정상 상태
        end
        
        % 100% (1.0)를 초과하는 물리적 오류 방어
        res.efficiency = min(1.0, res.efficiency);
        
    catch ME
        res.error_msg = ME.message;
    end
end