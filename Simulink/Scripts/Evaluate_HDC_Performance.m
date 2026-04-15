% =========================================================================
% HDC Model vs SPS Performance Evaluation Dataset Generator
% (Staircase Power Sweep on V_LV Grid, Multi-Point Analysis per Sim)
% =========================================================================

function Evaluate_HDC_Performance()
    %% 0. 환경 설정
    clear; clc; close all;
    
    fprintf('=========================================================================\n');
    fprintf('  HDC Model Performance Evaluator (Staircase Sweep Mode)\n');
    fprintf('  Target: V_LV Steps -> [1000->100W Sweep] & [-1000->-100W Sweep]\n');
    fprintf('  Worker Count: 4 | Simulation Time: 0.31s per Sweep\n');
    fprintf('=========================================================================\n\n');
    
    % 경로 설정
    scriptDir = fileparts(mfilename('fullpath'));
    projectDir = fileparts(scriptDir);
    dataDir = fullfile(projectDir, 'Data');
    if ~exist(dataDir, 'dir'), mkdir(dataDir); end
    
    % 설정 및 결과 파일명
    checkpointFile = fullfile(dataDir, 'HDC_Eval_Checkpoint.mat');
    BATCH_SIZE = 4;  % 워커 수에 맞춰 4로 설정
    
    timestamp = datestr(now, 'yyyymmdd_HHMMss');
    resultFile = fullfile(dataDir, sprintf('HDC_Eval_Result_%s.mat', timestamp));
    csvFile    = fullfile(dataDir, sprintf('HDC_Eval_Result_%s.csv', timestamp));
    
    %% 1. 시스템 및 스윕 파라미터 설정
    spec = struct();
    spec.V_in = 380; spec.n = 4; spec.fs = 80e3;
    
    % [중요 유지] 시뮬레이션 시간 설계
    % 한 번의 시뮬레이션(0.31초) 동안 10개의 전력 지령을 계단식으로 모두 처리합니다.
    % 이렇게 묶어서 처리하면 매 지령마다 시뮬레이션을 껐다 켜는 오버헤드가 없어져 속도가 비약적으로 상승합니다.
    SIM_TIME = 0.31;
    T_STARTUP = 0.04; 
    T_STEP = 0.03;
    
    % [수정됨] 분석 윈도우 구간 설정 (직전 1ms 배제)
    T_WIN_START = 0.006; % 분석 시작점 (해당 스텝 종료 6ms 전)
    T_WIN_END   = 0.001; % 분석 종료점 (해당 스텝 종료 1ms 전)
    
    P_step_val = 100;
    P_pos_vec = 1000:-P_step_val:100; % 10 points (양의 전력 스윕, 0W 제외)
    P_neg_vec = -1000:P_step_val:-100; % 10 points (음의 전력 스윕, 0W 제외)
    N_steps_per_sim = length(P_pos_vec);
    
    V_LV_vec = linspace(84, 110, 14); 
    
    % 시나리오 구성 (V_LV당 2회 시뮬레이션: Positive Sweep, Negative Sweep)
    eval_cases = [];
    for v = V_LV_vec
        eval_cases = [eval_cases; struct('V', v, 'P_vec', P_pos_vec, 'Type', 'Positive')];
        eval_cases = [eval_cases; struct('V', v, 'P_vec', P_neg_vec, 'Type', 'Negative')];
    end
    
    N_sims = length(eval_cases);
    N_total_points = N_sims * N_steps_per_sim;
    
    fprintf('>> 시뮬레이션 계획:\n');
    fprintf('   전압 구간 수: %d, 시뮬레이션 총 횟수: %d\n', length(V_LV_vec), N_sims);
    fprintf('   추출 예정 데이터 포인트: %d\n\n', N_total_points);
    
    %% 2. 데이터 저장 변수 및 체크포인트 복구
    dataset_V_LV = NaN(N_total_points, 1);
    dataset_P_ref = NaN(N_total_points, 1);
    dataset_Eff = NaN(N_total_points, 1);
    dataset_I_peak = NaN(N_total_points, 1);
    dataset_P_LV_act = NaN(N_total_points, 1);
    dataset_P_HV_act = NaN(N_total_points, 1);
    dataset_valid = false(N_total_points, 1);
    
    completed_sims = false(N_sims, 1);
    error_msg = cell(N_sims, 1);
    
    % [필수 유지] 체크포인트(이어하기) 로직
    if exist(checkpointFile, 'file')
        fprintf('>> [RESUME] 체크포인트 복구 중...\n');
        try
            cp = load(checkpointFile);
            if length(cp.completed_sims) == N_sims
                dataset_V_LV = cp.dataset_V_LV; dataset_P_ref = cp.dataset_P_ref;
                dataset_Eff = cp.dataset_Eff; dataset_I_peak = cp.dataset_I_peak;
                dataset_P_LV_act = cp.dataset_P_LV_act; dataset_P_HV_act = cp.dataset_P_HV_act;
                dataset_valid = cp.dataset_valid; completed_sims = cp.completed_sims;
                
                if isfield(cp, 'error_msg')
                    error_msg = cp.error_msg;
                end
                
                n_done = sum(completed_sims);
                fprintf('   복구 완료: %d / %d 시뮬레이션부터 재개합니다.\n\n', n_done, N_sims);
            else
                fprintf('   시뮬레이션 개수(Grid size) 불일치! 처음부터 다시 시작합니다.\n\n');
                completed_sims = false(N_sims, 1);
            end
        catch
            fprintf('   체크포인트 파일 로드 실패. 처음부터 다시 시작합니다.\n\n');
            completed_sims = false(N_sims, 1);
        end
    end
    
    %% 3. 병렬 풀 설정 (4개 워커)
    addpath(fullfile(projectDir, 'Modules'));
    modelName = 'Control_Logic';
    modelDir = fullfile(projectDir, 'Modules');
    originalDir = pwd;
    
    poolobj = gcp('nocreate');
    if isempty(poolobj) || poolobj.NumWorkers ~= 4
        if ~isempty(poolobj), delete(poolobj); end
        parpool(4);
    end
    
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    
    %% 4. 시뮬레이션 실행 (배치 단위)
    remaining_indices = find(~completed_sims);
    N_remaining = length(remaining_indices);
    cd(modelDir);
    
    N_batches = ceil(N_remaining / BATCH_SIZE);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    
    for batchIdx = 1:N_batches
        batch_indices = remaining_indices(((batchIdx-1)*BATCH_SIZE+1):min(batchIdx*BATCH_SIZE, N_remaining));
        n_batch = length(batch_indices);
        
        simIn_array = repmat(Simulink.SimulationInput(modelName), 1, n_batch);
        
        for k = 1:n_batch
            idx = batch_indices(k);
            case_info = eval_cases(idx);
            
            % [수정됨] 계단형(Staircase) 전력 지령 타임시리즈 생성부
            % 기존의 1스텝 지연(Delay) 버그를 수정하여 지령 인가 시간과 분석 시간이 정확히 일치하도록 동기화
            t_pts = zeros(N_steps_per_sim, 1);
            p_pts = zeros(N_steps_per_sim, 1);
            
            t_pts(1) = 0;
            p_pts(1) = case_info.P_vec(1);
            
            for s = 2:N_steps_per_sim
                t_pts(s) = T_STARTUP + (s-2)*T_STEP;
                p_pts(s) = case_info.P_vec(s);
            end
            
            % 마지막 시간 명시적 추가
            if t_pts(end) < SIM_TIME
                t_pts(end+1) = SIM_TIME; 
                p_pts(end+1) = p_pts(end);
            end
            
            ts_Power = timeseries(p_pts, t_pts);
            ts_Power.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');
            
            simIn = baseIn;
            t_fixed = [0; SIM_TIME];
            simIn = simIn.setVariable('V_batt_in', timeseries(case_info.V*ones(2,1), t_fixed));
            simIn = simIn.setVariable('Ref_Power', ts_Power);
            simIn = simIn.setVariable('Ref_VH', timeseries(spec.V_in*ones(2,1), t_fixed));
            simIn = simIn.setVariable('Ctrl_Mode', timeseries(3*ones(2,1), t_fixed)); 
            simIn = simIn.setVariable('Mode_Select', timeseries(0*ones(2,1), t_fixed)); 
            simIn = simIn.setVariable('PWM_Enable', timeseries(1*ones(2,1), t_fixed));
            simIn = simIn.setVariable('Phi_ext', timeseries(zeros(2,1), t_fixed));  
            simIn = simIn.setVariable('D1_ext', timeseries(0.5*ones(2,1), t_fixed));
            simIn = simIn.setVariable('D2_ext', timeseries(0.5*ones(2,1), t_fixed));
            
            simIn_array(k) = simIn;
        end
        
        try
            simOut_array = parsim(simIn_array, 'ShowProgress', 'on', ...
                 'SetupFcn', @()workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val, spec));
            
            for k = 1:n_batch
                sim_idx = batch_indices(k);
                simOut = simOut_array(k);
                case_info = eval_cases(sim_idx);
                
                if ~isempty(simOut.ErrorMessage)
                    error_msg{sim_idx} = simOut.ErrorMessage;
                    completed_sims(sim_idx) = true; continue;
                end
                
                % 신호 추출 및 계단별 데이터 분석 (분석 구간 파라미터 전달)
                res = analyze_staircase_sweep_enhanced(simOut, T_STARTUP, T_STEP, N_steps_per_sim, T_WIN_START, T_WIN_END, case_info.P_vec);
                
                for s = 1:N_steps_per_sim
                    row_idx = (sim_idx-1)*N_steps_per_sim + s;
                    if res.valid(s)
                        dataset_V_LV(row_idx) = case_info.V;
                        dataset_P_ref(row_idx) = case_info.P_vec(s);
                        dataset_Eff(row_idx) = res.efficiency(s);
                        dataset_I_peak(row_idx) = res.i_pk(s);
                        dataset_P_LV_act(row_idx) = res.p_lv(s);
                        dataset_P_HV_act(row_idx) = res.p_hv(s);
                        dataset_valid(row_idx) = true;
                    end
                end
                
                if all(~res.valid)
                    fprintf('   [Sim %d] 추출 실패: 신호를 찾을 수 없거나 조건을 만족하는 유효 데이터가 없습니다.\n', sim_idx);
                else
                    fprintf('   [Sim %d/%d] V=%.1fV, Type=%s | %d/%d 포인트 추출 성공\n', ...
                        sim_idx, N_sims, case_info.V, case_info.Type, sum(res.valid), N_steps_per_sim);
                end
                completed_sims(sim_idx) = true;
            end
        catch ME
            fprintf('[Batch %d] 오류: %s\n', batchIdx, ME.message);
        end
        
        save(checkpointFile, 'dataset_V_LV','dataset_P_ref','dataset_Eff','dataset_I_peak',...
             'dataset_P_LV_act','dataset_P_HV_act','dataset_valid','completed_sims','error_msg');
    end
    
    cd(originalDir);
    
    %% 5. 최종 결과 정리 및 저장
    valid_mask = dataset_valid;
    dataset = table(dataset_V_LV(valid_mask), dataset_P_ref(valid_mask), ...
                    dataset_P_LV_act(valid_mask), dataset_P_HV_act(valid_mask), ...
                    dataset_Eff(valid_mask) * 100, dataset_I_peak(valid_mask), ...
                    'VariableNames', {'V_LV_V', 'P_Ref_W', 'P_LV_act_W', 'P_HV_act_W', 'Efficiency_Percent', 'I_L_peak_A'});
    
    % [추가됨] 시뮬레이션 종료 시 명령창에 추출된 핵심 데이터(전력, 피크전류 등) 미리보기 출력
    fprintf('\n>> [결과 미리보기] 추출된 핵심 데이터 (상위 5개 행):\n');
    disp(head(dataset, 5));
    
    save(resultFile, 'dataset', '-v7.3');
    writetable(dataset, csvFile);
    fprintf('\n>> 완료! 추출된 총 유효 데이터 포인트: %d\n', sum(dataset_valid));
    if exist(checkpointFile, 'file'), delete(checkpointFile); end
end

%% --- Helper Functions ---

function workerSetupFcn(Kp, Ki, Kpv, Kiv, spec)
    assignin('base', 'C_HV', 1000e-6); assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01); assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp); assignin('base', 'Ki_ext', Ki);
    assignin('base', 'Kpv', Kpv); assignin('base', 'Kiv', Kiv);
    assignin('base', 'spec', spec); assignin('base', 'fs', spec.fs);
    assignin('base', 'n', spec.n); assignin('base', 'V_in', spec.V_in);
end

% 다중 레이어 신호 추출기
function [data, t] = extract_signal_enhanced(simOut, names)
    data = []; t = [];
    if ischar(names), names = {names}; end
    
    % 1. 직접 속성(To Workspace) 또는 필드 확인
    for k = 1:length(names)
        if isprop(simOut, names{k}) || isfield(simOut, names{k})
            raw = simOut.(names{k});
            if isa(raw, 'timeseries'), data = squeeze(raw.Data); t = raw.Time; return; end
            if isa(raw, 'Simulink.SimulationData.Signal'), data = squeeze(raw.Values.Data); t = raw.Values.Time; return; end
        end
    end
    
    % 2. Dataset(yout, logsout) 검색
    containers = {'yout', 'logsout'};
    for c = 1:length(containers)
        if isprop(simOut, containers{c}) && ~isempty(simOut.(containers{c}))
            ds = simOut.(containers{c});
            available_names = ds.getElementNames();
            for k = 1:length(names)
                match_idx = find(strcmpi(available_names, names{k}), 1);
                if ~isempty(match_idx)
                    element = ds.get(available_names{match_idx});
                    if ~isempty(element)
                        if isa(element.Values, 'timeseries')
                            data = squeeze(element.Values.Data); t = element.Values.Time; return;
                        elseif isprop(element, 'Values') && isa(element.Values, 'Simulink.SimulationData.Signal')
                            data = squeeze(element.Values.Data); t = element.Values.Time; return;
                        end
                    end
                end
            end
        end
    end
    
    % 디버깅 지원 기능
    persistent debug_shown;
    if isempty(debug_shown)
        fprintf('\n[Debug] 일부 신호 추출 실패. 시뮬레이션 출력 구조:\n');
        for c = 1:length(containers)
            if isprop(simOut, containers{c}) && ~isempty(simOut.(containers{c}))
                fprintf('  - %s 안의 신호: %s\n', containers{c}, strjoin(simOut.(containers{c}).getElementNames(), ', '));
            end
        end
        debug_shown = true;
    end
end

function res = analyze_staircase_sweep_enhanced(simOut, T_START, T_STEP, N_STEPS, T_WIN_START, T_WIN_END, P_REFS)
    res = struct('valid', false(N_STEPS, 1), 'efficiency', zeros(N_STEPS, 1), ...
                 'i_pk', zeros(N_STEPS, 1), 'p_lv', zeros(N_STEPS, 1), 'p_hv', zeros(N_STEPS, 1));
    
    % 사용자 지정 이름(I_L, P_LV, P_HV)을 최우선으로 검색
    [I_L, t_i] = extract_signal_enhanced(simOut, {'I_L', 'out_I_L', 'Inductor_Current'});
    [P_lv, t_lv] = extract_signal_enhanced(simOut, {'P_LV', 'out_P_LV', 'Power_LV'});
    [P_hv, t_hv] = extract_signal_enhanced(simOut, {'P_HV', 'out_P_HV', 'Power_HV'});
    
    % 신호 누락 시 전압*전류로 폴백(Fallback) 방어 코드 강화
    if isempty(P_lv)
        [v_lv, t_v] = extract_signal_enhanced(simOut, {'V_LV', 'out_V_LV', 'V_batt'});
        [i_lv, ~]   = extract_signal_enhanced(simOut, {'I_LV', 'out_I_LV', 'I_batt'});
        if ~isempty(v_lv) && ~isempty(i_lv)
            P_lv = v_lv .* i_lv; t_lv = t_v; 
        end
    end
    
    % P_HV 폴백 로직 추가
    if isempty(P_hv)
        [v_hv, t_v_hv] = extract_signal_enhanced(simOut, {'V_HV', 'out_V_HV', 'V_Grid'});
        [i_hv, ~]   = extract_signal_enhanced(simOut, {'I_HV', 'out_I_HV'});
        if ~isempty(v_hv) && ~isempty(i_hv)
            P_hv = v_hv .* i_hv; t_hv = t_v_hv;
        end
    end
    
    % I_L(인덕터 전류)이 없어도, P_lv와 P_hv(전력)가 있으면 동작하도록 완화
    if isempty(P_lv) || isempty(P_hv)
        return; % 전력 데이터가 없으면 효율과 타겟 추종을 확인할 수 없으므로 종료
    end
    
    has_IL = ~isempty(I_L);
    
    for s = 1:N_STEPS
        t_end_step = T_START + (s-1)*T_STEP;
        if s == 1, t_end_step = T_START; end
        
        % [수정됨] 직전 1ms를 배제한 순수 정상 상태 분석 구간 계산 (-0.006초 ~ -0.001초)
        t_win_start_time = t_end_step - T_WIN_START;
        t_win_end_time   = t_end_step - T_WIN_END;
        
        mask_lv = (t_lv >= t_win_start_time) & (t_lv <= t_win_end_time);
        mask_hv = (t_hv >= t_win_start_time) & (t_hv <= t_win_end_time);
        
        if any(mask_lv) && any(mask_hv)
            % 윈도우 내 데이터 추출
            p_lv_window = P_lv(mask_lv);
            p_hv_window = P_hv(mask_hv);
            
            % 설정된 명확한 윈도우 구간 전체의 평균 사용
            res.p_lv(s) = mean(p_lv_window);
            res.p_hv(s) = mean(p_hv_window);
            
            % I_L 신호가 존재하는 경우에만 피크 전류 추출 
            if has_IL
                mask_i = (t_i >= t_win_start_time) & (t_i <= t_win_end_time);
                if any(mask_i)
                    i_l_window = I_L(mask_i);
                    res.i_pk(s) = max(abs(i_l_window));
                end
            end
            
            % 효율 계산 로직 (전력 방향에 따른 명확한 기준 적용)
            if abs(res.p_lv(s)) > 5 && abs(res.p_hv(s)) > 5
                if P_REFS(s) > 0 % LV 방전 (LV -> HV)
                    res.efficiency(s) = abs(res.p_hv(s)) / abs(res.p_lv(s));
                elseif P_REFS(s) < 0 % LV 충전 (HV -> LV)
                    res.efficiency(s) = abs(res.p_lv(s)) / abs(res.p_hv(s));
                else
                    res.efficiency(s) = 0;
                end
                res.efficiency(s) = min(1.0, res.efficiency(s));
            end
            
            % 제어기 로직에 따른 타겟 전력 기준 선정
            if P_REFS(s) > 0
                controlled_p = res.p_hv(s);
            else
                controlled_p = res.p_lv(s);
            end
            
            % 오차 허용 범위를 2.5%로 복구 (최소 마진 5W)
            err_tol = max(abs(P_REFS(s)) * 0.025, 5); 
            
            % 결과가 오차 범위 밖으로 나오더라도 무조건 기록
            res.valid(s) = true;
        end
    end
end