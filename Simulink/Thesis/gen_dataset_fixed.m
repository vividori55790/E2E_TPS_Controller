function gen_dataset_fixed(outPrefix, v2idx, dIdx, batchSize, nWorkers)
% gen_dataset_fixed.m -- R6: the user's dataset experiment (Scripts/Generate_TPS_Dataset.m) run exactly as written,
% with ONE change: the D0 From Workspace block (From Workspace11) gets Interpolate = 'off' (+ OutputAfterFinalValue =
% 'Holding final value', which Simulink requires once interpolation is off; the data already end at SIM_TIME with the
% last value repeated). The generator meant a zero-order-hold staircase (its timeseries carries a ZOH flag), but the
% block's default "Interpolate data = on" makes D0 ramp linearly through every 1 ms step (R5, staircase_check.m:
% interp = 1 reproduces the checkpoint rows bit for bit).
% Same dataset-era model (git 35d5a78, dataset_era/), same 44 ms run, 39-step D0 staircase, 1 ms steps, 0.1 ms window
% before each step change, same worker variables, same signal lookup and window arithmetic. Nothing is saved into the
% model. Reduced grid (deadline): V2_vec(v2idx) x Dg(dIdx) x Dg(dIdx) of the generator's own 20-point grids
% (default V2 84 / 94.61 / 109.2 V, D = every other grid value + 0.5 -> 11 x 11), so every combination is also a row
% set of the original dataset.
% Restart-safe: each batch is written to <outPrefix>_part_NNN.csv; combinations already present in any part file are
% skipped on the next call. Extra columns (not in the original dataset): iL_mean (window mean of i_L), Ipk_prev
% (max|i_L| in the 0.1 ms window just before, a settling check), V_LV, V_HV (window means).
%
%   python automation/job.py submit R6 --interactive -- matlab -batch "cd('Simulink/Thesis'); gen_dataset_fixed"
%   smoke: matlab -batch "cd('Simulink/Thesis'); gen_dataset_fixed(fullfile(pwd,'out','dataset_fixed_smoke'), 1, [20 11], 4)"
    thesisDir = fileparts(mfilename('fullpath'));
    simDir = fileparts(thesisDir);
    eraDir = fullfile(thesisDir, 'dataset_era');
    if nargin < 1 || isempty(outPrefix), outPrefix = fullfile(thesisDir, 'out', 'dataset_fixed'); end
    if nargin < 2 || isempty(v2idx),     v2idx = [1, 9, 20]; end
    if nargin < 3 || isempty(dIdx),      dIdx = [1:2:19, 20]; end
    if nargin < 4 || isempty(batchSize), batchSize = 12; end
    if nargin < 5 || isempty(nWorkers),  nWorkers = 6; end

    % --- Generate_TPS_Dataset.m, verbatim
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    SIM_TIME = 0.044; T_STEP = 0.001; T_WINDOW = 0.0001; N_D0_STEPS = 39;
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;
    V2_vec = linspace(84, 109.2, 20); D1_vec = linspace(0.2, 0.5, 20); D2_vec = linspace(0.2, 0.5, 20);
    norm_x = linspace(-1, 1, N_D0_STEPS);
    D0_vec = sign(norm_x) .* 90 .* (1 - sqrt(1 - abs(norm_x)));
    T_STARTUP = 0.006;
    t_ts = 0; d_ts = D0_vec(1);
    for k = 1:(N_D0_STEPS - 1)
        t_ts(end + 1) = T_STARTUP + (k - 1) * T_STEP; %#ok<AGROW>
        d_ts(end + 1) = D0_vec(k + 1); %#ok<AGROW>
    end
    if t_ts(end) < SIM_TIME, t_ts(end + 1) = SIM_TIME; d_ts(end + 1) = d_ts(end); end
    ts_D0 = timeseries(d_ts', t_ts');
    ts_D0.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');

    % reduced grid (generator's ndgrid), run in priority order so that a partial run is already usable:
    % SPS first, then a coarse sub-grid (every other D value + 0.5), then the rest
    [V2_grid, D1_grid, D2_grid] = ndgrid(V2_vec(v2idx), D1_vec(dIdx), D2_vec(dIdx));
    [~, J1, J2] = ndgrid(1:numel(v2idx), 1:numel(dIdx), 1:numel(dIdx));
    V2_all = V2_grid(:); D1_all = D1_grid(:); D2_all = D2_grid(:);
    N_sims = numel(V2_all);
    nD = numel(dIdx); coarse = unique([1:2:nD, nD]);
    prio = 2 * ones(N_sims, 1);
    prio(ismember(J1(:), coarse) & ismember(J2(:), coarse)) = 1;
    prio(abs(D1_all - 0.5) < 1e-12 & abs(D2_all - 0.5) < 1e-12) = 0;
    [~, order] = sortrows([prio, (1:N_sims)']);
    V2_all = V2_all(order); D1_all = D1_all(order); D2_all = D2_all(order);
    fprintf('gen_dataset_fixed: %d combinations (V2 %s; D %s), %d D0 steps each -> %s_part_NNN.csv\n', N_sims, ...
            mat2str(V2_vec(v2idx), 6), mat2str(D1_vec(dIdx), 4), N_D0_STEPS, outPrefix);

    % --- resume: combinations already written
    [outDir, outBase] = fileparts(outPrefix);
    parts = dir(fullfile(outDir, [outBase '_part_*.csv']));
    done = false(N_sims, 1); lastPart = 0;
    for p = 1:numel(parts)
        T = readtable(fullfile(parts(p).folder, parts(p).name));
        for r = 1:height(T)
            j = find(abs(V2_all - T.V2(r)) < 1e-9 & abs(D1_all - T.D1(r)) < 1e-9 & abs(D2_all - T.D2(r)) < 1e-9, 1);
            if ~isempty(j), done(j) = true; end
        end
        tok = regexp(parts(p).name, '_part_(\d+)\.csv$', 'tokens', 'once');
        if ~isempty(tok), lastPart = max(lastPart, str2double(tok{1})); end
    end
    todo = find(~done);
    fprintf('  %d already in %d part file(s), %d to run\n', sum(done), numel(parts), numel(todo));
    if isempty(todo), fprintf('gen_dataset_fixed: nothing to do\n'); return; end

    addpath(simDir); addpath(fullfile(simDir, 'Modules')); addpath(eraDir);
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd; cleanup = onCleanup(@() cd(oldDir)); cd(eraDir);
    load_system(modelName);
    d0Blocks = find_system(modelName, 'BlockType', 'FromWorkspace', 'VariableName', 'D0');
    if numel(d0Blocks) ~= 1, error('expected one From Workspace block reading D0, found %d', numel(d0Blocks)); end
    fprintf('  D0 block %s: Interpolate %s -> off (per simulation, model not saved)\n', d0Blocks{1}, ...
            get_param(d0Blocks{1}, 'Interpolate'));

    t_input = [0; SIM_TIME];
    baseIn = Simulink.SimulationInput(modelName);
    baseIn = baseIn.setModelParameter('StopTime', num2str(SIM_TIME));
    baseIn = baseIn.setModelParameter('ReturnWorkspaceOutputs', 'on');
    baseIn = baseIn.setVariable('Ref_VH', timeseries(spec.V_in * ones(2, 1), t_input));
    baseIn = baseIn.setVariable('V_Grid', timeseries(spec.V_in * ones(2, 1), t_input));
    baseIn = baseIn.setVariable('Ref_Power', timeseries(zeros(2, 1), t_input));
    baseIn = baseIn.setVariable('Ref_I_ess_cmd', timeseries(zeros(2, 1), t_input));
    baseIn = baseIn.setVariable('Mode_Select', timeseries(1 * ones(2, 1), t_input));
    baseIn = baseIn.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2, 1), t_input));
    baseIn = baseIn.setVariable('PWM_Enable', timeseries(1 * ones(2, 1), t_input));
    baseIn = baseIn.setVariable('D0', ts_D0);
    baseIn = baseIn.setVariable('Phi_Manual', ts_D0);
    % the one change: a true staircase
    baseIn = baseIn.setBlockParameter(d0Blocks{1}, 'OutputAfterFinalValue', 'Holding final value');
    baseIn = baseIn.setBlockParameter(d0Blocks{1}, 'Interpolate', 'off');

    startPool(nWorkers);
    setup = @() workerSetupFcn(Kp_val, Ki_val, Kpv_val, Kiv_val, spec, eraDir);
    names = {'combo', 'V2', 'D1', 'D2', 'step', 'D0', 'I_pk', 'P_act', 'P_act_HV', 'valid', ...
             'iL_mean', 'Ipk_prev', 'V_LV', 'V_HV'};
    t0 = tic; nBatches = ceil(numel(todo) / batchSize);
    for b = 1:nBatches
        idx = todo((b - 1) * batchSize + 1:min(b * batchSize, numel(todo)));
        simIn = Simulink.SimulationInput.empty;
        for k = 1:numel(idx)
            j = idx(k);
            s = baseIn;
            s = s.setVariable('V_init_LV', V2_all(j), 'Workspace', modelName);
            s = s.setVariable('V_batt', V2_all(j), 'Workspace', modelName);
            s = s.setVariable('Ref_VL', timeseries(V2_all(j) * ones(2, 1), t_input));
            s = s.setVariable('D1', timeseries(D1_all(j) * ones(2, 1), t_input));
            s = s.setVariable('D2', timeseries(D2_all(j) * ones(2, 1), t_input));
            s = s.setVariable('D1_ext_in', timeseries(D1_all(j) * ones(2, 1), t_input));
            s = s.setVariable('D2_ext_in', timeseries(D2_all(j) * ones(2, 1), t_input));
            simIn(k) = s; %#ok<AGROW>
        end
        out = [];
        for attempt = 1:2
            try
                out = parsim(simIn, 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                             'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', 'SetupFcn', setup);
                break;
            catch ME
                fprintf('  [batch %d/%d] parsim failed (attempt %d): %s\n', b, nBatches, attempt, ME.message);
                out = [];
                startPool(nWorkers);
            end
        end
        if isempty(out), fprintf('  [batch %d/%d] skipped (rerun to retry)\n', b, nBatches); continue; end
        rows = cell(0, numel(names));
        for k = 1:numel(idx)
            j = idx(k); o = out(k);
            if ~isempty(o.ErrorMessage)
                fprintf('  combo %d (V2 %.4g, D1 %.4g, D2 %.4g) FAILED: %s\n', j, V2_all(j), D1_all(j), D2_all(j), ...
                        o.ErrorMessage);
                continue;
            end
            res = analyze_sim_data(o, T_STARTUP, T_STEP, N_D0_STEPS, T_WINDOW);
            if ~isempty(res.error_msg), fprintf('  combo %d analysis FAILED: %s\n', j, res.error_msg); continue; end
            for d = 1:N_D0_STEPS
                rows(end + 1, :) = {j, V2_all(j), D1_all(j), D2_all(j), d, D0_vec(d), res.i_pk(d), res.p_act(d), ...
                                    res.p_act_hv(d), res.valid(d), res.il_mean(d), res.i_pk_prev(d), ...
                                    res.v_lv(d), res.v_hv(d)}; %#ok<AGROW>
            end
        end
        clear out o simIn
        if ~isempty(rows)
            lastPart = lastPart + 1;
            f = sprintf('%s_part_%03d.csv', outPrefix, lastPart);
            writetable(cell2table(rows, 'VariableNames', names), f);
            fprintf('  [batch %d/%d] %d combos -> %s (%.0f s elapsed)\n', b, nBatches, size(rows, 1) / N_D0_STEPS, ...
                    f, toc(t0));
        end
    end
    fprintf('gen_dataset_fixed: done in %.0f s\n', toc(t0));
end

function startPool(nWorkers)
% explicit pool (stair_i0 on 10-27 got no pool from parsim's auto-start and ran serially)
    p = gcp('nocreate');
    if ~isempty(p) && p.Connected, return; end
    if ~isempty(p), delete(p); end
    c = parcluster('Processes');
    for attempt = 1:2
        try
            if c.NumWorkers < nWorkers, c.NumWorkers = nWorkers; end
            parpool(c, nWorkers);
            fprintf('  pool: %d workers\n', nWorkers);
            return;
        catch ME
            fprintf('  parpool failed (attempt %d): %s\n', attempt, ME.message);
            % old crashed jobs kept in the local job storage ("crash dump files") -- remove the finished/failed ones only
            try
                js = c.Jobs;
                for q = 1:numel(js)
                    if ~any(strcmp(js(q).State, {'running', 'queued', 'pending'})), delete(js(q)); end
                end
            catch ME2
                fprintf('  job cleanup failed: %s\n', ME2.message);
            end
        end
    end
    fprintf('  WARNING: no parallel pool -- parsim will run serially\n');
end

function workerSetupFcn(Kp, Ki, Kpv, Kiv, spec, eraDir)
    addpath(eraDir);                                   % dataset-era DAB_Controller_System.m first on the path
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

function res = analyze_sim_data(simOut, T_STARTUP, T_STEP, N_D0_STEPS, T_WINDOW)
% Generate_TPS_Dataset.m's analyze_sim_data (same signals, same windows, same masks) + the extra columns
    res = struct('error_msg', '', 'p_act', NaN(N_D0_STEPS, 1), 'p_act_hv', NaN(N_D0_STEPS, 1), ...
                 'i_pk', NaN(N_D0_STEPS, 1), 'valid', false(N_D0_STEPS, 1), 'il_mean', NaN(N_D0_STEPS, 1), ...
                 'i_pk_prev', NaN(N_D0_STEPS, 1), 'v_lv', NaN(N_D0_STEPS, 1), 'v_hv', NaN(N_D0_STEPS, 1));
    try
        [I_L_data, t_sim] = extract_signal(simOut, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [P_lv_data, t_plv] = extract_signal(simOut, {'Power_LV'});
        [P_hv_data, t_phv] = extract_signal(simOut, {'Power_HV'});
        [v_lv, t_vl] = extract_signal(simOut, {'Voltage_Low1', 'V_LV1', 'V_LV'});
        [v_hv, t_vh] = extract_signal(simOut, {'Voltage_High1', 'V_HV1', 'V_HV'});
        if isempty(I_L_data) || isempty(P_lv_data) || isempty(P_hv_data)
            res.error_msg = 'Required signals not found. (Check To Workspace or Data Logging blocks)';
            return;
        end
        for d_idx = 1:N_D0_STEPS
            if d_idx == 1, t_end_step = T_STARTUP; else, t_end_step = T_STARTUP + (d_idx - 1) * T_STEP; end
            t_start_step = t_end_step - T_WINDOW;
            mask_i = (t_sim >= t_start_step) & (t_sim < t_end_step - 1e-6);
            mask_p_lv = (t_plv >= t_start_step) & (t_plv < t_end_step - 1e-6);
            mask_p_hv = (t_phv >= t_start_step) & (t_phv < t_end_step - 1e-6);
            if sum(mask_i) > 0 && sum(mask_p_lv) > 0 && sum(mask_p_hv) > 0
                res.i_pk(d_idx) = max(abs(I_L_data(mask_i)));
                res.p_act(d_idx) = mean(P_lv_data(mask_p_lv));
                res.p_act_hv(d_idx) = mean(P_hv_data(mask_p_hv));
                res.valid(d_idx) = true;
                res.il_mean(d_idx) = mean(I_L_data(mask_i));
                mprev = (t_sim >= t_start_step - T_WINDOW) & (t_sim < t_start_step - 1e-6);
                if any(mprev), res.i_pk_prev(d_idx) = max(abs(I_L_data(mprev))); end
                if ~isempty(v_lv)
                    m = (t_vl >= t_start_step) & (t_vl < t_end_step - 1e-6);
                    if any(m), res.v_lv(d_idx) = mean(v_lv(m)); end
                end
                if ~isempty(v_hv)
                    m = (t_vh >= t_start_step) & (t_vh < t_end_step - 1e-6);
                    if any(m), res.v_hv(d_idx) = mean(v_hv(m)); end
                end
            end
        end
    catch ME
        res.error_msg = ME.message;
    end
end

function [data, t] = extract_signal(simOut, names)
% Generate_TPS_Dataset.m's lookup (To Workspace variable, then logsout, then yout)
    data = []; t = [];
    for k = 1:numel(names)
        if isprop(simOut, names{k})
            raw = simOut.(names{k});
            if isa(raw, 'timeseries'), data = squeeze(raw.Data); t = raw.Time; return; end
        end
        if isprop(simOut, 'logsout')
            try
                raw = simOut.logsout.get(names{k});
                if ~isempty(raw) && isa(raw.Values, 'timeseries')
                    data = squeeze(raw.Values.Data); t = raw.Values.Time; return;
                end
            catch
            end
        end
        if isprop(simOut, 'yout')
            try
                raw = simOut.yout.getElement(names{k});
                if ~isempty(raw), data = squeeze(raw.Values.Data); t = raw.Values.Time; return; end
            catch
            end
        end
    end
end
