function verify_steady_fine(pointsCsv, outCsv, simTime)
% verify_steady_fine.m -- (10-30) verify_steady_state.m with a finer PWM edge / Simscape grid, per run.
%
% Why: duty_quant_check.py found that in the dataset-era model Ipk is exactly constant inside duty cells of 1/250
% (153/153 pairs identical to 1e-6 A) and jumps between neighbouring cells: at the SAME power, duties 1-2 cells
% (0.004-0.008) apart differ by 0.086 A (median) in Simulink but 0.0006 A in the ideal model. The controller and the
% Simscape local solver both step at 100 ns (125 steps per period, half period 62.5 steps). This copy reruns chosen
% points with Ts = 25 / 10 ns to see whether that roughness (and the OPT / M-S gaps of the same size) survives.
%
% Same as verify_steady_state.m (constant D0, last 1 ms, every-ms windows, i_L last period, per-period peaks) plus:
%   * the ORIGINAL controller dataset_era/DAB_Controller_System.m, as in every other job. (Until 10-30 13:00 this copy
%     put r11/DAB_Controller_System.m first on the path, but the current folder dataset_era/ shadows the path, so the
%     workers ran the original class and fine1 failed on all 117 runs: "... 'SyncPhaseUpdate'인 파라미터가 없음".)
%     The r11 copy is not needed here: in manual phase mode (Mode_Select 1, Ctrl_Mode_Ext 3) the controller's
%     one-period averages (fixed 125 samples) never reach the gates -- no PI, no feed-forward, flux balancing off in
%     mode 3 -- so Ts_ctrl alone sets the PWM edge grid (phase increment 360 Fs Ts_ctrl);
%   * per run: 'MATLAB System' Ts_ctrl = Ts and Solver Configuration2 LocalSolverSampleTime = Ts (points column Ts,
%     default 1e-7). The model's active configuration is variable-step (configSet4), so 25 / 10 ns sample times are
%     allowed; MATLAB System1 (efficiency analyzer, 500-sample buffer) stays at 100 ns;
%   * the cache folder is out/r11_cache (dataset_era/slprj, used by every other job, keeps its 100 ns build).
% Nothing is saved into dataset_era/ or Modules/.
%
%   python -I -X utf8 Simulink/Thesis/fine_grid_check.py make         -> out/fine_points_r1.csv
%   python automation/job.py submit fine1 --interactive --after r11smoke -- matlab -batch "cd('Simulink/Thesis'); verify_steady_fine_batch('out/fine_points_r1.csv', 'out/fine_sweep_r1.csv', 8, 0.020)"
%   python -I -X utf8 Simulink/Thesis/fine_grid_check.py compare

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    if nargin < 3 || isempty(simTime), simTime = 0.020; end
    pointsCsv = absPath(char(pointsCsv)); outCsv = absPath(char(outCsv));
    fprintf('verify_steady_fine: %s -> %s, %.1f ms per point\n', pointsCsv, outCsv, simTime * 1e3);

    pts = readtable(pointsCsv, 'TextType', 'string');
    N = height(pts);
    if ~ismember('Ts', pts.Properties.VariableNames), pts.Ts = 1e-7 * ones(N, 1); end
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;

    addpath(simDir); addpath(fullfile(simDir, 'Modules'));
    addpath(eraDir);                                   % last addpath = first on the path: old DAB_Controller_System.m
    cacheDir = fullfile(thesisDir, 'out', 'r11_cache');
    if ~isfolder(cacheDir), mkdir(cacheDir); end
    oldCache = Simulink.fileGenControl('get', 'CacheFolder');
    Simulink.fileGenControl('set', 'CacheFolder', cacheDir);
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd;
    cleanup = onCleanup(@() restore(oldDir, oldCache));
    cd(eraDir);
    w = which('DAB_Controller_System');                % checked after the cd: the current folder comes first
    fprintf('DAB_Controller_System -> %s\n', w);
    if ~contains(w, [filesep 'dataset_era' filesep]), error('the dataset-era controller is not the one in use'); end
    solverBlk = [modelName '/Solver' char(10) 'Configuration2'];

    t_in = [0; simTime];
    base = Simulink.SimulationInput(modelName);
    base = base.setModelParameter('StopTime', num2str(simTime));
    base = base.setModelParameter('ReturnWorkspaceOutputs', 'on');
    base = base.setVariable('Ref_VH', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('V_Grid', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('Ref_Power', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Ref_I_ess_cmd', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Mode_Select', timeseries(1 * ones(2, 1), t_in));
    base = base.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2, 1), t_in));
    base = base.setVariable('PWM_Enable', timeseries(1 * ones(2, 1), t_in));

    simIn(1:N) = Simulink.SimulationInput(modelName);
    for k = 1:N
        V2 = pts.V2(k); D0 = pts.D0(k); D1 = pts.D1(k); D2 = pts.D2(k); Ts = pts.Ts(k);
        s = base;
        s = s.setVariable('D0', timeseries(D0 * ones(2, 1), t_in));
        s = s.setVariable('Phi_Manual', timeseries(D0 * ones(2, 1), t_in));
        s = s.setVariable('V_init_LV', V2, 'Workspace', modelName);
        s = s.setVariable('V_batt', V2, 'Workspace', modelName);
        s = s.setVariable('Ref_VL', timeseries(V2 * ones(2, 1), t_in));
        s = s.setVariable('D1', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2', timeseries(D2 * ones(2, 1), t_in));
        s = s.setVariable('D1_ext_in', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2_ext_in', timeseries(D2 * ones(2, 1), t_in));
        s = s.setBlockParameter([modelName '/MATLAB System'], 'Ts_ctrl', num2str(Ts, 10));
        s = s.setBlockParameter(solverBlk, 'LocalSolverSampleTime', num2str(Ts, 10));
        simIn(k) = s;
    end

    setup = @() workerSetup(Kp_val, Ki_val, Kpv_val, Kiv_val, spec, eraDir);
    t0 = tic;
    usePar = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));
    if usePar
        try
            out = parsim(simIn, 'UseFastRestart', 'off', 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on', ...
                         'ShowSimulationManager', 'off', 'SetupFcn', setup);
        catch ME
            fprintf('parsim failed (%s); running sequentially\n', ME.message);
            usePar = false;
        end
    end
    if ~usePar
        setup();
        out = Simulink.SimulationOutput.empty;
        for k = 1:N
            out(k) = sim(simIn(k)); %#ok<AGROW>
            fprintf('  point %d/%d done (%.0f s)\n', k, N, toc(t0));
        end
    end
    fprintf('simulations done in %.0f s\n', toc(t0));

    winEnds = [0.010, 0.015:0.001:simTime];
    R = nan(N, 7 + 2 * numel(winEnds));
    err = strings(N, 1);
    Tsw = 1 / spec.fs; nW = 400;
    tW = simTime - Tsw + (0:nW - 1) * (Tsw / nW);
    Wv = nan(N, nW);
    nPP = round(1e-3 / Tsw);
    PPmax = nan(N, nPP); PPmin = nan(N, nPP);
    for k = 1:N
        o = out(k);
        if ~isempty(o.ErrorMessage), err(k) = string(o.ErrorMessage); continue; end
        [iL, tI] = extractSignal(o, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [pLV, tP] = extractSignal(o, {'Power_LV'});
        [pHV, tH] = extractSignal(o, {'Power_HV'});
        [vLV, tVL] = extractSignal(o, {'Voltage_Low1', 'V_LV1', 'V_LV'});
        [vHV, tVH] = extractSignal(o, {'Voltage_High1', 'V_HV1', 'V_HV'});
        if isempty(pLV) && ~isempty(vLV)
            [iLV, ~] = extractSignal(o, {'Current_Low1', 'I_LV1', 'I_LV'});
            if ~isempty(iLV), pLV = vLV .* iLV; tP = tVL; end
        end
        if isempty(pHV) && ~isempty(vHV)
            [iHV, ~] = extractSignal(o, {'Current_High1', 'I_HV1', 'I_HV'});
            if ~isempty(iHV), pHV = vHV .* iHV; tH = tVH; end
        end
        if isempty(iL), err(k) = "inductor current not logged"; continue; end
        w = @(t, te) (t >= te - 1e-3) & (t < te - 1e-6);
        m = w(tI, simTime);
        x = iL(m);
        R(k, 1) = max(abs(x));
        R(k, 2) = trapz(tI(m), x) / max(tI(find(m, 1, 'last')) - tI(find(m, 1)), eps);   % time-weighted mean
        R(k, 3) = sqrt(trapz(tI(m), x .^ 2) / max(tI(find(m, 1, 'last')) - tI(find(m, 1)), eps));
        if ~isempty(pLV), R(k, 4) = mean(pLV(w(tP, simTime))); end
        if ~isempty(pHV), R(k, 5) = mean(pHV(w(tH, simTime))); end
        if ~isempty(vLV), R(k, 6) = mean(vLV(w(tVL, simTime))); end
        if ~isempty(vHV), R(k, 7) = mean(vHV(w(tVH, simTime))); end
        for j = 1:numel(winEnds)
            R(k, 7 + j) = max(abs(iL(w(tI, winEnds(j)))));
            if ~isempty(pLV), R(k, 7 + numel(winEnds) + j) = mean(pLV(w(tP, winEnds(j)))); end
        end
        try
            mW = tI >= simTime - Tsw - 2e-7;
            [tu, iu] = unique(tI(mW));
            xW = iL(mW);
            Wv(k, :) = interp1(tu, xW(iu), tW, 'linear', NaN);
        catch
        end
        try
            for p = 1:nPP
                mp = (tI >= simTime - (nPP - p + 1) * Tsw) & (tI < simTime - (nPP - p) * Tsw);
                if any(mp), PPmax(k, p) = max(iL(mp)); PPmin(k, p) = min(iL(mp)); end
            end
        catch
        end
    end
    names = [{'Ipk_sim', 'iL_mean', 'iL_rms', 'P_LV', 'P_HV', 'V_LV', 'V_HV'}, ...
             arrayfun(@(t) sprintf('Ipk_at_%gms', t * 1e3), winEnds, 'UniformOutput', false), ...
             arrayfun(@(t) sprintf('P_LV_at_%gms', t * 1e3), winEnds, 'UniformOutput', false)];
    T = [pts, array2table(R, 'VariableNames', names), table(err, 'VariableNames', {'error'})];
    writetable(T, outCsv);
    fprintf('wrote %s (%d points, %d errors)\n', outCsv, N, sum(err ~= ""));
    try
        [pDir, fName] = fileparts(outCsv);
        wDir = fullfile(pDir, 'iL_wave');
        if ~isfolder(wDir), mkdir(wDir); end
        writetable([pts, array2table(Wv)], fullfile(wDir, [fName '.csv']));
    catch ME
        fprintf('i_L waveform not written: %s\n', ME.message);
    end
    try
        [pDir, fName] = fileparts(outCsv);
        pDir2 = fullfile(pDir, 'iL_pp');
        if ~isfolder(pDir2), mkdir(pDir2); end
        writetable([pts, array2table(PPmax, 'VariableNames', arrayfun(@(p) sprintf('max%d', p), 1:nPP, 'UniformOutput', false)), ...
                    array2table(PPmin, 'VariableNames', arrayfun(@(p) sprintf('min%d', p), 1:nPP, 'UniformOutput', false))], ...
                   fullfile(pDir2, [fName '.csv']));
    catch ME
        fprintf('i_L per-period peaks not written: %s\n', ME.message);
    end
end

function restore(oldDir, oldCache)
    cd(oldDir);
    Simulink.fileGenControl('set', 'CacheFolder', oldCache);
end

function p = absPath(p)
    if ~(numel(p) > 1 && p(2) == ':') && ~startsWith(p, {'\', '/'})
        p = fullfile(pwd, p);
    end
end

function workerSetup(Kp, Ki, Kpv, Kiv, spec, eraDir)
    addpath(eraDir);
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

function [data, t] = extractSignal(simOut, names)
% same lookup order as Generate_TPS_Dataset.m: To Workspace variable, logsout, yout
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
