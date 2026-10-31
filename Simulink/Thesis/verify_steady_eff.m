function verify_steady_eff(pointsCsv, outCsv, simTime, usePar, cacheName)
% verify_steady_eff.m -- (2026-10-31, final deadline, efficiency) verify_steady_fine.m plus what the efficiency verdict needs
% (rules: f1_eff.py header). Same constant-D0 run, same columns, and in addition, per run, over the last 1 ms:
%   P_HV_d, P_LV_d    direct time-weighted mean of Voltage_High1 * Current_High1, Voltage_Low1 * Current_Low1 at the
%                     Simscape local step (P_HV, P_LV = the analyzer's one-period averages, its block stays at 100 ns);
%   dV_HV, dV_LV      mean bus voltage, last 0.5 ms minus the 0.5 ms before (stored-energy check of the balance);
%   I_HV_rms, I_LV_rms, N_local (samples in the window: the step that really ran);
%   Eff_out, Eff_min, Eff_max, Loss_out, Cu_out, SemiHV_out, SemiLV_out, ZVS_out: the analyzer's other six outputs, which go
%                     only to Scope2/4/5/6/7/13 in the model; those scopes are made to log per run (nothing is saved).
% Points column Ts_solver (optional, default = Ts): the Simscape local solver step apart from the controller's Ts_ctrl.
% usePar 0 runs one by one (a smoke beside another MATLAB's pool). cacheName: folder under out/ (default r11_cache).
% The original controller and model, read only, as verify_steady_fine.m.

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    if nargin < 3 || isempty(simTime), simTime = 0.020; end
    if nargin < 4 || isempty(usePar), usePar = 1; end
    if nargin < 5 || isempty(cacheName), cacheName = 'r11_cache'; end
    pointsCsv = absPath(char(pointsCsv)); outCsv = absPath(char(outCsv));
    fprintf('verify_steady_eff: %s -> %s, %.1f ms per point\n', pointsCsv, outCsv, simTime * 1e3);

    pts = readtable(pointsCsv, 'TextType', 'string');
    N = height(pts);
    if ~ismember('Ts', pts.Properties.VariableNames), pts.Ts = 1e-7 * ones(N, 1); end
    if ~ismember('Ts_solver', pts.Properties.VariableNames), pts.Ts_solver = pts.Ts; end
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;

    addpath(simDir); addpath(fullfile(simDir, 'Modules'));
    addpath(eraDir);                                   % last addpath = first on the path: old DAB_Controller_System.m
    cacheDir = fullfile(thesisDir, 'out', cacheName);
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
    scopes = {'Scope2', 'Eff_out1'; 'Scope4', 'Loss_out1'; 'Scope5', 'Cu_out1'; 'Scope6', 'SemiHV_out1'; ...
              'Scope7', 'SemiLV_out1'; 'Scope13', 'ZVS_out1'};

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
    for q = 1:size(scopes, 1)
        blk = [modelName '/' scopes{q, 1}];
        base = base.setBlockParameter(blk, 'DataLogging', 'on');
        base = base.setBlockParameter(blk, 'DataLoggingVariableName', scopes{q, 2});
        base = base.setBlockParameter(blk, 'DataLoggingSaveFormat', 'Structure With Time');
        base = base.setBlockParameter(blk, 'DataLoggingLimitDataPoints', 'off');
        base = base.setBlockParameter(blk, 'DataLoggingDecimateData', 'off');
    end

    simIn(1:N) = Simulink.SimulationInput(modelName);
    for k = 1:N
        V2 = pts.V2(k); D0 = pts.D0(k); D1 = pts.D1(k); D2 = pts.D2(k);
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
        s = s.setBlockParameter([modelName '/MATLAB System'], 'Ts_ctrl', num2str(pts.Ts(k), 10));
        s = s.setBlockParameter(solverBlk, 'LocalSolverSampleTime', num2str(pts.Ts_solver(k), 10));
        simIn(k) = s;
    end

    setup = @() workerSetup(Kp_val, Ki_val, Kpv_val, Kiv_val, spec, eraDir);
    t0 = tic;
    usePar = usePar > 0 && license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel'));
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
    if simTime < 0.015, winEnds = simTime; end
    nExtra = 15;
    R = nan(N, 7 + 2 * numel(winEnds) + nExtra);
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
        [iLV, tIL] = extractSignal(o, {'Current_Low1', 'I_LV1', 'I_LV'});
        [iHV, tIH] = extractSignal(o, {'Current_High1', 'I_HV1', 'I_HV'});
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
        % ---- the columns this file adds
        c0 = 7 + 2 * numel(winEnds);
        R(k, c0 + 1) = directMean(vHV, tVH, iHV, tIH, simTime);
        R(k, c0 + 2) = directMean(vLV, tVL, iLV, tIL, simTime);
        R(k, c0 + 3) = halfDiff(vHV, tVH, simTime);
        R(k, c0 + 4) = halfDiff(vLV, tVL, simTime);
        R(k, c0 + 5) = rmsOver(iHV, tIH, simTime);
        R(k, c0 + 6) = rmsOver(iLV, tIL, simTime);
        R(k, c0 + 7) = sum(m);
        [e, te] = extractSignal(o, {scopes{1, 2}});
        if ~isempty(e)
            me = w(te, simTime);
            R(k, c0 + 8) = mean(e(me)); R(k, c0 + 9) = min(e(me)); R(k, c0 + 10) = max(e(me));
        end
        for q = 2:size(scopes, 1)                      % Loss, Cu, SemiHV, SemiLV, ZVS
            [e, te] = extractSignal(o, {scopes{q, 2}});
            if ~isempty(e), R(k, c0 + 9 + q) = mean(e(w(te, simTime))); end
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
             arrayfun(@(t) sprintf('P_LV_at_%gms', t * 1e3), winEnds, 'UniformOutput', false), ...
             {'P_HV_d', 'P_LV_d', 'dV_HV', 'dV_LV', 'I_HV_rms', 'I_LV_rms', 'N_local', 'Eff_out', 'Eff_min', 'Eff_max', ...
              'Loss_out', 'Cu_out', 'SemiHV_out', 'SemiLV_out', 'ZVS_out'}];
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

function y = directMean(v, tv, i, ti, te)
% time-weighted mean of v * i over the last 1 ms, on the current's time base
    y = NaN;
    if isempty(v) || isempty(i), return; end
    if numel(tv) ~= numel(ti) || any(tv ~= ti)
        [tu, iu] = unique(tv);
        v = interp1(tu, v(iu), ti, 'linear', NaN);
    end
    m = (ti >= te - 1e-3) & (ti < te - 1e-6);
    t = ti(m);
    y = trapz(t, v(m) .* i(m)) / max(t(end) - t(1), eps);
end

function y = halfDiff(v, t, te)
    y = NaN;
    if isempty(v), return; end
    a = (t >= te - 1e-3) & (t < te - 0.5e-3);
    b = (t >= te - 0.5e-3) & (t < te - 1e-6);
    y = mean(v(b)) - mean(v(a));
end

function y = rmsOver(x, t, te)
    y = NaN;
    if isempty(x), return; end
    m = (t >= te - 1e-3) & (t < te - 1e-6);
    tt = t(m);
    y = sqrt(trapz(tt, x(m) .^ 2) / max(tt(end) - tt(1), eps));
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
            if isstruct(raw) && isfield(raw, 'signals'), data = squeeze(raw.signals(1).values); t = raw.time; return; end   % a scope's log
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
