function verify_steady_param(pointsCsv, outCsv, chunk, simTime)
% verify_steady_param.m -- R10 (parameter robustness): verify_steady_state.m with per-run plant parameters, in chunks.
%
% Same model, inputs and measurement as verify_steady_state.m (dataset-era model, manual phase, constant D0 for simTime,
% last 1 ms; 1 ms window peaks from 15 ms; i_L over the last period -> out/iL_wave/). Extra point columns, one value
% per run, applied only through Simulink.SimulationInput (the model is never saved):
%   L_uH   series inductor Inductor3 'l' (nominal 90)
%   td_ns  dead time of the four Half-Bridge Drivers UCC0..UCC3, tdLH = tdHL (nominal 100)
%   V1     HV source Ref_VH (nominal 380 V); the controller does not read V1 in manual phase mode
% Written as a copy (10-30) because opt1/eff1/zvs1 were running verify_steady_state.m at the time; fold the
% parameter columns back into verify_steady_state.m once no job uses it.
% Chunks of `chunk` runs (default 24, the client memory limit of verify_steady_batch.m), one output file per chunk
% <outCsv without .csv>_partNNN.csv; a chunk whose output exists is skipped (restart-safe).
%
%   python automation/job.py submit rob1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_param('out/robust_points_r1.csv', 'out/robust_sweep_r1.csv')"

    if nargin < 3 || isempty(chunk),   chunk = 24; end
    if nargin < 4 || isempty(simTime), simTime = 0.030; end
    pointsCsv = absPath(char(pointsCsv)); outCsv = absPath(char(outCsv));
    pts = readtable(pointsCsv, 'TextType', 'string');
    for c = {'L_uH', 'td_ns', 'V1'}
        if ~ismember(c{1}, pts.Properties.VariableNames), error('verify_steady_param: column %s missing', c{1}); end
    end
    N = height(pts);
    [d, b] = fileparts(outCsv);
    for s = 1:chunk:N
        e = min(N, s + chunk - 1);
        pout = fullfile(d, sprintf('%s_part%03d.csv', b, s));
        if isfile(pout)
            fprintf('verify_steady_param: runs %d-%d of %d already in %s, skipped\n', s, e, N, pout);
            continue;
        end
        fprintf('verify_steady_param: runs %d-%d of %d -> %s\n', s, e, N, pout);
        runPoints(pts(s:e, :), pout, simTime);
    end
    fprintf('verify_steady_param: done (%d runs)\n', N);
end

function runPoints(pts, outCsv, simTime)
    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    if ~isfile(fullfile(eraDir, 'Control_Logic_dataset_era.slx'))
        [st, msg] = system(sprintf('python -I "%s"', fullfile(thesisDir, 'make_dataset_era.py')));
        fprintf('%s', msg);
        if st ~= 0, error('make_dataset_era.py failed'); end
    end
    N = height(pts);
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;

    addpath(simDir); addpath(fullfile(simDir, 'Modules'));
    addpath(eraDir);                                   % last addpath = first on the path: old DAB_Controller_System.m
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd;
    cleanup = onCleanup(@() cd(oldDir));
    cd(eraDir);

    t_in = [0; simTime];
    base = Simulink.SimulationInput(modelName);
    base = base.setModelParameter('StopTime', num2str(simTime));
    base = base.setModelParameter('ReturnWorkspaceOutputs', 'on');
    base = base.setVariable('Ref_Power', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Ref_I_ess_cmd', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Mode_Select', timeseries(1 * ones(2, 1), t_in));
    base = base.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2, 1), t_in));
    base = base.setVariable('PWM_Enable', timeseries(1 * ones(2, 1), t_in));

    simIn(1:N) = Simulink.SimulationInput(modelName);
    for k = 1:N
        V2 = pts.V2(k); D0 = pts.D0(k); D1 = pts.D1(k); D2 = pts.D2(k);
        s = base;
        s = s.setVariable('Ref_VH', timeseries(pts.V1(k) * ones(2, 1), t_in));
        s = s.setVariable('V_Grid', timeseries(pts.V1(k) * ones(2, 1), t_in));
        s = s.setBlockParameter([modelName '/Inductor3'], 'l', sprintf('%.9g', pts.L_uH(k) * 1e-6));
        for u = 0:3
            blk = sprintf('%s/UCC%d', modelName, u);
            s = s.setBlockParameter(blk, 'tdLH', sprintf('%.9g', pts.td_ns(k)));
            s = s.setBlockParameter(blk, 'tdHL', sprintf('%.9g', pts.td_ns(k)));
        end
        s = s.setVariable('D0', timeseries(D0 * ones(2, 1), t_in));
        s = s.setVariable('Phi_Manual', timeseries(D0 * ones(2, 1), t_in));
        s = s.setVariable('V_init_LV', V2, 'Workspace', modelName);
        s = s.setVariable('V_batt', V2, 'Workspace', modelName);
        s = s.setVariable('Ref_VL', timeseries(V2 * ones(2, 1), t_in));
        s = s.setVariable('D1', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2', timeseries(D2 * ones(2, 1), t_in));
        s = s.setVariable('D1_ext_in', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2_ext_in', timeseries(D2 * ones(2, 1), t_in));
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
    for k = 1:N
        o = out(k);
        if ~isempty(o.ErrorMessage), err(k) = string(o.ErrorMessage); continue; end
        [iL, tI] = extractSignal(o, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [pLV, tP] = extractSignal(o, {'Power_LV'});
        [pHV, tH] = extractSignal(o, {'Power_HV'});
        [vLV, tVL] = extractSignal(o, {'Voltage_Low1', 'V_LV1', 'V_LV'});
        [vHV, tVH] = extractSignal(o, {'Voltage_High1', 'V_HV1', 'V_HV'});
        if isempty(iL), err(k) = "inductor current not logged"; continue; end
        w = @(t, te) (t >= te - 1e-3) & (t < te - 1e-6);
        m = w(tI, simTime);
        x = iL(m);
        R(k, 1) = max(abs(x));
        R(k, 2) = trapz(tI(m), x) / max(tI(find(m, 1, 'last')) - tI(find(m, 1)), eps);
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
