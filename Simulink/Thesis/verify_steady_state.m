function verify_steady_state(pointsCsv, outCsv, simTime)
% verify_steady_state.m -- T6: steady-state check of the analytic TPS model against the Simulink plant.
%
% Runs Modules/Control_Logic.slx with the SAME variables as Scripts/Generate_TPS_Dataset.m (manual phase mode:
% Mode_Select = 1, Ctrl_Mode_Ext = 3) but with a CONSTANT D0 for simTime (default 30 ms) instead of the 1 ms
% staircase, and measures the last 1 ms: Ipk = max|i_L|, mean(i_L) (DC offset), rms(i_L), P_LV, P_HV, V_LV, V_HV.
% Settling is checked with Ipk / P_LV in 1 ms windows ending at 10, 15, 20, 25 ms as well.
% Points come from make_verify_points.py (out/verify_points.csv); the comparison is done in Python.
%
% Run in the logged-on desktop session (Session 0 stalls `matlab -batch`):
%   python automation/job.py submit t6_verify --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_state"
% Read-only use of the original model: nothing is saved into Modules/.

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    if nargin < 1 || isempty(pointsCsv), pointsCsv = fullfile(thesisDir, 'out', 'verify_points.csv'); end
    if nargin < 2 || isempty(outCsv),    outCsv    = fullfile(thesisDir, 'out', 'simulink_steady.csv'); end
    if nargin < 3 || isempty(simTime),   simTime   = 0.030; end
    fprintf('verify_steady_state: %s -> %s, %.1f ms per point\n', pointsCsv, outCsv, simTime * 1e3);

    pts = readtable(pointsCsv, 'TextType', 'string');
    N = height(pts);
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;

    addpath(simDir); addpath(fullfile(simDir, 'Modules'));
    modelName = 'Control_Logic';
    oldDir = pwd;
    cleanup = onCleanup(@() cd(oldDir));
    cd(fullfile(simDir, 'Modules'));

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
        simIn(k) = s;
    end

    setup = @() workerSetup(Kp_val, Ki_val, Kpv_val, Kiv_val, spec);
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

    winEnds = [0.010, 0.015, 0.020, 0.025, simTime];
    R = nan(N, 7 + 2 * numel(winEnds));
    err = strings(N, 1);
    for k = 1:N
        o = out(k);
        if ~isempty(o.ErrorMessage), err(k) = string(o.ErrorMessage); continue; end
        [iL, tI] = extractSignal(o, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [pLV, tP] = extractSignal(o, {'Power_LV'});
        [pHV, tH] = extractSignal(o, {'Power_HV'});
        [vLV, tVL] = extractSignal(o, {'V_LV1', 'V_LV'});
        [vHV, tVH] = extractSignal(o, {'V_HV1', 'V_HV'});
        if isempty(pLV) && ~isempty(vLV)
            [iLV, ~] = extractSignal(o, {'I_LV1', 'I_LV'});
            if ~isempty(iLV), pLV = vLV .* iLV; tP = tVL; end
        end
        if isempty(pHV) && ~isempty(vHV)
            [iHV, ~] = extractSignal(o, {'I_HV1', 'I_HV'});
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
    end
    names = [{'Ipk_sim', 'iL_mean', 'iL_rms', 'P_LV', 'P_HV', 'V_LV', 'V_HV'}, ...
             arrayfun(@(t) sprintf('Ipk_at_%gms', t * 1e3), winEnds, 'UniformOutput', false), ...
             arrayfun(@(t) sprintf('P_LV_at_%gms', t * 1e3), winEnds, 'UniformOutput', false)];
    T = [pts, array2table(R, 'VariableNames', names), table(err, 'VariableNames', {'error'})];
    writetable(T, outCsv);
    fprintf('wrote %s (%d points, %d errors)\n', outCsv, N, sum(err ~= ""));
end

function workerSetup(Kp, Ki, Kpv, Kiv, spec)
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
