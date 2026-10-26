function verify_pi_closed_loop(pointsCsv, outCsv, simTime)
% verify_pi_closed_loop.m -- R2(3): Simulink PI closed loop with external (D1, D2) = the project's control structure.
%
% DATASET-ERA model (as verify_steady_state.m) in its own "TPS, external D1/D2, PI on the outer phase" mode:
%   Mode_Select = 0 (PI active), Ctrl_Mode_Ext = 3 (D1_ext_in / D2_ext_in used directly, flux balancing off),
%   Ref_Power = the power command (sign: (-) = HV -> LV charging, (+) = LV -> HV), held constant; the controller's own
%   slew limiter (50 W/ms) ramps it, its current PI (P_ref / V_LV vs the 125-sample average of I_LV for charging,
%   P_ref / V_HV vs I_HV for discharging) closes the outer phase D0.
% No controller code is changed and the model is not saved. In this model the controller's Mode_Select, Kp_ext and
% Ki_ext inputs are Constant blocks (slx_trace_inputs.py: Constant21 = 1 = manual phase, Constant4 = 20,
% Constant6 = 490000), not workspace variables, so they are overridden per simulation with setBlockParameter:
% Mode_Select -> 0 always; Kp / Ki -> the CSV columns, or the model's own gains where Kp < 0.
% Measured over the last 1 ms of simTime (default 35 ms): Ipk = max|i_L|, mean / rms i_L, P_LV, P_HV, V_LV, V_HV;
% settling: Ipk, P_LV, P_HV in 1 ms windows ending at 15, 20, 25, 30 ms and simTime.
% Points: make_pi_points.py -> out/pi_points.csv (label, ctrl, dir, V2, P_ref, D1, D2, Kp, Ki).
%
%   python automation/job.py submit NAME --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_pi_closed_loop"
% Read-only use of the original model: nothing is saved into Modules/ or dataset_era/.

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    if ~isfile(fullfile(eraDir, 'Control_Logic_dataset_era.slx'))
        [st, msg] = system(sprintf('python -I "%s"', fullfile(thesisDir, 'make_dataset_era.py')));
        fprintf('%s', msg);
        if st ~= 0, error('make_dataset_era.py failed'); end
    end
    if nargin < 1 || isempty(pointsCsv), pointsCsv = fullfile(thesisDir, 'out', 'pi_points.csv'); end
    if nargin < 2 || isempty(outCsv),    outCsv    = fullfile(thesisDir, 'out', 'simulink_pi_raw.csv'); end
    if nargin < 3 || isempty(simTime),   simTime   = 0.035; end
    % relative paths are relative to the caller's folder: resolve them before cd(eraDir) below
    pointsCsv = absPath(char(pointsCsv)); outCsv = absPath(char(outCsv));
    fprintf('verify_pi_closed_loop: %s -> %s, %.1f ms per point\n', pointsCsv, outCsv, simTime * 1e3);

    pts = readtable(pointsCsv, 'TextType', 'string');
    N = height(pts);
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;     % stock values (Generate_TPS_Dataset.m)

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
    base = base.setVariable('Ref_VH', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('V_Grid', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('Ref_I_ess_cmd', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Mode_Select', timeseries(0 * ones(2, 1), t_in));      % (unused by this model, see above)
    base = base.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2, 1), t_in));    % (Constant8 = 3 in the model)
    base = base.setBlockParameter([modelName '/Constant21'], 'Value', '0');       % Mode_Select = 0: PI on the phase
    base = base.setVariable('PWM_Enable', timeseries(1 * ones(2, 1), t_in));
    base = base.setVariable('D0', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Phi_Manual', timeseries(zeros(2, 1), t_in));

    simIn(1:N) = Simulink.SimulationInput(modelName);
    for k = 1:N
        V2 = pts.V2(k); D1 = pts.D1(k); D2 = pts.D2(k);
        s = base;
        s = s.setVariable('Ref_Power', timeseries(pts.P_ref(k) * ones(2, 1), t_in));
        s = s.setVariable('V_init_LV', V2, 'Workspace', modelName);
        s = s.setVariable('V_batt', V2, 'Workspace', modelName);
        s = s.setVariable('Ref_VL', timeseries(V2 * ones(2, 1), t_in));
        s = s.setVariable('D1', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2', timeseries(D2 * ones(2, 1), t_in));
        s = s.setVariable('D1_ext_in', timeseries(D1 * ones(2, 1), t_in));
        s = s.setVariable('D2_ext_in', timeseries(D2 * ones(2, 1), t_in));
        if pts.Kp(k) >= 0                                % else: the model's own PI gains (Constant4 / Constant6)
            s = s.setBlockParameter([modelName '/Constant4'], 'Value', num2str(pts.Kp(k), 10));
            s = s.setBlockParameter([modelName '/Constant6'], 'Value', num2str(pts.Ki(k), 10));
        end
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

    winEnds = [0.015, 0.020, 0.025, 0.030, simTime];
    nw = numel(winEnds);
    R = nan(N, 7 + 3 * nw);
    err = strings(N, 1);
    for k = 1:N
        o = out(k);
        if ~isempty(o.ErrorMessage), err(k) = string(o.ErrorMessage); continue; end
        [iL, tI] = extractSignal(o, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [pLV, tP] = extractSignal(o, {'Power_LV'});
        [pHV, tH] = extractSignal(o, {'Power_HV'});
        [vLV, tVL] = extractSignal(o, {'Voltage_Low1', 'V_LV1', 'V_LV'});      % dataset-era names first
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
        for j = 1:nw
            R(k, 7 + j) = max(abs(iL(w(tI, winEnds(j)))));
            if ~isempty(pLV), R(k, 7 + nw + j) = mean(pLV(w(tP, winEnds(j)))); end
            if ~isempty(pHV), R(k, 7 + 2 * nw + j) = mean(pHV(w(tH, winEnds(j)))); end
        end
    end
    names = [{'Ipk_sim', 'iL_mean', 'iL_rms', 'P_LV', 'P_HV', 'V_LV', 'V_HV'}, ...
             arrayfun(@(t) sprintf('Ipk_at_%gms', t * 1e3), winEnds, 'UniformOutput', false), ...
             arrayfun(@(t) sprintf('P_LV_at_%gms', t * 1e3), winEnds, 'UniformOutput', false), ...
             arrayfun(@(t) sprintf('P_HV_at_%gms', t * 1e3), winEnds, 'UniformOutput', false)];
    T = [pts, array2table(R, 'VariableNames', names), table(err, 'VariableNames', {'error'})];
    writetable(T, outCsv);
    fprintf('wrote %s (%d points, %d errors)\n', outCsv, N, sum(err ~= ""));
end

function p = absPath(p)
    if ~(numel(p) > 1 && p(2) == ':') && ~startsWith(p, {'\', '/'})
        p = fullfile(pwd, p);
    end
end

function workerSetup(Kp, Ki, Kpv, Kiv, spec, eraDir)
    addpath(eraDir);                                   % parsim workers: old DAB_Controller_System.m first on the path
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
