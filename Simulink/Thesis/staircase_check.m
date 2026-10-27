function staircase_check(combosCsv, outCsv, interps)
% staircase_check.m -- re-run Generate_TPS_Dataset.m's OWN experiment (44 ms, 39-step D0 staircase, 1 ms per step,
% 0.1 ms window at the end of each step) for a few (V2, D1, D2) combinations, twice:
%   interp = 1 : the model as it is (From Workspace 'D0' has "Interpolate data" on -- its default)
%   interp = 0 : the same run with that block's Interpolate set to 'off', i.e. D0 is a true staircase
%                (OutputAfterFinalValue must then be 'Holding final value': 'Extrapolation' needs interpolation;
%                 the data already ends at SIM_TIME with the last value repeated, so this changes nothing else)
% Same dataset-era model (git 35d5a78) and the same variables / worker setup as the dataset generator.
% If interp = 1 reproduces the dataset rows and interp = 0 does not, the dataset's step values were taken while D0
% was moving. Nothing is saved into the model.
%   matlab -batch "cd('Simulink/Thesis'); staircase_check"
%   matlab -batch "cd('Simulink/Thesis'); staircase_check('', fullfile(pwd, 'out', 'staircase_check_interp0.csv'), 0)"
    thesisDir = fileparts(mfilename('fullpath'));
    simDir = fileparts(thesisDir);
    eraDir = fullfile(thesisDir, 'dataset_era');
    if nargin < 1 || isempty(combosCsv), combosCsv = fullfile(thesisDir, 'out', 'staircase_combos.csv'); end
    if nargin < 2 || isempty(outCsv),    outCsv    = fullfile(thesisDir, 'out', 'staircase_check.csv'); end
    if nargin < 3 || isempty(interps),   interps   = [1, 0]; end
    C = readtable(combosCsv, 'TextType', 'string');

    % --- the generator's grid and staircase, verbatim (Scripts/Generate_TPS_Dataset.m)
    V2_vec = linspace(84, 109.2, 20); Dg = linspace(0.2, 0.5, 20);
    SIM_TIME = 0.044; T_STARTUP = 0.006; T_STEP = 0.001; T_WINDOW = 0.0001; N_D0 = 39;
    norm_x = linspace(-1, 1, N_D0);
    D0_vec = sign(norm_x) .* 90 .* (1 - sqrt(1 - abs(norm_x)));
    t_ts = 0; d_ts = D0_vec(1);
    for k = 1:(N_D0 - 1)
        t_ts(end + 1) = T_STARTUP + (k - 1) * T_STEP; %#ok<AGROW>
        d_ts(end + 1) = D0_vec(k + 1); %#ok<AGROW>
    end
    if t_ts(end) < SIM_TIME, t_ts(end + 1) = SIM_TIME; d_ts(end + 1) = d_ts(end); end
    ts_D0 = timeseries(d_ts', t_ts');
    ts_D0.DataInfo.Interpolation = tsdata.interpolation('zero-order hold');
    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    Kp_val = 0.05; Ki_val = 50.0; Kpv_val = 5.0; Kiv_val = 5000;

    addpath(simDir); addpath(fullfile(simDir, 'Modules')); addpath(eraDir);
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd; cleanup = onCleanup(@() cd(oldDir)); cd(eraDir);
    load_system(modelName);
    d0Blocks = find_system(modelName, 'BlockType', 'FromWorkspace', 'VariableName', 'D0');
    fprintf('D0 From Workspace block(s): %s | Interpolate = %s, OutputAfterFinalValue = %s, SampleTime = %s\n', ...
            strjoin(d0Blocks, ', '), get_param(d0Blocks{1}, 'Interpolate'), ...
            get_param(d0Blocks{1}, 'OutputAfterFinalValue'), get_param(d0Blocks{1}, 'SampleTime'));

    t_in = [0; SIM_TIME];
    base = Simulink.SimulationInput(modelName);
    base = base.setModelParameter('StopTime', num2str(SIM_TIME));
    base = base.setModelParameter('ReturnWorkspaceOutputs', 'on');
    base = base.setVariable('Ref_VH', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('V_Grid', timeseries(spec.V_in * ones(2, 1), t_in));
    base = base.setVariable('Ref_Power', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Ref_I_ess_cmd', timeseries(zeros(2, 1), t_in));
    base = base.setVariable('Mode_Select', timeseries(1 * ones(2, 1), t_in));
    base = base.setVariable('Ctrl_Mode_Ext', timeseries(3 * ones(2, 1), t_in));
    base = base.setVariable('PWM_Enable', timeseries(1 * ones(2, 1), t_in));
    base = base.setVariable('D0', ts_D0);
    base = base.setVariable('Phi_Manual', ts_D0);

    nC = height(C); simIn = Simulink.SimulationInput.empty; meta = zeros(0, 5);
    for c = 1:nC
        [~, iv] = min(abs(V2_vec - C.V2(c))); V2 = V2_vec(iv);
        if C.D1(c) > 0.499, D1 = 0.5; else, [~, i1] = min(abs(Dg - C.D1(c))); D1 = Dg(i1); end
        if C.D2(c) > 0.499, D2 = 0.5; else, [~, i2] = min(abs(Dg - C.D2(c))); D2 = Dg(i2); end
        for interp = interps
            s = base;
            s = s.setVariable('V_init_LV', V2, 'Workspace', modelName);
            s = s.setVariable('V_batt', V2, 'Workspace', modelName);
            s = s.setVariable('Ref_VL', timeseries(V2 * ones(2, 1), t_in));
            s = s.setVariable('D1', timeseries(D1 * ones(2, 1), t_in));
            s = s.setVariable('D2', timeseries(D2 * ones(2, 1), t_in));
            s = s.setVariable('D1_ext_in', timeseries(D1 * ones(2, 1), t_in));
            s = s.setVariable('D2_ext_in', timeseries(D2 * ones(2, 1), t_in));
            if interp == 0
                for b = 1:numel(d0Blocks)
                    s = s.setBlockParameter(d0Blocks{b}, 'OutputAfterFinalValue', 'Holding final value');
                    s = s.setBlockParameter(d0Blocks{b}, 'Interpolate', 'off');
                end
            end
            simIn(end + 1) = s; %#ok<AGROW>
            meta(end + 1, :) = [c, interp, V2, D1, D2]; %#ok<AGROW>
        end
    end
    setup = @() workerSetup(Kp_val, Ki_val, Kpv_val, Kiv_val, spec, eraDir);
    t0 = tic;
    try
        out = parsim(simIn, 'UseFastRestart', 'off', 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on', ...
                     'ShowSimulationManager', 'off', 'SetupFcn', setup);
    catch ME
        fprintf('parsim failed (%s); running sequentially\n', ME.message);
        setup(); out = Simulink.SimulationOutput.empty;
        for k = 1:numel(simIn), out(k) = sim(simIn(k)); end %#ok<AGROW>
    end
    fprintf('simulations done in %.0f s\n', toc(t0));

    rows = {};
    for k = 1:numel(out)
        o = out(k);
        if ~isempty(o.ErrorMessage), fprintf('run %d failed: %s\n', k, o.ErrorMessage); continue; end
        [iL, tI] = extractSignal(o, {'Inductor_Current1', 'Inductor_Current', 'I_L1', 'I_L'});
        [pLV, tP] = extractSignal(o, {'Power_LV'});
        [pHV, tH] = extractSignal(o, {'Power_HV'});
        for d = 1:N_D0
            if d == 1, te = T_STARTUP; else, te = T_STARTUP + (d - 1) * T_STEP; end
            ts0 = te - T_WINDOW;                                    % the generator's window, verbatim
            mI = (tI >= ts0) & (tI < te - 1e-6); mP = (tP >= ts0) & (tP < te - 1e-6); mH = (tH >= ts0) & (tH < te - 1e-6);
            rows(end + 1, :) = {C.tag(meta(k, 1)), meta(k, 2), meta(k, 3), meta(k, 4), meta(k, 5), d, D0_vec(d), ...
                                max(abs(iL(mI))), mean(pLV(mP)), mean(pHV(mH))}; %#ok<AGROW>
        end
    end
    T = cell2table(rows, 'VariableNames', {'tag', 'interp', 'V2', 'D1', 'D2', 'step', 'D0_label', 'Ipk', 'P_LV', 'P_HV'});
    writetable(T, outCsv);
    fprintf('wrote %s (%d rows)\n', outCsv, height(T));
end

function workerSetup(Kp, Ki, Kpv, Kiv, spec, eraDir)
    addpath(eraDir);
    assignin('base', 'C_HV', 1000e-6); assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01); assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', Kp); assignin('base', 'Ki_ext', Ki);
    assignin('base', 'Kpv', Kpv); assignin('base', 'Kiv', Kiv);
    assignin('base', 'spec', spec); assignin('base', 'fs', spec.fs);
    assignin('base', 'n', spec.n); assignin('base', 'V_in', spec.V_in);
end

function [data, t] = extractSignal(simOut, names)
    data = []; t = [];
    for k = 1:numel(names)
        if isprop(simOut, names{k})
            raw = simOut.(names{k});
            if isa(raw, 'timeseries'), data = squeeze(raw.Data); t = raw.Time; return; end
        end
        if isprop(simOut, 'logsout')
            try
                raw = simOut.logsout.get(names{k});
                if ~isempty(raw) && isa(raw.Values, 'timeseries'), data = squeeze(raw.Values.Data); t = raw.Values.Time; return; end
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
