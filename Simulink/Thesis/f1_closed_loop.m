function f1_closed_loop(pointsCsv, outDir, batch, usePar, cacheName)
% f1_closed_loop.m -- (2026-10-31, final deadline) r11_closed_loop.m with the regression INSIDE the Simulink controller (rules: f1_regloop.py).
% Same closed loop (PI on the outer phase, the points' Ts for Ts_ctrl and the local solver, per-period phase latch, profile CSV for Ref_Power and
% Ref_VL) with v3/DAB_Controller_System.m, a copy of the r11 controller whose RegressionInLoop = true takes D1, D2 from smooth_reghdc_infer_v3(P_cmd,
% V_LV_f, V_HV_f) in its own step, on the measured one-period averages, once every RegHoldPeriods periods. The profile's D1, D2 columns are then unused.
% Points CSV columns: label, ctrl, V2, Ts, sync, Kp, Ki, simTime, profile, trim, reg (0/1), hold (periods).
% Outputs per run as r11_closed_loop.m (restart-safe), periods file with a V_HV column more. usePar 0: one by one (smoke beside another MATLAB's pool).
% cacheName: folder under out/ (default v3_cache). The original model is read only, nothing is saved.

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    v3Dir     = fullfile(thesisDir, 'v3');
    if nargin < 2 || isempty(outDir), outDir = fullfile(thesisDir, 'out', 'f1v3'); end
    if nargin < 3 || isempty(batch),  batch  = 4; end
    if nargin < 4 || isempty(usePar), usePar = 1; end
    if nargin < 5 || isempty(cacheName), cacheName = 'v3_cache'; end
    pointsCsv = absPath(char(pointsCsv)); outDir = absPath(char(outDir));
    if ~isfolder(outDir), mkdir(outDir); end
    fprintf('f1_closed_loop: %s -> %s\n', pointsCsv, outDir);

    pts = readtable(pointsCsv, 'TextType', 'string');
    todo = [];
    for k = 1:height(pts)
        if ~isfile(fullfile(outDir, pts.label(k) + "_summary.csv")), todo(end + 1) = k; end %#ok<AGROW>
    end
    fprintf('%d of %d runs to do\n', numel(todo), height(pts));
    if isempty(todo), return; end

    spec = struct('V_in', 380, 'n', 4, 'fs', 80e3, 'I_pri_max', 10, 'I_sec_max', 40);
    addpath(simDir); addpath(fullfile(simDir, 'Modules'));
    addpath(eraDir);
    cacheDir = fullfile(thesisDir, 'out', cacheName);
    if ~isfolder(cacheDir), mkdir(cacheDir); end
    oldCache = Simulink.fileGenControl('get', 'CacheFolder');
    Simulink.fileGenControl('set', 'CacheFolder', cacheDir);
    % after the cache change: 'set' takes the old cache folder off the path, and that folder may be this one
    addpath(thesisDir);                                % smooth_reghdc_params.m (and the original smooth_reghdc_infer.m, for its test)
    addpath(v3Dir);                                    % last addpath = first on the path: the V3 controller copy
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd;
    cleanup = onCleanup(@() restore(oldDir, oldCache));
    cd(v3Dir);                                         % the current folder comes before the path (see r11_closed_loop.m)
    w = which('DAB_Controller_System');
    fprintf('DAB_Controller_System -> %s\n', w);
    if ~contains(w, [filesep 'v3' filesep]), error('v3 controller copy is not the one in use'); end
    w = which('smooth_reghdc_infer_v3');
    fprintf('smooth_reghdc_infer_v3 -> %s\n', w);
    if isempty(w) || isempty(which('smooth_reghdc_params')), error('the regression files are not on the path'); end
    solverBlk = [modelName '/Solver' char(10) 'Configuration2'];
    trimBlocks = {'To Workspace1', 'To Workspace3', 'To Workspace4', 'To Workspace9', 'To Workspace11', ...
                  'To Workspace12', 'To Workspace13', 'To Workspace14', 'To Workspace15', 'To Workspace16'};

    for b0 = 1:batch:numel(todo)
        idx = todo(b0:min(b0 + batch - 1, numel(todo)));
        simIn(1:numel(idx)) = Simulink.SimulationInput(modelName); %#ok<AGROW>
        for j = 1:numel(idx)
            k = idx(j);
            pr = readtable(fullfile(thesisDir, char(pts.profile(k))));
            t = pr.t(:);
            ts = @(v) timeseries(v(:), t);
            s = Simulink.SimulationInput(modelName);
            s = s.setModelParameter('StopTime', num2str(pts.simTime(k), 10));
            s = s.setModelParameter('ReturnWorkspaceOutputs', 'on');
            s = s.setVariable('Ref_VH', ts(spec.V_in * ones(size(t))));
            s = s.setVariable('V_Grid', ts(spec.V_in * ones(size(t))));
            s = s.setVariable('Ref_I_ess_cmd', ts(zeros(size(t))));
            s = s.setVariable('Mode_Select', ts(zeros(size(t))));
            s = s.setVariable('Ctrl_Mode_Ext', ts(3 * ones(size(t))));
            s = s.setVariable('PWM_Enable', ts(ones(size(t))));
            s = s.setVariable('D0', ts(zeros(size(t))));
            s = s.setVariable('Phi_Manual', ts(zeros(size(t))));
            s = s.setVariable('Ref_Power', ts(pr.P_ref));
            s = s.setVariable('D1', ts(pr.D1));            s = s.setVariable('D2', ts(pr.D2));
            s = s.setVariable('D1_ext_in', ts(pr.D1));     s = s.setVariable('D2_ext_in', ts(pr.D2));
            s = s.setVariable('Ref_VL', ts(pr.V2));
            s = s.setVariable('V_init_LV', pr.V2(1), 'Workspace', modelName);
            s = s.setVariable('V_batt', pr.V2(1), 'Workspace', modelName);
            s = s.setBlockParameter([modelName '/Constant21'], 'Value', '0');
            s = s.setBlockParameter([modelName '/Constant4'], 'Value', num2str(pts.Kp(k), 10));
            s = s.setBlockParameter([modelName '/Constant6'], 'Value', num2str(pts.Ki(k), 10));
            s = s.setBlockParameter([modelName '/MATLAB System'], 'Ts_ctrl', num2str(pts.Ts(k), 10));
            s = s.setBlockParameter([modelName '/MATLAB System'], 'SyncPhaseUpdate', onoff(pts.sync(k)));
            s = s.setBlockParameter([modelName '/MATLAB System'], 'RegressionInLoop', onoff(pts.reg(k)));
            s = s.setBlockParameter([modelName '/MATLAB System'], 'RegHoldPeriods', num2str(pts.hold(k)));
            s = s.setBlockParameter(solverBlk, 'LocalSolverSampleTime', num2str(pts.Ts(k), 10));
            if pts.trim(k) > 0
                for q = 1:numel(trimBlocks)
                    s = s.setBlockParameter([modelName '/' trimBlocks{q}], 'Commented', 'on');
                end
            end
            simIn(j) = s;
        end
        fprintf('batch %d-%d of %d (%s)\n', b0, b0 + numel(idx) - 1, numel(todo), char(datetime('now')));
        setup = @() workerSetup(spec, eraDir, thesisDir, v3Dir);
        t0 = tic;
        par = usePar > 0 && license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel')) && numel(idx) > 1;
        if par
            try
                out = parsim(simIn(1:numel(idx)), 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                             'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', 'SetupFcn', setup);
            catch ME
                fprintf('parsim failed (%s); running sequentially\n', ME.message);
                par = false;
            end
        end
        if ~par
            setup();
            out = cell(1, numel(idx));
            for j = 1:numel(idx)
                try
                    out{j} = sim(simIn(j));
                catch ME                                 % keep going: the error lands in that run's summary
                    out{j} = ME.message;
                    fprintf('%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
                end
                fprintf('  run %d/%d done (%.0f s)\n', j, numel(idx), toc(t0));
            end
        else
            out = num2cell(out);
        end
        dt = toc(t0);
        fprintf('simulations done in %.0f s\n', dt);
        for j = 1:numel(idx)
            k = idx(j);
            writeRun(out{j}, pts(k, :), spec.fs, fullfile(outDir, char(pts.label(k))), dt);
        end
        clear out simIn
    end
    fprintf('f1_closed_loop: done\n');
end

function writeRun(o, row, fs, base, dt)
    err = "";
    S = nan(1, 4);
    try
        if ischar(o) || isstring(o), error('%s', o); end   % sequential run that threw
        if ~isempty(o.ErrorMessage), error('%s', o.ErrorMessage); end
        [iL, tI] = extractSignal(o, {'Inductor_Current1'});
        if isempty(iL), error('inductor current not logged'); end
        Tsw = 1 / fs;
        nP = floor(tI(end) / Tsw + 1e-9);
        bin = min(floor(tI / Tsw + 1e-9) + 1, nP + 1);
        keep = bin <= nP;
        tEnd = (1:nP)' * Tsw;
        Ipk = accumarray(bin(keep), abs(iL(keep)), [nP 1], @max, NaN);
        iLm = accumarray(bin(keep), iL(keep), [nP 1], @mean, NaN);
        P = table(tEnd, Ipk, iLm, 'VariableNames', {'t_end', 'Ipk', 'iL_mean'});
        sigs = {'P_LV', {'Power_LV'}, 'last'; 'P_HV', {'Power_HV'}, 'last'; 'V_LV', {'Voltage_Low1'}, 'mean'; ...
                'V_HV', {'Voltage_High1'}, 'mean'; ...
                'phi', {'Phi_ext_out1'}, 'mean'; 'D1', {'D1_out1'}, 'mean'; 'D2', {'D2_out1'}, 'mean'; ...
                'I_cmd', {'I_cmd_out1'}, 'mean'};
        for q = 1:size(sigs, 1)
            [x, tx] = extractSignal(o, sigs{q, 2});
            v = nan(nP, 1);
            if ~isempty(x)
                bx = floor(tx / Tsw + 1e-9) + 1;
                kx = bx <= nP;
                if strcmp(sigs{q, 3}, 'last')            % one-period moving average: its value at the period end
                    v = accumarray(bx(kx), x(kx), [nP 1], @(u) u(end), NaN);     % tx sorted -> order kept
                else
                    v = accumarray(bx(kx), x(kx), [nP 1], @mean, NaN);
                end
            end
            P.(sigs{q, 1}) = v;
        end
        writetable(P, [base '_periods.csv']);
        last = tI >= tI(end) - 1e-3;
        n1 = min(80, nP);
        S = [max(abs(iL(last))), mean(iL(last)), mean(P.P_LV(end - n1 + 1:end), 'omitnan'), mean(P.P_HV(end - n1 + 1:end), 'omitnan')];
    catch ME
        err = string(ME.message);
    end
    T = [row, array2table([S, dt], 'VariableNames', {'Ipk_last1ms', 'iL_mean_last1ms', 'P_LV_last1ms', ...
                                                     'P_HV_last1ms', 'batch_seconds'}), table(err, 'VariableNames', {'error'})];
    writetable(T, [base '_summary.csv']);
    fprintf('  %s: Ipk %.3f A, P_LV %.1f W %s\n', char(row.label), S(1), S(3), char(err));
end

function s = onoff(x)
    if x > 0, s = 'true'; else, s = 'false'; end
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

function workerSetup(spec, eraDir, thesisDir, v3Dir)
    addpath(eraDir);
    addpath(thesisDir);
    addpath(v3Dir);                                    % parsim workers: the V3 controller copy first on the path
    cd(v3Dir);                                         % ... and as the current folder
    if ~contains(which('DAB_Controller_System'), [filesep 'v3' filesep])
        error('worker: v3 controller copy is not the one in use');
    end
    assignin('base', 'C_HV', 1000e-6);
    assignin('base', 'R_ESR_HV', 0.05);
    assignin('base', 'R_ESR_LV', 0.01);
    assignin('base', 'R_batt_int', 0.15);
    assignin('base', 'Kp_ext', 0.05);
    assignin('base', 'Ki_ext', 50.0);
    assignin('base', 'Kpv', 5.0);
    assignin('base', 'Kiv', 5000);
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
