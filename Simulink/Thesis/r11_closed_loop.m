function r11_closed_loop(pointsCsv, outDir, batch)
% r11_closed_loop.m -- R11: the dataset-era model as a REAL closed loop (outer phase D0 by the controller's own PI,
% D1 / D2 = the regression controller's output fed as time series), with the PWM edge grid made finer.
%
% Why: R.7 showed that the stock PI mode cannot be judged in this model -- the controller and the Simscape local solver
% both run at 100 ns, so every edge sits on a 2.88 deg grid, the PI dithers between two grid levels inside a switching
% period and the dither leaves DC offsets and +1.2..3 A peaks whatever the gains. Here, per run and without saving the
% model:
%   * r11/DAB_Controller_System.m (a copy; first on the path AND the current folder, client and workers) sizes its
%     one-period averages from Ts_ctrl and can latch
%     the leg phases once per period (SyncPhaseUpdate, like an MCU timer's preloaded compare registers);
%   * 'MATLAB System' Ts_ctrl and Solver Configuration2 LocalSolverSampleTime = the points' Ts (e.g. 25 or 10 ns);
%   * Constant21 (Mode_Select) = 0 -> PI on the outer phase, Constant4 / Constant6 = Kp / Ki;
%   * Ref_Power, D1_ext_in / D2_ext_in (and D1 / D2), Ref_VL = the profile CSV (columns t, P_ref, D1, D2, V2);
%   * MATLAB System1 (Direct_Efficiency_Analyzer, Power_LV / Power_HV) stays at 100 ns: its buffer holds <= 500 samples;
%   * optional (trim = 1): To Workspace blocks not needed here are commented out, to keep the outputs small.
% Points CSV columns: label, ctrl, V2, Ts, sync, Kp, Ki, simTime, profile (path relative to Simulink/Thesis), trim.
% Outputs per run (restart-safe: a run whose summary exists is skipped):
%   <outDir>/<label>_periods.csv  one row per switching period: t_end, Ipk (max|i_L|), iL_mean, P_LV, P_HV (the
%                                 analyzer's one-period averages at the period end), V_LV, phi, D1, D2, I_cmd (means)
%   <outDir>/<label>_summary.csv  the point row + last-1-ms Ipk, iL_mean, P_LV, P_HV, run time, error
%
%   python -I -X utf8 Simulink/Thesis/r11_step.py smoke       -> out/r11_points_smoke.csv + profiles
%   python automation/job.py submit r11smoke --interactive --after eff2 -- matlab -batch "cd('Simulink/Thesis'); r11_closed_loop('out/r11_points_smoke.csv', 'out/r11')"
%   python -I -X utf8 Simulink/Thesis/r11_step.py compare
% Read-only use of the original model and controller: nothing is saved into dataset_era/ or Modules/.

    thesisDir = fileparts(mfilename('fullpath'));
    simDir    = fileparts(thesisDir);
    eraDir    = fullfile(thesisDir, 'dataset_era');
    r11Dir    = fullfile(thesisDir, 'r11');
    if nargin < 2 || isempty(outDir), outDir = fullfile(thesisDir, 'out', 'r11'); end
    if nargin < 3 || isempty(batch),  batch  = 6; end
    pointsCsv = absPath(char(pointsCsv)); outDir = absPath(char(outDir));
    if ~isfolder(outDir), mkdir(outDir); end
    fprintf('r11_closed_loop: %s -> %s\n', pointsCsv, outDir);

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
    addpath(r11Dir);                                   % last addpath = first on the path: the R11 controller copy
    % keep the dataset-era cache (dataset_era/slprj, used by every other job) untouched by the modified class
    cacheDir = fullfile(thesisDir, 'out', 'r11_cache');
    if ~isfolder(cacheDir), mkdir(cacheDir); end
    oldCache = Simulink.fileGenControl('get', 'CacheFolder');
    Simulink.fileGenControl('set', 'CacheFolder', cacheDir);
    modelName = 'Control_Logic_dataset_era';
    oldDir = pwd;
    cleanup = onCleanup(@() restore(oldDir, oldCache));
    % NOT cd(eraDir): the current folder comes before the path, so dataset_era/ made the original class shadow the
    % copy (r11smoke 10-30 12:51, all 5 runs: "... 'SyncPhaseUpdate'인 파라미터가 없음"); the model and
    % Direct_Efficiency_Analyzer.m are found on the path, every file name here is absolute
    cd(r11Dir);
    w = which('DAB_Controller_System');
    fprintf('DAB_Controller_System -> %s\n', w);
    if ~contains(w, [filesep 'r11' filesep]), error('r11 controller copy is not the one in use'); end
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
            s = s.setBlockParameter(solverBlk, 'LocalSolverSampleTime', num2str(pts.Ts(k), 10));
            if pts.trim(k) > 0
                for q = 1:numel(trimBlocks)
                    s = s.setBlockParameter([modelName '/' trimBlocks{q}], 'Commented', 'on');
                end
            end
            simIn(j) = s;
        end
        fprintf('batch %d-%d of %d (%s)\n', b0, b0 + numel(idx) - 1, numel(todo), char(datetime('now')));
        setup = @() workerSetup(spec, eraDir, r11Dir, cacheDir);
        t0 = tic;
        usePar = license('test', 'Distrib_Computing_Toolbox') && ~isempty(ver('parallel')) && numel(idx) > 1;
        if usePar
            try
                out = parsim(simIn(1:numel(idx)), 'UseFastRestart', 'off', 'ShowProgress', 'on', ...
                             'TransferBaseWorkspaceVariables', 'on', 'ShowSimulationManager', 'off', 'SetupFcn', setup);
            catch ME
                fprintf('parsim failed (%s); running sequentially\n', ME.message);
                usePar = false;
            end
        end
        if ~usePar
            setup();
            out = cell(1, numel(idx));
            for j = 1:numel(idx)
                try
                    out{j} = sim(simIn(j));
                catch ME                                 % keep going: the error lands in that run's summary
                    out{j} = ME.message;
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
    fprintf('r11_closed_loop: done\n');
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
        S = [max(abs(iL(last))), mean(iL(last)), mean(P.P_LV(end - 79:end), 'omitnan'), mean(P.P_HV(end - 79:end), 'omitnan')];
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

function workerSetup(spec, eraDir, r11Dir, cacheDir) %#ok<INUSD>
    addpath(eraDir);
    addpath(r11Dir);                                   % parsim workers: the R11 controller copy first on the path
    cd(r11Dir);                                        % ... and as the current folder (see the cd note above)
    if ~contains(which('DAB_Controller_System'), [filesep 'r11' filesep])
        error('worker: r11 controller copy is not the one in use');
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
