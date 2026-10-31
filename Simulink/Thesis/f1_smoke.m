function f1_smoke(part)
% f1_smoke.m -- (2026-10-31) can the three F1 runners run at all? One by one, beside the running queue, own cache folder out/f1_cache, short runs,
% NOT results: (a) the MATLAB test of the regression function; (b) verify_steady_eff, scope logging + solver step apart from Ts_ctrl; (c) f1_closed_loop,
% the regression inside the controller. Output under out/f1_smoke/. part 'b' or 'c' alone (one MATLAB session cannot hold both controller classes).
    if nargin < 1, part = 'bc'; end
    here = fileparts(mfilename('fullpath'));
    addpath(here);
    stamp = char(datetime('now', 'Format', 'HHmmss'));
    od = fullfile(here, 'out', 'f1_smoke', stamp);
    if ~isfolder(od), mkdir(od); end
    t0 = tic;
    try
        fprintf('== (a) test_smooth_reghdc_infer (%.0f s)\n', toc(t0));
        test_smooth_reghdc_infer();
        addpath(fullfile(here, 'v3'));
        test_smooth_reghdc_infer_v3();
    catch ME
        fprintf('(a) FAILED: %s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    end
    if contains(part, 'b')
    try
        fprintf('== (b) verify_steady_eff (%.0f s)\n', toc(t0));
        T = table(["s1 SPS Ts25"; "s2 SPS ctrl25 solver6"], ["SPS"; "SPS"], [84; 84], [100; 100], [6; 6], ...
                  [4.32 - 1e-4; 4.32 - 1e-4], [0.4999999; 0.4999999], [0.4999999; 0.4999999], [25e-9; 25e-9], [25e-9; 6.25e-9], ...
                  'VariableNames', {'label', 'ctrl', 'V2', 'P_target', 'level', 'D0', 'D1', 'D2', 'Ts', 'Ts_solver'});
        pin = fullfile(od, 'eff_in.csv');
        writetable(T, pin);
        verify_steady_eff(pin, fullfile(od, 'eff_out.csv'), 0.002, 0, 'f1_cache');
    catch ME
        fprintf('(b) FAILED: %s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    end
    end
    cd(here);
    if contains(part, 'c')
    try
        fprintf('== (c) f1_closed_loop (%.0f s)\n', toc(t0));
        t = (0:12.5e-6:0.002)';
        P = table(t, -300 * ones(size(t)), 0.4999999 * ones(size(t)), 0.4999999 * ones(size(t)), 109.2 * ones(size(t)), 'VariableNames', {'t', 'P_ref', 'D1', 'D2', 'V2'});
        prof = fullfile(od, 'prof.csv');
        writetable(P, prof);
        T = table(["c1 reg hold1"; "c2 reg hold8"], ["M-B"; "M-B"], [109.2; 109.2], [25e-9; 25e-9], [1; 1], [1; 1], [24500; 24500], [0.002; 0.002], ...
                  repmat(string(['out/f1_smoke/' stamp '/prof.csv']), 2, 1), [1; 1], [1; 1], [1; 8], ...
                  'VariableNames', {'label', 'ctrl', 'V2', 'Ts', 'sync', 'Kp', 'Ki', 'simTime', 'profile', 'trim', 'reg', 'hold'});
        pin = fullfile(od, 'loop_in.csv');
        writetable(T, pin);
        f1_closed_loop(pin, fullfile(od, 'loop'), 4, 0, 'f1_cache2');
    catch ME
        fprintf('(c) FAILED: %s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
    end
    end
    fprintf('f1_smoke: done (%.0f s)\n', toc(t0));
end
