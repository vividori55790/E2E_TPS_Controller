function ok = test_smooth_reghdc_infer()
% test_smooth_reghdc_infer -- smooth_reghdc_infer.m must reproduce the Python model on out/smooth_reghdc_testvec.csv
% (export_matlab.py). Tolerance 1e-9 in D (only the summation order differs from numpy).
    thesisDir = fileparts(mfilename('fullpath'));
    T = readtable(fullfile(thesisDir, 'out', 'smooth_reghdc_testvec.csv'));
    err = zeros(height(T), 1);
    for k = 1:height(T)
        [d1, d2] = smooth_reghdc_infer(T.P(k), T.V2(k));
        err(k) = max(abs(d1 - T.D1(k)), abs(d2 - T.D2(k)));
    end
    ok = max(err) < 1e-9;
    verdict = {'FAIL', 'PASS'};
    fprintf('test_smooth_reghdc_infer: %d vectors, max |dD| = %.3g -> %s\n', height(T), max(err), verdict{ok + 1});
end
