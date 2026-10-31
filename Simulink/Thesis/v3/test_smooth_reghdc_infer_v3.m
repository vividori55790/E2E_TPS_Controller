function ok = test_smooth_reghdc_infer_v3()
% test_smooth_reghdc_infer_v3 -- the copy without the persistent variable must equal the original (and so the Python model) on the test vectors at
% V1 = 380 V and on the same vectors with V1 = 361, 372.4, 387.6, 399 V. Exactly equal: the same operations in the same order.
    thesisDir = fileparts(fileparts(mfilename('fullpath')));
    T = readtable(fullfile(thesisDir, 'out', 'smooth_reghdc_testvec.csv'));
    worst = 0;
    for V1 = [380, 361, 372.4, 387.6, 399]
        for k = 1:height(T)
            [a1, a2] = smooth_reghdc_infer(T.P(k), T.V2(k), V1);
            [b1, b2] = smooth_reghdc_infer_v3(T.P(k), T.V2(k), V1);
            worst = max([worst, abs(a1 - b1), abs(a2 - b2)]);
        end
    end
    ok = worst == 0;
    verdict = {'FAIL', 'PASS'};
    fprintf('test_smooth_reghdc_infer_v3: %d vectors x 5 V1, max |dD| = %.3g -> %s\n', height(T), worst, verdict{ok + 1});
end
