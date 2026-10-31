function ok = test_analyzer_fix()
% test_analyzer_fix -- Direct_Efficiency_Analyzer_fixed on constant inputs, both directions, against out / in; the original block's I/O ratio on the same numbers for comparison. Not a model run.
    cases = [380, -2.70, 96, -10.00;     % HV -> LV: P_HV -1026 W, P_LV -960 W
             380,  2.40, 96,  10.00];    % LV -> HV: P_LV 960 W, P_HV 912 W
    ok = true;
    for k = 1:size(cases, 1)
        a = Direct_Efficiency_Analyzer_fixed();
        c = cases(k, :);
        for s = 1:130
            e = a(c(1), c(2), c(3), c(4));
        end
        PH = c(1) * c(2); PL = c(3) * c(4);
        want = 100 * min(abs(PH), abs(PL)) / max(abs(PH), abs(PL));
        if PH >= 0, orig = 100 * abs(PL) / PH; else, orig = 100 * abs(PH) / abs(PL); end
        ok = ok && abs(e - want) < 1e-9;
        fprintf('case %d: P_HV %.0f W, P_LV %.0f W: fixed %.3f %%, out / in %.3f %%, the original block''s I/O ratio %.3f %% (above 100 %% it falls through to the loss estimate)\n', k, PH, PL, e, want, orig);
    end
    verdict = {'FAIL', 'PASS'};
    fprintf('test_analyzer_fix: %s\n', verdict{ok + 1});
end
