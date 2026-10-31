function [D1, D2] = smooth_reghdc_infer_v3(P_ref, V2, V1)
% smooth_reghdc_infer_v3 -- ../smooth_reghdc_infer.m line for line, without its persistent variable: "Simulink does not support persistent variables
% in System objects inside MATLAB System blocks" (f1smokec, 2026-10-31). The parameters are a compile-time constant here. Checked against the
% original on out/smooth_reghdc_testvec.csv and on V1 != 380 V by test_smooth_reghdc_infer_v3.m.
%#codegen
    p = coder.const(smooth_reghdc_params());
    s = 380 / max(V1, 1);
    u = min(max(abs(P_ref) * s^2 / p.P_MAX, 0), 1);
    v = min(max((V2 * s - p.V2_LO) / (p.V2_HI - p.V2_LO), 0), 1);
    h = sqrt(2) * cos([u, v] * p.W + p.b);          % 1 x D random-Fourier (HDC continuous) encoding
    y = h * p.M / sqrt(p.D) + p.c;                  % 1 x 2 linear read-out
    t = y / p.tau;                                  % softplus_tau, overflow-safe: tau*(max(t,0) + log1p(exp(-|t|)))
    sp = p.tau * (max(t, 0) + log1p(exp(-abs(t))));
    delta = min(sp, 0.5 - p.D_MIN);
    D1 = 0.5 - delta(1);
    D2 = 0.5 - delta(2);
end
