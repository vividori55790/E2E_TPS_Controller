"""
plant_conv_probe.py -- is plant_full.FullPlant.steady_fixed converged where t7_outliers.py found jumps?

For a few (D0 scan x point x variant) cases: Newton (8 iterations = the default, warm/cold) vs a reference made by
plain time stepping (many periods from the Newton result; the LC mode decays with tau = 2L/R ~ 100 periods) ->
residuals |i(T) - i(0)|, |vc(T) - vc(0)|, and the P_LV / Ipk error of the Newton answer.

    python -I -X utf8 Simulink/Thesis/plant_conv_probe.py
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import plant_verdict as pv  # noqa: E402

CASES = [  # (variant, |P|, V2, D1, D2) from out/t7_outliers.log
    ("C", 33.9, 92.63, 0.4219, 0.4396),
    ("B", 418.0, 100.90, 0.5, 0.5),
    ("A", 28.6, 92.66, 0.4338, 0.4485),
    ("B", 122.5, 91.17, 0.4582, 0.4712),
]


def relax(pl, D0, D1, D2, V1b, V2b, x0, periods):
    sch = pl.schedule(D0, D1, D2)
    i, vc = x0
    for _ in range(periods):
        i, vc, _, _, _ = pl.period(i, vc, sch, V1b, V2b)
    return i, vc


def main():
    t0 = time.time()
    for var, P, V2, d1, d2 in CASES:
        pl = pf.FullPlant(quant=False, **pv.VARIANTS[var])
        one = lambda x: np.array([x], float)                    # noqa: E731
        op = pl.operating_point(one(P), one(d1), one(d2), one(V2))
        D0c = float(op["D0"][0])
        D0s = D0c + np.linspace(-3.0, 3.0, 25)
        n = D0s.size
        V1b, V2b = np.full(n, float(op["V1_bus"][0])), np.full(n, float(op["V2_bus"][0]))
        D1, D2 = np.full(n, d1), np.full(n, d2)
        r8 = pl.steady_fixed(D0s, D1, D2, V1b, V2b)
        r40 = pl.steady_fixed(D0s, D1, D2, V1b, V2b, iters=40)
        # reference: time stepping from the 40-iteration Newton point; check that it does not drift
        i_ref, vc_ref = relax(pl, D0s, D1, D2, V1b, V2b, (r40["i0"], r40["vc0"]), 600)
        ref = pl.steady_fixed(D0s, D1, D2, V1b, V2b, iters=0, x0=(i_ref, vc_ref))
        print(f"== variant {var} |P| {P} W V2 {V2} V D1 {d1} D2 {d2}: operating_point D0 {D0c:.3f} Ipk "
              f"{float(op['Ipk'][0]):.4f} (ideal D0 {float(op['D0_ideal'][0]):.3f})")
        print(f"   {'D0':>7s} | {'P8':>8s} {'Ipk8':>7s} {'res_i8':>8s} | {'P40':>8s} {'Ipk40':>7s} {'res_i40':>8s} | "
              f"{'Pref':>8s} {'Ipkref':>7s} {'drift_i':>8s} {'drift_vc':>8s}")
        for k in range(n):
            print(f"   {D0s[k]:7.3f} | {-r8['P_LV'][k]:8.2f} {r8['Ipk'][k]:7.4f} {r8['resid'][k]:8.1e} | "
                  f"{-r40['P_LV'][k]:8.2f} {r40['Ipk'][k]:7.4f} {r40['resid'][k]:8.1e} | {-ref['P_LV'][k]:8.2f} "
                  f"{ref['Ipk'][k]:7.4f} {abs(i_ref[k] - r40['i0'][k]):8.1e} {abs(vc_ref[k] - r40['vc0'][k]):8.1e}")
        print(f"   ({time.time() - t0:.0f} s)", flush=True)


if __name__ == "__main__":
    sys.exit(main())
