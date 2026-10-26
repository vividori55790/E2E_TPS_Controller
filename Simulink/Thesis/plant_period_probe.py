"""
plant_period_probe.py -- what is the steady state where the period-1 Newton does not converge (dead time, light load,
k ~ 1; test_plant_full.py item 5)? Time-steps those points for many periods and checks period-1 / period-2 /
non-periodic behaviour: |x(n) - x(n-1)| and |x(n) - x(n-2)| over the last periods, and the spread of the per-period Ipk
and power over the last 1 ms (80 periods = the Simulink measurement window).

    python -I -X utf8 Simulink/Thesis/plant_period_probe.py
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402


def main():
    t0 = time.time()
    b = pf.FullPlant(Ron=0.040, droop=False, quant=False)
    m = 40
    D0s = np.linspace(-7.0, -1.0, m)
    d1, d2 = np.full(m, 0.4338), np.full(m, 0.4485)
    Vb1, Vb2 = np.full(m, 380.0), np.full(m, 92.66)
    r = b.steady_fixed(D0s, d1, d2, Vb1, Vb2)
    bad = np.where(~r["conv"])[0]
    print(f"{bad.size}/{m} not converged: D0 = {np.round(D0s[bad], 3)}")
    idx = np.concatenate([bad, np.where(r["conv"])[0][:3]])          # + 3 converged points as a control
    sch = [(u[:, idx], lo[:, idx]) for u, lo in b.schedule(D0s, d1, d2)]
    i, vc = r["i0"][idx].copy(), r["vc0"][idx].copy()
    n_per = 3000
    hist_i, hist_v, ipk, pw = [], [], [], []
    for k in range(n_per):
        i, vc, _, Pl, w = b.period(i, vc, sch, Vb1[idx], Vb2[idx], record=k >= n_per - 80)
        hist_i.append(i.copy())
        hist_v.append(vc.copy())
        if w is not None:
            ipk.append(np.abs(w).max(axis=0))
            pw.append(-Pl)
    H, V = np.array(hist_i), np.array(hist_v)
    ipk, pw = np.array(ipk), np.array(pw)
    print(f"time stepping {n_per} periods ({time.time() - t0:.0f} s)")
    print(f"{'D0':>7s} {'conv':>4s} | {'|di| p1':>8s} {'|di| p2':>8s} {'|dvc| p1':>8s} | Ipk Newton  last-1ms min..max   "
          f"| P Newton   last-1ms mean (min..max)")
    for j, k in enumerate(idx):
        p1 = np.abs(H[-1, j] - H[-2, j])
        p2 = np.abs(H[-1, j] - H[-3, j])
        v1 = np.abs(V[-1, j] - V[-2, j])
        print(f"{D0s[k]:7.3f} {str(bool(r['conv'][k])):>5s} | {p1:8.1e} {p2:8.1e} {v1:8.1e} | {r['Ipk'][k]:.4f}  "
              f"{ipk[:, j].min():.4f}..{ipk[:, j].max():.4f} | {-r['P_LV'][k]:8.2f}  {pw[:, j].mean():8.2f} "
              f"({pw[:, j].min():.2f}..{pw[:, j].max():.2f})")
    # drift over the last 1000 periods (slow LC mode still decaying, or a sustained oscillation?)
    amp = H[-1000:].max(axis=0) - H[-1000:].min(axis=0)
    amp2 = H[-200:].max(axis=0) - H[-200:].min(axis=0)
    print("i(0) peak-to-peak over the last 1000 / 200 periods:", np.round(amp, 4), np.round(amp2, 4))


if __name__ == "__main__":
    sys.exit(main())
