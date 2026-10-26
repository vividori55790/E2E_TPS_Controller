"""
plant_attractor_probe.py -- where FullPlant.steady_fixed (Newton + polish + relax) and a cold-start time-stepping run
disagree (test_plant_full.py item 5): bistability (a stable period-1 orbit next to a sustained oscillation), or not
enough settling? Per point: the steady_fixed answer (period-1 / time-stepped, spectral radius), cold start after 600
and after 1500 periods, and the steady_fixed state continued for 1500 periods.

    python -I -X utf8 Simulink/Thesis/plant_attractor_probe.py
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
    V1, V2 = np.full(m, 380.0), np.full(m, 92.66)
    sch = b.schedule(D0s, d1, d2)
    new = b.steady_fixed(D0s, d1, d2, V1, V2)
    rad = b._map_radius(new["i0"], new["vc0"], sch, V1, V2)
    s600 = b._relax(np.zeros(m), np.zeros(m), sch, V1, V2, 600, 80)
    s1500 = b._relax(s600[0], s600[1], sch, V1, V2, 820, 80)            # = cold start after 1500 periods
    cont = b._relax(new["i0"], new["vc0"], sch, V1, V2, 1420, 80)        # steady_fixed state continued
    print(f"({time.time() - t0:.0f} s)  Ipk [A] / P [W]:  steady_fixed | cold 600 | cold 1500 | steady_fixed state + 1500")
    for k in range(m):
        flag = "p1" if new["conv"][k] else ("cyc" if new["cycle"][k] else "?")
        d = max(abs(new["Ipk"][k] - s1500[2][k]), abs(cont[2][k] - s1500[2][k]))
        print(f"D0 {D0s[k]:7.3f} {flag:>3s} rho {rad[k]:.3f} | {new['Ipk'][k]:.4f} {-new['P_LV'][k]:7.2f} | {s600[2][k]:.4f} "
              f"{-s600[4][k]:7.2f} | {s1500[2][k]:.4f} {-s1500[4][k]:7.2f} | {cont[2][k]:.4f} {-cont[4][k]:7.2f}"
              f"{'   <-- differs' if d > 2e-3 else ''}")


if __name__ == "__main__":
    sys.exit(main())
