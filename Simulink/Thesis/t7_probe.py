"""
t7_probe.py -- shape of the plant's Ipk near SPS at k ~ 1 (is the b3 degradation a jump at D = 0.5, a steep slope,
or a solver artefact?). Plant = plant_verdict's FullPlant(Ron 52 mOhm, quant=False); variants without dead time / Ron.

    python -I -X utf8 Simulink/Thesis/t7_probe.py        (about 6 min; out/t7_probe.log)
Columns: Ipk(Delta) - Ipk(SPS) [mA] (positive = worse than SPS) and the PI steady-state D0 [deg] per plant variant.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402

_log = open(os.path.join(HERE, "out", "t7_probe.log"), "w", encoding="utf-8")


def print(*a):                                                 # noqa: A001 -- tee to out/t7_probe.log
    s = " ".join(str(x) for x in a)
    sys.stdout.write(s + "\n")
    sys.stdout.flush()
    _log.write(s + "\n")
    _log.flush()


deltas = np.array([0.0, 1e-4, 5e-4, 1e-3, 2e-3, 4e-3, 8e-3, 0.015, 0.025, 0.04, 0.06])
cases = [(190.9, 93.30, "D1"), (37.3, 94.38, "both"), (84.6, 96.58, "both"), (207.4, 97.46, "D2"), (400.0, 84.0, "both")]
plants = [("Ron52 td100", pf.FullPlant(Ron=0.052, quant=False)),
          ("Ron52 td0", pf.FullPlant(Ron=0.052, quant=False, td=0.0)),
          ("Ron0 td100 nodroop", pf.FullPlant(Ron=0.0, quant=False, droop=False))]
for P, V2, which in cases:
    d1 = 0.5 - deltas * (which in ("D1", "both"))
    d2 = 0.5 - deltas * (which in ("D2", "both"))
    print(f"\n|P| {P} W, V2 {V2} V (k {4 * V2 / 380:.3f}), shrink {which}")
    print(f"{'delta':>7s} " + " ".join(f"{name:>22s}" for name, _ in plants) + f" {'ideal':>8s}")
    res = []
    for name, pl in plants:
        r = pl.operating_point(np.full(deltas.size, P), d1, d2, np.full(deltas.size, V2))
        res.append(r)
    for j, dl in enumerate(deltas):
        cells = " ".join(f"{(r['Ipk'][j] - r['Ipk'][0]) * 1e3:+9.2f}m D0 {r['D0'][j]:6.2f}" for r in res)
        print(f"{dl:7.4f} {cells} {(res[0]['Ipk_ideal'][j] - res[0]['Ipk_ideal'][0]) * 1e3:+7.2f}m")
    print("SPS Ipk: " + ", ".join(f"{name} {r['Ipk'][0]:.4f}" for (name, _), r in zip(plants, res))
          + f", ideal {res[0]['Ipk_ideal'][0]:.4f}")
