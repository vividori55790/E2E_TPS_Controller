"""
k1_light_probe.py -- R7 check (2026-10-27 23:50): is the +0.46 A "TPS gain" at k ~ 1, 100 W in the Python-plant
true-staircase replay (dataset_fixed_compare_py_zoh.log, V2 94.61 V HV->LV: SPS 0.89 A, TPS (0.326, 0.326) 0.43 A)
real, or an artifact of the R7 method (Ipk interpolated linearly in power between the dataset's D0 labels, 2.4 deg
apart near 0)? At k ~ 1 light load the dead time makes P(D0) strongly nonlinear (plateau where the switching current
is ~0), so a straight line between two labels can sit far above the true Ipk(P) curve.

Same plant as replay_grid.py (Ron 52 mOhm, droop, 2.88 deg edge grid, td 100 ns), steady state at fixed D0
(plant.steady), on (i) a fine D0 sweep (0.05 deg) and (ii) only the dataset's D0 labels; Ipk at the target power by
staircase_compare.crossing on each. Also without the 2.88 deg grid (quant off) and without dead time (td 0).

    python -I -X utf8 Simulink/Thesis/k1_light_probe.py  -> out/k1_light_probe.{log,csv}
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import transient_replay as tr  # noqa: E402
from staircase_compare import crossing  # noqa: E402

OUT = os.path.join(HERE, "out")
TARGETS = (100.0, 150.0, 200.0, 300.0, 600.0)
COMBOS = ((0.5, 0.5), (0.4526, 0.4526), (0.3895, 0.3895), (0.3263, 0.3263), (0.2632, 0.2632))


def sweep(plant, D0, D1, D2, V2):
    r = plant.steady(D0, np.full_like(D0, D1), np.full_like(D0, D2), np.full_like(D0, V2))
    return r["P_LV"], r["Ipk"], r["cycle"]


def main():
    import pandas as pd
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    fine = np.round(np.arange(-8.0, 30.0 + 1e-9, 0.05), 4)
    lab = tr.d0_grid()
    lab = lab[(lab > -8.5) & (lab < 30.5)]
    # steps 125 (the 100 ns sample grid, as replay_grid.py); a 1250-step run was stopped after 10 min (relax-heavy)
    plants = {"replay (quant, td 100 ns)": pf.FullPlant(Ron=0.052, steps=125),
              "no 2.88 grid (quant off)": pf.FullPlant(Ron=0.052, steps=125, quant=False),
              "no dead time (td 0, quant off)": pf.FullPlant(Ron=0.052, steps=125, quant=False, td=0.0)}
    out(f"D0 labels used (dataset grid, -8..30 deg): {', '.join(f'{x:.2f}' for x in lab)}")
    out("HV->LV (D0 > 0, P_LV < 0); Ipk at |P| by the rising crossing closest to D0 = 0, linear in power between "
        "neighbouring D0 points; dI = Ipk(SPS) - Ipk(TPS)")
    for V2 in (94.61, 95.0):
        for name, plant in plants.items():
            t0 = time.time()
            res = {}
            for c in COMBOS:
                Pf, If, cf = sweep(plant, fine, c[0], c[1], V2)
                Pl, Il, _ = sweep(plant, lab, c[0], c[1], V2)
                res[c] = (Pf, If, cf, Pl, Il)
            out(f"\nV2 {V2} V (k {4 * V2 / 380:.3f}), plant {name}  [{time.time() - t0:.0f} s]")
            out(f"  {'(D1, D2)':>14s} | " + " | ".join(f"{p:>5.0f} W fine / labels" for p in TARGETS))
            for c in COMBOS:
                Pf, If, cf, Pl, Il = res[c]
                cells = []
                for p in TARGETS:
                    a = crossing(fine, Pf, If, p, +1)
                    b = crossing(lab, Pl, Il, p, +1)
                    cells.append(f"{a[0]:6.3f} / {b[0]:6.3f}")
                    rows.append(dict(V2=V2, plant=name, D1=c[0], D2=c[1], P=p, Ipk_fine=a[0], D0_fine=a[1],
                                     Ipk_labels=b[0], D0_labels=b[1]))
                ncyc = int(cf.sum())
                out(f"  ({c[0]:.3f},{c[1]:.3f}) | " + " | ".join(cells) + (f"   [{ncyc} fine pts no period-1]" if ncyc else ""))
            sps = res[(0.5, 0.5)]
            for p in TARGETS:
                s_f = crossing(fine, sps[0], sps[1], p, +1)[0]
                s_l = crossing(lab, sps[3], sps[4], p, +1)[0]
                bf = min((crossing(fine, res[c][0], res[c][1], p, +1)[0], c) for c in COMBOS[1:])
                bl = min((crossing(lab, res[c][3], res[c][4], p, +1)[0], c) for c in COMBOS[1:])
                out(f"  {p:5.0f} W: best-TPS dI fine {s_f - bf[0]:+.3f} A at {bf[1]} | labels {s_l - bl[0]:+.3f} A at {bl[1]}")
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "k1_light_probe.csv"), index=False)
    # the SPS / (0.326, 0.326) P(D0), Ipk(D0) curves of the replay plant at 94.61 V, for the record
    plant = plants["replay (quant, td 100 ns)"]
    out("\nreplay plant, V2 94.61 V: P_LV(D0) and Ipk(D0) on the dataset labels")
    for c in ((0.5, 0.5), (0.3263, 0.3263)):
        Pl, Il, _ = sweep(plant, lab, c[0], c[1], 94.61)
        out(f"  {c}: " + "  ".join(f"[{d:+.1f}: {-p:6.1f} W {i:.3f} A]" for d, p, i in zip(lab, Pl, Il) if -2 < d < 16))
    open(os.path.join(OUT, "k1_light_probe.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
