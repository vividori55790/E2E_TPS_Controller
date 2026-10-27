"""
mb_vs_grid_probe.py -- R7 check (2026-10-28 00:00): R.8's preliminary R7 table compares the best run of the R6 grid
on the Python plant's true-staircase replay with M-B measured in Simulink (R.4), i.e. two different plants and two
different measurements. Here everything is on ONE plant and ONE measurement: the steady state at fixed D0
(plant.steady), Ipk at the target power by the R7 rule (rising crossing closest to D0 = 0, linear in power between
neighbouring D0 points; staircase_compare.crossing), for SPS, M-B (b3; its (D1, D2) at the target (|P|, V2)) and every
(D1, D2) of the R6 grid (11 x 11, gen_dataset_fixed.m defaults), HV->LV.
  D0 'fine'   : 0.25 deg sweep (the PI steady state up to the interpolation)
  D0 'labels' : only the dataset's 39 D0 labels (the R7 method on a dataset)
Plants: 'replay' = replay_grid.py's (Ron 52 mOhm, droop, 2.88 deg edge grid, td 100 ns); 'nogrid' = same without the
2.88 deg edge grid (MCU HRTIM-like). The Python plant is supporting evidence; Simulink decides.

    python -I -X utf8 Simulink/Thesis/mb_vs_grid_probe.py  -> out/mb_vs_grid_probe.{log,csv}
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import plant_full as pf  # noqa: E402
import transient_replay as tr  # noqa: E402
from staircase_compare import crossing  # noqa: E402

OUT = os.path.join(HERE, "out")
TARGETS = (100.0, 150.0, 200.0, 300.0, 600.0, 1000.0)
V2S = tuple(np.linspace(84.0, 109.2, 20)[[0, 8, 19]])


def sweep(plant, D0, D1, D2, V2, chunk=6000):
    """steady state on the outer product (combos x D0) -> P_LV, Ipk of shape (n_combo, n_D0)"""
    nc, nd = D1.size, D0.size
    a = np.repeat(D1, nd)
    b = np.repeat(D2, nd)
    d = np.tile(D0, nc)
    P = np.empty(a.size)
    I = np.empty(a.size)
    for s in range(0, a.size, chunk):
        sl = slice(s, s + chunk)
        r = plant.steady(d[sl], a[sl], b[sl], np.full(d[sl].size, V2))
        P[sl], I[sl] = r["P_LV"], r["Ipk"]
    return P.reshape(nc, nd), I.reshape(nc, nd)


def main():
    import pandas as pd
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    Dg = np.linspace(0.2, 0.5, 20)[list(range(0, 19, 2)) + [19]]
    G1, G2 = np.meshgrid(Dg, Dg, indexing="ij")
    g1, g2 = G1.ravel(), G2.ravel()
    isps = int(np.where((g1 == 0.5) & (g2 == 0.5))[0][0])
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    fine = np.round(np.arange(-10.0, 75.0 + 1e-9, 0.25), 4)
    lab = tr.d0_grid()
    lab = lab[(lab > -12.0) & (lab < 80.0)]
    plants = {"replay": pf.FullPlant(Ron=0.052, steps=125),
              "nogrid": pf.FullPlant(Ron=0.052, steps=125, quant=False)}
    out("HV->LV; Ipk at |P| = R7 rule on the steady state at fixed D0; dI = Ipk(SPS) - Ipk(x)")
    for pname, plant in plants.items():
        for V2 in V2S:
            t0 = time.time()
            Tp = np.array(TARGETS)
            m1, m2 = mb.predict(Tp, np.full(Tp.size, V2))
            m1, m2 = np.asarray(m1, float).ravel(), np.asarray(m2, float).ravel()
            D1 = np.concatenate([g1, m1])
            D2 = np.concatenate([g2, m2])
            res = {"fine": (fine,) + sweep(plant, fine, D1, D2, V2), "labels": (lab,) + sweep(plant, lab, D1, D2, V2)}
            out(f"\nplant {pname}, V2 {V2:.4g} V (k {4 * V2 / 380:.3f})  [{time.time() - t0:.0f} s]")
            out(f"  {'|P|':>5s} {'D0':>6s} | {'SPS':>6s} | {'M-B (D1,D2)':>14s} {'Ipk':>6s} {'dI':>6s} | "
                f"{'grid best (D1,D2)':>17s} {'Ipk':>6s} {'dI':>6s} | best - M-B")
            for kind, (D0, P, I) in res.items():
                for j, p in enumerate(TARGETS):
                    Is = crossing(D0, P[isps], I[isps], p, +1)[0]
                    Im = crossing(D0, P[g1.size + j], I[g1.size + j], p, +1)[0]
                    Ig = np.array([crossing(D0, P[c], I[c], p, +1)[0] for c in range(g1.size)])
                    Ig[isps] = np.nan
                    b = int(np.nanargmin(np.where(np.isfinite(Ig), Ig, np.nan))) if np.isfinite(Ig).any() else -1
                    Ib = Ig[b] if b >= 0 else np.nan
                    out(f"  {p:5.0f} {kind:>6s} | {Is:6.3f} | ({m1[j]:.3f},{m2[j]:.3f}) {Im:6.3f} {Is - Im:+6.3f} | "
                        f"({g1[b]:.3f},{g2[b]:.3f})   {Ib:6.3f} {Is - Ib:+6.3f} | {Im - Ib:+6.3f}")
                    rows.append(dict(plant=pname, V2=V2, P=p, D0_set=kind, Ipk_SPS=Is, MB_D1=m1[j], MB_D2=m2[j],
                                     Ipk_MB=Im, dI_MB=Is - Im, best_D1=g1[b], best_D2=g2[b], Ipk_best=Ib,
                                     dI_best=Is - Ib))
            pd.DataFrame(rows).to_csv(os.path.join(OUT, "mb_vs_grid_probe.csv"), index=False)
            open(os.path.join(OUT, "mb_vs_grid_probe.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
