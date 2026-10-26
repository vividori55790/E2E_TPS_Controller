"""
t7_outliers.py -- the points where a T7 model is > 20 mA worse than SPS on some plant variant (plant_verdict_B_*_k1.npz):
(D1, D2), the ideal-model prediction, the plant dI per variant, and the plant's operating point (D0 branch) for SPS and
for the controller, to tell dead-time noise (+-20 mA, t7_probe.py) from a branch / first-crossing jump.

    python -I -X utf8 Simulink/Thesis/t7_outliers.py [--region k1] [--models b8,b9,b10] [--thr 0.02]
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import plant_verdict as pv  # noqa: E402

OUT = os.path.join(HERE, "out")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--region", default="k1")
    ap.add_argument("--models", default="b8,b9,b10")
    ap.add_argument("--thr", type=float, default=0.02)
    a = ap.parse_args()
    runs = {v: np.load(os.path.join(OUT, f"plant_verdict_B_{v}_{a.region}.npz")) for v in pv.VARIANTS}
    P, V = runs["A"]["P"], runs["A"]["V"]
    names = a.models.split(",")
    ctrls = dict(zip(names, pv.load_models(names)))
    plants = {v: pf.FullPlant(quant=False, **kw) for v, kw in pv.VARIANTS.items()}
    for tag, c in ctrls.items():
        key = c.name.replace(" ", "_")
        d1, d2 = (np.asarray(x, float).ravel() for x in c.predict(P, V))
        dI = np.stack([runs[v]["SPS"][0] - runs[v][key][0] for v in pv.VARIANTS])
        dIi = runs["A"]["SPS"][1] - runs["A"][key][1]
        idx = np.where(np.nanmin(dI, axis=0) < -a.thr)[0]
        print(f"== {tag}: {idx.size} points > {a.thr * 1e3:.0f} mA worse on some variant")
        for i in idx[np.argsort(np.nanmin(dI[:, idx], axis=0))]:
            print(f"  |P| {P[i]:6.1f} W V2 {V[i]:6.2f} V  D1 {d1[i]:.4f} D2 {d2[i]:.4f} (0.5-min {0.5 - min(d1[i], d2[i]):.4f}) "
                  f"ideal dI {dIi[i] * 1e3:6.1f} mA | plant dI " + " ".join(f"{v} {x * 1e3:7.1f}" for v, x in
                                                                         zip(pv.VARIANTS, dI[:, i])))
            for v in pv.VARIANTS:
                pl = plants[v]
                line = f"      {v}:"
                for lab, x1, x2 in (("SPS", 0.5, 0.5), ("ctrl", d1[i], d2[i])):
                    one = lambda x: np.array([x], float)           # noqa: E731
                    r = pl.operating_point(one(P[i]), one(x1), one(x2), one(V[i]))
                    # P(D0) scan over the bisection window: is the crossing unique (monotone) or not?
                    D0s = float(np.nan_to_num(r["D0_ideal"][0], nan=60.0)) + np.linspace(-15.0, 15.0, 61)
                    n_ = D0s.size
                    Vb = np.full(n_, float(r["V2_bus"][0]))
                    s = pl.steady_fixed(D0s, np.full(n_, x1), np.full(n_, x2), np.full(n_, float(r["V1_bus"][0])), Vb)
                    p = -s["P_LV"]
                    cross = np.where(np.diff(np.sign(p - P[i])) != 0)[0]
                    first = D0s[cross[0]] if cross.size else np.nan
                    line += (f" {lab} Ipk {float(r['Ipk'][0]):.4f} D0 {float(r['D0'][0]):7.3f} (ideal "
                             f"{float(r['D0_ideal'][0]):7.3f}, crossings {cross.size}, first ~{first:7.2f}, "
                             f"P non-monotone {int(np.sum(np.diff(p) < 0))})")
                print(line)


if __name__ == "__main__":
    sys.exit(main())
