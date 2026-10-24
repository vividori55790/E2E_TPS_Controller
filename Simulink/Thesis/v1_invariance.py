"""
v1_invariance.py -- T14 (reduced): what happens when the HV link V1 is not exactly 380 V?

Scaling of the ideal DAB: i_L = V1/(wL) * F(theta; k, D0, D1, D2) with k = n V2 / V1, and P = V1^2/(wL) * G(k, D).
So the optimal (D1, D2) at (|P|, V2, V1) equal those at V1 = 380 V for the equivalent point
    V2' = V2 * 380 / V1,   |P|' = |P| * (380 / V1)^2,
and Ipk scales by V1/380. A third input therefore collapses in the ideal model: a 2-input controller fed the
physics-normalised inputs (|P|', V2') is V1-invariant at zero cost, while the same controller fed the raw
(|P|, V2) puts the k = 1 ridge at the wrong V2. (Non-idealities -- dead time, R -- break the scaling slightly; that
is the only place a true 3-input model could earn its memory.)

Check numerically on random points with V1 ~ U[361, 399] (+-5 %): exact scaling identity on random D, then
SPS vs b3 raw inputs vs b3 normalised inputs (PI steady state, exact model with V1).
Output: out/v1_invariance.log, out/v1_invariance.csv
"""
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")


def load_b3():
    d = json.load(open(os.path.join(OUT, "model_smooth_reghdc_b3.json"), encoding="utf-8"))
    m = ct.SmoothRegHDC(D=d["D"], sigma=d["sigma"], tau=d["tau"], name=d["name"])
    m.W, m.b, m.M, m.c = (np.asarray(d[k], float) for k in ("W", "b", "M", "c"))
    return m


def stats(dI, ok):
    f = dI[ok]
    return dict(mean=f.mean(), worst=f.min(), deg5=np.mean(f < -0.005) * 100, deg_any=np.mean(f < -1e-4) * 100,
                infeasible=int((~ok).sum()))


def main():
    lines = []

    def out(s=""):
        print(s, flush=True)
        lines.append(s)

    rng = np.random.default_rng(14)
    # 1. the scaling identity on random (D0, D1, D2)
    n = 2000
    V1 = rng.uniform(361, 399, n)
    V2 = rng.uniform(84, 109.2, n)
    D0, D1, D2 = rng.uniform(-60, 60, n), rng.uniform(0.1, 0.5, n), rng.uniform(0.1, 0.5, n)
    a = tm.evaluate(D0, D1, D2, V2, V1)
    b = tm.evaluate(D0, D1, D2, V2 * tm.V1_NOM / V1, tm.V1_NOM)
    s = V1 / tm.V1_NOM
    eI = np.abs(a["Ipk"] - b["Ipk"] * s).max()
    eP = np.abs(a["P"] - b["P"] * s ** 2).max()
    out(f"scaling identity on {n} random (D0, D1, D2, V2, V1): max |Ipk - s*Ipk'| = {eI:.2e} A, "
        f"max |P - s^2*P'| = {eP:.2e} W  (s = V1/380)")

    # 2. controllers under V1 = U[361, 399]
    n = 6000
    P = rng.uniform(0, ct.P_MAX, n)
    V2 = rng.uniform(ct.V2_LO, ct.V2_HI, n)
    V1 = rng.uniform(361, 399, n)
    sps = ps.sps_reference(P, V2, V1)
    m = load_b3()
    rows = []
    d1r, d2r = m.predict(P, V2)                                       # raw inputs (ignores V1)
    s = tm.V1_NOM / V1
    Pn, V2n = P * s ** 2, V2 * s
    d1n, d2n = m.predict(Pn, V2n)                                     # physics-normalised inputs
    inside = (Pn <= ct.P_MAX) & (V2n >= ct.V2_LO) & (V2n <= ct.V2_HI)
    out(f"\n{n} random points, |P| U[0, 1200] W, V2 U[84, 109.2] V, V1 U[361, 399] V; normalised point inside the "
        f"trained box for {inside.mean() * 100:.1f} % (outside -> input clipped)")
    out(f"{'controller':34s} {'mean dI [A]':>11s} {'worst [mA]':>11s} {'>5 mA worse':>12s} {'>0.1 mA':>8s} {'infeas':>7s}")
    for name, d1, d2 in (("b3, raw (|P|, V2)", d1r, d2r), ("b3, normalised (|P|', V2')", d1n, d2n)):
        r = ps.operating_point(P, d1, d2, V2, V1)
        dI = sps["Ipk"] - r["Ipk"]
        for sub, msk in (("all", np.ones(n, bool)), ("inside", inside), ("outside", ~inside)):
            st = stats(dI[msk], r["ok"][msk])
            out(f"{name + ' / ' + sub:34s} {st['mean']:11.4f} {st['worst'] * 1e3:11.1f} {st['deg5']:11.2f}% "
                f"{st['deg_any']:7.2f}% {st['infeasible']:7d}")
            rows.append(dict(controller=name, subset=sub, **{k: float(v) for k, v in st.items()}))
    # where does the raw-input controller lose? near k = 1 of the actual V1
    r = ps.operating_point(P, d1r, d2r, V2, V1)
    dI = sps["Ipk"] - r["Ipk"]
    bad = r["ok"] & (dI < -0.005)
    if bad.any():
        k = tm.N_TURNS * V2 / V1
        out(f"\nraw-input b3, points > 5 mA worse than SPS: {bad.sum()}; k range {k[bad].min():.3f}-{k[bad].max():.3f}, "
            f"|P| {P[bad].min():.0f}-{P[bad].max():.0f} W, V1 {V1[bad].min():.0f}-{V1[bad].max():.0f} V; "
            f"worst {dI[bad].min() * 1e3:.0f} mA at |P| {P[np.argmin(np.where(bad, dI, 0))]:.0f} W")
    with open(os.path.join(OUT, "v1_invariance.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    open(os.path.join(OUT, "v1_invariance.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
