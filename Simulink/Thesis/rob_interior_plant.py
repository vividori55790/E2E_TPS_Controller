"""
rob_interior_plant.py -- (2026-10-31 11:16) [참고: Python plant on the 25 ns grid, NOT Simulink] does L +-10 % or td 200 ns push M-B above SPS at
an interior V2?

R10 ran its five variants at V2 84 and 109.2 V only. v1x (10-31 10:58) showed what that hid for V1 +-5 %: 12 of 18 cases near k = 1 break, with V1 moving k.
L and td do not move k, so the same mechanism is not expected, but they have not been run at an interior V2 either. This scan only decides whether a
Simulink job is worth queueing. It is calibrated on the V1 +-5 % cases of v1x, whose Simulink answer is known: if the plant finds those it can see this kind of break.

    python -I -X utf8 Simulink/Thesis/rob_interior_plant.py   -> out/rob_interior_plant.{csv,log}
"""
import os
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # runs beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import fine_grid_check as fg  # noqa: E402
import pi_steady as ps  # noqa: E402
import plant_full as pf  # noqa: E402
import r13_map as rm  # noqa: E402
import robust_check as rc  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
V2S = (88.2, 90.3, 92.4, 94.5, 96.6, 98.7, 100.8, 102.9, 105.0)
PS = (100.0, 300.0)
JS = np.arange(-3, 6)
CAL = {"V1-5": (88.2, 90.3, 92.4), "V1+5": (98.7, 100.8, 102.9)}


def duties(mb, P, V2):
    d1, d2 = mb.predict(np.array([P]), np.array([V2]))
    return min(float(np.ravel(d1)[0]), sw.D_SAFE), min(float(np.ravel(d2)[0]), sw.D_SAFE)


def ipk_at_power(variant, cf):
    """cf = [(V2, P, D1, D2)] -> Ipk at |P| on the plant with that variant, NaN where the scan does not bracket |P|"""
    p = rc.NOM if variant == "nominal" else rc.params(variant)
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=25.0, L=p["L_uH"] * 1e-6, td=p["td_ns"] * 1e-9, V1=p["V1"])
    V, P, D1, D2 = (np.array([c[i] for c in cf]) for i in range(4))
    D0i = ps.operating_point(P, D1, D2, V)["D0"]               # nominal ideal model: only the centre of the scan
    J = np.floor(np.nan_to_num(D0i) / fg.G).astype(int)[:, None] + JS[None, :]
    n, k = J.shape
    sd = [fg._safe(float(j) * fg.G, a, b, rm.TS) for j, a, b in zip(J.ravel(), np.repeat(D1, k), np.repeat(D2, k))]
    r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]), np.full(n * k, p["V1"]), np.repeat(V, k))
    Pd, I = (-r["P_LV"]).reshape(n, k), r["Ipk"].reshape(n, k)
    out = np.full(n, np.nan)
    for a in range(n):
        for i in range(k - 1):
            if Pd[a, i] <= P[a] <= Pd[a, i + 1]:
                out[a] = I[a, i] + (P[a] - Pd[a, i]) / (Pd[a, i + 1] - Pd[a, i]) * (I[a, i + 1] - I[a, i])
                break
    return out


def main():
    import pandas as pd
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    rows = []
    for variant, pts in [(v, [(V2, P) for V2 in V2S for P in PS]) for v in ("nominal", "L-10", "L+10", "td200")] + [(v, [(V2, 100.0) for V2 in vs]) for v, vs in CAL.items()]:
        cf = []
        for V2, P in pts:
            d = duties(mb, P, V2)
            cf += [(V2, P, sw.D_SAFE, sw.D_SAFE), (V2, P, d[0], d[1])]
        I = ipk_at_power(variant, cf)
        for i, (V2, P) in enumerate(pts):
            s, b = I[2 * i], I[2 * i + 1]
            rows.append(dict(variant=variant, V2=V2, P=P, SPS=s, MB=b, dI=s - b, tol=max(rc.TOL_A, rc.TOL_REL * s) if np.isfinite(s) else np.nan))
        print(f"{variant} done", flush=True)
    T = pd.DataFrame(rows)
    T["worse"] = T["dI"] < -T["tol"]
    X = pd.read_csv(os.path.join(OUT, "v1x_check.csv"))
    lines = ["rob_interior_plant: [참고] Python plant, 25 ns grid, NOT Simulink. dI = Ipk(SPS) - Ipk(M-B) [A], worse = dI < -max(0.03 A, 2 %)", ""]
    for v in ("nominal", "L-10", "L+10", "td200"):
        g = T[T["variant"] == v]
        lines.append(f"{v}: worse {int(g['worse'].sum())} of {int(np.isfinite(g['dI']).sum())}")
        for P in PS:
            lines.append(f"  {P:5.0f} W: " + "  ".join(f"{r.V2:g} {r.dI:+.3f}{'*' if r.worse else ''}" for r in g[g["P"] == P].itertuples()))
    lines += ["", "calibration, V1 +-5 % at 100 W: V2, plant dI, Simulink dI (80-period maximum, median), Simulink rule"]
    for r in T[T["variant"].isin(CAL)].itertuples():
        x = X[(X["set"] == "A") & (X["variant"] == r.variant) & (X["V2"] == r.V2) & (X["P"] == r.P)].iloc[0]
        lines.append(f"  {r.variant} {r.V2:g} V: {r.dI:+.3f}{'*' if r.worse else ''}  {x['dI_b']:+.3f} {x['dI_b_med']:+.3f} {x['code_b']}")
    T.to_csv(os.path.join(OUT, "rob_interior_plant.csv"), index=False, float_format="%.5g")
    open(os.path.join(OUT, "rob_interior_plant.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
