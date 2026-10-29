"""
ref1_compare.py -- how large can the TPS peak-current reduction be in reference [1] (Li et al., JESTPE 2023) compared
with this design? Ideal steady-state model only (tps_model.py) -> REFERENCE values, not Simulink-verified.

[1] (specs as noted by the user, 2026-10-29): V1 200 V, V2 160-230 V, n 1, fs 20 kHz, L 140 uH.
This design: V1 380 V, V2 84-109.2 V, n 4, fs 80 kHz, L 90 uH.

Scaling (RESULTS 6): i_L = V1/(w L) F(theta; k, D0, D1, D2), P = V1^2/(w L) G(k, D) with k = n V2 / V1. So the relative
reduction Ipk(TPS*)/Ipk(SPS) - 1 depends only on (k, p), p = P / P_base, P_base = V1^2 / (8 fs L) (= SPS power at
D0 = 90 deg, k = 1). The two converters can be compared at the same (k, p).

TPS* = best (D1, D2) on a 0.005 grid in [0.05, 0.5], D0 at the PI's first crossing (as in the controllers, pi_steady.py),
SPS = D1 = D2 = 0.5 at its first crossing. Also: Ipk vs D (D1 = D2 = D) at k = 1, V2 200 V, 100 W for [1] Fig. 14(a).

    python -I -X utf8 Simulink/Thesis/ref1_compare.py      -> out/ref1_compare.{log,csv}
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")
REF1 = dict(name="[1] Li 2023", V1=200.0, n=1.0, fs=20e3, L=140e-6, V2s=(160.0, 180.0, 200.0, 215.0, 230.0),
            Ps=(100.0, 200.0, 300.0, 500.0, 1000.0))
OURS = dict(name="this design", V1=380.0, n=4.0, fs=80e3, L=90e-6, V2s=(84.0, 95.0, 109.2),
            Ps=(100.0, 200.0, 300.0, 600.0, 1000.0))


def p_abs(phi, D1, D2, V2, c):
    """|P| on the first branch, phi = centre phase in [0, 90] deg (same as pi_steady._p_abs, with the converter c)"""
    return -tm.evaluate(phi - 180.0 * (D2 - D1), D1, D2, V2, V1=c["V1"], n=c["n"], fs=c["fs"], L=c["L"])["P"]


def first_crossing(P, D1, D2, V2, c, iters=40):
    """Ipk at the PI's first crossing |P|(phi) = P for arrays D1, D2 (NaN where the branch cannot deliver P)"""
    D1, D2 = np.broadcast_arrays(np.asarray(D1, float), np.asarray(D2, float))
    grid = np.arange(0.0, 90.0 + 1e-9, 1.0)
    pg = p_abs(grid[None, :], D1.ravel()[:, None], D2.ravel()[:, None], V2, c)
    pm = np.maximum.accumulate(pg, axis=1)
    ok = pm[:, -1] >= P
    j = np.where(ok, np.argmax(pm >= P, axis=1), grid.size - 1)
    lo, hi = grid[np.maximum(j - 1, 0)], grid[j]
    d1, d2 = D1.ravel(), D2.ravel()
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        f = p_abs(mid, d1, d2, V2, c) >= P
        hi, lo = np.where(f, mid, hi), np.where(f, lo, mid)
    r = tm.evaluate(hi - 180.0 * (d2 - d1), d1, d2, V2, V1=c["V1"], n=c["n"], fs=c["fs"], L=c["L"])
    return np.where(ok, r["Ipk"], np.nan).reshape(D1.shape)


def best_tps(P, V2, c, step=0.005):
    d = np.round(np.arange(0.05, 0.5 + 1e-9, step), 6)
    G1, G2 = np.meshgrid(d, d, indexing="ij")
    I = first_crossing(P, G1, G2, V2, c)
    k = int(np.nanargmin(I))
    return float(I.flat[k]), float(G1.flat[k]), float(G2.flat[k])


def main():
    os.makedirs(OUT, exist_ok=True)
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("ref1_compare.py -- ideal steady-state model (tps_model.py), REFERENCE ONLY (not Simulink-verified)")
    out("relative reduction = 1 - Ipk(best TPS, PI first crossing, D grid 0.005) / Ipk(SPS, first crossing)")
    for c in (REF1, OURS):
        pb = c["V1"] ** 2 / (8.0 * c["fs"] * c["L"])
        out("")
        out(f"{c['name']}: V1 {c['V1']:g} V, n {c['n']:g}, fs {c['fs'] / 1e3:g} kHz, L {c['L'] * 1e6:g} uH, "
            f"P_base = V1^2/(8 fs L) = {pb:.0f} W")
        out("    V2      k   circ.peak |    |P|    p[pu] | SPS Ipk | best TPS Ipk (D1, D2)    | reduction")
        for V2 in c["V2s"]:
            k = c["n"] * V2 / c["V1"]
            circ = c["V1"] * abs(1.0 - k) / (4.0 * c["fs"] * c["L"])
            for P in c["Ps"]:
                isps = float(first_crossing(P, 0.5, 0.5, V2, c))
                itps, d1, d2 = best_tps(P, V2, c)
                itps = min(itps, isps)
                red = isps - itps
                out(f"  {V2:6.1f}  {k:5.3f}  {circ:6.2f} A  | {P:6.0f} W  {P / pb:6.3f} | {isps:6.3f}  | "
                    f"{itps:6.3f} ({d1:.3f}, {d2:.3f})    | {red:5.3f} A  {100 * red / isps:5.1f} %")
                rows.append(dict(conv=c["name"], V2=V2, k=round(k, 4), circ_A=round(circ, 4), P=P, p_pu=round(P / pb, 4),
                                 Ipk_sps=round(isps, 4), Ipk_tps=round(itps, 4), D1=d1, D2=d2,
                                 red_A=round(red, 4), red_pct=round(100 * red / isps, 2)))

    # [1] Fig. 14(a): V2 200 V (k = 1), 100 W -- Ipk vs D with D1 = D2 = D
    c = REF1
    out("")
    out("[1] Fig. 14(a) check: V2 200 V (k = 1), |P| 100 W, D1 = D2 = D (here 0.5 = SPS; [1]'s scale is 2 D, 1 = SPS)")
    out("    D (2D)        Ipk     Ipk/Ipk_SPS")
    isps = float(first_crossing(100.0, 0.5, 0.5, 200.0, c))
    for D in (0.5, 0.49, 0.48, 0.47, 0.46, 0.45, 0.44, 0.43, 0.425, 0.42, 0.41, 0.40, 0.38, 0.35, 0.30):
        i = float(first_crossing(100.0, D, D, 200.0, c))
        out(f"  {D:5.3f} ({2 * D:5.3f})  {i:7.4f} A  {i / isps:7.4f}")
    itps, d1, d2 = best_tps(100.0, 200.0, c)
    out(f"  best (D1, D2) on the grid: ({d1:.3f}, {d2:.3f}) Ipk {itps:.4f} A vs SPS {isps:.4f} A")
    out("  one inner phase only (other one = SPS 0.5), same point:")
    out("    D (2D)    Ipk(D1 = D, D2 = 0.5)    Ipk(D1 = 0.5, D2 = D)")
    for D in (0.5, 0.475, 0.45, 0.425, 0.42, 0.40):
        a = float(first_crossing(100.0, D, 0.5, 200.0, c))
        b = float(first_crossing(100.0, 0.5, D, 200.0, c))
        out(f"  {D:5.3f} ({2 * D:5.3f})  {a:7.4f} A ({a / isps:6.4f})   {b:7.4f} A ({b / isps:6.4f})")

    with open(os.path.join(OUT, "ref1_compare.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(OUT, "ref1_compare.log"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
