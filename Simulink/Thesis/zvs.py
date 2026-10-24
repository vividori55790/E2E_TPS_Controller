"""
zvs.py -- T11: zero-voltage-switching map of the controllers (side metric; the controllers minimise Ipk, not
switching loss). Ideal steady-state model, PI steady state, test set (out/pointwise_test.npz, 10 000 points).

Turn-on of a leg's upper switch is soft (ZVS) when the out-of-midpoint current is negative at that instant (it
then charges the midpoint up to the rail through the dead time); the lower switch turn-on half a period later has
the mirrored condition, which is the same by half-wave symmetry. Out-of-midpoint currents: i_A = i, i_B = -i,
i_C = -n i, i_D = +n i (tps_model.py PWM convention). Turn-on instants: A at 0, B at 360 D1, C at D0,
D at D0 + 360 D2 (deg). So
    ZVS_A: i(0) < -I_min    ZVS_B: i(360 D1) > I_min    ZVS_C: i(D0) > I_min / n    ZVS_D: i(D0 + 360 D2) < -I_min / n
I_min (the current needed to swing Coss in the dead time) is unknown for this plant; reported at 0 and 0.5 A
(primary-side switch current; the secondary carries n times the current, so its threshold is I_min/n in i).
Also: hard-switched current proxy = sum over hard legs of |switch current| * V_bus (W/Hz-ish, relative only).

Output: out/zvs.log, out/zvs_table.csv
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")


def edge_currents(D0, D1, D2, V2):
    th = np.stack([np.zeros_like(D0), 360.0 * D1, D0, D0 + 360.0 * D2], axis=1)      # (N, 4)
    return tm.current_at(th, D0, D1, D2, V2)


def zvs_flags(I, I_min):
    n = tm.N_TURNS
    return np.stack([I[:, 0] < -I_min, I[:, 1] > I_min, I[:, 2] > I_min / n, I[:, 3] < -I_min / n], axis=1)


def hard_proxy(I, flags, V2):
    n = tm.N_TURNS
    sw = np.abs(I) * np.array([1.0, 1.0, n, n])                                      # switch currents
    vb = np.stack([np.full_like(V2, tm.V1_NOM)] * 2 + [V2] * 2, axis=1)
    return np.sum(np.where(flags, 0.0, sw * vb), axis=1)


def main():
    import evaluate as ev
    z = np.load(os.path.join(OUT, "pointwise_test.npz"))
    P, V = z["P"], z["V2"]
    ctrls, best = ev.build(b_tags=["3"])
    keep = ["SPS", "ICROS2026 surface (P>0)", "LUT v2 49x22", "Smooth RegHDC (b3)"]
    sel = {c.name: c for c in ctrls if c.name in keep}
    rows, lines = [], []

    def out(s=""):
        print(s, flush=True)
        lines.append(s)

    series = [(name, sel[name]) for name in keep if name in sel] + [("pointwise optimum", None)]
    out(f"test set {P.size} points; ZVS of the 4 legs at the PI steady state (ideal model)")
    out(f"{'controller':28s} {'I_min':>5s} {'all-ZVS %':>9s} {'mean hard legs':>14s} {'A':>6s} {'B':>6s} {'C':>6s} "
        f"{'D':>6s} {'hard proxy (rel. SPS)':>22s}")
    base_proxy = None
    for name, c in series:
        if c is None:
            d1, d2, D0 = z["D1"], z["D2"], z["phi"] - 180.0 * (z["D2"] - z["D1"])
            ok = np.isfinite(D0)
        else:
            d1, d2 = c.predict(P, V)
            d1, d2 = np.asarray(d1, float).ravel(), np.asarray(d2, float).ravel()
            r = ps.operating_point(P, d1, d2, V)
            D0, ok = r["D0"], r["ok"]
        I = edge_currents(np.nan_to_num(D0), d1, d2, V)     # pi_steady's branch (phi >= 0, HV -> LV)
        for I_min in (0.0, 0.5):
            f = zvs_flags(I, I_min)[ok]
            hp = hard_proxy(I[ok], f, V[ok]).mean()
            if I_min == 0.0 and name == "SPS":
                base_proxy = hp
            rel = hp / base_proxy if base_proxy else np.nan
            out(f"{name:28s} {I_min:5.1f} {f.all(axis=1).mean() * 100:9.1f} {(~f).sum(axis=1).mean():14.2f} "
                + " ".join(f"{x * 100:5.1f}%" for x in f.mean(axis=0)) + f" {rel:22.3f}")
            rows.append(dict(controller=name, I_min=I_min, all_zvs_pct=f.all(axis=1).mean() * 100,
                             hard_legs=(~f).sum(axis=1).mean(), A=f[:, 0].mean(), B=f[:, 1].mean(), C=f[:, 2].mean(),
                             D=f[:, 3].mean(), hard_proxy_rel=rel))
    # where SPS loses ZVS and whether the TPS controller recovers it: by k and load
    k = tm.N_TURNS * V / tm.V1_NOM
    out("\nall-ZVS share (I_min = 0) by region:")
    regions = [("|P| < 300 W", P < 300), ("300-700 W", (P >= 300) & (P < 700)), (">= 700 W", P >= 700),
               ("k < 0.95", k < 0.95), ("0.95 <= k <= 1.05", (k >= 0.95) & (k <= 1.05)), ("k > 1.05", k > 1.05)]
    hdr = f"{'controller':28s} " + " ".join(f"{r[0]:>18s}" for r in regions)
    out(hdr)
    for name, c in series:
        if c is None:
            d1, d2, D0 = z["D1"], z["D2"], z["phi"] - 180.0 * (z["D2"] - z["D1"])
        else:
            d1, d2 = c.predict(P, V)
            d1, d2 = np.asarray(d1, float).ravel(), np.asarray(d2, float).ravel()
            D0 = ps.operating_point(P, d1, d2, V)["D0"]
        f = zvs_flags(edge_currents(np.nan_to_num(D0), d1, d2, V), 0.0).all(axis=1)
        out(f"{name:28s} " + " ".join(f"{f[m].mean() * 100:17.1f}%" for _, m in regions))
    with open(os.path.join(OUT, "zvs_table.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    open(os.path.join(OUT, "zvs.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
