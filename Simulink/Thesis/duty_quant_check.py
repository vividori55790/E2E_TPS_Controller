"""
duty_quant_check.py -- (2026-10-30) is the Simulink Ipk a step function of the duties? (report only, no verdict)

Why: in ms_compare round 1, M-S and M-B differ by 0.004 in D1/D2 at 84 V 150 W and the Simulink Ipk differs by 0.16 A,
while at 94 V 100 W / 95 V 70 W a 0.004 difference gives the identical Ipk to 1e-6 A. The controller of the dataset-era
model (dataset_era/DAB_Controller_System.m, STAGE 7) compares a phase counter that advances 2.88 deg per 100 ns step
(Ts_ctrl = 100 ns, 125 steps per 12.5 us period) with each leg phase p + DT and p + 180 - DT (DT = 100 ns = 2.88 deg).
Since 180 deg = 62.5 steps, a leg's on-edge moves when p/2.88 crosses an integer and its off-edge when it crosses a
half-integer, so every duty is quantised to cells of 1/250 = 0.004 (D1 -> p3 = 360 D1; D2 -> p7 = D0 + 360 D2, and the
D0 levels -1.44 - 2.88 k sit at half-integers, so the D2 cells are again floor(250 D2)).

Prediction: two runs at the same (V2, D0, run length) whose duties fall in the same cells (floor(250 D1), floor(250 D2))
give the same Ipk; neighbouring cells can jump. This script checks it on every Simulink steady-state run in out/
(constant-D0 sweeps: R.4, R8, R9, OPT, k1map, M-S), and measures how large one cell step is (light vs heavier load).
Cell boundaries at exact integers of 250 D (D = 0.5, 0.4, ...) are knife edges of the float phase counter; pairs with a
duty within 1e-6 of a boundary are reported apart.

    python -I -X utf8 Simulink/Thesis/duty_quant_check.py   -> out/duty_quant_check.{log,csv}
"""
import glob
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "out")
FAMS = ("simulink_sweep", "eff_sweep", "zvs_sweep", "k1map_sweep", "opt_sweep")
CELL = 250.0
SAME = 1e-6                     # A: "identical" Ipk


def runs():
    fr = []
    for fam in FAMS:
        for f in sorted(glob.glob(os.path.join(OUT, f"{fam}_*.csv"))):
            if f.endswith("_in.csv"):
                continue
            T = pd.read_csv(f)
            if not {"V2", "D0", "D1", "D2", "Ipk_sim"} <= set(T.columns):
                continue
            ms = [int(m.group(1)) for c in T.columns for m in [re.match(r"Ipk_at_(\d+)ms$", c)] if m]
            if "error" in T.columns:
                T = T[T["error"].isna() | (T["error"].astype(str).str.strip() == "")]
            T = T.assign(src=os.path.basename(f), t_end=max(ms) if ms else 0)
            fr.append(T[["src", "t_end", "V2", "D0", "D1", "D2", "Ipk_sim", "iL_mean", "P_LV"]])
    R = pd.concat(fr, ignore_index=True).dropna(subset=["Ipk_sim"])
    R = R.round({"V2": 4, "D0": 4})
    R = R.drop_duplicates(subset=["t_end", "V2", "D0", "D1", "D2"])          # exact reruns are deterministic (<= 1e-10 A)
    x1, x2 = R["D1"] * CELL, R["D2"] * CELL
    R["c1"], R["c2"] = np.floor(x1 + 1e-9).astype(int), np.floor(x2 + 1e-9).astype(int)
    R["edge"] = (np.abs(x1 - np.round(x1)) < 250e-6) | (np.abs(x2 - np.round(x2)) < 250e-6)
    return R


def pairs(R):
    out = []
    for _, g in R.groupby(["t_end", "V2", "D0"]):
        if len(g) < 2:
            continue
        a = g.to_dict("records")
        for i in range(len(a)):
            for j in range(i + 1, len(a)):
                p, q = a[i], a[j]
                d1, d2 = abs(p["D1"] - q["D1"]), abs(p["D2"] - q["D2"])
                if max(d1, d2) > 0.0125 or max(d1, d2) < 1e-9:
                    continue
                dc = abs(p["c1"] - q["c1"]) + abs(p["c2"] - q["c2"])
                out.append(dict(t_end=p["t_end"], V2=p["V2"], D0=p["D0"], D1a=p["D1"], D2a=p["D2"], D1b=q["D1"],
                                D2b=q["D2"], dD=max(d1, d2), dcell=dc, edge=bool(p["edge"] or q["edge"]),
                                P=0.5 * (abs(p["P_LV"]) + abs(q["P_LV"])), Ia=p["Ipk_sim"], Ib=q["Ipk_sim"],
                                dI=abs(p["Ipk_sim"] - q["Ipk_sim"]), dP=abs(p["P_LV"] - q["P_LV"]),
                                osc=max(abs(p["iL_mean"]), abs(q["iL_mean"]))))
    return pd.DataFrame(out)


def same_power(R, max_cells=3):
    """duty pairs a few cells apart compared at the SAME transferred power: each duty's Ipk is interpolated in measured
    power between two adjacent 2.88-deg D0 levels (the OPT way), at the middle of the power range both pairs cover"""
    R = R.assign(Pd=-R["P_LV"], key=list(zip(R["D1"].round(9), R["D2"].round(9))))
    out = []
    for (t, v), g in R.groupby(["t_end", "V2"]):
        seg = {}
        for k, h in g.groupby("key"):
            h = h.sort_values("D0")                                # adjacent levels; power need not fall with D0 near 0
            D0, P, I, osc = h["D0"].to_numpy(), h["Pd"].to_numpy(), h["Ipk_sim"].to_numpy(), h["iL_mean"].abs().to_numpy()
            s = []
            for i in range(len(h) - 1):
                if abs(abs(D0[i] - D0[i + 1]) - 2.88) < 1e-3 and min(P[i], P[i + 1]) > 0 and abs(P[i + 1] - P[i]) > 1e-6 \
                        and max(osc[i], osc[i + 1]) <= 0.02:
                    lo, hi = (i, i + 1) if P[i] < P[i + 1] else (i + 1, i)
                    s.append((P[lo], P[hi], I[lo], I[hi]))
            if s:
                seg[k] = (int(h["c1"].iloc[0]), int(h["c2"].iloc[0]), s)
        ks = list(seg)
        for i in range(len(ks)):
            for j in range(i + 1, len(ks)):
                a, b = seg[ks[i]], seg[ks[j]]
                dc = abs(a[0] - b[0]) + abs(a[1] - b[1])
                if dc > max_cells or max(abs(ks[i][0] - ks[j][0]), abs(ks[i][1] - ks[j][1])) > 0.0125:
                    continue
                best = None
                for (p0, p1, i0, i1) in a[2]:
                    for (q0, q1, j0, j1) in b[2]:
                        lo, hi = max(p0, q0), min(p1, q1)
                        if hi > lo and (best is None or hi - lo > best[1] - best[0]):
                            best = (lo, hi, p0, p1, i0, i1, q0, q1, j0, j1)
                if best is None:
                    continue
                lo, hi, p0, p1, i0, i1, q0, q1, j0, j1 = best
                Ps = 0.5 * (lo + hi)
                Ia = i0 + (i1 - i0) * (Ps - p0) / (p1 - p0)
                Ib = j0 + (j1 - j0) * (Ps - q0) / (q1 - q0)
                out.append(dict(t_end=t, V2=v, P=Ps, D1a=ks[i][0], D2a=ks[i][1], D1b=ks[j][0], D2b=ks[j][1], dcell=dc,
                                Ia=Ia, Ib=Ib, dI=abs(Ia - Ib)))
    return pd.DataFrame(out)


def main():
    R = runs()
    Q = pairs(R)
    lines = []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    o("duty_quant_check: Simulink Ipk vs duty cells floor(250 D) (report only)")
    o(f"runs {len(R)} (distinct t_end/V2/D0/D1/D2), files {R['src'].nunique()}, pairs at the same (t_end, V2, D0) with "
      f"0 < max|dD| <= 0.0125: {len(Q)}")
    if Q.empty:
        return
    Q.to_csv(os.path.join(OUT, "duty_quant_check.csv"), index=False, float_format="%.6g")
    clean = Q["osc"] <= 0.02
    o("\n(a) same cells vs different cells (clean pairs, |mean i_L| <= 20 mA, no knife-edge duty):")
    o(f"{'cells apart':>11} {'pairs':>6} {'identical Ipk (<1e-6 A)':>24} {'|dIpk| median':>14} {'p90':>7} {'max':>7}")
    for dc in (0, 1, 2, 3):
        g = Q[clean & ~Q["edge"] & (Q["dcell"] == dc if dc < 3 else Q["dcell"] >= 3)]
        if g.empty:
            continue
        o(f"{('>= 3' if dc == 3 else str(dc)):>11} {len(g):6d} {int((g['dI'] < SAME).sum()):12d} ({(g['dI'] < SAME).mean():5.1%})"
          f"      {g['dI'].median():14.4f} {g['dI'].quantile(0.9):7.4f} {g['dI'].max():7.4f}")
    g = Q[clean & Q["edge"]]
    o(f"knife-edge pairs (a duty on a cell boundary within 1e-6): {len(g)}, identical {int((g['dI'] < SAME).sum())}")
    g0 = Q[clean & ~Q["edge"] & (Q["dcell"] == 0)]
    bad = g0[g0["dI"] >= SAME]
    if len(bad):
        o("same-cell pairs that differ (first 10):")
        for _, r in bad.head(10).iterrows():
            o(f"   {r['V2']:g} V D0 {r['D0']:g}: ({r['D1a']:.5f}, {r['D2a']:.5f}) vs ({r['D1b']:.5f}, {r['D2b']:.5f}) "
              f"dI {r['dI']:.4f} A, t_end {r['t_end']}")
    o("\n(b) size of one cell step (clean pairs exactly one cell apart), by power and V2:")
    g1 = Q[clean & ~Q["edge"] & (Q["dcell"] == 1)].copy()
    g1["band"] = pd.cut(g1["P"], [0, 150, 300, 600, 2000], labels=["< 150 W", "150-300 W", "300-600 W", ">= 600 W"])
    g1["rel"] = g1["dI"] / g1[["Ia", "Ib"]].mean(axis=1)
    o(f"{'band':>10} {'pairs':>6} {'|dIpk| median':>14} {'p90':>7} {'max':>7} {'rel median':>11} {'|dP| median W':>14}")
    for b, g in g1.groupby("band", observed=True):
        o(f"{b:>10} {len(g):6d} {g['dI'].median():14.4f} {g['dI'].quantile(0.9):7.4f} {g['dI'].max():7.4f} "
          f"{g['rel'].median():11.2%} {g['dP'].median():14.2f}")
    for v, g in g1.groupby("V2"):
        if len(g) >= 5:
            o(f"   V2 {v:g} V: {len(g)} pairs, |dIpk| median {g['dI'].median():.4f}, p90 {g['dI'].quantile(0.9):.4f} A")
    S = same_power(R)
    if len(S):
        import pi_steady as ps                                     # reference only: ideal model, same pairs, same power
        A = ps.operating_point(S["P"], S["D1a"], S["D2a"], S["V2"])["Ipk"]
        B = ps.operating_point(S["P"], S["D1b"], S["D2b"], S["V2"])["Ipk"]
        S["dI_model"] = np.abs(A - B)
        S.to_csv(os.path.join(OUT, "duty_quant_samepower.csv"), index=False, float_format="%.6g")
        S["band"] = pd.cut(S["P"], [0, 150, 300, 600, 2000], labels=["< 150 W", "150-300 W", "300-600 W", ">= 600 W"])
        o("\n(d) the same comparison at the SAME power (each duty interpolated between two adjacent D0 levels, clean runs);"
          " last column = the ideal analytic model for the same pairs (reference only, not Simulink):")
        o(f"{'cells apart':>11} {'band':>10} {'pairs':>6} {'|dIpk| median':>14} {'p90':>7} {'max':>7} {'model median':>13}")
        for dc in (0, 1, 2, 3):
            for b, g in S[S["dcell"] == dc].groupby("band", observed=True):
                o(f"{dc:11d} {b:>10} {len(g):6d} {g['dI'].median():14.4f} {g['dI'].quantile(0.9):7.4f} {g['dI'].max():7.4f}"
                  f" {g['dI_model'].median():13.4f}")
        g = S[S["dcell"].between(1, 2)]
        o(f"1-2 cells apart, all bands: {len(g)} pairs, Simulink |dIpk| median {g['dI'].median():.4f} A vs model "
          f"{g['dI_model'].median():.4f} A (ratio {g['dI'].median() / max(g['dI_model'].median(), 1e-9):.0f}x)")
    o("\n(c) what a 0.004 duty change means: 50 ns of a pulse edge = one 100 ns controller step at half-step offset;"
      " real MCU timers (e.g. 184 ps HRTIM) resolve it ~500x finer.")
    open(os.path.join(OUT, "duty_quant_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("wrote out/duty_quant_check.{log,csv}")


if __name__ == "__main__":
    main()
