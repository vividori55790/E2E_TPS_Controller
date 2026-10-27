"""
dataset_fixed_analyze.py -- R7 (2026-10-27): the user's pipeline step "dataset -> optimum", done directly on the data
(no surrogate, no ML), on the user's original dataset and on the fixed dataset (gen_dataset_fixed.m: the same
experiment with the D0 block's interpolation off), side by side, plus the Simulink PI steady state of R.4.

For every (V2, D1, D2) run the 39 D0 steps give (measured P_LV, Ipk) along D0. At a target |P| and direction the run's
operating point is the rising crossing closest to D0 = 0 (what a PI started at D0 = 0 reaches first; same rule as
staircase_compare.crossing), Ipk linear in power between the two bracketing steps. Then per (V2, |P|, direction):
SPS = the D1 = D2 = 0.5 run, best TPS = the run with the lowest Ipk at that power, dI = Ipk(SPS) - Ipk(best TPS).
The original dataset is evaluated twice: on its full 20 x 20 (D1, D2) grid and on the fixed dataset's 11 x 11 subset
(same combinations, so the two datasets differ only by the D0 interpolation).
Power sign as in the dataset: P_LV < 0 <=> D0 > 0 = HV->LV (the main text, R.4); P_LV > 0 <=> D0 < 0 = LV->HV.

    python -I -X utf8 Simulink/Thesis/dataset_fixed_analyze.py [--prefix out/dataset_fixed]
        -> out/dataset_fixed_compare.{log,csv}, out/fig_dataset_fixed.png
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from staircase_compare import crossing  # noqa: E402

OUT = os.path.join(HERE, "out")
TARGETS = (100.0, 150.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)
DIRS = ((+1, "HV->LV"), (-1, "LV->HV"))


def load_fixed(prefix):
    import pandas as pd
    parts = sorted(glob.glob(prefix + "_part_*.csv"))
    if not parts:
        return None
    d = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True)
    return d.rename(columns={"I_pk": "Ipk", "P_act": "P"})


def load_original():
    import pandas as pd
    d = pd.read_csv(os.path.join(OUT, "checkpoint_20260316_021229.csv"))
    d = d[d["valid"] == 1]
    return d.rename(columns={"I_pk": "Ipk", "P_act": "P"})


def runs(df, V2):
    """{(D1, D2): (D0, P, Ipk)} for one V2"""
    g = df[np.isclose(df["V2"], V2)]
    out = {}
    for (d1, d2), r in g.groupby(["D1", "D2"]):
        r = r.sort_values("D0")
        out[(round(float(d1), 6), round(float(d2), 6))] = (r["D0"].to_numpy(float), r["P"].to_numpy(float),
                                                           r["Ipk"].to_numpy(float))
    return out


def optimum(R, P, s, keep=None):
    """SPS Ipk, best TPS (Ipk, D1, D2) at |P| = P in direction s over the runs R (optionally only the keys in keep)"""
    sps = R.get((0.5, 0.5))
    Is = crossing(*sps, P, s)[0] if sps is not None else np.nan
    best = (np.inf, np.nan, np.nan)
    for k, (D0, Pm, I) in R.items():
        if k == (0.5, 0.5) or (keep is not None and k not in keep):
            continue
        Ic = crossing(D0, Pm, I, P, s)[0]
        if np.isfinite(Ic) and Ic < best[0]:
            best = (Ic, k[0], k[1])
    return Is, best


def r4_reference():
    """R.4 Simulink PI steady state (HV->LV): {(V2, P): (Ipk_SPS, dI_M-B)}"""
    import pandas as pd
    f = os.path.join(OUT, "simulink_pi_compare.csv")
    if not os.path.isfile(f):
        return {}
    t = pd.read_csv(f)
    ref = {}
    for (v, p), g in t.groupby(["V2", "P"]):
        sps = g[g["ctrl"] == "SPS"]["Ipk_simulink"]
        mb = g[g["ctrl"] == "M-B"]["dI_simulink"]
        if len(sps) and len(mb):
            ref[(float(v), float(p))] = (float(sps.iloc[0]), float(mb.iloc[0]))
    return ref


def main():
    import pandas as pd
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", default=os.path.join(OUT, "dataset_fixed"))
    ap.add_argument("--tag", default="")
    ap.add_argument("--fixed-label", default="fixed dataset (Simulink, true staircase), same method")
    a = ap.parse_args()
    fx = load_fixed(a.prefix)
    if fx is None:
        sys.exit(f"no part files {a.prefix}_part_*.csv")
    og = load_original()
    ref = r4_reference()
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    V2s = sorted(fx["V2"].unique())
    out(f"fixed dataset {os.path.relpath(a.prefix, HERE)}_part_*.csv: {fx.groupby(['V2', 'D1', 'D2']).ngroups} runs, "
        f"V2 {', '.join(f'{v:.4g}' for v in V2s)}")
    out("dI = Ipk(SPS) - Ipk(best TPS run) at the same measured |P| (rising crossing closest to D0 = 0); "
        "positive = TPS lower")
    out("columns: original dataset on its full 20x20 grid | original on the fixed grid | fixed dataset | "
        "R.4 Simulink PI steady state (M-B, HV->LV, V2 84/95/109.2)")
    for V2 in V2s:
        Rf = runs(fx, V2)
        Ro = runs(og, V2)
        keep = set(Rf.keys())
        k = 4.0 * V2 / 380.0
        for s, lab in DIRS:
            out(f"\nV2 {V2:.4g} V (k {k:.3f}), {lab}")
            out(f"  {'|P|':>5s} | {'orig full: SPS  best  dI':>27s} | {'orig sub: SPS  best  dI':>26s} | "
                f"{'fixed: SPS  best (D1,D2)  dI':>38s} | {'R.4: SPS  dI(M-B)':>18s}")
            for P in TARGETS:
                o_full = optimum(Ro, P, s)
                o_sub = optimum(Ro, P, s, keep)
                f_ = optimum(Rf, P, s)
                v4 = min((84.0, 95.0, 109.2), key=lambda x: abs(x - V2))
                r4 = ref.get((v4, P)) if (s == +1 and abs(v4 - V2) < 0.5) else None
                cell = lambda o: (f"{o[0]:6.2f} {o[1][0]:5.2f} {o[0] - o[1][0]:+5.2f}"  # noqa: E731
                                  if np.isfinite(o[0]) and np.isfinite(o[1][0]) else "   n/a")
                fcell = (f"{f_[0]:6.2f} {f_[1][0]:5.2f} ({f_[1][1]:.3f},{f_[1][2]:.3f}) {f_[0] - f_[1][0]:+5.2f}"
                         if np.isfinite(f_[0]) and np.isfinite(f_[1][0]) else "   n/a")
                rcell = f"{r4[0]:6.3f} {r4[1]:+6.3f}" if r4 else ""
                out(f"  {P:5.0f} | {cell(o_full):>27s} | {cell(o_sub):>26s} | {fcell:>38s} | {rcell:>18s}")
                for name, o in (("orig_full", o_full), ("orig_sub", o_sub), ("fixed", f_)):
                    rows.append(dict(V2=V2, direction=lab, P=P, source=name, Ipk_SPS=o[0], Ipk_TPS=o[1][0],
                                     D1=o[1][1], D2=o[1][2], dI=o[0] - o[1][0],
                                     dI_pct=100 * (o[0] - o[1][0]) / o[0] if np.isfinite(o[0]) else np.nan))
                if r4:
                    rows.append(dict(V2=v4, direction=lab, P=P, source="R4_MB_steady", Ipk_SPS=r4[0],
                                     Ipk_TPS=r4[0] - r4[1], D1=np.nan, D2=np.nan, dI=r4[1], dI_pct=100 * r4[1] / r4[0]))
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, f"dataset_fixed_compare{a.tag}.csv"), index=False)

    out("\nsummary (mean over the |P| targets where both SPS and a TPS run reach the power):")
    for (V2, lab), g in T[T["source"] != "R4_MB_steady"].groupby(["V2", "direction"]):
        cells = []
        for name in ("orig_full", "orig_sub", "fixed"):
            x = g[g["source"] == name]
            cells.append(f"{name} dI {x['dI'].mean():+.2f} A ({x['dI_pct'].mean():+.0f} %), SPS {x['Ipk_SPS'].mean():.2f} A")
        out(f"  V2 {V2:.4g} {lab:7s}: " + " | ".join(cells))
    open(os.path.join(OUT, f"dataset_fixed_compare{a.tag}.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(T, a.tag, a.fixed_label)


def fig(T, tag="", fixed_label="fixed dataset (Simulink, true staircase), same method"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    H = T[T["direction"] == "HV->LV"]
    V2s = sorted(H[H["source"] == "fixed"]["V2"].unique())
    f, ax = plt.subplots(2, len(V2s), figsize=(4.4 * len(V2s), 7.0), constrained_layout=True, sharex=True,
                         squeeze=False)
    sty = {"orig_full": (ORANGE, "--", "x", "original dataset (D0 interpolated), full grid"),
           "fixed": (BLUE, "-", "o", fixed_label),
           "R4_MB_steady": (AQUA, "none", "D", "Simulink PI steady state, M-B (R.4)")}
    for j, V2 in enumerate(V2s):
        for name, (c, ls, mk, lab) in sty.items():
            if name == "R4_MB_steady":
                g = H[(H["source"] == name) & (np.abs(H["V2"] - V2) < 0.5)]
            else:
                g = H[(H["source"] == name) & np.isclose(H["V2"], V2)]
            g = g.sort_values("P")
            if not len(g):
                continue
            ax[0, j].plot(g["P"], g["Ipk_SPS"], linestyle=ls, marker=mk, color=c, lw=2, ms=6, label=lab)
            ax[1, j].plot(g["P"], g["dI"], linestyle=ls, marker=mk, color=c, lw=2, ms=6)
        ax[1, j].axhline(0.0, color="#c3c2b7", lw=1)
        ax[0, j].set_title(f"V2 = {V2:.4g} V (k = {4 * V2 / 380:.3f}), HV -> LV")
        ax[1, j].set_xlabel("|P| [W] (measured)")
    ax[0, 0].set_ylabel("SPS Ipk at that power [A]")
    ax[1, 0].set_ylabel("Ipk(SPS) - Ipk(best TPS) [A]")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="upper left")
    ax[1, -1].text(0.98, 0.97, "best TPS = lowest-Ipk run of the grid at that power", transform=ax[1, -1].transAxes,
                   ha="right", va="top", fontsize=7.5, color=MUTED)
    ax[0, -1].yaxis.label.set_color(INK2)
    f.savefig(os.path.join(OUT, f"fig_dataset_fixed{tag}.png"), dpi=130)
    plt.close(f)


if __name__ == "__main__":
    main()
