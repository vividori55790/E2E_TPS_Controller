"""
ipk_decompose.py -- 2026-10-28: Simulink-only decomposition of the user's dataset "reduction" (no analytic model,
no Python plant). At the same measured power (HV->LV, first rising crossing, linear in power; dataset_fixed_analyze):

    A = dataset SPS Ipk              (user's checkpoint, D0 block interpolating = staircase_check interp=1, bit-exact)
    B = dataset grid-best TPS Ipk    (lowest Ipk over the 20 x 20 (D1, D2) grid at that power)
    C = Simulink steady SPS Ipk      (R.4, constant D0 30 ms, two bracketing D0 steps interpolated in power)
    D = Simulink steady M-B Ipk      (R.4, same method, the proposed controller's (D1, D2))

    A - B  =  (A - C)          +  (C - D)            +  (D - B)
    dataset "reduction" = SPS row over-measured + M-B steady reduction + M-B above the dataset's best TPS row

The last term is the only place where the dataset could still claim more than M-B; its TPS row B is itself measured
on the moving D0, so D - B is an upper bound on what a steady TPS could add (R6 / r5r6 step 2 measures B in steady
state). k ~ 1: the dataset grid has 94.61 V, R.4 has 95 V (k 0.996 vs 1.000) -> shown, not averaged.

Second part (B in steady state): add_best_tps_steady_points.py put the dataset's best-TPS combos of the D - B > 0
points ("BT_" labels) into r5r6_chain step 2. When out/stair_steady_part*.csv exist, their Simulink steady Ipk at the
same power (B_st, same crossing rule) replaces B: D - B_st is what a steady TPS of the dataset really adds over M-B.
--py adds the Python plant's steady prediction for the same labels (not a verdict; ~1 min).

    python -I -X utf8 Simulink/Thesis/ipk_decompose.py [--py]  -> out/ipk_decompose.{log,csv}
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from staircase_compare import crossing  # noqa: E402

OUT = os.path.join(HERE, "out")
PS = (100.0, 300.0, 600.0, 1000.0)
V2S = ((84.0, 84.0), (94.61052632, 95.0), (109.2, 109.2))   # (dataset grid V2, R.4 V2)


def main():
    ds = pd.read_csv(os.path.join(OUT, "dataset_fixed_compare_py_zoh.csv"))
    ds = ds[(ds["source"] == "orig_full") & (ds["direction"] == "HV->LV")]
    r4 = pd.read_csv(os.path.join(OUT, "simulink_pi_compare.csv"))
    rows = []
    for v_ds, v_r4 in V2S:
        for P in PS:
            d = ds[np.isclose(ds["V2"], v_ds, atol=1e-3) & np.isclose(ds["P"], P)]
            s = r4[np.isclose(r4["V2"], v_r4) & np.isclose(r4["P"], P)]
            if d.empty or s.empty:
                continue
            sps, mb = s[s["ctrl"] == "SPS"].iloc[0], s[s["ctrl"] == "M-B"].iloc[0]
            A, B = float(d["Ipk_SPS"].iloc[0]), float(d["Ipk_TPS"].iloc[0])
            C, D = float(sps["Ipk_simulink"]), float(mb["Ipk_simulink"])
            rows.append(dict(V2_dataset=round(v_ds, 2), V2_R4=v_r4, P=P, A_ds_SPS=A, B_ds_bestTPS=B,
                             B_D1=float(d["D1"].iloc[0]), B_D2=float(d["D2"].iloc[0]), C_sl_SPS=C, D_sl_MB=D,
                             ds_reduction=A - B, sps_overmeasure=A - C, mb_reduction=C - D, mb_minus_dsTPS=D - B,
                             clean=bool(sps["clean"]) and bool(mb["clean"]), k1=v_r4 == 95.0))
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, "ipk_decompose.csv"), index=False)

    lines = ["ipk_decompose (HV->LV, same measured power; A = dataset SPS, B = dataset grid-best TPS, "
             "C = Simulink steady SPS (R.4), D = Simulink steady M-B (R.4)); A - B = (A - C) + (C - D) + (D - B)",
             f"{'V2':>12} {'P':>6} | {'A':>6} {'B':>6} {'(D1,D2)B':>15} {'C':>6} {'D':>6} | {'A-B':>6} = "
             f"{'A-C':>6} + {'C-D':>6} + {'D-B':>6}  note"]
    for r in T.itertuples():
        v = f"{r.V2_dataset:.2f}" + ("/95" if r.k1 else "")
        note = ("k~1: V2 differs" if r.k1 else "") + ("" if r.clean else " R.4 oscillating")
        lines.append(f"{v:>12} {r.P:6.0f} | {r.A_ds_SPS:6.3f} {r.B_ds_bestTPS:6.3f} ({r.B_D1:.3f},{r.B_D2:.3f}) "
                     f"{r.C_sl_SPS:6.3f} {r.D_sl_MB:6.3f} | {r.ds_reduction:+6.3f} = {r.sps_overmeasure:+6.3f} + "
                     f"{r.mb_reduction:+6.3f} + {r.mb_minus_dsTPS:+6.3f}  {note}")
    for name, m in (("84 / 109.2 V clean", (~T["k1"]) & T["clean"]), ("all k != 1", ~T["k1"]), ("k ~ 1", T["k1"])):
        t = T[m]
        if t.empty:
            continue
        share = t["sps_overmeasure"].sum() / t["ds_reduction"].sum()
        lines.append(f"mean {name:>20} ({len(t)} pts): A-B {t.ds_reduction.mean():+.3f} = A-C "
                     f"{t.sps_overmeasure.mean():+.3f} + C-D {t.mb_reduction.mean():+.3f} + D-B "
                     f"{t.mb_minus_dsTPS.mean():+.3f} A; SPS over-measure share {100 * share:.0f} %; "
                     f"D-B max {t.mb_minus_dsTPS.max():+.3f} A; dataset reduction "
                     f"{100 * (t.ds_reduction / t.A_ds_SPS).mean():.0f} % of its SPS vs M-B "
                     f"{100 * (t.mb_reduction / t.C_sl_SPS).mean():.0f} % of steady SPS")
    lines += steady_best(T, "--py" in sys.argv)
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT, "ipk_decompose.log"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    fig(T[(~T["k1"]) & T["clean"]].reset_index(drop=True))


def fig(t):
    """(a) the four Simulink peaks at the same power: dataset rows (moving D0, hollow) vs steady state (filled);
    (b) the dataset's reduction A - B split into SPS over-measure A - C and M-B's steady reduction C - D (+ D - B)"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    names = [f"{v:g} V\n{p:.0f} W" for v, p in zip(t["V2_R4"], t["P"])]
    x = np.arange(len(t))
    f, (a1, a2) = plt.subplots(1, 2, figsize=(12.4, 4.6), constrained_layout=True,
                               gridspec_kw={"width_ratios": [1, 1.12]})

    dx = 0.13
    a1.vlines(x - dx, t["C_sl_SPS"], t["A_ds_SPS"], color=ORANGE, lw=2, alpha=0.45, zorder=2)
    a1.vlines(x + dx, t["D_sl_MB"], t["B_ds_bestTPS"], color=BLUE, lw=2, alpha=0.45, zorder=2)
    for c, m, col, hollow, lab in (("A_ds_SPS", "o", ORANGE, True, "A  dataset SPS row (D0 moving)"),
                                   ("C_sl_SPS", "o", ORANGE, False, "C  SPS, steady state (R.4)"),
                                   ("B_ds_bestTPS", "s", BLUE, True, "B  dataset best TPS row (D0 moving)"),
                                   ("D_sl_MB", "s", BLUE, False, "D  M-B, steady state (R.4)")):
        a1.scatter(x + (dx if m == "s" else -dx), t[c], s=60 if hollow else 90, marker=m, lw=2,
                   zorder=4 - hollow, facecolors="#fcfcfb" if hollow else col,
                   edgecolors=col if hollow else "#fcfcfb", label=lab)
    i = int(np.argmax(t["sps_overmeasure"]))
    a1.annotate(f"SPS row over-measured\n+{t['sps_overmeasure'][i]:.2f} A", (x[i] - dx, (t["A_ds_SPS"][i]
                + t["C_sl_SPS"][i]) / 2), xytext=(-10, 0), textcoords="offset points", ha="right", va="center",
                fontsize=8, color=INK2)
    a1.set_xticks(x, names, fontsize=8)
    a1.set_ylabel("Ipk at the same measured power [A]")
    a1.set_ylim(0, None)
    a1.set_title("(a) Simulink peaks, HV→LV: hollow = dataset rows, filled = steady state", loc="left")
    a1.legend(fontsize=8, frameon=False, loc="upper left")

    groups = list(names) + ["mean\n(7 points)"]
    xg = np.arange(len(groups))
    cols = [("ds_reduction", "#c3c2b7", "A − B  dataset 'reduction'"),
            ("sps_overmeasure", ORANGE, "A − C  SPS row over-measure"),
            ("mb_reduction", BLUE, "C − D  M-B steady-state reduction")]
    w = 0.26
    for j, (c, col, lab) in enumerate(cols):
        v = np.append(t[c].to_numpy(), t[c].mean())
        a2.bar(xg + (j - 1) * w, v, w * 0.92, color=col, label=f"{lab} (mean {v[-1]:+.2f} A)", zorder=3)
    a2.axvline(xg[-1] - 0.5, color=MUTED, lw=0.8, ls=":")
    a2.axhline(0, color="#c3c2b7", lw=0.8)
    a2.set_xticks(xg, groups, fontsize=8)
    a2.set_ylabel("Ipk difference [A]")
    a2.set_title(f"(b) A − B = (A − C) + (C − D) + (D − B);  mean D − B = {t['mb_minus_dsTPS'].mean():+.3f} A",
                 loc="left")
    a2.set_ylim(None, 1.25 * a2.get_ylim()[1])
    a2.legend(fontsize=8, frameon=False, loc="upper left")
    f.savefig(os.path.join(OUT, "fig_ipk_decompose.png"), dpi=130)
    plt.close(f)


def steady_best(T, py):
    """B in steady state for the BT_ labels (Simulink if r5r6 step 2 ran; optionally the Python plant)"""
    pts = pd.read_csv(os.path.join(OUT, "stair_steady_points.csv"))
    bt = pts[pts["label"].str.startswith("BT_")]
    if bt.empty:
        return []
    parts = [p for p in sorted(glob.glob(os.path.join(OUT, "stair_steady_part*.csv"))) if not p.endswith("_in.csv")]
    st = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True) if parts else None
    pyr = {}
    if py:
        import plant_full as pf
        r = pf.FullPlant(Ron=0.052, steps=125).steady(bt["D0"].to_numpy(float), bt["D1"].to_numpy(float),
                                                      bt["D2"].to_numpy(float), bt["V2"].to_numpy(float))
        b = bt.assign(P_LV=r["P_LV"], Ipk=r["Ipk_sampled"])
        pyr = {k: g.sort_values("D0") for k, g in b.groupby("label")}
    out = ["", "B in steady state (dataset best-TPS combos, BT_ labels; HV->LV): B = dataset row (moving D0), "
               "B_st = Simulink constant-D0 30 ms (r5r6 step 2), D = R.4 M-B",
           f"{'label':>28} | {'B':>6} {'B_st':>6} {'D':>6} | {'D-B':>6} {'D-B_st':>7} | {'C-B_st':>6}  "
           f"(C = R.4 SPS)" + ("  | B_py (Python plant; compare with Python M-B, R.8 (2))" if py else "")]
    for lab, g in bt.groupby("label", sort=False):
        P = float(lab.split("_")[2].rstrip("W"))
        V2 = float(g["V2"].iloc[0])
        r = T[np.isclose(T["V2_dataset"], round(V2, 2)) & np.isclose(T["P"], P)].iloc[0]
        Bst = np.nan
        if st is not None:
            s = st[(st["label"] == lab) & np.isfinite(st["Ipk_sim"])].sort_values("D0")
            if len(s) >= 2:
                Bst, _ = crossing(s["D0"].to_numpy(float), s["P_LV"].to_numpy(float), s["Ipk_sim"].to_numpy(float), P, +1)
        line = (f"{lab:>28} | {r.B_ds_bestTPS:6.3f} {Bst:6.3f} {r.D_sl_MB:6.3f} | {r.mb_minus_dsTPS:+6.3f} "
                f"{r.D_sl_MB - Bst:+7.3f} | {r.C_sl_SPS - Bst:+6.3f}")
        if lab in pyr:
            q = pyr[lab]
            Bpy, _ = crossing(q["D0"].to_numpy(float), q["P_LV"].to_numpy(float), q["Ipk"].to_numpy(float), P, +1)
            line += f"  | {Bpy:6.3f}"
        out.append(line + ("" if np.isfinite(Bst) else "  (Simulink steady: not run yet)"))
    return out


if __name__ == "__main__":
    sys.exit(main())
