"""
staircase_compare.py -- R5 (2026-10-27): the user's dataset experiment re-run in Simulink, with and without the D0
interpolation, against Simulink steady state. Simulink only: no analytic model, no surrogate.

Inputs (all Simulink, dataset-era model git 35d5a78):
  out/staircase_check.csv          staircase_check.m interp = 1: Generate_TPS_Dataset.m as written (D0 block interpolates)
  out/staircase_check_interp0.csv  staircase_check.m interp = 0: same run, D0 block Interpolate off (a true staircase)
  out/checkpoint_20260316_021229.csv   the user's checkpoint (= dataset), from inspect_checkpoint.m
  out/stair_steady_part*.csv       verify_steady_batch.m on out/stair_steady_points.csv: constant D0 for 30 ms at the
                                   SAME (V2, D1, D2, D0 label), last 1 ms (the steady-state reference)

(a) does interp = 1 reproduce the checkpoint rows?  (b) are the interp = 0 windows the steady state?
(c) the user's question: at the SAME measured power, how much lower is Ipk with the TPS combination than with SPS
    -- in the dataset (interp = 1), with a true staircase (interp = 0), and in steady state?
Same-power comparison: along each run's D0 sweep, the rising crossing of the target power closest to D0 = 0 (what a
PI started at D0 = 0 reaches first), Ipk linearly interpolated in power between the two bracketing steps.
Power sign as in the dataset: P_LV > 0 <=> D0 < 0 ("P>0"), P_LV < 0 <=> D0 > 0 ("P<0" = HV->LV, the main text).

    python -I -X utf8 Simulink/Thesis/staircase_compare.py points   -> out/stair_steady_points.csv
    python -I -X utf8 Simulink/Thesis/staircase_compare.py          -> out/staircase_compare.{log,csv}
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "out")
STEADY_STEPS = (9, 11, 13, 15, 17, 19, 20, 21, 23, 25, 27, 29, 31)     # D0 labels -31.6 ... +31.6 deg
TARGETS = (100.0, 150.0, 300.0, 600.0, 1000.0)


def load():
    import pandas as pd
    s1 = pd.read_csv(os.path.join(OUT, "staircase_check.csv"))
    s1 = s1[s1["interp"] == 1]
    f0 = os.path.join(OUT, "staircase_check_interp0.csv")
    s0 = pd.read_csv(f0) if os.path.isfile(f0) else None
    parts = sorted(glob.glob(os.path.join(OUT, "stair_steady_part*.csv")))
    parts = [p for p in parts if not p.endswith("_in.csv")]
    st = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True) if parts else None
    if st is not None:
        st = st.rename(columns={"label": "tag", "Ipk_sim": "Ipk"})
    return s1, s0, st


def points():
    s1, _, _ = load()
    p = s1[s1["step"].isin(STEADY_STEPS)][["tag", "V2", "D1", "D2", "step", "D0_label"]]
    p = p.rename(columns={"tag": "label", "D0_label": "D0"})[["label", "V2", "D0", "D1", "D2", "step"]]
    f = os.path.join(OUT, "stair_steady_points.csv")
    p.to_csv(f, index=False, float_format="%.15g")
    print(f"wrote {f} ({len(p)} points: {p['label'].nunique()} combos x {len(STEADY_STEPS)} D0 labels)")


def crossing(D0, P, I, target, s):
    """Ipk at the rising crossing of |P| = target (direction s: -1 -> D0 < 0, P_LV > 0; +1 -> D0 > 0, P_LV < 0)
    closest to D0 = 0, linear in power between the bracketing steps. NaN if never reached."""
    o = np.argsort(s * D0)
    x, p, i = s * D0[o], -s * P[o], I[o]
    best = (np.inf, np.nan, np.nan)
    for k in range(1, len(x)):
        if p[k - 1] < target <= p[k]:
            w = (target - p[k - 1]) / (p[k] - p[k - 1])
            xc = x[k - 1] + w * (x[k] - x[k - 1])
            if abs(xc) < best[0]:
                best = (abs(xc), i[k - 1] + w * (i[k] - i[k - 1]), s * xc)
    return best[1], best[2]


def main():
    import pandas as pd
    s1, s0, st = load()
    ck = pd.read_csv(os.path.join(OUT, "checkpoint_20260316_021229.csv"))
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    tags = list(dict.fromkeys(s1["tag"]))
    out("R5 staircase re-run (Generate_TPS_Dataset.m experiment: 44 ms, 39 D0 steps of 1 ms, 0.1 ms window)")
    out(f"combos: {', '.join(tags)}")

    # (a) interp = 1 vs the checkpoint
    out("\n(a) interp = 1 (D0 block as in the model: Interpolate on) vs the checkpoint rows, same (V2, D1, D2, D0 label)")
    for t in tags:
        g = s1[s1["tag"] == t].sort_values("step")
        V2, D1, D2 = g.iloc[0][["V2", "D1", "D2"]]
        m = ck[np.isclose(ck["V2"], V2) & np.isclose(ck["D1"], D1) & np.isclose(ck["D2"], D2)].sort_values("D0")
        if len(m) != len(g):
            out(f"  {t:24s} checkpoint rows {len(m)} != {len(g)}")
            continue
        dI = np.abs(g["Ipk"].to_numpy() - m["I_pk"].to_numpy()).max()
        dP = np.abs(g["P_LV"].to_numpy() - m["P_act"].to_numpy()).max()
        dH = np.abs(g["P_HV"].to_numpy() - m["P_act_HV"].to_numpy()).max()
        out(f"  {t:24s} V2 {V2:6.2f} D1 {D1:.4f} D2 {D2:.4f}: max|dIpk| {dI:.2e} A, max|dP_LV| {dP:.2e} W, "
            f"max|dP_HV| {dH:.2e} W")

    srcs = [("dataset(interp=1)", s1)]
    if s0 is not None:
        srcs.append(("staircase(interp=0)", s0))
    else:
        out("\n(interp = 0 rows not available yet)")
    if st is not None:
        srcs.append(("steady(const D0 30 ms)", st))
    else:
        out("(steady-state rows not available yet)")

    # (b) the staircase windows vs steady state at the same D0 label
    if st is not None:
        out("\n(b) window values vs Simulink steady state at the SAME (V2, D1, D2, D0 label); diff = window - steady")
        out(f"  {'combo':24s} {'source':20s} {'n':>3s} | {'dIpk mean':>9s} {'max|dIpk|':>9s} {'worst at D0':>11s} | "
            f"{'dP mean':>8s} {'max|dP|':>8s}")
        for t in tags:
            gs = st[st["tag"] == t].set_index("step")
            for name, src in srcs[:-1]:
                g = src[(src["tag"] == t) & src["step"].isin(gs.index)].set_index("step").loc[gs.index]
                dI = g["Ipk"].to_numpy() - gs["Ipk"].to_numpy()
                dP = g["P_LV"].to_numpy() - gs["P_LV"].to_numpy()
                k = int(np.nanargmax(np.abs(dI)))
                out(f"  {t:24s} {name:20s} {len(dI):3d} | {np.nanmean(dI):+9.3f} {np.nanmax(np.abs(dI)):9.3f} "
                    f"{gs['D0'].to_numpy()[k]:+11.2f} | {np.nanmean(dP):+8.1f} {np.nanmax(np.abs(dP)):8.1f}")
                for j, stp in enumerate(gs.index):
                    rows.append(dict(part="b", tag=t, source=name, step=int(stp), D0=gs["D0"].iloc[j],
                                     Ipk=g["Ipk"].iloc[j], P_LV=g["P_LV"].iloc[j], Ipk_steady=gs["Ipk"].iloc[j],
                                     P_steady=gs["P_LV"].iloc[j], dIpk=dI[j], dP=dP[j]))
        out("  steady settling (Ipk at 30 ms - at 20 ms): max %.4f A" %
            np.nanmax(np.abs(st["Ipk"] - st["Ipk_at_20ms"])) if "Ipk_at_20ms" in st else "")

    # (c) TPS vs SPS at the same measured power
    out("\n(c) Ipk(SPS) - Ipk(TPS combo) at the same measured power (positive = TPS lower); same-power crossing "
        "closest to D0 = 0")
    hdr = " ".join(f"{n:>22s}" for n, _ in srcs)
    out(f"  {'V2':>6s} {'TPS combo':24s} {'dir':>4s} {'|P|':>5s} | {hdr}")
    for V2 in sorted(s1["V2"].unique()):
        sps = [t for t in tags if t.startswith("SPS") and np.isclose(s1[s1["tag"] == t]["V2"].iloc[0], V2)]
        tps = [t for t in tags if not t.startswith("SPS") and np.isclose(s1[s1["tag"] == t]["V2"].iloc[0], V2)]
        if not sps:
            continue
        for t in tps:
            for s, lab in ((+1, "P<0"), (-1, "P>0")):
                for P in TARGETS:
                    cells = []
                    for name, src in srcs:
                        a = src[src["tag"] == sps[0]]
                        b = src[src["tag"] == t]
                        D0c = "D0" if "D0" in a.columns else "D0_label"
                        Ia, xa = crossing(a[D0c].to_numpy(float), a["P_LV"].to_numpy(float), a["Ipk"].to_numpy(float), P, s)
                        Ib, xb = crossing(b[D0c].to_numpy(float), b["P_LV"].to_numpy(float), b["Ipk"].to_numpy(float), P, s)
                        d = Ia - Ib
                        cells.append(f"{Ia:6.2f}-{Ib:5.2f}={d:+5.2f}" if np.isfinite(d) else f"{'n/a':>22s}")
                        rows.append(dict(part="c", tag=t, source=name, V2=V2, direction=lab, P=P, Ipk_SPS=Ia, Ipk_TPS=Ib,
                                         dI=d, dI_pct=100 * d / Ia if np.isfinite(d) else np.nan, D0_SPS=xa, D0_TPS=xb))
                    out(f"  {V2:6.1f} {t:24s} {lab:>4s} {P:5.0f} | " + " ".join(f"{c:>22s}" for c in cells))

    pd.DataFrame(rows).to_csv(os.path.join(OUT, "staircase_compare.csv"), index=False)
    open(os.path.join(OUT, "staircase_compare.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(s1, s0, st, tags)
    print(f"wrote out/staircase_compare.{{log,csv}}, out/fig_staircase_compare.png")


def fig(s1, s0, st, tags):
    """Ipk vs measured power along the D0 > 0 sweep (HV->LV), SPS vs the TPS combinations; top: the dataset's own
    experiment (D0 interpolated), bottom: true staircase + steady state at the same D0 labels"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, MUTED, ORANGE, _style
    _style(plt)
    V2s = sorted(s1["V2"].unique())
    f, ax = plt.subplots(2, len(V2s), figsize=(4.8 * len(V2s), 7.6), constrained_layout=True, sharex=True,
                         sharey=True, squeeze=False)
    for j, V2 in enumerate(V2s):
        here = [t for t in tags if np.isclose(s1[s1["tag"] == t]["V2"].iloc[0], V2)]
        sps = [t for t in here if t.startswith("SPS")]
        tps = sorted(t for t in here if not t.startswith("SPS"))
        colours = dict(zip(sps + tps, [BLUE, ORANGE, AQUA]))
        for t in sps + tps:
            g1 = s1[(s1["tag"] == t)]
            D1, D2 = g1["D1"].iloc[0], g1["D2"].iloc[0]
            lab = "SPS" if t.startswith("SPS") else f"TPS D1 {D1:.3f}, D2 {D2:.3f}"
            # HV->LV rows near the D0 > 0 branch (dead time moves the zero-power point to D0 < 0 at light load)
            g1 = g1[(g1["D0_label"] >= -16) & (g1["P_LV"] < 0)].sort_values("D0_label")
            ax[0, j].plot(-g1["P_LV"], g1["Ipk"], "-", marker="x", ms=5, color=colours[t], lw=1.8, label=lab)
            if s0 is not None:
                g0 = s0[(s0["tag"] == t) & (s0["D0_label"] >= -16) & (s0["P_LV"] < 0)].sort_values("D0_label")
                ax[1, j].plot(-g0["P_LV"], g0["Ipk"], "-", marker="o", ms=4, color=colours[t], lw=1.8,
                              label=lab + " (staircase)" if j == 0 else None)
            if st is not None:
                gs = st[(st["tag"] == t) & (st["D0"] >= -16) & (st["P_LV"] < 0)].sort_values("D0")
                ax[1, j].plot(-gs["P_LV"], gs["Ipk"], linestyle="none", marker="D", ms=8, mfc="none", mew=1.8,
                              color=colours[t], label="steady state (30 ms const D0)" if (j == 0 and t == sps[0]) else None)
        ax[0, j].set_title(f"V2 = {V2:g} V (k = {4 * V2 / 380:.3f}), HV -> LV\ndataset experiment as run (D0 interpolated)")
        ax[1, j].set_title("same run, D0 interpolation off (true 1 ms staircase)\n+ Simulink steady state (diamonds)")
        ax[1, j].set_xlabel("measured |P_LV| [W] along the D0 > 0 sweep")
        ax[0, j].set_xlim(0, 1300)
        ax[0, j].set_ylim(0, None)
    ax[0, 0].set_ylabel("Ipk [A] (0.1 ms window, max|i_L|)")
    ax[1, 0].set_ylabel("Ipk [A]")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="upper left")
    ax[1, 0].legend(fontsize=7.5, frameon=False, loc="upper left")
    ax[1, -1].text(0.98, 0.03, "Simulink only (dataset-era model 35d5a78)", transform=ax[1, -1].transAxes,
                   ha="right", va="bottom", fontsize=7.5, color=MUTED)
    f.savefig(os.path.join(OUT, "fig_staircase_compare.png"), dpi=130)
    plt.close(f)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "points":
        points()
    else:
        main()
