"""
efficiency_sweep.py -- (R8, 2026-10-29) efficiency and losses of SPS / M-A / M-B in the Simulink PI steady state.

Same method as simulink_sweep.py (R.4): for each (controller, V2, |P|) the two adjacent 2.88-deg D0 levels that bracket
|P| are run with constant D0 for 30 ms (verify_steady_state.m, last 1 ms), and every quantity is interpolated linearly
in the measured delivered power. HV -> LV: delivered Pd = -P_LV, input Pin = -P_HV (both measured at the bridge-side
terminals, after the 0.05 Ohm source resistances), loss = Pin - Pd, efficiency = |P| / (|P| + loss).
Reads every Simulink run already made (out/simulink_sweep_r*.csv of R.4) plus this script's own rounds
(out/eff_sweep_r*.csv), so R.4's 100/300/600/1000 W points are reused and only 200/400/800 W need new runs.

    python -I -X utf8 Simulink/Thesis/efficiency_sweep.py next      -> out/eff_points_rN.csv (runs still needed)
    python automation/job.py submit eff1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_batch('out/eff_points_r1.csv', 'out/eff_sweep_r1.csv', 24, 0.030)"
    python -I -X utf8 Simulink/Thesis/efficiency_sweep.py compare   -> out/efficiency_compare.{csv,log}
"""
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
CTRLS = ("SPS", "M-A", "M-B", "M-S")
RUN_CTRLS = ("SPS", "M-A", "M-B")          # M-S (10-30, OPT4) is reported from its own runs, never planned here
MS_DIFF = 0.003                            # as surface_opt_check: M-S closer than this to M-B was not run (= M-B)
V2S = (84.0, 95.0, 109.2)
PS_ = (100.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)


def configs(ctrls=RUN_CTRLS):
    load = lambda t: ct.SmoothRegHDC.from_json(os.path.join(OUT, f"model_smooth_reghdc_{t}.json"))  # noqa: E731
    models = {"SPS": ct.SPS, "M-A": lambda: load("MA_neg"), "M-B": lambda: load("b3"), "M-S": lambda: load("MS")}
    ms = {c: models[c]() for c in ctrls}
    rows = []
    for c, m in ms.items():
        for V2 in V2S:
            for P in PS_:
                d1, d2 = m.predict(np.array([P]), np.array([V2]))
                # same rounding / knife-edge cap as simulink_sweep.configs, so R.4's runs match these configurations
                rows.append(dict(ctrl=c, V2=V2, P=P, D1=round(min(float(np.ravel(d1)[0]), sw.D_SAFE), 9),
                                 D2=round(min(float(np.ravel(d2)[0]), sw.D_SAFE), 9)))
    return rows


def result_files():
    """R.4 and R8 runs, plus the OPT check's 30 ms runs (same method; 10-30: they bracket some M-B / SPS configurations
    here, so `next` does not plan them twice). Not the 40 ms k = 1 runs."""
    fs = glob.glob(os.path.join(OUT, "simulink_sweep_r*.csv")) + glob.glob(os.path.join(OUT, "eff_sweep_r*.csv"))
    fs += glob.glob(os.path.join(OUT, "opt_sweep_mb_r*.csv")) + glob.glob(os.path.join(OUT, "opt_sweep_grid_r*.csv"))
    fs += glob.glob(os.path.join(OUT, "opt_sweep_ms_r*.csv"))      # 10-30: the M-S check runs (30 ms, same method)
    return [f for f in sorted(fs) if not f.endswith("_in.csv")]


def results():
    import pandas as pd
    fs = result_files()
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["Pin"] = -d["P_HV"]
    d["level"] = np.round((d["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last")


def next_round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "eff_sweep_r*.csv"))
          for m in [re.search(r"eff_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    cf = configs()
    fresh = [c for c in cf if sw.runs_of(d, c).empty]
    pts = []
    for c, (j0, j1) in zip(fresh, sw.predicted_levels(fresh) if fresh else []):
        pts += [sw.point(c, j0), sw.point(c, j1)]
    for c in cf:
        g = sw.runs_of(d, c)
        if g.empty:
            continue
        b, way = sw.bracket(g, c["P"])
        below = g[(g["Pd"] > 0) & (g["Pd"] <= c["P"])]
        if b is None and way < 0 and not below.empty:
            # |P| in a gap between non-adjacent measured levels: fill from below (the same fix as surface_opt_check.plan,
            # 10-30 -- bracket() would say "go below the lowest level", i.e. into the other power direction)
            j = int(below["level"].max())
            pts += [sw.point(c, j + 1), sw.point(c, j + 2)]
        elif b is None:
            j = int(g["level"].max() + 1) if way > 0 else int(g["level"].min() - 1)
            pts += [sw.point(c, j), sw.point(c, j + way)]
    n_b = sum(1 for c in cf if not sw.runs_of(d, c).empty and sw.bracket(sw.runs_of(d, c), c["P"])[0] is not None)
    print(f"{n_b} / {len(cf)} configurations already bracketed by existing Simulink runs")
    if pts:
        sw.write(os.path.join(OUT, f"eff_points_r{next_round_no()}.csv"), pts,
                 list(zip(d["V2"], d["D0"], d["D1"], d["D2"])))
    else:
        print("every configuration is bracketed")


def compare():
    import pandas as pd
    d = results()
    rows = []
    cf = configs(CTRLS)
    mbd = {(c["V2"], c["P"]): (c["D1"], c["D2"]) for c in cf if c["ctrl"] == "M-B"}
    for c in cf:
        g = sw.runs_of(d, c)
        b, _ = sw.bracket(g, c["P"]) if not g.empty else (None, 0)
        rec = dict(c, same_as_mb=False)
        if b is None and c["ctrl"] == "M-S":
            b1, b2 = mbd[(c["V2"], c["P"])]
            if max(abs(c["D1"] - b1), abs(c["D2"] - b2)) < MS_DIFF:      # M-S = M-B by construction: M-B's runs
                g = sw.runs_of(d, dict(c, D1=b1, D2=b2))
                b, _ = sw.bracket(g, c["P"]) if not g.empty else (None, 0)
                rec["same_as_mb"] = b is not None
        if b is not None:
            lo, hi = b
            t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
            lin = lambda k: lo[k] + t * (hi[k] - lo[k])  # noqa: E731
            loss = lin("Pin") - lin("Pd")
            rec.update(Ipk=lin("Ipk_sim"), Irms=lin("iL_rms"), Pin=lin("Pin"), loss=loss, eta=c["P"] / (c["P"] + loss),
                       iL_mean_max=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])), P_lo=lo["Pd"], P_hi=hi["Pd"])
        else:
            rec.update(Ipk=np.nan, Irms=np.nan, Pin=np.nan, loss=np.nan, eta=np.nan, iL_mean_max=np.nan)
        rows.append(rec)
    T = pd.DataFrame(rows)
    s = T[T["ctrl"] == "SPS"].set_index(["V2", "P"])
    ref = lambda k: np.array([s.loc[(v, p), k] for v, p in zip(T["V2"], T["P"])])  # noqa: E731
    T["dIpk"] = ref("Ipk") - T["Ipk"]
    T["dloss"] = ref("loss") - T["loss"]                         # positive = less loss than SPS
    T["deta_pp"] = 100.0 * (T["eta"] - ref("eta"))
    T["clean"] = (T["iL_mean_max"] <= 0.02) & (ref("iL_mean_max") <= 0.02)
    T.to_csv(os.path.join(OUT, "efficiency_compare.csv"), index=False, float_format="%.6g")

    L = ["efficiency_sweep.py: Simulink PI steady state (dataset-era model), HV -> LV, constant-D0 levels bracketing |P|, "
         "all quantities interpolated in the measured delivered power. eta = |P| / (|P| + loss), loss = Pin - Pd.",
         "dIpk, dloss = SPS - controller (positive = better than SPS); deta in percentage points; "
         "* = oscillation (|mean i_L| > 20 mA in this run or its SPS reference); '-' = not bracketed yet", ""]
    for (v, p), g in T.groupby(["V2", "P"], sort=True):
        cells = []
        for c in CTRLS:
            r = g[g["ctrl"] == c].iloc[0]
            if not np.isfinite(r["Ipk"]):
                cells.append(f"{c:4s} -")
                continue
            mark = ("" if r["clean"] else "*") + (" =M-B" if r["same_as_mb"] else "")
            cells.append(f"{c:4s} Ipk {r['Ipk']:5.3f} Irms {r['Irms']:5.3f} loss {r['loss']:6.2f} W eta {100 * r['eta']:6.2f} %"
                         + ("" if c == "SPS" else f" (dI {r['dIpk']:+.3f} A, dloss {r['dloss']:+.2f} W, "
                                                  f"deta {r['deta_pp']:+.2f} pp)") + mark)
        L.append(f"V2 {v:5.1f} |P| {p:5.0f} W: " + " | ".join(cells))
    L.append("(M-S, 10-30: from the OPT4 check runs; '=M-B' = M-S within 0.003 of M-B, not run, M-B's runs used)")
    L.append("")
    for c in CTRLS[1:]:
        g = T[(T["ctrl"] == c) & T["clean"] & np.isfinite(T["eta"])]
        if len(g):
            L.append(f"summary {c} (clean, n = {len(g)}): mean dIpk {g['dIpk'].mean():+.3f} A, mean dloss {g['dloss'].mean():+.2f} W, "
                     f"mean deta {g['deta_pp'].mean():+.3f} pp, worst deta {g['deta_pp'].min():+.3f} pp")
    both = T[T["ctrl"].isin(["M-B", "M-S"])].pivot_table(index=["V2", "P"], columns="ctrl", values=["clean", "dIpk", "deta_pp"],
                                                          aggfunc="first")
    ok = both[("clean", "M-B")].astype(bool) & both[("clean", "M-S")].astype(bool) & both[("dIpk", "M-S")].notna() \
        & both[("dIpk", "M-B")].notna()
    if ok.any():
        b = both[ok]
        L.append(f"M-B vs M-S on the same clean configurations (n = {int(ok.sum())}): mean dIpk {b[('dIpk', 'M-B')].mean():+.3f} / "
                 f"{b[('dIpk', 'M-S')].mean():+.3f} A, mean deta {b[('deta_pp', 'M-B')].mean():+.3f} / {b[('deta_pp', 'M-S')].mean():+.3f} pp")
    nb = int(np.isfinite(T.loc[T["ctrl"].isin(RUN_CTRLS), "eta"]).sum())
    L.append(f"bracketed configurations: {nb} / {int(T['ctrl'].isin(RUN_CTRLS).sum())} (SPS, M-A, M-B); "
             f"M-S {int(np.isfinite(T.loc[T['ctrl'] == 'M-S', 'eta']).sum())} / {int((T['ctrl'] == 'M-S').sum())} "
             f"({int(T['same_as_mb'].sum())} of them = M-B)")
    print("\n".join(L))
    open(os.path.join(OUT, "efficiency_compare.log"), "w", encoding="utf-8").write("\n".join(L) + "\n")
    fig(T)
    return T


def fig(T):
    """Fig. 26-28 format of the reference ([1]): Ipk and efficiency against |P|, one column per V2 -> out/fig_efficiency.png.
    Hollow marker = oscillating run (|mean i_L| > 20 mA, here or in its SPS reference); M-S drawn without a marker where it
    equals M-B (not run)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, YELLOW, _style  # noqa: F401
    _style(plt)
    sty = {"SPS": (MUTED, "o", "--"), "M-A": (ORANGE, "s", "-"), "M-B": (BLUE, "^", "-"), "M-S": (YELLOW, "D", "-")}
    f, ax = plt.subplots(2, len(V2S), figsize=(13.0, 6.6), sharex=True, constrained_layout=True)
    for j, v in enumerate(V2S):
        for c in CTRLS:
            full = T[(T["ctrl"] == c) & (T["V2"] == v)].set_index("P").reindex(PS_)   # NaN rows break the line at gaps
            g = full[np.isfinite(full["Ipk"])].reset_index()
            if g.empty:
                continue
            col, mk, ls = sty[c]
            for i, (key, scale) in enumerate((("Ipk", 1.0), ("eta", 100.0))):
                a = ax[i, j]
                a.plot(full.index, scale * full[key].astype(float), ls=ls, color=col, lw=1.6, zorder=2 + CTRLS.index(c))
                run = ~g["same_as_mb"].astype(bool)
                cl = g["clean"].astype(bool)
                a.plot(g.loc[run & cl, "P"], scale * g.loc[run & cl, key], mk, color=col, ms=5.5, mec="#fcfcfb", mew=1.2,
                       ls="none", zorder=3 + CTRLS.index(c))
                a.plot(g.loc[run & ~cl, "P"], scale * g.loc[run & ~cl, key], mk, color=col, ms=5.5, mfc="#fcfcfb", mew=1.3,
                       ls="none", zorder=3 + CTRLS.index(c))
        k = 4 * v / 380.0
        ax[0, j].set_title(f"V2 = {v:g} V (k = {k:.2f})")
        ax[1, j].set_xlabel("|P| [W]  (HV -> LV)")
    ax[0, 0].set_ylabel("Ipk [A]")
    ax[1, 0].set_ylabel("efficiency [%]")
    for c in CTRLS:
        col, mk, ls = sty[c]
        ax[0, 0].plot([], [], ls=ls, marker=mk, color=col, lw=1.6, ms=5.5, label=c)
    ax[0, 0].plot([], [], "o", mfc="#fcfcfb", mec=INK2, ms=5.5, label="hollow = oscillating run")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="upper left")
    f.suptitle("Simulink steady state (dataset-era model, 100 ns grid): peak current and efficiency vs |P|. "
               "M-S without marker = same as M-B (not run).", fontsize=9.5, color=INK2)
    f.savefig(os.path.join(OUT, "fig_efficiency.png"), dpi=130)
    plt.close(f)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "compare"
    {"next": next_round, "compare": compare}[cmd]()
