"""
simulink_sweep.py -- R2(3): the Simulink PI steady state by constant-D0 runs on the controller's 2.88 deg phase grid.

Why not the closed loop itself (verify_pi_closed_loop.m, pilots out/simulink_pi_pilot*.csv): the dataset-era controller
places every edge on its 100 ns sample grid (2.88 deg, ~160 W per step at 1 kW), so its PI can only dither between two
adjacent phase levels; the dither makes successive periods asymmetric (mean i_L 0.1-0.5 A) and raised the SPS peak by
1.2-3 A over the constant-D0 steady state at the same power, for every gain tried (the model's own Kp 20 / Ki 490000
chatter at 16-18 A). An MCU timer (HRTIM 184 ps) has no such grid. The dither-free PI steady state is the steady state
at the D0 that delivers the commanded power: here it is interpolated (linear in the measured power) between the two
adjacent grid levels whose constant-D0 steady states bracket the command, from verify_steady_state.m runs (the T6 path:
30 ms, last 1 ms). Levels are commanded half a step below the grid point (edges are rounded UP to the grid).

  make     round 1: for each configuration the two levels predicted (plant_full, quantised, Ron 40 mOhm) to bracket |P*|
           -> out/sweep_points_r1.csv
  next     after a round: levels one further out where the measured powers do not bracket |P*| -> out/sweep_points_r<k>.csv
  compare  interpolated Ipk per configuration vs SPS + the analytic and plant yardsticks -> out/simulink_pi_compare.{csv,log},
           out/fig_simulink_pi_compare.png
Configurations: V2 {84, 95, 109.2} x |P| {100, 300, 600, 1000} W x {SPS, M-A, M-B0, M-B}, HV -> LV.

    python -I -X utf8 Simulink/Thesis/simulink_sweep.py make
    python automation/job.py submit sw1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_state('out/sweep_points_r1.csv', 'out/simulink_sweep_r1.csv')"
    python -I -X utf8 Simulink/Thesis/simulink_sweep.py next      (if needed), then compare
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import plant_full as pf  # noqa: E402

OUT = os.path.join(HERE, "out")
G = 2.88                                   # deg, the controller's 100 ns grid at 80 kHz
CTRLS = ("SPS", "M-A", "M-B0", "M-B")
# The controller's phase counter accumulates 2.88 deg in floating point and a leg is on while mod(cp - p, 360) is in
# [0, 180). An edge phase p exactly ON the grid is a knife edge: rounding noise in cp toggles that sample from period to
# period (62/63-sample pulses), which excites the series-C / L mode (mean i_L 0.2-0.5 A over 1 ms) and inflates Ipk.
# Round 1 commanded D0 = j g - g/2, so with D2 = 0.5 the secondary edge D0 + 180 fell exactly on the grid (SPS at
# 95 V / D0 10.08: 1.659 A vs 1.256 A for D2 = 0.49984 at the same D0). Every duty is therefore capped at D_SAFE:
# 3.6e-5 deg below SPS, physically SPS, but off the knife edge (the same rounding direction every period).
D_SAFE = 0.4999999


def configs():
    load = lambda t: ct.SmoothRegHDC.from_json(os.path.join(OUT, f"model_smooth_reghdc_{t}.json"))  # noqa: E731
    ms = {"SPS": ct.SPS(), "M-A": load("MA_neg"), "M-B0": load("a"), "M-B": load("b3")}
    rows = []
    for c, m in ms.items():
        for V2 in (84.0, 95.0, 109.2):
            for P in (100.0, 300.0, 600.0, 1000.0):
                d1, d2 = m.predict(np.array([P]), np.array([V2]))
                rows.append(dict(ctrl=c, V2=V2, P=P, D1=round(min(float(np.ravel(d1)[0]), D_SAFE), 9),
                                 D2=round(min(float(np.ravel(d2)[0]), D_SAFE), 9)))
    return rows


def write(path, pts, done=()):
    """unique (V2, D0, D1, D2) runs only (SPS levels are shared between power targets), minus those already run."""
    import csv
    seen = set(done)
    uniq = []
    for p in pts:
        k = (p["V2"], p["D0"], p["D1"], p["D2"])
        if k not in seen:
            seen.add(k)
            uniq.append(p)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2"])
        w.writeheader()
        for p in uniq:
            w.writerow(p)
    print(f"wrote {path}: {len(uniq)} runs ({len(pts)} before removing duplicates)")


def point(c, j):
    return dict(label=f"{c['ctrl']} {c['P']:.0f}W {c['V2']:g}V L{j}", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"],
                level=j, D0=round(j * G - 0.5 * G, 6), D1=c["D1"], D2=c["D2"])


def make():
    cf = configs()
    pts = []
    for c, (j0, j1) in zip(cf, predicted_levels(cf)):
        pts += [point(c, j0), point(c, j1)]
    write(os.path.join(OUT, "sweep_points_r1.csv"), pts)


def result_files():
    """round outputs incl. verify_steady_batch chunks (..._partNNN.csv), not its transient inputs (..._in.csv)."""
    return [f for f in sorted(glob.glob(os.path.join(OUT, "simulink_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results():
    import pandas as pd
    fs = result_files()
    if not fs:
        sys.exit("no out/simulink_sweep_r*.csv yet")
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["level"] = np.round((d["D0"] + 0.5 * G) / G).astype(int)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last"), len(fs)


def runs_of(d, c):
    """every valid run of this configuration's (V2, D1, D2), any level. Rounds 1-2 wrote duties with 6 decimals, so
    6e-7 is a match; but a run with a duty of exactly 0.5 sat on the knife edge (see D_SAFE) and is never used."""
    m = (np.isclose(d["V2"], c["V2"]) & (np.abs(d["D1"] - c["D1"]) <= 6e-7) & (np.abs(d["D2"] - c["D2"]) <= 6e-7)
         & (d["D1"] < 0.5) & (d["D2"] < 0.5))
    return d[m].sort_values("level")


def next_round_no():
    import re
    ns = [int(m.group(1)) for f in result_files() for m in [re.search(r"simulink_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def predicted_levels(cf):
    """for each configuration the two adjacent grid levels the plant (quantised, Ron 40 mOhm) predicts to bracket |P|."""
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True)
    P = np.array([c["P"] for c in cf])
    V = np.array([c["V2"] for c in cf])
    D1 = np.array([c["D1"] for c in cf])
    D2 = np.array([c["D2"] for c in cf])
    D0i = ps.operating_point(P, D1, D2, V)["D0"]
    js = np.arange(-4, 9)                                       # levels around the ideal crossing
    J = np.floor(np.nan_to_num(D0i) / G)[:, None] + js[None, :]
    n, k = J.shape
    r = pl.steady_fixed((J * G - 0.5 * G).ravel(), np.repeat(D1, k), np.repeat(D2, k), np.full(n * k, 380.0),
                        np.repeat(V, k))
    Pd = (-r["P_LV"]).reshape(n, k)                             # delivered HV -> LV
    out = []
    for i, c in enumerate(cf):
        above = np.where(Pd[i] >= c["P"])[0]
        a = max(above[0] if above.size else k - 1, 1)
        out.append((int(J[i, a - 1]), int(J[i, a])))
    return out


def bracket(g, P):
    """(lo, hi) adjacent-level rows whose delivered powers bracket P, else None + the direction to extend."""
    Pd = g["Pd"].to_numpy()
    lv = g["level"].to_numpy()
    for i in range(len(g) - 1):
        if lv[i + 1] == lv[i] + 1 and Pd[i] <= P <= Pd[i + 1]:
            return (g.iloc[i], g.iloc[i + 1]), 0
    return None, (+1 if Pd.max() < P else -1)


def next_round():
    d, nr = results()
    pts = []
    cf = configs()
    fresh = [c for c in cf if runs_of(d, c).empty]              # e.g. the D_SAFE versions of the knife-edge runs
    for c, (j0, j1) in zip(fresh, predicted_levels(fresh) if fresh else []):
        pts += [point(c, j0), point(c, j1)]
    for c in cf:
        g = runs_of(d, c)
        if g.empty:
            continue
        b, way = bracket(g, c["P"])
        if b is None:
            j = int(g["level"].max() + 1) if way > 0 else int(g["level"].min() - 1)
            pts += [point(c, j), point(c, j + way)]
    # runs that failed in an earlier round (e.g. the 4 stopped at 15:31 on 10-26) are run again
    import pandas as pd
    raw = pd.concat([pd.read_csv(f) for f in result_files()])
    bad = raw[raw["error"].notna() & (raw["error"].astype(str).str.strip() != "")] if "error" in raw else raw.iloc[:0]
    for _, r in bad.iterrows():
        if any(np.isclose(r["V2"], c["V2"]) and abs(r["D1"] - c["D1"]) < 1e-9 and abs(r["D2"] - c["D2"]) < 1e-9
               for c in cf):
            pts.append(dict(label=r["label"], ctrl=r["ctrl"], V2=r["V2"], P_target=r["P_target"],
                            level=int(r["level"]), D0=r["D0"], D1=r["D1"], D2=r["D2"]))
    done = list(zip(d["V2"], d["D0"], d["D1"], d["D2"]))
    if pts:
        write(os.path.join(OUT, f"sweep_points_r{next_round_no()}.csv"), pts, done)
    else:
        print("every configuration is bracketed")


def compare():
    import pandas as pd
    d, _ = results()
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=False)
    rows = []
    for c in configs():
        g = runs_of(d, c)
        b, _ = bracket(g, c["P"]) if not g.empty else (None, 0)
        rec = dict(c)
        if b is not None:
            lo, hi = b
            t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
            rec.update(Ipk_simulink=lo["Ipk_sim"] + t * (hi["Ipk_sim"] - lo["Ipk_sim"]), P_lo=lo["Pd"], P_hi=hi["Pd"],
                       Ipk_lo=lo["Ipk_sim"], Ipk_hi=hi["Ipk_sim"], iL_mean_max=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])),
                       settle_max=max(abs(lo["Ipk_at_30ms"] - lo["Ipk_at_25ms"]), abs(hi["Ipk_at_30ms"] - hi["Ipk_at_25ms"])))
        else:
            rec.update(Ipk_simulink=np.nan)
        rows.append(rec)
    T = pd.DataFrame(rows)
    P, V, D1, D2 = (T[k].to_numpy(float) for k in ("P", "V2", "D1", "D2"))
    T["Ipk_analytic"] = ps.operating_point(P, D1, D2, V)["Ipk"]
    r = pl.operating_point(P, D1, D2, V)
    T["Ipk_plant"] = np.where(r["ok"], r["Ipk"], np.nan)
    T["plant_cycle"] = r["cycle"]
    for y in ("analytic", "plant", "simulink"):
        s = T[T["ctrl"] == "SPS"].set_index(["V2", "P"])[f"Ipk_{y}"]
        ref = np.array([s.loc[(v, p)] for v, p in zip(T["V2"], T["P"])])
        T[f"dI_{y}"] = ref - T[f"Ipk_{y}"]
        T[f"dI_pct_{y}"] = 100.0 * T[f"dI_{y}"] / ref
    # 'clean': no sustained low-frequency oscillation in the two bracketing runs of this configuration AND of its SPS
    # reference (|mean i_L| over the last 1 ms <= 20 mA; with series caps a settled current has zero mean)
    osc = T.get("iL_mean_max", pd.Series(np.nan, index=T.index))
    sref = T[T["ctrl"] == "SPS"].set_index(["V2", "P"])["iL_mean_max"]
    s_osc = np.array([sref.loc[(v, p)] for v, p in zip(T["V2"], T["P"])])
    T["clean"] = (osc <= 0.02) & (s_osc <= 0.02)
    T.to_csv(os.path.join(OUT, "simulink_pi_compare.csv"), index=False, float_format="%.5g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("Simulink PI steady state (constant-D0 grid levels bracketing |P*|, Ipk interpolated in the measured power) vs "
        "the analytic model and the Python plant; HV -> LV; dI = Ipk(SPS) - Ipk(controller)")
    for (v, p), g in T.groupby(["V2", "P"], sort=False):
        out(f"V2 {v:5.1f} |P| {p:5.0f} W:")
        for _, r_ in g.iterrows():
            br = (f"  [P {r_['P_lo']:.0f}..{r_['P_hi']:.0f} W, Ipk {r_['Ipk_lo']:.3f}..{r_['Ipk_hi']:.3f}, |mean i| "
                  f"{r_['iL_mean_max']:.3f}]" if np.isfinite(r_["Ipk_simulink"]) else "  [not bracketed]")
            out(f"   {r_['ctrl']:5s} D {r_['D1']:.3f}/{r_['D2']:.3f} | " + " | ".join(
                f"{y[:4]} {r_[f'Ipk_{y}']:.3f} ({r_[f'dI_{y}']:+.3f}, {r_[f'dI_pct_{y}']:+5.1f} %)"
                for y in ("analytic", "plant", "simulink")) + br)
    for title, sel in (("all 12 points", np.ones(len(T), bool)),
                       ("points without oscillation in Simulink (|mean i_L| <= 20 mA, controller and SPS)", T["clean"])):
        out(f"\nsummary, {title}: mean dI [A] / mean dI [%] / worst dI [A] / points > 5 mA worse than SPS (n)")
        for c in CTRLS[1:]:
            g = T[(T["ctrl"] == c) & sel]
            out(f"  {c:5s} " + " | ".join(
                f"{y}: {g[f'dI_{y}'].mean():+.3f} / {g[f'dI_pct_{y}'].mean():+5.1f} % / {g[f'dI_{y}'].min():+.3f} / "
                f"{int((g[f'dI_{y}'] < -0.005).sum())} ({int(g[f'dI_{y}'].notna().sum())})"
                for y in ("analytic", "plant", "simulink")))
    open(os.path.join(OUT, "simulink_pi_compare.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(T)


def fig(T):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, INK2, ORANGE, _style
    _style(plt)
    col = {"M-A": ORANGE, "M-B0": AQUA, "M-B": BLUE}
    lab = {"M-A": "M-A (existing dataset target)", "M-B0": "M-B0 (steady-state surface)", "M-B": "M-B (+ physics E2E)"}
    Vs = sorted(T["V2"].unique())
    # where the SPS reference run itself oscillates (109.2 V 600 W: 5.15 A vs analytic 3.43 A), every Simulink dI there
    # is measured against a wrong baseline: leave a gap instead of letting that point set the axis (2026-10-28)
    sps = T[T["ctrl"] == "SPS"]
    bad = {(v, p) for v, p, m in zip(sps["V2"], sps["P"], sps["iL_mean_max"]) if m > 0.02}
    f, ax = plt.subplots(2, len(Vs), figsize=(4.6 * len(Vs), 7.2), constrained_layout=True, sharex=True, squeeze=False)
    for j, v in enumerate(Vs):
        for c in ("M-A", "M-B0", "M-B"):
            g = T[(T["V2"] == v) & (T["ctrl"] == c)].sort_values("P")
            cl = g["clean"].astype(bool).to_numpy() if "clean" in g else np.ones(len(g), bool)
            keep = np.array([(v, p) not in bad for p in g["P"]])
            for r, (yk, ya) in enumerate((("dI_simulink", "dI_analytic"), ("dI_pct_simulink", "dI_pct_analytic"))):
                ax[r, j].plot(g["P"], g[yk].where(keep), "-", color=col[c], lw=2, label=lab[c] if r == 0 else None)
                ax[r, j].plot(g["P"][cl & keep], g[yk][cl & keep], "o", color=col[c], ms=6)
                ax[r, j].plot(g["P"][~cl & keep], g[yk][~cl & keep], "o", mfc="white", mec=col[c], mew=1.5, ms=7)
                ax[r, j].plot(g["P"], g[ya], ":", color=col[c], lw=1.2)
        for r in (0, 1):
            ax[r, j].axhline(0.0, color="#c3c2b7", lw=1)
        for p in sorted(p for vv, p in bad if vv == v):
            for r in (0, 1):
                ax[r, j].axvline(p, color="#c3c2b7", lw=1, ls="--", zorder=0)
            ax[0, j].text(p, 0.98, "SPS reference oscillates:\nno Simulink value", transform=ax[0, j].get_xaxis_transform(),
                          ha="center", va="top", fontsize=7.5, color=INK2)
        ax[0, j].set_title(f"V2 = {v:g} V (k = {4 * v / 380:.3f}), HV -> LV")
        ax[1, j].set_xlabel("|P| [W]")
    ax[0, 0].set_ylabel("Ipk reduction vs SPS [A]\nsolid: Simulink PI steady state, dotted: analytic")
    ax[1, 0].set_ylabel("Ipk reduction vs SPS [%]")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="lower left")
    ax[1, -1].text(0.98, 0.97, "hollow: the controller's Simulink run kept a low-frequency\noscillation (|mean i_L| > 20 mA)",
                   transform=ax[1, -1].transAxes, ha="right", va="top", fontsize=7.5, color=INK2)
    f.savefig(os.path.join(OUT, "fig_simulink_pi_compare.png"), dpi=130)
    plt.close(f)


def fig_only():
    import pandas as pd
    fig(pd.read_csv(os.path.join(OUT, "simulink_pi_compare.csv")))


if __name__ == "__main__":
    {"make": make, "next": next_round, "compare": compare, "fig": fig_only}[sys.argv[1] if len(sys.argv) > 1 else "make"]()
