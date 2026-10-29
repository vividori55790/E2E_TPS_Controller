"""
k1_map.py -- how far does the k = 1 light-load effect of OPT2 reach? (OPT4 groundwork, 2026-10-30)

OPT2 (surface_opt_check.py, rule (ii) 'real'): at V2 = 95.0 V the lowest power-carrying D0 level (4.32 deg) gives
SPS 0.738 A at 73 W but D1 = D2 = 0.35-0.45 only 0.415 A at 65-72 W; one level up (7.20 deg) SPS is best (0.840 A at
250 W vs 1.24 A). The analytic model and the Python plant have no such SPS floor (95 V 100 W: 0.27 / 0.28 A, R.4).
Before a retrained surface may use it, its extent in V2 and |P| is measured here -- Simulink only, same method as OPT2
(constant D0 for 40 ms, last 1 ms, max |i_L|), no rule, a map:
  V2 {93, 94, 95, 96, 97} V x {SPS, D1 = D2 = 0.475, 0.45, 0.40} x D0 levels 1-4 (1.44 ... 10.08 deg, HV -> LV)
  + SPS at V2 {90, 100} V, levels 1-4 (does the SPS floor exist away from k = 1?)
95 V runs already in out/opt_sweep_k1_r*.csv are not repeated. verify_steady_state.m also writes, since 10-30, the
max/min of i_L in each of the last 80 switching periods (out/iL_pp/) -- which periods carry the 1 ms peak.

    python -I -X utf8 Simulink/Thesis/k1_map.py make      -> out/k1map_points_r1.csv
    python automation/job.py submit k1map --interactive --after optR -- matlab -batch "cd('Simulink/Thesis'); verify_steady_batch('out/k1map_points_r1.csv', 'out/k1map_sweep_r1.csv', 24, 0.040)"
    python -I -X utf8 Simulink/Thesis/k1_map.py compare   -> out/k1_map.{csv,log}, out/fig_k1_map.png
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
V2S = (93.0, 94.0, 95.0, 96.0, 97.0)
V2_SPS_ONLY = (90.0, 100.0)
DUTIES = (sw.D_SAFE, 0.475, 0.45, 0.40)
LEVELS = (1, 2, 3, 4)
TARGETS = (100.0, 150.0, 200.0)       # interpolated comparison (2.88-deg brackets are wide here: read with the levels)
# runs of OPT2 repeated on purpose (deterministic, same Ipk): they predate out/iL_pp/, and these are the runs behind the
# k = 1 verdict -- SPS 4.32 deg (0.738 A in 1 ms, 0.41 A in its last period), SPS 7.20 deg, D = 0.45 4.32 deg
RERUN = ((95.0, sw.D_SAFE, 2), (95.0, sw.D_SAFE, 3), (95.0, 0.45, 2))


def _name(d):
    return "SPS" if d >= 0.4999 else f"D={d:.3f}"


def configs():
    cf = []
    for V2 in V2S:
        for d in DUTIES:
            cf.append(dict(ctrl=_name(d), V2=V2, P=0.0, D1=round(d, 9), D2=round(d, 9)))
    for V2 in V2_SPS_ONLY:
        cf.append(dict(ctrl="SPS", V2=V2, P=0.0, D1=sw.D_SAFE, D2=sw.D_SAFE))
    return cf


def result_files():
    fs = glob.glob(os.path.join(OUT, "k1map_sweep_r*.csv")) + glob.glob(os.path.join(OUT, "opt_sweep_k1_r*.csv"))
    return [f for f in sorted(fs) if not f.endswith("_in.csv")]


def results():
    import pandas as pd
    fs = result_files()
    if not fs:
        return None
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["level"] = np.round((d["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last")


def make():
    d = results()
    done = [] if d is None else list(zip(d["V2"], d["D0"], d["D1"], d["D2"]))
    pts = []
    for c in configs():
        for j in LEVELS:
            p = sw.point(c, j)
            p["label"] = f"{c['ctrl']} {c['V2']:g}V L{j}"
            p["P_target"] = 0.0
            pts.append(p)
    path = os.path.join(OUT, "k1map_points_r1.csv")
    sw.write(path, pts, done)
    import csv
    with open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2"])
        for V2, d, j in RERUN:
            c = dict(ctrl=_name(d), V2=V2, P=0.0, D1=round(d, 9), D2=round(d, 9))
            p = sw.point(c, j)
            p.update(label=f"{c['ctrl']} {V2:g}V L{j} rerun (per-period peaks)", P_target=0.0)
            w.writerow(p)
    print(f"appended {len(RERUN)} repeated OPT2 runs (per-period peaks)")


def per_period():
    """{(V2, D0, D1, D2): (max of the per-period peaks, median of them, n periods within 5 % of the max)}"""
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "*.csv"))):
        b = os.path.basename(f)
        if not (b.startswith("k1map_sweep_") or b.startswith("opt_sweep_k1_")):
            continue
        t = pd.read_csv(f)
        mx = [c for c in t.columns if c.startswith("max") and c[3:].isdigit()]
        mn = [c for c in t.columns if c.startswith("min") and c[3:].isdigit()]
        for _, r in t.iterrows():
            pk = np.maximum(r[mx].to_numpy(float), -r[mn].to_numpy(float))
            if np.isfinite(pk).any():
                top = np.nanmax(pk)
                out[(r["V2"], r["D0"], r["D1"], r["D2"])] = (top, float(np.nanmedian(pk)),
                                                              int(np.sum(pk >= 0.95 * top)))
    return out


def compare():
    import pandas as pd
    d = results()
    if d is None:
        sys.exit("no k=1 runs yet")
    pp = per_period()
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    rows = []
    out("k = 1 light-load map, Simulink (dataset-era model), constant D0 40 ms, last 1 ms, HV -> LV")
    out("per D0 level: delivered power [W] / max|i_L| over the last 1 ms [A] (per-period median peak, periods near the max)")
    for V2 in sorted(set(V2S) | set(V2_SPS_ONLY)):
        out(f"\nV2 {V2:g} V (k = {4 * V2 / 380:.3f})")
        for c in configs():
            if c["V2"] != V2:
                continue
            g = sw.runs_of(d, c)
            g = g[g["level"].isin(LEVELS)]
            cells = []
            for j in LEVELS:
                r = g[g["level"] == j]
                if r.empty:
                    cells.append(f"{'–':>24}")
                    continue
                r = r.iloc[0]
                k = (r["V2"], r["D0"], r["D1"], r["D2"])
                extra = f" ({pp[k][1]:.2f}, {pp[k][2]:>2})" if k in pp else ""
                cells.append(f"{round(r['Pd']) or 0:6.0f} W {r['Ipk_sim']:5.3f}{extra:>11}")
                rows.append(dict(V2=V2, ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], level=j, D0=r["D0"], Pd=r["Pd"],
                                 Ipk=r["Ipk_sim"], iL_mean=r["iL_mean"], pp_median=pp.get(k, (np.nan,) * 3)[1],
                                 pp_n_top=pp.get(k, (np.nan,) * 3)[2]))
            out(f"  {c['ctrl']:>9} | " + " | ".join(cells))
        for P in TARGETS:
            best = []
            for c in configs():
                if c["V2"] != V2:
                    continue
                g = sw.runs_of(d, c)
                if g.empty:
                    continue
                b = sw.bracket(g, P)[0]
                if b is None:
                    continue
                lo, hi = b
                t = (P - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
                best.append((lo["Ipk_sim"] + t * (hi["Ipk_sim"] - lo["Ipk_sim"]), c["ctrl"], lo["Pd"], hi["Pd"]))
            sps = [b for b in best if b[1] == "SPS"]
            if sps and len(best) > 1:
                tps = min(b for b in best if b[1] != "SPS")
                out(f"  {P:5.0f} W interpolated: SPS {sps[0][0]:.3f} ({sps[0][2]:.0f}-{sps[0][3]:.0f} W), best D1 = D2 "
                    f"{tps[1]} {tps[0]:.3f} ({tps[2]:.0f}-{tps[3]:.0f} W): SPS - best {sps[0][0] - tps[0]:+.3f} A")
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, "k1_map.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "k1_map.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(T)
    print("wrote out/k1_map.{csv,log}, out/fig_k1_map.png")


def fig(T):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, MUTED, ORANGE, YELLOW, _style
    _style(plt)
    vs = [v for v in V2S if (T["V2"] == v).any()]
    if not vs:
        return
    f, ax = plt.subplots(1, len(vs), figsize=(3.3 * len(vs), 3.6), sharey=True, constrained_layout=True)
    ax = np.atleast_1d(ax)
    cols = {"SPS": MUTED, "D=0.475": YELLOW, "D=0.450": ORANGE, "D=0.400": BLUE}
    for a, v in zip(ax, vs):
        for name, g in T[T["V2"] == v].groupby("ctrl"):
            g = g.sort_values("Pd")
            a.plot(g["Pd"], g["Ipk"], "-o", ms=4, lw=1.4, color=cols.get(name, AQUA), label=name)
        a.set_title(f"V2 = {v:g} V (k = {4 * v / 380:.3f})", fontsize=9)
        a.set_xlabel("delivered power [W] (one point per 2.88-deg D0 level)")
        a.set_xlim(0, 500)
    ax[0].set_ylabel("max |i_L| over the last 1 ms [A]")
    ax[0].legend(fontsize=7.5, frameon=False)
    f.savefig(os.path.join(OUT, "fig_k1_map.png"), dpi=130)
    plt.close(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "compare"])
    a = ap.parse_args()
    make() if a.cmd == "make" else compare()


if __name__ == "__main__":
    main()
