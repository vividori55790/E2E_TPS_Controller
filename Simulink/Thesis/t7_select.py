"""
t7_select.py -- T7 final-model selection from the plant robustness runs (plant_verdict.py --part B --variant {A,B,C}
--region {all,k1} --tag _X_Y).

The true plant is not known to better than the validate_plant.py spread (Ron 0-52 mOhm, droop or not), and at k ~ 1
light load the dead-time sign of Ipk - Ipk_SPS flips between those variants (t7_probe.py).  So a controller is judged
on the worst plant variant at each point: dI_rob(x) = min_v [Ipk_SPS,v(x) - Ipk_ctrl,v(x)].

Selection rule (goal 3 first, then goal 1): among the models whose robust > 5 mA degradation is 0 % in both regions,
the largest mean dI over the variants on the 'all' region; if none qualifies, the fewest robust > 5 mA points.

    python -I -X utf8 Simulink/Thesis/t7_select.py            -> out/t7_select.{log,csv}, out/fig_t7_robust.png
"""
import csv
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
VARIANTS = ("A", "B", "C")
REGIONS = ("all", "k1")
DESC = {"A": "Ron 52 mOhm + droop", "B": "Ron 40 mOhm, no droop", "C": "Ron 0, no droop"}


def load(region):
    """{model: (I (3, n) plant, Ii (n,) ideal)}, P, V; the SPS key is 'SPS'."""
    runs = [np.load(os.path.join(OUT, f"plant_verdict_B_{v}_{region}.npz")) for v in VARIANTS]
    P, V = runs[0]["P"], runs[0]["V"]
    for r in runs[1:]:
        assert np.array_equal(r["P"], P) and np.array_equal(r["V"], V), "variants must share the points"
    names = [k for k in runs[0].files if k not in ("P", "V")]
    data = {k: (np.stack([r[k][0] for r in runs]), runs[0][k][1]) for k in names if all(k in r.files for r in runs)}
    return data, P, V


def label(k):
    """'Smooth_RegHDC_(b3)' -> 'b3', 'LUT_49x22_E2E_(L49m)' -> 'L49m' (the plant_verdict.load_models tag)."""
    m = re.search(r"\(([^)]+)\)", k)
    return m.group(1) if m else k


def main():
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    rows, robust = [], {}
    for region in REGIONS:
        data, P, V = load(region)
        I_sps, Ii_sps = data["SPS"]
        out(f"== region {region}: {P.size} points, |P| {P.min():.0f}-{P.max():.0f} W, V2 {V.min():.1f}-{V.max():.1f} V; "
            f"plant variants " + ", ".join(f"{v} = {DESC[v]}" for v in VARIANTS))
        out(f"{'model':16s} | " + " | ".join(f"{v}: mean   worst  >5mA" for v in VARIANTS)
            + f" | robust: mean  worst  >5mA >20mA | ideal: mean worst")
        for k, (I, Ii) in data.items():
            if k == "SPS":
                continue
            dI = I_sps - I                                          # (3, n), positive = reduction
            ok = np.all(np.isfinite(dI), axis=0)
            rob = np.min(dI, axis=0)
            dIi = Ii_sps - Ii
            per = []
            for j, v in enumerate(VARIANTS):
                d = dI[j][ok]
                per.append((d.mean(), d.min(), np.mean(d < -0.005)))
            r = rob[ok]
            out(f"{label(k):16s} | " + " | ".join(f"{m:.4f} {w * 1e3:6.1f} {f * 100:4.2f}%" for m, w, f in per)
                + f" | {np.mean(dI[:, ok]):.4f} {r.min() * 1e3:6.1f} {np.mean(r < -0.005) * 100:4.2f}% "
                  f"{np.mean(r < -0.02) * 100:4.2f}% | {np.nanmean(dIi):.4f} {np.nanmin(dIi) * 1e3:5.1f}"
                + (f"  [{(~ok).sum()} infeasible]" if (~ok).any() else ""))
            bad = ok & (rob < -0.005)
            if bad.any():
                out(f"{'':16s}   robust >5 mA points: |P| {P[bad].min():.0f}-{P[bad].max():.0f} W, V2 {V[bad].min():.1f}-"
                    f"{V[bad].max():.1f} V; worst at |P| {P[ok][np.argmin(r)]:.0f} W, V2 {V[ok][np.argmin(r)]:.1f} V")
            rows.append(dict(region=region, model=label(k),
                             **{f"mean_{v}": per[j][0] for j, v in enumerate(VARIANTS)},
                             **{f"worst_{v}": per[j][1] for j, v in enumerate(VARIANTS)},
                             **{f"deg5_{v}": per[j][2] for j, v in enumerate(VARIANTS)},
                             mean_all_variants=float(np.mean(dI[:, ok])), robust_worst=float(r.min()),
                             robust_deg5=float(np.mean(r < -0.005)), robust_deg20=float(np.mean(r < -0.02)),
                             ideal_mean=float(np.nanmean(dIi)), ideal_worst=float(np.nanmin(dIi)),
                             infeasible=int((~ok).sum())))
            robust[(region, label(k))] = (P, V, rob)
        out()

    with open(os.path.join(OUT, "t7_select.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    models = sorted({r["model"] for r in rows})
    by = {(r["region"], r["model"]): r for r in rows}
    full = [m for m in models if all((g, m) in by for g in REGIONS)]
    clean = [m for m in full if all(by[(g, m)]["robust_deg5"] == 0 for g in REGIONS)]
    if clean:
        best = max(clean, key=lambda m: by[("all", m)]["mean_all_variants"])
        why = f"robust >5 mA = 0 % in both regions ({', '.join(clean)}); largest mean over variants on 'all'"
    else:
        best = min(full, key=lambda m: (sum(by[(g, m)]["robust_deg5"] for g in REGIONS),
                                        -by[("all", m)]["mean_all_variants"]))
        why = "no model is clean on every variant; fewest robust >5 mA points"
    out(f"selected: {best}  ({why})")
    for g in REGIONS:
        r = by[(g, best)]
        out(f"  {g}: mean over variants {r['mean_all_variants']:.4f} A, robust worst {r['robust_worst'] * 1e3:.1f} mA, "
            f"robust >5 mA {r['robust_deg5'] * 100:.2f} %, ideal mean {r['ideal_mean']:.4f} A")
    open(os.path.join(OUT, "t7_select.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    show = [m for m in ("b3", best) if ("k1", m) in robust]
    show = list(dict.fromkeys(show))
    fig, ax = plt.subplots(1, len(show), figsize=(5.2 * len(show), 4.2), squeeze=False)
    for a, m in zip(ax[0], show):
        P, V, rob = robust[("k1", m)]
        lim = 0.03
        sc = a.scatter(P, V, c=np.clip(rob, -lim, lim), cmap="RdBu", vmin=-lim, vmax=lim, s=12)
        bad = rob < -0.005
        a.scatter(P[bad], V[bad], marker="x", c="k", s=28, label="> 5 mA worse than SPS (worst variant)")
        a.set_title(f"{m}: worst-variant dI on the plant (k~1 band)")
        a.set_xlabel("|P| [W]")
        a.set_ylabel("V2 [V]")
        a.legend(loc="upper right", fontsize=7)
        fig.colorbar(sc, ax=a, label="dI = Ipk_SPS - Ipk [A] (clipped +-30 mA)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_t7_robust.png"), dpi=130)


if __name__ == "__main__":
    sys.exit(main())
