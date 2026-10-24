"""
validate_plant.py -- T6 fallback, step 1: does the Python plant reproduce what Simulink actually produced?

Reference: the dataset rows whose D0 label is +90 deg (Simulink/Data/TPS_Dataset_20260316_021229.csv, 8000 rows =
20 V2 x 20 D1 x 20 D2). +90 is the last step of every run, so D0 is held there (no next grid value to ramp to; see
out/plant_params.md section 6 item 1) and the window should be at steady state; every other label sees a ramping D0.
Compared: dataset i_pk_pri_A, P_act_W (LV bus, + = LV -> HV) and P_act_HV_W (HV side, same sign) against
  ideal      tps_model at the label V2 (lossless, no dead time)
  plant      FullPlant variants (Ron 40 / 52 mOhm, dead time 100 ns, 2.88 deg edge quantisation, body diode,
             1000 uF series caps, source droop), Ipk both continuous and sampled on the 100 ns grid
Metrics: bias (model - Simulink) and RMSE of Ipk and P, R^2 of Ipk. Output out/validate_plant.log/.csv and
out/fig_validate_plant.png (Ipk model vs Simulink for ideal and the best plant).

    python -I -X utf8 Simulink/Thesis/validate_plant.py [--n 8000] [--label 90]
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")
DATA = os.path.join(os.path.dirname(HERE), "Data", "TPS_Dataset_20260316_021229.csv")


def stats(model, ref):
    e = model - ref
    ss = np.sum((ref - ref.mean()) ** 2)
    return dict(bias=float(e.mean()), rmse=float(np.sqrt(np.mean(e ** 2))), p95=float(np.percentile(np.abs(e), 95)),
                r2=float(1.0 - np.sum(e ** 2) / ss))


def main():
    import pandas as pd
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=8000)
    ap.add_argument("--label", type=float, default=90.0)
    a = ap.parse_args()
    t0 = time.time()
    d = pd.read_csv(DATA)
    s = d[np.isclose(d["D0_deg"], a.label)]
    if a.n < len(s):
        s = s.sample(a.n, random_state=0)
    D0, D1, D2, V2 = (s[c].to_numpy(float) for c in ("D0_deg", "D1", "D2", "V2_V"))
    I_ref, P_ref, PH_ref = (s[c].to_numpy(float) for c in ("i_pk_pri_A", "P_act_W", "P_act_HV_W"))
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"{len(s)} dataset rows with D0 label {a.label:g} deg; Simulink Ipk {I_ref.min():.2f}-{I_ref.max():.2f} A, "
        f"P_LV {P_ref.min():.0f}..{P_ref.max():.0f} W; losses |P_LV|-|P_HV| median "
        f"{np.median(np.abs(P_ref) - np.abs(PH_ref)):.1f} W ({np.median(1 - np.abs(PH_ref) / np.abs(P_ref)) * 100:.1f} %)")
    out(f"{'model':44s} {'Ipk bias':>9s} {'rmse':>7s} {'p95':>7s} {'R2':>7s} | {'P_LV bias':>9s} {'rmse':>7s} | "
        f"{'P_HV bias':>9s} {'rmse':>7s}")
    ideal = tm.evaluate(D0, D1, D2, V2)

    def row(name, Ipk, P, PH):
        si, sp, sh = stats(Ipk, I_ref), stats(P, P_ref), stats(PH, PH_ref)
        out(f"{name:44s} {si['bias']:+9.3f} {si['rmse']:7.3f} {si['p95']:7.3f} {si['r2']:7.4f} | {sp['bias']:+9.1f} "
            f"{sp['rmse']:7.1f} | {sh['bias']:+9.1f} {sh['rmse']:7.1f}")
        rows.append(dict(model=name, **{f"I_{k}": v for k, v in si.items()}, **{f"P_{k}": v for k, v in sp.items()},
                         **{f"PH_{k}": v for k, v in sh.items()}))
        return si["rmse"]

    row("ideal (tps_model, label V2)", ideal["Ipk"], ideal["P"], ideal["P"])
    variants = [
        ("plant Ron40 td100 quant diode droop", dict()),
        ("plant Ron52 td100 quant diode droop", dict(Ron=0.052)),
        ("plant Ron40+Rs12 td100 quant diode droop", dict(Rs_dev=0.012)),
        ("plant Ron52 noclamp td100 quant droop", dict(Ron=0.052, rev_clamp=False)),
        ("plant Ron40+Rs12 noclamp td100 quant droop", dict(Rs_dev=0.012, rev_clamp=False)),
        ("plant Ron40 td100 noquant diode droop", dict(quant=False)),
        ("plant Ron40 td0 quant diode droop", dict(td=0.0)),
        ("plant Ron40 td100 quant diode nodroop", dict(droop=False)),
        ("plant Ron0 td100 quant diode droop", dict(Ron=1e-6)),
    ]
    best = (np.inf, None, None)
    keep = {}
    for name, kw in variants:
        p = pf.FullPlant(**kw)
        r = p.steady(D0, D1, D2, V2)
        PH = -r["P_HV"]                                   # dataset sign: + = LV -> HV (into the HV source)
        e1 = row(name + " [sampled]", r["Ipk_sampled"], r["P_LV"], PH)
        row(name + " [continuous]", r["Ipk"], r["P_LV"], PH)
        keep[name] = r
        if e1 < best[0]:
            best = (e1, name, r)
        out(f"    ({time.time() - t0:.0f} s, max shooting residual {r['resid'].max():.1e} A, V2 bus shift "
            f"{np.abs(r['V2_bus'] - V2).max():.2f} V max)")
    # residual structure of the best variant: by D1/D2 (TPS depth) and V2
    _, bname, br = best
    e = br["Ipk_sampled"] - I_ref
    out(f"\nbest: {bname}; residual (model - Simulink) of Ipk by region:")
    for lab, m in (("SPS (D1 = D2 = 0.5)", (D1 > 0.499) & (D2 > 0.499)), ("min(D1, D2) < 0.3", np.minimum(D1, D2) < 0.3),
                   ("V2 < 92", V2 < 92), ("92 <= V2 <= 98", (V2 >= 92) & (V2 <= 98)), ("V2 > 98", V2 > 98)):
        if m.any():
            out(f"  {lab:22s} n {m.sum():5d}  bias {e[m].mean():+.3f} A  rmse {np.sqrt(np.mean(e[m] ** 2)):.3f} A")
    with open(os.path.join(OUT, f"validate_plant_D0_{a.label:g}.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    open(os.path.join(OUT, f"validate_plant_D0_{a.label:g}.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    np.savez(os.path.join(OUT, f"validate_plant_D0_{a.label:g}.npz"), D0=D0, D1=D1, D2=D2, V2=V2, I_ref=I_ref,
             P_ref=P_ref, PH_ref=PH_ref, I_ideal=ideal["Ipk"], I_best=br["Ipk_sampled"], P_best=br["P_LV"])
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(9, 4.2), constrained_layout=True)
        for k, (lab, Im) in enumerate((("ideal model", ideal["Ipk"]), (f"Python plant ({bname.split(' [')[0]})",
                                                                        br["Ipk_sampled"]))):
            ax[k].scatter(I_ref, Im, s=2, alpha=0.35, color="#2a6fbb", lw=0)
            lim = [0, max(I_ref.max(), Im.max()) * 1.05]
            ax[k].plot(lim, lim, color="#888", lw=0.8)
            st = stats(Im, I_ref)
            ax[k].set_title(f"{lab}\nbias {st['bias']:+.3f} A, RMSE {st['rmse']:.3f} A", fontsize=9)
            ax[k].set_xlabel("Simulink i_pk (dataset, D0 = +90 deg) [A]")
            ax[k].set_ylabel("model i_pk [A]")
            ax[k].set_xlim(lim)
            ax[k].set_ylim(lim)
            ax[k].grid(alpha=0.3, lw=0.5)
        fig.savefig(os.path.join(OUT, "fig_validate_plant.png"), dpi=150)
    except Exception as ex:  # noqa: BLE001
        out(f"(figure skipped: {ex})")
    out(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
