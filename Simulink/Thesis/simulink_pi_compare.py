"""
simulink_pi_compare.py -- R2(3) result: the Simulink PI closed loop (verify_pi_closed_loop.m -> out/simulink_pi_raw.csv)
for SPS, M-A, M-B0, M-B at the same power command, next to the same configurations on the two other yardsticks:
  analytic  ideal steady-state model, PI first crossing (pi_steady.operating_point)
  plant     the .slx-extracted Python plant (plant_full, Ron 40 mOhm, no droop = best fit to the steady rows), PI first
            crossing; time-stepped where there is no stable period-1 steady state
  simulink  the dataset-era Simulink model, its own current PI on the outer phase with external D1 / D2,
            max |i_L| over the last 1 ms of the run
dI = Ipk(SPS) - Ipk(controller) at the same (direction, V2, |P|), positive = reduction.
Settling check (Simulink): the regulated power within 2 % of the command at 30 ms and at the end, and the Ipk change
between the last two 1 ms windows.

    python -I -X utf8 Simulink/Thesis/simulink_pi_compare.py [--raw out/simulink_pi_raw.csv]
        -> out/simulink_pi_compare.{csv,log}, out/fig_simulink_pi_compare.png
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402
import plant_full as pf  # noqa: E402

OUT = os.path.join(HERE, "out")
CTRLS = ("SPS", "M-A", "M-B0", "M-B")


def main():
    import pandas as pd
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", default=os.path.join(OUT, "simulink_pi_raw.csv"))
    ap.add_argument("--no-plant", action="store_true")
    a = ap.parse_args()
    d = pd.read_csv(a.raw)
    Pa, V2 = d["P_ref"].abs().to_numpy(float), d["V2"].to_numpy(float)
    D1, D2 = d["D1"].to_numpy(float), d["D2"].to_numpy(float)
    d["Ipk_analytic"] = ps.operating_point(Pa, D1, D2, V2)["Ipk"]
    if not a.no_plant:
        pl = pf.FullPlant(Ron=0.040, droop=False, quant=False)
        r = pl.operating_point(Pa, D1, D2, V2)
        d["Ipk_plant"] = np.where(r["ok"], r["Ipk"], np.nan)
        d["plant_cycle"] = r["cycle"]
    else:
        d["Ipk_plant"] = np.nan
        d["plant_cycle"] = False
    reg = np.where(d["dir"] == "HV->LV", d["P_LV"], d["P_HV"])          # the power the controller regulates
    reg30 = np.where(d["dir"] == "HV->LV", d["P_LV_at_30ms"], d["P_HV_at_30ms"])
    d["P_err_pct"] = 100.0 * (np.abs(reg) - Pa) / Pa
    d["P_err30_pct"] = 100.0 * (np.abs(reg30) - Pa) / Pa
    last = [c for c in d.columns if c.startswith("Ipk_at_")]
    d["Ipk_last2_change"] = (d[last[-1]] - d[last[-2]]).abs()
    d["settled"] = (d["P_err_pct"].abs() < 2.0) & (d["P_err30_pct"].abs() < 2.0) & (d["Ipk_last2_change"] < 0.05)
    key = ["dir", "V2", "P_abs"]
    d["P_abs"] = Pa
    sps = d[d["ctrl"] == "SPS"].set_index(key)
    rows = []
    for _, r in d.iterrows():
        s = sps.loc[(r["dir"], r["V2"], r["P_abs"])]
        rec = dict(dir=r["dir"], V2=r["V2"], P=r["P_abs"], ctrl=r["ctrl"], D1=r["D1"], D2=r["D2"],
                   settled=bool(r["settled"] and s["settled"]), P_err_pct=r["P_err_pct"], iL_mean=r["iL_mean"])
        for y, col in (("analytic", "Ipk_analytic"), ("plant", "Ipk_plant"), ("simulink", "Ipk_sim")):
            rec[f"Ipk_{y}"] = r[col]
            rec[f"Ipk_sps_{y}"] = s[col]
            rec[f"dI_{y}"] = s[col] - r[col]
            rec[f"dI_pct_{y}"] = 100.0 * (s[col] - r[col]) / s[col]
        rows.append(rec)
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, "simulink_pi_compare.csv"), index=False, float_format="%.5g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"Simulink PI closed loop ({os.path.basename(a.raw)}): {len(d)} runs, settled {int(d['settled'].sum())}; "
        f"gains Kp {sorted(d['Kp'].unique())}, Ki {sorted(d['Ki'].unique())}")
    if (~d["settled"]).any():
        for _, r in d[~d["settled"]].iterrows():
            out(f"  not settled: {r['label']}: power error {r['P_err_pct']:+.1f} % (30 ms {r['P_err30_pct']:+.1f} %), "
                f"Ipk change last 1 ms {r['Ipk_last2_change']:.3f} A, mean i_L {r['iL_mean']:+.3f} A")
    out("\nIpk [A] (dI vs SPS [A], %) per configuration -- analytic | plant | simulink")
    for (dr, v, p), g in T.groupby(["dir", "V2", "P"], sort=False):
        out(f"{dr} V2 {v:5.1f} |P| {p:5.0f} W:")
        for _, r in g.iterrows():
            out(f"   {r['ctrl']:5s} D {r['D1']:.3f}/{r['D2']:.3f} | " + " | ".join(
                f"{r[f'Ipk_{y}']:.3f} ({r[f'dI_{y}']:+.3f}, {r[f'dI_pct_{y}']:+5.1f} %)" for y in ("analytic", "plant", "simulink"))
                + ("" if r["settled"] else "  [not settled]"))
    out("\nsummary over the HV->LV grid (12 points): mean dI [A] / mean dI [%] / worst dI [A] / points > 5 mA worse")
    for c in CTRLS[1:]:
        g = T[(T["ctrl"] == c) & (T["dir"] == "HV->LV")]
        out(f"  {c:5s} " + " | ".join(
            f"{y}: {g[f'dI_{y}'].mean():+.3f} / {g[f'dI_pct_{y}'].mean():+5.1f} % / {g[f'dI_{y}'].min():+.3f} / "
            f"{int((g[f'dI_{y}'] < -0.005).sum())}" for y in ("analytic", "plant", "simulink")))
    open(os.path.join(OUT, "simulink_pi_compare.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(T)


def fig(T):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, MUTED, ORANGE, _style
    _style(plt)
    col = {"M-A": ORANGE, "M-B0": AQUA, "M-B": BLUE}
    lab = {"M-A": "M-A (existing dataset target)", "M-B0": "M-B0 (steady-state surface)", "M-B": "M-B (+ physics E2E)"}
    H = T[T["dir"] == "HV->LV"]
    Vs = sorted(H["V2"].unique())
    figr, ax = plt.subplots(2, len(Vs), figsize=(4.6 * len(Vs), 7.2), constrained_layout=True, sharex=True, squeeze=False)
    for j, v in enumerate(Vs):
        for c in ("M-A", "M-B0", "M-B"):
            g = H[(H["V2"] == v) & (H["ctrl"] == c)].sort_values("P")
            ax[0, j].plot(g["P"], g["dI_simulink"], "o-", color=col[c], lw=2, label=lab[c])
            ax[0, j].plot(g["P"], g["dI_analytic"], ":", color=col[c], lw=1.2)
            ax[1, j].plot(g["P"], g["dI_pct_simulink"], "o-", color=col[c], lw=2)
        for r in (0, 1):
            ax[r, j].axhline(0.0, color="#c3c2b7", lw=1)
        ax[0, j].set_title(f"V2 = {v:g} V (k = {4 * v / 380:.3f}), HV -> LV")
        ax[1, j].set_xlabel("|P| [W]")
    ax[0, 0].set_ylabel("Ipk reduction vs SPS [A]\n(solid: Simulink PI closed loop, dotted: analytic)")
    ax[1, 0].set_ylabel("Ipk reduction vs SPS [%] (Simulink)")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="lower left")
    ax[1, 0].text(0.02, 0.03, "", transform=ax[1, 0].transAxes, color=MUTED)
    figr.savefig(os.path.join(OUT, "fig_simulink_pi_compare.png"), dpi=130)
    plt.close(figr)


if __name__ == "__main__":
    sys.exit(main())
