"""
compare_simulink.py -- T6 analysis: analytic model vs steady-state Simulink (out/simulink_steady.csv from
verify_steady_state.m).

Per point the analytic model is re-evaluated at the MEASURED bridge voltages (V1 := V_HV, V2 := V_LV; the battery
has 0.15 ohm internal resistance, so V_LV moves with the current) and the same D0, D1, D2. Reported:
  residuals Ipk_sim - Ipk_model, P_LV - P_model (and relative), DC offset mean(i_L), settling (Ipk / P_LV in 1 ms
  windows ending at 10 ... 30 ms), direction symmetry (P<0 vs P>0 pairs), and the k~1 verdict: the ICROS2026
  "optimal" (D1, D2) vs SPS at the same power, both measured in Simulink.
Writes out/simulink_compare.csv and out/fig_simulink_compare.png.
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")


def main():
    d = pd.read_csv(os.path.join(OUT, "simulink_steady.csv"))
    bad = d["error"].fillna("").astype(str).str.len() > 0
    if bad.any():
        print(f"{int(bad.sum())} points failed:\n", d.loc[bad, ["label", "error"]].to_string())
    d = d[~bad].copy()
    V1 = d["V_HV"].fillna(tm.V1_NOM).to_numpy()
    V2 = d["V_LV"].fillna(d["V2"]).to_numpy()
    e = tm.evaluate(d["D0"].to_numpy(), d["D1"].to_numpy(), d["D2"].to_numpy(), V2, V1)
    d["Ipk_model_meas"], d["P_model_meas"], d["Irms_model_meas"] = e["Ipk"], e["P"], e["Irms"]
    d["dIpk"] = d["Ipk_sim"] - d["Ipk_model_meas"]
    d["dIpk_rel"] = d["dIpk"] / d["Ipk_model_meas"]
    d["dP"] = d["P_LV"] - d["P_model_meas"]
    d["dP_rel"] = d["dP"] / d["P_model_meas"].abs().clip(lower=50.0)
    d["dIrms"] = d["iL_rms"] - d["Irms_model_meas"]
    d["settle_Ipk"] = (d["Ipk_at_25ms"] - d["Ipk_sim"]).abs()
    d.to_csv(os.path.join(OUT, "simulink_compare.csv"), index=False, float_format="%.6g")
    cols = ["label", "V2", "D0", "D1", "D2", "V_LV", "V_HV", "P_LV", "P_model_meas", "Ipk_sim", "Ipk_model_meas",
            "dIpk", "iL_mean", "iL_rms", "Irms_model_meas", "settle_Ipk"]
    print(d[cols].to_string(float_format=lambda x: f"{x:8.3f}"))
    for grp, m in (("SPS", d["label"] == "SPS"), ("TPS", d["label"].str.startswith("TPS")),
                   ("ICROS", d["label"].str.startswith("ICROS"))):
        x = d[m]
        if len(x):
            print(f"\n{grp}: {len(x)} pts | dIpk mean {x['dIpk'].mean():+.3f} A, max|.| {x['dIpk'].abs().max():.3f} A, "
                  f"rel mean {x['dIpk_rel'].mean() * 100:+.1f} % | dP mean {x['dP'].mean():+.1f} W, rel max "
                  f"{x['dP_rel'].abs().max() * 100:.1f} % | DC offset max {x['iL_mean'].abs().max():.3f} A | "
                  f"settling (25 vs 30 ms) max {x['settle_Ipk'].max():.4f} A")
    # direction symmetry
    tp = d[d["label"].str.startswith("TPS")]
    if len(tp):
        key = tp["label"].str.replace(r" P[<>]0$", "", regex=True) + tp["V2"].astype(str)
        g = tp.assign(key=key).groupby("key")["Ipk_sim"].agg(["min", "max"])
        print(f"\nP<0 vs P>0 Ipk difference in Simulink: max {np.max(g['max'] - g['min']):.4f} A over {len(g)} pairs")
    # k ~ 1 verdict: ICROS (D1, D2) vs SPS at the same nominal power
    ic = d[d["label"].str.startswith("ICROS")]
    if len(ic):
        print("\nICROS2026 'optimal' vs SPS in Simulink (same nominal power):")
        for lab in ic["label"].unique():
            if lab.startswith("ICROS-SPS"):
                continue
            a = ic[ic["label"] == lab]
            b = ic[ic["label"] == lab.replace("ICROS ", "ICROS-SPS ")]
            for _, r in a.iterrows():
                s = b[np.isclose(b["V2"], r["V2"])]
                if len(s):
                    s = s.iloc[0]
                    print(f"  {lab} V2 {r['V2']:.0f}: TPS Ipk {r['Ipk_sim']:.3f} A @ {r['P_LV']:.0f} W vs SPS "
                          f"{s['Ipk_sim']:.3f} A @ {s['P_LV']:.0f} W -> reduction {s['Ipk_sim'] - r['Ipk_sim']:+.3f} A "
                          f"(model {s['Ipk_model_meas'] - r['Ipk_model_meas']:+.3f} A)")
                else:
                    print(f"  {lab} V2 {r['V2']:.0f}: Ipk {r['Ipk_sim']:.3f} A (model {r['Ipk_model_meas']:.3f}) "
                          f"@ {r['P_LV']:.0f} W (SPS point itself)")
    fig(d)


def fig(d):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.4), constrained_layout=True)
    groups = (("SPS", "#2a78d6", "o"), ("TPS", "#eb6834", "s"), ("ICROS", "#1baf7a", "^"))
    for g, col, mk in groups:
        x = d[d["label"].str.startswith(g)]
        ax[0].plot(x["Ipk_model_meas"], x["Ipk_sim"], mk, color=col, ms=7, label=g)
        ax[1].plot(x["P_model_meas"], x["P_LV"], mk, color=col, ms=7, label=g)
        ax[2].plot(x["P_LV"].abs(), x["iL_mean"], mk, color=col, ms=7, label=g)
    lim = [0, max(d["Ipk_sim"].max(), d["Ipk_model_meas"].max()) * 1.05]
    ax[0].plot(lim, lim, color="#c3c2b7", lw=1)
    ax[0].set(xlabel="model Ipk at measured V_LV, V_HV [A]", ylabel="Simulink Ipk (last 1 ms) [A]",
              title="peak current")
    pl = [d[["P_model_meas", "P_LV"]].min().min() * 1.05, d[["P_model_meas", "P_LV"]].max().max() * 1.05]
    ax[1].plot(pl, pl, color="#c3c2b7", lw=1)
    ax[1].set(xlabel="model P [W]", ylabel="Simulink P_LV [W]", title="power (dataset sign)")
    ax[2].axhline(0, color="#c3c2b7", lw=1)
    ax[2].set(xlabel="|P_LV| [W]", ylabel="mean i_L over the last 1 ms [A]", title="DC offset at steady state")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(frameon=False, fontsize=8)
    fig.savefig(os.path.join(OUT, "fig_simulink_compare.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
