"""
plant_vs_simulink.py -- the Python plant (plant_full.FullPlant) against the Simulink steady-state runs of T6
(out/simulink_steady.csv: 52 points, constant D0 / D1 / D2 for 30 ms, last 1 ms measured; 18 SPS, 24 TPS v2, 10 ICROS2026).

Two plant answers per point:
  newton  the period-1 steady state (FullPlant.steady, damped Newton)
  time    the same experiment as Simulink: start from i = 0, vc = 0, constant duties for 30 ms (2400 periods) at the
          bus voltages of the steady state, then max |i| and mean P_LV / mean i over the last 1 ms (80 periods)
The time-domain answer also shows whether a point has a period-1 steady state at all: at light load the dead time
can leave a sustained low-frequency (series-C / L) oscillation, seen in Simulink as a non-zero mean current.

    python -I -X utf8 Simulink/Thesis/plant_vs_simulink.py      -> out/plant_vs_simulink.{log,csv}
"""
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
CONFIGS = {  # name: FullPlant kwargs (quant=True = the Simulink controller's 100 ns edge grid, as in these runs)
    "A_q": dict(Ron=0.052, droop=True, quant=True),
    "B_q": dict(Ron=0.040, droop=False, quant=True),
    "C_q": dict(Ron=0.0, droop=False, quant=True),
    "B": dict(Ron=0.040, droop=False, quant=False),
}


def time_domain(pl, D0, D1, D2, V1b, V2b, periods=2400, last=80):
    sch = pl.schedule(D0, D1, D2)
    i = np.zeros(D0.size)
    vc = np.zeros(D0.size)
    ipk = np.zeros(D0.size)
    p = np.zeros(D0.size)
    im = np.zeros(D0.size)
    for k in range(periods):
        rec = k >= periods - last
        i, vc, _, Pl, w = pl.period(i, vc, sch, V1b, V2b, record=rec)
        if rec:
            ipk = np.maximum(ipk, np.abs(w).max(axis=0))
            p += Pl / last
            im += w[:-1].mean(axis=0) / last
    return ipk, p, im


def main():
    import pandas as pd
    t0 = time.time()
    d = pd.read_csv(os.path.join(OUT, "simulink_steady.csv"))
    D0, D1, D2, V2 = (d[c].to_numpy(float) for c in ("D0", "D1", "D2", "V2"))
    grp = np.where(d["label"].str.startswith("SPS"), "SPS",
                   np.where(d["label"].str.startswith("TPS"), "TPS v2", "ICROS"))
    # analytic model at the MEASURED bus voltages (as compare_simulink.py)
    an = tm.evaluate(D0, D1, D2, d["V_LV"].to_numpy(float), d["V_HV"].to_numpy(float))
    res = {"analytic": dict(Ipk=an["Ipk"], P=an["P"], im=np.zeros(len(d)))}
    for name, kw in CONFIGS.items():
        pl = pf.FullPlant(**kw)
        r = pl.steady(D0, D1, D2, V2)
        res[f"{name}_newton"] = dict(Ipk=r["Ipk"], P=r["P_LV"], im=np.zeros(len(d)), conv=r["conv"])
        ipk, p, im = time_domain(pl, D0, D1, D2, r["V1_bus"], r["V2_bus"])
        res[f"{name}_time"] = dict(Ipk=ipk, P=p, im=im)
        print(f"{name}: {time.time() - t0:.0f} s ({int((~r['conv']).sum())} points without a converged period-1 "
              f"steady state)", flush=True)
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    simI, simP, simM = d["Ipk_sim"].to_numpy(float), d["P_LV"].to_numpy(float), d["iL_mean"].to_numpy(float)
    # tps_model's P has the sign of P_LV (test_plant_full.py: ideal FullPlant P_LV == tps_model P)
    out(f"Simulink (30 ms, last 1 ms) vs models at the same constant (D0, D1, D2); 52 points. err = model - Simulink")
    out(f"{'model':16s} | {'group':7s} {'n':>3s} | Ipk bias   RMSE    max|.| | P_LV bias  RMSE [W]")
    rows = []
    for name, r in res.items():
        for g in ("SPS", "TPS v2", "ICROS", "all"):
            m = np.ones(len(d), bool) if g == "all" else grp == g
            eI = r["Ipk"][m] - simI[m]
            eP = r["P"][m] - simP[m]
            out(f"{name:16s} | {g:7s} {m.sum():3d} | {eI.mean():+.3f} {np.sqrt(np.mean(eI ** 2)):.3f} "
                f"{np.abs(eI).max():.3f} | {eP.mean():+7.1f} {np.sqrt(np.mean(eP ** 2)):6.1f}")
            rows.append(dict(model=name, group=g, n=int(m.sum()), Ipk_bias=eI.mean(), Ipk_rmse=np.sqrt(np.mean(eI ** 2)),
                             Ipk_maxabs=np.abs(eI).max(), P_bias=eP.mean(), P_rmse=np.sqrt(np.mean(eP ** 2))))
    out()
    out("per point (Simulink | analytic | B_q time / newton | mean i: Simulink, B_q time):")
    for k in range(len(d)):
        rt, rn = res["B_q_time"], res["B_q_newton"]
        out(f"  {d['label'][k]:22s} V2 {V2[k]:5.1f} D0 {D0[k]:7.2f} D {D1[k]:.3f}/{D2[k]:.3f} | Ipk {simI[k]:.3f} P "
            f"{simP[k]:7.1f} | {res['analytic']['Ipk'][k]:.3f} {res['analytic']['P'][k]:7.1f} | {rt['Ipk'][k]:.3f} "
            f"{rt['P'][k]:7.1f} / {rn['Ipk'][k]:.3f} {rn['P'][k]:7.1f}{'' if rn['conv'][k] else ' (no p-1)'} | "
            f"{simM[k]:+.3f} {rt['im'][k]:+.3f}")
    with open(os.path.join(OUT, "plant_vs_simulink.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    out(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(OUT, "plant_vs_simulink.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
