"""
dataset_vs_simulink.py -- Simulink against Simulink, no analytic model and no D0 label involved (R3; reproduces the
2026-10-26 interactive check): the existing dataset's SPS rows (Simulink/Data/TPS_Dataset_20260316_021229.csv,
D1 = D2 = 0.5, 1 ms staircase windows) vs the T6 Simulink steady-state SPS runs (out/simulink_steady.csv: constant D0
+-8 / +-20 / +-35 deg for 30 ms) at the SAME measured LV power. Both come from the same Simulink plant (dead time,
Ron, diodes, series caps), so a difference here is the measurement, not a modelling gap.

For each V2 in {84, 109.2} V and power sign: the steady runs give Ipk(P_LV) at three powers; the dataset SPS rows
whose measured P_act_W lies inside that range are compared with the steady curve interpolated (quadratic through the
three points) at their own measured power.

    python -I -X utf8 Simulink/Thesis/dataset_vs_simulink.py      -> out/dataset_vs_simulink.{log,csv}
"""
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(HERE, "out")
DATASET = os.path.join(ROOT, "Simulink", "Data", "TPS_Dataset_20260316_021229.csv")


def main():
    import pandas as pd
    ds = pd.read_csv(DATASET)
    st = pd.read_csv(os.path.join(OUT, "simulink_steady.csv"))
    st = st[st["label"] == "SPS"]
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"dataset {os.path.relpath(DATASET, ROOT)} SPS rows vs Simulink steady-state SPS (out/simulink_steady.csv) at the "
        f"same measured P_LV; diff = dataset Ipk - steady Ipk")
    out(f"{'V2':>6s} {'dir':>8s} | {'P range [W]':>13s} {'n':>4s} | {'diff min':>8s} {'mean':>7s} {'max':>7s} | example")
    for V2 in (84.0, 109.2):
        for sgn, lab in ((+1, "LV->HV"), (-1, "HV->LV")):
            s = st[(np.isclose(st["V2"], V2)) & (np.sign(st["P_LV"]) == sgn)].sort_values("P_LV", key=np.abs)
            Pp, Ip = np.abs(s["P_LV"].to_numpy(float)), s["Ipk_sim"].to_numpy(float)
            c = np.polyfit(Pp, Ip, 2)
            d = ds[np.isclose(ds["V2_V"], V2) & (ds["D1"] > 0.4999) & (ds["D2"] > 0.4999) & (np.sign(ds["P_act_W"]) == sgn)]
            Pa = np.abs(d["P_act_W"].to_numpy(float))
            m = (Pa >= Pp.min()) & (Pa <= Pp.max())
            Pa, Id = Pa[m], d["i_pk_pri_A"].to_numpy(float)[m]
            diff = Id - np.polyval(c, Pa)
            k = int(np.argmax(diff))
            out(f"{V2:6.1f} {lab:>8s} | {Pp.min():5.0f}-{Pp.max():5.0f} {m.sum():5d} | {diff.min():+8.2f} {diff.mean():+7.2f} "
                f"{diff.max():+7.2f} | worst at {Pa[k]:.0f} W: dataset {Id[k]:.2f} A vs steady {np.polyval(c, Pa[k]):.2f} A")
            rows.append(dict(V2=V2, direction=lab, P_lo=Pp.min(), P_hi=Pp.max(), n=int(m.sum()), diff_min=diff.min(),
                             diff_mean=diff.mean(), diff_max=diff.max(), worst_P=Pa[k], worst_dataset=Id[k],
                             worst_steady=float(np.polyval(c, Pa[k]))))
    with open(os.path.join(OUT, "dataset_vs_simulink.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    open(os.path.join(OUT, "dataset_vs_simulink.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
