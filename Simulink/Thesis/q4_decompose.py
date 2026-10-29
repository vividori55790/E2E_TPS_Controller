"""
q4_decompose.py -- where the R.8 Q4 rule-2 gap comes from (10-30). Simulink numbers only, no new verdict.

r8_verdict.py rule 2 (registered 10-28) compares two reductions, each against its own SPS:
  fixed - M-B = [Ipk_SPS(stair) - Ipk_best(stair)] - [Ipk_SPS(steady) - Ipk_MB(steady)]
              = [Ipk_SPS(stair) - Ipk_SPS(steady)]   (the true 1 ms staircase's own SPS above its steady state)
              + [Ipk_MB(steady) - Ipk_best(stair)]   (the controllers at the same power)
The verdict stays what the rule says; this only splits it. Whether the grid-best combinations keep their staircase Ipk
in steady state is OPT1 (surface_opt_check.py, family grid: the same combinations at 30 ms constant D0).

    python -I -X utf8 Simulink/Thesis/q4_decompose.py      -> out/q4_decompose.log
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from r8_verdict import OSC  # noqa: E402

OUT = os.path.join(HERE, "out")


def table(path=os.path.join(OUT, "dataset_fixed_compare.csv")):
    T = pd.read_csv(path)
    T = T[T["direction"] == "HV->LV"].assign(V2=lambda d: d["V2"].round(2))
    f = T[T["source"] == "fixed"].set_index(["V2", "P"])
    m = T[T["source"] == "R4_MB_steady"].set_index(["V2", "P"])
    rows = []
    for k in m.index:
        if k[0] not in (84.0, 109.2) or k in OSC or k not in f.index:
            continue
        a, b = f.loc[k], m.loc[k]
        sps = a["Ipk_SPS"] - b["Ipk_SPS"]
        ctl = b["Ipk_TPS"] - a["Ipk_TPS"]
        rows.append(dict(V2=k[0], P=k[1], gap=a["dI"] - b["dI"], sps_part=sps, ctrl_part=ctl,
                         SPS_stair=a["Ipk_SPS"], SPS_steady=b["Ipk_SPS"], best_stair=a["Ipk_TPS"], MB_steady=b["Ipk_TPS"],
                         best=(a["D1"], a["D2"])))
    return pd.DataFrame(rows)


def main():
    d = table()
    lines = ["q4_decompose: R.8 Q4 rule 2 (fixed - M-B) split, HV->LV, k != 1, R.4 oscillating point excluded. Simulink only.",
             "  gap = (SPS stair - SPS steady) + (M-B steady - best TPS stair)  [A]", "",
             "   V2     |P| |   gap = SPS part + ctrl part | SPS stair/steady | M-B steady  best stair (D1, D2)"]
    for r in d.itertuples():
        lines.append(f"{r.V2:6.1f} {r.P:6.0f}  | {r.gap:+6.2f} = {r.sps_part:+6.2f} + {r.ctrl_part:+6.2f}    |"
                     f"   {r.SPS_stair:5.2f} / {r.SPS_steady:5.2f}  |   {r.MB_steady:5.3f}     {r.best_stair:5.3f}"
                     f" ({r.best[0]:.3f}, {r.best[1]:.3f})")
    g, s, c = d["gap"].mean(), d["sps_part"].mean(), d["ctrl_part"].mean()
    i = d["ctrl_part"].idxmax()
    lines += ["", f"mean over {len(d)} points: gap {g:+.3f} = SPS part {s:+.3f} ({100 * s / g:.0f} %) + ctrl part {c:+.3f} A",
              f"ctrl part max {d['ctrl_part'].max():+.3f} A @ {d.loc[i, 'V2']:g} V {d.loc[i, 'P']:.0f} W, "
              f"min {d['ctrl_part'].min():+.3f} A; points with ctrl part > 0.10 A: "
              + (", ".join(f"{r.V2:g} V {r.P:.0f} W" for r in d.itertuples() if r.ctrl_part > 0.10) or "none"),
              "Without the 109.2 V 300 W cell (true-staircase SPS 4.62 A vs steady 2.82 A): gap "
              f"{d[~((d.V2 == 109.2) & (d.P == 300))]['gap'].mean():+.3f} A.",
              "best stair = a 1 ms staircase window, not a steady state; OPT1 (grid family) measures it at 30 ms."]
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT, "q4_decompose.log"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    return d


if __name__ == "__main__":
    main()
