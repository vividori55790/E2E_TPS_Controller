"""
add_k1_steady_points.py -- (2026-10-27 23:55) append k ~ 1 points to out/stair_steady_points.csv, so that r5r6_chain's
step 2 (Simulink, constant D0, 30 ms) also measures the steady state at V2 = V2_vec(9) = 94.61 V, the R6 grid's k ~ 1
row, on the dataset's D0 labels near 0, for SPS and the two diagonal TPS combos that the R7 method picked there
(k1_light_probe.py). With R6's true-staircase rows at the same (V2, D1, D2, D0 label) this tells whether a k ~ 1
light-load "TPS gain" in R6 is a steady-state property or a history / transient effect of the staircase.
staircase_compare.py ignores these labels (it iterates over the staircase combos only). Idempotent.

    python -I -X utf8 Simulink/Thesis/add_k1_steady_points.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import transient_replay as tr  # noqa: E402

F = os.path.join(HERE, "out", "stair_steady_points.csv")


def main():
    import pandas as pd
    p = pd.read_csv(F)
    if p["label"].str.startswith("K1_").any():
        print("k ~ 1 points already there")
        return
    V2 = float(np.linspace(84.0, 109.2, 20)[8])
    Dg = np.linspace(0.2, 0.5, 20)
    g = tr.d0_grid()
    steps = [19, 20, 21, 22, 23, 24]                             # 1-based: -2.40, 0, 2.40, 4.87, 7.41, 10.03 deg
    rows = []
    for name, d in (("K1_SPS_94.61V", Dg[19]), ("K1_TPS0.453_94.61V", Dg[16]), ("K1_TPS0.326_94.61V", Dg[8])):
        for s in steps:
            rows.append(dict(label=name, V2=V2, D0=g[s - 1], D1=d, D2=d, step=s))
    p = pd.concat([p, pd.DataFrame(rows)], ignore_index=True)
    p.to_csv(F, index=False, float_format="%.15g")
    print(f"{len(rows)} points added -> {F} ({len(p)} rows)")


if __name__ == "__main__":
    main()
