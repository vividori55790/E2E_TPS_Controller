"""
add_best_tps_steady_points.py -- (2026-10-28 04:30) append "BT_" points to out/stair_steady_points.csv, so that
r5r6_chain's step 2 (Simulink, constant D0, 30 ms) also measures the steady state of the dataset's grid-best TPS
combos where ipk_decompose.py found M-B (R.4 Simulink steady) above the dataset's best TPS row (D - B > 0):
109.2 V 100 W (0.247, 0.200) +0.29 A, 84 V 300 W (0.374, 0.389) +0.10 A, 84 V 100 W (0.216, 0.232) +0.05 A.
Those dataset rows were measured on the moving D0 (interp = 1); their steady state at the same power decides whether
the dataset's TPS really reaches below M-B there. R6 (gen_dataset_fixed, 11 x 11 subset) does not contain these combos.
HV->LV side (D0 > 0, P_LV < 0): every D0 label from -4.87 deg (two labels below 0) up to one step past the label where the dataset's (lagging)
measured |P| first reaches 1.5 x the target -- steady |P| at a label is at least the lagging dataset value, so the
steady crossing is bracketed. ipk_decompose.py reads the results. Idempotent.

    python -I -X utf8 Simulink/Thesis/add_best_tps_steady_points.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import transient_replay as tr  # noqa: E402

F = os.path.join(HERE, "out", "stair_steady_points.csv")
# (V2 grid index, D1 index, D2 index, target |P|) on the dataset grid V2_vec = linspace(84, 109.2, 20), Dg = linspace(0.2, 0.5, 20)
COMBOS = ((19, 3, 0, 100.0), (0, 11, 12, 300.0), (0, 1, 2, 100.0))


def main():
    import pandas as pd
    p = pd.read_csv(F)
    if p["label"].str.startswith("BT_").any():
        print("BT_ points already there")
        return
    ds = pd.read_csv(os.path.join(HERE, "out", "checkpoint_20260316_021229.csv"))
    V2v, Dg, g = np.linspace(84.0, 109.2, 20), np.linspace(0.2, 0.5, 20), tr.d0_grid()
    rows = []
    for iv, i1, i2, P in COMBOS:
        V2, D1, D2 = V2v[iv], Dg[i1], Dg[i2]
        r = ds[np.isclose(ds["V2"], V2) & np.isclose(ds["D1"], D1) & np.isclose(ds["D2"], D2)].sort_values("D0")
        steps = np.arange(1, len(g) + 1)
        pos = steps[g > 1e-9]
        reach = [s for s in pos if -float(r[np.isclose(r["D0"], g[s - 1])]["P_act"].iloc[0]) >= 1.5 * P]
        last = min((reach[0] + 1) if reach else pos[-1], pos[-1])
        name = f"BT_{V2:.1f}V_{P:.0f}W_{D1:.3f}_{D2:.3f}"
        for s in [18, 19, 20] + [s for s in pos if s <= last]:     # 18, 19: in case |P| at D0 = 0 already exceeds P
            rows.append(dict(label=name, V2=V2, D0=g[s - 1], D1=D1, D2=D2, step=int(s)))
        print(f"{name}: steps 18..{last} (D0 {g[17]:.2f} ... {g[last - 1]:.2f} deg)")
    p = pd.concat([p, pd.DataFrame(rows)], ignore_index=True)
    p.to_csv(F, index=False, float_format="%.15g")
    print(f"{len(rows)} points added -> {F} ({len(p)} rows)")


if __name__ == "__main__":
    main()
