"""
k1_simulink_compare.py -- R7 k ~ 1 check on Simulink (ready 2026-10-28, run when r5r6_chain has produced its outputs):
at V2 = 94.61 V, for SPS, (0.453, 0.453), (0.326, 0.326) and the D0 labels -2.40 ... 10.03 deg, put side by side
  staircase : R6 true-staircase rows (out/dataset_fixed_part_*.csv, same (V2, D1, D2, D0 label))
  steady    : constant D0 for 30 ms (out/stair_steady_part*.csv, labels K1_*, add_k1_steady_points.py)
  python    : the Python plant's steady state at the same point (replay_grid.py's plant; reference only)
A large staircase - steady difference at a label = the k ~ 1 light-load history effect seen on the Python plant
(k1_history_probe.py) is also in Simulink, and R7's k ~ 1 light-load rows must be read with it.

    python -I -X utf8 Simulink/Thesis/k1_simulink_compare.py  -> out/k1_simulink_compare.log
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "out")


def main():
    import pandas as pd
    parts = [p for p in sorted(glob.glob(os.path.join(OUT, "stair_steady_part*.csv"))) if not p.endswith("_in.csv")]
    st = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True) if parts else None
    if st is not None:
        st = st[st["label"].astype(str).str.startswith("K1_")]
    if st is None or not len(st):
        sys.exit("no K1_ rows in out/stair_steady_part*.csv yet (r5r6_chain step 2)")
    fx = sorted(glob.glob(os.path.join(OUT, "dataset_fixed_part_*.csv")))
    fx = pd.concat([pd.read_csv(p) for p in fx], ignore_index=True) if fx else None
    import plant_full as pf
    plant = pf.FullPlant(Ron=0.052, steps=125)
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("V2 94.61 V, HV->LV labels (D0 > 0 <=> P_LV < 0); Ipk [A] / power HV->LV [W]")
    out(f"  {'label':>20s} {'D0':>6s} | {'R6 staircase':>15s} | {'Simulink steady':>15s} | {'Python steady':>15s} | stair - steady")
    for _, r in st.sort_values(["label", "D0"]).iterrows():
        s_txt, d = "   (not yet)", np.nan
        if fx is not None:
            m = fx[np.isclose(fx["V2"], r["V2"], atol=1e-6) & np.isclose(fx["D1"], r["D1"], atol=1e-6)
                   & np.isclose(fx["D2"], r["D2"], atol=1e-6) & np.isclose(fx["D0"], r["D0"], atol=1e-6)]
            if len(m):
                s = m.iloc[0]
                s_txt = f"{s['I_pk']:.3f} {-s['P_act']:7.1f}"
                d = s["I_pk"] - r["Ipk_sim"]
        p = plant.steady(np.array([r["D0"]]), np.array([r["D1"]]), np.array([r["D2"]]), np.array([r["V2"]]))
        out(f"  {r['label']:>20s} {r['D0']:+6.2f} | {s_txt:>15s} | {r['Ipk_sim']:.3f} {-r['P_LV']:7.1f} | "
            f"{p['Ipk'][0]:.3f} {-p['P_LV'][0]:7.1f} | {d:+.3f}")
    open(os.path.join(OUT, "k1_simulink_compare.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
