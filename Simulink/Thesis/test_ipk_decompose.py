"""
test_ipk_decompose.py -- checks ipk_decompose.steady_best's Simulink path before the real r5r6 step-2 output exists:
a fake stair_steady part (Python plant values at the BT_ labels) is written to a TEMP dir, never to out/ (any
out/stair_steady_part*.csv would make r5r6_chain skip step 2), and glob is redirected to it. B_st must then equal
the --py column.

    python -I -X utf8 Simulink/Thesis/test_ipk_decompose.py
"""
import glob
import os
import sys
import tempfile

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ipk_decompose as m  # noqa: E402
import plant_full as pf  # noqa: E402


def main():
    pts = pd.read_csv(os.path.join(m.OUT, "stair_steady_points.csv"))
    bt = pts[pts["label"].str.startswith("BT_")]
    r = pf.FullPlant(Ron=0.052, steps=125).steady(bt["D0"].to_numpy(float), bt["D1"].to_numpy(float),
                                                  bt["D2"].to_numpy(float), bt["V2"].to_numpy(float))
    f = os.path.join(tempfile.mkdtemp(), "stair_steady_part001.csv")
    bt.assign(Ipk_sim=r["Ipk_sampled"], P_LV=r["P_LV"]).to_csv(f, index=False)
    real = glob.glob
    m.glob.glob = lambda p: [f] if "stair_steady_part" in p else real(p)
    T = pd.read_csv(os.path.join(m.OUT, "ipk_decompose.csv"))
    lines = m.steady_best(T, True)
    print("\n".join(lines))
    ok = True
    for ln in lines[3:]:
        v = ln.split("|")
        bst, bpy = float(v[1].split()[1]), float(v[4].split()[0])
        ok &= bool(np.isfinite(bst)) and abs(bst - bpy) < 1e-9
    assert os.path.dirname(f) != m.OUT
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
