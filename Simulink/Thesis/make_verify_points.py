"""
make_verify_points.py -- T6: operating points for the steady-state Simulink check (verify_steady_state.m).

Writes out/verify_points.csv with columns label, V2, D0, D1, D2 and the analytic prediction (P in the dataset sign
convention, Ipk, Irms), so the MATLAB side only runs the model and the comparison is done back in Python.
  SPS   : V2 84 / 95 / 109.2 V x D0 +-8, +-20, +-35 deg
  TPS   : the v2 surface (D1, D2) at |P| 100, 300, 600, 1000 W, V2 84 / 95.2 / 109.2 V, D0 from the PI steady state,
          both power directions (P < 0: phi >= 0 branch; P > 0: mirrored branch)
  ICROS : the ICROS2026 dataset's "optimal" (D1, D2) at V2 90 / 100 V, P = -1200 W and its largest claimed gains
          (where the dataset's 1 ms staircase SPS reference looked worst), D0 from the PI steady state
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402
from smooth_surface_v2 import bilinear  # noqa: E402

OUT = os.path.join(HERE, "out")


def d0_both(P_abs, D1, D2, V2):
    """D0 for P = -|P| (phi >= 0 branch) and for P = +|P| (mirror: phi -> -phi)."""
    r = ps.operating_point(np.atleast_1d(P_abs), np.atleast_1d(D1), np.atleast_1d(D2), np.atleast_1d(V2))
    phi = float(r["phi"][0])
    sh = 180.0 * (D2 - D1)
    return phi - sh, -phi - sh


def main():
    rows = []
    for V2 in (84.0, 95.0, 109.2):
        for D0 in (-35.0, -20.0, -8.0, 8.0, 20.0, 35.0):
            rows.append(("SPS", V2, D0, 0.5, 0.5))
    S = np.load(os.path.join(OUT, "surface_v2.npz"))
    for V2 in (84.0, 95.2, 109.2):
        for Pa in (100.0, 300.0, 600.0, 1000.0):
            d1 = float(bilinear(S["smooth_D1"], S["P"], S["V2"], np.array([Pa]), np.array([V2]))[0])
            d2 = float(bilinear(S["smooth_D2"], S["P"], S["V2"], np.array([Pa]), np.array([V2]))[0])
            if d1 > 0.499 and d2 > 0.499:
                continue                                    # SPS there; covered above
            neg, pos = d0_both(Pa, d1, d2, V2)
            rows.append((f"TPS v2 |P|={Pa:.0f} P<0", V2, neg, d1, d2))
            rows.append((f"TPS v2 |P|={Pa:.0f} P>0", V2, pos, d1, d2))
    d = pd.read_csv(os.path.join(ROOT, "Simulink", "RegHDC_PostProcessed_Dataset.csv"))
    d = d.assign(gain=d["I_SPS"] - d["I_Opt"])
    pick = pd.concat([d[(d["Target_P_W"] == -1200) & d["Target_V2_V"].isin([90, 100])], d.nlargest(4, "gain")])
    for _, r in pick.drop_duplicates().iterrows():
        neg, pos = d0_both(abs(r["Target_P_W"]), r["Opt_D1"], r["Opt_D2"], r["Target_V2_V"])
        D0 = neg if r["Target_P_W"] < 0 else pos
        rows.append((f"ICROS P={r['Target_P_W']:.0f}", float(r["Target_V2_V"]), D0, float(r["Opt_D1"]),
                     float(r["Opt_D2"])))
        if r["Opt_D1"] < 0.5 or r["Opt_D2"] < 0.5:          # and the SPS point at the same power for reference
            sn, sp = d0_both(abs(r["Target_P_W"]), 0.5, 0.5, r["Target_V2_V"])
            rows.append((f"ICROS-SPS P={r['Target_P_W']:.0f}", float(r["Target_V2_V"]),
                         sn if r["Target_P_W"] < 0 else sp, 0.5, 0.5))
    df = pd.DataFrame(rows, columns=["label", "V2", "D0", "D1", "D2"])
    e = tm.evaluate(df["D0"].to_numpy(), df["D1"].to_numpy(), df["D2"].to_numpy(), df["V2"].to_numpy())
    df = df.assign(P_model=e["P"], Ipk_model=e["Ipk"], Irms_model=e["Irms"])
    df.to_csv(os.path.join(OUT, "verify_points.csv"), index=False, float_format="%.6g")
    print(df.to_string())


if __name__ == "__main__":
    main()
