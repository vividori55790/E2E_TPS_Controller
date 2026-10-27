"""
fig_surface3d.py -- inner-phase control surfaces in the style of ICROS2026 Fig. 2/3 (3-D, turbo, z 0.1-0.5).

Row 1: proposed M-B (Smooth RegHDC b3: steady-state optimum + physics E2E), one continuous regression of (|P|, V2).
Row 2: the existing dataset target (Simulink/Physics_Dataset_Final_Regression_Ready.csv, the file RegHDCTrainer.m reads).
Output: out/fig_surface3d.png
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402

OUT = os.path.join(HERE, "out")


def main():
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    P = np.linspace(-1200, 1200, 121)
    V = np.linspace(84.0, 109.2, 64)
    PP, VV = np.meshgrid(P, V)
    d1, d2 = mb.predict(np.abs(PP), VV)

    ds = pd.read_csv(os.path.join(HERE, "..", "Physics_Dataset_Final_Regression_Ready.csv"))
    Pd = np.sort(ds.Target_P_W.unique())
    Vd = np.sort(ds.Target_V2_V.unique())
    T1 = ds.pivot(index="Target_V2_V", columns="Target_P_W", values="Opt_D1").loc[Vd, Pd].values
    T2 = ds.pivot(index="Target_V2_V", columns="Target_P_W", values="Opt_D2").loc[Vd, Pd].values
    PPd, VVd = np.meshgrid(Pd, Vd)

    fig = plt.figure(figsize=(13, 10.5))
    panels = [(PP, VV, d1, "Proposed M-B (single E2E regression) - $D_1$"),
              (PP, VV, d2, "Proposed M-B (single E2E regression) - $D_2$"),
              (PPd, VVd, T1, "Existing dataset target - $D_1$"),
              (PPd, VVd, T2, "Existing dataset target - $D_2$")]
    for k, (X, Y, Z, title) in enumerate(panels, 1):
        ax = fig.add_subplot(2, 2, k, projection="3d")
        ax.plot_surface(X, Y, Z, cmap="turbo", vmin=0.1, vmax=0.5, rstride=1, cstride=1, linewidth=0, antialiased=True)
        ax.set_zlim(0.1, 0.5)
        ax.set_xlabel("Power P (W)")
        ax.set_ylabel("Voltage $V_2$ (V)")
        ax.set_zlabel(title.split(" - ")[1])
        ax.set_title(title, fontsize=11)
        ax.view_init(elev=28, azim=-135)
    fig.tight_layout()
    path = os.path.join(OUT, "fig_surface3d.png")
    fig.savefig(path, dpi=130)
    # a few numbers for the text
    print(f"M-B: D1 range {d1.min():.3f}-{d1.max():.3f}, D2 range {d2.min():.3f}-{d2.max():.3f}; "
          f"max adjacent step on this grid (20 W / 0.4 V): "
          f"{max(np.abs(np.diff(d1, axis=1)).max(), np.abs(np.diff(d2, axis=1)).max(), np.abs(np.diff(d1, axis=0)).max(), np.abs(np.diff(d2, axis=0)).max()):.4f}")
    for V2 in (84.0, 95.0, 109.2):
        a, b = mb.predict(np.array([0.0, 100, 300, 600, 1000]), np.full(5, V2))
        print(f"  V2 {V2:5.1f} V  |P| 0/100/300/600/1000 W:  D1 " + " ".join(f"{x:.3f}" for x in a) +
              "   D2 " + " ".join(f"{x:.3f}" for x in b))
    print("wrote", path)


if __name__ == "__main__":
    main()
