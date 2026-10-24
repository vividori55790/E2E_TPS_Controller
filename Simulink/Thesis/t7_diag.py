"""
t7_diag.py -- where and why the ideal-trained b3 is worse than SPS on the plant (plant_verdict.py part B output).

    python -I -X utf8 Simulink/Thesis/t7_diag.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import evaluate as ev  # noqa: E402

z = np.load(os.path.join(HERE, "out", "plant_verdict_B.npz"))
P, V = z["P"], z["V"]
S, B = z["SPS"], z["Smooth_RegHDC_(b3)"]
ctrls, best = ev.build(b_tags=["3"])
c = [c for c in ctrls if c.name == "Smooth RegHDC (b3)"][0]
d1, d2 = (np.ravel(x) for x in c.predict(P, V))
dI, dIi = S[0] - B[0], S[1] - B[1]
bad = dI < -0.005
print("n bad", bad.sum())
for i in np.argsort(dI)[:25]:
    print(f"{P[i]:7.1f} {V[i]:6.2f} k={4 * V[i] / 380:.3f} D1={d1[i]:.4f} D2={d2[i]:.4f} plantSPS={S[0][i]:.3f} "
          f"plant={B[0][i]:.3f} dI={dI[i] * 1e3:+7.1f}m ideal dI={dIi[i] * 1e3:+7.1f}m idealSPS={S[1][i]:.3f}")
reg = (V > 90) & (V < 100) & (P < 450)
print("region n", reg.sum(), "mean dI plant", dI[reg].mean(), "ideal", dIi[reg].mean(), "frac bad", bad[reg].mean())
dm = 0.5 - np.minimum(d1, d2)
print("0.5 - min(D) at bad: min %.4f max %.4f; in region overall: median %.4f" % (dm[bad].min(), dm[bad].max(),
                                                                                 np.median(dm[reg])))
# plant vs ideal: gain error as a function of how far from SPS the controller is
for lo, hi in [(0, 0.005), (0.005, 0.02), (0.02, 0.05), (0.05, 0.1), (0.1, 0.4)]:
    m = (dm >= lo) & (dm < hi)
    if m.any():
        print(f"0.5-minD in [{lo},{hi}): n {m.sum():4d}  plant dI {dI[m].mean() * 1e3:+7.1f}m  ideal {dIi[m].mean() * 1e3:+7.1f}m"
              f"  plant-ideal {np.mean(dI[m] - dIi[m]) * 1e3:+6.1f}m  bad {bad[m].sum()}")
