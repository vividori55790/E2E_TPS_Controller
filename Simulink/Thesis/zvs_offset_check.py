"""zvs_offset_check.py -- the F5 audit found that the final report's "-76 ... -83 %" (hard V x I sum against the time offset between the saved waveform and the gate grid) had no file behind it and was taken with 51 points. This redoes it on the 52 points of the
report. Not a verdict and no new run: the saved waveforms of the 25 ns map (out/iL_wave/), zvs_check.py's own edge analysis, only the common offset c is moved over -1.44 ... +0.90 deg in 0.18 deg steps (the fitted c is +0.36 deg; 0.72 deg = one 25 ns grid step).

    python -I -X utf8 Simulink/Thesis/zvs_offset_check.py    -> out/zvs_offset.log
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import audit_raw as ar  # noqa: E402
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import zvs_check as zv  # noqa: E402

zv.EDGE = 360.0 * rm.TS * zv.FS
d, frame = zv.load_runs25()
import glob  # noqa: E402

import pandas as pd  # noqa: E402

rows, waves = [], {}
for pat in ("fine_sweep_r*.csv", "r13a_sweep_r*.csv", "r13b_sweep_r*.csv"):
    for f in sorted(glob.glob(os.path.join(ar.OUT, "iL_wave", pat))):
        res = os.path.join(ar.OUT, os.path.basename(f))
        if not os.path.isfile(res):
            continue
        w, r = pd.read_csv(f), pd.read_csv(res)
        if len(w) != len(r):
            continue
        W = w[[f"Wv{j}" for j in range(1, zv.N_S + 1)]].to_numpy(float)
        for k in range(len(r)):
            x = r.iloc[k]
            if not np.isclose(x["Ts"], rm.TS, rtol=1e-6, atol=0) or not np.all(np.isfinite(W[k])):
                continue
            waves[(float(x["V2"]), int(x["level"]), round(float(x["D1"]), 9), round(float(x["D2"]), 9))] = (W[k], float(x["D0"]), float(x["D1"]), float(x["D2"]))


def hard_vi(g, lo, c, sigma, rho, v):
    a, b = (g[g["level"] == j].iloc[0] for j in (lo, lo + 1))
    total = 0.0
    margins = {}
    for rec in (a, b):
        w = waves[(float(rec["V2"]), int(rec["level"]), round(float(rec["D1"]), 9), round(float(rec["D2"]), 9))]
        e = zv.edges(w[0], zv.phases(w[1], w[2], w[3]), c, sigma, rho)
        for leg, (_, m) in e.items():
            margins[leg] = min(margins.get(leg, np.inf), m)
    for leg, m in margins.items():
        if m < 0:
            total += -m * (380.0 if leg in "AB" else v)
    return total


lines = [f"zvs_offset_check: common frame sigma {frame[0]:+d}, rho {frame[1]:+d}, fitted offset c {frame[2]:+.2f} deg; 52 clean map points (SPS and the proposed controller both |mean i_L| <= 20 mA); not a verdict"]
sigma, rho, c0 = frame
tab = []
for c in np.arange(-1.44, 0.90 + 1e-9, 0.18):
    sS = sB = 0.0
    n = 0
    for v, p, cs in rm.candidates():
        by = {x["name"]: x for x in cs}
        own = {nm: by[by[nm]["alias"]] if by[nm]["alias"] else by[nm] for nm in ("SPS", "M-B")}
        gS = fg.runs_of(d, dict(V2=v, D1=own["SPS"]["D1"], D2=own["SPS"]["D2"]), rm.TS)
        gB = fg.runs_of(d, dict(V2=v, D1=own["M-B"]["D1"], D2=own["M-B"]["D2"]), rm.TS)
        iS, oS, lS = fg.at_power(gS, p)
        iB, oB, lB = fg.at_power(gB, p)
        if lS is None or lB is None or oS > 0.02 or oB > 0.02:
            continue
        sS += hard_vi(gS, lS, c, sigma, rho, v)
        sB += hard_vi(gB, lB, c, sigma, rho, v)
        n += 1
    tab.append((c, n, sB, sS, 100 * (sB / sS - 1)))
    lines.append(f"  c {c:+.2f} deg: {n} points, proposed {sB:7.0f} VA, SPS {sS:7.0f} VA, {100 * (sB / sS - 1):+.1f} %")
vals = [x[-1] for x in tab]
lines.append(f"range {min(vals):+.1f} ... {max(vals):+.1f} %")
sS = pd.read_csv(os.path.join(ar.OUT, "zvs_map25.csv"))
lines.append(f"at the fitted offset the table out/zvs_map25.csv gives {sS[(sS.ctrl == 'M-B') & (sS.osc <= 0.02)].hard_VI.sum():.0f} / {sS[(sS.ctrl == 'SPS') & (sS.osc <= 0.02)].hard_VI.sum():.0f} VA (all clean points of each, not only the common 52)")
open(os.path.join(ar.OUT, "zvs_offset.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines))
