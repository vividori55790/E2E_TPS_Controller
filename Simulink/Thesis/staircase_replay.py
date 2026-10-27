"""
staircase_replay.py -- R5 supporting evidence while MATLAB is unavailable (2026-10-27): the same 5 (V2, D1, D2)
combinations as staircase_check.m, replayed through the Python plant (transient_replay.replay, the configuration that
fitted the dataset with RMSE 0.525 A on 2000 combinations: Ron 52 mOhm, droop, 2.88 deg edge grid, 125 steps/period).
  linear  D0 interpolated between the staircase points (what the Simulink block does: interp = 1)
  zoh     a true staircase (interp = 0)
  steady  the plant's periodic steady state at each D0 label (plant_full.steady)
Checks: does 'linear' reproduce the Simulink interp = 1 rerun (out/staircase_check.csv)? does 'zoh' reach the steady
state within each 1 ms step? Then the same-power SPS - TPS table (staircase_compare.crossing) for the three modes.
The Python plant is NOT the deciding evidence (Simulink is); this only says what to expect from interp = 0.

    python -I -X utf8 Simulink/Thesis/staircase_replay.py      -> out/staircase_replay.{log,csv}
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import transient_replay as tr  # noqa: E402
from staircase_compare import TARGETS, crossing  # noqa: E402

OUT = os.path.join(HERE, "out")


def main():
    import pandas as pd
    t0 = time.time()
    s1 = pd.read_csv(os.path.join(OUT, "staircase_check.csv"))
    s1 = s1[s1["interp"] == 1]
    tags = list(dict.fromkeys(s1["tag"]))
    c = s1.groupby("tag").first().loc[tags]
    V2, D1, D2 = c["V2"].to_numpy(float), c["D1"].to_numpy(float), c["D2"].to_numpy(float)
    g = tr.d0_grid()
    plant = pf.FullPlant(Ron=0.052, steps=125)
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"Python plant {plant}; combos: {', '.join(tags)}")
    res = {}
    for mode in ("linear", "zoh"):
        tm0 = time.time()
        I, PL, _ = tr.replay(plant, V2, D1, D2, mode)
        res[mode] = (I, PL)
        out(f"replay {mode}: {time.time() - tm0:.0f} s")
    tm0 = time.time()
    lab = np.repeat(g[:, None], V2.size, axis=1)
    r = plant.steady(lab.ravel(), np.tile(D1, g.size), np.tile(D2, g.size), np.tile(V2, g.size))
    # max|i| on the 100 ns samples, as the replay and Simulink measure it
    res["steady"] = (r["Ipk_sampled"].reshape(g.size, -1), r["P_LV"].reshape(g.size, -1))
    out(f"steady states at the labels: {time.time() - tm0:.0f} s")

    for mode, (I, PL) in res.items():
        for k, t in enumerate(tags):
            for d in range(g.size):
                rows.append(dict(tag=t, mode=mode, V2=V2[k], D1=D1[k], D2=D2[k], step=d + 1, D0_label=g[d],
                                 Ipk=I[d, k], P_LV=PL[d, k]))
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, "staircase_replay.csv"), index=False)

    out("\nPython 'linear' replay vs the Simulink interp = 1 rerun (= the dataset), per combination:")
    for k, t in enumerate(tags):
        sim = s1[s1["tag"] == t].sort_values("step")
        I, PL = res["linear"]
        eI = I[:, k] - sim["Ipk"].to_numpy()
        eP = PL[:, k] - sim["P_LV"].to_numpy()
        out(f"  {t:24s} Ipk bias {eI.mean():+.3f} rmse {np.sqrt(np.mean(eI ** 2)):.3f} A | P bias {eP.mean():+6.1f} "
            f"rmse {np.sqrt(np.mean(eP ** 2)):6.1f} W")
    out("\nPython 'zoh' (true staircase) vs Python steady state at the same label:")
    for k, t in enumerate(tags):
        eI = res["zoh"][0][:, k] - res["steady"][0][:, k]
        eP = res["zoh"][1][:, k] - res["steady"][1][:, k]
        m = np.abs(g) <= 32
        out(f"  {t:24s} |D0| <= 32: Ipk max|diff| {np.abs(eI[m]).max():.3f} A, mean {eI[m].mean():+.3f} | "
            f"P max|diff| {np.abs(eP[m]).max():5.1f} W")

    out("\nIpk(SPS) - Ipk(TPS) at the same measured power (Python plant), modes linear | zoh | steady:")
    for V2v in sorted(set(V2)):
        sps = [t for k, t in enumerate(tags) if t.startswith("SPS") and np.isclose(V2[k], V2v)]
        tps = [t for k, t in enumerate(tags) if not t.startswith("SPS") and np.isclose(V2[k], V2v)]
        for t in tps:
            for s, lab_ in ((+1, "P<0"), (-1, "P>0")):
                for P in TARGETS:
                    cells = []
                    for mode in ("linear", "zoh", "steady"):
                        a = T[(T["tag"] == sps[0]) & (T["mode"] == mode)]
                        b = T[(T["tag"] == t) & (T["mode"] == mode)]
                        Ia = crossing(a["D0_label"].to_numpy(), a["P_LV"].to_numpy(), a["Ipk"].to_numpy(), P, s)[0]
                        Ib = crossing(b["D0_label"].to_numpy(), b["P_LV"].to_numpy(), b["Ipk"].to_numpy(), P, s)[0]
                        cells.append(f"{Ia:5.2f}-{Ib:5.2f}={Ia - Ib:+5.2f}" if np.isfinite(Ia - Ib) else f"{'n/a':>17s}")
                    out(f"  {V2v:6.1f} {t:24s} {lab_} {P:5.0f} | " + " | ".join(cells))
    out(f"\ntotal {time.time() - t0:.0f} s")
    open(os.path.join(OUT, "staircase_replay.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
