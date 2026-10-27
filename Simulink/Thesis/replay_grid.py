"""
replay_grid.py -- R6/R7 supporting evidence while MATLAB waits for a sign-in (2026-10-27): the R6 grid
(gen_dataset_fixed.m defaults: V2 = V2_vec[1, 9, 20] = 84 / 94.61 / 109.2 V, D1, D2 = Dg[1:2:19, 20], 11 x 11) replayed
through the Python plant (transient_replay.replay: same 44 ms, 39-step run, same windows) with D0
  zoh     a true staircase (= the fixed dataset gen_dataset_fixed.m will make in Simulink)
  linear  the block's interpolation (= how the original dataset was made)
Written in gen_dataset_fixed.m's CSV format, so dataset_fixed_analyze.py reads it with --prefix.
The Python plant is NOT the deciding evidence; Simulink is.

    python -I -X utf8 Simulink/Thesis/replay_grid.py   -> out/dataset_fixed_py_{zoh,linear}_part_001.csv
"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import transient_replay as tr  # noqa: E402

OUT = os.path.join(HERE, "out")


def grid():
    V2v = np.linspace(84, 109.2, 20)[[0, 8, 19]]
    Dg = np.linspace(0.2, 0.5, 20)[list(range(0, 19, 2)) + [19]]
    V, A, B = np.meshgrid(V2v, Dg, Dg, indexing="ij")
    return V.ravel(), A.ravel(), B.ravel()


def main():
    import pandas as pd
    V2, D1, D2 = grid()
    g = tr.d0_grid()
    plant = pf.FullPlant(Ron=0.052, steps=125)
    print(f"{V2.size} combos, plant {plant}", flush=True)
    for mode in ("zoh", "linear"):
        t0 = time.time()
        I, PL, PH = tr.replay(plant, V2, D1, D2, mode)
        rows = []
        for k in range(V2.size):
            for d in range(g.size):
                rows.append(dict(combo=k + 1, V2=V2[k], D1=D1[k], D2=D2[k], step=d + 1, D0=g[d], I_pk=I[d, k],
                                 P_act=PL[d, k], P_act_HV=PH[d, k], valid=1))
        f = os.path.join(OUT, f"dataset_fixed_py_{mode}_part_001.csv")
        pd.DataFrame(rows).to_csv(f, index=False, float_format="%.15g")
        print(f"{mode}: {time.time() - t0:.0f} s -> {f}", flush=True)


if __name__ == "__main__":
    main()
