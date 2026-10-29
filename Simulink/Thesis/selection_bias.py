"""
selection_bias.py -- (2026-10-29) why the dataset's best-TPS rows look like steady state while its SPS rows do not,
from Simulink alone (out/staircase_compare.csv = staircase_check.m, interp 1 = the dataset run, interp 0 = true steps).

At the same measured power, inflation = Ipk(dataset run, D0 block interpolating) - Ipk(true 1 ms staircase), for the SPS
rows and for each TPS combination. The TPS combinations were picked from the dataset as the lowest Ipk at one
(V2, |P|) (tag TPS_best_<V2>V_<P>W, HV->LV). If the dataset's "best" is the minimum over ~400 noisy rows, the winning
combination is inflated least exactly at the power where it won and more elsewhere; SPS is one combination (nothing
is selected) and shows its inflation in full.

    python -I -X utf8 Simulink/Thesis/selection_bias.py   -> out/selection_bias.{log,csv}
"""
import os
import re

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
DS, ST0 = "dataset(interp=1)", "staircase(interp=0)"


def main():
    c = pd.read_csv(os.path.join(OUT, "staircase_compare.csv"))
    c = c[c["part"] == "c"]
    a = c[c["source"] == DS].set_index(["tag", "direction", "P"])
    b = c[c["source"] == ST0].set_index(["tag", "direction", "P"])
    j = a[["V2", "Ipk_SPS", "Ipk_TPS"]].join(b[["Ipk_SPS", "Ipk_TPS"]], rsuffix="_st0", how="inner").reset_index()
    j["infl_SPS"] = j["Ipk_SPS"] - j["Ipk_SPS_st0"]
    j["infl_TPS"] = j["Ipk_TPS"] - j["Ipk_TPS_st0"]
    win = j["tag"].str.extract(r"TPS_best_([\d.]+)V_(\d+)W").astype(float)
    j["win"] = np.isclose(j["P"], win[1]) & (j["direction"] == "P<0")
    j.to_csv(os.path.join(OUT, "selection_bias.csv"), index=False)

    L = ["selection_bias.py: inflation = Ipk(dataset run, D0 interpolating) - Ipk(true 1 ms staircase), same measured "
         "power, Simulink (staircase_check.m); * = the (power, HV->LV) where the dataset picked this combination as best",
         f"{'TPS combination':24s} {'dir':4s} {'|P|':>5s} | {'TPS: data - true':>18s} | {'SPS: data - true':>18s}"]
    for r in j.sort_values(["tag", "direction", "P"]).itertuples():
        L.append(f"{r.tag:24s} {r.direction:4s} {r.P:5.0f} | {r.Ipk_TPS:5.2f}-{r.Ipk_TPS_st0:5.2f}={r.infl_TPS:+5.2f}"
                 f"{'*' if r.win else ' '} | {r.Ipk_SPS:5.2f}-{r.Ipk_SPS_st0:5.2f}={r.infl_SPS:+5.2f}")
    w, o = j[j["win"]], j[~j["win"]]
    sps = j.drop_duplicates(["V2", "direction", "P"])["infl_SPS"]
    L += ["", "summary (A): mean / max inflation",
          f"  TPS at the winning cell    ({len(w):2d}): {w['infl_TPS'].mean():+.2f} / {w['infl_TPS'].max():+.2f}",
          f"  TPS everywhere else        ({len(o):2d}): {o['infl_TPS'].mean():+.2f} / {o['infl_TPS'].max():+.2f}",
          f"  SPS (one combination)      ({len(sps):2d}): {sps.mean():+.2f} / {sps.max():+.2f}"]
    for t, g in j.groupby("tag"):
        L.append(f"  {t}: winning cell {g.loc[g['win'], 'infl_TPS'].mean():+.2f}, "
                 f"other cells mean {g.loc[~g['win'], 'infl_TPS'].mean():+.2f}, max {g.loc[~g['win'], 'infl_TPS'].max():+.2f}")
    txt = "\n".join(L)
    print(txt)
    with open(os.path.join(OUT, "selection_bias.log"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")


if __name__ == "__main__":
    main()
