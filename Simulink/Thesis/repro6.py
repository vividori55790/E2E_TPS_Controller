"""
repro6.py -- (2026-10-31, final deadline F5 c) does the headline table come out the same from a new MATLAB session?

Runs: the exact rows (D0, D1, D2, V2, Ts) behind the headline points of the 25 ns map, 84 and 109.2 V x 100, 300, 1000 W x {SPS, M-B} x their two bracketing levels, rerun with the same runner verify_steady_fine_batch.m in a fresh session, 20 ms.

RULE (registered 2026-10-31 14:41, before any run of this file; do not change after the results). Per row, on the columns the map uses (Ipk_at_20ms, P_LV_at_20ms, P_HV): "같음" iff max |dIpk| <= 1e-6 A and max |dP| <= 1e-6 W over all rows, "격자 안" iff max |dIpk| <= 0.001 A and max |dP| <= 0.1 W, else "다름". Reported: the largest differences and which row.

    python -I -X utf8 Simulink/Thesis/repro6.py make       -> out/repro6_points.csv
    python automation/job.py submit repro6 --interactive --after f1icros -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); verify_steady_fine_batch('out/repro6_points.csv', 'out/repro6_sweep.csv', 12, 0.020)"
    python -I -X utf8 Simulink/Thesis/repro6.py compare    -> out/repro6_check.log
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402

OUT = os.path.join(HERE, "out")
TS = rm.TS
PTS = tuple((v, p) for v in (84.0, 109.2) for p in (100.0, 300.0, 1000.0))
KEYS = ["V2", "level", "D1", "D2"]
PC, IC = f"P_LV_at_{fg.WIN}ms", f"Ipk_at_{fg.WIN}ms"


def original():
    """the map's runs and the configurations: (config name, V2, P, level) rows"""
    import rev25 as rv
    d = rm.results_25()
    rows = []
    for c in rv.configs():                 # same SPS and M-B duties as the map's, the way the map rounds them
        if (c["V2"], c["P"]) not in PTS:
            continue
        g = fg.runs_of(d, c, TS)
        lo = fg.at_power(g, c["P"])[2]
        if lo is None:
            sys.exit(f"the map does not bracket {c}")
        for j in (lo, lo + 1):
            r = g[g["level"] == j].iloc[0]
            rows.append(dict(label=f"R {c['ctrl']} {c['P']:g}W {c['V2']:g}V L{j} T25", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], level=j, D0=r["D0"], D1=r["D1"], D2=r["D2"], Ts=TS, Ipk0=r[IC], P_LV0=r[PC], P_HV0=r["P_HV"], src=r["src"]))
    return rows


def make():
    import pandas as pd
    rows = original()
    T = pd.DataFrame(rows).drop_duplicates(subset=KEYS)
    T[["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2", "Ts"]].to_csv(os.path.join(OUT, "repro6_points.csv"), index=False, float_format="%.10g")
    print(f"wrote out/repro6_points.csv: {len(T)} runs from {sorted(set(T['src']))}")


def rule(dI, dP):
    dI, dP = np.nanmax(np.abs(dI)), np.nanmax(np.abs(dP))
    return "같음" if dI <= 1e-6 and dP <= 1e-6 else ("격자 안" if dI <= 0.001 and dP <= 0.1 else "다름")


def compare():
    import pandas as pd
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "repro6_sweep_part*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        sys.exit("no out/repro6_sweep_part*.csv yet")
    N = fg._load(fs, fine=True)
    O = pd.DataFrame(original()).drop_duplicates(subset=KEYS)
    lines = ["repro6: the headline rows of the 25 ns map rerun in a new MATLAB session, same runner, rule in the file header", ""]
    dI, dP, dH, miss = [], [], [], 0
    for r in O.itertuples():
        x = N[np.isclose(N["V2"], r.V2) & (N["level"] == r.level) & (np.abs(N["D1"] - r.D1) <= 1e-9) & (np.abs(N["D2"] - r.D2) <= 1e-9)]
        if x.empty:
            miss += 1
            continue
        a, b, c = x[IC].iloc[0] - r.Ipk0, x[PC].iloc[0] - r.P_LV0, x["P_HV"].iloc[0] - r.P_HV0
        dI.append(a)
        dP.append(max(abs(b), abs(c)))
        lines.append(f" {r.label:28s} Ipk {r.Ipk0:.6f} -> {x[IC].iloc[0]:.6f} ({a:+.2e} A), P_LV {r.P_LV0:.4f} -> {x[PC].iloc[0]:.4f} ({b:+.2e} W), P_HV ({c:+.2e} W)")
    v = rule(np.array(dI), np.array(dP)) if dI else "판정 불가"
    lines += ["", f"{len(dI)} of {len(O)} rows rerun, max |dIpk| {np.max(np.abs(dI)) if dI else np.nan:.3e} A, max |dP| {np.max(dP) if dP else np.nan:.3e} W", f"VERDICT: {v}" + (f" ({miss} rows missing)" if miss else "")]
    print("\n".join(lines))
    open(os.path.join(OUT, "repro6_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "compare"])
    a = ap.parse_args()
    {"make": make, "compare": compare}[a.cmd]()
