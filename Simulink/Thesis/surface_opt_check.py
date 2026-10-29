"""
surface_opt_check.py -- OPT1-OPT4 (user, 2026-10-29 19:10): is the proposed surface M-B (out/model_smooth_reghdc_b3.json)
really the optimal surface -- in Simulink? Rules fixed beforehand in surface_opt_verdict.py (OPT0).

Every number that enters a verdict is a Simulink steady state measured the R.4 way (simulink_sweep.py): constant D0 for
30 ms (k1: 40 ms, its light-load transients are slow -- R.8 Q3 saw 30 ms vs 20 ms differ by up to 0.2 A), last 1 ms,
two adjacent 2.88-deg D0 levels whose measured powers bracket |P|, Ipk interpolated in the measured power. HV -> LV.
The Python plant only picks the two D0 levels; the R6 1 ms staircase only picks the grid combinations to run.

Families (one points file per family and round, out/opt_points_<fam>_r<k>.csv -> out/opt_sweep_<fam>_r<k>_partNNN.csv):
  mb    OPT3: V2 {84, 109.2} x |P| {100, 300, 1000} W: M-B, SPS, M-B +-0.02 / +-0.04 in D1 and in D2 (the user's 9-point
        cross), plus two candidates the cross cannot reach: both duties +0.04 / +0.08 (R.8 Q2 found the dataset combo
        (0.247, 0.200) 0.20 A below M-B (0.199, 0.167) at 109.2 V 100 W, i.e. larger duties), plus that Q2 combo itself
        at its three points (BT). 30 ms. R.4's own runs (simulink_sweep_r*, eff_sweep_r*) are reused, not rerun.
  k1    OPT2: V2 95.0 x |P| {100, 300, 600} W: D1 = D2 in {SPS, 0.45, 0.40, 0.35, 0.30, 0.25, 0.20}, (0.45, 0.5),
        (0.5, 0.45), (0.40, 0.5), (0.5, 0.40), M-B. 40 ms, own runs only.
  grid  OPT1: V2 {84, 109.2} x |P| {100, 150, 200, 300, 400, 600, 800, 1000} W: the 3 lowest-Ipk TPS combinations of the
        R6 grid at that power (true 1 ms staircase, rising crossing closest to D0 = 0), SPS, M-B. 30 ms. Needs R6 to
        have all 121 combinations at that V2 (else that V2 is skipped; --partial uses what is there, for testing).
Combinations a 2.88-deg grid cannot reach (ideal model: |P| above the combination's maximum) are dropped and listed.

    python -I -X utf8 Simulink/Thesis/surface_opt_check.py make [--fams mb,k1,grid] [--partial]
    python automation/job.py submit opt1 --interactive -- matlab -batch "cd('Simulink/Thesis'); opt_chain(1)"
    python -I -X utf8 Simulink/Thesis/surface_opt_check.py next       (configurations not bracketed -> round k+1)
    python -I -X utf8 Simulink/Thesis/surface_opt_check.py compare    -> out/surface_opt_check.{csv,log},
                                                                         out/surface_opt_verdict.md
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import simulink_sweep as sw  # noqa: E402
import surface_opt_verdict as sv  # noqa: E402

OUT = os.path.join(HERE, "out")
FAMS = ("mb", "k1", "grid")
SIM_T = {"mb": 0.030, "k1": 0.040, "grid": 0.030}
MB_P = (100.0, 300.0, 1000.0)
K1_P = (100.0, 300.0, 600.0)
GRID_P = (100.0, 150.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)
GRID_TOP = 3
R6_PER_V2 = 121
BT = {(109.2, 100.0): (0.247368421, 0.2), (84.0, 300.0): (0.373684211, 0.389473684),
      (84.0, 100.0): (0.215789474, 0.231578947)}                    # R.8 Q2: dataset best-TPS combos (BT_ labels)


def _mb():
    return ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))


def _d(x):
    return round(min(max(float(x), 0.02), sw.D_SAFE), 9)


def _cfg(fam, ctrl, V2, P, D1, D2):
    return dict(fam=fam, ctrl=ctrl, V2=V2, P=P, D1=_d(D1), D2=_d(D2))


def mb_predict(P, V2):
    d1, d2 = _mb().predict(np.array([P]), np.array([V2]))
    return float(np.ravel(d1)[0]), float(np.ravel(d2)[0])


def _feasible(cf):
    """drop configurations whose |P| the ideal model cannot reach on the first branch (D0 NaN)"""
    if not cf:
        return cf, []
    P, V, D1, D2 = (np.array([c[k] for c in cf]) for k in ("P", "V2", "D1", "D2"))
    ok = np.isfinite(ps.operating_point(P, D1, D2, V)["D0"])
    return [c for c, o in zip(cf, ok) if o], [c for c, o in zip(cf, ok) if not o]


def _dedupe(cf):
    seen, out = set(), []
    for c in cf:
        k = (c["fam"], c["V2"], c["P"], c["D1"], c["D2"])
        if k not in seen:
            seen.add(k)
            out.append(c)
    return out


def configs_mb():
    cf = []
    for V2 in (84.0, 109.2):
        for P in MB_P:
            d1, d2 = mb_predict(P, V2)
            cf += [_cfg("mb", "M-B", V2, P, d1, d2), _cfg("mb", "SPS", V2, P, 0.5, 0.5)]
            for s in (-0.04, -0.02, 0.02, 0.04):
                cf += [_cfg("mb", f"D1{s:+.2f}", V2, P, d1 + s, d2), _cfg("mb", f"D2{s:+.2f}", V2, P, d1, d2 + s)]
            for s in (0.04, 0.08):
                cf.append(_cfg("mb", f"both{s:+.2f}", V2, P, d1 + s, d2 + s))
            if (V2, P) in BT:
                cf.append(_cfg("mb", "BT", V2, P, *BT[(V2, P)]))
    return cf


def configs_k1():
    cf = []
    for P in K1_P:
        d1, d2 = mb_predict(P, 95.0)
        cf += [_cfg("k1", "M-B", 95.0, P, d1, d2), _cfg("k1", "SPS", 95.0, P, 0.5, 0.5)]
        for d in (0.45, 0.40, 0.35, 0.30, 0.25, 0.20):
            cf.append(_cfg("k1", f"D={d:.2f}", 95.0, P, d, d))
        for a, b in ((0.45, 0.5), (0.5, 0.45), (0.40, 0.5), (0.5, 0.40)):
            cf.append(_cfg("k1", f"({a:.2f},{b:.2f})", 95.0, P, a, b))
    return cf


def r6_staircase(partial=False):
    """{V2: {(D1, D2): (D0, P, Ipk)}} from the R6 parts, only V2 with all combinations unless partial"""
    import dataset_fixed_analyze as dfa
    d = dfa.load_fixed(os.path.join(OUT, "dataset_fixed"))
    if d is None:
        return {}, {}
    out, count = {}, {}
    for V2 in (84.0, 109.2):
        R = dfa.runs(d, V2)
        count[V2] = len(R)
        if len(R) >= R6_PER_V2 or (partial and R):
            out[V2] = R
    return out, count


def grid_top(R, P, n=GRID_TOP):
    """the n lowest staircase Ipk TPS combinations at |P| (HV->LV, rising crossing closest to D0 = 0)"""
    from staircase_compare import crossing
    rows = []
    for k, (D0, Pm, I) in R.items():
        if k == (0.5, 0.5):
            continue
        Ic = crossing(D0, Pm, I, P, +1)[0]
        if np.isfinite(Ic):
            rows.append((Ic, k))
    return sorted(rows)[:n]


def configs_grid(partial=False, verbose=True):
    R6, count = r6_staircase(partial)
    if verbose:
        print("R6 combinations per V2: " + ", ".join(f"{v:g} V {n}/{R6_PER_V2}" for v, n in count.items())
              + ("" if R6 else " -> grid family skipped"))
    cf = []
    for V2, R in R6.items():
        for P in GRID_P:
            d1, d2 = mb_predict(P, V2)
            cf += [_cfg("grid", "M-B", V2, P, d1, d2), _cfg("grid", "SPS", V2, P, 0.5, 0.5)]
            for i, (Ic, (a, b)) in enumerate(grid_top(R, P)):
                c = _cfg("grid", f"G{i + 1}", V2, P, a, b)
                c["Ipk_stair"] = Ic
                cf.append(c)
    return cf


def configs(fams=FAMS, partial=False, verbose=True):
    cf = []
    if "mb" in fams:
        cf += configs_mb()
    if "k1" in fams:
        cf += configs_k1()
    if "grid" in fams:
        cf += configs_grid(partial, verbose)
    cf, bad = _feasible(_dedupe(cf))
    if verbose and bad:
        print(f"{len(bad)} configurations out of reach (ideal model), dropped: "
              + "; ".join(f"{c['fam']} {c['V2']:g} V {c['P']:.0f} W {c['ctrl']} ({c['D1']:.3f}, {c['D2']:.3f})"
                          for c in bad))
    return cf


def result_files(fam):
    fs = glob.glob(os.path.join(OUT, f"opt_sweep_{fam}_r*.csv"))
    if fam != "k1":                                           # same method and 30 ms: R.4 and R8 runs are reused
        fs += glob.glob(os.path.join(OUT, "simulink_sweep_r*.csv")) + glob.glob(os.path.join(OUT, "eff_sweep_r*.csv"))
        fs += glob.glob(os.path.join(OUT, f"opt_sweep_{'grid' if fam == 'mb' else 'mb'}_r*.csv"))
    return [f for f in sorted(fs) if not f.endswith("_in.csv")]


def results(fam):
    import pandas as pd
    fs = result_files(fam)
    if not fs:
        return None
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["level"] = np.round((d["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last")


def round_no(fam):
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, f"opt_sweep_{fam}_r*.csv"))
          for m in [re.search(rf"opt_sweep_{fam}_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def plan(fams, partial):
    """points still needed per family: fresh configurations -> predicted levels; unbracketed -> one level further"""
    cf = configs(fams, partial)
    for fam in fams:
        fc = [c for c in cf if c["fam"] == fam]
        if not fc:
            continue
        d = results(fam)
        pts = []
        fresh = [c for c in fc if d is None or sw.runs_of(d, c).empty]
        for c, (j0, j1) in zip(fresh, sw.predicted_levels(fresh) if fresh else []):
            pts += [sw.point(c, j0), sw.point(c, j1)]
        n_b = 0
        for c in fc:
            if d is None:
                continue
            g = sw.runs_of(d, c)
            if g.empty:
                continue
            b, way = sw.bracket(g, c["P"])
            if b is None:
                j = int(g["level"].max() + 1) if way > 0 else int(g["level"].min() - 1)
                pts += [sw.point(c, j), sw.point(c, j + way)]
            else:
                n_b += 1
        print(f"[{fam}] {len(fc)} configurations, {n_b} already bracketed by existing Simulink runs")
        if pts:
            done = [] if d is None else list(zip(d["V2"], d["D0"], d["D1"], d["D2"]))
            sw.write(os.path.join(OUT, f"opt_points_{fam}_r{round_no(fam)}.csv"), pts, done)
        else:
            print(f"[{fam}] nothing to run")


def measure(fams=FAMS, partial=False):
    """one row per configuration: Simulink steady Ipk at |P| (NaN if not bracketed), osc, settle"""
    import pandas as pd
    rows = []
    cf = configs(fams, partial, verbose=False)
    for fam in fams:
        d = results(fam)
        for c in (c for c in cf if c["fam"] == fam):
            rec = dict(c, Ipk=np.nan, osc=np.nan, settle=np.nan, src=fam)
            g = sw.runs_of(d, c) if d is not None else None
            b = sw.bracket(g, c["P"])[0] if g is not None and not g.empty else None
            if b is not None:
                lo, hi = b
                t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
                rec.update(Ipk=lo["Ipk_sim"] + t * (hi["Ipk_sim"] - lo["Ipk_sim"]),
                           osc=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])),
                           settle=max(abs(lo["Ipk_sim"] - lo["Ipk_at_25ms"]), abs(hi["Ipk_sim"] - hi["Ipk_at_25ms"])),
                           P_lo=lo["Pd"], P_hi=hi["Pd"])
            rows.append(rec)
    return pd.DataFrame(rows)


def grid_optimum(T):
    """Simulink grid optimum per (V2, P) of the grid family: min over clean G* and SPS steady values; only where at
    least one G* combination has been measured (else SPS would win by default)"""
    import pandas as pd
    t = T[(T["fam"] == "grid") & (T["ctrl"] != "M-B")]
    meas = {k for k, r in t.groupby(["V2", "P"]) if np.isfinite(r.loc[r["ctrl"].str.startswith("G"), "Ipk"]).any()}
    g = t[np.isfinite(t["Ipk"]) & (t["osc"] <= sv.OSC)]
    rows = [r.loc[r["Ipk"].idxmin(), ["V2", "P", "D1", "D2", "Ipk", "ctrl"]] for k, r in g.groupby(["V2", "P"])
            if k in meas]
    return pd.DataFrame(rows).reset_index(drop=True) if rows else pd.DataFrame(columns=["V2", "P", "D1", "D2", "Ipk"])


def compare(partial=False):
    T = measure(FAMS, partial)
    T.to_csv(os.path.join(OUT, "surface_opt_check.csv"), index=False, float_format="%.6g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    # rule (i) over every 30 ms candidate (mb + grid) at each (V2, P); the same (V2, P, D1, D2) once
    c30 = T[T["fam"].isin(["mb", "grid"])].sort_values("fam").drop_duplicates(["V2", "P", "D1", "D2"], keep="first")
    P1, code1 = sv.judge_points(c30)
    out("OPT rule (i): Ipk(M-B) - min over the Simulink candidates at the same power (steady, HV->LV) <= "
        "max(0.03 A, 2 % Ipk_SPS)")
    out(f"{'V2':>6} {'|P|':>5} | {'SPS':>6} {'M-B':>6} {'min':>6} best (D1, D2) ctrl | {'gap':>6} {'tol':>5} | "
        f"within tol (n, D1 range, D2 range) | verdict")
    for _, r in P1.iterrows():
        if r["verdict"] == "undecidable":
            out(f"{r['V2']:6g} {r['P']:5.0f} | {r['why']} ({int(r['n_clean'])}/{int(r['n'])} candidates measured)")
            continue
        out(f"{r['V2']:6g} {r['P']:5.0f} | {r['Ipk_SPS']:6.3f} {r['Ipk_MB']:6.3f} {r['Ipk_min']:6.3f} "
            f"({r['best_D1']:.3f}, {r['best_D2']:.3f}) {r['best_ctrl']:>8} | {r['gap']:+6.3f} {r['tol']:5.3f} | "
            f"{int(r['n_within'])}, {r['within_D1'][0]:.3f}-{r['within_D1'][1]:.3f}, "
            f"{r['within_D2'][0]:.3f}-{r['within_D2'][1]:.3f} | {r['verdict']}"
            + (f" ({int(r['unsettled'])} unsettled)" if r["unsettled"] else ""))
    out(f"=> rule (i): {code1}")

    code2, V = sv.valley(T[T["fam"] == "k1"])
    out("\nOPT rule (ii): V2 95 V, some clean candidate with min(D1, D2) < 0.45 more than 0.05 A below SPS (40 ms runs)")
    for _, r in V.iterrows():
        out(f"  {r['P']:5.0f} W ({r['D1']:.2f}, {r['D2']:.2f}) Ipk {r['Ipk']:.3f} vs SPS {r['Ipk_SPS']:.3f}: "
            f"gain {r['gain']:+.3f} A")
    out(f"=> rule (ii): {code2}")

    G = grid_optimum(T)
    code3, info = sv.shape(G)
    out("\nOPT rule (iii): Simulink grid optimum (steady state of the 3 best staircase combos + SPS)")
    for _, r in G.iterrows():
        out(f"  {r['V2']:6g} V {r['P']:5.0f} W: ({r['D1']:.3f}, {r['D2']:.3f}) {r['ctrl']} Ipk {r['Ipk']:.3f}")
    for v, o in info.items():
        out(f"  {v:g} V: {o}")
    out(f"=> rule (iii): {code3}")

    st = T[(T["fam"] == "grid") & T["ctrl"].str.startswith("G") & np.isfinite(T["Ipk"])] \
        if "Ipk_stair" in T else T.iloc[:0]
    if len(st):
        dd = (st["Ipk_stair"] - st["Ipk"]).abs()
        out(f"\nOPT1 check: R6 1 ms staircase vs steady state, same combination and power: |diff| mean {dd.mean():.3f}, "
            f"max {dd.max():.3f} A over {len(st)}")
    open(os.path.join(OUT, "surface_opt_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    return T, (code1, P1), (code2, V), (code3, G, info)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare"])
    ap.add_argument("--fams", default=",".join(FAMS))
    ap.add_argument("--partial", action="store_true", help="grid family from an incomplete R6 (testing only)")
    a = ap.parse_args()
    fams = tuple(f for f in a.fams.split(",") if f)
    if a.cmd in ("make", "next"):
        plan(fams, a.partial)
    else:
        compare(a.partial)


if __name__ == "__main__":
    main()
