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
OPT4 (10-30): the corrected surface M-S (train_corrected.py --tag MS), families ms / msk1, rules in ms_verdict.py:
    python -I -X utf8 Simulink/Thesis/surface_opt_check.py make --fams ms,msk1    (round 1; next --fams ms,msk1 after)
    python -I -X utf8 Simulink/Thesis/surface_opt_check.py ms_compare -> out/ms_check.{csv,log}, out/ms_verdict.md,
                                                                         out/fig_ms_check.png
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
SIM_T = {"mb": 0.030, "k1": 0.040, "grid": 0.030, "ms": 0.030, "msk1": 0.040}
# OPT4 (10-30, after round 2): verification runs of the corrected surface M-S (train_corrected.py --tag MS), only with
# --fams ms,msk1 (never part of the M-B verdict above). ms: 30 ms at V2 84 / 109.2 V (+ two intermediate V2, where the
# anchors' Gaussian tails reach and no OPT run exists); msk1: 40 ms in the k = 1 band (k1map's V2). M-S points whose
# duties are within MS_DIFF of M-B are not run (M-B's runs stand for them). Where no OPT family has M-B / SPS, they are
# run beside M-S.
MS_FAMS = ("ms", "msk1")
MS_TAG = "MS"
MS_P = (100.0, 150.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0)
MS_V2_EDGE = (84.0, 109.2)
MS_V2_MID = (89.0, 104.0)
MS_P_MID = (100.0, 200.0, 400.0, 600.0, 800.0)
MSK1_V2 = (93.0, 94.0, 95.0, 96.0, 97.0, 98.0)        # 98 V (10-30): the k = 1 cap anchors spill to 98 V 100-200 W
MSK1_P = (70.0, 100.0, 150.0, 200.0, 300.0)
MS_DIFF = 0.003
MB_P = (100.0, 300.0, 1000.0)
K1_P = (100.0, 300.0, 600.0)
GRID_P = (100.0, 150.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)
GRID_TOP = 3
R6_PER_V2 = 121
BT = {(109.2, 100.0): (0.247368421, 0.2), (84.0, 300.0): (0.373684211, 0.389473684),
      (84.0, 100.0): (0.215789474, 0.231578947)}                    # R.8 Q2: dataset best-TPS combos (BT_ labels)

# window sensitivity (flicker_check.py, 10-30): a few clean runs jump between two peak levels from one 1 ms window to the
# next. The verdict uses the end window (registered); these two only show how robust it is -- same bracket, other Ipk.
SENS = {"Ipk_at_25ms": "25 ms 창", "Ipk_max": "최대 15 ms–끝"}
FLICKER = 0.05                                                      # [A] window-peak span counted as flicker


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


def _ms():
    p = os.path.join(OUT, f"model_smooth_reghdc_{MS_TAG}.json")
    if not os.path.isfile(p):
        sys.exit(f"{p} not found: train it first (train_corrected.py --tag {MS_TAG})")
    return ct.SmoothRegHDC.from_json(p)


def configs_ms(fam="ms"):
    """OPT4: M-S where it differs from M-B; M-B and SPS beside it where no OPT family has them"""
    ms, mb = _ms(), _mb()
    if fam == "ms":
        pts = [(v, p, p not in GRID_P) for v in MS_V2_EDGE for p in MS_P] + [(v, p, True) for v in MS_V2_MID for p in MS_P_MID]
    else:
        pts = [(v, p, True) for v in MSK1_V2 for p in MSK1_P]
    cf = []
    for v, p, new in pts:
        s1, s2 = (float(np.ravel(x)[0]) for x in ms.predict(np.array([p]), np.array([v])))
        b1, b2 = (float(np.ravel(x)[0]) for x in mb.predict(np.array([p]), np.array([v])))
        if max(abs(s1 - b1), abs(s2 - b2)) < MS_DIFF:
            continue
        cf.append(_cfg(fam, "M-S", v, p, s1, s2))
        if new:
            cf += [_cfg(fam, "M-B", v, p, b1, b2), _cfg(fam, "SPS", v, p, 0.5, 0.5)]
    return cf


def configs(fams=FAMS, partial=False, verbose=True):
    cf = []
    if "mb" in fams:
        cf += configs_mb()
    if "k1" in fams:
        cf += configs_k1()
    if "grid" in fams:
        cf += configs_grid(partial, verbose)
    for f in MS_FAMS:
        if f in fams:
            cf += configs_ms(f)
    cf, bad = _feasible(_dedupe(cf))
    if verbose and bad:
        print(f"{len(bad)} configurations out of reach (ideal model), dropped: "
              + "; ".join(f"{c['fam']} {c['V2']:g} V {c['P']:.0f} W {c['ctrl']} ({c['D1']:.3f}, {c['D2']:.3f})"
                          for c in bad))
    return cf


def result_files(fam):
    fs = glob.glob(os.path.join(OUT, f"opt_sweep_{fam}_r*.csv"))
    if fam == "msk1":                                         # same method and 40 ms: OPT2 and k1map runs are reused
        fs += glob.glob(os.path.join(OUT, "opt_sweep_k1_r*.csv")) + glob.glob(os.path.join(OUT, "k1map_sweep_r*.csv"))
    elif fam != "k1":                                         # same method and 30 ms: R.4 and R8 runs are reused
        fs += glob.glob(os.path.join(OUT, "simulink_sweep_r*.csv")) + glob.glob(os.path.join(OUT, "eff_sweep_r*.csv"))
        fs += glob.glob(os.path.join(OUT, f"opt_sweep_{'grid' if fam == 'mb' else 'mb'}_r*.csv"))
        if fam == "ms":
            fs += glob.glob(os.path.join(OUT, "opt_sweep_grid_r*.csv"))
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
    d["Ipk_max"] = d[win_cols(d.columns) + ["Ipk_sim"]].max(axis=1)   # worst 1 ms window, 15 ms .. end
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last")


def win_cols(cols):
    """the 1 ms window peaks from 15 ms on: 15/20/25 ms in runs before 10-30, every ms since (verify_steady_state.m);
    older rows are NaN in the newer columns and max/min skip them"""
    out = []
    for c in cols:
        m = re.fullmatch(r"Ipk_at_(\d+(?:\.\d+)?)ms", c)
        if m and float(m.group(1)) >= 15:
            out.append(c)
    return out


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
            below = g[(g["Pd"] > 0) & (g["Pd"] <= c["P"])]
            if b is None and way < 0 and not below.empty:
                # |P| falls in a gap between two measured levels that are not adjacent (10-30: SPS reused from R.4's
                # 100/300/600/1000 W brackets left 84 V 400/800 W in such gaps, and bracket() said "go below the
                # lowest level", i.e. into the other power direction, for 4 rounds) -> fill the gap from below
                j = int(below["level"].max())
                pts += [sw.point(c, j + 1), sw.point(c, j + 2)]
            elif b is None:
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


def measure(fams=FAMS, partial=False, col="Ipk_sim", cf=None):
    """one row per configuration: Simulink steady Ipk at |P| (NaN if not bracketed), osc, settle, flicker.
    col: which 1 ms window gives Ipk (the bracket is always the end-window power); only Ipk_sim enters a verdict."""
    import pandas as pd
    rows = []
    cf = configs(fams, partial, verbose=False) if cf is None else cf
    for fam in fams:
        d = results(fam)
        for c in (c for c in cf if c["fam"] == fam):
            rec = dict(c, Ipk=np.nan, osc=np.nan, settle=np.nan, flicker=np.nan, src=fam)
            g = sw.runs_of(d, c) if d is not None else None
            b = sw.bracket(g, c["P"])[0] if g is not None and not g.empty else None
            if b is not None:
                lo, hi = b
                t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
                wc = ["Ipk_sim"] + win_cols(lo.index)
                span = [r[wc].max() - r[wc].min() for r in (lo, hi)]
                rec.update(Ipk=lo[col] + t * (hi[col] - lo[col]),
                           osc=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])),
                           settle=max(abs(lo["Ipk_sim"] - lo["Ipk_at_25ms"]), abs(hi["Ipk_sim"] - hi["Ipk_at_25ms"])),
                           flicker=max(span), P_lo=lo["Pd"], P_hi=hi["Pd"])
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


def _c30(T):
    """rule (i) candidates: every 30 ms candidate (mb + grid) at each (V2, P); the same (V2, P, D1, D2) once"""
    return T[T["fam"].isin(["mb", "grid"])].sort_values("fam").drop_duplicates(["V2", "P", "D1", "D2"], keep="first")


def _e_max(n, draws=200000, seed=0):
    """E[max of n standard normals] = E[X_1 - min(X_1..X_n)] / sigma for iid X (Monte Carlo)"""
    if n < 2:
        return 0.0
    z = np.random.default_rng(seed).standard_normal((draws, n))
    return float(np.mean(z[:, 0] - z.min(axis=1)))


def roughness(T):
    """NOT a verdict (added 10-30 after round 1; the rules in surface_opt_verdict.py are unchanged): how rough is the
    Simulink Ipk landscape at 0.02-duty steps? Second differences d2 = I(-h) - 2 I(0) + I(+h) along the mb cross axes
    (clean candidates, h = 0.02); for iid roughness sigma, Var(d2) = 6 sigma^2 (a smooth curvature only adds to it).
    Yardstick per point: if all n clean non-SPS candidates had the same true Ipk, the expected Ipk(M-B) - min would be
    sigma * E[max of n normals] (minimum-selection bias)."""
    mb = T[(T["fam"] == "mb")]
    d2 = {"light": [], "heavy": []}
    centred = {"light": [], "heavy": []}       # (d2, centred on M-B?) -- a minimum of max(...) is a V-shaped kink, so
    for (v, p), g in mb.groupby(["V2", "P"]):  # d2 centred on an optimal M-B is large and positive, not noise (10-30 r2)
        c = g[g["ctrl"] == "M-B"]
        if c.empty:
            continue
        c = c.iloc[0]
        for ax, other in (("D1", "D2"), ("D2", "D1")):
            s = g[np.isclose(g[other], c[other]) & np.isfinite(g["Ipk"]) & (g["osc"] <= sv.OSC)]
            m = dict(zip(np.round((s[ax] - c[ax]) / 0.02).astype(int), s["Ipk"]))
            for k in (-1, 0, 1):
                if all(o in m for o in (k - 1, k, k + 1)):
                    w = "light" if p <= 300 else "heavy"
                    d2[w].append(m[k - 1] - 2 * m[k] + m[k + 1])
                    centred[w].append(k == 0)
    sig = {k: (float(np.sqrt(np.mean(np.square(x)) / 6)) if x else np.nan, len(x)) for k, x in d2.items()}
    split = {}
    for k, x in d2.items():
        x, c = np.array(x), np.array(centred[k], bool)
        split[k] = dict(centre_mean=float(x[c].mean()) if c.any() else np.nan, n_centre=int(c.sum()),
                        sigma_off=float(np.sqrt(np.mean(np.square(x[~c])) / 6)) if (~c).any() else np.nan,
                        n_off=int((~c).sum()))
    sig["split"] = split
    rows = []
    for (v, p), g in _c30(T).groupby(["V2", "P"]):
        n = int(((g["ctrl"] != "SPS") & np.isfinite(g["Ipk"]) & (g["osc"] <= sv.OSC)).sum())
        w = "light" if p <= 300 else "heavy"
        s = np.nanmin([sig[w][0], split[w]["sigma_off"]])             # the smaller of the two: kinks at M-B are not
        #                                                                 roughness, and a small sigma is the strict side
        rows.append(dict(V2=v, P=p, n=n, sigma=s, expected_gap=s * _e_max(n) if np.isfinite(s) else np.nan))
    import pandas as pd
    return sig, pd.DataFrame(rows)


def undecided_gaps(c30, P1):
    """NOT a verdict (added 10-30 after round 2): at the points rule (i) cannot judge (M-B or SPS oscillating / not
    bracketed), M-B's own value and the best clean other candidate, so a large gap is not hidden by 'undecidable'"""
    rows = []
    for _, r in P1[P1["verdict"] == "undecidable"].iterrows():
        g = c30[np.isclose(c30["V2"], r["V2"]) & np.isclose(c30["P"], r["P"])]
        mb = g[(g["ctrl"] == "M-B") & np.isfinite(g["Ipk"])]
        oth = g[(g["ctrl"] != "M-B") & np.isfinite(g["Ipk"]) & (g["osc"] <= sv.OSC)]
        if mb.empty or oth.empty:
            continue
        m, b = mb.iloc[0], oth.loc[oth["Ipk"].idxmin()]
        rows.append(dict(V2=r["V2"], P=r["P"], why=r["why"], Ipk_MB=m["Ipk"], MB_osc=m["osc"] > sv.OSC,
                         MB_D1=m["D1"], MB_D2=m["D2"], best=b["ctrl"], best_D1=b["D1"], best_D2=b["D2"],
                         Ipk_best=b["Ipk"], gap=m["Ipk"] - b["Ipk"]))
    import pandas as pd
    return pd.DataFrame(rows)


def compare(partial=False):
    cf = configs(FAMS, partial, verbose=False)
    T = measure(FAMS, partial, cf=cf)
    T.to_csv(os.path.join(OUT, "surface_opt_check.csv"), index=False, float_format="%.6g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    c30 = _c30(T)
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
    UG = undecided_gaps(_c30(T), P1)
    for _, r in UG.iterrows():
        out(f"   undecidable {r['V2']:g} V {r['P']:.0f} W ({r['why']}), NOT a verdict: M-B ({r['MB_D1']:.3f}, "
            f"{r['MB_D2']:.3f}) {r['Ipk_MB']:.3f}{' (oscillating)' if r['MB_osc'] else ''} - best clean {r['best']} "
            f"({r['best_D1']:.3f}, {r['best_D2']:.3f}) {r['Ipk_best']:.3f} = {r['gap']:+.3f} A")

    # robustness only (not a verdict): the same rule with the peak read from another 1 ms window
    sens = {}
    for col, name in SENS.items():
        Ps, code_s = sv.judge_points(_c30(measure(FAMS, partial, col, cf)))
        sens[col] = (code_s, Ps)
        flips = [f"{r['V2']:g} V {r['P']:.0f} W {r['verdict']} (gap {r['gap']:+.3f})" for (_, r), (_, r0)
                 in zip(Ps.iterrows(), P1.iterrows()) if r["verdict"] != r0["verdict"]] if len(Ps) == len(P1) else ["?"]
        out(f"   window check ({col}): rule (i) {code_s}; points that change: " + ("; ".join(flips) or "none"))
    fl = c30[c30["flicker"] > FLICKER]
    out(f"   candidates bracketed by a run whose window peaks span > {FLICKER} A: {len(fl)}"
        + "".join(f"\n     {r['V2']:g} V {r['P']:.0f} W {r['ctrl']} ({r['D1']:.3f}, {r['D2']:.3f}) span {r['flicker']:.2f}"
                  for _, r in fl.iterrows()))
    sig, RG = roughness(T)
    out("   roughness (NOT a verdict): sigma from 2nd differences along the M-B cross, h = 0.02: "
        + ", ".join(f"{k} (|P| {'<=' if k == 'light' else '>'} 300 W) {sig[k][0]:.3f} A from {sig[k][1]}"
                    for k in ("light", "heavy")))
    for k, o in sig["split"].items():
        out(f"     {k}: d2 centred on M-B mean {o['centre_mean']:+.3f} A ({o['n_centre']}; > 0 = M-B at the bottom of a "
            f"V-shaped kink), sigma from the off-centre ones {o['sigma_off']:.3f} A ({o['n_off']})")
    out("     yardstick below: sigma = the smaller of the two estimates (strict for M-B)")
    gp = P1.set_index(["V2", "P"])["gap"] if "gap" in P1 else None
    for _, r in RG.iterrows():
        g = gp.get((r["V2"], r["P"]), np.nan) if gp is not None else np.nan
        if np.isfinite(g):
            out(f"     {r['V2']:g} V {r['P']:.0f} W: gap {g:+.3f} A vs 'all equal' expectation {r['expected_gap']:.3f} A "
                f"({int(r['n'])} clean non-SPS candidates)")

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
    miss = T[~np.isfinite(T["Ipk"])]
    out(f"\nnot bracketed yet: {len(miss)}/{len(T)} configurations ("
        + ", ".join(f"{f} {int((miss['fam'] == f).sum())}/{int((T['fam'] == f).sum())}" for f in FAMS) + ")")
    open(os.path.join(OUT, "surface_opt_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    res = dict(T=T, partial=partial, i=(code1, P1), sens=sens, ii=(code2, V), iii=(code3, G, info), stair=st,
               rough=(sig, RG), ug=UG)
    verdict_md(res)
    fig(res)
    print("wrote out/surface_opt_check.{csv,log}, out/surface_opt_verdict.md, out/fig_surface_opt_check.png")
    return res


def k1_levels(levels=(1, 2, 3, 4, 5)):
    """95 V OPT2 runs per D0 level, no interpolation: [(level, [(name, 'P W / Ipk A'), ...]), ...]"""
    d = results("k1")
    if d is None:
        return []
    d = d[np.isclose(d["V2"], 95.0)]
    cols = [("SPS", sw.D_SAFE, sw.D_SAFE)] + [(f"D1 = D2 = {x:.2f}", x, x) for x in (0.45, 0.40, 0.35, 0.30)]
    out = []
    for j in levels:
        cells = []
        for name, a, b in cols:
            r = d[(d["level"] == j) & (np.abs(d["D1"] - a) <= 6e-7) & (np.abs(d["D2"] - b) <= 6e-7)]
            cells.append((name, f"{round(r['Pd'].iloc[0]) or 0:.0f} / {r['Ipk_sim'].iloc[0]:.3f}" if len(r) else "–"))
        if any(c != "–" for _, c in cells):
            out.append((j, cells))
    return out


VERDICT_KO = {"optimal": "M-B가 Simulink 최적(허용오차 안)", "not_optimal": "M-B는 Simulink 최적이 아님",
              "undecidable": "판정 불가", "real": "k = 1 골 실재", "not_real": "k = 1 골 없음",
              "match": "곡면 모양 일치", "mismatch": "곡면 모양 불일치"}


def verdict_md(res):
    """out/surface_opt_verdict.md (Korean, for REPORT.md 11절 / RESULTS.md R.10)"""
    import datetime
    T, (code1, P1), (code2, V), (code3, G, info) = res["T"], res["i"], res["ii"], res["iii"]
    n_meas = {f: (int(np.isfinite(T.loc[T["fam"] == f, "Ipk"]).sum()), int((T["fam"] == f).sum())) for f in FAMS}
    complete = all(a == b and b > 0 for a, b in n_meas.values())
    md = ["# OPT 판정: 제안 곡면 M-B는 Simulink 최적인가? (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `surface_opt_check.py compare`. 판정 규칙은 `surface_opt_verdict.py`"
          "(OPT0, 커밋 fd95a30, OPT 결과 전 등록)이며 바꾸지 않았다. 수치는 모두 Simulink(데이터셋 시기 모델) 정상상태다. 각 값은"
          " 상수 D0 30 ms(k = 1 시험은 40 ms)를 돌리고 마지막 1 ms의 max|i_L|을 쓴다. 측정 전력이 목표를 감싸는 2.88° 두 계단 사이에서"
          " 전력으로 보간하며, 방향은 HV→LV다.",
          "", "측정된 구성: " + ", ".join(f"{f} {a}/{b}" for f, (a, b) in n_meas.items())
          + ("" if complete else " — **미완료: 아래 판정은 잠정이다**(감싸지 못한 구성은 `surface_opt_check.py next`로 다음 라운드)."
             + (" grid는 R6 미완료로 부분 데이터(--partial, 시험용)다." if res["partial"] else "")), ""]

    md += ["## 규칙 (i) 점마다 M-B가 Simulink 최적인가", "",
           "차 = Ipk(M-B) − 그 점 모든 깨끗한 후보의 최저 Ipk. 허용오차 = max(0.03 A, Ipk(SPS)의 2 %). 차가 허용오차 이하면 그 점에서 최적이다.", "",
           "| V2 | \\|P\\| | SPS | M-B | 최저 | 최저 후보 (D1, D2) | 차 | 허용 | 측정 후보 · 허용 안 후보 수 · D1 범위 · D2 범위 | 판정 |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for _, r in P1.iterrows():
        if r["verdict"] == "undecidable":
            md.append(f"| {r['V2']:g} V | {r['P']:.0f} W | – | – | – | – | – | – | {int(r['n_clean'])}/{int(r['n'])} 측정 | "
                      f"판정 불가({r['why']}) |")
            continue
        md.append(f"| {r['V2']:g} V | {r['P']:.0f} W | {r['Ipk_SPS']:.3f} | {r['Ipk_MB']:.3f} | {r['Ipk_min']:.3f} | "
                  f"({r['best_D1']:.3f}, {r['best_D2']:.3f}) {r['best_ctrl']} | {r['gap']:+.3f} | {r['tol']:.3f} | "
                  f"{int(r['n_clean'])}/{int(r['n'])} · {int(r['n_within'])} · {r['within_D1'][0]:.3f}–{r['within_D1'][1]:.3f} · "
                  f"{r['within_D2'][0]:.3f}–{r['within_D2'][1]:.3f} | "
                  + ("**최적 아님**" if r["verdict"] == "not_optimal" else "최적") + " |")
    fails = P1[P1["verdict"] == "not_optimal"]
    md += ["", f"단위 A. **판정 (i): {VERDICT_KO[code1]}**"
           + (f" — 깨지는 점 {len(fails)}개: " + ", ".join(f"{r['V2']:g} V {r['P']:.0f} W(차 {r['gap']:+.3f} A)"
                                                       for _, r in fails.iterrows()) if len(fails) else "") + "."]
    UG = res.get("ug")
    if UG is not None and len(UG):
        md += ["", "판정 불가 점의 참고값(판정 아님, 10-30 2라운드 뒤 추가): 규칙은 M-B나 SPS가 진동하거나 미포착이면 그 점을 판정하지"
               " 않는다. 그래도 큰 차가 '판정 불가'에 가려지지 않도록, M-B 값과 진동 없는 다른 후보 중 최저를 적는다.", "",
               "| V2 | \\|P\\| | 판정 불가 이유 | M-B (D1, D2) | M-B Ipk | 최저 깨끗한 후보 (D1, D2) | 그 Ipk | M-B − 최저 |",
               "|---|---|---|---|---|---|---|---|"]
        for _, r in UG.iterrows():
            md.append(f"| {r['V2']:g} V | {r['P']:.0f} W | {r['why']} | ({r['MB_D1']:.3f}, {r['MB_D2']:.3f}) | "
                      f"{r['Ipk_MB']:.3f}{' (진동)' if r['MB_osc'] else ''} | {r['best']} ({r['best_D1']:.3f}, "
                      f"{r['best_D2']:.3f}) | {r['Ipk_best']:.3f} | {r['gap']:+.3f} |")
    md += ["", "창 민감도(판정 아님, `flicker_check.py`): 일부 실행은 1 ms 창마다 피크가 두 값 사이를 오간다. 같은 두 계단을 쓰고 피크만"
           " 다른 창에서 읽어 규칙 (i)을 다시 적용하면 아래와 같다.", ""]
    for col, (code_s, Ps) in res["sens"].items():
        diff = [f"{r['V2']:g} V {r['P']:.0f} W → {VERDICT_KO.get(r['verdict'], r['verdict'])}(차 {r['gap']:+.3f})"
                for (_, r), (_, r0) in zip(Ps.iterrows(), P1.iterrows()) if r["verdict"] != r0["verdict"]] \
            if len(Ps) == len(P1) else ["점 목록이 달라 비교 불가"]
        md.append(f"- {SENS[col]}: {VERDICT_KO[code_s]}; 판정이 바뀌는 점 " + ("; ".join(diff) if diff else "없음"))
    c30 = _c30(T)
    fl = c30[c30["flicker"] > FLICKER]
    if len(fl):
        md.append(f"- 창마다 피크가 {FLICKER} A 넘게 달라지는 실행에 기댄 후보: " + "; ".join(
            f"{r['V2']:g} V {r['P']:.0f} W {r['ctrl']} ({r['D1']:.3f}, {r['D2']:.3f}) 폭 {r['flicker']:.2f} A"
            for _, r in fl.iterrows()))
    sig, RG = res["rough"]
    gp = P1.set_index(["V2", "P"])["gap"] if "gap" in P1 else None
    md += ["", "곡면의 들쭉날쭉함(판정 아님, 10-30 1라운드 뒤 추가; 규칙은 그대로): M-B 십자에서 0.02 듀티 간격 세 점의 2차 차분"
           " d2 = I(−h) − 2I(0) + I(+h)로 Simulink Ipk 곡면의 거칠기 σ = RMS(d2)/√6을 어림했다(매끄러운 곡률은 σ를 키우기만 한다). "
           + ", ".join(f"{'경부하(≤ 300 W)' if k == 'light' else '중부하 이상(> 300 W)'} σ ≈ {sig[k][0]:.3f} A(차분 {sig[k][1]}개)"
                       for k in ("light", "heavy")) + ". 후보 n개의 참값이 모두 같아도 Ipk(M-B) − 최저의 기댓값은"
           " σ·E[n개 정규 최댓값]이다(최솟값 선택 편향). "
           "(10-30 2라운드 뒤 추가) 최솟값이 max(…) 꼴이면 최적점은 V자 꺾임이라, M-B를 가운데 둔 d2는 잡음이 아니라 꺾임이다. "
           + ", ".join(f"{'경부하' if k == 'light' else '중부하 이상'}: M-B 중심 d2 평균 {o['centre_mean']:+.3f} A({o['n_centre']}개, "
                       f"양수 = M-B가 V자 바닥), 중심 밖 d2로 σ {o['sigma_off']:.3f} A({o['n_off']}개)"
                       for k, o in sig["split"].items())
           + ". 기댓값은 두 σ 어림 중 작은 쪽으로 계산한다(M-B에 엄격한 쪽). 점마다 견줄 값:", ""]
    for _, r in RG.iterrows():
        g = gp.get((r["V2"], r["P"]), np.nan) if gp is not None else np.nan
        if np.isfinite(g):
            md.append(f"- {r['V2']:g} V {r['P']:.0f} W: 차 {g:+.3f} A, '모두 같음' 기댓값 {r['expected_gap']:.3f} A"
                      f"(깨끗한 비SPS 후보 {int(r['n'])}개)")

    md += ["", "## 규칙 (ii) k = 1(95 V) 중앙의 깊은 골", "",
           "min(D1, D2) < 0.45인 깨끗한 후보 중 하나라도 같은 전력의 SPS보다 0.05 A 넘게 낮으면 골이 실재한다(40 ms 실행).", "",
           "| \\|P\\| | (D1, D2) | Ipk | SPS | 이득 |", "|---|---|---|---|---|"]
    for _, r in V.sort_values(["P", "D1", "D2"]).iterrows():
        md.append(f"| {r['P']:.0f} W | ({r['D1']:.2f}, {r['D2']:.2f}) | {r['Ipk']:.3f} | {r['Ipk_SPS']:.3f} | {r['gain']:+.3f} |")
    md += ["", f"**판정 (ii): {VERDICT_KO[code2]}**" + (f" — 최대 이득 {V['gain'].max():+.3f} A." if len(V) else ".")]
    lv = k1_levels()
    if lv:
        md += ["", "같은 D0 계단에서 직접 비교(보간 없음, 판정 아님): 95 V에서 2.88° 계단마다 측정 전력[W] / 마지막 1 ms max|i_L|[A]."
               " 이 전압에서는 한 계단이 70–230 W를 건너뛰므로, 규칙의 100 W 값은 두 계단 사이 직선 보간이다.", "",
               "| 계단 (D0) | " + " | ".join(n for n, _ in lv[0][1]) + " |", "|---" * (1 + len(lv[0][1])) + "|"]
        for j, cells in lv:
            md.append(f"| {j} ({(j - 0.5) * sw.G:.2f}°) | " + " | ".join(c for _, c in cells) + " |")

    md += ["", "## 규칙 (iii) 곡면 모양", "",
           "Simulink 격자 최적(R6 1 ms 계단 상위 3조합 + SPS를 정상상태로 잰 것 중 최저)이 M-B와 같은 쪽(84 V는 D1 < D2, 109.2 V는"
           " D2 < D1)에 있는지(75 % 이상) 본다. 또 |P|가 커질수록 0.5로 오르는지 본다(Spearman ≥ 0.8, 최대 전력의 min(D) ≥ 0.45).", "",
           "| V2 | \\|P\\| | 격자 최적 (D1, D2) | 후보 | Ipk |", "|---|---|---|---|---|"]
    for _, r in G.iterrows():
        md.append(f"| {r['V2']:g} V | {r['P']:.0f} W | ({r['D1']:.3f}, {r['D2']:.3f}) | {r['ctrl']} | {r['Ipk']:.3f} |")
    for v, o in info.items():
        md.append(f"- {v:g} V: 점 {o['n']}개, TPS 최적 {o['n_tps']}개 중 같은 쪽 비율 "
                  + (f"{o['side_frac'] * 100:.0f} %" if np.isfinite(o["side_frac"]) else "–")
                  + f", Spearman {o['rho']:.2f}, 최대 전력 min(D) {o['d_end']:.3f}" if o["n"] >= 3 else
                  f"- {v:g} V: 점 {o['n']}개 — 3개 미만이라 판정 불가")
    md += ["", f"**판정 (iii): {VERDICT_KO[code3]}**."]
    st = res["stair"]
    if len(st):
        dd = (st["Ipk_stair"] - st["Ipk"]).abs()
        md += ["", f"OPT1 확인: 같은 조합·같은 전력에서 R6 1 ms 계단 값과 정상상태의 |차|는 평균 {dd.mean():.3f} A, 최대 {dd.max():.3f} A다"
               f"({len(st)}개). 계단 값은 후보를 고르는 데만 썼다."]
    md += ["", "그림: `out/fig_surface_opt_check.png`. 표 원본: `out/surface_opt_check.csv`, 로그: `out/surface_opt_check.log`."]
    open(os.path.join(OUT, "surface_opt_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")


def fig(res):
    """top: M-B's D1/D2 against |P| with the Simulink-best candidate and the within-tolerance ranges; bottom: Ipk(M-B) -
    Ipk(best) against the tolerance; right: the k = 1 test (Ipk against D1 = D2 at 95 V, SPS dashed)"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, YELLOW, _style
    _style(plt)
    T, (code1, P1), (code2, V), (code3, G, info) = res["T"], res["i"], res["ii"], res["iii"]
    f, ax = plt.subplots(2, 3, figsize=(14.5, 7.8), constrained_layout=True, gridspec_kw=dict(height_ratios=[1.5, 1]))
    Pc = np.linspace(50, 1000, 200)
    for j, V2 in enumerate((84.0, 109.2)):
        a, b = ax[0, j], ax[1, j]
        d1, d2 = _mb().predict(Pc, np.full_like(Pc, V2))
        a.plot(Pc, np.ravel(d1), color=BLUE, lw=1.8, label="M-B D1")
        a.plot(Pc, np.ravel(d2), color=ORANGE, lw=1.8, label="M-B D2")
        p = P1[np.isclose(P1["V2"], V2) & (P1["verdict"] != "undecidable")] if len(P1) else P1
        for _, r in p.iterrows():
            for k, (col, dx) in enumerate(((BLUE, -6), (ORANGE, 6))):
                lo, hi = r["within_D1"] if k == 0 else r["within_D2"]
                a.plot([r["P"] + dx] * 2, [lo, hi], color=col, lw=5, alpha=0.25, solid_capstyle="butt")
            a.plot(r["P"] - 6, r["best_D1"], "o", color=BLUE, ms=6, mec="white", mew=0.8)
            a.plot(r["P"] + 6, r["best_D2"], "s", color=ORANGE, ms=6, mec="white", mew=0.8)
        a.plot([], [], "o", color=MUTED, label="Simulink best candidate (D1 o, D2 s)")
        a.plot([], [], color=MUTED, lw=5, alpha=0.3, label="candidates within tolerance")
        a.set_ylim(0.1, 0.52)
        a.set_xlim(0, 1050)
        a.set_title(f"V2 = {V2:g} V (k = {4 * V2 / 380:.3f}): duty vs |P|, HV -> LV")
        a.set_ylabel("duty")
        if len(p):
            ok = (p["verdict"] == "optimal").to_numpy()
            b.vlines(p["P"], 0, p["gap"], color=np.where(ok, AQUA, ORANGE), lw=2.5)
            b.scatter(p["P"], p["gap"], c=np.where(ok, AQUA, ORANGE), s=28, zorder=3)
            b.plot(p["P"], p["tol"], "_", color=INK2, ms=16, mew=2, label="tolerance max(0.03 A, 2 % SPS)")
        u = P1[np.isclose(P1["V2"], V2) & (P1["verdict"] == "undecidable")] if len(P1) else P1
        if len(u):
            b.plot(u["P"], np.zeros(len(u)), "x", color=MUTED, ms=7, label="not decidable (yet)")
        b.axhline(0, color=MUTED, lw=0.8)
        b.set_xlim(0, 1050)
        b.set_xlabel("|P| [W]")
        b.set_ylabel("Ipk(M-B) - Ipk(best) [A]")
        b.set_title("rule (i): stem above the tick = M-B not optimal there")
        if j == 0:
            a.legend(fontsize=7.5, frameon=False, loc="lower right")
            b.legend(fontsize=7.5, frameon=False, loc="upper left")
    k1 = T[(T["fam"] == "k1") & np.isfinite(T["Ipk"])]
    a = ax[0, 2]
    for P, col in zip(K1_P, (BLUE, ORANGE, AQUA)):
        g = k1[np.isclose(k1["P"], P)]
        eq = g[np.isclose(g["D1"], g["D2"]) & g["ctrl"].str.startswith(("D=", "SPS"))].sort_values("D1")
        if len(eq):
            a.plot(eq["D1"], eq["Ipk"], "-o", color=col, ms=4, lw=1.6, label=f"{P:.0f} W, D1 = D2")
        sp = g[g["ctrl"] == "SPS"]
        if len(sp):
            a.axhline(float(sp["Ipk"].iloc[0]), color=col, ls="--", lw=0.9)
        mb = g[g["ctrl"] == "M-B"]
        if len(mb):
            a.plot(mb["D1"], mb["Ipk"], "*", color=col, ms=11, mec="white")
    a.plot([], [], "*", color=MUTED, ms=10, label="M-B")
    a.plot([], [], "--", color=MUTED, lw=0.9, label="SPS level")
    a.set_xlabel("D1 = D2")
    a.set_ylabel("Ipk [A]")
    a.set_title("rule (ii): k = 1 (V2 = 95 V), 40 ms runs")
    a.legend(fontsize=7.5, frameon=False, loc="upper right")
    t = ax[1, 2]
    t.axis("off")
    meas = ", ".join(f"{fm} {int(np.isfinite(T.loc[T['fam'] == fm, 'Ipk']).sum())}/{int((T['fam'] == fm).sum())}"
                     for fm in FAMS)
    t.text(0.0, 0.95, "\n".join([f"rule (i)   M-B optimal at every point: {code1}",
                                 f"rule (ii)  deep k = 1 valley: {code2}",
                                 f"rule (iii) surface shape: {code3}", "",
                                 f"measured configurations: {meas}",
                                 "rules: surface_opt_verdict.py (registered before the runs)",
                                 "Simulink only (dataset-era model), steady state,",
                                 "two 2.88-deg D0 levels interpolated in power"]
                                + (["PARTIAL R6 (--partial): test output"] if res["partial"] else [])),
           va="top", ha="left", fontsize=9, color=INK2, family="monospace")
    f.savefig(os.path.join(OUT, "fig_surface_opt_check.png"), dpi=130)
    plt.close(f)


MS_VERDICT_KO = {"adopt": "M-S 채택(M-B 대체)", "partial": "부분(앵커 영역만 개선, 다른 곳 열화)",
                 "no_gain": "보정 효과 없음(M-B 유지)", "undecidable": "판정 불가"}
CLS_KO = {"better": "개선", "worse": "열화", "same": "같음", "fixes_osc": "M-B 진동 해소", "new_osc": "M-S 진동",
          "undecidable": "판정 불가"}


def ms_points(col="Ipk_sim"):
    """OPT4: one row per M-S operating point -- M-S, M-B and SPS measured the OPT way (families ms / msk1, every reusable
    run included), the anchor weight w of M-S's target there and the dominant anchor group (report only)"""
    import json

    import pandas as pd
    import train_corrected as tc
    ms, mb = _ms(), _mb()
    meta = json.load(open(os.path.join(OUT, f"model_smooth_reghdc_{MS_TAG}.json"), encoding="utf-8")).get("meta", {})
    A = pd.DataFrame(meta.get("anchors", []))
    cf = []
    for fam in MS_FAMS:
        for c in configs_ms(fam):
            if c["ctrl"] != "M-S":
                continue
            b1, b2 = mb_predict(c["P"], c["V2"])
            cf += [c, _cfg(fam, "M-B", c["V2"], c["P"], b1, b2), _cfg(fam, "SPS", c["V2"], c["P"], 0.5, 0.5)]
    cf = _dedupe(cf)
    T = measure(MS_FAMS, cf=cf, col=col)
    rows = []
    for (fam, v, p), g in T.groupby(["fam", "V2", "P"], sort=True):
        rec = dict(fam=fam, V2=v, P=p)
        for ctrl, k in (("M-S", "MS"), ("M-B", "MB"), ("SPS", "SPS")):
            r = g[g["ctrl"] == ctrl]
            r = r.iloc[0] if len(r) else None
            rec.update({f"Ipk_{k}": r["Ipk"] if r is not None else np.nan, f"osc_{k}": r["osc"] if r is not None else np.nan,
                        f"D1_{k}": r["D1"] if r is not None else np.nan, f"D2_{k}": r["D2"] if r is not None else np.nan,
                        f"flicker_{k}": r["flicker"] if r is not None else np.nan})
        if len(A):
            Wt = tc.weights(np.array([p]), np.array([v]), A)[0]
            _, _, w = tc.target(mb, np.array([p]), np.array([v]), A)
            a = A.iloc[int(np.argmax(Wt))]
            grp = "-" if Wt.max() < 0.01 else "k1" if a.get("mode", "set") == "cap" else ("ridge" if a["P"] <= 200 else "knee")
            rec.update(w=float(w[0]), group=grp)
        else:
            rec.update(w=0.0, group="")
        rows.append(rec)
    return pd.DataFrame(rows), A


def ms_compare():
    """OPT4 verdict on M-S (rules in ms_verdict.py, registered before the M-S runs) -> out/ms_check.{csv,log},
    out/ms_verdict.md, out/fig_ms_check.png"""
    import ms_verdict as mv
    Pts, A = ms_points()
    code, T, s = mv.judge(Pts)
    T.to_csv(os.path.join(OUT, "ms_check.csv"), index=False, float_format="%.6g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("OPT4: corrected surface M-S vs M-B (and SPS), Simulink steady state, HV -> LV (rules: ms_verdict.py)")
    out(f"{'fam':>4} {'V2':>6} {'|P|':>5} {'w':>4} {'grp':>5} | {'M-S (D1, D2)':>15} {'Ipk':>6} | {'M-B (D1, D2)':>15} {'Ipk':>6} | "
        f"{'SPS':>6} | {'gain':>6} {'tol':>5} | class")
    for _, r in T.iterrows():
        def ip(k):
            i, o = r[f"Ipk_{k}"], r[f"osc_{k}"]
            return f"{i:6.3f}{'~' if np.isfinite(o) and o > mv.OSC else ' '}" if np.isfinite(i) else f"{'–':>6} "
        out(f"{r['fam']:>4} {r['V2']:6g} {r['P']:5.0f} {r['w']:4.2f} {r['group']:>5} | ({r['D1_MS']:.3f}, {r['D2_MS']:.3f}) "
            f"{ip('MS')}| ({r['D1_MB']:.3f}, {r['D2_MB']:.3f}) {ip('MB')}| {ip('SPS')}| "
            + (f"{r['gain']:+6.3f}" if np.isfinite(r["gain"]) else f"{'':>6}") + f" {r['tol']:5.3f} | {r['cls']}"
            + (" [M-S > SPS]" if r["viol_MS"] is True else "") + (" [M-B > SPS]" if r["viol_MB"] is True else ""))
    out("(~ = oscillating, |mean i_L| > 20 mA)")
    for reg in ("anchor", "spill"):
        o = s[reg]
        out(f"{reg:>6}: {o['n']} points, {o['n_dec']} decidable, mean gain {o['mean_gain']:+.3f} A over {o['n_gain']}; "
            f"better {o['better']}, fixes_osc {o['fixes_osc']}, same {o['same']}, worse {o['worse']}, new_osc {o['new_osc']}")
    if s.get("A") is not None:
        out(f"A (anchor gains) {s['A']}, B (spill no harm) {s['B']}{'' if s.get('B_tested') else ' (not tested)'}, "
            f"C (SPS violations M-S {s['viol_MS']} <= M-B {s['viol_MB']} over {s['n_viol_checked']}) {s['C']}")
    out(f"=> OPT4 verdict: {code}")
    grp = {}
    for gname, g in T.groupby("group"):                         # report only
        gv = g["gain"].dropna()
        grp[gname] = (len(g), len(gv), float(gv.mean()) if len(gv) else np.nan)
        out(f"   group {gname} (report only): {len(g)} points, mean gain {grp[gname][2]:+.3f} A over {len(gv)}")
    Tw, _ = ms_points(col="Ipk_max")                           # window sensitivity (report only)
    code_w = mv.judge(Tw)[0]
    out(f"   window check (worst 1 ms window 15 ms - end): verdict {code_w}")
    open(os.path.join(OUT, "ms_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    ms_verdict_md(code, T, s, grp, code_w, A)
    ms_fig(T, code)
    print("wrote out/ms_check.{csv,log}, out/ms_verdict.md, out/fig_ms_check.png")
    return code, T, s


def ms_verdict_md(code, T, s, grp, code_w, A):
    import datetime
    n_meas = int(T["cls"].ne("undecidable").sum())
    md = ["# OPT4 판정: 보정 곡면 M-S는 Simulink에서 M-B보다 나은가? (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `surface_opt_check.py ms_compare`. 규칙은 `ms_verdict.py`(10-30 08:15, M-S"
          " Simulink 실행 전 등록)이며 바꾸지 않았다. 수치는 모두 Simulink(데이터셋 시기 모델) 정상상태다. 상수 D0 30 ms(k = 1 띠는"
          " 40 ms)를 돌리고 마지막 1 ms의 max|i_L|을 쓰며, 목표 전력을 감싸는 2.88° 두 계단 사이에서 전력으로 보간한다(HV→LV).",
          "", f"M-S = M-B + 앵커 보정(`train_corrected.py --delta`, 앵커 {len(A)}개 `out/opt4_anchors_r3.csv`). M-S가 M-B와 듀티 0.003"
          f" 넘게 다른 점만 돌렸다. 점 {len(T)}개 중 판정 가능 {n_meas}개."
          + ("" if n_meas == len(T) else " **미완료 점이 있다 — 아래 판정은 잠정이다**(`surface_opt_check.py next --fams ms,msk1`)."),
          "", "| 묶음 | V2 | \\|P\\| | w | 앵커 무리 | M-S (D1, D2) | M-S Ipk | M-B (D1, D2) | M-B Ipk | SPS | 이득 M-B − M-S | 허용 | 분류 |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]

    def ip(r, k):
        i, o = r[f"Ipk_{k}"], r[f"osc_{k}"]
        return (f"{i:.3f}" + (" (진동)" if np.isfinite(o) and o > 0.02 else "")) if np.isfinite(i) else "–"
    for _, r in T.iterrows():
        md.append(f"| {r['fam']} | {r['V2']:g} V | {r['P']:.0f} W | {r['w']:.2f} | {r['group']} | ({r['D1_MS']:.3f}, {r['D2_MS']:.3f}) | "
                  f"{ip(r, 'MS')} | ({r['D1_MB']:.3f}, {r['D2_MB']:.3f}) | {ip(r, 'MB')} | {ip(r, 'SPS')} | "
                  + (f"{r['gain']:+.3f}" if np.isfinite(r["gain"]) else "–") + f" | {r['tol']:.3f} | {CLS_KO[r['cls']]}"
                  + (" · M-S > SPS" if r["viol_MS"] is True else "") + (" · M-B > SPS" if r["viol_MB"] is True else "") + " |")
    md += ["", "단위 A. w = M-S 목표가 앵커 쪽으로 옮겨진 비율(0.5 이상 = 앵커 영역, 미만 = 번짐 영역)."]
    for reg, ko in (("anchor", "앵커 영역"), ("spill", "번짐 영역")):
        o = s[reg]
        md.append(f"- {ko}: {o['n']}점(판정 가능 {o['n_dec']}), 평균 이득 {o['mean_gain']:+.3f} A({o['n_gain']}점), 개선 {o['better']} · "
                  f"M-B 진동 해소 {o['fixes_osc']} · 같음 {o['same']} · 열화 {o['worse']} · M-S 진동 {o['new_osc']}")
    if s.get("A") is not None:
        md.append(f"- A(앵커 영역 이득 > 0.03 A, 개선 > 열화): {'성립' if s['A'] else '불성립'}; B(번짐 영역 평균 ≥ −0.03 A): "
                  f"{'성립' if s['B'] else '불성립'}{'' if s.get('B_tested') else '(3점 미만이라 시험 안 됨)'}; C(SPS보다 나쁜 점 수 M-S "
                  f"{s['viol_MS']} ≤ M-B {s['viol_MB']}, {s['n_viol_checked']}점에서): {'성립' if s['C'] else '불성립'}")
    md += ["", f"**판정: {MS_VERDICT_KO[code]}**.", "",
           "참고(판정 아님): 앵커 무리별 평균 이득 — " + "; ".join(
               f"{ {'knee': 'SPS 무릎', 'ridge': '109.2 V 경부하 능선', 'k1': 'k = 1 띠'}.get(k, k)} {n}점 중 {m}점 평균 {g:+.3f} A"
               for k, (n, m, g) in grp.items()) + f". 창 민감도(15 ms–끝 최대 창으로 같은 규칙): {MS_VERDICT_KO[code_w]}.",
           "", "그림: `out/fig_ms_check.png`. 표 원본: `out/ms_check.csv`, 로그: `out/ms_check.log`."]
    open(os.path.join(OUT, "ms_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")


def ms_fig(T, code):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    f, ax = plt.subplots(1, 2, figsize=(12.5, 4.2), constrained_layout=True)
    col = {"better": AQUA, "fixes_osc": AQUA, "same": MUTED, "worse": ORANGE, "new_osc": ORANGE, "undecidable": INK2}
    for a, fam, xk in ((ax[0], "ms", "P"), (ax[1], "msk1", "V2")):
        g = T[T["fam"] == fam]
        if g.empty:
            a.set_visible(False)
            continue
        y = g["gain"].fillna(0.0)
        num = g["gain"].notna()
        osc = g["cls"].isin(["fixes_osc", "new_osc"])
        und = g["cls"].eq("undecidable")
        a.scatter(g.loc[num, xk], y[num], c=[col[c] for c in g.loc[num, "cls"]], s=np.where(g.loc[num, "w"] >= 0.5, 46, 18),
                  zorder=3, edgecolors="white", linewidths=0.6)
        a.scatter(g.loc[osc, xk], y[osc], c=[col[c] for c in g.loc[osc, "cls"]], marker="X",
                  s=np.where(g.loc[osc, "w"] >= 0.5, 60, 30), zorder=3)             # oscillation class: no gain value
        a.scatter(g.loc[und, xk], y[und], facecolors="none", edgecolors=MUTED, s=16, linewidths=0.7, zorder=2)
        for _, r in g[~und].iterrows():
            a.annotate(f"{r['V2']:g}" if fam == "ms" else f"{r['P']:.0f}", (r[xk], 0.0 if not np.isfinite(r["gain"]) else r["gain"]),
                       fontsize=6.5, color=INK2, xytext=(3, 2), textcoords="offset points")
        a.axhline(0, color=MUTED, lw=0.8)
        a.vlines(g.loc[num, xk], -g.loc[num, "tol"], g.loc[num, "tol"], color=MUTED, alpha=0.5, lw=3, zorder=1)  # per-point tol
        a.set_xlabel("|P| [W] (label: V2)" if fam == "ms" else "V2 [V] (label: |P| W)")
        a.set_ylabel("Ipk(M-B) - Ipk(M-S) [A]")
        a.set_title(("knee / ridge anchors: V2 84, 89, 104, 109.2 V" if fam == "ms" else "k = 1 band (40 ms runs)")
                    + "  (big dots: anchor region w >= 0.5)", fontsize=9)
    ax[0].plot([], [], "o", color=AQUA, label="M-S better (or fixes M-B oscillation)")
    ax[0].plot([], [], "o", color=ORANGE, label="M-S worse (or oscillates)")
    ax[0].plot([], [], "o", color=MUTED, label="same within tolerance (grey bar = that point's tolerance)")
    ax[0].plot([], [], "X", color=INK2, label="oscillation class (M-B osc fixed / M-S osc new), no gain value")
    ax[0].plot([], [], "o", mfc="none", mec=MUTED, label="undecidable (not bracketed yet, or both oscillate)")
    ax[0].legend(fontsize=7.5, frameon=False, loc="upper right")
    f.suptitle(f"OPT4: M-S vs M-B in Simulink -- verdict {code} (rules: ms_verdict.py)", fontsize=10)
    f.savefig(os.path.join(OUT, "fig_ms_check.png"), dpi=130)
    plt.close(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "ms_compare"])
    ap.add_argument("--fams", default=",".join(FAMS))
    ap.add_argument("--partial", action="store_true", help="grid family from an incomplete R6 (testing only)")
    a = ap.parse_args()
    fams = tuple(f for f in a.fams.split(",") if f)
    if a.cmd in ("make", "next"):
        plan(fams, a.partial)
    elif a.cmd == "ms_compare":
        ms_compare()
    else:
        compare(a.partial)


if __name__ == "__main__":
    main()
