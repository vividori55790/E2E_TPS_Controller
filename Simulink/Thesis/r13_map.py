"""
r13_map.py -- (2026-10-30) R13 stage 1: a Simulink steady-state map on the 25 ns grid over V2 7 values x |P| 8 values,
SPS / M-B / M-S and a duty stencil around M-S. Stage 2 (retraining, M-D) is decided by the rule below.

The R13 plan (TASKS.md, 10-21): rebuild the user's dataset with the interpolation off at V2 7 values, take the Simulink
optimum, retrain the same E2E regression on it and compare with M-B. What the experiments since then changed:
  * the 1 ms staircase itself (R6, interpolation off) inflates the SPS rows: 87 % of "the fixed dataset beats M-B by
    0.40 A" is that inflation (q4_decompose.py), so a staircase is not an optimum source. Every value here is the
    constant-D0 steady state at two adjacent 2.88-deg D0 levels, interpolated at |P| -- the OPT / GRID / R.4 method.
  * the dataset-era model steps controller and Simscape at 100 ns: Ipk is a staircase in duty cells of 0.004 (~0.09 A
    at the same power, duty_quant_check.py) and 6 / 41 GRID 100 ns runs have |mean i_L| > 20 mA. On the 25 ns grid
    (fine1b + fineR, 101 runs) 3 do (21-45 mA, constant Ipk over the windows), the 1 ms window peaks from 10 ms on agree
    within 0.105 A (p90 0.0001 A), and the Python plant on a 25 ns grid predicts the Simulink power within 8 W MAE
    (100 ns: 44 W) and Ipk within 0.040 A (100 ns: 0.154 A; first 117 GRID runs). GRID (fine_grid_check.py compare,
    10-30 15:07): every "M-S lower than M-B" statement of the 100 ns runs is gone at 25 ns (5 / 5), "M-B above SPS" at
    84 V 400 / 800 W reversed / gone, the 8 "lower than SPS" statements kept. So the map is measured at 25 ns with
    the GRID runner and window (verify_steady_fine.m, 20 ms, Ipk_at_20ms / P_LV_at_20ms); the GRID runs at 84 /
    109.2 V are reused where the duties match. Set X re-measures at 25 ns the other 100 ns statements the report
    leans on (goal 3 violations at 93 V and of M-S, the k = 1 valley at 95 V, the M-S gains / losses at 96-97 V).

Grid: V2 (84.0, 88.2, 92.4, 96.6, 100.8, 105.0, 109.2) V (k = n V2 / V1 = 0.884 ... 1.149; 96.6 V = k 1.017 lies in the
k = 1 band of k1map), |P| (100, 150, 200, 300, 400, 600, 800, 1000) W, HV -> LV, V1 380 V. Candidates at each point:
SPS (D_SAFE, D_SAFE); M-B (out/model_smooth_reghdc_b3.json); M-S (out/model_smooth_reghdc_MS.json, the adopted surface);
the stencil M-S + (-d, 0), (+d, 0), (0, -d), (0, +d) on (D1, D2), d = 0.03 (30 duty cells at 25 ns), clamped to
[0.02, D_SAFE]. A candidate within 0.003 (max |dD|) of an earlier one in that order uses the earlier one's runs (so M-S
uses M-B's runs where they agree, as ms_compare does); an arm that lands on M-S itself, or that the ideal model cannot
reach on the first branch, is dropped. Value = Ipk at |P| by linear interpolation in the 20 ms window power between two
adjacent levels (fine_grid_check.at_power; D0 = j 2.88 deg, knife-edge phases 1e-4 deg earlier = fine_grid_check._point);
clean = both levels |mean i_L| <= 20 mA.

RULES (registered 2026-10-30 15:20, before any R13 run exists; do not change after the results):
  tol = max(0.03 A, 2 % Ipk(SPS)) at the point (2 % of Ipk(M-S) if SPS is not clean) -- the OPT / GRID tolerance.
  Q1 local optimality of M-S: decidable if M-S and at least two stencil arms are clean; gap = Ipk(M-S) - min over all
     clean candidates. gap <= tol -> "optimal", else "not optimal" (the best candidate is named).
  Q2 goal 3, for M-S and for M-B: d = Ipk(ctrl) - Ipk(SPS), both clean: d > tol "worse", d < -tol "better", else "same".
     Goal 3 holds on the map for a controller iff it has no "worse" point.
  Q3 headline: mean and median of Ipk(SPS) - Ipk(ctrl), in A and in % of Ipk(SPS), overall and per V2, over the points
     clean for both. Reported, no threshold.
  Q4 M-S vs M-B where they differ (> 0.003): d = Ipk(M-B) - Ipk(M-S), both clean: d > tol "better", d < -tol "worse",
     else "same". "M-S holds at 25 ns" iff #better >= #worse and the mean d >= 0; else "M-S does not hold".
  Stage 2 trigger: retrain (M-D = train_corrected.py --delta on M-B, as M-S was made) iff at least 5 decidable points
     are "not optimal". Its anchors are then every decidable point -- target = the best clean candidate there (M-S's
     own duties where M-S is optimal), wV 2.1 V (half the V2 spacing), wP half the gap to the nearer neighbouring |P|;
     its verification points and adoption rule are registered before its runs. Fewer than 5 -> no retraining.
  Fewer than 75 % of the 56 points decidable (Q1) -> every answer is reported as provisional.
  Q5 set X (not on the map; not counted in Q1-Q4): (93, 100), (93, 150), (84, 700), (109.2, 700), (95, 70), (95, 100),
     (96, 150), (96, 200), (96, 300), (97, 150) (V, W) x {SPS, M-B, M-S} (+ D1 = D2 = 0.40 at 95 V, the OPT (ii)
     candidate). Each pair (M-B, SPS), (M-S, SPS), (M-S, M-B), (D0.40, SPS) with a 100 ns statement "X lower than Y"
     (fine_grid_check.results_100, 20 ms window, both clean, |d| > max(0.03 A, 2 % Ipk(SPS at 100 ns))) is
     classified at 25 ns by fine_grid_check.rule_r2: kept / gone / reversed / undecidable.

    python -I -X utf8 Simulink/Thesis/r13_map.py predict          -> out/r13_pred.csv (levels from the 25 ns plant)
    python -I -X utf8 Simulink/Thesis/r13_map.py make --part a    -> out/r13a_points_r1.csv (set X + map core; b = arms)
    python automation/job.py submit r13a --interactive --after r12r1b -- matlab -batch "cd('Simulink/Thesis'); r13_rounds('a', 1, 5, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/r13_map.py next --part a    (not bracketed -> next round; r13_rounds calls it)
    python -I -X utf8 Simulink/Thesis/r13_map.py compare          -> out/r13_map.{csv,log}, out/r13_points.csv, out/r13_verdict.md
    python -I -X utf8 Simulink/Thesis/r13_map.py selftest         (the rules and the candidate set on made-up numbers)

ADDED 10-30 18:15, AFTER the r13a results were seen, so NOT part of any verdict above (reported beside them; the rules
and the rule values are untouched): each rule value is the largest of 80 period peaks. r13a shows that 15 % of the runs
are not periodic in that window (per-period peaks in out/iL_pp/) and that all three Q2 "worse" points have such a run
in their bracket: a single-period spike of about one grid step V1 Ts / L (0.106 A at 25 ns, 0.42 A at 100 ns), or a slow
offset cycle of i_L over tens of periods whose window mean stays under the 20 mA "clean" limit. `compare` therefore also
reports Q1-Q4 on the median period peak, the run classes, and the loss / efficiency at the same interpolation (R8 at
25 ns), and draws out/fig_r13_flicker.png.
"""
import argparse
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `predict` runs beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
G = sw.G
TS = fg.TS_FINE
WIN = fg.WIN
OSC = fg.OSC
TOL_ABS, TOL_REL = 0.03, 0.02
V2S = (84.0, 88.2, 92.4, 96.6, 100.8, 105.0, 109.2)
PS = (100.0, 150.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)
DELTA = 0.03
SAME = 0.003
ARMS = (("D1-", -1, 0), ("D1+", 1, 0), ("D2-", 0, -1), ("D2+", 0, 1))
ARM_NAMES = tuple(a[0] for a in ARMS)
CORE = ("SPS", "M-B", "M-S")
MIN_ARMS = 2
STAGE2_MIN = 5
DECIDABLE_MIN = 0.75
PLATEAU = 5.0                               # W per level (planning only): below this P(D0) is flat, see extend()
PRED = os.path.join(OUT, "r13_pred.csv")
X_PTS = ((93.0, 100.0, ()), (93.0, 150.0, ()), (84.0, 700.0, ()), (109.2, 700.0, ()), (95.0, 70.0, (0.40,)),
         (95.0, 100.0, (0.40,)), (96.0, 150.0, ()), (96.0, 200.0, ()), (96.0, 300.0, ()), (97.0, 150.0, ()))
X_PAIRS = (("M-B", "SPS"), ("M-S", "SPS"), ("M-S", "M-B"), ("D0.40", "SPS"))
PATS = ("fine_sweep_r*.csv", "r13a_sweep_r*.csv", "r13b_sweep_r*.csv")   # the 25 ns result files of the map


# ----------------------------------------------------------------------------------------------------- candidates
def _dd(x):
    return round(min(max(float(x), 0.02), sw.D_SAFE), 9)


def _near(a, b):
    return max(abs(a["D1"] - b["D1"]), abs(a["D2"] - b["D2"])) <= SAME


def _alias(c, earlier):
    c["alias"] = None
    for e in earlier:
        if _near(c, e):
            c["alias"] = e["alias"] or e["name"]
            break
    return c


def stencil(b, s):
    """candidate list at one point from M-B (b1, b2) and M-S (s1, s2): dicts name, D1, D2, alias (None = own runs)"""
    cs = [dict(name="SPS", D1=sw.D_SAFE, D2=sw.D_SAFE), dict(name="M-B", D1=round(b[0], 9), D2=round(b[1], 9)),
          dict(name="M-S", D1=_dd(s[0]), D2=_dd(s[1]))]
    cs += [dict(name=nm, D1=_dd(s[0] + a1 * DELTA), D2=_dd(s[1] + a2 * DELTA)) for nm, a1, a2 in ARMS]
    out = []
    for c in cs:
        if c["name"] in ARM_NAMES and _near(c, cs[2]):
            continue                                          # clamped onto M-S itself: not an arm
        out.append(_alias(c, out))
    return out


def core_x(b, s, ds):
    """set X: SPS, M-B, M-S (aliased as on the map) + D1 = D2 = d candidates"""
    cs = [c for c in stencil(b, s) if c["name"] in CORE]
    for dv in ds:
        cs.append(_alias(dict(name=f"D{dv:.2f}", D1=_dd(dv), D2=_dd(dv)), cs))
    return cs


def candidates(which="map"):
    """[(V2, P, [candidates])] over the map (or set X); non-core candidates the ideal model cannot reach on the first
    branch are dropped"""
    import pi_steady as ps
    import surface_opt_check as so
    ms, mb = so._ms(), so._mb()
    one = lambda m, p, v: tuple(float(np.ravel(x)[0]) for x in m.predict(np.array([p]), np.array([v])))  # noqa: E731
    if which == "map":
        pts = [(v, p, stencil(one(mb, p, v), one(ms, p, v))) for v in V2S for p in PS]
    else:
        pts = [(v, p, core_x(one(mb, p, v), one(ms, p, v), ds)) for v, p, ds in X_PTS]
    flat = [(i, c) for i, (v, p, cs) in enumerate(pts) for c in cs if c["alias"] is None]
    P = np.array([pts[i][1] for i, _ in flat])
    V = np.array([pts[i][0] for i, _ in flat])
    ok = np.isfinite(ps.operating_point(P, np.array([c["D1"] for _, c in flat]), np.array([c["D2"] for _, c in flat]),
                                        V)["D0"])
    bad = {(i, c["name"]) for (i, c), o in zip(flat, ok) if not o}
    out = []
    for i, (v, p, cs) in enumerate(pts):
        drop = {c["name"] for c in cs if (i, c["name"]) in bad and c["name"] not in CORE}
        if any((i, n) in bad for n in CORE):
            print(f"warning: {v:g} V {p:g} W: a core candidate is unreachable on the first branch (ideal model)")
        out.append((v, p, [c for c in cs if c["name"] not in drop and (c["alias"] not in drop)]))
    return out


def run_configs(part="all"):
    """the configurations that get their own runs in run order; part a = set X + the map's core candidates (SPS, M-B,
    M-S), part b = the map's stencil arms (two MATLAB jobs, so that other runs can go between them)"""
    cf = []
    if part in ("a", "all"):
        for k, (v, p, cs) in enumerate(candidates("x")):
            cf += [dict(set="X", pair=k, ctrl=c["name"], V2=v, P=p, D1=c["D1"], D2=c["D2"]) for c in cs
                   if c["alias"] is None]
    pts = candidates("map")
    for rank in ((0,) if part == "a" else (1,) if part == "b" else (0, 1)):
        for v, p, cs in pts:
            for c in cs:
                if c["alias"] is None and (c["name"] in CORE) == (rank == 0):
                    cf.append(dict(set="R", pair=V2S.index(v), ctrl=c["name"], V2=v, P=p, D1=c["D1"], D2=c["D2"]))
    return cf


# ----------------------------------------------------------------------------------------------------- data
def results_25(pats=PATS):
    fs = [f for pat in pats for f in sorted(glob.glob(os.path.join(OUT, pat))) if not f.endswith("_in.csv")]
    d = fg._load(fs, fine=True) if fs else None
    if d is None:
        return None
    d = d[np.isclose(d["Ts"], TS, rtol=1e-6, atol=0)]
    return d.drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def _runs(d, c):
    if d is None:
        import pandas as pd
        return pd.DataFrame()
    return fg.runs_of(d, c, TS)


# ----------------------------------------------------------------------------------------------------- planning
def first_bracket(lv, Pd, P):
    """the first adjacent pair (j, j + 1) from below whose powers bracket P, else None"""
    order = np.argsort(lv)
    lv, Pd = np.asarray(lv)[order], np.asarray(Pd)[order]
    for i in range(len(lv) - 1):
        if lv[i + 1] == lv[i] + 1 and min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]):
            return int(lv[i]), int(lv[i + 1])
    return None


def predict(cf=None, path=PRED):
    """two adjacent levels per configuration from the Python plant on the 25 ns grid -> out/r13_pred.csv"""
    import time

    import pandas as pd
    import pi_steady as ps
    import plant_full as pf
    t0 = time.time()
    cf = run_configs() if cf is None else cf
    n = len(cf)
    P, V = np.array([c["P"] for c in cf]), np.array([c["V2"] for c in cf])
    D1, D2 = np.array([c["D1"] for c in cf]), np.array([c["D2"] for c in cf])
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS * 1e9)
    D0i = ps.operating_point(P, D1, D2, V)["D0"]
    j0 = np.floor(np.nan_to_num(D0i) / G).astype(int)
    known = {}

    def ev(pairs):
        pairs = [q for q in pairs if q not in known]
        if not pairs:
            return
        I = np.array([i for i, _ in pairs])
        sd = [fg._safe(j * G, D1[i], D2[i], TS) for i, j in pairs]
        r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]),
                            np.full(len(pairs), 380.0), V[I])
        for q, a, b in zip(pairs, -r["P_LV"], r["Ipk"]):
            known[q] = (float(a), float(b))
        print(f"  plant: {len(pairs)} evaluations ({time.time() - t0:.0f} s)", flush=True)

    lo, hi = j0 - 1, j0 + 2
    ev([(i, j) for i in range(n) for j in range(lo[i], hi[i] + 1)])
    br = [None] * n
    for _ in range(8):
        todo = []
        for i in range(n):
            if br[i] is not None:
                continue
            js = list(range(lo[i], hi[i] + 1))
            Pd = [known[(i, j)][0] for j in js]
            br[i] = first_bracket(js, Pd, P[i])
            if br[i] is None:
                if max(Pd) < P[i]:
                    todo += [(i, hi[i] + 1), (i, hi[i] + 2)]
                    hi[i] += 2
                else:
                    todo += [(i, lo[i] - 1), (i, lo[i] - 2)]
                    lo[i] -= 2
        if not todo:
            break
        ev(todo)
    rows = []
    for i, c in enumerate(cf):
        b = br[i] or (int(j0[i]), int(j0[i]) + 1)
        rows.append(dict(c, lo=b[0], hi=b[1], found=br[i] is not None, P_lo=known.get((i, b[0]), (np.nan,))[0],
                         P_hi=known.get((i, b[1]), (np.nan,))[0], Ipk_lo=known.get((i, b[0]), (np.nan, np.nan))[1],
                         Ipk_hi=known.get((i, b[1]), (np.nan, np.nan))[1]))
    T = pd.DataFrame(rows)
    T.to_csv(path, index=False, float_format="%.9g")
    print(f"wrote {path}: {n} configurations, {int(T['found'].sum())} bracketed by the plant, "
          f"{len(known)} plant evaluations, {time.time() - t0:.0f} s")


def _pred_levels(path=PRED):
    import pandas as pd
    if not os.path.isfile(path):
        sys.exit(f"{path} not found: run the `predict` command first")
    T = pd.read_csv(path)
    return {(round(r["V2"], 6), round(r["P"], 6), round(r["D1"], 9), round(r["D2"], 9)): (int(r["lo"]), int(r["hi"]))
            for _, r in T.iterrows()}


def _key(c):
    return round(c["V2"], 6), round(c["P"], 6), round(c["D1"], 9), round(c["D2"], 9)


def make(part="a"):
    d, lv, pts, skip = results_25(), _pred_levels(), [], 0
    cf = run_configs(part)
    for c in cf:
        g = _runs(d, c)
        if not g.empty and np.isfinite(fg.at_power(g, c["P"])[0]):
            skip += 1
            continue
        have = set(g["level"].astype(int)) if not g.empty else set()
        if _key(c) not in lv:
            sys.exit(f"no predicted levels for {c} -- rerun `r13_map.py predict`")
        pts += [fg._point(c, j, TS) for j in lv[_key(c)] if j not in have]
    print(f"part {part}: {len(cf)} configurations, {skip} already bracketed by existing 25 ns runs")
    fg._write(os.path.join(OUT, f"r13{part}_points_r1.csv"), pts)


def round_no(part):
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, f"r13{part}_sweep_r*.csv"))
          for m in [re.search(rf"r13{part}_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def extend(lv, Pd, P, cap=3):
    """levels to add for a configuration whose measured levels do not bracket P: fill a gap between two non-adjacent
    levels that straddle P, else step past the end, as many levels as the end spacing predicts (1 ... cap); nothing
    if the power falls toward that end (past the first-branch peak: |P| is out of reach). A flat end (|slope| below
    PLATEAU W per level) is the dead-time plateau of P(D0), one 2.88 deg wide (10-30: 88.2 V 200 W M-B 192.15 / 192.10 W
    at j -1 / 0, 92.4 V SPS 122.6 / 121.1 W at j 0 / 1), not the peak: one more level"""
    order = np.argsort(lv)
    lv, Pd = np.asarray(lv, int)[order], np.asarray(Pd, float)[order]
    for i in range(len(lv) - 1):
        if min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]) and lv[i + 1] > lv[i] + 1:
            return list(range(lv[i] + 1, lv[i + 1]))
    up = Pd.max() < P
    k = 1
    if len(lv) >= 2:
        s = (Pd[-1] - Pd[-2]) / (lv[-1] - lv[-2]) if up else (Pd[1] - Pd[0]) / (lv[1] - lv[0])
        if not np.isfinite(s) or s <= -PLATEAU:
            return []
        if s >= PLATEAU:
            k = int(np.clip(np.ceil(abs(P - (Pd[-1] if up else Pd[0])) / s), 1, cap))
    end = int(lv[-1] if up else lv[0])
    return [end + (q if up else -q) for q in range(1, k + 1)]


def next_round(part="a"):
    d = results_25()
    if d is None:
        sys.exit("no 25 ns runs yet")
    lv0, pts = _pred_levels(), []
    for c in run_configs(part):
        g = _runs(d, c)
        if g.empty:
            pts += [fg._point(c, j, TS) for j in lv0.get(_key(c), ())]
            continue
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [fg._point(c, j, TS) for j in extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"r13{part}_points_r{round_no(part)}.csv"), pts)
    else:
        print(f"part {part}: every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def clean(x):
    return x is not None and np.isfinite(x[0]) and np.isfinite(x[1]) and x[1] <= OSC


def tol_at(vals):
    ref = vals.get("SPS") if clean(vals.get("SPS")) else vals.get("M-S")
    return max(TOL_ABS, TOL_REL * ref[0]) if clean(ref) else TOL_ABS


def rule_q1(vals, tol):
    """vals: name -> (Ipk, osc) -> (code, gap, best)"""
    if not clean(vals.get("M-S")) or sum(clean(vals.get(a)) for a in ARM_NAMES) < MIN_ARMS:
        return "undecidable", np.nan, None
    cl = {k: v[0] for k, v in vals.items() if clean(v)}
    best = min(cl, key=cl.get)
    gap = vals["M-S"][0] - cl[best]
    return ("optimal" if gap <= tol else "not optimal"), float(gap), best


def rule_q2(x, sps, tol):
    """controller x vs SPS -> worse / better / same / undecidable, d = Ipk(x) - Ipk(SPS)"""
    if not (clean(x) and clean(sps)):
        return "undecidable", np.nan
    d = x[0] - sps[0]
    return ("worse" if d > tol else "better" if d < -tol else "same"), float(d)


def rule_q4(ms, mb, tol):
    """M-S vs M-B -> better / worse / same / undecidable, d = Ipk(M-B) - Ipk(M-S)"""
    if not (clean(ms) and clean(mb)):
        return "undecidable", np.nan
    d = mb[0] - ms[0]
    return ("better" if d > tol else "worse" if d < -tol else "same"), float(d)


def q4_verdict(codes, ds):
    nb, nw = codes.count("better"), codes.count("worse")
    ds = [x for x in ds if np.isfinite(x)]
    if not ds:
        return "undecidable"
    return "M-S holds" if nb >= nw and np.mean(ds) >= 0 else "M-S does not hold"


def stage2(q1codes):
    return "retrain" if q1codes.count("not optimal") >= STAGE2_MIN else "no retraining"


def provisional(q1codes, n=len(V2S) * len(PS)):
    return sum(c != "undecidable" for c in q1codes) < DECIDABLE_MIN * n


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    c = lambda i, o=0.0: (i, o)  # noqa: E731
    base = {"SPS": c(2.0), "M-B": c(1.80), "M-S": c(1.70), "D1-": c(1.75), "D1+": c(1.72), "D2-": c(1.90)}
    chk("tol from SPS", round(tol_at(base), 6), 0.04)
    chk("tol floor", round(tol_at({"SPS": c(1.0)}), 6), 0.03)
    chk("tol from M-S when SPS oscillates", round(tol_at({"SPS": c(3.0, 0.05), "M-S": c(2.5)}), 6), 0.05)
    chk("Q1 optimal", rule_q1(base, 0.04)[0], "optimal")
    chk("Q1 not optimal names the arm", rule_q1(dict(base, **{"D1+": c(1.60)}), 0.04)[0::2], ("not optimal", "D1+"))
    chk("Q1 within tol stays optimal", rule_q1(dict(base, **{"D1+": c(1.665)}), 0.04)[0], "optimal")
    chk("Q1 SPS best", rule_q1(dict(base, SPS=c(1.50)), 0.04)[0::2], ("not optimal", "SPS"))
    chk("Q1 oscillating arm ignored", rule_q1(dict(base, **{"D1+": c(1.0, 0.05)}), 0.04)[0], "optimal")
    chk("Q1 one clean arm -> undecidable",
        rule_q1({"SPS": c(2.0), "M-S": c(1.7), "D1-": c(1.6), "D2-": c(np.nan)}, 0.04)[0], "undecidable")
    chk("Q1 M-S oscillating -> undecidable", rule_q1(dict(base, **{"M-S": c(1.7, 0.03)}), 0.04)[0], "undecidable")
    chk("Q2 worse", rule_q2(c(2.05), c(2.0), 0.04)[0], "worse")
    chk("Q2 same", rule_q2(c(2.03), c(2.0), 0.04)[0], "same")
    chk("Q2 better", rule_q2(c(1.9), c(2.0), 0.04)[0], "better")
    chk("Q2 SPS oscillating", rule_q2(c(1.9), c(2.0, 0.03), 0.04)[0], "undecidable")
    chk("Q4 better", rule_q4(c(1.7), c(1.8), 0.04)[0], "better")
    chk("Q4 worse", rule_q4(c(1.9), c(1.8), 0.04)[0], "worse")
    chk("Q4 verdict holds", q4_verdict(["better", "worse", "same"], [0.1, -0.05, 0.0]), "M-S holds")
    chk("Q4 verdict: more worse", q4_verdict(["better", "worse", "worse"], [0.3, -0.05, -0.05]), "M-S does not hold")
    chk("Q4 verdict: mean < 0", q4_verdict(["better", "worse"], [0.05, -0.2]), "M-S does not hold")
    chk("stage 2 at 5", stage2(["not optimal"] * 5 + ["optimal"] * 40), "retrain")
    chk("stage 2 at 4", stage2(["not optimal"] * 4 + ["optimal"] * 40), "no retraining")
    chk("provisional below 75 %", provisional(["optimal"] * 41 + ["undecidable"] * 15), True)
    chk("not provisional at 42/56", provisional(["optimal"] * 42 + ["undecidable"] * 14), False)
    s = stencil((0.40, 0.47), (0.45, 0.499))           # D2+ clamps onto M-S -> dropped; D1- (0.42) is new
    chk("stencil: arm onto M-S dropped", [x["name"] for x in s], ["SPS", "M-B", "M-S", "D1-", "D1+", "D2-"])
    s = stencil((0.40, 0.47), (0.48, 0.45))            # D1+ = 0.51 clamps to D_SAFE, 0.02 from M-S -> kept
    chk("stencil: clamp at D_SAFE", next(x for x in s if x["name"] == "D1+")["D1"], round(sw.D_SAFE, 9))
    s = stencil((0.40, 0.47), (0.401, 0.469))          # M-S within 0.003 of M-B -> M-B's runs; arms alias nothing
    chk("stencil: M-S aliases M-B", s[2]["alias"], "M-B")
    chk("stencil: arms around an aliased M-S kept", [x["name"] for x in s[3:]], list(ARM_NAMES))
    s = stencil((0.42, 0.47), (0.45, 0.47))            # D1- = 0.42 = M-B -> uses M-B's runs
    chk("stencil: arm on M-B uses M-B's runs", next(x for x in s if x["name"] == "D1-")["alias"], "M-B")
    s = stencil((0.49999, 0.49999), (0.4999, 0.49995))  # M-S = SPS: every + arm lands on M-S, - arms stay
    chk("stencil: M-S = SPS", ([x["name"] for x in s], s[2]["alias"]), (["SPS", "M-B", "M-S", "D1-", "D2-"], "SPS"))
    chk("set X: D0.40 on M-B uses its runs", [(x["name"], x["alias"]) for x in core_x((0.40, 0.40), (0.30, 0.30), (0.40,))],
        [("SPS", None), ("M-B", None), ("M-S", None), ("D0.40", "M-B")])
    chk("set X: no arms", [x["name"] for x in core_x((0.30, 0.35), (0.33, 0.36), ())], list(CORE))
    chk("bracket: first from below", first_bracket([0, 1, 2, 3], [50, 120, 90, 160], 100), (0, 1))
    chk("bracket: none", first_bracket([0, 1], [50, 80], 100), None)
    chk("extend up by the last spacing", extend([2, 3], [100, 150], 290), [4, 5, 6])
    chk("extend up capped", extend([2, 3], [100, 110], 900), [4, 5, 6])
    chk("extend down", extend([2, 3], [120, 170], 60), [1, 0])
    chk("extend fills a gap", extend([1, 4], [80, 260], 150), [2, 3])
    chk("extend single level", extend([5], [300], 200), [4])
    chk("extend past the peak -> nothing", extend([3, 4], [150, 140], 200), [])
    chk("extend dead-time plateau -> one level", extend([-1, 0], [192.15, 192.10], 200), [1])
    chk("extend small rise is a plateau too", extend([0, 1], [120.0, 122.0], 200), [2])
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def compare():
    import datetime

    import pandas as pd
    d = results_25()
    if d is None:
        sys.exit("no 25 ns runs yet")
    pts = candidates()
    rows, per = [], []
    for v, p, cs in pts:
        by = {c["name"]: c for c in cs}
        vals, lvs = {}, {}
        for c in cs:
            own = by[c["alias"]] if c["alias"] else c
            cfg = dict(set="R", pair=0, ctrl=own["name"], V2=v, P=p, D1=own["D1"], D2=own["D2"])
            I, o, lo = fg.at_power(_runs(d, cfg), p)
            vals[c["name"]], lvs[c["name"]] = (I, o), lo
            rows.append(dict(V2=v, P=p, cand=c["name"], alias=c["alias"] or "", D1=c["D1"], D2=c["D2"], Ipk=I, osc=o,
                             level_lo=lo))
        tol = tol_at(vals)
        q1, gap, best = rule_q1(vals, tol)
        q2s, d2s = rule_q2(vals.get("M-S"), vals.get("SPS"), tol)
        q2b, d2b = rule_q2(vals.get("M-B"), vals.get("SPS"), tol)
        differ = by["M-S"]["alias"] is None
        q4, d4 = rule_q4(vals.get("M-S"), vals.get("M-B"), tol) if differ else ("n/a", np.nan)
        sps = vals.get("SPS")
        per.append(dict(V2=v, P=p, tol=tol, I_SPS=sps[0], I_MB=vals["M-B"][0], I_MS=vals["M-S"][0], q1=q1, gap=gap,
                        best=best or "", best_D1=by[best]["D1"] if best else np.nan,
                        best_D2=by[best]["D2"] if best else np.nan, q2_MS=q2s, d_MS=d2s, q2_MB=q2b, d_MB=d2b,
                        q4=q4, d4=d4, MS_D1=by["M-S"]["D1"], MS_D2=by["M-S"]["D2"], MB_D1=by["M-B"]["D1"],
                        MB_D2=by["M-B"]["D2"], n_arms=sum(a in by for a in ARM_NAMES),
                        n_missing=sum(not np.isfinite(x[0]) for x in vals.values())))
    T, Q = pd.DataFrame(rows), pd.DataFrame(per)
    T.to_csv(os.path.join(OUT, "r13_map.csv"), index=False, float_format="%.6g")
    Q.to_csv(os.path.join(OUT, "r13_points.csv"), index=False, float_format="%.6g")
    lines = []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    f3 = lambda x: f"{x:6.3f}" if np.isfinite(x) else f"{'-':>6}"  # noqa: E731
    o(f"r13_map: Simulink steady state on the 25 ns grid, {WIN} ms window, Ipk at |P| (A); rules in the file header")
    o(f"{'V2':>6} {'|P|':>5} | {'SPS':>6} {'M-B':>6} {'M-S':>6} | " + " ".join(f"{a:>6}" for a in ARM_NAMES)
      + f" | {'tol':>5} Q1 (gap, best) | Q2 M-S, M-B | Q4")
    for (v, p, cs), q in zip(pts, per):
        x = T[(T["V2"] == v) & (T["P"] == p)].set_index("cand")
        cell = lambda n: (f3(x.loc[n, "Ipk"]) + ("~" if np.isfinite(x.loc[n, "osc"]) and x.loc[n, "osc"] > OSC else " ")  # noqa: E731
                          if n in x.index else f"{'.':>6} ")
        o(f"{v:6.1f} {p:5.0f} | {cell('SPS')}{cell('M-B')}{cell('M-S')}| " + "".join(cell(a) for a in ARM_NAMES)
          + f"| {q['tol']:.3f} {q['q1']:<11} {q['gap']:+.3f} {q['best']:<4} | {q['q2_MS']:<6} {q['q2_MB']:<6} | {q['q4']}")
    o("(~ = oscillating, |mean i_L| > 20 mA; . = candidate dropped; - = not bracketed yet)")
    q1c = list(Q["q1"])
    ndec = sum(c != "undecidable" for c in q1c)
    prov = provisional(q1c)
    o(f"\nQ1 local optimality of M-S: {ndec}/{len(Q)} points decidable{' -- PROVISIONAL (< 75 %)' if prov else ''}; "
      f"optimal {q1c.count('optimal')}, not optimal {q1c.count('not optimal')}")
    for _, q in Q[Q["q1"] == "not optimal"].iterrows():
        o(f"   {q['V2']:6.1f} V {q['P']:5.0f} W: best {q['best']} ({q['best_D1']:.4f}, {q['best_D2']:.4f}) "
          f"{q['gap']:.3f} A below M-S ({q['MS_D1']:.4f}, {q['MS_D2']:.4f}), tol {q['tol']:.3f}")
    s2 = stage2(q1c)
    o(f"=> stage 2: {s2} ({q1c.count('not optimal')} 'not optimal' points, trigger >= {STAGE2_MIN})")
    o("\nQ2 goal 3 (controller vs SPS):")
    q2res = {}
    for k, nm in (("MS", "M-S"), ("MB", "M-B")):
        cc = list(Q[f"q2_{k}"])
        q2res[nm] = cc.count("worse")
        o(f"   {nm}: worse {cc.count('worse')}, same {cc.count('same')}, better {cc.count('better')}, "
          f"undecidable {cc.count('undecidable')} -> goal 3 {'holds' if cc.count('worse') == 0 else 'broken'} on the map")
        for _, q in Q[Q[f"q2_{k}"] == "worse"].iterrows():
            o(f"      worse: {q['V2']:6.1f} V {q['P']:5.0f} W  +{q[f'd_{k}']:.3f} A (tol {q['tol']:.3f})")
    o("\nQ3 headline Ipk(SPS) - Ipk(ctrl), points clean for both:")
    q3 = {}
    for k, nm, ic in (("MS", "M-S", "I_MS"), ("MB", "M-B", "I_MB")):
        m = Q[Q[f"q2_{k}"] != "undecidable"]
        if m.empty:
            o(f"   {nm}: no clean point yet")
            continue
        dI = m["I_SPS"] - m[ic]
        pc = 100 * dI / m["I_SPS"]
        q3[nm] = (len(m), dI.mean(), dI.median(), pc.mean())
        o(f"   {nm}: {len(m)} points, mean {dI.mean():+.3f} A (median {dI.median():+.3f}), mean {pc.mean():+.1f} % of SPS")
        for v in V2S:
            mv = m[m["V2"] == v]
            if len(mv):
                dv = mv["I_SPS"] - mv[ic]
                o(f"      {v:6.1f} V: {len(mv)} points mean {dv.mean():+.3f} A ({(100 * dv / mv['I_SPS']).mean():+.1f} %)")
    q4c, q4d = list(Q.loc[Q["q4"] != "n/a", "q4"]), list(Q.loc[Q["q4"] != "n/a", "d4"])
    q4v = q4_verdict(q4c, q4d)
    o(f"\nQ4 M-S vs M-B at the {len(q4c)} points where they differ: better {q4c.count('better')}, same {q4c.count('same')}, "
      f"worse {q4c.count('worse')}, undecidable {q4c.count('undecidable')}; mean Ipk(M-B) - Ipk(M-S) "
      f"{np.nanmean(q4d) if np.isfinite(q4d).any() else np.nan:+.3f} A -> {q4v}")
    q5 = q5_rows(d)
    o("\nQ5 set X: 100 ns statements (20 ms window) re-measured at 25 ns (fine_grid_check.rule_r2):")
    for r in q5:
        o(f"   {r['V2']:6g} V {r['P']:5.0f} W  {r['statement']:<22} {r['d100']:+.3f} A at 100 ns -> {r['d25']:+.3f} A at 25 ns "
          f"(tol {r['tol25']:.3f}): {r['res']}" if r["res"] else
          f"   {r['V2']:6g} V {r['P']:5.0f} W  {r['pair']:<22} no 100 ns statement (100 ns {r['d100']:+.3f} A, 25 ns {r['d25']:+.3f} A)")
    nmiss = int(Q["n_missing"].sum())
    o(f"\ncandidate values not bracketed yet: {nmiss} (map), {sum(r['missing'] for r in q5)} (set X pairs)")
    _, res = _pp_check(per_period(d), pts, o)
    open(os.path.join(OUT, "r13_map.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"optimal": "최적(허용오차 안)", "not optimal": "최적 아님", "undecidable": "판정 불가"}
    md = ["# R13 1단계: 25 ns 격자 Simulink 정상상태 지도 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `r13_map.py compare`. 규칙은 `r13_map.py` 머리말(10-30 15:20, R13 실행 전 등록)이며"
          " 바꾸지 않았다. 모두 Simulink(데이터셋 시기 모델, 제어기·로컬 솔버 25 ns) 정상상태, 상수 D0 두 계단 전력 보간, 20 ms 창.", "",
          f"- 점 {len(Q)}개(V2 7값 × |P| 8값) 중 Q1 판정 가능 {ndec}개" + (" — **잠정**(75 % 미만)" if prov else ""),
          f"- Q1 M-S 국소 최적성: 최적 {q1c.count('optimal')}, 최적 아님 {q1c.count('not optimal')} → 2단계(재학습): **{'함' if s2 == 'retrain' else '안 함'}**",
          f"- Q2 목표 3: M-S가 SPS보다 나쁜 점 {q2res.get('M-S', 0)}, M-B {q2res.get('M-B', 0)}"]
    for nm, (n_, mu, med, pc) in q3.items():
        md.append(f"- Q3 {nm}: {n_}점 평균 SPS − {nm} {mu:+.3f} A(중앙값 {med:+.3f}), SPS 대비 {pc:+.1f} %")
    md.append(f"- Q4 M-S 대 M-B({len(q4c)}점): 개선 {q4c.count('better')}, 같음 {q4c.count('same')}, 열화 {q4c.count('worse')} → **{q4v}**")
    md += [f"- Q5 {r['V2']:g} V {r['P']:.0f} W: '{r['statement']}' {r['d100']:+.3f} A(100 ns) → {r['d25']:+.3f} A(25 ns): **{r['res']}**"
           for r in q5 if r["res"]]
    if nmiss:
        md.append(f"\n**아직 감싸지 못한 후보 값 {nmiss}개 — 잠정**(`r13_map.py next`).")
    md += ["", "| V2 | |P| | SPS | M-B | M-S | Q1 | 차(최선) | Q2 M-S | Q2 M-B |", "|---|---|---|---|---|---|---|---|---|"]
    for _, q in Q.iterrows():
        md.append(f"| {q['V2']:g} | {q['P']:.0f} | {f3(q['I_SPS'])} | {f3(q['I_MB'])} | {f3(q['I_MS'])} | {KO[q['q1']]} | "
                  f"{q['gap']:+.3f} ({q['best']}) | {q['q2_MS']} | {q['q2_MB']} |")
    md += ["", "## 판정 아닌 점검 (규칙 값은 80주기 피크의 최댓값이다; 아래는 결과를 본 뒤 정한 주기 피크 중앙값·손실, `r13_map.py` 머리말 10-30 18:15)", ""]
    md.append(f"- Q1 중앙값: 판정 가능 {res['q1'][2]}점, 최적 {res['q1'][0]}, 최적 아님 {res['q1'][1]}")
    for k, nm in (("MS", "M-S"), ("MB", "M-B")):
        if f"q3_{k}" in res:
            n_, mu, med, pc = res[f"q3_{k}"]
            md.append(f"- Q3 중앙값 {nm}: {n_}점 평균 {mu:+.3f} A(중앙값 {med:+.3f}), SPS 대비 {pc:+.1f} %")
        if f"eff_{k}" in res:
            n_, dl, de, nb = res[f"eff_{k}"]
            md.append(f"- 손실 {nm} − SPS: 깨끗한 {n_}점 평균 {dl:+.2f} W, Δη {de:+.3f} pp, η가 0.05 pp 넘게 낮은 점 {nb}")
    md.append(f"- Q4 중앙값: 개선 {res['q4'][0]}, 열화 {res['q4'][1]}, 평균 {res['q4'][2]:+.3f} A")
    md += ["", "표 원본 `out/r13_map.csv`(후보별), `out/r13_points.csv`(점별), `out/r13_points_med.csv`(중앙값·손실), 로그 `out/r13_map.log`,"
           " 그림 `out/fig_r13_map.png`, `out/fig_r13_flicker.png`."]
    open(os.path.join(OUT, "r13_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    fig(Q)
    fig_flicker()
    print("wrote out/r13_map.{log,csv}, out/r13_points.csv, out/r13_points_med.csv, out/r13_verdict.md, out/fig_r13_map.png, "
          "out/fig_r13_flicker.png")


def fig(Q):
    """three V2 x |P| maps: (a) SPS - M-S (Q2/Q3), (b) M-S - best candidate (Q1), (c) M-B - M-S (Q4); hatched = no value"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, Normalize
    from matplotlib.patches import Rectangle
    from evaluate import INK, INK2, MUTED, _style
    _style(plt)
    div = LinearSegmentedColormap.from_list("red_gray_blue", ["#e34948", "#f2a7a6", "#f0efec", "#86b6ef", "#1c5cab"])
    seq = LinearSegmentedColormap.from_list("orange", ["#fbe3d6", "#f2a37e", "#eb6834", "#a8441d"])
    nv, npw = len(V2S), len(PS)

    def grid(f):
        A = np.full((nv, npw), np.nan)
        for _, q in Q.iterrows():
            A[V2S.index(q["V2"]), PS.index(q["P"])] = f(q)
        return A
    a = grid(lambda q: -q["d_MS"] if q["q2_MS"] != "undecidable" else np.nan)
    b = grid(lambda q: q["gap"] if q["q1"] != "undecidable" else np.nan)
    c = grid(lambda q: q["d4"] if q["q4"] not in ("n/a", "undecidable") else np.nan)
    same = grid(lambda q: 1.0 if q["q4"] == "n/a" else np.nan)
    L = lambda A: max(0.05, float(np.nanmax(np.abs(A)))) if np.isfinite(A).any() else 0.05  # noqa: E731
    fig_, ax = plt.subplots(1, 3, figsize=(15.5, 4.6))
    panels = ((a, div, Normalize(-L(a), L(a)), "(a) Ipk(SPS) − Ipk(M-S) [A]\n+ = lower peak than SPS; × = worse than SPS beyond tol"),
              (b, seq, Normalize(0, L(b)), "(b) Ipk(M-S) − best candidate [A] (Q1)\nlabel = best candidate where beyond tol"),
              (c, div, Normalize(-L(c), L(c)), "(c) Ipk(M-B) − Ipk(M-S) [A] (Q4)\n+ = M-S lower; '=' = M-S uses M-B's duties"))
    for k, (A, cm, nm, title) in enumerate(panels):
        x = ax[k]
        im = x.imshow(np.ma.masked_invalid(A), cmap=cm, norm=nm, origin="lower", aspect="auto")
        for i in range(nv):
            for j in range(npw):
                if not np.isfinite(A[i, j]) and not (k == 2 and np.isfinite(same[i, j])):
                    x.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, hatch="///", edgecolor=MUTED, lw=0))
        for _, q in Q.iterrows():
            i, j = V2S.index(q["V2"]), PS.index(q["P"])
            if k == 0 and q["q2_MS"] == "worse":
                x.text(j, i, "×", ha="center", va="center", color=INK, fontsize=11)
            if k == 1 and q["q1"] == "not optimal":
                x.text(j, i, q["best"], ha="center", va="center", color=INK, fontsize=7)
            if k == 2 and q["q4"] == "n/a":
                x.text(j, i, "=", ha="center", va="center", color=MUTED, fontsize=9)
        x.set_xticks(range(npw), [f"{p:g}" for p in PS])
        x.set_yticks(range(nv), [f"{v:g}" for v in V2S])
        x.set_xlabel("|P| [W]")
        x.grid(False)
        x.set_title(title, fontsize=9, color=INK2)
        fig_.colorbar(im, ax=x, fraction=0.046, pad=0.03)
    ax[0].set_ylabel("V2 [V]")
    fig_.suptitle("R13 stage 1: Simulink steady state on the 25 ns grid (constant D0, two levels interpolated in |P|)",
                  fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_r13_map.png"), dpi=130)
    plt.close(fig_)


def q5_rows(d25):
    """set X: each pair's 100 ns statement (if any) classified at 25 ns exactly as GRID R2"""
    d100 = fg.results_100()
    rows = []
    for v, p, cs in candidates("x"):
        by = {c["name"]: c for c in cs}
        v25, v100 = {}, {}
        for c in cs:
            own = by[c["alias"]] if c["alias"] else c
            cfg = dict(set="X", pair=0, ctrl=own["name"], V2=v, P=p, D1=own["D1"], D2=own["D2"])
            v25[c["name"]] = fg.at_power(_runs(d25, cfg), p)[:2]
            v100[c["name"]] = fg.at_power(fg.runs_of(d100, cfg), p)[:2]
        s1, s2 = v100["SPS"], v25["SPS"]
        tol100 = max(TOL_ABS, TOL_REL * s1[0]) if np.isfinite(s1[0]) else TOL_ABS
        tol25 = max(TOL_ABS, TOL_REL * s2[0]) if clean(s2) else tol100
        for lo, hi in X_PAIRS:
            if lo not in by or hi not in by:
                continue
            a1, b1, a2, b2 = v100[lo], v100[hi], v25[lo], v25[hi]
            d100_, d25_ = b1[0] - a1[0], b2[0] - a2[0]
            res = fg.rule_r2(d100_, d25_, tol100, tol25, clean(a1) and clean(b1), clean(a2) and clean(b2))
            sg = np.sign(d100_) if np.isfinite(d100_) and d100_ != 0 else 1.0
            rows.append(dict(V2=v, P=p, pair=f"{lo} vs {hi}", res=res, tol25=tol25,
                             statement=(f"{lo} lower than {hi}" if sg > 0 else f"{hi} lower than {lo}"),
                             d100=abs(d100_) if res else d100_, d25=d25_ * sg if res else d25_,
                             missing=not (np.isfinite(a2[0]) and np.isfinite(b2[0]))))
    return rows


# ----------------------------------------------------------------------------------------------------- not a verdict
PP_N = 80                                   # periods in the last 1 ms
PERIODIC = 0.005                            # A: peak spread over the 80 periods at or below this = periodic
CELL = 380.0 * TS / 90e-6                   # A: V1 Ts / L, i_L moved by one grid step at the full V1


def per_period(d, pats=PATS):
    """the 25 ns runs d + from out/iL_pp/ (verify_steady_fine.m): pk_med, pk_min, spread = largest - smallest period peak,
    kind = periodic / cell (a rare spike of up to about 1.5 grid steps over an otherwise constant peak) / slow (the rest)"""
    import pandas as pd
    fr = []
    for pat in pats:
        for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", pat))):
            T = pd.read_csv(f)
            pk = np.maximum(T[[f"max{i}" for i in range(1, PP_N + 1)]].to_numpy(float),
                            -T[[f"min{i}" for i in range(1, PP_N + 1)]].to_numpy(float))
            fr.append(pd.DataFrame(dict(Ts=T["Ts"], V2=T["V2"], level=T["level"], D1=T["D1"].round(9), D2=T["D2"].round(9),
                                        pk_max=np.nanmax(pk, axis=1), pk_med=np.nanmedian(pk, axis=1),
                                        pk_min=np.nanmin(pk, axis=1))))
    P = pd.concat(fr, ignore_index=True)
    P = P[np.isclose(P["Ts"], TS, rtol=1e-6, atol=0)].drop(columns="Ts")
    P = P.drop_duplicates(subset=["V2", "level", "D1", "D2"], keep="last")
    J = d.assign(D1r=d["D1"].round(9), D2r=d["D2"].round(9)).merge(
        P.rename(columns=dict(D1="D1r", D2="D2r")), on=["V2", "level", "D1r", "D2r"], how="left")
    J["spread"] = J["pk_max"] - J["pk_min"]
    J["kind"] = np.where(J["spread"] <= PERIODIC, "periodic",
                         np.where((J["spread"] <= 1.5 * CELL) & (J["pk_med"] - J["pk_min"] <= 2 * PERIODIC), "cell", "slow"))
    J["loss"] = J["P_LV"] - J["P_HV"]
    return J


def _at_col(g, p, col):
    """the rule's interpolation (fg.at_power) applied to another column of the two bracketing runs"""
    if g.empty:
        return np.nan, np.nan
    r = fg.at_power(g.assign(**{f"Ipk_at_{WIN}ms": g[col]}), p)
    if r[2] is None:
        return np.nan, np.nan
    return r[0], float(g[g["level"].isin([r[2], r[2] + 1])]["spread"].max())


def _pp_check(J, pts, o):
    """Q1-Q4 again on the median period peak + the run classes + loss / efficiency -> per-point table"""
    import pandas as pd
    rows = []
    for v, p, cs in pts:
        by = {c["name"]: c for c in cs}
        med, rec = {}, dict(V2=v, P=p)
        for c in cs:
            own = by[c["alias"]] if c["alias"] else c
            g = fg.runs_of(J, dict(V2=v, D1=own["D1"], D2=own["D2"]), TS)
            osc = fg.at_power(g, p)[1] if not g.empty else np.nan
            m, s = _at_col(g, p, "pk_med")
            med[c["name"]] = (m, osc)
            if c["name"] in CORE:
                k = c["name"].replace("-", "")
                rec.update({f"med_{k}": m, f"spr_{k}": s, f"loss_{k}": _at_col(g, p, "loss")[0],
                            f"max_{k}": _at_col(g, p, f"Ipk_at_{WIN}ms")[0], f"osc_{k}": osc})
        tol = tol_at(med)
        rec["tol_med"] = tol
        rec["q1_med"], rec["gap_med"], rec["best_med"] = rule_q1(med, tol)
        rec["q2_MS_med"], rec["d_MS_med"] = rule_q2(med.get("M-S"), med.get("SPS"), tol)
        rec["q2_MB_med"], rec["d_MB_med"] = rule_q2(med.get("M-B"), med.get("SPS"), tol)
        if by["M-S"]["alias"] is None:
            rec["q4_med"], rec["d4_med"] = rule_q4(med.get("M-S"), med.get("M-B"), tol)
        else:
            rec["q4_med"], rec["d4_med"] = "n/a", np.nan
        rows.append(rec)
    M = pd.DataFrame(rows)
    M.to_csv(os.path.join(OUT, "r13_points_med.csv"), index=False, float_format="%.6g")
    o("\nNOT A VERDICT (the checks of the 10-30 18:15 note in the header, chosen after seeing r13a):")
    kinds = J["kind"].value_counts()
    o(f"  runs with per-period peaks: {int(J['pk_max'].notna().sum())}: periodic (spread <= {1e3 * PERIODIC:.0f} mA) "
      f"{int(kinds.get('periodic', 0))}, single-period spikes (<= 1.5 V1 Ts / L = {1.5 * CELL:.3f} A) "
      f"{int(kinds.get('cell', 0))}, slow / larger {int(kinds.get('slow', 0))}")
    t = J.assign(group=np.where(J["ctrl"].isin(CORE), J["ctrl"], "other")).groupby("group")["kind"]
    o("  not periodic per controller: " + ", ".join(f"{k} {int((g != 'periodic').sum())}/{len(g)}" for k, g in t))
    t = J.groupby("V2")["kind"]
    o("  not periodic per V2: " + ", ".join(f"{k:g} V {int((g != 'periodic').sum())}/{len(g)}" for k, g in t))
    res = dict()
    q1m = list(M["q1_med"])
    res["q1"] = (q1m.count("optimal"), q1m.count("not optimal"), sum(c != "undecidable" for c in q1m))
    o(f"  Q1 on the median: {res['q1'][2]}/{len(M)} decidable, optimal {res['q1'][0]}, not optimal {res['q1'][1]}")
    for _, q in M[M["q1_med"] == "not optimal"].iterrows():
        o(f"     {q['V2']:6.1f} V {q['P']:5.0f} W: best {q['best_med']}, {q['gap_med']:.3f} A below M-S")
    for k, nm in (("MS", "M-S"), ("MB", "M-B")):
        cc = list(M[f"q2_{k}_med"])
        o(f"  Q2 on the median, {nm}: worse {cc.count('worse')}, same {cc.count('same')}, better {cc.count('better')}, "
          f"undecidable {cc.count('undecidable')}")
        for _, q in M[M[f"q2_{k}_med"] == "worse"].iterrows():
            o(f"     worse: {q['V2']:6.1f} V {q['P']:5.0f} W +{q[f'd_{k}_med']:.3f} A (tol {q['tol_med']:.3f}), "
              f"bracket spread {nm} {q[f'spr_{k}']:.3f} A, SPS {q['spr_SPS']:.3f} A")
        m = M[M[f"q2_{k}_med"] != "undecidable"]
        if len(m):
            dI, pc = m["med_SPS"] - m[f"med_{k}"], 100 * (m["med_SPS"] - m[f"med_{k}"]) / m["med_SPS"]
            res[f"q3_{k}"] = (len(m), dI.mean(), dI.median(), pc.mean())
            o(f"  Q3 on the median, {nm}: {len(m)} points, mean {dI.mean():+.3f} A (median {dI.median():+.3f}), {pc.mean():+.1f} % of SPS; per V2 "
              + ", ".join(f"{v:g} V {(m[m['V2'] == v]['med_SPS'] - m[m['V2'] == v][f'med_{k}']).mean():+.3f}" for v in V2S
                          if (m["V2"] == v).any()))
    q4m = M[M["q4_med"] != "n/a"]
    res["q4"] = (int((q4m["q4_med"] == "better").sum()), int((q4m["q4_med"] == "worse").sum()), float(q4m["d4_med"].mean()))
    o(f"  Q4 on the median, M-S vs M-B ({len(q4m)} points): better {res['q4'][0]}, worse {res['q4'][1]}, mean {res['q4'][2]:+.3f} A")
    for k, nm in (("MB", "M-B"), ("MS", "M-S")):
        m = M[(M[f"osc_{k}"] <= OSC) & (M["osc_SPS"] <= OSC) & M[f"loss_{k}"].notna() & M["loss_SPS"].notna()]
        if not len(m):
            continue
        dl = m[f"loss_{k}"] - m["loss_SPS"]
        eta = lambda L: 100 * m["P"] / (m["P"] + L)  # noqa: E731
        de = eta(m[f"loss_{k}"]) - eta(m["loss_SPS"])
        res[f"eff_{k}"] = (len(m), dl.mean(), de.mean(), int((de < -0.05).sum()))
        o(f"  loss / efficiency at |P| (P_HV in - P_LV out, same interpolation), {nm} vs SPS, {len(m)} clean points: loss {dl.mean():+.2f} W, "
          f"eta {de.mean():+.3f} pp, lower eta beyond 0.05 pp at {int((de < -0.05).sum())}; per V2 (pp) "
          + ", ".join(f"{v:g} V {de[m['V2'] == v].mean():+.2f}" for v in V2S if (m["V2"] == v).any()))
        for _, q in m[de < -0.05].iterrows():
            o(f"     lower eta: {q['V2']:6.1f} V {q['P']:5.0f} W loss {q[f'loss_{k}']:.2f} vs {q['loss_SPS']:.2f} W")
    return M, res


FLICKER = (("periodic", "M-S", 105.0, 6, 0.4612, 0.4211), ("single-period spikes", "M-S", 109.2, 7, 0.3436, 0.2883),
           ("slow offset cycle", "M-B", 105.0, 6, 0.4590, 0.4162), ("slow offset cycle", "M-B", 92.4, 2, 0.4865, 0.5000))


def fig_flicker():
    """small multiples: the peak of each of the last 80 periods of four 25 ns runs, same y span, one grid step marked"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    from evaluate import BLUE, INK, INK2, MUTED, YELLOW, _style
    _style(plt)
    fs = [f for pat in ("fine_sweep_r*.csv", "r13a_sweep_r*.csv") for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", pat)))]
    A = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    A = A[np.isclose(A["Ts"], TS, rtol=1e-6, atol=0)]
    fig_, ax = plt.subplots(1, 4, figsize=(13.5, 3.3), sharex=True)
    span = 0.60
    for a, (kind, ctrl, v, j, d1, d2) in zip(ax, FLICKER):
        r = A[np.isclose(A["V2"], v) & (A["level"] == j) & (np.abs(A["D1"] - d1) < 2e-4) & (np.abs(A["D2"] - d2) < 2e-4)].iloc[-1]
        pk = np.maximum(r[[f"max{i}" for i in range(1, PP_N + 1)]].to_numpy(float),
                        -r[[f"min{i}" for i in range(1, PP_N + 1)]].to_numpy(float))
        a.plot(np.arange(1, PP_N + 1), pk, color={"M-B": BLUE, "M-S": YELLOW}.get(ctrl, MUTED), lw=1.6)
        lo = 0.5 * (pk.max() + pk.min()) - 0.5 * span
        a.set_ylim(lo, lo + span)
        a.plot([6, 6], [lo + 0.04, lo + 0.04 + CELL], color=INK, lw=2.5, solid_capstyle="butt")
        a.text(8, lo + 0.04 + 0.5 * CELL, f"V1 Ts/L = {CELL:.3f} A", va="center", color=INK2, fontsize=8)
        a.set_title(f"{kind}\n{ctrl} ({d1:.3f}, {d2:.3f}), {v:g} V, level {j}, {-r['P_LV'] if 'P_LV' in r else 0:.0f} W".replace(", 0 W", ""),
                    fontsize=9, color=INK2)
        a.set_xlabel("period within the last 1 ms")
    ax[0].set_ylabel("peak |i_L| of the period [A]")
    fig_.suptitle("Period peaks of 25 ns Simulink runs (constant D0). Each panel spans 0.6 A; the rule's Ipk is the top of the trace",
                  fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_r13_flicker.png"), dpi=130)
    plt.close(fig_)


def show():
    """the candidate set and configuration count without the plant (quick look)"""
    for which in ("x", "map"):
        n = 0
        for v, p, cs in candidates(which):
            n += sum(c["alias"] is None for c in cs)
            print(f"{which:>3} {v:6.1f} V {p:5.0f} W: " + ", ".join(f"{c['name']}({c['D1']:.3f},{c['D2']:.3f})"
                                                              + (f"={c['alias']}" if c["alias"] else "") for c in cs))
        print(f"{which}: {n} candidate configurations with their own runs")
    print(f"run_configs: {len(run_configs())} configurations in run order (set X first, then map core, then arms)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["predict", "make", "next", "compare", "selftest", "show"])
    ap.add_argument("--part", choices=["a", "b"], default="a", help="make / next: a = set X + map core, b = map arms")
    a = ap.parse_args()
    if a.cmd in ("make", "next"):
        {"make": make, "next": next_round}[a.cmd](a.part)
    else:
        {"predict": predict, "compare": compare, "selftest": selftest, "show": show}[a.cmd]()


if __name__ == "__main__":
    main()
