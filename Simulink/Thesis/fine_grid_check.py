"""
fine_grid_check.py -- (2026-10-30) do the 0.1 A-scale Simulink differences survive a finer PWM / solver grid?

Background (duty_quant_check.py, Simulink data): the dataset-era model steps the controller and the Simscape local
solver at 100 ns (125 steps per 12.5 us). Ipk is exactly constant inside duty cells of 1/250 and jumps between cells;
at the same power, duties 1-2 cells apart differ by 0.086 A (median, 104 pairs) where the ideal model says 0.0006 A.
That is the size of the OPT tolerances (0.03-0.1 A), of the M-S gains and of "M-B is 0.075 / 0.172 A above SPS at
84 V 400 / 800 W". This check reruns chosen configurations at Ts = 25 ns (and the roughest pairs also at 10 ns) with
verify_steady_fine.m and compares with the 100 ns runs at the SAME 20 ms window (100 ns runs: Ipk_at_20ms /
P_LV_at_20ms; fine runs are 20 ms long), each configuration interpolated at |P| between two adjacent 2.88-deg D0
levels as in OPT. The D0 levels stay the 100 ns ones; on the 25 / 10 ns grids they sit exactly on a grid point, so
every edge phase that would be a knife edge is moved 1e-4 deg earlier (as D_SAFE does for D = 0.5).

Sets (out/fine_points_r1.csv; the order is the run order):
  S  duty line scans (neutral choice -- NOT the pairs with the largest 100 ns differences, which would shrink on any
     re-measurement by regression to the mean): at 84 V 150 / 400 W (scan D1) and 109.2 V 300 / 600 W (scan D2) the
     M-B duty + {-0.004, -0.002, 0, +0.002, +0.004} on the smaller duty, the other duty fixed, at BOTH 100 ns (the R11
     controller copy at 100 ns must reproduce the original model's 20 ms values: a check of the copy) and 25 ns.
  B  the points the conclusions hang on: 84 V 150 W (M-S 0.161 A below M-B with duties 0.004 apart), 84 V 400 / 800 W
     (M-B above SPS), 109.2 V 100 / 600 W (largest M-S gains) x {SPS, M-B, M-S}. 25 ns.
  C  the R.4 headline at 84 / 109.2 V x 100 / 300 / 1000 W x {SPS, M-B}. 25 ns.

RULES (registered 2026-10-30 before any fine-grid run exists; do not change after the results):
  R1 roughness, set S: per scan and grid, rough = RMS residual of a least-squares quadratic Ipk(d) through the 5 duties
     (all 5 bracketed and clean, |mean i_L| <= 20 mA; else the scan is not decidable on that grid).
     r = median rough(25 ns) / median rough(100 ns) over the scans decidable on both grids.
     r <= 1/3 -> "grid artifact" (the cell-scale roughness of the 100 ns model is mostly the 100 ns grid);
     r >= 2/3 -> "persists" (it is in the plant model, not the grid); else "partly".
     Fewer than 3 decidable scans, or median rough(100 ns) < 0.01 A (nothing to explain) -> "undecidable".
  R2 each set-B / set-C statement "X is lower than Y at (V2, |P|)" (M-S vs M-B, M-B vs SPS, M-S vs SPS) is "kept" at
     25 ns if the 25 ns difference has the same sign and exceeds tol = max(0.03 A, 2 % Ipk(SPS at 25 ns)); "gone" if
     |difference| <= tol; "reversed" if it exceeds tol with the other sign; "undecidable" if a value is missing or
     oscillating. Statements are taken from the 100 ns runs at the same 20 ms window (|difference| > tol there).
  R3 headline: mean Ipk(SPS) - Ipk(M-B) over the set-C points clean at both grids, at 100 ns and at 25 ns (20 ms
     window) -- reported as the two numbers and their difference, no threshold (the 30 ms 100 ns value is R.4's).

    python -I -X utf8 Simulink/Thesis/fine_grid_check.py make      -> out/fine_points_r1.csv
    python automation/job.py submit fine1 --interactive --after r11smoke -- matlab -batch "cd('Simulink/Thesis'); verify_steady_fine_batch('out/fine_points_r1.csv', 'out/fine_sweep_r1.csv', 12, 0.020)"
    python -I -X utf8 Simulink/Thesis/fine_grid_check.py next      (configurations not bracketed -> round k+1)
    python -I -X utf8 Simulink/Thesis/fine_grid_check.py compare   -> out/fine_grid_check.{log,csv}, out/fine_grid_verdict.md
    python -I -X utf8 Simulink/Thesis/fine_grid_check.py selftest  (the rules on made-up numbers)
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
G = sw.G
TS_COARSE = 1e-7
TS_FINE = 25e-9
WIN = 20                                   # ms: the comparison window (fine runs are 20 ms long)
OSC = 0.02
TOL_ABS, TOL_REL = 0.03, 0.02
NUDGE = 1e-4                               # deg: knife-edge phases moved this much earlier
S_PTS = ((84.0, 150.0, 1), (84.0, 400.0, 1), (109.2, 300.0, 2), (109.2, 600.0, 2))   # (V2, |P|, scanned duty)
S_OFF = (-0.004, -0.002, 0.0, 0.002, 0.004)
ROUGH_MIN = 0.01
B_PTS = ((84.0, 150.0), (84.0, 400.0), (84.0, 800.0), (109.2, 100.0), (109.2, 600.0))
C_PTS = tuple((v, p) for v in (84.0, 109.2) for p in (100.0, 300.0, 1000.0))
FILES_100 = ("simulink_sweep", "eff_sweep", "zvs_sweep", "k1map_sweep", "opt_sweep")


# ----------------------------------------------------------------------------------------------------- data
def _load(paths, fine=False):
    """fine runs keep their points file's level (a finer grid commands D0 = j G, see _point); earlier runs: from D0"""
    import pandas as pd
    fr = []
    for f in paths:
        T = pd.read_csv(f)
        if "error" in T.columns:
            T = T[T["error"].isna() | (T["error"].astype(str).str.strip() == "")]
        fr.append(T.assign(src=os.path.basename(f)))
    if not fr:
        return None
    d = pd.concat(fr, ignore_index=True)
    d["Pd"] = -d["P_LV"]
    if not fine:
        d["level"] = np.round((d["D0"] + 0.5 * G) / G).astype(int)
    d["level"] = d["level"].astype(int)
    if "Ts" not in d.columns:
        d["Ts"] = TS_COARSE
    d["Ts"] = d["Ts"].fillna(TS_COARSE)
    return d


def results_100():
    fs = [f for fam in FILES_100 for f in sorted(glob.glob(os.path.join(OUT, f"{fam}_*.csv")))
          if not f.endswith("_in.csv")]
    d = _load(fs)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="first")


def results_fine():
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "fine_sweep_r*.csv"))) if not f.endswith("_in.csv")]
    d = _load(fs, fine=True)
    return None if d is None else d.drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def runs_of(d, c, Ts=None):
    m = (np.isclose(d["V2"], c["V2"]) & (np.abs(d["D1"] - c["D1"]) <= 1e-6) & (np.abs(d["D2"] - c["D2"]) <= 1e-6)
         & (d["D1"] < 0.5) & (d["D2"] < 0.5))
    if Ts is not None:
        m &= np.isclose(d["Ts"], Ts, rtol=1e-6, atol=0)
    return d[m].sort_values("level")


def at_power(g, P, win=WIN):
    """Ipk at |P| from two adjacent levels bracketing P in the window's power; (Ipk, osc, lo level) or NaNs"""
    pc, ic = (f"P_LV_at_{win}ms", f"Ipk_at_{win}ms") if win else ("P_LV", "Ipk_sim")
    if g is None or g.empty or pc not in g or ic not in g:
        return np.nan, np.nan, None
    g = g.dropna(subset=[pc, ic])
    Pd, lv, I, osc = -g[pc].to_numpy(), g["level"].to_numpy(), g[ic].to_numpy(), g["iL_mean"].abs().to_numpy()
    for i in range(len(g) - 1):
        if lv[i + 1] == lv[i] + 1 and min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]) and Pd[i] != Pd[i + 1]:
            t = (P - Pd[i]) / (Pd[i + 1] - Pd[i])
            return I[i] + t * (I[i + 1] - I[i]), max(osc[i], osc[i + 1]), int(lv[i])
    return np.nan, np.nan, None


# ----------------------------------------------------------------------------------------------------- configs
def _cfg(st, pair, ctrl, V2, P, D1, D2):
    return dict(set=st, pair=pair, ctrl=ctrl, V2=float(V2), P=float(P), D1=round(float(D1), 9), D2=round(float(D2), 9))


def configs():
    import surface_opt_check as so
    ms, cf = so._ms(), []
    pred = lambda m, P, V: tuple(so._d(np.ravel(x)[0]) for x in m.predict(np.array([P]), np.array([V])))  # noqa: E731
    d = results_100()
    for i, (v, p, k) in enumerate(S_PTS):                    # S: scans around M-B at the middle of its 100 ns bracket
        b1, b2 = so.mb_predict(p, v)
        g = runs_of(d, _cfg("S", i, "c", v, p, b1, b2))
        br = sw.bracket(g, p)[0] if not g.empty else None
        Ps = round(0.5 * (br[0]["Pd"] + br[1]["Pd"]), 1) if br is not None else p
        for j, off in enumerate(S_OFF):
            x1, x2 = (b1 + off, b2) if k == 1 else (b1, b2 + off)
            cf.append(_cfg("S", i, f"s{j}", v, Ps, min(x1, sw.D_SAFE), min(x2, sw.D_SAFE)))
    for v, p in B_PTS:
        cf += [_cfg("B", 0, "SPS", v, p, sw.D_SAFE, sw.D_SAFE), _cfg("B", 0, "M-B", v, p, *so.mb_predict(p, v)),
               _cfg("B", 0, "M-S", v, p, *pred(ms, p, v))]
    for v, p in C_PTS:
        cf += [_cfg("C", 0, "SPS", v, p, sw.D_SAFE, sw.D_SAFE), _cfg("C", 0, "M-B", v, p, *so.mb_predict(p, v))]
    return cf


def _safe(D0, D1, D2, Ts):
    """move knife-edge phases (exactly on the Ts grid) NUDGE deg earlier: D0 first, then the duty of p3 / p7"""
    step = 360.0 * Ts * 80e3
    on = lambda p: abs(p / step - round(p / step)) < 1e-6  # noqa: E731
    if on(D0 % 360.0):
        D0 -= NUDGE
    if on(360.0 * D1):
        D1 -= NUDGE / 360.0
    if on((D0 + 360.0 * D2) % 360.0):
        D2 -= NUDGE / 360.0
    return round(D0, 6), round(D1, 9), round(D2, 9)


def _point(c, j, Ts):
    """level j: at 100 ns D0 = j G - G/2 exactly as every earlier run (the secondary edges then fire on the next grid
    point, i.e. at an effective D0 = j G); on a finer grid D0 = j G (minus NUDGE), the same effective phase"""
    if np.isclose(Ts, TS_COARSE):
        D0, D1, D2 = round(j * G - 0.5 * G, 6), c["D1"], c["D2"]
    else:
        D0, D1, D2 = _safe(j * G, c["D1"], c["D2"], Ts)
    return dict(label=f"{c['set']}{c['pair']} {c['ctrl']} {c['P']:g}W {c['V2']:g}V L{j} T{Ts * 1e9:.0f}", ctrl=c["ctrl"],
                V2=c["V2"], P_target=c["P"], level=j, D0=D0, D1=D1, D2=D2, Ts=Ts)


def ts_list(c):
    return (TS_COARSE, TS_FINE) if c["set"] == "S" else (TS_FINE,)


def levels_100(d, cfs):
    """the two adjacent levels that bracket |P| at 100 ns (end window), else the plant's prediction"""
    out, fresh = {}, []
    for i, c in enumerate(cfs):
        g = runs_of(d, c)
        b = sw.bracket(g, c["P"])[0] if not g.empty else None
        if b is not None:
            out[i] = (int(b[0]["level"]), int(b[1]["level"]))
        else:
            fresh.append(i)
    if fresh:
        for i, lv in zip(fresh, sw.predicted_levels([cfs[i] for i in fresh])):
            out[i] = lv
    return out


def _write(path, pts):
    import csv
    seen, uniq = set(), []
    for p in pts:
        k = (p["Ts"], p["V2"], p["level"], p["D1"], p["D2"])
        if k not in seen:
            seen.add(k)
            uniq.append(p)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2", "Ts"])
        w.writeheader()
        w.writerows(uniq)
    print(f"wrote {path}: {len(uniq)} runs ({len(pts)} before removing duplicates)")


def make():
    d = results_100()
    cfs = configs()
    lv = levels_100(d, cfs)
    for i, c in enumerate(cfs):                              # a scan runs every duty at its centre's two levels
        if c["set"] == "S":
            lv[i] = next(lv[k] for k, e in enumerate(cfs) if e["set"] == "S" and e["pair"] == c["pair"] and e["ctrl"] == "s2")
    order = {"S": 0, "B": 1, "C": 2}
    pts = []
    for i, c in enumerate(cfs):
        for Ts in ts_list(c):
            pts += [dict(_point(c, j, Ts), _rank=order[c["set"]]) for j in lv[i]]
    pts = [{k: v for k, v in p.items() if k != "_rank"} for p in sorted(pts, key=lambda p: p["_rank"])]
    for c in cfs:
        print(f"  {c['set']}{c['pair']} {c['ctrl']:>4} {c['V2']:6g} V {c['P']:7.1f} W ({c['D1']:.5f}, {c['D2']:.5f})")
    _write(os.path.join(OUT, "fine_points_r1.csv"), pts)


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "fine_sweep_r*.csv"))
          for m in [re.search(r"fine_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    df = results_fine()
    if df is None:
        sys.exit("no fine runs yet")
    cfs, pts = configs(), []
    for c in cfs:
        for Ts in ts_list(c):
            g = runs_of(df, c, Ts)
            if g.empty or np.isfinite(at_power(g, c["P"])[0]):
                continue
            Pd = -g[f"P_LV_at_{WIN}ms"].to_numpy()
            j = int(g["level"].max() + 1) if np.nanmax(Pd) < c["P"] else int(g["level"].min() - 1)
            pts += [_point(c, j, Ts)]
    if pts:
        _write(os.path.join(OUT, f"fine_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def _clean(i, o):
    return bool(np.isfinite(i) and np.isfinite(o) and o <= OSC)


def rough(off, I):
    """RMS residual of a least-squares quadratic through (offset, Ipk); NaN unless all values are there"""
    off, I = np.asarray(off, float), np.asarray(I, float)
    if len(I) < 4 or not np.all(np.isfinite(I)):
        return np.nan
    c = np.polyfit(off, I, 2)
    return float(np.sqrt(np.mean((I - np.polyval(c, off)) ** 2)))


def rule_r1(scans):
    """scans: list of dict(r100, r25) (NaN = not decidable on that grid) -> (code, r, n)"""
    ok = [x for x in scans if np.isfinite(x["r100"]) and np.isfinite(x["r25"])]
    if len(ok) < 3:
        return "undecidable", np.nan, len(ok)
    m100, m25 = np.median([x["r100"] for x in ok]), np.median([x["r25"] for x in ok])
    if m100 < ROUGH_MIN:
        return "undecidable", np.nan, len(ok)
    r = m25 / m100
    return ("grid artifact" if r <= 1 / 3 else "persists" if r >= 2 / 3 else "partly"), float(r), len(ok)


def rule_r2(d100, d25, tol100, tol25, clean100, clean25):
    """a statement 'X lower than Y' (d = Ipk(Y) - Ipk(X) at 100 ns > tol) -> kept / gone / reversed / undecidable;
    None if there is no statement at 100 ns"""
    if not clean100 or not np.isfinite(d100) or abs(d100) <= tol100:
        return None
    if not clean25 or not np.isfinite(d25):
        return "undecidable"
    if abs(d25) <= tol25:
        return "gone"
    return "kept" if np.sign(d25) == np.sign(d100) else "reversed"


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    X = lambda a, b: dict(r100=a, r25=b)  # noqa: E731
    chk("R1 artifact", rule_r1([X(0.06, 0.01), X(0.05, 0.02), X(0.08, 0.0)])[0], "grid artifact")
    chk("R1 persists", rule_r1([X(0.06, 0.05), X(0.05, 0.06), X(0.08, 0.07)])[0], "persists")
    chk("R1 partly", rule_r1([X(0.06, 0.03), X(0.05, 0.02), X(0.08, 0.03)])[0], "partly")
    chk("R1 too few", rule_r1([X(0.06, 0.03), X(0.05, np.nan), X(np.nan, 0.03)])[0], "undecidable")
    chk("R1 nothing to explain", rule_r1([X(0.006, 0.001), X(0.005, 0.0), X(0.008, 0.003)])[0], "undecidable")
    chk("rough: parabola -> 0", round(rough(S_OFF, [1 + 50 * x * x for x in S_OFF]), 12), 0.0)
    chk("rough: missing -> NaN", bool(np.isnan(rough(S_OFF, [1, 1, np.nan, 1, 1]))), True)
    chk("rough: staircase > 0", rough(S_OFF, [1.0, 1.0, 1.1, 1.1, 1.2]) > 0.01, True)
    chk("R2 kept", rule_r2(0.17, 0.12, 0.074, 0.074, True, True), "kept")
    chk("R2 gone", rule_r2(0.17, 0.05, 0.074, 0.074, True, True), "gone")
    chk("R2 reversed", rule_r2(0.17, -0.10, 0.074, 0.074, True, True), "reversed")
    chk("R2 no statement", rule_r2(0.05, 0.3, 0.074, 0.074, True, True), None)
    chk("R2 not clean at 25 ns", rule_r2(0.17, 0.12, 0.074, 0.074, True, False), "undecidable")
    chk("R2 osc at 100 ns -> no statement", rule_r2(0.17, 0.12, 0.074, 0.074, False, True), None)
    cS = dict(set="S", pair=0, ctrl="s0", V2=84.0, P=1.0, D1=0.3, D2=0.3)
    chk("point: 100 ns keeps D0 = jG - G/2", _point(cS, -1, TS_COARSE)["D0"], -4.32)
    chk("point: 25 ns D0 = jG - nudge", _point(cS, -1, TS_FINE)["D0"], -2.8801)
    chk("knife: D1 0.45 at 25 ns moved", _safe(-2.88, 0.45, 0.3, 25e-9)[1], round(0.45 - NUDGE / 360, 9))
    chk("knife: D1 0.4513 at 25 ns kept", _safe(-2.88, 0.4513, 0.3, 25e-9)[1], 0.4513)
    chk("knife: D2 kept when D0 nudged", _safe(-2.88, 0.3, 0.45, 25e-9)[2], 0.45)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def compare():
    import datetime

    import pandas as pd
    d1, df = results_100(), results_fine()
    if df is None:
        sys.exit("no fine runs yet")
    rows = []
    for c in configs():
        rec = dict(c)
        g1 = runs_of(d1, c)
        rec["I100"], rec["o100"], _ = at_power(g1, c["P"])
        rec["I100end"], rec["o100end"], _ = at_power(g1, c["P"], win=None)
        for Ts, k in ((TS_FINE, "25"), (TS_COARSE, "100c")):  # 100c = the R11 controller copy at 100 ns (scans)
            rec[f"I{k}"], rec[f"o{k}"], _ = at_power(runs_of(df, c, Ts), c["P"])
        rows.append(rec)
    T = pd.DataFrame(rows)
    T.to_csv(os.path.join(OUT, "fine_grid_check.csv"), index=False, float_format="%.6g")
    lines, md = [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    o(f"fine_grid_check: 100 ns vs 25 ns grid, Simulink steady state, {WIN} ms window, Ipk at |P| (A)")
    o(f"{'set':>4} {'ctrl':>4} {'V2':>6} {'|P|':>7} {'(D1, D2)':>18} | {'100ns':>7} {'(30ms)':>7} {'copy':>7} | {'25ns':>7}")
    fmt = lambda i, oo: (f"{i:7.3f}" + ("~" if np.isfinite(oo) and oo > OSC else " ")) if np.isfinite(i) else f"{'-':>7} "  # noqa: E731
    for _, r in T.iterrows():
        o(f"{r['set']}{r['pair']:<3} {r['ctrl']:>4} {r['V2']:6g} {r['P']:7.1f} ({r['D1']:.5f}, {r['D2']:.5f}) | "
          f"{fmt(r['I100'], r['o100'])}{fmt(r['I100end'], r['o100end'])}{fmt(r['I100c'], r['o100c'])}| {fmt(r['I25'], r['o25'])}")
    cp = T.dropna(subset=["I100", "I100c"])
    if len(cp):
        o(f"copy check (R11 controller at 100 ns vs the original model's 20 ms values, {len(cp)} configurations): "
          f"max |dIpk| {np.abs(cp['I100'] - cp['I100c']).max():.2e} A")
    o("(~ = oscillating, |mean i_L| > 20 mA)")
    # R1
    Sx, scans = T[T["set"] == "S"], []
    o("\nR1 duty line scans (set S): Ipk at the same power vs offset on the scanned duty; rough = RMS residual of a quadratic")
    for k, g in Sx.groupby("pair"):
        g = g.sort_values("ctrl")
        v, p, ax = S_PTS[int(k)]
        Ic = g["I100c"].where(g["o100c"] <= OSC)
        I100 = Ic if Ic.notna().all() else g["I100"].where(g["o100"] <= OSC)  # the copy's own runs, else the original
        I25 = g["I25"].where(g["o25"] <= OSC)
        x = dict(r100=rough(S_OFF, I100), r25=rough(S_OFF, I25))
        scans.append(x)
        o(f"   scan {k}: {v:g} V {g['P'].iloc[0]:.1f} W, D{ax} {g['D' + str(ax)].iloc[2]:.4f} + {S_OFF}")
        o("      100 ns " + " ".join(f"{i:7.3f}" for i in I100) + f"   rough {x['r100']:.4f}")
        o("       25 ns " + " ".join(f"{i:7.3f}" for i in I25) + f"   rough {x['r25']:.4f}")
    code1, r1, n1 = rule_r1(scans)
    o(f"=> R1: {code1} (median rough 25 ns / 100 ns = {r1:.2f}, {n1} scans decidable on both grids)")
    # R2
    o("\nR2 statements from the 100 ns runs (same 20 ms window) checked at 25 ns:")
    st = []
    for (v, p), g in T[T["set"].isin(["B", "C"])].groupby(["V2", "P"]):
        x = {r["ctrl"]: r for _, r in g.iterrows()}
        if "SPS" not in x:
            continue
        s = x["SPS"]
        tol100 = max(TOL_ABS, TOL_REL * s["I100"]) if np.isfinite(s["I100"]) else TOL_ABS
        tol25 = max(TOL_ABS, TOL_REL * s["I25"]) if _clean(s["I25"], s["o25"]) else tol100
        for lo, hi in (("M-B", "SPS"), ("M-S", "M-B"), ("M-S", "SPS")):
            if lo not in x or hi not in x:
                continue
            a, b = x[lo], x[hi]
            d100, d25 = b["I100"] - a["I100"], b["I25"] - a["I25"]
            res = rule_r2(d100, d25, tol100, tol25, _clean(a["I100"], a["o100"]) and _clean(b["I100"], b["o100"]),
                          _clean(a["I25"], a["o25"]) and _clean(b["I25"], b["o25"]))
            if res is None:
                continue
            w = f"{lo} lower than {hi}" if d100 > 0 else f"{hi} lower than {lo}"
            st.append(dict(V2=v, P=p, statement=w, d100=abs(d100), d25=d25 * np.sign(d100), res=res))
            o(f"   {v:6g} V {p:6.0f} W  {w:<20} by {abs(d100):.3f} A at 100 ns -> 25 ns {d25 * np.sign(d100):+.3f} A "
              f"(tol {tol25:.3f}): {res}")
    # R3
    C = T[T["set"] == "C"]
    h = []
    for (v, p), g in C.groupby(["V2", "P"]):
        x = {r["ctrl"]: r for _, r in g.iterrows()}
        if {"SPS", "M-B"} <= set(x) and all(_clean(x[k][f"I{t}"], x[k][f"o{t}"]) for k in ("SPS", "M-B") for t in ("100", "25")):
            h.append((x["SPS"]["I100"] - x["M-B"]["I100"], x["SPS"]["I25"] - x["M-B"]["I25"],
                      x["SPS"]["I100end"] - x["M-B"]["I100end"]))
    if h:
        h = np.array(h)
        o(f"\nR3 headline (set C, {len(h)} points clean at both grids): mean Ipk(SPS) - Ipk(M-B) = {h[:, 0].mean():+.3f} A "
          f"at 100 ns / {h[:, 1].mean():+.3f} A at 25 ns ({WIN} ms window); 100 ns end window {h[:, 2].mean():+.3f} A")
    else:
        o("\nR3 headline: no set-C point clean at both grids yet")
    nmiss = int(T[["I25"]].isna().sum().iloc[0])
    o(f"\nconfigurations without a 25 ns value yet: {nmiss} of {len(T)}")
    open(os.path.join(OUT, "fine_grid_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    md += ["# 격자 세분 점검: 100 ns 격자의 0.1 A 차가 25 ns(10 ns)에서도 남는가? (자동 생성)", "",
           f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `fine_grid_check.py compare`. 규칙은 이 파일 머리말(10-30, 세분 격자 실행 전"
           " 등록)이며 바꾸지 않았다. 모두 Simulink(데이터셋 시기 모델) 정상상태, 상수 D0, **20 ms 창**(100 ns 실행은 Ipk_at_20ms)으로 같게 비교했다.",
           "", f"- R1(듀티 선 스캔 거칠기): **{code1}** (25 ns / 100 ns 거칠기 중앙값 비 {r1:.2f}, 두 격자 모두 판정 가능한 스캔 {n1}개)"]
    md += [f"- R2 {s['V2']:g} V {s['P']:.0f} W: '{s['statement']}' {s['d100']:.3f} A(100 ns) → {s['d25']:+.3f} A(25 ns): **{s['res']}**"
           for s in st]
    if len(h):
        md.append(f"- R3(R.4 대표 {len(h)}점, 둘 다 깨끗): SPS − M-B 평균 {h[:, 0].mean():+.3f} A(100 ns) → {h[:, 1].mean():+.3f} A(25 ns)")
    if nmiss:
        md.append(f"\n**25 ns 값이 없는 구성 {nmiss}/{len(T)}개 — 잠정**(`fine_grid_check.py next`).")
    md += ["", "표 원본 `out/fine_grid_check.csv`, 로그 `out/fine_grid_check.log`."]
    open(os.path.join(OUT, "fine_grid_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/fine_grid_check.{log,csv}, out/fine_grid_verdict.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    {"make": make, "next": next_round, "compare": compare, "selftest": selftest}[a.cmd]()


if __name__ == "__main__":
    main()
