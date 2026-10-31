"""
g12_check.py -- (2026-10-31 00:23) why does every 10 ns run flicker, and what do the 25 ns statements do on a grid without that?

grid10_check.py compare (10-31 00:12) kept its rules, but its T3 shows something those rules did not expect: all 42
configurations are non-periodic at 10 ns, 29 of them exactly periodic at 25 ns. SPS itself goes from spread 0 at 25 ns to
0.06-0.25 A at 10 ns, so the rule value (the maximum of the 80 period peaks) of the SPS reference is lifted by something other
than the current in a settled period. Until that is understood the 10 ns statements (goal 3 breaks gone, headline +0.403 A)
cannot be read at face value. Not the controller: the phase counter replayed in IEEE double at 10 ns gives one gate pattern over
1600 periods for every leg (after the first period; %TEMP% replay, same arithmetic as pwm_counter_check.py).
Some peak sequences hop in steps of 2 V1 Ts / L = 0.084 A (109.2 V 1 kW: 4.544 / 4.628; 105 V 400 W: 2.703 / 2.789 / 2.875), i.e. one
grid step of a primary edge.

Hypothesis H1: the Half-Bridge Driver delays tp = 30 ns and tp + td = 130 ns fall exactly on the 10 ns grid (3 and 13 steps), so
whether an edge lands on one step or the next is decided by rounding. They do not at 25 ns (1.2, 5.2) or 100 ns (0.3, 1.3).
Hypothesis H2: the body diode's transit time TT = 16 ns starts to be resolved below about 16 ns.

Runs (same runner settings as g10: constant D0, 20 ms):
  A  10 ns, 6 configurations {SPS, M-B} x {84 V 1 kW, 109.2 V 1 kW (SPS) / 300 W (M-B), 105 V 400 W}, the two bracketing levels of g10:
     tp35 = tp 35 ns (3.5, 13.5 steps; a common delay, no relative phase), 12 runs
     TT0  = TT 0 at nominal tp, 6 runs (one level per configuration)
     ctl  = nominal through the new runner (verify_steady_fine_tp.m), 2 runs: must reproduce g10 to 1e-6 A, else the set is void
  B  12.5 ns (V1 Ts / L = 0.053 A; tp 2.4 steps, tp + td 10.4 steps, nominal plant, exactly half of 25 ns), all g10 configurations.

RULES (registered 2026-10-31 00:23, before any of these runs exists; do not change after the results). Nothing here touches the
verdict of grid10_check.py: T2 "not converged" and C1-C4 stay on record as they are.
  A1 a run has "collapsed" iff spread <= max(5 mA, 0.25 x its spread at 10 ns nominal).
     Set X has collapsed iff at least 75 % of its runs have. tp35 and 12.5 ns are judged on the same 12 runs.
     "explained" = both tp35 and 12.5 ns collapsed -> the flicker is the delay / grid coincidence (H1), a property of this 10 ns run;
     "partly" = exactly one of them; "not explained" = neither. TT0 reported on its own ("recovery matters" iff collapsed).
  B1 at 12.5 ns the same rules as grid10_check.py with 12.5 ns in place of 10 ns: T1 (rule_r2 against the 25 ns statements),
     T2 (headline, converged iff |25 ns - 12.5 ns| <= 0.03 A), C1, C2, C3, C4.
  F  what the report then says about the fine grid:
     "explained": the 12.5 ns statements replace the 10 ns ones (the 10 ns values stay as a record of the effect);
     otherwise each statement is given at both grids and counts only if both agree: C1 "SPS lower than X" stands iff kept at both,
     does not stand iff not kept at both, else "grid dependent"; same for the k = 1 statement C2.
  M  (reported, no threshold, never a verdict) the same statements on the median of the 80 period peaks at all three grids.

    python -I -X utf8 Simulink/Thesis/g12_check.py selftest    (the rules on made-up numbers)
    python -I -X utf8 Simulink/Thesis/g12_check.py make        -> out/dg10_points.csv, out/g12_points_r1.csv
    python automation/job.py submit g12 --interactive --after md1 -- matlab -batch "cd('Simulink/Thesis'); g12_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/g12_check.py next        (not bracketed -> next round; g12_rounds calls it)
    python -I -X utf8 Simulink/Thesis/g12_check.py compare     -> out/g12_check.{log,csv}, out/g12_configs.csv, out/g12_verdict.md
                                                                  (before the runs: the 25 / 10 ns part only -> out/g10_side.log)
"""
import argparse
import csv
import datetime
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import grid10_check as g10  # noqa: E402
import r13_map as rm  # noqa: E402

OUT = os.path.join(HERE, "out")
TS25, TS10, TS12 = rm.TS, g10.TS10, 1.25e-8
WIN = fg.WIN
COLLAPSE, MAJORITY = 0.25, 0.75
PERIODIC = rm.PERIODIC
GRIDS = (("25", TS25), ("12.5", TS12), ("10", TS10))
A_CONFIGS = ((84.0, 1000.0, "SPS"), (109.2, 1000.0, "SPS"), (105.0, 400.0, "SPS"), (84.0, 1000.0, "M-B"), (109.2, 300.0, "M-B"), (105.0, 400.0, "M-B"))
TT0_LEVEL = {(84.0, "SPS"): 1, (109.2, "SPS"): 1, (105.0, "SPS"): 1, (84.0, "M-B"): 0, (109.2, "M-B"): 0, (105.0, "M-B"): 0}   # 0 = lower, 1 = upper


def results_12():
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "g12_sweep_r*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "g12_sweep_r*.csv"))
          for m in [re.search(r"g12_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def _a_config(v, p, ctrl):
    return next(c for c in g10.configs() if c["V2"] == v and c["P"] == p and c["ctrl"] == ctrl)


def make():
    d10 = g10.results_10()
    pts, seen = [], set()
    for c in g10.configs():
        lo = fg.at_power(fg.runs_of(d10, c, TS10), c["P"])[2]
        if lo is None:
            sys.exit(f"no 10 ns bracket for {c}")
        pts += [fg._point(c, j, TS12) for j in (lo, lo + 1)]
    fg._write(os.path.join(OUT, "g12_points_r1.csv"), pts)
    rows = []

    def add(c, j, variant, tp, tt):
        p = fg._point(c, j, TS10)
        rows.append(dict(p, label=f"{p['label']} {variant}", variant=variant, tp_ns=tp, TT_ns=tt))
    for v, pw, ctrl in A_CONFIGS:
        c = _a_config(v, pw, ctrl)
        lo = fg.at_power(fg.runs_of(d10, c, TS10), pw)[2]
        add(c, lo, "tp35", 35, 16)
        add(c, lo + 1, "tp35", 35, 16)
        add(c, lo + TT0_LEVEL[(v, ctrl)], "TT0", 30, 0)
        if (v, ctrl) in ((109.2, "SPS"), (105.0, "M-B")):
            add(c, lo + 1, "ctl", 30, 16)
    with open(os.path.join(OUT, "dg10_points.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2", "Ts", "variant", "tp_ns", "TT_ns"])
        w.writeheader()
        w.writerows(rows)
    print(f"wrote out/dg10_points.csv: {len(rows)} runs")


def next_round():
    d = results_12()
    if d is None:
        sys.exit("no 12.5 ns runs yet")
    pts = []
    for c in g10.configs():
        g = fg.runs_of(d, c, TS12)
        if g.empty:
            continue
        if not np.isfinite(fg.at_power(g, c["P"])[0]):
            pts += [fg._point(c, j, TS12) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"g12_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def collapsed(spread, spread10):
    return bool(np.isfinite(spread) and np.isfinite(spread10) and spread <= max(PERIODIC, COLLAPSE * spread10))


def set_collapsed(flags):
    flags = list(flags)
    return len(flags) > 0 and sum(flags) / len(flags) >= MAJORITY


def rule_a1(tp35, ts12):
    """tp35, ts12: per-run 'collapsed' flags of the two sets -> explained / partly / not explained / undecidable"""
    if not list(tp35) or not list(ts12):
        return "undecidable"
    a, b = set_collapsed(tp35), set_collapsed(ts12)
    return "explained" if a and b else "partly" if a or b else "not explained"


def rule_f(a1, res12, res10):
    """one statement's classification at the two grids -> stands / does not stand / grid dependent"""
    if a1 == "explained":
        return "stands" if res12 == "kept" else "does not stand"
    if res12 == "kept" and res10 == "kept":
        return "stands"
    if res12 != "kept" and res10 != "kept":
        return "does not stand"
    return "grid dependent"


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("collapse below 5 mA", collapsed(0.004, 0.01), True)
    chk("collapse a quarter", collapsed(0.05, 0.25), True)
    chk("no collapse", collapsed(0.07, 0.25), False)
    chk("no collapse NaN", collapsed(np.nan, 0.25), False)
    chk("set 75 %", set_collapsed([True, True, True, False]), True)
    chk("set below 75 %", set_collapsed([True, True, False, False]), False)
    chk("set empty", set_collapsed([]), False)
    t, f = [True] * 4, [False] * 4
    chk("explained", rule_a1(t, t), "explained")
    chk("partly tp", rule_a1(t, f), "partly")
    chk("partly grid", rule_a1(f, t), "partly")
    chk("not explained", rule_a1(f, f), "not explained")
    chk("undecidable", rule_a1([], t), "undecidable")
    chk("F explained kept", rule_f("explained", "kept", "gone"), "stands")
    chk("F explained gone", rule_f("explained", "gone", "kept"), "does not stand")
    chk("F other both kept", rule_f("partly", "kept", "kept"), "stands")
    chk("F other both not", rule_f("not explained", "gone", "reversed"), "does not stand")
    chk("F other differ", rule_f("partly", "kept", "gone"), "grid dependent")
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def _pp_all():
    pp = {"10": g10._pp("g10_sweep_r*.csv", TS10), "12.5": g10._pp("g12_sweep_r*.csv", TS12), "25": {}}
    for pat in ("fine_sweep_r*.csv", "r13a_sweep_r*.csv", "r13b_sweep_r*.csv"):
        pp["25"].update(g10._pp(pat, TS25))
    return pp


def _value(c, d, pp, Ts):
    """at |P| on one grid -> dict(I rule value, osc, lo level, spread (larger of the two levels), med interpolated at |P|)"""
    g = fg.runs_of(d, c, Ts) if d is not None else None
    if g is None or g.empty:
        return dict(I=np.nan, osc=np.nan, lo=None, spread=np.nan, med=np.nan)
    a = fg.at_power(g, c["P"])
    r = dict(I=a[0], osc=a[1], lo=a[2], spread=np.nan, med=np.nan)
    if a[2] is None:
        return r
    rows = [g[g["level"] == j].iloc[0] for j in (a[2], a[2] + 1)]
    x = [pp.get((round(c["V2"], 6), j, round(float(q["D1"]), 9), round(float(q["D2"]), 9))) for j, q in zip((a[2], a[2] + 1), rows)]
    if any(e is None for e in x):
        return r
    Pd = [float(-q[f"P_LV_at_{WIN}ms"]) for q in rows]
    t = (c["P"] - Pd[0]) / (Pd[1] - Pd[0])
    r.update(spread=max(e[0] - e[2] for e in x), med=x[0][1] + t * (x[1][1] - x[0][1]))
    return r


def _a_table(pp, d10, o):
    """mechanism runs: per run the 80-period spread on each variant -> (rows, flags)"""
    import pandas as pd
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "dg10_sweep_part*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        return None, {}
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    ppa = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "dg10_sweep_part*.csv"))):
        T = pd.read_csv(f)
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float), -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T.itertuples(), pk):
            ppa[(r.variant, round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = (np.nanmax(x), np.nanmedian(x), np.nanmin(x))
    rows, flags = [], {"tp35": [], "TT0": [], "12.5": []}
    o("\nA  mechanism runs: 80-period peak spread and median [A] (10 ns nominal = g10; ctl must equal it)")
    o(f"{'V2':>6} {'ctrl':>4} {'lvl':>4} | {'10 ns':>7} {'tp35':>7} {'TT0':>7} {'12.5 ns':>7} | med {'10 ns':>7} {'tp35':>7} {'TT0':>7} {'12.5':>7} | Ipk {'10 ns':>7} {'tp35':>7} {'TT0':>7} | P[W] {'10 ns':>7} {'tp35':>7} {'TT0':>7}")
    for (v, lv, D1, D2, ctrl), g in d.groupby(["V2", "level", "D1", "D2", "ctrl"]):
        key = (round(v, 6), int(lv), round(D1, 9), round(D2, 9))
        n = pp["10"].get(key)
        base = fg.runs_of(d10, dict(V2=v, D1=D1, D2=D2), TS10)
        base = base[base["level"] == lv]
        s10 = n[0] - n[2] if n else np.nan

        def var(name):
            q = ppa.get((name,) + key)
            return (q[0] - q[2], q[1]) if q else (np.nan, np.nan)
        sp, st = var("tp35"), var("TT0")
        q12 = [e for k, e in pp["12.5"].items() if k[0] == key[0] and k[1] == key[1] and abs(k[2] - key[2]) < 1e-6 and abs(k[3] - key[3]) < 1e-6]
        s12, m12 = (q12[0][0] - q12[0][2], q12[0][1]) if q12 else (np.nan, np.nan)
        if np.isfinite(sp[0]):
            flags["tp35"].append(collapsed(sp[0], s10))
            flags["12.5"].append(collapsed(s12, s10))
        if np.isfinite(st[0]):
            flags["TT0"].append(collapsed(st[0], s10))

        def col(name, c):
            h = g[g["variant"] == name]
            return float(h[c].iloc[0]) if len(h) else np.nan
        b_i, b_p = (float(base[f"Ipk_at_{WIN}ms"].iloc[0]), float(-base[f"P_LV_at_{WIN}ms"].iloc[0])) if len(base) else (np.nan, np.nan)
        o(f"{v:6g} {ctrl:>4} {int(lv):4d} | {s10:7.3f} {sp[0]:7.3f} {st[0]:7.3f} {s12:7.3f} | med {(n[1] if n else np.nan):7.3f} {sp[1]:7.3f} {st[1]:7.3f} {m12:7.3f} | "
          f"Ipk {b_i:7.3f} {col('tp35', f'Ipk_at_{WIN}ms'):7.3f} {col('TT0', f'Ipk_at_{WIN}ms'):7.3f} | P[W] {b_p:7.1f} {-col('tp35', f'P_LV_at_{WIN}ms'):7.1f} {-col('TT0', f'P_LV_at_{WIN}ms'):7.1f}")
        rows.append(dict(V2=v, ctrl=ctrl, level=int(lv), s10=s10, s_tp35=sp[0], s_TT0=st[0], s12=s12))
        ic = col("ctl", f"Ipk_at_{WIN}ms")
        if np.isfinite(ic):
            o(f"       ctl: |Ipk(new runner) - Ipk(g10)| = {abs(ic - b_i):.2e} A" + ("" if abs(ic - b_i) <= 1e-6 else "   ** NOT REPRODUCED: the A set is void **"))
    return rows, flags


def compare():
    import pandas as pd
    d25, d10, d12 = rm.results_25(), g10.results_10(), results_12()
    pp = _pp_all()
    data = {"25": d25, "12.5": d12, "10": d10}
    lines, cf_rows = [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)
    cfs = g10.configs()
    o("g12_check: 25 ns statements at 12.5 ns and 10 ns; rules in the file header. Spread = 80-period peak max - min, larger of the two levels")
    o(f"{'set':>3} {'V2':>6} {'|P|':>5} {'ctrl':>6} | Ipk {'25':>6} {'12.5':>6} {'10':>6} | spread {'25':>6} {'12.5':>6} {'10':>6} | median {'25':>6} {'12.5':>6} {'10':>6}")
    per_point = []
    for i, (st, _, v, p, names) in enumerate(g10.POINTS):
        vals = {}
        for c in [x for x in cfs if x["pair"] == i]:
            vals[c["ctrl"]] = {g: _value(c, data[g], pp[g], ts) for g, ts in GRIDS}
            w = vals[c["ctrl"]]
            o(f"{st:>3} {v:6g} {p:5.0f} {c['ctrl']:>6} | Ipk {w['25']['I']:6.3f} {w['12.5']['I']:6.3f} {w['10']['I']:6.3f} | spread {w['25']['spread']:6.3f} {w['12.5']['spread']:6.3f} {w['10']['spread']:6.3f} | "
              f"median {w['25']['med']:6.3f} {w['12.5']['med']:6.3f} {w['10']['med']:6.3f}")
            cf_rows.append(dict(set=st, V2=v, P=p, ctrl=c["ctrl"], **{f"{k}{g}": w[g][k] for g in w for k in ("I", "osc", "spread", "med")}))
        per_point.append((st, v, p, vals))
    C = pd.DataFrame(cf_rows)
    C.to_csv(os.path.join(OUT, "g12_configs.csv" if d12 is not None else "g10_side_configs.csv"), index=False, float_format="%.6g")
    for g, cell in (("25", 380.0 * TS25 / 90e-6), ("12.5", 380.0 * TS12 / 90e-6), ("10", 380.0 * TS10 / 90e-6)):
        s = C[f"spread{g}"].dropna()
        if s.empty:
            continue
        o(f"spread at {g} ns: {int((s > PERIODIC).sum())}/{len(s)} configurations non-periodic, median {s.median():.3f} A, max {s.max():.3f} A; V1 Ts / L = {cell:.3f} A")
        for ctrl in ("SPS", "M-B", "M-S"):
            q = C[C["ctrl"] == ctrl][f"spread{g}"].dropna()
            if len(q):
                o(f"    {ctrl}: {int((q > PERIODIC).sum())}/{len(q)} non-periodic, median {q.median():.3f}, range {q.min():.3f}-{q.max():.3f} A")
    if d12 is not None:
        # (10-31 09:50, added after the first 12.5 ns results, reported only) how far one configuration moves between grids
        o("grid to grid, same configuration [A]: |Ipk rule| where both grids are periodic, |median| over all")
        for a, b in (("25", "12.5"), ("10", "12.5")):
            both = C[(C[f"spread{a}"] <= PERIODIC) & (C[f"spread{b}"] <= PERIODIC)]
            dr, dm = (both[f"I{a}"] - both[f"I{b}"]).abs().dropna(), (C[f"med{a}"] - C[f"med{b}"]).abs().dropna()
            line = f"    {a} vs {b} ns: median of the medians {dm.median():.3f}, max {dm.max():.3f}, within 0.03 A {int((dm <= 0.03).sum())}/{len(dm)}"
            if len(dr):
                line = f"    {a} vs {b} ns: rule, periodic on both {len(dr)} configurations: median {dr.median():.3f}, max {dr.max():.3f}; median of the medians {dm.median():.3f}, max {dm.max():.3f}, within 0.03 A {int((dm <= 0.03).sum())}/{len(dm)}"
            o(line)

    def pair(vals, g, k):
        return {n: (w[g][k], w[g]["osc"]) for n, w in vals.items()}

    def classify(k, g):
        rows = []
        for st, v, p, vals in per_point:
            for r in g10.statements(pair(vals, "25", k), pair(vals, g, k)):
                rows.append(dict(set=st, V2=v, P=p, **r))
        return rows

    def t2(k, g):
        h = []
        for st, v, p, vals in per_point:
            if st != "H":
                continue
            a, b, c_, d_ = vals["SPS"]["25"], vals["M-B"]["25"], vals["SPS"][g], vals["M-B"][g]
            ok = all(g10.clean((e[k], e["osc"])) for e in (a, b, c_, d_))
            h.append((a[k] - b[k] if ok else np.nan, c_[k] - d_[k] if ok else np.nan))
        return g10.rule_t2(h)

    def q1_of(rows):
        return [r for r in rows if r["set"] == "Q" or (r["set"] == "W" and r["lo"] == "SPS" and r["hi"] == "M-S") or (r["set"] == "W" and r["V2"] == 92.4 and r["lo"] == "SPS")]

    def counts(rows):
        res = [r["res"] for r in rows]
        return ", ".join(f"{x} {res.count(x)}" for x in ("kept", "gone", "reversed", "undecidable"))

    o("\nM  statements on the median of the 80 period peaks (reported, never a verdict):")
    t10m = t2("med", "10")
    o(f"   T2 on medians, 10 ns: 25 ns {t10m[1]:+.3f} A, 10 ns {t10m[2]:+.3f} A -> {t10m[0]}")
    rows_m10 = classify("med", "10")
    for r in rows_m10:
        o(f"   {r['set']} {r['V2']:6g} V {r['P']:5.0f} W {r['lo']} lower than {r['hi']}: {r['d25']:+.3f} A at 25 ns -> {r['d10']:+.3f} A at 10 ns (tol {r['tol10']:.3f}): {r['res']}")
    o(f"   counts at 10 ns: {counts(rows_m10)}; Q1 statements {counts(q1_of(rows_m10))}")
    rows_a, flags = _a_table(pp, d10, o)
    verdict = None
    if d12 is not None:
        rows12, rows10 = classify("I", "12.5"), classify("I", "10")
        rm12 = classify("med", "12.5")
        k10 = {(r["set"], r["V2"], r["P"], r["lo"], r["hi"]): r["res"] for r in rows10}
        o("\nB1 T1 statements of the 25 ns map at 12.5 ns (rule_r2), the 10 ns class beside:")
        for r in rows12:
            o(f"   {r['set']} {r['V2']:6g} V {r['P']:5.0f} W  {r['lo']} lower than {r['hi']}: {r['d25']:+.3f} A at 25 ns -> {r['d10']:+.3f} A at 12.5 ns (tol {r['tol10']:.3f}): {r['res']}; 10 ns: {k10.get((r['set'], r['V2'], r['P'], r['lo'], r['hi']))}")
        code, m25, m12 = t2("I", "12.5")
        tm = t2("med", "12.5")
        o(f"   T2 headline SPS - M-B: 25 ns {m25:+.3f} A, 12.5 ns {m12:+.3f} A -> {code}; on medians {tm[1]:+.3f} -> {tm[2]:+.3f} A ({tm[0]})")
        o(f"   median statements at 12.5 ns: {counts(rm12)}")
        a1 = rule_a1(flags.get("tp35", []), flags.get("12.5", []))
        o(f"\nA1 tp35 collapsed {sum(flags.get('tp35', []))}/{len(flags.get('tp35', []))}, 12.5 ns {sum(flags.get('12.5', []))}/{len(flags.get('12.5', []))} -> {a1}; "
          f"TT0 {sum(flags.get('TT0', []))}/{len(flags.get('TT0', []))} -> {'recovery matters' if set_collapsed(flags.get('TT0', [])) else 'no'}")
        w = [r for r in rows12 if r["set"] == "W" and r["lo"] == "SPS"]
        k = next((r for r in rows12 if r["set"] == "K" and r["V2"] == 95.0 and r["lo"] == "SPS"), None)
        h_res = [r["res"] for r in rows12 if r["set"] == "H"]
        o("\nConclusions (F):")
        fin = []
        for r in w:
            res10 = k10.get((r["set"], r["V2"], r["P"], r["lo"], r["hi"]))
            f = rule_f(a1, r["res"], res10)
            fin.append((r, res10, f))
            o(f"  C1 {r['V2']:g} V {r['P']:.0f} W SPS lower than {r['hi']}: 12.5 ns {r['res']}, 10 ns {res10} -> {f}")
        if k is not None:
            res10 = k10.get((k["set"], k["V2"], k["P"], k["lo"], k["hi"]))
            o(f"  C2 k = 1 statement 95 V SPS lower than D = 0.40: 12.5 ns {k['res']}, 10 ns {res10} -> {rule_f(a1, k['res'], res10)}")
        c3 = g10.c3_map(code, h_res)
        o(f"  C3 at 12.5 ns: {'yes' if c3 else 'no'} (T2 {code}; H statements: " + ", ".join(f"{x} {h_res.count(x)}" for x in ("kept", "gone", "reversed", "undecidable")) + ")")
        o(f"  C4 Q1 statements at 12.5 ns: {counts(q1_of(rows12))}; at 10 ns: {counts(q1_of(rows10))}")
        verdict = dict(a1=a1, rows12=rows12, k10=k10, fin=fin, code=code, m25=m25, m12=m12, tm=tm, c3=c3, flags=flags)
        pd.DataFrame(rows12).to_csv(os.path.join(OUT, "g12_check.csv"), index=False, float_format="%.6g")
    nmiss = int(C["I12.5"].isna().sum())
    if d12 is not None and nmiss:
        o(f"\n{nmiss} configurations not bracketed at 12.5 ns yet -> provisional")
    log = "g12_check.log" if d12 is not None else "g10_side.log"
    open(os.path.join(OUT, log), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    if verdict is not None:
        _verdict_md(verdict)
    print(f"wrote out/{log}")


def _verdict_md(v):
    KO = {"kept": "유지", "gone": "사라짐", "reversed": "뒤집힘", "undecidable": "판정 불가", None: "진술 없음"}
    FK = {"stands": "성립", "does not stand": "성립 안 함", "grid dependent": "격자에 따라 다름"}
    AK = {"explained": "설명됨(H1)", "partly": "부분", "not explained": "설명 안 됨", "undecidable": "판정 불가"}
    md = ["# 10 ns 흔들림의 원인과 12.5 ns 재검증 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `g12_check.py compare`. 규칙은 머리말(10-31 00:23, 실행 전 등록)이며 바꾸지 않았다. `grid10_check.py`의 판정은 그대로 둔다.", "",
          f"- A1 드라이버 지연이 10 ns 격자와 겹치는 것이 원인인가: **{AK[v['a1']]}** (tp 35 ns {sum(v['flags'].get('tp35', []))}/{len(v['flags'].get('tp35', []))}, 12.5 ns {sum(v['flags'].get('12.5', []))}/{len(v['flags'].get('12.5', []))}, TT 0 {sum(v['flags'].get('TT0', []))}/{len(v['flags'].get('TT0', []))})",
          f"- T2 대표 6점 SPS − B: 25 ns {v['m25']:+.3f} A, 12.5 ns {v['m12']:+.3f} A → **{'수렴' if v['code'] == 'converged' else '미수렴' if v['code'] == 'not converged' else '판정 불가'}** (주기 중앙값 {v['tm'][1]:+.3f} → {v['tm'][2]:+.3f} A, 판정 아님)", "",
          "| 묶음 | V2 | \\|P\\| | 25 ns 진술 | 25 ns 차 | 12.5 ns 차 | 허용 | 12.5 ns | 10 ns |", "|---|---|---|---|---|---|---|---|---|"]
    for r in v["rows12"]:
        md.append(f"| {r['set']} | {r['V2']:g} V | {r['P']:.0f} W | {r['lo']} < {r['hi']} | {r['d25']:+.3f} | {r['d10']:+.3f} | {r['tol10']:.3f} | {KO[r['res']]} | {KO[v['k10'].get((r['set'], r['V2'], r['P'], r['lo'], r['hi']))]} |")
    md += [""] + [f"- C1 {r['V2']:g} V {r['P']:.0f} W \"SPS < {r['hi']}\": **{FK[f]}**" for r, _, f in v["fin"]]
    md += [f"- C3 12.5 ns: **{'수렴' if v['c3'] else '수렴 아님'}**", "", "표 원본 `out/g12_check.csv`, `out/g12_configs.csv`, 로그 `out/g12_check.log`."]
    open(os.path.join(OUT, "g12_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
