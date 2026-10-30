"""
grid10_check.py -- (2026-10-30 18:22) do the statements of the 25 ns map (r13_map.py) survive a 10 ns grid?

GRID (fine_grid_check.py) found that the 100 ns grid of the dataset-era model produced differences of 0.1-1 A that are
gone at 25 ns. r13a (10-30 18:08) asks the same question one level down: on the 25 ns map three points are "worse than
SPS" by 0.05-0.14 A, each with a bracketing run whose period peaks are not constant (r13_map.py compare, out/iL_pp/), and
the single-period spikes of such runs cluster at one grid step V1 Ts / L = 0.106 A (0.042 A at 10 ns). Until a finer grid
is measured those statements, and the map's reductions, could be grid-scale as well. Same runner (verify_steady_fine.m),
same window (20 ms), same method (constant D0, two adjacent 2.88-deg levels, Ipk interpolated in |P|).

Configurations (first levels = the 25 ns bracket of the same configuration; `next` adds levels where 10 ns does not bracket):
  W  the points behind the map's Q2 "worse" statements and its largest Q4 differences: 92.4 V 300 W {SPS, M-B},
     96.6 V 100 W and 105 V 400 W {SPS, M-B, M-S}
  K  the reversed k = 1 statements of Q5: 95 V 100 W {SPS, D1 = D2 = 0.40}, 96 V 200 W {SPS, M-S}
  H  the headline set of GRID R3: 84 / 109.2 V x 100 / 300 / 1000 W {SPS, M-B}

RULES (registered 2026-10-30 18:22, before any 10 ns run exists; do not change after the results):
  Values: fine_grid_check.at_power, 20 ms window; clean = both levels |mean i_L| <= 20 mA;
          tol = max(0.03 A, 2 % Ipk(SPS at that grid)) (2 % of the larger value if SPS is not clean).
  T1 every pair (X, Y) of a point with a 25 ns statement "X lower than Y" (both clean, |d| > tol at 25 ns) is classified at
     10 ns by fine_grid_check.rule_r2: kept / gone / reversed / undecidable.
  T2 headline: mean Ipk(SPS) - Ipk(M-B) over the H points clean at both grids, at 25 and at 10 ns.
     "converged" iff |difference| <= 0.03 A, else "not converged"; fewer than 4 points -> "undecidable".
  T3 reported, no threshold: per run the spread of the 80 period peaks at both grids, in A and in grid steps V1 Ts / L;
     per configuration the change of the median-peak value between the grids.
  Conclusions:
    C1 each W statement "SPS lower than ..." stands at 10 ns iff "kept" (gone / reversed -> it was grid scale);
    C2 "no k = 1 valley at 95 V" (25 ns: SPS lower than D = 0.40) stands unless that statement is "reversed";
    C3 "the 25 ns map has converged for its reductions" iff T2 "converged" and no H statement is "gone" or "reversed".

ADDED 2026-10-30 22:10, after r13b (the map's Q1 is final: 11 "not optimal" points -> stage 2, M-D) and before any 10 ns run
exists; the rules above are untouched. The 100 ns retraining (M-S) chased grid-scale differences, so the statements that
trigger the 25 ns retraining go through the same test before M-D is believed:
  Q  the Q1 "best candidate lower than M-S" statements not in W: 96.6 V 150 W, 100.8 V 100 / 300 W, 105 V 100 / 150 / 200 W,
     109.2 V 200 / 300 / 400 W, {M-S, the best candidate}. T1 classifies them as above (W holds the other two: 92.4 V 300 W,
     96.6 V 100 W, SPS lower than M-S).
  C4 reported, no threshold: how many of the 11 Q1 statements are kept / gone / reversed / undecidable at 10 ns.

    python -I -X utf8 Simulink/Thesis/grid10_check.py make       -> out/g10_points_r1.csv
    python automation/job.py submit g10 --interactive --after r11r2 -- matlab -batch "cd('Simulink/Thesis'); grid10_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/grid10_check.py next       (not bracketed -> next round; grid10_rounds calls it)
    python -I -X utf8 Simulink/Thesis/grid10_check.py compare    -> out/g10_check.{log,csv}, out/g10_verdict.md
    python -I -X utf8 Simulink/Thesis/grid10_check.py selftest   (the rules on made-up numbers)
"""
import argparse
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402

OUT = os.path.join(HERE, "out")
TS10 = 1e-8
WIN = fg.WIN
OSC = fg.OSC
TOL_ABS, TOL_REL = fg.TOL_ABS, fg.TOL_REL
CONVERGED = 0.03
MIN_H = 4
POINTS = (("W", "map", 92.4, 300.0, ("SPS", "M-B")), ("W", "map", 96.6, 100.0, ("SPS", "M-B", "M-S")),
          ("W", "map", 105.0, 400.0, ("SPS", "M-B", "M-S")), ("K", "x", 95.0, 100.0, ("SPS", "D0.40")),
          ("K", "x", 96.0, 200.0, ("SPS", "M-S"))) + tuple(
    ("H", "map", v, p, ("SPS", "M-B")) for v in (84.0, 109.2) for p in (100.0, 300.0, 1000.0)) + tuple(
    ("Q", "map", v, p, ("M-S", best)) for v, p, best in (
        (96.6, 150.0, "D1+"), (100.8, 100.0, "D1-"), (100.8, 300.0, "D1+"), (105.0, 100.0, "D1+"), (105.0, 150.0, "D2+"),
        (105.0, 200.0, "D2-"), (109.2, 200.0, "D1+"), (109.2, 300.0, "D1+"), (109.2, 400.0, "D2-")))


def configs():
    """the run configurations: dict(set, pair, ctrl, V2, P, D1, D2); a candidate on another's duties (alias) keeps its name
    but the alias's duties, so the two share their runs"""
    cf = []
    tab = {w: {(v, p): cs for v, p, cs in rm.candidates(w)} for w in ("map", "x")}
    for i, (st, which, v, p, names) in enumerate(POINTS):
        by = {c["name"]: c for c in tab[which][(v, p)]}
        for nm in names:
            c = by[nm]
            own = by[c["alias"]] if c["alias"] else c
            cf.append(dict(set=st, pair=i, ctrl=nm, V2=v, P=p, D1=own["D1"], D2=own["D2"]))
    return cf


def results_10():
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "g10_sweep_r*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        return None
    d = fg._load(fs, fine=True)
    return d.drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "g10_sweep_r*.csv"))
          for m in [re.search(r"g10_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def make():
    d25, pts = rm.results_25(), []
    for c in configs():
        g = rm._runs(d25, c)
        lo = fg.at_power(g, c["P"])[2] if not g.empty else None
        if lo is None:
            sys.exit(f"no 25 ns bracket for {c}")
        pts += [fg._point(c, j, TS10) for j in (lo, lo + 1)]
    fg._write(os.path.join(OUT, "g10_points_r1.csv"), pts)


def next_round():
    d = results_10()
    if d is None:
        sys.exit("no 10 ns runs yet")
    pts = []
    for c in configs():
        g = fg.runs_of(d, c, TS10)
        if g.empty:
            continue
        if not np.isfinite(fg.at_power(g, c["P"])[0]):
            pts += [fg._point(c, j, TS10) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"g10_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def clean(x):
    return x is not None and np.isfinite(x[0]) and np.isfinite(x[1]) and x[1] <= OSC


def tol_of(vals):
    s = vals.get("SPS")
    if clean(s):
        return max(TOL_ABS, TOL_REL * s[0])
    big = [v[0] for v in vals.values() if v is not None and np.isfinite(v[0])]
    return max(TOL_ABS, TOL_REL * max(big)) if big else TOL_ABS


def statements(v25, v10):
    """{name: (Ipk, osc)} at both grids -> rows (lo, hi, d25, d10, tol25, tol10, res): "lo lower than hi" at 25 ns"""
    t25, t10, rows = tol_of(v25), tol_of(v10), []
    names = list(v25)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if not (clean(v25[a]) and clean(v25[b])):
                continue
            d = v25[b][0] - v25[a][0]
            if abs(d) <= t25:
                continue
            lo, hi = (a, b) if d > 0 else (b, a)
            d10 = v10[hi][0] - v10[lo][0]
            res = fg.rule_r2(abs(d), d10, t25, t10, True, clean(v10[lo]) and clean(v10[hi]))
            rows.append(dict(lo=lo, hi=hi, d25=abs(d), d10=d10, tol25=t25, tol10=t10, res=res))
    return rows


def rule_t2(pairs):
    """pairs: [(dI at 25 ns, dI at 10 ns)], NaN where not clean -> (code, mean25, mean10)"""
    ok = [(a, b) for a, b in pairs if np.isfinite(a) and np.isfinite(b)]
    if len(ok) < MIN_H:
        return "undecidable", np.nan, np.nan
    m25, m10 = float(np.mean([a for a, _ in ok])), float(np.mean([b for _, b in ok]))
    return ("converged" if abs(m25 - m10) <= CONVERGED else "not converged"), m25, m10


def c1_stands(res):
    return res == "kept"


def c2_stands(res):
    return res != "reversed"


def c3_map(t2_code, h_res):
    return t2_code == "converged" and not any(r in ("gone", "reversed") for r in h_res)


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    c = lambda i, o=0.0: (i, o)  # noqa: E731
    r = statements({"SPS": c(1.0), "M-B": c(1.2)}, {"SPS": c(1.0), "M-B": c(1.1)})[0]
    chk("kept", (r["lo"], r["hi"], r["res"]), ("SPS", "M-B", "kept"))
    chk("gone", statements({"SPS": c(1.0), "M-B": c(1.2)}, {"SPS": c(1.0), "M-B": c(1.01)})[0]["res"], "gone")
    chk("reversed", statements({"SPS": c(1.0), "M-B": c(1.2)}, {"SPS": c(1.0), "M-B": c(0.9)})[0]["res"], "reversed")
    chk("undecidable at 10 ns", statements({"SPS": c(1.0), "M-B": c(1.2)}, {"SPS": c(1.0), "M-B": c(1.2, 0.03)})[0]["res"],
        "undecidable")
    chk("no statement within tol", statements({"SPS": c(1.0), "M-B": c(1.02)}, {"SPS": c(1.0), "M-B": c(1.2)}), [])
    chk("no statement if 25 ns oscillates", statements({"SPS": c(1.0, 0.03), "M-B": c(1.2)}, {"SPS": c(1.0), "M-B": c(1.2)}), [])
    r = statements({"SPS": c(2.0), "M-B": c(1.5), "M-S": c(1.3)}, {"SPS": c(2.0), "M-B": c(1.5), "M-S": c(1.5)})
    chk("three controllers", [(x["lo"], x["hi"], x["res"]) for x in r],
        [("M-B", "SPS", "kept"), ("M-S", "SPS", "kept"), ("M-S", "M-B", "gone")])
    chk("tol from SPS", round(tol_of({"SPS": c(3.0), "M-B": c(1.0)}), 6), 0.06)
    chk("tol when SPS oscillates", round(tol_of({"SPS": c(3.0, 0.03), "M-B": c(2.0)}), 6), 0.06)
    chk("T2 converged", rule_t2([(0.3, 0.32), (0.5, 0.51), (0.1, 0.09), (0.2, 0.21)])[0], "converged")
    chk("T2 not converged", rule_t2([(0.3, 0.4), (0.5, 0.6), (0.1, 0.2), (0.2, 0.3)])[0], "not converged")
    chk("T2 too few", rule_t2([(0.3, 0.3), (0.5, np.nan), (0.1, 0.1), (0.2, 0.2)])[0], "undecidable")
    chk("C1 only kept stands", [c1_stands(x) for x in ("kept", "gone", "reversed", "undecidable")], [True, False, False, False])
    chk("C2 only reversed breaks", [c2_stands(x) for x in ("kept", "gone", "reversed", "undecidable")], [True, True, False, True])
    chk("C3 true", c3_map("converged", ["kept", "kept", "undecidable"]), True)
    chk("C3 a gone H statement", c3_map("converged", ["kept", "gone"]), False)
    chk("C3 not converged", c3_map("not converged", ["kept"]), False)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def _pp(prefix, Ts):
    """(V2, level, D1, D2) -> (pk_max, pk_med, pk_min) from out/iL_pp/"""
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", prefix))):
        T = pd.read_csv(f)
        T = T[np.isclose(T["Ts"], Ts, rtol=1e-6, atol=0)]
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float),
                        -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T.itertuples(), pk):
            out[(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = (np.nanmax(x), np.nanmedian(x), np.nanmin(x))
    return out


def compare():
    import datetime

    import pandas as pd
    d25, d10 = rm.results_25(), results_10()
    if d10 is None:
        sys.exit("no 10 ns runs yet")
    pp25, pp10 = {}, _pp("g10_sweep_r*.csv", TS10)
    for pat in ("fine_sweep_r*.csv", "r13a_sweep_r*.csv", "r13b_sweep_r*.csv"):
        pp25.update(_pp(pat, rm.TS))
    cell10, cell25 = 380.0 * TS10 / 90e-6, rm.CELL
    lines, st_rows, h_pairs, cf_rows = [], [], [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    o(f"grid10_check: 25 ns map statements at Ts = 10 ns; rules in the file header (V1 Ts / L: {cell25:.3f} A at 25 ns, {cell10:.3f} A at 10 ns)")
    cfs = configs()
    o(f"{'set':>3} {'V2':>6} {'|P|':>5} {'ctrl':>6} | {'Ipk 25':>7} {'Ipk 10':>7} {'diff':>7} | {'mean mA 25':>10} {'10':>6} | "
      f"{'spread 25':>9} {'10':>6} | {'med 25':>7} {'med 10':>7} | level 25, 10 ns; P at the levels (10 ns)")
    for i, (st, _, v, p, names) in enumerate(POINTS):
        v25, v10 = {}, {}
        for c in [x for x in cfs if x["pair"] == i]:
            g25, g10 = rm._runs(d25, c), fg.runs_of(d10, c, TS10)
            a, b = fg.at_power(g25, p), fg.at_power(g10, p) if not g10.empty else (np.nan, np.nan, None)
            v25[c["ctrl"]], v10[c["ctrl"]] = a[:2], b[:2]

            def spread(pp, lo, g):
                if lo is None:
                    return np.nan, np.nan
                x = [pp.get((round(c["V2"], 6), j, round(float(g[g["level"] == j]["D1"].iloc[0]), 9),
                             round(float(g[g["level"] == j]["D2"].iloc[0]), 9))) for j in (lo, lo + 1)]
                if any(e is None for e in x):
                    return np.nan, np.nan
                Pd = [float(-g[g["level"] == j][f"P_LV_at_{WIN}ms"].iloc[0]) for j in (lo, lo + 1)]
                t = (p - Pd[0]) / (Pd[1] - Pd[0])
                return max(e[0] - e[2] for e in x), x[0][1] + t * (x[1][1] - x[0][1])
            s25, m25 = spread(pp25, a[2], g25)
            s10, m10 = spread(pp10, b[2], g10)
            pw = ", ".join(f"{float(-g10[g10['level'] == j][f'P_LV_at_{WIN}ms'].iloc[0]):.1f}" for j in (b[2], b[2] + 1)) if b[2] is not None else ""
            o(f"{st:>3} {v:6g} {p:5.0f} {c['ctrl']:>6} | {a[0]:7.3f} {b[0]:7.3f} {b[0] - a[0]:+7.3f} | {1e3 * a[1]:10.1f} {1e3 * b[1]:6.1f} | "
              f"{s25:9.3f} {s10:6.3f} | {m25:7.3f} {m10:7.3f} | {a[2]}, {b[2]}; {pw}")
            cf_rows.append(dict(set=st, V2=v, P=p, ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], Ipk25=a[0], Ipk10=b[0], osc25=a[1], osc10=b[1],
                                spread25=s25, spread10=s10, med25=m25, med10=m10, lo25=a[2], lo10=b[2]))
        for r in statements(v25, v10):
            st_rows.append(dict(set=st, V2=v, P=p, **r))
        if st == "H":
            s25_, b25, s10_, b10 = v25["SPS"], v25["M-B"], v10["SPS"], v10["M-B"]
            both = clean(s25_) and clean(b25) and clean(s10_) and clean(b10)
            h_pairs.append((s25_[0] - b25[0] if both else np.nan, s10_[0] - b10[0] if both else np.nan))
    T = pd.DataFrame(st_rows)
    C = pd.DataFrame(cf_rows)
    T.to_csv(os.path.join(OUT, "g10_check.csv"), index=False, float_format="%.6g")
    C.to_csv(os.path.join(OUT, "g10_configs.csv"), index=False, float_format="%.6g")
    o("\nT1 statements of the 25 ns map at 10 ns (rule_r2):")
    for r in st_rows:
        o(f"   {r['set']} {r['V2']:6g} V {r['P']:5.0f} W  {r['lo']} lower than {r['hi']}: {r['d25']:+.3f} A at 25 ns -> {r['d10']:+.3f} A at 10 ns "
          f"(tol {r['tol10']:.3f}): {r['res']}")
    t2, m25, m10 = rule_t2(h_pairs)
    o(f"\nT2 headline SPS - M-B, {sum(np.isfinite(a) for a, _ in h_pairs)} H points clean at both grids: {m25:+.3f} A at 25 ns, {m10:+.3f} A at 10 ns -> {t2}")
    dm = (C["med10"] - C["med25"]).abs()
    o(f"T3 median-peak change between the grids: median {dm.median():.3f} A, max {dm.max():.3f} A, within 0.03 A at {int((dm <= 0.03).sum())}/{int(dm.notna().sum())} configurations")
    nz = C[C["spread25"] > rm.PERIODIC]
    if len(nz):
        o(f"   configurations with a non-periodic bracketing run at 25 ns: {len(nz)}; their spread {nz['spread25'].median():.3f} A -> {nz['spread10'].median():.3f} A "
          f"({nz['spread25'].median() / cell25:.2f} -> {nz['spread10'].median() / cell10:.2f} grid steps); non-periodic at 10 ns: {int((nz['spread10'] > rm.PERIODIC).sum())}")
    o(f"   non-periodic at 10 ns among those periodic at 25 ns: {int(((C['spread25'] <= rm.PERIODIC) & (C['spread10'] > rm.PERIODIC)).sum())}")
    w = [r for r in st_rows if r["set"] == "W" and r["lo"] == "SPS"]
    k = next((r for r in st_rows if r["set"] == "K" and r["V2"] == 95.0 and r["lo"] == "SPS"), None)
    h_res = [r["res"] for r in st_rows if r["set"] == "H"]
    c3 = c3_map(t2, h_res)
    o("\nConclusions:")
    for r in w:
        o(f"  C1 {r['V2']:g} V {r['P']:.0f} W SPS lower than {r['hi']}: {'stands at 10 ns' if c1_stands(r['res']) else 'does not stand (' + str(r['res']) + ')'}")
    o(f"  C2 no k = 1 valley at 95 V: {'stands' if k is not None and c2_stands(k['res']) else 'broken' if k is not None else 'no 25 ns statement'}"
      + (f" ({k['res']})" if k is not None else ""))
    o(f"  C3 the 25 ns map has converged for its reductions: {'yes' if c3 else 'no'} (T2 {t2}; H statements: "
      + ", ".join(f"{x} {h_res.count(x)}" for x in ("kept", "gone", "reversed", "undecidable")) + ")")
    q1_st = [r for r in st_rows if r["set"] == "Q" or (r["set"] == "W" and r["lo"] == "SPS" and r["hi"] == "M-S")
             or (r["set"] == "W" and r["V2"] == 92.4 and r["lo"] == "SPS")]
    q1_res = [r["res"] for r in q1_st]
    c4 = ", ".join(f"{x} {q1_res.count(x)}" for x in ("kept", "gone", "reversed", "undecidable"))
    o(f"  C4 the map's Q1 'not optimal' statements at 10 ns ({len(q1_st)} measured of 11): {c4}")
    nmiss = int(C["Ipk10"].isna().sum())
    if nmiss:
        o(f"\n{nmiss} configurations not bracketed at 10 ns yet -> provisional")
    open(os.path.join(OUT, "g10_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"kept": "유지", "gone": "사라짐", "reversed": "뒤집힘", "undecidable": "판정 불가"}
    md = ["# 25 ns 지도 진술의 10 ns 재검증 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `grid10_check.py compare`. 규칙은 머리말(10-30 18:22, 10 ns 실행 전 등록)이며 바꾸지 않았다. 모두 Simulink"
          "(데이터셋 시기 모델, 제어기·로컬 솔버 10 ns) 정상상태, 상수 D0 두 계단 전력 보간, 20 ms 창.", "",
          "| 묶음 | V2 | \\|P\\| | 25 ns 진술 | 25 ns 차 | 10 ns 차 | 허용 | 분류 |", "|---|---|---|---|---|---|---|---|"]
    for r in st_rows:
        md.append(f"| {r['set']} | {r['V2']:g} V | {r['P']:.0f} W | {r['lo']} < {r['hi']} | {r['d25']:+.3f} | {r['d10']:+.3f} | {r['tol10']:.3f} | {KO[r['res']]} |")
    md += ["", f"- T2 대표 6점 SPS − B: 25 ns {m25:+.3f} A, 10 ns {m10:+.3f} A → **{'수렴' if t2 == 'converged' else '미수렴' if t2 == 'not converged' else '판정 불가'}**",
           f"- T3 주기 피크 중앙값 변화: 중앙값 {dm.median():.3f} A, 최대 {dm.max():.3f} A, 0.03 A 안 {int((dm <= 0.03).sum())}/{int(dm.notna().sum())}구성"]
    for r in w:
        md.append(f"- C1 {r['V2']:g} V {r['P']:.0f} W \"SPS < {r['hi']}\": **{'10 ns에서도 성립' if c1_stands(r['res']) else '성립 안 함(' + KO[r['res']] + ')'}**")
    if k is not None:
        md.append(f"- C2 95 V k = 1 골 없음: **{'유지' if c2_stands(k['res']) else '깨짐'}** ({KO[k['res']]})")
    md.append(f"- C3 25 ns 지도의 저감은 수렴: **{'그렇다' if c3 else '아니다'}**")
    md.append(f"- C4 지도 Q1 \"최적 아님\" 진술(측정 {len(q1_st)}/11)의 10 ns 분류: {', '.join(f'{KO[x]} {q1_res.count(x)}' for x in KO)} (머리말 22:10 추가, 문턱 없음)")
    if nmiss:
        md.append(f"\n**10 ns에서 아직 감싸지 못한 구성 {nmiss}개 — 잠정.**")
    md += ["", "표 원본 `out/g10_check.csv`(진술), `out/g10_configs.csv`(구성), 로그 `out/g10_check.log`."]
    open(os.path.join(OUT, "g10_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/g10_check.{log,csv}, out/g10_configs.csv, out/g10_verdict.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
