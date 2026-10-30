"""
h12_check.py -- (2026-10-31) goal 3 for the main surface M-B: do the "SPS lower than M-B" statements survive a grid that can resolve them?

md_check.py compare (10-31 08:06, the final M-D verdict "no gain") measured M-B on 48 points that were never anchors. M-B is above
SPS by more than tol at two of them, both at 98.7 V and in both kinds:
   175 W  rule 1.120 vs 1.020 A (+0.099), median 1.068 vs 1.020 A (+0.048)
   350 W  rule 1.591 vs 1.530 A (+0.062), median 1.590 vs 1.530 A (+0.061); both runs periodic
M-D does not fix them (it keeps M-B's duties there). g12_check.py re-measures the map's goal 3 statements (92.4 V 300 W, 105 V 400 W)
but not these, and 0.05-0.06 A is half of one 25 ns grid step (V1 Ts / L = 0.106 A). The map's statements did not hold at 10 ns, so these
may not either. Across the whole 98.7 V slice the median saving is -0.006, -0.048, -0.016, -0.061, +0.019, +0.021 A (125 ... 900 W).

Runs (verify_steady_fine.m: constant D0, 20 ms, two adjacent levels bracketing |P|, Ipk interpolated at |P|):
  F  the decisive set, Ts = 6.25 ns (V1 Ts / L = 0.026 A, the first grid whose step is below the smallest tol 0.03 A; tp 4.8 and tp + td 20.8
     steps, so no coincidence as at 10 ns), levels 0.72 deg apart (about 60 W, against 240-290 W for the map's 2.88 deg):
     98.7 V 175, 250, 350 W, 92.4 V 300 W, 105 V 400 W x {SPS, M-B}. First levels from the Python plant at 6.25 ns (only to pick them).
  S  the map's method at the grids of g12, Ts = 12.5 ns and 10 ns, 2.88 deg levels (first levels = the 25 ns bracket):
     98.7 V x |P| 125, 175, 250, 350, 500 W x {SPS, M-B}. Read beside g12; never decides here.

RULES (registered 2026-10-31 08:10 with S alone, before any of these runs; amended 08:36, still before any run: F added and made decisive,
because S cannot decide. The steps of 12.5 and 10 ns, 0.053 and 0.042 A, are still above tol. No fine-grid value of these points existed
at either time. Do not change after the results.)
  Values, tol, clean, statements: as grid10_check.py. A statement "SPS lower than M-B" exists at 25 ns where the difference is above
     tol = max(0.03 A, 2 % Ipk(SPS)) with both clean. Each kind on its own: "rule" = largest of the 80 period peaks, "med" = their median.
     The four 25 ns statements on this list: 98.7 V 175 W, 350 W (both kinds), 92.4 V 300 W (both kinds), 105 V 400 W (rule only).
  T  each statement, kind by kind, at 6.25 ns: kept / gone / reversed / undecidable (fine_grid_check.rule_r2, tol from the 6.25 ns SPS).
  C  a point, over the kinds that hold a statement: "stands" iff all kept; "does not stand" iff all gone or reversed; else "depends on the value".
     A point with no 25 ns statement is "new" iff at 6.25 ns SPS is lower than M-B by more than tol in both kinds, both clean; else "none".
  V  the answer: "breaks" if a point stands or is new; "gone" if every statement does not stand and nothing is new; else "open".
     "breaks": goal 3 fails on a grid that can resolve it. A surface with a wider SPS band is then the next thing to design and judge,
     with its own rules. "gone": the statements are 25 ns ones and only the record of them stays. "open": reported as it is.
  Reported, never a verdict: the S slice at 25, 12.5, 10 ns with rule_r2 and g12's rule_f; spreads; SPS - M-B at every point of F and over
     the S slice per grid and kind.

    python -I -X utf8 Simulink/Thesis/h12_check.py selftest
    python -I -X utf8 Simulink/Thesis/h12_check.py make        -> out/h12_points_r1.csv
    python automation/job.py submit h12 --interactive --after edge25 -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); h12_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/h12_check.py next        (not bracketed -> next round; h12_rounds calls it)
    python -I -X utf8 Simulink/Thesis/h12_check.py compare     -> out/h12_check.{log,csv}, out/h12_verdict.md
"""
import argparse
import datetime
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `make` runs beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import g12_check as g12  # noqa: E402
import grid10_check as g10  # noqa: E402
import md_check as md  # noqa: E402
import r13_map as rm  # noqa: E402

OUT = os.path.join(HERE, "out")
TS25, TS12, TS10, TS6 = rm.TS, g12.TS12, g10.TS10, 6.25e-9
WIN = fg.WIN
GF = 0.72                                   # deg: the level spacing of set F, four 6.25 ns steps at 80 kHz
V2S = 98.7
PS = (125.0, 175.0, 250.0, 350.0, 500.0)
F_PTS = ((98.7, 175.0), (98.7, 250.0), (98.7, 350.0), (92.4, 300.0), (105.0, 400.0))
NAMES = ("SPS", "M-B")
KINDS = (("rule", "I"), ("med", "med"))
FINE = (("12.5", TS12), ("10", TS10))


def _table():
    """(V2, |P|) -> candidates of md_check.py (hold-out) and of the map, same duties as the 25 ns runs"""
    tab = {(v, p): cs for v, p, _, cs in md.hold_cands()}
    tab.update({(v, p): cs for v, p, cs in rm.candidates("map")})
    return tab


def _cfg(tab, st, v, p, nm):
    by = {c["name"]: c for c in tab[(v, p)]}
    c = by[nm]
    own = by[c["alias"]] if c["alias"] else c
    return dict(set=st, pair=int(p), ctrl=nm, V2=v, P=p, D1=own["D1"], D2=own["D2"])


def configs():
    tab = _table()
    return [_cfg(tab, "S", V2S, p, nm) for p in PS for nm in NAMES]


def configs_f():
    tab = _table()
    return [_cfg(tab, "F", v, p, nm) for v, p in F_PTS for nm in NAMES]


def _point_f(c, j):
    D0, D1, D2 = fg._safe(j * GF, c["D1"], c["D2"], TS6)
    return dict(label=f"F {c['ctrl']} {c['P']:g}W {c['V2']:g}V L{j} T6", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], level=j, D0=D0, D1=D1, D2=D2, Ts=TS6)


def _levels_f(cf):
    """the first adjacent pair from below whose plant powers bracket |P|; the plant only picks the levels"""
    import plant_full as pf
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS6 * 1e9)
    js, seen, out = np.arange(-4, 44), {}, []
    for c in cf:
        key = (c["V2"], c["D1"], c["D2"])
        if key not in seen:
            sd = [fg._safe(int(j) * GF, c["D1"], c["D2"], TS6) for j in js]
            r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]),
                                np.full(len(js), 380.0), np.full(len(js), c["V2"]))
            seen[key] = -np.asarray(r["P_LV"], float)
            print(f"  plant at 6.25 ns: {c['ctrl']} {c['V2']:g} V done", flush=True)
        b = rm.first_bracket(js, seen[key], c["P"])
        if b is None:
            sys.exit(f"plant does not bracket {c}")
        out.append(b)
    return out


def results_fine():
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "h12_sweep_r*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "h12_sweep_r*.csv"))
          for m in [re.search(r"h12_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def make():
    cf, pts = configs_f(), []
    for c, b in zip(cf, _levels_f(cf)):
        pts += [_point_f(c, int(j)) for j in b]
    d25 = rm.results_25(md.PATS)
    for c in configs():
        lo = fg.at_power(rm._runs(d25, c), c["P"])[2]
        if lo is None:
            sys.exit(f"no 25 ns bracket for {c}")
        for _, ts in FINE:
            pts += [fg._point(c, j, ts) for j in (lo, lo + 1)]
    fg._write(os.path.join(OUT, "h12_points_r1.csv"), pts)


def next_round():
    d = results_fine()
    if d is None:
        sys.exit("no h12 runs yet")
    pts = []

    def more(c, ts):
        g = fg.runs_of(d, c, ts)
        if g.empty or np.isfinite(fg.at_power(g, c["P"])[0]):
            return []
        return [int(j) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    for c in configs_f():
        pts += [_point_f(c, j) for j in more(c, TS6)]
    for c in configs():
        for _, ts in FINE:
            pts += [fg._point(c, j, ts) for j in more(c, ts)]
    if pts:
        fg._write(os.path.join(OUT, f"h12_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def rule_c(res):
    """the T classes of one point's statements -> stands / does not stand / depends on the value; None if there is no statement"""
    res = list(res)
    if not res:
        return None
    if all(r == "kept" for r in res):
        return "stands"
    if all(r in ("gone", "reversed") for r in res):
        return "does not stand"
    return "depends on the value"


def is_new(vals6):
    """no 25 ns statement at this point: at 6.25 ns SPS lower than M-B beyond tol in both kinds, both clean. vals6: kind -> {name: (Ipk, osc)}"""
    for v in vals6.values():
        if not (g10.clean(v["SPS"]) and g10.clean(v["M-B"])) or v["M-B"][0] - v["SPS"][0] <= g10.tol_of(v):
            return False
    return True


def rule_v(cs):
    """the points' C classes ('new' and 'none' included) -> breaks / gone / open"""
    cs = list(cs)
    if any(c in ("stands", "new") for c in cs):
        return "breaks"
    st = [c for c in cs if c != "none"]
    if st and all(c == "does not stand" for c in st):
        return "gone"
    return "open"


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("C all kept", rule_c(["kept", "kept"]), "stands")
    chk("C one kind only", rule_c(["kept"]), "stands")
    chk("C gone and reversed", rule_c(["gone", "reversed"]), "does not stand")
    chk("C differ", rule_c(["kept", "gone"]), "depends on the value")
    chk("C undecidable", rule_c(["gone", "undecidable"]), "depends on the value")
    chk("C no statement", rule_c([]), None)
    v = lambda a, b: {"SPS": (a, 0.0), "M-B": (b, 0.0)}  # noqa: E731
    chk("new both kinds", is_new({"rule": v(1.0, 1.1), "med": v(1.0, 1.1)}), True)
    chk("new one kind only", is_new({"rule": v(1.0, 1.1), "med": v(1.0, 1.01)}), False)
    chk("new not clean", is_new({"rule": {"SPS": (1.0, 0.0), "M-B": (1.1, 0.03)}, "med": v(1.0, 1.1)}), False)
    chk("V stands", rule_v(["stands", "does not stand", "none"]), "breaks")
    chk("V new", rule_v(["does not stand", "new"]), "breaks")
    chk("V all gone", rule_v(["does not stand", "does not stand", "none"]), "gone")
    chk("V open", rule_v(["depends on the value", "does not stand", "none"]), "open")
    chk("V nothing", rule_v(["none"]), "open")
    cf, cs = configs_f(), configs()
    chk("configurations F, S", (len(cf), len(cs)), (10, 10))
    chk("M-B off SPS at every point", all(c["D1"] < 0.4999 or c["D2"] < 0.4999 for c in cf + cs if c["ctrl"] == "M-B"), True)
    chk("SPS duties", {(c["D1"], c["D2"]) for c in cf + cs if c["ctrl"] == "SPS"}, {(rm.sw.D_SAFE, rm.sw.D_SAFE)})
    chk("F point on the Ts grid after nudge", abs(_point_f(cf[0], 8)["D0"] - 8 * GF) < 1e-3, True)
    chk("grid step below the smallest tol", 380.0 * TS6 / 90e-6 < 0.03, True)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def _a1():
    """g12's verdict on the 10 ns flicker, from its own rule and runs; None before g12 has run"""
    d12, d10 = g12.results_12(), g10.results_10()
    if d12 is None:
        return None
    _, flags = g12._a_table(g12._pp_all(), d10, lambda *_: None)
    return g12.rule_a1(flags.get("tp35", []), flags.get("12.5", []))


def compare():
    import pandas as pd
    d25, d = rm.results_25(md.PATS), results_fine()
    if d is None:
        sys.exit("no h12 runs yet")
    pp25 = {}
    for pat in md.PATS:
        pp25.update(g10._pp(pat, TS25))
    pp = {"25": pp25, "12.5": g10._pp("h12_sweep_r*.csv", TS12), "10": g10._pp("h12_sweep_r*.csv", TS10), "6.25": g10._pp("h12_sweep_r*.csv", TS6)}
    data = {"25": (d25, TS25), "12.5": (d, TS12), "10": (d, TS10), "6.25": (d, TS6)}
    lines, rows = [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    def val(c, gs):
        return {g: g12._value(c, data[g][0], pp[g], data[g][1]) for g in gs}
    o("h12_check: M-B goal 3 statements; rules in the file header. F = decisive: 6.25 ns, 0.72 deg levels. S = reported: 98.7 V slice at 12.5 and 10 ns, 2.88 deg levels")
    o(f"{'V2':>6} {'|P|':>5} {'ctrl':>4} | Ipk {'25':>6} {'6.25':>6} | median {'25':>6} {'6.25':>6} | spread {'25':>6} {'6.25':>6} | mean i_L {'25':>6} {'6.25':>6}")
    perF = {}
    for c in configs_f():
        w = val(c, ("25", "6.25"))
        perF.setdefault((c["V2"], c["P"]), {})[c["ctrl"]] = w
        o(f"{c['V2']:6g} {c['P']:5.0f} {c['ctrl']:>4} | Ipk {w['25']['I']:6.3f} {w['6.25']['I']:6.3f} | median {w['25']['med']:6.3f} {w['6.25']['med']:6.3f} | "
          f"spread {w['25']['spread']:6.3f} {w['6.25']['spread']:6.3f} | mean i_L {w['25']['osc']:6.3f} {w['6.25']['osc']:6.3f}")
        rows.append(dict(set="F", V2=c["V2"], P=c["P"], ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], **{f"{k}{g}": w[g][k] for g in w for k in ("I", "med", "osc", "spread")}))
    o("\nDecisive: F, statements of the 25 ns values at 6.25 ns")
    found, finals = [], {}
    for (v, p), vals in perF.items():
        res, v6 = [], {}
        for kind, k in KINDS:
            pair = lambda g: {n: (vals[n][g][k], vals[n][g]["osc"]) for n in NAMES}  # noqa: E731, B023
            v6[kind] = pair("6.25")
            for r in g10.statements(pair("25"), pair("6.25")):
                found.append(dict(V2=v, P=p, kind=kind, lo=r["lo"], hi=r["hi"], d25=r["d25"], d6=r["d10"], tol=r["tol10"], res=r["res"]))
                o(f"   {v:6g} V {p:5.0f} W {kind:>4}: {r['lo']} lower than {r['hi']} by {r['d25']:+.3f} A at 25 ns -> {r['d10']:+.3f} A at 6.25 ns (tol {r['tol10']:.3f}): {r['res']}")
                if r["lo"] == "SPS":
                    res.append(r["res"])
        c = rule_c(res)
        finals[(v, p)] = c if c else ("new" if is_new(v6) else "none")
        o(f"C  {v:g} V {p:.0f} W: {finals[(v, p)]}; SPS - M-B at 6.25 ns: rule {v6['rule']['SPS'][0] - v6['rule']['M-B'][0]:+.3f} A, median {v6['med']['SPS'][0] - v6['med']['M-B'][0]:+.3f} A")
    miss = sum(not np.isfinite(w[n]["6.25"]["I"]) for w in perF.values() for n in NAMES)
    verdict = rule_v(finals.values())
    o(f"\nVERDICT: {verdict}" + (f" (PROVISIONAL: {miss} values not bracketed yet)" if miss else ""))
    a1 = _a1()
    o(f"\nReported, not a verdict: S, {V2S:g} V slice at 25, 12.5, 10 ns, 2.88 deg levels; A1 from g12: {a1}")
    o(f"{'|P|':>5} {'ctrl':>4} | Ipk {'25':>6} {'12.5':>6} {'10':>6} | median {'25':>6} {'12.5':>6} {'10':>6} | spread {'25':>6} {'12.5':>6} {'10':>6} | mean i_L {'25':>6} {'12.5':>6} {'10':>6}")
    perS, foundS = {}, []
    for c in configs():
        w = val(c, ("25", "12.5", "10"))
        perS.setdefault(c["P"], {})[c["ctrl"]] = w
        o(f"{c['P']:5.0f} {c['ctrl']:>4} | Ipk {w['25']['I']:6.3f} {w['12.5']['I']:6.3f} {w['10']['I']:6.3f} | median {w['25']['med']:6.3f} {w['12.5']['med']:6.3f} {w['10']['med']:6.3f} | "
          f"spread {w['25']['spread']:6.3f} {w['12.5']['spread']:6.3f} {w['10']['spread']:6.3f} | mean i_L {w['25']['osc']:6.3f} {w['12.5']['osc']:6.3f} {w['10']['osc']:6.3f}")
        rows.append(dict(set="S", V2=V2S, P=c["P"], ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], **{f"{k}{g}": w[g][k] for g in w for k in ("I", "med", "osc", "spread")}))
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "h12_check.csv"), index=False, float_format="%.6g")
    for kind, k in KINDS:
        s = []
        for g in ("25", "12.5", "10"):
            x = [perS[p]["SPS"][g][k] - perS[p]["M-B"][g][k] for p in PS if all(g10.clean((perS[p][n][g][k], perS[p][n][g]["osc"])) for n in NAMES)]
            s.append(f"{g} ns {np.mean(x):+.3f} A over {len(x)} points" if x else f"{g} ns no clean point")
        o(f"   SPS - M-B over the slice, {kind}: " + ", ".join(s))
    for p in PS:
        for kind, k in KINDS:
            pair = lambda g: {n: (perS[p][n][g][k], perS[p][n][g]["osc"]) for n in NAMES}  # noqa: E731, B023
            for a, b in zip(g10.statements(pair("25"), pair("12.5")), g10.statements(pair("25"), pair("10"))):
                f = g12.rule_f(a1, a["res"], b["res"]) if a1 else None
                foundS.append(dict(P=p, kind=kind, lo=a["lo"], hi=a["hi"], d25=a["d25"], d12=a["d10"], d10=b["d10"], res12=a["res"], res10=b["res"], f=f))
                o(f"   {p:5.0f} W {kind:>4}: {a['lo']} lower than {a['hi']} by {a['d25']:+.3f} A -> 12.5 ns {a['d10']:+.3f} (tol {a['tol10']:.3f}) {a['res']}, 10 ns {b['d10']:+.3f} (tol {b['tol10']:.3f}) {b['res']} -> g12 F: {f}")
    open(os.path.join(OUT, "h12_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"breaks": "목표 3이 깨진다", "gone": "진술들은 25 ns의 것", "open": "열림(값에 따라 다름)"}
    CK = {"stands": "성립", "does not stand": "성립 안 함", "depends on the value": "값에 따라 다름", "new": "새 위반", "none": "진술 없음"}
    RK = {"kept": "유지", "gone": "사라짐", "reversed": "뒤집힘", "undecidable": "판정 불가"}
    FK = {"stands": "성립", "does not stand": "성립 안 함", "grid dependent": "격자에 따라 다름", None: "g12 대기"}
    md_ = ["# 주 곡면 B의 목표 3 진술을 격자 한 스텝이 허용보다 작은 격자로 (자동 생성)", "",
           f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `h12_check.py compare`. 규칙은 머리말(10-31 08:10 등록, 08:36 F 추가, 모두 실행 전)이며 바꾸지 않았다.", "",
           f"**판정: {KO[verdict]}**" + (" — 잠정" if miss else ""), "",
           "| 점 | 값 | 25 ns 진술 | 25 ns 차 [A] | 6.25 ns 차 [A] | 허용 [A] | 6.25 ns |", "|---|---|---|---|---|---|---|"]
    for x in found:
        md_.append(f"| {x['V2']:g} V {x['P']:.0f} W | {'80주기 최대' if x['kind'] == 'rule' else '주기 중앙값'} | {x['lo']} < {x['hi']} | {x['d25']:+.3f} | {x['d6']:+.3f} | {x['tol']:.3f} | {RK[x['res']]} |")
    md_ += [""] + [f"- {v:g} V {p:.0f} W: **{CK[c]}**" for (v, p), c in finals.items()]
    md_ += ["", f"보고만: 98.7 V 2.88° 계단, 12.5·10 ns(g12 A1: {a1})", "", "| \\|P\\| | 값 | 25 ns 진술 | 25 ns 차 | 12.5 ns 차 | 12.5 ns | 10 ns 차 | 10 ns | g12 규칙 F |", "|---|---|---|---|---|---|---|---|---|"]
    for x in foundS:
        md_.append(f"| {x['P']:.0f} W | {'80주기 최대' if x['kind'] == 'rule' else '주기 중앙값'} | {x['lo']} < {x['hi']} | {x['d25']:+.3f} | {x['d12']:+.3f} | {RK[x['res12']]} | {x['d10']:+.3f} | {RK[x['res10']]} | {FK[x['f']]} |")
    md_ += ["", "표 원본 `out/h12_check.csv`, 로그 `out/h12_check.log`."]
    open(os.path.join(OUT, "h12_verdict.md"), "w", encoding="utf-8").write("\n".join(md_) + "\n")
    print("wrote out/h12_check.{log,csv}, out/h12_verdict.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
