"""
rev6_check.py -- (2026-10-31) goal 3 in the LV -> HV direction on a grid that can resolve it: the question of h12_check.py for this direction.

rev25.py compare (10-31 12:42, 28 map points, 25 ns): one "나쁨", 100.8 V 600 W, rule value only (SPS 2.252, M-B 2.321 A, -0.069 A, tol 0.045; median
-0.030 A, M-B's mean i_L 17 mA); one "판정 불가", 92.4 V 300 W (SPS mean i_L 26 mA; M-B lower by 0.069 A). Both are below one 25 ns grid step (0.106 A).
Nothing has been measured off the map in this direction. HV -> LV has its open candidates at 98.7 V 175, 350 W (h12). In LV -> HV the measured bus
voltage V_LV is 0.1-1.1 V below HV -> LV at the same source V2 and |P| (rev25.py side), so the bridge sees a lower k and the same mechanism, if it is one,
would sit at a slightly higher source V2. The map's 4.2 V spacing skips that.

Runs (verify_steady_fine.m: constant D0, 20 ms, two adjacent levels bracketing |P|, Ipk interpolated at |P|), set F as in h12: Ts = 6.25 ns (V1 Ts / L = 0.026 A,
below the smallest tol), levels 0.72 deg apart, D0 < 0, level j < 0:
  100.8 V 600 W, 92.4 V 300 W, 98.7 V 175, 350 W, 100.8 V 175, 350 W x {SPS, M-B}, the duties of rev25.py. First levels from the Python plant at 6.25 ns with the
  LV bus sag put in (only to pick them). Power = +P_LV. The first bracket going away from D0 = 0 counts.

RULES (registered 2026-10-31 12:54, before any run of this file and before any 6.25 ns value of this direction; do not change after the results). Those of h12_check.py,
the same functions, with the 25 ns values taken from rev25.py:
  A statement "SPS lower than M-B" exists at 25 ns where the difference is above tol = max(0.03 A, 2 % Ipk(SPS)) with both clean, each kind on its own (rule = largest of
     the 80 period peaks, med = their median). The one statement on this list: 100.8 V 600 W, kind rule. 92.4 V 300 W has none (SPS not clean); the off-map points have no 25 ns value.
  T  each statement at 6.25 ns: kept / gone / reversed / undecidable (fine_grid_check.rule_r2, tol from the 6.25 ns SPS).
  C  a point, over the kinds holding a statement: "stands" iff all kept; "does not stand" iff all gone or reversed; else "depends on the value". A point with no statement is
     "new" iff at 6.25 ns SPS is lower than M-B by more than tol in both kinds, both clean; else "none".
  V  "breaks" if a point stands or is new; "gone" if every statement does not stand and nothing is new; else "open".
What the answer means. "breaks": goal 3 fails in LV -> HV on a grid that resolves it: 0절 goal 3 says so with the points; the SPS band that h12 "breaks" would widen must then also
cover this direction. "gone": the one LV -> HV violation is a 25 ns one, and the off-map points measured here hold; 0절 says goal 3 holds at the 28 map points + these points. "open": as it is.
Reported, never a verdict: SPS - M-B at every point on both kinds; spreads; at 98.7 V 175, 350 W and 92.4 V 300 W against HV -> LV at 6.25 ns (out/h12_check.csv) once h12 has run.

    python -I -X utf8 Simulink/Thesis/rev6_check.py selftest
    python -I -X utf8 Simulink/Thesis/rev6_check.py make        -> out/rev6_points_r1.csv, out/rev6_predict.log ([참고] plant)
    python automation/job.py submit rev6 --interactive --after tgt25b -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); rev6_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/rev6_check.py next        (not bracketed -> next round; rev6_rounds calls it)
    python -I -X utf8 Simulink/Thesis/rev6_check.py compare     -> out/rev6_check.{log,csv}, out/rev6_verdict.md
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
import grid10_check as g10  # noqa: E402
import h12_check as h12  # noqa: E402
import r13_map as rm  # noqa: E402
import rev25 as rv  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS25, TS6, GF = rm.TS, h12.TS6, h12.GF
PC = rv.PC
F_PTS = ((100.8, 600.0), (92.4, 300.0), (98.7, 175.0), (98.7, 350.0), (100.8, 175.0), (100.8, 350.0))
NAMES = h12.NAMES
KINDS = h12.KINDS


def configs():
    import surface_opt_check as so
    cf = []
    for V2, P in F_PTS:
        b = so.mb_predict(P, V2)
        b = (sw.D_SAFE, sw.D_SAFE) if max(abs(b[0] - sw.D_SAFE), abs(b[1] - sw.D_SAFE)) <= rm.SAME else (round(b[0], 9), round(b[1], 9))   # as in rev25.configs
        cf.append(dict(set="F", pair="", ctrl="SPS", V2=V2, P=P, D1=sw.D_SAFE, D2=sw.D_SAFE))
        cf.append(dict(set="F", pair="", ctrl="M-B", V2=V2, P=P, D1=b[0], D2=b[1]))
    return cf


def _point(c, m):
    """m = levels away from D0 = 0 in this direction; the run's level j = -m"""
    D0, D1, D2 = fg._safe(-m * GF, c["D1"], c["D2"], TS6)
    return dict(label=f"F {c['ctrl']} {c['P']:g}W {c['V2']:g}V L{-m} T6", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], level=-m, D0=D0, D1=D1, D2=D2, Ts=TS6)


def _levels(cf):
    """(first adjacent pair from below whose plant powers bracket |P|, plant Ipk at |P|); the plant only picks the levels"""
    import plant_full as pf
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS6 * 1e9)
    ms, seen, out = np.arange(-8, 45), {}, []
    for c in cf:
        Vb = c["V2"] - pl.R_lv * c["P"] / c["V2"]                   # the LV source sags in this direction (Simulink: V_LV = V2 - 0.05 ohm x I_LV)
        key = (c["V2"], c["P"], c["D1"], c["D2"])
        if key not in seen:
            sd = [fg._safe(-int(m) * GF, c["D1"], c["D2"], TS6) for m in ms]
            r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]), np.full(len(ms), 380.0), np.full(len(ms), Vb))
            seen[key] = (np.asarray(r["P_LV"], float), np.asarray(r["Ipk"], float))
            print(f"  plant at 6.25 ns: {c['ctrl']} {c['V2']:g} V {c['P']:g} W done", flush=True)
        Pp, Ip = seen[key]
        b = rm.first_bracket(ms, Pp, c["P"])
        if b is None:
            sys.exit(f"plant does not bracket {c}")
        i = int(b[0] - ms[0])
        out.append((b, Ip[i] + (c["P"] - Pp[i]) / (Pp[i + 1] - Pp[i]) * (Ip[i + 1] - Ip[i])))
    return out


def results_6():
    fs = [f for f in sorted(glob.glob(os.path.join(OUT, "rev6_sweep_r*.csv"))) if not f.endswith("_in.csv")]
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["Ts", "V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "rev6_sweep_r*.csv"))
          for m in [re.search(r"rev6_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def make():
    cf, pts, lines = configs(), [], ["[참고] Python plant at 6.25 ns, LV bus sag put in, Simulink not run yet: Ipk at |P| and SPS - M-B", ""]
    lv = _levels(cf)
    for c, (b, _) in zip(cf, lv):
        pts += [_point(c, int(m)) for m in b]
    for i in range(0, len(cf), 2):
        c, a, b = cf[i], lv[i][1], lv[i + 1][1]
        lines.append(f"{c['V2']:6g} V {c['P']:5.0f} W: SPS {a:.3f}, M-B {b:.3f} A, SPS - M-B {a - b:+.3f} A; M-B duties ({cf[i + 1]['D1']:.4f}, {cf[i + 1]['D2']:.4f}); levels m {lv[i][0]} {lv[i + 1][0]}")
    fg._write(os.path.join(OUT, "rev6_points_r1.csv"), pts)
    open(os.path.join(OUT, "rev6_predict.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def _mirror(d, c, Ts):
    g = fg.runs_of(d, c, Ts) if d is not None else None
    if g is None or g.empty:
        return None
    return g.assign(**{"level": -g["level"], PC: -g[PC]}).sort_values("level")


def next_round():
    d = results_6()
    if d is None:
        sys.exit("no out/rev6_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = _mirror(d, c, TS6)
        if g is None:
            sys.exit(f"no runs at all for {c} -- round 1 not finished?")
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [_point(c, int(m)) for m in rm.extend(g["level"].to_numpy(), -g[PC].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"rev6_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


def _value(d, pp, c, Ts):
    """at |P| in this direction on one grid -> dict(I rule value, osc, lo level, spread (larger of the two levels), med interpolated at |P|)"""
    nan = dict(I=np.nan, osc=np.nan, lo=None, spread=np.nan, med=np.nan)
    g = _mirror(d, c, Ts)
    if g is None:
        return nan
    a = fg.at_power(g, c["P"])
    r = dict(I=a[0], osc=a[1], lo=a[2], spread=np.nan, med=np.nan)
    if a[2] is None:
        return r
    rows = [g[g["level"] == m].iloc[0] for m in (a[2], a[2] + 1)]
    x = [pp.get((round(c["V2"], 6), -m, round(float(q["D1"]), 9), round(float(q["D2"]), 9))) for m, q in zip((a[2], a[2] + 1), rows)]
    if any(e is None for e in x):
        return r
    Pd = [float(-q[PC]) for q in rows]
    t = (c["P"] - Pd[0]) / (Pd[1] - Pd[0])
    r.update(spread=max(e[0] - e[2] for e in x), med=x[0][1] + t * (x[1][1] - x[0][1]))
    return r


def _forward():
    """(V2, P) -> {kind: SPS - M-B} of HV -> LV at 6.25 ns, once h12 has run"""
    import pandas as pd
    p = os.path.join(OUT, "h12_check.csv")
    if not os.path.isfile(p):
        return {}
    T = pd.read_csv(p)
    T = T[T["set"] == "F"]
    out = {}
    for (v, q), g in T.groupby(["V2", "P"]):
        s, b = g[g["ctrl"] == "SPS"].iloc[0], g[g["ctrl"] == "M-B"].iloc[0]
        out[(v, q)] = {kind: s[f"{k}6.25"] - b[f"{k}6.25"] for kind, k in KINDS}
    return out


def compare(data=None, write_out=True):
    """data: (d25, pp25, d6, pp6) for the self-test"""
    import pandas as pd
    d25, pp25, d6, pp6 = data if data else (rv.results(), g10._pp("rev25_sweep_r*.csv", TS25), results_6(), g10._pp("rev6_sweep_r*.csv", TS6))
    if d6 is None:
        sys.exit("no rev6 runs yet")
    lines, rows, found, finals, perF = [], [], [], {}, {}

    def o(x=""):
        print(x, flush=True)
        lines.append(x)
    o("rev6_check: M-B goal 3 in LV -> HV at 6.25 ns, 0.72 deg levels; rules in the file header (those of h12_check.py)")
    o(f"{'V2':>6} {'|P|':>5} {'ctrl':>4} | Ipk {'25':>6} {'6.25':>6} | median {'25':>6} {'6.25':>6} | spread {'25':>6} {'6.25':>6} | mean i_L {'25':>6} {'6.25':>6}")
    for c in configs():
        w = {"25": _value(d25, pp25, c, TS25), "6.25": _value(d6, pp6, c, TS6)}
        perF.setdefault((c["V2"], c["P"]), {})[c["ctrl"]] = w
        o(f"{c['V2']:6g} {c['P']:5.0f} {c['ctrl']:>4} | Ipk {w['25']['I']:6.3f} {w['6.25']['I']:6.3f} | median {w['25']['med']:6.3f} {w['6.25']['med']:6.3f} | "
          f"spread {w['25']['spread']:6.3f} {w['6.25']['spread']:6.3f} | mean i_L {w['25']['osc']:6.3f} {w['6.25']['osc']:6.3f}")
        rows.append(dict(V2=c["V2"], P=c["P"], ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], **{f"{k}{g}": w[g][k] for g in w for k in ("I", "med", "osc", "spread")}))
    o("\nStatements of the 25 ns values at 6.25 ns")
    fw = _forward()
    summ = []
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
        c = h12.rule_c(res)
        finals[(v, p)] = c if c else ("new" if h12.is_new(v6) else "none")
        dr, dm = v6["rule"]["SPS"][0] - v6["rule"]["M-B"][0], v6["med"]["SPS"][0] - v6["med"]["M-B"][0]
        f = fw.get((v, p))
        summ.append(dict(V2=v, P=p, c=finals[(v, p)], dr=dr, dm=dm, f=f))
        o(f"C  {v:g} V {p:.0f} W: {finals[(v, p)]}; SPS - M-B at 6.25 ns: rule {dr:+.3f} A, median {dm:+.3f} A" + (f"; HV -> LV at 6.25 ns: rule {f['rule']:+.3f}, median {f['med']:+.3f} A" if f else ""))
    miss = sum(not np.isfinite(w[n]["6.25"]["I"]) for w in perF.values() for n in NAMES)
    verdict = h12.rule_v(finals.values())
    o(f"\nVERDICT: {verdict}" + (f" (PROVISIONAL: {miss} values not bracketed yet)" if miss else ""))
    if not write_out:
        return verdict, finals
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "rev6_check.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "rev6_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"breaks": "LV→HV에서 목표 3이 깨진다", "gone": "LV→HV 진술은 25 ns의 것", "open": "열림(값에 따라 다름)"}
    CK = {"stands": "성립", "does not stand": "성립 안 함", "depends on the value": "값에 따라 다름", "new": "새 위반", "none": "진술 없음"}
    RK = {"kept": "유지", "gone": "사라짐", "reversed": "뒤집힘", "undecidable": "판정 불가"}
    md = ["# LV→HV 목표 3을 격자 한 스텝이 허용보다 작은 격자로 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `rev6_check.py compare`. 규칙은 머리말(10-31 12:54 실행 전 등록, `h12_check.py`와 같은 함수)이며 바꾸지 않았다. 모두 Simulink, 6.25 ns, 0.72° 계단, 상수 D0 20 ms.", "",
          f"**판정: {KO[verdict]}**" + (" — 잠정" if miss else ""), "",
          "| 점 | 값 | 25 ns 진술 | 25 ns 차 [A] | 6.25 ns 차 [A] | 허용 [A] | 6.25 ns |", "|---|---|---|---|---|---|---|"]
    for x in found:
        md.append(f"| {x['V2']:g} V {x['P']:.0f} W | {'80주기 최대' if x['kind'] == 'rule' else '주기 중앙값'} | {x['lo']} < {x['hi']} | {x['d25']:+.3f} | {x['d6']:+.3f} | {x['tol']:.3f} | {RK[x['res']]} |")
    md += ["", "| 점 | 분류 | SPS − B, 6.25 ns, 80주기 최대 [A] | 주기 중앙값 [A] | HV→LV 6.25 ns 최대 / 중앙값 [A] |", "|---|---|---|---|---|"]
    for s in summ:
        md.append(f"| {s['V2']:g} V {s['P']:.0f} W | {CK[s['c']]} | {s['dr']:+.3f} | {s['dm']:+.3f} | " + (f"{s['f']['rule']:+.3f} / {s['f']['med']:+.3f}" if s["f"] else "—") + " |")
    md += ["", "표 원본 `out/rev6_check.csv`, 로그 `out/rev6_check.log`."]
    open(os.path.join(OUT, "rev6_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/rev6_check.{log,csv}, out/rev6_verdict.md")
    return verdict, finals


def selftest():
    """the registered rule end to end on made-up runs"""
    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    cf = configs()
    chk("6 points x 2 controllers", len(cf), 12)
    chk("the rev25 duties on the map", [(c["D1"], c["D2"]) for c in cf if c["V2"] == 100.8 and c["P"] == 600.0], [(c["D1"], c["D2"]) for c in rv.configs() if c["V2"] == 100.8 and c["P"] == 600.0])
    p = _point(cf[0], 8)
    chk("level -8 is D0 < 0 at 0.72 deg", (p["level"], abs(p["D0"] + 8 * GF) < 1e-3), (-8, True))
    chk("grid step below the smallest tol", 380.0 * TS6 / 90e-6 < 0.03, True)

    def runs(Ts, base, dI):
        """dI: (V2, P) -> Ipk(M-B) - Ipk(SPS); two levels per configuration bracketing |P| + a decoy further out"""
        out, pp = [], {}
        for c in cf:
            m0, d = int(c["P"] // 50) + 3, dI.get((c["V2"], c["P"]), -0.1) if c["ctrl"] == "M-B" else 0.0
            for m, Pr in ((m0, c["P"] - 25.0), (m0 + 1, c["P"] + 25.0), (m0 + 30, c["P"] - 25.0), (m0 + 31, c["P"] + 25.0)):
                I = base + d + (5.0 if m > m0 + 1 else 0.0)
                out.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=-m, D0=-m * GF, D1=c["D1"], D2=c["D2"], Ts=Ts, **{PC: Pr, rv.IC: I}, iL_mean=0.001, P_LV=Pr, P_HV=Pr - 3.0, error=""))
                pp[(round(c["V2"], 6), -m, round(c["D1"], 9), round(c["D2"], 9))] = (I, I, I)
        return pd.DataFrame(out), pp
    d25, pp25 = runs(TS25, 2.0, {(100.8, 600.0): 0.1})
    d6, pp6 = runs(TS6, 2.0, {(100.8, 600.0): 0.0})
    v, fin = compare((d25, pp25, d6, pp6), write_out=False)
    chk("the statement goes -> gone", v, "gone")
    chk("the statement's point", fin[(100.8, 600.0)], "does not stand")
    d6, pp6 = runs(TS6, 2.0, {(100.8, 600.0): 0.1})
    v, fin = compare((d25, pp25, d6, pp6), write_out=False)
    chk("the statement stays -> breaks", (v, fin[(100.8, 600.0)]), ("breaks", "stands"))
    d6, pp6 = runs(TS6, 2.0, {(98.7, 175.0): 0.1})
    v, fin = compare((d25, pp25, d6, pp6), write_out=False)
    chk("an off-map point above SPS at 6.25 ns -> new, breaks", (v, fin[(98.7, 175.0)]), ("breaks", "new"))
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
