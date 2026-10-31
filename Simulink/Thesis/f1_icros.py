"""
f1_icros.py -- (2026-10-31, final deadline F1 / V1) the ICROS2026 number "largest reduction 3.392 A" against Simulink at its presumed operating point.

The source of 3.392 A is not stored anywhere (the trainer that printed it was overwritten; REPORT 6.7 presumes I_SPS(dataset) - I_Model at V2 110 V, light load, HV -> LV: the dataset's
largest I_SPS - I_Opt is at -150 W 110 V). "Unreachable" so far rests on the ideal model (icros_max_check.py) and on 109.2 V Simulink SPS values. Here the point itself is measured.

Runs (the 25 ns map's method: verify_steady_fine.m, constant D0, 20 ms, two adjacent 2.88 deg levels, Ipk interpolated at |P|, HV -> LV, V1 380 V, 25 ns):
  V2 110 V (just above the map's 109.2 V; M-B's input clips at 109.2 V) x |P| {150, 200} W x
    SPS; M-B; DATA = the dataset's own row (Opt_D1, Opt_D2); ICROS = the re-implemented RegHDC's duties (model_legacy_icros.npz); IDEAL = the ideal model's optimum on 0.0025 cells;
    the grid D1 {0.20, 0.25, 0.30, 0.35} x D2 {0.15, 0.20, 0.25, 0.30}.
  The Python plant only picks first levels; `next` extends. A configuration whose power cannot reach |P| is left out and reported.

RULES (registered 2026-10-31 14:41 before any run of this file; do not change after the results). Each value kind on its own: rule = largest of the 80 period peaks, med = their median.
  clean: |mean i_L| <= 20 mA in both bracketing runs, as the map.
  B1  "Ipk(SPS) < 3.392 A" at each power and kind, with SPS clean. A reduction cannot exceed the peak it starts from.
  B2  dI_max = Ipk(SPS) - min over clean configurations of Ipk, each power and kind (with the duties); the Simulink counterpart of the claim.
  B3  min Ipk over clean configurations against the current 3.392 A would need from the dataset's SPS row (I_SPS(dataset) - 3.392 = 1.344 A at 150 W, 1.515 A at 200 W): is it measured?
  V   "도달 불가" if B1 holds at both powers in both kinds. "배제 못 함" if not B1 somewhere and dI_max < 3.392 A everywhere. "도달" if dI_max >= 3.392 A anywhere (the user's side is right,
      and the sentence changes). "판정 불가" if an SPS value is not clean or not bracketed at the end.
What the answer means. "도달 불가": REPORT 0절 and the final report say, as [S], that at V2 110 V, 150 and 200 W, HV -> LV, 25 ns, SPS itself peaks below 3.392 A, so the number cannot be a steady-state
  reduction there; that its source and operating point remain presumed stays beside it. Nothing is said about other operating points beyond the map's values.
Reported, never a verdict: dataset I_SPS / Simulink SPS, Simulink Ipk at the dataset's duty against its I_Opt, at M-B, ICROS and IDEAL.

    python -I -X utf8 Simulink/Thesis/f1_icros.py selftest
    python -I -X utf8 Simulink/Thesis/f1_icros.py make        -> out/f1icros_points_r1.csv
    python automation/job.py submit f1icros --interactive --after f1eff -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); f1icros_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/f1_icros.py next        (f1icros_rounds calls it)
    python -I -X utf8 Simulink/Thesis/f1_icros.py compare     -> out/f1icros_check.{csv,log}, out/f1icros_verdict.md
"""
import argparse
import datetime
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `make` / `next` run beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import edge25 as eg  # noqa: E402
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import robust_check as rc  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS, G = rm.TS, fg.G
V2 = 110.0
PS = (150.0, 200.0)
CLAIM = 3.392
ICROS_CSV = os.path.join(ROOT, "Simulink", "RegHDC_PostProcessed_Dataset.csv")
IDEAL = {150.0: (0.2425, 0.2025), 200.0: (0.2800, 0.2325)}    # out/icros_max_check.log (ideal model, 0.0025 cells); only chooses a duty to run
GRID_D1, GRID_D2 = (0.20, 0.25, 0.30, 0.35), (0.15, 0.20, 0.25, 0.30)
PC, IC = eg.PC, eg.IC


def dataset_row(P):
    import pandas as pd
    d = pd.read_csv(ICROS_CSV)
    r = d[(d["Target_V2_V"] == 110) & (d["Target_P_W"] == -P)].iloc[0]
    return dict(D1=float(r["Opt_D1"]), D2=float(r["Opt_D2"]), I_SPS=float(r["I_SPS"]), I_Opt=float(r["I_Opt"]))


def configs():
    import r12_ramp as r12
    import surface_opt_check as so
    cf = []

    def add(name, P, a, b):
        cf.append(dict(set="I", pair="", ctrl=name, V2=V2, P=P, D1=round(min(float(a), sw.D_SAFE), 9), D2=round(min(float(b), sw.D_SAFE), 9)))
    for P in PS:
        add("SPS", P, sw.D_SAFE, sw.D_SAFE)
        add("M-B", P, *so.mb_predict(P, V2))
        r = dataset_row(P)
        add("DATA", P, r["D1"], r["D2"])
        d1, d2 = r12.duties("ICROS", np.array([P]), V2)
        add("ICROS", P, d1[0], d2[0])
        add("IDEAL", P, *IDEAL[P])
        for a in GRID_D1:
            for b in GRID_D2:
                add(f"G{a:.2f}/{b:.2f}", P, a, b)
    return cf


def predicted_levels(cf):
    """as edge25.predicted_levels on the 2.88 deg levels; the plant only picks them"""
    import pi_steady as ps
    import plant_full as pf
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS * 1e9)
    P, V, D1, D2 = (np.array([c[k] for c in cf]) for k in ("P", "V2", "D1", "D2"))
    j0 = np.round(np.nan_to_num(ps.operating_point(P, D1, D2, V)["D0"]) / G).astype(int)
    known = {}

    def ev(pairs):
        q = sorted({(V[i], D1[i], D2[i], j) for i, j in pairs} - set(known))
        if not q:
            return
        sd = [fg._safe(j * G, a, b, TS) for _, a, b, j in q]
        r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]), np.full(len(q), 380.0), np.array([x[0] for x in q]))
        known.update(zip(q, -np.asarray(r["P_LV"], float)))
        print(f"  plant: {len(q)} evaluations", flush=True)
    n = len(cf)
    lo, hi, br = j0 - 2, j0 + 3, [None] * n
    ev([(i, j) for i in range(n) for j in range(lo[i], hi[i] + 1)])
    for _ in range(10):
        todo = []
        for i in range(n):
            if br[i] is not None:
                continue
            js = list(range(lo[i], hi[i] + 1))
            Pd = [known[(V[i], D1[i], D2[i], j)] for j in js]
            br[i] = rm.first_bracket(js, Pd, P[i])
            if br[i] is None:
                if max(Pd) < P[i]:
                    if hi[i] - j0[i] > 12:      # past the first branch's peak: this duty cannot reach |P|
                        continue
                    todo += [(i, hi[i] + q) for q in range(1, 3)]
                    hi[i] += 2
                else:
                    todo += [(i, lo[i] - q) for q in range(1, 3)]
                    lo[i] -= 2
        if not todo:
            break
        ev(todo)
    return br


def make():
    cf = configs()
    pts = []
    # the named duties and the SPS first: chunks are restart-safe, so a run cut off at the deadline still answers B1
    order = sorted(range(len(cf)), key=lambda i: (cf[i]["ctrl"].startswith("G"), i))
    br = predicted_levels(cf)
    for i in order:
        if br[i] is None:
            print(f"  plant does not reach |P|, not run: {cf[i]}")
            continue
        pts += [fg._point(cf[i], j, TS) for j in br[i]]
    fg._write(os.path.join(OUT, "f1icros_points_r1.csv"), pts)


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "f1icros_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results(files=None):
    fs = result_files() if files is None else files
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "f1icros_sweep_r*.csv"))
          for m in [re.search(r"f1icros_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/f1icros_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = fg.runs_of(d, c, TS)
        if g.empty or np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [fg._point(c, j, TS) for j in rm.extend(g["level"].to_numpy(), -g[PC].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"f1icros_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration that was run is bracketed or out of reach")


def _pp():
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "f1icros_sweep_r*.csv"))):
        T = pd.read_csv(f)
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float), -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T.itertuples(), pk):
            out[(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = (float(np.nanmedian(x)), float(np.nanmax(x)), float(np.nanmin(x)))
    return out


def rule_v(sps_vals, dI_max):
    """sps_vals: {(P, kind): (Ipk(SPS), clean)}; dI_max: {(P, kind): value or NaN}"""
    if any((not np.isfinite(v[0])) or (not v[1]) for v in sps_vals.values()):
        return "undecided"
    if any(np.isfinite(x) and x >= CLAIM for x in dI_max.values()):
        return "reached"
    return "unreachable" if all(v[0] < CLAIM for v in sps_vals.values()) else "not excluded"


def compare(d=None, pp=None, write_out=True):
    import pandas as pd
    d = results() if d is None else d
    if d is None:
        sys.exit("no out/f1icros_sweep_r*.csv yet")
    pp = _pp() if pp is None else pp
    lines, rows = ["f1_icros: 3.392 A at its presumed point, V2 110 V, HV -> LV, 25 ns Simulink (dataset-era model); rules in the file header", ""], []
    sps_vals, dmax, best, need, seen = {}, {}, {}, {}, {}
    for P in PS:
        data = dataset_row(P)
        need[P] = data["I_SPS"] - CLAIM
        cs = [c for c in configs() if c["P"] == P]
        vals = {c["ctrl"]: eg.value(d, pp, c) for c in cs}
        for c in cs:
            v = vals[c["ctrl"]]
            rows.append(dict(P=P, ctrl=c["ctrl"], D1=c["D1"], D2=c["D2"], I=v["I"], med=v["med"], osc=v["osc"], spread=v["spread"], loss=v["loss"], D0=v["D0"]))
        s = vals["SPS"]
        for kind, k in (("rule", "I"), ("med", "med")):
            ok = {n: v[k] for n, v in vals.items() if np.isfinite(v[k]) and v["osc"] <= rc.OSC}
            sps_vals[(P, kind)] = (s[k], np.isfinite(s["osc"]) and s["osc"] <= rc.OSC)
            if ok:
                n = min(ok, key=ok.get)
                dmax[(P, kind)], best[(P, kind)] = s[k] - ok[n], (n, ok[n])
            else:
                dmax[(P, kind)], best[(P, kind)] = np.nan, ("", np.nan)
        seen[P] = (len(cs), sum(np.isfinite(v["I"]) for v in vals.values()), sum(np.isfinite(v["I"]) and v["osc"] <= rc.OSC for v in vals.values()), vals, data)
        lines.append(f"|P| {P:.0f} W: {seen[P][1]} of {seen[P][0]} configurations bracketed, {seen[P][2]} clean")
        for n in ("SPS", "M-B", "DATA", "ICROS", "IDEAL"):
            v = vals[n]
            c = next(x for x in cs if x["ctrl"] == n)
            lines.append(f"  {n:6s} ({c['D1']:.4f}, {c['D2']:.4f}): rule {v['I']:.3f}, median {v['med']:.3f} A, mean i_L {v['osc']:.4f} A, spread {v['spread']:.3f} A, D0 {v['D0']:.2f} deg")
        lines.append(f"  dataset: I_SPS {data['I_SPS']:.3f} A (/ Simulink SPS {data['I_SPS'] / s['I']:.2f}), I_Opt {data['I_Opt']:.3f} A at ({data['D1']:.4f}, {data['D2']:.4f}); 3.392 A would need {need[P]:.3f} A")
        for kind in ("rule", "med"):
            lines.append(f"  {kind}: SPS {sps_vals[(P, kind)][0]:.3f} A, lowest {best[(P, kind)][1]:.3f} A at {best[(P, kind)][0]}, dI_max {dmax[(P, kind)]:+.3f} A; B1 {'yes' if sps_vals[(P, kind)][0] < CLAIM else 'no'}; "
                         f"B3 lowest above the {need[P]:.3f} A needed: {'yes' if best[(P, kind)][1] > need[P] else 'no'}")
    v = rule_v(sps_vals, dmax)
    n_open = sum(s[0] - s[1] for s in seen.values())
    lines += ["", f"VERDICT: {v}" + (f" ({n_open} configurations not bracketed: out of reach or unfinished)" if n_open else "")]
    print("\n".join(lines))
    if not write_out:
        return v, dmax
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "f1icros_check.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "f1icros_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"unreachable": "도달 불가", "not excluded": "배제 못 함", "reached": "도달", "undecided": "판정 불가"}
    md = ["# ICROS2026의 3.392 A, 추정 운전점의 Simulink 측정 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `f1_icros.py compare`. 규칙은 실행 전(10-31 14:41) 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), V2 110 V, HV→LV, V1 380 V, 상수 D0 20 ms, 두 계단 전력 보간. 3.392 A의 출처와 운전점은 저장된 근거가 없는 추정이다.", "",
          f"**판정: {KO[v]}**", "", "| \\|P\\| | 값 | SPS [A] | 측정 최저 [A] (듀티) | 최대 저감 [A] | 3.392 A에 필요한 전류(데이터셋 SPS 기준) [A] |", "|---|---|---|---|---|---|"]
    for P in PS:
        for kind, name in (("rule", "80주기 최대"), ("med", "주기 중앙값")):
            md.append(f"| {P:.0f} W | {name} | {sps_vals[(P, kind)][0]:.3f} | {best[(P, kind)][1]:.3f} ({best[(P, kind)][0]}) | {dmax[(P, kind)]:+.3f} | {need[P]:.3f} |")
    md += ["", "표 원본 `out/f1icros_check.csv`, 로그 `out/f1icros_check.log`."]
    open(os.path.join(OUT, "f1icros_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/f1icros_check.{csv,log}, out/f1icros_verdict.md")
    return v, dmax


def selftest():
    """the registered rule on made-up values, the configuration list, compare end to end on made-up runs"""
    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    cf = configs()
    chk("21 configurations per power", len(cf), 42)
    chk("the dataset row at 150 W", round(dataset_row(150.0)["I_SPS"], 3), 4.736)
    chk("needed current at 150 W, as REPORT 6.7", round(dataset_row(150.0)["I_SPS"] - CLAIM, 3), 1.344)
    chk("level 3 is 3 grid steps of the map's levels minus the nudge", fg._point(cf[0], 3, TS)["D0"], round(3 * G - fg.NUDGE, 6))
    s = {(p, k): (2.5, True) for p in PS for k in ("rule", "med")}
    chk("SPS below the claim -> unreachable", rule_v(s, {x: 0.7 for x in s}), "unreachable")
    chk("SPS above somewhere, no such reduction -> not excluded", rule_v({**s, (150.0, "rule"): (3.5, True)}, {x: 0.7 for x in s}), "not excluded")
    chk("a reduction of 3.392 A measured -> reached", rule_v({**s, (150.0, "rule"): (4.5, True)}, {**{x: 0.7 for x in s}, (150.0, "rule"): 3.4}), "reached")
    chk("SPS not clean -> undecided", rule_v({**s, (150.0, "med"): (2.5, False)}, {x: 0.7 for x in s}), "undecided")
    chk("SPS missing -> undecided", rule_v({**s, (150.0, "med"): (np.nan, True)}, {x: 0.7 for x in s}), "undecided")
    rows, pp = [], {}
    for c in cf:
        base = 2.5 if c["ctrl"] == "SPS" else 1.8
        for j, Pr in ((1, c["P"] - 50.0), (2, c["P"] + 50.0)):
            D0, D1, D2 = fg._safe(j * G, c["D1"], c["D2"], TS)
            rows.append(dict(ctrl=c["ctrl"], V2=V2, level=j, D0=D0, D1=D1, D2=D2, Ts=TS, **{PC: -Pr, IC: base}, iL_mean=0.001, P_LV=-Pr, P_HV=-Pr - 3.0))
            pp[(round(V2, 6), j, round(D1, 9), round(D2, 9))] = (base, base, base)
    v, dmax = compare(pd.DataFrame(rows).drop_duplicates(subset=["V2", "level", "D1", "D2"]), pp, write_out=False)
    chk("made-up runs -> unreachable", v, "unreachable")
    chk("made-up runs: dI_max", round(dmax[(150.0, "rule")], 6), 0.7)
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
