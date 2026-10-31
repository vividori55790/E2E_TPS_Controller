"""
v1e_check.py -- (2026-10-31 11:12) how much V1 error can the surface take: V1 = 380 V x (1 + e), |e| = 1 and 2 %.

v1x (final 10-31 10:58): at V1 +-5 % M-B, which does not know V1, is above SPS in 12 of 18 cases near k = 1 (up to 0.270 A), and the
normalised input M-Bk removes that, so the proposal is now M-B with the normalised input. Simulink has only seen e = 0 and +-5 %, and
two questions need the values between them: (1) can a link regulated to +-1-2 % do without the normalisation; (2) how good must the V1
reading of the normalised input be. Both are the same run: M-B at a true V1 = 380 (1 + e) gets exactly the input M-Bk computes when it
reads 380 V at that true V1. Reference only (properties of the model, out/v1x_side.log, not a plant result): a reading error of 1 % moves
the commanded duty by at most 0.09 (99th percentile 0.07, median 0.005), and OPT25 found the optimum flat within about 0.05 around the
surface, so the answer may change between 0.5 and 2 %. Nothing about the size of the Ipk change at these e is known yet.

Runs (as v1x_check.py: 25 ns, constant D0 20 ms, 20 ms window, two adjacent 2.88-deg levels bracketing |P|, Ipk interpolated in power;
verify_steady_fine_param.m; the Python plant with that V1 only picks the levels):
  V1 372.4, 376.2 V (e -2, -1 %) x V2 {88.2, 90.3, 92.4}, V1 383.8, 387.6 V (e +1, +2 %) x V2 {98.7, 100.8, 102.9},
  and V1 380 V (e 0, the baseline) x all six V2, x |P| {100, 200} W x {SPS, M-B}.
Same V2, |P| and M-B duties as set A of v1x_check.py, so with its +-5 % values each (V2, |P|) gives one series in V1. Duties within 0.003
of SPS use the SPS runs (r13_map.SAME), as on the map.

RULE (registered 2026-10-31 11:12, before any run of this file; do not change after the results). Two values per run: the 80-period
maximum and the median period peak (out/iL_pp/; why: r13_map.py header, note of 10-30 18:15). Per case, on each value:
robust_check.classify(SPS, M-B) = "판정 불가" if not bracketed or |mean i_L| > 20 mA in a bracketing run, dI = Ipk(SPS) - Ipk(M-B),
tol = max(0.03 A, 2 % Ipk(SPS)), "열화" if dI < -tol, "같음" if |dI| <= tol, "저감" if dI > tol.
For |e| = 1 % and 2 % separately, over its 12 cases (two signs x three V2 x two |P|):
  N(value) = cases "열화" on that value whose (V2, |P|) is not already "열화" at e = 0 on that value: what the V1 error adds.
  "판정 불가" if fewer than 75 % of the 12 are decidable on the 80-period maximum;
  "깨짐" if N >= 3 on both values; "없음" if N = 0 on both; "일부" otherwise.
Reported alongside: every case with k and duties; the baseline count; per (V2, |P|) the series of dI over e, with +-5 % from
out/v1x_check.csv; the e = 0 values against the 25 ns map where the map has the point (out/r13_points.csv, the control: same runner path,
same runs up to the plant's level choice, expected to agree inside tol).
What each answer lets the report say, fixed now: "없음" at |e| = x: no V1 tolerance problem up to x without the normalisation, and a reading
error of x is harmless with it. "일부" or "깨짐" at x: both need better than x, and the largest dI there is stated next to the adoption.

    python -I -X utf8 Simulink/Thesis/v1e_check.py make      -> out/v1e_points_r1.csv
    python automation/job.py submit v1e --interactive --after g12a -- matlab -batch "cd('Simulink/Thesis'); v1e_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/v1e_check.py predict   -> out/v1e_predict.log ([참고] the same cases on the 25 ns plant, written before any run, not a verdict)
    python -I -X utf8 Simulink/Thesis/v1e_check.py next      (not bracketed -> next round; v1e_rounds calls it)
    python -I -X utf8 Simulink/Thesis/v1e_check.py compare   -> out/v1e_check.{csv,log}, out/v1e_verdict.md
    python -I -X utf8 Simulink/Thesis/v1e_check.py selftest
"""
import argparse
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `make` / `next` run beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import robust25 as rb  # noqa: E402
import robust_check as rc  # noqa: E402
import simulink_sweep as sw  # noqa: E402
import v1x_check as vx  # noqa: E402

OUT = os.path.join(HERE, "out")
WIN = fg.WIN
E_V1 = {"V1-2": 372.4, "V1-1": 376.2, "V1+0": 380.0, "V1+1": 383.8, "V1+2": 387.6}
rc.VARIANTS.update({v: dict(V1=x) for v, x in E_V1.items()})          # rb.predicted_levels, rc.params know the variants by name
LOW, HIGH = vx.A_V2["V1-5"], vx.A_V2["V1+5"]
V2_OF = {"V1-2": LOW, "V1-1": LOW, "V1+0": LOW + HIGH, "V1+1": HIGH, "V1+2": HIGH}
PS = (100.0, 200.0)
ABS_E = {"1 %": ("V1-1", "V1+1"), "2 %": ("V1-2", "V1+2")}
N_BREAK = 3
DECIDABLE = 0.75
PATS = ("v1e_sweep_r*.csv",)
CODE = vx.CODE
TXT = {"breaks": "깨짐", "none": "없음", "some": "일부", "undecided": "판정 불가"}


def cases():
    """[(variant, V2, P)]"""
    return [(v, V2, P) for v, vs in V2_OF.items() for V2 in vs for P in PS]


def controllers(mb, V2, P):
    """(ctrl -> (D1, D2)); M-B does not know V1, so one case's duties depend on (V2, |P|) only"""
    sps, b = (sw.D_SAFE, sw.D_SAFE), vx._duties(mb, P, V2, 1.0)
    return {"SPS": sps, "M-B": sps if vx._near(b, sps) else b}


def configs():
    mb, cf = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json")), []
    for v, V2, P in cases():
        for ctrl, (d1, d2) in controllers(mb, V2, P).items():
            cf.append(dict(ctrl=ctrl, V2=V2, P=P, D1=d1, D2=d2, variant=v, **rc.params(v)))
    return cf


def make():
    cf = configs()
    pts = []
    for c, b in zip(cf, rb.predicted_levels(cf)):
        pts += [rb.point(c, j) for j in b]
    rb.write(os.path.join(OUT, "v1e_points_r1.csv"), pts)


def predict():
    """[참고, Python plant, NOT Simulink] the same cases on the 25 ns plant, written before any run; not part of the rule. -> out/v1e_predict.log"""
    import rob_interior_plant as rp
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    lines = ["v1e_check predict: [참고] Python plant, 25 ns grid, NOT Simulink. dI = Ipk(SPS) - Ipk(M-B) [A], * = dI < -max(0.03 A, 2 %)"]
    for v, vs in V2_OF.items():
        pts = [(V2, P) for V2 in vs for P in PS]
        cf = []
        for V2, P in pts:
            ds = controllers(mb, V2, P)
            cf += [(V2, P) + ds["SPS"], (V2, P) + ds["M-B"]]
        ipk = rp.ipk_at_power(v, cf)
        for i, (V2, P) in enumerate(pts):
            s, b = ipk[2 * i], ipk[2 * i + 1]
            lines.append(f"  {v} V1 {E_V1[v]:.1f} V {V2:6.1f} V k {4 * V2 / E_V1[v]:.3f} {P:4.0f} W: SPS {s:.3f} M-B {b:.3f} dI {s - b:+.3f}{'*' if s - b < -max(rc.TOL_A, rc.TOL_REL * s) else ''}")
        print(f"{v} done", flush=True)
    open(os.path.join(OUT, "v1e_predict.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def own_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "v1e_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results():
    return rb.results(own_files())


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "v1e_sweep_r*.csv"))
          for m in [re.search(r"v1e_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/v1e_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = rb.runs_of(d, c)
        if g.empty:
            sys.exit(f"no runs at all for {c} -- round 1 not finished?")
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [rb.point(c, j) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        rb.write(os.path.join(OUT, f"v1e_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def rule_e(n_max, n_med, n_decidable, n_cases):
    """N on the two values, cases decidable on the 80-period maximum, cases of this |e|"""
    if n_cases == 0 or n_decidable < DECIDABLE * n_cases:
        return "undecided"
    if n_max >= N_BREAK and n_med >= N_BREAK:
        return "breaks"
    return "none" if n_max == 0 and n_med == 0 else "some"


def added(T, col):
    """cases of T worse on code column col whose (V2, P) is not already worse at e = 0"""
    base = {(r.V2, r.P) for r in T[(T["variant"] == "V1+0") & (T[col] == "worse")].itertuples()}
    return [(r.V2, r.P) for r in T[(T["variant"] != "V1+0") & (T[col] == "worse")].itertuples() if (r.V2, r.P) not in base]


def compare(d=None, pp=None, write_out=True):
    import datetime

    import pandas as pd
    d = results() if d is None else d
    pp = rb._pp(PATS) if pp is None else pp
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    rows = []
    for v, V2, P in cases():
        ds = controllers(mb, V2, P)
        val = {k: vx.value(d, pp, dict(ctrl=k, V2=V2, P=P, D1=x[0], D2=x[1], variant=v)) for k, x in ds.items()}
        s, b = val["SPS"], val["M-B"]
        cb, db, tol = rc.classify(s[0], b[0], s[1], b[1])
        cm, dm, tolm = rc.classify(s[2], b[2], s[1], b[1])
        rows.append(dict(variant=v, V1=E_V1[v], V2=V2, P=P, k=4 * V2 / E_V1[v], SPS=s[0], MB=b[0], osc_SPS=s[1], osc_MB=b[1], dI=db, tol=tol, code=cb,
                         med_SPS=s[2], med_MB=b[2], dI_med=dm, tol_med=tolm, code_med=cm, D1=ds["M-B"][0], D2=ds["M-B"][1]))
    T = pd.DataFrame(rows)
    n_open = int((~np.isfinite(T["SPS"]) | ~np.isfinite(T["MB"])).sum())
    lines = ["v1e_check: V1 = 380 V (1 + e), 25 ns Simulink (dataset-era model), HV->LV, dI = Ipk(SPS) - Ipk(M-B) [A]; rule in the file header", "",
             "  variant  V1     V2      k     |P| |  SPS    M-B  |   dI     tol   code       | median dI  code       | (D1, D2)"]
    for r in T.itertuples():
        lines.append(f"  {r.variant:6s} {r.V1:6.1f} {r.V2:6.1f} {r.k:6.3f} {r.P:6.0f} | {r.SPS:5.3f}  {r.MB:5.3f} | {r.dI:+6.3f}  {r.tol:5.3f}  {r.code:10s} | {r.dI_med:+6.3f}     {r.code_med:10s} | ({r.D1:.3f}, {r.D2:.3f})")
    answers = {}
    B = T[T["variant"] == "V1+0"]
    lines += ["", f"baseline e = 0: worse {list(B['code']).count('worse')} (maximum), {list(B['code_med']).count('worse')} (median) of {len(B)}"]
    for name, vs in ABS_E.items():
        A = T[T["variant"].isin(vs + ("V1+0",))]
        E = T[T["variant"].isin(vs)]
        nx, nm = len(added(A, "code")), len(added(A, "code_med"))
        dec = int((E["code"] != "undecided").sum())
        answers[name] = rule_e(nx, nm, dec, len(E))
        lines.append(f"|e| = {name}: decidable {dec}/{len(E)}, added worse N {nx} (maximum), {nm} (median), largest above SPS {-E['dI'].min():+.3f} / {-E['dI_med'].min():+.3f} A: {answers[name]}")
    if n_open:
        lines.append(f"{n_open} cases not bracketed yet -> provisional")
    series = _series(T)
    lines += ["", "dI over e per (V2, |P|), maximum / median, +-5 % from out/v1x_check.csv:"] + series
    ctl = _control(T)
    lines += ["", "e = 0 against the 25 ns map (out/r13_points.csv, control): V2, |P|, SPS map, SPS here, M-B map, M-B here"] + ctl
    md = ["# V1 오차 ±1·±2 %에서 곡면 B (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `v1e_check.py compare`. 규칙은 실행 전(10-31 11:12) `v1e_check.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), 상수 D0 20 ms, 두 계단 전력 보간. Python 플랜트는 계단 선택에만 썼다.", "",
          "| V1 | e | V2 | k | \\|P\\| | SPS [A] | B [A] | ΔI | 허용 | 판정 | ΔI(중앙값) | 판정(중앙값) |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.V1:g} V | {r.variant[2:]} % | {r.V2:g} V | {r.k:.3f} | {r.P:.0f} W | {r.SPS:.3f} | {r.MB:.3f} | {r.dI:+.3f} | {r.tol:.3f} | {CODE[r.code]} | {r.dI_med:+.3f} | {CODE[r.code_med]} |")
    md += [""] + [f"**|e| = {k}: {TXT[v]}.**" for k, v in answers.items()]
    if n_open:
        md.append(f"※ 아직 감싸지 못한 경우 {n_open}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "v1e_check.csv"), index=False, float_format="%.5g")
        open(os.path.join(OUT, "v1e_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        open(os.path.join(OUT, "v1e_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, answers


def _series(T):
    """one line per (V2, |P|): dI over e, with the +-5 % of v1x_check; reported alongside"""
    import pandas as pd
    f = os.path.join(OUT, "v1x_check.csv")
    X = pd.read_csv(f) if os.path.isfile(f) else None
    out = []
    for V2 in LOW + HIGH:
        for P in PS:
            cells = []
            five = "V1-5" if V2 in LOW else "V1+5"
            if X is not None:
                x = X[(X["set"] == "A") & (X["variant"] == five) & (X["V2"] == V2) & (X["P"] == P)]
                if not x.empty:
                    cells.append((five, float(x["dI_b"].iloc[0]), float(x["dI_b_med"].iloc[0])))
            for v in ("V1-2", "V1-1", "V1+0", "V1+1", "V1+2"):
                y = T[(T["variant"] == v) & (T["V2"] == V2) & (T["P"] == P)]
                if not y.empty:
                    cells.append((v, float(y["dI"].iloc[0]), float(y["dI_med"].iloc[0])))
            cells.sort(key=lambda c: rc.params(c[0])["V1"])
            out.append(f"  {V2:6.1f} V {P:5.0f} W: " + "  ".join(f"{c[0][2:]} {c[1]:+.3f}/{c[2]:+.3f}" for c in cells))
    return out


def _control(T):
    """the e = 0 runs against the 25 ns map where the map has the point"""
    import pandas as pd
    f = os.path.join(OUT, "r13_points.csv")
    if not os.path.isfile(f):
        return ["  no out/r13_points.csv"]
    Q = pd.read_csv(f)
    out = []
    for r in T[T["variant"] == "V1+0"].itertuples():
        q = Q[(np.abs(Q["V2"] - r.V2) < 1e-6) & (Q["P"] == r.P)]
        if not q.empty:
            q = q.iloc[0]
            out.append(f"  {r.V2:6.1f} V {r.P:5.0f} W: {q['I_SPS']:.3f} {r.SPS:.3f} {q['I_MB']:.3f} {r.MB:.3f}")
    return out


def selftest():
    """the registered rule end to end on made-up Simulink runs"""
    import tempfile

    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("cases", len(cases()), 4 * 6 + 12)
    chk("variants known", [rc.params(v)["V1"] for v in E_V1], list(E_V1.values()))
    chk("M-B duties as in v1x set A", controllers(ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json")), 90.3, 100.0)["M-B"],
        vx.controllers(ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json")), "V1-5", 90.3, 100.0)["M-B"])
    chk("rule breaks", rule_e(3, 3, 12, 12), "breaks")
    chk("rule some: one value below 3", rule_e(3, 2, 12, 12), "some")
    chk("rule some: one value above 0", rule_e(0, 1, 12, 12), "some")
    chk("rule none", rule_e(0, 0, 12, 12), "none")
    chk("rule undecided", rule_e(5, 5, 8, 12), "undecided")
    A = pd.DataFrame(dict(variant=["V1+0", "V1-1", "V1-1"], V2=[88.2, 88.2, 90.3], P=[100.0, 100.0, 100.0], code=["worse", "worse", "worse"]))
    chk("added leaves out what is already worse at e = 0", added(A, "code"), [(90.3, 100.0)])
    cf = configs()

    def runs(worse):
        rows = []
        for c in cf:
            base = 2.0 + c["P"] / 1000.0 + (0.2 if c["ctrl"] == "M-B" and (c["variant"], c["V2"], c["P"]) in worse else 0.0)
            j0 = int(c["P"] // 50)
            for j, Pd in ((j0, 50.0 * j0 - 25.0), (j0 + 1, 50.0 * j0 + 25.0)):
                rows.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=j, D0=j * fg.G, D1=c["D1"], D2=c["D2"], Ts=rm.TS, variant=c["variant"], V1=c["V1"],
                                 **{f"P_LV_at_{WIN}ms": -Pd, f"Ipk_at_{WIN}ms": base}, iL_mean=0.001, P_LV=-Pd, error=""))
        return pd.DataFrame(rows)

    def pp_of(F):
        return {(r.variant, round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9)): getattr(r, f"Ipk_at_{WIN}ms") for r in F.itertuples()}
    worse = {("V1-2", 88.2, 100.0), ("V1+2", 98.7, 100.0), ("V1-2", 90.3, 200.0), ("V1-1", 88.2, 100.0)}
    F = runs(worse)
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "fake.csv")
        F.to_csv(f, index=False)
        T, ans = compare(rb.results([f]), pp_of(F), write_out=False)
    chk("fake: 3 added at 2 % -> breaks", ans["2 %"], "breaks")
    chk("fake: 1 added at 1 % -> some", ans["1 %"], "some")
    chk("fake: every case judged", int((T["code"] == "undecided").sum()), 0)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "predict", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "predict": predict, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
