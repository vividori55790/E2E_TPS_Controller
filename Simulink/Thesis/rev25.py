"""
rev25.py -- (2026-10-31 01:32) the LV -> HV direction on the 25 ns grid: does the (|P|, V2) input hold in Simulink?

Every Simulink result so far is HV -> LV. The controller takes |P| because the ideal model is symmetric (test_tps_model.py,
reference only), and the report lists LV -> HV as "미실행". Goal 3 says "전 구간", and the Simulink plant is not obviously
symmetric: the dead time shifts P(D0) (SPS at V1 361 V, D0 -2.88 deg still sends 6.8 W HV -> LV, rob25 run), so the same |P|
sits at a different |D0| in the two directions.

Runs (25 ns, constant D0 20 ms, 20 ms window, two adjacent 2.88-deg levels bracketing |P|, verify_steady_fine.m, nominal plant):
V2 {84, 88.2, 92.4, 96.6, 100.8, 105, 109.2} x |P| {100, 300, 600, 1000} x {SPS, M-B}, the same duties as the 25 ns map, D0 < 0,
level j < 0 (D0 = j G - NUDGE as in fine_grid_check._point). Power = +P_LV. The first bracket going away from D0 = 0 counts (the
first crossing of the PI). The Python plant on the 25 ns grid only picks the levels. Forward values = the 25 ns map
(out/r13_points.csv rule value, out/r13_points_med.csv median period peak).

RULE (registered 2026-10-31 01:32, before any run of this file; do not change after the results). Two values per run: the
80-period maximum and the median period peak (out/iL_pp/; why: r13_map.py header, note of 10-30 18:15). Per point, on each value:
dI_r = Ipk_r(SPS) - Ipk_r(M-B), tol_r = max(0.03 A, 2 % Ipk_r(SPS)), robust_check.classify: "판정 불가" if not bracketed or |mean
i_L| > 20 mA in a bracketing run, "나쁨" if dI_r < -tol_r, "같음" if |dI_r| <= tol_r, "저감" if dI_r > tol_r.
  R1 (goal 3 in this direction): the number of "나쁨" on each value and the list.
  R2 (same reduction in both directions): where the point is decidable in both directions, ddI = dI_r - dI_f; "대칭" at the
     point if |ddI| <= max(tol_r, tol_f). Verdict over the 28 points: "판정 불가" if fewer than 75 % are decidable in both
     directions; "대칭" if on both values at least 75 % of them are "대칭" and |mean ddI| <= 0.03 A; "비대칭" if on either value
     fewer than 50 % are "대칭" or |mean ddI| > 0.10 A; "부분" otherwise.
Reported alongside: mean dI_r, per V2 and per |P|; Ipk_r(SPS) - Ipk_f(SPS) (is SPS itself the same?); |D0| of the bracket in the
two directions; loss and efficiency at |P| (P_HV, P_LV, same interpolation).

    python -I -X utf8 Simulink/Thesis/rev25.py make      -> out/rev25_points_r1.csv
    python automation/job.py submit rev25 --interactive --after v1x -- matlab -batch "cd('Simulink/Thesis'); rev25_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/rev25.py next      (not bracketed -> next round; rev25_rounds calls it)
    python -I -X utf8 Simulink/Thesis/rev25.py compare   -> out/rev25_check.{csv,log}, out/rev25_verdict.md
    python -I -X utf8 Simulink/Thesis/rev25.py selftest
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
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import robust_check as rc  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS = rm.TS
WIN = fg.WIN
G = fg.G
V2S = rm.V2S
PS = (100.0, 300.0, 600.0, 1000.0)
CTRLS = ("SPS", "M-B")
DECIDABLE = 0.75
SYM_OK, SYM_BAD = 0.75, 0.50
MEAN_OK, MEAN_BAD = 0.03, 0.10
PC, IC = f"P_LV_at_{WIN}ms", f"Ipk_at_{WIN}ms"
CODE = {"worse": "나쁨", "same": "같음", "better": "저감", "undecided": "판정 불가"}
R2_TXT = {"sym": "대칭", "asym": "비대칭", "partial": "부분", "undecided": "판정 불가"}


def configs():
    import surface_opt_check as so
    cf = []
    for V2 in V2S:
        for P in PS:
            b = so.mb_predict(P, V2)
            b = (sw.D_SAFE, sw.D_SAFE) if max(abs(b[0] - sw.D_SAFE), abs(b[1] - sw.D_SAFE)) <= rm.SAME else (round(b[0], 9), round(b[1], 9))   # as on the map: M-B = SPS
            cf.append(dict(set="V", pair="", ctrl="SPS", V2=V2, P=P, D1=sw.D_SAFE, D2=sw.D_SAFE))
            cf.append(dict(set="V", pair="", ctrl="M-B", V2=V2, P=P, D1=b[0], D2=b[1]))
    return cf


def predicted_levels(cf):
    """[(j, j - 1)]: the first pair going away from D0 = 0 whose powers (Python plant on the 25 ns grid, +P_LV) bracket |P|"""
    import pi_steady as ps
    import plant_full as pf
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS * 1e9)
    P, V, D1, D2 = (np.array([c[k] for c in cf]) for k in ("P", "V2", "D1", "D2"))
    D0i = ps.operating_point(P, D1, D2, V)["D0"]                       # ideal model: only the centre of the scan
    j0 = np.ceil(np.nan_to_num(np.abs(D0i)) / G).astype(int)
    ms = np.arange(-3, 7)                                              # m = -j = levels away from D0 = 0
    M = np.maximum(j0[:, None] + ms[None, :], 0)
    n, k = M.shape
    sd = [fg._safe(-float(m) * G, a, b, TS) for m, a, b in zip(M.ravel(), np.repeat(D1, k), np.repeat(D2, k))]
    r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]),
                        np.full(n * k, 380.0), np.repeat(V, k))
    Pr = np.asarray(r["P_LV"]).reshape(n, k)
    out = []
    for i, c in enumerate(cf):
        u, idx = np.unique(M[i], return_index=True)
        b = rm.first_bracket(u, Pr[i][idx], c["P"])
        if b is None:
            print(f"  plant does not bracket: {c}")
            b = (int(j0[i]), int(j0[i]) + 1)
        out.append((-b[0], -b[1]))
    print(f"  {n} configurations, {n * k} plant evaluations", flush=True)
    return out


def make():
    cf = configs()
    pts = []
    for c, b in zip(cf, predicted_levels(cf)):
        pts += [fg._point(c, j, TS) for j in b]
    fg._write(os.path.join(OUT, "rev25_points_r1.csv"), pts)


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "rev25_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results(files=None):
    fs = result_files() if files is None else files
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["V2", "level", "D1", "D2"], keep="last")


def mirrored(d, c):
    """the runs of one configuration with the level and the power mirrored, so that the forward helpers read them: level m = -j,
    P_LV -> -P_LV (fg.at_power takes -P_LV as the power), sorted from D0 = 0 outward"""
    g = fg.runs_of(d, c, TS)
    if g.empty:
        return g
    return g.assign(**{"level": -g["level"], PC: -g[PC], "P_HV": -g["P_HV"], "P_LV": -g["P_LV"]}).sort_values("level")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "rev25_sweep_r*.csv"))
          for m in [re.search(r"rev25_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/rev25_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = mirrored(d, c)
        if g.empty:
            sys.exit(f"no runs at all for {c} -- round 1 not finished?")
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [fg._point(c, -m, TS) for m in rm.extend(g["level"].to_numpy(), -g[PC].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"rev25_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


def _pp():
    """(V2, level j, D1, D2) -> median period peak, from out/iL_pp/"""
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "rev25_sweep_r*.csv"))):
        T = pd.read_csv(f)
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float),
                        -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T.itertuples(), pk):
            out[(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = float(np.nanmedian(x))
    return out


def value(d, pp, c):
    """(rule value, osc, median period peak, loss, D0 of the run nearest D0 = 0 in the bracket) at |P|, or NaNs"""
    g = mirrored(d, c)
    I, o, lo = fg.at_power(g, c["P"]) if not g.empty else (np.nan, np.nan, None)
    if lo is None:
        return np.nan, np.nan, np.nan, np.nan, np.nan
    x = g[g["level"].isin([lo, lo + 1])]
    t = (c["P"] - (-x[PC].iloc[0])) / ((-x[PC].iloc[1]) - (-x[PC].iloc[0]))
    ms = [pp.get((round(c["V2"], 6), -int(r.level), round(r.D1, 9), round(r.D2, 9))) for r in x.itertuples()]
    med = ms[0] + t * (ms[1] - ms[0]) if all(m is not None for m in ms) else np.nan
    # mirrored: P_LV = -(LV delivered), P_HV = -(HV received); loss = LV in - HV out
    loss = (-x["P_LV"]) - (-x["P_HV"])
    return I, o, med, float(loss.iloc[0] + t * (loss.iloc[1] - loss.iloc[0])), float(x["D0"].iloc[0] + t * (x["D0"].iloc[1] - x["D0"].iloc[0]))


def rule_r2(ddI, tol, ddI_med, tol_med, n_points):
    """ddI, tol ...: arrays over the points decidable in both directions, rule value and median"""
    if n_points == 0 or len(ddI) < DECIDABLE * n_points:
        return "undecided", {}
    s, sm = float(np.mean(np.abs(ddI) <= tol)), float(np.mean(np.abs(ddI_med) <= tol_med))
    m, mm = float(np.mean(ddI)), float(np.mean(ddI_med))
    info = dict(sym=s, sym_med=sm, mean=m, mean_med=mm)
    if s >= SYM_OK and sm >= SYM_OK and abs(m) <= MEAN_OK and abs(mm) <= MEAN_OK:
        return "sym", info
    if s < SYM_BAD or sm < SYM_BAD or abs(m) > MEAN_BAD or abs(mm) > MEAN_BAD:
        return "asym", info
    return "partial", info


def forward():
    """(V2, P) -> dict of the 25 ns map: rule value and median"""
    import pandas as pd
    Q = pd.read_csv(os.path.join(OUT, "r13_points.csv"))
    M = pd.read_csv(os.path.join(OUT, "r13_points_med.csv"))
    out = {}
    for r in Q.itertuples():
        m = M[(M["V2"] == r.V2) & (M["P"] == r.P)].iloc[0]
        out[(r.V2, r.P)] = dict(I_SPS=r.I_SPS, I_MB=r.I_MB, osc=max(m["osc_SPS"], m["osc_MB"]), med_SPS=m["med_SPS"], med_MB=m["med_MB"], loss_SPS=m["loss_SPS"], loss_MB=m["loss_MB"])
    return out


def compare(d=None, pp=None, fw=None, write_out=True):
    import datetime

    import pandas as pd
    d = results() if d is None else d
    if d is None:
        sys.exit("no out/rev25_sweep_r*.csv yet")
    pp, fw = _pp() if pp is None else pp, forward() if fw is None else fw
    by = {(c["ctrl"], c["V2"], c["P"]): c for c in configs()}
    rows = []
    for V2 in V2S:
        for P in PS:
            s, b = value(d, pp, by[("SPS", V2, P)]), value(d, pp, by[("M-B", V2, P)])
            f = fw[(V2, P)]
            cr, dr, tr = rc.classify(s[0], b[0], s[1], b[1])
            cm, dm, tm = rc.classify(s[2], b[2], s[1], b[1])
            cf, df, tf = rc.classify(f["I_SPS"], f["I_MB"], f["osc"], f["osc"])
            cfm, dfm, tfm = rc.classify(f["med_SPS"], f["med_MB"], f["osc"], f["osc"])
            rows.append(dict(V2=V2, P=P, k=4 * V2 / 380.0, SPS_r=s[0], MB_r=b[0], osc_SPS=s[1], osc_MB=b[1], dI_r=dr, tol_r=tr, code_r=cr, med_SPS_r=s[2], med_MB_r=b[2],
                             dI_r_med=dm, tol_r_med=tm, code_r_med=cm, SPS_f=f["I_SPS"], MB_f=f["I_MB"], dI_f=df, tol_f=tf, code_f=cf, dI_f_med=dfm, tol_f_med=tfm,
                             ddI=dr - df, ddI_med=dm - dfm, tol_pair=np.nanmax([tr, tf]), tol_pair_med=np.nanmax([tm, tfm]), dSPS=s[0] - f["I_SPS"],
                             D0_SPS=s[4], D0_MB=b[4], loss_SPS_r=s[3], loss_MB_r=b[3], loss_SPS_f=f["loss_SPS"], loss_MB_f=f["loss_MB"]))
    T = pd.DataFrame(rows)
    both = T[(T["code_r"] != "undecided") & (T["code_f"] != "undecided") & np.isfinite(T["ddI_med"])]
    r2, info = rule_r2(both["ddI"].to_numpy(), both["tol_pair"].to_numpy(), both["ddI_med"].to_numpy(), both["tol_pair_med"].to_numpy(), len(T))
    dec = T[T["code_r"] != "undecided"]
    n_open = int((~np.isfinite(T["SPS_r"]) | ~np.isfinite(T["MB_r"])).sum())
    lines = ["rev25: LV->HV, 25 ns Simulink (dataset-era model), dI = Ipk(SPS) - Ipk(M-B) [A]; rule in the file header", "",
             "    V2    |P| | LV->HV SPS    M-B     dI    tol | code      | median dI | HV->LV SPS    M-B     dI | ddI    median | SPS r-f | D0 SPS, M-B [deg]"]
    for r in T.itertuples():
        lines.append(f" {r.V2:6.1f} {r.P:6.0f} | {r.SPS_r:9.3f}  {r.MB_r:5.3f}  {r.dI_r:+6.3f} {r.tol_r:5.3f} | {r.code_r:9s} | {r.dI_r_med:+8.3f}  | {r.SPS_f:9.3f}  {r.MB_f:5.3f}  {r.dI_f:+6.3f} | {r.ddI:+6.3f} {r.ddI_med:+6.3f}  | "
                     f"{r.dSPS:+6.3f}  | {r.D0_SPS:7.2f} {r.D0_MB:7.2f}")
    lines += ["", f"R1: LV->HV decidable {len(dec)}/{len(T)}; worse {int((T['code_r'] == 'worse').sum())} (rule value), {int((T['code_r_med'] == 'worse').sum())} (median); same {int((T['code_r'] == 'same').sum())}, better {int((T['code_r'] == 'better').sum())}; "
              f"mean dI {dec['dI_r'].mean():+.3f} A, median peak {dec['dI_r_med'].mean():+.3f} A, relative {100 * (dec['dI_r'] / dec['SPS_r']).mean():+.1f} %",
              f"R2: {r2} {info}, points decidable in both directions {len(both)}/{len(T)}"]
    if n_open:
        lines.append(f"{n_open} configurations not bracketed yet -> provisional")
    md = ["# LV→HV 방향, 25 ns 격자 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `rev25.py compare`. 규칙은 실행 전(10-31 01:32) `rev25.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), 상수 D0 20 ms, 두 계단 전력 보간. Python 플랜트는 계단 선택에만 썼다.", "",
          "| V2 | \\|P\\| | LV→HV SPS [A] | LV→HV B [A] | ΔI [A] | 허용 | 판정 | ΔI(중앙값) | HV→LV ΔI [A] | ddI [A] | ddI(중앙값) | SPS 역 − 정 [A] | D0 SPS, B [°] |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.V2:g} V | {r.P:.0f} W | {r.SPS_r:.3f} | {r.MB_r:.3f} | {r.dI_r:+.3f} | {r.tol_r:.3f} | {CODE[r.code_r]} | {r.dI_r_med:+.3f} | {r.dI_f:+.3f} | {r.ddI:+.3f} | {r.ddI_med:+.3f} | {r.dSPS:+.3f} | {r.D0_SPS:.2f}, {r.D0_MB:.2f} |")
    md += ["", f"**R1 (목표 3, 이 방향): 판정 가능 {len(dec)}/{len(T)}, 나쁨 규칙 값 {int((T['code_r'] == 'worse').sum())}, 중앙값 {int((T['code_r_med'] == 'worse').sum())}**; 평균 저감 {dec['dI_r'].mean():+.3f} A(중앙값 값 {dec['dI_r_med'].mean():+.3f} A).",
           f"**R2 (두 방향의 저감이 같은가): {R2_TXT[r2]}.** " + (f"대칭 점 비율 {100 * info['sym']:.0f} %·{100 * info['sym_med']:.0f} %, 평균 ddI {info['mean']:+.3f}·{info['mean_med']:+.3f} A(규칙 값·중앙값), 두 방향 모두 판정 가능 {len(both)}/{len(T)}." if info else "")]
    if n_open:
        md.append(f"※ 아직 감싸지 못한 구성 {n_open}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "rev25_check.csv"), index=False, float_format="%.5g")
        open(os.path.join(OUT, "rev25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        open(os.path.join(OUT, "rev25_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, r2


def selftest():
    """the registered rule end to end on made-up Simulink runs"""
    import tempfile

    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    cf = configs()
    chk("28 points x 2 controllers", len(cf), 56)
    chk("level -3 is D0 < 0", fg._point(cf[0], -3, TS)["D0"], round(-3 * G - fg.NUDGE, 6))
    chk("R2 symmetric", rule_r2(np.full(28, 0.01), np.full(28, 0.03), np.full(28, 0.0), np.full(28, 0.03), 28)[0], "sym")
    chk("R2 mean too large", rule_r2(np.full(28, 0.029), np.full(28, 0.03), np.full(28, 0.031), np.full(28, 0.05), 28)[0], "partial")
    chk("R2 asymmetric on the median alone", rule_r2(np.full(28, 0.0), np.full(28, 0.03), np.array([0.2] * 15 + [0.0] * 13), np.full(28, 0.03), 28)[0], "asym")
    chk("R2 undecided", rule_r2(np.zeros(20), np.full(20, 0.03), np.zeros(20), np.full(20, 0.03), 28)[0], "undecided")

    def fw(gap):
        return {(V2, P): dict(I_SPS=2.0 + P / 1000.0, I_MB=2.0 + P / 1000.0 - 0.1, osc=0.001, med_SPS=2.0 + P / 1000.0 + gap, med_MB=2.0 + P / 1000.0 - 0.1 + gap,
                              loss_SPS=5.0, loss_MB=4.0) for V2 in V2S for P in PS}

    def runs(worse=(), osc_at=()):
        rows = []
        for c in cf:
            pt = (c["V2"], c["P"])
            base = 2.0 + c["P"] / 1000.0 - (0.0 if c["ctrl"] == "SPS" else (-0.2 if pt in worse else 0.1))
            m0 = int(c["P"] // 50) + 3
            for m, Pr in ((m0, 50.0 * (m0 - 3) - 25.0), (m0 + 1, 50.0 * (m0 - 3) + 25.0)):
                rows.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=-m, D0=-m * G, D1=c["D1"], D2=c["D2"], Ts=TS, **{PC: Pr, IC: base}, iL_mean=0.05 if pt in osc_at and c["ctrl"] == "M-B" else 0.001,
                                 P_LV=Pr, P_HV=Pr - 3.0, error=""))
        # a second-branch decoy further out: the same power again; the first bracket from D0 = 0 must be taken
        far = [dict(r, level=r["level"] - 40, **{IC: r[IC] + 5.0}) for r in rows]
        return pd.DataFrame(rows + far)

    def pp_of(F):
        return {(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9)): getattr(r, IC) for r in F.itertuples()}
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "fake.csv")
        F = runs()
        F.to_csv(f, index=False)
        T, r2 = compare(results([f]), pp_of(F), fw(0.0), write_out=False)
        chk("fake: same reduction -> R2 sym", r2, "sym")
        chk("fake: the bracket nearest D0 = 0 is taken", bool((T["SPS_r"] < 4.0).all()), True)
        F = runs(worse=[(84.0, 100.0), (84.0, 300.0)], osc_at=[(109.2, 1000.0)])
        F.to_csv(f, index=False)
        T, r2 = compare(results([f]), pp_of(F), fw(0.0), write_out=False)
        chk("fake: two worse points", list(T[T["code_r"] == "worse"][["V2", "P"]].itertuples(index=False, name=None)), [(84.0, 100.0), (84.0, 300.0)])
        chk("fake: the oscillating point is undecided", T[(T["V2"] == 109.2) & (T["P"] == 1000.0)]["code_r"].iloc[0], "undecided")
        chk("fake: mean ddI small enough, two points off -> R2 sym", r2, "sym")
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
