"""
edge25.py -- (2026-10-31 02:15) the two ends of the specified power range on the 25 ns grid, HV -> LV.

The specification and the model's input are |P| <= 1.2 kW and goal 3 says "전 구간", but every Simulink evaluation of a controller so
far has a target in 70-1000 W (the 25 ns map 100-1000 W; `out/r13_map.csv`, `out/ms_check.csv`; the dataset replays are wider but are
1 ms transients). Below 100 W the surface runs down to its floor
D 0.1 (84 V 25 W (0.114, 0.145)), so the pulse is about 14 dead times long, and P(D0) is shifted by the dead time by as much as
the target power itself (10-30 k1map: first level +91 W at 93 V, -86 W at 97 V): the place where the ideal-model training says
least. Above 1 kW the surface is the end of the regression's input domain (u = 1).

Runs (25 ns, constant D0 20 ms, 20 ms window, nominal plant, verify_steady_fine.m, D0 > 0 = HV -> LV):
  E: V2 {84, 88.2, 92.4, 96.6, 100.8, 105, 109.2} x |P| {25, 50, 75, 1200} x {SPS, M-B}
  C (control): 100 W x the 7 V2 and 1000 W x {84, 96.6, 109.2} x {SPS, M-B}: the same points as the 25 ns map.
Two adjacent levels bracket |P|. Level spacing = 0.72 deg (one 25 ns grid step), D0 = j 0.72 - NUDGE, not the map's 2.88 deg: one
map level is about 140 W wide on SPS, wider than these targets. The first adjacent pair from below that brackets |P| counts
(fine_grid_check.at_power, as the map). The Python plant on the 25 ns grid only picks the levels.

RULE (registered 2026-10-31 02:15, before any run of this file; do not change after the results). Two values per run, the 80-period
maximum and the median period peak (out/iL_pp/; why: r13_map.py header, note of 10-30 18:15). Per point, on each value:
dI = Ipk(SPS) - Ipk(M-B), robust_check.classify: "판정 불가" if not bracketed or |mean i_L| > 20 mA in a bracketing run, "나쁨" if
dI < -max(0.03 A, 2 % Ipk(SPS)), "같음" within it, "저감" above it. Per end (light = the 21 points of 25, 50, 75 W; heavy = the 7
points of 1200 W), n_dec decidable points:
  "판정 불가" if n_dec < 75 % of the end's points;
  "깨짐" if ("나쁨" > 25 % of n_dec on both values) or (mean dI < -0.03 A on both values);
  "유지" if "나쁨" <= 10 % of n_dec on both values and mean dI >= 0 on both values;
  "부분" otherwise.
  C: dI_E - dI_map on the 10 control points: "간격 영향 없음" if |mean| <= 0.03 A on both values, else "간격 영향 있음" (the map's values
     stay as they are; the difference is then stated beside its 100 W reductions).
Reported alongside: relative dI, per V2 and per |P|; loss and efficiency at |P|, B - SPS (at 25-75 W loss is a large part of P);
which runs are not periodic; D0 of the bracket.

    python -I -X utf8 Simulink/Thesis/edge25.py make      -> out/edge25_points_r1.csv
    python automation/job.py submit edge25 --interactive --after flip25 -- matlab -batch "cd('Simulink/Thesis'); edge25_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/edge25.py next      (not bracketed -> next round; edge25_rounds calls it)
    python -I -X utf8 Simulink/Thesis/edge25.py compare   -> out/edge25_check.{csv,log}, out/edge25_verdict.md
    python -I -X utf8 Simulink/Thesis/edge25.py selftest
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
GE = 0.72                                   # deg: one 25 ns grid step at 80 kHz
V2S = rm.V2S
LIGHT, HEAVY = (25.0, 50.0, 75.0), (1200.0,)
C_PTS = tuple((v, 100.0) for v in V2S) + tuple((v, 1000.0) for v in (84.0, 96.6, 109.2))
DECIDABLE, BAD_HI, BAD_LO, MEAN_BAD, C_MEAN = 0.75, 0.25, 0.10, 0.03, 0.03
PC, IC = f"P_LV_at_{WIN}ms", f"Ipk_at_{WIN}ms"
CODE = {"worse": "나쁨", "same": "같음", "better": "저감", "undecided": "판정 불가"}
END_TXT = {"holds": "유지", "broken": "깨짐", "partial": "부분", "undecided": "판정 불가"}


def configs():
    import surface_opt_check as so
    cf = []
    for V2, P, st in [(v, p, "E") for v in V2S for p in LIGHT + HEAVY] + [(v, p, "C") for v, p in C_PTS]:
        b = so.mb_predict(P, V2)
        b = (sw.D_SAFE, sw.D_SAFE) if max(abs(b[0] - sw.D_SAFE), abs(b[1] - sw.D_SAFE)) <= rm.SAME else (round(b[0], 9), round(b[1], 9))   # as on the map: M-B = SPS
        cf.append(dict(set=st, ctrl="SPS", V2=V2, P=P, D1=sw.D_SAFE, D2=sw.D_SAFE))
        cf.append(dict(set=st, ctrl="M-B", V2=V2, P=P, D1=b[0], D2=b[1]))
    return cf


def _point(c, j):
    D0, D1, D2 = fg._safe(j * GE, c["D1"], c["D2"], TS)
    return dict(label=f"{c['set']} {c['ctrl']} {c['P']:g}W {c['V2']:g}V L{j} T25", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], level=j, D0=D0, D1=D1, D2=D2, Ts=TS)


def predicted_levels(cf):
    """[(j, j + 1)]: the first pair from below whose powers (Python plant on the 25 ns grid, -P_LV) bracket |P|; the same duties at
    the same V2 share their evaluations"""
    import pi_steady as ps
    import plant_full as pf
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS * 1e9)
    P, V, D1, D2 = (np.array([c[k] for c in cf]) for k in ("P", "V2", "D1", "D2"))
    j0 = np.round(np.nan_to_num(ps.operating_point(P, D1, D2, V)["D0"]) / GE).astype(int)    # ideal model: only the centre of the scan
    known = {}

    def ev(pairs):
        q = sorted({(V[i], D1[i], D2[i], j) for i, j in pairs} - set(known))
        if not q:
            return
        sd = [fg._safe(j * GE, a, b, TS) for _, a, b, j in q]
        r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]),
                            np.full(len(q), 380.0), np.array([x[0] for x in q]))
        known.update(zip(q, -np.asarray(r["P_LV"], float)))
        print(f"  plant: {len(q)} evaluations", flush=True)
    n = len(cf)
    lo, hi, br = j0 - 4, j0 + 4, [None] * n
    ev([(i, j) for i in range(n) for j in range(lo[i], hi[i] + 1)])
    for _ in range(12):
        todo = []
        for i in range(n):
            if br[i] is not None:
                continue
            js = list(range(lo[i], hi[i] + 1))
            Pd = [known[(V[i], D1[i], D2[i], j)] for j in js]
            br[i] = rm.first_bracket(js, Pd, P[i])
            if br[i] is None:
                if max(Pd) < P[i]:
                    todo += [(i, hi[i] + q) for q in range(1, 5)]
                    hi[i] += 4
                else:
                    todo += [(i, lo[i] - q) for q in range(1, 5)]
                    lo[i] -= 4
        if not todo:
            break
        ev(todo)
    for i, c in enumerate(cf):
        if br[i] is None:
            print(f"  plant does not bracket: {c}")
            br[i] = (int(j0[i]), int(j0[i]) + 1)
    return br


def make():
    cf = configs()
    pts = []
    for c, b in zip(cf, predicted_levels(cf)):
        pts += [_point(c, j) for j in b]
    fg._write(os.path.join(OUT, "edge25_points_r1.csv"), pts)


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "edge25_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results(files=None):
    fs = result_files() if files is None else files
    if not fs:
        return None
    return fg._load(fs, fine=True).drop_duplicates(subset=["V2", "level", "D1", "D2"], keep="last")


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "edge25_sweep_r*.csv"))
          for m in [re.search(r"edge25_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def extend(lv, Pd, P, cap=8):
    """levels to add for a configuration whose measured levels do not bracket P (as r13_map.extend, on the 0.72 deg levels: a flat
    end is below a quarter of its plateau slope, and a step goes further): fill a gap that straddles P, else step past the end"""
    order = np.argsort(lv)
    lv, Pd = np.asarray(lv, int)[order], np.asarray(Pd, float)[order]
    for i in range(len(lv) - 1):
        if min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]) and lv[i + 1] > lv[i] + 1:
            return list(range(lv[i] + 1, lv[i + 1]))
    up = Pd.max() < P
    k = 1
    if len(lv) >= 2:
        s = (Pd[-1] - Pd[-2]) / (lv[-1] - lv[-2]) if up else (Pd[1] - Pd[0]) / (lv[1] - lv[0])
        if not np.isfinite(s) or s <= -rm.PLATEAU / 4:
            return []
        if s >= rm.PLATEAU / 4:
            k = int(np.clip(np.ceil(abs(P - (Pd[-1] if up else Pd[0])) / s), 1, cap))
    end = int(lv[-1] if up else lv[0])
    return [end + (q if up else -q) for q in range(1, k + 1)]


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/edge25_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = fg.runs_of(d, c, TS)
        if g.empty:
            sys.exit(f"no runs at all for {c} -- round 1 not finished?")
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [_point(c, j) for j in extend(g["level"].to_numpy(), -g[PC].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"edge25_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


def _pp():
    """(V2, level, D1, D2) -> (median, largest, smallest period peak), from out/iL_pp/"""
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "edge25_sweep_r*.csv"))):
        T = pd.read_csv(f)
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float),
                        -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T.itertuples(), pk):
            out[(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = (float(np.nanmedian(x)), float(np.nanmax(x)), float(np.nanmin(x)))
    return out


def value(d, pp, c):
    """dict at |P|: rule value, osc, median period peak, loss, D0, peak spread (the larger of the two runs); NaNs if not bracketed"""
    g = fg.runs_of(d, c, TS)
    I, o, lo = fg.at_power(g, c["P"]) if not g.empty else (np.nan, np.nan, None)
    if lo is None:
        return dict(I=np.nan, osc=np.nan, med=np.nan, loss=np.nan, D0=np.nan, spread=np.nan)
    x = g[g["level"].isin([lo, lo + 1])].dropna(subset=[PC, IC])
    t = (c["P"] - (-x[PC].iloc[0])) / ((-x[PC].iloc[1]) - (-x[PC].iloc[0]))
    ms = [pp.get((round(c["V2"], 6), int(r.level), round(r.D1, 9), round(r.D2, 9))) for r in x.itertuples()]
    ok = all(m is not None for m in ms)
    med = ms[0][0] + t * (ms[1][0] - ms[0][0]) if ok else np.nan
    loss = x["P_LV"] - x["P_HV"]
    return dict(I=I, osc=o, med=med, loss=float(loss.iloc[0] + t * (loss.iloc[1] - loss.iloc[0])), D0=float(x["D0"].iloc[0] + t * (x["D0"].iloc[1] - x["D0"].iloc[0])),
                spread=max(m[1] - m[2] for m in ms) if ok else np.nan)


def end_rule(rows):
    """the registered per-end rule on the rows of one end (keys code, code_med, dI, dI_med) -> (verdict, info)"""
    n = len(rows)
    dec = [r for r in rows if r["code"] != "undecided" and r["code_med"] != "undecided"]
    if n == 0 or len(dec) < DECIDABLE * n:
        return "undecided", dict(n_dec=len(dec), n=n)
    w, wm = sum(r["code"] == "worse" for r in dec), sum(r["code_med"] == "worse" for r in dec)
    m, mm = float(np.mean([r["dI"] for r in dec])), float(np.mean([r["dI_med"] for r in dec]))
    info = dict(n_dec=len(dec), n=n, worse=w, worse_med=wm, mean=m, mean_med=mm)
    if (w > BAD_HI * len(dec) and wm > BAD_HI * len(dec)) or (m < -MEAN_BAD and mm < -MEAN_BAD):
        return "broken", info
    if w <= BAD_LO * len(dec) and wm <= BAD_LO * len(dec) and m >= 0 and mm >= 0:
        return "holds", info
    return "partial", info


def control_rule(diff, diff_med):
    if not len(diff):
        return "undecided"
    return "same" if abs(float(np.mean(diff))) <= C_MEAN and abs(float(np.mean(diff_med))) <= C_MEAN else "differs"


def map_values():
    """(V2, P) -> the 25 ns map (2.88 deg levels): dI on the rule value and the median"""
    import pandas as pd
    Q = pd.read_csv(os.path.join(OUT, "r13_points.csv"))
    M = pd.read_csv(os.path.join(OUT, "r13_points_med.csv"))
    out = {}
    for r in Q.itertuples():
        m = M[(M["V2"] == r.V2) & (M["P"] == r.P)].iloc[0]
        out[(r.V2, r.P)] = dict(dI=r.I_SPS - r.I_MB, dI_med=m["med_SPS"] - m["med_MB"])
    return out


def compare(d=None, pp=None, mp=None, write_out=True):
    import datetime

    import pandas as pd
    d = results() if d is None else d
    if d is None:
        sys.exit("no out/edge25_sweep_r*.csv yet")
    pp, mp = _pp() if pp is None else pp, map_values() if mp is None else mp
    by = {(c["set"], c["ctrl"], c["V2"], c["P"]): c for c in configs()}
    rows = []
    for st, V2, P in sorted({(c["set"], c["V2"], c["P"]) for c in by.values()}):
        s, b = value(d, pp, by[(st, "SPS", V2, P)]), value(d, pp, by[(st, "M-B", V2, P)])
        cr, dI, tol = rc.classify(s["I"], b["I"], s["osc"], b["osc"])
        cm, dIm, tolm = rc.classify(s["med"], b["med"], s["osc"], b["osc"])
        rows.append(dict(set=st, V2=V2, P=P, k=4 * V2 / 380.0, D1=by[(st, "M-B", V2, P)]["D1"], D2=by[(st, "M-B", V2, P)]["D2"], SPS=s["I"], MB=b["I"], osc_SPS=s["osc"], osc_MB=b["osc"], dI=dI, tol=tol, code=cr,
                         med_SPS=s["med"], med_MB=b["med"], dI_med=dIm, tol_med=tolm, code_med=cm, loss_SPS=s["loss"], loss_MB=b["loss"], eta_SPS=P / (P + s["loss"]), eta_MB=P / (P + b["loss"]),
                         D0_SPS=s["D0"], D0_MB=b["D0"], spread_SPS=s["spread"], spread_MB=b["spread"]))
    T = pd.DataFrame(rows)
    E, C = T[T["set"] == "E"], T[T["set"] == "C"]
    light, lt = end_rule(E[E["P"].isin(LIGHT)].to_dict("records"))
    heavy, ht = end_rule(E[E["P"].isin(HEAVY)].to_dict("records"))
    cd = C[(C["code"] != "undecided") & (C["code_med"] != "undecided")]
    diff = np.array([r.dI - mp[(r.V2, r.P)]["dI"] for r in cd.itertuples()]) if len(cd) else np.array([])
    diffm = np.array([r.dI_med - mp[(r.V2, r.P)]["dI_med"] for r in cd.itertuples()]) if len(cd) else np.array([])
    ctl = control_rule(diff, diffm)
    n_open = int((~np.isfinite(T["SPS"]) | ~np.isfinite(T["MB"])).sum())

    def line(r):
        return (f" {r.V2:6.1f} {r.P:6.0f} | SPS {r.SPS:6.3f}  B {r.MB:6.3f}  dI {r.dI:+6.3f} tol {r.tol:5.3f} {r.code:9s} | median {r.med_SPS:6.3f} {r.med_MB:6.3f} {r.dI_med:+6.3f} {r.code_med:9s} | "
                f"eta {100 * r.eta_SPS:5.1f} {100 * r.eta_MB:5.1f} % loss {r.loss_SPS:5.2f} {r.loss_MB:5.2f} W | D0 {r.D0_SPS:6.2f} {r.D0_MB:6.2f} | spread {r.spread_SPS:5.3f} {r.spread_MB:5.3f}")
    lines = ["edge25: the ends of the power range, HV->LV, 25 ns Simulink (dataset-era model), dI = Ipk(SPS) - Ipk(M-B) [A]; rule in the file header", "", "E"]
    lines += [line(r) for r in E.itertuples()]
    lines += ["", "C (control: the 25 ns map's points on 0.72 deg levels)"] + [line(r) for r in C.itertuples()]
    lines += ["", f"light end (25-75 W): {light} {lt}", f"heavy end (1200 W): {heavy} {ht}", f"control: {ctl}, dI - map mean {np.mean(diff) if len(diff) else np.nan:+.3f} A, median {np.mean(diffm) if len(diffm) else np.nan:+.3f} A, {len(diff)} points"]
    for P, g in E.groupby("P"):
        x = g[g["code"] != "undecided"]
        lines.append(f"  |P| {P:5.0f} W: decidable {len(x)}/{len(g)}, mean dI {x['dI'].mean():+.3f} A, median peak {x['dI_med'].mean():+.3f} A, relative {100 * (x['dI'] / x['SPS']).mean():+.1f} %, "
                     f"loss B - SPS {(x['loss_MB'] - x['loss_SPS']).mean():+.2f} W, eta B - SPS {100 * (x['eta_MB'] - x['eta_SPS']).mean():+.2f} pp")
    if n_open:
        lines.append(f"{n_open} points not bracketed yet -> provisional")
    md = ["# 사양 전력 범위의 양 끝, HV→LV, 25 ns 격자 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `edge25.py compare`. 규칙은 실행 전(10-31 02:15) `edge25.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), 상수 D0 20 ms, 0.72° 간격 두 계단 전력 보간. Python 플랜트는 계단 선택에만 썼다.", "",
          "| V2 | \\|P\\| | k | B 듀티 | SPS [A] | B [A] | ΔI [A] | 판정 | 중앙값 SPS, B [A] | ΔI(중앙값) | 판정(중앙값) | η SPS, B [%] |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.V2:g} V | {r.P:.0f} W{' (대조)' if r.set == 'C' else ''} | {r.k:.3f} | ({r.D1:.3f}, {r.D2:.3f}) | {r.SPS:.3f} | {r.MB:.3f} | {r.dI:+.3f} | {CODE[r.code]} | {r.med_SPS:.3f}, {r.med_MB:.3f} | {r.dI_med:+.3f} | {CODE[r.code_med]} | {100 * r.eta_SPS:.1f}, {100 * r.eta_MB:.1f} |")
    md += ["", f"**경부하 끝 25–75 W: {END_TXT[light]}** {lt}", f"**고부하 끝 1200 W: {END_TXT[heavy]}** {ht}", f"**대조(0.72° 계단 − 지도의 2.88° 계단): {'간격 영향 없음' if ctl == 'same' else '간격 영향 있음' if ctl == 'differs' else '판정 불가'}**, ΔI 차 평균 {np.mean(diff) if len(diff) else np.nan:+.3f} A, 중앙값 {np.mean(diffm) if len(diffm) else np.nan:+.3f} A."]
    if n_open:
        md.append(f"※ 아직 감싸지 못한 점 {n_open}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "edge25_check.csv"), index=False, float_format="%.5g")
        open(os.path.join(OUT, "edge25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        open(os.path.join(OUT, "edge25_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, light, heavy, ctl


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
    chk("38 points x 2 controllers", len(cf), 76)
    chk("level 3 is 3 grid steps minus the nudge", _point(cf[0], 3)["D0"], round(3 * GE - fg.NUDGE, 6))
    mk = lambda c, cm, a, am: dict(code=c, code_med=cm, dI=a, dI_med=am)  # noqa: E731
    chk("end: all better -> holds", end_rule([mk("better", "better", 0.2, 0.2)] * 21)[0], "holds")
    chk("end: 2 of 21 worse -> holds", end_rule([mk("worse", "worse", -0.1, -0.1)] * 2 + [mk("better", "better", 0.2, 0.2)] * 19)[0], "holds")
    chk("end: 3 of 21 worse -> partial", end_rule([mk("worse", "worse", -0.1, -0.1)] * 3 + [mk("better", "better", 0.2, 0.2)] * 18)[0], "partial")
    chk("end: 6 of 21 worse -> broken", end_rule([mk("worse", "worse", -0.1, -0.1)] * 6 + [mk("better", "better", 0.2, 0.2)] * 15)[0], "broken")
    chk("end: worse on one value only -> partial", end_rule([mk("worse", "same", -0.1, 0.0)] * 6 + [mk("better", "better", 0.2, 0.2)] * 15)[0], "partial")
    chk("end: mean below -0.03 on both -> broken", end_rule([mk("same", "same", -0.04, -0.04)] * 7)[0], "broken")
    chk("end: 5 of 7 decidable -> undecided", end_rule([mk("undecided", "undecided", 0.0, 0.0)] * 2 + [mk("better", "better", 0.2, 0.2)] * 5)[0], "undecided")
    chk("end: 6 of 7 decidable, none worse -> holds", end_rule([mk("undecided", "undecided", 0.0, 0.0)] + [mk("better", "better", 0.2, 0.2)] * 6)[0], "holds")
    chk("control: small mean -> same", control_rule(np.array([0.02, -0.02]), np.array([0.01, 0.0])), "same")
    chk("control: large mean on the median -> differs", control_rule(np.array([0.0, 0.0]), np.array([0.05, 0.05])), "differs")
    chk("extend fills a gap", extend([2, 5], [10.0, 40.0], 25.0), [3, 4])
    chk("extend steps up with the slope", extend([2, 3], [10.0, 20.0], 45.0), [4, 5, 6])
    chk("extend steps down", extend([2, 3], [10.0, 20.0], -5.0), [1, 0])
    chk("extend stops past the peak", extend([2, 3], [20.0, 10.0], 45.0), [])

    def runs(worse=(), osc_at=()):
        rows = []
        seen = set()
        for c in cf:
            pt = (c["V2"], c["P"])
            m0 = int(c["P"] // 10) + 3
            base = 1.0 + c["P"] / 1000.0 - (0.0 if c["ctrl"] == "SPS" else (-0.2 if pt in worse else 0.1))
            for m, Pr in ((m0, 10.0 * (m0 - 3) - 5.0), (m0 + 1, 10.0 * (m0 - 3) + 5.0)):
                k = (c["V2"], m, c["D1"], c["D2"])
                if k in seen:
                    continue
                seen.add(k)
                rows.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=m, D0=m * GE, D1=c["D1"], D2=c["D2"], Ts=TS, **{PC: -Pr, IC: base}, iL_mean=0.05 if pt in osc_at and c["ctrl"] == "M-B" else 0.001,
                                 P_LV=-Pr, P_HV=-Pr - 3.0, error=""))
        return pd.DataFrame(rows)

    def pp_of(F):
        return {(round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9)): (getattr(r, IC), getattr(r, IC), getattr(r, IC)) for r in F.itertuples()}
    mp = {pt: dict(dI=0.1, dI_med=0.1) for pt in C_PTS}
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "fake.csv")
        F = runs()
        F.to_csv(f, index=False)
        T, light, heavy, ctl = compare(results([f]), pp_of(F), mp, write_out=False)
        chk("fake: every point bracketed", int(T["SPS"].notna().sum()), 38)
        chk("fake: reduction everywhere -> light holds", light, "holds")
        chk("fake: reduction everywhere -> heavy holds", heavy, "holds")
        chk("fake: control equal to the map", ctl, "same")
        chk("fake: the efficiency uses the loss", round(float(T["eta_SPS"].iloc[0]), 6), round(float(T["P"].iloc[0] / (T["P"].iloc[0] + 3.0)), 6))
        F = runs(worse=[(v, 25.0) for v in V2S[:6]], osc_at=[(109.2, 1200.0), (105.0, 1200.0)])
        F.to_csv(f, index=False)
        T, light, heavy, ctl = compare(results([f]), pp_of(F), mp, write_out=False)
        chk("fake: 6 of 21 worse -> light broken", light, "broken")
        chk("fake: 2 of 7 oscillating -> heavy undecided", heavy, "undecided")
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
