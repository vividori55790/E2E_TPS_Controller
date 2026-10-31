"""
v1x_check.py -- (2026-10-31 01:32) V1 +-5 % at the V2 where k crosses 1: does M-B go above SPS there, and does the
V1-normalised input fix it?

R10 on the 25 ns grid (robust25.py, final 10-31 01:25) has V2 only at the two ends of the range. Its one "열화" (V1 -5 %, 84 V
300 W, -0.088 A, periodic runs, the same point as at 100 ns) is the one case where V1 moves k toward 1. M-B does not know V1:
at an interior V2 a 5 % change brings k to 1 while M-B still outputs the TPS duties of the nominal k, where SPS has no
circulating current to remove. Reference only (ideal model, %TEMP%\\v1x_pred.py, seen before this rule; not Simulink): worst
points V1 361 V 90.3 V 100 W SPS 0.287 / M-B 0.446 A, V1 399 V 100.8 V 100 W 0.396 / 0.528 A, and the normalised input is
back at SPS +0.000...+0.003 A. So "one point" may be an artefact of the two V2 of R10. The 100 ns judgement of the normalised
input ("109.2 V 쪽이 나빠 채택 안 함", robust_check.py note of 10-30 13:00, six cases, reported alongside) was made on a grid whose
0.1 A differences have since gone, and on oscillating runs.

Runs (25 ns, constant D0 20 ms, 20 ms window, two adjacent 2.88-deg levels bracketing |P|, Ipk interpolated in power, as in
robust25.py; verify_steady_fine_param.m; the Python plant with the variant's V1 only picks the levels):
  set A = V1 361 V x V2 {88.2, 90.3, 92.4} (k 0.977, 1.001, 1.024), V1 399 V x V2 {98.7, 100.8, 102.9} (k 0.989, 1.011, 1.032)
          x |P| {100, 200, 300} x {SPS, M-B, M-Bk}. 18 cases.
  set B = V1 361 and 399 V x V2 {84, 109.2} x |P| {100, 300, 1000} x {M-Bk}; SPS and M-B there are the rob25 runs. 12 cases.
M-Bk = M-B fed (|P| (380/V1)^2, V2 380/V1), smooth_reghdc_infer.m's optional V1 and smooth_reghdc_v1_equiv() of the C export;
the encoder clamps V2 to 84-109.2 V (84 V at V1 399 V, 109.2 V at V1 361 V sit on that clamp). At V1 = 380 V, M-Bk = M-B exactly.
Duties within 0.003 of SPS (M-Bk: or of M-B) use those runs, as on the map (r13_map.SAME); added 01:36, before any run.

RULE (registered 2026-10-31 01:32, before any run of this file; do not change after the results). Two values per run: the
80-period maximum (the rule value of every earlier check) and the median period peak (out/iL_pp/; why: r13_map.py header, note
of 10-30 18:15). Per case and controller X, on each value: robust_check.classify(SPS, X) = "판정 불가" if not bracketed or
|mean i_L| > 20 mA in a bracketing run, dI = Ipk(SPS) - Ipk(X), tol = max(0.03 A, 2 % Ipk(SPS)), "열화" if dI < -tol, "같음" if
|dI| <= tol, "저감" if dI > tol.
  Q1 (set A, X = M-B): does the V1 tolerance push M-B above SPS where k crosses 1?
     "실재" if "열화" in >= 3 cases on both values; "없음" if in 0 on both; "일부" otherwise.
  Q2 (sets A + B, cases decidable for both M-B and M-Bk; gap = Ipk(M-B) - Ipk(M-Bk)): does the normalised input earn its place?
     a. N_worse(M-Bk) <= N_worse(M-B) on each value, and fewer in total over the two values
     b. mean gap >= +0.01 A on both values
     c. gap < -tol in at most 2 cases on each value
     "판정 불가" if fewer than 75 % of the 30 are decidable for both; otherwise "채택" if a, b and c; "근거 없음" if not a; "부분" otherwise.
Reported alongside: every case, k, the duties, mean dI per V1 and per set. This replaces the 100 ns judgement of the normalised
input; set A is new, so "the one 열화 point" of R10 is reported with Q1 next to it, and R10's own verdict stays as registered.

    python -I -X utf8 Simulink/Thesis/v1x_check.py make      -> out/v1x_points_r1.csv
    python automation/job.py submit v1x --interactive --after g12 -- matlab -batch "cd('Simulink/Thesis'); v1x_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/v1x_check.py next      (not bracketed -> next round; v1x_rounds calls it)
    python -I -X utf8 Simulink/Thesis/v1x_check.py compare   -> out/v1x_check.{csv,log}, out/v1x_verdict.md
    python -I -X utf8 Simulink/Thesis/v1x_check.py fig       -> out/fig_v1x.png (set A against k; added after the results, not a verdict)
    python -I -X utf8 Simulink/Thesis/v1x_check.py side      -> out/v1x_side.log (means, R10's rule on M-Bk, the 100 W line, V1 reading error; added after the results, not a verdict)
    python -I -X utf8 Simulink/Thesis/v1x_check.py selftest
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

OUT = os.path.join(HERE, "out")
WIN = fg.WIN
MK = "M-Bk"
A_V2 = {"V1-5": (88.2, 90.3, 92.4), "V1+5": (98.7, 100.8, 102.9)}
A_P = (100.0, 200.0, 300.0)
B_V2 = (84.0, 109.2)
B_P = (100.0, 300.0, 1000.0)
Q1_REAL = 3
GAP_MIN = 0.01
GAP_WORSE_MAX = 2
DECIDABLE = 0.75
PATS = ("rob25_sweep_r*.csv", "v1x_sweep_r*.csv")
CODE = {"worse": "열화", "same": "같음", "better": "저감", "undecided": "판정 불가"}
Q1_TXT = {"real": "실재", "none": "없음", "some": "일부"}
Q2_TXT = {"adopt": "채택", "no": "근거 없음", "partial": "부분", "undecided": "판정 불가"}


def _duties(mb, P, V2, r):
    d1, d2 = mb.predict(np.array([P * r * r]), np.array([V2 * r]))
    return round(min(float(np.ravel(d1)[0]), sw.D_SAFE), 9), round(min(float(np.ravel(d2)[0]), sw.D_SAFE), 9)


def cases():
    """[(set, variant, V2, P)]"""
    a = [("A", v, V2, P) for v, vs in A_V2.items() for V2 in vs for P in A_P]
    return a + [("B", v, V2, P) for v in A_V2 for V2 in B_V2 for P in B_P]


def _near(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1])) <= rm.SAME


def controllers(mb, v, V2, P):
    """(ctrl -> (D1, D2)) of one case; duties within SAME of SPS (or M-Bk of M-B) use those runs, as on the map"""
    r = rc.NOM["V1"] / rc.params(v)["V1"]
    sps, b, k = (sw.D_SAFE, sw.D_SAFE), _duties(mb, P, V2, 1.0), _duties(mb, P, V2, r)
    b = sps if _near(b, sps) else b
    k = sps if _near(k, sps) else b if _near(k, b) else k
    return {"SPS": sps, "M-B": b, MK: k}


def configs(own_only=True):
    mb, cf = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json")), []
    for st, v, V2, P in cases():
        ds = controllers(mb, v, V2, P)
        for ctrl, (d1, d2) in ds.items():
            if own_only and st == "B" and (ctrl != MK or ds[MK] in (ds["SPS"], ds["M-B"])):
                continue                                  # SPS and M-B of set B are the rob25 runs
            cf.append(dict(ctrl=ctrl, V2=V2, P=P, D1=d1, D2=d2, variant=v, **rc.params(v)))
    return cf


def make():
    cf = configs()
    pts = []
    for c, b in zip(cf, rb.predicted_levels(cf)):
        pts += [rb.point(c, j) for j in b]
    rb.write(os.path.join(OUT, "v1x_points_r1.csv"), pts)


def own_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "v1x_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results():
    return rb.results(rb.result_files() + own_files())


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "v1x_sweep_r*.csv"))
          for m in [re.search(r"v1x_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    pts = []
    for c in configs():
        g = rb.runs_of(d, c)
        if g.empty:
            sys.exit(f"no runs at all for {c} -- round 1 not finished?")
        if np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [rb.point(c, j) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        rb.write(os.path.join(OUT, f"v1x_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def rule_q1(codes_max, codes_med):
    n, m = codes_max.count("worse"), codes_med.count("worse")
    if n >= Q1_REAL and m >= Q1_REAL:
        return "real"
    return "none" if n == 0 and m == 0 else "some"


def rule_q2(T, n_cases):
    """T: cases decidable for both M-B and M-Bk, columns code_b, code_k, code_b_med, code_k_med, gap, gap_med, tol, tol_med"""
    if n_cases == 0 or len(T) < DECIDABLE * n_cases:
        return "undecided", {}
    nb, nk = int((T["code_b"] == "worse").sum()), int((T["code_k"] == "worse").sum())
    mb, mk = int((T["code_b_med"] == "worse").sum()), int((T["code_k_med"] == "worse").sum())
    a = nk <= nb and mk <= mb and nk + mk < nb + mb
    b = T["gap"].mean() >= GAP_MIN and T["gap_med"].mean() >= GAP_MIN
    wg, wm = int((T["gap"] < -T["tol"]).sum()), int((T["gap_med"] < -T["tol_med"]).sum())
    c = wg <= GAP_WORSE_MAX and wm <= GAP_WORSE_MAX
    info = dict(nb=nb, nk=nk, mb=mb, mk=mk, a=a, b=b, c=c, gap=float(T["gap"].mean()), gap_med=float(T["gap_med"].mean()), wg=wg, wm=wm)
    return ("no" if not a else "adopt" if b and c else "partial"), info


def value(d, pp, c):
    """(Ipk rule value, osc, median period peak) at |P|"""
    g = rb.runs_of(d, c)
    I, o, lo = fg.at_power(g, c["P"]) if not g.empty else (np.nan, np.nan, None)
    med = np.nan
    if lo is not None:
        x = g[g["level"].isin([lo, lo + 1])]
        ms = [pp.get((c["variant"], round(c["V2"], 6), int(r.level), round(r.D1, 9), round(r.D2, 9))) for r in x.itertuples()]
        if all(m is not None for m in ms):
            Pd = -x[f"P_LV_at_{WIN}ms"].to_numpy()
            med = ms[0] + (c["P"] - Pd[0]) / (Pd[1] - Pd[0]) * (ms[1] - ms[0])
    return I, o, med


def compare(d=None, pp=None, write_out=True):
    import datetime

    import pandas as pd
    d = results() if d is None else d
    pp = rb._pp(PATS) if pp is None else pp
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    rows = []
    for st, v, V2, P in cases():
        ds = controllers(mb, v, V2, P)
        val = {k: value(d, pp, dict(ctrl=k, V2=V2, P=P, D1=x[0], D2=x[1], variant=v)) for k, x in ds.items()}
        s, b, k = val["SPS"], val["M-B"], val[MK]
        cb, db, tol = rc.classify(s[0], b[0], s[1], b[1])
        ck, dk, _ = rc.classify(s[0], k[0], s[1], k[1])
        cbm, dbm, tolm = rc.classify(s[2], b[2], s[1], b[1])
        ckm, dkm, _ = rc.classify(s[2], k[2], s[1], k[1])
        rows.append(dict(set=st, variant=v, V2=V2, P=P, k=4 * V2 / rc.params(v)["V1"], SPS=s[0], MB=b[0], MBk=k[0], osc_SPS=s[1], osc_MB=b[1], osc_MBk=k[1],
                         dI_b=db, dI_k=dk, tol=tol, code_b=cb, code_k=ck, med_SPS=s[2], med_MB=b[2], med_MBk=k[2], dI_b_med=dbm, dI_k_med=dkm, tol_med=tolm,
                         code_b_med=cbm, code_k_med=ckm, gap=b[0] - k[0], gap_med=b[2] - k[2], D1=ds["M-B"][0], D2=ds["M-B"][1], D1k=ds[MK][0], D2k=ds[MK][1]))
    T = pd.DataFrame(rows)
    n_open = int((~np.isfinite(T["SPS"]) | ~np.isfinite(T["MB"]) | ~np.isfinite(T["MBk"])).sum())
    A = T[T["set"] == "A"]
    q1 = rule_q1(list(A["code_b"]), list(A["code_b_med"]))
    both = T[(T["code_b"] != "undecided") & (T["code_k"] != "undecided") & np.isfinite(T["gap_med"])]
    q2, info = rule_q2(both, len(T))
    lines = ["v1x_check: V1 +-5 %, 25 ns Simulink (dataset-era model), HV->LV, dI = Ipk(SPS) - Ipk(X) [A]; rule in the file header", "",
             " set variant   V2      k     |P| |  SPS    M-B    M-Bk |  dI(B)  dI(Bk)   tol | code B      code Bk     | median dI(B) dI(Bk) | (D1, D2)  (D1k, D2k)"]
    for r in T.itertuples():
        lines.append(f"  {r.set}  {r.variant:6s} {r.V2:6.1f} {r.k:6.3f} {r.P:6.0f} | {r.SPS:5.3f}  {r.MB:5.3f}  {r.MBk:5.3f} | {r.dI_b:+6.3f}  {r.dI_k:+6.3f}  {r.tol:5.3f} | "
                     f"{r.code_b:11s} {r.code_k:11s} | {r.dI_b_med:+6.3f} {r.dI_k_med:+6.3f} | ({r.D1:.3f}, {r.D2:.3f}) ({r.D1k:.3f}, {r.D2k:.3f})")
    lines += ["", f"Q1 (set A, M-B vs SPS): worse {list(A['code_b']).count('worse')} (rule value), {list(A['code_b_med']).count('worse')} (median) of {len(A)}: {q1}",
              f"Q2 ({len(both)}/{len(T)} decidable for both): {q2} {info}"]
    if n_open:
        lines.append(f"{n_open} cases not bracketed yet -> provisional")
    md = ["# V1 ±5 %, k가 1을 지나는 V2와 V1 정규화 입력 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `v1x_check.py compare`. 규칙은 실행 전(10-31 01:32) `v1x_check.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), 상수 D0 20 ms, 두 계단 전력 보간. "
          "Python 플랜트는 계단 선택에만 썼다.", "",
          "| 묶음 | 변형 | V2 | k | \\|P\\| | SPS [A] | B [A] | 정규화 B [A] | ΔI B | ΔI 정규화 | 허용 | B 판정 | 정규화 판정 | ΔI B(중앙값) | ΔI 정규화(중앙값) |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.set} | {r.variant} | {r.V2:g} V | {r.k:.3f} | {r.P:.0f} W | {r.SPS:.3f} | {r.MB:.3f} | {r.MBk:.3f} | {r.dI_b:+.3f} | {r.dI_k:+.3f} | {r.tol:.3f} | {CODE[r.code_b]} | {CODE[r.code_k]} | {r.dI_b_med:+.3f} | {r.dI_k_med:+.3f} |")
    md += ["", f"**Q1 (묶음 A, B 대 SPS): {Q1_TXT[q1]}.** 열화 규칙 값 {list(A['code_b']).count('worse')}, 중앙값 {list(A['code_b_med']).count('worse')} / {len(A)}경우.",
           f"**Q2 (둘 다 판정 가능 {len(both)}/{len(T)}경우): {Q2_TXT[q2]}.** " + (f"열화 B {info['nb']}·{info['mb']}, 정규화 {info['nk']}·{info['mk']}(규칙 값·중앙값), 평균 gap {info['gap']:+.3f}·{info['gap_med']:+.3f} A, 허용 넘게 나빠진 경우 {info['wg']}·{info['wm']}." if info else "")]
    if n_open:
        md.append(f"※ 아직 감싸지 못한 경우 {n_open}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "v1x_check.csv"), index=False, float_format="%.5g")
        open(os.path.join(OUT, "v1x_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        open(os.path.join(OUT, "v1x_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, q1, q2


def fig():
    """set A: reduction of M-B and M-Bk against k, one panel per |P|. Added after the results, not a verdict. -> out/fig_v1x.png"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    from evaluate import BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    T = pd.read_csv(os.path.join(OUT, "v1x_check.csv"))
    A = T[T["set"] == "A"].sort_values("k")
    S = "#fcfcfb"
    series = (("dI_b", "dI_b_med", "code_b", BLUE, "o"), ("dI_k", "dI_k_med", "code_k", ORANGE, "s"))
    fig_, ax = plt.subplots(1, len(A_P), figsize=(11.5, 4.4), sharey=True)
    for a, P in zip(ax, A_P):
        g = A[A["P"] == P]
        a.axhspan(-0.03, 0.03, color=MUTED, alpha=0.15, lw=0, zorder=0)
        a.axvline(1.0, color=MUTED, lw=0.8, ls="--", zorder=1)
        a.axhline(0.0, color=MUTED, lw=0.8, zorder=1)
        for y, ym, code, c, m in series:
            ok = g[g[code] != "undecided"]
            a.vlines(g["k"], g[y], g[ym], color=c, lw=1.0, zorder=2)
            a.scatter(g["k"], g[ym], s=26, marker="_", color=c, linewidths=1.3, zorder=3)
            for v, face, ls in (("V1-5", c, "-"), ("V1+5", S, "--")):
                h = ok[ok["variant"] == v]
                a.plot(h["k"], h[y], color=c, lw=1.5, ls=ls, zorder=2)
                a.scatter(h["k"], h[y], s=52, marker=m, facecolors=face, edgecolors=c if face == S else S, linewidths=1.4, zorder=4)
            u = g[g[code] == "undecided"]
            a.scatter(u["k"], u[y], s=52, marker="x", color=c, linewidths=1.6, zorder=4)
        a.set_title(f"|P| = {P:g} W", loc="left")
        a.set_xlabel("k = n·V2 / V1")
        a.set_xlim(0.970, 1.038)
    ax[0].set_ylabel("Ipk(SPS) − Ipk(controller) [A]\n(below 0: higher than SPS)")
    ax[0].text(0.9725, -0.205, "B, input (|P|, V2)", color=INK2, fontsize=9)
    ax[0].text(0.9725, 0.042, "B, normalised input", color=INK2, fontsize=9)
    hs = [Line2D([], [], color=BLUE, lw=1.5, marker="o", markeredgecolor=S, ms=7, label="B, input (|P|, V2)"),
          Line2D([], [], color=ORANGE, lw=1.5, marker="s", markeredgecolor=S, ms=7, label="B, input (|P|·(380/V1)², V2·380/V1)"),
          Line2D([], [], color=INK2, lw=1.5, marker="o", markeredgecolor=S, ms=7, label="V1 361 V"),
          Line2D([], [], color=INK2, lw=1.5, ls="--", marker="o", markerfacecolor=S, markeredgecolor=INK2, markeredgewidth=1.4, ms=7, label="V1 399 V"),
          Line2D([], [], ls="", marker="_", color=INK2, markeredgewidth=1.3, ms=7, label="median period peak (marker: 80-period maximum)"),
          Line2D([], [], ls="", marker="x", color=INK2, markeredgewidth=1.6, ms=7, label="i_L mean > 20 mA, not judged"),
          Patch(facecolor=MUTED, alpha=0.15, label="allowance ±0.03 A")]
    ax[2].legend(handles=hs, fontsize=8.5, frameon=False, loc="lower right")
    fig_.suptitle("Simulink steady state, 25 ns grid, V1 ±5 %: near k = 1 controller B goes above SPS, the normalised input stays at SPS", fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_v1x.png"), dpi=130)
    plt.close(fig_)


def side():
    """numbers around the verdict, computed after the results; none of them is a verdict. -> out/v1x_side.log"""
    import pandas as pd
    T = pd.read_csv(os.path.join(OUT, "v1x_check.csv"))
    R = pd.read_csv(os.path.join(OUT, "rob25_check.csv"))
    both = T[(T["code_b"] != "undecided") & (T["code_k"] != "undecided") & np.isfinite(T["gap_med"])]
    lines = ["v1x_check side: not a verdict (rule and verdict: compare), computed after the results", "",
             "mean reduction [A], cases decidable for that controller: dI B, dI M-Bk (80-period maximum), then the median period peak"]
    for name, g in [(f"{st} {v}", T[(T["set"] == st) & (T["variant"] == v)]) for st in "AB" for v in A_V2] + [(st, T[T["set"] == st]) for st in "AB"]:
        b, k = g[g["code_b"] != "undecided"], g[g["code_k"] != "undecided"]
        lines.append(f"  {name:8s} n {len(b):2d} / {len(k):2d}: {b['dI_b'].mean():+.3f} {k['dI_k'].mean():+.3f} | {b['dI_b_med'].mean():+.3f} {k['dI_k_med'].mean():+.3f}")
    lines += ["", "gap = Ipk(M-B) - Ipk(M-Bk), cases decidable for both: maximum, median"]
    for name, g in [(st, both[both["set"] == st]) for st in "AB"] + [("all", both)]:
        lines.append(f"  {name:4s} n {len(g):2d}: {g['gap'].mean():+.3f} {g['gap_med'].mean():+.3f}")
    step = {v: rc.params(v)["V1"] * 25e-9 / (rc.NOM["L_uH"] * 1e-6) for v in A_V2}
    A = T[T["set"] == "A"]
    far = A[(A["dI_b"] < -A["variant"].map(step)) & (A["dI_b_med"] < -A["variant"].map(step))]
    lines += ["", f"set A: M-B higher than SPS by more than one grid step V1 Ts/L on both values: {len(far)} cases"]
    lines += [f"  {r.variant} {r.V2:g} V {r.P:.0f} W: {r.dI_b:+.3f} {r.dI_b_med:+.3f} (step {step[r.variant]:.3f})" for r in far.itertuples()]
    # R10's rule (robust_check.verdict) with M-Bk in place of M-B: at L +-10 % and td 200 ns V1 = 380 V, so M-Bk = M-B
    codes = list(R[~R["variant"].isin(A_V2)]["code"]) + list(T[T["set"] == "B"]["code_k"])
    lines += ["", f"R10's rule on M-Bk (18 cases of rob25 at V1 = 380 V + set B): {rc.verdict(codes)}; " + ", ".join(f"{c} {codes.count(c)}" for c in ("better", "same", "worse", "undecided"))]
    # the 100 W line of robust25.py fig (fitted on nominal V2 and on M-B at V1 +-5 %), against the M-Bk runs that were not in it
    s, b = 8.55, -0.452
    lines += ["", f"M-Bk at 100 W, set B, against the line dI = {s} |1-k| {b} of robust25.py fig (not refitted): k, dI M-Bk, residual, residual on the median"]
    for r in T[(T["set"] == "B") & (T["P"] == 100.0)].itertuples():
        x = abs(1.0 - r.k)
        lines.append(f"  k {r.k:.3f} ({r.code_k}): {r.dI_k:+.3f} {r.dI_k - (s * x + b):+.3f} {r.dI_k_med - (s * x + b):+.3f}")
    # how far a wrong V1 reading moves the commanded duties: a property of the model, no plant
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    P, V2 = (x.ravel() for x in np.meshgrid(np.linspace(25.0, 1200.0, 48), np.linspace(84.0, 109.2, 64)))

    def duties(V1):
        r = rc.NOM["V1"] / V1
        d1, d2 = mb.predict(P * r * r, V2 * r)
        return np.ravel(d1), np.ravel(d2)
    lines += ["", f"V1 reading error, {P.size} points 25-1200 W x 84-109.2 V, max over D1, D2 and both signs of |dD|: largest, 99th percentile, median"]
    for V1 in (361.0, 380.0, 399.0):
        base = duties(V1)
        for err in (0.005, 0.01, 0.02):
            d = np.max([np.abs(base[i] - duties(V1 * (1 + sg * err))[i]) for i in (0, 1) for sg in (-1.0, 1.0)], axis=0)
            lines.append(f"  V1 {V1:.0f} V, {100 * err:.1f} %: {d.max():.4f} {np.percentile(d, 99):.4f} {np.median(d):.4f}")
    open(os.path.join(OUT, "v1x_side.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))


def selftest():
    """the registered rule end to end on made-up Simulink runs"""
    import tempfile

    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("30 cases", len(cases()), 30)
    cf_own, cf_all = configs(), configs(own_only=False)
    chk("own runs at most A x 3 + B x M-Bk", len(cf_own) <= 18 * 3 + 12, True)
    chk("all configurations", len(cf_all), 30 * 3)
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    chk("M-Bk at k = 1 uses the SPS runs", controllers(mb, "V1-5", 90.3, 100.0)[MK], (sw.D_SAFE, sw.D_SAFE))
    chk("M-Bk = M-B at the normalised input", controllers(mb, "V1+5", 109.2, 300.0)[MK],
        _duties(mb, 300.0 * (380.0 / 399.0) ** 2, 109.2 * 380.0 / 399.0, 1.0))
    chk("Q1 real", rule_q1(["worse"] * 3 + ["same"] * 15, ["worse"] * 3 + ["same"] * 15), "real")
    chk("Q1 not real on the median alone", rule_q1(["worse"] * 3 + ["same"] * 15, ["worse"] * 2 + ["same"] * 16), "some")
    chk("Q1 none", rule_q1(["same"] * 18, ["better"] * 18), "none")

    def q2t(nb, nk, gap, wg=0, n=30):
        """n cases: nb M-B worse, nk M-Bk worse, mean gap, wg of them gap < -tol, the same on the median"""
        g = np.full(n, (gap * n + 0.1 * wg) / (n - wg))
        g[:wg] = -0.1
        return pd.DataFrame(dict(code_b=["worse"] * nb + ["same"] * (n - nb), code_k=["worse"] * nk + ["same"] * (n - nk), code_b_med=["worse"] * nb + ["same"] * (n - nb),
                                 code_k_med=["worse"] * nk + ["same"] * (n - nk), gap=g, gap_med=g, tol=0.03, tol_med=0.03))
    chk("Q2 adopt", rule_q2(q2t(4, 0, 0.05), 30)[0], "adopt")
    chk("Q2 no: M-Bk no better", rule_q2(q2t(4, 4, 0.05), 30)[0], "no")
    chk("Q2 no: nothing to fix", rule_q2(q2t(0, 0, 0.05), 30)[0], "no")
    chk("Q2 partial: mean gap small", rule_q2(q2t(4, 0, 0.005), 30)[0], "partial")
    chk("Q2 partial: breaks three others", rule_q2(q2t(4, 1, 0.05, wg=3), 30)[0], "partial")
    chk("Q2 undecided", rule_q2(q2t(4, 0, 0.05, n=22), 30)[0], "undecided")

    def runs(worse_b):
        rows = []
        for c in cf_all:
            is_sps = c["D1"] == sw.D_SAFE and c["D2"] == sw.D_SAFE
            is_b = c["ctrl"] == "M-B" and (c["variant"], c["V2"], c["P"]) in worse_b
            base = 2.0 + c["P"] / 1000.0 + (0.0 if is_sps else 0.2 if is_b else -0.05 if c["ctrl"] == MK else 0.0)
            j0 = int(c["P"] // 50)
            for j, Pd in ((j0, 50.0 * j0 - 25.0), (j0 + 1, 50.0 * j0 + 25.0)):
                rows.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=j, D0=j * fg.G, D1=c["D1"], D2=c["D2"], Ts=rm.TS, variant=c["variant"], V1=c["V1"],
                                 **{f"P_LV_at_{WIN}ms": -Pd, f"Ipk_at_{WIN}ms": base}, iL_mean=0.001, P_LV=-Pd, error=""))
        return pd.DataFrame(rows)

    def pp_of(F):
        return {(r.variant, round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9)): getattr(r, f"Ipk_at_{WIN}ms") for r in F.itertuples()}
    wb = {(v, V2, 100.0) for v in A_V2 for V2 in A_V2[v][:2]}
    F = runs(wb)
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "fake.csv")
        F.to_csv(f, index=False)
        T, q1, q2 = compare(rb.results([f]), pp_of(F), write_out=False)
    chk("fake: M-B worse at 4 set A cases -> Q1", q1, "real")
    chk("fake: the M-Bk runs fix them -> Q2", q2, "adopt")
    chk("fake: the 4 cases listed", sorted(T[T["code_b"] == "worse"][["variant", "V2"]].itertuples(index=False, name=None)),
        sorted((v, V2) for v, V2, _ in wb))
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "fig", "side", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare, "fig": fig, "side": side}[a.cmd]()


if __name__ == "__main__":
    main()
