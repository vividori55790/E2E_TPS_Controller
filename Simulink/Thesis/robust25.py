"""
robust25.py -- (2026-10-30 18:24) R10 again on the 25 ns grid: is M-B still no worse than SPS when the plant differs?

R10 (robust_check.py, final 10-30 13:35) ran at 100 ns and said "열화 있음": V1 -5 % at 84 V 300 W, M-B 0.090 A above SPS, with
7 of 30 cases undecidable (109.2 V oscillation). GRID and the 25 ns map (fine_grid_check.py, r13_map.py) have since
shown that differences of this size at 100 ns do not survive a finer grid, so the 100 ns verdict is not an answer.

Same cases as R10: L -10 / +10 %, td 200 ns, V1 -5 / +5 % (one at a time, through SimulationInput, model never saved) x V2
{84, 109.2} x |P| {100, 300, 1000} W x {SPS, M-B} with the R.4 duties (robust_check.configs). Measurement = the 25 ns map:
verify_steady_fine_param.m (Ts 25 ns), constant D0 20 ms, 20 ms window, two adjacent 2.88-deg levels bracketing |P|, Ipk
interpolated in power (fine_grid_check.at_power). The Python plant on a 25 ns grid with the variant's L, td, V1 only picks
the levels. Nominal = the 25 ns map (out/r13_points.csv).

RULE (registered 2026-10-30 18:24, before any run of this file; the rule of robust_check.py, unchanged, do not change after
the results): per case dI = Ipk(SPS) - Ipk(M-B), tol = max(0.03 A, 2 % Ipk(SPS)).
  "판정 불가" if either controller is not bracketed or oscillates (|mean i_L| > 20 mA in a bracketing run);
  "열화" if dI < -tol; "SPS와 같음" if |dI| <= tol; "저감" if dI > tol.
  verdict: "견고" if no case is "열화" and at least 75 % of the 30 are decidable; "열화 있음" otherwise (listed); "판정 불가"
  if fewer than 75 % are decidable.
Reported alongside (not part of the verdict): retention = dI(variant) / dI(nominal, 25 ns) where nominal dI > tol; dI on the
median period peak (out/iL_pp/; why: r13_map.py header, note of 10-30 18:15).

    python -I -X utf8 Simulink/Thesis/robust25.py make       -> out/rob25_points_r1.csv
    python automation/job.py submit rob25 --interactive --after g10 -- matlab -batch "cd('Simulink/Thesis'); rob25_rounds(1, 4, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/robust25.py next       (not bracketed -> next round; rob25_rounds calls it)
    python -I -X utf8 Simulink/Thesis/robust25.py compare    -> out/rob25_check.{csv,log}, out/rob25_verdict.md
    python -I -X utf8 Simulink/Thesis/robust25.py selftest
"""
import argparse
import csv
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

OUT = os.path.join(HERE, "out")
TS = rm.TS
WIN = fg.WIN
G = fg.G
FIELDS = ["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2", "Ts", "variant", "L_uH", "td_ns", "V1"]


def configs():
    return rc.configs()


def point(c, j):
    return dict(fg._point(dict(c, set=c["variant"] + " ", pair=""), j, TS), variant=c["variant"], L_uH=c["L_uH"],
                td_ns=c["td_ns"], V1=c["V1"])


def write(path, pts):
    seen, uniq = set(), []
    for p in pts:
        k = (p["variant"], p["V2"], p["level"], p["D1"], p["D2"])
        if k not in seen:
            seen.add(k)
            uniq.append(p)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows({k: p[k] for k in FIELDS} for p in uniq)
    print(f"wrote {path}: {len(uniq)} runs ({len(pts)} before removing duplicates)")


def predicted_levels(cf):
    """two adjacent 2.88-deg levels that the Python plant on the 25 ns grid (with the variant's L, td, V1) predicts to bracket
    |P|, the first pair from below"""
    import pi_steady as ps
    import plant_full as pf
    out = [None] * len(cf)
    for v in rc.VARIANTS:
        idx = [i for i, c in enumerate(cf) if c["variant"] == v]
        if not idx:
            continue
        p = rc.params(v)
        pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, grid_ns=TS * 1e9, L=p["L_uH"] * 1e-6, td=p["td_ns"] * 1e-9, V1=p["V1"])
        P, V, D1, D2 = (np.array([cf[i][k] for i in idx]) for k in ("P", "V2", "D1", "D2"))
        D0i = ps.operating_point(P, D1, D2, V)["D0"]                   # nominal ideal model: only the centre of the scan
        js = np.arange(-4, 8)
        J = np.floor(np.nan_to_num(D0i) / G).astype(int)[:, None] + js[None, :]
        n, k = J.shape
        sd = [fg._safe(float(j) * G, a, b, TS) for j, a, b in zip(J.ravel(), np.repeat(D1, k), np.repeat(D2, k))]
        r = pl.steady_fixed(np.array([s[0] for s in sd]), np.array([s[1] for s in sd]), np.array([s[2] for s in sd]),
                            np.full(n * k, p["V1"]), np.repeat(V, k))
        Pd = (-r["P_LV"]).reshape(n, k)
        for a, i in enumerate(idx):
            b = rm.first_bracket(J[a], Pd[a], cf[i]["P"])
            if b is None:
                print(f"  plant does not bracket: {cf[i]}")
                j0 = int(np.floor(np.nan_to_num(D0i[a]) / G))
                b = (j0, j0 + 1)
            out[i] = b
        print(f"  {v}: {n} configurations, {n * k} plant evaluations", flush=True)
    return out


def make():
    cf = configs()
    pts = []
    for c, b in zip(cf, predicted_levels(cf)):
        pts += [point(c, j) for j in b]
    write(os.path.join(OUT, "rob25_points_r1.csv"), pts)


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "rob25_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results(files=None):
    fs = result_files() if files is None else files
    if not fs:
        return None
    d = fg._load(fs, fine=True)
    return d.drop_duplicates(subset=["variant", "V2", "level", "D1", "D2"], keep="last")


def runs_of(d, c):
    return fg.runs_of(d[d["variant"] == c["variant"]], c, TS)


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "rob25_sweep_r*.csv"))
          for m in [re.search(r"rob25_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/rob25_sweep_r*.csv yet")
    pts = []
    for c in configs():
        g = runs_of(d, c)
        if g.empty or np.isfinite(fg.at_power(g, c["P"])[0]):
            continue
        pts += [point(c, j) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        write(os.path.join(OUT, f"rob25_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed")


def _pp(pats=("rob25_sweep_r*.csv",)):
    """(variant, V2, level, D1, D2) -> median period peak, from out/iL_pp/"""
    import pandas as pd
    out = {}
    for pat in pats:
        for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", pat))):
            T = pd.read_csv(f)
            pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float),
                            -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
            for r, x in zip(T.itertuples(), pk):
                out[(r.variant, round(r.V2, 6), int(r.level), round(r.D1, 9), round(r.D2, 9))] = float(np.nanmedian(x))
    return out


def _nominal():
    """(V2, P) -> (Ipk SPS, Ipk M-B, tol) on the 25 ns map"""
    import pandas as pd
    f = os.path.join(OUT, "r13_points.csv")
    if not os.path.isfile(f):
        return {}
    Q = pd.read_csv(f)
    return {(r.V2, r.P): (r.I_SPS, r.I_MB, r.tol) for r in Q.itertuples()}


def compare(d=None, write_out=True):
    import datetime

    import pandas as pd
    d = results() if d is None else d
    if d is None:
        sys.exit("no out/rob25_sweep_r*.csv yet")
    pp, nom, rows = _pp(), _nominal(), []
    val = {}
    for c in configs():
        g = runs_of(d, c)
        I, o, lo = fg.at_power(g, c["P"]) if not g.empty else (np.nan, np.nan, None)
        med = np.nan
        if lo is not None:
            x = g[g["level"].isin([lo, lo + 1])]
            ms = [pp.get((c["variant"], round(c["V2"], 6), int(r.level), round(r.D1, 9), round(r.D2, 9))) for r in x.itertuples()]
            if all(m is not None for m in ms):
                Pd = -x[f"P_LV_at_{WIN}ms"].to_numpy()
                med = ms[0] + (c["P"] - Pd[0]) / (Pd[1] - Pd[0]) * (ms[1] - ms[0])
        val[(c["variant"], c["V2"], c["P"], c["ctrl"])] = (I, o, med, c["D1"], c["D2"])
    for v in rc.VARIANTS:
        for V2 in rc.V2S:
            for P in rc.PS:
                s, b = val[(v, V2, P, "SPS")], val[(v, V2, P, "M-B")]
                code, dI, tol = rc.classify(s[0], b[0], s[1], b[1])
                n = nom.get((V2, P), (np.nan, np.nan, np.nan))
                dI0 = n[0] - n[1]
                ret = dI / dI0 if np.isfinite(dI) and np.isfinite(dI0) and dI0 > n[2] else np.nan
                rows.append(dict(variant=v, V2=V2, P=P, Ipk_SPS=s[0], Ipk_MB=b[0], osc_SPS=s[1], osc_MB=b[1], dI=dI, tol=tol, code=code,
                                 dI_nominal=dI0, retention=ret, dI_med=s[2] - b[2], D1=b[3], D2=b[4]))
    T = pd.DataFrame(rows)
    verdict = rc.verdict(list(T["code"]))
    dec = T[T["code"] != "undecided"]
    n_open = int((~np.isfinite(T["Ipk_SPS"]) | ~np.isfinite(T["Ipk_MB"])).sum())
    lines = ["robust25: R10 on the 25 ns grid, Simulink (dataset-era model), HV->LV, dI = Ipk(SPS) - Ipk(M-B) [A]; rule in the file header",
             "", " variant    V2     |P| |  SPS    M-B     dI    tol  | code        | nominal  retention | dI(median peak)"]
    for r in T.itertuples():
        lines.append(f" {r.variant:7s} {r.V2:6.1f} {r.P:6.0f} | {r.Ipk_SPS:5.3f}  {r.Ipk_MB:5.3f}  {r.dI:+6.3f} {r.tol:5.3f} | {r.code:11s} | "
                     f"{r.dI_nominal:+6.3f}  " + (f"{100 * r.retention:4.0f} %" if np.isfinite(r.retention) else "  -   ") + f"  | {r.dI_med:+6.3f}")
    lines += ["", f"decidable {len(dec)}/{len(T)}; worse {int((T['code'] == 'worse').sum())}, same {int((T['code'] == 'same').sum())}, "
              f"better {int((T['code'] == 'better').sum())}; mean dI over the decidable {dec['dI'].mean():+.3f} A; verdict: {verdict}"]
    if n_open:
        lines.append(f"{n_open} cases not bracketed yet -> provisional")
    md = ["# R10 파라미터 견고성, 25 ns 격자 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `robust25.py compare`. 규칙은 `robust_check.py`의 것 그대로이고 25 ns 실행 전(10-30 18:24) `robust25.py` 머리말에 다시 적었다. "
          "모두 Simulink(데이터셋 시기 모델, 제어기·로컬 솔버 25 ns), 상수 D0 20 ms, 두 계단 전력 보간. Python 플랜트는 계단 선택에만 썼다.", "",
          "| 변형 | V2 | \\|P\\| | SPS [A] | B [A] | ΔI [A] | 허용 | 판정 | 공칭 ΔI (25 ns 지도) | 유지율 | ΔI(주기 피크 중앙값, 판정 아님) |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.variant} | {r.V2:g} V | {r.P:.0f} W | {r.Ipk_SPS:.3f} | {r.Ipk_MB:.3f} | {r.dI:+.3f} | {r.tol:.3f} | {rc.CODE_TXT[r.code]} | {r.dI_nominal:+.3f} | "
                  + (f"{100 * r.retention:.0f} %" if np.isfinite(r.retention) else "–") + f" | {r.dI_med:+.3f} |")
    md += ["", "**판정**: " + rc.VERDICT_TXT[verdict]
           + (": " + ", ".join(f"{r.variant} {r.V2:g} V {r.P:.0f} W ({r.dI:+.3f} A)" for r in T.itertuples() if r.code == "worse") if verdict == "worse" else "")
           + f". 판정 가능 {len(dec)}/{len(T)}."]
    if n_open:
        md.append(f"※ 아직 감싸지 못한 경우 {n_open}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "rob25_check.csv"), index=False, float_format="%.5g")
        open(os.path.join(OUT, "rob25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        open(os.path.join(OUT, "rob25_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, verdict


def fig():
    """reduction against |1 - k|: the 25 ns map (nominal plant) and the V1 variants of this file, 100 / 300 / 1000 W. Not a verdict.
    -> out/fig_rob25_k.png; the 100 W line is fitted to the decidable points with |1 - k| >= 0.06 (below that the reduction is 0)"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    from matplotlib.lines import Line2D
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    T = pd.read_csv(os.path.join(OUT, "rob25_check.csv"))
    T = T[T["variant"].isin(("V1-5", "V1+5"))]
    Q = pd.read_csv(os.path.join(OUT, "r13_points.csv"))
    Q = Q[Q["P"].isin(rc.PS)]
    v1 = {"V1-5": 361.0, "V1+5": 399.0}
    rows = [dict(k=4 * r.V2 / 380.0, P=r.P, dI=r.I_SPS - r.I_MB, rel=100 * (r.I_SPS - r.I_MB) / r.I_SPS, kind="nom") for r in Q.itertuples() if r.q2_MB != "undecidable"]
    rows += [dict(k=4 * r.V2 / v1[r.variant], P=r.P, dI=r.dI, rel=100 * r.dI / r.Ipk_SPS, kind="osc" if r.code == "undecided" else "v1") for r in T.itertuples()]
    R = pd.DataFrame(rows)
    R["x"] = np.abs(1.0 - R["k"])
    col = {100.0: BLUE, 300.0: ORANGE, 1000.0: AQUA}
    S = "#fcfcfb"
    fig_, ax = plt.subplots(1, 2, figsize=(11.5, 4.6))
    for a, y in zip(ax, ("dI", "rel")):
        a.axhline(0.0, color=MUTED, lw=0.8)
        for P, c in col.items():
            for lo, m in ((True, "o"), (False, "^")):
                g = R[(R["P"] == P) & ((R["k"] < 1.0) == lo)]
                n, h, o = g[g["kind"] == "nom"], g[g["kind"] == "v1"], g[g["kind"] == "osc"]
                a.scatter(n["x"], n[y], s=46, marker=m, color=c, edgecolors=S, linewidths=0.8, zorder=3)
                a.scatter(h["x"], h[y], s=46, marker=m, facecolors=S, edgecolors=c, linewidths=1.5, zorder=3)
                a.scatter(o["x"], o[y], s=46, marker="x", color=c, linewidths=1.5, zorder=3)
    f = R[(R["P"] == 100.0) & (R["kind"] != "osc") & (R["x"] >= 0.06)]
    s, b = np.polyfit(f["x"], f["dI"], 1)
    r2 = np.corrcoef(f["x"], f["dI"])[0, 1] ** 2
    print(f"100 W line over {len(f)} points: slope {s:.2f} A, intercept {b:.3f} A, zero at |1-k| = {-b / s:.3f}, R2 {r2:.3f}")
    print(f"  residual max {np.abs(f['dI'] - (s * f['x'] + b)).max():.3f} A")
    xs = np.array([-b / s, 0.22])
    ax[0].plot(xs, s * xs + b, color=BLUE, lw=1.2, zorder=2)
    ax[0].text(0.205, s * 0.205 + b + 0.05, f"100 W fit: {s:.1f} A × (|1−k| − {-b / s:.2f})", color=INK2, fontsize=8.5, ha="right", va="bottom")
    bound = 380.0 / (4 * 80e3 * 90e-6)
    xb = np.array([0.0, 0.22])
    ax[0].plot(xb, bound * xb, color=MUTED, lw=1.0, ls="--", zorder=1)
    ax[0].text(0.222, bound * 0.205, "bound V1·|1−k|/(4·fs·L), V1 380 V\n(reference, circulating component)", color=INK2, fontsize=8.5, ha="right", va="top")
    ax[0].set_ylabel("Ipk(SPS) − Ipk(controller B) [A]")
    ax[1].set_ylabel("reduction, % of Ipk(SPS)")
    for a in ax:
        a.set_xlabel("|1 − k|,  k = n·V2 / V1")
        a.set_xlim(0.0, 0.225)
    ax[0].set_title("Absolute reduction", loc="left")
    ax[1].set_title("Relative reduction", loc="left")
    hs = [Line2D([], [], ls="", marker="o", color=c, markeredgecolor=S, ms=7, label=f"{P:g} W") for P, c in col.items()]
    hs += [Line2D([], [], ls="", marker="o", color=INK2, markeredgecolor=S, ms=7, label="nominal plant, k < 1"),
           Line2D([], [], ls="", marker="^", color=INK2, markeredgecolor=S, ms=7, label="nominal plant, k > 1"),
           Line2D([], [], ls="", marker="o", markerfacecolor=S, markeredgecolor=INK2, markeredgewidth=1.5, ms=7, label="V1 ±5 % at 84 / 109.2 V"),
           Line2D([], [], ls="", marker="x", color=INK2, markeredgewidth=1.5, ms=7, label="V1 −5 %, controller oscillates")]
    ax[1].legend(handles=hs, fontsize=8.5, frameon=False, loc="upper left")
    fig_.suptitle("Simulink steady state, 25 ns grid, HV→LV: the reduction follows |1 − k|, and V1 ±5 % moves along the same trend", fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_rob25_k.png"), dpi=130)
    plt.close(fig_)
    return R


def selftest():
    """the registered rule (robust_check.classify / verdict) end to end on made-up Simulink runs"""
    import tempfile

    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    cf = configs()
    chk("30 cases x 2 controllers", len(cf), 60)
    chk("variant carried to the point", {k: point(cf[0], 2)[k] for k in ("variant", "Ts", "level")}, dict(variant=cf[0]["variant"], Ts=TS, level=2))
    chk("D0 of level 2 on the 25 ns grid", point(cf[0], 2)["D0"], round(2 * G - fg.NUDGE, 6))
    rows = []
    for c in cf:
        worse = c["variant"] == "td200" and c["V2"] == 84.0 and c["P"] == 1000.0 and c["ctrl"] == "M-B"
        osc = 0.05 if c["variant"] == "L+10" and c["V2"] == 109.2 and c["P"] == 100.0 and c["ctrl"] == "M-B" else 0.001
        base = 2.0 + c["P"] / 1000.0 - (0.0 if c["ctrl"] == "SPS" else (-0.2 if worse else 0.1))
        j0 = int(c["P"] // 50)
        for j, Pd in ((j0, 50.0 * j0 - 25.0), (j0 + 1, 50.0 * j0 + 25.0)):
            rows.append(dict(ctrl=c["ctrl"], V2=c["V2"], level=j, D0=j * G, D1=c["D1"], D2=c["D2"], Ts=TS, variant=c["variant"],
                             **{f"P_LV_at_{WIN}ms": -Pd, f"Ipk_at_{WIN}ms": base}, iL_mean=osc, P_LV=-Pd, error=""))
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "rob25_sweep_r1_part001.csv")
        pd.DataFrame(rows).to_csv(f, index=False)
        T, v = compare(results([f]), write_out=False)
    chk("fake verdict", v, "worse")
    chk("fake worse case", list(T[T["code"] == "worse"][["variant", "V2", "P"]].itertuples(index=False, name=None)), [("td200", 84.0, 1000.0)])
    chk("fake oscillating case undecided", T[(T["variant"] == "L+10") & (T["V2"] == 109.2) & (T["P"] == 100.0)]["code"].iloc[0], "undecided")
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "fig", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare, "fig": fig}[a.cmd]()


if __name__ == "__main__":
    main()
