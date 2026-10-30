"""
robust_check.py -- R10 (user, 10-29): parameter robustness of M-B in Simulink. Does M-B stay no worse than SPS when
the plant differs from the one it was trained for? Rule fixed here on 2026-10-30, before any R10 run existed.

Variants (one parameter at a time, applied by verify_steady_param.m through SimulationInput, model never saved):
  L-10 / L+10    series inductor 81 / 99 uH (nominal 90)
  td200          driver dead time 200 ns (nominal 100)
  V1-5 / V1+5    HV source 361 / 399 V (nominal 380)
Points: V2 {84, 109.2} x |P| {100, 300, 1000} W, HV -> LV (k != 1; at 95 V M-B outputs SPS, so the question is empty).
Controllers SPS and M-B with exactly the R.4 duties (simulink_sweep.configs, D_SAFE-capped). Measurement = R.4: constant
D0 30 ms, last 1 ms, the two adjacent 2.88-deg levels whose measured powers bracket |P|, Ipk interpolated in power.
The Python plant (with the variant's L, td, V1) only picks the two levels. Nominal = R.4 (out/simulink_pi_compare.csv).

RULE (registered 10-30, do not change after results):
  per (variant, point): dI = Ipk(SPS) - Ipk(M-B), tolerance tol = max(0.03 A, 2 % of Ipk(SPS)) (the OPT0 rule (i) size).
  - "판정 불가" if either controller is not bracketed or either side oscillates (|mean i_L| > 20 mA in a bracketing run).
  - "열화" if dI < -tol;  "SPS와 같음" if |dI| <= tol;  "저감" if dI > tol.
  verdict: "견고" if no point of any variant is "열화" (and at least 75 % of the 30 cases are decidable);
           "열화 있음" otherwise (list them); "판정 불가" if fewer than 75 % are decidable.
  Reported alongside (not part of the verdict): retention = dI(variant) / dI(nominal) where nominal dI > tol.

ADDED 10-30 13:00, after the rob4 results were seen (so NOT part of the verdict, reported alongside only): "M-Bk" =
M-B fed the V1-normalised input (|P| (V1n/V1)^2, V2 V1n/V1), V1n = 380 V, which keeps k = n V2 / V1 and the power per
unit (|P| 2 fs L / (n V1 V2)) of the nominal plant, so the ideal model's optimum maps exactly -- the V1-equivalent
input already built into smooth_reghdc_infer.m (optional V1) and the C export (RESULTS 6, T14, analytic only until
now). rob4 showed V1 -5 % at
84 V 300 W "열화"; the ideal model (reference) predicts it (k 0.884 -> 0.931: the nominal duties are too aggressive) and
that M-Bk removes it. Run only where V1 moves k toward 1 (V1 -5 % at 84 V, V1 +5 % at 109.2 V): there the normalised
V2 (88.4 / 104.0 V) is inside the training range 84-109.2 V; the other two cases would extrapolate the encoder.

ADDED 10-30 14:00, BEFORE any M-S run on a variant plant: the same per-case rule and the same verdict, applied to
M-S (the corrected surface adopted 10-30 11:45 by the rule in ms_verdict.py, RESULTS R.10) in place of M-B. M-S gets
its own runs only where its duty differs from M-B's by more than MS_SAME = 0.003 (the "M-S = M-B" convention of
efficiency_sweep / surface_opt_check): 84 V 300 W and 109.2 V 100 / 300 / 1000 W; at 84 V 100 W and 1000 W the M-B runs
stand for M-S. Reported as a second verdict ("M-S 견고성"); the R10 verdict above stays the M-B one. Final R10 (M-B,
10-30 13:35, 30/30 bracketed): 23/30 decidable, "열화 있음" (V1 -5 % 84 V 300 W, -0.090 A).

    python -I -X utf8 Simulink/Thesis/robust_check.py make        -> out/robust_points_r1.csv
    python automation/job.py submit rob1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_param('out/robust_points_r1.csv', 'out/robust_sweep_r1.csv')"
    python -I -X utf8 Simulink/Thesis/robust_check.py next        -> out/robust_points_r<k+1>.csv (unbracketed)
    python -I -X utf8 Simulink/Thesis/robust_check.py compare     -> out/robust_check.{csv,log}, out/robust_verdict.md
    python -I -X utf8 Simulink/Thesis/robust_check.py selftest
"""
import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import plant_full as pf  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
NOM = dict(L_uH=90.0, td_ns=100.0, V1=380.0)
EXTRA = "M-Bk"                               # reported alongside only (see the 10-30 13:00 note above)
EXTRA_CASES = (("V1-5", 84.0), ("V1+5", 109.2))
MS = "M-S"                                   # second verdict, rule registered 10-30 14:00 (see above)
MS_SAME = 0.003
VARIANTS = {"L-10": dict(L_uH=81.0), "L+10": dict(L_uH=99.0), "td200": dict(td_ns=200.0),
            "V1-5": dict(V1=361.0), "V1+5": dict(V1=399.0)}
V2S = (84.0, 109.2)
PS = (100.0, 300.0, 1000.0)
CTRLS = ("SPS", "M-B")
OSC = 0.02                                   # [A] |mean i_L| over the last 1 ms above this = oscillating
TOL_A, TOL_REL = 0.03, 0.02
DECIDABLE = 0.75
FIELDS = ["label", "ctrl", "V2", "P_target", "level", "D0", "D1", "D2", "variant", "L_uH", "td_ns", "V1"]


def params(var):
    p = dict(NOM)
    p.update(VARIANTS[var])
    return p


def configs(extra=False):
    base = [c for c in sw.configs() if c["ctrl"] in CTRLS and c["V2"] in V2S and c["P"] in PS]
    cf = [dict(c, variant=v, **params(v)) for v in VARIANTS for c in base]
    if extra:
        mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
        for v, V2 in EXTRA_CASES:
            r = NOM["V1"] / params(v)["V1"]
            for P in PS:
                d1, d2 = mb.predict(np.array([P * r * r]), np.array([V2 * r]))
                cf.append(dict(ctrl=EXTRA, V2=V2, P=P, D1=round(min(float(np.ravel(d1)[0]), sw.D_SAFE), 9),
                               D2=round(min(float(np.ravel(d2)[0]), sw.D_SAFE), 9), variant=v, **params(v)))
        for c in ms_points(base):
            cf += [dict(c, variant=v, **params(v)) for v in VARIANTS]
    return cf


def ms_points(base):
    """M-S where its duty differs from the R.4 M-B duty by more than MS_SAME (elsewhere the M-B runs stand for it)"""
    ms = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_MS.json"))
    out = []
    for c in base:
        if c["ctrl"] != "M-B":
            continue
        d1, d2 = (min(float(np.ravel(x)[0]), sw.D_SAFE) for x in ms.predict(np.array([c["P"]]), np.array([c["V2"]])))
        if max(abs(d1 - c["D1"]), abs(d2 - c["D2"])) > MS_SAME:
            out.append(dict(ctrl=MS, V2=c["V2"], P=c["P"], D1=round(d1, 9), D2=round(d2, 9)))
    return out


def predicted_levels(cf):
    """two adjacent 2.88-deg levels the quantised Python plant with the variant's L, td, V1 predicts to bracket |P|"""
    out = [None] * len(cf)
    for v in VARIANTS:
        idx = [i for i, c in enumerate(cf) if c["variant"] == v]
        if not idx:
            continue
        p = params(v)
        pl = pf.FullPlant(Ron=0.040, droop=False, quant=True, L=p["L_uH"] * 1e-6, td=p["td_ns"] * 1e-9, V1=p["V1"])
        P, V, D1, D2 = (np.array([cf[i][k] for i in idx]) for k in ("P", "V2", "D1", "D2"))
        D0i = ps.operating_point(P, D1, D2, V)["D0"]                 # nominal ideal model: only the centre of the scan
        js = np.arange(-6, 11)
        J = np.floor(np.nan_to_num(D0i) / sw.G)[:, None] + js[None, :]
        n, k = J.shape
        r = pl.steady_fixed((J * sw.G - 0.5 * sw.G).ravel(), np.repeat(D1, k), np.repeat(D2, k),
                            np.full(n * k, p["V1"]), np.repeat(V, k))
        Pd = (-r["P_LV"]).reshape(n, k)
        for a, i in enumerate(idx):
            above = np.where(Pd[a] >= cf[i]["P"])[0]
            j = max(above[0] if above.size else k - 1, 1)
            out[i] = (int(J[a, j - 1]), int(J[a, j]))
    return out


def point(c, j):
    p = sw.point(c, j)
    p["label"] = f"{c['variant']} " + p["label"]
    p.update(variant=c["variant"], L_uH=c["L_uH"], td_ns=c["td_ns"], V1=c["V1"])
    return p


def key(p):
    return (p["variant"], p["V2"], p["D0"], p["D1"], p["D2"])


def write(path, pts, done=()):
    seen, uniq = set(done), []
    for p in pts:
        if key(p) not in seen:
            seen.add(key(p))
            uniq.append(p)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for p in uniq:
            w.writerow({k: p[k] for k in FIELDS})
    print(f"wrote {path}: {len(uniq)} runs ({len(pts)} before removing duplicates)")


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "robust_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def results(files=None):
    import pandas as pd
    fs = result_files() if files is None else files
    if not fs:
        return None
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["level"] = np.round((d["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    return d.drop_duplicates(subset=["variant", "V2", "D0", "D1", "D2"], keep="last")


def runs_of(d, c):
    return sw.runs_of(d[d["variant"] == c["variant"]], c)


def round_no():
    ns = [int(m.group(1)) for f in result_files() for m in [re.search(r"robust_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def make():
    cf = configs()
    pts = []
    for c, (j0, j1) in zip(cf, predicted_levels(cf)):
        pts += [point(c, j0), point(c, j1)]
    write(os.path.join(OUT, "robust_points_r1.csv"), pts)


def next_round():
    d = results()
    if d is None:
        sys.exit("no out/robust_sweep_r*.csv yet")
    cf, pts = configs(extra=True), []
    fresh = [c for c in cf if runs_of(d, c).empty]
    for c, (j0, j1) in zip(fresh, predicted_levels(fresh) if fresh else []):
        # + one level above: in rob4 every configuration the prediction missed was short of |P| (17/17)
        pts += [point(c, j0), point(c, j1), point(c, j1 + 1)]
    for c in cf:
        g = runs_of(d, c)
        if g.empty:
            continue
        b, way = sw.bracket(g, c["P"])
        below = g[(g["Pd"] > 0) & (g["Pd"] <= c["P"])]
        if b is None and way < 0 and not below.empty:
            # |P| in a gap between non-adjacent measured levels (the surface_opt_check.plan() fix of 10-30)
            j = int(below["level"].max())
            pts += [point(c, j + 1), point(c, j + 2)]
        elif b is None:
            j = int(g["level"].max() + 1) if way > 0 else int(g["level"].min() - 1)
            n = 2
            e = g.iloc[-2:] if way > 0 else g.iloc[:2]
            if way > 0 and len(e) == 2:
                # rob4 (10-30): the variant plants move the power per level, so extend by the last measured step
                # far enough to reach |P| in this round (at most 4 levels), not always by two
                step = (e["Pd"].iloc[1] - e["Pd"].iloc[0]) / (e["level"].iloc[1] - e["level"].iloc[0])
                if step > 0:
                    n = int(np.clip(np.ceil((c["P"] - e["Pd"].iloc[1]) / step) + 1, 2, 4))
            pts += [point(c, j + way * i) for i in range(n)]
    if pts:
        write(os.path.join(OUT, f"robust_points_r{round_no()}.csv"), pts,
              [(r.variant, r.V2, r.D0, r.D1, r.D2) for r in d.itertuples()])
    else:
        print("every configuration is bracketed")


def measure(d, cf):
    """per configuration: Ipk at |P| (NaN if not bracketed) and the larger |mean i_L| of the two bracketing runs"""
    rows = []
    for c in cf:
        g = runs_of(d, c) if d is not None else None
        b = sw.bracket(g, c["P"])[0] if g is not None and not g.empty else None
        r = dict(c, Ipk=np.nan, osc=np.nan)
        if b is not None:
            lo, hi = b
            t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
            r.update(Ipk=lo["Ipk_sim"] + t * (hi["Ipk_sim"] - lo["Ipk_sim"]),
                     osc=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])))
        rows.append(r)
    return rows


def classify(i_sps, i_mb, osc_sps, osc_mb):
    """the registered per-case rule -> (code, dI, tol)"""
    if not (np.isfinite(i_sps) and np.isfinite(i_mb)):
        return "undecided", np.nan, np.nan
    tol = max(TOL_A, TOL_REL * i_sps)
    dI = i_sps - i_mb
    if not (osc_sps <= OSC and osc_mb <= OSC):
        return "undecided", dI, tol
    return ("worse" if dI < -tol else "same" if dI <= tol else "better"), dI, tol


def verdict(codes):
    n = len(codes)
    dec = [c for c in codes if c != "undecided"]
    if n == 0 or len(dec) < DECIDABLE * n:
        return "undecided"
    return "worse" if "worse" in dec else "robust"


def nominal():
    """R.4 nominal (V2, P) -> (Ipk_SPS, Ipk_MB, clean) from out/simulink_pi_compare.csv"""
    import pandas as pd
    f = os.path.join(OUT, "simulink_pi_compare.csv")
    if not os.path.isfile(f):
        return {}
    T = pd.read_csv(f)
    out = {}
    for V2 in V2S:
        for P in PS:
            s = T[(np.isclose(T["V2"], V2)) & (T["P"] == P)].set_index("ctrl")
            if "SPS" in s.index and "M-B" in s.index:
                out[(V2, P)] = (s.loc["SPS", "Ipk_simulink"], s.loc["M-B", "Ipk_simulink"], bool(s.loc["M-B", "clean"]))
    return out


CODE_TXT = {"worse": "열화", "same": "SPS와 같음", "better": "저감", "undecided": "판정 불가"}
VERDICT_TXT = {"robust": "**견고하다**: 다섯 변형 어디서도 제어기 B가 SPS보다 허용오차 넘게 나빠지지 않는다",
               "worse": "**열화 있음**: 아래 점에서 제어기 B가 SPS보다 허용오차 넘게 나쁘다",
               "undecided": "**판정 불가**: 판정 가능한 경우가 75 % 미만이다"}


def compare(d=None, write_out=True):
    import pandas as pd
    d = results() if d is None else d
    cf = configs(extra=True)
    M = measure(d, cf)
    nom = nominal()
    rows = []
    for v in VARIANTS:
        for V2 in V2S:
            for P in PS:
                s, b, k, q = (next((m for m in M if m["variant"] == v and m["V2"] == V2 and m["P"] == P and m["ctrl"] == c),
                                   None) for c in ("SPS", "M-B", EXTRA, MS))
                code, dI, tol = classify(s["Ipk"], b["Ipk"], s["osc"], b["osc"])
                ms_own = q is not None
                q = q or b                                   # M-S = M-B here: the M-B runs stand for M-S
                code_s, dI_s, _ = classify(s["Ipk"], q["Ipk"], s["osc"], q["osc"])
                n_s, n_b, n_clean = nom.get((V2, P), (np.nan, np.nan, False))
                dI0 = n_s - n_b
                ret = dI / dI0 if np.isfinite(dI) and np.isfinite(dI0) and dI0 > max(TOL_A, TOL_REL * n_s) else np.nan
                k = k or dict(Ipk=np.nan, osc=np.nan, D1=np.nan, D2=np.nan)
                code_k, dI_k, _ = classify(s["Ipk"], k["Ipk"], s["osc"], k["osc"])
                rows.append(dict(variant=v, V2=V2, P=P, Ipk_SPS=s["Ipk"], Ipk_MB=b["Ipk"], osc_SPS=s["osc"], osc_MB=b["osc"],
                                 dI=dI, tol=tol, code=code, dI_nominal=dI0, nominal_clean=n_clean, retention=ret,
                                 D1=b["D1"], D2=b["D2"], Ipk_MBk=k["Ipk"], osc_MBk=k["osc"], dI_k=dI_k,
                                 code_k=code_k if (v, V2) in EXTRA_CASES else "", D1k=k["D1"], D2k=k["D2"],
                                 Ipk_MS=q["Ipk"], osc_MS=q["osc"], dI_s=dI_s, code_s=code_s, ms_own=ms_own,
                                 D1s=q["D1"], D2s=q["D2"]))
    T = pd.DataFrame(rows)
    v = verdict(list(T["code"]))
    v_s = verdict(list(T["code_s"]))
    lines = ["robust_check: R10 parameter robustness, Simulink (dataset-era model), HV->LV, dI = Ipk(SPS) - Ipk(M-B) [A]",
             f"rule registered 10-30 before results: tol = max({TOL_A} A, {100 * TOL_REL:.0f} % Ipk_SPS); osc > {OSC} A = undecided",
             "", " variant    V2     |P| |  SPS    M-B     dI    tol  | code        | nominal dI  retention"]
    for r in T.itertuples():
        lines.append(f" {r.variant:7s} {r.V2:6.1f} {r.P:6.0f} | {r.Ipk_SPS:5.3f}  {r.Ipk_MB:5.3f}  {r.dI:+6.3f} {r.tol:5.3f} |"
                     f" {r.code:11s} | {r.dI_nominal:+6.3f}{'' if r.nominal_clean else '°'}    "
                     + (f"{100 * r.retention:4.0f} %" if np.isfinite(r.retention) else "  –"))
    dec = T[T["code"] != "undecided"]
    lines += ["", f"decidable {len(dec)}/{len(T)}; worse {int((T['code'] == 'worse').sum())}, same {int((T['code'] == 'same').sum())},"
              f" better {int((T['code'] == 'better').sum())}; verdict: {v}",
              "° nominal R.4 point not clean (oscillating reference)"]
    md = ["# R10 파라미터 견고성 판정 (Simulink)", "",
          "생성: `robust_check.py compare`. 규칙은 2026-10-30 R10 실행 전에 이 스크립트 머리말에 정했다. 수치는 모두 Simulink "
          "(데이터셋 시기 모델, 상수 D0 30 ms, 2.88° 두 계단 전력 보간, HV→LV)이다. Python 플랜트는 계단 선택에만 썼다.", "",
          "| 변형 | V2 | \\|P\\| | SPS [A] | 제어기 B [A] | ΔI [A] | 허용오차 | 판정 | 공칭 ΔI (R.4) | 유지율 |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.variant} | {r.V2:g} V | {r.P:.0f} W | {r.Ipk_SPS:.3f} | {r.Ipk_MB:.3f} | {r.dI:+.3f} | {r.tol:.3f} | "
                  f"{CODE_TXT[r.code]} | {r.dI_nominal:+.3f}{'' if r.nominal_clean else '°'} | "
                  + (f"{100 * r.retention:.0f} %" if np.isfinite(r.retention) else "–") + " |")
    md += ["", "° 공칭 R.4 기준이 진동한 점.", "", "**판정**: " + VERDICT_TXT[v]
           + (": " + ", ".join(f"{r.variant} {r.V2:g} V {r.P:.0f} W ({r.dI:+.3f} A)" for r in T.itertuples() if r.code == "worse")
              if v == "worse" else "") + f". 판정 가능 {len(dec)}/{len(T)}."]
    n_open = int((~np.isfinite(T["Ipk_SPS"]) | ~np.isfinite(T["Ipk_MB"])).sum())
    if n_open:      # wording only (10-30): unbracketed cases are still to be measured, not undecidable
        md.append(f"※ 아직 감싸지 못한 경우 {n_open}개 — **잠정**(다음 라운드 `robust_check.py next` 뒤 다시 판정).")
        lines.append(f"{n_open} cases not bracketed yet -> provisional")
    X = T[T["code_k"] != ""]
    if len(X):
        lines += ["", f"alongside, NOT part of the verdict (added 10-30 13:00 after rob4): {EXTRA} = M-B at (|P| (380/V1)^2,"
                  " V2 380/V1), only where V1 moves k toward 1", " variant    V2     |P| |  SPS    M-B    M-Bk  | dI(M-B) dI(M-Bk)"
                  "  tol  | code(M-Bk)  | D1k    D2k"]
        for r in X.itertuples():
            lines.append(f" {r.variant:7s} {r.V2:6.1f} {r.P:6.0f} | {r.Ipk_SPS:5.3f}  {r.Ipk_MB:5.3f}  {r.Ipk_MBk:5.3f} | {r.dI:+6.3f}"
                         f"  {r.dI_k:+6.3f}  {r.tol:5.3f} | {r.code_k:11s} | {r.D1k:.3f}  {r.D2k:.3f}")
        md += ["", "## 판정 밖 보고: V1 정규화 입력 (M-Bk)", "",
               "rob4 결과를 본 뒤(10-30 13:00) 더한 점검이라 **위 판정에 들어가지 않는다**. M-Bk = 제어기 B에 측정 V1으로 정규화한 입력 "
               "(|P|·(380/V1)², V2·380/V1)을 넣은 것(k = nV2/V1과 단위 전력을 공칭 플랜트와 같게 맞추는 상사 변환). V1이 k를 1 쪽으로 "
               "옮기는 두 경우만 쟀다(정규화 V2 88.4 / 104.0 V가 학습 범위 84–109.2 V 안).", "",
               "| 변형 | V2 | \\|P\\| | SPS [A] | B [A] | M-Bk [A] | ΔI B | ΔI M-Bk | 허용오차 | M-Bk 분류 | M-Bk (D1, D2) |",
               "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in X.itertuples():
            md.append(f"| {r.variant} | {r.V2:g} V | {r.P:.0f} W | {r.Ipk_SPS:.3f} | {r.Ipk_MB:.3f} | {r.Ipk_MBk:.3f} | {r.dI:+.3f} | "
                      f"{r.dI_k:+.3f} | {r.tol:.3f} | {CODE_TXT[r.code_k]} | ({r.D1k:.3f}, {r.D2k:.3f}) |")
    # second verdict: M-S, the same rule (registered 10-30 14:00 before any M-S variant run)
    dec_s = T[T["code_s"] != "undecided"]
    n_open_s = int((~np.isfinite(T["Ipk_SPS"]) | ~np.isfinite(T["Ipk_MS"])).sum())
    lines += ["", f"second verdict (rule registered 10-30 14:00 before the runs): {MS} in place of M-B; '=M-B' = the M-B runs"
              " stand for M-S (duty within 0.003)", " variant    V2     |P| |  SPS    M-S    M-B  | dI(M-S) dI(M-B)  tol  |"
              " code(M-S)   | runs   D1s    D2s"]
    for r in T.itertuples():
        lines.append(f" {r.variant:7s} {r.V2:6.1f} {r.P:6.0f} | {r.Ipk_SPS:5.3f}  {r.Ipk_MS:5.3f}  {r.Ipk_MB:5.3f} | {r.dI_s:+6.3f}"
                     f"  {r.dI:+6.3f}  {r.tol:5.3f} | {r.code_s:11s} | {'own ' if r.ms_own else '=M-B'}  {r.D1s:.3f}  {r.D2s:.3f}")
    lines.append(f"M-S: decidable {len(dec_s)}/{len(T)}; worse {int((T['code_s'] == 'worse').sum())}, same "
                 f"{int((T['code_s'] == 'same').sum())}, better {int((T['code_s'] == 'better').sum())}; verdict: {v_s}"
                 + (f"; {n_open_s} cases not bracketed yet -> provisional" if n_open_s else ""))
    md += ["", "## 두 번째 판정: 채택 곡면 M-S의 견고성 (같은 규칙, 10-30 14:00 M-S 변형 실행 전에 등록)", "",
           "M-S = 10-30 11:45 규칙(`ms_verdict.py`)으로 채택된 보정 곡면. 위와 같은 경우별 규칙·판정을 B 대신 M-S에 적용한다. M-S의 듀티가 "
           "B와 0.003 넘게 다른 점(84 V 300 W, 109.2 V 100 / 300 / 1000 W)만 M-S를 따로 쟀고, 나머지(84 V 100 W, 1 kW)는 B 실행이 M-S를 "
           "대신한다(\"=B\"). 위의 R10 판정(B)은 그대로다.", "",
           "| 변형 | V2 | \\|P\\| | SPS [A] | M-S [A] | B [A] | ΔI M-S | ΔI B | 허용오차 | M-S 판정 | 실행 | M-S (D1, D2) |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in T.itertuples():
        md.append(f"| {r.variant} | {r.V2:g} V | {r.P:.0f} W | {r.Ipk_SPS:.3f} | {r.Ipk_MS:.3f} | {r.Ipk_MB:.3f} | {r.dI_s:+.3f} | "
                  f"{r.dI:+.3f} | {r.tol:.3f} | {CODE_TXT[r.code_s]} | {'M-S' if r.ms_own else '=B'} | ({r.D1s:.3f}, {r.D2s:.3f}) |")
    md += ["", "**M-S 판정**: " + VERDICT_TXT[v_s].replace("제어기 B", "M-S")
           + (": " + ", ".join(f"{r.variant} {r.V2:g} V {r.P:.0f} W ({r.dI_s:+.3f} A)" for r in T.itertuples() if r.code_s == "worse")
              if v_s == "worse" else "") + f". 판정 가능 {len(dec_s)}/{len(T)}."]
    if n_open_s:
        md.append(f"※ M-S: 아직 감싸지 못한 경우 {n_open_s}개 — **잠정**.")
    if write_out:
        T.to_csv(os.path.join(OUT, "robust_check.csv"), index=False, float_format="%.5g")
        with open(os.path.join(OUT, "robust_check.log"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        with open(os.path.join(OUT, "robust_verdict.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(md) + "\n")
    print("\n".join(lines))
    return T, v


def selftest():
    """the registered rule on fake inputs"""
    ok = True

    def chk(name, got, want):
        nonlocal ok
        good = got == want
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'} {name}: {got} (want {want})")

    chk("better", classify(2.0, 1.5, 0.0, 0.0)[0], "better")
    chk("same (within 0.04 = 2 % of 2.0)", classify(2.0, 2.035, 0.0, 0.0)[0], "same")
    chk("worse", classify(2.0, 2.05, 0.0, 0.0)[0], "worse")
    chk("floor 0.03 A at small SPS", classify(1.0, 1.025, 0.0, 0.0)[0], "same")
    chk("oscillating M-B -> undecided", classify(2.0, 1.5, 0.0, 0.05)[0], "undecided")
    chk("unbracketed -> undecided", classify(np.nan, 1.5, 0.0, 0.0)[0], "undecided")
    chk("verdict robust", verdict(["better"] * 20 + ["same"] * 5 + ["undecided"] * 5), "robust")
    chk("verdict worse", verdict(["better"] * 29 + ["worse"]), "worse")
    chk("verdict undecided (<75 %)", verdict(["better"] * 20 + ["undecided"] * 10), "undecided")
    cf = configs()
    chk("30 cases x 2 controllers", len(cf), 60)
    chk("R.4 duties reused (M-B 84 V 100 W)", next((c["D1"], c["D2"]) for c in cf if c["ctrl"] == "M-B" and c["V2"] == 84.0
                                                   and c["P"] == 100.0) == next((c["D1"], c["D2"]) for c in sw.configs()
                                                   if c["ctrl"] == "M-B" and c["V2"] == 84.0 and c["P"] == 100.0), True)
    # end-to-end on fake Simulink files: two levels per configuration bracketing |P|, M-B 0.1 A below SPS except one.
    # One power per level (Pd = 50 j - 25), as in a real run: SPS shares its duties, hence its runs, across |P|.
    import pandas as pd
    import tempfile
    base = [c for c in sw.configs() if c["ctrl"] in CTRLS and c["V2"] in V2S and c["P"] in PS]
    chk("M-S own runs at the registered four points", sorted((c["V2"], c["P"]) for c in ms_points(base)),
        [(84.0, 300.0), (109.2, 100.0), (109.2, 300.0), (109.2, 1000.0)])
    rows = []
    for c in configs(extra=True):
        worse = c["variant"] == "td200" and c["V2"] == 84.0 and c["P"] == 1000.0 and c["ctrl"] == "M-B"
        base = 2.0 + c["P"] / 1000.0 - (0.0 if c["ctrl"] == "SPS" else (-0.2 if worse else 0.1))
        j0 = int(c["P"] // 50)
        for j, Pd in ((j0, 50.0 * j0 - 25.0), (j0 + 1, 50.0 * j0 + 25.0)):
            rows.append(dict(c, label="x", P_target=c["P"], level=j, D0=j * sw.G - 0.5 * sw.G, Ipk_sim=base,
                             iL_mean=0.001, P_LV=-Pd, error=""))
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "robust_sweep_r1_part001.csv")
        pd.DataFrame(rows).drop(columns=["P"]).to_csv(f, index=False)
        T, v = compare(results([f]), write_out=False)
    chk("fake end-to-end verdict", v, "worse")
    chk("fake end-to-end worse case", list(T[T["code"] == "worse"][["variant", "V2", "P"]].itertuples(index=False, name=None)),
        [("td200", 84.0, 1000.0)])
    chk("fake M-S: own runs at 4 x 5 cases", int(T["ms_own"].sum()), 20)
    chk("fake M-S inherits the M-B case where M-S = M-B (td200 84 V 1 kW)",
        list(T[T["code_s"] == "worse"][["variant", "V2", "P"]].itertuples(index=False, name=None)), [("td200", 84.0, 1000.0)])
    chk("fake M-S verdict", verdict(list(T["code_s"])), "worse")
    print("ALL PASS" if ok else "FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "make":
        make()
    elif a.cmd == "next":
        next_round()
    elif a.cmd == "compare":
        compare()
    else:
        sys.exit(0 if selftest() else 1)


if __name__ == "__main__":
    main()
