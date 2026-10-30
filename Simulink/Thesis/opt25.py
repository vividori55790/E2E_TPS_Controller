"""
opt25.py -- (2026-10-30) is there a better (D1, D2) FAR from the surface, on the 25 ns grid?

The user's first question (10-29 19:10) is whether the proposed surface is the Simulink optimum. The only global search so far is
OPT1 (surface_opt_check.py), an 11 x 11 duty grid at 84 / 94.6 / 109.2 V on the 100 ns grid, and GRID / R13 showed that 100 ns
differences of 0.1-1 A are grid scale. On the 25 ns grid the optimality question has only been asked locally: Q1 of r13_map.py looks
at M-S +- 0.03. This file asks it globally: at each of the 56 map points, measure in Simulink the duty pairs far from M-B and M-S
that have the best chance of beating them.

Which pairs to measure is chosen with the Python plant on the 25 ns grid (FullPlant, grid_ns 25, Ron 40 mOhm: power MAE 8 W, Ipk MAE
0.040 A against 25 ns Simulink on the GRID runs). The plant only picks; every number that is reported comes from Simulink.

Scan: D1, D2 in 0.100, 0.125, ... 0.475, D_SAFE (17 x 17), dropped where the ideal model cannot reach |P| on the first branch or its Ipk is
more than FILTER 0.6 A above the best of the grid at that point (ideal-model Ipk is within about 0.3 A of the plant at light load).
Pick, per point: the grid pairs farther than FAR from M-B, M-S and SPS (max |dD|), with plant Ipk <= the plant's min(M-B, M-S) + MARGIN;
best first, each pick at least SEP from the earlier picks, at most K. At |P| <= 400 W also the plant's best pair at least FAR10 0.10 from M-B
and M-S, whatever its plant Ipk: the scan says such pairs are worse (grid best below the surface by more than 0.03 A at 0 of 56 points), and
that is the plant's word, which is not evidence here, so it gets a Simulink sample where the map's gaps are.

RULES (registered 2026-10-30 22:21, after the plant scan, before any OPT25 Simulink run; do not change after the results):
  Values: as the map (r13_map.py): 25 ns, 20 ms window, constant D0, two adjacent 2.88-deg levels, Ipk interpolated at |P|; clean = both
     levels |mean i_L| <= 20 mA. Two kinds: rule = largest of the 80 period peaks, med = their median. tol = max(0.03 A, 2 % Ipk(SPS)), of
     the same kind (2 % Ipk(M-S) if SPS is not clean).
  O1 per point, per controller X in (M-B, M-S), per kind: gap = Ipk(X) - the lowest clean far candidate. gap > tol -> "far better", else
     "no"; X oscillating or no clean far candidate -> "undecidable".
  O2 per kind: nobody "far better" -> "no far optimum"; at least BRANCH_MIN 5 points for M-B or for M-S -> "different branch"; else "isolated
     points". The median kind is the answer to the user's question and the rule kind is reported beside it, because the rule value carries
     one grid step of noise (r13_map.py note 18:15, Q1 table): a "far better" on the rule kind alone is that noise until shown otherwise.
  Reported, never a verdict: plant against Simulink over the measured candidates (mean, MAE, correlation), the far candidates' distance above M-B.

    python -I -X utf8 Simulink/Thesis/opt25.py scan --part k --parts n     -> out/opt25_scan_p<k>.csv (plant, can run beside MATLAB)
    python -I -X utf8 Simulink/Thesis/opt25.py pick                        -> out/opt25_cands.csv, summary
    python -I -X utf8 Simulink/Thesis/opt25.py make                        -> out/opt25_points_r1.csv
    python automation/job.py submit o25r --interactive --after ... -- matlab -batch "cd('Simulink/Thesis'); opt25_rounds(1, 5, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/opt25.py next                        (not bracketed -> next round; opt25_rounds calls it)
    python -I -X utf8 Simulink/Thesis/opt25.py compare                     -> out/opt25_check.{log,csv}, out/opt25_verdict.md
    python -I -X utf8 Simulink/Thesis/opt25.py selftest
"""
import argparse
import datetime
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # the scan runs beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS = rm.TS
WIN = fg.WIN
OSC = fg.OSC
DG = np.round(np.arange(0.10, 0.4751, 0.025), 3)
FILTER = 0.6                                # A: analytic Ipk above the analytic best of the grid by more: not scanned
FAR = 0.02                                  # max |dD| from M-B, M-S and SPS
SEP = 0.05                                  # max |dD| between two picks of a point
K = 3
MARGIN = 0.10                               # A, plant Ipk above the plant's min(M-B, M-S)
FAR10, FAR10_P = 0.10, 400.0                # the sample of pairs the plant rules out: max |dD| from M-B and M-S, up to this |P|
BRANCH_MIN = 5
CANDS = os.path.join(OUT, "opt25_cands.csv")
PATS = rm.PATS + ("opt25_sweep_r*.csv",)
KINDS = ("rule", "med")


def _one(m, p, v):
    return tuple(float(np.ravel(x)[0]) for x in m.predict(np.array([p]), np.array([v])))


def grid_configs(part, parts):
    """every duty pair of the grid the ideal model can reach at the point and does not rule out, + SPS, M-B, M-S"""
    import pi_steady as ps
    import surface_opt_check as so
    mb, ms = so._mb(), so._ms()
    duties = np.append(DG, sw.D_SAFE)
    D1g, D2g = (x.ravel() for x in np.meshgrid(duties, duties, indexing="ij"))
    pts = [(v, p) for v in rm.V2S for p in rm.PS]
    cf = []
    for k, (v, p) in enumerate(pts):
        if k % parts != part:
            continue
        P, V = np.full_like(D1g, p), np.full_like(D1g, v)
        r = ps.operating_point(P, D1g, D2g, V)
        ip = r["Ipk"]
        ok = np.isfinite(r["D0"]) & np.isfinite(ip)
        keep = ok & (ip <= np.nanmin(ip[ok]) + FILTER)
        cf += [dict(set="G", pair=k, ctrl=f"G{a:.3f}_{b:.3f}", V2=v, P=p, D1=round(float(a), 9), D2=round(float(b), 9))
               for a, b in zip(D1g[keep], D2g[keep])]
        b1, s1 = _one(mb, p, v), _one(ms, p, v)
        cf.append(dict(set="G", pair=k, ctrl="M-B", V2=v, P=p, D1=rm._dd(b1[0]), D2=rm._dd(b1[1])))
        cf.append(dict(set="G", pair=k, ctrl="M-S", V2=v, P=p, D1=rm._dd(s1[0]), D2=rm._dd(s1[1])))
    return cf


def scan(part, parts):
    cf = grid_configs(part, parts)
    print(f"part {part}/{parts}: {len(cf)} configurations", flush=True)
    rm.predict(cf, os.path.join(OUT, f"opt25_scan_p{part}.csv"))


def load_scan():
    import pandas as pd
    fs = sorted(glob.glob(os.path.join(OUT, "opt25_scan_p*.csv")))
    if not fs:
        sys.exit("no scan files")
    T = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    t = (T["P"] - T["P_lo"]) / (T["P_hi"] - T["P_lo"])
    T["Ipk_p"] = np.where(T["found"], T["Ipk_lo"] + t * (T["Ipk_hi"] - T["Ipk_lo"]), np.nan)
    return T


def pick_at(rows, refs):
    """rows: (D1, D2, Ipk_p) of the grid pairs at one point, refs: (D1, D2) of M-B, M-S, SPS, limit: plant Ipk ceiling (last item
    of refs tuple) -> the picks, best first"""
    (pts, limit) = refs
    far = [r for r in rows if np.isfinite(r[2]) and r[2] <= limit and all(max(abs(r[0] - a), abs(r[1] - b)) > FAR for a, b in pts)]
    out = []
    for r in sorted(far, key=lambda x: x[2]):
        if all(max(abs(r[0] - q[0]), abs(r[1] - q[1])) >= SEP for q in out):
            out.append(r)
        if len(out) == K:
            break
    return out


def pick_far10(rows, pts, picked):
    """the plant's best pair at least FAR10 from every pair in pts (M-B, M-S) and SEP from the earlier picks, or None"""
    far = [r for r in rows if np.isfinite(r[2]) and all(max(abs(r[0] - a), abs(r[1] - b)) >= FAR10 for a, b in pts)
           and all(max(abs(r[0] - q[0]), abs(r[1] - q[1])) >= SEP for q in picked)]
    return min(far, key=lambda x: x[2]) if far else None


def pick():
    import pandas as pd
    T = load_scan()
    rows, summ = [], []
    for (v, p), g in T.groupby(["V2", "P"]):
        G = g[g["ctrl"].str.startswith("G")]
        b, s = g[g["ctrl"] == "M-B"].iloc[0], g[g["ctrl"] == "M-S"].iloc[0]
        base = np.nanmin([b["Ipk_p"], s["Ipk_p"]])
        lim = base + MARGIN
        grid = [(r["D1"], r["D2"], r["Ipk_p"]) for _, r in G.iterrows()]
        ch = pick_at(grid, ([(b["D1"], b["D2"]), (s["D1"], s["D2"]), (sw.D_SAFE, sw.D_SAFE)], lim))
        f10 = pick_far10(grid, [(b["D1"], b["D2"]), (s["D1"], s["D2"])], ch) if p <= FAR10_P else None
        best = G.loc[G["Ipk_p"].idxmin()] if G["Ipk_p"].notna().any() else None
        summ.append(dict(V2=v, P=p, plant_MB=b["Ipk_p"], plant_MS=s["Ipk_p"], plant_grid_best=best["Ipk_p"] if best is not None else np.nan,
                         best_D1=best["D1"] if best is not None else np.nan, best_D2=best["D2"] if best is not None else np.nan,
                         n_scanned=len(G), n_found=int(G["found"].sum()), n_picks=len(ch),
                         far10_plant=f10[2] - base if f10 else np.nan))
        for i, r in enumerate(ch + ([f10] if f10 else [])):
            lv = g[(g["ctrl"].str.startswith("G")) & (np.abs(g["D1"] - r[0]) < 1e-9) & (np.abs(g["D2"] - r[1]) < 1e-9)].iloc[0]
            rows.append(dict(V2=v, P=p, rank=i, D1=r[0], D2=r[1], plant=r[2], plant_base=base, lo=int(lv["lo"]), hi=int(lv["hi"])))
    C = pd.DataFrame(rows, columns=["V2", "P", "rank", "D1", "D2", "plant", "plant_base", "lo", "hi"])
    C.to_csv(CANDS, index=False, float_format="%.9g")
    Z = pd.DataFrame(summ)
    Z.to_csv(os.path.join(OUT, "opt25_plant_summary.csv"), index=False, float_format="%.6g")
    print(Z.to_string(index=False, float_format=lambda x: f"{x:.3f}"))
    print(f"{len(C)} picks over {int((Z['n_picks'] > 0).sum())}/{len(Z)} points -> about {2 * len(C)} runs; plant: grid best below min(M-B, M-S) by more than 0.03 A at "
          f"{int((np.minimum(Z['plant_MB'], Z['plant_MS']) - Z['plant_grid_best'] > 0.03).sum())} points")
    return C


# ----------------------------------------------------------------------------------------------------- runs
def configs():
    import pandas as pd
    C = pd.read_csv(CANDS)
    return [dict(set="O", pair=int(r["rank"]), ctrl=f"O{int(r['rank'])}", V2=float(r["V2"]), P=float(r["P"]), D1=round(float(r["D1"]), 9),
                 D2=round(float(r["D2"]), 9), lo=int(r["lo"]), hi=int(r["hi"])) for _, r in C.iterrows()]


def results():
    return rm.results_25(PATS)


def make():
    d, pts, skip = results(), [], 0
    cf = configs()
    for c in cf:
        g = rm._runs(d, c)
        if not g.empty and np.isfinite(fg.at_power(g, c["P"])[0]):
            skip += 1
            continue
        have = set(g["level"].astype(int)) if not g.empty else set()
        pts += [fg._point(c, j, TS) for j in (c["lo"], c["hi"]) if j not in have]
    print(f"{len(cf)} configurations, {skip} already bracketed by existing 25 ns runs")
    fg._write(os.path.join(OUT, "opt25_points_r1.csv"), pts)


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "opt25_sweep_r*.csv"))
          for m in [re.search(r"opt25_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d, pts = results(), []
    for c in configs():
        g = rm._runs(d, c)
        if g.empty:
            pts += [fg._point(c, j, TS) for j in (c["lo"], c["hi"])]
        elif not np.isfinite(fg.at_power(g, c["P"])[0]):
            pts += [fg._point(c, j, TS) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"opt25_points_r{round_no()}.csv"), pts)
    else:
        print("every OPT25 configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def rule_far(x, far, tol):
    """controller value x = (Ipk, osc), far = [(Ipk, osc)] of the far candidates -> (code, gap, index of the lowest clean far candidate)"""
    cl = [(a[0], i) for i, a in enumerate(far) if rm.clean(a)]
    if not rm.clean(x) or not cl:
        return "undecidable", np.nan, None
    lo, i = min(cl)
    gap = x[0] - lo
    return ("far better" if gap > tol else "no"), float(gap), i


def rule_conclusion(codes_b, codes_s):
    """codes per kind for M-B / M-S -> the conclusion on the median kind (the rule value carries one grid step of noise; both reported)"""
    nb, ns = codes_b.count("far better"), codes_s.count("far better")
    if nb == 0 and ns == 0:
        return "no far optimum"
    return "different branch" if max(nb, ns) >= BRANCH_MIN else "isolated points"


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    c = lambda i, o=0.0: (i, o)  # noqa: E731
    chk("far better", rule_far(c(1.2), [c(1.0), c(1.1)], 0.05)[0::2], ("far better", 0))
    chk("within tol", rule_far(c(1.04), [c(1.0)], 0.05)[0], "no")
    chk("oscillating candidate ignored", rule_far(c(1.2), [c(1.0, 0.03)], 0.05)[0], "undecidable")
    chk("oscillating controller", rule_far(c(1.2, 0.03), [c(1.0)], 0.05)[0], "undecidable")
    chk("no candidate", rule_far(c(1.2), [], 0.05)[0], "undecidable")
    chk("conclusion none", rule_conclusion(["no", "no"], ["no", "undecidable"]), "no far optimum")
    chk("conclusion isolated", rule_conclusion(["far better"] * 4, ["no"]), "isolated points")
    chk("conclusion branch", rule_conclusion(["far better"] * 5, ["no"]), "different branch")
    refs = ([(0.3, 0.3), (0.31, 0.31), (sw.D_SAFE, sw.D_SAFE)], 1.1)
    rows = [(0.305, 0.305, 0.9), (0.35, 0.35, 1.0), (0.36, 0.36, 1.01), (0.45, 0.45, 1.05), (0.2, 0.2, 1.2), (0.25, 0.25, 1.09), (0.15, 0.15, 1.08),
            (0.12, 0.12, np.nan)]
    chk("pick: far, spaced, at most K", [r[:2] for r in pick_at(rows, refs)], [(0.35, 0.35), (0.45, 0.45), (0.15, 0.15)])
    chk("far10: the best one, at the distance", pick_far10(rows, refs[0][:2], [])[:2], (0.45, 0.45))
    chk("far10: ignores the margin", pick_far10([(0.18, 0.18, 9.0), (0.25, 0.25, 0.5)], refs[0][:2], [])[:2], (0.18, 0.18))
    chk("far10: spaced from the picks", pick_far10(rows, refs[0][:2], [(0.45, 0.45, 1.0)])[:2], (0.15, 0.15))
    chk("far10: none", pick_far10([(0.32, 0.32, 1.0)], refs[0][:2], []), None)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def compare():
    import pandas as pd
    d = results()
    J = rm.per_period(d, PATS)
    pl = pd.read_csv(CANDS)
    Q = pd.read_csv(os.path.join(OUT, "r13_points.csv")).set_index(["V2", "P"])
    tab = {(v, p): cs for v, p, cs in rm.candidates("map")}
    lines, recs, cand_rows = [], [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    def val(own, v, p):
        g = fg.runs_of(J, dict(V2=v, D1=own["D1"], D2=own["D2"]), TS)
        if g.empty:
            return dict(rule=(np.nan, np.nan), med=(np.nan, np.nan))
        I, os_, _ = fg.at_power(g, p)
        return dict(rule=(I, os_), med=(rm._at_col(g, p, "pk_med")[0], os_))

    o("opt25: duty pairs far from the surface, 25 ns Simulink, 20 ms window, Ipk at |P| (A); rules in the file header")
    for (v, p), cs in tab.items():
        by = {c["name"]: c for c in cs}
        base = {}
        for nm in ("SPS", "M-B", "M-S"):
            own = by[by[nm]["alias"]] if by[nm]["alias"] else by[nm]
            base[nm] = val(own, v, p)
        far = pl[(np.isclose(pl["V2"], v)) & (np.isclose(pl["P"], p))]
        fv = [val(dict(D1=r["D1"], D2=r["D2"]), v, p) for _, r in far.iterrows()]
        rec = dict(V2=v, P=p, n_far=len(far))
        for kind in KINDS:
            sps, ms = base["SPS"][kind], base["M-S"][kind]
            ref = sps if rm.clean(sps) else ms
            tol = max(rm.TOL_ABS, rm.TOL_REL * ref[0]) if rm.clean(ref) else rm.TOL_ABS
            rec[f"tol_{kind}"] = tol
            for nm in ("M-B", "M-S"):
                code, gap, i = rule_far(base[nm][kind], [x[kind] for x in fv], tol)
                k = nm.replace("-", "")
                rec.update({f"code_{k}_{kind}": code, f"gap_{k}_{kind}": gap, f"I_{k}_{kind}": base[nm][kind][0]})
                if i is not None and nm == "M-B":
                    rec[f"best_{kind}"] = f"({far.iloc[i]['D1']:.3f}, {far.iloc[i]['D2']:.3f})"
                    rec[f"Ibest_{kind}"] = fv[i][kind][0]
        recs.append(rec)
        for (_, r), x in zip(far.iterrows(), fv):
            cand_rows.append(dict(V2=v, P=p, rank=r["rank"], D1=r["D1"], D2=r["D2"], plant=r["plant"], plant_base=r["plant_base"], I_rule=x["rule"][0],
                                  I_med=x["med"][0], osc=x["rule"][1], I_MB_rule=base["M-B"]["rule"][0], I_MB_med=base["M-B"]["med"][0]))
    R = pd.DataFrame(recs)
    F = pd.DataFrame(cand_rows)
    R.to_csv(os.path.join(OUT, "opt25_check.csv"), index=False, float_format="%.6g")
    F.to_csv(os.path.join(OUT, "opt25_cands_simulink.csv"), index=False, float_format="%.6g")
    res = {}
    for kind in KINDS:
        cb, cs_ = list(R[f"code_MB_{kind}"]), list(R[f"code_MS_{kind}"])
        res[kind] = rule_conclusion(cb, cs_)
        o(f"\n{'rule value' if kind == 'rule' else 'median period peak'}: M-B far better at {cb.count('far better')}, no at {cb.count('no')}, undecidable {cb.count('undecidable')}; "
          f"M-S far better at {cs_.count('far better')}, no at {cs_.count('no')}, undecidable {cs_.count('undecidable')} -> {res[kind]}")
        for _, r in R[(R[f"code_MB_{kind}"] == "far better") | (R[f"code_MS_{kind}"] == "far better")].iterrows():
            o(f"   {r['V2']:6.1f} V {r['P']:5.0f} W: lowest far {r.get(f'best_{kind}')} {r.get(f'Ibest_{kind}', np.nan):.3f} A; M-B {r[f'I_MB_{kind}']:.3f} (gap {r[f'gap_MB_{kind}']:+.3f}), "
              f"M-S {r[f'I_MS_{kind}']:.3f} (gap {r[f'gap_MS_{kind}']:+.3f}), tol {r[f'tol_{kind}']:.3f}")
    ok = F[F["I_rule"].notna()]
    if len(ok):
        c1 = np.corrcoef(ok["plant"], ok["I_med"])[0, 1] if ok["I_med"].notna().all() else np.nan
        o(f"\nNot a verdict: plant Ipk vs Simulink over {len(ok)} measured far candidates: mean plant - median peak {(ok['plant'] - ok['I_med']).mean():+.3f} A, MAE {(ok['plant'] - ok['I_med']).abs().mean():.3f} A, r {c1:.3f}")
        o(f"Not a verdict: far candidates clean {int((ok['osc'] <= OSC).sum())}/{len(ok)}; mean median-peak gap to M-B (far - M-B) {(ok['I_med'] - ok['I_MB_med']).mean():+.3f} A, lowest {(ok['I_med'] - ok['I_MB_med']).min():+.3f} A")
    n_pts = int((R["n_far"] > 0).sum())
    o(f"\npoints with far candidates {n_pts}/{len(R)}; the other points: the plant saw no far pair within {MARGIN} A of the surface")
    open(os.path.join(OUT, "opt25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"no far optimum": "먼 최적 없음", "isolated points": "몇 점에서만", "different branch": "다른 가지 있음"}
    md = ["# OPT25: 곡면에서 먼 듀티의 25 ns Simulink 검증 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `opt25.py compare`. 규칙은 머리말(실행 전 등록). 후보 선택만 25 ns Python 플랜트로 했고, 값은 모두 Simulink 정상상태다.", "",
          f"- 80주기 최대: **{KO[res['rule']]}**", f"- 주기 중앙값: **{KO[res['med']]}**", "", "표 원본 `out/opt25_check.csv`, `out/opt25_cands_simulink.csv`, 로그 `out/opt25_check.log`."]
    open(os.path.join(OUT, "opt25_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/opt25_check.{log,csv}, out/opt25_cands_simulink.csv, out/opt25_verdict.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["scan", "pick", "make", "next", "compare", "selftest"])
    ap.add_argument("--part", type=int, default=0)
    ap.add_argument("--parts", type=int, default=1)
    a = ap.parse_args()
    if a.cmd == "scan":
        scan(a.part, a.parts)
    elif a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    else:
        {"pick": pick, "make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
