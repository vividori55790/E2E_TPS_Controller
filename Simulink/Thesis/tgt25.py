"""
tgt25.py -- (2026-10-31 12:15) the sentence the thesis ends on, on the 25 ns grid: "what decides the performance is the training
target, not the model structure" (REPORT 10), and the steady-state comparison with the ICROS2026 surface.

What that sentence rests on today:
  * controller A (same structure as B, the dataset as target) against B: 12 points at 100 ns only (out/simulink_pi_compare.csv).
    REPORT 5 says so itself ("제어기 A는 25 ns로 다시 재지 않았다"). One grid step is 0.42 A at 100 ns, and that grid has already
    produced a k = 1 valley and M-S gains that the 25 ns grid erased, so "A above SPS at 6 of 12 points, 95 V 300 W +0.49 A" has
    to be measured again before the thesis says it. The direction is not obvious either way: 100 ns inflates SPS at 95 V (0.753 ->
    0.297 A at 100 W), which flatters A there.
  * the E2E fine-tuning (B0 -> B): the same 12 points at 100 ns.
  * HDC against LUT, and how small the model can be: the ideal model only ([참고], RESULTS 5: LUT 49 x 22 0.0815 A, D = 64 0.0812 A,
    LUT 13 x 6 0.0804 A against B 0.0815 A).
  * the ICROS2026 surface: only on the closed-loop ramps (R12, two V2). No steady-state comparison with B exists, although the
    report's title comparison is with ICROS2026.

Candidates, every one fixed before this file (none is retrained):
  M-A    out/model_smooth_reghdc_MA_neg.json   same structure as B, dataset target (the HV -> LV half), 10-26
  ICROS  r12_ramp.duties("ICROS", ...)         the ICROS2026 structure re-implemented, classifier = the dataset's labels + FSM lock-in
                                               (its best case; the paper's classifier was 97.44 % accurate), 10-30
  M-B0   out/model_smooth_reghdc_a.json        same structure as B, steady-state surface, no E2E, 10-21
  L49    out/model_lut_L49.npz                 bilinear LUT 49 x 22, 4328 B, the same E2E from the same surface, 10-24
  D64    out/model_smooth_reghdc_b7_D64.json   same structure as B with D = 64, 1288 B, the same E2E, 10-24
  L25    out/model_lut_L25.npz                 LUT 25 x 11, 1116 B, the same E2E, 10-24 (its V2 axis ends at 108 V: 109.2 V reads the last row)
  L13    out/model_lut_L13.npz                 LUT 13 x 6, 328 B, the same E2E, 10-24 (the same axis end)

Points: set M = the 56 map points (r13_map.V2S x PS). Set R = 95 V x 100, 300, 600, 1000 W (the rest of the 100 ns table; only M-A,
ICROS, M-B0). SPS and M-B are the runs that exist (the map, md1). Runs: as the map. 25 ns, constant D0 20 ms, 20 ms window, two
adjacent 2.88-deg levels from the 25 ns Python plant (levels only), Ipk interpolated at |P| (fine_grid_check.at_power), clean = both
levels |mean i_L| <= 20 mA, two kinds: "rule" = the largest of the 80 period peaks, "med" = their median. A candidate within SAME
0.003 of an earlier one in the order SPS, M-B, M-A, ICROS, M-B0, L49, D64, L25, L13 uses the earlier one's runs.

RULES (registered 2026-10-31 12:15, before any run of this file; do not change after the results). tol = max(0.03 A, 2 % Ipk(SPS))
of the same kind at the point (2 % Ipk(M-B) if SPS is not clean), md_check.tol_of. Each pair: md_check.classify (gain = Ipk(other) -
Ipk(B); better / worse beyond tol, both clean; fixes_osc / new_osc count with better / worse without a gain value; undecidable if a
value is missing or both oscillate). up = better + fixes_osc, down = worse + new_osc. Below 75 % decidable -> "undecidable". All on
set M, on both kinds:
  QA  goal 3 for controller A: n = number of M points with Ipk(A) - Ipk(SPS) > tol (both clean). "real" if n >= 5 on both kinds
      (the count the map's retraining trigger used), "none" if n = 0 on both, else "some".
  QT  "the training target decides", B against A: "splits" if on both kinds mean gain >= SPLIT 0.05 A (about half of B's own mean
      saving on the map, +0.115 A; four fifths of the uniform-|P| one, +0.063 A) and up >= 2 down; "reversed" if on both kinds
      mean gain <= -0.05 A and down >= 2 up; "tie" if on both kinds |mean gain| <= TIE 0.01 A (the M-D threshold); else "partial".
  QI  B against ICROS: the same rule as QT ("splits" = B lower).
  QE  the E2E step, B against B0, on the points where their runs differ: "gain" if on both kinds mean gain >= 0.01 A and up >= down;
      "loss" if on both kinds mean gain <= -0.01 A and down >= up; else "no gain".
  QS(X) structure and size, X in L49, D64, L25, L13, against B on the points where X's runs differ from B's (a point where X uses B's
      runs says nothing, so it is not counted). within = share of those points classified "same".
      "same"  if on both kinds |mean gain| <= 0.02 A, within >= 0.70 (the M-S / B pair on the map, which the report calls a draw, has
              23 of 29) and goal-3 count(X) <= count(B) + 2 over set M;
      "diff"  if on either kind |mean gain| > 0.05 A or within < 0.50 or goal-3 count(X) > count(B) + 5;
      else "partial".
  F   the sentence: "holds" if QT = splits and QS(L49) = QS(D64) = same; "target only" if QT = splits otherwise; "structure only" if
      QT != splits and QS(L49) = QS(D64) = same; "neither" otherwise; "undecidable" if any of the three is.
What gets written for each answer, decided now:
  F holds       REPORT 10 keeps the sentence, and 5 replaces its 100 ns table with these values.
  target only   REPORT 10 keeps "the target decides" and states which structure or size differed by how much instead of "not the structure".
  structure / neither   REPORT 5 and 10 lose "the target decides"; they state the measured gain of B over A.
  QS(D64) = same  "D = 64 gives the same reduction at a quarter of the memory and of the MACs" goes next to the 4.6 kB claim; the main
                surface stays B, because every other result in the report is B's.
Reported, never a verdict: the mean saving against SPS of each candidate per kind, per V2 and per |P| band; the losses; set R; the 12-point
100 ns statements re-classified by fine_grid_check.rule_r2 (kept / gone / reversed), which say which sentences of REPORT 5 survive.

    python -I -X utf8 Simulink/Thesis/tgt25.py selftest
    python -I -X utf8 Simulink/Thesis/tgt25.py predict       -> out/tgt25_pred.csv (levels from the 25 ns plant, beside MATLAB)
    python -I -X utf8 Simulink/Thesis/tgt25.py make          -> out/tgt25a_points_r1.csv, out/tgt25b_points_r1.csv
    python automation/job.py submit tgt25a --interactive --after v1e -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); tgt25_rounds('a', 1, 5, '<python.exe>')"
    python automation/job.py submit tgt25b --interactive --after tgt25a -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); tgt25_rounds('b', 1, 5, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/tgt25.py next --part a (not bracketed -> next round; tgt25_rounds calls it)
    python -I -X utf8 Simulink/Thesis/tgt25.py compare       -> out/tgt25_check.{csv,log}, out/tgt25_verdict.md
"""
import argparse
import datetime
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `predict` runs beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import fine_grid_check as fg  # noqa: E402
import md_check as md  # noqa: E402
import r13_map as rm  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS = rm.TS
OSC = rm.OSC
PRED = os.path.join(OUT, "tgt25_pred.csv")
PATS = md.PATS + ("tgt25a_sweep_r*.csv", "tgt25b_sweep_r*.csv")
PART = {"a": ("M-A", "ICROS", "M-B0"), "b": ("L49", "D64", "L25", "L13")}
NAMES = ("SPS", "M-B") + PART["a"] + PART["b"]
KEY = {"SPS": "SPS", "M-B": "MB", "M-A": "MA", "ICROS": "IC", "M-B0": "B0", "L49": "L49", "D64": "D64", "L25": "L25", "L13": "L13"}
R_PTS = ((95.0, 100.0), (95.0, 300.0), (95.0, 600.0), (95.0, 1000.0))
KINDS = ("rule", "med")
DECIDABLE = 0.75
SPLIT, TIE = 0.05, 0.01
E_MIN = 0.01
S_MEAN, S_WITHIN, S_G3 = 0.02, 0.70, 2
D_MEAN, D_WITHIN, D_G3 = 0.05, 0.50, 5
QA_REAL = 5
_cache = {}


# ----------------------------------------------------------------------------------------------------- candidates
def _models():
    if not _cache:
        from train_lut_e2e import TrainLUT
        load = lambda t: ct.SmoothRegHDC.from_json(os.path.join(OUT, f"model_smooth_reghdc_{t}.json"))  # noqa: E731
        lut = lambda t: TrainLUT.load(os.path.join(OUT, f"model_lut_{t}.npz"))  # noqa: E731
        _cache.update({"M-B": load("b3"), "M-A": load("MA_neg"), "M-B0": load("a"), "D64": load("b7_D64"),
                       "L49": lut("L49"), "L25": lut("L25"), "L13": lut("L13")})
    return _cache


def duty(name, p, v):
    if name == "SPS":
        return sw.D_SAFE, sw.D_SAFE
    if name == "ICROS":
        import r12_ramp as r12
        d = r12.duties("ICROS", np.array([p]), v)
    else:
        d = _models()[name].predict(np.array([p]), np.array([v]))
    return tuple(float(np.ravel(x)[0]) for x in d)


def cands(names, p, v):
    out = []
    for n in names:
        d = duty(n, p, v)
        out.append(rm._alias(dict(name=n, D1=rm._dd(d[0]), D2=rm._dd(d[1])), out))
    return out


def points():
    """[(set, V2, |P|, candidates)]"""
    pts = [("M", v, p, cands(NAMES, p, v)) for v in rm.V2S for p in rm.PS]
    pts += [("R", v, p, cands(("SPS", "M-B") + PART["a"], p, v)) for v, p in R_PTS]
    return pts


def configs(part="all"):
    """the configurations with their own runs, in run order (part a = the targets, part b = the structures)"""
    names = PART["a"] + PART["b"] if part == "all" else PART[part]
    pts, cf = points(), []
    for nm in names:
        for st, v, p, cs in pts:
            c = next((x for x in cs if x["name"] == nm), None)
            if c is not None and c["alias"] is None:
                cf.append(dict(set=st, pair=rm.V2S.index(v) if v in rm.V2S else 9, ctrl=nm, V2=v, P=p, D1=c["D1"], D2=c["D2"]))
    return cf


def results():
    return rm.results_25(PATS)


def predict():
    rm.predict(configs(), PRED)


def make():
    d, lv = results(), rm._pred_levels(PRED)
    for part in ("a", "b"):
        pts, skip = [], 0
        cf = configs(part)
        for c in cf:
            g = rm._runs(d, c)
            if not g.empty and np.isfinite(fg.at_power(g, c["P"])[0]):
                skip += 1
                continue
            have = set(g["level"].astype(int)) if not g.empty else set()
            if rm._key(c) not in lv:
                sys.exit(f"no predicted levels for {c} -- rerun `tgt25.py predict`")
            pts += [fg._point(c, j, TS) for j in lv[rm._key(c)] if j not in have]
        print(f"part {part}: {len(cf)} configurations, {skip} already bracketed by existing 25 ns runs")
        fg._write(os.path.join(OUT, f"tgt25{part}_points_r1.csv"), pts)


def round_no(part):
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, f"tgt25{part}_sweep_r*.csv"))
          for m in [re.search(rf"tgt25{part}_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round(part):
    d, lv0, pts = results(), rm._pred_levels(PRED), []
    for c in configs(part):
        g = rm._runs(d, c)
        if g.empty:
            pts += [fg._point(c, j, TS) for j in lv0.get(rm._key(c), ())]
        elif not np.isfinite(fg.at_power(g, c["P"])[0]):
            pts += [fg._point(c, j, TS) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{rm.WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"tgt25{part}_points_r{round_no(part)}.csv"), pts)
    else:
        print(f"part {part}: every configuration is bracketed (or out of reach on the first branch)")


# ----------------------------------------------------------------------------------------------------- rules
def stats(codes, gains):
    """-> dict(mean, up, down, same, dec, n, within) for one kind"""
    codes, g = list(codes), np.array(list(gains), float)
    n, dec = len(codes), sum(c != "undecidable" for c in codes)
    fin = g[np.isfinite(g)]
    same = codes.count("same")
    return dict(mean=float(fin.mean()) if len(fin) else np.nan, up=codes.count("better") + codes.count("fixes_osc"), down=codes.count("worse") + codes.count("new_osc"),
                same=same, dec=dec, n=n, within=same / len(fin) if len(fin) else np.nan)


def _undecided(st):
    return any(s["n"] == 0 or s["dec"] < DECIDABLE * s["n"] or not np.isfinite(s["mean"]) for s in st.values())


def rule_qa(n_worse, decidable):
    """n_worse, decidable: kind -> count, share of decidable points"""
    if any(decidable[k] < DECIDABLE for k in KINDS):
        return "undecidable"
    if all(n_worse[k] >= QA_REAL for k in KINDS):
        return "real"
    if all(n_worse[k] == 0 for k in KINDS):
        return "none"
    return "some"


def rule_split(st):
    """QT, QI: st = kind -> stats of B against the other controller"""
    if _undecided(st):
        return "undecidable"
    s = list(st.values())
    if all(x["mean"] >= SPLIT and x["up"] >= 2 * x["down"] for x in s):
        return "splits"
    if all(x["mean"] <= -SPLIT and x["down"] >= 2 * x["up"] for x in s):
        return "reversed"
    if all(abs(x["mean"]) <= TIE for x in s):
        return "tie"
    return "partial"


def rule_qe(st):
    if _undecided(st):
        return "undecidable"
    s = list(st.values())
    if all(x["mean"] >= E_MIN and x["up"] >= x["down"] for x in s):
        return "gain"
    if all(x["mean"] <= -E_MIN and x["down"] >= x["up"] for x in s):
        return "loss"
    return "no gain"


def rule_qs(st, g3x, g3b):
    """st = kind -> stats of B against X on the points where X has its own runs; g3x, g3b = kind -> goal-3 count over set M"""
    if _undecided(st):
        return "undecidable"
    s = list(st.values())
    if any(abs(x["mean"]) > D_MEAN or x["within"] < D_WITHIN for x in s) or any(g3x[k] > g3b[k] + D_G3 for k in KINDS):
        return "diff"
    if all(abs(x["mean"]) <= S_MEAN and x["within"] >= S_WITHIN for x in s) and all(g3x[k] <= g3b[k] + S_G3 for k in KINDS):
        return "same"
    return "partial"


def rule_f(qt, s49, s64):
    if "undecidable" in (qt, s49, s64):
        return "undecidable"
    st = s49 == "same" and s64 == "same"
    if qt == "splits":
        return "holds" if st else "target only"
    return "structure only" if st else "neither"


# ----------------------------------------------------------------------------------------------------- compare
def table(J):
    import pandas as pd
    rows = []
    for st, v, p, cs in points():
        vals, by = md._vals(J, v, p, cs), {c["name"]: c for c in cs}
        rec = dict(set=st, V2=v, P=p)
        for nm in NAMES:
            k = KEY[nm]
            if nm in vals:
                rec.update({f"I_{k}": vals[nm]["rule"][0], f"med_{k}": vals[nm]["med"][0], f"osc_{k}": vals[nm]["rule"][1], f"loss_{k}": vals[nm]["loss"],
                            f"D1_{k}": by[nm]["D1"], f"D2_{k}": by[nm]["D2"], f"own_{k}": by[nm]["alias"] or nm})
            else:
                rec.update({f"I_{k}": np.nan, f"med_{k}": np.nan, f"osc_{k}": np.nan, f"loss_{k}": np.nan, f"D1_{k}": np.nan, f"D2_{k}": np.nan, f"own_{k}": ""})
        for kind in KINDS:
            rec[f"tol_{kind}"] = md.tol_of(vals["SPS"][kind], vals["M-B"][kind])
        rows.append(rec)
    return pd.DataFrame(rows)


def _val(r, nm, kind):
    k = KEY[nm]
    return (r[f"I_{k}" if kind == "rule" else f"med_{k}"], r[f"osc_{k}"])


def pair(T, other, kind, only_diff=False):
    """B against `other` over the rows of T -> stats; with only_diff the points where the two share their runs are left out"""
    codes, gains = [], []
    for _, r in T.iterrows():
        if only_diff and r[f"own_{KEY[other]}"] == r["own_MB"]:
            continue
        c, g = md.classify(_val(r, "M-B", kind), _val(r, other, kind), r[f"tol_{kind}"])
        codes.append(c)
        gains.append(g)
    return stats(codes, gains)


def worse_than_sps(T, nm, kind):
    """-> (count, list of (V2, P, excess), share of decidable points)"""
    bad, dec = [], 0
    for _, r in T.iterrows():
        x, s = _val(r, nm, kind), _val(r, "SPS", kind)
        if md.clean(x) and md.clean(s):
            dec += 1
            if md.goal3(x, s, r[f"tol_{kind}"]):
                bad.append((r["V2"], r["P"], x[0] - s[0]))
    return len(bad), bad, dec / max(len(T), 1)


def statements_100(T):
    """the 12-point 100 ns table: statements "X lower than Y" re-classified at 25 ns -> rows"""
    import pandas as pd
    S = pd.read_csv(os.path.join(OUT, "simulink_pi_compare.csv"))
    at = lambda c, v, p: S[(S["ctrl"] == c) & (S["V2"] == v) & (S["P"] == p)].iloc[0]  # noqa: E731
    out = []
    for (x, y) in (("M-A", "SPS"), ("M-B", "M-A"), ("M-B", "M-B0"), ("M-B", "SPS")):
        for _, r in T.iterrows():
            v, p = r["V2"], r["P"]
            if not ((v in (84.0, 109.2) and p in (100.0, 300.0, 600.0, 1000.0)) or v == 95.0):
                continue
            a, b, s = at(x, v, p), at(y, v, p), at("SPS", v, p)
            d100, tol100 = b["Ipk_simulink"] - a["Ipk_simulink"], max(rm.TOL_ABS, rm.TOL_REL * s["Ipk_simulink"])
            cl100 = bool(a["clean"]) and bool(b["clean"])
            res = dict(pair=f"{x} lower than {y}", V2=v, P=p, d100=d100, cl100=cl100)
            for kind in KINDS:
                vx, vy = _val(r, x, kind), _val(r, y, kind)
                d25 = vy[0] - vx[0]
                res[f"d25_{kind}"] = d25
                res[f"r2_{kind}"] = fg.rule_r2(d100, d25, tol100, r[f"tol_{kind}"], cl100, md.clean(vx) and md.clean(vy))
            out.append(res)
    return pd.DataFrame(out)


TXT = {"real": "실재", "none": "없음", "some": "일부", "undecidable": "판정 불가", "splits": "가른다", "reversed": "반대", "tie": "안 가른다", "partial": "부분", "gain": "이득 있음", "loss": "손해",
       "no gain": "이득 없음", "same": "같음", "diff": "다름", "holds": "성립", "target only": "목표 쪽만", "structure only": "구조 쪽만", "neither": "불성립"}


def predicted():
    """REFERENCE ONLY, Python plant before any Simulink run of this file, never a verdict (added 12:22, no rule touched): the same
    numbers on the plant's two levels (out/tgt25_pred.csv, out/r13_pred.csv, out/md_pred.csv; one value, so both kinds are the same)"""
    import pandas as pd
    P = pd.concat([pd.read_csv(f) for f in (PRED, rm.PRED, md.PRED)], ignore_index=True)

    def val(v, p, c):
        g = P[np.isclose(P["V2"], v) & np.isclose(P["P"], p) & (np.abs(P["D1"] - c["D1"]) <= 1e-6) & (np.abs(P["D2"] - c["D2"]) <= 1e-6) & P["found"].astype(bool)]
        if g.empty:
            return np.nan
        r = g.iloc[0]
        return float(r["Ipk_lo"] + (p - r["P_lo"]) / (r["P_hi"] - r["P_lo"]) * (r["Ipk_hi"] - r["Ipk_lo"]))
    rows = []
    for st, v, p, cs in points():
        by, rec = {c["name"]: c for c in cs}, dict(set=st, V2=v, P=p)
        for nm in NAMES:
            k = KEY[nm]
            c = by.get(nm)
            x = val(v, p, by[c["alias"]] if c and c["alias"] else c) if c else np.nan
            rec.update({f"I_{k}": x, f"med_{k}": x, f"osc_{k}": 0.0 if np.isfinite(x) else np.nan, f"loss_{k}": np.nan, f"D1_{k}": c["D1"] if c else np.nan, f"D2_{k}": c["D2"] if c else np.nan, f"own_{k}": (c["alias"] or nm) if c else ""})
        for kind in KINDS:
            rec[f"tol_{kind}"] = max(rm.TOL_ABS, rm.TOL_REL * rec["I_SPS"]) if np.isfinite(rec["I_SPS"]) else rm.TOL_ABS
        rows.append(rec)
    _, ans, _, lines = compare(T=pd.DataFrame(rows), write_out=False)
    txt = ["[참고] Python plant on the 25 ns grid, before any Simulink run of tgt25; NOT a verdict. Only the rule value is meaningful (the two kinds are one number)."] + lines
    open(os.path.join(OUT, "tgt25_predict.log"), "w", encoding="utf-8").write("\n".join(txt) + "\n")
    print("\n".join(txt))
    return ans


def compare(J=None, write_out=True, T=None):
    import pandas as pd
    if T is None:
        if J is None:
            d = results()
            if d is None:
                sys.exit("no 25 ns runs")
            J = rm.per_period(d, PATS)
        T = table(J)
    M, R = T[T["set"] == "M"], T[T["set"] == "R"]
    lines = []

    def o(x=""):
        lines.append(x)
        if write_out:
            print(x, flush=True)

    f3 = lambda x: f"{x:6.3f}" if np.isfinite(x) else f"{'-':>6}"  # noqa: E731
    o("tgt25: Ipk [A] at |P|, 25 ns Simulink (dataset-era model), rule value (80-period maximum); ~ = oscillating; rules in the file header")
    o(f"{'set':>3} {'V2':>6} {'|P|':>5} | " + " ".join(f"{n:>7}" for n in NAMES))
    for _, r in T.iterrows():
        o(f"{r['set']:>3} {r['V2']:6.1f} {r['P']:5.0f} | " + " ".join(
            f3(r[f"I_{KEY[n]}"]) + ("~" if np.isfinite(r[f"osc_{KEY[n]}"]) and r[f"osc_{KEY[n]}"] > OSC else " ") for n in NAMES))
    ans = {}
    n_miss = {nm: int(M[f"I_{KEY[nm]}"].isna().sum()) for nm in NAMES}
    o("\nQA goal 3 for controller A (set M):")
    wa = {k: worse_than_sps(M, "M-A", k) for k in KINDS}
    ans["QA"] = rule_qa({k: wa[k][0] for k in KINDS}, {k: wa[k][2] for k in KINDS})
    for k in KINDS:
        o(f"   {k}: above SPS at {wa[k][0]} points, decidable {100 * wa[k][2]:.0f} %: " + ", ".join(f"{v:g} V {p:.0f} W +{e:.3f}" for v, p, e in wa[k][1]))
    o(f"   QA = {ans['QA']}")
    st = {}
    for q, other, fn in (("QT", "M-A", rule_split), ("QI", "ICROS", rule_split)):
        st[q] = {k: pair(M, other, k) for k in KINDS}
        ans[q] = fn(st[q])
    st["QE"] = {k: pair(M, "M-B0", k, only_diff=True) for k in KINDS}
    ans["QE"] = rule_qe(st["QE"])
    g3b = {k: worse_than_sps(M, "M-B", k)[0] for k in KINDS}
    for x in PART["b"]:
        q = f"QS({x})"
        st[q] = {k: pair(M, x, k, only_diff=True) for k in KINDS}
        g3x = {k: worse_than_sps(M, x, k)[0] for k in KINDS}
        ans[q] = rule_qs(st[q], g3x, g3b)
        st[q]["g3"] = g3x
    ans["F"] = rule_f(ans["QT"], ans["QS(L49)"], ans["QS(D64)"])
    o("\nB against each (gain = Ipk(other) - Ipk(B); up = B lower beyond tol + other oscillates; down = the reverse):")
    for q, s in st.items():
        o(f"   {q}: {ans[q]}")
        for k in KINDS:
            x = s[k]
            o(f"      {k}: mean gain {x['mean']:+.3f} A; up {x['up']}, down {x['down']}, same {x['same']}, within {x['within']:.2f}, decidable {x['dec']}/{x['n']}" + (f"; goal 3 count {s['g3'][k]} (B {g3b[k]})" if "g3" in s else ""))
    o(f"\nF: {ans['F']}")
    miss = {nm: n for nm, n in n_miss.items() if n}
    if miss:
        o(f"PROVISIONAL: not bracketed yet on set M: {miss}")
    o("\nReported, not a verdict: mean saving against SPS over the clean points of each candidate (set M)")
    for nm in NAMES[1:]:
        for kind, pre in (("rule", "I"), ("med", "med")):
            k = KEY[nm]
            m = M[(M[f"osc_{k}"] <= OSC) & (M["osc_SPS"] <= OSC) & M[f"{pre}_{k}"].notna() & M[f"{pre}_SPS"].notna()]
            if len(m):
                dI = m[f"{pre}_SPS"] - m[f"{pre}_{k}"]
                by_v = " ".join(f"{v:g}:{g:+.3f}" for v, g in dI.groupby(m["V2"]).mean().items())
                bands = " ".join(f"{a}-{b} W {dI[(m['P'] >= a) & (m['P'] <= b)].mean():+.3f}" for a, b in ((100, 200), (300, 400), (600, 1000)))
                o(f"   {nm:6s} {kind:4s}: {len(m)} points, mean {dI.mean():+.3f} A (median {dI.median():+.3f}), {(100 * dI / m[f'{pre}_SPS']).mean():+.1f} %, loss {(m[f'loss_{k}'] - m['loss_SPS']).mean():+.2f} W | {bands} | {by_v}")
    o("\nReported: set R (95 V), rule value, median")
    for _, r in R.iterrows():
        o(f"   {r['P']:5.0f} W: " + "  ".join(f"{n} {f3(r['I_' + KEY[n]])} {f3(r['med_' + KEY[n]])}" for n in ("SPS", "M-B") + PART["a"]))
    S = statements_100(T)
    o("\nReported: the 100 ns statements of the 12-point table at 25 ns (rule_r2)")
    for q, g in S.groupby("pair", sort=False):
        o(f"   {q}:")
        for kind in KINDS:
            c = g[f"r2_{kind}"].dropna()
            o(f"      {kind}: statements {len(c)}: kept {int((c == 'kept').sum())}, gone {int((c == 'gone').sum())}, reversed {int((c == 'reversed').sum())}, undecidable {int((c == 'undecidable').sum())}")
        for _, x in g.iterrows():
            if x["r2_rule"] is not None and isinstance(x["r2_rule"], str):
                o(f"      {x['V2']:g} V {x['P']:.0f} W: 100 ns {x['d100']:+.3f}, 25 ns {x['d25_rule']:+.3f} / {x['d25_med']:+.3f} A -> {x['r2_rule']} / {x['r2_med']}")
    if write_out:
        T.to_csv(os.path.join(OUT, "tgt25_check.csv"), index=False, float_format="%.6g")
        S.to_csv(os.path.join(OUT, "tgt25_statements.csv"), index=False, float_format="%.6g")
        open(os.path.join(OUT, "tgt25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
        vmd = ["# 학습 목표와 구조, 25 ns 격자 (자동 생성)", "",
               f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `tgt25.py compare`. 규칙은 실행 전(10-31 12:15) `tgt25.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, 25 ns), 지도 56점, 상수 D0 두 계단 전력 보간, 20 ms 창.", "",
               f"**F (\"성능을 정하는 것은 구조가 아니라 학습 목표\"): {TXT[ans['F']]}**", ""]
        vmd += ["| 질문 | 판정 | 평균 이득 80주기 최대 / 주기 중앙값 [A] | B 낮음 / 상대 낮음 (최대) | 판정 가능 |", "|---|---|---|---|---|"]
        vmd.append(f"| QA 제어기 A가 SPS보다 높은 점(80주기 최대 / 중앙값: {wa['rule'][0]} / {wa['med'][0]}) | {TXT[ans['QA']]} | | | |")
        for q, s in st.items():
            vmd.append(f"| {q} | {TXT[ans[q]]} | {s['rule']['mean']:+.3f} / {s['med']['mean']:+.3f} | {s['rule']['up']} / {s['rule']['down']} | {s['rule']['dec']}/{s['rule']['n']} |")
        if miss:
            vmd += ["", f"※ 아직 감싸지 못한 값(지도): {miss} — **잠정**."]
        vmd += ["", "표 원본 `out/tgt25_check.csv`, `out/tgt25_statements.csv`, 로그 `out/tgt25_check.log`."]
        open(os.path.join(OUT, "tgt25_verdict.md"), "w", encoding="utf-8").write("\n".join(vmd) + "\n")
    return T, ans, st, lines


# ----------------------------------------------------------------------------------------------------- selftest
def _fake_J(off):
    """made-up runs: Ipk = base(V2) + power / 1000 + off(name) for each set of duties, powers every 50 W"""
    import pandas as pd
    rows, seen = [], set()
    for st, v, p, cs in points():
        for c in cs:
            if c["alias"] is not None or (v, c["D1"], c["D2"]) in seen:
                continue
            seen.add((v, c["D1"], c["D2"]))
            for lv in range(0, 25):
                pd_w = 50.0 * lv
                i = 1.0 + pd_w / 1000.0 + off.get(c["name"], 0.0)
                rows.append(dict(Ts=TS, V2=v, level=lv, D1=c["D1"], D2=c["D2"], **{f"P_LV_at_{rm.WIN}ms": -pd_w, f"Ipk_at_{rm.WIN}ms": i}, iL_mean=0.001,
                                 pk_med=i, spread=0.0, loss=2.0, P_LV=-pd_w, P_HV=-pd_w - 2.0))
    return pd.DataFrame(rows).sort_values(["V2", "D1", "D2", "level"])


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    mk = lambda mean, up, down, same=10, dec=20, n=20: dict(mean=mean, up=up, down=down, same=same, dec=dec, n=n, within=same / max(dec, 1))  # noqa: E731
    two = lambda a, b=None: {"rule": a, "med": a if b is None else b}  # noqa: E731
    chk("QA real", rule_qa(dict(rule=5, med=5), dict(rule=1.0, med=1.0)), "real")
    chk("QA some: median below", rule_qa(dict(rule=5, med=4), dict(rule=1.0, med=1.0)), "some")
    chk("QA none", rule_qa(dict(rule=0, med=0), dict(rule=1.0, med=1.0)), "none")
    chk("QA some: one kind", rule_qa(dict(rule=2, med=0), dict(rule=1.0, med=1.0)), "some")
    chk("QA undecidable", rule_qa(dict(rule=5, med=5), dict(rule=0.7, med=1.0)), "undecidable")
    chk("split", rule_split(two(mk(0.05, 8, 4))), "splits")
    chk("split: not 2 to 1", rule_split(two(mk(0.2, 8, 5))), "partial")
    chk("split: median below", rule_split(two(mk(0.2, 8, 2), mk(0.04, 8, 2))), "partial")
    chk("reversed", rule_split(two(mk(-0.06, 2, 4))), "reversed")
    chk("tie", rule_split(two(mk(0.01, 3, 3))), "tie")
    chk("split undecidable", rule_split(two(mk(0.2, 8, 2, dec=14))), "undecidable")
    chk("E gain", rule_qe(two(mk(0.01, 3, 3))), "gain")
    chk("E no gain: more down", rule_qe(two(mk(0.02, 3, 4))), "no gain")
    chk("E no gain: one kind", rule_qe(two(mk(0.02, 3, 1), mk(0.0, 1, 1))), "no gain")
    chk("E loss", rule_qe(two(mk(-0.02, 1, 3))), "loss")
    g0, g = dict(rule=2, med=0), lambda a, b: dict(rule=a, med=b)  # noqa: E731
    chk("S same", rule_qs(two(mk(0.02, 2, 2, same=14)), g(4, 2), g0), "same")
    chk("S partial: mean", rule_qs(two(mk(0.03, 2, 2, same=14)), g(2, 0), g0), "partial")
    chk("S partial: within", rule_qs(two(mk(0.0, 4, 4, same=12)), g(2, 0), g0), "partial")
    chk("S partial: goal 3", rule_qs(two(mk(0.0, 2, 2, same=14)), g(5, 0), g0), "partial")
    chk("S diff: mean", rule_qs(two(mk(0.06, 2, 2, same=14)), g(2, 0), g0), "diff")
    chk("S diff: within", rule_qs(two(mk(0.0, 5, 5, same=9)), g(2, 0), g0), "diff")
    chk("S diff: goal 3", rule_qs(two(mk(0.0, 2, 2, same=14)), g(8, 0), g0), "diff")
    chk("S undecidable", rule_qs(two(mk(0.0, 2, 2, dec=10)), g(2, 0), g0), "undecidable")
    chk("F holds", rule_f("splits", "same", "same"), "holds")
    chk("F target only", rule_f("splits", "same", "diff"), "target only")
    chk("F structure only", rule_f("tie", "same", "same"), "structure only")
    chk("F neither", rule_f("partial", "diff", "same"), "neither")
    chk("F undecidable", rule_f("splits", "undecidable", "same"), "undecidable")
    pts = points()
    chk("56 + 4 points", (sum(x[0] == "M" for x in pts), sum(x[0] == "R" for x in pts)), (56, 4))
    chk("set R has no structure candidates", all(c["name"] not in PART["b"] for x in pts if x[0] == "R" for c in x[3]), True)
    chk("SPS never aliased", all(x[3][0]["name"] == "SPS" and x[3][0]["alias"] is None for x in pts), True)
    chk("M-B equals the map's M-B", (lambda c: (abs(c[1]["D1"] - 0.212494) < 1e-5, abs(c[1]["D2"] - 0.244112) < 1e-5))(pts[0][3]), (True, True))
    c0 = configs()
    chk("no configuration for SPS or M-B", any(c["ctrl"] in ("SPS", "M-B") for c in c0), False)
    chk("the parts add up", len(configs("a")) + len(configs("b")), len(c0))
    chk("the file's order: part a first", c0[0]["ctrl"], "M-A")
    base = {"SPS": 0.0, "M-B": -0.1, "M-A": 0.2, "ICROS": 0.1, "M-B0": -0.05, "L49": -0.1, "D64": -0.1, "L25": -0.11, "L13": 0.3}
    T, ans, st, _ = compare(_fake_J(base), write_out=False)
    chk("fake: QA", ans["QA"], "real")
    chk("fake: QT", ans["QT"], "splits")
    chk("fake: QI", ans["QI"], "splits")
    chk("fake: QE", ans["QE"], "gain")
    chk("fake: QS", (ans["QS(L49)"], ans["QS(D64)"], ans["QS(L25)"], ans["QS(L13)"]), ("same", "same", "same", "diff"))
    chk("fake: F", ans["F"], "holds")
    chk("fake: the points B shares with a twin are not counted", st["QS(L49)"]["rule"]["n"] < 56, True)
    T, ans, st, _ = compare(_fake_J(dict(base, **{"M-A": -0.1, "L49": -0.2})), write_out=False)
    chk("fake 2: QT", ans["QT"], "tie" if abs(st["QT"]["rule"]["mean"]) <= TIE else "partial")
    chk("fake 2: QS(L49) better beyond 0.05 is still different", ans["QS(L49)"], "diff")
    chk("fake 2: F", ans["F"], "neither")
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["selftest", "predict", "predicted", "make", "next", "compare"])
    ap.add_argument("--part", default="a", choices=["a", "b"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    if a.cmd == "next":
        next_round(a.part)
    else:
        {"predict": predict, "predicted": predicted, "make": make, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
