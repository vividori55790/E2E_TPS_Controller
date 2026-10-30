"""
md_check.py -- (2026-10-30) R13 stage 2: the retrained surface M-D and the rules that decide whether it replaces M-B / M-S.

r13_map.py compare (r13b finished 10-30 22:04): Q1 on the rule value = 11 "not optimal" of 52 decidable points, trigger 5, so
the rule registered at 15:20 says retrain. (On the median period peak it is 4 of 52, which would not trigger; the median is not
part of that rule and the rule is followed.) The previous retraining, M-S, was adopted on 100 ns differences that the 25 ns grid
then erased, and an anchor is by construction the lowest of several candidates. So this time:
  * the decision is taken on points that were never anchors. At an anchor point M-D's value may be the very run that won the
    selection there, which is biased low; those values are reported and used for goal 3 only;
  * every comparison has to hold on the median period peak too. The rule value is the largest of 80 period peaks and carries
    one grid step of noise (r13_map.py note 18:15);
  * a 25 ns "adopt" is not final until the gains are re-measured at 10 ns (stage 3 below).

M-D (the 15:20 rule, unchanged): train_corrected.py --anchors out/r13_anchors.csv --tag MD --delta, alpha 1e-3, no anchor
weight, on M-B's encoder and readout. Anchors = every Q1-decidable map point: target = M-S's own duties where Q1 is "optimal",
the best clean candidate's duties where it is "not optimal"; wV 2.1 V, wP half the gap to the nearer neighbouring |P|.

RULES (registered 2026-10-30 22:13, after the map's Q1, before M-D is trained and before any M-D run; do not change after
the results):
  S  goal 2, before any run: the largest |dDelta/dx| (normalised inputs, the train_corrected.py number) over the dense grid must
     be <= JAC_MAX 1.5 x M-B's. Otherwise "too_rough": M-D is not run and not adopted.
  Values: as the map. 25 ns, 20 ms window, constant D0, two adjacent 2.88-deg levels, Ipk interpolated at |P|
     (fine_grid_check.at_power); clean = both levels |mean i_L| <= 20 mA. Each Ipk in two kinds: "rule" = the largest of the 80
     period peaks (Ipk_at_20ms), "med" = their median (out/iL_pp/). Candidates within SAME 0.003 of an earlier one in the order
     SPS, M-B, M-S, M-D use the earlier one's runs.
     tol = max(0.03 A, 2 % Ipk(SPS)) at the point, of the same kind (2 % Ipk(M-D) if SPS is not clean).
  Sets: H hold-out, off the map and never an anchor: V2 86.1, 90.3, 94.5, 98.7, 102.9, 107.1 V x |P| 125, 175, 250, 350, 500, 900 W,
        + the R.4 points at 95 V (100, 300, 600, 1000 W), + 8 random points (seed 20261030, V2 uniform 84-109.2 V, |P| log-uniform
        100-1000 W, redrawn if within 1 V and 5 % of a map or H point) = 48 points, {SPS, M-B, M-S, M-D}.
        M the 56 map points, M-D (the other candidates are the map's runs).
  Per H point, per reference X in (M-B, M-S), per kind: gain = Ipk(X) - Ipk(M-D).
     better / worse / same: both clean, gain > tol / < -tol / else; fixes_osc: M-D clean, X oscillating (counts with better, no gain
     value); new_osc: X clean, M-D oscillating (counts with worse, no gain value); undecidable: a value missing, or both oscillating.
  A(X, kind) passes iff at least DECIDABLE_MIN 75 % of H is decidable, the mean gain over the H points with a gain value is
     >= GAIN_MIN 0.01 A (about a tenth of M-B's mean saving on the map), and #better + #fixes_osc >= #worse + #new_osc.
     Below 75 % decidable -> "undecidable".
  C(kind) goal 3 on H + M: #points where M-D is above SPS by more than tol (both clean) <= the same count for M-B and for M-S.
  Verdict: too_rough if S fails; undecidable if any A is undecidable;
     adopt    A(M-B, rule), A(M-B, med), A(M-S, rule), A(M-S, med), C(rule), C(med) all hold;
     partial  A holds in both kinds against one of M-B / M-S, and C(rule), C(med);
     no_gain  otherwise. The surface stays as it was.
  Stage 3, only after adopt: the 8 H points with the largest |gain against M-B| (rule value) x {M-B, M-D} at Ts = 10 ns, same method.
     Each 25 ns statement "M-D lower than M-B" / "M-B lower than M-D" (> tol) is classified by fine_grid_check.rule_r2. The adoption
     stands iff #kept >= #gone + #reversed; else it is reported as "adopt at 25 ns only (grid scale)".
  Reported, never a verdict: the same A / C numbers on M (in-sample), M-D's gap to the best map candidate, M-S against M-B on H,
     mean saving against SPS on H (kind by kind), loss against SPS, the fit error at the anchors.

    python -I -X utf8 Simulink/Thesis/md_check.py selftest         (the rules on made-up numbers)
    python -I -X utf8 Simulink/Thesis/md_check.py anchors          -> out/r13_anchors.csv
    python -I -X utf8 Simulink/Thesis/train_corrected.py --anchors out/r13_anchors.csv --tag MD --delta
    python -I -X utf8 Simulink/Thesis/md_check.py smooth           (rule S)
    python -I -X utf8 Simulink/Thesis/md_check.py predict          -> out/md_pred.csv (levels from the 25 ns plant)
    python -I -X utf8 Simulink/Thesis/md_check.py make             -> out/md_points_r1.csv
    python automation/job.py submit md1 --interactive --after rob25 -- matlab -batch "cd('Simulink/Thesis'); md_rounds(1, 5, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/md_check.py next             (not bracketed -> next round; md_rounds calls it)
    python -I -X utf8 Simulink/Thesis/md_check.py compare          -> out/md_check.{log,csv}, out/md_verdict.md, out/fig_md_check.png
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
import r13_map as rm  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
TS = rm.TS
WIN = fg.WIN
OSC = fg.OSC
SAME = rm.SAME
TOL_ABS, TOL_REL = rm.TOL_ABS, rm.TOL_REL
JAC_MAX = 1.5
GAIN_MIN = 0.01
DECIDABLE_MIN = 0.75
WV = 2.1
V2H = (86.1, 90.3, 94.5, 98.7, 102.9, 107.1)
PH = (125.0, 175.0, 250.0, 350.0, 500.0, 900.0)
R4 = tuple((95.0, p) for p in (100.0, 300.0, 600.0, 1000.0))
N_RANDOM, SEED = 8, 20261030
TAG = "MD"
MODEL = os.path.join(OUT, f"model_smooth_reghdc_{TAG}.json")
ANCH = os.path.join(OUT, "r13_anchors.csv")
PRED = os.path.join(OUT, "md_pred.csv")
PATS = rm.PATS + ("md_sweep_r*.csv",)
NAMES = ("SPS", "M-B", "M-S", "M-D")
KEY = {"SPS": "SPS", "M-B": "MB", "M-S": "MS", "M-D": "MD"}
KINDS = ("rule", "med")


# ----------------------------------------------------------------------------------------------------- anchors, model
def wp_of(p):
    i = rm.PS.index(p)
    return 0.5 * min(abs(rm.PS[j] - p) for j in (i - 1, i + 1) if 0 <= j < len(rm.PS))


def anchor_rows(Q):
    """the 15:20 rule on the table of out/r13_points.csv: every decidable point; M-S's duties if optimal, else the best's"""
    rows = []
    for _, q in Q.iterrows():
        if q["q1"] == "undecidable":
            continue
        opt = q["q1"] == "optimal"
        rows.append(dict(V2=q["V2"], P=q["P"], D1=q["MS_D1"] if opt else q["best_D1"], D2=q["MS_D2"] if opt else q["best_D2"],
                         wV=WV, wP=wp_of(q["P"]), mode="set",
                         source="M-S (Q1 optimal)" if opt else f"{q['best']} {q['gap']:.3f} A below M-S (Q1)"))
    return rows


def anchors():
    import pandas as pd
    Q = pd.read_csv(os.path.join(OUT, "r13_points.csv"))
    miss = Q[Q["n_missing"] > 0]
    if len(miss):
        print(f"warning: {len(miss)} map points still have unbracketed candidates: "
              + ", ".join(f"{r['V2']:g} V {r['P']:g} W" for _, r in miss.iterrows()))
    A = pd.DataFrame(anchor_rows(Q), columns=["V2", "P", "D1", "D2", "wV", "wP", "source", "mode"])
    with open(ANCH, "w", newline="", encoding="utf-8") as f:
        f.write(f"# R13 stage 2 anchors (md_check.py anchors, {datetime.datetime.now():%Y-%m-%d %H:%M}, from out/r13_points.csv = "
                "r13_map.py compare): the 15:20 rule of r13_map.py\n")
        A.to_csv(f, index=False, float_format="%.6f")
    print(f"wrote {ANCH}: {len(A)} anchors, {int(A['source'].str.contains('below').sum())} away from M-S")


def _md():
    if not os.path.isfile(MODEL):
        sys.exit(f"{MODEL} not found: train it first (train_corrected.py --tag {TAG} --delta)")
    return ct.SmoothRegHDC.from_json(MODEL)


def _one(m, p, v):
    return tuple(float(np.ravel(x)[0]) for x in m.predict(np.array([p]), np.array([v])))


def rule_s(jm, jb):
    return bool(jm <= JAC_MAX * jb)


def smooth():
    """rule S + the largest step between adjacent dense-grid nodes (reported)"""
    import surface_opt_check as so
    from smooth_surface_v2 import dense_grid
    _, _, PP, VV = dense_grid()
    P, V = PP.ravel(), VV.ravel()
    jac = {}
    for nm, m in (("M-B", so._mb()), ("M-S", so._ms()), ("M-D", _md())):
        jac[nm] = float(np.abs(m.jacobian(P, V)).max())
        d1, d2 = (np.ravel(x).reshape(PP.shape) for x in m.predict(P, V))
        step = lambda a, ax: float(np.abs(np.diff(a, axis=ax)).max())  # noqa: E731
        op = (PP >= 100) & (PP <= 1000)
        lo = lambda a: np.where(op, a, np.nan)  # noqa: E731
        sp = max(np.nanmax(np.abs(np.diff(lo(x), axis=1))) for x in (d1, d2))
        sv = max(np.nanmax(np.abs(np.diff(lo(x), axis=0))) for x in (d1, d2))
        print(f"{nm}: max |dDelta/dx| {jac[nm]:.2f}; largest step between nodes over the whole grid {max(step(d1, 1), step(d2, 1)):.4f} per 10 W, "
              f"{max(step(d1, 0), step(d2, 0)):.4f} per 0.4 V; at 100-1000 W {sp:.4f} per 10 W, {sv:.4f} per 0.4 V")
    ok = rule_s(jac["M-D"], jac["M-B"])
    print(f"rule S: M-D {jac['M-D']:.2f} vs {JAC_MAX} x M-B = {JAC_MAX * jac['M-B']:.2f} -> {'pass' if ok else 'FAIL (too_rough)'}")
    return ok


# ----------------------------------------------------------------------------------------------------- points
def _near_any(v, p, pts):
    return any(abs(v - a) < 1.0 and abs(p - b) < 0.05 * b for a, b in pts)


def hold_points():
    """(V2, |P|, source) of the hold-out set: grid, R.4 at 95 V, random"""
    pts = [(v, p, "G") for v in V2H for p in PH] + [(v, p, "R4") for v, p in R4]
    rng = np.random.default_rng(SEED)
    mp = [(v, p) for v in rm.V2S for p in rm.PS]
    out = []
    while len(out) < N_RANDOM:
        v = round(float(rng.uniform(84.0, 109.2)), 1)
        p = float(round(np.exp(rng.uniform(np.log(100.0), np.log(1000.0)))))
        if not _near_any(v, p, mp + [(a, b) for a, b, _ in pts + out]):
            out.append((v, p, "RND"))
    return pts + out


def cands_hold(b, s, m):
    """SPS, M-B, M-S, M-D with aliases from the three (D1, D2)"""
    cs = [dict(name="SPS", D1=sw.D_SAFE, D2=sw.D_SAFE), dict(name="M-B", D1=rm._dd(b[0]), D2=rm._dd(b[1])),
          dict(name="M-S", D1=rm._dd(s[0]), D2=rm._dd(s[1])), dict(name="M-D", D1=rm._dd(m[0]), D2=rm._dd(m[1]))]
    out = []
    for c in cs:
        out.append(rm._alias(c, out))
    return out


def hold_cands():
    import surface_opt_check as so
    mb, ms, md = so._mb(), so._ms(), _md()
    return [(v, p, src, cands_hold(_one(mb, p, v), _one(ms, p, v), _one(md, p, v))) for v, p, src in hold_points()]


def map_cands():
    """the map's candidates (r13_map.candidates) + M-D, which uses an earlier candidate's runs when within SAME"""
    md, out = _md(), []
    for v, p, cs in rm.candidates("map"):
        x = _one(md, p, v)
        cs = list(cs) + [rm._alias(dict(name="M-D", D1=rm._dd(x[0]), D2=rm._dd(x[1])), cs)]
        out.append((v, p, cs))
    return out


def configs():
    """the configurations with their own runs, decisive set first"""
    cf = []
    for k, (v, p, _, cs) in enumerate(hold_cands()):
        cf += [dict(set="H", pair=k, ctrl=c["name"], V2=v, P=p, D1=c["D1"], D2=c["D2"]) for c in cs if c["alias"] is None]
    for v, p, cs in map_cands():
        c = cs[-1]
        if c["alias"] is None:
            cf.append(dict(set="M", pair=rm.V2S.index(v), ctrl="M-D", V2=v, P=p, D1=c["D1"], D2=c["D2"]))
    return cf


def results():
    return rm.results_25(PATS)


def predict():
    rm.predict(configs(), PRED)


def make():
    d, lv, pts, skip = results(), rm._pred_levels(PRED), [], 0
    cf = configs()
    for c in cf:
        g = rm._runs(d, c)
        if not g.empty and np.isfinite(fg.at_power(g, c["P"])[0]):
            skip += 1
            continue
        have = set(g["level"].astype(int)) if not g.empty else set()
        if rm._key(c) not in lv:
            sys.exit(f"no predicted levels for {c} -- rerun `md_check.py predict`")
        pts += [fg._point(c, j, TS) for j in lv[rm._key(c)] if j not in have]
    print(f"{len(cf)} configurations ({sum(c['set'] == 'H' for c in cf)} hold-out), {skip} already bracketed by existing 25 ns runs")
    fg._write(os.path.join(OUT, "md_points_r1.csv"), pts)


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "md_sweep_r*.csv"))
          for m in [re.search(r"md_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d, lv0, pts = results(), rm._pred_levels(PRED), []
    for c in configs():
        g = rm._runs(d, c)
        if g.empty:
            pts += [fg._point(c, j, TS) for j in lv0.get(rm._key(c), ())]
        elif not np.isfinite(fg.at_power(g, c["P"])[0]):
            pts += [fg._point(c, j, TS) for j in rm.extend(g["level"].to_numpy(), -g[f"P_LV_at_{WIN}ms"].to_numpy(), c["P"])]
    if pts:
        fg._write(os.path.join(OUT, f"md_points_r{round_no()}.csv"), pts)
    else:
        print("every M-D configuration is bracketed")


# ----------------------------------------------------------------------------------------------------- rules
def clean(x):
    return rm.clean(x)


def tol_of(sps, md):
    ref = sps if clean(sps) else md
    return max(TOL_ABS, TOL_REL * ref[0]) if clean(ref) else TOL_ABS


def classify(md, x, tol):
    """M-D (value, osc) against X (value, osc) -> (class, gain = X - M-D)"""
    cm, cx = clean(md), clean(x)
    if cm and cx:
        g = x[0] - md[0]
        return ("better" if g > tol else "worse" if g < -tol else "same"), float(g)
    hm, hx = md is not None and np.isfinite(md[0]), x is not None and np.isfinite(x[0])
    if cm and hx:
        return "fixes_osc", np.nan
    if cx and hm:
        return "new_osc", np.nan
    return "undecidable", np.nan


def rule_a(codes, gains):
    """-> (pass / fail / undecidable, mean gain, #up, #down, #decidable)"""
    codes = list(codes)
    n, dec = len(codes), sum(c != "undecidable" for c in codes)
    g = [x for x in gains if np.isfinite(x)]
    up = codes.count("better") + codes.count("fixes_osc")
    down = codes.count("worse") + codes.count("new_osc")
    if n == 0 or dec < DECIDABLE_MIN * n or not g:
        return "undecidable", np.nan, up, down, dec
    mu = float(np.mean(g))
    return ("pass" if mu >= GAIN_MIN and up >= down else "fail"), mu, up, down, dec


def rule_c(n_md, n_mb, n_ms):
    return n_md <= n_mb and n_md <= n_ms


def verdict(s_ok, a, c):
    """a: (ref, kind) -> pass / fail / undecidable; c: kind -> bool"""
    if not s_ok:
        return "too_rough"
    if any(x == "undecidable" for x in a.values()):
        return "undecidable"
    both = lambda ref: all(a[(ref, k)] == "pass" for k in KINDS)  # noqa: E731
    cc = all(c[k] for k in KINDS)
    if both("M-B") and both("M-S") and cc:
        return "adopt"
    if (both("M-B") or both("M-S")) and cc:
        return "partial"
    return "no_gain"


def stage3_stands(res):
    res = list(res)
    return res.count("kept") >= res.count("gone") + res.count("reversed")


def goal3(x, sps, tol):
    return bool(clean(x) and clean(sps) and x[0] - sps[0] > tol)


def selftest():
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    c = lambda i, o=0.0: (i, o)  # noqa: E731
    chk("S at the limit", rule_s(4.5, 3.0), True)
    chk("S beyond", rule_s(4.51, 3.0), False)
    chk("tol from SPS", round(tol_of(c(2.0), c(1.0)), 6), 0.04)
    chk("tol floor", round(tol_of(c(1.0), c(1.0)), 6), 0.03)
    chk("tol from M-D when SPS oscillates", round(tol_of(c(3.0, 0.05), c(2.5)), 6), 0.05)
    chk("better", classify(c(1.0), c(1.1), 0.03)[0], "better")
    chk("worse", classify(c(1.1), c(1.0), 0.03)[0], "worse")
    chk("same", classify(c(1.0), c(1.02), 0.03)[0], "same")
    chk("gain value", round(classify(c(1.0), c(1.02), 0.03)[1], 6), 0.02)
    chk("fixes_osc", classify(c(1.0), c(1.5, 0.03), 0.03)[0], "fixes_osc")
    chk("new_osc", classify(c(1.5, 0.03), c(1.0), 0.03)[0], "new_osc")
    chk("both oscillate", classify(c(1.5, 0.03), c(1.0, 0.03), 0.03)[0], "undecidable")
    chk("missing", classify(c(np.nan, np.nan), c(1.0), 0.03)[0], "undecidable")
    chk("missing median, clean X", classify(c(np.nan, 0.0), c(1.0), 0.03)[0], "undecidable")
    chk("A mean below 0.01", rule_a(["better", "same", "same", "worse"], [0.06, 0.0, 0.0, -0.05])[0], "fail")
    chk("A pass at 0.01", rule_a(["better", "same", "same", "worse"], [0.08, 0.0, 0.0, -0.04])[0], "pass")
    chk("A more worse", rule_a(["better", "worse", "worse"], [0.5, -0.04, -0.04])[0], "fail")
    chk("A fixes count up", rule_a(["fixes_osc", "better", "worse", "worse"], [np.nan, 0.5, -0.04, -0.04])[0], "pass")
    chk("A new_osc count down", rule_a(["better", "new_osc", "new_osc"], [0.5, np.nan, np.nan])[0], "fail")
    chk("A undecidable below 75 %", rule_a(["better"] * 2 + ["undecidable"], [0.5, 0.5, np.nan])[0], "undecidable")
    chk("A at 75 %", rule_a(["better"] * 3 + ["undecidable"], [0.5, 0.5, 0.5, np.nan])[0], "pass")
    chk("C holds on a tie", rule_c(2, 2, 2), True)
    chk("C worse than M-B", rule_c(3, 2, 4), False)
    chk("C worse than M-S", rule_c(3, 4, 2), False)
    aa = {(r, k): "pass" for r in ("M-B", "M-S") for k in KINDS}
    cc = {k: True for k in KINDS}
    chk("adopt", verdict(True, aa, cc), "adopt")
    chk("too rough first", verdict(False, aa, cc), "too_rough")
    chk("undecidable", verdict(True, {**aa, ("M-S", "med"): "undecidable"}, cc), "undecidable")
    chk("partial: not against M-S", verdict(True, {**aa, ("M-S", "rule"): "fail"}, cc), "partial")
    chk("partial: not against M-B", verdict(True, {**aa, ("M-B", "med"): "fail"}, cc), "partial")
    chk("no gain: rule only", verdict(True, {**aa, ("M-S", "med"): "fail", ("M-B", "med"): "fail"}, cc), "no_gain")
    chk("no gain: goal 3 worse", verdict(True, aa, dict(cc, med=False)), "no_gain")
    chk("stage 3 stands", stage3_stands(["kept", "gone", "kept", "undecidable"]), True)
    chk("stage 3 tie stands", stage3_stands(["kept", "reversed"]), True)
    chk("stage 3 falls", stage3_stands(["kept", "gone", "reversed"]), False)
    chk("goal 3 above", goal3(c(1.1), c(1.0), 0.03), True)
    chk("goal 3 within", goal3(c(1.02), c(1.0), 0.03), False)
    chk("goal 3 oscillating SPS", goal3(c(1.1), c(1.0, 0.03), 0.03), False)
    chk("wP", [wp_of(p) for p in rm.PS], [25.0, 25.0, 25.0, 50.0, 50.0, 100.0, 100.0, 100.0])
    import pandas as pd
    Q = pd.DataFrame([dict(V2=84.0, P=100.0, q1="optimal", MS_D1=0.2, MS_D2=0.3, best_D1=0.25, best_D2=0.35, best="M-B", gap=0.0),
                      dict(V2=84.0, P=300.0, q1="not optimal", MS_D1=0.2, MS_D2=0.3, best_D1=0.23, best_D2=0.3, best="D1+", gap=0.1),
                      dict(V2=84.0, P=400.0, q1="undecidable", MS_D1=0.2, MS_D2=0.3, best_D1=np.nan, best_D2=np.nan, best="", gap=np.nan)])
    r = anchor_rows(Q)
    chk("anchors: decidable only", len(r), 2)
    chk("anchors: M-S if optimal", (r[0]["D1"], r[0]["D2"]), (0.2, 0.3))
    chk("anchors: best if not", (r[1]["D1"], r[1]["D2"], r[1]["wP"], r[1]["wV"]), (0.23, 0.3, 50.0, 2.1))
    a = cands_hold((0.3, 0.3), (0.301, 0.302), (0.35, 0.3))
    chk("hold: M-S on M-B", (a[2]["alias"], a[3]["alias"]), ("M-B", None))
    a = cands_hold((0.3, 0.3), (0.35, 0.35), (0.351, 0.349))
    chk("hold: M-D on M-S", (a[2]["alias"], a[3]["alias"]), (None, "M-S"))
    a = cands_hold((0.4999, 0.4999), (0.45, 0.45), (0.4995, 0.4995))
    chk("hold: M-B and M-D on SPS", (a[1]["alias"], a[3]["alias"]), ("SPS", "SPS"))
    h = hold_points()
    chk("hold-out size", len(h), 48)
    chk("hold-out sources", [sum(x[2] == s for x in h) for s in ("G", "R4", "RND")], [36, 4, 8])
    chk("hold-out off the map", any((v, p) in [(a_, b_) for a_ in rm.V2S for b_ in rm.PS] for v, p, _ in h), False)
    chk("hold-out fixed", hold_points() == h, True)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


# ----------------------------------------------------------------------------------------------------- compare
def _vals(J, v, p, cs):
    """name -> {rule: (Ipk, osc), med: (Ipk, osc), loss}"""
    by, out = {c["name"]: c for c in cs}, {}
    for c in cs:
        own = by[c["alias"]] if c["alias"] else c
        g = fg.runs_of(J, dict(V2=v, D1=own["D1"], D2=own["D2"]), TS)
        I, o, _ = fg.at_power(g, p)
        m = rm._at_col(g, p, "pk_med")[0] if not g.empty else np.nan
        loss = rm._at_col(g, p, "loss")[0] if not g.empty else np.nan
        out[c["name"]] = dict(rule=(I, o), med=(m, o), loss=loss)
    return out


def _record(st, v, p, src, cs, vals):
    rec = dict(set=st, src=src, V2=v, P=p)
    by = {c["name"]: c for c in cs}
    for nm in NAMES:
        k = KEY[nm]
        rec.update({f"I_{k}": vals[nm]["rule"][0], f"med_{k}": vals[nm]["med"][0], f"osc_{k}": vals[nm]["rule"][1],
                    f"loss_{k}": vals[nm]["loss"], f"D1_{k}": by[nm]["D1"], f"D2_{k}": by[nm]["D2"], f"alias_{k}": by[nm]["alias"] or ""})
    for kind in KINDS:
        t = tol_of(vals["SPS"][kind], vals["M-D"][kind])
        rec[f"tol_{kind}"] = t
        for ref in ("M-B", "M-S"):
            rec[f"cls_{KEY[ref]}_{kind}"], rec[f"gain_{KEY[ref]}_{kind}"] = classify(vals["M-D"][kind], vals[ref][kind], t)
        rec[f"mdms_{kind}"] = classify(vals["M-S"][kind], vals["M-B"][kind], t)[1]
        for nm in ("M-B", "M-S", "M-D"):
            rec[f"g3_{KEY[nm]}_{kind}"] = goal3(vals[nm][kind], vals["SPS"][kind], t)
    return rec


def compare():
    import pandas as pd
    d = results()
    if d is None:
        sys.exit("no 25 ns runs")
    J = rm.per_period(d, PATS)
    recs = []
    for v, p, src, cs in hold_cands():
        recs.append(_record("H", v, p, src, cs, _vals(J, v, p, cs)))
    for v, p, cs in map_cands():
        recs.append(_record("M", v, p, "MAP", cs, _vals(J, v, p, cs)))
    R = pd.DataFrame(recs)
    R.to_csv(os.path.join(OUT, "md_check.csv"), index=False, float_format="%.6g")
    H, M = R[R["set"] == "H"], R[R["set"] == "M"]
    lines = []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)

    f3 = lambda x: f"{x:6.3f}" if np.isfinite(x) else f"{'-':>6}"  # noqa: E731
    o("md_check: M-D at 25 ns, 20 ms window, Ipk at |P| (A); rules in the file header")
    o(f"{'set':>3} {'V2':>6} {'|P|':>5} | {'SPS':>6} {'M-B':>6} {'M-S':>6} {'M-D':>6} | med: {'SPS':>6} {'M-B':>6} {'M-S':>6} {'M-D':>6} | "
      f"{'tol':>5} vs M-B  vs M-S (rule)")
    for _, r in R.iterrows():
        rule = [f3(r[f"I_{KEY[n]}"]) + ("~" if np.isfinite(r[f"osc_{KEY[n]}"]) and r[f"osc_{KEY[n]}"] > OSC else " ") for n in NAMES]
        med = [f3(r[f"med_{KEY[n]}"]) + " " for n in NAMES]
        o(f"{r['set']:>3} {r['V2']:6.1f} {r['P']:5.0f} | {''.join(rule)}| med: {''.join(med)}| {r['tol_rule']:.3f} {r['cls_MB_rule']:<11} {r['cls_MS_rule']:<11}")
    o("(~ = oscillating, |mean i_L| > 20 mA; - = not bracketed yet)")

    def block(T, name):
        res = {}
        for ref in ("M-B", "M-S"):
            for kind in KINDS:
                k = f"{KEY[ref]}_{kind}"
                res[(ref, kind)] = rule_a(T[f"cls_{k}"], T[f"gain_{k}"])
                code, mu, up, down, dec = res[(ref, kind)]
                cc = list(T[f"cls_{k}"])
                o(f"   {name} A({ref}, {kind}): {code}; mean gain {mu:+.3f} A over {int(T[f'gain_{k}'].notna().sum())} points; "
                  f"better {cc.count('better')} + fixes_osc {cc.count('fixes_osc')} vs worse {cc.count('worse')} + new_osc {cc.count('new_osc')}, "
                  f"same {cc.count('same')}, decidable {dec}/{len(T)}")
        return res

    o("\nDecisive: the hold-out set H")
    AH = block(H, "H")
    o("\nReported, not a verdict: the map M (in-sample, anchors picked as the lowest of several candidates)")
    block(M, "M")
    C = {}
    o("\nGoal 3 on H + M (points above SPS beyond tol):")
    for kind in KINDS:
        n = {nm: int(R[f"g3_{KEY[nm]}_{kind}"].sum()) for nm in ("M-B", "M-S", "M-D")}
        C[kind] = rule_c(n["M-D"], n["M-B"], n["M-S"])
        o(f"   C({kind}): M-D {n['M-D']}, M-B {n['M-B']}, M-S {n['M-S']} -> {'holds' if C[kind] else 'fails'}")
        for nm in ("M-B", "M-S", "M-D"):
            bad = R[R[f"g3_{KEY[nm]}_{kind}"]]
            if len(bad):
                o(f"      {nm}: " + ", ".join(f"{x['V2']:g} V {x['P']:.0f} W ({x['I_' + KEY[nm]] if kind == 'rule' else x['med_' + KEY[nm]]:.3f} vs "
                                              f"{x['I_SPS'] if kind == 'rule' else x['med_SPS']:.3f} A)" for _, x in bad.iterrows()))
    s_ok = _s_rule()
    v = verdict(s_ok, {k: x[0] for k, x in AH.items()}, C)
    miss = int(R[[f"I_{KEY[n]}" for n in NAMES]].isna().sum().sum())
    o(f"\nVERDICT: {v}" + (f" (PROVISIONAL: {miss} values not bracketed yet)" if miss else ""))
    o("\nReported, not a verdict:")
    for kind, pre in (("rule", "I"), ("med", "med")):
        for nm in ("M-B", "M-S", "M-D"):
            k = KEY[nm]
            m = H[(H[f"osc_{k}"] <= OSC) & (H["osc_SPS"] <= OSC) & H[f"{pre}_{k}"].notna() & H[f"{pre}_SPS"].notna()]
            if len(m):
                dI = m[f"{pre}_SPS"] - m[f"{pre}_{k}"]
                dl = m[f"loss_{k}"] - m["loss_SPS"]
                o(f"   H, {kind}: SPS - {nm} over {len(m)} clean points: mean {dI.mean():+.3f} A (median {dI.median():+.3f}), "
                  f"{(100 * dI / m[f'{pre}_SPS']).mean():+.1f} % of SPS; loss {dl.mean():+.2f} W")
    for kind in KINDS:
        g = H[f"mdms_{kind}"]
        if g.notna().any():
            o(f"   H, {kind}: M-B - M-S (the map's Q4 off the map): mean {g.mean():+.3f} A over {int(g.notna().sum())} points")
    for s, x in H.groupby("src"):
        o(f"   H subset {s}: {len(x)} points, mean gain against M-B {x['gain_MB_rule'].mean():+.3f} A (rule), {x['gain_MB_med'].mean():+.3f} A (median); "
          f"against M-S {x['gain_MS_rule'].mean():+.3f}, {x['gain_MS_med'].mean():+.3f} A")
    for lab, m in (("V2 <= 92", H[H["V2"] <= 92.4]), ("92 < V2 < 100", H[(H["V2"] > 92.4) & (H["V2"] < 100)]), ("V2 >= 100", H[H["V2"] >= 100])):
        o(f"   H {lab}: {len(m)} points, mean gain against M-B {m['gain_MB_rule'].mean():+.3f} A (rule), {m['gain_MB_med'].mean():+.3f} A (median)")
    for lab, m in (("|P| <= 350 W", H[H["P"] <= 350]), ("|P| > 350 W", H[H["P"] > 350])):
        o(f"   H {lab}: {len(m)} points, mean gain against M-B {m['gain_MB_rule'].mean():+.3f} A (rule), {m['gain_MB_med'].mean():+.3f} A (median)")
    for kind, pre, name, q1, gp in (("rule", "I", "r13_points.csv", "q1", "gap"), ("med", "med", "r13_points_med.csv", "q1_med", "gap_med")):
        q = pd.read_csv(os.path.join(OUT, name))
        q = q[q[q1] != "undecidable"].assign(best_val=lambda x: x[f"{pre}_MS"] - x[gp])[["V2", "P", "best_val"]].merge(M, on=["V2", "P"])
        q = q[q["osc_MD"] <= OSC]
        g = q[f"{pre}_MD"] - q["best_val"]
        o(f"   M, {kind}: M-D - best map candidate over {len(q)} points: mean {g.mean():+.3f} A, median {g.median():+.3f}, largest {g.max():+.3f} A; "
          f"above tol at {int((g > q[f'tol_{kind}']).sum())} points")
    open(os.path.join(OUT, "md_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    KO = {"adopt": "채택", "partial": "부분", "no_gain": "이득 없음", "undecidable": "판정 불가", "too_rough": "매끄러움 실패"}
    md = ["# R13 2단계: 재학습 곡면 M-D의 판정 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `md_check.py compare`. 규칙은 `md_check.py` 머리말(10-30 22:13, M-D 학습·실행 전 등록)이며 바꾸지 않았다. 모두 Simulink"
          "(데이터셋 시기 모델, 제어기·로컬 솔버 25 ns) 정상상태, 상수 D0 두 계단 전력 보간, 20 ms 창.", "",
          f"**판정: {KO[v]}**" + (f" — 잠정(감싸지 못한 값 {miss}개)" if miss else ""), "",
          "| 기준 | 값 | 판정 | 평균 이득 [A] | 나아짐 + 진동 해소 | 나빠짐 + 새 진동 | 판정 가능 |", "|---|---|---|---|---|---|---|"]
    for (ref, kind), (code, mu, up, down, dec) in AH.items():
        md.append(f"| {ref} | {'80주기 최대' if kind == 'rule' else '주기 중앙값'} | {code} | {mu:+.3f} | {up} | {down} | {dec}/{len(H)} |")
    md += ["", f"목표 3(H + 지도): 규칙 값 {'유지' if C['rule'] else '악화'}, 중앙값 {'유지' if C['med'] else '악화'}. 지도 점 값은 앵커 선택 편향이 있어 판정에 쓰지 않는다.", "",
           "표 원본 `out/md_check.csv`, 로그 `out/md_check.log`, 그림 `out/fig_md_check.png`."]
    open(os.path.join(OUT, "md_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    fig(R)
    print("wrote out/md_check.{log,csv}, out/md_verdict.md, out/fig_md_check.png")


def _s_rule():
    """rule S from the models"""
    import surface_opt_check as so
    from smooth_surface_v2 import dense_grid
    _, _, PP, VV = dense_grid()
    P, V = PP.ravel(), VV.ravel()
    return rule_s(float(np.abs(_md().jacobian(P, V)).max()), float(np.abs(so._mb().jacobian(P, V)).max()))


def fig(R):
    """gain of M-D at each operating point: columns against M-B and M-S, rows rule value and median; H large, map small"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, Normalize
    from evaluate import INK2, MUTED, _style
    _style(plt)
    div = LinearSegmentedColormap.from_list("red_gray_blue", ["#e34948", "#f2a7a6", "#f0efec", "#86b6ef", "#1c5cab"])
    vals = np.concatenate([R[f"gain_{r}_{k}"].to_numpy(float) for r in ("MB", "MS") for k in KINDS])
    L = max(0.05, float(np.nanmax(np.abs(vals)))) if np.isfinite(vals).any() else 0.05
    nm = Normalize(-L, L)
    f, ax = plt.subplots(2, 2, figsize=(11.5, 7.2), sharex=True, sharey=True, constrained_layout=True)
    sc = None
    for i, kind in enumerate(KINDS):
        for j, (rk, ref) in enumerate((("MB", "M-B"), ("MS", "M-S"))):
            a = ax[i, j]
            g = R[f"gain_{rk}_{kind}"].to_numpy(float)
            isH = (R["set"] == "H").to_numpy()
            for mask, size, lw in ((~isH, 34, 0.4), (isH, 120, 0.8)):
                ok = mask & np.isfinite(g)
                sc = a.scatter(R["P"][ok], R["V2"][ok], c=g[ok], cmap=div, norm=nm, s=size, edgecolors=INK2, linewidths=lw, zorder=3)
                no = mask & ~np.isfinite(g)
                a.scatter(R["P"][no], R["V2"][no], marker="x", s=size * 0.4, color=MUTED, linewidths=1.0, zorder=2)
            a.set_xscale("log")
            a.set_xticks((100, 200, 300, 500, 1000), ("100", "200", "300", "500", "1000"))
            a.minorticks_off()
            a.set_title(f"Ipk({ref}) − Ipk(M-D), {'largest of 80 period peaks (rule)' if kind == 'rule' else 'median period peak'}", fontsize=9, color=INK2)
    for a in ax[1]:
        a.set_xlabel("|P| [W]")
    for a in ax[:, 0]:
        a.set_ylabel("V2 [V]")
    f.colorbar(sc, ax=ax, fraction=0.025, pad=0.02, label="[A], + = M-D lower")
    f.suptitle("R13 stage 2: M-D at 25 ns. Large = hold-out (decides), small = map (anchors, in-sample), × = no value", fontsize=11)
    f.savefig(os.path.join(OUT, "fig_md_check.png"), dpi=130)
    plt.close(f)


def figk():
    """SPS - M-B against k on the map and the hold-out set together, 22 V2 values instead of 7 (added 10-31 08:12 after the verdict,
    never a verdict) -> out/fig_md_k.png, out/md_k.log, out/md_k.csv"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import pandas as pd
    from matplotlib.lines import Line2D
    from evaluate import AQUA, BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    R = pd.read_csv(os.path.join(OUT, "md_check.csv"))
    R = R[(R["osc_MB"] <= OSC) & (R["osc_SPS"] <= OSC) & R["I_MB"].notna() & R["I_SPS"].notna()].copy()
    R["k"] = 4 * R["V2"] / 380.0
    R["x"] = np.abs(1.0 - R["k"])
    for kind, pre in (("rule", "I"), ("med", "med")):
        R[f"dI_{kind}"] = R[f"{pre}_SPS"] - R[f"{pre}_MB"]
    rows, lines = [], []

    def o(x=""):
        print(x, flush=True)
        lines.append(x)
    o("SPS - M-B per V2, map + hold-out, clean points (A); above = M-B above SPS by more than tol, below = M-B below SPS by more than tol")
    o(f"{'V2':>6} {'k':>6} {'n':>3} | med: {'mean':>7} {'min':>7} {'max':>7} {'above':>5} {'below':>5} | rule: {'mean':>7} {'above':>5} {'below':>5} | {'loss':>6} W | off SPS")
    for v, g in R.groupby("V2"):
        rec = dict(V2=v, k=4 * v / 380.0, n=len(g))
        for kind in ("rule", "med"):
            d, t = g[f"dI_{kind}"], g[f"tol_{kind}"]
            rec.update({f"mean_{kind}": d.mean(), f"min_{kind}": d.min(), f"max_{kind}": d.max(), f"above_{kind}": int((d < -t).sum()), f"below_{kind}": int((d > t).sum())})
        rec["loss"] = (g["loss_MB"] - g["loss_SPS"]).mean()
        rec["off"] = int((g[["D1_MB", "D2_MB"]].min(axis=1) < 0.497).sum())
        rows.append(rec)
        o(f"{v:6g} {rec['k']:6.3f} {len(g):3d} | med: {rec['mean_med']:+7.3f} {rec['min_med']:+7.3f} {rec['max_med']:+7.3f} {rec['above_med']:5d} {rec['below_med']:5d} | "
          f"rule: {rec['mean_rule']:+7.3f} {rec['above_rule']:5d} {rec['below_rule']:5d} | {rec['loss']:+6.2f} W | {rec['off']}/{len(g)}")
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "md_k.csv"), index=False, float_format="%.6g")
    for lim in (0.03, 0.04, 0.05, 0.06):
        b = R[R["x"] <= lim]
        o(f"|1 - k| <= {lim:.2f}: {len(b)} points, V2 {b['V2'].min():g}-{b['V2'].max():g} V; median kind mean {b['dI_med'].mean():+.3f} A, above tol {int((b['dI_med'] < -b['tol_med']).sum())}, "
          f"below tol {int((b['dI_med'] > b['tol_med']).sum())}; rule mean {b['dI_rule'].mean():+.3f} A, above {int((b['dI_rule'] < -b['tol_rule']).sum())}, below {int((b['dI_rule'] > b['tol_rule']).sum())}; "
          f"loss {(b['loss_MB'] - b['loss_SPS']).mean():+.2f} W")
    open(os.path.join(OUT, "md_k.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    groups = (("|P| ≤ 200 W", R["P"] <= 200, BLUE, "o"), ("250–500 W", (R["P"] > 200) & (R["P"] <= 500), ORANGE, "s"), ("600–1000 W", R["P"] > 500, AQUA, "^"))
    S = "#fcfcfb"
    f, ax = plt.subplots(1, 2, figsize=(11.5, 4.6))
    for a, ylim in zip(ax, ((-0.15, 1.05), (-0.11, 0.11))):
        a.axhline(0.0, color=MUTED, lw=0.8, zorder=1)
        a.axhspan(-0.03, 0.03, color="#eeede8", lw=0, zorder=0)
        for _, mask, c, m in groups:
            for st, hollow in (("M", False), ("H", True)):
                g = R[mask & (R["set"] == st)]
                if hollow:
                    a.scatter(g["k"], g["dI_med"], s=42, marker=m, facecolors=S, edgecolors=c, linewidths=1.4, zorder=3)
                else:
                    a.scatter(g["k"], g["dI_med"], s=42, marker=m, color=c, edgecolors=S, linewidths=0.8, zorder=3)
        a.set_ylim(*ylim)
        a.set_xlabel("k = n·V2 / V1")
    ax[0].set_ylabel("Ipk(SPS) − Ipk(controller B), median period peak [A]")
    ax[0].set_title("All clean points", loc="left")
    ax[1].set_xlim(0.93, 1.07)
    ax[1].set_title("Around k = 1: grey band = ±0.03 A, the smallest tolerance of the rules", loc="left")
    hs = [Line2D([], [], ls="", marker=m, color=c, markeredgecolor=S, ms=7, label=n) for n, _, c, m in groups]
    hs += [Line2D([], [], ls="", marker="o", color=INK2, markeredgecolor=S, ms=7, label="map (56 points)"),
           Line2D([], [], ls="", marker="o", markerfacecolor=S, markeredgecolor=INK2, markeredgewidth=1.4, ms=7, label="hold-out (48 points, never anchors)")]
    ax[0].legend(handles=hs, fontsize=8.5, frameon=False, loc="upper center")
    f.suptitle("Simulink steady state, 25 ns grid, HV→LV: no saving within about ±4 % of k = 1, and controller B can sit above SPS there", fontsize=11)
    f.tight_layout()
    f.savefig(os.path.join(OUT, "fig_md_k.png"), dpi=130)
    plt.close(f)
    print("wrote out/md_k.{log,csv}, out/fig_md_k.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["selftest", "anchors", "smooth", "predict", "make", "next", "compare", "figk"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"anchors": anchors, "smooth": smooth, "predict": predict, "make": make, "next": next_round, "compare": compare, "figk": figk}[a.cmd]()


if __name__ == "__main__":
    main()
