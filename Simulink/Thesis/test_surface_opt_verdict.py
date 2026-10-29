"""
test_surface_opt_verdict.py -- surface_opt_verdict.py's rules on made-up inputs (no files read or written).
  (i)   M-B is the minimum -> optimal; M-B 0.10 A above a candidate (SPS 2 A, tol 0.04) -> not optimal; the tolerance
        edge (SPS 1.0 A -> tol 0.03: gap 0.035 fails; SPS 2.5 A -> tol 0.05: gap 0.035 passes); an oscillating
        candidate below M-B is ignored; oscillating M-B -> undecidable; one failing point makes the surface not optimal
  (ii)  a 0.40/0.40 candidate 0.06 A below SPS -> real; 0.04 A -> not real; 0.45/0.45 even 0.2 A below -> not counted;
        oscillating low-duty candidate -> not counted; SPS missing -> undecidable
  (iii) M-B-like grid optimum (84 V D1 < D2, 109.2 V D2 < D1, rising to SPS) -> match; 84 V on the wrong side ->
        mismatch; falling with |P| -> mismatch; SPS everywhere -> mismatch; 2 points -> undecidable

    python -I -X utf8 Simulink/Thesis/test_surface_opt_verdict.py
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import surface_opt_verdict as sv  # noqa: E402

FAIL = []


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        FAIL.append(name)


def c(V2, P, ctrl, D1, D2, Ipk, osc=0.0, settle=0.0):
    return dict(V2=V2, P=P, ctrl=ctrl, D1=D1, D2=D2, Ipk=Ipk, osc=osc, settle=settle, src="test")


def rule_i():
    base = [c(84.0, 100.0, "SPS", 0.5, 0.5, 2.0), c(84.0, 100.0, "M-B", 0.21, 0.24, 1.30),
            c(84.0, 100.0, "x1", 0.23, 0.24, 1.32), c(84.0, 100.0, "x2", 0.19, 0.24, 1.45)]
    T, code = sv.judge_points(pd.DataFrame(base))
    check("(i) M-B minimum -> optimal", code == "optimal" and T["gap"].iloc[0] == 0 and T["n_within"].iloc[0] == 2)
    T, code = sv.judge_points(pd.DataFrame(base + [c(84.0, 100.0, "bt", 0.25, 0.20, 1.20)]))
    check("(i) M-B 0.10 A above (tol 0.04) -> not optimal",
          code == "not_optimal" and abs(T["gap"].iloc[0] - 0.10) < 1e-12 and abs(T["tol"].iloc[0] - 0.04) < 1e-12)
    edge = lambda s: [c(109.2, 300.0, "SPS", 0.5, 0.5, s), c(109.2, 300.0, "M-B", 0.35, 0.30, 0.900),  # noqa: E731
                      c(109.2, 300.0, "y", 0.37, 0.30, 0.865)]
    check("(i) SPS 1.0 A, gap 0.035 > tol 0.03 -> not optimal", sv.judge_points(pd.DataFrame(edge(1.0)))[1] == "not_optimal")
    check("(i) SPS 2.5 A, gap 0.035 <= tol 0.05 -> optimal", sv.judge_points(pd.DataFrame(edge(2.5)))[1] == "optimal")
    T, code = sv.judge_points(pd.DataFrame(base + [c(84.0, 100.0, "osc", 0.25, 0.20, 1.00, osc=0.05)]))
    check("(i) oscillating candidate ignored", code == "optimal" and T["n_clean"].iloc[0] == 4)
    bad = [dict(r) for r in base]
    bad[1]["osc"] = 0.03
    T, code = sv.judge_points(pd.DataFrame(bad))
    check("(i) oscillating M-B -> undecidable", code == "undecidable" and T["verdict"].iloc[0] == "undecidable")
    two = base + [c(109.2, 100.0, "SPS", 0.5, 0.5, 2.3), c(109.2, 100.0, "M-B", 0.20, 0.17, 1.76),
                  c(109.2, 100.0, "bt", 0.247, 0.20, 1.57)]
    T, code = sv.judge_points(pd.DataFrame(two))
    check("(i) one failing point -> surface not optimal",
          code == "not_optimal" and list(T["verdict"]) == ["optimal", "not_optimal"])
    nan = base + [c(109.2, 300.0, "SPS", 0.5, 0.5, np.nan), c(109.2, 300.0, "M-B", 0.35, 0.30, 0.9)]
    T, code = sv.judge_points(pd.DataFrame(nan))
    check("(i) SPS not bracketed -> that point undecidable, the rest decides", code == "optimal"
          and T.set_index("V2").loc[109.2, "verdict"] == "undecidable")


def rule_ii():
    def at95(gain40, gain45=0.0, osc40=0.0, sps=True):
        r = []
        for p, s in ((100.0, 0.75), (300.0, 1.0), (600.0, 1.95)):
            if sps:
                r.append(c(95.0, p, "SPS", 0.5, 0.5, s))
            r.append(c(95.0, p, "d40", 0.40, 0.40, s - (gain40 if p == 300.0 else -0.1), osc=osc40))
            r.append(c(95.0, p, "d45", 0.45, 0.45, s - gain45))
            r.append(c(95.0, p, "a40", 0.50, 0.40, s + 0.05))
        r.append(c(84.0, 300.0, "far", 0.30, 0.30, 0.1))           # another V2 never counts
        return pd.DataFrame(r)
    check("(ii) 0.40/0.40 0.06 A below SPS -> real", sv.valley(at95(0.06))[0] == "real")
    check("(ii) 0.04 A -> not real", sv.valley(at95(0.04))[0] == "not_real")
    check("(ii) 0.45/0.45 0.2 A below -> not counted", sv.valley(at95(0.0, gain45=0.2))[0] == "not_real")
    check("(ii) oscillating 0.40 candidate -> not counted", sv.valley(at95(0.3, osc40=0.05))[0] == "not_real")
    check("(ii) no SPS -> undecidable", sv.valley(at95(0.3, sps=False))[0] == "undecidable")


def rule_iii():
    P = [100.0, 300.0, 600.0, 1000.0]
    good = ([dict(V2=84.0, P=p, D1=d1, D2=d2) for p, d1, d2 in zip(P, (0.21, 0.36, 0.44, 0.5), (0.24, 0.41, 0.5, 0.5))]
            + [dict(V2=109.2, P=p, D1=d1, D2=d2) for p, d1, d2 in zip(P, (0.25, 0.35, 0.49, 0.5), (0.20, 0.30, 0.42, 0.5))])
    code, info = sv.shape(pd.DataFrame(good))
    check("(iii) M-B-like optimum -> match", code == "match" and info[84.0]["side_frac"] == 1.0)
    wrong = [dict(r) for r in good]
    for r in wrong[:3]:
        r["D1"], r["D2"] = r["D2"], r["D1"]
    check("(iii) 84 V on the wrong side -> mismatch", sv.shape(pd.DataFrame(wrong))[0] == "mismatch")
    fall = [dict(r) for r in good]
    for r, d in zip(fall[4:], (0.45, 0.35, 0.30, 0.25)):
        r["D1"], r["D2"] = d + 0.05, d
    check("(iii) falling with |P| -> mismatch", sv.shape(pd.DataFrame(fall))[0] == "mismatch")
    sps = [dict(V2=v, P=p, D1=0.5, D2=0.5) for v in (84.0, 109.2) for p in P]
    check("(iii) SPS everywhere -> mismatch", sv.shape(pd.DataFrame(sps))[0] == "mismatch")
    check("(iii) 2 points at 84 V -> undecidable", sv.shape(pd.DataFrame(good[2:]))[0] == "undecidable")
    eq = [dict(r) for r in good]
    eq[0]["D1"] = eq[0]["D2"] = 0.20                                # equal duties are not the opposite side
    check("(iii) equal duties count as the same side", sv.shape(pd.DataFrame(eq))[0] == "match")


if __name__ == "__main__":
    rule_i()
    rule_ii()
    rule_iii()
    print("ALL PASS" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}")
    sys.exit(1 if FAIL else 0)
