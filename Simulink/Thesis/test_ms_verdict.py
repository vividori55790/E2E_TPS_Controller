"""
test_ms_verdict.py -- ms_verdict.py's rules on made-up inputs (no files read or written).
  per point: tolerance from SPS (2.5 A -> 0.05: gain 0.04 is 'same', 0.06 'better'), from M-B when SPS oscillates;
             M-B oscillating + M-S clean -> fixes_osc; M-S oscillating -> new_osc; both oscillating -> undecidable;
             anchor / spill split at w = 0.5; SPS violation flags
  surface:   clear anchor gains + harmless spill -> adopt; mean anchor gain 0.02 A -> no_gain; one big gain but more worse
             than better -> no_gain; spill mean -0.05 A -> partial; M-S adds an SPS violation -> partial; 2 decidable
             anchor points -> undecidable; fixes_osc counts for M-S; < 3 spill gains -> B not tested (holds)

    python -I -X utf8 Simulink/Thesis/test_ms_verdict.py
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ms_verdict as mv  # noqa: E402

FAIL = []


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        FAIL.append(name)


def pt(P, w, ms, mb, sps=2.0, o_ms=0.0, o_mb=0.0, o_sps=0.0, V2=84.0):
    return dict(V2=V2, P=P, w=w, group="t", Ipk_MS=ms, osc_MS=o_ms, Ipk_MB=mb, osc_MB=o_mb, Ipk_SPS=sps, osc_SPS=o_sps)


def per_point():
    r = mv.classify(pd.Series(pt(100, 1.0, 1.96, 2.00, sps=2.5)))
    check("tol from SPS 2.5 A = 0.05, gain 0.04 -> same", abs(r["tol"] - 0.05) < 1e-12 and r["cls"] == "same")
    r = mv.classify(pd.Series(pt(100, 1.0, 1.94, 2.00, sps=2.5)))
    check("gain 0.06 > 0.05 -> better", r["cls"] == "better" and abs(r["gain"] - 0.06) < 1e-12)
    r = mv.classify(pd.Series(pt(100, 1.0, 2.06, 2.00, sps=2.5)))
    check("gain -0.06 -> worse", r["cls"] == "worse")
    r = mv.classify(pd.Series(pt(100, 1.0, 1.90, 3.00, sps=1.0, o_sps=0.05)))
    check("SPS oscillating -> tol from M-B (3.0 A -> 0.06), SPS check None",
          abs(r["tol"] - 0.06) < 1e-12 and r["viol_MS"] is None and r["viol_MB"] is None)
    r = mv.classify(pd.Series(pt(100, 1.0, 1.50, 2.00, o_mb=0.1)))
    check("M-B oscillating, M-S clean -> fixes_osc, no gain value", r["cls"] == "fixes_osc" and np.isnan(r["gain"]))
    r = mv.classify(pd.Series(pt(100, 1.0, 1.50, 2.00, o_ms=0.1)))
    check("M-S oscillating, M-B clean -> new_osc", r["cls"] == "new_osc")
    r = mv.classify(pd.Series(pt(100, 1.0, 1.50, 2.00, o_ms=0.1, o_mb=0.1)))
    check("both oscillating -> undecidable", r["cls"] == "undecidable")
    r = mv.classify(pd.Series(pt(100, 1.0, np.nan, 2.00, o_ms=np.nan)))
    check("M-S not bracketed -> undecidable", r["cls"] == "undecidable")
    r = mv.classify(pd.Series(pt(100, 0.49, 1.9, 2.0)))
    check("w 0.49 -> spill, w 0.5 -> anchor",
          r["region"] == "spill" and mv.classify(pd.Series(pt(100, 0.5, 1.9, 2.0)))["region"] == "anchor")
    r = mv.classify(pd.Series(pt(100, 1.0, 2.10, 1.98, sps=2.0)))
    check("SPS 2.0 tol 0.04: M-S 2.10 violates, M-B 1.98 not", r["viol_MS"] is True and r["viol_MB"] is False)


def surface():
    anchor = [pt(400, 0.9, 2.50, 2.61, sps=2.7), pt(800, 0.8, 3.76, 3.88, sps=4.0), pt(600, 1.0, 3.55, 3.95, sps=4.0)]
    spill = [pt(200, 0.1, 1.40, 1.41), pt(300, 0.2, 1.80, 1.78), pt(1000, 0.0, 4.33, 4.34, sps=4.6)]
    code, T, s = mv.judge(pd.DataFrame(anchor + spill))
    check("clear anchor gains + harmless spill -> adopt", code == "adopt" and s["A"] and s["B"] and s["C"]
          and s["B_tested"] and s["anchor"]["better"] == 3)

    small = [pt(400, 0.9, 2.59, 2.61), pt(800, 0.8, 3.86, 3.88), pt(600, 1.0, 3.93, 3.95, sps=4.0)]
    code, T, s = mv.judge(pd.DataFrame(small + spill))
    check("mean anchor gain 0.02 A -> no_gain", code == "no_gain" and not s["A"])

    lop = [pt(400, 0.9, 2.10, 2.61), pt(800, 0.8, 3.95, 3.88), pt(600, 1.0, 4.03, 3.95, sps=4.0)]
    code, T, s = mv.judge(pd.DataFrame(lop + spill))
    check("one big gain, two worse (mean +0.11) -> no_gain", code == "no_gain" and s["anchor"]["mean_gain"] > 0.03)

    harm = [pt(200, 0.1, 1.46, 1.41), pt(300, 0.2, 1.84, 1.78), pt(1000, 0.0, 4.38, 4.34, sps=4.6)]
    code, T, s = mv.judge(pd.DataFrame(anchor + harm))
    check("spill mean -0.05 A -> partial", code == "partial" and s["A"] and not s["B"])

    viol = anchor + spill + [pt(150, 0.2, 2.06, 1.99, sps=2.0)]
    code, T, s = mv.judge(pd.DataFrame(viol))
    check("M-S adds an SPS violation -> partial (C)", code == "partial" and not s["C"] and s["viol_MS"] == 1
          and s["viol_MB"] == 0)

    code, T, s = mv.judge(pd.DataFrame(anchor[:2] + spill))
    check("2 decidable anchor points -> undecidable", code == "undecidable")

    osc = [pt(400, 0.9, 2.50, 2.61, sps=2.7), pt(800, 0.8, 3.76, 3.88, sps=4.0), pt(200, 0.7, 2.04, 2.45, o_mb=0.13)]
    code, T, s = mv.judge(pd.DataFrame(osc + spill))
    check("fixes_osc counts for M-S (2 better + 1 fixes_osc) -> adopt", code == "adopt"
          and s["anchor"]["fixes_osc"] == 1 and s["anchor"]["n_gain"] == 2)

    code, T, s = mv.judge(pd.DataFrame(anchor + spill[:2]))
    check("< 3 spill gains -> B holds, not tested", code == "adopt" and s["B"] and not s["B_tested"])


if __name__ == "__main__":
    per_point()
    surface()
    print("ALL PASS" if not FAIL else f"{len(FAIL)} FAILED: {FAIL}")
    sys.exit(1 if FAIL else 0)
