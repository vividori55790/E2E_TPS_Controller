"""
ms_verdict.py -- (OPT4, 2026-10-30 08:15) the rules that decide "does the corrected surface M-S (train_corrected.py --tag
MS --delta, anchors out/opt4_anchors_r3.csv) do better than M-B in Simulink?". Registered BEFORE any M-S Simulink run
exists (the M-S points out/opt_points_ms_r1.csv / msk1_r1.csv were written 08:02-08:05 and wait for job opt5, which had
not started). Do not change them after the results come in. test_ms_verdict.py checks every branch on made-up inputs.

Values: the same Simulink steady state as OPT1-3 (surface_opt_verdict.py): constant D0, 30 ms (msk1: 40 ms), last 1 ms
(end window), two adjacent 2.88-deg D0 levels bracketing |P| in measured power, Ipk interpolated; HV -> LV. A value whose
bracketing runs oscillate (max |mean i_L| > OSC 20 mA) is not clean. Only points where M-S differs from M-B by more than
0.003 in a duty were run (elsewhere M-S = M-B by construction).

Per point (V2, |P|):
  tol  = max(0.03 A, 2 % of Ipk(SPS)), as rule (i); if SPS is missing or not clean, 2 % of Ipk(M-B).
  gain = Ipk(M-B) - Ipk(M-S)  (> 0: M-S lower).
  class  better    M-S and M-B clean, gain > tol
         worse     M-S and M-B clean, gain < -tol
         same      M-S and M-B clean, |gain| <= tol
         fixes_osc M-S clean, M-B oscillating   (counted with 'better', no gain value)
         new_osc   M-S oscillating, M-B clean   (counted with 'worse', no gain value)
         undecidable  either missing, or both oscillating
  SPS check (goal 3): where SPS and the controller are clean, the controller "violates" if Ipk - Ipk(SPS) > tol.
  region anchor = the M-S target moved at least half-way to an anchor there (anchor weight w >= W_ANCHOR 0.5),
         spill  = the rest (w < 0.5: what the correction does where it was not asked to).
Surface verdict:
  A (anchor region gains): >= N_MIN 3 decidable anchor points (else 'undecidable'), mean gain over the anchor points
    with a gain value > 0.03 A, and (better + fixes_osc) > (worse + new_osc) there.
  B (spill does no harm): mean gain over the spill points with a gain value >= -0.03 A (fewer than 3 such points: B holds
    and is reported as 'not tested').
  C (goal 3 not made worse): over the points where SPS, M-S and M-B are all clean, #M-S violations <= #M-B violations.
  'adopt' (M-S replaces M-B) if A, B and C; 'partial' (gains at the anchors, harm elsewhere) if A only; 'no_gain' if A
  fails; 'undecidable' if A cannot be evaluated.
Reported, never a verdict: the breakdown by anchor group (knee = 84/109.2 V >= 400 W, ridge = 109.2 V <= 200 W, k1 = the
cap anchors), M-S at the anchor points vs the measured candidate the anchor came from, the 1 ms window sensitivity.

    import ms_verdict as mv
    mv.judge(points) -> (code, per-point DataFrame, summary dict)
"""
import numpy as np
import pandas as pd

TOL_ABS, TOL_REL = 0.03, 0.02
OSC = 0.02
W_ANCHOR = 0.5
N_MIN = 3
MEAN_GAIN = 0.03                         # A: mean anchor-region gain must exceed this
SPILL_HARM = 0.03                        # B: mean spill-region gain must be >= -this

# points: one row per (V2, P) with
#   V2, P, w (anchor weight of M-S's target there), group (str, report only),
#   Ipk_MS, osc_MS, Ipk_MB, osc_MB, Ipk_SPS, osc_SPS   (Ipk NaN = not bracketed; osc NaN = not bracketed)


def _clean(i, o):
    return bool(np.isfinite(i) and np.isfinite(o) and o <= OSC)


def classify(r):
    ms, mb, sps = (_clean(r[f"Ipk_{k}"], r[f"osc_{k}"]) for k in ("MS", "MB", "SPS"))
    ref = r["Ipk_SPS"] if sps else (r["Ipk_MB"] if np.isfinite(r["Ipk_MB"]) else np.nan)
    tol = max(TOL_ABS, TOL_REL * ref) if np.isfinite(ref) else TOL_ABS
    gain = r["Ipk_MB"] - r["Ipk_MS"] if (ms and mb) else np.nan
    has_ms, has_mb = np.isfinite(r["Ipk_MS"]), np.isfinite(r["Ipk_MB"])
    if ms and mb:
        cls = "better" if gain > tol else "worse" if gain < -tol else "same"
    elif ms and has_mb and not mb:
        cls = "fixes_osc"
    elif mb and has_ms and not ms:
        cls = "new_osc"
    else:
        cls = "undecidable"
    v_ms = (r["Ipk_MS"] - r["Ipk_SPS"] > tol) if (sps and ms) else None
    v_mb = (r["Ipk_MB"] - r["Ipk_SPS"] > tol) if (sps and mb) else None
    return dict(tol=tol, gain=gain, cls=cls, viol_MS=v_ms, viol_MB=v_mb,
                region="anchor" if r["w"] >= W_ANCHOR else "spill")


def judge(points):
    """-> (code 'adopt' / 'partial' / 'no_gain' / 'undecidable', per-point DataFrame, summary dict)"""
    P = points.copy().reset_index(drop=True)
    if len(P):
        P = pd.concat([P, pd.DataFrame([classify(r) for _, r in P.iterrows()])], axis=1)
    else:
        P = pd.DataFrame(columns=list(points.columns) + ["tol", "gain", "cls", "viol_MS", "viol_MB", "region"])
    s = {}
    for reg in ("anchor", "spill"):
        g = P[P["region"] == reg]
        dec = g[g["cls"] != "undecidable"]
        gv = g["gain"].dropna()
        s[reg] = dict(n=len(g), n_dec=len(dec), n_gain=len(gv), mean_gain=float(gv.mean()) if len(gv) else np.nan,
                      better=int((g["cls"] == "better").sum()), fixes_osc=int((g["cls"] == "fixes_osc").sum()),
                      same=int((g["cls"] == "same").sum()), worse=int((g["cls"] == "worse").sum()),
                      new_osc=int((g["cls"] == "new_osc").sum()))
    a = s["anchor"]
    if a["n_dec"] < N_MIN:
        s.update(A=None, B=None, C=None)
        return "undecidable", P, s
    A = bool(a["n_gain"] > 0 and a["mean_gain"] > MEAN_GAIN
             and a["better"] + a["fixes_osc"] > a["worse"] + a["new_osc"])
    sp = s["spill"]
    B = True if sp["n_gain"] < N_MIN else bool(sp["mean_gain"] >= -SPILL_HARM)
    s["B_tested"] = sp["n_gain"] >= N_MIN
    both = P[P["viol_MS"].notna() & P["viol_MB"].notna()]
    n_ms, n_mb = int(both["viol_MS"].astype(bool).sum()), int(both["viol_MB"].astype(bool).sum())
    C = bool(n_ms <= n_mb)
    s.update(A=A, B=B, C=C, viol_MS=n_ms, viol_MB=n_mb, n_viol_checked=len(both))
    code = ("adopt" if (B and C) else "partial") if A else "no_gain"
    return code, P, s
