"""
surface_opt_verdict.py -- (OPT0, 2026-10-29 21:00) the rules that decide "is the proposed surface M-B really the optimal
surface?" in Simulink. Registered BEFORE any OPT result exists (OPT2/OPT3 had not been submitted; the R6 grid had 18 of
363 combinations). The user's task text (automation/TASKS.md, 10-29 19:10, OPT0) gave the three rules; this file only
pins down the details the text leaves open. Do not change them after the results come in.
test_surface_opt_verdict.py checks every branch on made-up inputs.

What counts as a Simulink candidate value
  Only steady-state values: constant D0 for 30 ms (40 ms in OPT2), last 1 ms, two adjacent 2.88-deg D0 levels whose
  measured delivered powers bracket |P|, Ipk interpolated linearly in that power (simulink_sweep.py, = PI steady state).
  1 ms staircase values (R6 grid) only choose which combinations get such runs; they never enter the rules.
  A candidate whose bracketing runs oscillate (max |mean i_L| over the last 1 ms > OSC = 20 mA; with series caps a
  settled current has zero mean, R.4's 'clean') is left out of the minimum. If M-B's or SPS's own value oscillates
  or is missing, the point is "판정 불가" (undecidable) -- never a pass.
  Settling (|Ipk at the end - Ipk at 25 ms| > 0.02 A) is reported but does not change a verdict.

(i)  per point (V2, |P|), HV -> LV: gap = Ipk(M-B) - min over every clean candidate Ipk (SPS and M-B included).
     tol = max(TOL_ABS 0.03 A, TOL_REL 2 % of Ipk(SPS)).  gap <= tol: "M-B가 Simulink 최적" at that point, else not.
     Also counted: candidates within tol of the minimum (how flat the optimum is).
     Surface verdict: optimal if every decidable point passes; if any decidable point fails, M-B is not the Simulink
     optimum (the fix in OPT4: retrain on the Simulink optimum = R13 brought forward).
(ii) "the deep k = 1 valley is real": at V2 = 95.0 V (OPT2 runs only), |P| in {100, 300, 600} W, some clean candidate
     with min(D1, D2) < 0.45 has Ipk(SPS) - Ipk(candidate) > VALLEY_GAIN 0.05 A at the same power.  Otherwise not real.
     If SPS itself is missing or oscillating at all three powers: undecidable.
(iii) shape: the Simulink grid optimum (D1*, D2*) per (V2, |P|), V2 in {84, 109.2}, SPS allowed as the optimum:
     side  -- M-B's side is s = +1 at 84 V (D1 < D2) and -1 at 109.2 V (D2 < D1). A point with a TPS optimum is on the
              same side if s (D2* - D1*) >= 0 (equal duties are not the opposite side). Pass if >= SIDE_FRAC 75 % of the
              TPS-optimum points are on the same side; no TPS-optimum point at all (SPS best everywhere) fails.
     rise  -- d* = min(D1*, D2*) (0.5 for SPS) against |P|: Spearman rho >= RISE_RHO 0.8 and d* at the largest |P|
              >= RISE_END 0.45.  Needs >= 3 points (else undecidable).
     The shape matches if side and rise pass at both V2.

    import surface_opt_verdict as sv
    sv.judge_points(cands) / sv.valley(cands) / sv.shape(grid_opt)
"""
import numpy as np
import pandas as pd

TOL_ABS, TOL_REL = 0.03, 0.02          # (i)
OSC = 0.02                             # |mean i_L| [A] above which a run counts as oscillating
SETTLE = 0.02                          # [A], reported only
VALLEY_V2, VALLEY_D, VALLEY_GAIN = 95.0, 0.45, 0.05
VALLEY_P = (100.0, 300.0, 600.0)
SIDE = {84.0: +1, 109.2: -1}
SIDE_FRAC, RISE_RHO, RISE_END = 0.75, 0.8, 0.45

# cands: one row per (V2, P, candidate) with
#   V2, P, ctrl ('SPS', 'M-B' or any other name), D1, D2, Ipk (steady, same power; NaN if not bracketed),
#   osc (max |mean i_L| of the two bracketing runs [A]), settle ([A], may be NaN), src (where the runs come from)


def _clean(c):
    return np.isfinite(c["Ipk"]) & (c["osc"].fillna(np.inf) <= OSC)


def judge_points(cands):
    """rule (i) per (V2, P) -> DataFrame, plus the surface verdict code 'optimal' / 'not_optimal' / 'undecidable'."""
    rows = []
    for (v, p), g in cands.groupby(["V2", "P"], sort=True):
        ok = _clean(g)
        mb, sps = g[g["ctrl"] == "M-B"], g[g["ctrl"] == "SPS"]
        rec = dict(V2=v, P=p, n=len(g), n_clean=int(ok.sum()))
        if mb.empty or sps.empty or not bool(_clean(mb).any()) or not bool(_clean(sps).any()):
            why = "M-B 없음" if mb.empty else "SPS 없음" if sps.empty else \
                  "M-B 진동/미포착" if not bool(_clean(mb).any()) else "SPS 진동/미포착"
            rec.update(verdict="undecidable", why=why)
            rows.append(rec)
            continue
        i_mb = float(mb.loc[_clean(mb), "Ipk"].min())
        i_sps = float(sps.loc[_clean(sps), "Ipk"].min())
        gc = g[ok]
        b = gc.loc[gc["Ipk"].idxmin()]
        tol = max(TOL_ABS, TOL_REL * i_sps)
        gap = i_mb - float(b["Ipk"])
        within = gc[gc["Ipk"] - float(b["Ipk"]) <= tol + 1e-12]
        rec.update(Ipk_MB=i_mb, Ipk_SPS=i_sps, Ipk_min=float(b["Ipk"]), best_ctrl=b["ctrl"], best_D1=float(b["D1"]),
                   best_D2=float(b["D2"]), best_src=b.get("src", ""), gap=gap, tol=tol, n_within=len(within),
                   within_D1=(float(within["D1"].min()), float(within["D1"].max())),
                   within_D2=(float(within["D2"].min()), float(within["D2"].max())),
                   unsettled=int((g["settle"].fillna(0) > SETTLE).sum()) if "settle" in g else 0,
                   verdict="optimal" if gap <= tol + 1e-12 else "not_optimal", why="")
        rows.append(rec)
    T = pd.DataFrame(rows)
    dec = T[T["verdict"] != "undecidable"] if len(T) else T
    code = "undecidable" if dec.empty else ("optimal" if (dec["verdict"] == "optimal").all() else "not_optimal")
    return T, code


def valley(cands):
    """rule (ii) -> (code 'real' / 'not_real' / 'undecidable', DataFrame of the low-duty candidates and their gain)."""
    c = cands[np.isclose(cands["V2"], VALLEY_V2) & cands["P"].isin(VALLEY_P)]
    rows, any_sps = [], False
    for p, g in c.groupby("P", sort=True):
        sps = g[(g["ctrl"] == "SPS") & _clean(g)]
        if sps.empty:
            continue
        any_sps = True
        i_sps = float(sps["Ipk"].min())
        low = g[(np.minimum(g["D1"], g["D2"]) < VALLEY_D) & _clean(g)]
        for _, r in low.iterrows():
            rows.append(dict(P=p, D1=r["D1"], D2=r["D2"], Ipk=r["Ipk"], Ipk_SPS=i_sps, gain=i_sps - r["Ipk"]))
    T = pd.DataFrame(rows, columns=["P", "D1", "D2", "Ipk", "Ipk_SPS", "gain"])
    if not any_sps:
        return "undecidable", T
    return ("real" if (T["gain"] > VALLEY_GAIN).any() else "not_real"), T


def _spearman(x, y):
    rx, ry = pd.Series(x).rank().to_numpy(), pd.Series(y).rank().to_numpy()
    if np.std(rx) == 0 or np.std(ry) == 0:
        return np.nan
    return float(np.corrcoef(rx, ry)[0, 1])


def shape(grid_opt):
    """rule (iii). grid_opt: V2, P, D1, D2 of the Simulink grid optimum (SPS = 0.5, 0.5). -> (code 'match' /
    'mismatch' / 'undecidable', {V2: dict(side_frac, n_tps, side_ok, rho, d_end, rise_ok)})."""
    out = {}
    for v, s in SIDE.items():
        g = grid_opt[np.isclose(grid_opt["V2"], v)].sort_values("P")
        if len(g) < 3:
            out[v] = dict(n=len(g), side_frac=np.nan, n_tps=0, side_ok=None, rho=np.nan, d_end=np.nan, rise_ok=None)
            continue
        tps = g[np.minimum(g["D1"], g["D2"]) < 0.4999]
        side = (s * (tps["D2"] - tps["D1"]) >= 0) if len(tps) else pd.Series([], dtype=bool)
        frac = float(side.mean()) if len(tps) else np.nan
        d = np.minimum(g["D1"], g["D2"]).to_numpy()
        rho = _spearman(g["P"].to_numpy(), d)
        d_end = float(d[-1])
        rise_ok = bool(np.isfinite(rho) and rho >= RISE_RHO and d_end >= RISE_END)
        # SPS optimal at every |P| (no TPS point) is not M-B's shape: M-B is TPS at light load at both V2
        out[v] = dict(n=len(g), side_frac=frac, n_tps=len(tps),
                      side_ok=bool(len(tps) > 0 and frac >= SIDE_FRAC), rho=rho, d_end=d_end, rise_ok=rise_ok)
    if any(o["side_ok"] is None for o in out.values()):
        return "undecidable", out
    return ("match" if all(o["side_ok"] and o["rise_ok"] for o in out.values()) else "mismatch"), out
