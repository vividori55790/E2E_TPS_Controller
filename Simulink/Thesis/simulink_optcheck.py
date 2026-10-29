"""
simulink_optcheck.py -- is the proposed surface (M-B, Smooth RegHDC b3) really the optimum? Checked in SIMULINK only.

At each operating point the peak current of M-B's (D1, D2) is compared, at the SAME delivered power, with every other
candidate (D1, D2) run in the dataset-era Simulink model (verify_steady_state.m, constant D0 for 30 ms, the two adjacent
2.88 deg grid levels that bracket |P*|, Ipk interpolated in the measured power = the dither-free PI steady state; the
same method as R.4 / simulink_sweep.py, whose helpers are reused).

Points: V2 {84, 95, 109.2} V x |P| {100, 300, 600, 1000} W, HV -> LV.
Candidates per point:
  M-B          the proposed controller's (D1, D2)
  SPS          D1 = D2 = 0.5 (D_SAFE)
  local        M-B +- 1 step in D1 and D2, step 0.024 (3 PWM grid steps of 0.008), 3 x 3 incl. M-B
  global       the 3 best (D1, D2) of a 0.05 lattice by the Python plant's prediction (REFERENCE ONLY: used to choose
               what to run, never as a result)
  probe        D1 = D2 = 0.30 and 0.40 (the "deep centre" question at k = 1; a shallow/deep check elsewhere)
Verdict rules, fixed BEFORE any result (2026-10-29 19:10):
  best = lowest Simulink Ipk among the bracketed candidates of the point (only 'clean' runs: |mean i_L| <= 20 mA)
  M-B is OPTIMAL at the point if Ipk(M-B) - Ipk(best) <= tol, tol = max(0.020 A, 0.02 * Ipk(SPS))
  captured share = (Ipk_SPS - Ipk_MB) / (Ipk_SPS - Ipk_best) when Ipk_SPS - Ipk_best > tol (else "SPS-optimal point")
  centre question: at V2 = 95 V the probes must not beat SPS by more than tol for "no centre dip".
Limits: a local 3 x 3 + a few global picks is not an exhaustive search; the result says "no better candidate among those
run", and the log lists every candidate with its Simulink Ipk.

    python -I -X utf8 Simulink/Thesis/simulink_optcheck.py make       -> out/optcheck_points_r1.csv
    python automation/job.py submit optcheck1 --interactive -- matlab -batch "cd('Simulink/Thesis'); verify_steady_state('out/optcheck_points_r1.csv', 'out/optcheck_steady_r1.csv')"
    python -I -X utf8 Simulink/Thesis/simulink_optcheck.py next       (levels further out where |P*| is not bracketed)
    python -I -X utf8 Simulink/Thesis/simulink_optcheck.py verdict    -> out/optcheck.{csv,log}, out/fig_optcheck.png
"""
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import plant_full as pf  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
V2S = (84.0, 95.0, 109.2)
PS = (100.0, 300.0, 600.0, 1000.0)
STEP = 0.024
DMIN = 0.10


def _clip(x):
    return round(min(max(float(x), DMIN), sw.D_SAFE), 9)


def configs():
    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=False)
    lat = np.round(np.arange(0.10, 0.5001, 0.05), 4)
    L1, L2 = [a.ravel() for a in np.meshgrid(lat, lat, indexing="ij")]
    L1, L2 = np.minimum(L1, sw.D_SAFE), np.minimum(L2, sw.D_SAFE)
    rows = []
    for V2 in V2S:
        for P in PS:
            d1, d2 = mb.predict(np.array([P]), np.array([V2]))
            c1, c2 = _clip(np.ravel(d1)[0]), _clip(np.ravel(d2)[0])
            cand = [("M-B", c1, c2), ("SPS", sw.D_SAFE, sw.D_SAFE)]
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    if i or j:
                        cand.append((f"loc{i:+d}{j:+d}", _clip(c1 + i * STEP), _clip(c2 + j * STEP)))
            r = pl.operating_point(np.full(L1.size, P), L1, L2, np.full(L1.size, V2))     # reference: picks only
            ipk = np.where(r["ok"], r["Ipk"], np.inf)
            for n, k in enumerate(np.argsort(ipk)[:3]):
                cand.append((f"glob{n + 1}", _clip(L1[k]), _clip(L2[k])))
            cand += [("probe0.30", 0.30, 0.30), ("probe0.40", 0.40, 0.40)]
            seen = set()
            for tag, a, b in cand:
                if (a, b) in seen:
                    continue
                seen.add((a, b))
                rows.append(dict(ctrl=tag, V2=V2, P=P, D1=a, D2=b))
    return rows


def _point(c, j):
    p = sw.point(c, j)
    p["label"] = f"OPT {c['ctrl']} {c['P']:.0f}W {c['V2']:g}V {c['D1']:.3f}/{c['D2']:.3f} L{j}"
    return p


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "optcheck_steady_r*.csv"))) if not f.endswith("_in.csv")]


def results():
    import pandas as pd
    fs = result_files()
    if not fs:
        sys.exit("no out/optcheck_steady_r*.csv yet")
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    d["Pd"] = -d["P_LV"]
    d["level"] = np.round((d["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    return d.drop_duplicates(subset=["V2", "D0", "D1", "D2"], keep="last")


def _next_no():
    ns = [int(m.group(1)) for f in result_files() for m in [re.search(r"optcheck_steady_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def make():
    cf = configs()
    pts = []
    for c, (j0, j1) in zip(cf, sw.predicted_levels(cf)):
        pts += [_point(c, j0), _point(c, j1)]
    sw.write(os.path.join(OUT, "optcheck_points_r1.csv"), pts)
    print(f"{len(cf)} candidate configurations at {len(V2S) * len(PS)} points")


def next_round():
    d = results()
    cf = configs()
    pts = []
    fresh = [c for c in cf if sw.runs_of(d, c).empty]
    for c, (j0, j1) in zip(fresh, sw.predicted_levels(fresh) if fresh else []):
        pts += [_point(c, j0), _point(c, j1)]
    for c in cf:
        g = sw.runs_of(d, c)
        if g.empty:
            continue
        b, way = sw.bracket(g, c["P"])
        if b is None:
            j = int(g["level"].max() + 1) if way > 0 else int(g["level"].min() - 1)
            pts += [_point(c, j), _point(c, j + way)]
    done = list(zip(d["V2"], d["D0"], d["D1"], d["D2"]))
    n = _next_no()
    if pts:
        sw.write(os.path.join(OUT, f"optcheck_points_r{n}.csv"), pts, done)
    else:
        print("every candidate is bracketed")


def verdict():
    import pandas as pd
    d = results()
    rows = []
    for c in configs():
        g = sw.runs_of(d, c)
        b, _ = sw.bracket(g, c["P"]) if not g.empty else (None, 0)
        rec = dict(c)
        if b is not None:
            lo, hi = b
            t = (c["P"] - lo["Pd"]) / max(hi["Pd"] - lo["Pd"], 1e-9)
            rec.update(Ipk=lo["Ipk_sim"] + t * (hi["Ipk_sim"] - lo["Ipk_sim"]),
                       iL_mean_max=max(abs(lo["iL_mean"]), abs(hi["iL_mean"])))
        else:
            rec.update(Ipk=np.nan, iL_mean_max=np.nan)
        rows.append(rec)
    T = pd.DataFrame(rows)
    T["clean"] = T["iL_mean_max"] <= 0.02
    T.to_csv(os.path.join(OUT, "optcheck.csv"), index=False, float_format="%.5g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("Simulink optimality check of the proposed surface (M-B). Same delivered power, HV -> LV. "
        "Rules fixed before the runs (see the module docstring).")
    summ = []
    for (v, p), g in T.groupby(["V2", "P"], sort=False):
        ok = g[g["clean"] & g["Ipk"].notna()]
        mb = g[g["ctrl"] == "M-B"].iloc[0]
        sps = g[g["ctrl"] == "SPS"].iloc[0]
        out(f"\nV2 {v:5.1f} V  |P| {p:5.0f} W  ({len(ok)} of {len(g)} candidates bracketed and clean)")
        for _, r in g.sort_values("Ipk").iterrows():
            flag = "" if r["clean"] else "  [oscillating / not bracketed - excluded]"
            out(f"   {r['ctrl']:10s} D1 {r['D1']:.3f} D2 {r['D2']:.3f}  Ipk {r['Ipk']:.3f} A{flag}")
        if ok.empty or not (np.isfinite(mb["Ipk"]) and mb["clean"]) or not (np.isfinite(sps["Ipk"]) and sps["clean"]):
            out("   verdict: NOT DECIDED (M-B or SPS not bracketed / oscillating)")
            summ.append(dict(V2=v, P=p, verdict="undecided"))
            continue
        best = ok.loc[ok["Ipk"].idxmin()]
        tol = max(0.020, 0.02 * sps["Ipk"])
        gap = mb["Ipk"] - best["Ipk"]
        room = sps["Ipk"] - best["Ipk"]
        share = (sps["Ipk"] - mb["Ipk"]) / room if room > tol else np.nan
        vd = "OPTIMAL" if gap <= tol else "NOT optimal"
        extra = ""
        if v == 95.0:
            pr = ok[ok["ctrl"].str.startswith("probe")]
            beat = (sps["Ipk"] - pr["Ipk"]).max() if not pr.empty else np.nan
            extra = f"; centre probes beat SPS by at most {beat:+.3f} A -> " + ("NO centre dip" if not (beat > tol) else "CENTRE DIP")
        out(f"   verdict: {vd}  (M-B {mb['Ipk']:.3f}, best {best['ctrl']} {best['Ipk']:.3f}, gap {gap:+.3f} A, tol {tol:.3f}; "
            f"SPS {sps['Ipk']:.3f}" + (f", M-B captures {100 * share:.0f} % of the best reduction" if np.isfinite(share)
                                      else ", SPS-optimal point") + extra + ")")
        summ.append(dict(V2=v, P=p, verdict=vd, gap=gap, tol=tol, share=share))
    S = pd.DataFrame(summ)
    out(f"\nsummary: {int((S['verdict'] == 'OPTIMAL').sum())} OPTIMAL, {int((S['verdict'] == 'NOT optimal').sum())} NOT optimal, "
        f"{int((S['verdict'] == 'undecided').sum())} undecided of {len(S)} points")
    open(os.path.join(OUT, "optcheck.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    {"make": make, "next": next_round, "verdict": verdict}.get(cmd, lambda: print(__doc__))()
