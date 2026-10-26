"""
plant_verdict.py -- T6 verdicts on the Simulink-faithful Python plant (plant_full.FullPlant: Ron, 100 ns dead time,
2.88 deg edge quantisation, body diode, series caps, source droop; validated against the dataset's steady rows and
by replaying the dataset generation, see validate_plant.py / transient_replay.py).

  A  k ~ 1 question (the user saw TPS gains there in the old data; the ideal model says ~0): at V2 = 93.6 / 95 / 96.4 V
     (k = 0.985 / 1 / 1.015) and, for contrast, 84 and 109.2 V, |P| in {50, 100, 200, 400, 700, 1000} W: best (D1, D2)
     on a grid vs SPS, both at the plant's PI steady state.
  B  goal 3 on the plant: SPS vs the proposed b3 vs the E2E LUTs (all trained on the ideal model), at random points,
     PI steady state on the plant; how many points are worse than SPS, and how the gain compares with the ideal
     model's prediction.
Ipk = continuous peak (1250 steps per period). Power direction HV -> LV (pi_steady's branch).

    python -I -X utf8 Simulink/Thesis/plant_verdict.py --part A
    python -I -X utf8 Simulink/Thesis/plant_verdict.py --part B --n 1200
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import plant_full as pf  # noqa: E402

OUT = os.path.join(HERE, "out")


def part_a(plant, out, chunk=900):
    V2s = [84.0, 93.6, 95.0, 96.4, 109.2]
    Ps = [50.0, 100.0, 200.0, 400.0, 700.0, 1000.0]
    dg = np.array([0.2, 0.25, 0.3, 0.35, 0.4, 0.425, 0.45, 0.475, 0.5])
    G1, G2 = np.meshgrid(dg, dg, indexing="ij")
    g1, g2 = G1.ravel(), G2.ravel()
    PP, VV = np.meshgrid(Ps, V2s, indexing="ij")
    pts = np.stack([PP.ravel(), VV.ravel()], axis=1)
    M = g1.size
    P = np.repeat(pts[:, 0], M)
    V = np.repeat(pts[:, 1], M)
    D1 = np.tile(g1, len(pts))
    D2 = np.tile(g2, len(pts))
    Ipk = np.full(P.size, np.nan)
    Iid = np.full(P.size, np.nan)
    for s in range(0, P.size, chunk):
        sl = slice(s, s + chunk)
        r = plant.operating_point(P[sl], D1[sl], D2[sl], V[sl])
        Ipk[sl] = np.where(r["ok"], r["Ipk"], np.nan)
        Iid[sl] = r["Ipk_ideal"]
    Ipk = Ipk.reshape(len(pts), M)
    Iid = Iid.reshape(len(pts), M)
    sps = M - 1                                                 # (0.5, 0.5) is the last grid entry
    rows = []
    out(f"{'|P| W':>6s} {'V2 V':>6s} {'k':>6s} | {'plant SPS':>9s} {'plant best':>10s} {'gain':>7s} {'at D1,D2':>11s} | "
        f"{'ideal SPS':>9s} {'ideal best':>10s} {'gain':>7s}")
    for j, (p, v) in enumerate(pts):
        b = int(np.nanargmin(Ipk[j]))
        bi = int(np.nanargmin(Iid[j]))
        gain, gain_i = Ipk[j, sps] - Ipk[j, b], Iid[j, sps] - Iid[j, bi]
        out(f"{p:6.0f} {v:6.1f} {4 * v / 380:6.3f} | {Ipk[j, sps]:9.3f} {Ipk[j, b]:10.3f} {gain:+7.3f} "
            f"{g1[b]:5.3f},{g2[b]:5.3f} | {Iid[j, sps]:9.3f} {Iid[j, bi]:10.3f} {gain_i:+7.3f}")
        rows.append(dict(P=p, V2=v, k=4 * v / 380, plant_sps=Ipk[j, sps], plant_best=Ipk[j, b], plant_gain=gain,
                         best_D1=g1[b], best_D2=g2[b], ideal_sps=Iid[j, sps], ideal_best=Iid[j, bi], ideal_gain=gain_i))
    with open(os.path.join(OUT, "plant_verdict_A.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def load_models(names):
    """'SPS', '<tag>' with out/model_smooth_reghdc_<tag>.json (b3, a, MA_neg, ...), 'L<...>' (out/model_lut_L<...>.npz)."""
    from train_lut_e2e import TrainLUT
    sel = []
    for s in names:
        if s == "SPS":
            sel.append(ct.SPS())
        elif os.path.exists(os.path.join(OUT, f"model_smooth_reghdc_{s}.json")):
            m = ct.SmoothRegHDC.from_json(os.path.join(OUT, f"model_smooth_reghdc_{s}.json"))
            m.name = f"Smooth RegHDC ({s})"
            sel.append(m)
        else:
            m = TrainLUT.load(os.path.join(OUT, f"model_lut_{s}.npz"))
            m.name = f"LUT {m.P.size}x{m.V.size} E2E ({s})"
            sel.append(m)
    return sel


def part_b(plant, out, n, chunk=600, models="SPS,b3,L49s3,L49", region="all", tag=""):
    rng = np.random.default_rng(606 if region == "all" else 707)
    if region == "all":
        P = rng.uniform(25.0, ct.P_MAX, n)
        V = rng.uniform(ct.V2_LO, ct.V2_HI, n)
    else:                                                       # T7: the k ~ 1 light-load band where b3 degraded
        P = rng.uniform(25.0, 500.0, n)
        V = rng.uniform(88.0, 102.0, n)
    sel = load_models(models.split(","))
    res = {}
    for c in sel:
        t0 = time.time()
        d1, d2 = c.predict(P, V)
        d1, d2 = np.asarray(d1, float).ravel(), np.asarray(d2, float).ravel()
        I = np.full(n, np.nan)
        Ii = np.full(n, np.nan)
        cv = np.zeros(n)
        for s in range(0, n, chunk):
            sl = slice(s, s + chunk)
            r = plant.operating_point(P[sl], d1[sl], d2[sl], V[sl])
            I[sl] = np.where(r["ok"], r["Ipk"], np.nan)
            Ii[sl] = r["Ipk_ideal"]
            cv[sl] = r.get("conv", np.ones(r["Ipk"].size, bool))
        res[c.name] = (I, Ii, cv)                                    # npz rows: plant Ipk, ideal Ipk, period-1 converged
        out(f"  {c.name}: {time.time() - t0:.0f} s, {int((cv < 0.5).sum())} points without a converged period-1 "
            f"steady state")
    I_sps, Ii_sps, _ = res["SPS"]
    out(f"\n{n} random points, |P| U[{P.min():.0f}, {P.max():.0f}] W, V2 U[{V.min():.1f}, {V.max():.1f}] V; "
        f"dI = Ipk(SPS) - Ipk(controller), both on the plant")
    out(f"{'controller':30s} {'plant mean dI':>13s} {'worst':>8s} {'>5 mA worse':>12s} {'>20 mA':>7s} {'infeas':>6s} | "
        f"{'ideal mean dI':>13s} {'worst':>8s}")
    rows = []
    for name, (I, Ii, _) in res.items():
        dI = I_sps - I
        dIi = Ii_sps - Ii
        ok = np.isfinite(dI)
        out(f"{name:30s} {np.mean(dI[ok]):13.4f} {np.min(dI[ok]) * 1e3:7.1f}m {np.mean(dI[ok] < -0.005) * 100:11.2f}% "
            f"{np.mean(dI[ok] < -0.02) * 100:6.2f}% {(~ok).sum():6d} | {np.nanmean(dIi):13.4f} {np.nanmin(dIi) * 1e3:7.1f}m")
        rows.append(dict(controller=name, plant_mean_dI=np.mean(dI[ok]), plant_worst=np.min(dI[ok]),
                         plant_deg5=np.mean(dI[ok] < -0.005), plant_deg20=np.mean(dI[ok] < -0.02),
                         infeasible=int((~ok).sum()), ideal_mean_dI=np.nanmean(dIi), ideal_worst=np.nanmin(dIi)))
        if name != "SPS":
            bad = ok & (dI < -0.005)
            if bad.any():
                out(f"    worse-than-SPS points: |P| {P[bad].min():.0f}-{P[bad].max():.0f} W, V2 {V[bad].min():.1f}-"
                    f"{V[bad].max():.1f} V, median |P| {np.median(P[bad]):.0f} W")
    with open(os.path.join(OUT, f"plant_verdict_B{tag}.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    np.savez(os.path.join(OUT, f"plant_verdict_B{tag}.npz"), P=P, V=V,
             **{k.replace(" ", "_"): np.stack(v) for k, v in res.items()})


VARIANTS = {   # T7 robustness: the plant variants that bracket validate_plant.py's fit (all td 100 ns, no edge grid)
    "A": dict(Ron=0.052, droop=True),                           # default (as verdictB)
    "B": dict(Ron=0.040, droop=False),                          # best Ipk bias on the 8000 steady rows (+0.008 A)
    "C": dict(Ron=0.0, droop=False),                            # lossless channel: t7_probe sign flip at k ~ 1
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", choices=("A", "B"), required=True)
    ap.add_argument("--n", type=int, default=1200)
    ap.add_argument("--Ron", type=float, default=0.052)
    ap.add_argument("--variant", default="", choices=("", "A", "B", "C"), help="plant variant (overrides --Ron)")
    ap.add_argument("--region", default="all", choices=("all", "k1"))
    ap.add_argument("--models", default="SPS,b3,L49s3,L49")
    ap.add_argument("--tag", default="", help="output suffix: plant_verdict_B<tag>.{log,csv,npz}")
    a = ap.parse_args()
    t0 = time.time()
    # no 2.88 deg edge quantisation here: it is the Simulink controller's 100 ns sample grid (the MCU HRTIM resolves
    # 184 ps), and on a quantised D0 the PI cannot settle (power is a staircase in D0; it would dither)
    kw = VARIANTS[a.variant] if a.variant else dict(Ron=a.Ron)
    plant = pf.FullPlant(quant=False, **kw)
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"plant: {plant} variant {a.variant or '-'} {kw}")
    if a.part == "A":
        part_a(plant, out)
    else:
        part_b(plant, out, a.n, models=a.models, region=a.region, tag=a.tag)
    out(f"done in {time.time() - t0:.0f} s")
    open(os.path.join(OUT, f"plant_verdict_{a.part}{a.tag}.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
