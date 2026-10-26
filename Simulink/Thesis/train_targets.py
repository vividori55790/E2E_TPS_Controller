"""
train_targets.py -- R1 (scope change 2026-10-26): ONE E2E regression controller (no SPS/TPS classifier, no FSM;
outputs (D1, D2), D0 closed by the PI), trained on two different targets with the same structure and procedure.

  M-A   the existing pipeline's target: Simulink/Physics_Dataset_Final_Regression_Ready.csv (Test.m output, read as-is
        by the latest pure-regression trainer Simulink/Data/RegHDCTrainer.m; signed P +-1200 W x V2 80/90/100/110 V)
  M-B0  the steady-state optimal smooth surface (surface_v2 sweep, lam 2e-3, smooth_surface_v2.py)
        = model_smooth_reghdc_a.json (train_e2e.py a), already trained -- not refitted here
  M-B   M-B0 + physics E2E fine-tuning = model_smooth_reghdc_b3.json (train_e2e.py b), already trained

Same structure: SmoothRegHDC D 256, sigma 0.1, tau 0.004, seed 0 -> the SAME random encoder (W, b) as M-B0.
Same procedure: stage (a) of train_e2e.py -- the target table is bilinearly densified to |P| 10 W x V2 0.4 V
(84-109.2 V) and the read-out is fitted by ridge (alpha 1e-3). Only the target differs.
The dataset's two power directions disagree (max |dD| 0.09 / 0.14 between +P and -P at the same |P|, V2; in steady
state the optimum is direction-independent), so M-A is fitted per direction:
  MA_neg  dataset P <= 0 = HV -> LV (P_act_W = -primary input power) = the direction of every plant / Simulink check
  MA_pos  dataset P >= 0 (LV -> HV); evaluated on |P| like the ICROS2026 rows of evaluate.py
LegacyRegHDC (D 32, K 8, Q 32, gamma 8; the paper's setting) on the same dataset is built by evaluate.build
(cache out/model_legacy_final.npz).

    python -I -X utf8 Simulink/Thesis/train_targets.py      -> out/model_smooth_reghdc_MA_{neg,pos}.json, out/train_targets.log
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
from smooth_surface_v2 import bilinear, dense_grid  # noqa: E402

OUT = os.path.join(HERE, "out")
FINAL_CSV = os.path.join(ROOT, "Simulink", "Physics_Dataset_Final_Regression_Ready.csv")


def dataset_tables(path=FINAL_CSV):
    """{+1: (P, V, T1, T2), -1: ...} on |P| (rows with sign(P) * sgn >= 0, P = 0 in both), like evaluate.icros_tables."""
    import pandas as pd
    d = pd.read_csv(path)
    out = {}
    for sgn in (+1, -1):
        s = d[np.sign(d["Target_P_W"]) * sgn >= 0]
        s = s.assign(Pa=s["Target_P_W"].abs())
        P = np.sort(s["Pa"].unique())
        V = np.sort(s["Target_V2_V"].unique())
        T1 = s.pivot_table(index="Target_V2_V", columns="Pa", values="Opt_D1").loc[V, P].to_numpy()
        T2 = s.pivot_table(index="Target_V2_V", columns="Pa", values="Opt_D2").loc[V, P].to_numpy()
        out[sgn] = (P, V, T1, T2)
    return d, out


def fit_target(P, V, T1, T2, ref):
    """stage (a) with the M-B0 hyper-parameters and encoder: densify, ridge."""
    meta = ref.meta
    _, _, DP, DV = dense_grid()
    t1 = bilinear(T1, P, V, DP.ravel(), DV.ravel())
    t2 = bilinear(T2, P, V, DP.ravel(), DV.ravel())
    m = ct.SmoothRegHDC(D=ref.Dh, sigma=ref.sigma, tau=ref.tau, seed=int(meta.get("seed", 0)))
    assert np.allclose(m.W, ref.W) and np.allclose(m.b, ref.b), "encoder must be the M-B0 encoder"
    m.fit_ridge(DP.ravel(), DV.ravel(), t1, t2, alpha=float(meta["alpha"]))
    f1, f2 = m.predict(DP.ravel(), DV.ravel())
    rmse = float(np.sqrt(np.mean(np.concatenate([(f1 - t1) ** 2, (f2 - t2) ** 2]))))
    return m, rmse


def main():
    from train_e2e import Evaluator
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    ref = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_a.json"))
    _, tabs = dataset_tables()
    ev = Evaluator()
    out(f"M-A = SmoothRegHDC(D {ref.Dh}, sigma {ref.sigma}, tau {ref.tau}, alpha {ref.meta['alpha']}, seed "
        f"{int(ref.meta.get('seed', 0))}) on {os.path.relpath(FINAL_CSV, ROOT)}; validation = train_e2e.Evaluator "
        f"({ev.P.size} random points, ideal steady-state model, PI first crossing)")
    rep = ev(ref, 20.0, 1e-3)
    out(f"  M-B0 (a) for reference: val mean dI {rep['mean_dI']:.4f} A, worst {rep['min_dI'] * 1e3:+.1f} mA, "
        f">5 mA worse {rep['degraded']}, infeasible {rep['infeasible']}")
    for sgn, tag, lab in ((-1, "MA_neg", "P <= 0, HV -> LV"), (+1, "MA_pos", "P >= 0, LV -> HV")):
        P, V, T1, T2 = tabs[sgn]
        m, rmse = fit_target(P, V, T1, T2, ref)
        m.name = f"M-A Smooth RegHDC (dataset {'P<0' if sgn < 0 else 'P>0'})"
        rep = ev(m, 20.0, 1e-3)
        out(f"  {tag} ({lab}): table {T1.shape[1]} x {T1.shape[0]} (|P| {P.min():.0f}-{P.max():.0f} W, V2 {V.min():.0f}-"
            f"{V.max():.0f} V), fit RMSE(D) {rmse:.4f}, sum|M| {np.abs(m.M).sum():.1f} | val mean dI {rep['mean_dI']:.4f} A, "
            f"worst {rep['min_dI'] * 1e3:+.1f} mA, >5 mA worse {rep['degraded']} / {ev.P.size}, infeasible {rep['infeasible']}")
        meta = {k: float(v) for k, v in rep.items()}
        meta.update(stage="MA", target=os.path.basename(FINAL_CSV), direction=lab, rmse_D=rmse,
                    alpha=float(ref.meta["alpha"]), seed=int(ref.meta.get("seed", 0)))
        m.to_json(os.path.join(OUT, f"model_smooth_reghdc_{tag}.json"), **meta)
    open(os.path.join(OUT, "train_targets.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
