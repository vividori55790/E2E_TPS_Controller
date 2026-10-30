"""
train_corrected.py -- OPT4 (2026-10-30): the proposed surface M-B plus Simulink-measured local corrections -> M-S.

OPT round 1 (surface_opt_check.py -> out/surface_opt_verdict.md) found M-B Simulink-optimal at 600-1000 W and off only at
light load: (a) at k = 1 (95 V) Simulink prefers D1 = D2 ~ 0.40-0.45 at the lowest power-carrying D0 level (~70 W:
0.415 vs SPS 0.738 A), where M-B is SPS; (b) at 109.2 V 100 W M-B's duty pair sits where the peak flickers between 1 ms
windows, while three steady candidates are 0.19-0.25 A lower. The analytic model has neither effect, so E2E training on
it cannot find them.

Correction by anchors. Each anchor (V2, P, D1, D2, wV [V], wP [W], source) is a Simulink-measured better duty pair at
one operating point. Gaussian weights w_i = exp(-((P - P_i)/wP_i)^2 - ((V2 - V2_i)/wV_i)^2), w = max_i w_i, and
    target = (1 - w) * M-B + w * (sum_i w_i anchor_i / sum_i w_i).
The same Smooth RegHDC (M-B's encoder W, b: D 256, sigma 0.1, tau 0.004, seed 0) is refitted by ridge (alpha 1e-3, as
stage (a)) on the dense (|P|, V2) grid of smooth_surface_v2. No analytic E2E fine-tuning afterwards: it would pull the
anchors back to the ideal-model optimum. M-B itself came out of E2E training, so a plain ridge refit does not reproduce
it exactly; that refit error is reported as the baseline.
Anchors: a CSV with columns V2, P, D1, D2, wV, wP, source (each row cites the Simulink file it comes from).
The analytic-model numbers printed here are reference only (LOOP_PROMPT 검증 원칙); M-S is judged in Simulink.

    python -I -X utf8 Simulink/Thesis/train_corrected.py --anchors out/opt4_anchors.csv --tag MS
       -> out/model_smooth_reghdc_<tag>.json, out/train_corrected_<tag>.log, out/fig_train_corrected_<tag>.png
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
from smooth_surface_v2 import dense_grid  # noqa: E402

OUT = os.path.join(HERE, "out")
ALPHA = 1e-3


def load_anchors(path):
    import pandas as pd
    a = pd.read_csv(path, comment="#")
    need = {"V2", "P", "D1", "D2", "wV", "wP", "source"}
    miss = need - set(a.columns)
    if miss:
        sys.exit(f"{path}: missing columns {sorted(miss)}")
    return a


def weights(P, V, A):
    """(N, n_anchor) Gaussian weights"""
    P, V = np.asarray(P, float)[:, None], np.asarray(V, float)[:, None]
    return np.exp(-((P - A["P"].to_numpy()) / A["wP"].to_numpy()) ** 2
                  - ((V - A["V2"].to_numpy()) / A["wV"].to_numpy()) ** 2)


def target(mb, P, V, A):
    """optional column mode: 'set' (default) blends towards the anchor's duties; 'cap' (10-30, k = 1 band of k1map) only
    pulls a duty DOWN to the anchor's value where the target is above it -- k1map measured D1 = D2 = 0.475 better than
    SPS at 95-97 V light load, but nothing below M-B's own duties where M-B is already TPS (97-98 V, 100 W: 0.40-0.47)"""
    d1, d2 = (np.ravel(x) for x in mb.predict(P, V))
    w = np.zeros_like(d1)
    if A is None or len(A) == 0:
        return d1, d2, w
    mode = A["mode"].fillna("set").to_numpy() if "mode" in A else np.full(len(A), "set")
    S, C = A[mode == "set"], A[mode == "cap"]
    if len(S):
        Wt = weights(P, V, S)
        w = Wt.max(axis=1)
        s = np.maximum(Wt.sum(axis=1), 1e-300)
        a1 = (Wt * S["D1"].to_numpy()).sum(axis=1) / s
        a2 = (Wt * S["D2"].to_numpy()).sum(axis=1) / s
        d1, d2 = (1 - w) * d1 + w * a1, (1 - w) * d2 + w * a2
    if len(C):
        Wc = weights(P, V, C)
        for j, (c1, c2) in enumerate(zip(C["D1"].to_numpy(), C["D2"].to_numpy())):
            d1 = d1 - Wc[:, j] * np.maximum(d1 - c1, 0.0)
            d2 = d2 - Wc[:, j] * np.maximum(d2 - c2, 0.0)
        w = np.maximum(w, Wc.max(axis=1))
    return d1, d2, w


def fit(mb, P, V, t1, t2, alpha=ALPHA, sample_w=None):
    m = ct.SmoothRegHDC(D=mb.Dh, sigma=mb.sigma, tau=mb.tau, seed=0)
    assert np.allclose(m.W, mb.W) and np.allclose(m.b, mb.b), "encoder must be M-B's encoder"
    return m.fit_ridge(P, V, t1, t2, alpha=alpha, sample_w=sample_w)


def fit_delta(mb, P, V, t1, t2, w, alpha=ALPHA, wmin=1e-3, sample_w=None):
    """--delta (10-30, after OPT round 2): M-S = M-B's readout + a ridge-fitted correction of the pre-activation.
    The correction's target is exactly 0 where the anchor weight w <= wmin, so far from the anchors M-S stays on M-B up
    to the correction's own spill-over. The plain refit (fit) moves M-B by up to 0.007 everywhere -- e.g. off the
    V-shaped Simulink optimum at 84 V 1 kW, where 0.02 in D1 costs 0.07-0.21 A.
    sample_w (--anchor-weight with --delta): where M-B is saturated at SPS (k = 1 band), a duty change of 0.02 needs the
    pre-activation to cross 0 by ~0.02; a fit that gets 60 % of the way leaves the duty at 0.498 -- weighting helps."""
    m = ct.SmoothRegHDC(D=mb.Dh, sigma=mb.sigma, tau=mb.tau, seed=0)
    assert np.allclose(m.W, mb.W) and np.allclose(m.b, mb.b), "encoder must be M-B's encoder"
    H = mb.features(P, V)
    y0, tau = mb.raw(H), mb.tau
    delta = np.stack([0.5 - np.asarray(t1, float), 0.5 - np.asarray(t2, float)], axis=-1)
    y = np.where(delta > 4 * tau, delta, tau * np.log(np.expm1(np.clip(delta, 1e-6, None) / tau)))
    y = np.maximum(y, -6 * tau)                                  # the same softplus inverse and floor as fit_ridge
    r = np.where((y <= -6 * tau + 1e-12) & (y0 <= -6 * tau), 0.0, y - y0)   # both saturated at SPS: no change
    r = r * (np.asarray(w) > wmin)[:, None]
    Hs = H / np.sqrt(mb.Dh)
    Ha = np.concatenate([Hs, np.ones((Hs.shape[0], 1))], axis=1)
    sw_ = np.ones(Ha.shape[0]) if sample_w is None else np.asarray(sample_w, float)
    sol = np.linalg.solve(Ha.T @ (Ha * sw_[:, None]) + alpha * np.eye(Ha.shape[1]), Ha.T @ (r * sw_[:, None]))
    m.M, m.c = mb.M + sol[:-1], mb.c + sol[-1]
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--anchors", default=os.path.join(OUT, "opt4_anchors.csv"))
    ap.add_argument("--tag", default="MS")
    ap.add_argument("--alpha", type=float, default=ALPHA, help="ridge alpha (stage (a) used 1e-3)")
    ap.add_argument("--anchor-weight", type=float, default=0.0,
                    help="sample weight 1 + k * w(P, V2): makes the fit honour narrow anchors (0 = plain stage (a))")
    ap.add_argument("--delta", action="store_true",
                    help="fit only the correction on top of M-B's readout (M-S = M-B exactly far from the anchors)")
    a = ap.parse_args()
    lines = []
    name = f"M-{a.tag[1:]}" if a.tag.startswith("M") else a.tag

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    mb = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    A = load_anchors(a.anchors)
    _, _, PP, VV = dense_grid()
    P, V = PP.ravel(), VV.ravel()
    b1, b2 = (np.ravel(x) for x in mb.predict(P, V))

    base = fit(mb, P, V, b1, b2)
    f1, f2 = (np.ravel(x) for x in base.predict(P, V))
    out(f"baseline: ridge refit of M-B's own outputs (alpha {ALPHA:g}): RMSE {np.sqrt(np.mean((f1 - b1) ** 2 + (f2 - b2) ** 2) / 2):.4f}, "
        f"max |dD| {max(np.abs(f1 - b1).max(), np.abs(f2 - b2).max()):.4f}")

    t1, t2, w = target(mb, P, V, A)
    if a.delta:
        ms = fit_delta(mb, P, V, t1, t2, w, alpha=a.alpha,
                       sample_w=None if a.anchor_weight == 0 else 1.0 + a.anchor_weight * w)
    else:
        ms = fit(mb, P, V, t1, t2, alpha=a.alpha, sample_w=1.0 + a.anchor_weight * w)
    m1, m2 = (np.ravel(x) for x in ms.predict(P, V))
    out(f"{name} = {'M-B + ridge fit of the correction (--delta)' if a.delta else 'ridge fit of the corrected target'}, "
        f"{len(A)} anchors from {os.path.relpath(a.anchors, HERE)} (alpha {a.alpha:g}, anchor weight {a.anchor_weight:g})")
    out(f"  RMSE to target {np.sqrt(np.mean((m1 - t1) ** 2 + (m2 - t2) ** 2) / 2):.4f}, max |dD| "
        f"{max(np.abs(m1 - t1).max(), np.abs(m2 - t2).max()):.4f}")
    far = w < 0.01
    out(f"  away from the anchors (w < 0.01, {far.mean() * 100:.0f} % of the grid): max |{name} - M-B| "
        f"{max(np.abs(m1 - b1)[far].max(), np.abs(m2 - b2)[far].max()):.4f} (baseline refit {max(np.abs(f1 - b1)[far].max(), np.abs(f2 - b2)[far].max()):.4f})")
    op = far & (P >= 100) & (P <= 1000)                    # the max above sits at P = 0 (109.2 V); the operating range:
    dd = np.maximum(np.abs(m1 - b1), np.abs(m2 - b2))
    if op.any():                                           # none when anchors cover the whole range (R13 stage 2)
        i = int(np.argmax(np.where(op, dd, -1)))
        out(f"    of which at 100-1000 W: max {dd[op].max():.4f} (at {V[i]:.1f} V {P[i]:.0f} W), mean {dd[op].mean():.4f}")
    out(f"  at the anchors: anchor (D1, D2) -> {name} (D1, D2), M-B (D1, D2)")
    for _, r in A.iterrows():
        s1, s2 = (float(np.ravel(x)[0]) for x in ms.predict(np.array([r["P"]]), np.array([r["V2"]])))
        q1, q2 = (float(np.ravel(x)[0]) for x in mb.predict(np.array([r["P"]]), np.array([r["V2"]])))
        out(f"    {r['V2']:6.2f} V {r['P']:6.0f} W: ({r['D1']:.3f}, {r['D2']:.3f}) -> ({s1:.3f}, {s2:.3f}), M-B ({q1:.3f}, {q2:.3f})"
            f"  [{r['source']}]")
    jm = np.abs(ms.jacobian(P, V)).max()
    out(f"  max |dDelta/dx| (normalised inputs): {name} {jm:.2f}, M-B {np.abs(mb.jacobian(P, V)).max():.2f}")

    try:                                                   # reference only: what the ideal model thinks of M-S
        import pi_steady as ps
        rng = np.random.default_rng(1)
        Pr, Vr = rng.uniform(50, 1200, 4000), rng.uniform(84, 109.2, 4000)
        res = {}
        for nm, mdl in (("M-B", mb), (name, ms)):
            d1, d2 = (np.ravel(x) for x in mdl.predict(Pr, Vr))
            ip = ps.operating_point(Pr, d1, d2, Vr)["Ipk"]
            isps = ps.operating_point(Pr, np.full_like(Pr, 0.5), np.full_like(Pr, 0.5), Vr)["Ipk"]
            res[nm] = isps - ip
        out(f"  [참고, 해석 모델] 4000 random points, dI = Ipk_SPS - Ipk: M-B mean {np.nanmean(res['M-B']):+.4f} A, "
            f"{name} mean {np.nanmean(res[name]):+.4f} A, {name} worst {np.nanmin(res[name]):+.3f} A "
            f"(the ideal model has no k = 1 valley, so an anchor there costs here by construction)")
    except Exception as e:                                 # noqa: BLE001
        out(f"  (analytic reference skipped: {e})")

    path = os.path.join(OUT, f"model_smooth_reghdc_{a.tag}.json")
    ms.to_json(path, stage="OPT4", base="b3", alpha=a.alpha, anchor_weight=a.anchor_weight, delta=a.delta,
               anchors=A.to_dict("records"))
    out(f"wrote {os.path.relpath(path, HERE)}")
    fig(mb, ms, A, a.tag, name)
    open(os.path.join(OUT, f"train_corrected_{a.tag}.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


def fig(mb, ms, A, tag, name="M-S"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, MUTED, ORANGE, _style
    _style(plt)
    Pc = np.linspace(0, 1200, 241)
    vs = (84.0, 96.0, 97.0, 109.2)          # 96/97 V: the k = 1 cap anchors (95 V stays on M-B, see opt4_anchors_r3.csv)
    if len(A) > 20:                         # anchors on the R13 map: its V2 values, the k = 1 band and the side with the gaps
        vs = (84.0, 96.6, 105.0, 109.2)
    f, ax = plt.subplots(1, len(vs), figsize=(4.2 * len(vs), 3.6), sharey=True, constrained_layout=True)
    for a_, v in zip(ax, vs):
        Vc = np.full_like(Pc, v)
        q1, q2 = (np.ravel(x) for x in mb.predict(Pc, Vc))
        s1, s2 = (np.ravel(x) for x in ms.predict(Pc, Vc))
        t1, t2, _ = target(mb, Pc, Vc, A)
        a_.plot(Pc, q1, color=BLUE, lw=1.2, ls="--", label="M-B D1")
        a_.plot(Pc, q2, color=ORANGE, lw=1.2, ls="--", label="M-B D2")
        a_.plot(Pc, t1, color=MUTED, lw=0.8, ls=":", label="target")
        a_.plot(Pc, t2, color=MUTED, lw=0.8, ls=":")
        a_.plot(Pc, s1, color=BLUE, lw=1.8, label=f"{name} D1")
        a_.plot(Pc, s2, color=ORANGE, lw=1.8, label=f"{name} D2")
        near = A[np.abs(A["V2"] - v) <= A["wV"]]
        a_.plot(near["P"], near["D1"], "o", color=BLUE, ms=6, mec="white")
        a_.plot(near["P"], near["D2"], "s", color=ORANGE, ms=6, mec="white")
        a_.set_title(f"V2 = {v:g} V", fontsize=9)
        a_.set_xlabel("|P| [W]")
    ax[0].set_ylabel("duty")
    ax[0].legend(fontsize=7.5, frameon=False, loc="lower right")
    f.savefig(os.path.join(OUT, f"fig_train_corrected_{tag}.png"), dpi=130)
    plt.close(f)


if __name__ == "__main__":
    main()
