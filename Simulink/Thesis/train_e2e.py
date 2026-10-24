"""
train_e2e.py -- T3: the proposed Smooth RegHDC controller, trained in two stages.

(a) surface regression  (python train_e2e.py a)
    Ridge read-out on a T1 target surface (bilinearly densified to |P| 10 W x V2 0.4 V). The hyper-parameters
    (target lam, D, sigma, alpha, tau) are chosen by what the controller does on random operating points --
    the PI steady-state Ipk -- not by RMSE(D):
        score = mean(dI) - beta * mean(relu(-dI)) - gamma * mean ||dDelta/dx||^2,   dI = Ipk_SPS - Ipk  [A]
(b) physics-in-the-loop E2E fine-tuning  (python train_e2e.py b --init a ...)
    On fresh uniform random (|P|, V2) batches minimise directly
        mean(Ipk - Ipk_SPS) + beta * mean(relu(Ipk - Ipk_SPS)) + beta_inf * mean(relu(|P*| - Pmax) / 100 W)
        + gamma * mean ||dDelta/dx||_F^2
    with Ipk at the PI steady state (pi_steady first crossing). dIpk/dD comes from the implicit function theorem
    on the power constraint p(phi, D1, D2) = |P*| -- the same thing as re-solving phi after a finite difference
    in D, but needing no extra root search:
        dIpk/dD = dIpk/dD|phi - (dIpk/dphi) (dp/dD) / (dp/dphi)        (partials by finite differences)
    then the chain rule through Delta = softplus_tau(y) to the read-out M, c. The encoder (W, b) stays the fixed
    random projection, as in HDC. Adam, cosine learning-rate decay, the best validation state is kept.
    T7 (--margin m): Ipk - Ipk_SPS is charged m (1 - exp(-(Delta1 + Delta2) / d0)) in the loss, the hinge and the
    validation score, so the controller leaves SPS only where the predicted reduction exceeds m (dead-time
    robustness on the non-ideal plant; see `departure`). b8+ start from b3 (--src b3).

Models: out/model_smooth_reghdc_a.json, out/model_smooth_reghdc_b<tag>.json; tables out/train_e2e_a.csv,
out/train_e2e_b<tag>_hist.csv. Validation = 4000 uniform random points (seed 101); T5 uses a different seed.
"""
import argparse
import csv
import itertools
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402
from smooth_surface_v2 import bilinear, dense_grid, steps  # noqa: E402

OUT = os.path.join(HERE, "out")
VAL_SEED, VAL_N = 101, 4000


def random_points(n, rng):
    return rng.uniform(0.0, ct.P_MAX, n), rng.uniform(ct.V2_LO, ct.V2_HI, n)


# ------------------------------------------------------------------ evaluation
def forward(m, H):
    """Delta (N, 2) and its derivative wrt the pre-activation (0 where Delta is capped at 0.5 - D_MIN)."""
    y = m.raw(H)
    sp = ct.softplus(y, m.tau)
    cap = sp < 0.5 - ct.D_MIN
    return np.where(cap, sp, 0.5 - ct.D_MIN), ct.sigmoid(y / m.tau) * cap


def jac_sq(m, P, V):
    J = m.jacobian(P, V)                                    # (N, 2 in, 2 out), wrt normalised inputs
    return (J ** 2).sum(axis=(1, 2)), np.abs(J).max(axis=(1, 2))


def departure(delta, margin, d0):
    """T7 SPS-departure cost margin * (1 - exp(-(Delta1 + Delta2) / d0)) [A] and its gradient wrt Delta (N, 2).
    On the dead-time plant, leaving SPS by Delta ~ 1e-3..0.06 moves Ipk erratically by +-20 mA at k ~ 1 (sign set by
    Ron / droop details, t7_probe.py), so a departure has to buy at least `margin` of predicted reduction; below
    that the optimum is exactly SPS, which is SPS on any plant."""
    if margin <= 0.0:
        return np.zeros(delta.shape[0]), np.zeros_like(delta)
    ex = np.exp(-delta.sum(axis=1) / d0)
    return margin * (1.0 - ex), np.repeat((margin / d0 * ex)[:, None], 2, axis=1)


class Evaluator:
    """Fixed random validation set + the 25 W x 1.2 V grid of T1 (for the adjacent-step metric)."""

    def __init__(self, n=VAL_N, seed=VAL_SEED):
        self.P, self.V = random_points(n, np.random.default_rng(seed))
        self.sps = ps.sps_reference(self.P, self.V)["Ipk"]
        S = np.load(os.path.join(OUT, "surface_v2.npz"))
        self.gP, self.gV = S["P"], S["V2"]
        self.GP, self.GV = np.meshgrid(self.gP, self.gV)
        _, _, self.DP, self.DV = dense_grid()

    def __call__(self, m, beta, gamma, margin=0.0, d0=0.005):
        d1, d2 = m.predict(self.P, self.V)
        d1, d2 = np.asarray(d1, float).ravel(), np.asarray(d2, float).ravel()
        r = ps.operating_point(self.P, d1, d2, self.V)
        dI = self.sps - r["Ipk"]
        ok = r["ok"]
        dIf = dI[ok]
        delta = np.stack([0.5 - d1, 0.5 - d2], axis=1)
        dIr = (dI - departure(delta, margin, d0)[0])[ok]              # = dI when margin = 0
        dsum = delta.sum(axis=1)[ok]
        j2, jmax = jac_sq(m, self.DP.ravel(), self.DV.ravel())
        g1, g2 = m.predict(self.GP, self.GV)
        st = steps(g1, g2)
        rep = dict(mean_dI=float(dIf.mean()) if ok.any() else np.nan,
                   min_dI=float(dIf.min()) if ok.any() else np.nan,
                   max_dI=float(dIf.max()) if ok.any() else np.nan,
                   hinge=float(np.maximum(-dIf, 0).mean()) if ok.any() else np.nan,
                   degraded=int(np.sum(dIf < -0.005)), degraded_any=int(np.sum(dIf < -1e-4)),
                   infeasible=int((~ok).sum()), jac2=float(j2.mean()), jac_max=float(jmax.max()),
                   max_step=st["max_step"], tv=st["tv"],
                   # departures from SPS (Delta1 + Delta2 > 1e-3) that buy less than 25 mA: the dead-time risk band
                   unpaid=int(np.sum((dsum > 1e-3) & (dIf < 0.025))), departed=int(np.sum(dsum > 1e-3)))
        rep["score"] = (float(dIr.mean()) - beta * float(np.maximum(-dIr, 0).mean()) - gamma * rep["jac2"]
                        - 1.0 * rep["infeasible"])
        return rep


# ------------------------------------------------------------------ stage (a)
def stage_a(a):
    t0 = time.time()
    SW = np.load(os.path.join(OUT, "surface_v2_sweep.npz"))
    gP, gV = SW["P"], SW["V2"]
    p, v, DP, DV = dense_grid()
    ev = Evaluator()
    lams = [float(x) for x in a.lams.split(",")]
    grid = list(itertools.product(lams, [int(x) for x in a.Ds.split(",")], [float(x) for x in a.sigmas.split(",")],
                                  [float(x) for x in a.alphas.split(",")], [float(x) for x in a.taus.split(",")]))
    print(f"stage (a): {len(grid)} configurations, validation {VAL_N} random points", flush=True)
    rows, best, best_D = [], None, {}
    for lam, D, sig, alpha, tau in grid:
        il = int(np.argmin(np.abs(SW["lam"] - lam)))
        T1 = bilinear(SW["D1"][il], gP, gV, DP.ravel(), DV.ravel())
        T2 = bilinear(SW["D2"][il], gP, gV, DP.ravel(), DV.ravel())
        m = ct.SmoothRegHDC(D=D, sigma=sig, tau=tau, seed=a.seed).fit_ridge(DP.ravel(), DV.ravel(), T1, T2, alpha=alpha)
        f1, f2 = m.predict(DP.ravel(), DV.ravel())
        rmse = float(np.sqrt(np.mean(np.concatenate([(f1 - T1) ** 2, (f2 - T2) ** 2]))))
        rep = ev(m, a.beta, a.gamma)
        rep.update(lam=SW["lam"][il], D=D, sigma=sig, alpha=alpha, tau=tau, rmse_D=rmse,
                   sum_abs_M=float(np.abs(m.M).sum()))
        rows.append(rep)
        if best is None or rep["score"] > best[0]["score"]:
            best = (rep, m)
        if D not in best_D or rep["score"] > best_D[D][0]["score"]:
            best_D[D] = (rep, m)
        print(f"lam {lam:7.1e} D {D:4d} sig {sig:5.3f} a {alpha:7.1e} tau {tau:6.4f} | RMSE(D) {rmse:.4f} "
              f"| dI mean {rep['mean_dI']:.4f} min {rep['min_dI'] * 1e3:+7.1f} mA deg {rep['degraded']:3d} "
              f"inf {rep['infeasible']} | jac2 {rep['jac2']:7.2f} max {rep['jac_max']:6.2f} step {rep['max_step']:.3f} "
              f"| score {rep['score']:.4f} ({time.time() - t0:.0f} s)", flush=True)
    keys = ["lam", "D", "sigma", "alpha", "tau", "rmse_D", "sum_abs_M", "mean_dI", "min_dI", "max_dI", "hinge",
            "degraded", "degraded_any", "infeasible", "jac2", "jac_max", "max_step", "tv", "score"]
    with open(os.path.join(OUT, "train_e2e_a.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in sorted(rows, key=lambda r: -r["score"]):
            w.writerow([f"{r[k]:.6g}" for k in keys])
    for suffix, (rep, m) in [("", best)] + [(f"_D{D}", best_D[D]) for D in sorted(best_D)]:
        m.name = "Smooth RegHDC (a)"
        meta = {k: float(v) for k, v in rep.items()}
        meta.update(stage="a", beta=a.beta, gamma=a.gamma, seed=a.seed)
        m.to_json(os.path.join(OUT, f"model_smooth_reghdc_a{suffix}.json"), **meta)
        print(f"\nbest (a){suffix}:", {k: round(float(v), 5) for k, v in rep.items()})
    print("top 10 by score:")
    for r in sorted(rows, key=lambda r: -r["score"])[:10]:
        print("  ", {k: (round(float(r[k]), 5)) for k in keys})
    print(f"done in {time.time() - t0:.0f} s")


# ------------------------------------------------------------------ stage (b)
def _ipk_p(phi, D1, D2, V2):
    r = tm.evaluate(phi - 180.0 * (D2 - D1), D1, D2, V2)
    return r["Ipk"], -r["P"]


def ift_grad(phi, D1, D2, V2, hD=1e-6, hphi=1e-5):
    """dIpk/dD1, dIpk/dD2 along the PI steady state (power held at |P*|). Backward differences in D so that
    D = 0.5 is never exceeded (the feasible direction is always D decreasing from SPS)."""
    f0, p0 = _ipk_p(phi, D1, D2, V2)
    fp, pp = _ipk_p(phi + hphi, D1, D2, V2)
    fm, pm = _ipk_p(phi - hphi, D1, D2, V2)
    f_phi, p_phi = (fp - fm) / (2 * hphi), (pp - pm) / (2 * hphi)
    f1, p1 = _ipk_p(phi, D1 - hD, D2, V2)
    f2, p2 = _ipk_p(phi, D1, D2 - hD, V2)
    fD = np.stack([(f0 - f1) / hD, (f0 - f2) / hD], axis=1)
    pD = np.stack([(p0 - p1) / hD, (p0 - p2) / hD], axis=1)
    good = p_phi > 1e-2                                                 # W/deg; tangential crossings are skipped
    g = fD - (f_phi / np.where(good, p_phi, 1.0))[:, None] * pD
    return np.where(good[:, None], g, 0.0), good


def pmax_grad(P_abs, D1, D2, V2, hD=1e-6):
    """Pmax of the branch on the PI scan grid and dPmax/dD (envelope theorem: derivative at the argmax)."""
    grid = np.arange(0.0, ps.PHI_MAX + 1e-9, 1.0)
    pg = ps._p_abs(grid[None, :], D1[:, None], D2[:, None], V2[:, None], np.full((D1.size, 1), tm.V1_NOM))
    phi = grid[np.argmax(pg, axis=1)]
    _, p0 = _ipk_p(phi, D1, D2, V2)
    _, p1 = _ipk_p(phi, D1 - hD, D2, V2)
    _, p2 = _ipk_p(phi, D1, D2 - hD, V2)
    return p0, np.stack([(p0 - p1) / hD, (p0 - p2) / hD], axis=1)


def loss_grad(m, P, V, sps, beta, gamma, beta_inf, margin=0.0, d0=0.005):
    """Training loss on one batch and its gradient wrt (M, c). margin > 0: Ipk is charged the SPS-departure cost
    (T7, `departure`) in both the main term and the hinge."""
    N = P.size
    sq = np.sqrt(m.Dh)
    H = m.features(P, V)
    delta, g = forward(m, H)
    D1, D2 = 0.5 - delta[:, 0], 0.5 - delta[:, 1]
    r = ps.operating_point(P, D1, D2, V)
    ok = r["ok"]
    c_dep, g_dep = departure(delta, margin, d0)
    e = np.where(ok, r["Ipk"] - sps + c_dep, 0.0)
    L_main = e.sum() / N
    L_deg = beta * np.maximum(e, 0).sum() / N
    dL_dD = np.zeros((N, 2))
    gi, good = ift_grad(np.nan_to_num(r["phi"]), D1, D2, V)
    w = np.where(ok, (1.0 + beta * (e > 0)) / N, 0.0)
    dL_dD += w[:, None] * (gi - g_dep)                                  # dDelta/dD = -1
    L_inf = 0.0
    if (~ok).any():
        bi = ~ok
        pm, dpm = pmax_grad(P[bi], D1[bi], D2[bi], V[bi])
        short = np.maximum(P[bi] - pm, 0.0) / 100.0
        L_inf = beta_inf * short.sum() / N
        dL_dD[bi] += -(beta_inf / 100.0 / N) * dpm * (short > 0)[:, None]
    dL_dy = -dL_dD * g                                              # D = 0.5 - Delta
    gM = H.T @ dL_dy / sq
    gc = dL_dy.sum(axis=0)
    # smoothness: gamma * mean ||g * dy/dx||^2 with g held fixed (stop-gradient through the gate)
    x = ct.norm_inputs(P, V)
    z = x @ m.W + m.b
    dH = -np.sqrt(2.0) * np.sin(z)[:, None, :] * m.W[None, :, :]   # (N, 2 in, D)
    J = (dH @ m.M / sq) * g[:, None, :]                              # (N, 2 in, 2 out)
    L_s = gamma * (J ** 2).sum() / N
    gM += 2.0 * gamma / N * np.einsum("nid,nio->do", dH, J * g[:, None, :]) / sq
    parts = dict(main=L_main, deg=L_deg, inf=L_inf, smooth=L_s, n_inf=int((~ok).sum()), n_skip=int((ok & ~good).sum()))
    return L_main + L_deg + L_inf + L_s, gM, gc, parts


def stage_b(a):
    t0 = time.time()
    tag = a.tag
    src_path = os.path.join(OUT, f"model_smooth_reghdc_{a.src}.json")
    if a.init == "a":
        m = ct.SmoothRegHDC.from_json(src_path)
    else:
        src = ct.SmoothRegHDC.from_json(src_path)
        m = ct.SmoothRegHDC(D=src.Dh, sigma=src.sigma, tau=src.tau, seed=a.seed)
        m.c = np.full(2, a.sps_init_y * m.tau)                       # Delta ~ 0: starts as SPS
    m.name = f"Smooth RegHDC (b{tag})"
    ev = Evaluator()
    rng = np.random.default_rng(a.seed + 7)
    th = [m.M, m.c]
    mom = [np.zeros_like(x) for x in th]
    vel = [np.zeros_like(x) for x in th]
    b1, b2, eps = 0.9, 0.999, 1e-12
    rep = ev(m, a.beta, a.gamma, a.margin, a.d0)
    rep.update(step=0, loss=np.nan)
    hist = [rep]
    best = (rep["score"], m.M.copy(), m.c.copy(), 0)
    print(f"stage (b{tag}) init={a.init} src={a.src} D={m.Dh} sigma={m.sigma} tau={m.tau} beta={a.beta} "
          f"gamma={a.gamma} margin={a.margin} d0={a.d0} lr={a.lr} batch={a.batch} steps={a.steps}", flush=True)
    print(f"  step     0: val dI mean {rep['mean_dI']:.4f} min {rep['min_dI'] * 1e3:+.1f} mA deg {rep['degraded']} "
          f"inf {rep['infeasible']} jac2 {rep['jac2']:.2f} max {rep['jac_max']:.2f} step {rep['max_step']:.3f} "
          f"unpaid {rep['unpaid']}/{rep['departed']} score {rep['score']:.4f}", flush=True)
    run = []
    for it in range(1, a.steps + 1):
        P, V = random_points(a.batch, rng)
        sps = ps.sps_reference(P, V)["Ipk"]
        L, gM, gc, parts = loss_grad(m, P, V, sps, a.beta, a.gamma, a.beta_inf, a.margin, a.d0)
        run.append([L, parts["main"], parts["deg"], parts["smooth"], parts["n_inf"], parts["n_skip"]])
        lr = (a.lr_min + 0.5 * (a.lr - a.lr_min) * (1 + np.cos(np.pi * it / a.steps))) * min(1.0, it / a.warmup)
        for x, gx, mo, ve in zip(th, (gM, gc), mom, vel):
            mo *= b1
            mo += (1 - b1) * gx
            ve *= b2
            ve += (1 - b2) * gx * gx
            x -= lr * (mo / (1 - b1 ** it)) / (np.sqrt(ve / (1 - b2 ** it)) + eps)
        if it % a.eval_every == 0 or it == a.steps:
            rep = ev(m, a.beta, a.gamma, a.margin, a.d0)
            ra = np.mean(run, axis=0)
            run = []
            rep.update(step=it, loss=float(ra[0]), tr_main=float(ra[1]), tr_deg=float(ra[2]), tr_smooth=float(ra[3]),
                       tr_inf=float(ra[4]), tr_skip=float(ra[5]), lr=lr)
            hist.append(rep)
            mark = ""
            if rep["score"] > best[0]:
                best = (rep["score"], m.M.copy(), m.c.copy(), it)
                mark = " *"
            print(f"  step {it:5d}: loss {ra[0]:+.4f} (main {ra[1]:+.4f} deg {ra[2]:.4f} smooth {ra[3]:.4f} "
                  f"inf {ra[4]:.1f} skip {ra[5]:.1f}) | val dI mean {rep['mean_dI']:.4f} min {rep['min_dI'] * 1e3:+.1f} "
                  f"mA deg {rep['degraded']} inf {rep['infeasible']} jac2 {rep['jac2']:.2f} max {rep['jac_max']:.2f} "
                  f"step {rep['max_step']:.3f} unpaid {rep['unpaid']}/{rep['departed']} score {rep['score']:.4f}{mark} "
                  f"({time.time() - t0:.0f} s)", flush=True)
    _, m.M, m.c, it_best = best
    rep = ev(m, a.beta, a.gamma, a.margin, a.d0)
    keys = ["step", "loss", "mean_dI", "min_dI", "max_dI", "hinge", "degraded", "degraded_any", "infeasible", "jac2",
            "jac_max", "max_step", "tv", "unpaid", "departed", "score"]
    with open(os.path.join(OUT, f"train_e2e_b{tag}_hist.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in hist:
            w.writerow([f"{float(r[k]):.6g}" for k in keys])
    meta = {k: float(v) for k, v in rep.items()}
    meta.update(stage=f"b{tag}", init=a.init, src=a.src, beta=a.beta, gamma=a.gamma, beta_inf=a.beta_inf, lr=a.lr,
                batch=a.batch, steps=a.steps, best_step=it_best, seed=a.seed, margin=a.margin, d0=a.d0)
    m.to_json(os.path.join(OUT, f"model_smooth_reghdc_b{tag}.json"), **meta)
    print(f"\nbest step {it_best}:", {k: (round(v, 5) if isinstance(v, float) else v) for k, v in rep.items()})
    print(f"done in {time.time() - t0:.0f} s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("a", "b"))
    ap.add_argument("--beta", type=float, default=4.0, help="weight of the hinge on Ipk > Ipk_SPS")
    ap.add_argument("--gamma", type=float, default=1e-3, help="weight of mean ||dDelta/dx||^2")
    ap.add_argument("--seed", type=int, default=0)
    # (a)
    ap.add_argument("--lams", default="1e-3,2e-3,5e-3,1e-2,2e-2")
    ap.add_argument("--Ds", default="64,128,256")
    ap.add_argument("--sigmas", default="0.1,0.15,0.2,0.25,0.35")
    # alpha = 1e-5 scores the same (within 1e-4 A) but gives sum|M| ~ 900, which the int16 MCU form cannot afford
    ap.add_argument("--alphas", default="1e-4,1e-3")
    ap.add_argument("--taus", default="0.004")
    # (b)
    ap.add_argument("--init", default="a", choices=("a", "sps"))
    ap.add_argument("--src", default="a", help="stage-(a) model to start from / copy the encoder of: a, a_D128, ...")
    ap.add_argument("--sps-init-y", type=float, default=-3.0, help="initial pre-activation / tau for --init sps")
    ap.add_argument("--tag", default="")
    ap.add_argument("--beta-inf", type=float, default=10.0)
    ap.add_argument("--margin", type=float, default=0.0, help="T7 SPS-departure cost [A] (0: off, as b1-b7)")
    ap.add_argument("--d0", type=float, default=0.005, help="departure scale in Delta1 + Delta2")
    # Adam moves all 2D read-out weights at once, so Delta moves by ~ lr * sqrt(2 D): 2e-3 overshoots by 0.4 A
    ap.add_argument("--lr", type=float, default=2e-4)
    ap.add_argument("--lr-min", type=float, default=2e-6)
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--batch", type=int, default=4096)
    ap.add_argument("--steps", type=int, default=1200)
    ap.add_argument("--eval-every", type=int, default=50)
    a = ap.parse_args()
    stage_a(a) if a.stage == "a" else stage_b(a)


if __name__ == "__main__":
    main()
