"""
train_lut_e2e.py -- T13: the physics-in-the-loop E2E fine-tuning of train_e2e.py (b), applied to the bilinear LUT
baseline, so that "proposed vs LUT" compares two parameterisations trained the same way, not a trained model with
an untrained table.

    python train_lut_e2e.py --tag L49 --stride 1 --init v2      # 49 x 22 table from the T1 surface (4328 B int16)
    python train_lut_e2e.py --tag L25 --stride 2 --init v2      # 25 x 11 (1116 B)
    python train_lut_e2e.py --tag L49s --stride 1 --init sps    # from SPS, no surface (as b5 for the RegHDC)

Same loss as train_e2e.loss_grad (PI steady-state Ipk - Ipk_SPS, hinge beta on worse-than-SPS, beta_inf on
infeasible, gamma * mean ||dDelta/dx||^2 in the normalised inputs), same implicit-function-theorem gradient
dIpk/dD, same validation set (4000 points, seed 101), same score and best-state selection, Adam with cosine decay.
The only LUT-specific parts: dD/dT are the bilinear weights of the 4 corner entries, the smoothness term is the
cell-wise constant gradient of the bilinear patch, and after each step the table is projected to [D_MIN, 0.5].
Batches are stratified (k points uniform in every cell) -- with 4096 iid points a 49 x 22 table gets ~4 points
per cell, and Adam's per-entry normalisation turns that sparsity into noise.

Output: out/model_lut_<tag>.npz, out/train_lut_<tag>_hist.csv.
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
from evaluate import LUT16  # noqa: E402
from train_e2e import Evaluator, ift_grad, pmax_grad  # noqa: E402

OUT = os.path.join(HERE, "out")


class TrainLUT(LUT16):
    """LUT16 with the cell lookup exposed, the Jacobian of Delta = 0.5 - D (normalised inputs) and npz I/O."""

    def cell(self, P_abs, V2):
        p = np.clip(np.asarray(P_abs, float).ravel(), self.P[0], self.P[-1])
        v = np.clip(np.asarray(V2, float).ravel(), self.V[0], self.V[-1])
        ip = np.clip(np.searchsorted(self.P, p) - 1, 0, self.P.size - 2)
        iv = np.clip(np.searchsorted(self.V, v) - 1, 0, self.V.size - 2)
        tp = (p - self.P[ip]) / (self.P[ip + 1] - self.P[ip])
        tv = (v - self.V[iv]) / (self.V[iv + 1] - self.V[iv])
        return iv, ip, tp, tv

    def _h(self, ip, iv):
        return ((self.P[ip + 1] - self.P[ip]) / ct.P_MAX, (self.V[iv + 1] - self.V[iv]) / (ct.V2_HI - ct.V2_LO))

    def jacobian(self, P_abs, V2):
        iv, ip, tp, tv = self.cell(P_abs, V2)
        hu, hv = self._h(ip, iv)
        J = np.zeros((iv.size, 2, 2))
        for o, T in enumerate((self.T1, self.T2)):
            a, b, c, d = T[iv, ip], T[iv, ip + 1], T[iv + 1, ip], T[iv + 1, ip + 1]
            J[:, 0, o] = -((b - a) * (1 - tv) + (d - c) * tv) / hu        # Delta = 0.5 - D
            J[:, 1, o] = -((c - a) * (1 - tp) + (d - b) * tp) / hv
        return J

    def save(self, path, **meta):
        np.savez(path, P=self.P, V=self.V, T1=self.T1, T2=self.T2, name=self.name,
                 meta_keys=np.array(list(meta.keys())), meta_vals=np.array([float(v) for v in meta.values()]))

    @classmethod
    def load(cls, path):
        z = np.load(path)
        m = cls(z["P"], z["V"], z["T1"], z["T2"], name=str(z["name"]))
        m.meta = dict(zip([str(k) for k in z["meta_keys"]], z["meta_vals"].tolist()))
        return m


def lut_loss_grad(m, P, V, sps, beta, gamma, beta_inf):
    """train_e2e.loss_grad for a table: loss and gradient wrt (T1, T2)."""
    N = P.size
    iv, ip, tp, tv = m.cell(P, V)
    raw1, raw2 = m._interp(m.T1, P, V), m._interp(m.T2, P, V)
    D1, D2 = np.clip(raw1, ct.D_MIN, 0.5), np.clip(raw2, ct.D_MIN, 0.5)
    r = ps.operating_point(P, D1, D2, V)
    ok = r["ok"]
    e = np.where(ok, r["Ipk"] - sps, 0.0)
    L_main = e.sum() / N
    L_deg = beta * np.maximum(e, 0).sum() / N
    gi, good = ift_grad(np.nan_to_num(r["phi"]), D1, D2, V)
    w = np.where(ok, (1.0 + beta * (e > 0)) / N, 0.0)
    dL_dD = w[:, None] * gi
    L_inf = 0.0
    if (~ok).any():
        bi = ~ok
        pm, dpm = pmax_grad(P[bi], D1[bi], D2[bi], V[bi])
        short = np.maximum(P[bi] - pm, 0.0) / 100.0
        L_inf = beta_inf * short.sum() / N
        dL_dD[bi] += -(beta_inf / 100.0 / N) * dpm * (short > 0)[:, None]
    # the lower clip blocks the gradient; the upper one (D = 0.5, SPS) is handled by the projection after the step
    dL_dD *= np.stack([raw1 >= ct.D_MIN, raw2 >= ct.D_MIN], axis=1)
    corners = ((iv, ip, (1 - tv) * (1 - tp)), (iv, ip + 1, (1 - tv) * tp),
               (iv + 1, ip, tv * (1 - tp)), (iv + 1, ip + 1, tv * tp))
    hu, hv = m._h(ip, iv)
    # d(Ju)/d(a, b, c, d) and d(Jv)/d(a, b, c, d) for D (the sign of Delta does not matter in the square)
    dJu = (-(1 - tv) / hu, (1 - tv) / hu, -tv / hu, tv / hu)
    dJv = (-(1 - tp) / hv, -tp / hv, (1 - tp) / hv, tp / hv)
    grads, L_s = [], 0.0
    for o, T in enumerate((m.T1, m.T2)):
        a, b, c, d = T[iv, ip], T[iv, ip + 1], T[iv + 1, ip], T[iv + 1, ip + 1]
        Ju = ((b - a) * (1 - tv) + (d - c) * tv) / hu
        Jv = ((c - a) * (1 - tp) + (d - b) * tp) / hv
        L_s += gamma * (Ju ** 2 + Jv ** 2).sum() / N
        G = np.zeros_like(T)
        for k, (ri, ci, wt) in enumerate(corners):
            np.add.at(G, (ri, ci), dL_dD[:, o] * wt + 2.0 * gamma / N * (Ju * dJu[k] + Jv * dJv[k]))
        grads.append(G)
    parts = dict(main=L_main, deg=L_deg, inf=L_inf, smooth=L_s, n_inf=int((~ok).sum()), n_skip=int((ok & ~good).sum()))
    return L_main + L_deg + L_inf + L_s, grads, parts


def stratified(m, k, rng):
    """k uniform points in every cell of the table."""
    nV, nP = m.V.size - 1, m.P.size - 1
    iv = np.repeat(np.arange(nV), nP * k)
    ip = np.tile(np.repeat(np.arange(nP), k), nV)
    P = m.P[ip] + rng.uniform(0, 1, ip.size) * (m.P[ip + 1] - m.P[ip])
    V = m.V[iv] + rng.uniform(0, 1, iv.size) * (m.V[iv + 1] - m.V[iv])
    return P, V


def initial(a):
    S = np.load(os.path.join(OUT, "surface_v2.npz"))
    gP, gV = S["P"][::a.stride], S["V2"][::a.stride]
    if a.init == "v2":
        T1, T2 = S["smooth_D1"][::a.stride, ::a.stride], S["smooth_D2"][::a.stride, ::a.stride]
    else:
        T1, T2 = np.full((gV.size, gP.size), 0.5), np.full((gV.size, gP.size), 0.5)
    return TrainLUT(gP, gV, T1.copy(), T2.copy(), name=f"LUT {gP.size}x{gV.size} E2E ({a.tag})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--stride", type=int, default=1, help="1: 49 x 22 (T1 grid), 2: 25 x 11")
    ap.add_argument("--init", default="v2", choices=("v2", "sps"))
    ap.add_argument("--beta", type=float, default=20.0)       # = b3 (the selected proposed model)
    ap.add_argument("--gamma", type=float, default=1e-3)
    ap.add_argument("--beta-inf", type=float, default=10.0)
    ap.add_argument("--lr", type=float, default=2e-4)         # per table entry, in units of D
    ap.add_argument("--lr-min", type=float, default=2e-6)
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--k", type=int, default=0, help="points per cell (0: ~4096 per batch)")
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--eval-every", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    t0 = time.time()
    m = initial(a)
    k = a.k or max(1, int(round(4096 / ((m.P.size - 1) * (m.V.size - 1)))))
    ev = Evaluator()
    rng = np.random.default_rng(a.seed + 7)
    th = [m.T1, m.T2]
    mom = [np.zeros_like(x) for x in th]
    vel = [np.zeros_like(x) for x in th]
    b1, b2, eps = 0.9, 0.999, 1e-12
    rep = ev(m, a.beta, a.gamma)
    rep.update(step=0, loss=np.nan)
    hist = [rep]
    best = (rep["score"], m.T1.copy(), m.T2.copy(), 0)
    print(f"{m.name}: init={a.init} beta={a.beta} gamma={a.gamma} lr={a.lr} k={k}/cell "
          f"(batch {k * (m.P.size - 1) * (m.V.size - 1)}) steps={a.steps}", flush=True)
    print(f"  step     0: val dI mean {rep['mean_dI']:.4f} min {rep['min_dI'] * 1e3:+.1f} mA deg {rep['degraded']} "
          f"inf {rep['infeasible']} jac2 {rep['jac2']:.2f} max {rep['jac_max']:.2f} step {rep['max_step']:.3f} "
          f"score {rep['score']:.4f}", flush=True)
    run = []
    for it in range(1, a.steps + 1):
        P, V = stratified(m, k, rng)
        sps = ps.sps_reference(P, V)["Ipk"]
        L, grads, parts = lut_loss_grad(m, P, V, sps, a.beta, a.gamma, a.beta_inf)
        run.append([L, parts["main"], parts["deg"], parts["smooth"], parts["n_inf"], parts["n_skip"]])
        lr = (a.lr_min + 0.5 * (a.lr - a.lr_min) * (1 + np.cos(np.pi * it / a.steps))) * min(1.0, it / a.warmup)
        for x, gx, mo, ve in zip(th, grads, mom, vel):
            mo *= b1
            mo += (1 - b1) * gx
            ve *= b2
            ve += (1 - b2) * gx * gx
            x -= lr * (mo / (1 - b1 ** it)) / (np.sqrt(ve / (1 - b2 ** it)) + eps)
            np.clip(x, ct.D_MIN, 0.5, out=x)
        if it % a.eval_every == 0 or it == a.steps:
            rep = ev(m, a.beta, a.gamma)
            ra = np.mean(run, axis=0)
            run = []
            rep.update(step=it, loss=float(ra[0]))
            hist.append(rep)
            mark = ""
            if rep["score"] > best[0]:
                best = (rep["score"], m.T1.copy(), m.T2.copy(), it)
                mark = " *"
            print(f"  step {it:5d}: loss {ra[0]:+.4f} (main {ra[1]:+.4f} deg {ra[2]:.4f} smooth {ra[3]:.4f} "
                  f"inf {ra[4]:.1f}) | val dI mean {rep['mean_dI']:.4f} min {rep['min_dI'] * 1e3:+.1f} mA deg "
                  f"{rep['degraded']} inf {rep['infeasible']} jac2 {rep['jac2']:.2f} step {rep['max_step']:.3f} "
                  f"score {rep['score']:.4f}{mark} ({time.time() - t0:.0f} s)", flush=True)
    _, T1, T2, it_best = best
    m.T1[...], m.T2[...] = T1, T2
    rep = ev(m, a.beta, a.gamma)
    keys = ["step", "loss", "mean_dI", "min_dI", "max_dI", "hinge", "degraded", "degraded_any", "infeasible", "jac2",
            "jac_max", "max_step", "tv", "score"]
    with open(os.path.join(OUT, f"train_lut_{a.tag}_hist.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in hist:
            w.writerow([f"{float(r[k]):.6g}" for k in keys])
    meta = {k: float(v) for k, v in rep.items()}
    meta.update(beta=a.beta, gamma=a.gamma, beta_inf=a.beta_inf, lr=a.lr, k=k, steps=a.steps, best_step=it_best,
                stride=a.stride, init_sps=float(a.init == "sps"))
    m.save(os.path.join(OUT, f"model_lut_{a.tag}.npz"), **meta)
    print(f"\nbest step {it_best}:", {kk: (round(v, 5) if isinstance(v, float) else v) for kk, v in rep.items()})
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
