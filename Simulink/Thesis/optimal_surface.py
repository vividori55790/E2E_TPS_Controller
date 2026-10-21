"""
optimal_surface.py -- Phase 2: the optimal TPS control surface on the exact model.

Controller contract (agreed with the user): the regression model outputs (D1, D2) from (P*, V2);
D0 is closed by the PI loop. For fixed (D1, D2) and a fixed branch the D0 that delivers P* is unique,
so the operating point the PI settles on is the FIRST crossing of P*(phi_c) from phi_c = 0, where
phi_c = D0 + 180 (D2 - D1) is the centre-to-centre phase. By mirror symmetry the optimal (D1, D2)
depends on |P| only (checked in test_tps_model.py).

Steps
  1. landscape: for every (V2, |P|) on the grid and every (D1, D2) on a 0.005 lattice, the exact
     Ipk / Irms at the PI operating point (or infeasible).
  2. pointwise optimum: lattice argmin, then continuous refinement (Nelder-Mead on (D1, D2) with an
     exact root solve for phi_c).
  3. smooth surface: per point the admissible set is {Ipk <= Ipk_opt + eps, Ipk <= Ipk_SPS}; choose
     from it the field that minimises roughness (sum of squared neighbour differences) plus a small
     RMS-current tie-breaker, by iterated conditional modes.

Outputs go to Simulink/Thesis/out/.
"""
import argparse
import os
import sys
import time

import numpy as np
from scipy.optimize import brentq, minimize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")

D_MIN, D_MAX = 0.10, 0.50
PHI_MAX = 90.0          # |phi_c| bound of the PI branch (deg)


# ------------------------------------------------------------------ helpers
def sps_point(P_abs, V2, V1=tm.V1_NOM):
    """Exact SPS operating point for |P| (phi_c in deg, Ipk, Irms); NaN beyond the SPS limit."""
    K = tm.N_TURNS * V1 * V2 / (2.0 * tm.FS * tm.L_IND)
    disc = 1.0 - 4.0 * np.asarray(P_abs, float) / K
    d = np.where(disc >= 0, 0.5 * (1.0 - np.sqrt(np.clip(disc, 0, None))), np.nan)
    phi = 180.0 * d
    r = tm.evaluate(phi, 0.5, 0.5, V2, V1)          # phi > 0 -> P < 0; |P| identical by symmetry
    return phi, r["Ipk"], r["Irms"]


def d0_from_phi(phi_c, D1, D2):
    return phi_c - 180.0 * (D2 - D1)


def p_abs_at(phi_c, D1, D2, V2, V1=tm.V1_NOM):
    """|P| delivered at centre phase phi_c >= 0 (P is negative there in the dataset convention)."""
    return -tm.evaluate(d0_from_phi(phi_c, D1, D2), D1, D2, V2, V1)["P"]


def solve_phi(P_abs, D1, D2, V2, V1=tm.V1_NOM, phi_grid=None):
    """First crossing of |P|(phi_c) = P_abs on [0, PHI_MAX]; NaN when the branch cannot deliver it."""
    if P_abs <= 0:
        return 0.0
    g = phi_grid if phi_grid is not None else np.linspace(0, PHI_MAX, 181)
    p = p_abs_at(g, D1, D2, V2, V1)
    idx = np.flatnonzero(p >= P_abs)
    if idx.size == 0:
        return np.nan
    j = idx[0]
    if j == 0:
        return 0.0
    return brentq(lambda x: p_abs_at(x, D1, D2, V2, V1) - P_abs, g[j - 1], g[j], xtol=1e-9)


def point_metrics(P_abs, D1, D2, V2, V1=tm.V1_NOM):
    phi = solve_phi(P_abs, D1, D2, V2, V1)
    if np.isnan(phi):
        return np.nan, np.nan, np.nan
    r = tm.evaluate(d0_from_phi(phi, D1, D2), D1, D2, V2, V1)
    return phi, float(r["Ipk"]), float(r["Irms"])


# ------------------------------------------------------------------ 1. landscape
def landscape(P_list, V2_list, d_step=0.005, phi_step=0.25, V1=tm.V1_NOM, verbose=True):
    dv = np.round(np.arange(D_MIN, D_MAX + 1e-9, d_step), 6)
    D1g, D2g = np.meshgrid(dv, dv, indexing="ij")
    D1f, D2f = D1g.ravel(), D2g.ravel()
    C = D1f.size
    phis = np.arange(0.0, PHI_MAX + 1e-9, phi_step)
    nP, nV = len(P_list), len(V2_list)
    Ipk = np.full((nV, nP, C), np.nan, np.float32)
    Irms = np.full((nV, nP, C), np.nan, np.float32)
    PHI = np.full((nV, nP, C), np.nan, np.float32)
    t0 = time.time()
    P_arr = np.asarray(P_list, float)
    for iv, V2 in enumerate(V2_list):
        for c0 in range(0, C, 1500):
            c1 = min(C, c0 + 1500)
            d1, d2 = D1f[c0:c1, None], D2f[c0:c1, None]
            r = tm.evaluate(d0_from_phi(phis[None, :], d1, d2), d1, d2, V2, V1)
            p = -r["P"]                                             # (c, phi) |P| on the branch phi >= 0
            pmax = np.maximum.accumulate(p, axis=1)                 # first crossing via running max
            for ip, Pt in enumerate(P_arr):
                if Pt <= 0:
                    j = np.zeros(c1 - c0, int)
                    ok = np.ones(c1 - c0, bool)
                    phi_s = np.zeros(c1 - c0)
                else:
                    ok = pmax[:, -1] >= Pt
                    j = np.argmax(pmax >= Pt, axis=1)
                    jm = np.maximum(j - 1, 0)
                    pa, pb = p[np.arange(c1 - c0), jm], p[np.arange(c1 - c0), j]
                    frac = np.where(pb > pa, (Pt - pa) / np.where(pb > pa, pb - pa, 1.0), 0.0)
                    phi_s = phis[jm] + np.clip(frac, 0, 1) * (phis[j] - phis[jm])
                rr = tm.evaluate(d0_from_phi(phi_s, d1[:, 0], d2[:, 0]), d1[:, 0], d2[:, 0], V2, V1)
                Ipk[iv, ip, c0:c1] = np.where(ok, rr["Ipk"], np.nan)
                Irms[iv, ip, c0:c1] = np.where(ok, rr["Irms"], np.nan)
                PHI[iv, ip, c0:c1] = np.where(ok, phi_s, np.nan)
        if verbose:
            print(f"  landscape V2={V2:6.2f}  ({iv + 1}/{nV}, {time.time() - t0:5.1f} s)", flush=True)
    return dict(dv=dv, D1=D1f, D2=D2f, Ipk=Ipk, Irms=Irms, PHI=PHI, P=P_arr, V2=np.asarray(V2_list, float))


# ------------------------------------------------------------------ 2. pointwise optimum
def refine(P_abs, V2, x0, V1=tm.V1_NOM):
    def f(x):
        d1, d2 = x
        if not (D_MIN <= d1 <= D_MAX and D_MIN <= d2 <= D_MAX):
            return 1e3
        _, ipk, _ = point_metrics(P_abs, d1, d2, V2, V1)
        return 1e3 if np.isnan(ipk) else ipk
    best = minimize(f, x0, method="Nelder-Mead", options=dict(xatol=1e-5, fatol=1e-7, maxiter=400))
    x = np.clip(best.x, D_MIN, D_MAX)
    return x, f(x)


def pointwise_optimum(L, V1=tm.V1_NOM, verbose=True):
    nV, nP, _ = L["Ipk"].shape
    res = dict(D1=np.zeros((nV, nP)), D2=np.zeros((nV, nP)), Ipk=np.zeros((nV, nP)), Irms=np.zeros((nV, nP)),
               phi=np.zeros((nV, nP)), Ipk_sps=np.zeros((nV, nP)), Irms_sps=np.zeros((nV, nP)),
               phi_sps=np.zeros((nV, nP)))
    t0 = time.time()
    for iv, V2 in enumerate(L["V2"]):
        for ip, Pt in enumerate(L["P"]):
            ps, isps, rsps = sps_point(Pt, V2, V1)
            res["Ipk_sps"][iv, ip], res["Irms_sps"][iv, ip], res["phi_sps"][iv, ip] = isps, rsps, ps
            ipk = L["Ipk"][iv, ip].astype(float)
            c = int(np.nanargmin(ipk))
            x0 = np.array([L["D1"][c], L["D2"][c]])
            x, f = refine(Pt, V2, x0, V1)
            if not f < ipk[c]:
                x, f = x0, ipk[c]
            # SPS is part of the search space; never report something worse than it
            if f > isps:
                x, f = np.array([0.5, 0.5]), isps
            phi, ipk_x, irms_x = point_metrics(Pt, x[0], x[1], V2, V1)
            res["D1"][iv, ip], res["D2"][iv, ip] = x
            res["Ipk"][iv, ip], res["Irms"][iv, ip], res["phi"][iv, ip] = ipk_x, irms_x, phi
        if verbose:
            print(f"  optimum V2={V2:6.2f}  ({iv + 1}/{nV}, {time.time() - t0:5.1f} s)", flush=True)
    return res


# ------------------------------------------------------------------ 3. smooth surface
def smooth_surface(L, opt, eps_abs=0.01, eps_rel=0.005, lam=1.0, mu=0.02, iters=60, verbose=True):
    """Iterated conditional modes over the admissible candidate sets.
    energy = sum_i [ mu * Irms_i / Irms_sps_i ] + lam * sum_<i,j> |D_i - D_j|^2 / h^2,
    neighbours on the (V2, |P|) grid, distances measured in units of the lattice spacing."""
    nV, nP, C = L["Ipk"].shape
    D1c, D2c = L["D1"], L["D2"]
    tol = np.maximum(eps_abs, eps_rel * opt["Ipk"])
    cand = []
    for iv in range(nV):
        row = []
        for ip in range(nP):
            ipk = L["Ipk"][iv, ip].astype(float)
            lim = np.minimum(opt["Ipk"][iv, ip] + tol[iv, ip], opt["Ipk_sps"][iv, ip] + 1e-9)
            idx = np.flatnonzero(ipk <= lim)
            if idx.size == 0:                                   # the lattice misses the refined optimum
                idx = np.array([int(np.nanargmin(ipk))])
            row.append(idx)
        cand.append(row)
    unary = [[mu * L["Irms"][iv, ip, cand[iv][ip]].astype(float) / opt["Irms_sps"][iv, ip]
              for ip in range(nP)] for iv in range(nV)]
    # initial field: the RMS tie-breaker alone
    sel = np.array([[cand[iv][ip][int(np.argmin(unary[iv][ip]))] for ip in range(nP)] for iv in range(nV)])
    # the lattice spacing in the two grid directions is normalised to one step
    for it in range(iters):
        changed = 0
        for iv in range(nV):
            for ip in range(nP):
                idx = cand[iv][ip]
                e = unary[iv][ip].copy()
                for jv, jp in ((iv - 1, ip), (iv + 1, ip), (iv, ip - 1), (iv, ip + 1)):
                    if 0 <= jv < nV and 0 <= jp < nP:
                        s = sel[jv, jp]
                        e += lam * ((D1c[idx] - D1c[s]) ** 2 + (D2c[idx] - D2c[s]) ** 2) / 0.005 ** 2 * 1e-3
                k = idx[int(np.argmin(e))]
                if k != sel[iv, ip]:
                    sel[iv, ip] = k
                    changed += 1
        if verbose:
            print(f"  ICM iter {it + 1}: {changed} changes", flush=True)
        if changed == 0:
            break
    out = dict(D1=D1c[sel], D2=D2c[sel], Ipk=np.take_along_axis(L["Ipk"], sel[..., None], 2)[..., 0].astype(float),
               Irms=np.take_along_axis(L["Irms"], sel[..., None], 2)[..., 0].astype(float),
               phi=np.take_along_axis(L["PHI"], sel[..., None], 2)[..., 0].astype(float))
    return out


def roughness(D1, D2):
    g = []
    for A in (D1, D2):
        g.append(np.abs(np.diff(A, axis=0)).max())
        g.append(np.abs(np.diff(A, axis=1)).max())
    tv = sum(np.abs(np.diff(A, axis=k)).sum() for A in (D1, D2) for k in (0, 1))
    return dict(max_step=max(g), total_variation=float(tv))


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--p-step", type=float, default=25.0)
    ap.add_argument("--p-max", type=float, default=1200.0)
    ap.add_argument("--v2-min", type=float, default=84.0)
    ap.add_argument("--v2-max", type=float, default=109.2)
    ap.add_argument("--v2-step", type=float, default=1.2)
    ap.add_argument("--d-step", type=float, default=0.005)
    ap.add_argument("--eps-abs", type=float, default=0.01)
    ap.add_argument("--eps-rel", type=float, default=0.005)
    ap.add_argument("--lam", type=float, default=1.0)
    ap.add_argument("--tag", default="v1")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    P_list = np.arange(0.0, a.p_max + 1e-9, a.p_step)
    V2_list = np.round(np.arange(a.v2_min, a.v2_max + 1e-9, a.v2_step), 4)
    cache = os.path.join(OUT, f"landscape_{a.tag}.npz")
    if os.path.exists(cache):
        print("landscape: cached", cache)
        L = dict(np.load(cache))
    else:
        print(f"landscape: {len(V2_list)} x {len(P_list)} operating points, D step {a.d_step}")
        L = landscape(P_list, V2_list, a.d_step)
        np.savez_compressed(cache, **L)
    opt = pointwise_optimum(L)
    sm = smooth_surface(L, opt, a.eps_abs, a.eps_rel, a.lam)
    np.savez_compressed(os.path.join(OUT, f"surface_{a.tag}.npz"), P=L["P"], V2=L["V2"],
                        **{f"opt_{k}": v for k, v in opt.items()}, **{f"smooth_{k}": v for k, v in sm.items()})
    # summary
    red_opt = opt["Ipk_sps"] - opt["Ipk"]
    red_sm = opt["Ipk_sps"] - sm["Ipk"]
    gap = sm["Ipk"] - opt["Ipk"]
    print("\n=== summary (|P| grid incl. 0 W; mirrored to negative power by symmetry) ===")
    print(f"pointwise optimum : mean dI {red_opt.mean():.3f} A, max {red_opt.max():.3f} A, "
          f"min {red_opt.min():.4f} A, roughness {roughness(opt['D1'], opt['D2'])}")
    print(f"smooth surface    : mean dI {red_sm.mean():.3f} A, max {red_sm.max():.3f} A, "
          f"min {red_sm.min():.4f} A, gap to opt mean {gap.mean():.4f} / max {gap.max():.4f} A, "
          f"roughness {roughness(sm['D1'], sm['D2'])}")


if __name__ == "__main__":
    main()
