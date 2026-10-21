"""
pointwise_random.py -- reference optimum (D1, D2) at arbitrary (|P|, V2) points, for the evaluation sets of T5.

The grid landscape of optimal_surface.py shares one V2 per row, so it does not carry over to random points
(6561 lattice candidates x 361 phases per point). Here: a vectorised multi-start pattern search on
(D1, D2) in [D_MIN, 0.5]^2, all points at once, with the PI steady state solved exactly (pi_steady):
8 compass + diagonal moves, step 0.03 halved on failure down to 4e-4.
Starts per point: SPS, the exact optima of the 4 surrounding grid nodes (surface_v1.npz, Nelder-Mead refined)
and their bilinear blend; the best result over all starts is kept.

  --check N : compare with brute force (0.005 lattice + Nelder-Mead, as optimal_surface.py) on N random points.

Sets: val (seed 101, 4000; the validation set of train_e2e.py), test (seed 2026, 10000), dense (|P| 10 W x V2 0.4 V).
Output out/pointwise_<set>.npz: P, V2, D1, D2, Ipk, Irms, phi, Ipk_sps, Irms_sps.
"""
import argparse
import os
import sys
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
from smooth_surface_v2 import bilinear, dense_grid  # noqa: E402

OUT = os.path.join(HERE, "out")
DIRS = np.array([(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)], float)
SETS = {"val": (101, 4000), "test": (2026, 10000)}


def point_set(name):
    if name == "dense":
        _, _, DP, DV = dense_grid()
        return DP.ravel(), DV.ravel()
    seed, n = SETS[name]
    rng = np.random.default_rng(seed)
    return rng.uniform(0.0, ct.P_MAX, n), rng.uniform(ct.V2_LO, ct.V2_HI, n)


def ipk_at(P, V, X):
    r = ps.operating_point(P, X[:, 0], X[:, 1], V)
    return np.where(r["ok"], r["Ipk"], np.inf)


def pattern_search(P, V, x0, s0=0.03, s_min=4e-4, max_iter=80):
    x = np.clip(x0, ct.D_MIN, 0.5)
    f = ipk_at(P, V, x)
    s = np.full(P.size, s0)
    for _ in range(max_iter):
        idx = np.flatnonzero(s >= s_min)
        if idx.size == 0:
            break
        cand = np.clip(x[idx, None, :] + s[idx, None, None] * DIRS[None], ct.D_MIN, 0.5)       # (n, 8, 2)
        fc = ipk_at(np.repeat(P[idx], 8), np.repeat(V[idx], 8), cand.reshape(-1, 2)).reshape(idx.size, 8)
        j = np.argmin(fc, axis=1)
        fb = fc[np.arange(idx.size), j]
        imp = fb < f[idx] - 1e-9
        x[idx[imp]] = cand[imp, j[imp]]
        f[idx[imp]] = fb[imp]
        s[idx[~imp]] *= 0.5
    return x, f


def starts(P, V):
    S = np.load(os.path.join(OUT, "surface_v1.npz"))
    gP, gV, O1, O2 = S["P"], S["V2"], S["opt_D1"], S["opt_D2"]
    ip = np.clip(np.searchsorted(gP, P) - 1, 0, gP.size - 2)
    iv = np.clip(np.searchsorted(gV, V) - 1, 0, gV.size - 2)
    out = [np.stack([np.full(P.size, 0.5), np.full(P.size, 0.5)], 1),
           np.stack([bilinear(O1, gP, gV, P, V), bilinear(O2, gP, gV, P, V)], 1)]
    for dv in (0, 1):
        for dp in (0, 1):
            out.append(np.stack([O1[iv + dv, ip + dp], O2[iv + dv, ip + dp]], 1))
    return out


def solve_chunk(args):
    P, V = args
    best_x, best_f = None, None
    for x0 in starts(P, V):
        x, f = pattern_search(P, V, x0)
        if best_x is None:
            best_x, best_f = x, f
        else:
            b = f < best_f
            best_x[b], best_f[b] = x[b], f[b]
    return best_x, best_f


def solve(P, V, procs=6, chunk=500):
    parts = [(P[i:i + chunk], V[i:i + chunk]) for i in range(0, P.size, chunk)]
    if procs > 1:
        with Pool(procs) as pool:
            res = pool.map(solve_chunk, parts)
    else:
        res = [solve_chunk(p) for p in parts]
    return np.concatenate([r[0] for r in res]), np.concatenate([r[1] for r in res])


def brute(P, V):
    """Lattice argmin + Nelder-Mead, exactly as optimal_surface.py does on the grid."""
    import optimal_surface as osf
    dv = np.round(np.arange(ct.D_MIN, 0.5 + 1e-9, 0.005), 6)
    D1g, D2g = [g.ravel() for g in np.meshgrid(dv, dv, indexing="ij")]
    out = np.zeros(P.size)
    for i in range(P.size):
        f = ipk_at(np.full(D1g.size, P[i]), np.full(D1g.size, V[i]), np.stack([D1g, D2g], 1))
        c = int(np.argmin(f))
        _, fr = osf.refine(P[i], V[i], np.array([D1g[c], D2g[c]]))
        out[i] = min(f[c], fr)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sets", default="val,test,dense")
    ap.add_argument("--procs", type=int, default=6)
    ap.add_argument("--check", type=int, default=0)
    a = ap.parse_args()
    t0 = time.time()
    if a.check:
        rng = np.random.default_rng(7)
        P, V = rng.uniform(0, ct.P_MAX, a.check), rng.uniform(ct.V2_LO, ct.V2_HI, a.check)
        x, f = solve(P, V, procs=1)
        fb = brute(P, V)
        d = f - fb
        sps = ps.sps_reference(P, V)["Ipk"]
        print(f"check on {a.check} random points ({time.time() - t0:.0f} s): pattern - brute force Ipk: "
              f"mean {d.mean() * 1e3:+.3f} mA, max {d.max() * 1e3:+.3f} mA, min {d.min() * 1e3:+.3f} mA; "
              f"mean gain vs SPS {np.mean(sps - f):.4f} (pattern) / {np.mean(sps - fb):.4f} A (brute)")
        for i in np.argsort(-d)[:5]:
            print(f"   P {P[i]:7.1f} V2 {V[i]:6.2f}: pattern {f[i]:.4f} brute {fb[i]:.4f} SPS {sps[i]:.4f}  D {x[i]}")
        return
    for name in a.sets.split(","):
        P, V = point_set(name)
        x, f = solve(P, V, procs=a.procs)
        r = ps.operating_point(P, x[:, 0], x[:, 1], V)
        s = ps.sps_reference(P, V)
        np.savez_compressed(os.path.join(OUT, f"pointwise_{name}.npz"), P=P, V2=V, D1=x[:, 0], D2=x[:, 1],
                            Ipk=r["Ipk"], Irms=r["Irms"], phi=r["phi"], Ipk_sps=s["Ipk"], Irms_sps=s["Irms"])
        dI = s["Ipk"] - r["Ipk"]
        print(f"{name}: {P.size} points, infeasible {int((~r['ok']).sum())}, mean gain vs SPS {np.nanmean(dI):.4f} A, "
              f"max {np.nanmax(dI):.4f}, min {np.nanmin(dI) * 1e3:+.3f} mA ({time.time() - t0:.0f} s)", flush=True)


if __name__ == "__main__":
    main()
