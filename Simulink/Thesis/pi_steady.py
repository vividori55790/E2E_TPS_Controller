"""
pi_steady.py -- where the D0 PI loop settles, vectorised.

For a controller that sets (D1, D2) and lets a PI loop close D0 on the power error, the steady state
is the first crossing of |P|(phi_c) = |P*| on the branch phi_c in [0, PHI_MAX] (phi_c = centre phase).
This module finds it for many samples at once: a coarse scan brackets the first crossing, bisection
finishes it. Samples whose branch cannot deliver |P*| are flagged infeasible (the PI would saturate).
"""
import numpy as np

import tps_model as tm

PHI_MAX = 90.0


def _p_abs(phi, D1, D2, V2, V1):
    return -tm.evaluate(phi - 180.0 * (D2 - D1), D1, D2, V2, V1)["P"]


def operating_point(P_abs, D1, D2, V2, V1=tm.V1_NOM, scan_step=1.0, iters=30):
    """Returns dict(phi, D0, Ipk, Irms, ok, Pmax) for arrays of equal shape (1-D)."""
    P_abs, D1, D2, V2, V1 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(P_abs, D1, D2, V2, V1)]
    M = P_abs.size
    grid = np.arange(0.0, PHI_MAX + 1e-9, scan_step)
    p = _p_abs(grid[None, :], D1[:, None], D2[:, None], V2[:, None], V1[:, None])   # (M, G)
    pm = np.maximum.accumulate(p, axis=1)
    ok = pm[:, -1] >= P_abs - 1e-9
    j = np.argmax(pm >= P_abs[:, None], axis=1)
    j = np.where(ok, j, grid.size - 1)
    lo = grid[np.maximum(j - 1, 0)]
    hi = grid[j]
    zero = P_abs <= 0
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        f = _p_abs(mid, D1, D2, V2, V1) >= P_abs
        hi = np.where(f, mid, hi)
        lo = np.where(f, lo, mid)
    phi = np.where(zero, 0.0, hi)
    phi = np.where(ok, phi, np.nan)
    D0 = phi - 180.0 * (D2 - D1)
    r = tm.evaluate(np.nan_to_num(D0), D1, D2, V2, V1)
    return dict(phi=phi, D0=D0, Ipk=np.where(ok, r["Ipk"], np.nan), Irms=np.where(ok, r["Irms"], np.nan),
                ok=ok, Pmax=pm[:, -1])


def sps_reference(P_abs, V2, V1=tm.V1_NOM):
    """Exact SPS operating point (closed form) for comparison."""
    P_abs, V2, V1 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(P_abs, V2, V1)]
    K = tm.N_TURNS * V1 * V2 / (2.0 * tm.FS * tm.L_IND)
    disc = 1.0 - 4.0 * P_abs / K
    d = np.where(disc >= 0, 0.5 * (1.0 - np.sqrt(np.clip(disc, 0, None))), np.nan)
    phi = 180.0 * d
    r = tm.evaluate(phi, 0.5, 0.5, V2, V1)
    return dict(phi=phi, Ipk=r["Ipk"], Irms=r["Irms"])
