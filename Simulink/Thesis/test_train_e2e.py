"""
test_train_e2e.py -- checks of the gradients used by train_e2e.py stage (b).
  1. implicit-function dIpk/dD at the PI steady state  ==  finite difference with phi re-solved
  2. loss_grad wrt the read-out M  ==  finite difference of the batch loss (exact for gamma = 0; with gamma > 0
     the gate is held fixed in the smoothness term, so only approximate)
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import train_e2e as te  # noqa: E402
from smooth_surface_v2 import bilinear, dense_grid  # noqa: E402


def check(name, cond, msg):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {msg}")
    return bool(cond)


def main():
    S = np.load(os.path.join(HERE, "out", "surface_v2.npz"))
    _, _, DP, DV = dense_grid()
    T1 = bilinear(S["smooth_D1"], S["P"], S["V2"], DP.ravel(), DV.ravel())
    T2 = bilinear(S["smooth_D2"], S["P"], S["V2"], DP.ravel(), DV.ravel())
    m = ct.SmoothRegHDC(D=128, sigma=0.25).fit_ridge(DP.ravel(), DV.ravel(), T1, T2, alpha=1e-4)
    P, V = te.random_points(2000, np.random.default_rng(3))
    d1, d2 = m.predict(P, V)
    r = ps.operating_point(P, d1, d2, V)
    g, good = te.ift_grad(r["phi"], d1, d2, V)
    ok = True
    h = 1e-6
    for j in range(2):
        e = np.zeros(2)
        e[j] = h
        r1 = ps.operating_point(P, d1 - e[0], d2 - e[1], V)
        fd = (r["Ipk"] - r1["Ipk"]) / h
        sel = good & r["ok"] & r1["ok"]
        err = np.abs(fd - g[:, j])[sel]
        bad = err > 1e-2 * np.abs(g[sel, j]) + 1e-3      # the reference's own bisection noise is ~1e-4 at h = 1e-6
        ok &= check(f"IFT dIpk/dD{j + 1} = re-solved FD", bad.mean() < 0.01,
                    f"median |g| {np.median(np.abs(g[sel, j])):.3f} A/unit, median err {np.median(err):.2e}, "
                    f"{bad.sum()} of {sel.sum()} off (kinks), {(~good).sum()} tangential skipped")
    sps = ps.sps_reference(P, V)["Ipk"]
    # 3rd case: T7 SPS-departure cost (margin 25 mA, d0 0.005), gamma = 0 -> exact
    for gam, mar, tol in ((0.0, 0.0, 0.02), (1e-3, 0.0, 0.1), (0.0, 0.025, 0.02)):
        L, gM, gc, parts = te.loss_grad(m, P, V, sps, 4.0, gam, 10.0, mar)
        rel = []
        for i, o in ((3, 0), (17, 1), (60, 0), (100, 1), (127, 0)):
            hh = 1e-5
            M0 = m.M.copy()
            m.M[i, o] += hh
            Lp = te.loss_grad(m, P, V, sps, 4.0, gam, 10.0, mar)[0]
            m.M[i, o] -= 2 * hh
            Lm = te.loss_grad(m, P, V, sps, 4.0, gam, 10.0, mar)[0]
            m.M = M0
            fd = (Lp - Lm) / (2 * hh)
            rel.append(abs(fd - gM[i, o]) / max(abs(fd), 1e-3 * np.abs(gM).max()))
        ok &= check(f"loss_grad dL/dM = FD (gamma={gam:g}, margin={mar:g})", max(rel) < tol,
                    f"max rel err {max(rel):.3g} over 5 entries; parts {parts}")
    c, gd = te.departure(np.array([[0.0, 0.0], [0.01, 0.003], [0.2, 0.1]]), 0.025, 0.005)
    ok &= check("departure cost", abs(c[0]) < 1e-15 and abs(c[2] - 0.025) < 1e-12 and abs(gd[0, 0] - 5.0) < 1e-12,
                f"cost {c}, grad at 0 {gd[0]}")
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
