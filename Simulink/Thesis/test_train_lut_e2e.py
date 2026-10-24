"""
test_train_lut_e2e.py -- checks of train_lut_e2e.py (T13).
  1. TrainLUT.predict == GridLUT on the T1 surface; jacobian == finite difference inside cells
  2. lut_loss_grad wrt table entries == central finite difference of the batch loss (gamma = 0 and 1e-3),
     on entries strictly inside (D_MIN, 0.5) so that the projection / clip is not active
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
import train_lut_e2e as tl  # noqa: E402


def check(name, cond, msg):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {msg}")
    return bool(cond)


def main():
    S = np.load(os.path.join(HERE, "out", "surface_v2.npz"))
    m = tl.TrainLUT(S["P"], S["V2"], S["smooth_D1"].copy(), S["smooth_D2"].copy())
    ref = ct.GridLUT(S["P"], S["V2"], S["smooth_D1"], S["smooth_D2"])
    rng = np.random.default_rng(5)
    P, V = rng.uniform(0, ct.P_MAX, 3000), rng.uniform(ct.V2_LO, ct.V2_HI, 3000)
    ok = True
    a1, a2 = m.predict(P, V)
    b1, b2 = ref.predict(P, V)
    ok &= check("predict = GridLUT", max(np.abs(a1 - b1).max(), np.abs(a2 - b2).max()) < 1e-12, "max diff 0")
    J = m.jacobian(P, V)
    h = 1e-7
    du, dv = h * ct.P_MAX, h * (ct.V2_HI - ct.V2_LO)
    for j, (dp, dvv) in enumerate(((du, 0.0), (0.0, dv))):
        p1, p2 = m.predict(P + dp, V + dvv)
        m1, m2 = m.predict(P - dp, V - dvv)
        fd = -np.stack([(p1 - m1), (p2 - m2)], axis=1) / (2 * h)
        iv, ip, tp, tv = m.cell(P, V)
        inside = (tp > 1e-4) & (tp < 1 - 1e-4) & (tv > 1e-4) & (tv < 1 - 1e-4) & (a1 > ct.D_MIN) & (a2 > ct.D_MIN)
        err = np.abs(fd - J[:, j, :])[inside].max()
        ok &= check(f"jacobian d/d{'uv'[j]} = FD", err < 1e-5, f"max err {err:.2e} on {inside.sum()} interior points")
    sps = ps.sps_reference(P, V)["Ipk"]
    cand = np.argwhere((m.T1 > 0.15) & (m.T1 < 0.48))
    picks = [tuple(cand[i]) for i in np.linspace(0, len(cand) - 1, 6).astype(int)]
    for gam, tol in ((0.0, 0.02), (1e-3, 0.02)):
        L, (G1, G2), parts = tl.lut_loss_grad(m, P, V, sps, 20.0, gam, 10.0)
        rel = []
        for (iv, ip) in picks:
            for T, G in ((m.T1, G1), (m.T2, G2)):
                if not (0.15 < T[iv, ip] < 0.48):
                    continue
                hh = 1e-5
                T[iv, ip] += hh
                Lp = tl.lut_loss_grad(m, P, V, sps, 20.0, gam, 10.0)[0]
                T[iv, ip] -= 2 * hh
                Lm = tl.lut_loss_grad(m, P, V, sps, 20.0, gam, 10.0)[0]
                T[iv, ip] += hh
                fd = (Lp - Lm) / (2 * hh)
                rel.append(abs(fd - G[iv, ip]) / max(abs(fd), 1e-3 * max(np.abs(G1).max(), np.abs(G2).max())))
        ok &= check(f"lut_loss_grad dL/dT = FD (gamma={gam:g})", max(rel) < tol,
                    f"max rel err {max(rel):.3g} over {len(rel)} entries; parts {parts}")
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
