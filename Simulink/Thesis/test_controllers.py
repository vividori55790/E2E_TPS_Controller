"""Self-checks for pi_steady.py and controllers.py (T2). Run: python -I -X utf8 Simulink/Thesis/test_controllers.py"""
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import optimal_surface as osf  # noqa: E402
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + ("  " + detail if detail else ""))
    return bool(ok)


def main():
    rng = np.random.default_rng(7)
    ok = True

    # ---------------------------------------------------------------- pi_steady
    M = 2000
    P = rng.uniform(0, 1200, M)
    V2 = rng.uniform(84, 109.2, M)
    s = ps.sps_reference(P, V2)
    r = ps.operating_point(P, 0.5, 0.5, V2)
    e_phi, e_ipk = np.nanmax(np.abs(r["phi"] - s["phi"])), np.nanmax(np.abs(r["Ipk"] - s["Ipk"]))
    ok &= check("SPS through the PI root finder = closed-form SPS", r["ok"].all() and e_phi < 1e-6 and e_ipk < 1e-6,
                f"max |dphi| {e_phi:.1e} deg, max |dIpk| {e_ipk:.1e} A")

    D1 = rng.uniform(0.1, 0.5, M)
    D2 = rng.uniform(0.1, 0.5, M)
    t0 = time.time()
    r = ps.operating_point(P, D1, D2, V2)
    t_vec = time.time() - t0
    Pdel = -tm.evaluate(r["D0"][r["ok"]], D1[r["ok"]], D2[r["ok"]], V2[r["ok"]])["P"]
    e_p = np.abs(Pdel - P[r["ok"]]).max()
    ok &= check("PI operating point delivers |P*| exactly", e_p < 1e-3, f"max |dP| {e_p:.1e} W over {r['ok'].sum()} feasible")
    ok &= check("PI operating point has P < 0 branch sign (phi_c >= 0)", np.all(r["phi"][r["ok"]] >= 0))

    # independent scalar reference (scan 0.5 deg + brentq) on a subset
    n_ref = 400
    ref = np.array([osf.point_metrics(P[m], D1[m], D2[m], V2[m]) for m in range(n_ref)])
    feas_ref = ~np.isnan(ref[:, 0])
    same_feas = feas_ref == r["ok"][:n_ref]
    both = feas_ref & r["ok"][:n_ref]
    d_phi = np.abs(ref[both, 0] - r["phi"][:n_ref][both])
    d_ipk = np.abs(ref[both, 1] - r["Ipk"][:n_ref][both])
    ok &= check("feasibility agrees with the scalar brentq reference", same_feas.mean() >= 0.995,
                f"{(~same_feas).sum()} / {n_ref} differ")
    ok &= check("first crossing agrees with the scalar reference", np.quantile(d_phi, 0.99) < 1e-5 and
                np.quantile(d_ipk, 0.99) < 1e-5, f"99% |dphi| {np.quantile(d_phi, 0.99):.1e} deg, max {d_phi.max():.2e}; "
                f"max |dIpk| {d_ipk.max():.2e} A ({both.sum()} pts, vectorised {M} pts in {t_vec:.2f} s)")

    r = ps.operating_point(np.array([1200.0, 50.0]), 0.1, 0.1, 95.0)
    ok &= check("narrow pulses cannot deliver 1.2 kW (flagged infeasible)", (not r["ok"][0]) and r["ok"][1]
                and r["Pmax"][0] < 1200, f"Pmax {r['Pmax'][0]:.0f} W")
    r = ps.operating_point(np.zeros(3), np.array([0.5, 0.3, 0.2]), np.array([0.5, 0.3, 0.4]), 90.0)
    ok &= check("|P*| = 0 settles at phi_c = 0", np.allclose(r["phi"], 0.0) and r["ok"].all())

    # ---------------------------------------------------------------- SPS controller
    sps = ct.SPS()
    d1, d2 = sps.predict(P, V2)
    ok &= check("SPS controller outputs 0.5 / 0.5", np.all(d1 == 0.5) and np.all(d2 == 0.5) and d1.shape == P.shape)

    # ---------------------------------------------------------------- GridLUT on surface v2
    S = np.load(os.path.join(HERE, "out", "surface_v2.npz"))
    Pa, Va, T1, T2 = S["P"], S["V2"], S["smooth_D1"], S["smooth_D2"]
    lut = ct.GridLUT(Pa, Va, T1, T2, name="LUT v2")
    PP, VV = np.meshgrid(Pa, Va)
    g1, g2 = lut.predict(PP, VV)
    ok &= check("GridLUT reproduces the table at the nodes", np.abs(g1 - T1).max() < 1e-12 and np.abs(g2 - T2).max() < 1e-12)
    Pm, Vm = 0.5 * (Pa[:-1] + Pa[1:]), 0.5 * (Va[:-1] + Va[1:])
    PM, VM = np.meshgrid(Pm, Vm)
    m1, _ = lut.predict(PM, VM)
    avg = 0.25 * (T1[:-1, :-1] + T1[:-1, 1:] + T1[1:, :-1] + T1[1:, 1:])
    ok &= check("GridLUT is bilinear at cell centres", np.abs(m1 - avg).max() < 1e-12)
    c1, _ = lut.predict(np.array([-50.0, 5000.0]), np.array([70.0, 130.0]))
    ok &= check("GridLUT clamps outside the table", np.allclose(c1, [T1[0, 0], T1[-1, -1]]))
    r = ps.operating_point(PP.ravel(), g1.ravel(), g2.ravel(), VV.ravel())
    dI = (ps.sps_reference(PP.ravel(), VV.ravel())["Ipk"] - r["Ipk"]).reshape(PP.shape)
    ok &= check("LUT closed loop = stored surface metrics", np.nanmax(np.abs(dI - S["smooth_dI"])) < 1e-9
                and np.nanmin(dI) > -1e-6, f"mean dI {dI.mean():.4f} A, min {dI.min() * 1e3:+.3f} mA")

    # ---------------------------------------------------------------- LegacyRegHDC
    Psig = np.concatenate([PP.ravel(), -PP.ravel()])
    Vsig = np.concatenate([VV.ravel(), VV.ravel()])
    y1 = np.concatenate([T1.ravel(), T1.ravel()])
    y2 = np.concatenate([T2.ravel(), T2.ravel()])
    t0 = time.time()
    leg = ct.LegacyRegHDC(D=32, K=8, Q=32, gamma=8.0, seed=0).fit(Psig, Vsig, y1, y2, epochs=40, patience=40)
    t_fit = time.time() - t0
    base = np.sqrt(np.mean((np.concatenate([y1, y2]) - np.concatenate([y1, y2]).mean()) ** 2))
    p1, p2 = leg.predict(PP.ravel(), VV.ravel(), sign=+1.0)
    n1, n2 = leg.predict(PP.ravel(), VV.ravel(), sign=-1.0)
    rm = np.sqrt(np.mean(np.concatenate([p1 - T1.ravel(), p2 - T2.ravel()]) ** 2))
    ok &= check("LegacyRegHDC learns (val RMSE below the constant predictor)", leg.val_rmse < 0.7 * base,
                f"val RMSE {leg.val_rmse:.4f} vs constant {base:.4f}, train-grid RMSE {rm:.4f}, {t_fit:.1f} s / 40 ep")
    ok &= check("LegacyRegHDC outputs stay in [D_MIN, 0.5]", min(p1.min(), p2.min(), n1.min(), n2.min()) >= ct.D_MIN
                and max(p1.max(), p2.max(), n1.max(), n2.max()) <= 0.5)
    ok &= check("LegacyRegHDC memory (D=32, K=8, Q=32) = 4160 B as in RegHDCTrainer", leg.memory_bytes() == 4160,
                f"{leg.memory_bytes()} B")
    q_levels = np.unique(leg.encode(np.linspace(0, 1200, 2000), 95.0), axis=0).shape[0]
    ok &= check("LegacyRegHDC encoding is piecewise constant (quantised)", q_levels <= 17,
                f"{q_levels} distinct codes for |P| 0..1200 W (Q = 32 over -1200..1200)")

    # ---------------------------------------------------------------- SmoothRegHDC
    sm = ct.SmoothRegHDC(D=128, sigma=0.25, tau=0.004, seed=0).fit_ridge(PP.ravel(), VV.ravel(), T1.ravel(), T2.ravel(),
                                                                       alpha=1e-6)
    s1, s2 = sm.predict(PP.ravel(), VV.ravel())
    rs = np.sqrt(np.mean(np.concatenate([s1 - T1.ravel(), s2 - T2.ravel()]) ** 2))
    ok &= check("SmoothRegHDC ridge fits the surface", rs < 0.02, f"grid RMSE(D) {rs:.4f}")
    ok &= check("SmoothRegHDC outputs stay in [D_MIN, 0.5]", min(s1.min(), s2.min()) >= ct.D_MIN - 1e-12
                and max(s1.max(), s2.max()) <= 0.5)
    # Jacobian against central differences in the normalised inputs
    Pt = rng.uniform(50, 1150, 300)
    Vt = rng.uniform(85, 108, 300)
    J = sm.jacobian(Pt, Vt)
    h = 1e-6
    fd = np.zeros_like(J)
    for i, (dp, dv) in enumerate(((h * ct.P_MAX, 0.0), (0.0, h * (ct.V2_HI - ct.V2_LO)))):
        a1, a2 = sm.predict(Pt + dp, Vt + dv)
        b1, b2 = sm.predict(Pt - dp, Vt - dv)
        fd[:, i, 0] = -(a1 - b1) / (2 * h)
        fd[:, i, 1] = -(a2 - b2) / (2 * h)
    e_j = np.abs(J - fd).max()
    ok &= check("SmoothRegHDC jacobian = finite differences", e_j < 1e-4 * max(1.0, np.abs(fd).max()),
                f"max err {e_j:.2e}, max |J| {np.abs(fd).max():.2f}")
    smq = ct.SmoothRegHDCQ(sm, bits=16, cos_table=1024)
    q1, q2 = smq.predict(PP.ravel(), VV.ravel())
    e_q = max(np.abs(q1 - s1).max(), np.abs(q2 - s2).max())
    ok &= check("int16 + 1024-entry cos table stays close to float", e_q < 5e-3,
                f"max |dD| {e_q:.2e}, memory {smq.memory_bytes()} B vs float {sm.memory_bytes()} B")
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
