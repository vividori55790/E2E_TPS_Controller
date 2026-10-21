"""Self-checks for tps_model.py. Run: python -I Simulink/Thesis/test_tps_model.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tps_model as tm  # noqa: E402


def check(name, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + name + ("" if ok else "  " + detail))
    return ok


def main():
    rng = np.random.default_rng(1)
    M = 300
    D0 = rng.uniform(-90, 90, M)
    D1 = rng.uniform(0.05, 0.5, M)
    D2 = rng.uniform(0.05, 0.5, M)
    V2 = rng.uniform(80, 112, M)
    r = tm.evaluate(D0, D1, D2, V2)
    Pb, Ib, Rb = np.empty(M), np.empty(M), np.empty(M)
    for m in range(M):
        th, vp, vs, i = tm.waveform(D0[m], D1[m], D2[m], V2[m], N=36000)
        Pb[m] = -np.mean(vp * i)
        Ib[m] = np.abs(i).max()
        Rb[m] = np.sqrt(np.mean(i * i))
    ok = True
    ok &= check("P matches brute force", np.allclose(r["P"], Pb, atol=0.5), f"max err {np.abs(r['P'] - Pb).max():.3f} W")
    ok &= check("Ipk matches brute force", np.allclose(r["Ipk"], Ib, atol=2e-3), f"max err {np.abs(r['Ipk'] - Ib).max():.4f} A")
    ok &= check("Irms matches brute force", np.allclose(r["Irms"], Rb, atol=2e-3), f"max err {np.abs(r['Irms'] - Rb).max():.4f} A")

    d0 = np.linspace(-90, 90, 181)
    s = tm.evaluate(d0, 0.5, 0.5, 95.0)
    ok &= check("SPS power = textbook formula", np.allclose(s["P"], tm.sps_power(d0, 95.0), atol=1e-6))
    k = tm.N_TURNS * 95.0 / tm.V1_NOM
    ipk_sps = tm.V1_NOM / (4 * tm.FS * tm.L_IND) * (2 * k * np.abs(d0) / 180 + (1 - k))   # k = 1 here
    ok &= check("SPS peak = textbook formula (k=1)", np.allclose(s["Ipk"], ipk_sps, atol=1e-6))

    # mirror symmetry: centre phase phi_c = D0 + 180 (D2 - D1) -> -phi_c reverses power, keeps the current stress
    D0m = -D0 - 360.0 * (D2 - D1)
    keep = np.abs(D0m) <= 180
    rm = tm.evaluate(D0m[keep], D1[keep], D2[keep], V2[keep])
    ok &= check("P(-phi_c) = -P(phi_c)", np.allclose(rm["P"], -r["P"][keep], atol=1e-6))
    ok &= check("Ipk(-phi_c) = Ipk(phi_c)", np.allclose(rm["Ipk"], r["Ipk"][keep], atol=1e-9))
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
