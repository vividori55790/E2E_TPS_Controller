"""
tps_limit.py -- how much can TPS reduce the SPS peak current at all? (R3 physics explanation)

1. Unconstrained exhaustive search on the ideal steady-state model: D1, D2 in [0.01, 0.5] (step 0.01), D0 over the
   whole period (-180..180 deg, step 0.1; every branch, not only the PI's first crossing), feasible if the delivered
   power is within +-0.5 % of P*; minimum Ipk vs the SPS Ipk at the same power (closed form).
2. The bound: TPS can only remove the voltage-mismatch circulating part of the SPS peak. At zero power SPS carries a
   triangular current with peak V1 |1 - k| / (4 fs L) (k = n V2 / V1): 84 V -> 1.53 A, 109.2 V -> 1.97 A, 95 V -> 0.
   The power-carrying part is the same for every modulation at a given power, so the possible reduction shrinks in
   relative terms as the load grows.
3. For comparison: the constrained optimum the controllers use (PI first crossing, D in [0.1, 0.5]; pointwise_random /
   optimal_surface) at the same points.

    python -I -X utf8 Simulink/Thesis/tps_limit.py      -> out/tps_limit.{log,csv}
"""
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402
import pointwise_random as pr  # noqa: E402
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")
V2S = (84.0, 95.0, 109.2)
PS_ = (100.0, 200.0, 300.0, 600.0, 1000.0)


def exhaustive(P, V2, dstep=0.01, phstep=0.1, tol=0.005):
    d = np.round(np.arange(0.01, 0.5 + 1e-9, dstep), 6)
    G1, G2 = np.meshgrid(d, d, indexing="ij")
    g1, g2 = G1.ravel(), G2.ravel()
    best = (np.inf, None)
    for lo in np.arange(-180.0, 180.0, 30.0):                 # D0 in chunks of 30 deg (memory)
        ph = np.arange(lo, lo + 30.0 - 1e-9, phstep)
        r = tm.evaluate(ph[None, :], g1[:, None], g2[:, None], V2)
        ok = np.abs(np.abs(r["P"]) - P) <= tol * P
        I = np.where(ok, r["Ipk"], np.inf)
        k = int(np.argmin(I))
        if I.flat[k] < best[0]:
            i, j = np.unravel_index(k, I.shape)
            best = (float(I.flat[k]), (float(ph[j]), float(g1[i]), float(g2[i])))
    return best


def main():
    t0 = time.time()
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    circ = lambda V2: tm.V1_NOM * abs(1.0 - tm.N_TURNS * V2 / tm.V1_NOM) / (4.0 * tm.FS * tm.L_IND)  # noqa: E731
    out(f"V1 {tm.V1_NOM:g} V, n {tm.N_TURNS:g}, fs {tm.FS / 1e3:g} kHz, L {tm.L_IND * 1e6:g} uH; circulating SPS peak at "
        f"zero power V1|1-k|/(4 fs L): " + ", ".join(f"{V2:g} V (k {tm.N_TURNS * V2 / tm.V1_NOM:.3f}) {circ(V2):.3f} A"
                                                    for V2 in V2S))
    out(f"{'V2':>6s} {'|P|':>6s} | {'SPS Ipk':>8s} | {'unconstrained min':>17s} {'red.':>7s} {'%':>6s} (D0, D1, D2) | "
        f"{'PI 1st crossing opt':>19s} {'red.':>7s} {'%':>6s}")
    rows = []
    for V2 in V2S:
        for P in PS_:
            s = ps.sps_reference(np.array([P]), np.array([V2]))
            I_sps = float(s["Ipk"][0])
            I_un, arg = exhaustive(P, V2)
            # constrained (the controllers' setting): multi-start pattern search of pointwise_random.py
            _, f = pr.solve(np.array([P]), np.array([V2]), procs=1)
            I_pi = float(f[0])
            rows.append(dict(V2=V2, P=P, k=tm.N_TURNS * V2 / tm.V1_NOM, circ=circ(V2), Ipk_sps=I_sps,
                             Ipk_unconstrained=I_un, D0=arg[0], D1=arg[1], D2=arg[2], Ipk_pi_opt=I_pi))
            out(f"{V2:6.1f} {P:6.0f} | {I_sps:8.3f} | {I_un:17.3f} {I_sps - I_un:7.3f} {100 * (I_sps - I_un) / I_sps:5.1f}% "
                f"({arg[0]:6.1f}, {arg[1]:.2f}, {arg[2]:.2f}) | {I_pi:19.3f} {I_sps - I_pi:7.3f} "
                f"{100 * (I_sps - I_pi) / I_sps:5.1f}%  ({time.time() - t0:.0f} s)")
    with open(os.path.join(OUT, "tps_limit.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    open(os.path.join(OUT, "tps_limit.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    sys.exit(main())
