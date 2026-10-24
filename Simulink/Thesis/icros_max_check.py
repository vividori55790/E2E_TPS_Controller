"""
icros_max_check.py -- T10: can the ICROS2026 single-point maximum reduction (3.392 A) exist in steady state?
Brute force (D1, D2) on a 0.0025 grid at the dataset's largest-claim points, PI steady state (first crossing),
ideal model; prints SPS, best TPS, and the reduction. Output: out/icros_max_check.log

    python -I -X utf8 Simulink/Thesis/icros_max_check.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402

g = np.arange(0.1, 0.5 + 1e-9, 0.0025)
G1, G2 = np.meshgrid(g, g, indexing="ij")
lines = []
for P, V2, claim_sps in ((150.0, 110.0, 4.7363), (200.0, 110.0, 4.9067), (150.0, 109.2, None), (50.0, 110.0, None),
                         (0.0, 109.2, None)):
    n = G1.size
    r = ps.operating_point(np.full(n, P), G1.ravel(), G2.ravel(), np.full(n, V2))
    I = np.where(r["ok"], r["Ipk"], np.inf)
    b = int(np.argmin(I))
    sps = ps.sps_reference(np.array([P]), np.array([V2]))["Ipk"][0]
    s = (f"|P| {P:6.1f} W  V2 {V2:6.1f} V : steady SPS {sps:.3f} A, best TPS {I[b]:.3f} A at D1 {G1.ravel()[b]:.4f} "
         f"D2 {G2.ravel()[b]:.4f} -> reduction {sps - I[b]:.3f} A"
         + (f"   | dataset SPS {claim_sps:.3f} A: 3.392 A would need {claim_sps - 3.392:.3f} A" if claim_sps else ""))
    print(s, flush=True)
    lines.append(s)
open(os.path.join(HERE, "out", "icros_max_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
