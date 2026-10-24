"""
test_export_matlab.py -- MATLAB is not runnable here, so check the MATLAB deployment from Python:
parse the literals of the generated smooth_reghdc_params.m and run the smooth_reghdc_infer.m algorithm (transcribed
line by line, incl. the overflow-safe softplus) on out/smooth_reghdc_testvec.csv. Checks the generator and the
algorithm, not MATLAB itself.
"""
import csv
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def parse_params(path):
    t = open(path, encoding="utf-8").read()
    p = {}
    for name, body in re.findall(r"p\.(\w+) = (.*?);\n(?=p\.|end)", t, re.S):
        body = body.replace("...", "").strip().strip("[]")
        rows = [r for r in body.split(";") if r.strip()]
        p[name] = np.array([[float(x) for x in r.split()] for r in rows])
    return p


def infer(p, P_ref, V2, V1=380.0):             # = smooth_reghdc_infer.m
    s = lambda k: p[k][0, 0]                   # noqa: E731
    sc = 380.0 / max(V1, 1.0)
    u = min(max(abs(P_ref) * sc ** 2 / s("P_MAX"), 0.0), 1.0)
    v = min(max((V2 * sc - s("V2_LO")) / (s("V2_HI") - s("V2_LO")), 0.0), 1.0)
    h = np.sqrt(2) * np.cos(np.array([[u, v]]) @ p["W"] + p["b"])
    y = h @ p["M"] / np.sqrt(s("D")) + p["c"]
    t = y / s("tau")
    sp = s("tau") * (np.maximum(t, 0) + np.log1p(np.exp(-np.abs(t))))
    d = np.minimum(sp, 0.5 - s("D_MIN"))
    return 0.5 - d[0, 0], 0.5 - d[0, 1]


def main():
    p = parse_params(os.path.join(HERE, "smooth_reghdc_params.m"))
    shapes = {k: v.shape for k, v in p.items()}
    D = int(p["D"][0, 0])
    ok = shapes["W"] == (2, D) and shapes["b"] == (1, D) and shapes["M"] == (D, 2) and shapes["c"] == (1, 2)
    rows = list(csv.DictReader(open(os.path.join(HERE, "out", "smooth_reghdc_testvec.csv"), encoding="utf-8")))
    err = 0.0
    for r in rows:
        d1, d2 = infer(p, float(r["P"]), float(r["V2"]))
        err = max(err, abs(d1 - float(r["D1"])), abs(d2 - float(r["D2"])))
    ok &= err < 1e-12
    print(f"[{'PASS' if ok else 'FAIL'}] smooth_reghdc_params.m literals + infer algorithm: shapes {shapes}, "
          f"{len(rows)} vectors, max |dD| {err:.2e}")


if __name__ == "__main__":
    main()
