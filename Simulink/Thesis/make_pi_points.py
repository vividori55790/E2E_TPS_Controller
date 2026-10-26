"""
make_pi_points.py -- R2(3) points for verify_pi_closed_loop.m (Simulink PI closed loop, external D1/D2).

  full:   V2 {84, 95, 109.2} x |P| {100, 300, 600, 1000} W x {SPS, M-A, M-B0, M-B}, HV -> LV (P_ref < 0)
          + LV -> HV subset V2 {84, 109.2} x |P| {100, 300} W (M-A = the dataset's P > 0 half there)
  --pilot SPS at (84 V, 100 W), (95 V, 1000 W), (109.2 V, 300 W) HV -> LV with the model's own PI gains and with
          10x lower gains, plus M-B at (109.2 V, 100 W) and M-A at (84 V, 300 W) with the model's gains
Controllers (train_targets.py): M-A = model_smooth_reghdc_MA_neg / MA_pos, M-B0 = (a), M-B = b3. (D1, D2) are the
controllers' outputs at (|P|, V2 = the source label); D0 is left to the PI. Kp = Ki = -1 means "the model's own PI
gains" (Constant4 = 20, Constant6 = 490000 in the dataset-era model, slx_trace_inputs.py).

    python -I -X utf8 Simulink/Thesis/make_pi_points.py [--pilot] [--kp -1 --ki -1] [--out out/pi_points.csv]
"""
import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402

OUT = os.path.join(HERE, "out")


def models():
    load = lambda t: ct.SmoothRegHDC.from_json(os.path.join(OUT, f"model_smooth_reghdc_{t}.json"))  # noqa: E731
    return {"SPS": (ct.SPS(), ct.SPS()), "M-A": (load("MA_neg"), load("MA_pos")), "M-B0": (load("a"), load("a")),
            "M-B": (load("b3"), load("b3"))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pilot", action="store_true")
    ap.add_argument("--kp", type=float, default=-1.0)
    ap.add_argument("--ki", type=float, default=-1.0)
    ap.add_argument("--out", default=os.path.join(OUT, "pi_points.csv"))
    a = ap.parse_args()
    ms = models()
    rows = []

    def add(ctrl, direction, V2, P, kp, ki):
        m = ms[ctrl][0 if direction == "HV->LV" else 1]
        d1, d2 = m.predict(np.array([P]), np.array([V2]))
        d1, d2 = float(np.ravel(d1)[0]), float(np.ravel(d2)[0])
        sgn = -1.0 if direction == "HV->LV" else 1.0                 # controller sign: (-) = charging HV -> LV
        rows.append(dict(label=f"{ctrl} {direction} {P:.0f}W {V2:g}V", ctrl=ctrl, dir=direction, V2=V2,
                         P_ref=sgn * P, D1=round(d1, 6), D2=round(d2, 6), Kp=kp, Ki=ki))

    if a.pilot:
        # pilot 3 (14:19): the model's own gains (Kp 20, Ki 490000) chatter (Ipk 16-18 A), Kp 2 / Ki 49000 tracks the
        # power but dithers (+1.3-2 A); integral-dominated gains for a loop gain dI/dphi ~ 0.6 A/deg:
        for kp, ki in ((0.05, 10000.0), (0.2, 20000.0)):
            for V2, P in ((84.0, 100.0), (95.0, 1000.0), (109.2, 300.0)):
                add("SPS", "HV->LV", V2, P, kp, ki)
        add("M-B", "HV->LV", 109.2, 100.0, 0.05, 10000.0)
        add("M-A", "HV->LV", 84.0, 300.0, 0.05, 10000.0)
    else:
        for ctrl in ms:
            for V2 in (84.0, 95.0, 109.2):
                for P in (100.0, 300.0, 600.0, 1000.0):
                    add(ctrl, "HV->LV", V2, P, a.kp, a.ki)
            for V2 in (84.0, 109.2):
                for P in (100.0, 300.0):
                    add(ctrl, "LV->HV", V2, P, a.kp, a.ki)
    with open(a.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {a.out}: {len(rows)} points")
    for r in rows:
        print(f"  {r['label']:28s} D1 {r['D1']:.4f} D2 {r['D2']:.4f} P_ref {r['P_ref']:+7.1f} Ki {r['Ki']:g}")


if __name__ == "__main__":
    sys.exit(main())
