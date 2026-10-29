"""
pwm_counter_check.py -- does the dataset-era controller's floating-point phase counter move its gate edges over time?
(2026-10-30, OPT round 1: SPS at 95 V 73 W peaks at 0.738 A in the 1 ms window but 0.41 A in its last period; some
clean runs 'flicker' between two peak levels from one 1 ms window to the next.)

DAB_Controller_System.m (dataset_era/, STAGE 7) every Ts_ctrl = 100 ns:
    PhaseCounter = mod(PhaseCounter + (Ts_ctrl / Ts_sw) * 360, 360)
    G1 on if DT <= rS1 < 180 - DT, G2 on if 180 + DT <= rS1 < 360 - DT, rS = mod(cp - phi + 360, 360) per leg,
    DT = (DeadTime_ns 1e-9 / Ts_sw) * 360 = 0 here (the drivers add 100 ns), phi = 0 (leg 1), D1 360 (leg 3), D0 (leg 5),
    D0 + D2 360 (leg 7). Only the high-side commands G1/G3/G5/G7 are used (plant_params.md 4).
A boundary that falls on a counter value (k * 2.88 deg) is a knife edge: whether 'cp >= boundary' holds at that step
depends on the rounding of the accumulated counter, and the accumulated error drifts from period to period.
This script replays the counter in IEEE double (Python float = MATLAB double; MATLAB mod emulated as a - floor(a/m) m,
which is what mod computes for these positive operands) and reports, per leg and per configuration, whether the gate
pattern changes between switching periods within a 40 ms run. Pure arithmetic, no plant: it says whether the gate
timing itself moves, not what that does to the current (that is Simulink's job; OPT/k1map per-period peaks).

    python -I -X utf8 Simulink/Thesis/pwm_counter_check.py   -> out/pwm_counter_check.log
"""
import math
import os
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
TS_CTRL, FS, DEAD_NS = 1e-7, 80e3, 0.0     # DeadTime_ns = Constant7 = 0 in the model (out/plant_params.md 4):
                                                # the 100 ns dead time comes from the UCC half-bridge drivers
STEPS = 125


def counter(n_periods):
    ts_sw = 1 / FS
    inc = (TS_CTRL / ts_sw) * 360
    pc, cps = 0.0, np.empty(n_periods * STEPS)
    for i in range(n_periods * STEPS):
        a = pc + inc
        pc = a - math.floor(a / 360.0) * 360.0
        cps[i] = pc
    return cps


def gates(cps, phi):
    ts_sw = 1 / FS
    dt = (DEAD_NS * 1e-9 / ts_sw) * 360
    p = math.fmod(phi, 360.0)
    if p < 0:
        p += 360.0
    r = np.mod(cps - p + 360.0, 360.0)
    hi = (r >= dt) & (r < 180 - dt)
    lo = (r >= 180 + dt) & (r < 360 - dt)
    return hi.reshape(-1, STEPS), lo.reshape(-1, STEPS)


def describe(cps, phi):
    """number of distinct per-period gate patterns, and the periods where the pattern changes"""
    hi, lo = gates(cps, phi)
    pats = [(h.tobytes(), l.tobytes()) for h, l in zip(hi, lo)]
    c = Counter(pats)
    ch = [i for i in range(1, len(pats)) if pats[i] != pats[i - 1]]
    on = sorted({(int(h.sum()), int(l.sum())) for h, l in zip(hi, lo)})
    return len(c), ch, on


def main():
    n = 3200                                            # 40 ms at 80 kHz
    cps = counter(n)
    ts_sw = 1 / FS
    inc = (TS_CTRL / ts_sw) * 360
    dt = (DEAD_NS * 1e-9 / ts_sw) * 360
    lines = [f"increment {inc!r} deg, dead time {dt!r} deg, equal: {inc == dt}",
             f"counter after 1 / 10 / {n} periods (exact 0 would be no drift): "
             f"{cps[STEPS - 1]!r}, {cps[10 * STEPS - 1]!r}, {cps[-1]!r}",
             f"max |counter - nearest k * increment| over the run: {np.max(np.abs(cps - np.round(cps / inc) * inc)):.3e} deg", ""]
    lines.append("leg phase phi [deg] -> distinct gate patterns over 3200 periods (high-side on-steps, low-side on-steps), "
                 "first periods where the pattern changes")
    cases = [("leg 1 (phi = 0, every configuration)", 0.0)]
    for d in (0.4999999, 0.475, 0.45, 0.40):
        cases.append((f"leg 3 / leg 7 offset, D = {d}", d * 360))
    for j in (1, 2, 3, 4, 5):
        d0 = (j - 0.5) * 2.88
        cases.append((f"leg 5, D0 level {j} ({d0:.2f} deg)", d0))
        cases.append((f"leg 7, D0 level {j} + SPS D2 = 0.4999999", d0 + 0.4999999 * 360))
        cases.append((f"leg 7, D0 level {j} + D2 = 0.45", d0 + 0.45 * 360))
    for name, phi in cases:
        k, ch, on = describe(cps, phi)
        lines.append(f"  {name:<44} phi {phi:10.6f}: {k} pattern(s) {on}"
                     + (f"; changes at periods {ch[:8]}{' ...' if len(ch) > 8 else ''} ({len(ch)} changes)" if ch else ""))
    txt = "\n".join(lines)
    print(txt)
    open(os.path.join(OUT, "pwm_counter_check.log"), "w", encoding="utf-8").write(txt + "\n")


if __name__ == "__main__":
    main()
