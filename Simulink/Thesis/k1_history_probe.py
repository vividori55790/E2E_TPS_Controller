"""
k1_history_probe.py -- (2026-10-28) why the Python-plant true-staircase replay at k ~ 1 (V2 94.61 V) measures, at the
D0 labels 0 and 2.40 deg, Ipk / P far from the steady state at the same fixed D0 (SPS: replay 14 W 0.755 A and 145 W
0.955 A, steady 159 W 0.444 A and 157 W 0.439 A; k1_light_probe.py). Slow transient (1 ms too short) or another
coexisting stable state reached from the staircase's history?
Same plant as replay_grid.py. Time-stepped per period; at the end of every 1 ms after the label is applied, over the
dataset's 0.1 ms window: max |i|, mean power HV->LV.
  history : the dataset staircase up to the label (zoh, 1 ms steps from -90 deg), then the label held for 20 ms
  fresh   : zero initial state, the label held for 21 ms

    python -I -X utf8 Simulink/Thesis/k1_history_probe.py  -> out/k1_history_probe.log
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import transient_replay as tr  # noqa: E402

OUT = os.path.join(HERE, "out")
V2 = float(np.linspace(84.0, 109.2, 20)[8])


def run(plant, D0_seq, D, report_from):
    """D0_seq: list of (D0, n_periods); one column per entry of D (D1 = D2 = D). Returns per-1 ms block rows after
    the period index report_from: (t_ms, Ipk, P_HV->LV, mean i)."""
    N = D.size
    i, vc = np.zeros(N), np.zeros(N)
    V1b, V2b = np.full(N, plant.V1), np.full(N, V2)
    per_ms = int(round(1e-3 * plant.fs))
    win = int(round(tr.T_WIN * plant.fs))                         # the dataset's 0.1 ms window at the end of each ms
    rows, k = [], 0
    blk = None
    for d0, n in D0_seq:
        sch = plant.schedule(np.full(N, d0), D, D)
        for _ in range(n):
            i, vc, Eh, El, wave = plant.period(i, vc, sch, V1b, V2b, record=True)
            V2b = V2 - plant.R_lv * El / V2
            V1b = plant.V1 - plant.R_hv * Eh / plant.V1
            if k >= report_from and (k - report_from) % per_ms >= per_ms - win:
                if blk is None:
                    blk = [np.zeros(N), np.zeros(N), np.zeros(N), 0]
                blk[0] = np.maximum(blk[0], np.abs(wave).max(axis=0))
                blk[1] += -El
                blk[2] += wave[:-1].mean(axis=0)
                blk[3] += 1
                if blk[3] == win:
                    rows.append(((k + 1 - report_from) / per_ms, blk[0], blk[1] / win, blk[2] / win))
                    blk = None
            k += 1
    return rows


def main():
    plant = pf.FullPlant(Ron=0.052, steps=125)
    per_ms = int(round(1e-3 * plant.fs))
    g = tr.d0_grid()
    D = np.array([0.5, 0.326316, 0.452632])
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"plant {plant}, V2 {V2:.4f} V; columns D1 = D2 = {', '.join(f'{x:.3f}' for x in D)}")
    for idx in (19, 20):                                          # 0-based: labels 0.00 and 2.40 deg
        lab = g[idx]
        # the dataset staircase: g[0] until T_START (6 ms), then 1 ms per label up to this one, then hold 20 ms
        seq = [(g[0], int(round(tr.T_START * plant.fs)))] + [(g[j], per_ms) for j in range(1, idx)]
        seq += [(lab, 21 * per_ms)]
        start = sum(n for _, n in seq[:-1])
        hist = run(plant, seq, D, start)
        fresh = run(plant, [(lab, 21 * per_ms)], D, 0)
        out(f"\nD0 label {lab:+.2f} deg: per 1 ms block after the label is applied (Ipk A / P W)")
        out(f"  {'t ms':>5s} | " + " | ".join(f"history D={x:.3f}  fresh D={x:.3f}" for x in D))
        for (t, Ih, Ph, mh), (_, If, Pf, mf) in zip(hist, fresh):
            if t in (1, 2, 3, 5, 10, 15, 21):
                out(f"  {t:5.0f} | " + " | ".join(f"{Ih[c]:.3f} {Ph[c]:6.1f}   {If[c]:.3f} {Pf[c]:6.1f}" for c in range(D.size)))
    open(os.path.join(OUT, "k1_history_probe.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
