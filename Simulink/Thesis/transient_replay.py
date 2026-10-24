"""
transient_replay.py -- T6 fallback, step 2: replay the dataset generation (Scripts/Generate_TPS_Dataset.m) with the
Python plant, period by period over the whole 44 ms run, and compare the extracted windows with the dataset.

Run: D0 timeseries points (0, D0_1), (6 ms, D0_2), (7 ms, D0_3), ..., (43 ms, D0_39), (44 ms, D0_39); window of label
d = [t_d - 0.1 ms, t_d - 1 us) with t_1 = 6 ms, t_d = 6 + (d - 1) ms; Ipk = max |i_L| on the 100 ns samples,
P = mean LV / HV power. Two hypotheses for how From Workspace applies D0 (out/plant_params.md section 6 item 1):
  zoh     the timeseries' zero-order-hold flag is honoured: every window sees its own label, 0.9 ms after the step
  linear  the block interpolates linearly between the points: window d sees ~95 % of the way to label d + 1 (ramping)
The plant is plant_full.FullPlant (125 steps per period = the 100 ns grid of the controller / Simscape solver),
D0 constant within a period (updated each 12.5 us), source droop updated from the previous period's power.
Start: i = 0, series caps discharged.

    python -I -X utf8 Simulink/Thesis/transient_replay.py --n 2000
Output: out/transient_replay.log, out/transient_replay.npz, out/fig_transient_replay.png
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import tps_model as tm  # noqa: E402

OUT = os.path.join(HERE, "out")
DATA = os.path.join(os.path.dirname(HERE), "Data", "TPS_Dataset_20260316_021229.csv")
N_D0 = 39
T_SIM, T_START, T_STEP, T_WIN = 0.044, 0.006, 0.001, 0.0001


def d0_grid():
    x = np.linspace(-1, 1, N_D0)
    return np.sign(x) * 90.0 * (1.0 - np.sqrt(1.0 - np.abs(x)))


def d0_timeseries():
    g = d0_grid()
    t = [0.0] + [T_START + (k - 1) * T_STEP for k in range(1, N_D0)] + [T_SIM]
    d = [g[0]] + [g[k] for k in range(1, N_D0)] + [g[-1]]
    return np.array(t), np.array(d)


def d0_at(t, mode):
    ts, ds = d0_timeseries()
    if mode == "linear":
        return np.interp(t, ts, ds)
    j = np.searchsorted(ts, t, side="right") - 1
    return ds[np.clip(j, 0, ds.size - 1)]


def replay(plant, V2, D1, D2, mode):
    Ts = 1.0 / plant.fs
    nper = int(round(T_SIM / Ts))
    N = V2.size
    i = np.zeros(N)
    vc = np.zeros(N)
    V1b, V2b = np.full(N, plant.V1), V2.copy()
    ends = np.array([T_START] + [T_START + (d - 1) * T_STEP for d in range(2, N_D0 + 1)])
    Ipk = np.zeros((N_D0, N))
    PL = np.zeros((N_D0, N))
    PH = np.zeros((N_D0, N))
    cnt = np.zeros(N_D0)
    for j in range(nper):
        t0 = j * Ts
        D0 = np.full(N, d0_at(t0 + 0.5 * Ts, mode))
        sch = plant.schedule(D0, D1, D2)
        i, vc, Eh, El, wave = plant.period(i, vc, sch, V1b, V2b, record=True)
        V2b = V2 - plant.R_lv * El / V2
        V1b = plant.V1 - plant.R_hv * Eh / plant.V1
        # windows [t_end - 0.1 ms, t_end - 1 us): whole periods inside
        inw = (t0 >= ends - T_WIN - 1e-12) & (t0 + Ts <= ends + 1e-12)
        for d in np.nonzero(inw)[0]:
            Ipk[d] = np.maximum(Ipk[d], np.abs(wave).max(axis=0))
            PL[d] += El
            PH[d] -= Eh                                              # dataset sign (+ = LV -> HV)
            cnt[d] += 1
    return Ipk, PL / cnt[:, None], PH / cnt[:, None]


def main():
    import pandas as pd
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=2000)
    ap.add_argument("--modes", default="zoh,linear")
    ap.add_argument("--Ron", type=float, default=0.052)
    a = ap.parse_args()
    t0 = time.time()
    d = pd.read_csv(DATA)
    g = d0_grid()
    combos = d[np.isclose(d["D0_deg"], g[0])][["V2_V", "D1", "D2"]].drop_duplicates()
    combos = combos.sample(min(a.n, len(combos)), random_state=0).to_numpy(float)
    V2, D1, D2 = combos[:, 0], combos[:, 1], combos[:, 2]
    # dataset rows of these combos, as (39, N) arrays
    key = pd.MultiIndex.from_arrays([np.round(d["V2_V"], 6), np.round(d["D1"], 6), np.round(d["D2"], 6),
                                     np.round(d["D0_deg"], 6)])
    dd = d.set_index(key)
    ref = {c: np.zeros((N_D0, V2.size)) for c in ("i_pk_pri_A", "P_act_W", "P_act_HV_W")}
    for k in range(N_D0):
        idx = pd.MultiIndex.from_arrays([np.round(V2, 6), np.round(D1, 6), np.round(D2, 6),
                                         np.full(V2.size, np.round(g[k], 6))])
        sub = dd.loc[idx]
        for c in ref:
            ref[c][k] = sub[c].to_numpy(float)
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    plant = pf.FullPlant(Ron=a.Ron, steps=125)
    out(f"{V2.size} (V2, D1, D2) combos x {N_D0} D0 labels; plant {plant}")
    # steady-state predictions at the labels (ideal model and the plant) for reference
    lab = np.repeat(g[:, None], V2.size, axis=1)
    ideal = tm.evaluate(lab.ravel(), np.tile(D1, N_D0), np.tile(D2, N_D0), np.tile(V2, N_D0))
    I_id = ideal["Ipk"].reshape(N_D0, -1)
    P_id = ideal["P"].reshape(N_D0, -1)
    res = dict(V2=V2, D1=D1, D2=D2, labels=g, I_ref=ref["i_pk_pri_A"], P_ref=ref["P_act_W"], PH_ref=ref["P_act_HV_W"],
               I_ideal=I_id, P_ideal=P_id)

    def report(name, I, P):
        e, ep = I - ref["i_pk_pri_A"], P - ref["P_act_W"]
        out(f"{name:40s} Ipk bias {e.mean():+.3f} A rmse {np.sqrt(np.mean(e ** 2)):.3f} A p95 "
            f"{np.percentile(np.abs(e), 95):.3f} A | P bias {ep.mean():+7.1f} W rmse {np.sqrt(np.mean(ep ** 2)):6.1f} W")
        return e

    out("\nsteady state at the label D0 vs dataset:")
    report("ideal model at label", I_id, P_id)
    for mode in a.modes.split(","):
        tm0 = time.time()
        I, PL, PH = replay(plant, V2, D1, D2, mode)
        out(f"\nreplay '{mode}' ({time.time() - tm0:.0f} s):")
        e = report(f"plant replay ({mode})", I, PL)
        res[f"I_{mode}"], res[f"P_{mode}"], res[f"PH_{mode}"] = I, PL, PH
        # signature: labels 69.35 and 90 give the same power under 'linear'
        r = np.median(PL[-2] / PL[-1])
        out(f"  median P(label 69.35) / P(label 90): replay {r:.4f}, dataset "
            f"{np.median(ref['P_act_W'][-2] / ref['P_act_W'][-1]):.4f}")
        out("  per-label Ipk bias (replay - dataset) [A]: " + " ".join(f"{g[k]:.0f}:{e[k].mean():+.2f}" for k in
                                                                        range(0, N_D0, 3)))
    np.savez(os.path.join(OUT, "transient_replay.npz"), **res)
    open(os.path.join(OUT, "transient_replay.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        modes = [m for m in a.modes.split(",") if f"I_{m}" in res]
        fig, ax = plt.subplots(1, 1 + len(modes), figsize=(4.3 * (1 + len(modes)), 4.2), constrained_layout=True)
        series = [("ideal steady state at label D0", I_id)] + [(f"Python replay, D0 {m}", res[f"I_{m}"]) for m in modes]
        for k, (title, I) in enumerate(series):
            x, y = ref["i_pk_pri_A"].ravel(), I.ravel()
            ax[k].scatter(x, y, s=1, alpha=0.15, color="#2a6fbb", lw=0)
            lim = [0, max(x.max(), y.max()) * 1.03]
            ax[k].plot(lim, lim, color="#888", lw=0.8)
            e = y - x
            ax[k].set_title(f"{title}\nbias {e.mean():+.3f} A, RMSE {np.sqrt(np.mean(e ** 2)):.3f} A", fontsize=9)
            ax[k].set_xlabel("dataset i_pk (Simulink, 1 ms staircase) [A]")
            ax[k].set_ylabel("model [A]")
            ax[k].set_xlim(lim)
            ax[k].set_ylim(lim)
            ax[k].grid(alpha=0.3, lw=0.5)
        fig.savefig(os.path.join(OUT, "fig_transient_replay.png"), dpi=150)
    except Exception as ex:  # noqa: BLE001
        out(f"(figure skipped: {ex})")
    out(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
