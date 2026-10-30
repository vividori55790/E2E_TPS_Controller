"""
r11_step.py -- R11: closed-loop PI step response in the dataset-era Simulink model with a finer PWM edge grid
(`r11_closed_loop.m`, controller copy `r11/DAB_Controller_System.m`). SPS vs M-B: D1, D2 = the regression output for the
commanded |P| and V2, fed as time series (the MCU computes them from the same command); the outer phase is closed by the
controller's own LV-current PI (Kp, Ki on Constant4 / Constant6, Mode_Select = 0).

Why a finer grid: R.7 -- at the model's 100 ns step every edge sits on a 2.88 deg grid, the PI dithers between two
levels and the dither leaves DC offsets and +1.2..3 A peaks whatever the gains. `smoke` first checks, on one operating
point, that a finer step (25 / 10 ns) plus a once-per-period phase latch gives a clean steady state close to the R.4
constant-D0 value, and how long a run takes. `make` then writes the step tests.

    python -I -X utf8 Simulink/Thesis/r11_step.py smoke              -> out/r11_points_smoke.csv, out/r11_profiles/
    python automation/job.py submit r11smoke --interactive --after eff2 -- matlab -batch "cd('Simulink/Thesis'); r11_closed_loop('out/r11_points_smoke.csv', 'out/r11')"
    python -I -X utf8 Simulink/Thesis/r11_step.py make --ts 2.5e-8   -> out/r11_points_r1.csv (after the smoke verdict)
    python -I -X utf8 Simulink/Thesis/r11_step.py compare            -> out/r11_check.{csv,log}, out/fig_r11_step.png
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
RUNS = os.path.join(OUT, "r11")
PROF = os.path.join(OUT, "r11_profiles")
KP, KI = 2.0, 49000.0            # R.7 pilot: the gains that tracked power within 0.2 % at the 100 ns grid
TSW = 1 / 80e3
SEG = 0.012                      # [s] per power level in `make`
PROFILES = {"heavy": (1000.0, 500.0, 1000.0), "light": (300.0, 100.0, 300.0)}
V2S = (84.0, 109.2)
SETTLE_TOL = 0.02                # settled when |P_LV - P_target| <= max(2 %, 5 W) for the rest of the segment
OSC = 0.02                       # [A] |mean i_L| above this = not a clean steady state (R.4)


def duties(ctrl, P, V2):
    if ctrl == "SPS":
        return sw.D_SAFE, sw.D_SAFE
    m = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_b3.json"))
    d1, d2 = m.predict(np.array([P]), np.array([V2]))
    return min(float(np.ravel(d1)[0]), sw.D_SAFE), min(float(np.ravel(d2)[0]), sw.D_SAFE)


def write_profile(name, ctrl, V2, levels, seg):
    """P_ref < 0 = HV -> LV (controller sign); steps at k * seg, held by duplicate-free knots 1 ns apart"""
    import pandas as pd
    t, P = [0.0], [levels[0]]
    for k, lv in enumerate(levels[1:], 1):
        t += [k * seg - 1e-9, k * seg]
        P += [levels[k - 1], lv]
    t.append(len(levels) * seg)
    P.append(levels[-1])
    D = [duties(ctrl, p, V2) for p in P]
    os.makedirs(PROF, exist_ok=True)
    path = os.path.join(PROF, f"{name}.csv")
    pd.DataFrame(dict(t=t, P_ref=[-p for p in P], D1=[d[0] for d in D], D2=[d[1] for d in D], V2=V2)).to_csv(
        path, index=False, float_format="%.10g")
    return os.path.relpath(path, HERE).replace(os.sep, "/")


def write_points(path, rows):
    import pandas as pd
    pd.DataFrame(rows).to_csv(path, index=False, float_format="%.10g")
    print(f"wrote {path}: {len(rows)} runs")


def smoke():
    """one operating point (SPS, 84 V, 1000 W constant, 10 ms): the stock grid (copy at 100 ns, no latch = the original
    controller, R.7 regression check), 25 ns with / without the latch, 10 ns with the latch, 25 ns latch untrimmed logs"""
    prof = write_profile("smoke_SPS_84V_1000W", "SPS", 84.0, (1000.0,), 0.010)
    rows = [dict(label=f"smoke_{tag}", ctrl="SPS", V2=84.0, Ts=ts, sync=sy, Kp=KP, Ki=KI, simTime=0.010, profile=prof,
                 trim=tr)
            for tag, ts, sy, tr in (("100ns", 1e-7, 0, 0), ("25ns_latch", 2.5e-8, 1, 1), ("25ns", 2.5e-8, 0, 1),
                                    ("10ns_latch", 1e-8, 1, 1), ("25ns_latch_full", 2.5e-8, 1, 0))]
    write_points(os.path.join(OUT, "r11_points_smoke.csv"), rows)


def make(ts, sync=1):
    rows = []
    for V2 in V2S:
        for pname, levels in PROFILES.items():
            for ctrl in ("SPS", "M-B"):
                name = f"r1_{ctrl.replace('-', '')}_{V2:g}V_{pname}"
                prof = write_profile(name, ctrl, V2, levels, SEG)
                rows.append(dict(label=name, ctrl=ctrl, V2=V2, Ts=ts, sync=sync, Kp=KP, Ki=KI,
                                 simTime=SEG * len(levels), profile=prof, trim=1))
    write_points(os.path.join(OUT, "r11_points_r1.csv"), rows)


# ---------------------------------------------------------------- analysis
def load():
    import pandas as pd
    runs = []
    for f in sorted(glob.glob(os.path.join(RUNS, "*_summary.csv"))):
        s = pd.read_csv(f).iloc[0].to_dict()
        pf = f.replace("_summary.csv", "_periods.csv")
        s["periods"] = pd.read_csv(pf) if os.path.isfile(pf) else None
        prof = os.path.join(HERE, str(s["profile"]))
        s["prof"] = pd.read_csv(prof) if os.path.isfile(prof) else None
        runs.append(s)
    return runs


def ref_ipk():
    """R.4 / R8 constant-D0 steady state (out/efficiency_compare.csv), linear in |P| between its powers"""
    import pandas as pd
    p = os.path.join(OUT, "efficiency_compare.csv")
    if not os.path.isfile(p):
        return lambda *_: np.nan
    e = pd.read_csv(p)

    def f(ctrl, V2, P):
        g = e[(e["ctrl"] == ctrl) & np.isclose(e["V2"], V2)].dropna(subset=["Ipk"]).sort_values("P")
        return float(np.interp(P, g["P"], g["Ipk"])) if len(g) and g["P"].min() <= P <= g["P"].max() else np.nan
    return f


def segments(s):
    """(t_start, t_end, P_target) from the profile (P_target as delivered power, > 0 = HV -> LV)"""
    pr = s["prof"]
    ts = [0.0] + [float(t) for t, t0 in zip(pr["t"].iloc[1:], pr["t"].iloc[:-1]) if t - t0 < 2e-9]
    te = ts[1:] + [float(s["simTime"])]
    return [(a, b, -float(np.interp(a + 1e-6, pr["t"], pr["P_ref"]))) for a, b in zip(ts, te)]


def seg_metrics(per, a, b, P, P0):
    """steady = the last 2 ms of the segment; settling from the step at a; overshoot = the largest excursion beyond the
    target in the step direction after the delivered power first reaches it (P0 = the previous target, 0 at start).
    A sample at exactly t = a belongs to the next period (MATLAB bins floor(t / Tsw) + 1), so the period ending at a is
    the last one before the step."""
    g = per[(per["t_end"] > a + 1e-9) & (per["t_end"] <= b + 1e-9)]
    Pd = -g["P_LV"].to_numpy()                                   # P_LV > 0 = LV -> HV, so delivered = -P_LV
    st = g[g["t_end"] > b - 2e-3]
    tol = max(SETTLE_TOL * abs(P), 5.0)
    bad = np.abs(Pd - P) > tol
    t_set = np.nan
    if not bad[-1]:
        last_bad = np.where(bad)[0]
        t_set = (g["t_end"].iloc[last_bad[-1] + 1] if len(last_bad) else g["t_end"].iloc[0]) - a
    dev = np.sign(P - P0) * (Pd - P)                            # > 0 = beyond the target in the step direction
    crossed = np.where(dev >= 0)[0]
    return dict(P=P, Ipk_steady=float(st["Ipk"].max()), Ipk_steady_med=float(st["Ipk"].median()),
                iL_mean_steady=float(st["iL_mean"].abs().max()), P_err_pct=float(100 * (-st["P_LV"].mean() - P) / P),
                t_settle_ms=float(t_set * 1e3), overshoot_W=float(dev[crossed[0]:].max()) if len(crossed) else 0.0,
                Ipk_transient=float(g[g["t_end"] <= a + 2e-3]["Ipk"].max()), phi_steady=float(st["phi"].mean()))


def compare():
    import pandas as pd
    runs = load()
    if not runs:
        print("no R11 runs yet (out/r11/)")
        return
    ref = ref_ipk()
    lines, rows = [], []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out("R11 closed loop (dataset-era Simulink model, PI on the outer phase, finer edge grid via r11/ copy); "
        f"steady = last 2 ms of each segment; settled = |P - P*| <= max({SETTLE_TOL:.0%}, 5 W) to the segment end")
    out("Ipk_ref = R.4 / R8 constant-D0 steady state at the same (ctrl, V2, |P|) (out/efficiency_compare.csv, 100 ns grid)")
    for s in runs:
        if not isinstance(s.get("error"), float) and str(s.get("error", "")).strip() not in ("", "nan"):
            out(f"{s['label']}: ERROR {s['error']}")
            continue
        per = s["periods"]
        if per is None or s["prof"] is None:
            out(f"{s['label']}: periods or profile missing")
            continue
        out(f"\n{s['label']} ({s['ctrl']}, {s['V2']:g} V, Ts {s['Ts'] * 1e9:g} ns, latch {int(s['sync'])}, Kp {s['Kp']:g}, "
            f"Ki {s['Ki']:g}; batch {s['batch_seconds']:.0f} s)")
        P0 = 0.0
        for a, b, P in segments(s):
            m = seg_metrics(per, a, b, P, P0)
            P0 = P
            r = ref(s["ctrl"], s["V2"], P)
            rows.append(dict(label=s["label"], ctrl=s["ctrl"], V2=s["V2"], Ts=s["Ts"], sync=s["sync"], t0=a, t1=b,
                             Ipk_ref=r, **m))
            out(f"  {a * 1e3:4.0f}-{b * 1e3:4.0f} ms {P:6.0f} W: Ipk steady {m['Ipk_steady']:.3f} (median period "
                f"{m['Ipk_steady_med']:.3f}) vs ref {r:.3f} A | max|mean i_L| {m['iL_mean_steady']:.3f} A"
                f"{' (osc)' if m['iL_mean_steady'] > OSC else ''} | P error {m['P_err_pct']:+.2f} % | settle "
                f"{m['t_settle_ms']:.2f} ms | overshoot {m['overshoot_W']:+.0f} W | Ipk first 2 ms {m['Ipk_transient']:.3f} A"
                f" | phi {m['phi_steady']:.2f} deg")
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "r11_check.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "r11_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(runs)
    print("wrote out/r11_check.{csv,log}, out/fig_r11_step.png")


def fig(runs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, MUTED, ORANGE, YELLOW, _style
    _style(plt)
    good = [s for s in runs if s["periods"] is not None]
    groups = sorted({(s["V2"], s["label"].rsplit("_", 1)[-1]) for s in good if s["label"].startswith("r1_")})
    smoke = [s for s in good if s["label"].startswith("smoke_")]
    cols = groups if groups else [("smoke", "")]
    fig_, ax = plt.subplots(2, len(cols), figsize=(4.4 * len(cols), 6.4), squeeze=False)
    if groups:
        for j, (V2, pname) in enumerate(groups):
            for s in good:
                if s["label"].startswith("r1_") and np.isclose(s["V2"], V2) and s["label"].endswith(pname):
                    p, c = s["periods"], (MUTED if s["ctrl"] == "SPS" else BLUE)
                    ax[0, j].plot(p["t_end"] * 1e3, p["Ipk"], color=c, lw=1.2, label=s["ctrl"])
                    ax[1, j].plot(p["t_end"] * 1e3, -p["P_LV"], color=c, lw=1.2, label=s["ctrl"])
            ax[0, j].set_title(f"{V2:g} V, {pname} steps: Ipk per period")
            ax[1, j].set_title("delivered power (one-period average)")
    else:
        cmap = [MUTED, BLUE, ORANGE, YELLOW, "#1baf7a"]
        for k, s in enumerate(smoke):
            p = s["periods"]
            ax[0, 0].plot(p["t_end"] * 1e3, p["Ipk"], color=cmap[k % 5], lw=1.0, label=s["label"][6:])
            ax[1, 0].plot(p["t_end"] * 1e3, -p["P_LV"], color=cmap[k % 5], lw=1.0, label=s["label"][6:])
        ax[0, 0].set_title("smoke: SPS 84 V 1000 W, Ipk per period")
        ax[1, 0].set_title("delivered power (one-period average)")
    for a in ax.ravel():
        a.set_xlabel("t [ms]")
    ax[0, 0].set_ylabel("max |i_L| per period [A]")
    ax[1, 0].set_ylabel("P [W]")
    ax[0, 0].legend(frameon=False, fontsize=8)
    fig_.suptitle("R11 closed loop in Simulink (PI on the outer phase, finer edge grid)", fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_r11_step.png"), dpi=130)
    plt.close(fig_)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("smoke", "make", "compare"))
    ap.add_argument("--ts", type=float, default=2.5e-8)
    ap.add_argument("--sync", type=int, default=1)
    a = ap.parse_args()
    if a.cmd == "smoke":
        smoke()
    elif a.cmd == "make":
        make(a.ts, a.sync)
    else:
        compare()
