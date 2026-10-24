"""
evaluate.py -- T4 baselines + T5 common evaluation of every controller on the exact model (PI steady state).

Controllers (T4)
  SPS
  ICROS2026 surface (P>0 / P<0)    the ICROS2026 training surface (RegHDC_PostProcessed_Dataset.csv) replayed as a
                                   bilinear LUT -- the best the published pipeline could reproduce
  ICROS2026 RegHDC (P>0 / P<0)     LegacyRegHDC (D 32, K 8, Q 32, gamma 8) retrained on that dataset
  Legacy RegHDC (v2 surface)       the same architecture trained on the new smooth surface (T1)
  LUT v2 49x22 / LUT v2 25x11      the T1 surface as an int16 bilinear table (full / 1/4 memory)
  Smooth RegHDC (a), (b*), (b*) int16 (C)   proposed (train_e2e.py); all b variants are evaluated, b* = best
                                   validation score among them (validation set seed 101, disjoint from the test set);
                                   int16 (C) = bit-exact emulation of the exported MCU code (export_mcu.py)
Sets: test = 10 000 uniform random points (seed 2026), dense = |P| 10 W x V2 0.4 V. Reference optimum from
pointwise_random.py (pattern search, = brute force on a 60-point check).

Outputs: out/eval_table.csv (+ printed), out/eval_icros_recheck.csv, figures out/fig_eval_*.png.
"""
import argparse
import csv
import glob
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import pi_steady as ps  # noqa: E402
from pointwise_random import point_set  # noqa: E402
from smooth_surface_v2 import dense_grid, steps  # noqa: E402

OUT = os.path.join(HERE, "out")
ICROS_CSV = os.path.join(ROOT, "Simulink", "RegHDC_PostProcessed_Dataset.csv")
GAIN_TOL = 0.005          # a point is in the "TPS gain region" if the optimum beats SPS by more than this [A]
DEG_TOL = 0.005           # degraded = worse than SPS by more than this [A]


# ------------------------------------------------------------------ controllers
class Signed:
    """Evaluate a signed-input controller on one power direction."""

    def __init__(self, base, sign, name):
        self.base, self.sign, self.name = base, sign, name

    def predict(self, P_abs, V2):
        return self.base.predict(P_abs, V2, sign=self.sign)

    def memory_bytes(self):
        return self.base.memory_bytes()

    def ops(self):
        return self.base.ops()


class LUT16(ct.GridLUT):
    """GridLUT stored as int16 (Q15 duty) on uniform axes (axes are two numbers each)."""

    def memory_bytes(self):
        return 2 * (self.T1.size + self.T2.size) + 4 * 4


def icros_tables():
    import pandas as pd
    d = pd.read_csv(ICROS_CSV)
    out = {}
    for sgn in (+1, -1):
        s = d[np.sign(d["Target_P_W"]) * sgn >= 0]
        s = s.assign(Pa=s["Target_P_W"].abs())
        P = np.sort(s["Pa"].unique())
        V = np.sort(s["Target_V2_V"].unique())
        T1 = s.pivot_table(index="Target_V2_V", columns="Pa", values="Opt_D1").loc[V, P].to_numpy()
        T2 = s.pivot_table(index="Target_V2_V", columns="Pa", values="Opt_D2").loc[V, P].to_numpy()
        out[sgn] = (P, V, T1, T2)
    return d, out


def legacy(name, P, V2, D1, D2, cache):
    m = ct.LegacyRegHDC(D=32, K=8, Q=32, gamma=8.0, seed=0, name=name)
    path = os.path.join(OUT, cache)
    if os.path.exists(path):
        z = np.load(path)
        m.C, m.M, m.val_rmse = z["C"], z["M"], float(z["val_rmse"])
    else:
        m.fit(P, V2, D1, D2, epochs=400, patience=40, seed=0)
        np.savez(path, C=m.C, M=m.M, val_rmse=m.val_rmse)
    return m


def build(b_tags=None, final=None):
    """final: b tag of the proposed model (T7: chosen on the non-ideal plant, plant_verdict.py); None = the
    ideal-model validation score below (picks b3)."""
    ctrls = [ct.SPS()]
    d, tabs = icros_tables()
    for sgn, lab in ((+1, "P>0"), (-1, "P<0")):
        P, V, T1, T2 = tabs[sgn]
        ctrls.append(ct.GridLUT(P, V, T1, T2, name=f"ICROS2026 surface ({lab})"))
    lg = legacy("ICROS2026 RegHDC", d["Target_P_W"].to_numpy(float), d["Target_V2_V"].to_numpy(float),
                d["Opt_D1"].to_numpy(float), d["Opt_D2"].to_numpy(float), "model_legacy_icros.npz")
    ctrls += [Signed(lg, +1, "ICROS2026 RegHDC (P>0)"), Signed(lg, -1, "ICROS2026 RegHDC (P<0)")]
    S = np.load(os.path.join(OUT, "surface_v2.npz"))
    gP, gV, T1, T2 = S["P"], S["V2"], S["smooth_D1"], S["smooth_D2"]
    PP, VV = np.meshgrid(gP, gV)
    Pm = np.concatenate([PP.ravel(), -PP.ravel()])
    lv = legacy("Legacy RegHDC (v2 surface)", Pm, np.tile(VV.ravel(), 2), np.tile(T1.ravel(), 2),
                np.tile(T2.ravel(), 2), "model_legacy_v2.npz")
    ctrls.append(Signed(lv, +1, "Legacy RegHDC (v2 surface)"))
    ctrls.append(LUT16(gP, gV, T1, T2, name=f"LUT v2 {gP.size}x{gV.size}"))
    ctrls.append(LUT16(gP[::2], gV[::2], T1[::2, ::2], T2[::2, ::2], name=f"LUT v2 {gP[::2].size}x{gV[::2].size}"))
    # T13: the same physics E2E fine-tuning applied to the tables (train_lut_e2e.py); L49s3 starts from SPS (no surface)
    from train_lut_e2e import TrainLUT
    for tag in ("L49", "L49s3", "L25", "L13"):
        path = os.path.join(OUT, f"model_lut_{tag}.npz")
        if os.path.exists(path):
            m = TrainLUT.load(path)
            m.name = f"LUT {m.P.size}x{m.V.size} E2E ({tag})"
            ctrls.append(m)
    a = ct.SmoothRegHDC.from_json(os.path.join(OUT, "model_smooth_reghdc_a.json"))
    a.name = "Smooth RegHDC (a)"
    ctrls.append(a)
    bs = []
    for path in sorted(glob.glob(os.path.join(OUT, "model_smooth_reghdc_b*.json"))):
        tag = os.path.basename(path)[len("model_smooth_reghdc_b"):-len(".json")]
        if b_tags and tag not in b_tags:
            continue
        m = ct.SmoothRegHDC.from_json(path)
        m.name = f"Smooth RegHDC (b{tag})"
        bs.append(m)
    best = None
    if bs:
        # b* by the validation objective with one common weighting, not each run's own. The hinge weight is 20, not
        # the training default 4: goal 3 (never worse than SPS) outranks the last 0.05 mA of mean reduction
        # (b1/b6 vs b3 on validation: mean 0.08571 vs 0.08567 A, worst point -4.6 vs -2.3 mA).
        def vscore(m):
            mt = m.meta
            return mt["mean_dI"] - 20.0 * mt["hinge"] - 1e-3 * mt["jac2"] - mt["infeasible"]
        best = max(bs, key=vscore)
        if final:
            best = [m for m in bs if m.name == f"Smooth RegHDC (b{final})"][0]
        ctrls += bs
    from export_mcu import SmoothRegHDCFixed
    ctrls.append(SmoothRegHDCFixed(best if best is not None else a, name=(best or a).name + " int16 (C)"))
    return ctrls, best


# ------------------------------------------------------------------ metrics
def run(ctrl, P, V):
    d1, d2 = ctrl.predict(P, V)
    d1, d2 = np.asarray(d1, float).ravel(), np.asarray(d2, float).ravel()
    r = ps.operating_point(P, d1, d2, V)
    return d1, d2, r


def smoothness(ctrl):
    p, v, DP, DV = dense_grid()
    d1, d2 = ctrl.predict(DP.ravel(), DV.ravel())
    D1, D2 = np.asarray(d1).reshape(DP.shape), np.asarray(d2).reshape(DP.shape)
    st = steps(D1, D2)
    hu, hv = (p[1] - p[0]) / ct.P_MAX, (v[1] - v[0]) / (ct.V2_HI - ct.V2_LO)
    S = np.load(os.path.join(OUT, "surface_v2.npz"))
    GP, GV = np.meshgrid(S["P"], S["V2"])
    g1, g2 = ctrl.predict(GP, GV)
    sg = steps(np.asarray(g1).reshape(GP.shape), np.asarray(g2).reshape(GP.shape))
    return dict(grid_step=sg["max_step"], dense_step=st["max_step"], grad_u=st["max_dP"] / hu,
                grad_v=st["max_dV"] / hv, tv_dense=st["tv"]), (D1, D2)


def metrics(ctrl, P, V, ref):
    d1, d2, r = run(ctrl, P, V)
    ok = r["ok"]
    dI = ref["Ipk_sps"] - r["Ipk"]
    dIrms = ref["Irms_sps"] - r["Irms"]
    gain = ref["Ipk_sps"] - ref["Ipk"]
    reg = gain > GAIN_TOL
    loss = r["Ipk"] - ref["Ipk"]
    f = lambda x: x[ok]  # noqa: E731
    return dict(mean_dI=float(np.mean(f(dI))), mean_dI_gain=float(np.mean(dI[ok & reg])),
                max_dI=float(np.max(f(dI))), min_dI=float(np.min(f(dI))),
                deg_pct=100.0 * float(np.mean(f(dI) < -DEG_TOL)), deg_any_pct=100.0 * float(np.mean(f(dI) < -1e-4)),
                loss_mean=float(np.mean(f(loss))), loss_p99=float(np.percentile(f(loss), 99)),
                loss_max=float(np.max(f(loss))), capture=float(np.mean(f(dI)) / np.mean(gain[ok])),
                infeasible=int((~ok).sum()), mean_dIrms=float(np.mean(f(dIrms)))), dict(dI=dI, d1=d1, d2=d2)


# ------------------------------------------------------------------ ICROS2026 re-check
def icros_recheck():
    import pandas as pd
    d = pd.read_csv(ICROS_CSV)
    Pa, V = d["Target_P_W"].abs().to_numpy(float), d["Target_V2_V"].to_numpy(float)
    r = ps.operating_point(Pa, d["Opt_D1"].to_numpy(float), d["Opt_D2"].to_numpy(float), V)
    s = ps.sps_reference(Pa, V)
    d = d.assign(I_SPS_exact=s["Ipk"], I_Opt_exact=r["Ipk"], claimed_red=d["I_SPS"] - d["I_Opt"],
                 exact_red=s["Ipk"] - r["Ipk"])
    d.to_csv(os.path.join(OUT, "eval_icros_recheck.csv"), index=False, float_format="%.5g")
    i = int(np.argmax(d["claimed_red"].to_numpy()))
    row = d.iloc[i]
    print("\nICROS2026 dataset re-check on the exact steady-state model (196 points):")
    print(f"  claimed reduction I_SPS - I_Opt: mean {d['claimed_red'].mean():.3f} A, max {row['claimed_red']:.3f} A "
          f"at P {row['Target_P_W']:.0f} W, V2 {row['Target_V2_V']:.0f} V")
    print(f"  at that point exact: SPS {row['I_SPS_exact']:.3f} A (dataset {row['I_SPS']:.3f}), with its (D1, D2) "
          f"{row['I_Opt_exact']:.3f} A (dataset {row['I_Opt']:.3f}) -> exact reduction {row['exact_red']:.3f} A")
    print(f"  exact reduction over the dataset: mean {d['exact_red'].mean():.3f} A, max {d['exact_red'].max():.3f} A, "
          f"min {d['exact_red'].min():.3f} A; dataset SPS - exact SPS: mean {np.mean(d['I_SPS'] - d['I_SPS_exact']):+.3f}"
          f" A, max {np.max(d['I_SPS'] - d['I_SPS_exact']):+.3f} A")
    return d


# ------------------------------------------------------------------ figures
BLUE, ORANGE, AQUA, YELLOW = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#898781", "#e1e0d9"


def _style(plt):
    plt.rcParams.update({"font.family": ["Segoe UI", "DejaVu Sans"], "axes.edgecolor": "#c3c2b7",
                         "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True,
                         "grid.color": GRID, "grid.linewidth": 0.6, "axes.titlesize": 10, "axes.titlecolor": INK,
                         "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": "#fcfcfb",
                         "axes.facecolor": "#fcfcfb"})


def _diverging():
    from matplotlib.colors import LinearSegmentedColormap
    # red (worse than SPS) <- gray -> blue (reduction)
    return LinearSegmentedColormap.from_list("rb", ["#a1282b", "#e34948", "#f0efec", "#3987e5", "#104281"])


def _sequential():
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list("blu", ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"])


def fig_maps(path, rows):
    """rows: [(name, D1, D2, dI)] on the dense grid."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    p, v, _, _ = dense_grid()
    fig, ax = plt.subplots(len(rows), 3, figsize=(15, 3.3 * len(rows)), constrained_layout=True, squeeze=False)
    lim = 0.3
    for r, (name, D1, D2, dI) in enumerate(rows):
        for c, (A, title) in enumerate(((D1, "D1"), (D2, "D2"), (dI, "Ipk reduction vs SPS [A]"))):
            a = ax[r, c]
            if c < 2:
                im = a.pcolormesh(p, v, A, cmap=_sequential().reversed(), vmin=0.1, vmax=0.5, shading="nearest")
            else:
                im = a.pcolormesh(p, v, np.clip(A, -lim, lim), cmap=_diverging(), vmin=-lim, vmax=lim,
                                  shading="nearest")
                bad = A < -DEG_TOL
                if bad.any():
                    PP, VV = np.meshgrid(p, v)
                    a.plot(PP[bad], VV[bad], "x", color=INK, ms=2.5, mew=0.6)
                a.text(0.99, 0.03, f"x: worse than SPS by > {DEG_TOL * 1e3:.0f} mA ({int(bad.sum())} pts)",
                       transform=a.transAxes, ha="right", fontsize=7.5, color=INK2)
            a.grid(False)
            a.set_title(f"{name} - {title}")
            a.set_xlabel("|P| [W]")
            a.set_ylabel("V2 [V]")
            fig.colorbar(im, ax=a, shrink=0.9)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_slices(path, series, opt):
    """series: [(name, colour, D1, D2, dI)] on the dense grid; opt = (D1, D2, dI) pointwise optimum."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _style(plt)
    p, v, _, _ = dense_grid()
    rows = [np.argmin(np.abs(v - x)) for x in (84.0, 95.2, 109.2)]
    fig, ax = plt.subplots(3, 3, figsize=(15, 10), constrained_layout=True, sharex=True)
    for c, iv in enumerate(rows):
        for r, (k, yl) in enumerate(((0, "D1"), (1, "D2"), (2, "Ipk reduction vs SPS [A]"))):
            a = ax[r, c]
            a.plot(p, opt[k][iv], color=MUTED, lw=1.2, ls=(0, (2, 2)), label="pointwise optimum")
            for name, col, *vals in series:
                a.plot(p, vals[k][iv], color=col, lw=2, label=name)
            if r == 2:
                a.axhline(0.0, color="#c3c2b7", lw=1)
                a.set_xlabel("|P| [W]")
            a.set_ylabel(yl)
            if r == 0:
                a.set_title(f"V2 = {v[iv]:.1f} V")
    ax[0, 0].legend(fontsize=8, frameon=False, loc="lower right")
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--b-tags", default=None, help="comma list of b tags to include (default all)")
    ap.add_argument("--final", default=None, help="b tag of the proposed model (default: ideal validation score)")
    a = ap.parse_args()
    t0 = time.time()
    ctrls, best = build(a.b_tags.split(",") if a.b_tags else None, a.final)
    print(f"{len(ctrls)} controllers; b* = {best.name if best else None} ({time.time() - t0:.0f} s)", flush=True)
    refs = {}
    for name in ("test", "dense"):
        f = os.path.join(OUT, f"pointwise_{name}.npz")
        if not os.path.exists(f):
            sys.exit(f"missing {f}: run pointwise_random.py first")
        refs[name] = dict(np.load(f))
        P, V = point_set(name)
        assert np.allclose(P, refs[name]["P"]) and np.allclose(V, refs[name]["V2"])
    rows, maps = [], {}
    for c in ctrls:
        rep = dict(name=c.name, memory_B=int(c.memory_bytes()), ops=int(c.ops()))
        for name in ("test", "dense"):
            ref = refs[name]
            m, raw = metrics(c, ref["P"], ref["V2"], ref)
            rep.update({f"{name}_{k}": v for k, v in m.items()})
            if name == "dense":
                maps[c.name] = raw
        sm, _ = smoothness(c)
        rep.update(sm)
        rows.append(rep)
        print(f"{c.name:34s} | test dI mean {rep['test_mean_dI']:.4f} gain-reg {rep['test_mean_dI_gain']:.4f} "
              f"min {rep['test_min_dI'] * 1e3:+8.1f} mA deg {rep['test_deg_pct']:5.2f}% loss {rep['test_loss_mean'] * 1e3:6.2f}"
              f" mA (p99 {rep['test_loss_p99'] * 1e3:6.1f}) inf {rep['test_infeasible']} | dense min "
              f"{rep['dense_min_dI'] * 1e3:+8.1f} mA deg {rep['dense_deg_pct']:5.2f}% | step {rep['grid_step']:.3f} "
              f"grad u {rep['grad_u']:6.1f} v {rep['grad_v']:6.1f} | {rep['memory_B']} B ({time.time() - t0:.0f} s)",
              flush=True)
    keys = list(rows[0].keys())
    with open(os.path.join(OUT, "eval_table.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in rows:
            w.writerow([r[k] if isinstance(r[k], str) else f"{r[k]:.6g}" for k in keys])
    icros_recheck()

    # figures on the dense grid
    _, _, DP, _ = dense_grid()
    sh = DP.shape
    ref = refs["dense"]
    gain = (ref["Ipk_sps"] - ref["Ipk"]).reshape(sh)
    names = ["ICROS2026 surface (P>0)", "Legacy RegHDC (v2 surface)", "LUT v2 49x22"]
    if best is not None:
        names.append(best.name)
    fig_rows = [("pointwise optimum", ref["D1"].reshape(sh), ref["D2"].reshape(sh), gain)]
    fig_rows += [(n, maps[n]["d1"].reshape(sh), maps[n]["d2"].reshape(sh), maps[n]["dI"].reshape(sh)) for n in names]
    fig_maps(os.path.join(OUT, "fig_eval_maps.png"), fig_rows)
    # fixed categorical order: proposed = slot 1 (blue), ICROS2026 = slot 2 (orange), legacy-on-v2 = slot 3 (aqua)
    ser = [(n, col, maps[n]["d1"].reshape(sh), maps[n]["d2"].reshape(sh), maps[n]["dI"].reshape(sh))
           for n, col in zip(["ICROS2026 surface (P>0)", "Legacy RegHDC (v2 surface)"] + ([best.name] if best else []),
                             [ORANGE, AQUA, BLUE])]
    fig_slices(os.path.join(OUT, "fig_eval_slices.png"), ser, (ref["D1"].reshape(sh), ref["D2"].reshape(sh), gain))
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
