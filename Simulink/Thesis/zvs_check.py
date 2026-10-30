"""
zvs_check.py -- R9: which bridge legs switch with ZVS in Simulink steady state, SPS vs M-A vs M-B (the R8 configurations:
V2 {84, 95, 109.2} x |P| {100 .. 1000} W, HV -> LV).

Data: since 10-30 verify_steady_state.m saves i_L over the last switching period (400 samples, the window
[end - 12.5 us, end)) in out/iL_wave/<result file name>.csv, one row per run, in the same order as the result file.
The R.4 runs behind M-B / SPS / M-A are older, so `make` lists their bracketing D0 levels again (out/zvs_points_r1.csv);
rerunning them repeats those runs (deterministic) with the waveform and the every-ms window peaks.

Model facts used (out/plant_params.md sections 0 and 4, plant_full.py): four legs, each a 50 % square wave of command
phase p -- A (S1/S2, primary) 0, B (S3/S4) 360 D1, C (S5/S6, secondary) D0, D (S7/S8) D0 + 360 D2 [deg]; the command
edge sits on the 100 ns grid rounded up (q(p) = 2.88 ceil(p / 2.88)); the half-bridge driver turns the outgoing device
off at the command edge and the incoming one on td = 100 ns later; Coss = 0 (ideal MOSFET + body diode). So a turn-on is
ZVS iff the incoming device's body diode carries the current during td, i.e. the current INTO the leg's midpoint pulls
it to the incoming rail: rising edge (lower -> upper) needs current into the node > 0, falling edge < 0.
Currents into the nodes per unit of the logged i_L: A -rho, B +rho, C +rho sigma n, D -rho sigma n. rho (i_L's sign)
and sigma (transformer orientation) are not assumed, and neither is the time offset c between the saved window and the
gate grid: all three are fitted per run so that the ideal slope (V1 (uA - uB) - sigma n V2 (uC - uD)) / L (dead time
ignored) correlates best with the measured di/dt; |r| is reported (should be close to 1).
An edge is ZVS if the sign is right both at the outgoing turn-off and at the incoming turn-on td later, 'marginal' if
only at one of them (the current crosses zero inside the dead time), else hard. A leg is ZVS if both its edges are,
hard if either is hard. A configuration takes its two bracketing runs (as in simulink_sweep.py): ZVS on a leg only if
both runs are. Half-wave symmetry max|i(t) + i(t + T/2)| / max|i| is reported as a check of the steady state.

    python -I -X utf8 Simulink/Thesis/zvs_check.py make       -> out/zvs_points_r1.csv
    python automation/job.py submit zvs1 --interactive --after eff1 -- matlab -batch "cd('Simulink/Thesis'); verify_steady_batch('out/zvs_points_r1.csv', 'out/zvs_sweep_r1.csv', 24, 0.030)"
    python -I -X utf8 Simulink/Thesis/zvs_check.py compare    -> out/zvs_check.{csv,log}, out/zvs_runs.csv
    python -I -X utf8 Simulink/Thesis/zvs_check.py selftest   (Python plant waveforms through the same analysis; reference only)
"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
G = TD = 2.88                    # deg: 100 ns gate grid and driver dead time at 80 kHz
N_S = 400                        # samples per saved period
V1, L, NT, FS = 380.0, 90e-6, 4.0, 80e3
INTO = {"A": -1.0, "B": +1.0, "C": +1.0, "D": -1.0}
LEGS = "ABCD"
OSC = 0.02                       # [A] |mean i_L| above which a run oscillates (R.4 'clean')
I_ZC = 0.2                       # [A] a hard edge below this node current is reported apart (TPS shapes i_L to ~0 there)


def q(p):
    """command edge on the 100 ns grid, rounded up (plant_params.md 4.)"""
    return np.ceil(np.round(np.mod(p, 360.0) / G, 9)) * G


def phases(D0, D1, D2):
    return {"A": 0.0, "B": 360.0 * D1, "C": D0, "D": D0 + 360.0 * D2}


def _upper(tau, p, c):
    """1 where the leg's upper device is commanded on (dead time ignored), window angle tau, frame offset c"""
    on, off = q(p) + c, q(p + 180.0) + c
    return (np.mod(tau - on, 360.0) < np.mod(off - on, 360.0)).astype(float)


C_GRID = np.arange(-3 * G, 2 * G + 1e-9, 0.45)      # frame offsets tried [deg]
SIGMAS = (1, -1)


def corr_table(i, ph, V2):
    """signed correlation of the ideal slope with the measured di/dt, shape (2 sigma, len(C_GRID)); NaN if flat.
    One run cannot always tell sigma and rho apart (near D0 = 0 the primary and secondary edges coincide), so the
    frame (sigma, rho, c) is fitted over all runs at once (fit_frame) -- they are properties of the model, not of a run."""
    T = np.full((2, len(C_GRID)), np.nan)
    tau = np.arange(N_S) * 360.0 / N_S
    s = (np.roll(i, -1) - np.roll(i, 1)) * FS * N_S / 2.0
    if not np.all(np.isfinite(i)) or s.std() == 0:
        return T
    for j, c in enumerate(C_GRID):
        u = {k: _upper(tau, ph[k], c) for k in LEGS}
        for a, sigma in enumerate(SIGMAS):
            m = (V1 * (u["A"] - u["B"]) - sigma * NT * V2 * (u["C"] - u["D"])) / L
            if m.std() > 0:
                T[a, j] = np.corrcoef(m, s)[0, 1]
    return T


def fit_frame(tables):
    """(sigma, rho) maximising the summed correlation over the runs; c = the median of the runs' own best offsets in
    that (sigma, rho) (the summed surface is pulled by flat light-load runs; near-zero edge currents move ~0.14 A
    per 0.45 deg at 9 A/us, so the offset matters)"""
    S = np.nansum(np.array(tables), axis=0)
    a, j = np.unravel_index(np.nanargmax(np.abs(S)), S.shape)
    rho = int(np.sign(S[a, j]))
    own = [C_GRID[np.nanargmax(rho * t[a])] for t in tables if np.isfinite(t[a]).any()]
    return SIGMAS[a], rho, float(np.median(own))


def _at(i, tau):
    x = np.mod(tau, 360.0) / 360.0 * N_S
    k = int(np.floor(x)) % N_S
    f = x - np.floor(x)
    return float(i[k] * (1 - f) + i[(k + 1) % N_S] * f)


def edge_list(i, ph, c, sigma, rho):
    """every command edge: (leg, window angle of the command edge, verdict, node current -- the weaker of the
    turn-off / turn-on instants, signed so that > 0 helps)"""
    res = []
    for leg in LEGS:
        k = INTO[leg] * rho * (sigma * NT if leg in "CD" else 1.0)
        for pe, req in ((ph[leg], +1.0), (ph[leg] + 180.0, -1.0)):
            t0 = q(pe) + c
            i0, i1 = k * _at(i, t0), k * _at(i, t0 + TD)
            ok = (req * i0 > 0) + (req * i1 > 0)
            res.append((leg, float(np.mod(t0, 360.0)), "ZVS" if ok == 2 else "marginal" if ok == 1 else "hard",
                        req * min(req * i0, req * i1)))
    return res


def edges(i, ph, c, sigma, rho):
    """per leg: verdict and the node currents at the two edges (turn-off / turn-on instants)"""
    out = {}
    el = edge_list(i, ph, c, sigma, rho)
    for leg in LEGS:
        v = [e[2] for e in el if e[0] == leg]
        cur = [e[3] for e in el if e[0] == leg]
        verdict = "hard" if "hard" in v else "ZVS" if v == ["ZVS", "ZVS"] else "marginal"
        out[leg] = (verdict, min(abs(x) for x in cur) * (1 if verdict == "ZVS" else -1))
    return out


def analyse(i, D0, D1, D2, V2, frame, table):
    """one run in the common frame (sigma, rho, c); r_cal = its own correlation in that frame, c_own = the offset this
    run alone would pick (a check that the frame is common)"""
    i = np.asarray(i, float)
    sigma, rho, c = frame
    a, j = SIGMAS.index(sigma), int(np.argmin(np.abs(C_GRID - c)))
    rec = dict(r_cal=rho * table[a, j], c_own=float(C_GRID[np.nanargmax(rho * table[a])]) if np.isfinite(table[a]).any()
               else np.nan, asym=float(np.max(np.abs(i + np.roll(i, N_S // 2))) / max(np.max(np.abs(i)), 1e-9)))
    if not np.all(np.isfinite(i)):
        return rec
    for leg, (v, m) in edges(i, phases(D0, D1, D2), c, sigma, rho).items():
        rec[f"{leg}_zvs"] = v
        rec[f"{leg}_margin"] = m                                # A: > 0 = ZVS with that much current to spare
    return rec


def analyse_all(waves, ops):
    """waves: (n, N_S); ops: list of (D0, D1, D2, V2) -> (frame, list of per-run dicts)"""
    tabs = [corr_table(w, phases(*op[:3]), op[3]) for w, op in zip(waves, ops)]
    frame = fit_frame(tabs)
    return frame, [analyse(w, *op, frame, t) for w, op, t in zip(waves, ops, tabs)]


# ---------------------------------------------------------------- Simulink runs
def make():
    import efficiency_sweep as es
    d = es.results()
    pts, n = [], 0
    for c in es.configs():
        g = sw.runs_of(d, c)
        b = sw.bracket(g, c["P"])[0] if not g.empty else None
        if b is None:
            continue                                            # eff1 runs these with the waveform capture already
        n += 1
        pts += [sw.point(c, int(b[0]["level"])), sw.point(c, int(b[1]["level"]))]
    print(f"{n} configurations bracketed by runs without a waveform")
    sw.write(os.path.join(OUT, "zvs_points_r1.csv"), pts)


def load_runs(keep_waves=False):
    """one row per Simulink run that has a saved waveform: its result row + the analysis
    (keep_waves: also return the waveforms and operating points, for the frame-offset check)"""
    import pandas as pd
    rows, waves, ops = [], [], []
    for f in sorted(glob.glob(os.path.join(OUT, "iL_wave", "*.csv"))):
        res = os.path.join(OUT, os.path.basename(f))
        if not os.path.isfile(res):
            print(f"no result file for {f}, skipped")
            continue
        w, r = pd.read_csv(f), pd.read_csv(res)
        if len(w) != len(r):
            print(f"{f}: {len(w)} waveforms vs {len(r)} result rows, skipped")
            continue
        W = w[[f"Wv{j}" for j in range(1, N_S + 1)]].to_numpy(float)
        for k in range(len(r)):
            x = r.iloc[k]
            if (isinstance(x.get("error"), str) and x["error"].strip()) or not np.all(np.isfinite(W[k])):
                continue
            V2b = x["V_LV"] if np.isfinite(x.get("V_LV", np.nan)) else x["V2"]     # bridge-side LV voltage
            rows.append(dict(file=os.path.basename(f), label=x.get("label", ""), ctrl=x.get("ctrl", ""), V2=x["V2"],
                             D0=x["D0"], D1=x["D1"], D2=x["D2"], Pd=-x["P_LV"], iL_mean=x["iL_mean"],
                             Ipk_sim=x["Ipk_sim"], iL_rms=x["iL_rms"], loss=x["P_LV"] - x["P_HV"], V_LV=V2b))
            waves.append(W[k])
            ops.append((x["D0"], x["D1"], x["D2"], V2b))
    if not rows:
        return (pd.DataFrame(), None, None, None) if keep_waves else (pd.DataFrame(), None)
    waves = np.array(waves)
    frame, an = analyse_all(waves, ops)
    d = pd.DataFrame([dict(r, **a) for r, a in zip(rows, an)])
    d["level"] = np.round((d["D0"] + 0.5 * G) / G).astype(int)
    keep = ~d.duplicated(subset=["V2", "D0", "D1", "D2"], keep="last")
    if keep_waves:                                              # index = row of waves
        return (d[keep].reset_index(drop=True), frame, waves[keep.to_numpy()],
                [o for o, k in zip(ops, keep) if k])
    return d[keep].reset_index(drop=True), frame


def config_table(d):
    """per R8 configuration: the two bracketing runs (simulink_sweep.bracket) -> leg verdicts and margins"""
    import pandas as pd
    import efficiency_sweep as es
    rows = []
    for c in es.configs():
        g = sw.runs_of(d, c)
        b = sw.bracket(g, c["P"])[0] if not g.empty else None
        rec = dict(c)
        if b is None:
            rows.append(rec)
            continue
        lo, hi = b
        rec["run_lo"], rec["run_hi"] = int(lo.name), int(hi.name)
        rec["osc"] = max(abs(lo["iL_mean"]), abs(hi["iL_mean"]))
        rec["r_min"] = min(lo["r_cal"], hi["r_cal"])
        rec["hard_VI"] = 0.0
        for leg in LEGS:
            v = (lo[f"{leg}_zvs"], hi[f"{leg}_zvs"])
            rec[leg] = "hard" if "hard" in v else "ZVS" if v == ("ZVS", "ZVS") else "marginal"
            rec[f"{leg}_margin"] = min(lo[f"{leg}_margin"], hi[f"{leg}_margin"])
            if rec[leg] == "hard" and rec[f"{leg}_margin"] > -I_ZC:
                rec[leg] = "hard~0"                             # wrong sign, but switching (almost) no current
            if rec[f"{leg}_margin"] < 0:                        # hard or marginal: V x I switched (A, B at V1; C, D at V2)
                rec["hard_VI"] += -rec[f"{leg}_margin"] * (V1 if leg in "AB" else c["V2"])
        rows.append(rec)
    return pd.DataFrame(rows)


C_CHECK = (-2.79, -2.34, -1.44)  # [deg] offsets for the frame check: the all-ZVS 1 kW runs pick -2.34 (some -2.79);
#                                   hard-switched primary legs delay the transition by td, pulling 109.2 V runs to -0.54


def frame_check(d, frame, waves, ops, T0, out):
    """which leg verdicts change if the window offset c is moved off the median (not part of the method; reported)"""
    import pandas as pd
    short = {"ZVS": "Z", "marginal": "m", "hard": "H", "hard~0": "h"}
    out(f"\nframe check: leg verdicts again with the offset c moved from {frame[2]:+.2f} deg (each run's own best: the 1 kW "
        "all-ZVS runs pick -2.34, runs with hard primary legs about -0.54, since a hard edge moves the voltage step td later)")
    have = T0["A"].apply(lambda x: isinstance(x, str)) if "A" in T0 else pd.Series(False, index=T0.index)
    for cc in C_CHECK:
        tabs = [corr_table(w, phases(*op[:3]), op[3]) for w, op in zip(waves, ops)]
        an = [analyse(w, *op, (frame[0], frame[1], cc), t) for w, op, t in zip(waves, ops, tabs)]
        dc = d.copy()
        for k in an[0]:
            if k.endswith("_zvs") or k.endswith("_margin"):
                dc[k] = [a.get(k, np.nan) for a in an]
        T = config_table(dc)
        flips = []
        for i in T0.index[have]:
            a = "".join(short[T0.loc[i, x]] for x in LEGS)
            b = "".join(short.get(T.loc[i, x], "-") for x in LEGS)
            if a != b:
                flips.append(f"{T0.loc[i, 'ctrl']} {T0.loc[i, 'V2']:g} V {T0.loc[i, 'P']:.0f} W {a}->{b}")
        hv = {k: (T0.loc[have & (T0['ctrl'] == k), "hard_VI"].sum(), T.loc[have & (T0['ctrl'] == k), "hard_VI"].sum())
              for k in T0.loc[have, "ctrl"].unique()}
        out(f"  c {cc:+.2f} deg: {len(flips)} of {int(have.sum())} configurations change; summed hard V*I "
            + ", ".join(f"{k} {a / 1e3:.2f} -> {b / 1e3:.2f} kVA" for k, (a, b) in hv.items()))
        for f in flips:
            out(f"     {f}")


TT_DIODE = 16e-9                 # [s] body-diode transit time in the model (plant_params.md 3.): Qrr ~ TT * I


def loss_check(d, out):
    """does the wrong-way switched current show up in the Simulink loss? Least squares over the clean runs
    (|mean i_L| <= OSC, P > 20 W): loss = R I_rms^2 (+ k * 2 TT fs sum V I, the reverse-recovery estimate of the hard
    edges, two per leg and period) (+ c). A fit, not a verdict (reference only)."""
    m = d[(d["iL_mean"].abs() <= OSC) & (d["Pd"] > 20)].copy()
    vi = np.zeros(len(m))
    for leg in LEGS:
        mg = m[f"{leg}_margin"].to_numpy(float)
        vi += np.where(mg < 0, -mg, 0.0) * (V1 if leg in "AB" else m["V_LV"].to_numpy(float))
    X = np.c_[m["iL_rms"].to_numpy(float) ** 2, 2 * TT_DIODE * FS * vi, np.ones(len(m))]
    y = m["loss"].to_numpy(float)
    out(f"\nloss check (reference, a fit over {len(m)} clean runs with a waveform, P > 20 W; Simulink loss = P_HV in - P_LV out):")
    for cols, name in (([0], "R Irms^2"), ([0, 1], "R Irms^2 + k RR"), ([0, 1, 2], "R Irms^2 + k RR + c")):
        b = np.linalg.lstsq(X[:, cols], y, rcond=None)[0]
        e = y - X[:, cols] @ b
        out(f"  {name:22s}: coefficients " + ", ".join(f"{v:.3g}" for v in b)
            + f"; R^2 {1 - (e ** 2).sum() / ((y - y.mean()) ** 2).sum():.3f}, rms residual {np.sqrt((e ** 2).mean()):.2f} W")
    out("  (k = 1 would mean each hard edge costs V * TT * I; a k that changes sign with c means the hard edges are not "
        "seen in the Simulink loss -- Coss = 0, ideal channel)")


FIG_WAVES = ((84.0, 100.0), (109.2, 300.0), (109.2, 800.0))   # bottom row: SPS vs M-B, the lower bracketing run


def fig(T, d, frame, waves, ops):
    """out/fig_zvs_check.png: top, wrong-way V x I per V2 against |P|; bottom, one Simulink period of i_L with every
    command edge marked by its verdict (o ZVS, d marginal, x hard), SPS vs M-B"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, INK2, MUTED, ORANGE, _style
    _style(plt)
    col = {"SPS": MUTED, "M-A": ORANGE, "M-B": BLUE}
    have = T[T["A"].apply(lambda x: isinstance(x, str))]
    fig_, ax = plt.subplots(2, 3, figsize=(13.5, 7.6))
    for j, V2 in enumerate(sorted(have["V2"].unique())):
        a = ax[0, j]
        for k in ("SPS", "M-A", "M-B"):
            g = have[(have["ctrl"] == k) & np.isclose(have["V2"], V2)].sort_values("P")
            if g.empty:
                continue
            a.plot(g["P"], g["hard_VI"], "-", color=col[k], lw=1.8, label=k)
            osc = g["osc"] > OSC
            a.plot(g["P"][~osc], g["hard_VI"][~osc], "o", color=col[k], ms=6, mec="white", mew=0.8)
            a.plot(g["P"][osc], g["hard_VI"][osc], "o", mfc="white", mec=col[k], ms=6, mew=1.4)
        kk = NT * V2 / V1                                       # k = n V2 / V1, as everywhere in RESULTS / REPORT
        a.set_title(f"V2 = {V2:g} V (k = {kk:.2f}): wrong-way switched V x I")
        a.set_xlabel("|P| [W]")
        a.set_ylim(-40, 1550)
        if j == 0:
            a.set_ylabel("sum over hard / marginal legs [VA]")
            a.plot([], [], "o", mfc="white", mec=MUTED, ms=6, mew=1.4, label="oscillating run")
            a.legend(frameon=False, fontsize=8)
    tau = np.arange(N_S) * 360.0 / N_S
    mk = {"ZVS": "o", "marginal": "D", "hard": "X"}
    for j, (V2, P) in enumerate(FIG_WAVES):
        a = ax[1, j]
        for k in ("SPS", "M-B"):
            r = T[(T["ctrl"] == k) & np.isclose(T["V2"], V2) & np.isclose(T["P"], P)]
            if r.empty or not np.isfinite(r["run_lo"].iloc[0]):
                continue
            n = int(r["run_lo"].iloc[0])
            i, (D0, D1, D2, V2b) = waves[n], ops[n]
            a.plot(tau, i, color=col[k], lw=1.6,
                   label=f"{k} ({D1:.3f}, {D2:.3f}), D0 {D0:.2f} deg, {d.loc[n, 'Pd']:.0f} W")
            lab = {}                                            # legs switching at the same instant share one label
            for leg, t0, v, cur in edge_list(i, phases(D0, D1, D2), frame[2], frame[0], frame[1]):
                y = _at(i, t0)
                a.plot(t0, y, mk[v], color=col[k], ms=7 if v != "hard" else 8, mec="white" if v != "hard" else col[k],
                       mew=0.8, zorder=4)
                if v != "ZVS":
                    lab.setdefault(round(t0, 1), (t0, y, []))[2].append(leg)
            for t0, y, legs in lab.values():
                a.annotate(",".join(legs), (t0, y), xytext=(4, 4) if t0 < 330 else (-14, 4), textcoords="offset points",
                           fontsize=7.5, color=INK2)
        a.axhline(0, color=MUTED, lw=0.8)
        a.set_title(f"{V2:g} V, {P:.0f} W: i_L over one period (Simulink)")
        a.set_xlabel("window angle [deg]")
        a.set_xlim(0, 360)
        if j == 0:
            a.set_ylabel("i_L [A]")
            for v in ("ZVS", "marginal", "hard"):
                a.plot([], [], mk[v], color=INK2, ms=6, label=f"edge: {v}")
        a.legend(frameon=True, framealpha=0.85, edgecolor="none", fontsize=7.5, loc="best")
    fig_.suptitle("R9 ZVS per leg, Simulink steady state (HV -> LV; Coss = 0 model, dead time 100 ns)", fontsize=11)
    fig_.tight_layout()
    fig_.savefig(os.path.join(OUT, "fig_zvs_check.png"), dpi=130)
    plt.close(fig_)


def compare():
    import efficiency_sweep as es
    d, frame, waves, ops = load_runs(keep_waves=True)
    if d.empty:
        print("no waveform runs yet (out/iL_wave/)")
        return
    d.to_csv(os.path.join(OUT, "zvs_runs.csv"), index=False, float_format="%.6g")
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    out(f"runs with a waveform: {len(d)}; common frame sigma {frame[0]:+d}, rho {frame[1]:+d}, offset c {frame[2]:+.2f} deg "
        f"(per-run best offset {d['c_own'].min():+.2f}..{d['c_own'].max():+.2f}); slope correlation in that frame min "
        f"{d['r_cal'].min():.3f}, median {d['r_cal'].median():.3f}; half-wave asymmetry max {d['asym'].max():.3f}")
    T = config_table(d)
    T.to_csv(os.path.join(OUT, "zvs_check.csv"), index=False, float_format="%.6g")
    out("\nZVS per leg (A, B primary; C, D secondary), Simulink steady state, HV -> LV; Z = ZVS, m = marginal (current "
        f"reverses inside the dead time), H = hard, h = hard at |i| < {I_ZC} A (about zero-current switching), "
        "~ = oscillating run, - = no waveform yet")
    short = {"ZVS": "Z", "marginal": "m", "hard": "H", "hard~0": "h"}
    out(f"{'V2':>6} {'|P|':>5} | " + " | ".join(f"{k:^7}" for k in es.CTRLS))
    for (V2, P), g in T.groupby(["V2", "P"], sort=True):
        cells = []
        for k in es.CTRLS:
            r = g[g["ctrl"] == k]
            if r.empty or "A" not in r or not isinstance(r["A"].iloc[0], str):
                cells.append(f"{'-':^7}")
                continue
            r = r.iloc[0]
            cells.append(f"{''.join(short[r[x]] for x in LEGS):>5}{'~' if r['osc'] > OSC else ' ':1} ")
        out(f"{V2:6g} {P:5.0f} | " + " | ".join(cells))
    have = T[T["A"].apply(lambda x: isinstance(x, str))] if "A" in T else T.iloc[:0]
    out("\nhard-switched legs (clean runs only), count over the measured configurations:")
    for k in es.CTRLS:
        h = have[(have["ctrl"] == k) & (have["osc"] <= OSC)]
        if len(h):
            n_h = int(sum((h[x] == "hard").sum() for x in LEGS))
            n_z = int(sum((h[x] == "hard~0").sum() for x in LEGS))
            out(f"  {k}: {n_h} of {4 * len(h)} legs hard, {n_z} more hard at |i| < {I_ZC} A, in {len(h)} configurations "
                + "(" + ", ".join(f"{x} {int((h[x] == 'hard').sum())}" for x in LEGS) + ")")
    out("\ncurrent switched the wrong way (hard or marginal legs), primary legs at V1 / secondary legs at V2 [A, own side], "
        "and the sum of V x I over those legs [VA] -- a proxy for turn-on loss (Simulink has Coss = 0, so it shows no "
        "switching loss itself); ~ = oscillating")
    out(f"{'V2':>6} {'|P|':>5} | " + " | ".join(f"{k + ' pri/sec  VI':^22}" for k in es.CTRLS))
    for (V2, P), g in T.groupby(["V2", "P"], sort=True):
        cells = []
        for k in es.CTRLS:
            r = g[g["ctrl"] == k]
            if r.empty or "A" not in r or not isinstance(r["A"].iloc[0], str):
                cells.append(f"{'-':^22}")
                continue
            r = r.iloc[0]
            pri = max(0.0, -min(r["A_margin"], r["B_margin"]))
            sec = max(0.0, -min(r["C_margin"], r["D_margin"]))
            cells.append(f"{pri:5.2f} / {sec:5.2f} {r['hard_VI']:6.0f}{'~' if r['osc'] > OSC else ' '}")
        out(f"{V2:6g} {P:5.0f} | " + " | ".join(cells))
    both = have[have["osc"] <= OSC].pivot_table(index=["V2", "P"], columns="ctrl", values="hard_VI")
    for k in ("M-A", "M-B"):
        if "SPS" in both and k in both:
            x = both[["SPS", k]].dropna()
            if len(x):
                out(f"  {k} vs SPS, both clean ({len(x)} configurations): summed V x I {x[k].sum():.0f} vs "
                    f"{x['SPS'].sum():.0f} VA; lower in {int((x[k] < x['SPS'] - 1).sum())}, higher in "
                    f"{int((x[k] > x['SPS'] + 1).sum())}")
    frame_check(d, frame, waves, ops, T, out)
    loss_check(d, out)
    open(os.path.join(OUT, "zvs_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    fig(T, d, frame, waves, ops)
    print("wrote out/zvs_check.{csv,log}, out/zvs_runs.csv, out/fig_zvs_check.png")


def selftest():
    """the same analysis on Python plant waveforms (reference only). Must hold: the common frame is the plant's own
    (sigma = rho = +1, offset within one sample), and the textbook SPS pattern -- light load k < 1 (84 V) loses ZVS on
    the secondary legs C, D; light load k > 1 (109.2 V) on the primary legs A, B; 84 V 1 kW is ZVS on every leg.
    (109.2 V 1 kW SPS is primary 'marginal' in the plant: i_L(0) = -0.84 A rises 0.9 A within the 100 ns dead time.)"""
    import plant_full as pf
    import pi_steady as ps
    pl = pf.FullPlant(Ron=0.040, droop=False, quant=True)
    cases = [(109.2, 1000.0, 0.5, 0.5), (84.0, 1000.0, 0.5, 0.5), (84.0, 100.0, 0.5, 0.5), (109.2, 100.0, 0.5, 0.5),
             (109.2, 100.0, 0.199, 0.167), (84.0, 100.0, 0.212, 0.244), (95.0, 300.0, 0.5, 0.5),
             (84.0, 300.0, 0.358, 0.407), (109.2, 300.0, 0.350, 0.296)]
    waves, ops = [], []
    for V2, P, D1, D2 in cases:
        D1s, D2s = min(D1, sw.D_SAFE), min(D2, sw.D_SAFE)
        D0 = float(ps.operating_point(np.array([P]), np.array([D1s]), np.array([D2s]), np.array([V2]))["D0"][0])
        r = pl.steady_fixed(np.array([D0]), np.array([D1s]), np.array([D2s]), np.array([380.0]), np.array([V2]))
        w = r["wave"][:-1, 0]
        waves.append(np.interp(np.arange(N_S) / N_S, np.arange(len(w)) / len(w), w, period=1.0))
        ops.append((D0, D1s, D2s, V2))
    frame, an = analyse_all(np.array(waves), ops)
    print(f"common frame: sigma {frame[0]:+d} rho {frame[1]:+d} c {frame[2]:+.2f} deg")
    ok = frame[0] == 1 and frame[1] == 1 and abs(frame[2]) <= 0.9
    want = {(84.0, 1000.0, 0.5): "ZZZZ", (84.0, 100.0, 0.5): "ZZHH", (109.2, 100.0, 0.5): "HHZZ"}
    for (V2, P, D1, D2), (D0, *_), a in zip(cases, ops, an):
        legs = "".join({"ZVS": "Z", "marginal": "m", "hard": "H"}[a[f"{k}_zvs"]] for k in LEGS)
        print(f"{V2:6g} V {P:5.0f} W ({D1:.3f}, {D2:.3f}) D0 {D0:6.2f}: r {a['r_cal']:+.3f} (own offset {a['c_own']:+.2f}) "
              f"asym {a['asym']:.3f} | legs {legs} | margins " + " ".join(f"{a[f'{k}_margin']:+.2f}" for k in LEGS)
              + (f"  [want {want[(V2, P, D1)]}]" if (V2, P, D1) in want else ""))
        if (V2, P, D1) in want:
            ok &= legs == want[(V2, P, D1)]
    print("selftest", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "compare"
    {"make": make, "compare": compare, "selftest": selftest}[cmd]()
