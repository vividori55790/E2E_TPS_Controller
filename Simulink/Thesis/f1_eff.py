"""
f1_eff.py -- (2026-10-31, final deadline F1 / V4 + F2) is the efficiency in this model a number at all, and what does the analyzer's `Efficiency` output show?

The user: "내가 재봤을 때 효율이 제대로 안 나오던데". Known before this file:
  (a) the loss P_LV - P_HV does not settle with the grid: on the same inputs 12.5 ns = 0.77 x 25 ns (20 runs), 10 ns = 0.97 x 12.5 ns, 100 -> 25 ns about half (84 V 1 kW SPS eta 95.13 -> 97.73 %).
      84 V 100 W, eta(M-B) - eta(SPS) is -1.31 pp at 100 ns and +0.57 pp at 25 ns: even the sign moves. So the report's "+0.33 pp" (25 ns, 52 points) is not a converged value yet.
  (b) Direct_Efficiency_Analyzer.m takes "P_HV >= 0: HV is the input", while in this model P_HV < 0 in HV -> LV and P_HV > 0 in LV -> HV (sensors: positive = LV source -> bus, bus -> HV source). So
      its I/O ratio is upside down in both directions, above 100 %, and it should fall through to Eff_Loss = (P_in - P_Loss_Total) / P_in, with P_Loss_Total = i_L^2 x 0.05 ohm (the 90 uH has r = 0 in this
      model) + 4 x one HV switch + 4 x one LV switch, sampled at 100 ns. Read from the code only; `Efficiency` has never been logged (it goes to Scope2 alone).
  Where the sensors sit: V_HV, I_HV and V_LV, I_LV are on the smooth side of the bus caps (47 uF, 1000 uF), so 100 ns sampling should not alias the edges. To be seen, not assumed.
  Seen before these rules were written (the runner's smoke f1smokeb2, 10-31 14:36-14:38, two runs of 2 ms, not 20 ms, SPS 84 V D0 4.32 deg, 349 W, no bracket; not a result): analyzer and direct means within 1 mW; Ts_ctrl 25 ns with solver 25 ns L = 5.51 W, with solver 6.25 ns
  L = 3.19 W; 1.36 ohm x i_rms^2 = 2.32 W, 1.77 ohm = 3.02 W; Eff_out 99.29 % against the direct ratio 98.45 and 99.09 %. So Q3, Q4 and Q6 below are written knowing where they probably land; they are kept as yes/no tests over all runs so that the smoke's one point cannot decide them.

Runs (verify_steady_eff.m: as verify_steady_fine.m, constant D0, 20 ms, last 1 ms, plus direct V x I means at the local step and the six analyzer outputs; V1 380 V):
  S  same inputs on four grids. Points (V2, |P|) = 84 V 100 W (the sign flip), 84 V 1000 W (largest absolute loss change), 109.2 V 100 W (largest eta gain) x HV -> LV, LV -> HV = 6 points, x {SPS, M-B*}
     x the two adjacent 2.88 deg levels that bracket |P| on the 25 ns map or rev25 x Ts {100, 25, 12.5, 6.25 ns} = 96 runs (+ `next` where a grid's power leaves the bracket).
     M-B* = M-B's duties rounded up to the 25 ns duty cell (1/500), minus the nudge: at 25 ns the same gate timing as M-B, and at 25, 12.5, 6.25 ns the SAME gate timing on all three (every edge on the
     0.72 deg lattice). 100 ns cannot hit that lattice (125 steps per period, half a period is 62.5 steps): there it is the same command, every edge rounded to 2.88 deg, as all 100 ns values before.
  X  solver step apart from the controller grid: Ts_ctrl 25 ns, Ts_solver 6.25 ns; 84 V 1000 W, 109.2 V 100 W, HV -> LV, {SPS, M-B*}, the same two levels = 8 runs.

RULES (registered 2026-10-31 14:41, before any run of this file; do not change after the results). Loss L = P_LV_d - P_HV_d - dE/dt, the direct means at the local step, dE/dt = stored energy drift of
the two bus caps over the window (C V dV/dt from dV_LV, dV_HV). The same with the analyzer's means is L_a.
  Q1 absolute loss converged? r = L(6.25) / L(12.5) over the S runs with L(12.5) > 0.5 W. "수렴" iff median |r - 1| <= 0.05 and max |r - 1| <= 0.15, else "미수렴".
  Q2 efficiency gain established? eta at |P| per grid (two levels bracketing |P| in the direct power, first pair outward from D0 = 0; HV -> LV eta = P / (P + L), LV -> HV eta = (P - L) / P), de = eta(M-B*) - eta(SPS) in pp.
     "확정" iff at all 6 points de is defined on 12.5 and 6.25 ns, |de(6.25) - de(12.5)| <= 0.10 pp, and their signs agree wherever |de(6.25)| > 0.10 pp. Else "미확정", and the report then says "효율 이득은 이 모델에서 확정되지 않았다".
     Q2b "25 ns 값 유효" iff in addition |de(25) - de(6.25)| <= 0.10 pp at all 6 points, else "25 ns 값은 수렴값이 아니다": then "+0.33 pp" may not be quoted as a gain, only as the 25 ns value.
  Q3 is the analyzer's 100 ns sampling the cause? delta = median |L_a - L| over the S runs at 25, 12.5, 6.25 ns; Lam = median |L(25) - L(6.25)| over the S configurations. "분석 블록 아님" iff delta <= 0.1 Lam, "분석 블록" iff delta >= 0.5 Lam, else "일부".
  Q4 solver step or controller grid? X_i = [L(25c, 6.25s) - L(6.25c, 6.25s)] / [L(25c, 25s) - L(6.25c, 6.25s)] over the X runs with |denominator| >= 0.2 W (at least 4, else "판정 불가").
     "솔버 스텝" iff |median X| <= 0.2, "제어기 격자" iff median X >= 0.8, else "둘 다".
  Q5 energy balance at 6.25 ns: P_R = 34 Ron i_L,rms^2 (2 primary + 2 secondary conducting devices, the secondary at 4 x the current; Ron 40 mohm, or 52 mohm if the block adds RS 2 + RD 10 mohm, which plant_params.md could not
     settle) and P_D = fs td sum over the 8 edges Vf(I) I (body diode during td = 100 ns, Vf from Is 1e-9, N 1.2; I = |i_L| on the primary legs, 4 |i_L| on the secondary legs, from the last period's waveform). A run balances iff
     0.9 (P_R40 + P_D) <= L <= 1.1 (P_R52 + P_D). Over the S runs at 6.25 ns with L > 0.5 W: "맞음" iff all balance, "대체로" iff >= 75 %, else "안 맞음".
  Q6 the `Efficiency` output, per direction, over the S runs at 25, 12.5, 6.25 ns. eta_m = direct out / in. (i) "추정식" iff |Eff_out - min(99.9, 100 (P_a - Loss_out) / P_a)| <= 0.1 pp in >= 90 % of them, P_a = |P_LV| in HV -> LV and P_HV in LV -> HV (the block's upside-down input).
     (ii) "효율로 쓸 수 없음" iff |Eff_out - 100 eta_m| > 0.5 pp in >= 50 % of them; else "쓸 수 있음".
  Q7 is the grid dependence proportional to the step? a = [L(25) - L(12.5)] / [L(12.5) - L(6.25)] over the S rows with |L(12.5) - L(6.25)| >= 0.1 W. "비례" iff median a is in [1.5, 2.5] (a first-order solver gives 2), else "비례 아님". Only "비례" allows quoting L(0) = 2 L(6.25) - L(12.5) and
     de(0) from it, and then only as an extrapolation beside the measured values, not as a measurement.
Note 2026-11-01 04:25, after the first compare, the rules unchanged: the code of Q5 (and three reported lines) picked the 6.25 ns runs with np.isclose's default atol 1e-8, which also takes 12.5 ns: it counted 28 of 47 runs over both grids instead of the registered 6.25 ns runs. Fixed to atol = 0; the rule text was always "at 6.25 ns". Q5's answer did not change.
What the answers mean. Q2 "확정": the 6 de values at 6.25 ns are the efficiency result, and the other points keep their 25 ns values with Q2b's sentence beside them. Q2 "미확정": no efficiency gain is claimed. Q1 "미수렴" alone: absolute efficiencies are not quoted
as converged, whatever Q2 says. Q3 + Q4 name the cause (or say it was not separated). Q6 tells the user why their reading differed; the copy with the fix is v4/Direct_Efficiency_Analyzer_fixed.m, the user's file stays as it is.
Reported, never a verdict: the loss table per grid, the same with the analyzer's means, L(0) and de(0), L / (P_R + P_D) per grid and L(0) / (P_R + P_D), de computed from P_R + P_D alone (the conduction part, i_rms settles early), drift terms, de with the analyzer's means, non-periodic runs (iL_pp spread), N_local against the step asked for.

    python -I -X utf8 Simulink/Thesis/f1_eff.py selftest
    python -I -X utf8 Simulink/Thesis/f1_eff.py make        -> out/f1eff_points_r1.csv
    python automation/job.py submit f1eff --interactive --after f1v3 -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); f1eff_rounds(1, 3, '<python.exe>')"
    python -I -X utf8 Simulink/Thesis/f1_eff.py next        (f1eff_rounds calls it)
    python -I -X utf8 Simulink/Thesis/f1_eff.py compare     -> out/f1eff_check.{csv,log}, out/f1eff_verdict.md
"""
import argparse
import datetime
import glob
import os
import re
import sys

for _v in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_v, "1")          # `make` / `next` run beside MATLAB's six workers
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fine_grid_check as fg  # noqa: E402
import r13_map as rm  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
G, NUDGE = fg.G, fg.NUDGE
T100, T25, T12, T6 = 1e-7, 25e-9, 12.5e-9, 6.25e-9
GRIDS = (T100, T25, T12, T6)
GRID_NAME = {T100: "100", T25: "25", T12: "12.5", T6: "6.25"}
FINE3 = (T25, T12, T6)
PTS = ((84.0, 100.0), (84.0, 1000.0), (109.2, 100.0))
X_PTS = ((84.0, 1000.0), (109.2, 100.0))
DIRS = (1, -1)                              # +1 HV -> LV (P_LV < 0), -1 LV -> HV (P_LV > 0)
DIR_NAME = {1: "HV->LV", -1: "LV->HV"}
CTRLS = ("SPS", "M-B*")
CELL = 1 / 500                              # the 25 ns duty cell at 80 kHz
C_LV, C_HV = 1000e-6, 47e-6                 # F, plant_params.md
FS, TD, TP = 80e3, 100e-9, 30e-9
IS, N_D, VT = 1e-9, 1.2, 0.02585
R40, R52 = 34 * 0.040, 34 * 0.052
Q1_MED, Q1_MAX, Q1_MIN_L = 0.05, 0.15, 0.5
Q2_TOL = 0.10
Q3_LO, Q3_HI = 0.1, 0.5
Q4_LO, Q4_HI, Q4_DEN, Q4_MIN = 0.2, 0.8, 0.2, 4
Q5_LOW, Q5_HIGH, Q5_MOST = 0.9, 1.1, 0.75
Q6_BRANCH, Q6_BRANCH_N, Q6_WRONG, Q6_WRONG_N = 0.1, 0.90, 0.5, 0.50
Q7_LO, Q7_HI, Q7_DEN = 1.5, 2.5, 0.1
SNAP_EPS = 1e-9


def snap(D):
    """M-B's duty rounded up to the 25 ns cell (what the 25 ns grid fires anyway), the SPS cap kept, the knife edge moved as fg._safe does"""
    x = np.ceil(float(D) / CELL - SNAP_EPS) * CELL
    if x >= 0.5 - 1e-9:
        return sw.D_SAFE
    return round(x - NUDGE / 360.0, 9)


def configs():
    import surface_opt_check as so
    cf = []
    for V2, P in PTS:
        b = so.mb_predict(P, V2)
        for d in DIRS:
            cf.append(dict(ctrl="SPS", V2=V2, P=P, dirn=d, D1=sw.D_SAFE, D2=sw.D_SAFE))
            cf.append(dict(ctrl="M-B*", V2=V2, P=P, dirn=d, D1=snap(b[0]), D2=snap(b[1])))
    return cf


def _map_levels():
    """(V2, P, dirn, ctrl) -> (lo, hi) levels j bracketing |P| on the 25 ns map / rev25 (measured), for the exact M-B of those runs"""
    import rev25 as rv
    df, dr, out = rm.results_25(), rv.results(), {}
    for c in rv.configs():
        if (c["V2"], c["P"]) not in PTS:
            continue
        name = "SPS" if c["ctrl"] == "SPS" else "M-B*"
        g = fg.runs_of(df, c, T25)
        lo = fg.at_power(g, c["P"])[2]
        if lo is None:
            sys.exit(f"the map does not bracket {c}")
        out[(c["V2"], c["P"], 1, name)] = (lo, lo + 1)
        g = rv.mirrored(dr, c)
        lo = fg.at_power(g, c["P"])[2]
        if lo is None:
            sys.exit(f"rev25 does not bracket {c}")
        out[(c["V2"], c["P"], -1, name)] = (-lo, -lo - 1)
    return out


def _point(c, j, Ts, Ts_solver=None):
    Ts_solver = Ts if Ts_solver is None else Ts_solver
    D0 = round(j * G - 0.5 * G, 6) if np.isclose(Ts, T100) else round(j * G - NUDGE, 6)
    return dict(label=f"E {c['ctrl']} {c['P']:g}W {c['V2']:g}V {DIR_NAME[c['dirn']]} L{j} T{Ts * 1e9:g} S{Ts_solver * 1e9:g}", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], dirn=c["dirn"],
                level=j, D0=D0, D1=c["D1"], D2=c["D2"], Ts=Ts, Ts_solver=Ts_solver)


def _write(path, pts):
    import csv
    seen, uniq = set(), []
    for p in pts:
        k = (p["Ts"], p["Ts_solver"], p["V2"], p["dirn"], p["level"], p["D1"], p["D2"])
        if k not in seen:
            seen.add(k)
            uniq.append(p)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["label", "ctrl", "V2", "P_target", "dirn", "level", "D0", "D1", "D2", "Ts", "Ts_solver"])
        w.writeheader()
        w.writerows(uniq)
    print(f"wrote {path}: {len(uniq)} runs ({len(pts)} before removing duplicates)")


def make():
    lv = _map_levels()
    cf = configs()
    pts = []
    for Ts in GRIDS[:3]:                     # cheap grids first; chunks are restart-safe, a run cut off at the deadline keeps what ran
        for c in cf:
            pts += [_point(c, j, Ts) for j in lv[(c["V2"], c["P"], c["dirn"], c["ctrl"])]]
    for c in cf:
        if c["dirn"] == 1 and (c["V2"], c["P"]) in X_PTS:
            pts += [_point(c, j, T25, T6) for j in lv[(c["V2"], c["P"], 1, c["ctrl"])]]
    for c in cf:
        pts += [_point(c, j, T6) for j in lv[(c["V2"], c["P"], c["dirn"], c["ctrl"])]]
    _write(os.path.join(OUT, "f1eff_points_r1.csv"), pts)


def result_files():
    return [f for f in sorted(glob.glob(os.path.join(OUT, "f1eff_sweep_r*.csv"))) if not f.endswith("_in.csv")]


def load(files=None):
    import pandas as pd
    fs = result_files() if files is None else files
    if not fs:
        return None
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")].copy()
    return derive(d.drop_duplicates(subset=["Ts", "Ts_solver", "V2", "dirn", "level", "D1", "D2"], keep="last"))


def derive(d):
    """adds L (direct, drift removed), La, Lraw, Pd (outward power, direct), eta_m"""
    dE = C_LV * d["V_LV"] * d["dV_LV"] / 0.5e-3 + C_HV * d["V_HV"] * d["dV_HV"] / 0.5e-3
    d = d.assign(dE=dE, Lraw=d["P_LV_d"] - d["P_HV_d"], L=d["P_LV_d"] - d["P_HV_d"] - dE, La=d["P_LV"] - d["P_HV"] - dE, Pd=-d["dirn"] * d["P_LV_d"], Pa=-d["dirn"] * d["P_LV"], m=d["dirn"] * d["level"])
    return d.assign(eta_m=np.where(d["dirn"] > 0, np.abs(d["P_LV_d"]) / np.abs(d["P_HV_d"]), d["P_HV_d"] / d["P_LV_d"]))


def runs(d, c, Ts, Ts_solver=None):
    Ts_solver = Ts if Ts_solver is None else Ts_solver
    m = np.isclose(d["V2"], c["V2"]) & (d["dirn"] == c["dirn"]) & (np.abs(d["D1"] - c["D1"]) <= 1e-6) & (np.abs(d["D2"] - c["D2"]) <= 1e-6) \
        & np.isclose(d["Ts"], Ts, rtol=1e-6, atol=0) & np.isclose(d["Ts_solver"], Ts_solver, rtol=1e-6, atol=0)
    return d[m].sort_values("m")


def at_power(g, P, col, pcol="Pd"):
    """col at |P| from the first adjacent outward pair bracketing P in pcol; NaN if none"""
    if g is None or g.empty:
        return np.nan
    m, Pd, y = g["m"].to_numpy(), g[pcol].to_numpy(), g[col].to_numpy()
    for i in range(len(g) - 1):
        if m[i + 1] == m[i] + 1 and min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]) and Pd[i] != Pd[i + 1]:
            return float(y[i] + (P - Pd[i]) / (Pd[i + 1] - Pd[i]) * (y[i + 1] - y[i]))
    return np.nan


def at_grid(S, t):
    """the rows on grid t. np.isclose's default atol of 1e-8 cannot tell 6.25 ns from 12.5 ns, so never use it on these values (found 11-01 04:25, after the first compare: see the note under RULES)"""
    return S[np.isclose(S["Ts"], t, rtol=1e-6, atol=0)]


def round_no():
    ns = [int(m.group(1)) for f in glob.glob(os.path.join(OUT, "f1eff_sweep_r*.csv"))
          for m in [re.search(r"f1eff_sweep_r(\d+)", os.path.basename(f))] if m]
    return max(ns) + 1 if ns else 1


def next_round():
    d = load()
    if d is None:
        sys.exit("no out/f1eff_sweep_r*.csv yet")
    pts = []
    for c in configs():
        for Ts in GRIDS:
            g = runs(d, c, Ts)
            if g.empty:
                print(f"no runs yet: {c} at {Ts}")
                continue
            if np.isfinite(at_power(g, c["P"], "L")):
                continue
            pts += [_point(c, c["dirn"] * int(m), Ts) for m in rm.extend(g["m"].to_numpy(), g["Pd"].to_numpy(), c["P"])]
    if pts:
        _write(os.path.join(OUT, f"f1eff_points_r{round_no()}.csv"), pts)
    else:
        print("every configuration is bracketed on every grid")


# ------------------------------------------------------------------------------------------------- rule functions
def eta_at(P, L, dirn):
    return P / (P + L) if dirn > 0 else (P - L) / P


def rule_q1(r):
    r = np.asarray(r, float)
    r = r[np.isfinite(r)]
    if not len(r):
        return "미수렴", np.nan, np.nan
    a = np.abs(r - 1.0)
    return ("수렴" if np.median(a) <= Q1_MED and a.max() <= Q1_MAX else "미수렴"), float(np.median(a)), float(a.max())


def rule_q2(de12, de6, de25):
    """arrays over the 6 points -> (Q2, Q2b)"""
    de12, de6, de25 = (np.asarray(x, float) for x in (de12, de6, de25))
    if not (np.all(np.isfinite(de12)) and np.all(np.isfinite(de6))):
        return "미확정", "25 ns 값은 수렴값이 아니다"
    sign_ok = np.all((np.abs(de6) <= Q2_TOL) | (np.sign(de6) == np.sign(de12)))
    if not (np.all(np.abs(de6 - de12) <= Q2_TOL) and sign_ok):
        return "미확정", "25 ns 값은 수렴값이 아니다"
    ok25 = np.all(np.isfinite(de25)) and np.all(np.abs(de25 - de6) <= Q2_TOL)
    return "확정", ("25 ns 값 유효" if ok25 else "25 ns 값은 수렴값이 아니다")


def rule_q3(delta, lam):
    if not np.isfinite(delta) or not np.isfinite(lam):
        return "판정 불가"
    return "분석 블록 아님" if delta <= Q3_LO * lam else ("분석 블록" if delta >= Q3_HI * lam else "일부")


def rule_q4(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if len(x) < Q4_MIN:
        return "판정 불가", np.nan
    m = float(np.median(x))
    return ("솔버 스텝" if abs(m) <= Q4_LO else ("제어기 격자" if m >= Q4_HI else "둘 다")), m


def rule_q5(ok):
    ok = np.asarray(ok, bool)
    if not len(ok):
        return "판정 불가"
    return "맞음" if ok.all() else ("대체로" if ok.mean() >= Q5_MOST else "안 맞음")


def rule_q6(branch, wrong):
    branch, wrong = np.asarray(branch, bool), np.asarray(wrong, bool)
    return ("추정식" if len(branch) and branch.mean() >= Q6_BRANCH_N else "추정식 아님"), ("효율로 쓸 수 없음" if len(wrong) and wrong.mean() >= Q6_WRONG_N else "쓸 수 있음")


def rule_q7(a):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    if not len(a):
        return "비례 아님", np.nan
    m = float(np.median(a))
    return ("비례" if Q7_LO <= m <= Q7_HI else "비례 아님"), m


def vf(i):
    return N_D * VT * np.log1p(np.asarray(i, float) / IS)


def diode_power(wave, D0, D1, D2):
    """P_D from the last period of i_L (400 samples over 360 deg): 8 edges, td each, HV legs |i|, LV legs 4 |i|; NaN if no waveform"""
    if wave is None or not np.all(np.isfinite(wave)):
        return np.nan
    n = len(wave)
    tp = 360.0 * FS * TP
    hv = [0.0, 180.0, 360.0 * D1, 360.0 * D1 + 180.0]
    lv = [D0, D0 + 180.0, D0 + 360.0 * D2, D0 + 360.0 * D2 + 180.0]
    at = lambda p: abs(float(wave[int(round(((p + tp) % 360.0) / 360.0 * n)) % n]))  # noqa: E731
    return FS * TD * (sum(vf(at(p)) * at(p) for p in hv) + sum(vf(4 * at(p)) * 4 * at(p) for p in lv))


def _waves():
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_wave", "f1eff_sweep_r*.csv"))):
        if f.endswith("_in.csv"):
            continue
        T = pd.read_csv(f)
        cols = [c for c in T.columns if re.fullmatch(r"Wv\d+", c)]
        for r, x in zip(T["label"], T[cols].to_numpy(float)):
            out[str(r)] = x
    return out


def _spread():
    import pandas as pd
    out = {}
    for f in sorted(glob.glob(os.path.join(OUT, "iL_pp", "f1eff_sweep_r*.csv"))):
        if f.endswith("_in.csv"):
            continue
        T = pd.read_csv(f)
        pk = np.maximum(T[[f"max{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float), -T[[f"min{i}" for i in range(1, rm.PP_N + 1)]].to_numpy(float))
        for r, x in zip(T["label"], pk):
            out[str(r)] = float(np.nanmax(x) - np.nanmin(x))
    return out


def compare(d=None, waves=None, spread=None, write_out=True):
    import pandas as pd
    d = load() if d is None else d
    if d is None:
        sys.exit("no out/f1eff_sweep_r*.csv yet")
    waves = _waves() if waves is None else waves
    spread = _spread() if spread is None else spread
    lines = ["f1_eff: efficiency on four grids, same inputs, Simulink (dataset-era model), rules in the file header. L = P_LV_d - P_HV_d - dE/dt [W], direct means at the local step, last 1 ms", ""]
    S = d[np.isclose(d["Ts"], d["Ts_solver"], rtol=1e-6, atol=0)].copy()
    S = S.assign(P_D=[diode_power(waves.get(str(r.label)), r.D0, r.D1, r.D2) for r in S.itertuples()], spread=[spread.get(str(r.label), np.nan) for r in S.itertuples()])
    S = S.assign(P_R40=R40 * S["iL_rms"] ** 2, P_R52=R52 * S["iL_rms"] ** 2)
    S = S.assign(P_RD=S["P_R52"] + S["P_D"].fillna(0.0))
    S = S.assign(rho=S["L"] / (S["P_R40"] + S["P_D"].fillna(0.0)), bal=(S["L"] >= Q5_LOW * (S["P_R40"] + S["P_D"].fillna(0.0))) & (S["L"] <= Q5_HIGH * (S["P_R52"] + S["P_D"].fillna(0.0))))
    # ---- the same-input table: the rows present on every grid
    key = ["ctrl", "V2", "P_target", "dirn", "level", "D1", "D2"]
    rows = []
    for k, g in S.groupby(key):
        r = dict(zip(key, k))
        for Ts in GRIDS:
            x = g[np.isclose(g["Ts"], Ts, rtol=1e-6, atol=0)]
            n = GRID_NAME[Ts]
            r.update({f"L{n}": x["L"].iloc[0] if len(x) else np.nan, f"La{n}": x["La"].iloc[0] if len(x) else np.nan, f"Pd{n}": x["Pd"].iloc[0] if len(x) else np.nan, f"rho{n}": x["rho"].iloc[0] if len(x) else np.nan,
                      f"rms{n}": x["iL_rms"].iloc[0] if len(x) else np.nan, f"Ipk{n}": x["Ipk_sim"].iloc[0] if len(x) else np.nan, f"osc{n}": abs(x["iL_mean"].iloc[0]) if len(x) else np.nan, f"spread{n}": x["spread"].iloc[0] if len(x) else np.nan,
                      f"Eff{n}": x["Eff_out"].iloc[0] if len(x) else np.nan, f"eta{n}": 100 * x["eta_m"].iloc[0] if len(x) else np.nan})
        rows.append(r)
    T = pd.DataFrame(rows)
    lines.append("Same input, loss L per grid [W] (L(0) = 2 L(6.25) - L(12.5), reported only) | delivered or input |P| at 6.25 ns | L / (P_R40 + P_D) at 100, 25, 12.5, 6.25 ns")
    for r in T.itertuples():
        c = r._asdict()
        lines.append(f" {DIR_NAME[r.dirn]} {r.V2:6.1f} V {r.P_target:5.0f} W {r.ctrl:5s} L{r.level:+3d}: " + " ".join(f"{c['_' + str(9)] if False else T.loc[r.Index, 'L' + GRID_NAME[t]]:7.2f}" for t in GRIDS)
                     + f" | L(0) {2 * T.loc[r.Index, 'L6.25'] - T.loc[r.Index, 'L12.5']:7.2f} | {T.loc[r.Index, 'Pd6.25']:7.1f} W | " + " ".join(f"{T.loc[r.Index, 'rho' + GRID_NAME[t]]:5.2f}" for t in GRIDS))
    # ---- Q1
    x = T[T["L12.5"] > Q1_MIN_L]
    q1, q1m, q1x = rule_q1((x["L6.25"] / x["L12.5"]).to_numpy())
    lines += ["", f"Q1 absolute loss, L(6.25) / L(12.5) over {x['L6.25'].notna().sum()} rows: {q1}, median |r - 1| {q1m:.3f}, max {q1x:.3f}"]
    for a, b in (("25", "100"), ("12.5", "25"), ("6.25", "12.5")):
        y = T[T["L" + b] > Q1_MIN_L]
        rr = (y["L" + a] / y["L" + b]).dropna()
        if len(rr):
            lines.append(f"   L({a}) / L({b}): median {rr.median():.3f}, range {rr.min():.3f} ... {rr.max():.3f}, {len(rr)} rows")
    # ---- Q2
    lines += ["", "Q2 efficiency at |P| per grid, HV->LV eta = P / (P + L), LV->HV eta = (P - L) / P; de = eta(M-B*) - eta(SPS) [pp]; with the analyzer's means in brackets"]
    de, eff_rows = {t: [] for t in GRIDS}, []
    by = {(c["ctrl"], c["V2"], c["P"], c["dirn"]): c for c in configs()}
    for V2, P in PTS:
        for dn in DIRS:
            rec = dict(V2=V2, P=P, dirn=dn)
            txt = []
            for Ts in GRIDS:
                n = GRID_NAME[Ts]
                ls, lb = (at_power(runs(S, by[(c, V2, P, dn)], Ts), P, "L") for c in CTRLS)
                las, lab = (at_power(runs(S, by[(c, V2, P, dn)], Ts), P, "La", "Pa") for c in CTRLS)
                es, eb = eta_at(P, ls, dn), eta_at(P, lb, dn)
                dd = 100 * (eb - es)
                de[Ts].append(dd)
                rs, rb = (at_power(runs(S, by[(c, V2, P, dn)], Ts), P, "P_RD") for c in CTRLS)
                rec.update({f"L_SPS_{n}": ls, f"L_B_{n}": lb, f"eta_SPS_{n}": 100 * es, f"eta_B_{n}": 100 * eb, f"de_{n}": dd, f"de_a_{n}": 100 * (eta_at(P, lab, dn) - eta_at(P, las, dn)), f"de_c_{n}": 100 * (eta_at(P, rb, dn) - eta_at(P, rs, dn)), f"PRD_SPS_{n}": rs, f"PRD_B_{n}": rb})
                txt.append(f"{n}: SPS {ls:6.2f} B {lb:6.2f} W, eta {100 * es:6.2f} {100 * eb:6.2f} %, de {dd:+5.2f} ({rec[f'de_a_{n}']:+5.2f})")
            l0s, l0b = 2 * rec["L_SPS_6.25"] - rec["L_SPS_12.5"], 2 * rec["L_B_6.25"] - rec["L_B_12.5"]
            rec.update(L0_SPS=l0s, L0_B=l0b, de_0=100 * (eta_at(P, l0b, dn) - eta_at(P, l0s, dn)))
            eff_rows.append(rec)
            lines.append(f" {DIR_NAME[dn]} {V2:6.1f} V {P:5.0f} W | " + " | ".join(txt) + f" | L(0) SPS {l0s:.2f} B {l0b:.2f} W, de(0) {rec['de_0']:+.2f} | P_R52 + P_D at 6.25 ns: SPS {rec['PRD_SPS_6.25']:.2f} B {rec['PRD_B_6.25']:.2f} W, de from them {rec['de_c_6.25']:+.2f}")
    q2, q2b = rule_q2(de[T12], de[T6], de[T25])
    lines.append(f"Q2: {q2}; Q2b: {q2b}")
    # ---- Q3
    F = S[S["Ts"].isin(FINE3) | np.isclose(S["Ts"], T6) | np.isclose(S["Ts"], T12) | np.isclose(S["Ts"], T25)]
    F = F[~np.isclose(F["Ts"], T100)]
    delta = float((F["L"] - F["La"]).abs().median()) if len(F) else np.nan
    lam = float((T["L25"] - T["L6.25"]).abs().median()) if len(T) else np.nan
    q3 = rule_q3(delta, lam)
    lines += ["", f"Q3 analyzer: median |L_a - L| {delta:.3f} W (per grid " + ", ".join(f"{GRID_NAME[t]}: {(at_grid(S, t)['L'] - at_grid(S, t)['La']).abs().median():.3f}" for t in GRIDS) + f"), median |L(25) - L(6.25)| {lam:.3f} W -> {q3}"]
    # ---- Q4
    X = d[np.isclose(d["Ts"], T25) & np.isclose(d["Ts_solver"], T6)]
    xs = []
    for r in X.itertuples():
        a = T[(T["ctrl"] == r.ctrl) & (T["V2"] == r.V2) & (T["dirn"] == r.dirn) & (T["level"] == r.level) & (np.abs(T["D1"] - r.D1) <= 1e-6) & (np.abs(T["D2"] - r.D2) <= 1e-6)]
        if a.empty:
            continue
        den = a["L25"].iloc[0] - a["L6.25"].iloc[0]
        val = (r.L - a["L6.25"].iloc[0]) / den if abs(den) >= Q4_DEN else np.nan
        xs.append(val)
        lines.append(f"   X {r.label}: L(25c, 6.25s) {r.L:.2f}, L(25, 25) {a['L25'].iloc[0]:.2f}, L(6.25, 6.25) {a['L6.25'].iloc[0]:.2f} W -> {val:+.3f}")
    q4, q4m = rule_q4(xs)
    lines.append(f"Q4 solver or controller: median X {q4m:+.3f} over {int(np.isfinite(np.array(xs, float)).sum())} runs -> {q4}")
    # ---- Q5
    B = at_grid(S, T6)
    B = B[B["L"] > Q1_MIN_L]
    q5 = rule_q5(B["bal"].to_numpy())
    lines += ["", f"Q5 balance at 6.25 ns: {int(B['bal'].sum())} of {len(B)} runs -> {q5}"]
    for r in B.itertuples():
        lines.append(f"   {r.label}: L {r.L:7.2f}  P_R40 {r.P_R40:7.2f}  P_R52 {r.P_R52:7.2f}  P_D {r.P_D:5.2f}  rho {r.rho:5.2f}  drift {r.dE:+.3f} W  {'ok' if r.bal else 'OUT'}")
    # ---- Q6
    q6 = {}
    lines += ["", "Q6 the Efficiency output against the direct ratio"]
    for dn in DIRS:
        Y = S[(S["dirn"] == dn) & ~np.isclose(S["Ts"], T100)]
        Pa = np.abs(Y["P_LV"]) if dn > 0 else Y["P_HV"]
        est = np.minimum(99.9, 100.0 * (Pa - Y["Loss_out"]) / Pa)
        br, wr = np.abs(Y["Eff_out"] - est) <= Q6_BRANCH, np.abs(Y["Eff_out"] - 100.0 * Y["eta_m"]) > Q6_WRONG
        q6[dn] = rule_q6(br.to_numpy(), wr.to_numpy())
        lines.append(f" {DIR_NAME[dn]}: {q6[dn][0]} ({int(br.sum())} of {len(Y)}), {q6[dn][1]} (off by > {Q6_WRONG} pp in {int(wr.sum())} of {len(Y)}); median Eff_out - eta_m {(Y['Eff_out'] - 100 * Y['eta_m']).median():+.2f} pp, range {(Y['Eff_out'] - 100 * Y['eta_m']).min():+.2f} ... {(Y['Eff_out'] - 100 * Y['eta_m']).max():+.2f} pp")
    # ---- Q7
    y = T[(T["L12.5"] - T["L6.25"]).abs() >= Q7_DEN]
    q7, q7m = rule_q7(((y["L25"] - y["L12.5"]) / (y["L12.5"] - y["L6.25"])).to_numpy())
    lines += ["", f"Q7 proportional to the step: median a {q7m:.2f} over {len(y)} rows -> {q7}"]
    z = at_grid(S, T6)
    n_exp = np.round(0.999e-3 / S["Ts_solver"])
    lines.append(f"N_local / the step asked for: {(S['N_local'] / n_exp).min():.3f} ... {(S['N_local'] / n_exp).max():.3f}; drift |dE/dt| at most {S['dE'].abs().max():.3f} W; non-periodic runs (80-period spread > 5 mA) per grid: " + ", ".join(f"{GRID_NAME[t]}: {int((at_grid(S, t)['spread'] > 0.005).sum())} of {len(at_grid(S, t))}" for t in GRIDS) + f"; at 6.25 ns L / (P_R52 + P_D) median {(z['L'] / z['P_RD']).median():.2f}")
    miss = int(sum(np.isnan(np.array(v, float)).sum() for v in de.values()))
    lines += ["", f"VERDICT: Q1 {q1}; Q2 {q2}; Q2b {q2b}; Q3 {q3}; Q4 {q4}; Q5 {q5}; Q6 HV->LV {q6[1]}, LV->HV {q6[-1]}; Q7 {q7}" + (f" (PROVISIONAL: {miss} de values not bracketed)" if miss else "")]
    print("\n".join(lines))
    res = dict(q1=q1, q2=q2, q2b=q2b, q3=q3, q4=q4, q5=q5, q6=q6, q7=q7, T=T, eff=pd.DataFrame(eff_rows))
    if not write_out:
        return res
    T.to_csv(os.path.join(OUT, "f1eff_check.csv"), index=False, float_format="%.6g")
    res["eff"].to_csv(os.path.join(OUT, "f1eff_eta.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "f1eff_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    md = ["# 효율: 같은 입력을 네 격자로, 직접 평균한 V·I로 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `f1_eff.py compare`. 규칙은 실행 전(10-31 14:41) 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델), V1 380 V, 상수 D0 20 ms, 마지막 1 ms. L = P_LV − P_HV − 버스 저장 에너지 변화.", "",
          f"**판정**: 절대 손실 {q1}. 효율 이득 {q2}({q2b}). 분석 블록 샘플링: {q3}. 원인: {q4}. 에너지 수지(6.25 ns): {q5}. `Efficiency` 출력: HV→LV {q6[1][0]}, {q6[1][1]}; LV→HV {q6[-1][0]}, {q6[-1][1]}. 격자 의존: {q7}.", "",
          "| 방향 | 점 | 손실 SPS, B [W] 100 ns | 25 ns | 12.5 ns | 6.25 ns | Δη [pp] 100 | 25 | 12.5 | 6.25 | 외삽 Δη(h→0) |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in eff_rows:
        md.append(f"| {DIR_NAME[r['dirn']].replace('->', '→')} | {r['V2']:g} V {r['P']:.0f} W | " + " | ".join(f"{r['L_SPS_' + GRID_NAME[t]]:.2f}, {r['L_B_' + GRID_NAME[t]]:.2f}" for t in GRIDS) + " | " + " | ".join(f"{r['de_' + GRID_NAME[t]]:+.2f}" for t in GRIDS) + f" | {r['de_0']:+.2f} |")
    md += ["", "표 원본 `out/f1eff_check.csv`, `out/f1eff_eta.csv`, 로그 `out/f1eff_check.log`."]
    open(os.path.join(OUT, "f1eff_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/f1eff_check.{csv,log}, out/f1eff_eta.csv, out/f1eff_verdict.md")
    return res


def _fake(numeric=20.0, analyzer_err=0.0, flip=False):
    """made-up runs: L = R i^2 + numeric x h [ns] / 25 (for a controller-grid cause use it via Ts, a solver cause via Ts_solver), M-B* 1 W lower; `flip`: M-B* loss depends on the grid so that de moves"""
    import pandas as pd
    rows = []
    for c in configs():
        for Ts, Tss in [(t, t) for t in GRIDS] + ([(T25, T6)] if (c["V2"], c["P"]) in X_PTS and c["dirn"] == 1 else []):
            m0 = int(c["P"] // 100)
            for m, Pr in ((m0, c["P"] - 40.0), (m0 + 1, c["P"] + 40.0)):
                rms = 1.0 + c["P"] / 500.0
                L = R40 * rms ** 2 * 1.02 + numeric * (Tss * 1e9 / 25.0) * 0.1 - (1.0 if c["ctrl"] != "SPS" else 0.0) + (4.0 * Tss * 1e9 / 25.0 if flip and c["ctrl"] != "SPS" else 0.0)
                PL = -c["dirn"] * Pr
                Pin = abs(PL) + L if c["dirn"] > 0 else abs(PL) - L
                PH = -Pin if c["dirn"] > 0 else Pin
                pa = abs(PL) if c["dirn"] > 0 else Pin
                rows.append(dict(N_local=round(0.999e-3 / Tss), label=f"{c['ctrl']} {c['V2']} {c['P']} {c['dirn']} {m} {Ts} {Tss}", ctrl=c["ctrl"], V2=c["V2"], P_target=c["P"], dirn=c["dirn"], level=c["dirn"] * m, D0=c["dirn"] * m * G, D1=c["D1"], D2=c["D2"], Ts=Ts, Ts_solver=Tss,
                                 iL_mean=0.001, iL_rms=rms, Ipk_sim=rms * 1.5, P_LV=PL + analyzer_err, P_HV=PH, P_LV_d=PL, P_HV_d=PH, V_LV=c["V2"], V_HV=380.0, dV_LV=0.0, dV_HV=0.0, Eff_out=100.0 * (pa - 3.0) / pa, Eff_min=90.0, Eff_max=99.0, Loss_out=3.0, Cu_out=1.0, SemiHV_out=1.0, SemiLV_out=1.0, ZVS_out=0.5))
    return derive(pd.DataFrame(rows))


def selftest():
    """the registered rules on made-up numbers, then compare end to end on made-up runs"""
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    cf = configs()
    chk("6 points x 2 controllers", len(cf), 12)
    chk("an SPS duty stays at the cap", snap(0.4999), sw.D_SAFE)
    chk("a duty goes up to the 25 ns cell, minus the nudge", snap(0.4431), round(0.444 - NUDGE / 360.0, 9))
    chk("on the lattice it stays", snap(0.444), round(0.444 - NUDGE / 360.0, 9))
    c = cf[1]
    p = [_point(c, 3, t) for t in FINE3]
    chk("the same numbers on the three fine grids", len({(x["D0"], x["D1"], x["D2"]) for x in p}), 1)
    chk("level j at 100 ns: mid cell as before", _point(c, 3, T100)["D0"], round(3 * G - 0.5 * G, 6))
    x0 = _point(cf[0], 2, T25, T6)
    chk("the X run keeps Ts_ctrl and Ts_solver apart", (x0["Ts"], x0["Ts_solver"]), (T25, T6))
    chk("Q1 settled", rule_q1([1.02, 0.98, 1.04])[0], "수렴")
    chk("Q1 one far off", rule_q1([1.02, 0.98, 1.2])[0], "미수렴")
    chk("Q1 median off", rule_q1([1.08, 1.08, 1.08])[0], "미수렴")
    z = np.zeros(6)
    chk("Q2 settled, 25 ns the same", rule_q2(z + 0.5, z + 0.45, z + 0.5), ("확정", "25 ns 값 유효"))
    chk("Q2 settled, 25 ns off", rule_q2(z + 0.5, z + 0.45, z + 0.8), ("확정", "25 ns 값은 수렴값이 아니다"))
    chk("Q2 moved", rule_q2(z + 0.5, z + 0.3, z + 0.5)[0], "미확정")
    chk("Q2 sign flip above the band", rule_q2(z - 0.02, z + 0.05, z)[0], "확정")
    chk("Q2 sign flip counted only above 0.10 pp", rule_q2(z - 0.02, z + 0.08, z)[0], "확정")
    chk("Q2 a missing value", rule_q2(np.append(z[:5], np.nan), z, z)[0], "미확정")
    chk("Q3 small", rule_q3(0.1, 2.0), "분석 블록 아님")
    chk("Q3 large", rule_q3(1.5, 2.0), "분석 블록")
    chk("Q3 between", rule_q3(0.5, 2.0), "일부")
    chk("Q4 solver", rule_q4([0.1, 0.0, -0.1, 0.05])[0], "솔버 스텝")
    chk("Q4 controller", rule_q4([0.9, 1.0, 0.85, 1.1])[0], "제어기 격자")
    chk("Q4 both", rule_q4([0.5, 0.5, 0.5, 0.5])[0], "둘 다")
    chk("Q4 too few", rule_q4([0.1, 0.1, np.nan, np.nan])[0], "판정 불가")
    chk("Q5", (rule_q5([True, True]), rule_q5([True, True, True, False]), rule_q5([True, False])), ("맞음", "대체로", "안 맞음"))
    chk("Q6", rule_q6([True] * 9 + [False], [True, False]), ("추정식", "효율로 쓸 수 없음"))
    w = np.zeros(400) + 2.0
    chk("diode power on a flat 2 A", round(float(diode_power(w, 10.0, 0.4, 0.4)), 3), round(float(FS * TD * (4 * vf(2.0) * 2.0 + 4 * vf(8.0) * 8.0)), 3))
    chk("no waveform -> NaN", np.isnan(diode_power(None, 0, 0.5, 0.5)), True)
    import pandas as pd
    chk("at_grid keeps one grid only", [len(at_grid(pd.DataFrame(dict(Ts=GRIDS)), t)) for t in GRIDS], [1, 1, 1, 1])
    chk("Q7", (rule_q7([2.0, 2.1, 1.9])[0], rule_q7([1.0, 1.1, 0.9])[0], rule_q7([])[0]), ("비례", "비례 아님", "비례 아님"))
    chk("eta conventions", (round(eta_at(100.0, 5.0, 1), 4), round(eta_at(100.0, 5.0, -1), 4)), (round(100 / 105, 4), 0.95))
    r = compare(_fake(), {}, {}, write_out=False)
    chk("made-up: solver step (L follows Ts_solver)", r["q4"], "솔버 스텝")
    chk("made-up: analyzer = direct", r["q3"], "분석 블록 아님")
    chk("made-up: balance", r["q5"] in ("맞음", "대체로", "안 맞음"), True)
    chk("made-up: M-B* 1 W lower -> gain established", r["q2"], "확정")
    chk("made-up: Efficiency reads the estimate", r["q6"][1][0], "추정식")
    r = compare(_fake(flip=True), {}, {}, write_out=False)
    chk("made-up: M-B* loss moves with the grid -> not established", r["q2"], "미확정")
    r = compare(_fake(analyzer_err=5.0), {}, {}, write_out=False)
    chk("made-up: the analyzer's means off by 5 W -> analyzer", r["q3"], "분석 블록")
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "next", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "next": next_round, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
