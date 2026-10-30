"""
r12_ramp.py -- R12 (user, 10-29): what removing the SPS/TPS classifier buys, shown in the R11 closed loop (dataset-era
Simulink model, PI on the outer phase, 25 ns edge grid + once-per-period phase latch; runner r11_closed_loop.m).
A slow HV -> LV power ramp across the SPS/TPS boundary; D1, D2 = each controller's output for the commanded |P|.

Controllers
  ICROS   the ICROS2026 structure re-implemented: RegHDC (D 32, K 8, Q 32, gamma 8; controllers.LegacyRegHDC retrained on
          Simulink/RegHDC_PostProcessed_Dataset.csv -- evaluate.py's out/model_legacy_icros.npz), signed input
          (HV -> LV = P < 0, the controller's Ref_Power sign), plus the SPS/TPS classifier replaced by the dataset's own
          labels (a row is SPS-optimal when I_Opt == I_SPS) with the FSM lock-in (SPS from the first SPS-labelled |P|
          upward): boundary = midpoint between the last TPS row below it and that first SPS row, at the nearest dataset
          V2 (90 V: 625 W, 110 V: 775 W). The paper's classifier was 97.44 % accurate, so this is its best case.
  M-B     the proposed single regression (out/model_smooth_reghdc_b3.json)
  M-S     the corrected surface adopted 10-30 (out/model_smooth_reghdc_MS.json), reported alongside
  SPS     D1 = D2 = D_SAFE (no inner phase at all: the PI alone)
Profile: 100 W for 6 ms (start-up), ramp to 900 W over 40 ms (20 W/ms), 900 W for 4 ms; one knot per switching period
(12.5 us), D1 / D2 evaluated at each knot (From Workspace interpolates linearly: a jump spreads over one period).
V2: 90 V (k 0.947, an ICROS dataset row) and 109.2 V (k 1.149).

RULE (registered 10-30 14:10, before any R12 run; do not change after results):
  window = the ramp, 6-46 ms. Per (V2, controller):
    m1  D step   = largest change of the commanded D1 or D2 between consecutive knots (one period)
    m2  tracking = max |e| and RMS e over the window, e = P_delivered - P_ref per period [W] (P_delivered = -P_LV)
    m3  spike    = max over the window of (Ipk - env) [A], env = running median of the per-period Ipk over 41 periods
  per V2:  "효과 확인" if m1(ICROS) >= 5 m1(M-B) and (max|e|(ICROS) >= 2 max|e|(M-B) or m3(ICROS) >= m3(M-B) + 0.1 A);
           "효과 없음" if max|e|(ICROS) <= 1.2 max|e|(M-B) and m3(ICROS) <= m3(M-B) + 0.03 A;  otherwise "부분".
  The rule compares ICROS with M-B (the user's R12 wording); M-S and SPS are reported with the same numbers.
  Round 1 (r12r1b, 10-30 15:41): 90 V "부분", 109.2 V "효과 확인". The fixed PI gains limit-cycle at ~4 kHz wherever the phase
  gain is high (R11 round 1: SPS at light load, every controller at 109.2 V 1 kW, M-B / M-S at 109.2 V 500 W; on the 90 V ramp
  M-B / M-S at ~330-580 W), so m2 / m3 mix the PI's limit cycle with the classifier's jumps.
ROUND 2 (registered 10-30 15:45, before the R11 round-2 results that set it): the same profiles, window and rule, with (Kp, Ki)
  scaled by the largest factor of R11 round 2 (0.5, then 0.25) at which every R11 steady segment (last 2 ms) of SPS, M-B and
  M-S has a phase peak-to-peak <= 2 deg; if neither factor qualifies, 0.25 is used and that is reported.
  (Code added 10-30 18:50, before r11r2 has run: round2_factor() = that rule on out/r11_check.csv, `make --round 2`, labels r2_*
  in the same out/r12, `compare` judges each round separately: round 2 -> out/r12_check_r2.{csv,log}, out/r12_verdict_r2.md,
  out/fig_r12_ramp_r2.png.)

    python -I -X utf8 Simulink/Thesis/r12_ramp.py make       -> out/r12_points_r1.csv, out/r12_profiles/
    python -I -X utf8 Simulink/Thesis/r12_ramp.py make --round 2 -> out/r12_points_r2.csv (after `r11_step.py compare` on r11r2)
    python automation/job.py submit r12r1 --interactive --after rob6 -- matlab -batch "cd('Simulink/Thesis'); r11_closed_loop('out/r12_points_r1.csv', 'out/r12')"
      (10-30 13:58: run as r12r1b behind fineR -> r11r1b -> rob6b, on the renamed copy out/r12_points_r1b.csv; same out/r12)
    python -I -X utf8 Simulink/Thesis/r12_ramp.py compare    -> out/r12_check.{csv,log}, out/r12_verdict.md, out/fig_r12_ramp.png
    python -I -X utf8 Simulink/Thesis/r12_ramp.py selftest
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402
import simulink_sweep as sw  # noqa: E402

OUT = os.path.join(HERE, "out")
RUNS = os.path.join(OUT, "r12")
PROF = os.path.join(OUT, "r12_profiles")
ICROS_CSV = os.path.join(ROOT, "Simulink", "RegHDC_PostProcessed_Dataset.csv")
TSW = 1 / 80e3
TS, SYNC, KP, KI = 2.5e-8, 1, 2.0, 49000.0     # the R11 smoke choice (10-30): 25 ns grid + per-period latch
T_HOLD, T_RAMP, T_END = 0.006, 0.040, 0.050
P0, P1 = 100.0, 900.0
V2S = (90.0, 109.2)
CTRLS = ("ICROS", "M-B", "M-S", "SPS")
WIN = (T_HOLD, T_HOLD + T_RAMP)
ENV_N = 41
M1_RATIO, E_RATIO, SPIKE_A = 5.0, 2.0, 0.10      # "효과 확인"
NONE_E, NONE_SPIKE = 1.2, 0.03                  # "효과 없음"
FACTORS, P2P_MAX, R11_RUNS = (0.5, 0.25), 2.0, 12   # ROUND 2: gain factors in order, deg, R11 runs per factor
_cache = {}


# ---------------------------------------------------------------- controllers
def icros_boundary(V2):
    """FSM lock-in boundary |P| of the dataset labels (HV -> LV side, nearest dataset V2)"""
    import pandas as pd
    d = pd.read_csv(ICROS_CSV)
    vs = np.sort(d["Target_V2_V"].unique())
    v = vs[np.argmin(np.abs(vs - V2))]
    s = d[(d["Target_V2_V"] == v) & (d["Target_P_W"] < 0)].assign(Pa=lambda x: -x["Target_P_W"]).sort_values("Pa")
    sps = np.isclose(s["I_Opt"].to_numpy(), s["I_SPS"].to_numpy(), rtol=0, atol=1e-9)
    first = int(np.argmax(sps)) if sps.any() else len(s)
    if first >= len(s):
        return np.inf
    return 0.5 * (s["Pa"].iloc[first - 1] + s["Pa"].iloc[first]) if first > 0 else 0.0


def icros_model():
    if "ICROS" not in _cache:
        m = ct.LegacyRegHDC(D=32, K=8, Q=32, gamma=8.0, seed=0, name="ICROS2026 RegHDC")
        z = np.load(os.path.join(OUT, "model_legacy_icros.npz"))
        m.C, m.M = z["C"], z["M"]
        _cache["ICROS"] = m
    return _cache["ICROS"]


def duties(ctrl, P, V2):
    """commanded (D1, D2) arrays for |P| array P at V2 (D_SAFE-capped like every Simulink run)"""
    P = np.asarray(P, float)
    if ctrl == "SPS":
        return np.full(P.shape, sw.D_SAFE), np.full(P.shape, sw.D_SAFE)
    if ctrl == "ICROS":
        d1, d2 = icros_model().predict(P, np.full(P.shape, V2), sign=-1.0)
        sps = P >= icros_boundary(V2)
        d1, d2 = np.where(sps, 0.5, d1), np.where(sps, 0.5, d2)
    else:
        if ctrl not in _cache:
            _cache[ctrl] = ct.SmoothRegHDC.from_json(os.path.join(OUT, {"M-B": "model_smooth_reghdc_b3.json",
                                                                         "M-S": "model_smooth_reghdc_MS.json"}[ctrl]))
        d1, d2 = _cache[ctrl].predict(P, np.full(P.shape, V2))
    return np.minimum(np.ravel(d1), sw.D_SAFE), np.minimum(np.ravel(d2), sw.D_SAFE)


def p_ref(t):
    t = np.asarray(t, float)
    return np.where(t < T_HOLD, P0, np.where(t < T_HOLD + T_RAMP, P0 + (P1 - P0) * (t - T_HOLD) / T_RAMP, P1))


def knots():
    return np.round(np.arange(0.0, T_END + 0.5 * TSW, TSW), 12)


# ---------------------------------------------------------------- make
def round2_factor(T=None):
    """the ROUND 2 rule: (factor, {factor: worst steady phase p2p [deg]}) from R11 round 2 (out/r11_check.csv); None if a
    factor's runs are not all there yet"""
    import pandas as pd
    T = pd.read_csv(os.path.join(OUT, "r11_check.csv")) if T is None else T
    worst = {}
    for g in FACTORS:
        x = T[T["label"].str.startswith(f"r2_g{g:g}_") & T["ctrl"].isin(["SPS", "M-B", "M-S"])]
        if x["label"].nunique() < R11_RUNS or "phi_p2p" not in x:
            return None, worst
        worst[g] = float(x["phi_p2p"].max())
    return next((g for g in FACTORS if worst[g] <= P2P_MAX), FACTORS[-1]), worst


def make(rnd=1):
    import pandas as pd
    os.makedirs(PROF, exist_ok=True)
    gain = 1.0
    if rnd == 2:
        gain, worst = round2_factor()
        if gain is None:
            sys.exit("R11 round 2 is not complete: run r11r2, then `r11_step.py compare`")
        print(f"ROUND 2 gain factor {gain:g}, worst steady phase p2p per factor {worst} deg (limit {P2P_MAX:g})"
              + ("" if worst[gain] <= P2P_MAX else "  -- NEITHER factor qualifies: 0.25 used, as registered"))
    rows = []
    t = knots()
    P = p_ref(t)
    for V2 in V2S:
        for ctrl in CTRLS:
            d1, d2 = duties(ctrl, P, V2)
            name = f"r12_{ctrl.replace('-', '')}_{V2:g}V_ramp"
            path = os.path.join(PROF, f"{name}.csv")
            pd.DataFrame(dict(t=t, P_ref=-P, D1=d1, D2=d2, V2=V2)).to_csv(path, index=False, float_format="%.10g")
            rows.append(dict(label=name if rnd == 1 else f"r{rnd}_{ctrl.replace('-', '')}_{V2:g}V_ramp", ctrl=ctrl, V2=V2, Ts=TS, sync=SYNC,
                             Kp=KP * gain, Ki=KI * gain, simTime=T_END, profile=os.path.relpath(path, HERE).replace(os.sep, "/"), trim=1))
            print(f"  {rows[-1]['label']}: m1 (commanded D step) {m1(d1, d2, t):.4f}"
                  + (f", ICROS boundary {icros_boundary(V2):.0f} W" if ctrl == "ICROS" else ""))
    pd.DataFrame(rows).to_csv(os.path.join(OUT, f"r12_points_r{rnd}.csv"), index=False, float_format="%.10g")
    print(f"wrote out/r12_points_r{rnd}.csv: {len(rows)} runs, {len(t)} knots each, Kp {KP * gain:g}, Ki {KI * gain:g}")


# ---------------------------------------------------------------- metrics (the registered rule)
def m1(d1, d2, t):
    w = (t >= WIN[0]) & (t <= WIN[1])
    return float(max(np.abs(np.diff(np.asarray(d1)[w])).max(), np.abs(np.diff(np.asarray(d2)[w])).max()))


def running_median(x, n=ENV_N):
    x = np.asarray(x, float)
    h = n // 2
    pad = np.concatenate([np.full(h, x[0]), x, np.full(h, x[-1])])
    return np.array([np.median(pad[i:i + n]) for i in range(x.size)])


def run_metrics(per, prof):
    """m2, m3 (+ helpers) of one run over the ramp window; per = the runner's *_periods.csv"""
    t = per["t_end"].to_numpy()
    Pd = -per["P_LV"].to_numpy()
    e = Pd - p_ref(t)
    I = per["Ipk"].to_numpy()
    env = running_median(I)
    w = (t > WIN[0] + 1e-9) & (t <= WIN[1] + 1e-9)
    dD = np.nan
    if "D1" in per and per["D1"].notna().any():      # applied duties (per-period means), a check of m1
        dD = float(np.nanmax(np.abs(np.diff(per["D1"].to_numpy()[w]))))
        dD = max(dD, float(np.nanmax(np.abs(np.diff(per["D2"].to_numpy()[w])))))
    k = int(np.argmax(np.where(w, I - env, -np.inf)))
    return dict(e_max=float(np.nanmax(np.abs(e[w]))), e_rms=float(np.sqrt(np.nanmean(e[w] ** 2))),
                spike=float(np.nanmax((I - env)[w])), t_spike_ms=float(t[k] * 1e3), P_spike=float(p_ref(t[k])),
                Ipk_max=float(np.nanmax(I[w])), dD_applied=dD,
                iL_mean_win=float(abs(np.nanmean(per["iL_mean"].to_numpy()[w]))))


def verdict(r):
    """r: {ctrl: {m1, e_max, spike}} for one V2 -> 효과 확인 / 효과 없음 / 부분"""
    a, b = r["ICROS"], r["M-B"]
    if a["m1"] >= M1_RATIO * b["m1"] and (a["e_max"] >= E_RATIO * b["e_max"] or a["spike"] >= b["spike"] + SPIKE_A):
        return "효과 확인"
    if a["e_max"] <= NONE_E * b["e_max"] and a["spike"] <= b["spike"] + NONE_SPIKE:
        return "효과 없음"
    return "부분"


# ---------------------------------------------------------------- compare
def load(rnd=1):
    import pandas as pd
    runs = {}
    for f in sorted(glob.glob(os.path.join(RUNS, ("r12_" if rnd == 1 else f"r{rnd}_") + "*_summary.csv"))):
        s = pd.read_csv(f).iloc[0].to_dict()
        pf = f.replace("_summary.csv", "_periods.csv")
        s["periods"] = pd.read_csv(pf) if os.path.isfile(pf) else None
        prof = os.path.join(HERE, str(s["profile"]))
        s["prof"] = pd.read_csv(prof) if os.path.isfile(prof) else None
        runs[(s["ctrl"], float(s["V2"]))] = s
    return runs


def compare():
    compare_round(1)
    if load(2):
        compare_round(2)


def compare_round(rnd):
    import pandas as pd
    runs = load(rnd)
    lines, rows = [], []
    tag = "" if rnd == 1 else f"_r{rnd}"

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    gains = sorted({float(s["Kp"]) / KP for s in runs.values()})
    out(f"R12 classifier removal, round {rnd} (Kp x {', '.join(f'{g:g}' for g in gains)}), Simulink closed loop (R11 setup: PI outer phase, 25 ns grid + latch), HV -> LV ramp "
        f"{P0:.0f} -> {P1:.0f} W in {T_RAMP * 1e3:.0f} ms; window {WIN[0] * 1e3:.0f}-{WIN[1] * 1e3:.0f} ms")
    out("rule registered 10-30 14:10 before the runs, round 2 gain rule 10-30 15:45 (r12_ramp.py header)")
    md = [f"# R12 분류기 제거 효과 — Simulink 폐루프 전력 램프, {rnd}라운드 (PI 이득 × {', '.join(f'{g:g}' for g in gains)})", "",
          "생성: `r12_ramp.py compare`. 규칙은 2026-10-30 14:10 실행 전에(2라운드 이득 선택은 15:45, R11 2라운드 전에) 이 스크립트 머리말에 정했다. 수치는 모두 Simulink(데이터셋 시기 모델, "
          "R11 폐루프: PI가 외부 위상, 25 ns 격자 + 주기 경계 래치)다.", "",
          "| V2 | 제어기 | m1 듀티 한 주기 최대 변화 | 적용 듀티 변화 | 전력 오차 최대 [W] | 전력 오차 RMS [W] | 피크 튐 m3 [A] (시각, 전력) | 램프 중 최대 Ipk [A] | 창 평균 i_L [A] |",
          "|---|---|---|---|---|---|---|---|---|"]
    verdicts = {}
    for V2 in V2S:
        res = {}
        for ctrl in CTRLS:
            s = runs.get((ctrl, V2))
            if s is None or s["periods"] is None or s["prof"] is None:
                out(f"{V2:g} V {ctrl}: no run yet")
                continue
            if not isinstance(s.get("error"), float) and str(s.get("error", "")).strip() not in ("", "nan"):
                out(f"{V2:g} V {ctrl}: ERROR {s['error']}")
                continue
            pr = s["prof"]
            r = dict(m1=m1(pr["D1"].to_numpy(), pr["D2"].to_numpy(), pr["t"].to_numpy()),
                     **run_metrics(s["periods"], pr))
            res[ctrl] = r
            rows.append(dict(V2=V2, ctrl=ctrl, **r))
            out(f"{V2:5.1f} V {ctrl:5s}: m1 {r['m1']:.4f} (applied {r['dD_applied']:.4f}) | e max {r['e_max']:6.1f} W,"
                f" rms {r['e_rms']:5.1f} W | spike {r['spike']:.3f} A at {r['t_spike_ms']:.2f} ms ({r['P_spike']:.0f} W) |"
                f" Ipk max {r['Ipk_max']:.3f} A | window mean i_L {r['iL_mean_win']:.4f} A")
            md.append(f"| {V2:g} V | {ctrl} | {r['m1']:.4f} | {r['dD_applied']:.4f} | {r['e_max']:.1f} | {r['e_rms']:.1f} | "
                      f"{r['spike']:.3f} ({r['t_spike_ms']:.1f} ms, {r['P_spike']:.0f} W) | {r['Ipk_max']:.3f} | {r['iL_mean_win']:.4f} |")
        if "ICROS" in res and "M-B" in res:
            verdicts[V2] = verdict(res)
            out(f"  verdict {V2:g} V: {verdicts[V2]}")
    md += ["", "**판정(규칙)**: " + (", ".join(f"{V2:g} V {v}" for V2, v in verdicts.items()) if verdicts else "실행 결과 없음"),
           "", "m1 = 명령 듀티의 한 주기(12.5 µs) 최대 변화, 적용 듀티 변화 = 실행 출력(주기 평균)의 같은 값, 전력 오차 = 주기마다 "
           "(전달 전력 − 기준), 피크 튐 = 주기 피크 − 41주기 이동 중앙값의 최대."]
    if rows:
        pd.DataFrame(rows).to_csv(os.path.join(OUT, f"r12_check{tag}.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, f"r12_check{tag}.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    open(os.path.join(OUT, f"r12_verdict{tag}.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    if runs:
        fig(runs, tag)
    print(f"wrote out/r12_check{tag}.{{csv,log}}, out/r12_verdict{tag}.md" + (f", out/fig_r12_ramp{tag}.png" if runs else ""))


def fig(runs, tag=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, MUTED, ORANGE, YELLOW, _style
    _style(plt)
    col = {"SPS": MUTED, "ICROS": ORANGE, "M-B": BLUE, "M-S": YELLOW}
    f, ax = plt.subplots(3, len(V2S), figsize=(5.2 * len(V2S), 8.4), squeeze=False, sharex=True)
    for j, V2 in enumerate(V2S):
        top = 0.0
        for ctrl in CTRLS:
            s = runs.get((ctrl, V2))
            if s is None or s["periods"] is None:
                continue
            p, pr = s["periods"], s["prof"]
            ax[0, j].plot(pr["t"] * 1e3, pr["D1"], color=col[ctrl], lw=1.2, label=f"{ctrl} D1")
            ax[0, j].plot(pr["t"] * 1e3, pr["D2"], color=col[ctrl], lw=1.0, ls="--")
            ax[1, j].plot(p["t_end"] * 1e3, -p["P_LV"] - p_ref(p["t_end"]), color=col[ctrl], lw=0.9, label=ctrl)
            ax[2, j].plot(p["t_end"] * 1e3, p["Ipk"], color=col[ctrl], lw=0.9, label=ctrl)
            top = max(top, float(p.loc[p["t_end"] >= WIN[0], "Ipk"].max()))
        ax[0, j].set_title(f"{V2:g} V: commanded D1 (solid), D2 (dashed)")
        ax[1, j].set_title("power error P_delivered - P_ref [W]")
        ax[2, j].set_title("max |i_L| per period [A] (start-up clipped)")
        ax[1, j].set_ylim(-150, 150)
        if top > 0:
            ax[2, j].set_ylim(0, 1.1 * top)
        for a in ax[:, j]:
            a.axvspan(0, WIN[0] * 1e3, color="0.92", lw=0)
        ax[2, j].set_xlabel("t [ms]  (ramp 100 -> 900 W from 6 to 46 ms)")
    ax[0, 0].legend(frameon=False, fontsize=8, ncol=2)
    ax[2, 0].legend(frameon=False, fontsize=8)
    f.suptitle("R12: SPS/TPS classifier (ICROS2026 structure) vs single regression, Simulink closed loop", fontsize=11)
    f.tight_layout()
    f.savefig(os.path.join(OUT, f"fig_r12_ramp{tag}.png"), dpi=130)
    plt.close(f)


# ---------------------------------------------------------------- selftest
def selftest():
    """the registered rule on fake inputs, the controllers on the ramp (Python only)"""
    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        good = got == want
        ok &= good
        print(f"  {'PASS' if good else 'FAIL'} {name}: {got} (want {want})")

    base = dict(m1=0.001, e_max=10.0, spike=0.05)
    chk("effect via tracking", verdict({"ICROS": dict(m1=0.1, e_max=25.0, spike=0.05), "M-B": base}), "효과 확인")
    chk("effect via spike", verdict({"ICROS": dict(m1=0.1, e_max=11.0, spike=0.16), "M-B": base}), "효과 확인")
    chk("no effect", verdict({"ICROS": dict(m1=0.1, e_max=11.0, spike=0.07), "M-B": base}), "효과 없음")
    chk("partial", verdict({"ICROS": dict(m1=0.1, e_max=15.0, spike=0.07), "M-B": base}), "부분")
    chk("no D steps -> not confirmed", verdict({"ICROS": dict(m1=0.003, e_max=30.0, spike=0.3), "M-B": base}), "부분")
    chk("ICROS boundary 90 V", icros_boundary(90.0), 625.0)
    chk("ICROS boundary 109.2 V (110 V row)", icros_boundary(109.2), 775.0)
    t = knots()
    P = p_ref(t)
    chk("ramp ends", (P[0], P[-1], round(float(p_ref(0.026)), 6)), (100.0, 900.0, 500.0))
    d1, d2 = duties("ICROS", P, 90.0)
    chk("ICROS SPS above the boundary", bool(np.all(d1[P >= 625.0] == sw.D_SAFE)), True)
    e1, e2 = duties("M-B", P, 90.0)
    chk("ICROS commanded step >> M-B", m1(d1, d2, t) > 20 * m1(e1, e2, t), True)
    # fake run: a power error spike after each ICROS jump is caught by m2, a step in Ipk by m3
    tt = np.arange(1, int(T_END / TSW) + 1) * TSW
    per = pd.DataFrame(dict(t_end=tt, Ipk=1.0 + 0.002 * (tt > 0.02) + 0.3 * ((tt > 0.03) & (tt < 0.0302)),
                            iL_mean=0.0, P_LV=-(p_ref(tt) + 40.0 * ((tt > 0.03) & (tt < 0.0305)))))
    r = run_metrics(per, None)
    chk("fake e_max = the 40 W excursion", round(r["e_max"], 6), 40.0)
    chk("fake spike = the 0.3 A burst", round(r["spike"], 3), 0.3)

    def r11(p05, p025):
        """a fake out/r11_check.csv: 12 runs per factor, worst phase p2p as given"""
        return pd.DataFrame([dict(label=f"r2_g{g:g}_x{k}", ctrl="SPS", phi_p2p=(w if k == 0 else 0.5)) for g, w in ((0.5, p05), (0.25, p025))
                             for k in range(R11_RUNS)])
    chk("round 2: 0.5 qualifies", round2_factor(r11(1.5, 1.0))[0], 0.5)
    chk("round 2: only 0.25 qualifies", round2_factor(r11(5.0, 1.0))[0], 0.25)
    chk("round 2: neither -> 0.25", round2_factor(r11(5.0, 4.0))[0], 0.25)
    chk("round 2: exactly 2 deg qualifies", round2_factor(r11(2.0, 1.0))[0], 0.5)
    chk("round 2: incomplete -> None", round2_factor(r11(1.0, 1.0).iloc[:-1])[0], None)
    print("ALL PASS" if ok else "FAILED")
    return ok


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("make", "compare", "selftest"))
    ap.add_argument("--round", type=int, default=1, help="make: 1 = the R11 gains, 2 = the ROUND 2 gain factor")
    a = ap.parse_args()
    if a.cmd == "make":
        make(a.round)
    elif a.cmd == "compare":
        compare()
    else:
        sys.exit(0 if selftest() else 1)
