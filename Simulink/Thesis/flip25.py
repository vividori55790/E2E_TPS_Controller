"""
flip25.py -- (2026-10-31 02:18) the closed loop through zero power: HV -> LV 300 W to LV -> HV 300 W and back (R11 setup: dataset-era
model, PI on the outer phase, 25 ns grid + per-period latch, gain x 0.5, runner r11_closed_loop.m).

Why: the converter is bidirectional and the controller's input is |P|. So far the PI has only run HV -> LV, and rev25 measures LV -> HV
with a constant D0 only. On the way through zero (a) the surface runs down to its floor D 0.1 and back: it is continuous there but, as a
function of the signed P, has a kink (|dD/dP| about 0.001 per W at 0 W, surface B); (b) the phase gain dP/dphi is small at small D;
(c) P(D0) has the dead-time plateau around D0 = 0; (d) D0 changes sign; (e) the controller's own PI changes feedback at P_ref = 0
(DAB_Controller_System.m STAGE 3: LV current against P_ref / V_LV when P_ref <= 0, HV current against P_ref / V_HV when P_ref > 0), so the
same power error is a current error about V_LV / V_HV as large, about a quarter, in the LV -> HV direction. Nothing here has been run yet, so
this is a test, not a demonstration.

Runs: V2 {84, 95, 109.2} V x {SPS, M-B} = 6, 40 ms. P_ref: -300 W (HV -> LV, the controller's sign) for 6 ms, ramp to +300 W in 12 ms
(50 W/ms), +300 W for 6 ms, ramp back in 12 ms, -300 W for 4 ms. One knot per switching period, D1 / D2 from |P_ref| at each knot.

RULE (registered 2026-10-31 02:18, before any run of this file; do not change after the results). Over the two ramps (6-18, 24-36 ms), per
run: e = P - P_ref per period with P the power the PI regulates (P_LV if P_ref <= 0, P_HV if P_ref > 0), e_rms, e_max; spike = max(Ipk -
41-period running median), as r12_ramp m3; Ipk_max. Per V2, B against SPS:
  F1 tracking   e_rms(B) <= max(50 W, 2 e_rms(SPS))
  F2 spike      spike(B) <= spike(SPS) + 0.10 A
  F3 peak       Ipk_max(B) <= Ipk_max(SPS) + max(0.10 A, 5 % Ipk_max(SPS))
"문제 없음" if all three hold, otherwise "B 비용 있음" with the ones that fail.
Reported alongside: median and largest period peak where |P_ref| <= 50 W (the part of the surface no steady test has reached); the
commanded duty step per period; the hold medians (last 2 ms) against the constant-D0 values, LV -> HV against rev25 once it has run,
HV -> LV against the 25 ns map; the time from P_ref crossing 0 to P_LV crossing 0.

    python -I -X utf8 Simulink/Thesis/flip25.py make     -> out/flip25_points.csv, out/flip25_profiles/
    python automation/job.py submit flip25 --interactive --after rev25 -- matlab -batch "cd('Simulink/Thesis'); r11_closed_loop('out/flip25_points.csv', 'out/flip25')"
    python -I -X utf8 Simulink/Thesis/flip25.py compare  -> out/flip25_check.{csv,log}, out/flip25_verdict.md, out/fig_flip25.png
    python -I -X utf8 Simulink/Thesis/flip25.py selftest
"""
import argparse
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import r12_ramp as r12  # noqa: E402

OUT = os.path.join(HERE, "out")
RUNS = os.path.join(OUT, "flip25")
PROF = os.path.join(OUT, "flip25_profiles")
TSW = r12.TSW
TS, SYNC = r12.TS, r12.SYNC
GAIN = 0.5                                  # R11 round 2: the largest factor with no limit cycle (r12_ramp.py ROUND 2)
KP, KI = r12.KP * GAIN, r12.KI * GAIN
V2S = (84.0, 95.0, 109.2)
CTRLS = ("SPS", "M-B")
PW = 300.0
T1, T2, T3, T4, T_END = 0.006, 0.018, 0.024, 0.036, 0.040
RAMPS = ((T1, T2), (T3, T4))
HOLDS = (("HV->LV", T1 - 0.002, T1), ("LV->HV", T3 - 0.002, T3), ("HV->LV again", T_END - 0.002, T_END))
ZERO = 50.0                                 # [W] |P_ref| at or below this = the bottom of the surface
E_ABS, E_RATIO, SPIKE_A, PK_ABS, PK_REL = 50.0, 2.0, 0.10, 0.10, 0.05


def p_ref(t):
    """signed, controller's sign: < 0 = HV -> LV"""
    t = np.asarray(t, float)
    return np.interp(t, (0.0, T1, T2, T3, T4, T_END), (-PW, -PW, PW, PW, -PW, -PW))


def knots():
    return np.round(np.arange(0.0, T_END + 0.5 * TSW, TSW), 12)


def make():
    import pandas as pd
    os.makedirs(PROF, exist_ok=True)
    t = knots()
    P = p_ref(t)
    rows = []
    for V2 in V2S:
        for ctrl in CTRLS:
            d1, d2 = r12.duties(ctrl, np.abs(P), V2)
            name = f"flip_{ctrl.replace('-', '')}_{V2:g}V"
            path = os.path.join(PROF, f"{name}.csv")
            pd.DataFrame(dict(t=t, P_ref=P, D1=d1, D2=d2, V2=V2)).to_csv(path, index=False, float_format="%.10g")
            rows.append(dict(label=name, ctrl=ctrl, V2=V2, Ts=TS, sync=SYNC, Kp=KP, Ki=KI, simTime=T_END, profile=os.path.relpath(path, HERE).replace(os.sep, "/"), trim=1))
            print(f"  {name}: duty at 0 W ({np.min(d1):.3f}, {np.min(d2):.3f}), commanded duty step {max(np.abs(np.diff(d1)).max(), np.abs(np.diff(d2)).max()):.5f} per period")
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "flip25_points.csv"), index=False, float_format="%.10g")
    print(f"wrote out/flip25_points.csv: {len(rows)} runs, {len(t)} knots each, Kp {KP:g}, Ki {KI:g}")


def run_metrics(per):
    t = per["t_end"].to_numpy()
    Pr = p_ref(t)
    e = np.where(Pr <= 0, per["P_LV"].to_numpy(), per["P_HV"].to_numpy()) - Pr      # the power the PI regulates, see the header
    I = per["Ipk"].to_numpy()
    env = r12.running_median(I)
    w = np.zeros(t.size, bool)
    for a, b in RAMPS:
        w |= (t > a + 1e-9) & (t <= b + 1e-9)
    z = w & (np.abs(Pr) <= ZERO)
    r = dict(e_rms=float(np.sqrt(np.nanmean(e[w] ** 2))), e_max=float(np.nanmax(np.abs(e[w]))), spike=float(np.nanmax((I - env)[w])), Ipk_max=float(np.nanmax(I[w])),
             Ipk_zero_med=float(np.nanmedian(I[z])), Ipk_zero_max=float(np.nanmax(I[z])), e_zero_max=float(np.nanmax(np.abs(e[z]))), t_spike_ms=float(t[int(np.argmax(np.where(w, I - env, -np.inf)))] * 1e3))
    for name, a, b in HOLDS:
        h = (t > a + 1e-9) & (t <= b + 1e-9)
        r[f"med {name}"] = float(np.nanmedian(I[h]))
    lag = []
    Ps = per["P_LV"].to_numpy()
    for a, b, tz in ((T1, T2, 0.5 * (T1 + T2)), (T3, T4, 0.5 * (T3 + T4))):
        s = np.sign(p_ref(b))
        k = np.where((t > tz) & (s * Ps > 0))[0]            # the source power has the new sign for good from here
        bad = np.where((t > tz) & (t <= b) & (s * Ps <= 0))[0]
        lag.append(float((t[bad[-1] + 1] if len(bad) else t[k[0]]) - tz) * 1e3 if len(k) else np.nan)
    r["lag_ms"] = float(np.nanmax(lag))
    return r


def verdict(s, b):
    """the registered rule: s, b = run_metrics of SPS and M-B at one V2 -> (text, failed)"""
    fail = []
    if b["e_rms"] > max(E_ABS, E_RATIO * s["e_rms"]):
        fail.append("F1")
    if b["spike"] > s["spike"] + SPIKE_A:
        fail.append("F2")
    if b["Ipk_max"] > s["Ipk_max"] + max(PK_ABS, PK_REL * s["Ipk_max"]):
        fail.append("F3")
    return ("문제 없음" if not fail else "B 비용 있음"), fail


def load():
    import pandas as pd
    runs = {}
    for f in sorted(glob.glob(os.path.join(RUNS, "flip_*_summary.csv"))):
        s = pd.read_csv(f).iloc[0].to_dict()
        pf = f.replace("_summary.csv", "_periods.csv")
        s["periods"] = pd.read_csv(pf) if os.path.isfile(pf) else None
        runs[(s["ctrl"], float(s["V2"]))] = s
    return runs


def refs():
    """the constant-D0 values at 300 W: (ctrl, V2, direction) -> median period peak; the 25 ns map, rev25 if it has run"""
    import pandas as pd
    out = {}
    p = os.path.join(OUT, "r13_points_med.csv")
    if os.path.isfile(p):
        for r in pd.read_csv(p).itertuples():
            if r.P == PW:
                out[("SPS", r.V2, "HV->LV")], out[("M-B", r.V2, "HV->LV")] = r.med_SPS, r.med_MB
    p = os.path.join(OUT, "rev25_check.csv")
    if os.path.isfile(p):
        for r in pd.read_csv(p).itertuples():
            if r.P == PW:
                out[("SPS", r.V2, "LV->HV")], out[("M-B", r.V2, "LV->HV")] = r.med_SPS_r, r.med_MB_r
    return out


def compare():
    import datetime

    import pandas as pd
    runs = load()
    if not runs:
        sys.exit("no out/flip25/ runs yet")
    ref = refs()
    lines, rows, md = [], [], ["# 전력 부호가 바뀌는 폐루프 (자동 생성)", "",
                               f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `flip25.py compare`. 규칙은 실행 전(10-31 02:18) `flip25.py` 머리말에 등록했다. 모두 Simulink(데이터셋 시기 모델, PI 폐루프, 25 ns + 래치, 이득 ½).", "",
                               "| V2 | 제어기 | 전력 오차 RMS / 최대 [W] | 피크 튐 [A] | 램프 최대 Ipk [A] | \\|P_ref\\| ≤ 50 W: Ipk 중앙값 / 최대 [A], 오차 최대 [W] | 부호 지연 [ms] | 유지 중앙값 HV→LV, LV→HV, HV→LV [A] |", "|---|---|---|---|---|---|---|---|"]

    def out(x=""):
        print(x, flush=True)
        lines.append(x)
    out("flip25: HV->LV 300 W -> LV->HV 300 W -> HV->LV 300 W, Simulink closed loop, 25 ns + latch, gain x 0.5; rule in the file header")
    verdicts = {}
    for V2 in V2S:
        res = {}
        for ctrl in CTRLS:
            s = runs.get((ctrl, V2))
            if s is None or s["periods"] is None:
                out(f"{V2:g} V {ctrl}: no run yet")
                continue
            if not isinstance(s.get("error"), float) and str(s.get("error", "")).strip() not in ("", "nan"):
                out(f"{V2:g} V {ctrl}: ERROR {s['error']}")
                continue
            r = run_metrics(s["periods"])
            res[ctrl] = r
            rows.append(dict(V2=V2, ctrl=ctrl, **r))
            hold = ", ".join(f"{n} {r[f'med {n}']:.3f}" + (f" (constant D0 {ref[(ctrl, V2, n.split()[0])]:.3f})" if (ctrl, V2, n.split()[0]) in ref else "") for n, _, _ in HOLDS)
            out(f"{V2:5.1f} V {ctrl:3s}: e rms {r['e_rms']:6.1f}, max {r['e_max']:6.1f} W | spike {r['spike']:.3f} A at {r['t_spike_ms']:.2f} ms | Ipk max {r['Ipk_max']:.3f} A | |P_ref| <= {ZERO:.0f} W: Ipk median {r['Ipk_zero_med']:.3f}, "
                f"max {r['Ipk_zero_max']:.3f} A, e max {r['e_zero_max']:.1f} W | sign lag {r['lag_ms']:.2f} ms | hold {hold}")
            md.append(f"| {V2:g} V | {ctrl} | {r['e_rms']:.1f} / {r['e_max']:.1f} | {r['spike']:.3f} ({r['t_spike_ms']:.1f} ms) | {r['Ipk_max']:.3f} | {r['Ipk_zero_med']:.3f} / {r['Ipk_zero_max']:.3f}, {r['e_zero_max']:.1f} | {r['lag_ms']:.2f} | "
                      + ", ".join(f"{r[f'med {n}']:.3f}" for n, _, _ in HOLDS) + " |")
        if len(res) == len(CTRLS):
            verdicts[V2] = verdict(res["SPS"], res["M-B"])
            out(f"  verdict {V2:g} V: {verdicts[V2][0]} {verdicts[V2][1]}")
    md += ["", "**판정(규칙)**: " + (", ".join(f"{V2:g} V {v[0]}{' ' + str(v[1]) if v[1] else ''}" for V2, v in verdicts.items()) if verdicts else "실행 결과 없음"), "",
           f"전력 오차 = 주기마다 P_LV − P_ref, 피크 튐 = 주기 피크 − 41주기 이동 중앙값의 최대(두 램프). 부호 지연 = P_ref가 0을 지난 뒤 P_LV가 새 부호로 자리 잡기까지. F1 RMS ≤ max({E_ABS:.0f} W, SPS의 {E_RATIO:.0f}배), F2 튐 ≤ SPS + {SPIKE_A} A, F3 최대 ≤ SPS + max({PK_ABS} A, {100 * PK_REL:.0f} %)."]
    if rows:
        pd.DataFrame(rows).to_csv(os.path.join(OUT, "flip25_check.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "flip25_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    open(os.path.join(OUT, "flip25_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    fig(runs)


def fig(runs):
    """one column per V2; period peak, source power against the command, phase, duties, each on its own axes"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from evaluate import BLUE, MUTED, _style
    _style(plt)
    f, ax = plt.subplots(4, len(V2S), figsize=(4.6 * len(V2S), 8.6), squeeze=False, sharex=True, gridspec_kw=dict(height_ratios=(3, 2, 2, 1.5)))
    for j, V2 in enumerate(V2S):
        for ctrl, c in (("SPS", MUTED), ("M-B", BLUE)):
            s = runs.get((ctrl, V2))
            if s is None or s["periods"] is None:
                continue
            p = s["periods"]
            p = p[p["t_end"] >= 3e-3]
            t = p["t_end"] * 1e3
            ax[0, j].plot(t, p["Ipk"], color=c, lw=1.0, label=ctrl)
            ax[1, j].plot(t, p["P_LV"], color=c, lw=1.0)
            ax[2, j].plot(t, p["phi"], color=c, lw=1.0)
            ax[3, j].plot(t, p["D1"], color=c, lw=1.0)
            ax[3, j].plot(t, p["D2"], color=c, lw=1.0, ls=":")
        tt = knots()
        ax[1, j].plot(tt[tt >= 3e-3] * 1e3, p_ref(tt[tt >= 3e-3]), color="#333333", ls="--", lw=0.8)
        ax[1, j].set_ylim(-1.6 * PW, 1.6 * PW)
        ax[0, j].set_title(f"{V2:g} V: Ipk per period", fontsize=9)
        ax[1, j].set_title("source power P_LV, one-period average (dashed = command)", fontsize=9)
        ax[2, j].set_title("PI phase", fontsize=9)
        ax[3, j].set_title("commanded D1 (solid), D2 (dotted)", fontsize=9)
        ax[3, j].set_xlabel("t [ms]")
    ax[0, 0].set_ylabel("[A]")
    ax[1, 0].set_ylabel("[W]")
    ax[2, 0].set_ylabel("[deg]")
    ax[0, 0].legend(frameon=False, fontsize=8)
    f.suptitle("Closed loop through zero power: HV->LV 300 W -> LV->HV 300 W -> HV->LV 300 W (25 ns edge grid, period latch, gain x0.5; first 3 ms not shown)", fontsize=11)
    f.tight_layout()
    f.savefig(os.path.join(OUT, "fig_flip25.png"), dpi=130)
    plt.close(f)


def selftest():
    """the registered rule end to end on made-up runs"""
    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("P_ref starts HV -> LV", float(p_ref(0.0)), -PW)
    chk("P_ref crosses 0 mid-ramp", round(float(p_ref(0.5 * (T1 + T2))), 6), 0.0)
    chk("P_ref is LV -> HV in the hold", float(p_ref(0.5 * (T2 + T3))), PW)

    def per(lag=0.0, spike=0.0, err=0.0, base=1.0):
        t = knots()[1:]
        P = p_ref(t - lag) + err * np.sin(2 * np.pi * t / 1e-3)
        I = base + np.abs(p_ref(t)) / 300.0
        I[int(np.argmin(np.abs(t - 0.012)))] += spike
        return pd.DataFrame(dict(t_end=t, P_LV=P, P_HV=P, Ipk=I))
    s, b = run_metrics(per()), run_metrics(per(spike=0.5))
    chk("no error, no spike", (round(s["e_rms"], 6), round(s["spike"], 6)), (0.0, 0.0))
    chk("the spike is found (0.5 A over a V-shaped peak: the running median sits up to 0.03 A above the bottom)", bool(0.45 <= b["spike"] <= 0.5), True)
    chk("the spike's time", round(b["t_spike_ms"], 1), 12.0)
    chk("F2 fails on the spike", verdict(s, b), ("B 비용 있음", ["F2", "F3"]) if b["Ipk_max"] > s["Ipk_max"] + PK_ABS else ("B 비용 있음", ["F2"]))
    chk("lag read back", round(run_metrics(per(lag=0.5e-3))["lag_ms"], 1), 0.5)
    chk("same runs -> no problem", verdict(s, s), ("문제 없음", []))
    t = run_metrics(per(err=100.0))
    chk("F1 fails on large tracking error", verdict(s, t)[1], ["F1"])
    chk("F1 passes within twice SPS", verdict(dict(s, e_rms=60.0), dict(s, e_rms=110.0))[1], [])
    chk("F3 uses 5 % of SPS when larger", verdict(dict(s, Ipk_max=4.0), dict(s, Ipk_max=4.15))[1], [])
    chk("F3 fails above 5 %", verdict(dict(s, Ipk_max=4.0), dict(s, Ipk_max=4.25))[1], ["F3"])
    chk("the zero window sees only |P_ref| <= 50 W", round(s["Ipk_zero_max"], 3), round(1.0 + ZERO / 300.0, 3))
    w = per()
    chk("LV -> HV: the HV side is the regulated power, a loss on P_LV is no error", round(run_metrics(w.assign(P_LV=lambda d: np.where(p_ref(d["t_end"]) > 0, d["P_LV"] + 5.0, d["P_LV"])))["e_max"], 6), 0.0)
    chk("LV -> HV: an HV-side error counts", round(run_metrics(w.assign(P_HV=lambda d: np.where(p_ref(d["t_end"]) > 0, d["P_HV"] + 5.0, d["P_HV"])))["e_max"], 6), 5.0)
    print("ALL PASS" if ok else "SOME FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["make", "compare", "selftest"])
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    {"make": make, "compare": compare}[a.cmd]()


if __name__ == "__main__":
    main()
