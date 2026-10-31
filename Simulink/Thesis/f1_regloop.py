"""
f1_regloop.py -- (2026-10-31, final deadline F1 / V3) the regression INSIDE the Simulink controller, in a closed loop.

Until now no Simulink run has executed the regression: M-B's D1, D2 were computed in Python and fed in as time series from the source V2 and the target |P|. A product runs it on the measured bus voltages. This puts it
in the controller copy v3/DAB_Controller_System.m (= the r11 copy, RegressionInLoop = true): smooth_reghdc_infer_v3(P_cmd, V_LV_f, V_HV_f) on the one-period moving averages, the V1-normalised input of the proposal, every
RegHoldPeriods switching periods, capped at D_SAFE as every run. smooth_reghdc_infer_v3.m is smooth_reghdc_infer.m without its persistent variable, which a MATLAB System block refuses (smoke 10-31); equal to it to 0 on the test vectors.

Runs (f1_closed_loop.m, the setup of R12 round 2: PI on the outer phase, gains x 0.5 = Kp 1, Ki 24500, 25 ns, per-period latch, HV -> LV ramp 100 -> 900 W in 40 ms, 50 ms, V1 380 V):
  V2 {90, 109.2} V x hold {1, 8} periods (12.5 us, 100 us; 100 us is an allowance for an MCU that needs tens of us per inference, nothing has been measured on an MCU) + control: the copy with RegressionInLoop = false, against the existing external runs out/r12/r2_MB_*.

RULES (registered 2026-10-31 14:41, before any run of this file; do not change after the results). Window = the ramp, 6-46 ms, as R12. Against the external run of the same V2, per period:
  C  control: max |Ipk(copy, external path) - Ipk(r2_MB)| <= 0.001 A at both V2 -> "복사본 경로 같음", else the comparison below is reported but "판정 불가".
  S  in-loop against external, each V2 and hold: "같음" iff  dD = max |D(in-loop) - D(external)| over D1, D2 <= 0.01 and |e_max difference| <= 15 W and |spike difference| <= 0.106 A (one 25 ns grid step) and |Ipk_max difference| <= 0.106 A; else "다름".
     e_max, spike, Ipk_max = r12_ramp.run_metrics, the numbers of R12's table. 15 W: the RMS tracking error of those runs is 10-12 W.
  V  "제어기 안에서도 같음" iff both V2 at hold 1 are "같음" and C holds. "다름" iff C holds and one is "다름". Hold 8 gets its own line under the same rule and does not enter V.
What the answer means. "같음": the report says the regression ran inside the Simulink controller in this one closed loop (two V2, one ramp, 25 ns, copy, gains x 0.5) with the same result as the external input, and the steady-state tables stay external input, said so. "다름": the
in-loop numbers are given and the difference is named; the external-input tables then carry that as a limit. Not a test of an MCU: same double precision, no quantisation, no measured inference time, no ADC.
[참고, before the run] from the external runs' measured V_LV and the Python model: V_LV - V2 +0.05 ... +0.51 V (90 V), +0.05 ... +0.41 V (109.2 V); predicted dD 0.003 at 90 V, 0.000 at 109.2 V (the input clips at 109.2 V); V_HV was not recorded there.
Reported, never a verdict: V_LV, V_HV over the ramp; steady 900 W tail (last 2 ms) Ipk and power; m1 of the applied duties; the start-up (0-6 ms), when the moving averages begin at zero.

    python -I -X utf8 Simulink/Thesis/f1_regloop.py selftest
    python -I -X utf8 Simulink/Thesis/f1_regloop.py make       -> out/f1v3_points.csv
    python automation/job.py submit f1v3 --interactive --after rev6 -- matlab -softwareopengl -batch "cd('Simulink/Thesis'); f1_closed_loop('out/f1v3_points.csv', 'out/f1v3', 6)"
    python -I -X utf8 Simulink/Thesis/f1_regloop.py compare    -> out/f1v3_check.{csv,log}, out/f1v3_verdict.md
"""
import argparse
import datetime
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import r12_ramp as r12  # noqa: E402

OUT = os.path.join(HERE, "out")
RUNS = os.path.join(OUT, "f1v3")
V2S = r12.V2S
HOLDS = (1, 8)
KP, KI = 1.0, 24500.0                       # R12 round 2 (out/r12/r2_MB_*_summary.csv)
DD_MAX, E_MAX, SPIKE_MAX, IPK_MAX, CTL_MAX = 0.01, 15.0, 0.106, 0.106, 0.001


def make():
    import pandas as pd
    rows = []
    for V2 in V2S:
        prof = f"out/r12_profiles/r12_MB_{V2:g}V_ramp.csv"
        if not os.path.isfile(os.path.join(HERE, prof)):
            sys.exit(f"missing {prof}")
        base = dict(ctrl="M-B", V2=V2, Ts=r12.TS, sync=r12.SYNC, Kp=KP, Ki=KI, simTime=r12.T_END, profile=prof, trim=1)
        rows.append(dict(label=f"v3_ctl_{V2:g}V", **base, reg=0, hold=1))
        for h in HOLDS:
            rows.append(dict(label=f"v3_MB_{V2:g}V_h{h}", **base, reg=1, hold=h))
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "f1v3_points.csv"), index=False, float_format="%.10g")
    print(f"wrote out/f1v3_points.csv: {len(rows)} runs")


def rule_s(dD, de, ds, di):
    return "같음" if dD <= DD_MAX and abs(de) <= E_MAX and abs(ds) <= SPIKE_MAX and abs(di) <= IPK_MAX else "다름"


def rule_v(ctl, s1):
    """ctl: {V2: max |dIpk|}, s1: {V2: "같음" / "다름"} at hold 1"""
    if len(ctl) < len(V2S) or len(s1) < len(V2S) or any(not (np.isfinite(x) and x <= CTL_MAX) for x in ctl.values()):
        return "판정 불가"
    return "제어기 안에서도 같음" if all(x == "같음" for x in s1.values()) else "다름"


def _ok(s):
    return s is not None and (isinstance(s.get("error"), float) or str(s.get("error", "")).strip() in ("", "nan"))


def load_run(label, folder=RUNS):
    import pandas as pd
    f = os.path.join(folder, f"{label}_summary.csv")
    if not os.path.isfile(f):
        return None
    s = pd.read_csv(f).iloc[0].to_dict()
    p = os.path.join(folder, f"{label}_periods.csv")
    s["periods"] = pd.read_csv(p) if os.path.isfile(p) else None
    return s


def compare(get=None, ext=None, write_out=True):
    import pandas as pd
    get = load_run if get is None else get
    ext = (lambda v: load_run(f"r2_MB_{v:g}V_ramp", os.path.join(OUT, "r12"))) if ext is None else ext
    lines, rows, ctl, s1 = ["f1_regloop: the regression inside the Simulink controller against the external input, R12 round 2 setup, ramp window 6-46 ms; rules in the file header", ""], [], {}, {}
    md_rows = []
    for V2 in V2S:
        e = ext(V2)
        if e is None or e["periods"] is None:
            lines.append(f"{V2:g} V: no external run")
            continue
        pe = e["periods"]
        t = pe["t_end"].to_numpy()
        w = (t > r12.WIN[0] + 1e-9) & (t <= r12.WIN[1] + 1e-9)
        me = r12.run_metrics(pe, None)
        lines.append(f"{V2:g} V external: e_max {me['e_max']:.1f} W, rms {me['e_rms']:.1f} W, spike {me['spike']:.3f} A, Ipk max {me['Ipk_max']:.3f} A")
        c = get(f"v3_ctl_{V2:g}V")
        if _ok(c) and c["periods"] is not None and len(c["periods"]) == len(pe):
            ctl[V2] = float(np.nanmax(np.abs(c["periods"]["Ipk"].to_numpy() - pe["Ipk"].to_numpy())))
            lines.append(f"  control (copy, external path): max |dIpk| {ctl[V2]:.6f} A over the whole run")
        else:
            lines.append(f"  control: {'ERROR ' + str(c.get('error')) if c else 'no run yet'}")
        for h in HOLDS:
            r = get(f"v3_MB_{V2:g}V_h{h}")
            if not _ok(r) or r["periods"] is None or len(r["periods"]) != len(pe):
                lines.append(f"  hold {h}: {'ERROR ' + str(r.get('error')) if r else 'no run yet'}")
                continue
            p = r["periods"]
            m = r12.run_metrics(p, None)
            dD = float(max(np.nanmax(np.abs(p["D1"].to_numpy() - pe["D1"].to_numpy())[w]), np.nanmax(np.abs(p["D2"].to_numpy() - pe["D2"].to_numpy())[w])))
            de, ds, di = m["e_max"] - me["e_max"], m["spike"] - me["spike"], m["Ipk_max"] - me["Ipk_max"]
            code = rule_s(dD, de, ds, di)
            if h == 1:
                s1[V2] = code
            dI = np.abs(p["Ipk"].to_numpy() - pe["Ipk"].to_numpy())[w]
            tail = t > r12.T_END - 0.002
            lines.append(f"  hold {h}: {code} | dD {dD:.4f}, e_max {m['e_max']:.1f} ({de:+.1f}) W, rms {m['e_rms']:.1f} W, spike {m['spike']:.3f} ({ds:+.3f}) A, Ipk max {m['Ipk_max']:.3f} ({di:+.3f}) A | period |dIpk| median {np.nanmedian(dI):.3f}, 95 % {np.nanpercentile(dI, 95):.3f} A"
                         f" | V_LV - V2 {np.nanmin(p['V_LV'].to_numpy()[w] - V2):+.3f} ... {np.nanmax(p['V_LV'].to_numpy()[w] - V2):+.3f} V, V_HV {np.nanmin(p['V_HV'].to_numpy()[w]):.2f} ... {np.nanmax(p['V_HV'].to_numpy()[w]):.2f} V"
                         f" | tail 900 W: Ipk {np.nanmedian(p['Ipk'].to_numpy()[tail]):.3f} vs {np.nanmedian(pe['Ipk'].to_numpy()[tail]):.3f} A, P_LV {np.nanmean(p['P_LV'].to_numpy()[tail]):.1f} vs {np.nanmean(pe['P_LV'].to_numpy()[tail]):.1f} W")
            rows.append(dict(V2=V2, hold=h, code=code, dD=dD, e_max=m["e_max"], e_max_ext=me["e_max"], e_rms=m["e_rms"], spike=m["spike"], spike_ext=me["spike"], Ipk_max=m["Ipk_max"], Ipk_max_ext=me["Ipk_max"], dI_med=float(np.nanmedian(dI)), dI_95=float(np.nanpercentile(dI, 95))))
            md_rows.append(f"| {V2:g} V | {h}주기 | {code} | {dD:.4f} | {m['e_max']:.1f} / {me['e_max']:.1f} | {m['spike']:.3f} / {me['spike']:.3f} | {m['Ipk_max']:.3f} / {me['Ipk_max']:.3f} | {np.nanmedian(dI):.3f} |")
    v = rule_v(ctl, s1)
    lines += ["", f"VERDICT: {v}"]
    print("\n".join(lines))
    if not write_out:
        return v, rows
    pd.DataFrame(rows).to_csv(os.path.join(OUT, "f1v3_check.csv"), index=False, float_format="%.6g")
    open(os.path.join(OUT, "f1v3_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    md = ["# 회귀를 Simulink 제어기 안에서 돌린 폐루프 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `f1_regloop.py compare`. 규칙은 실행 전 머리말에 등록했다. Simulink(데이터셋 시기 모델, 제어기 사본), 25 ns, 위상 래치, PI 이득 ×0.5, HV→LV 램프 100→900 W 40 ms, V1 380 V. 입력은 측정한 V_LV, V_HV의 한 주기 이동평균과 전력 지령이다. MCU 시험이 아니다(배정밀도, 양자화 없음, 추론 시간 없음).", "",
          f"**판정: {v}**", "", "| V2 | 추론 주기 | 판정 | 듀티 차 최대 | 전력 오차 최대 [W] 안/밖 | 피크 튐 [A] 안/밖 | 최대 Ipk [A] 안/밖 | 주기별 |ΔIpk| 중앙값 [A] |", "|---|---|---|---|---|---|---|---|"] + md_rows
    open(os.path.join(OUT, "f1v3_verdict.md"), "w", encoding="utf-8").write("\n".join(md) + "\n")
    print("wrote out/f1v3_check.{csv,log}, out/f1v3_verdict.md")
    return v, rows


def selftest():
    """the registered rule on made-up numbers and made-up runs"""
    import pandas as pd
    ok = True

    def chk(name, got, want):
        nonlocal ok
        ok &= got == want
        print(f"{'PASS' if got == want else 'FAIL'} {name}: {got}")
    chk("all small -> same", rule_s(0.003, 5.0, 0.05, 0.05), "같음")
    chk("duty apart -> differs", rule_s(0.02, 5.0, 0.05, 0.05), "다름")
    chk("tracking apart -> differs", rule_s(0.003, 16.0, 0.05, 0.05), "다름")
    chk("spike apart -> differs", rule_s(0.003, 5.0, -0.2, 0.05), "다름")
    chk("peak apart -> differs", rule_s(0.003, 5.0, 0.05, 0.2), "다름")
    chk("V same", rule_v({90.0: 0.0, 109.2: 0.0}, {90.0: "같음", 109.2: "같음"}), "제어기 안에서도 같음")
    chk("V one differs", rule_v({90.0: 0.0, 109.2: 0.0}, {90.0: "같음", 109.2: "다름"}), "다름")
    chk("V control off -> undecided", rule_v({90.0: 0.01, 109.2: 0.0}, {90.0: "같음", 109.2: "같음"}), "판정 불가")
    chk("V a run missing -> undecided", rule_v({90.0: 0.0, 109.2: 0.0}, {90.0: "같음"}), "판정 불가")
    t = r12.knots()[1:]
    P = r12.p_ref(t)

    def per(shift=0.0, d=0.45):
        return pd.DataFrame(dict(t_end=t, Ipk=1.0 + P / 1000.0 + shift, iL_mean=0.0, P_LV=-P, P_HV=-P - 3.0, V_LV=90.2, V_HV=379.9, D1=d, D2=d))
    ex = {V2: dict(periods=per(), error=np.nan) for V2 in V2S}
    runs = {f"v3_ctl_{V2:g}V": dict(periods=per(), error=np.nan) for V2 in V2S}
    runs.update({f"v3_MB_{V2:g}V_h{h}": dict(periods=per(0.01, 0.452), error=np.nan) for V2 in V2S for h in HOLDS})
    v, rows = compare(runs.get, ex.get, write_out=False)
    chk("made-up runs -> same", v, "제어기 안에서도 같음")
    chk("made-up runs: dD", round(rows[0]["dD"], 6), 0.002)
    runs["v3_MB_90V_h1"] = dict(periods=per(0.3), error=np.nan)
    chk("made-up: 0.3 A higher at hold 1 -> differs", compare(runs.get, ex.get, write_out=False)[0], "다름")
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
