"""
flicker_check.py -- (10-30) how much does a steady-state Ipk depend on WHICH 1 ms window is read?

verify_steady_state.m records max|i_L| in 1 ms windows ending at 10, 15, 20, 25 ms and at the end (30 ms). Every verdict
uses the end window (Ipk_sim). A few clean runs (|mean i_L| <= 20 mA) do not hold one value: their peak jumps between
two levels from window to window (e.g. 109.2 V, D0 15.84, M-B: 2.13 / 2.07 / 1.66 / 2.13 A). If such a run brackets a
compared point, the comparison depends on the window. This script
  1. lists every clean steady run (R.4 sweep, R8 efficiency, R.8 same-label) whose peak over the 15 ms .. end windows
     spans more than FLICKER A, and
  2. recomputes R.8 Q2 (M-B vs the dataset's best TPS combination, same power, HV -> LV) with each window and with the
     maximum over the 15 ms .. end windows (the worst peak seen in steady state).
It does not change any registered verdict (r8_verdict.py, surface_opt_verdict.py); it reports how robust they are.

    python -I -X utf8 Simulink/Thesis/flicker_check.py   -> out/flicker_check.{log,csv}
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import simulink_sweep as sw  # noqa: E402
from staircase_compare import crossing  # noqa: E402

OUT = os.path.join(HERE, "out")
FLICKER = 0.05                                  # [A] span of the window peaks counted as flicker
OSC = 0.02                                      # [A] |mean i_L| above which a run oscillates (R.4 'clean')
WIN = ("15", "20", "25")                        # the end window is Ipk_sim / P_LV
Q2 = {(109.2, 100.0): (0.247368421, 0.2), (84.0, 300.0): (0.373684211, 0.389473684),
      (84.0, 100.0): (0.215789474, 0.231578947)}


def _files(pat):
    return [f for f in sorted(glob.glob(os.path.join(OUT, pat))) if not f.endswith("_in.csv")]


def load():
    # zvs_sweep_r* (10-30, zvs_check.py make) reruns R.4 / R8 levels with the every-ms windows: last, so it wins below
    fs = _files("simulink_sweep_r*.csv") + _files("eff_sweep_r*.csv") + _files("stair_steady_part*.csv") \
        + _files("opt_sweep_*_r*.csv") + _files("zvs_sweep_r*.csv")
    d = pd.concat([pd.read_csv(f).assign(file=os.path.basename(f)) for f in fs], ignore_index=True)
    if "error" in d:
        d = d[d["error"].isna() | (d["error"].astype(str).str.strip() == "")]
    from surface_opt_check import win_cols
    cols = win_cols(d.columns) + ["Ipk_sim"]                    # 15/20/25 ms before 10-30, every ms since
    d["Ipk_max"] = d[cols].max(axis=1)
    d["span"] = d["Ipk_max"] - d[cols].min(axis=1)
    return d.reset_index(drop=True)


def windows(g):
    """{window name: (P_LV, Ipk)} arrays of the runs g"""
    w = {f"{x} ms": (g[f"P_LV_at_{x}ms"].to_numpy(float), g[f"Ipk_at_{x}ms"].to_numpy(float)) for x in WIN}
    w["끝 (판정값)"] = (g["P_LV"].to_numpy(float), g["Ipk_sim"].to_numpy(float))
    w["최대 15 ms–끝"] = (g["P_LV"].to_numpy(float), g["Ipk_max"].to_numpy(float))
    return w


def mb_runs(d, V2, P):
    """M-B's R.4 runs at (V2, P): the configuration simulink_sweep used (its duties as written in the sweep files)"""
    r4 = d[d["file"].str.startswith("simulink_sweep_r") & (d["ctrl"] == "M-B") & np.isclose(d["V2"], V2)
           & np.isclose(d["P_target"], P)]
    if r4.empty:
        return r4
    c = dict(V2=V2, D1=float(r4["D1"].iloc[0]), D2=float(r4["D2"].iloc[0]))
    g = d[d["file"].str.startswith(("simulink_sweep_r", "eff_sweep_r", "zvs_sweep_r"))].copy()
    g["level"] = np.round((g["D0"] + 0.5 * sw.G) / sw.G).astype(int)
    g = sw.runs_of(g, c).drop_duplicates(subset=["D0"], keep="last")          # a zvs_sweep rerun if there is one
    g["Pd"] = -g["P_LV"]
    b = sw.bracket(g, P)[0]
    return pd.DataFrame(list(b)) if b is not None else g.iloc[:0]


def at_power(P_LV, I, P):
    """linear in delivered power between the two runs (HV -> LV, P_LV < 0)"""
    Pd = -P_LV
    t = (P - Pd[0]) / max(Pd[1] - Pd[0], 1e-9)
    return I[0] + t * (I[1] - I[0])


def main():
    d = load()
    lines = []

    def out(x=""):
        print(x, flush=True)
        lines.append(x)

    clean = d["iL_mean"].abs() <= OSC
    f = d[clean & (d["span"] > FLICKER)]
    out(f"1. 정상상태 실행 {len(d)}개 중 진동 없는(|mean i_L| <= {OSC * 1e3:.0f} mA) {int(clean.sum())}개에서, 15 ms·20 ms·25 ms·끝"
        f" 1 ms 창의 피크가 {FLICKER} A 넘게 달라지는 실행: {len(f)}개 ({len(f) / max(int(clean.sum()), 1) * 100:.1f} %)")
    out(f"{'file':<28} {'V2':>7} {'D0':>7} {'D1':>6} {'D2':>6} | {'15ms':>6} {'20ms':>6} {'25ms':>6} {'end':>6} | "
        f"{'span':>5} {'|P| end':>7}")
    for _, r in f.sort_values(["V2", "D1", "D0"]).iterrows():
        out(f"{r['file'][:28]:<28} {r['V2']:7.2f} {r['D0']:7.2f} {r['D1']:6.3f} {r['D2']:6.3f} | "
            f"{r['Ipk_at_15ms']:6.3f} {r['Ipk_at_20ms']:6.3f} {r['Ipk_at_25ms']:6.3f} {r['Ipk_sim']:6.3f} | "
            f"{r['span']:5.2f} {abs(r['P_LV']):7.1f}")

    st = d[d["file"].str.startswith("stair_steady_part")]
    out("\n2. R.8 Q2를 창마다 다시 계산: D − B_st = Ipk(M-B) − Ipk(데이터셋 최저 조합), 같은 전력, HV→LV, 단위 A")
    out("   (M-B는 R.4의 두 D0 계단, 조합은 같은 라벨 정상상태 곡선의 첫 교차; 끝 창이 r8_verdict.md의 값)")
    rows = []
    for (V2, P), (a, b) in Q2.items():
        g_mb = mb_runs(d, V2, P)
        g_bt = st[np.isclose(st["V2"], V2) & (np.abs(st["D1"] - a) < 1e-4) & (np.abs(st["D2"] - b) < 1e-4)
                  & st["label"].astype(str).str.startswith("BT_")].sort_values("D0")
        if len(g_mb) != 2 or len(g_bt) < 2:
            out(f"   {V2:g} V {P:.0f} W: 실행 부족 (M-B {len(g_mb)}, 조합 {len(g_bt)})")
            continue
        w_mb, w_bt = windows(g_mb), windows(g_bt)
        near = g_bt[(g_bt["P_LV"] < 0) & (-g_bt["P_LV"]).between(0.5 * P, 2.0 * P)]   # the runs around the crossing
        flick = bool((g_mb["span"] > FLICKER).any() or (near["span"] > FLICKER).any())
        out(f"   {V2:g} V {P:.0f} W, 조합 ({a:.3f}, {b:.3f}), M-B ({g_mb['D1'].iloc[0]:.3f}, {g_mb['D2'].iloc[0]:.3f})"
            + (" — 비교에 쓰인 실행 중 창마다 달라지는 것이 있다" if flick else ""))
        for k in w_mb:
            i_mb = at_power(*w_mb[k], P)
            i_bt = crossing(g_bt["D0"].to_numpy(float), w_bt[k][0], w_bt[k][1], P, +1)[0]
            rows.append(dict(V2=V2, P=P, window=k, Ipk_MB=i_mb, Ipk_BT=i_bt, gap=i_mb - i_bt))
            out(f"      {k:<12} M-B {i_mb:6.3f}  조합 {i_bt:6.3f}  D − B_st {i_mb - i_bt:+6.3f}")
    R = pd.DataFrame(rows)
    R.to_csv(os.path.join(OUT, "flicker_check.csv"), index=False, float_format="%.6g")
    if len(R):
        out("\n   창별 최대 D − B_st (세 점 중): " + ", ".join(f"{k} {g['gap'].max():+.3f}" for k, g in
                                                         R.groupby("window", sort=False)))

    z = d[d["file"].str.startswith("zvs_sweep_r")]
    if len(z):
        o = d[d["file"].str.startswith(("simulink_sweep_r", "eff_sweep_r"))]
        key = ["V2", "D0", "D1", "D2"]
        m = z.merge(o, on=key, suffixes=("", "_orig")).drop_duplicates(subset=key, keep="last")
        dI, dP = (m["Ipk_sim"] - m["Ipk_sim_orig"]).abs(), (m["P_LV"] - m["P_LV_orig"]).abs()
        out(f"\n3. 재실행 재현성: zvs_sweep(10-30)이 R.4/R8과 같은 (V2, D0, D1, D2)를 다시 돌린 {len(m)}회 — 끝 창 |ΔIpk| 최대 "
            f"{dI.max():.2e} A, |ΔP_LV| 최대 {dP.max():.2e} W" + (" (같다)" if dI.max() < 1e-6 else " — **다르다**"))
        zf = z[(z["iL_mean"].abs() <= OSC) & (z["span"] > FLICKER)]
        from surface_opt_check import win_cols
        cols = win_cols(z.columns) + ["Ipk_sim"]
        if len(zf):
            out("   재실행 중 매 1 ms 창(15 ms–끝)의 피크가 오가는 실행: " + "; ".join(
                f"{r['V2']:g} V D0 {r['D0']:.2f} ({r['D1']:.3f}, {r['D2']:.3f}) {r[cols].min():.3f}–{r[cols].max():.3f} A, "
                f"끝 {r['Ipk_sim']:.3f}" for _, r in zf.iterrows()))
    open(os.path.join(OUT, "flicker_check.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("wrote out/flicker_check.{log,csv}")


if __name__ == "__main__":
    main()
