"""
r8_verdict.py -- (2026-10-28) the R.8 verdicts, in Korean, from r5r6_chain's Simulink outputs -> out/r8_verdict.md.
The rules below were fixed on 10-28 at 12:30, before any of those outputs existed (MATLAB had been waiting for a
MathWorks sign-in since 10-27 19:11). test_r8_verdict.py checks every branch on made-up inputs. r8_finalize.py runs
this last; it reads only the analyses' CSVs and the raw stair_steady parts, so it can also run alone. A missing input
gives "아직 없음", never a verdict.

  Q1 (R5 b/c) staircase_compare.csv: in the cells where the dataset's own run (interp = 1) shows SPS - TPS >= 0.5 A at
     the same power, r = mean(true staircase, interp = 0) / mean(interp = 1).  r >= 0.75: the user is right (the
     reduction survives a true staircase) and the M-B conclusion has to change;  r <= 0.5: mostly the D0
     interpolation;  in between: partly.
  Q2 (R.8 d) BT_ labels: D - B_st = M-B's steady peak (R.4) minus the dataset's best TPS combination measured in steady
     state, same power.  max <= 0.05 A: the same;  <= 0.15 A: M-B gives up a little (dead time it was not trained on);
     larger: a real gap.
  Q3 (k ~ 1) K1_ labels: SPS - TPS in steady state at 100-300 W, V2 94.61 V.  max <= 0.05 A: no gain, M-B keeping SPS
     is right;  <= 0.15 A: a small dead-time gain M-B does not take;  larger: a real k ~ 1 gain (R.4's "0 at 95 V"
     changes).
  Q4 (R6/R7) dataset_fixed_compare.csv (true staircase, 11 x 11 grid, grid-best TPS): r = fixed / original dataset on
     the same grid, judged as Q1;  and fixed - M-B (R.4) at the same (V2, P), k != 1, R.4 oscillating point left out:
     mean <= 0.1 A: M-B matches the fixed dataset's best.

    python -I -X utf8 Simulink/Thesis/r8_verdict.py [--out DIR]   -> DIR/r8_verdict.md
"""
import argparse
import datetime
import glob
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "out")
from staircase_compare import crossing  # noqa: E402

DS, ST0, STD = "dataset(interp=1)", "staircase(interp=0)", "steady(const D0 30 ms)"
BIG = 0.5                     # Q1/Q4: a "large" dataset reduction [A]
USER, ARTEFACT = 0.75, 0.5    # Q1/Q4: share of it left with a true staircase
EQ, SMALL = 0.05, 0.15        # Q2/Q3 [A]
MB_GAP = 0.10                 # Q4 [A]
K1_P = (100.0, 150.0, 200.0, 300.0)
OSC = {(109.2, 600.0)}        # R.4 point whose SPS reference still oscillated
# Q2: Python plant's prediction (same plant, M-B minus B in steady state; RESULTS R.8 (d)), quoted beside Simulink
BT_PY = {"BT_109.2V_100W_0.247_0.200": 0.11, "BT_84.0V_300W_0.374_0.389": 0.02, "BT_84.0V_100W_0.216_0.232": -0.05}


def pick(df, **kv):
    m = np.ones(len(df), bool)
    for k, v in kv.items():
        m &= (df[k] == v).to_numpy() if isinstance(v, str) else np.isclose(df[k].to_numpy(float), v)
    return df[m]


def ratio_code(r):
    if not np.isfinite(r):
        return "undecidable"
    return "user" if r >= USER else "artefact" if r <= ARTEFACT else "partial"


RATIO_TXT = {"user": "**사용자 쪽이 맞다.** 진짜 계단에서도 데이터셋 저감의 {p:.0f} %가 남는다. M-B 결론(정상상태 저감 "
                     "0.2–0.5 A)을 고쳐야 한다",
             "artefact": "**데이터셋 저감의 대부분은 D0 보간 산물이다.** 진짜 계단에서는 {p:.0f} %만 남는다",
             "partial": "**일부만 보간 산물이다.** 진짜 계단에서 {p:.0f} %가 남는다(규칙의 중간 구간)",
             "undecidable": "판정할 칸이 없다"}
TAIL = {"user": "사용자 쪽이 맞다(규칙상 M-B 결론을 고쳐야 한다)", "artefact": "데이터셋 저감의 대부분은 D0 보간 산물이다",
        "partial": "일부만 보간 산물이다", "undecidable": "판정할 칸이 없다"}


def q1(out, md):
    md += ["## Q1. 데이터셋 실험에서 D0 보간만 끄면(진짜 1 ms 계단) 같은 전력의 큰 저감이 남는가? (R5 (b)(c))", ""]
    f = os.path.join(out, "staircase_compare.csv")
    c = pd.read_csv(f) if os.path.isfile(f) else None
    if c is None or ST0 not in set(c["source"]):
        md.append("아직 없음: `staircase_check_interp0.csv`(r5r6 1단계 또는 잡 stair_i0)가 없다.")
        return "missing", ""
    combos = {}
    fc = os.path.join(out, "staircase_check.csv")
    if os.path.isfile(fc):
        s = pd.read_csv(fc)
        combos = {t: (g["D1"].iloc[0], g["D2"].iloc[0]) for t, g in s.groupby("tag")}
    cc = c[c["part"] == "c"]
    piv = cc.pivot_table(index=["tag", "V2", "direction", "P"], columns="source", values="dI", aggfunc="first")
    big = piv[piv[DS] >= BIG].dropna(subset=[ST0])
    m1, m0 = big[DS].mean(), big[ST0].mean()
    r = m0 / m1 if len(big) else np.nan
    code = ratio_code(r)
    md += [f"규칙: 데이터셋 실행(보간 켬)에서 같은 전력 SPS − TPS가 {BIG} A 이상인 칸 {len(big)}개에서 "
           f"r = 평균(진짜 계단) / 평균(데이터셋). r ≥ {USER}이면 사용자 쪽, r ≤ {ARTEFACT}이면 보간 산물, 그 사이는 일부.",
           "", "| V2 | TPS (D1, D2) | 방향 | \\|P\\| | 데이터셋(보간) | 진짜 계단 | 정상상태(같은 D0 라벨 곡선) |",
           "|---|---|---|---|---|---|---|"]
    for (tag, V2, d, P), row in big.iterrows():
        dd = combos.get(tag)
        stv = row[STD] if STD in big.columns else np.nan
        md.append(f"| {V2:g} V | " + (f"({dd[0]:.3f}, {dd[1]:.3f})" if dd else tag)
                  + f" | {'HV→LV' if d == 'P<0' else 'LV→HV'} | {P:.0f} W | {row[DS]:+.2f} | {row[ST0]:+.2f} | "
                  + (f"{stv:+.2f}" if np.isfinite(stv) else "–") + " |")
    md += ["", f"단위 A, 칸 = Ipk(SPS) − Ipk(TPS). 평균: 데이터셋 {m1:+.2f} A → 진짜 계단 {m0:+.2f} A, r = {r:.2f}.",
           "", "**판정**: " + RATIO_TXT[code].format(p=100 * r) + "."]

    # SPS baseline: the true-staircase SPS rows against the steady SPS of R.4 (same power, HV->LV, k != 1)
    T = pd.read_csv(os.path.join(out, "ipk_decompose.csv"))
    T = T[(~T["k1"]) & T["clean"]]
    txt = []
    for src, lab in ((DS, "데이터셋 SPS 행(보간)"), (ST0, "진짜 계단 SPS 행")):
        d = []
        for r_ in T.itertuples():
            x = pick(cc, source=src, direction="P<0", V2=r_.V2_dataset, P=r_.P)["Ipk_SPS"]
            d.append(x.iloc[0] - r_.C_sl_SPS if len(x) else np.nan)
        d = np.asarray(d, float)
        d = d[np.isfinite(d)]
        if len(d):
            txt.append(f"{lab} − R.4 정상상태 SPS 평균 {d.mean():+.2f} A, 최대 |차| {np.abs(d).max():.2f} A({len(d)}점)")
    if txt:
        md += ["", "SPS 기준선(HV→LV, 같은 전력, k ≠ 1, 진동 점 제외): " + "; ".join(txt) + "."]
    b = c[c["part"] == "b"]
    if len(b):
        bl = []
        for src, lab in ((DS, "데이터셋(보간)"), (ST0, "진짜 계단")):
            x = b[b["source"] == src]["dIpk"].abs()
            if len(x):
                bl.append(f"{lab} 평균 {x.mean():.2f} A, 최대 {x.max():.2f} A")
        md += ["", "같은 D0 라벨에서 1 ms 창 값과 정상상태(상수 D0 30 ms)의 |ΔIpk|: " + "; ".join(bl) + "."]
    else:
        md += ["", "같은 라벨의 정상상태 행은 아직 없다(r5r6 2단계)."]
    sent = (f"Simulink에서 D0 보간만 끈 진짜 1 ms 계단으로 사용자 데이터셋 실험을 다시 돌리면, 데이터셋에서 {BIG} A 이상이던 "
            f"같은 전력 저감 {len(big)}칸의 평균이 {m1:+.2f} A에서 {m0:+.2f} A가 되어 {100 * r:.0f} %가 남는다. {TAIL[code]}."
            if np.isfinite(r) else "진짜 계단 결과는 있으나 비교할 큰 저감 칸이 없다.")
    return code, sent


def q2(out, md, st):
    md += ["", "## Q2. 데이터셋 최저 TPS 조합을 정상상태로 재도 M-B보다 낮은가? (R.8 (d), BT_ 23점)", ""]
    T = pd.read_csv(os.path.join(out, "ipk_decompose.csv"))
    pts = pd.read_csv(os.path.join(out, "stair_steady_points.csv"))
    bt = pts[pts["label"].astype(str).str.startswith("BT_")]
    rows = []
    for lab, g in bt.groupby("label", sort=False):
        P = float(lab.split("_")[2].rstrip("W"))
        V2 = float(g["V2"].iloc[0])
        r = pick(T, V2_dataset=round(V2, 2), P=P).iloc[0]
        Bst = np.nan
        if st is not None:
            s = st[(st["label"] == lab) & np.isfinite(st["Ipk_sim"])].sort_values("D0")
            if len(s) >= 2:
                Bst, _ = crossing(s["D0"].to_numpy(float), s["P_LV"].to_numpy(float), s["Ipk_sim"].to_numpy(float),
                                  P, +1)
        rows.append(dict(lab=lab, V2=V2, P=P, D1=g["D1"].iloc[0], D2=g["D2"].iloc[0], B=r.B_ds_bestTPS, Bst=Bst,
                         C=r.C_sl_SPS, D=r.D_sl_MB))
    R = pd.DataFrame(rows)
    if R.empty or not np.isfinite(R["Bst"]).any():
        md.append("아직 없음: BT_ 라벨의 Simulink 정상상태(r5r6 2단계)가 없다.")
        return "missing", ""
    R["gap"] = R["D"] - R["Bst"]
    md += [f"규칙: D − B_st(M-B 정상상태 − 데이터셋 최저 조합의 정상상태, 같은 전력)의 최대가 {EQ} A 이하면 같다, "
           f"{SMALL} A 이하면 작은 차(데드타임 미학습 몫), 그보다 크면 실재하는 차.", "",
           "| V2 | \\|P\\| | 조합 (D1, D2) | B 데이터셋 행 | B_st 정상상태 | D M-B | C SPS | D − B | D − B_st | "
           "Python 예측(같은 플랜트 차) |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in R.itertuples():
        md.append(f"| {r.V2:g} V | {r.P:.0f} W | ({r.D1:.3f}, {r.D2:.3f}) | {r.B:.3f} | {r.Bst:.3f} | {r.D:.3f} | "
                  f"{r.C:.3f} | {r.D - r.B:+.3f} | {r.gap:+.3f} | {BT_PY.get(r.lab, np.nan):+.2f} |")
    k = int(np.nanargmax(R["gap"].to_numpy()))
    g, where = R["gap"].iloc[k], f"{R['V2'].iloc[k]:g} V {R['P'].iloc[k]:.0f} W"
    code = "equal" if g <= EQ else "small" if g <= SMALL else "real"
    txt = {"equal": f"**같다.** M-B의 Simulink 정상상태 피크는 데이터셋 최저 TPS 조합의 정상상태와 {EQ} A 안에서 같다"
                    f"(최대 {g:+.3f} A, {where})",
           "small": f"**작은 차.** 데이터셋 최저 조합이 정상상태에서도 M-B보다 최대 {g:.2f} A 낮다({where}). 데드타임을 모르는 "
                    "이상 모델로 학습한 몫이다(Python 예측과 같은 방향)",
           "real": f"**실재하는 차.** 데이터셋 최저 조합이 정상상태에서도 M-B보다 {g:.2f} A 낮다({where}). "
                   "\"M-B ≈ 데이터셋 최저 TPS\"를 고쳐야 한다"}[code]
    md += ["", "단위 A, 같은 측정 전력(HV→LV). C − B_st가 그 조합의 정상상태 저감이고, C − D가 M-B의 저감이다.", "",
           "**판정**: " + txt + "."]
    tail = {"equal": "둘은 같다", "small": "M-B가 데드타임 미학습 몫만큼 덜 줄인다",
            "real": "실재하는 차다(규칙상 \"M-B ≈ 데이터셋 최저 TPS\"를 고쳐야 한다)"}[code]
    return code, f"데이터셋 최저 TPS 조합을 정상상태로 재면 M-B와의 차(D − B_st)는 최대 {g:+.2f} A({where})로, {tail}."


def q3(md, st):
    md += ["", "## Q3. k ≈ 1(94.61 V) 경부하에서 TPS가 정상상태로 SPS보다 낮은가? (K1_ 18점)", ""]
    k1 = st[st["label"].astype(str).str.startswith("K1_")] if st is not None else None
    if k1 is None or k1.empty:
        md.append("아직 없음: K1_ 라벨의 Simulink 정상상태(r5r6 2단계)가 없다.")
        return "missing", ""
    cur = {lab: g[np.isfinite(g["Ipk_sim"])].sort_values("D0") for lab, g in k1.groupby("label")}
    sps = [lab for lab in cur if "_SPS_" in lab]
    tps = [lab for lab in cur if "_SPS_" not in lab]

    def at(lab, P):
        g = cur[lab]
        if len(g) < 2:
            return np.nan
        return crossing(g["D0"].to_numpy(float), g["P_LV"].to_numpy(float), g["Ipk_sim"].to_numpy(float), P, +1)[0]

    md += [f"규칙: 같은 전력 SPS − TPS(정상상태, HV→LV)의 최대가 {EQ} A 이하면 이득 없음(M-B가 SPS를 유지하는 것이 맞다), "
           f"{SMALL} A 이하면 작은 데드타임 이득(M-B가 얻지 않는 몫), 그보다 크면 k ≈ 1 이득이 실재한다.", "",
           "| \\|P\\| | SPS Ipk | " + " | ".join(f"{lab.split('_')[1]} Ipk / 이득" for lab in tps) + " |",
           "|---|---|" + "---|" * len(tps)]
    best, where = -np.inf, ""
    for P in K1_P:
        Is = at(sps[0], P) if sps else np.nan
        cells = []
        for lab in tps:
            It = at(lab, P)
            gain = Is - It
            cells.append(f"{It:.3f} / {gain:+.3f}" if np.isfinite(gain) else "–")
            if np.isfinite(gain) and gain > best:
                best, where = gain, f"{lab.split('_')[1]} {P:.0f} W"
        md.append(f"| {P:.0f} W | " + (f"{Is:.3f}" if np.isfinite(Is) else "–") + " | " + " | ".join(cells) + " |")
    if not np.isfinite(best):
        md += ["", "**판정**: 판정 불가(목표 전력에 닿는 곡선이 없다)."]
        return "undecidable", ""
    code = "none" if best <= EQ else "small" if best <= SMALL else "real"
    txt = {"none": f"**이득 없음.** 최대 {best:+.3f} A. M-B가 k ≈ 1에서 SPS를 유지하는 것이 맞다(Python 예측 0.09 A 이하와 같은 결론)",
           "small": f"**작은 이득.** 최대 {best:.2f} A({where}). 데드타임이 만드는 몫이고 M-B는 이것을 얻지 않는다(한계로 기록, "
                    "Python 예측 0.02–0.09 A)",
           "real": f"**k ≈ 1 이득이 실재한다.** 최대 {best:.2f} A({where}). R.4의 \"95 V에서 0\"과 M-B가 SPS를 유지하는 설계를 "
                   "고쳐야 한다"}[code]
    settle = np.nanmax(np.abs(k1["Ipk_sim"] - k1["Ipk_at_20ms"])) if "Ipk_at_20ms" in k1 else np.nan
    md += ["", "**판정**: " + txt + ".",
           "", f"정착 점검: 30 ms와 20 ms 피크의 차가 최대 {settle:.3f} A다"
           + (" (0.02 A 초과: 느린 과도가 남았을 수 있다)." if settle > 0.02 else ".")
           + " 이 영역에는 정상상태가 둘 이상 공존한다(R.3, R.8 점검 (1)). 이 실행은 초기 상태 0에서 상수 D0로 출발해 닿는 상태 "
           "하나만 잰다."]
    tail = {"none": "M-B가 SPS를 유지하는 것이 맞다", "small": "작은 데드타임 이득이고 M-B는 이것을 얻지 않는다",
            "real": "k ≈ 1 이득이 실재한다(규칙상 R.4의 \"95 V에서 0\"을 고쳐야 한다)"}[code]
    return code, f"k ≈ 1(94.61 V) 경부하의 정상상태 TPS 이득은 최대 {best:+.2f} A({where})로, {tail}."


def q4(out, md):
    md += ["", "## Q4. 수정 데이터셋(R6, 진짜 계단)의 격자 최저 TPS를 원 데이터셋·M-B와 비교 (R7)", ""]
    f = os.path.join(out, "dataset_fixed_compare.csv")
    parts = sorted(glob.glob(os.path.join(out, "dataset_fixed_part_*.csv")))
    if not parts or not os.path.isfile(f):
        md.append("아직 없음: `dataset_fixed_part_*.csv`(r5r6 3단계)가 없다.")
        return ("missing", "missing"), ""
    n = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True).groupby(["V2", "D1", "D2"]).ngroups
    T = pd.read_csv(f)
    T = T[T["direction"] == "HV->LV"].assign(V2=lambda d: d["V2"].round(2))
    dI = T.pivot_table(index=["V2", "P"], columns="source", values="dI", aggfunc="first")
    sps = T.pivot_table(index=["V2", "P"], columns="source", values="Ipk_SPS", aggfunc="first")
    kne1 = [i for i in dI.index if i[0] in (84.0, 109.2)]
    big = dI.loc[kne1]
    big = big[big["orig_sub"] >= BIG].dropna(subset=["fixed"])
    m1, m0 = big["orig_sub"].mean(), big["fixed"].mean()
    r = m0 / m1 if len(big) else np.nan
    code = ratio_code(r)
    md += [f"수정 데이터셋: {n}/363 조합(부분 결과면 그만큼만). 같은 측정 전력, HV→LV, 단위 A, 칸 = Ipk(SPS) − Ipk(격자 최저 TPS).",
           "", f"규칙 1(Q1과 같다): 원 데이터셋(같은 격자)에서 {BIG} A 이상인 칸 {len(big)}개에서 r = 평균(수정) / 평균(원).",
           f"규칙 2: k ≠ 1에서 (수정 − M-B)의 평균이 {MB_GAP} A 이하면 M-B가 수정 데이터셋 최선과 같은 수준이다.", "",
           "| V2 | \\|P\\| | 원 데이터셋(같은 격자) | 수정 데이터셋 | M-B (R.4) | 수정 − M-B | SPS 수정 / R.4 |",
           "|---|---|---|---|---|---|---|"]
    gaps = []
    for (V2, P) in kne1:
        row = dI.loc[(V2, P)]
        mb = row.get("R4_MB_steady", np.nan)
        gap = row["fixed"] - mb if np.isfinite(mb) and (V2, P) not in OSC else np.nan
        if np.isfinite(gap):
            gaps.append((gap, f"{V2:g} V {P:.0f} W"))
        s = sps.loc[(V2, P)]
        md.append(f"| {V2:g} V | {P:.0f} W | {row['orig_sub']:+.2f} | {row['fixed']:+.2f} | "
                  + (f"{mb:+.2f}" + ("°" if (V2, P) in OSC else "") if np.isfinite(mb) else "–") + " | "
                  + (f"{gap:+.2f}" if np.isfinite(gap) else "–")
                  + f" | {s['fixed']:.2f} / " + (f"{s['R4_MB_steady']:.2f}" if np.isfinite(s.get('R4_MB_steady', np.nan))
                                                 else "–") + " |")
    g_mean = np.mean([x for x, _ in gaps]) if gaps else np.nan
    g_max = max(gaps) if gaps else (np.nan, "")
    mcode = "missing" if not gaps else "match" if g_mean <= MB_GAP else "gap"
    mtxt = {"missing": "M-B와 같은 점이 아직 없다",
            "match": f"**M-B는 수정 데이터셋의 격자 최선과 같은 수준이다**(수정 − M-B 평균 {g_mean:+.2f} A, 최대 {g_max[0]:+.2f} A "
                     f"@ {g_max[1]})",
            "gap": f"**수정 데이터셋 최선이 M-B보다 평균 {g_mean:.2f} A 더 줄인다**(최대 {g_max[0]:+.2f} A @ {g_max[1]}). 정상상태 "
                   "Simulink 자료로 E2E를 보정하면 얻을 여지다"}[mcode]
    k1 = [i for i in dI.index if abs(i[0] - 94.61) < 0.01 and i[1] <= 300]
    k1max = max((dI.loc[i]["fixed"] for i in k1 if np.isfinite(dI.loc[i]["fixed"])), default=np.nan)
    md += ["", "° R.4 기준 자체가 진동한 점(규칙 2에서 뺐다).",
           "", f"평균(규칙 1): 원 {m1:+.2f} A → 수정 {m0:+.2f} A, r = {r:.2f}.",
           "", "**판정 1**: " + RATIO_TXT[code].format(p=100 * r) + ".", "", "**판정 2**: " + mtxt + ".",
           "", f"k ≈ 1(94.61 V, 300 W 이하) 수정 데이터셋의 최대 저감은 {k1max:+.2f} A다. 계단 이력이 섞일 수 있으므로 Q3의 정상상태와 "
           "함께 읽는다."]
    tail = {"missing": "M-B와 비교할 점이 아직 없다", "match": f"M-B와 같은 수준이다(평균 차 {g_mean:+.2f} A)",
            "gap": f"M-B보다 평균 {g_mean:.2f} A(최대 {g_max[0]:.2f} A, {g_max[1]}) 더 줄인다"}[mcode]
    sent = (f"수정 데이터셋({n}/363 조합)에서 같은 방법으로 고른 격자 최저 TPS의 저감은 원 데이터셋 큰 저감의 {100 * r:.0f} %다"
            f"({TAIL[code].split('(')[0]}). " if np.isfinite(r) else
            f"수정 데이터셋({n}/363 조합)에는 원 데이터셋의 큰 저감 칸과 겹치는 점이 아직 없다. ") + f"이 격자 최저 TPS는 {tail}."
    return (code, mcode), sent


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    out = ap.parse_args(argv).out
    parts = [p for p in sorted(glob.glob(os.path.join(out, "stair_steady_part*.csv"))) if not p.endswith("_in.csv")]
    st = pd.concat([pd.read_csv(p) for p in parts], ignore_index=True) if parts else None
    npts = len(pd.read_csv(os.path.join(out, "stair_steady_points.csv")))
    nst = int(np.isfinite(st["Ipk_sim"]).sum()) if st is not None else 0
    md = ["# R.8 판정 (자동 생성)", "",
          f"생성 {datetime.datetime.now():%Y-%m-%d %H:%M}, `r8_verdict.py`. 판정 규칙은 2026-10-28 12:30에 정했다. 그때 Simulink "
          "결과는 하나도 없었다(스크립트 머리말). 수치는 모두 Simulink(데이터셋 시기 모델 35d5a78)이고, Python 값은 비교용 "
          "예측으로만 적었다.",
          f"입력: 진짜 계단 {'있음' if os.path.isfile(os.path.join(out, 'staircase_check_interp0.csv')) else '없음'}, "
          f"같은 라벨 정상상태 {nst}/{npts}점, 수정 데이터셋 part {len(glob.glob(os.path.join(out, 'dataset_fixed_part_*.csv')))}개.",
          ""]
    c1, s1 = q1(out, md)
    c2, s2 = q2(out, md, st)
    c3, s3 = q3(md, st)
    c4, s4 = q4(out, md)
    md += ["", "## RESULTS 0절에 넣을 문장(자동 조립, 다듬어 쓸 것)", ""]
    sents = [s for s in (s1, s2, s3, s4) if s]
    md.append(" ".join(sents) if sents else "Simulink 결과가 아직 없다. RESULTS의 현재 결론(R.8 (ii)·R7은 Python 예비값)이 "
                                            "그대로다.")
    if c1 == "user" or c4[0] == "user" or c2 == "real" or c3 == "real":
        md += ["", "**주의: 규칙상 RESULTS 0절의 결론을 고쳐야 하는 판정이 있다(위 판정 참고).**"]
    f = os.path.join(out, "r8_verdict.md")
    with open(f, "w", encoding="utf-8") as fh:
        fh.write("\n".join(md) + "\n")
    codes = dict(q1=c1, q2=c2, q3=c3, q4=c4[0], q4_mb=c4[1])
    print(f"wrote {f}: {codes}")
    return codes


if __name__ == "__main__":
    main()
