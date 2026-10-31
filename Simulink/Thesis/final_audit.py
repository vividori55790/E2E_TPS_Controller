"""final_audit.py -- F5 (a): every number of FINAL_REPORT.md against the raw run files. Written in a later turn than the report, not trusting it.

Each row names a text of the report (it must be found there, once), the number tokens in it and the value recomputed here. A token matches if the recomputed value, rounded to the token's own decimals, equals it.
Coverage: every number token of the report has to be claimed by a row. Tokens nobody claims are listed in out/final_audit_uncovered.txt and counted: the audit is finished when mismatches and uncovered are both 0.

Kinds of row
  raw      recomputed from the runner's result files (out/*_sweep*.csv, out/iL_pp/, the closed-loop *_periods.csv) with audit_raw.py, which imports none of the study's comparison code
  derived  recomputed from a table a script wrote, where the raw step is not redone here (waveform analysis, the dataset decomposition); the note says which table
  log      the value exists only in a log or a job record: the audit checks the text is really there
  code     read from the model / the exported C / the rule's text
  given    a definition, condition or design value restated in the report, with where it comes from

    python -I -X utf8 Simulink/Thesis/final_audit.py            -> out/final_audit.csv, out/final_audit_uncovered.txt, out/final_audit.log
"""
import glob
import json
import os
import re
import sys
import zipfile

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import audit_raw as ar  # noqa: E402

OUT = ar.OUT
ROOT = os.path.dirname(os.path.dirname(HERE))
REPORT = open(os.path.join(HERE, "FINAL_REPORT.md"), encoding="utf-8").read()
TOKEN = re.compile(r"(?<![A-Za-z0-9_.,])[+−-]?\d[\d,]*(?:\.\d+)?(?![A-Za-z_])")
ROWS, COVER = [], []
SEC = ["?"]


def _num(tok):
    return float(tok.replace(",", "").replace("−", "-").replace("+", ""))


def _dec(tok):
    return len(tok.split(".")[1]) if "." in tok else 0


def _find(snippet, start=0):
    i = REPORT.find(snippet, start)
    if i < 0:
        return None
    return i


def A(snippet, pairs, src, kind="raw", note="", nth=0):
    """pairs: (token, value). value None = nothing to recompute (kind given / log). A str value is compared as text."""
    i = _find(snippet)
    for _ in range(nth):
        i = None if i is None else _find(snippet, i + 1)
    if i is None:
        ROWS.append(dict(sec=SEC[0], text=snippet[:90], token="", reported="", recomputed="", match="본문에 없음", kind=kind, src=src, note=note))
        return
    pos = 0
    for tok, val in pairs:
        j = snippet.find(tok, pos)
        if j < 0:
            ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed="", match="토큰 없음", kind=kind, src=src, note=note))
            continue
        COVER.append((i + j, i + j + len(tok)))
        pos = j + len(tok)
        if val is None:
            ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed="", match="대상 아님", kind=kind, src=src, note=note))
        elif isinstance(val, str):
            ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed=val, match="일치" if val == tok else "불일치", kind=kind, src=src, note=note))
        else:
            d = _dec(tok)
            rec = f"{val:.{d}f}"
            ok = abs(float(rec) - _num(tok)) < 0.5 * 10 ** (-d - 3)
            ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed=f"{val:.{d + 3}f}", match="일치" if ok else "불일치", kind=kind, src=src, note=note))


def G(snippet, toks, src, note=""):
    A(snippet, [(t, None) for t in toks], src, kind="given", note=note)


def L(snippet, toks, logfile, needle, note="", kind="log"):
    """the value is only in a log or record: the needle has to be in that file"""
    path = os.path.join(ROOT, logfile) if logfile.startswith(("automation", "Simulink")) else os.path.join(OUT, logfile)
    txt = open(path, encoding="utf-8", errors="replace").read() if os.path.exists(path) else ""
    hit = needle in txt
    i = _find(snippet)
    pos = 0
    for tok in toks:
        if i is None:
            ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed="", match="본문에 없음", kind=kind, src=logfile, note=needle))
            continue
        j = snippet.find(tok, pos)
        if j >= 0:
            COVER.append((i + j, i + j + len(tok)))
            pos = j + len(tok)
        ROWS.append(dict(sec=SEC[0], text=snippet[:90], token=tok, reported=tok, recomputed=needle[:80], match=("일치" if hit and j >= 0 else "불일치"), kind=kind, src=logfile, note=note))


# ------------------------------------------------------------------------------------------------- the run families
MAP = ar.load(["fine", "r13a", "r13b"])
MAP = MAP[np.abs(MAP["Ts"] / 25e-9 - 1) < 1e-6]
EDGE = ar.load("edge25")
MD = ar.load(["md", "r13a"])
MD = MD[np.abs(MD["Ts"] / 25e-9 - 1) < 1e-6]
ALL25 = ar.load(["fine", "r13a", "r13b", "md", "tgt25a", "tgt25b", "opt25"])
ALL25 = ALL25[np.abs(ALL25["Ts"] / 25e-9 - 1) < 1e-6]
V1X = ar.load(["v1x", "rob25"])
V1E = ar.load("v1e")
REV = ar.load("rev25")
REV6 = ar.load("rev6")
H12 = ar.load("h12")
G12 = ar.load("g12")
G10 = ar.load("g10")
DG10 = ar.load("dg10")
FE = ar.load("f1eff")
FI = ar.load("f1icros")


def pair(d, v, p, sign=1, Ts=None):
    return ar.value(d, v, p, "SPS", sign=sign, Ts=Ts), ar.value(d, v, p, "B", sign=sign, Ts=Ts)


def cell(text, s, b, kind="raw", src="out/fine,r13a,r13b_sweep*"):
    a = re.match(r"([\d.]+) → ([\d.]+) \(([+−]?[\d.]+) %\)", text)
    A(text, [(a.group(1), s), (a.group(2), b), (a.group(3), 100 * (s - b) / s)], src, kind=kind)


M = {}
for v in ar.V2N:
    for p in ar.PN:
        s, b = pair(MAP, v, p)
        M[(v, p)] = dict(V2=v, P=p, S=s["Iw"], B=b["Iw"], Sm=s["pk_med"], Bm=b["pk_med"], clean=s["clean"] and b["clean"], so=s["osc"], bo=b["osc"])
M = pd.DataFrame(M.values())
Mc = M[M["clean"]]
E = {}
for v in ar.V2N:
    for p in (25.0, 50.0, 75.0, 100.0, 1000.0, 1200.0):
        s, b = pair(EDGE, v, p)
        if s is not None and b is not None:
            E[(v, p)] = dict(V2=v, P=p, S=s["Iw"], B=b["Iw"], Sm=s["pk_med"], Bm=b["pk_med"], clean=s["clean"] and b["clean"])
Ed = pd.DataFrame(E.values())
mv = lambda v, p: M[(M.V2 == v) & (M.P == p)].iloc[0]  # noqa: E731
ev = lambda v, p: Ed[(Ed.V2 == v) & (Ed.P == p)].iloc[0]  # noqa: E731

# ------------------------------------------------------------------------------------------------- 1. 결론과 권고
SEC[0] = "1"
_ctl = open(os.path.join(HERE, "dataset_era", "DAB_Controller_System.m"), encoding="utf-8", errors="replace").read()
_mj = json.load(open(os.path.join(OUT, "model_smooth_reghdc_b3.json"), encoding="utf-8"))
with zipfile.ZipFile(os.path.join(HERE, "dataset_era", "Control_Logic_dataset_era.slx")) as _z:
    _x = _z.read("simulink/systems/system_root.xml").decode("utf-8", errors="replace")
SLX = {k: float(re.search(r'<P Name="%s">([^<]*)</P>' % k, _x).group(1)) for k in ("MaxSlewRate", "SoftStartTime")}
_fs = float(re.search(r"Fs = (\d+e\d+)", _ctl).group(1))
_lk = float(re.search(r"L_leakage = (\d+e-\d+)", _ctl).group(1))
_nt = float(re.search(r"n_transformer = (\d+)", _ctl).group(1))
A("1차 380 V, 2차 84–109.2 V, 변압비 4, 직렬 인덕터 90 µH, 80 kHz, 최대 1.2 kW", [("380", 380.0), ("84", _mj["V2_LO"]), ("109.2", _mj["V2_HI"]), ("4", _nt), ("90", _lk * 1e6), ("80", _fs / 1e3), ("1.2", _mj["P_MAX"] / 1e3)], "dataset_era/DAB_Controller_System.m, out/model_smooth_reghdc_b3.json", kind="code")
G("양방향 DAB 컨버터", [], "")
G("전압비 k = 4·V2/V1이 1에서", ["4", "1에서"], "정의")
G("k가 1 근처(V2 약 91–99 V)에 머무는 제품에는 100–1000 W에서", ["1", "91", "99", "100", "1000"], "|1 − k| ≤ 0.04는 V2 91.2–98.8 V")
x, y = mv(84.0, 100.0), mv(109.2, 100.0)
A("피크 전류 저감은 100 W에서 84 V 1.747 → 1.252 A(28 %), 109.2 V 2.269 → 1.410 A(38 %)", [("100", 100.0), ("84", 84.0), ("1.747", x.S), ("1.252", x.B), ("28", 100 * (x.S - x.B) / x.S), ("109.2", 109.2), ("2.269", y.S), ("1.410", y.B), ("38", 100 * (y.S - y.B) / y.S)], "map")
x, y = mv(84.0, 1000.0), mv(109.2, 1000.0)
A("1 kW에서 1.0 %, 3.3 %", [("1", 1.0), ("1.0", 100 * (x.S - x.B) / x.S), ("3.3", 100 * (y.S - y.B) / y.S)], "map")
e12 = Ed[Ed.P == 1200.0]
A("1.2 kW에서 일곱 V2 평균 0.9 %", [("1.2", 1.2), ("0.9", (100 * (e12.S - e12.B) / e12.S).mean())], "edge25, 0.72°")
A("k ≈ 1(92.4, 96.6 V)에서는 100–1000 W 여덟 점 평균이 −0.008, +0.002 A", [("1", 1.0), ("92.4", 92.4), ("96.6", 96.6), ("100", 100.0), ("1000", 1000.0), ("−0.008", (Mc[Mc.V2 == 92.4].S - Mc[Mc.V2 == 92.4].B).mean()), ("+0.002", (Mc[Mc.V2 == 96.6].S - Mc[Mc.V2 == 96.6].B).mean())], "map, 8점씩")
_all = pd.concat([M[M.clean].assign(src="map"), Ed[Ed.clean & Ed.P.isin([25.0, 50.0, 75.0, 1200.0])].assign(src="edge")])
_lim = {}
for v, g in _all.groupby("V2"):
    g = g.sort_values("P")
    p5 = None
    for r_ in g.itertuples():
        if 100 * (r_.S - r_.B) / r_.S > 5.0:
            p5 = r_.P
        else:
            break
    _lim[v] = p5
A("저감이 5 %를 넘는 범위는 84 V 300 W, 88.2 V 150 W, 92.4 V 50 W, 100.8 V 100 W, 105 V 200 W, 109.2 V 400 W 이하이고 96.6 V에는 없다(80주기 최댓값, 25 W부터)", [("5", 5.0), ("84", 84.0), ("300", _lim[84.0]), ("88.2", 88.2), ("150", _lim[88.2]), ("92.4", 92.4), ("50", _lim[92.4]), ("100.8", 100.8), ("100", _lim[100.8]), ("105", 105.0), ("200", _lim[105.0]), ("109.2", 109.2), ("400", _lim[109.2]), ("96.6", 96.6), ("80", 80.0), ("25", 25.0)], "map, edge25; 25 W부터 이어지는 구간", note=str(_lim))
ROWS.append(dict(sec=SEC[0], text="96.6 V에는 없다", token="-", reported="없음", recomputed=str(_lim[96.6]), match="일치" if _lim[96.6] is None else "불일치", kind="raw", src="edge25", note=""))
G("LV→HV는 100–1000 W, V1 변동은 일부 점만", ["100", "1000"], "실행 목록")
G("위상 분해능 0.72° 이하와 주기 동기 위상 갱신을 전제로", ["0.72"], "r11")

SEC[0] = "읽는 법"
G("(음수 = 제안 제어기가 높음)로 사실상 0이다", ["0이다"], "문장")
G('"a / b"로 적은 값은 a가 이 최댓값 기준, b가 80주기 피크의', ["80"], "정의")
G("원본 100 ns에서 25 ns로", ["100", "25"], "verify_steady_fine.m")
G("V1은 380 V다", ["380"], "공통 조건")
G("20 ms 실행 두 개", ["20"], "verify_steady_fine.m")
G("마지막 1 ms(80주기)", ["1", "80"], "Ipk_at_20ms, iL_pp 80열")
A("380 V × 25 ns / 90 µH = 0.106 A", [("380", 380.0), ("25", 25.0), ("90", 90.0), ("0.106", 380 * 25e-9 / 90e-6)], "계산", kind="code")
G("허용치 max(0.03 A, SPS 피크의 2 %)", ["0.03", "2"], "규칙(r13_map.py 등)")
G("평균 전류가 20 mA를 넘는", ["20"], "규칙")
A('"기본 56점"은 V2 7개(84–109.2 V, 4.2 V 간격) × |P| 8단(100–1000 W)', [("56", float(len(ar.V2N) * len(ar.PN))), ("7", float(len(ar.V2N))), ("84", ar.V2N[0]), ("109.2", ar.V2N[-1]), ("4.2", ar.V2N[1] - ar.V2N[0]), ("8", float(len(ar.PN))), ("100", ar.PN[0]), ("1000", ar.PN[-1])], "r13_map.py V2S, PS", kind="code")

# ------------------------------------------------------------------------------------------------- 2.
SEC[0] = "2"
G("ICROS 2026 논문", ["2026"], "Simulink/Docs/ICROS2026.docx")
G("1 ms 계단 Simulink 데이터셋", ["1"], "데이터셋 생성 실험")
_m = json.load(open(os.path.join(OUT, "model_smooth_reghdc_b3.json"), encoding="utf-8"))
A("입력 (|P|, V2)를 256차원", [("256", float(_m["D"]))], "out/model_smooth_reghdc_b3.json", kind="code")
G("D1, D2 = 0.5 − Δ(Δ ≥ 0)", ["0.5", "0"], "smooth_reghdc_infer.m")
G("SPS(D1 = D2 = 0.5)는 Δ가 0으로", ["0.5", "0"], "smooth_reghdc_infer.m")
G("V1이 380 V에서 벗어나면 측정한 V1로 입력을 (|P|·(380/V1)², V2·380/V1)", ["380", "380", "380"], "smooth_reghdc_infer.m s = 380 / V1")

# target A, ICROS, B0 and the small models: duties from tgt25_check.csv, values from the raw runs
TG = pd.read_csv(os.path.join(OUT, "tgt25_check.csv"))
TG = TG[TG["set"] == "M"]
ORDER = ["SPS", "MB", "MA", "IC", "B0", "L49", "D64", "L25", "L13"]
_ma = ar._model("model_smooth_reghdc_MA_neg.json")
_b0 = ar._model("model_smooth_reghdc_a.json")
_d64 = ar._model("model_smooth_reghdc_b7_D64.json")
res = []
dcheck = []
for _, q in TG.iterrows():
    v, p = q.V2, float(q.P)
    D = {k: (float(q[f"D1_{k}"]), float(q[f"D2_{k}"])) for k in ORDER}
    D["SPS"] = (ar.D_SAFE, ar.D_SAFE)
    D["MB"] = ar.mb(p, v)
    for nm, mdl in (("MA", _ma), ("B0", _b0), ("D64", _d64)):
        d1, d2 = ar.mb(p, v, model=mdl)
        dcheck.append(max(abs(d1 - D[nm][0]), abs(d2 - D[nm][1])))
    vals = {}
    own = {}
    for i, k in enumerate(ORDER):
        o = k
        for e in ORDER[:i]:
            if max(abs(D[k][0] - D[e][0]), abs(D[k][1] - D[e][1])) <= ar.SAME:
                o = own[e]
                break
        own[k] = o
        r = ar.pick(ALL25, v, D[o][0], D[o][1], tol=1.5e-5)
        vals[k] = ar.at(r, p) if len(r) else None
    rec = dict(V2=v, P=p)
    for k in ORDER:
        z = vals[k]
        rec[f"I_{k}"], rec[f"m_{k}"], rec[f"c_{k}"], rec[f"own_{k}"] = (z["Iw"], z["pk_med"], z["clean"], own[k]) if z else (np.nan, np.nan, False, own[k])
    res.append(rec)
T = pd.DataFrame(res)


def tol_kind(sps):
    return np.maximum(0.03, 0.02 * sps)


both = T[T.c_SPS & T.c_MA]
nA_r = int(((both.I_MA - both.I_SPS) > tol_kind(both.I_SPS)).sum())
nA_m = int(((both.m_MA - both.m_SPS) > tol_kind(both.m_SPS)).sum())
bAB = T[T.c_MB & T.c_MA]
q = T[(T.V2 == 96.6) & (T.P == 150.0)].iloc[0]
A("기본 56점 중 16 / 15점에서 SPS보다 높고(가장 큰 곳 96.6 V 150 W 0.665 → 1.507 A), 제안 제어기가 그 제어기보다 평균 0.134 / 0.139 A 낮다",
  [("56", float(len(T))), ("16", float(nA_r)), ("15", float(nA_m)), ("96.6", 96.6), ("150", 150.0), ("0.665", q.I_SPS), ("1.507", q.I_MA), ("0.134", (bAB.I_MA - bAB.I_MB).mean()), ("0.139", (bAB.m_MA - bAB.m_MB).mean())],
  "tgt25a, map; 듀티 tgt25_check.csv, M-A는 JSON과 대조", note="최대 위반 " + str(both.assign(x=both.I_MA - both.I_SPS).sort_values("x").iloc[-1][["V2", "P", "x"]].to_dict()))

# ------------------------------------------------------------------------------------------------- 3. table 1
SEC[0] = "3 표 1"
G("25, 1200 W는 0.72° 간격, 나머지는 2.88° 간격", ["25", "1200", "0.72", "2.88"], "edge25.py, r13_map.py")
for (v, cells) in ((84.0, ("1.559 → 0.638 (+59 %)", "1.747 → 1.252 (+28 %)", "2.260 → 2.138 (+5.4 %)", "4.345 → 4.300 (+1.0 %)", "5.061 → 5.032 (+0.6 %)")),
                   (92.4, ("0.412 → 0.326 (+21 %)", "0.601 → 0.596 (+0.8 %)", "1.114 → 1.162 (−4.4 %)", "3.165 → 3.166 (0.0 %)", "3.850 → 3.854 (−0.1 %)")),
                   (96.6, ("0.304 → 0.357 (−18 %)", "0.521 → 0.535 (−2.6 %)", "1.105 → 1.101 (+0.3 %)", "3.361 → 3.354 (+0.2 %)", "4.095 → 4.088 (+0.2 %)")),
                   (109.2, ("2.079 → 0.703 (+66 %)", "2.269 → 1.410 (+38 %)", "2.783 → 2.495 (+10 %)", "4.758 → 4.599 (+3.3 %)", "5.378 → 5.232 (+2.7 %)"))):
    for p, t in zip((25.0, 100.0, 300.0, 1000.0, 1200.0), cells):
        z = ev(v, p) if p in (25.0, 1200.0) else mv(v, p)
        cell(t, z.S, z.B, src="edge25 0.72°" if p in (25.0, 1200.0) else "map 2.88°")
for t, tk in (("| V2 (k) | 25 W | 100 W | 300 W | 1000 W | 1200 W |", ["25", "100", "300", "1000", "1200"]),):
    G(t, tk, "표 머리")
for v, k in ((84.0, "0.884"), (92.4, "0.973"), (96.6, "1.017"), (109.2, "1.149")):
    A(f"| {v:g} V ({k}) |", [(f"{v:g}", v), (k, 4 * v / 380.0)], "k = 4 V2 / V1", kind="code")
hv = {}
for v, p in ((98.7, 175.0), (98.7, 250.0), (98.7, 350.0), (92.4, 300.0), (105.0, 400.0)):
    s, b = pair(H12, v, p, Ts=6.25e-9)
    hv[(v, p)] = (s, b)
s, b = hv[(92.4, 300.0)]
ROWS.append(dict(sec=SEC[0], text="92.4 V 300 W는 6.25 ns에서 차 0.000 A (근거표)", token="0.000", reported="0.000", recomputed=f"{s['Iw'] - b['Iw']:.5f}", match="일치" if abs(round(s["Iw"] - b["Iw"], 3)) == 0 else "불일치", kind="raw", src="h12", note="본문에서 뺀 값, 근거표"))

SEC[0] = "그림 1"
G("음수 칸(4장): 92.4 V 300 W, 105 V 400 W는 6.25 ns에서 사라졌고, 96.6 V 25–75 W는 더 고운 격자로", ["4", "92.4", "300", "105", "400", "6.25", "96.6", "25", "75"], "h12, edge25")
_a, _b = mv(96.6, 100.0), mv(92.4, 400.0)
A("96.6 V 100 W, 92.4 V 400 W는 이 계단에서 허용치 안이다", [("96.6", 96.6), ("100", 100.0), ("92.4", 92.4), ("400", 400.0)], "map", note=f"{_a.B - _a.S:+.3f}, {_b.B - _b.S:+.3f} A")
ROWS.append(dict(sec=SEC[0], text="96.6 V 100 W, 92.4 V 400 W 허용치 안", token="-", reported="허용치 안", recomputed=f"{_a.B - _a.S:+.3f}, {_b.B - _b.S:+.3f}", match="일치" if (_a.B - _a.S) <= ar.tol(_a.S) and (_b.B - _b.S) <= ar.tol(_b.S) else "불일치", kind="raw", src="map", note=""))
G("그림 1. 피크 저감률 [%], 80주기 최댓값, 25–75, 1200 W는 0.72° 계단, 나머지는 2.88° 계단", ["80", "25", "75", "1200", "0.72", "2.88"], "final_fig.py")
G("음수 칸은 그림 1과 4장을", ["4"], "장 번호")
G("경부하와 k ≠ 1에 몰려", ["1"], "문장")

SEC[0] = "3 본문"
dI, dIm = Mc.S - Mc.B, Mc.Sm - Mc.Bm
A("판정 가능한 기본 52점의 평균은 +0.115 / +0.127 A(+5.4 / +5.9 %)", [("52", float(len(Mc))), ("+0.115", dI.mean()), ("+0.127", dIm.mean()), ("+5.4", 100 * (dI / Mc.S).mean()), ("+5.9", 100 * (dIm / Mc.Sm).mean())], "map")
wr = []
for v, g in Mc.groupby("V2"):
    g = g.sort_values("P")
    sp = g.P.iloc[-1] - g.P.iloc[0]
    wr.append(dict(a=np.trapezoid(g.S - g.B, g.P) / sp, r=np.trapezoid((g.S - g.B) / g.S, g.P) / sp, m=np.trapezoid(g.Sm - g.Bm, g.P) / sp, q=np.trapezoid((g.Sm - g.Bm) / g.Sm, g.P) / sp))
wr = pd.DataFrame(wr)
A("전력에 고르게 가중하면 +0.063 / +0.076 A(+2.5 / +3.0 %)", [("+0.063", wr.a.mean()), ("+0.076", wr.m.mean()), ("+2.5", 100 * wr.r.mean()), ("+3.0", 100 * wr.q.mean())], "map, V2마다 사다리꼴")

MC = pd.read_csv(os.path.join(OUT, "md_check.csv"))
H = MC[MC["set"] == "H"]
_ms = ar._model("model_smooth_reghdc_MS.json")
_md = ar._model("model_smooth_reghdc_MD.json")
rows = []
for _, q in H.iterrows():
    v, p = float(q.V2), float(q.P)
    D = dict(S=(ar.D_SAFE, ar.D_SAFE), B=ar.mb(p, v), MS=ar.mb(p, v, model=_ms), MD=ar.mb(p, v, model=_md))
    own, vals = {}, {}
    names = ["S", "B", "MS", "MD"]
    for i, k in enumerate(names):
        o = k
        for e in names[:i]:
            if max(abs(D[k][0] - D[e][0]), abs(D[k][1] - D[e][1])) <= ar.SAME:
                o = own[e]
                break
        own[k] = o
        r = ar.pick(MD, v, D[o][0], D[o][1], tol=1.5e-5)
        vals[k] = ar.at(r, p) if len(r) else None
    z = dict(V2=v, P=p)
    for k in names:
        w = vals[k]
        z.update({f"I{k}": w["Iw"] if w else np.nan, f"m{k}": w["pk_med"] if w else np.nan, f"c{k}": bool(w and w["clean"])})
    rows.append(z)
HH = pd.DataFrame(rows)
Hc = HH[HH.cS & HH.cB]
A("기본 56점에 없는 V2·전력 48점 중 판정 가능한 44점(84, 109.2 V 없음)은 +0.069 / +0.072 A(+3.1 / +3.2 %)",
  [("56", 56.0), ("48", float(len(HH))), ("44", float(len(Hc))), ("84", 84.0), ("109.2", 109.2), ("+0.069", (Hc.IS - Hc.IB).mean()), ("+0.072", (Hc.mS - Hc.mB).mean()), ("+3.1", 100 * ((Hc.IS - Hc.IB) / Hc.IS).mean()), ("+3.2", 100 * ((Hc.mS - Hc.mB) / Hc.mS).mean())], "md, 48점 중 깨끗한 점", note=f"V2 {Hc.V2.min():g}–{Hc.V2.max():g} V")
PI = pd.read_csv(os.path.join(OUT, "simulink_pi_compare.csv"))
pc = lambda v: float(PI[(PI.V2 == v) & (PI.P == 100) & (PI.ctrl.isin(["M-B", "B", "MB", "Smooth RegHDC (b3)"]))].dI_pct_simulink.iloc[0])  # noqa: E731
A("100 W 저감은 원본 100 ns에서 84 V 23.9 %, 109.2 V 23.2 %", [("100", 100.0), ("100", 100.0), ("84", 84.0), ("23.9", pc(84.0)), ("109.2", 109.2), ("23.2", pc(109.2))], "out/simulink_pi_compare.csv", kind="derived")
z = ev(109.2, 100.0)
A("109.2 V 100 W를 0.72° 계단으로 다시 재면 제안 제어기 1.476 / 1.371 A(저감 35 / 40 %)", [("109.2", 109.2), ("100", 100.0), ("0.72", 0.72), ("1.476", z.B), ("1.371", z.Bm), ("35", 100 * (z.S - z.B) / z.S), ("40", 100 * (z.Sm - z.Bm) / z.Sm)], "edge25 set C")
six = [(v, p) for v in (84.0, 109.2) for p in (100.0, 300.0, 1000.0)]
d25 = [mv(v, p).S - mv(v, p).B for v, p in six]
d12 = []
d12m = []
for v, p in six:
    s, b = pair(G12, v, p, Ts=12.5e-9)
    d12.append(s["Iw"] - b["Iw"])
    d12m.append(s["pk_med"] - b["pk_med"])
A("대표 6점(84, 109.2 V × 100, 300, 1000 W)의 평균 저감은 25 ns +0.328 A, 12.5 ns +0.334 A", [("6", 6.0), ("84", 84.0), ("109.2", 109.2), ("100", 100.0), ("300", 300.0), ("1000", 1000.0), ("25", 25.0), ("+0.328", np.mean(d25)), ("12.5", 12.5), ("+0.334", np.mean(d12))], "map, g12")

# optimality: everything measured at a map point
CANDS = pd.read_csv(os.path.join(OUT, "opt25_cands.csv"))
POOL = ar.load(["fine", "r13a", "r13b", "opt25"])
POOL = POOL[np.abs(POOL["Ts"] / 25e-9 - 1) < 1e-6]
opt = []
nfar = 0
for v in ar.V2N:
    for p in ar.PN:
        ms = ar.mb(p, v, model=_ms)
        b = ar.mb(p, v)
        duties = [(ar.D_SAFE, ar.D_SAFE), b, ms]
        for dd1, dd2 in ((-0.03, 0.0), (0.03, 0.0), (0.0, -0.03), (0.0, 0.03)):
            duties.append((round(min(max(ms[0] + dd1, 0.02), ar.D_SAFE), 9), round(min(max(ms[1] + dd2, 0.02), ar.D_SAFE), 9)))
        far = CANDS[(np.abs(CANDS.V2 - v) < 1e-9) & (np.abs(CANDS.P - p) < 1e-9)]
        nfar += len(far)
        duties += [(float(r.D1), float(r.D2)) for r in far.itertuples()]
        vals = []
        for k, dd in enumerate(duties):
            own = dd
            for e in duties[:k]:
                if k < 7 and max(abs(dd[0] - e[0]), abs(dd[1] - e[1])) <= ar.SAME:
                    own = e
                    break
            if max(abs(own[0] - 0.5), abs(own[1] - 0.5)) <= ar.SAME:
                own = (ar.D_SAFE, ar.D_SAFE)
            r = ar.pick(POOL, v, own[0], own[1], tol=1.5e-5)
            vals.append(ar.at(r, p) if len(r) else None)
        s, bb = vals[0], vals[1]
        every_r = [w["Iw"] for w in vals if w and w["clean"]]
        every_m = [w["pk_med"] for w in vals if w and w["clean"]]
        opt.append(dict(V2=v, P=p, clean=bool(s and bb and s["clean"] and bb["clean"]), S=s["Iw"], B=bb["Iw"], Sm=s["pk_med"], Bm=bb["pk_med"], best=min(every_r), bestm=min(every_m)))
O = pd.DataFrame(opt)
Oc = O[O.clean]
A("곡면에서 먼 듀티 155쌍까지 포함해 잰 후보 중 가장 낮은 값과 비교하면 저감의 83 / 91 %를 얻는다", [("155", float(nfar)), ("83", 100 * (Oc.S - Oc.B).sum() / (Oc.S - Oc.best).sum()), ("91", 100 * (Oc.Sm - Oc.Bm).sum() / (Oc.Sm - Oc.bestm).sum())], "map, opt25; 후보 opt25_cands.csv + M-S ±0.03")
# far pairs against the proposed controller, each kind (opt25.py O1: gap > tol)
_far = []
for v in ar.V2N:
    for p in ar.PN:
        b = ar.mb(p, v)
        own = b if max(abs(b[0] - 0.5), abs(b[1] - 0.5)) > ar.SAME else (ar.D_SAFE, ar.D_SAFE)
        vb = ar.at(ar.pick(POOL, v, own[0], own[1], tol=1.5e-5), p)
        vs = ar.at(ar.pick(POOL, v, ar.D_SAFE, ar.D_SAFE), p)
        lo_r = lo_m = None
        for r_ in CANDS[(np.abs(CANDS.V2 - v) < 1e-9) & (np.abs(CANDS.P - p) < 1e-9)].itertuples():
            w = ar.at(ar.pick(POOL, v, float(r_.D1), float(r_.D2), tol=1.5e-5), p)
            if w and w["clean"]:
                dist = max(abs(r_.D1 - b[0]), abs(r_.D2 - b[1]))
                lo_r = (w["Iw"], dist) if lo_r is None or w["Iw"] < lo_r[0] else lo_r
                lo_m = (w["pk_med"], dist) if lo_m is None or w["pk_med"] < lo_m[0] else lo_m
        tr = max(0.03, 0.02 * vs["Iw"]) if vs["clean"] else 0.02 * vb["Iw"]
        tm = max(0.03, 0.02 * vs["pk_med"]) if vs["clean"] else 0.02 * vb["pk_med"]
        _far.append(dict(V2=v, P=p, ok=bool(vb["clean"]) and lo_r is not None, gr=vb["Iw"] - lo_r[0] if lo_r else np.nan, tr=tr, dr=lo_r[1] if lo_r else np.nan, gm=vb["pk_med"] - lo_m[0] if lo_m else np.nan, tm=tm))
FR = pd.DataFrame(_far)
FRd = FR[FR.ok]
fr_r, fr_m = FRd[FRd.gr > FRd.tr], FRd[FRd.gm > FRd.tm]
A("먼 듀티가 허용치 넘게 낮은 점은 제안 제어기가 판정 가능한 54점 중 9 / 3점이고(차가 가장 큰 곳 0.211 / 0.131 A, 중앙값의 3점은 모두 105 V)", [("54", float(len(FRd))), ("9", float(len(fr_r))), ("3", float(len(fr_m))), ("0.211", fr_r.gr.max()), ("0.131", fr_m.gm.max()), ("3", float(len(fr_m))), ("105", float(fr_m.V2.min()))], "map, opt25", note=f"중앙값 3점의 V2 범위 {fr_m.V2.min():g}–{fr_m.V2.max():g} V")
A("9점의 쌍은 곡면에서 듀티로 0.025–0.053 떨어진", [("9", float(len(fr_r))), ("0.025", fr_r.dr.min()), ("0.053", fr_r.dr.max())], "map, opt25")
L('"다른 가지(곡면과 다른 모양의 최적) 있음" / "몇 점에서만"이다', [], "opt25_verdict.md", "다른 가지 있음")
gc = HH[HH.cB & HH.cMD]
A("25 ns 측정값으로 다시 학습한 곡면도 학습에 쓰지 않은 48점(47점 판정 가능)에서 이득이 없었다(제안 − 재학습 −0.004 / +0.004 A, 문턱 +0.01 A)", [("25", 25.0), ("48", float(len(HH))), ("47", float((HH.cB | HH.cMD).sum())), ("−0.004", (gc.IB - gc.IMD).mean()), ("+0.004", (gc.mB - gc.mMD).mean()), ("+0.01", 0.01)], "md; 값이 있는 점 평균; 문턱 md_check.py GAIN_MIN", note=f"{len(gc)}점")


# closed loop ------------------------------------------------------------------------------------------
def periods(sub, name):
    return pd.read_csv(os.path.join(OUT, sub, f"{name}_periods.csv"))


def spike(per, t0, t1):
    """max of Ipk - running median over 41 periods, window (t0, t1]"""
    I = per["Ipk"]
    env = I.rolling(41, center=True, min_periods=1).median()
    w = (per["t_end"] > t0 + 1e-9) & (per["t_end"] <= t1 + 1e-9)
    return float((I - env)[w].max()), float(I[w].max())


R12 = {(c, v): spike(periods("r12", f"r2_{c}_{v:g}V_ramp"), 6e-3, 46e-3) for c in ("ICROS", "MB", "SPS") for v in (90.0, 109.2)}
_r2 = pd.read_csv(os.path.join(OUT, "r12", "r2_MB_109.2V_ramp_summary.csv")).iloc[0]
_r1 = pd.read_csv(os.path.join(OUT, "r12", "r12_MB_109.2V_ramp_summary.csv")).iloc[0]
A("100 → 900 W 전력 램프(40 ms, 폐루프, 주기 경계 위상 갱신을 넣은 제어기 사본, 25 ns, PI 이득은 원본의 ½인 Kp 1, Ki 24500)", [("100", 100.0), ("900", 900.0), ("40", 40.0), ("25", _r2.Ts * 1e9), ("1", float(_r2.Kp)), ("24500", float(_r2.Ki))], "out/r12/r2_*_summary.csv", kind="code")
A("처음 쓴 PI 이득(Kp 2, Ki 49000)", [("2", float(_r1.Kp)), ("49000", float(_r1.Ki))], "out/r12/r12_*_summary.csv", kind="code")
A("피크가 3.18 A(109.2 V), 1.03 A(90 V) 튀었고, 제안 제어기는 0.23, 0.18 A로 SPS(0.23, 0.18 A)와 같았다",
  [("3.18", R12[("ICROS", 109.2)][0]), ("109.2", 109.2), ("1.03", R12[("ICROS", 90.0)][0]), ("90", 90.0), ("0.23", R12[("MB", 109.2)][0]), ("0.18", R12[("MB", 90.0)][0]), ("0.23", R12[("SPS", 109.2)][0]), ("0.18", R12[("SPS", 90.0)][0])], "out/r12/r2_*_periods.csv")
sp1 = spike(periods("r12", "r12_MB_109.2V_ramp"), 6e-3, 46e-3)
A("한계 주기로 1.39 A(109.2 V) 튀었다", [("1.39", sp1[0]), ("109.2", 109.2)], "out/r12/r12_MB_109.2V_ramp_periods.csv")


# R11, gain 1/2: every step of the two profiles
def r11_step(ctrl, v, prof, seg, g="r2_g0.5"):
    per = periods("r11", f"{g}_{ctrl}_{v:g}V_{prof}")
    a, b = {0: (0.0, 0.012), 1: (0.012, 0.024), 2: (0.024, 0.036)}[seg]
    w = per[(per.t_end > a + 1e-9) & (per.t_end <= a + 2e-3 + 1e-9)]
    st = per[(per.t_end > b - 2e-3 + 1e-9) & (per.t_end <= b + 1e-9)]
    return float(w.Ipk.max()), float(st.Ipk.median())


C11 = pd.read_csv(os.path.join(OUT, "r11_check.csv"))
C11 = C11[C11.label.str.startswith("r2_g0.5")]
tr = {}
for v in (84.0, 109.2):
    for prof in ("light", "heavy"):
        for seg in (1, 2):
            tr[(v, prof, seg)] = {c: r11_step(c, v, prof, seg)[0] for c in ("SPS", "MB")}
x, y = tr[(84.0, "light", 1)], tr[(109.2, "light", 1)]
A("300 → 100 W 계단에서 SPS, 제안 순으로 84 V 4.89, 2.83 A, 109.2 V 3.58, 2.42 A", [("300", 300.0), ("100", 100.0), ("84", 84.0), ("4.89", x["SPS"]), ("2.83", x["MB"]), ("109.2", 109.2), ("3.58", y["SPS"]), ("2.42", y["MB"])], "out/r11/r2_g0.5_*_periods.csv, 계단 직후 2 ms 최대")
above = [(k, t["MB"] - t["SPS"], t) for k, t in tr.items() if t["MB"] > t["SPS"]]
w = tr[(109.2, "heavy", 1)]
A("같은 폐루프의 전력 계단(84, 109.2 V × 100↔300 W, 500↔1000 W, 명령 변화율 10 kW/ms, 계단 직후 2 ms 최대)에서도", [("84", 84.0), ("109.2", 109.2), ("100", 100.0), ("300", 300.0), ("500", 500.0), ("1000", 1000.0), ("10", SLX["MaxSlewRate"] / 1e6), ("2", 2.0)], "r11_step.py PROFILES, 모델의 MaxSlewRate 1e+7 W/s", kind="code")
A("여덟 계단 중 일곱에서 SPS 이하였고 109.2 V 500 → 1000 W는 0.18 A(1.3 %) 컸다",
  [("109.2", 109.2), ("500", 500.0), ("1000", 1000.0), ("0.18", tr[(109.2, "heavy", 2)]["MB"] - tr[(109.2, "heavy", 2)]["SPS"]), ("1.3", 100 * (tr[(109.2, "heavy", 2)]["MB"] / tr[(109.2, "heavy", 2)]["SPS"] - 1))], "out/r11/", note=f"SPS보다 높은 계단 수 {len(above)} / {len(tr)}: {[k for k, _, _ in above]}")
ROWS.append(dict(sec=SEC[0], text="여덟 계단 중 일곱", token="일곱", reported="7/8", recomputed=f"{len(tr) - len(above)}/{len(tr)}", match="일치" if (len(tr), len(above)) == (8, 1) else "불일치", kind="raw", src="out/r11/", note=""))

# ZVS
Z = pd.read_csv(os.path.join(OUT, "zvs_map25.csv"))
zb = Z[Z.osc <= ar.OSC].pivot_table(index=["V2", "P"], columns="ctrl", values="hard_VI")[["SPS", "M-B"]].dropna()
A("기본 52점 합으로 22,619 → 4,599 VA(저감 80 %", [("52", float(len(zb))), ("22,619", zb.SPS.sum()), ("4,599", zb["M-B"].sum()), ("80", 100 * (1 - zb["M-B"].sum() / zb.SPS.sum()))], "out/zvs_map25.csv", kind="derived", note="파형 분석은 다시 하지 않고 표를 다시 합산; 오프셋 점검이 적합값에서 같은 합을 낸다")
_off = re.findall(r"([+-][\d.]+) %$", open(os.path.join(OUT, "zvs_offset.log"), encoding="utf-8").read(), flags=re.M)
_off = [float(x) for x in _off]
A("시각 오프셋을 옮기는 점검에서 76–83 %", [("76", -max(_off)), ("83", -min(_off))], "out/zvs_offset.log (zvs_offset_check.py, 52점)", kind="derived")
_zv = Z[Z.osc <= ar.OSC].pivot_table(index=["V2", "P"], columns="ctrl", values="hard_VI")[["SPS", "M-B"]].dropna()
_byv = _zv.groupby(level="V2").sum()
A("점별로는 28점에서 낮고 9점에서 높으며, V2별 합은 92.4 V에서만 높다(415 → 617 VA)", [("28", float((_zv["M-B"] < _zv.SPS - 1).sum())), ("9", float((_zv["M-B"] > _zv.SPS + 1).sum())), ("92.4", float(_byv[_byv["M-B"] > _byv.SPS].index[0])), ("415", _byv.loc[92.4, "SPS"]), ("617", _byv.loc[92.4, "M-B"])], "out/zvs_map25.csv, 1 VA 넘는 차", kind="derived", note=f"합이 높은 V2 {list(_byv[_byv['M-B'] > _byv.SPS].index)}")

# efficiency -------------------------------------------------------------------------------------------
Cl, Ch = 1000e-6, 47e-6
FE = FE.assign(dE=Cl * FE.V_LV * FE.dV_LV / 0.5e-3 + Ch * FE.V_HV * FE.dV_HV / 0.5e-3)
FE = FE.assign(Lf=FE.P_LV_d - FE.P_HV_d - FE.dE, La=FE.P_LV - FE.P_HV - FE.dE, Pdir=-FE.dirn * FE.P_LV_d)
Sx = FE[np.abs(FE.Ts / FE.Ts_solver - 1) < 1e-9]
Xx = FE[np.abs(FE.Ts / FE.Ts_solver - 1) >= 1e-9]
key = ["ctrl", "V2", "P_target", "dirn", "level"]
TSG = {100: 1e-7, 25: 25e-9, 12.5: 12.5e-9, 6.25: 6.25e-9}


def at_grid(d, ns):
    return d[np.abs(d.Ts / TSG[ns] - 1) < 1e-6]


def eff_point(v, p, dirn, ns):
    out = {}
    for c in ("SPS", "M-B*"):
        g = at_grid(Sx, ns)
        g = g[(g.ctrl == c) & (np.abs(g.V2 - v) < 1e-9) & (g.P_target == p) & (g.dirn == dirn)]
        g = g.sort_values("level", ascending=dirn > 0, key=lambda s: s)
        Pd, lv, Lv = g.Pdir.to_numpy(), g.level.to_numpy(), g.Lf.to_numpy()
        val = None
        for i in range(len(g) - 1):
            if abs(lv[i + 1] - lv[i]) == 1 and min(Pd[i], Pd[i + 1]) <= p <= max(Pd[i], Pd[i + 1]) and Pd[i] != Pd[i + 1]:
                t = (p - Pd[i]) / (Pd[i + 1] - Pd[i])
                val = Lv[i] + t * (Lv[i + 1] - Lv[i])
                break
        out[c] = val
    eta = (lambda L0: 100 * p / (p + L0)) if dirn > 0 else (lambda L0: 100 * (p - L0) / p)
    return out["SPS"], out["M-B*"], eta(out["M-B*"]) - eta(out["SPS"]) if out["SPS"] is not None and out["M-B*"] is not None else None


SEC[0] = "3 표 2"
TAB2 = (("HV→LV 84 V 100 W", 84.0, 100.0, 1, ["3.94 → 5.34 (−1.28)", "2.31 → 1.72 (+0.57)", "1.85 → 1.07 (+0.76)", "1.59 → 0.74 (+0.83)", "1.33 → 0.42 (+0.89)"]),
        ("LV→HV 84 V 100 W", 84.0, 100.0, -1, ["6.19 → 4.70 (+1.49)", "2.53 → 1.47 (+1.06)", "1.93 → 0.95 (+0.98)", "1.65 → 0.67 (+0.98)", "1.37 → 0.39 (+0.97)"]),
        ("HV→LV 84 V 1 kW", 84.0, 1000.0, 1, ["51.24 → 44.65 (+0.60)", "23.25 → 21.59 (+0.16)", "18.44 → 17.62 (+0.08)", "16.05 → 15.65 (+0.04)", "13.65 → 13.67 (0.00)"]),
        ("LV→HV 84 V 1 kW", 84.0, 1000.0, -1, ["50.59 → 43.77 (+0.68)", "23.94 → 22.15 (+0.18)", "19.21 → 18.28 (+0.09)", "16.84 → 16.34 (+0.05)", "14.47 → 14.40 (+0.01)"]),
        ("HV→LV 109.2 V 100 W", 109.2, 100.0, 1, ["6.83 → 5.23 (+1.42)", "3.61 → 1.81 (+1.70)", "2.87 → 1.10 (+1.70)", "2.51 → 0.75 (+1.71)", "2.15 → 0.39 (+1.72)"]),
        ("LV→HV 109.2 V 100 W", 109.2, 100.0, -1, ["5.29 → 7.01 (−1.72)", "3.20 → 2.36 (+0.84)", "2.65 → 1.47 (+1.18)", "2.37 → 1.01 (+1.36)", "2.09 → 0.55 (+1.54)"]))
de_all = {}
for name, v, p, dirn, cells in TAB2:
    G(f"| {name} |", [t for t in re.findall(r"[\d.]+", name)], "표 첫 칸")
    vals = {ns: eff_point(v, p, dirn, ns) for ns in TSG}
    de_all[(v, p, dirn)] = vals
    for ns, t in zip((100, 25, 12.5, 6.25), cells[:4]):
        a = re.match(r"([\d.]+) → ([\d.]+) \(([+−]?[\d.]+)\)", t)
        s_, b_, de_ = vals[ns]
        A(t, [(a.group(1), s_), (a.group(2), b_), (a.group(3), de_)], "f1eff_sweep*", note=f"{name}, {ns} ns")
    a = re.match(r"([\d.]+) → ([\d.]+) \(([+−]?[\d.]+)\)", cells[4])
    s0, b0 = 2 * vals[6.25][0] - vals[12.5][0], 2 * vals[6.25][1] - vals[12.5][1]
    eta = (lambda L0: 100 * p / (p + L0)) if dirn > 0 else (lambda L0: 100 * (p - L0) / p)  # noqa: B023
    A(cells[4], [(a.group(1), s0), (a.group(2), b0), (a.group(3), eta(b0) - eta(s0))], "외삽 2 L(6.25) − L(12.5)", note=name)
G("| 방향, 점 | 100 ns | 25 ns | 12.5 ns | 6.25 ns | 외삽 |", ["100", "25", "12.5", "6.25"], "표 머리")
G("25 ns 이하는 게이트 시각까지 같고, 듀티를 25 ns 칸으로", ["25", "25"], "f1_eff.py M-B*")
G("외삽은 2·L(6.25) − L(12.5)이고", ["2", "6.25", "12.5"], "f1_eff.py Q7")
G("상수 위상 두 계단 보간, V1 380 V", ["380"], "공통")

SEC[0] = "3 효율"
pv = Sx.pivot_table(index=key, columns=Sx.Ts.map(lambda t: round(t * 1e9, 2)), values="Lf")
z = pv[pv[12.5] > 0.5]
r = (z[6.25] / z[12.5] - 1).abs()
A("12.5 → 6.25 ns에서도 중앙값 13 %(최대 33 %) 더 줄고", [("12.5", 12.5), ("6.25", 6.25), ("13", 100 * r.median()), ("33", 100 * r.max())], "f1eff", note=f"{len(z)}행")
xk = ["ctrl", "V2", "P_target", "dirn", "level"]
xm = Xx.merge(at_grid(Sx, 6.25)[xk + ["Lf"]], on=xk, suffixes=("", "_6"))
A("제어기 25 ns, 솔버 6.25 ns의 값이 6.25 ns 값과 0.008 W 안", [("25", 25.0), ("6.25", 6.25), ("6.25", 6.25), ("0.008", (xm.Lf - xm.Lf_6).abs().max())], "f1eff X 묶음", note=f"{len(xm)}회")
fine = Sx[Sx.Ts < 30e-9]
ROWS.append(dict(sec=SEC[0], text="분석 블록 샘플링이 아니다 (근거표 0.002 W)", token="0.002", reported="0.002", recomputed=f"{(fine.La - fine.Lf).abs().median():.5f}", match="일치" if round((fine.La - fine.Lf).abs().median(), 3) == 0.002 else "불일치", kind="raw", src="f1eff", note="본문에서 뺀 값, 근거표"))
A("(여섯 칸 모두 12.5, 6.25 ns의 효율 차 0.10 pp 안)", [("12.5", 12.5), ("6.25", 6.25), ("0.10", 0.10)], "f1_eff.py Q2", kind="code")
dd = {k: abs(t[6.25][2] - t[12.5][2]) for k, t in de_all.items()}
A("LV→HV 109.2 V 100 W가 넘었고(0.18 pp)", [("109.2", 109.2), ("100", 100.0), ("0.18", dd[(109.2, 100.0, -1)])], "f1eff", note="다른 칸 최대 " + f"{max(x for k, x in dd.items() if k != (109.2, 100.0, -1)):.3f}")
_q5 = re.findall(r"T6\.25 S6\.25: L\s+([\d.]+)\s+P_R40\s+([\d.]+)\s+P_R52\s+([\d.]+)\s+P_D\s+([\d.]+)", open(os.path.join(OUT, "f1eff_check.log"), encoding="utf-8").read())
_q5 = np.array(_q5, float)
okb = (_q5[:, 0] >= 0.9 * (_q5[:, 1] + _q5[:, 3])) & (_q5[:, 0] <= 1.1 * (_q5[:, 2] + _q5[:, 3]))
A("(규칙상 23회 중 16회만 맞음)", [("23", float(len(_q5))), ("16", float(okb.sum()))], "out/f1eff_check.log의 행별 L, P_R, P_D", kind="derived", note="P_R, P_D는 파형 계산값; 로그의 반올림 값으로 경계식을 다시 적용")
_rows = []
for v in ar.V2N:
    for p in ar.PN:
        s_, b_ = pair(MAP, v, p)
        if s_["clean"] and b_["clean"]:
            Ls, Lb = s_["P_LV"] - s_["P_HV"], b_["P_LV"] - b_["P_HV"]
            _rows.append(100 * p / (p + Lb) - 100 * p / (p + Ls))
A("25 ns 기본 52점 평균 +0.33 pp는 수렴값이 아니다", [("25", 25.0), ("52", float(len(_rows))), ("+0.33", float(np.mean(_rows)))], "map, 같은 보간, P_LV − P_HV")
L("SPS 쪽 두 계단이 131–181 W 떨어져 보간으로 최대 0.2 pp 부풀었을 수 있다", ["131", "181", "0.2"], "f1eff_side.log", "interpolation width", note="f1eff_side.py; 아래에서 폭을 다시 잰다")
wid = []
for (v, dirn) in ((84.0, 1), (84.0, -1), (109.2, 1), (109.2, -1)):
    g = at_grid(Sx, 6.25)
    g = g[(g.ctrl == "SPS") & (np.abs(g.V2 - v) < 1e-9) & (g.P_target == 100) & (g.dirn == dirn)].sort_values("level")
    wid.append(float(g.Pdir.max() - g.Pdir.min()))
ROWS.append(dict(sec=SEC[0], text="131–181 W 폭", token="131–181", reported="131–181", recomputed=f"{min(wid):.1f}–{max(wid):.1f}", match="일치" if (round(min(wid)) == 131 and round(max(wid)) == 181) else "불일치", kind="raw", src="f1eff SPS 100 W 두 계단", note=str([round(x, 1) for x in wid])))
G("100 W의 6.25 ns 값은 모두 양수", ["100", "6.25"], "표 2")
G("1 kW의 효율 차는 격자를 줄일수록 작아져 스텝 0 외삽에서 0이고", ["1", "0", "0"], "표 2")
G("(0.18 pp), 6.25 ns 손실은", ["6.25"], "표 2 조건")
F2 = Sx[Sx.Ts < 30e-9].copy()
_real = np.where(F2.P_LV < 0, 1, -1)              # the run's real flow; the block takes the output side as its input
F2["Pa"] = np.where(_real > 0, np.abs(F2.P_LV), F2.P_HV)
est = np.minimum(99.9, 100 * (F2.Pa - F2.Loss_out) / F2.Pa)
n_ok = int(((F2.Eff_out - est).abs() <= 0.1).sum())
ROWS.append(dict(sec=SEC[0], text="추정식 일치 (근거표 72회 중 71회)", token="71/72", reported="71/72", recomputed=f"{n_ok}/{len(F2)}", match="일치" if (n_ok, len(F2)) == (71, 72) else "불일치", kind="raw", src="f1eff 열 Eff_out, Loss_out", note="본문에서 뺀 값, 근거표"))
G("모델에 없는 0.05 Ω을 쓴 추정식", ["0.05"], "dataset_era/Direct_Efficiency_Analyzer.m")
G("비가 100 %를 넘고", ["100"], "코드 판독")
def eff_diff(dd_):
    real = np.where(dd_.P_LV < 0, 1, -1)
    eta = np.where(real > 0, np.abs(dd_.P_LV_d) / np.abs(dd_.P_HV_d), dd_.P_HV_d / dd_.P_LV_d)
    return dd_.Eff_out - 100 * eta


ss = lambda ns: at_grid(Sx, ns)  # noqa: E731
A("입출력 비와의 차는 중앙값 +3.25 pp(100 ns), +0.19 pp(6.25 ns)", [("+3.25", eff_diff(ss(100)).median()), ("100", 100.0), ("+0.19", eff_diff(ss(6.25)).median()), ("6.25", 6.25)], "f1eff, 실제 흐름 방향")

# ------------------------------------------------------------------------------------------------- 4.
SEC[0] = "4 정격"
a, b = ev(84.0, 1200.0), ev(109.2, 1200.0)
A("1.2 kW에서 84 V 5.061 → 5.032 A, 109.2 V 5.378 → 5.232 A(저감 2.7 %)이고 일곱 V2 평균 저감은 0.9 %다", [("1.2", 1.2), ("84", 84.0), ("5.061", a.S), ("5.032", a.B), ("109.2", 109.2), ("5.378", b.S), ("5.232", b.B), ("2.7", 100 * (b.S - b.B) / b.S), ("0.9", (100 * (e12.S - e12.B) / e12.S).mean())], "edge25")
codes = [(float(q.V2), q.S - q.B > ar.tol(q.S)) for q in e12.itertuples()]
n_red = sum(c for _, c in codes)
ROWS.append(dict(sec=SEC[0], text="규칙상 저감은 109.2 V 하나이고 나머지 여섯 점은 허용치 안", token="1/6", reported="1 + 6", recomputed=f"{n_red} + {len(codes) - n_red}, {[v for v, c in codes if c]}", match="일치" if n_red == 1 and [v for v, c in codes if c] == [109.2] else "불일치", kind="raw", src="edge25", note=""))
COVER.append((REPORT.find("규칙상 저감은 109.2 V") + 8, REPORT.find("규칙상 저감은 109.2 V") + 13))
t84, t109 = tr[(84.0, "heavy", 1)], tr[(109.2, "heavy", 1)]
t84b, t109b = tr[(84.0, "heavy", 2)], tr[(109.2, "heavy", 2)]
G("명령 변화율 10 kW/ms의 전력 계단", ["10"], "모델의 MaxSlewRate 1e+7 W/s")
A("(500 → 1000 W, 폐루프)의 과도 피크는 13.1–13.5 A로 정상상태의 약 3배이고 두 제어기가 거의 같다(SPS, 제안 순으로 84 V 13.09, 13.09 A, 109.2 V 13.31, 13.49 A)",
  [("500", 500.0), ("1000", 1000.0), ("13.1", min(t84b["SPS"], t84b["MB"], t109b["SPS"], t109b["MB"])), ("13.5", max(t84b["SPS"], t84b["MB"], t109b["SPS"], t109b["MB"])), ("3", t84b["SPS"] / mv(84.0, 1000.0).S), ("84", 84.0), ("13.09", t84b["SPS"]), ("13.09", t84b["MB"]), ("109.2", 109.2), ("13.31", t109b["SPS"]), ("13.49", t109b["MB"])], "out/r11/ 계단 직후 2 ms")

SEC[0] = "4 k≈1"
kk = pd.concat([Mc.assign(Im=Mc.Sm, Ibm=Mc.Bm)[["V2", "P", "S", "B", "Sm", "Bm"]], Hc.rename(columns=dict(IS="S", IB="B", mS="Sm", mB="Bm"))[["V2", "P", "S", "B", "Sm", "Bm"]]])
kk = kk.assign(k=4 * kk.V2 / 380.0)
k4 = kk[np.abs(1 - kk.k) <= 0.04]
lo = int(((k4.Sm - k4.Bm) > ar.tol(k4.Sm)).sum())
hi = int(((k4.Bm - k4.Sm) > ar.tol(k4.Sm)).sum())
A("|1 − k| ≤ 0.04(V2 91.3–98.7 V, 100–1000 W) 33점의 평균 저감은 −0.006 / −0.004 A이고, 제안 제어기가 허용치 넘게 낮은 점은 0, 높은 점은 3이다(셋 다 6.25 ns에서 사라짐)",
  [("1", 1.0), ("0.04", 0.04), ("91.3", k4.V2.min()), ("98.7", k4.V2.max()), ("100", k4.P.min()), ("1000", k4.P.max()), ("33", float(len(k4))), ("−0.006", (k4.S - k4.B).mean()), ("−0.004", (k4.Sm - k4.Bm).mean()), ("0", float(lo)), ("3", float(hi)), ("6.25", 6.25)], "map + md, 깨끗한 점")
_d = lambda v, p: float(ev(v, p).S - ev(v, p).B)  # noqa: E731
A("25, 50, 75 W 저감은 92.4 V에서 +0.086, +0.044, +0.020 A, 96.6 V에서 −0.053, −0.022, −0.142 A다(0.72° 계단)", [("25", 25.0), ("50", 50.0), ("75", 75.0), ("92.4", 92.4), ("+0.086", _d(92.4, 25.0)), ("+0.044", _d(92.4, 50.0)), ("+0.020", _d(92.4, 75.0)), ("96.6", 96.6), ("−0.053", _d(96.6, 25.0)), ("−0.022", _d(96.6, 50.0)), ("−0.142", _d(96.6, 75.0)), ("0.72", 0.72)], "edge25")
X95 = ar.pick(MAP, 95.0, ar.D_SAFE, ar.D_SAFE)
X40 = ar.pick(MAP, 95.0, 0.40, 0.40)
A("잰 두 점(95 V 70, 100 W, D1 = D2 = 0.40)에서 SPS 0.208, 0.297 A가 TPS 0.252, 0.360 A보다 낮다", [("95", 95.0), ("70", 70.0), ("100", 100.0), ("0.40", 0.40), ("0.208", ar.at(X95, 70.0)["Iw"]), ("0.297", ar.at(X95, 100.0)["Iw"]), ("0.252", ar.at(X40, 70.0)["Iw"]), ("0.360", ar.at(X40, 100.0)["Iw"])], "r13a 묶음 X")
G("k = 1에서 TPS가 SPS보다 낮은 점도", ["1"], "")
L("원본 100 ns에서는 TPS가 0.286, 0.149 A 낮게 나왔고 그 부호가 25 ns에서 뒤집힌다", ["100", "0.286", "0.149", "25"], "r13_map.log", "95 V    70 W  D0.40 lower than SPS   +0.286 A at 100 ns -> -0.044 A at 25 ns (tol 0.030): reversed", note="100 W는 +0.149 → −0.063 A, reversed")
G("k ≈ 1에서는 100–1000 W에서 이득이 없다", ["1", "100", "1000"], "")

SEC[0] = "4 V1"


def vx(variant, v, p, ctrl, V1):
    g = V1X[(V1X.variant == variant)]
    if ctrl == "SPS":
        dd_ = (ar.D_SAFE, ar.D_SAFE)
    elif ctrl == "B":
        dd_ = ar.mb(p, v)
    else:
        dd_ = ar.mb(p, v, V1=V1)
    if max(abs(dd_[0] - 0.5), abs(dd_[1] - 0.5)) <= ar.SAME:
        dd_ = (ar.D_SAFE, ar.D_SAFE)
    if ctrl == "K":
        bb_ = ar.mb(p, v)
        if max(abs(dd_[0] - bb_[0]), abs(dd_[1] - bb_[1])) <= ar.SAME:
            dd_ = bb_
            if max(abs(dd_[0] - 0.5), abs(dd_[1] - 0.5)) <= ar.SAME:
                dd_ = (ar.D_SAFE, ar.D_SAFE)
    r = ar.pick(g, v, dd_[0], dd_[1], tol=1.5e-5)
    return ar.at(r, p) if len(r) else None


cases = []
for variant, V1, Vs in (("V1-5", 361.0, (88.2, 90.3, 92.4)), ("V1+5", 399.0, (98.7, 100.8, 102.9))):
    for v in Vs:
        for p in (100.0, 200.0, 300.0):
            cases.append(("A", variant, V1, v, p))
for variant, V1 in (("V1-5", 361.0), ("V1+5", 399.0)):
    for v in (84.0, 109.2):
        for p in (100.0, 300.0, 1000.0):
            cases.append(("B", variant, V1, v, p))
vr = []
for st, variant, V1, v, p in cases:
    s_, b_, k_ = vx(variant, v, p, "SPS", V1), vx(variant, v, p, "B", V1), vx(variant, v, p, "K", V1)
    vr.append(dict(set=st, variant=variant, V1=V1, V2=v, P=p, S=s_["Iw"], Sm=s_["pk_med"], cS=s_["clean"], B=b_["Iw"], Bm=b_["pk_med"], cB=b_["clean"], K=k_["Iw"], Km=k_["pk_med"], cK=k_["clean"]))
VR = pd.DataFrame(vr)
VA = VR[VR.set == "A"]
worse = lambda x, s_, c: (s_ - x < -np.maximum(0.03, 0.02 * s_)) & c  # noqa: E731
nA = int(worse(VA.B, VA.S, VA.cS & VA.cB).sum())
q = VR[(VR.V1 == 361.0) & (VR.V2 == 90.3) & (VR.P == 100.0)].iloc[0]
A("V1이 ±5 % 움직여 k가 1을 지나는 V2(361 V에서 88.2–92.4 V, 399 V에서 98.7–102.9 V), 100–300 W의 18경우 중 12경우에서 SPS보다 높아진다",
  [("5", 5.0), ("1", 1.0), ("361", 361.0), ("88.2", 88.2), ("92.4", 92.4), ("399", 399.0), ("98.7", 98.7), ("102.9", 102.9), ("100", 100.0), ("300", 300.0), ("18", float(len(VA))), ("12", float(nA))], "v1x")
A("가장 큰 곳은 361 V 90.3 V 100 W의 0.371 → 0.641 A(0.270 / 0.247 A 높음)", [("361", 361.0), ("90.3", 90.3), ("100", 100.0), ("0.371", q.S), ("0.641", q.B), ("0.270", q.B - q.S), ("0.247", q.Bm - q.Sm)], "v1x", note="A 묶음 최대 " + f"{(VA.B - VA.S).max():.3f}")
both = VR[VR.cS & VR.cB & VR.cK]
nb, nk = int(worse(both.B, both.S, True).sum()), int(worse(both.K, both.S, True).sum())
nbm, nkm = int(worse(both.Bm, both.Sm, True).sum()), int(worse(both.Km, both.Sm, True).sum())
_gap, _gm = both.B - both.K, both.Bm - both.Km
A("양 끝 V2를 더한 30경우(둘 다 판정 가능 27)에서 이런 경우가 12에서 0이 되고, 제안 제어기의 피크가 평균 0.052 / 0.047 A 낮아진다", [("30", float(len(VR))), ("27", float(len(both))), ("12", float(nb)), ("0", float(nk)), ("0.052", _gap.mean()), ("0.047", _gm.mean())], "v1x + rob25", note=f"중앙값 {nbm}, {nkm}")
_c1 = both[(both.V1 == 361.0) & (both.V2 == 90.3) & (both.P == 300.0)].iloc[0]
_c2 = both[(both.V1 == 399.0) & (both.V2 == 109.2) & (both.P == 300.0)].iloc[0]
_cost = both[(both.B - both.K) < -np.maximum(0.03, 0.02 * both.S)]
A("361 V 90.3 V 300 W(0.040 / 0.040 A), 399 V 109.2 V 300 W(0.148 / 0.077 A)는 정규화 쪽이 허용치 넘게 높다", [("361", 361.0), ("90.3", 90.3), ("300", 300.0), ("0.040", _c1.K - _c1.B), ("0.040", _c1.Km - _c1.Bm), ("399", 399.0), ("109.2", 109.2), ("300", 300.0), ("0.148", _c2.K - _c2.B), ("0.077", _c2.Km - _c2.Bm)], "v1x + rob25", note=f"허용치 넘는 경우 {len(_cost)}; 정규화 − SPS 최대 {(both.K - both.S).max():+.3f} A")
ROWS.append(dict(sec=SEC[0], text="둘 다 SPS보다 높지는 않다", token="-", reported="허용치 안", recomputed=f"{(_c1.K - _c1.S):+.3f}, {(_c2.K - _c2.S):+.3f} (허용 {ar.tol(_c1.S):.3f}, {ar.tol(_c2.S):.3f})", match="일치" if (_c1.K - _c1.S) <= ar.tol(_c1.S) and (_c2.K - _c2.S) <= ar.tol(_c2.S) else "불일치", kind="raw", src="v1x", note=""))
E1 = {}


def ve(variant, v, p):
    g = V1E[V1E.variant == variant]
    s_ = ar.at(ar.pick(g, v, ar.D_SAFE, ar.D_SAFE), p)
    dd_ = ar.mb(p, v)
    if max(abs(dd_[0] - 0.5), abs(dd_[1] - 0.5)) <= ar.SAME:
        return s_, s_
    return s_, ar.at(ar.pick(g, v, dd_[0], dd_[1], tol=1.5e-5), p)


s_, b_ = ve("V1-1", 92.4, 100.0)
s2, b2 = ve("V1-2", 92.4, 100.0)
_ne = {}
for mag, vs in ((1, (("V1-1", (88.2, 90.3, 92.4)), ("V1+1", (98.7, 100.8, 102.9)))), (2, (("V1-2", (88.2, 90.3, 92.4)), ("V1+2", (98.7, 100.8, 102.9))))):
    nr = nm = 0
    for var, Vs in vs:
        for v in Vs:
            for p in (100.0, 200.0):
                a_, c_ = ve(var, v, p)
                z0, z1 = ve("V1+0", v, p)
                if a_["clean"] and c_["clean"]:
                    base_r = z0["clean"] and z1["clean"] and (z1["Iw"] - z0["Iw"]) > ar.tol(z0["Iw"])
                    base_m = z0["clean"] and z1["clean"] and (z1["pk_med"] - z0["pk_med"]) > ar.tol(z0["pk_med"])
                    nr += int((c_["Iw"] - a_["Iw"]) > ar.tol(a_["Iw"]) and not base_r)
                    nm += int((c_["pk_med"] - a_["pk_med"]) > ar.tol(a_["pk_med"]) and not base_m)
    _ne[mag] = (nr, nm)
A("1 % 낮으면 92.4 V 100 W에서 0.476 → 0.527 A(0.051 A 높음, 25 ns 한 스텝의 절반, 더 고운 격자로 재지 않음), 2 % 낮으면 0.352 → 0.534 A(0.182 A 높음)다(±1 %, ±2 % 각각 100, 200 W, V2 여섯 값, 12경우 중 최댓값 기준 2경우, 중앙값 기준 1경우)",
  [("1", 1.0), ("92.4", 92.4), ("100", 100.0), ("0.476", s_["Iw"]), ("0.527", b_["Iw"]), ("0.051", b_["Iw"] - s_["Iw"]), ("25", 25.0), ("2", 2.0), ("0.352", s2["Iw"]), ("0.534", b2["Iw"]), ("0.182", b2["Iw"] - s2["Iw"]), ("1", 1.0), ("2", 2.0), ("100", 100.0), ("200", 200.0), ("12", 12.0), ("2", float(_ne[1][0])), ("1", float(_ne[1][1]))], "v1e", note=f"1 %: {_ne[1]}, 2 %: {_ne[2]}")
G("링크가 ±1 %로 조정돼도", ["1"], "v1e 규칙 문장")
G("V1 판독도 1 %보다 좋아야 한다. 0.5 %와", ["1 %", "0.5"], "v1e 규칙 문장")
G("입력은 380 V로 두고", ["380"], "v1e_check.py")

SEC[0] = "4 LV→HV"
Rv = []
for v in ar.V2N:
    for p in (100.0, 300.0, 600.0, 1000.0):
        s_, b_ = pair(REV, v, p, sign=-1)
        f = mv(v, p)
        Rv.append(dict(V2=v, P=p, S=s_["Iw"], B=b_["Iw"], Sm=s_["pk_med"], Bm=b_["pk_med"], c=s_["clean"] and b_["clean"], fS=f.S, fB=f.B, fSm=f.Sm, fBm=f.Bm, fc=f.clean))
Rv = pd.DataFrame(Rv)
rc, fc_ = Rv[Rv.c], Rv[Rv.fc]
_cm = Rv[Rv.c & Rv.fc]
A("7 V2 × 100, 300, 600, 1000 W의 28점 중 두 방향 모두 판정 가능한 26점의 평균 저감은 LV→HV +0.118 / +0.127 A, HV→LV +0.122 / +0.131 A다",
  [("7", 7.0), ("100", 100.0), ("300", 300.0), ("600", 600.0), ("1000", 1000.0), ("28", float(len(Rv))), ("26", float(len(_cm))), ("+0.118", (_cm.S - _cm.B).mean()), ("+0.127", (_cm.Sm - _cm.Bm).mean()), ("+0.122", (_cm.fS - _cm.fB).mean()), ("+0.131", (_cm.fSm - _cm.fBm).mean())], "rev25, map", note=f"빠진 점 LV→HV {Rv[~Rv.c][['V2', 'P']].values.tolist()}, HV→LV {Rv[~Rv.fc][['V2', 'P']].values.tolist()}; 방향별 27점 평균 {(rc.S - rc.B).mean():+.3f} / {(rc.Sm - rc.Bm).mean():+.3f}, {(fc_.fS - fc_.fB).mean():+.3f} / {(fc_.fSm - fc_.fBm).mean():+.3f} A")
q = Rv[(Rv.V2 == 100.8) & (Rv.P == 600.0)].iloc[0]
s6, b6 = pair(REV6, 100.8, 600.0, sign=-1)
A("25 ns에서 제안 제어기가 높게 나온 점은 100.8 V 600 W 하나이고(0.069 / 0.030 A 높음, 중앙값은 허용치 안), 6.25 ns에서는 사라진다(SPS가 0.004 / 0.012 A 높음)",
  [("25", 25.0), ("100.8", 100.8), ("600", 600.0), ("0.069", q.B - q.S), ("0.030", q.Bm - q.Sm), ("6.25", 6.25), ("0.004", s6["Iw"] - b6["Iw"]), ("0.012", s6["pk_med"] - b6["pk_med"])], "rev25, rev6", note=f"허용 {ar.tol(q.Sm):.3f}")
six_r = [(100.8, 600.0), (92.4, 300.0), (98.7, 175.0), (98.7, 350.0), (100.8, 175.0), (100.8, 350.0)]
dsix = []
for v, p in six_r:
    s_, b_ = pair(REV6, v, p, sign=-1)
    dsix.append(s_["Iw"] - b_["Iw"])
A("6.25 ns로 잰 것은 이 점을 포함해 여섯 점(28점 안의 둘과 밖의 넷)뿐이고 모두 제안 제어기가 최댓값으로 0.003–0.054 A 낮다. 28점의 나머지 26점은 25 ns 값이다", [("6.25", 6.25), ("28", 28.0), ("0.003", min(dsix)), ("0.054", max(dsix)), ("28", 28.0), ("26", 28.0 - 2.0), ("25", 25.0)], "rev6")
G("25–75 W와 1200 W", ["25", "75", "1200"], "재지 않음")
qs = Rv.assign(d=lambda t: t.S - t.fS)
qm = qs.loc[qs.d.abs().idxmax()]
A("SPS 피크도 방향에 따라 최대 0.489 A 다르므로(109.2 V 1 kW)", [("0.489", abs(qm.d)), ("109.2", qm.V2), ("1", qm.P / 1000.0)], "rev25, map")

SEC[0] = "4 어디서도"
four = [(92.4, 300.0), (105.0, 400.0)]
big = []
for v, p in four:
    q = mv(v, p)
    big.append(q.B - q.S)
for p in (175.0, 350.0):
    q = HH[(HH.V2 == 98.7) & (HH.P == p)].iloc[0]
    big.append(q.IB - q.IS)
d6 = [hv[k][0]["Iw"] - hv[k][1]["Iw"] for k in ((98.7, 175.0), (98.7, 350.0), (92.4, 300.0), (105.0, 400.0))]
A("판정 가능한 96점에서 25 ns 값이 낸 위반은 네 점(92.4 V 300 W, 105 V 400 W, 98.7 V 175, 350 W; 최댓값으로 0.049–0.103 A 높음)이고, 전류 한 스텝이 허용치보다 작은 6.25 ns에서 모두 성립하지 않았다(저감 0.000 … +0.098 A)",
  [("96", float(len(Mc) + len(Hc))), ("25", 25.0), ("92.4", 92.4), ("300", 300.0), ("105", 105.0), ("400", 400.0), ("98.7", 98.7), ("175", 175.0), ("350", 350.0), ("0.049", min(big)), ("0.103", max(big)), ("6.25", 6.25), ("0.000", min(d6)), ("+0.098", max(d6))], "map, md, h12")
n_und = int((~M.clean).sum()) + int((~(HH.cS & HH.cB)).sum())
n_cl = len(Mc) + len(Hc)
A("다시 잰 것은 이 넷과 98.7 V 250 W, 다섯 점뿐이고 나머지 91점은 25 ns 값이다", [("98.7", 98.7), ("250", 250.0), ("91", float(n_cl - 5)), ("25", 25.0)], "map + md 집계")
A("HV→LV, V1 380 V, 100–1000 W에서 기본 56점과 다른 48점, 합 104점 중 판정 불가 8점은 답이 없다", [("380", 380.0), ("100", 100.0), ("1000", 1000.0), ("56", float(len(M))), ("48", float(len(HH))), ("104", float(len(M) + len(HH))), ("8", float(n_und))], "map + md 집계")
s_, b_ = pair(H12[np.abs(H12.Ts / 12.5e-9 - 1) < 1e-6], 98.7, 175.0, Ts=12.5e-9)
A("판정 기준이 아닌 12.5 ns(2.88° 계단)에서는 98.7 V 175 W가 0.046 / 0.014 A 높아 최댓값만 남는다", [("12.5", 12.5), ("2.88", 2.88), ("98.7", 98.7), ("175", 175.0), ("0.046", b_["Iw"] - s_["Iw"]), ("0.014", b_["pk_med"] - s_["pk_med"])], "h12 묶음 S")
s_, b_ = pair(G12, 96.6, 100.0, Ts=12.5e-9)
z = mv(96.6, 100.0)
w = ev(96.6, 100.0)
A("6.25 ns로 재지 않은 후보도 있다. 96.6 V 100 W는 2.88° 계단에서 0.014 A 높아 허용치 안이지만 0.72° 계단에서 0.033 A(허용 0.030 A), 12.5 ns에서 0.035 / 0.002 A 높다",
  [("6.25", 6.25), ("96.6", 96.6), ("100", 100.0), ("2.88", 2.88), ("0.014", z.B - z.S), ("0.72", 0.72), ("0.033", w.B - w.S), ("0.030", ar.tol(w.S)), ("12.5", 12.5), ("0.035", b_["Iw"] - s_["Iw"]), ("0.002", b_["pk_med"] - s_["pk_med"])], "map, edge25 set C, g12")
q, w = ev(96.6, 25.0), ev(96.6, 75.0)
A("사양 끝의 96.6 V 25, 75 W는 0.053 / 0.025, 0.142 / 0.055 A 높다", [("96.6", 96.6), ("25", 25.0), ("75", 75.0), ("0.053", q.B - q.S), ("0.025", q.Bm - q.Sm), ("0.142", w.B - w.S), ("0.055", w.Bm - w.Sm)], "edge25")
s_, b_ = ve("V1+0", 98.7, 200.0)
A("98.7 V 200 W는 최댓값으로만 0.032 A 높다(중앙값은 0.014 A 낮음)", [("98.7", 98.7), ("200", 200.0), ("0.032", b_["Iw"] - s_["Iw"]), ("0.014", s_["pk_med"] - b_["pk_med"])], "v1e 기준선")

SEC[0] = "4 응답"
cm = C11.assign(step=lambda t: t.t0)
stt = cm[(cm.t0 > 0)]
st8 = stt[stt.ctrl.isin(["M-B", "SPS"])].pivot_table(index=["V2", "t0", "P"], columns="ctrl", values="t_settle_ma_ms")
q = st8.loc[(84.0, 0.012, 100)]
oth = st8.drop(index=(84.0, 0.012, 100))
A("84 V 300 → 100 W 정착은 3.44 ms로 SPS 1.35 ms보다 길고(폐루프, PI 이득 ½), 다른 일곱 계단은 0.7–1.1 ms로", [("84", 84.0), ("300", 300.0), ("100", 100.0), ("3.44", q["M-B"]), ("1.35", q["SPS"]), ("0.7", oth["M-B"].min()), ("1.1", oth["M-B"].max())], "out/r11_check.csv (r11_step.py의 이동평균 정착)", kind="derived", note=f"계단 {len(st8)}, SPS 다른 계단 {oth['SPS'].min():.2f}–{oth['SPS'].max():.2f}")
V2E = pd.read_csv(os.path.join(OUT, "r11_v2.csv"))
up = V2E[(V2E.window == "edge up + 2 ms") & V2E.label.str.startswith("r3_g0.5")].pivot_table(index="P", columns="ctrl", values="Ipk_max")
A("V2가 0.5 ms에 84 → 95 V로 오르는 극단 엣지에서는 제안 제어기의 피크가 SPS보다 0.80 A(300 W, 12.04 대 11.25 A), 0.29 A(1 kW, 16.13 대 15.85 A) 높다",
  [("0.5", 0.5), ("84", 84.0), ("95", 95.0), ("0.80", up.loc[300, "M-B"] - up.loc[300, "SPS"]), ("300", 300.0), ("12.04", up.loc[300, "M-B"]), ("11.25", up.loc[300, "SPS"]), ("0.29", up.loc[1000, "M-B"] - up.loc[1000, "SPS"]), ("1", 1.0), ("16.13", up.loc[1000, "M-B"]), ("15.85", up.loc[1000, "SPS"])], "out/r11_v2.csv (r11_step.py V2_PROFILES)", kind="derived")

# ------------------------------------------------------------------------------------------------- 5.
SEC[0] = "5 측정"
G("정규화에는 나눗셈 1회와 곱셈 3회", ["1", "3"], "smooth_reghdc_infer.m: s = 380/V1, s², P s², V2 s")
V3 = {}
for v in (90.0, 109.2):
    ext = periods("r12", f"r2_MB_{v:g}V_ramp")
    for h in (1, 8):
        inn = periods("f1v3", f"v3_MB_{v:g}V_h{h}")
        V3[(v, h)] = (spike(inn, 6e-3, 46e-3)[1], spike(ext, 6e-3, 46e-3)[1])
A("폐루프 램프(90, 109.2 V, 100 → 900 W, 정규화 입력 포함)는 듀티를 밖에서 넣은 실행과 규칙상 같았고(최대 피크 차 +0.036, +0.102 A, 문턱 0.106 A), 추론을 8주기(100 µs)마다 해도 같았다",
  [("90", 90.0), ("109.2", 109.2), ("100", 100.0), ("900", 900.0), ("+0.036", V3[(90.0, 1)][0] - V3[(90.0, 1)][1]), ("+0.102", V3[(109.2, 1)][0] - V3[(109.2, 1)][1]), ("0.106", 0.106), ("8", 8.0), ("100", 8 * 12.5)], "out/f1v3, out/r12", note=f"8주기 {V3[(90.0, 8)][0] - V3[(90.0, 8)][1]:+.3f}, {V3[(109.2, 8)][0] - V3[(109.2, 8)][1]:+.3f}")
st = {}
for v in (90.0, 109.2):
    ext = periods("r12", f"r2_MB_{v:g}V_ramp")
    inn = periods("f1v3", f"v3_MB_{v:g}V_h1")
    st[v] = (float(ext[ext.t_end <= 6e-3 + 1e-9].Ipk.max()), float(inn[inn.t_end <= 6e-3 + 1e-9].Ipk.max()), float(inn.D1.iloc[0]), float(inn.D2.iloc[0]))
A("듀티가 (0.447, 0.500)으로 시작해 기동 0–6 ms 최대 피크가 밖에서 넣은 실행의 21.9, 19.9 A에서 27.1, 26.3 A로 올랐다",
  [("0.447", st[90.0][2]), ("0.500", st[90.0][3]), ("0", 0.0), ("6", 6.0), ("21.9", st[90.0][0]), ("19.9", st[109.2][0]), ("27.1", st[90.0][1]), ("26.3", st[109.2][1])], "out/f1v3, out/r12")
G("이동평균이 0에서 시작하면", ["0"], "f1v3")
A("모델의 소프트 스타트가 0.2 ms인 시험", [("0.2", SLX["SoftStartTime"] * 1e3)], "모델의 SoftStartTime", kind="code")

SEC[0] = "5 분해능"
PIL = pd.read_csv(os.path.join(OUT, "simulink_pi_pilot3_modelgains.csv"))
_p = PIL[(PIL.ctrl == "SPS") & (PIL.V2 == 84.0) & (PIL.Kp == 2)].iloc[0]
A("(84 V 100 W 정상상태 1.73 A가 폐루프 3.1 A)", [("84", 84.0), ("100", 100.0), ("1.73", float(PI[(PI.V2 == 84) & (PI.P == 100) & (PI.ctrl == "SPS")].Ipk_simulink.iloc[0])), ("3.1", float(_p.Ipk_sim))], "simulink_pi_compare.csv, simulink_pi_pilot3_modelgains.csv", kind="derived", note="Kp 2, Ki 49000")
G("원본 100 ns 제어기(위상 2.88°)", ["100", "2.88"], "G = 360 fs Ts")
G("25 ns(0.72°)와", ["25", "0.72"], "360 fs Ts")


def cv(v, p, c, col):
    if p == 500:
        return 0.5 * (cv(v, 400.0, c, col) + cv(v, 600.0, c, col))
    return mv(v, float(p))[col]


_cl = []
for c, nm in (("S", "SPS"), ("B", "MB")):
    for v in (84.0, 109.2):
        for prof, seq in (("light", ((0.0, 0.012, 300), (0.012, 0.024, 100), (0.024, 0.036, 300))), ("heavy", ((0.0, 0.012, 1000), (0.012, 0.024, 500), (0.024, 0.036, 1000)))):
            per = periods("r11", f"r2_g0.5_{nm}_{v:g}V_{prof}")
            for a_, b_, p in seq:
                st2 = per[(per.t_end > b_ - 2e-3 + 1e-9) & (per.t_end <= b_ + 1e-9)]
                st1 = per[(per.t_end > b_ - 1e-3 + 1e-9) & (per.t_end <= b_ + 1e-9)]
                _cl.append(dict(c=c, dm=st2.Ipk.median() - cv(v, p, c + "m", c + "m") if False else st2.Ipk.median() - cv(v, p, c, c + "m"), dx=st1.Ipk.max() - cv(v, p, c, c), phi=st2.phi.max() - st2.phi.min()))
CL = pd.DataFrame(_cl)
A("폐루프 주기 피크 중앙값이 상수 위상 중앙값과 −0.037 … +0.054 A 안으로 같았다(SPS, 제안 제어기 24구간, 마지막 2 ms)", [("−0.037", CL.dm.min()), ("+0.054", CL.dm.max()), ("24", float(len(CL))), ("2", 2.0)], "out/r11/r2_g0.5_*_periods.csv, map; 500 W는 400, 600 W 평균")
A("폐루프 마지막 1 ms 최댓값이 표 1 방식보다 SPS 0.010–0.217 A, 제안 제어기 0.024–0.167 A 크므로", [("1", 1.0), ("1", 1.0), ("0.010", CL[CL.c == "S"].dx.min()), ("0.217", CL[CL.c == "S"].dx.max()), ("0.024", CL[CL.c == "B"].dx.min()), ("0.167", CL[CL.c == "B"].dx.max())], "같음")
A("PI 이득은 Kp 1, Ki 24500에서 안정했고(위상 흔들림 최대 0.61°)", [("1", 1.0), ("24500", 24500.0), ("0.61", CL.phi.max())], "out/r11/r2_g0.5_*", note="마지막 2 ms")
_fq = []
for name, segs in (("r1_SPS_84V_light", ((0.008, 0.012), (0.020, 0.024), (0.032, 0.036))), ("r1_MB_109.2V_heavy", ((0.008, 0.012), (0.020, 0.024), (0.032, 0.036))), ("r1_SPS_109.2V_heavy", ((0.008, 0.012), (0.032, 0.036)))):
    per = periods("r11", name)
    for a_, b_ in segs:
        x = per[(per.t_end > a_) & (per.t_end <= b_)].phi.to_numpy()
        x = x - x.mean()
        f = np.fft.rfftfreq(len(x), 12.5e-6)
        _fq.append(float(f[1:][np.argmax(np.abs(np.fft.rfft(x * np.hanning(len(x))))[1:])]))
A("Kp 2, Ki 49000에서는 위상 이득이 큰 곳에서 약 4 kHz 한계 주기", [("2", 2.0), ("49000", float(pd.read_csv(os.path.join(OUT, "r11", "r1_SPS_84V_light_summary.csv")).Ki.iloc[0])), ("4", np.median(_fq) / 1e3)], "out/r11/r1_*_periods.csv 위상의 FFT", note=f"{min(_fq) / 1e3:.2f}–{max(_fq) / 1e3:.2f} kHz")
G("그래서 위상 분해능 0.72° 이하와", ["0.72"], "")

SEC[0] = "5 MCU"
L("int16 구현 4,618 B, float32 구현 5,128 B(코드 제외)", ["4,618", "5,128"], "export_mcu.log", "memory 4618 B; float32 memory 5128 B", kind="code", note="float32 = 4 (2 D + D + 2 D + 2) B: W, b, M, c")
L("20,253점 시험하면 int16 경로가", ["20,253"], "c_compile_check.log", "y_float32_bits_identical             20253/20253", kind="code")
L("듀티 차는 최대 2.6×10⁻⁴(float32 경로 5×10⁻⁷)다", ["2.6", "10"], "c_compile_check.log", "max_dD_C_q15_vs_float64_model        2.615e-04", kind="code")
L("(float32 경로 5×10⁻⁷)", ["5", "10"], "c_compile_check.log", "max_dD_C_f32_vs_float64_model        5.048e-07", kind="code")

G("연산량은 256차원 × 곱셈 4회와 코사인 표(1025개) 보간", ["256", "4", "1025"], "out/smooth_reghdc.c")
hdr = open(os.path.join(OUT, "SmoothRegHDC_STM32.h"), encoding="utf-8", errors="replace").read() + open(os.path.join(OUT, "smooth_reghdc.c"), encoding="utf-8", errors="replace").read()
ROWS.append(dict(sec=SEC[0], text="코사인 표 1025", token="1025", reported="1025", recomputed=str(re.findall(r"1025|1 << *10|COS_N\w* *\d+", hdr)[:3]), match="일치" if "1025" in hdr or re.search(r"1 *<< *10", hdr) else "불일치", kind="code", src="out/SmoothRegHDC_STM32.h", note=""))
with zipfile.ZipFile(os.path.join(ROOT, "Simulink", "Docs", "ICROS2026.docx")) as zz:
    DOC = re.sub(r"<[^>]+>", "", zz.read("word/document.xml").decode("utf-8"))
ROWS.append(dict(sec="5 MCU", text="논문의 약 60 µs", token="60", reported="60", recomputed=str(re.findall(r"approximately 60 ?㎲", DOC)[:1]), match="일치" if re.search(r"60 ?㎲", DOC) else "불일치", kind="code", src="ICROS2026.docx", note=""))
COVER.append((REPORT.find("논문의 약 60") + 5, REPORT.find("논문의 약 60") + 7))
ROWS.append(dict(sec="6", text="3.392 A in the paper", token="3.392", reported="3.392", recomputed="3.392" if "3.392" in DOC else "없음", match="일치" if "3.392" in DOC else "불일치", kind="code", src="ICROS2026.docx", note=""))
SMALL = {}
for nm, k in (("LUT 49×22", "L49"), ("64차원", "D64"), ("LUT 25×11", "L25"), ("LUT 13×6", "L13")):
    g = T[T[f"own_{k}"] != T["own_MB"]]        # a point where it shares the proposed controller's runs says nothing
    v_ = g[g[f"c_{k}"] & g.c_MB]
    tl = tol_kind(v_.I_SPS)
    tm = tol_kind(v_.m_SPS)
    SMALL[k] = dict(n=len(v_), r=(v_[f"I_{k}"] - v_.I_MB).mean(), m=(v_[f"m_{k}"] - v_.m_MB).mean(), ir=100 * float(((v_[f"I_{k}"] - v_.I_MB).abs() <= tl).mean()), im=100 * float(((v_[f"m_{k}"] - v_.m_MB).abs() <= tm).mean()))
A("LUT 49×22(4,328 B)는 규칙상 \"같음\"이고", [("49", 49.0), ("22", 22.0), ("4,328", 4328.0)], "tgt25b, 실행이 다른 점", note=f"LUT − 제안 {SMALL['L49']['r']:+.3f} / {SMALL['L49']['m']:+.3f} A, 근거표")
small3 = [SMALL[k] for k in ("D64", "L25", "L13")]
A("64차원(1,288 B), LUT 25×11(1,116 B), 13×6(328 B)은 \"부분\"이다(평균 피크는 제안 제어기보다 가장 크게 0.029 / 0.015 A 낮지만 허용치 안의 점이 58–70 / 78–88 %)",
  [("64", 64.0), ("1,288", 1288.0), ("25", 25.0), ("11", 11.0), ("1,116", 1116.0), ("13", 13.0), ("6", 6.0), ("328", 328.0), ("0.029", max(-z_["r"] for z_ in small3)), ("0.015", -SMALL["L13"]["m"]), ("58", min(z_["ir"] for z_ in small3)), ("70", max(z_["ir"] for z_ in small3)), ("78", min(z_["im"] for z_ in small3)), ("88", max(z_["im"] for z_ in small3))], "tgt25b", note=str({k: {a: round(float(b), 3) for a, b in SMALL[k].items()} for k in SMALL}))

SEC[0] = "5 범위"
G("V2 84–109.2 V, HV→LV 25–1200 W, LV→HV 100–1000 W, V1 380 V이고, V1 361–399 V는 100–1000 W의 일부", ["84", "109.2", "25", "1200", "100", "1000", "380", "361", "399", "100", "1000"], "실행 목록")
FL = pd.read_csv(os.path.join(OUT, "flip25_check.csv"))
lag = FL[FL.V2 == 95].lag_ms
A("±300 W, 50 W/ms 한 가지를 84, 95, 109.2 V에서 재서 규칙상 문제가 없었고, 95 V에서는 두 제어기 모두 0 W 둘레에서 전력이 약 1 ms 멈췄다", [("300", 300.0), ("50", 600.0 / 12.0), ("84", 84.0), ("95", 95.0), ("109.2", 109.2), ("95", 95.0), ("0", 0.0), ("1", float(lag.mean()))], "out/flip25_check.csv", kind="derived", note=f"지연 {list(lag)} ms")
G("25 W 미만에서는 모든 비교의 기준으로 잰 SPS(D1 = D2 = 0.5)", ["25", "0.5"], "권고")
G("(399 V 84 V, 361 V 109.2 V가 그 경우이고 위 30경우에 들어 있다)", ["399", "84", "361", "109.2", "30"], "v1x_check.py 머리말: 정규화 입력이 경계로 잘리는 경우")

SEC[0] = "5 이식"
RB = {}
for variant in ("L-10", "L+10", "td200"):
    for v in (84.0, 109.2):
        for p in (100.0, 300.0, 1000.0):
            g = V1X[V1X.variant == variant]
            s_ = ar.at(ar.pick(g, v, ar.D_SAFE, ar.D_SAFE), p)
            b_ = ar.at(ar.pick(g, v, *ar.mb(p, v), tol=1.5e-5), p)
            f = mv(v, p)
            if f.S - f.B > ar.tol(f.S):
                RB[(variant, v, p)] = 100 * (s_["Iw"] - b_["Iw"]) / (f.S - f.B)
ll = [x for k, x in RB.items() if k[0].startswith("L")]
tt = [x for k, x in RB.items() if k[0] == "td200"]
A("L ±10 %만이면 재학습 없이 저감의 72–136 %가 남았고 데드타임 200 ns(공칭 100 ns)는 96–102 %다(84, 109.2 V × 100, 300, 1000 W, 저감이 허용치 안인 84 V 1 kW 제외)", [("10", 10.0), ("72", min(ll)), ("136", max(ll)), ("200", 200.0), ("100", 100.0), ("96", min(tt)), ("102", max(tt)), ("84", 84.0), ("109.2", 109.2), ("100", 100.0), ("300", 300.0), ("1000", 1000.0), ("84", 84.0), ("1", 1.0)], "rob25; 공칭 데드타임 plant_params.md")
G("V1만 바뀌면 정규화로 옮겨 간다(잰 범위는 380 V ±5 %)", ["380", "5"], "v1x")
G("위상 0.72° 이하 격자로", ["0.72"], "")
G("기본 56점과 같은 방식으로", ["56"], "")
jb = json.load(open(os.path.join(ROOT, "automation", "state", "jobs", "r13a.json"), encoding="utf-8"))
hrs = (pd.Timestamp(jb["end"]) - pd.Timestamp(jb["start"])).total_seconds() / 3600
A("(이 연구는 238회, 약 3시간, 6 병렬)", [("238", 238.0), ("3", hrs), ("6", 6.0)], "automation/state/jobs/r13a.json, final_counts.log", kind="log", note=f"{hrs:.2f} h; 보정 곡면 실행 포함")
mns = (pd.Timestamp("2026-10-21 17:46:16") - pd.Timestamp("2026-10-21 17:28:40")).total_seconds() / 60
A("해석 모델 위의 미세조정(약 18분)", [("18", mns)], "automation/state/jobs/e2e_b3.log", kind="log")
g_ = T[T.own_B0 != T.own_MB]
g_ = g_[g_.c_B0 & g_.c_MB]
A("평균 0.018 / 0.012 A의 대부분이 105 V 400 W 한 점의 차", [("0.018", -(g_.I_B0 - g_.I_MB).mean()), ("0.012", -(g_.m_B0 - g_.m_MB).mean()), ("105", 105.0), ("400", 400.0)], "tgt25a", note=f"미세조정 전 곡면이 낮음; {len(g_)}점; 그 점 제외 " + f"{(g_[~((g_.V2 == 105.0) & (g_.P == 400.0))].I_B0 - g_[~((g_.V2 == 105.0) & (g_.P == 400.0))].I_MB).mean():+.3f} / {(g_[~((g_.V2 == 105.0) & (g_.P == 400.0))].m_B0 - g_[~((g_.V2 == 105.0) & (g_.P == 400.0))].m_MB).mean():+.3f} A")
G("기존 1 ms 계단 데이터셋을 학습 목표로", ["1"], "")
_pw = json.load(open(os.path.join(ROOT, "automation", "state", "jobs", "pointwise.json"), encoding="utf-8"))
A("(이 연구의 점별 최적화 약 6분)", [("6", (pd.Timestamp(_pw["end"]) - pd.Timestamp(_pw["start"])).total_seconds() / 60)], "automation/state/jobs/pointwise.json", kind="log", note="5 프로세스")

# ------------------------------------------------------------------------------------------------- 6.
SEC[0] = "6"
Cc = [(v, p) for v, p in [(v, 100.0) for v in ar.V2N] + [(84.0, 1000.0), (96.6, 1000.0), (109.2, 1000.0)]]
dd_r = [(ev(v, p).S - ev(v, p).B) - (mv(v, p).S - mv(v, p).B) for v, p in Cc]
dd_m = [(ev(v, p).Sm - ev(v, p).Bm) - (mv(v, p).Sm - mv(v, p).Bm) for v, p in Cc]
A("0.72°와 2.88° 대조 10점(저감, 0.72° − 2.88° 평균 −0.015 / −0.001 A)", [("0.72", 0.72), ("2.88", 2.88), ("10", float(len(Cc))), ("0.72", 0.72), ("2.88", 2.88), ("−0.015", np.mean(dd_r)), ("−0.001", np.mean(dd_m))], "edge25 set C, map")
L("Simulink 실행은 4,819회이고, 141회가 오류로 끝나", ["4,819", "141"], "final_counts.log", "total 4819 runs, 141 failed", note="final_counts.py를 이 턴에 다시 돌려 확인; 정상상태 4,375, 폐루프 81, 고정 듀티 계단 363; 설정 잘못 137(fine1 117 + g12 20), 의도한 실패 4(sw1)")
A("위 24구간으로", [("24", float(len(CL)))], "위 행")
G("대표 6점의 23행을", ["6", "23"], "repro6.py, out/repro6_check.log")
G("피크와 전력의 차는 0이다", ["0"], "out/repro6_check.log")
per25 = MAP.assign(spread=MAP.pk_max - MAP.pk_min)
n_all, n_np = len(per25), int((per25.spread > 0.005).sum())
A("25 ns에서 709회 중 76회(11 %)는 주기적이지 않았고 그중 72회는 전환 전류가 0.2 A 이하인 엣지를 가졌다", [("25", 25.0), ("709", float(n_all)), ("76", float(n_np)), ("11", 100.0 * n_np / n_all), ("72", 72.0), ("0.2", 0.2)], "fine, r13a, r13b, iL_pp", note="72는 zvs_map25.log")
L("그중 72회는 전환 전류가", ["72"], "zvs_map25.log", "edge current <= 0.2 A: 72/403", kind="log")
A("허용치(0.03 A)를 넘어도 한 스텝(0.106 A) 안의 25 ns 차이는", [("0.03", 0.03), ("0.106", 380 * 25e-9 / 90e-6), ("25", 25.0)], "계산", kind="code")
G("(모델에 없는 요소는 7장)", ["7"], "장 번호")
G("4장의 위반 후보가", ["4"], "장 번호")
CFG = pd.read_csv(os.path.join(OUT, "g12_configs.csv"))
rg = []
for r_ in CFG.itertuples():
    pass
Cf = CFG.dropna(subset=["I25", "I12.5"])
Cper = Cf[(Cf["spread25"] <= 0.005) & (Cf["spread12.5"] <= 0.005)]
dg = (Cper["I25"] - Cper["I12.5"]).abs()
A("같은 42개 운전 조합을 12.5 ns로 재면 두 격자에서 주기적인 27개의 값 차가 중앙값 0.003 A, 최대 0.024 A다", [("42", float(len(CFG))), ("12.5", 12.5), ("27", float(len(Cper))), ("0.003", dg.median()), ("0.024", dg.max())], "out/g12_configs.csv", kind="derived")
L("10 ns는 모든 실행이 흔들려 쓰지 않았다(드라이버 지연을 5 ns 옮기면 12회 중 10회, 12.5 ns면 9회에서 사라져", ["10", "5", "12", "10", "12.5", "9"], "g12_verdict.md", "tp 35 ns 10/12, 12.5 ns 9/12", note="아래 원자료 재계산 행과 대조")
# A1 from the raw period peaks
nom = G10.assign(spread=G10.pk_max - G10.pk_min)
tp = DG10[DG10.variant == "tp35"].assign(spread=lambda t: t.pk_max - t.pk_min)
g12r = G12.assign(spread=G12.pk_max - G12.pk_min)
col_tp = col_12 = 0
for r_ in tp.itertuples():
    a = nom[(np.abs(nom.V2 - r_.V2) < 1e-9) & (np.abs(nom.D1 - r_.D1) < 1.5e-6) & (np.abs(nom.D2 - r_.D2) < 1.5e-6) & (nom.level == r_.level)]
    b = g12r[(np.abs(g12r.V2 - r_.V2) < 1e-9) & (np.abs(g12r.D1 - r_.D1) < 1.5e-6) & (np.abs(g12r.D2 - r_.D2) < 1.5e-6) & (g12r.level == r_.level)]
    s10 = float(a.spread.iloc[-1]) if len(a) else np.nan
    if r_.spread <= max(0.005, 0.25 * s10):
        col_tp += 1
    if len(b) and float(b.spread.iloc[-1]) <= max(0.005, 0.25 * s10):
        col_12 += 1
ROWS.append(dict(sec=SEC[0], text="tp 35 ns 12회 중 10회, 12.5 ns 9회 (원자료)", token="10/9", reported="10, 9 of 12", recomputed=f"{col_tp}, {col_12} of {len(tp)}", match="일치" if (col_tp, col_12, len(tp)) == (10, 9, 12) else "불일치", kind="raw", src="iL_pp/dg10, g10, g12", note=""))
nall = len(nom)
nfl = int((nom.spread > 0.005).sum())
ROWS.append(dict(sec=SEC[0], text="10 ns에서는 모든 실행이 흔들렸는데", token="모든", reported="모든 실행", recomputed=f"{nfl}/{nall} (폭 > 5 mA)", match="일치" if nfl == nall else "불일치", kind="raw", src="iL_pp/g10", note=""))
L("생성 실험을 다시 돌린 195행이 비트 단위로 같았고", ["195"], "staircase_compare.log", "max|dIpk| 0.00e+00 A, max|dP_LV| 0.00e+00 W", note="staircase_compare.py")
L("(보간만 끄면 0.5 A 넘던 저감 13칸의 평균이 1.42 → 0.47 A, 100 ns)", ["0.5", "13", "1.42", "0.47", "100"], "r8_verdict.md", "0.5 A 이상이던 같은 전력 저감 13칸의 평균이 +1.42 A에서 +0.47 A", note="r8_finalize.py, 원본 100 ns 격자")
IPD = pd.read_csv(os.path.join(OUT, "ipk_decompose.csv"))
i7 = IPD[(IPD.clean) & (IPD.V2_dataset.isin([84.0, 109.2]))]
_c25 = np.array([mv(r_.V2_dataset, float(r_.P)).S for r_ in i7.itertuples()])
_d25 = np.array([mv(r_.V2_dataset, float(r_.P)).B for r_ in i7.itertuples()])
A("84, 109.2 V 대표 7점의 평균 1.35 A는 SPS 행 과대 1.15 A와 제안 제어기의 정상상태 저감 0.21 A로 나뉜다(나머지 −0.01 A, 정상상태는 원본 100 ns 값이다. 25 ns 값으로 하면 1.18, 0.29, −0.11 A)",
  [("84", 84.0), ("109.2", 109.2), ("7", float(len(i7))), ("1.35", i7.ds_reduction.mean()), ("1.15", i7.sps_overmeasure.mean()), ("0.21", i7.mb_reduction.mean()), ("−0.01", (i7.D_sl_MB - i7.B_ds_bestTPS).mean()), ("100", 100.0), ("25", 25.0), ("1.18", (i7.A_ds_SPS - _c25).mean()), ("0.29", (_c25 - _d25).mean()), ("−0.11", (_d25 - i7.B_ds_bestTPS).mean())], "out/ipk_decompose.csv (데이터셋 행, 100 ns), map (25 ns)", kind="derived")
dFI = {}
_FC = pd.read_csv(os.path.join(OUT, "f1icros_check.csv"))
for p in (150.0, 200.0):
    s_ = ar.at(ar.pick(FI, 110.0, ar.D_SAFE, ar.D_SAFE), p)
    tab = _FC[_FC.P == p]
    vals = []
    for r_ in tab.itertuples():
        dd_ = (ar.D_SAFE, ar.D_SAFE) if r_.ctrl == "SPS" else (float(r_.D1), float(r_.D2))
        w_ = ar.at(ar.pick(FI, 110.0, dd_[0], dd_[1], tol=1.5e-5), p)
        if w_ and w_["clean"]:
            vals.append((w_["Iw"], w_["pk_med"]))
    dFI[p] = (s_["Iw"], s_["Iw"] - min(x[0] for x in vals), s_["pk_med"] - min(x[1] for x in vals), len(tab))
A("논문의 최대 저감 3.392 A는", [("3.392", 3.392)], "ICROS2026.docx", kind="code")
A("그 점(V2 110 V, 150, 200 W)에서는 SPS 피크 자체가 2.505, 2.633 A여서", [("110", 110.0), ("150", 150.0), ("200", 200.0), ("2.505", dFI[150.0][0]), ("2.633", dFI[200.0][0])], "f1icros")
ROWS.append(dict(sec=SEC[0], text="가장 큰 저감 (근거표 0.759 / 0.785, 0.632 / 0.642 A, 21쌍)", token="-", reported="0.759 0.785 0.632 0.642 21", recomputed=f"{dFI[150.0][1]:.3f} {dFI[150.0][2]:.3f} {dFI[200.0][1]:.3f} {dFI[200.0][2]:.3f} {dFI[150.0][3]}", match="일치" if f"{dFI[150.0][1]:.3f} {dFI[150.0][2]:.3f} {dFI[200.0][1]:.3f} {dFI[200.0][2]:.3f} {dFI[150.0][3]}" == "0.759 0.785 0.632 0.642 21" else "불일치", kind="raw", src="f1icros", note="본문에서 뺀 값, 근거표"))
q = ev(109.2, 25.0)
fg_max = pd.concat([Mc.assign(x=Mc.S - Mc.B)[["V2", "P", "x"]], Ed[Ed.clean & Ed.P.isin([25.0, 50.0, 75.0, 1200.0])].assign(x=lambda t: t.S - t.B)[["V2", "P", "x"]]])
A("그림 1에서 가장 큰 저감은 109.2 V 25 W의 1.376 A다", [("1", None), ("109.2", 109.2), ("25", 25.0), ("1.376", q.S - q.B)], "edge25", note="그림 칸 최대 " + str(fg_max.loc[fg_max.x.idxmax()].to_dict()))
bIC = T[T.c_IC & T.c_MB]
A("(제안 제어기보다 평균 0.415 / 0.422 A 높음)", [("0.415", (bIC.I_IC - bIC.I_MB).mean()), ("0.422", (bIC.m_IC - bIC.m_MB).mean())], "tgt25a")

SEC[0] = "7"
G("경부하 k ≠ 1 몇 점(예: 84, 109.2 V × 100, 300 W)", ["1", "84", "109.2", "100", "300"], "제안")
G("다이오드 역회복 값은 0으로 바꿔도", ["0"], "g12a")
G("연산이 8주기보다 오래 걸리면", ["8"], "f1v3")
G("V1 0.5 %, V2 판독 오차", ["0.5"], "미완료")
L("(Python 플랜트 54경우에서 위반 없음 [참고])", ["54"], "rob_interior_plant.log", "td200: worse 0 of 18", note="V2 9개 × 100, 300 W × L−10, L+10, td200")
G("끝내지 못한 것은 96.6 V 25, 75, 100 W, 98.7 V 200 W와 V1 1 % 어긋남 92.4 V 100 W의", ["96.6", "25", "75", "100", "98.7", "200", "1 %", "92.4", "100"], "미완료 목록")
G("재측정, 판정 불가 8점", ["8"], "")
G("±300 W보다 크거나", ["300"], "미완료 목록")


# ------------------------------------------------------------------------------------------------- scan: is any M-B above SPS statement missing from the report?
SCAN = []


def scan(d, label, sign=1, Ts=None):
    pts = d[d.ctrl.astype(str).str.contains("M-B")][["V2", "P_target"]].drop_duplicates()
    for r_ in pts.itertuples():
        v, p = float(r_.V2), float(r_.P_target)
        s_, b_ = ar.value(d, v, p, "SPS", sign=sign, Ts=Ts), ar.value(d, v, p, "B", sign=sign, Ts=Ts)
        if s_ and b_ and s_["clean"] and b_["clean"]:
            a, m = b_["Iw"] - s_["Iw"], b_["pk_med"] - s_["pk_med"]
            if a > ar.tol(s_["Iw"]) or m > ar.tol(s_["pk_med"]):
                SCAN.append(dict(set=label, V2=v, P=p, above_rule=a, above_med=m, tol=ar.tol(s_["Iw"])))


scan(MAP, "map + X (25 ns, 2.88)")
scan(MD, "off-map (25 ns, 2.88)")
scan(EDGE, "edge + control (25 ns, 0.72)")
scan(V1E[V1E.V1 == 380.0], "v1e baseline (25 ns, 2.88)")
scan(H12[np.abs(H12.Ts / 6.25e-9 - 1) < 1e-6], "h12 (6.25 ns, 0.72)", Ts=6.25e-9)
scan(H12[np.abs(H12.Ts / 12.5e-9 - 1) < 1e-6], "h12 (12.5 ns, 2.88)", Ts=12.5e-9)
scan(G12, "g12 (12.5 ns, 2.88)", Ts=12.5e-9)
scan(FI, "f1icros 110 V (25 ns)")
_tt = DG10[DG10.variant == "TT0"]
_ttd = 0.0
for r_ in _tt.itertuples():
    a = G10[(np.abs(G10.V2 - r_.V2) < 1e-9) & (np.abs(G10.D1 - r_.D1) < 1.5e-6) & (np.abs(G10.D2 - r_.D2) < 1.5e-6) & (G10.level == r_.level)]
    if len(a):
        _ttd = max(_ttd, abs(float(a.Iw.iloc[-1]) - r_.Iw), abs(float(a.P_LV.iloc[-1]) - r_.P_LV), abs(float(a.P_HV.iloc[-1]) - r_.P_HV))
ROWS.append(dict(sec="7", text="다이오드 역회복 값은 0으로 바꿔도 결과가 비트 단위로 같아", token="0", reported="0", recomputed=f"{_ttd:.3e} ({len(_tt)}회)", match="일치" if _ttd == 0 else "불일치", kind="raw", src="dg10 TT0, g10", note=""))
_rl = pd.read_csv(os.path.join(OUT, "rev6_check.csv"))
_rep = open(os.path.join(OUT, "repro6_check.log"), encoding="utf-8").read()
_n23 = len(re.findall(r"Ipk [\d.]+ -> [\d.]+ \(\+0\.00e\+00 A\), P_LV [-\d.]+ -> [-\d.]+ \(\+0\.00e\+00 W\), P_HV \(\+0\.00e\+00 W\)", _rep))
ROWS.append(dict(sec="6", text="대표 6점을 만든 23행 ... 차가 0", token="23", reported="23", recomputed=str(_n23), match="일치" if _n23 == 23 else "불일치", kind="log", src="out/repro6_check.log", note="행마다 Ipk, P_LV, P_HV 차 0"))
SC = pd.DataFrame(SCAN).sort_values(["set", "V2", "P"]).round(4)

# ------------------------------------------------------------------------------------------------- output
Rw = pd.DataFrame(ROWS)
Rw.to_csv(os.path.join(OUT, "final_audit.csv"), index=False, encoding="utf-8-sig")
# coverage
cov = np.zeros(len(REPORT) + 1, bool)
for a, b in COVER:
    cov[a:b] = True
unc = []
for m in TOKEN.finditer(REPORT):
    a, b = m.span()
    if cov[a:b].any():
        continue
    line_start = REPORT.rfind("\n", 0, a) + 1
    line = REPORT[line_start:REPORT.find("\n", a)]
    if line.startswith("#") or line.startswith("![") or line.startswith("|---") or line.startswith("작성 2026"):
        continue
    ctx = REPORT[max(0, a - 12):b + 12].replace("\n", " ")
    if re.search(r"(표|그림) ?\d", REPORT[max(0, a - 3):b]) or REPORT[b:b + 1] == "차" or re.match(r"[①②③④]", REPORT[max(0, a - 1):a]):
        continue
    unc.append(f"{m.group()}\t{ctx}")
open(os.path.join(OUT, "final_audit_uncovered.txt"), "w", encoding="utf-8").write("\n".join(unc) + "\n")
lines = []


def o(x=""):
    print(x, flush=True)
    lines.append(str(x))


cnt = Rw["match"].value_counts().to_dict()
o(f"final_audit: {len(Rw)} tokens; {cnt}; uncovered number tokens {len(unc)}")
o(f"M-A / B0 / D64 duties of tgt25_check.csv against this file's own evaluation of their JSON: largest difference {max(dcheck):.2e}")
bad = Rw[~Rw["match"].isin(["일치", "대상 아님"])]
pd.set_option("display.width", 240, "display.max_colwidth", 70, "display.max_rows", 400)
if len(bad):
    o("\nnot matching:")
    o(bad[["sec", "token", "recomputed", "match", "text", "note"]].to_string(index=False))
o("\nscan: every M-B above SPS by more than tol on the 80-period maximum or the median, both clean, all 25 ns and finer nominal runs, HV -> LV, V1 380 V:")
o(SC.to_string(index=False))
o("\nnotes:")
for r_ in Rw[(Rw.note.astype(str) != "") & Rw["match"].isin(["일치"])].drop_duplicates(["text", "note"]).itertuples():
    if any(ch.isdigit() for ch in str(r_.note)):
        o(f"  [{r_.sec}] {r_.text[:50]} :: {r_.note}")
open(os.path.join(OUT, "final_audit.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")
