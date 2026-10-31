"""audit_raw.py -- the raw-data side of the final report's audit (F5, final_audit.py). Written on purpose without importing any of the study's comparison code
(fine_grid_check, r13_map, md_check ...): it reads the runner's own result files and redoes the steps those scripts do, so an error there shows up here as a mismatch.

  load(prefix)         every row of out/<prefix>_sweep*.csv without an error, with the 80 period peaks of out/iL_pp/<same file> (pk_max, pk_med)
  mb(P, V2)            the proposed controller's (D1, D2), evaluated from out/model_smooth_reghdc_b3.json by the formula of smooth_reghdc_infer.m, not by controllers.py
  at(rows, P)          value at |P|: the first adjacent pair of levels, going away from D0 = 0, whose window powers bracket |P|, linear in that power
"""
import glob
import json
import math
import os
import re

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
V1N = 380.0
L = 90e-6
D_SAFE = 0.4999999
OSC = 0.020
SAME = 0.003
PN = (100.0, 150.0, 200.0, 300.0, 400.0, 600.0, 800.0, 1000.0)
V2N = (84.0, 88.2, 92.4, 96.6, 100.8, 105.0, 109.2)


def _model(name="model_smooth_reghdc_b3.json"):
    m = json.load(open(os.path.join(OUT, name), encoding="utf-8"))
    return dict(W=np.array(m["W"], float), b=np.array(m["b"], float), M=np.array(m["M"], float), c=np.array(m["c"], float), D=int(m["D"]), tau=float(m["tau"]),
                P_MAX=float(m["P_MAX"]), V2_LO=float(m["V2_LO"]), V2_HI=float(m["V2_HI"]), D_MIN=float(m["D_MIN"]))


_B = None


def mb(P, V2, V1=V1N, model=None):
    """(D1, D2) of the proposed controller, smooth_reghdc_infer.m line by line. V1 is the normalised input; V1 = 380 V gives the plain one"""
    global _B
    if model is None:
        if _B is None:
            _B = _model()
        p = _B
    else:
        p = model
    s = V1N / max(V1, 1.0)
    u = min(max(abs(P) * s * s / p["P_MAX"], 0.0), 1.0)
    v = min(max((V2 * s - p["V2_LO"]) / (p["V2_HI"] - p["V2_LO"]), 0.0), 1.0)
    W = p["W"]
    x = np.array([u, v])
    # W is D x 2 or 2 x D depending on how it was stored
    if W.shape[0] == 2 and W.shape[1] == p["D"]:
        phase = x @ W
    else:
        phase = W @ x
    h = math.sqrt(2.0) * np.cos(phase + p["b"])
    M = p["M"]
    y = (h @ M) / math.sqrt(p["D"]) + p["c"]
    t = y / p["tau"]
    sp = p["tau"] * (np.maximum(t, 0.0) + np.log1p(np.exp(-np.abs(t))))
    delta = np.minimum(sp, 0.5 - p["D_MIN"])
    return float(0.5 - delta[0]), float(0.5 - delta[1])


def _pp_file(f):
    g = os.path.join(OUT, "iL_pp", os.path.basename(f))
    return g if os.path.exists(g) else None


def _peaks(path):
    T = pd.read_csv(path)
    mx = [c for c in T.columns if re.fullmatch(r"max\d+", c)]
    mn = [c for c in T.columns if re.fullmatch(r"min\d+", c)]
    pk = np.maximum(T[mx].to_numpy(float), -T[mn].to_numpy(float))
    return pd.DataFrame(dict(label=T["label"], pk_max=np.nanmax(pk, axis=1), pk_med=np.nanmedian(pk, axis=1), pk_min=np.nanmin(pk, axis=1)))


def load(prefix, pp=True, win=20):
    """prefix may be a list. Returns rows with Pw = the 'win' ms window's P_LV, Iw = its peak (80-period largest), pk_* from the period file."""
    fr = []
    for pre in ([prefix] if isinstance(prefix, str) else prefix):
        fs = [f for f in sorted(glob.glob(os.path.join(OUT, f"{pre}_sweep*.csv"))) if not f.endswith("_in.csv")]
        for f in fs:
            T = pd.read_csv(f)
            if "error" in T.columns:
                T = T[T["error"].isna() | (T["error"].astype(str).str.strip() == "")]
            if T.empty:
                continue
            if pp:
                g = _pp_file(f)
                if g:
                    T = T.merge(_peaks(g).drop_duplicates("label"), on="label", how="left")
            T = T.assign(src=os.path.basename(f))
            fr.append(T)
    d = pd.concat(fr, ignore_index=True)
    pc, ic = f"P_LV_at_{win}ms", f"Ipk_at_{win}ms"
    if pc in d.columns:
        d["Pw"] = d[pc]
        d["Iw"] = d[ic]
    return d


def pick(d, V2, D1, D2, Ts=None, tol=1.5e-6, V1=None):
    m = (np.abs(d["V2"] - V2) < 1e-9) & (np.abs(d["D1"] - D1) <= tol) & (np.abs(d["D2"] - D2) <= tol)
    if Ts is not None:
        m &= np.abs(d["Ts"] / Ts - 1) < 1e-6
    if V1 is not None and "V1" in d.columns:
        m &= np.abs(d["V1"] - V1) < 1e-9
    g = d[m]
    return g.sort_values("src").drop_duplicates("level", keep="last").sort_values("level")


def at(rows, P, sign=1, cols=("Iw", "pk_med")):
    """sign +1: HV -> LV, power = -P_LV, levels upward from the first; sign -1: LV -> HV, power = +P_LV, levels downward. None if not bracketed."""
    g = rows.dropna(subset=["Pw", "Iw"]).sort_values("level", ascending=sign > 0)
    Pd, lv = (-sign * g["Pw"]).to_numpy(), g["level"].to_numpy()
    for i in range(len(g) - 1):
        if abs(lv[i + 1] - lv[i]) == 1 and min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]) and Pd[i] != Pd[i + 1]:
            t = (P - Pd[i]) / (Pd[i + 1] - Pd[i])
            a, b = g.iloc[i], g.iloc[i + 1]
            out = {c: a[c] + t * (b[c] - a[c]) for c in cols if c in g.columns}
            out.update(osc=max(abs(a["iL_mean"]), abs(b["iL_mean"])), lo=int(min(lv[i], lv[i + 1])), t=t, a=a, b=b,
                       clean=max(abs(a["iL_mean"]), abs(b["iL_mean"])) <= OSC)
            for c in ("P_LV", "P_HV"):
                if c in g.columns:
                    out[c] = a[c] + t * (b[c] - a[c])
            return out
    return None


def value(d, V2, P, ctrl, sign=1, Ts=None, D=None, V1=None):
    """the value of controller ctrl ('SPS' or 'B', or D = (D1, D2)) at (V2, P). B within SAME of SPS uses the SPS runs, as every study script does."""
    if ctrl == "SPS":
        d1, d2 = D_SAFE, D_SAFE
    elif D is not None:
        d1, d2 = D
    else:
        d1, d2 = mb(P, V2)
    if max(abs(d1 - 0.5), abs(d2 - 0.5)) <= SAME:
        d1, d2 = D_SAFE, D_SAFE
    r = pick(d, V2, d1, d2, Ts=Ts, V1=V1, tol=6e-6)
    return at(r, P, sign=sign) if len(r) else None


def tol(sps):
    return np.maximum(0.03, 0.02 * sps)
