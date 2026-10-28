"""
test_r8_verdict.py -- r8_verdict.py's rules on made-up inputs in TEMP dirs (never out/: a stair_steady part there would
make r5r6_chain skip step 2). Five cases, each must give the verdict codes its numbers call for:
  none      : no Simulink output yet                         -> every question "missing"
  artefact  : true staircase keeps 30 % of the dataset's reduction, M-B = best steady combination + 0.02 A,
              k ~ 1 gain 0.02 A, fixed dataset = the Python zoh replay (its own r ~ 0.3)
  partial   : 60 %, 0.10 A, 0.10 A, fixed = 60 % of the original
  user      : 90 %, 0.30 A, 0.30 A, fixed = 90 % of the original
  mb_match  : as artefact, but the fixed grid best = M-B + 0.03 A wherever R.4 has M-B -> Q4 "match"
Steady curves are linear in D0 so that the same-power value is known exactly (BT_: B_st = D - gap; K1_: SPS - TPS = gain).

    python -I -X utf8 Simulink/Thesis/test_r8_verdict.py
"""
import os
import shutil
import sys
import tempfile

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import r8_verdict as v  # noqa: E402

OUT = v.OUT


def make(share, gap, gain, fixed_share, fixed_mb=None):
    tmp = tempfile.mkdtemp(prefix="r8v_")
    for f in ("ipk_decompose.csv", "stair_steady_points.csv", "staircase_check.csv"):
        shutil.copy(os.path.join(OUT, f), tmp)
    c = pd.read_csv(os.path.join(OUT, "staircase_compare.csv"))
    c = c[(c["part"] == "c") & (c["source"] == v.DS)]
    if share is None:
        c.to_csv(os.path.join(tmp, "staircase_compare.csv"), index=False)
        return tmp
    sps0 = c["Ipk_SPS"] - 0.5                                  # a true staircase lowers the SPS rows
    s0 = c.assign(source=v.ST0, dI=c["dI"] * share, Ipk_SPS=sps0, Ipk_TPS=sps0 - c["dI"] * share)
    pd.concat([c, s0, c.assign(source=v.STD, dI=c["dI"] * share)]).to_csv(
        os.path.join(tmp, "staircase_compare.csv"), index=False)
    open(os.path.join(tmp, "staircase_check_interp0.csv"), "w").write("made up\n")

    T = pd.read_csv(os.path.join(OUT, "ipk_decompose.csv"))
    pts = pd.read_csv(os.path.join(OUT, "stair_steady_points.csv"))
    rows = []
    for r in pts.itertuples(index=False):
        if r.label.startswith("BT_"):
            P = float(r.label.split("_")[2].rstrip("W"))
            D = T[np.isclose(T["V2_dataset"], round(r.V2, 2)) & np.isclose(T["P"], P)]["D_sl_MB"].iloc[0]
            pl, ipk = -60.0 * r.D0, D - gap + 0.001 * (60.0 * r.D0 - P)
        elif r.label.startswith("K1_"):
            sl = 55.0 if "_SPS_" in r.label else 50.0
            pl, ipk = -sl * r.D0, 0.45 + 0.002 * sl * r.D0 - (0.0 if "_SPS_" in r.label else gain)
        else:
            pl, ipk = -40.0 * r.D0, 2.0                        # staircase labels: not read by r8_verdict
        rows.append(dict(r._asdict(), Ipk_sim=ipk, P_LV=pl, Ipk_at_20ms=ipk))
    pd.DataFrame(rows).to_csv(os.path.join(tmp, "stair_steady_part001.csv"), index=False)

    shutil.copy(os.path.join(OUT, "dataset_fixed_py_zoh_part_001.csv"), os.path.join(tmp, "dataset_fixed_part_001.csv"))
    d = pd.read_csv(os.path.join(OUT, "dataset_fixed_compare_py_zoh.csv"))
    if fixed_share is not None:
        sub = d[d["source"] == "orig_sub"].set_index(["V2", "direction", "P"])["dI"]
        m = d["source"] == "fixed"
        d.loc[m, "dI"] = [fixed_share * sub.get((a, b, p), np.nan)
                          for a, b, p in zip(d.loc[m, "V2"], d.loc[m, "direction"], d.loc[m, "P"])]
    if fixed_mb is not None:                                   # fixed grid best = M-B + fixed_mb where R.4 exists
        mb = d[d["source"] == "R4_MB_steady"].set_index(["V2", "direction", "P"])["dI"]
        for i in d.index[d["source"] == "fixed"]:
            k = (d.at[i, "V2"], d.at[i, "direction"], d.at[i, "P"])
            if k in mb.index:
                d.at[i, "dI"] = mb[k] + fixed_mb
    d.to_csv(os.path.join(tmp, "dataset_fixed_compare.csv"), index=False)
    return tmp


def main():
    cases = {
        "none": ((None, 0, 0, None), dict(q1="missing", q2="missing", q3="missing", q4="missing", q4_mb="missing")),
        "artefact": ((0.3, 0.02, 0.02, None), dict(q1="artefact", q2="equal", q3="none", q4="artefact")),
        "partial": ((0.6, 0.10, 0.10, 0.6), dict(q1="partial", q2="small", q3="small", q4="partial")),
        "user": ((0.9, 0.30, 0.30, 0.9), dict(q1="user", q2="real", q3="real", q4="user")),
        "mb_match": ((0.3, 0.02, 0.02, None, 0.03), dict(q4_mb="match")),
    }
    ok = True
    for name, (args, want) in cases.items():
        tmp = make(*args)
        assert os.path.abspath(tmp) != os.path.abspath(OUT)
        got = v.main(["--out", tmp])
        md = open(os.path.join(tmp, "r8_verdict.md"), encoding="utf-8").read()
        bad = {k: (got[k], w) for k, w in want.items() if got[k] != w}
        nan_txt = "nan" in md.replace("Python", "")
        print(f"{name:9s} {'PASS' if not bad and not nan_txt else 'FAIL'} {got}" + (f"  mismatch {bad}" if bad else "")
              + ("  'nan' in report" if nan_txt else ""))
        ok &= not bad and not nan_txt
        if name == "user":
            assert "결론을 고쳐야 하는 판정" in md
        shutil.rmtree(tmp)
    print("ALL PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
