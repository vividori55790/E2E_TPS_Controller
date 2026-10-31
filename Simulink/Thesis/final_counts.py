"""final_counts.py -- how many Simulink runs stand behind the final report, and how many failed. Written for the report's reliability section and for the audit (F5 d).

Counted, each from where the runner itself recorded it:
  steady   the runner lines 'wrote ... (N points, M errors)' in automation/state/jobs/*.log (automation/job_errors.py reads the same lines)
  closed   the rows of out/{r11,r12,flip25,f1v3}/*_summary.csv and their `error` column (those runners write no such line)
  fixed    gen_dataset_fixed's combinations in the r5r6 log, one 1 ms staircase each (no error column: the job ended normally)
Not counted: smoke jobs without runner lines, the dataset generation rerun (jobs staircase_check, stair_check, stair_i0: their logs record only the file's 195 rows, not a run count), and jobs that stopped before their first simulation (some were stopped on purpose by moving their input file; see PROGRESS).
Which failed runs were run again is checked here too, so "오류는 모두 다시 돌렸다" rests on a file comparison and not on memory.

    python -I -X utf8 Simulink/Thesis/final_counts.py        -> out/final_counts.log
"""
import glob
import os
import re

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
OUT = os.path.join(HERE, "out")
LOGS = os.path.join(ROOT, "automation", "state", "jobs")
LINE = re.compile(r"wrote .*?([^\\/]+\.csv) \((\d+) points?, (\d+) errors?\)")


def steady():
    per, seen = {}, set()
    for f in sorted(glob.glob(os.path.join(LOGS, "*.log")), key=os.path.getmtime):
        name = os.path.basename(f)[:-4]
        with open(f, encoding="utf-8", errors="replace") as h:
            found = LINE.findall(h.read())
        if found:
            per[name] = (sum(int(x[1]) for x in found), sum(int(x[2]) for x in found))
            seen.add(name)
    return per


def closed():
    out = {}
    for sub in ("r11", "r12", "flip25", "f1v3"):
        n = e = 0
        for f in glob.glob(os.path.join(OUT, sub, "*_summary.csv")):
            d = pd.read_csv(f)
            n += len(d)
            if "error" in d.columns:
                x = d["error"].dropna().astype(str)
                e += int((x.str.strip() != "").sum())
        out[sub] = (n, e)
    return out


def fixed():
    with open(os.path.join(LOGS, "r5r6.log"), encoding="utf-8", errors="replace") as h:
        m = re.search(r"gen_dataset_fixed: (\d+) combinations", h.read())
    return int(m.group(1)) if m else 0


def rerun_check():
    """the three bundles with errors: is every failed row in a later file?"""
    lines = []
    d = pd.read_csv(os.path.join(OUT, "simulink_sweep_r1.csv"))
    e = d[d["error"].notna()]
    later = pd.concat([pd.read_csv(f) for f in sorted(glob.glob(os.path.join(OUT, "simulink_sweep_r*.csv"))) if not f.endswith("_r1.csv") and not f.endswith("_in.csv")])
    hit = sum(((later["V2"] == r.V2) & ((later["D0"] - r.D0).abs() < 1e-6) & ((later["D1"] - r.D1).abs() < 1e-6) & ((later["D2"] - r.D2).abs() < 1e-6)).any() for r in e.itertuples())
    lines.append(f"sw1: {len(e)} failed rows (a controller copy made to fail on purpose to end an orphaned job, PROGRESS 10-26), {hit} of them in a later simulink_sweep_r* file")
    return lines


def main():
    st, cl = steady(), closed()
    s_runs, s_err = sum(v[0] for v in st.values()), sum(v[1] for v in st.values())
    c_runs, c_err = sum(v[0] for v in cl.values()), sum(v[1] for v in cl.values())
    fx = fixed()
    lines = ["final_counts: Simulink runs behind the final report", "", "steady-state runner lines, per job (runs, errors):"]
    lines += [f"  {k:14s} {v[0]:5d} {v[1]:4d}" for k, v in st.items()]
    lines += ["", f"steady-state runner lines: {s_runs} runs, {s_err} failed, {s_runs - s_err} good", "closed-loop summaries (rows, error cells):"]
    lines += [f"  {k:14s} {v[0]:5d} {v[1]:4d}" for k, v in cl.items()]
    lines += [f"closed-loop: {c_runs} runs, {c_err} failed", f"fixed-duty staircase rows of gen_dataset_fixed (one run per combination): {fx}", ""]
    lines += [f"total {s_runs + c_runs + fx} runs, {s_err + c_err} failed, {s_runs + c_runs + fx - s_err - c_err} good", ""]
    lines += ["failed bundles with errors:"] + [f"  {k}: {v[1]} of {v[0]}" for k, v in st.items() if v[1]] + rerun_check()
    if "fine1" in st and "fine1b" in st:
        lines.append(f"fine1: every run failed (MATLAB's current folder shadowed the controller copy, PROGRESS 10-30), same point file run again as fine1b: {st['fine1b'][0]} runs, {st['fine1b'][1]} errors")
    if "g12" in st and "g12a" in st:
        lines.append(f"g12: its first 20 runs failed (the runner copy, PROGRESS 10-31), run again as g12a: {st['g12a'][0]} runs, {st['g12a'][1]} errors")
    print("\n".join(lines))
    open(os.path.join(OUT, "final_counts.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
