"""
r8_finalize.py -- (2026-10-28) one command for the R.8 update once MATLAB runs again (r5r6_chain): reports which
Simulink outputs exist and runs every analysis whose inputs are there, in this order:
  1. staircase_compare.py      R5 (b)(c): interp = 0 staircase vs steady state      needs staircase_check_interp0.csv
  2. ipk_decompose.py          A - B = (A - C) + (C - D) + (D - B), BT_ steady B     always (BT_ part: stair_steady)
  3. k1_simulink_compare.py    k ~ 1 staircase vs steady (K1_ labels)               needs stair_steady + dataset_fixed
  4. dataset_fixed_analyze.py  R6 / R7: grid-best TPS vs SPS vs M-B                  needs dataset_fixed_part_*.csv
  5. r8_verdict.py             the verdicts by the rules fixed 10-28 12:30 -> out/r8_verdict.md (Korean)   always
Everything goes to out/r8_finalize.log (each script also writes its own log). Partial R6 output is fine.

    python -I -X utf8 Simulink/Thesis/r8_finalize.py
"""
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")


def main():
    have = {
        "interp0": os.path.isfile(os.path.join(OUT, "staircase_check_interp0.csv")),
        "steady": [p for p in glob.glob(os.path.join(OUT, "stair_steady_part*.csv")) if not p.endswith("_in.csv")],
        "fixed": glob.glob(os.path.join(OUT, "dataset_fixed_part_*.csv")),
    }
    lines = [f"Simulink outputs: staircase_check_interp0.csv {'yes' if have['interp0'] else 'NO'}, "
             f"stair_steady parts {len(have['steady'])}, dataset_fixed parts {len(have['fixed'])}"]
    runs = [("staircase_compare.py", have["interp0"] or bool(have["steady"])),
            ("ipk_decompose.py --py", True),
            ("k1_simulink_compare.py", bool(have["steady"])),
            ("dataset_fixed_analyze.py", bool(have["fixed"])),
            ("r8_verdict.py", True)]
    for script, ok in runs:
        lines.append(f"\n===== {script}" + ("" if ok else ": skipped (inputs missing)"))
        if not ok:
            continue
        name, *args = script.split()
        r = subprocess.run([sys.executable, "-I", "-X", "utf8", os.path.join(HERE, name), *args], capture_output=True,
                           text=True, encoding="utf-8", errors="replace", cwd=os.path.dirname(os.path.dirname(HERE)))
        lines.append(r.stdout.rstrip())
        if r.returncode:
            lines.append(f"[rc {r.returncode}] {r.stderr.strip()[-2000:]}")
    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(OUT, "r8_finalize.log"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")


if __name__ == "__main__":
    main()
