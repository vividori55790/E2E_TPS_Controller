"""Per job: runs and errors counted from the runner lines 'wrote ... (N points, M errors)' in automation/state/jobs/*.log.

Added 10-31 after g12: its first 20 runs all failed in the first minute and the job went on with 83 good ones, so the status
line (last log line) and a look at the later chunks both said "no errors". Run this after a job ends, before writing "오류 0".
    python automation/job_errors.py            every job
    python automation/job_errors.py g12 v1x    those jobs
"""
import glob
import os
import re
import sys

LOGS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "state", "jobs")
LINE = re.compile(r"wrote .*?([^\\/]+\.csv) \((\d+) points?, (\d+) errors?\)")


def main(names):
    for f in sorted(glob.glob(os.path.join(LOGS, "*.log")), key=os.path.getmtime):
        name = os.path.basename(f)[:-4]
        if names and name not in names:
            continue
        with open(f, encoding="utf-8", errors="replace") as h:
            found = LINE.findall(h.read())
        if not found:
            if names:
                print(f"{name:14s} no runner lines")
            continue
        runs, errors = sum(int(x[1]) for x in found), sum(int(x[2]) for x in found)
        bad = [f"{x[0]}: {x[2]}/{x[1]}" for x in found if int(x[2])]
        print(f"{name:14s} runs {runs:5d} errors {errors:4d}" + ("   " + "; ".join(bad[:6]) + (" ..." if len(bad) > 6 else "") if bad else ""))


if __name__ == "__main__":
    main(sys.argv[1:])
