"""
test_c_compile_check.py -- the C compile check is not vacuous: a clean build passes, a build with one broken operation
(>> 15 -> >> 14 in the cos-table interpolation) fails the exactness checks, and neither run touches out/c_compile_check.*.

  python -I -X utf8 Simulink/Thesis/test_c_compile_check.py
"""
import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
SCRIPT = os.path.join(HERE, "c_compile_check.py")


def digest(path):
    return hashlib.md5(open(path, "rb").read()).hexdigest() if os.path.exists(path) else None


def run(*args):
    p = subprocess.run([sys.executable, "-I", "-X", "utf8", SCRIPT, "--n", "2000"] + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
    return p.returncode, p.stdout


def result(txt, name):
    m = re.search(r"^\s+" + re.escape(name) + r"\s+\S+\s+(PASS|FAIL)", txt, flags=re.M)
    return m.group(1) if m else None


def main():
    before = {f: digest(os.path.join(OUT, f)) for f in ("c_compile_check.log", "c_compile_check.csv")}
    ok = True

    rc, txt = run("--no-write")
    good = rc == 0 and "ALL PASS" in txt and result(txt, "int64_accumulators_identical") == "PASS"
    print(f"[{'PASS' if good else 'FAIL'}] clean build: rc {rc}, int64 {result(txt, 'int64_accumulators_identical')}")
    ok &= good

    rc, txt = run("--mutate")
    r_acc, r_y = result(txt, "int64_accumulators_identical"), result(txt, "y_float32_bits_identical")
    r_build = result(txt, "compile_link_delivered_file")
    good = rc == 1 and "MUTANT build A" in txt and r_build == "PASS" and r_acc == "FAIL" and r_y == "FAIL"
    print(f"[{'PASS' if good else 'FAIL'}] mutant build: rc {rc}, compiles {r_build}, int64 {r_acc}, y bits {r_y}")
    ok &= good

    after = {f: digest(os.path.join(OUT, f)) for f in before}
    good = before == after
    print(f"[{'PASS' if good else 'FAIL'}] out/c_compile_check.* untouched by the test runs")
    ok &= good

    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
