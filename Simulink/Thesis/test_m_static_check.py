"""
test_m_static_check.py -- the checker must pass the .m files that already ran in MATLAB and must flag broken copies
(written to a TEMP dir, never next to the real files) of r5r6_chain.m: a dropped `end`, an extra `end`, a dropped ')',
an unterminated string. Also: `end` as an index, `case 'if'`, transposes, %{ %} blocks and `...` must not trip it.
    python -I -X utf8 Simulink/Thesis/test_m_static_check.py
"""
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m_static_check as M  # noqa: E402

RAN_IN_MATLAB = ["verify_steady_state.m", "verify_steady_batch.m", "staircase_check.m", "inspect_checkpoint.m",
                 "verify_pi_closed_loop.m", "test_smooth_reghdc_infer.m", "smooth_reghdc_infer.m"]


def run(text, tmp, name):
    p = os.path.join(tmp, name)
    with open(p, "w", encoding="utf-8") as f:
        f.write(text)
    return M.check(p)


def main():
    ok = True
    for f in RAN_IN_MATLAB:
        e = M.check(os.path.join(HERE, f))
        ok &= not e
        print(f"{'PASS' if not e else 'FAIL'} known-good {f} {e if e else ''}")
    src = open(os.path.join(HERE, "r5r6_chain.m"), encoding="utf-8").read()
    lines = src.splitlines()
    # the first line that is exactly `end` after an `if` inside the main function
    i_end = next(i for i, s in enumerate(lines) if s.strip() == "end" and i > 10)
    i_paren = next(i for i, s in enumerate(lines) if "fullfile(out, 'out')" in s or "fullfile(here, 'out')" in s)
    cases = {
        "dropped end": lines[:i_end] + lines[i_end + 1:],
        "extra end": lines[:i_end] + ["    end"] + lines[i_end:],
        "dropped )": lines[:i_paren] + [lines[i_paren].replace("'out')", "'out'", 1)] + lines[i_paren + 1:],
        "open string": lines[:i_paren] + [lines[i_paren].replace("'out'", "'out", 1)] + lines[i_paren + 1:],
    }
    with tempfile.TemporaryDirectory() as tmp:
        e = run(src, tmp, "r5r6_chain.m")
        ok &= not e
        print(f"{'PASS' if not e else 'FAIL'} unmodified copy {e if e else ''}")
        for name, ls in cases.items():
            e = run("\n".join(ls) + "\n", tmp, "broken.m")
            ok &= bool(e)
            print(f"{'PASS' if e else 'FAIL'} detects {name}: {e[:2]}")
        tricky = "\n".join([
            "function y = f(x, c)",
            "%{",
            "if while for (( [ end end",
            "%}",
            "    a = x(end) + x(end - 1); b = {1, 2}; b{end + 1} = x';",
            "    s = 'it''s (an [end'; t = \"if ( for\"; u = [a' x.'];",
            "    m = [1 2",
            "         3 4];",
            "    z = foo(1, ...  ( comment with end",
            "            2);",
            "    switch c",
            "        case 'if'",
            "            y = 1;",
            "        otherwise",
            "            y = s(end);",
            "    end",
            "    try, q = 1; catch ME, disp(ME.message); end",
            "end",
        ])
        e = run(tricky + "\n", tmp, "tricky.m")
        ok &= not e
        print(f"{'PASS' if not e else 'FAIL'} tricky constructs {e if e else ''}")
        # (2026-10-30) classdef: properties / methods blocks with attributes, and `methods(obj)` as a call elsewhere
        cls = "\n".join([
            "classdef K < matlab.System",
            "    properties(Nontunable)",
            "        a = 1;",
            "    end",
            "    methods(Access = protected)",
            "        function y = stepImpl(obj, u)",
            "            m = methods(obj); y = u(end) + numel(m);",
            "        end",
            "    end",
            "end",
        ])
        e = run(cls + "\n", tmp, "K.m")
        ok &= not e
        print(f"{'PASS' if not e else 'FAIL'} classdef blocks {e if e else ''}")
        e = run(cls.replace("    end\n    methods", "    methods", 1) + "\n", tmp, "K2.m")
        ok &= bool(e)
        print(f"{'PASS' if e else 'FAIL'} detects unclosed properties block: {e[:2]}")
    print("ALL PASS" if ok else "SOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
