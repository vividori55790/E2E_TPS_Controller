"""
m_static_check.py -- (2026-10-28) crude syntax check of MATLAB .m files without MATLAB: block keywords vs `end`,
and (), [], {} balance per statement. The loop cannot run MATLAB while the sign-in stall lasts, and a MATLAB file it
edits is first executed by an unattended job, so a missing `end` or bracket would cost a whole run.
Not a parser: comments, strings ('...' / "..."), transposes, %{ %} blocks and `...` continuations are handled; `end`
inside any bracket is an index. Validate on files that already ran in MATLAB (they must pass).
    python -I -X utf8 Simulink/Thesis/m_static_check.py [files...]      (default: every .m in Simulink/Thesis)
"""
import glob
import os
import re
import sys

OPEN = {"function", "if", "for", "parfor", "while", "switch", "try", "spmd"}
PAIR = {")": "(", "]": "[", "}": "{"}


def strip_line(line):
    """remove a comment and string contents; keep code characters (strings become '').
    ' is a transpose only right after an identifier, a closing bracket, '.' or another transpose (no space between);
    otherwise it opens a char vector (`case 'if'`, `[a 'b']`, `disp('x')`)."""
    out, i, n = [], 0, len(line)
    while i < n:
        c = line[i]
        if c == "%":
            break
        if line.startswith("...", i):                         # continuation: rest of the line is a comment
            out.append(" ... ")
            break
        if c == '"' or (c == "'" and not (i > 0 and (line[i - 1].isalnum() or line[i - 1] in "_)]}.'"))):
            q, j = c, i + 1
            while j < n:
                if line[j] == q:
                    if j + 1 < n and line[j + 1] == q:          # doubled quote inside a string
                        j += 2
                        continue
                    break
                j += 1
            if j >= n:
                raise ValueError("unterminated string")
            out.append('""')                                   # placeholder that cannot read as a transpose
            i = j + 1
            continue
        out.append(c)
        i += 1
    return "".join(out)


def check(path):
    errs, depth_stack, brackets = [], [], []
    in_block = False
    for ln, raw in enumerate(open(path, encoding="utf-8", errors="replace").read().splitlines(), 1):
        s = raw.strip()
        if s == "%{":
            in_block = True
            continue
        if in_block:
            if s == "%}":
                in_block = False
            continue
        try:
            code = strip_line(raw)
        except ValueError as e:
            errs.append(f"{ln}: {e}")
            continue
        cont = code.rstrip().endswith("...")
        for m in re.finditer(r"[A-Za-z_]\w*|[()\[\]{}]|\.(?=[A-Za-z_])", code):
            t = m.group(0)
            if t in "([{":
                brackets.append((t, ln))
            elif t in ")]}":
                if not brackets or brackets[-1][0] != PAIR[t]:
                    errs.append(f"{ln}: unmatched '{t}'")
                else:
                    brackets.pop()
            elif brackets:
                continue                                       # inside brackets: `end` is an index
            elif t == "end" and not code[:m.start()].rstrip().endswith("."):
                if not depth_stack:
                    errs.append(f"{ln}: `end` without an open block")
                else:
                    depth_stack.pop()
            elif t in OPEN and not code[:m.start()].rstrip().endswith("."):
                depth_stack.append((t, ln))
        if not cont and any(b[0] == "(" for b in brackets):
            # () may span lines only with `...` ([] and {} literals may: a newline is a row separator there)
            errs.append(f"{ln}: '(' opened on line {brackets[-1][1]} not closed at the end of the line")
            brackets = [b for b in brackets if b[0] != "("]
    for t, ln in depth_stack:
        if t != "function":                                    # a file may omit the final `end` of functions only if
            errs.append(f"{ln}: `{t}` block never closed")      # none has one; mixed use is caught below
    funcs = sum(1 for t, _ in depth_stack if t == "function")
    if funcs and funcs != len(re.findall(r"^\s*function\b", open(path, encoding="utf-8", errors="replace").read(), re.M)):
        errs.append(f"some but not all `function` blocks are closed ({funcs} open)")
    for b, ln in brackets:
        errs.append(f"{ln}: '{b}' never closed")
    return errs


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    files = argv or sorted(glob.glob(os.path.join(here, "*.m")))
    bad = 0
    for f in files:
        errs = check(f)
        print(f"{'OK  ' if not errs else 'FAIL'} {os.path.basename(f)}")
        for e in errs:
            print("     ", e)
        bad += bool(errs)
    print("ALL PASS" if not bad else f"{bad} file(s) with problems")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
