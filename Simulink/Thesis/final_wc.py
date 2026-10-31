"""final_wc.py -- word count of FINAL_REPORT.md the way the task counts it (whitespace-separated, like wc -w): whole file, and body = without the table rows, figure lines and HTML comments. Also reports forbidden tokens in the body (internal names, paths)."""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
s = open(os.path.join(HERE, "FINAL_REPORT.md"), encoding="utf-8").read()
s2 = re.sub(r"<!--.*?-->", "", s, flags=re.S)
lines = s2.split("\n")
tab = [x for x in lines if x.lstrip().startswith("|")]
fig = [x for x in lines if x.lstrip().startswith("![")]
body = [x for x in lines if not x.lstrip().startswith("|") and not x.lstrip().startswith("![")]
sect = {}
cur = "(head)"
for x in body:
    if x.startswith("## "):
        cur = x[3:]
    sect[cur] = sect.get(cur, 0) + len(x.split())
print(f"whole file {len(s.split())} words; without comments {len(s2.split())}; table rows {len(tab)} lines, {sum(len(x.split()) for x in tab)} tokens; body {sum(len(x.split()) for x in body)} words")
for k, v in sect.items():
    print(f"  {v:5d}  {k}")
bad = [r"M-B\b", r"M-A\b", r"M-S\b", r"M-D\b", r"R1[0-3]\b", r"OPT\d", r"\bf1[a-z0-9]+\b", r"\br13[ab]?\b", r"\bg12a?\b", r"\bh12\b", r"\brev\d+\b", r"\.py\b", r"\.csv\b", r"\.slx\b", r"\.md\b", r"out/", r"Simulink/", r"\[대기"]
for b in bad:
    m = re.findall(b, s2)
    if m:
        print("  found", b, len(m))
print("tables:", len(re.findall(r"^\*\*표 \d", s2, flags=re.M)), "figures:", len(fig))
