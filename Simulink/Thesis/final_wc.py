"""final_wc.py -- word count of FINAL_REPORT.md the way the task counts it (whitespace-separated, like wc -w): whole file, and body = without the table rows, figure lines and HTML comments. Also reports forbidden tokens in the body (internal names, paths), the count with a number and its unit taken as one word (the way a reader counts 어절), and an estimate of A4 pages (no renderer here, so an estimate: 10.5 pt, 25 mm margins, Hangul 1 em, other characters 0.5 em)."""
import math
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
joined = re.sub(r"(\d) (V|W|A|ns|ms|µs|µH|kHz|kW|kB|B|VA|Ω|pp|%|°)(?![가-힣A-Za-z])", r"\1\2", "\n".join(body))
print(f"body with a number and its unit as one word: {len(joined.split())} words; body characters {sum(len(x) for x in body)} (with spaces)")

EM_MM, WIDTH_MM, HEIGHT_MM = 10.5 * 0.3528, 160.0, 247.0


def n_lines(t, width=WIDTH_MM):
    w = sum(1.0 if "가" <= c <= "힣" else 0.5 for c in re.sub(r"[*#|`]", "", t))
    return max(1, math.ceil(w * EM_MM / width)) if t.strip() else 0


text_lines = sum(n_lines(x) for x in body) + sum(1 for x in body if x.strip())  # a blank line's worth between paragraphs
rows = []
for x in tab:
    cells = [c for c in x.strip().strip("|").split("|")]
    if set("".join(cells).strip()) <= {"-", " "}:
        continue
    rows.append(max(n_lines(c, WIDTH_MM / len(cells)) for c in cells))
fig_mm = len(fig) * WIDTH_MM * 3.3 / 7.4
for spacing in (1.3, 1.6):
    lh = 10.5 * spacing * 0.3528
    print(f"estimate at line spacing {spacing:.0%}: text {text_lines} lines, tables {sum(rows)} lines, figure {fig_mm:.0f} mm -> {((text_lines + sum(rows)) * lh + fig_mm) / HEIGHT_MM:.1f} A4 pages")
bad = [r"M-B\b", r"M-A\b", r"M-S\b", r"M-D\b", r"R1[0-3]\b", r"OPT\d", r"\bf1[a-z0-9]+\b", r"\br13[ab]?\b", r"\bg12a?\b", r"\bh12\b", r"\brev\d+\b", r"\.py\b", r"\.csv\b", r"\.slx\b", r"\.md\b", r"Simulink/", r"\[대기"]
body_text = "\n".join(body)
for b in bad:
    m = re.findall(b, body_text)
    if m:
        print("  found", b, len(m))
print("tables:", len(re.findall(r"^\*\*표 \d", s2, flags=re.M)), "figures:", len(fig))
