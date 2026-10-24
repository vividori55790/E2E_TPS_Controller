"""
automation/loop_runner.py -- autonomous development loop for the DAB thesis (ICROS2026 -> graduation thesis).

The runner is the hands, the agent (Claude Code CLI, headless) is the brain:
  * every iteration = one agent turn that reads automation/TASKS.md + PROGRESS.md + job status, does the next
    piece of work, updates those files and ends with a ```json REPORT``` block;
  * long computations never run inside a turn: the agent submits them with automation/job.py and ends the turn;
    the runner then waits for the jobs WITHOUT spending Claude usage (compute and Claude overlap);
  * pacing on the ACCOUNT's real utilisation (the CLI's rate_limit_event, as in GradThesis loop_runner v0.4.28):
    the weekly allowance is spent on an even schedule, the 5-hour window is kept below five_max, and a limit
    message sleeps until its reset and resumes the same turn;
  * after each turn the runner commits Simulink/Thesis and automation (local only, never pushes);
  * a STOP file ends it cleanly from any session; the heartbeat tells a reconnecting session it is alive.

Pieces adapted from GradThesis/loop_runner.py (find_claude_bin, parse_cli_output, limit parsing, rate-limit probe).

Usage
  python automation/loop_runner.py                 run (normally started detached by automation/start_loop.ps1)
  python automation/loop_runner.py --status        heartbeat, last turns, usage, jobs
  python automation/loop_runner.py --stop "why"    graceful stop at the next check
  python automation/loop_runner.py --once          one turn, then exit (testing)
  python automation/loop_runner.py --dry-run       write the prompt, call nothing
"""
import argparse
import datetime as dt
import glob
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AUTO = os.path.join(ROOT, "automation")
STATE_DIR = os.path.join(AUTO, "state")
STATE = os.path.join(STATE_DIR, "state.json")
LOG = os.path.join(STATE_DIR, "loop_log.md")
HEARTBEAT = os.path.join(STATE_DIR, "heartbeat.txt")
STOPFILE = os.path.join(STATE_DIR, "STOP")
PROMPTS = os.path.join(STATE_DIR, "prompts")
REPORTS = os.path.join(STATE_DIR, "reports")
TASKS = os.path.join(AUTO, "TASKS.md")
PROGRESS = os.path.join(AUTO, "PROGRESS.md")
PROMPT_FILE = os.path.join(AUTO, "LOOP_PROMPT.md")
COMMIT_PATHS = ["Simulink/Thesis", "automation"]
WEEK_S = 7 * 86400

CFG = dict(models=["claude-opus-5-5"], probe_model="claude-haiku-4-5-20251001",
           five_max=0.85, week_max=1.00, week_burst=0.05,
           turn_timeout_s=3 * 3600, min_gap_s=60, job_wait_max_min=240, effort="", boost_effort="",
           skip_permissions=False, max_fail=6)

ALLOWED_TOOLS = ",".join([
    "Read", "Edit", "Write", "Glob", "Grep", "WebSearch", "WebFetch", "TodoWrite",
    "Bash(python:*)", "Bash(python3:*)", "Bash(py:*)",
    "Bash(git status:*)", "Bash(git diff:*)", "Bash(git log:*)", "Bash(git show:*)", "Bash(git add:*)",
    "Bash(git commit:*)", "Bash(git mv:*)", "Bash(git rm:*)",
    "Bash(ls:*)", "Bash(cat:*)", "Bash(head:*)", "Bash(tail:*)", "Bash(grep:*)", "Bash(find:*)", "Bash(wc:*)",
    "Bash(mkdir:*)", "Bash(cp:*)", "Bash(mv:*)", "Bash(cd:*)", "Bash(echo:*)", "Bash(sort:*)", "Bash(diff:*)",
])
DISALLOWED_TOOLS = ",".join(["Bash(git push:*)", "Bash(git reset --hard:*)", "Bash(rm -rf:*)",
                             "Bash(git checkout:*)", "Bash(git clean:*)"])
CLAUDE_BIN = [None]


# ------------------------------------------------------------------ small utilities
def now():
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def log(line):
    os.makedirs(STATE_DIR, exist_ok=True)
    msg = f"[{now()}] {line}"
    with io.open(LOG, "a", encoding="utf-8") as f:
        f.write(msg + "\n")
    # the detached console is cp949: a U+2212 in a turn summary killed the runner on 10-21 (silent 3-day stop)
    enc = getattr(sys.stdout, "encoding", None) or "utf-8"
    try:
        print(msg.encode(enc, errors="replace").decode(enc, errors="replace"), flush=True)
    except Exception:
        pass


def load_json(p, default):
    try:
        return json.load(io.open(p, encoding="utf-8"))
    except Exception:
        return default


def save_json(p, obj):
    tmp = p + ".tmp"
    json.dump(obj, io.open(tmp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    os.replace(tmp, p)


def heartbeat(info=""):
    os.makedirs(STATE_DIR, exist_ok=True)
    io.open(HEARTBEAT, "w", encoding="utf-8").write(f"{time.time():.0f} {now()} pid={os.getpid()} {info}\n")


def stop_requested():
    return os.path.exists(STOPFILE)


def tail(path, n):
    try:
        return "".join(io.open(path, encoding="utf-8", errors="replace").readlines()[-n:])
    except Exception:
        return ""


def sh(cmd, timeout=600):
    p = subprocess.run(cmd, cwd=ROOT, shell=isinstance(cmd, str), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, encoding="utf-8", errors="replace", timeout=timeout)
    return p.returncode, p.stdout or ""


def sleep_checked(seconds, why):
    if seconds <= 0:
        return
    log(f"{why}: sleeping {seconds / 60:.0f} min (until {(dt.datetime.now() + dt.timedelta(seconds=seconds)).strftime('%m-%d %H:%M')})")
    end = time.time() + seconds
    while time.time() < end:
        time.sleep(min(60, max(1, end - time.time())))
        heartbeat(f"sleep:{why[:40]}")
        if stop_requested():
            log("  stop requested -- waking early")
            return


# ------------------------------------------------------------------ the CLI
def claude_version(path):
    try:
        p = subprocess.run([path, "--version"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                           encoding="utf-8", errors="replace", timeout=30)
        if "claude code" not in (p.stdout or "").lower():
            return ()
        m = re.search(r"(\d+)\.(\d+)\.(\d+)", p.stdout or "")
        return tuple(int(x) for x in m.groups()) if m else ()
    except Exception:
        return ()


def find_claude_bin():
    """Newest Claude Code CLI on the machine. NOTE: on this desktop PATH's `claude` is an unrelated Python
    package (Python313/Scripts/claude.exe); only binaries that answer '... (Claude Code)' are accepted."""
    home = os.path.expanduser("~")
    cands = [shutil.which("claude") or "",
             os.path.join(os.environ.get("APPDATA", ""), "npm", "claude.cmd"),
             os.path.join(home, ".local", "bin", "claude.exe"),
             os.path.join(home, ".claude", "local", "claude.exe")]
    for ide in (".vscode", ".vscode-server", ".vscode-insiders", ".cursor"):
        cands += glob.glob(os.path.join(home, ide, "extensions", "anthropic.claude-code-*", "resources",
                                        "native-binary", "claude*.exe"))
    best, bestv, seen = None, (), set()
    for c in cands:
        if not c or not os.path.isfile(c) or os.path.normcase(c) in seen:
            continue
        seen.add(os.path.normcase(c))
        v = claude_version(c)
        if v and v > bestv:
            best, bestv = c, v
    return best


def ensure_cli(wait_s=1800):
    end = time.time() + wait_s
    while True:
        if CLAUDE_BIN[0] and os.path.isfile(CLAUDE_BIN[0]):
            return True
        b = find_claude_bin()
        if b:
            if b != CLAUDE_BIN[0]:
                log(f"claude CLI: {b} {'.'.join(map(str, claude_version(b)))}")
            CLAUDE_BIN[0] = b
            return True
        if time.time() > end:
            return False
        heartbeat("waiting-for-cli")
        time.sleep(60)


def parse_stream(raw):
    """stream-json lines -> (final text, meta, rate_limits)"""
    meta = dict(session_id=None, cost=0.0, num_turns=0, is_error=False, api_error_status=None, result="",
                duration_ms=0, out_tokens=0, cache_read=0)
    rl = None
    for ln in (raw or "").splitlines():
        ln = ln.strip()
        if not ln.startswith("{"):
            continue
        try:
            j = json.loads(ln)
        except Exception:
            continue
        if not isinstance(j, dict):
            continue
        if isinstance(j.get("rate_limit_info"), dict):
            got = _rate_from_info(j["rate_limit_info"])
            if got:
                rl = got
        if j.get("type") == "result":
            u = j.get("usage") or {}
            meta.update(session_id=j.get("session_id"), cost=float(j.get("total_cost_usd") or 0.0),
                        num_turns=int(j.get("num_turns") or 0), is_error=bool(j.get("is_error")),
                        api_error_status=j.get("api_error_status"), result=j.get("result") or "",
                        duration_ms=int(j.get("duration_ms") or 0), out_tokens=int(u.get("output_tokens") or 0),
                        cache_read=int(u.get("cache_read_input_tokens") or 0))
    return meta["result"], meta, rl


def _rate_from_info(info):
    w = info.get("unifiedWindows") or {}

    def one(key):
        d = w.get(key) or {}
        try:
            return float(d["utilization"]), float(d["resetsAt"])
        except Exception:
            return None, None
    five, week = one("five_hour"), one("seven_day")
    if five[0] is None and week[0] is None:
        return None
    return dict(five=five, week=week, status=str(info.get("status") or ""), ts=time.time())


_MONTHS = {m: i for i, m in enumerate(("jan", "feb", "mar", "apr", "may", "jun", "jul", "aug", "sep", "oct", "nov", "dec"), 1)}


def parse_reset_time(text):
    d = re.search(r"resets?\s+(?:on\s+)?([A-Za-z]{3})[A-Za-z]*\.?\s+(\d{1,2})(?:st|nd|rd|th)?,?\s*(?:at\s+)?"
                  r"(\d{1,2})(?::(\d{2}))?\s*(am|pm)", text or "", flags=re.I)
    if d and d.group(1).lower() in _MONTHS:
        h = int(d.group(3)) % 12 + (12 if d.group(5).lower() == "pm" else 0)
        n = dt.datetime.now()
        try:
            t = dt.datetime(n.year, _MONTHS[d.group(1).lower()], int(d.group(2)), h, int(d.group(4) or 0))
        except ValueError:
            return None
        if t < n - dt.timedelta(days=180):
            t = t.replace(year=n.year + 1)
        return t
    m = re.search(r"resets?\s+(\d{1,2})(?::(\d{2}))?\s*(am|pm)", text or "", flags=re.I)
    if not m:
        return None
    h = int(m.group(1)) % 12 + (12 if m.group(3).lower() == "pm" else 0)
    t = dt.datetime.now().replace(hour=h, minute=int(m.group(2) or 0), second=0, microsecond=0)
    return t if t > dt.datetime.now() else t + dt.timedelta(days=1)


def is_limit(meta):
    r = (meta.get("result") or "").lower()
    return meta.get("api_error_status") == 429 or (meta.get("is_error") and "limit" in r)


# ------------------------------------------------------------------ usage pacing
USAGE = dict(last=None, ts=0.0)


def probe_usage(force=False):
    """The account's real utilisation (the user's own sessions included), from one tiny haiku call."""
    if not force and USAGE["last"] and time.time() - USAGE["ts"] < 300:
        return USAGE["last"]
    USAGE["ts"] = time.time()
    try:
        if not ensure_cli(60):
            return None
        cmd = [CLAUDE_BIN[0], "-p", "Reply with the single word ok.", "--model", CFG["probe_model"],
               "--output-format", "stream-json", "--verbose", "--no-session-persistence"]
        p = subprocess.run(cmd, cwd=tempfile.gettempdir(), timeout=120, stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8",
                           errors="replace", env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        _, _, rl = parse_stream(p.stdout)
        USAGE["last"] = rl
        return rl
    except Exception as e:
        log(f"  usage probe failed: {type(e).__name__}: {str(e)[:120]}")
        USAGE["last"] = None
        return None


def usage_text(u):
    if not u:
        return "account usage: unknown"
    (f, fr), (w, wr) = u["five"], u["week"]
    parts = []
    if f is not None:
        parts.append(f"5h {f:.0%} (resets {dt.datetime.fromtimestamp(fr):%H:%M})" if fr else f"5h {f:.0%}")
    if w is not None and wr:
        gone = (time.time() - (wr - WEEK_S)) / WEEK_S
        parts.append(f"week {w:.0%} vs schedule {gone:.0%} (resets {dt.datetime.fromtimestamp(wr):%m-%d %H:%M})")
    return "account " + ", ".join(parts)


def pace_gate():
    """-> (ok, wake_ts, why, behind). Even weekly schedule: allowed = gone + burst*(1-gone)."""
    u = probe_usage()
    if not u:
        return True, None, "usage unknown -- running, limits will be slept through", False
    t = time.time()
    (f, fr), (w, wr) = u["five"], u["week"]
    waits, behind = [], False
    if f is not None and fr and fr > t and f >= CFG["five_max"]:
        waits.append((fr + 60, f"5h window {f:.0%} >= {CFG['five_max']:.0%}"))
    if w is not None and wr and wr > t:
        start = wr - WEEK_S
        gone = (t - start) / WEEK_S
        allowed = min(CFG["week_max"], gone + CFG["week_burst"] * (1 - gone))
        behind = (gone - w) >= 0.03 and (f is None or f < 0.5)
        if w >= CFG["week_max"]:
            waits.append((wr + 60, f"weekly allowance spent ({w:.0%})"))
        elif w > allowed:
            b = CFG["week_burst"]
            wake = start + (w - b) / max(1e-6, 1 - b) * WEEK_S
            waits.append((min(wr + 60, max(t + 120, wake)), f"week {w:.0%} ahead of the even schedule {allowed:.0%}"))
    if not waits:
        return True, None, usage_text(u), behind
    wake, why = max(waits)
    return False, wake, why + " | " + usage_text(u), False


# ------------------------------------------------------------------ jobs (compute that needs no Claude)
def jobs_status():
    rc, out = sh([sys.executable, os.path.join(AUTO, "job.py"), "status"], timeout=120)
    return out.strip()


def running_jobs():
    n = 0
    for p in glob.glob(os.path.join(STATE_DIR, "jobs", "*.json")):
        j = load_json(p, {})
        if j.get("status") == "running" and not os.path.exists(p[:-5] + ".done"):
            # an --interactive job that never started (no desktop logon -> no .log yet) cannot finish; don't wait on it
            if j.get("interactive") and not os.path.exists(p[:-5] + ".log"):
                continue
            n += 1
    return n


def wait_for_jobs(max_min):
    t0 = time.time()
    start_n = running_jobs()
    if start_n == 0:
        return
    log(f"waiting for {start_n} running job(s), up to {max_min} min (no Claude usage while waiting)")
    while time.time() - t0 < max_min * 60:
        time.sleep(60)
        heartbeat("wait-jobs")
        if stop_requested():
            return
        n = running_jobs()
        if n < start_n:
            log(f"  a job finished ({start_n - n} of {start_n}) after {(time.time() - t0) / 60:.0f} min")
            return


# ------------------------------------------------------------------ one turn
def build_prompt(state):
    base = io.open(PROMPT_FILE, encoding="utf-8").read()
    dyn = [
        "\n\n---\n## 지금 상태 (러너가 매 턴 붙이는 부분)",
        f"- 시각: {now()} / 반복 {state.get('iter', 0) + 1} / 마감 2026-10-28",
        f"- 사용량: {usage_text(USAGE['last'])}",
        "- 실행 중/최근 잡 (python automation/job.py status):\n```\n" + (jobs_status() or "(없음)") + "\n```",
        "- 최근 러너 로그:\n```\n" + tail(LOG, 15) + "```",
    ]
    hist = state.get("history", [])[-3:]
    if hist:
        dyn.append("- 직전 턴 보고:\n```json\n" + json.dumps(hist, ensure_ascii=False, indent=1)[:4000] + "\n```")
    return base + "\n".join(dyn)


def extract_report(text):
    for block in reversed(re.findall(r"```json\s*(\{.*?\})\s*```", text or "", flags=re.S)):
        try:
            d = json.loads(block)
            if isinstance(d, dict) and "summary" in d:
                return d
        except Exception:
            continue
    return None


def run_turn(state, dry_run=False, effort=""):
    os.makedirs(PROMPTS, exist_ok=True)
    os.makedirs(REPORTS, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    prompt = build_prompt(state)
    io.open(os.path.join(PROMPTS, f"{stamp}.md"), "w", encoding="utf-8").write(prompt)
    if dry_run:
        log(f"dry-run: prompt written ({len(prompt)} chars)")
        return None
    if not ensure_cli():
        raise RuntimeError("no Claude Code CLI found for 30 min")
    session = state.get("resume_session")
    this_prompt = prompt if not session else (
        "이전 턴이 사용 한도로 끊겼다가 지금 이어진다. 파일에 이미 한 작업은 남아 있다(다시 하지 말 것). "
        "남은 작업을 마치고 마지막에 JSON 보고 블록을 써라.")
    for model in CFG["models"]:
        cmd = [CLAUDE_BIN[0], "-p", "--model", model, "--output-format", "stream-json", "--verbose"]
        if CFG["skip_permissions"]:
            cmd += ["--dangerously-skip-permissions"]
        else:
            cmd += ["--permission-mode", "acceptEdits", "--allowedTools", ALLOWED_TOOLS,
                    "--disallowedTools", DISALLOWED_TOOLS]
        if effort:
            cmd += ["--effort", effort]
        if session:
            cmd += ["--resume", session]
        log(f"turn {state.get('iter', 0) + 1} with {model}" + (f" effort={effort}" if effort else "")
            + (f" [resume {session[:8]}]" if session else ""))
        heartbeat("turn")
        try:
            p = subprocess.run(cmd, cwd=ROOT, input=this_prompt, timeout=CFG["turn_timeout_s"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8",
                               errors="replace", env=dict(os.environ, PYTHONIOENCODING="utf-8"))
            raw, err, rc = p.stdout or "", p.stderr or "", p.returncode
        except subprocess.TimeoutExpired as e:
            raw = e.stdout.decode("utf-8", "replace") if isinstance(e.stdout, bytes) else (e.stdout or "")
            err, rc = "timeout", -1
        io.open(os.path.join(REPORTS, f"{stamp}_{model}.jsonl"), "w", encoding="utf-8").write(raw + "\n--- stderr ---\n" + err)
        text, meta, rl = parse_stream(raw)
        if rl:
            USAGE["last"], USAGE["ts"] = rl, time.time()
        log(f"  rc {rc}, calls {meta['num_turns']}, out {meta['out_tokens'] / 1e3:.1f} k, cache-read "
            f"{meta['cache_read'] / 1e6:.2f} M, ${meta['cost']:.2f} (list price)")
        state["spent_usd"] = float(state.get("spent_usd", 0.0)) + meta["cost"]
        if is_limit(meta):
            t = parse_reset_time(meta["result"]) or (dt.datetime.now() + dt.timedelta(minutes=45))
            state["resume_session"] = meta["session_id"] if meta["num_turns"] > 1 else session
            save_json(STATE, state)
            sleep_checked(max(60, (t - dt.datetime.now()).total_seconds() + 120), f"limit ({meta['result'][:80]})")
            return "limit"
        if effort and rc != 0 and "effort" in (err + meta["result"]).lower():
            log(f"  the CLI rejected --effort {effort}; dropping it for this run")
            CFG["boost_effort"] = CFG["effort"] = ""
            return "retry"
        if rc != 0 or meta["is_error"] or not text:
            log(f"  turn failed: {(meta['result'] or err)[-300:]}")
            if meta["session_id"] and meta["num_turns"] > 2:
                state["resume_session"] = meta["session_id"]
            return "fail"
        state.pop("resume_session", None)
        rep = extract_report(text) or dict(summary=text[-600:], parse="no json block")
        rep["ts"] = now()
        rep["cost_usd"] = round(meta["cost"], 2)
        state.setdefault("history", []).append(rep)
        state["history"] = state["history"][-40:]
        return rep
    return "fail"


def commit(rep):
    rc, out = sh(["git", "status", "--porcelain", "--"] + COMMIT_PATHS)
    if not out.strip():
        return
    sh(["git", "add", "-A", "--"] + COMMIT_PATHS)
    summ = (rep or {}).get("summary", "loop iteration") if isinstance(rep, dict) else "loop iteration"
    msg = f"thesis-loop: {summ[:200]}\n\nautomation/loop_runner.py iteration\n\nCo-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
    rc, out = sh(["git", "commit", "-m", msg, "--"] + COMMIT_PATHS)
    log(f"  commit: {out.strip().splitlines()[0] if out.strip() else rc}")


# ------------------------------------------------------------------ main
def status():
    print("heartbeat:", tail(HEARTBEAT, 1).strip() or "(none)")
    if os.path.exists(HEARTBEAT):
        age = time.time() - os.path.getmtime(HEARTBEAT)
        print(f"  age {age / 60:.1f} min ->", "ALIVE" if age < 180 else "NOT RUNNING (stale)")
    print("STOP file:", "present" if stop_requested() else "absent")
    s = load_json(STATE, {})
    print(f"iterations {s.get('iter', 0)}, list-price spend ${s.get('spent_usd', 0):.2f}")
    for h in s.get("history", [])[-5:]:
        print(" -", h.get("ts"), "|", str(h.get("summary"))[:160])
    print("\njobs:\n" + jobs_status())
    print("\nlast log lines:\n" + tail(LOG, 12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status", action="store_true")
    ap.add_argument("--stop", nargs="?", const="stop requested")
    ap.add_argument("--clear-stop", action="store_true")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--skip-permissions", action="store_true")
    ap.add_argument("--model", action="append")
    ap.add_argument("--effort", default="")
    ap.add_argument("--boost-effort", default="")
    ap.add_argument("--usage", action="store_true", help="print the account usage and exit")
    a = ap.parse_args()
    os.makedirs(STATE_DIR, exist_ok=True)
    if a.status:
        return status()
    if a.stop:
        io.open(STOPFILE, "w", encoding="utf-8").write(f"{now()} {a.stop}\n")
        print("STOP written; the runner ends at its next check.")
        return
    if a.usage:
        ensure_cli(60)
        print(usage_text(probe_usage(force=True)))
        return
    if a.clear_stop and os.path.exists(STOPFILE):
        os.remove(STOPFILE)
    if a.model:
        CFG["models"] = a.model
    CFG["skip_permissions"] = a.skip_permissions
    CFG["effort"], CFG["boost_effort"] = a.effort, a.boost_effort
    if stop_requested():
        print("STOP file present -- remove it with --clear-stop to start.")
        return
    state = load_json(STATE, {})
    log(f"=== runner start pid {os.getpid()} (models {CFG['models']}) ===")
    fails = 0
    while True:
        heartbeat("loop")
        if stop_requested():
            log("STOP file found -- exiting cleanly")
            break
        ok, wake, why, behind = pace_gate() if not a.dry_run else (True, None, "dry-run", False)
        if not ok:
            sleep_checked(max(120, wake - time.time()), f"pacing: {why}")
            continue
        log(f"pacing ok: {why}")
        effort = CFG["boost_effort"] if (behind and CFG["boost_effort"]) else CFG["effort"]
        try:
            rep = run_turn(state, a.dry_run, effort)
        except Exception as e:
            log(f"turn crashed: {type(e).__name__}: {e}")
            rep = "fail"
        if a.dry_run:
            break
        if rep in ("limit", "retry"):
            continue
        if rep == "fail":
            fails += 1
            save_json(STATE, state)
            if fails >= CFG["max_fail"]:
                log(f"{fails} failed turns in a row -- pausing 60 min")
                sleep_checked(3600, "repeated failures")
                fails = 0
            else:
                sleep_checked(300 * fails, "after a failed turn")
            continue
        fails = 0
        state["iter"] = int(state.get("iter", 0)) + 1
        save_json(STATE, state)
        try:
            commit(rep)
        except Exception as e:
            log(f"  commit failed: {e}")
        log(f"turn done: {str(rep.get('summary'))[:200]}")
        if rep.get("goal_met"):
            log("agent reports every task done -- exiting")
            break
        if a.once:
            break
        if rep.get("wait_for_jobs") and running_jobs():
            wait_for_jobs(min(CFG["job_wait_max_min"], int(rep.get("wait_minutes") or CFG["job_wait_max_min"])))
        else:
            sleep_checked(CFG["min_gap_s"], "gap between turns")
    log("=== runner exit ===")


if __name__ == "__main__":
    main()
