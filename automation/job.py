"""
automation/job.py -- background compute jobs that run WITHOUT Claude (the loop overlaps them with agent turns).

  python automation/job.py submit NAME [--interactive] [--cwd DIR] -- CMD [ARGS...]
  python automation/job.py status
  python automation/job.py tail NAME [-n 40]
  python automation/job.py kill NAME
  python automation/job.py wait NAME [NAME ...]     (one line per job as it ends; returns when all have)

Each job gets automation/state/jobs/NAME.{json,cmd,log,done}. NAME.done holds the exit code when it ends.

--interactive runs the job in the logged-on user's desktop session through a one-shot scheduled task.
Use it for MATLAB / Simulink: measured 2026-10-21, `matlab -batch` started from Session 0 (ssh, or an S4U task
such as this loop) stalls after ~5-9 s of CPU and never finishes, presumably waiting on a licence / sign-in
step it cannot show. Plain Python jobs run fine detached under the loop.
"""
import argparse
import datetime as dt
import io
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JOBS = os.path.join(ROOT, "automation", "state", "jobs")


def _p(name, ext):
    return os.path.join(JOBS, f"{name}.{ext}")


def _load(name):
    try:
        return json.load(io.open(_p(name, "json"), encoding="utf-8"))
    except Exception:
        return {}


def _save(name, d):
    json.dump(d, io.open(_p(name, "json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)


def _quote(a):
    return '"' + a.replace('"', '\\"') + '"' if (" " in a or "\t" in a or '"' in a) else a


def submit(name, cmd, cwd, interactive):
    os.makedirs(JOBS, exist_ok=True)
    old = _refresh(name)
    if old.get("status") == "running" and not os.path.exists(_p(name, "done")):
        print(f"REFUSED: job {name} is still running (pid {old.get('pid')})")
        return 2
    for ext in ("log", "done"):
        if os.path.exists(_p(name, ext)):
            os.remove(_p(name, ext))
    cwd = os.path.abspath(cwd or ROOT)
    line = " ".join(_quote(c) for c in cmd)
    body = ["@echo off", f'cd /d "{cwd}"',
            f'echo ==== job {name} start %DATE% %TIME% >> "{_p(name, "log")}"',
            f'{line} >> "{_p(name, "log")}" 2>&1',
            f'echo %ERRORLEVEL% > "{_p(name, "done")}"',
            f'echo ==== job {name} exit %ERRORLEVEL% %DATE% %TIME% >> "{_p(name, "log")}"']
    task = f"DABJob-{name}"
    meta = dict(name=name, cmd=line, argv=list(cmd), cwd=cwd, interactive=interactive, status="running",
                start=dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), start_ts=time.time())
    if interactive:
        body.append(f'schtasks /Delete /TN "{task}" /F >nul 2>&1')
        io.open(_p(name, "cmd"), "w", encoding="ascii", errors="replace").write("\r\n".join(body) + "\r\n")
        # wscript runs the .cmd hidden (window style 0) and waits: a console on the user's desktop can be closed
        # by accident (2026-10-24 18:35: t6_verify started at logon and ended in ^C)
        io.open(_p(name, "vbs"), "w", encoding="ascii", errors="replace").write(
            'CreateObject("WScript.Shell").Run "cmd.exe /c ""' + _p(name, "cmd") + '""", 0, True\r\n')
    if interactive:
        ps = (
            "$me=[Security.Principal.WindowsIdentity]::GetCurrent().Name;"
            f"$a=New-ScheduledTaskAction -Execute 'wscript.exe' -Argument '//B //Nologo \"{_p(name, 'vbs')}\"';"
            "$pr=New-ScheduledTaskPrincipal -UserId $me -LogonType Interactive -RunLevel Limited;"
            "$s=New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries "
            "-ExecutionTimeLimit ([TimeSpan]::Zero) -MultipleInstances IgnoreNew;"
            # nobody may be logged on (then Start-ScheduledTask silently does nothing): also fire at next logon
            "$t=New-ScheduledTaskTrigger -AtLogOn -User $me;"
            f"Register-ScheduledTask -TaskName '{task}' -Action $a -Principal $pr -Settings $s -Trigger $t -Force"
            " | Out-Null;"
            f"Start-ScheduledTask -TaskName '{task}'; 'started'")
        r = subprocess.run(["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", ps],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                           errors="replace", timeout=120)
        if "started" not in (r.stdout or ""):
            print("FAILED to start the interactive task:\n" + (r.stdout or ""))
            return 3
        meta["task"] = task
        meta["pid"] = None
    else:
        # A detached supervisor (job.py _run NAME) starts the command with CREATE_NO_WINDOW and the log as its
        # stdout/stderr, waits, and writes NAME.done. The earlier "cmd.exe /c NAME.cmd" with ">> log" lost all
        # output of external programs (found 2026-10-21): a console program started by a console-less cmd.exe
        # gets a fresh hidden console and writes there, so only cmd's own echo lines reached the log.
        _save(name, meta)
        flags = 0x00000008 | 0x00000200 | 0x08000000   # DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP | NO_WINDOW
        pr = subprocess.Popen([sys.executable, os.path.abspath(__file__), "_run", name], cwd=cwd, creationflags=flags,
                              stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              close_fds=True)
        meta["pid"] = pr.pid
    _save(name, meta)
    print(f"submitted {name} ({'interactive session' if interactive else 'detached'}): {line}")
    print(f"  log: {_p(name, 'log')}")
    return 0


def _run(name):
    """Supervisor of a detached job (see submit)."""
    import shutil
    d = _load(name)
    argv = list(d["argv"])
    argv[0] = shutil.which(argv[0]) or argv[0]
    stamp = lambda: dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-4]   # noqa: E731
    with open(_p(name, "log"), "ab", buffering=0) as logf:
        logf.write(f"==== job {name} start {stamp()}\n".encode())
        try:
            rc = subprocess.call(argv, cwd=d["cwd"], stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT,
                                 creationflags=0x08000000)                     # CREATE_NO_WINDOW
        except OSError as e:
            logf.write(f"cannot start {argv[0]}: {e}\n".encode())
            rc = 9009
        logf.write(f"==== job {name} exit {rc} {stamp()}\n".encode())
    io.open(_p(name, "done"), "w", encoding="ascii").write(f"{rc}\n")
    return 0


def _boot_ts():
    try:
        import ctypes
        ctypes.windll.kernel32.GetTickCount64.restype = ctypes.c_ulonglong
        return time.time() - ctypes.windll.kernel32.GetTickCount64() / 1000.0
    except Exception:
        return 0.0


def _pid_alive(pid):
    try:
        import ctypes
        k = ctypes.windll.kernel32
        h = k.OpenProcess(0x1000, False, int(pid))            # PROCESS_QUERY_LIMITED_INFORMATION
        if not h:
            return False
        code = ctypes.c_ulong()
        ok = k.GetExitCodeProcess(h, ctypes.byref(code))
        k.CloseHandle(h)
        return bool(ok) and code.value == 259                  # STILL_ACTIVE
    except Exception:
        return True                                            # cannot tell -> do not declare it dead


def _refresh(name):
    d = _load(name)
    if not d:
        return d
    if d.get("status") == "running" and not os.path.exists(_p(name, "done")):
        # a job started before the last boot, or whose detached supervisor is gone, will never write .done
        # (2026-10-24: the PC shut down mid-run; such jobs would otherwise count as running forever)
        dead = float(d.get("start_ts", 0)) < _boot_ts() - 5
        if not dead and d.get("pid") and not d.get("interactive"):
            dead = not _pid_alive(d["pid"]) and not os.path.exists(_p(name, "done"))
        if dead:
            io.open(_p(name, "done"), "w", encoding="ascii").write("-2\n")
            d["status"] = "died"
            d["end"] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            _save(name, d)
            return d
    if d.get("status") == "running" and os.path.exists(_p(name, "done")):
        try:
            rc = int(io.open(_p(name, "done"), encoding="ascii", errors="ignore").read().strip() or "0")
        except ValueError:
            rc = -1
        d["status"] = "done" if rc == 0 else f"failed(rc={rc})"
        d["end"] = dt.datetime.fromtimestamp(os.path.getmtime(_p(name, "done"))).strftime("%Y-%m-%d %H:%M:%S")
        _save(name, d)
    return d


def _read_text(path):
    """job log as text: UTF-8 if it decodes, else cp949 (MATLAB's Korean messages on this PC), never raising"""
    b = io.open(path, "rb").read()
    try:
        return b.decode("utf-8")
    except UnicodeDecodeError:
        return b.decode("cp949", errors="replace")


def status():
    if not os.path.isdir(JOBS):
        print("(no jobs)")
        return 0
    names = sorted({f.rsplit(".", 1)[0] for f in os.listdir(JOBS) if f.endswith(".json")},
                   key=lambda n: _load(n).get("start_ts", 0))
    if not names:
        print("(no jobs)")
    for n in names:
        _refresh(n)
    for n in names[-15:]:
        d = _load(n)
        el = ""
        if d.get("status") == "running":
            el = f"{(time.time() - d.get('start_ts', time.time())) / 60:.0f} min"
            if os.path.exists(_p(n, "log")):
                el += f", log idle {(time.time() - os.path.getmtime(_p(n, 'log'))) / 60:.0f} min"
        last = ""
        if os.path.exists(_p(n, "log")):
            ls = [x.rstrip() for x in _read_text(_p(n, "log")).splitlines() if x.strip()]
            last = ls[-1][:110] if ls else ""
        print(f"{n:28s} {d.get('status', '?'):14s} {d.get('start', ''):19s} {el:24s} | {last}")
    return 0


def tail(name, n):
    if not os.path.exists(_p(name, "log")):
        print("(no log yet)")
        return 1
    print("\n".join(_read_text(_p(name, "log")).splitlines()[-n:]))
    return 0


def kill(name):
    d = _load(name)
    if d.get("task"):
        subprocess.run(["schtasks", "/End", "/TN", d["task"]], capture_output=True)
        subprocess.run(["schtasks", "/Delete", "/TN", d["task"], "/F"], capture_output=True)
    if d.get("pid"):
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(d["pid"])], capture_output=True)
    io.open(_p(name, "done"), "w", encoding="ascii").write("-9\n")
    _refresh(name)
    print(f"killed {name}")
    return 0


def wait(names, poll=15.0):
    """Print one line per job as it ends (in completion order); return when all have ended."""
    left = list(names)
    while left:
        for n in list(left):
            d = _refresh(n)
            if d.get("status") != "running":
                print(f"{n} {d.get('status', '?')} {d.get('end', '')}", flush=True)
                left.remove(n)
        if left:
            time.sleep(poll)
    return 0


def diag(name):
    """Scheduled-task state of an --interactive job, logged-on sessions, and MATLAB processes."""
    d = _load(name)
    if d.get("task"):
        r = subprocess.run(["schtasks", "/Query", "/TN", d["task"], "/V", "/FO", "LIST"], capture_output=True,
                           text=True, encoding="mbcs", errors="replace")
        keep = ("TaskName", "Status", "Last Run Time", "Last Result", "Logon Mode", "Run As User",
                "작업 이름", "상태", "마지막 실행 시간", "마지막 결과", "로그온 모드", "다음 사용자로 실행")
        lines = [x for x in (r.stdout or r.stderr).splitlines() if x.strip().startswith(keep)]
        print("\n".join(lines) or (r.stdout or r.stderr)[:800])
    # Windows Home has no query.exe; an explorer.exe in a session > 0 means someone is logged on there
    for cmd in (["tasklist", "/FI", "IMAGENAME eq explorer.exe", "/FO", "CSV"],
                ["tasklist", "/FI", "IMAGENAME eq MATLAB*", "/FO", "CSV"]):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, encoding="mbcs", errors="replace")
            print("$ " + " ".join(cmd) + "\n" + (r.stdout or r.stderr).strip())
        except OSError as e:
            print("$ " + " ".join(cmd) + f"\n  ({e})")
    return 0


def main():
    for st in (sys.stdout, sys.stderr):        # a cp949 console must not crash status/tail on a stray character;
        try:                                   # piped (loop_runner.sh reads UTF-8) -> write UTF-8
            st.reconfigure(errors="replace", **({} if st.isatty() else {"encoding": "utf-8"}))
        except (AttributeError, ValueError):
            pass
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("submit")
    s.add_argument("name")
    s.add_argument("--interactive", action="store_true")
    s.add_argument("--cwd", default=None)
    s.add_argument("rest", nargs=argparse.REMAINDER)
    sub.add_parser("status")
    t = sub.add_parser("tail")
    t.add_argument("name")
    t.add_argument("-n", type=int, default=40)
    k = sub.add_parser("kill")
    k.add_argument("name")
    g = sub.add_parser("diag")
    g.add_argument("name")
    r = sub.add_parser("_run")
    r.add_argument("name")
    wt = sub.add_parser("wait")
    wt.add_argument("names", nargs="+")
    a = ap.parse_args()
    if a.cmd == "wait":
        return wait(a.names)
    if a.cmd == "_run":
        return _run(a.name)
    if a.cmd == "diag":
        return diag(a.name)
    if a.cmd == "submit":
        rest = list(a.rest)
        # argparse.REMAINDER after NAME swallows options given as "NAME --interactive -- CMD"; peel them off
        while rest and rest[0] != "--" and rest[0].startswith("--"):
            opt = rest.pop(0)
            if opt == "--interactive":
                a.interactive = True
            elif opt == "--cwd" and rest:
                a.cwd = rest.pop(0)
            elif opt.startswith("--cwd="):
                a.cwd = opt.split("=", 1)[1]
            else:
                print(f"unknown option before '--': {opt}")
                return 1
        rest = rest[1:] if rest and rest[0] == "--" else rest
        if not rest:
            print("usage: job.py submit NAME [--interactive] -- CMD ...")
            return 1
        return submit(a.name, rest, a.cwd, a.interactive)
    if a.cmd == "status":
        return status()
    if a.cmd == "tail":
        return tail(a.name, a.n)
    if a.cmd == "kill":
        return kill(a.name)


if __name__ == "__main__":
    sys.exit(main() or 0)
