"""MATLAB 정지 진단(읽기 전용): MATLAB/MathWorks 프로세스의 CPU 누적 시간·부모, 산출물 상태.

Session 0에서 Get-Process/tasklist가 승인 대상이라 ctypes(Toolhelp 스냅샷 + GetProcessTimes)로 본다.
사용: python -I -X utf8 Simulink/Thesis/matlab_stall_probe.py
"""
import ctypes
import ctypes.wintypes as wt
import csv
import os
import time
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')

k32 = ctypes.WinDLL('kernel32', use_last_error=True)
TH32CS_SNAPPROCESS = 0x2
PROCESS_QUERY_LIMITED_INFORMATION = 0x1000


class PROCESSENTRY32W(ctypes.Structure):
    _fields_ = [('dwSize', wt.DWORD), ('cntUsage', wt.DWORD), ('th32ProcessID', wt.DWORD),
                ('th32DefaultHeapID', ctypes.c_void_p), ('th32ModuleID', wt.DWORD),
                ('cntThreads', wt.DWORD), ('th32ParentProcessID', wt.DWORD),
                ('pcPriClassBase', ctypes.c_long), ('dwFlags', wt.DWORD),
                ('szExeFile', ctypes.c_wchar * 260)]


def processes():
    snap = k32.CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0)
    e = PROCESSENTRY32W()
    e.dwSize = ctypes.sizeof(e)
    ok = k32.Process32FirstW(snap, ctypes.byref(e))
    while ok:
        yield e.th32ProcessID, e.th32ParentProcessID, e.szExeFile
        ok = k32.Process32NextW(snap, ctypes.byref(e))
    k32.CloseHandle(snap)


def times(pid):
    h = k32.OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, False, pid)
    if not h:
        return None
    c, x, kt, ut = (wt.FILETIME() for _ in range(4))
    ok = k32.GetProcessTimes(h, ctypes.byref(c), ctypes.byref(x), ctypes.byref(kt), ctypes.byref(ut))
    k32.CloseHandle(h)
    if not ok:
        return None
    ft = lambda f: (f.dwHighDateTime << 32 | f.dwLowDateTime)
    start = ft(c) / 1e7 - 11644473600
    return time.strftime('%m-%d %H:%M:%S', time.localtime(start)), (ft(kt) + ft(ut)) / 1e7


def main():
    procs = list(processes())
    names = {pid: n for pid, _, n in procs}
    keys = ('matlab', 'mathworks', 'simulink', 'explorer', 'winlogon')
    t0 = {}
    for pid, ppid, n in procs:
        if any(k in n.lower() for k in keys):
            t0[pid] = times(pid)
    time.sleep(5)
    print(f'{"pid":>7} {"ppid":>7} {"start":>15} {"cpu_s":>9} {"dcpu5s":>7}  name (parent)')
    cnt = Counter()
    for pid, ppid, n in procs:
        if pid not in t0:
            continue
        a, b = t0[pid], times(pid)
        cnt[n] += 1
        if n.lower() == 'mathworksservicehostwindow.exe':
            continue
        if a is None or b is None:
            print(f'{pid:>7} {ppid:>7} {"?":>15} {"?":>9} {"?":>7}  {n} ({names.get(ppid, "-")})')
        else:
            print(f'{pid:>7} {ppid:>7} {b[0]:>15} {b[1]:9.1f} {b[1] - a[1]:7.2f}  {n} ({names.get(ppid, "-")})')
    print('count:', dict(cnt))

    print('\n산출물:')
    for f in sorted(os.listdir(OUT)):
        if f.startswith(('staircase_check', 'stair_steady', 'dataset_fixed_part', 'R6_', 'r5r6')):
            p = os.path.join(OUT, f)
            print(f'  {f:45s} {os.path.getsize(p):>9d} B  {time.strftime("%m-%d %H:%M", time.localtime(os.path.getmtime(p)))}')
    p = os.path.join(OUT, 'staircase_check.csv')
    if os.path.exists(p):
        with open(p, encoding='utf-8') as fh:
            rows = list(csv.DictReader(fh))
        print('staircase_check.csv interp별 행 수:', dict(Counter(r.get('interp') for r in rows)))


if __name__ == '__main__':
    main()
