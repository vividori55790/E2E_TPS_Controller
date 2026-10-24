"""
slx_dump.py -- read-only dump of the blocks of a .slx (zip of XML) without MATLAB: every block's path, type,
library source and parameters. Used to extract the power-stage parameters of the Simulink plant (T6 fallback
while MATLAB cannot run: online licensing stalls in Session 0).

  python -I -X utf8 Simulink/Thesis/slx_dump.py Simulink/Modules/Control_Logic.slx [--grep REGEX] > out/x.txt
"""
import argparse
import re
import sys
import xml.etree.ElementTree as ET
import zipfile


def load_systems(path):
    z = zipfile.ZipFile(path)
    systems = {}
    for n in z.namelist():
        m = re.match(r"simulink/systems/system_(\w+)\.xml$", n)
        if m:
            systems[m.group(1)] = ET.fromstring(z.read(n))
    return systems


def walk(systems, sid="root", prefix=""):
    root = systems.get(sid)
    if root is None:
        return
    for b in root.findall("Block"):
        name = (b.get("Name") or "").replace("\n", " ")
        params = {}
        for p in b.findall("P"):
            params[p.get("Name")] = (p.text or "").strip()
        for ip in b.findall("InstanceData/P"):
            params["inst." + ip.get("Name")] = (ip.text or "").strip()
        path = f"{prefix}/{name}"
        yield path, b.get("BlockType"), params
        sub = b.find("System")
        if sub is not None and sub.get("Ref"):
            yield from walk(systems, sub.get("Ref").replace("system_", ""), path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slx")
    ap.add_argument("--grep", default="")
    ap.add_argument("--skip", default="Position|ZOrder|Ports|BackgroundColor|ForegroundColor|ShowName|FontSize|"
                                       "BlockMirror|BlockRotation|NamePlacement|Orientation|LibraryVersion|SID|"
                                       "ContentPreviewEnabled|IconDisplay|HideAutomaticName|RequestExecContextInheritance")
    a = ap.parse_args()
    systems = load_systems(a.slx)
    rx = re.compile(a.grep, re.I) if a.grep else None
    skip = re.compile(f"^(inst\\.)?({a.skip})$")
    for path, btype, params in walk(systems):
        line = f"{path}  [{btype}]"
        kv = {k: v for k, v in params.items() if not skip.match(k)}
        text = line + " " + " ".join(f"{k}={v}" for k, v in kv.items())
        if rx and not rx.search(text):
            continue
        print(line)
        for k, v in kv.items():
            print(f"    {k} = {v[:200]}")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
