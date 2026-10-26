"""
slx_trace_inputs.py -- where do the controller's inputs come from in the dataset-era model? (R2(3))

Parses simulink/systems/system_root.xml of dataset_era/Control_Logic_dataset_era.slx (read-only), and for every input
port of the DAB_Controller_System block follows the line back through Goto/From tags to its source block (Constant
value, From Workspace variable, ...). Commented blocks are flagged.

    python -I -X utf8 Simulink/Thesis/slx_trace_inputs.py [--sid 706]
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
SLX = os.path.join(HERE, "dataset_era", "Control_Logic_dataset_era.slx")
PORTS = ["V_HV", "I_HV", "V_LV", "I_LV", "I_L_pri", "Power_cmd_in", "PWM_En_in", "Kp_ext", "Ki_ext", "Mode_Select",
         "Phi_Manual", "Ctrl_Mode", "DeadTime_ns", "D1_ext_in", "D2_ext_in"]


def load(path=SLX):
    z = zipfile.ZipFile(path)
    root = ET.fromstring(z.read("simulink/systems/system_root.xml"))
    blocks = {}
    for b in root.iter("Block"):
        p = {e.get("Name"): (e.text or "") for e in b.findall("P")}
        blocks[b.get("SID")] = dict(type=b.get("BlockType"), name=(b.get("Name") or "").replace("\n", " "), p=p)
    edges = []                                                    # (src "SID#out:k", dst "SID#in:k")

    def walk(line, src):
        s = src
        for e in line.findall("P"):
            if e.get("Name") == "Src":
                s = e.text
        for e in line.findall("P"):
            if e.get("Name") == "Dst":
                edges.append((s, e.text))
        for br in line.findall("Branch"):
            walk(br, s)
    for ln in root.iter("Line"):
        if ln.find("P[@Name='Src']") is not None:
            walk(ln, None)
    return blocks, edges


def describe(sid, blocks):
    b = blocks.get(sid, {})
    p = b.get("p", {})
    extra = ", ".join(f"{k}={p[k]}" for k in ("Value", "VariableName", "GotoTag", "Gain", "Commented") if k in p)
    return f"{b.get('type')} '{b.get('name')}' (SID {sid}){' [' + extra + ']' if extra else ''}"


def trace(dst, blocks, edges, depth=0, seen=None):
    seen = seen or set()
    srcs = [s for s, d in edges if d == dst]
    out = []
    for s in srcs:
        sid = s.split("#")[0]
        if sid in seen:
            continue
        seen.add(sid)
        b = blocks.get(sid, {})
        out.append("  " * depth + "<- " + describe(sid, blocks))
        if b.get("type") == "From":
            tag = b["p"].get("GotoTag")
            gotos = [k for k, v in blocks.items() if v["type"] == "Goto" and v["p"].get("GotoTag") == tag]
            for g in gotos:
                out.append("  " * (depth + 1) + "== " + describe(g, blocks))
                out += trace(f"{g}#in:1", blocks, edges, depth + 2, seen)
        elif b.get("type") in ("DataTypeConversion", "Gain", "Saturate", "Switch", "Product", "Sum", "SignalConversion",
                               "Reference", "Memory", "UnitDelay", "ZeroOrderHold", "RateTransition"):
            for k in range(1, 4):
                out += trace(f"{sid}#in:{k}", blocks, edges, depth + 1, seen)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sid", default="706")
    a = ap.parse_args()
    blocks, edges = load()
    print("controller block:", describe(a.sid, blocks))
    for k, name in enumerate(PORTS, start=1):
        print(f"in:{k} {name}")
        for line in trace(f"{a.sid}#in:{k}", blocks, edges, 1):
            print(line)


if __name__ == "__main__":
    sys.exit(main())
