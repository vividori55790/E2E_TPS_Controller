"""
make_dataset_era.py -- extract the model version that produced Data/TPS_Dataset_20260316_021229.csv (git 35d5a78)
into Simulink/Thesis/dataset_era/ for the T6 MATLAB check (read-only use of history; nothing in Modules/ changes).

Why: HEAD's Modules/Control_Logic.slx ("Debuging Phase Shift") has every From Workspace input commented out and
replaced by constants (Phi 70 deg, D1 = D2 = 0.5, V2 96 V), and DAB_Controller_System.m changed by +333/-242 lines
since the dataset was generated (out/plant_params.md section 0). The copy is renamed Control_Logic_dataset_era.slx;
the old DAB_Controller_System.m sits next to it and shadows the HEAD version once dataset_era/ is first on the path.
The files are binary/large and git-ignored (dataset_era/ in .gitignore); re-run this script to recreate them.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
REV = "35d5a78"
FILES = [("Simulink/Modules/Control_Logic.slx", "Control_Logic_dataset_era.slx"),
         ("Simulink/DAB_Controller_System.m", "DAB_Controller_System.m"),
         ("Simulink/Direct_Efficiency_Analyzer.m", "Direct_Efficiency_Analyzer.m")]


def main():
    dst = os.path.join(HERE, "dataset_era")
    os.makedirs(dst, exist_ok=True)
    for src, name in FILES:
        blob = subprocess.run(["git", "show", f"{REV}:{src}"], cwd=ROOT, capture_output=True, check=True).stdout
        with open(os.path.join(dst, name), "wb") as f:
            f.write(blob)
        print(f"{REV}:{src} -> dataset_era/{name} ({len(blob)} B)")


if __name__ == "__main__":
    sys.exit(main())
