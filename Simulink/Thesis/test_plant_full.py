"""
test_plant_full.py -- checks of plant_full.FullPlant
  1. ideal settings (no quantisation, td 0, Ron 0, Vf 0, no droop) == tps_model on random (D0, D1, D2, V2)
  2. same with the series caps: steady state has zero mean current and the same Ipk (caps only absorb DC)
  3. lossy settings: P_HV (out of the HV bus) >= P_LV (into the LV bus) for HV -> LV, i.e. losses >= 0
  4. quantised gating: on-times are 62 or 63 samples
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plant_full as pf  # noqa: E402
import tps_model as tm  # noqa: E402


def check(name, cond, msg):
    print(f"[{'PASS' if cond else 'FAIL'}] {name}: {msg}")
    return bool(cond)


def main():
    rng = np.random.default_rng(11)
    N = 300
    D0, D1, D2 = rng.uniform(-70, 70, N), rng.uniform(0.15, 0.5, N), rng.uniform(0.15, 0.5, N)
    V2 = rng.uniform(84, 109.2, N)
    ideal = tm.evaluate(D0, D1, D2, V2)
    ok = True
    p = pf.FullPlant(Ron=0.0, td=0.0, quant=False, diode=False, Vf_const=0.0, droop=False, C3=1.0, C4=1.0)
    r = p.steady(D0, D1, D2, V2)
    p_c = pf.FullPlant(Ron=0.0, td=0.0, quant=False, diode=False, Vf_const=0.0, droop=False)
    rc = p_c.steady(D0, D1, D2, V2)
    print(f"  (info) real 1000 uF series caps alone: Ipk change vs ideal mean {np.mean(rc['Ipk'] - ideal['Ipk']) * 1e3:+.2f}"
          f" mA, max |.| {np.abs(rc['Ipk'] - ideal['Ipk']).max() * 1e3:.2f} mA")
    eI = np.abs(r["Ipk"] - ideal["Ipk"]).max()
    eP = np.abs(r["P_LV"] - ideal["P"]).max()
    mean_i = np.abs(r["wave"][:-1].mean(axis=0)).max()
    ok &= check("ideal FullPlant = tps_model", eI < 5e-3 and eP < 0.5,
                f"max |dIpk| {eI * 1e3:.2f} mA, max |dP| {eP:.3f} W, max |mean i| {mean_i:.1e} A, "
                f"residual {r['resid'].max():.1e} A")
    eH = np.abs(r["P_HV"] + r["P_LV"]).max()
    ok &= check("lossless power balance", eH < 0.5, f"max |P_HV_out - P_LV_in| {eH:.3f} W")
    q = pf.FullPlant()
    r2 = q.steady(np.abs(D0), D1, D2, V2)                      # D0 > 0: HV -> LV
    loss = r2["P_HV"] + r2["P_LV"]                              # out of HV bus minus into LV bus (P_LV < 0 here)
    ok &= check("losses >= 0 (Ron 40 mOhm, td 100 ns, diode)", loss.min() > -1e-3,
                f"loss min {loss.min():.2f} W, median {np.median(loss):.2f} W, residual {r2['resid'].max():.1e} A")
    up, lo = q._frac_leg(np.array([0.0, 37.3, 200.1]))
    samples = up.sum(axis=0) * q.dth / q.g + q.td_deg / q.g     # on-time incl. the dead time, in samples
    ok &= check("quantised on-time 62/63 samples", np.all(np.isin(np.round(samples, 6), [62.0, 63.0])),
                f"{np.round(samples, 3)}")
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
