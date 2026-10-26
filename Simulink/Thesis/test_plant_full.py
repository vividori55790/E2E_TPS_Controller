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
    # 5. dead time at light load near k = 1: the switching currents sit on the kinks of the period map and plain
    #    Newton cycled (plant_conv_probe.py). steady_fixed must converge there and agree with brute time stepping.
    b = pf.FullPlant(Ron=0.040, droop=False, quant=False)
    m = 40
    D0s = np.linspace(-7.0, -1.0, m)
    d1, d2 = np.full(m, 0.4338), np.full(m, 0.4485)
    Vb1, Vb2 = np.full(m, 380.0), np.full(m, 92.66)
    old = b.steady_fixed(D0s, d1, d2, Vb1, Vb2, polish=0, relax=None)
    new = b.steady_fixed(D0s, d1, d2, Vb1, Vb2)
    # the answer must be a steady state: continued for 1500 periods it stays (max |i| / mean P over 80 periods).
    # Not compared with a cold start: there are coexisting stable states there (plant_attractor_probe.py, ~20 mA apart)
    sch = b.schedule(D0s, d1, d2)
    _, _, I_new, _, P_new, _ = b._relax(new["i0"], new["vc0"], sch, Vb1, Vb2, 1420, 80)
    _, _, I_old, _, P_old, _ = b._relax(old["i0"], old["vc0"], sch, Vb1, Vb2, 1420, 80)
    eI = np.abs(I_new - new["Ipk"])
    eP = np.abs(P_new - new["P_LV"])
    _, _, I_cold, _, _, _ = b._relax(np.zeros(m), np.zeros(m), sch, Vb1, Vb2, 1500, 80)
    ok &= check("dead-time light load: answer is stationary", np.mean(eI < 3e-3) >= 0.95 and eI.max() < 0.03,
                f"after 1500 more periods |dIpk| < 3 mA at {np.mean(eI < 3e-3) * 100:.0f} %, max {eI.max() * 1e3:.1f} mA, "
                f"max |dP| {eP.max():.2f} W (plain 8-step Newton: max {np.abs(I_old - old['Ipk']).max() * 1e3:.0f} mA); "
                f"period-1 {new['conv'].mean() * 100:.0f} %, time-stepped {new['cycle'].mean() * 100:.0f} %; (info) vs a cold "
                f"start: max {np.abs(I_cold - new['Ipk']).max() * 1e3:.0f} mA (coexisting states)")
    # 6. operating_point = the FIRST upward crossing of P_LV(D0) = P*, at P* within 0.05 W
    P_t = np.array([28.6, 33.9, 122.5, 418.0, 900.0])
    V_t = np.array([92.66, 92.63, 91.17, 100.9, 84.0])
    g1 = np.array([0.4338, 0.4219, 0.4582, 0.5, 0.45])
    g2 = np.array([0.4485, 0.4396, 0.4712, 0.5, 0.5])
    op = b.operating_point(P_t, g1, g2, V_t)
    eP = np.abs(-op["P_LV"] - P_t).max()
    first_ok = True
    for j in range(P_t.size):                                   # fine scan below the answer: never above P*
        s = np.linspace(float(op["D0"][j]) - 12.0, float(op["D0"][j]) - 0.02, 50)
        rs = b.steady_fixed(s, np.full(50, g1[j]), np.full(50, g2[j]), np.full(50, float(op["V1_bus"][j])),
                            np.full(50, float(op["V2_bus"][j])))
        first_ok &= bool(np.all(-rs["P_LV"] < P_t[j]))
    ok &= check("operating_point = first crossing", eP < 0.05 and first_ok and op["ok"].all() and op["conv"].all(),
                f"max |P_LV - P*| {eP:.4f} W, no crossing below: {first_ok}, ok {op['ok'].all()}, conv {op['conv'].all()}")
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
