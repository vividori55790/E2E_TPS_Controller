"""
plant_sim.py -- switching-level steady-state simulation of the DAB power stage with non-idealities, vectorised
over operating points. T6 fallback while MATLAB cannot run (online licensing stalls in Session 0): it answers
"how far is the ideal piecewise-linear model (tps_model.py) from a plant with dead time and losses?".

Circuit (primary-referred, same PWM convention as tps_model.py / DAB_Controller_System.m):
    leg A upper gate on [0 + td, 180), lower on [180 + td, 360)          (deg; td = dead time)
    leg B the same, shifted by 360*D1;  legs C, D: shifted by D0 and D0 + 360*D2 on the secondary
    L di/dt = v_A - v_B - n (v_C - v_D) - R i
A conducting switch puts its rail on the midpoint minus Ron * i_out (synchronous: Ron in both directions).
In the dead time the leg's out-of-midpoint current picks the body diode: i_out > 0 -> lower diode (v = -Vf),
i_out < 0 -> upper diode (v = V + Vf). Coss resonant transitions are neglected (the dead time is long compared
with them, or the leg is not ZVS -- then the diode/hard-switching picture is what Simscape's switch models give).
Out-of-midpoint currents: i_A = i, i_B = -i, i_C = -n i, i_D = +n i.
DC buses are stiff (V1, V2 = bridge DC voltages, as in the analytic model; battery droop only moves V2).

Steady state: gating is half-wave antisymmetric, so i(t + Ts/2) = -i(t); solve G(i0) = i(Ts/2; i0) + i0 = 0 by
safeguarded secant (G is increasing, slope ~ 2). Each step integrates exactly with v constant over the step
(RL exponential). PI steady state (D0 closes |P_LV| = P*): first crossing, as pi_steady.py, by bisection on D0
around the ideal-model D0.

    python -I -X utf8 Simulink/Thesis/plant_sim.py --selftest
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402
import tps_model as tm  # noqa: E402


class Plant:
    def __init__(self, V1=tm.V1_NOM, n=tm.N_TURNS, fs=tm.FS, L=tm.L_IND, R=0.0, td=0.0, Ron1=0.0, Ron2=0.0,
                 Vf1=0.0, Vf2=0.0, steps_half=1800):
        self.V1, self.n, self.fs, self.L, self.R = V1, n, fs, L, R
        self.td, self.Ron1, self.Ron2, self.Vf1, self.Vf2 = td, Ron1, Ron2, Vf1, Vf2
        self.Nh = steps_half
        self.dth = 180.0 / steps_half                       # deg per step
        self.dt = self.dth / (360.0 * fs)
        self.td_deg = td * fs * 360.0
        # total series resistance seen by i (primary-referred): R + 2 Ron1 + n^2 * 2 Ron2 when both legs conduct
        self.Req = R + 2 * Ron1 + 2 * n * n * Ron2

    def __repr__(self):
        return (f"Plant(R={self.R:g} ohm, td={self.td * 1e9:.0f} ns, Ron1={self.Ron1:g}, Ron2={self.Ron2:g}, "
                f"Vf1={self.Vf1:g}, Vf2={self.Vf2:g}, steps/half={self.Nh})")

    # ---- gating: fraction of every step spent upper-on / lower-on (the rest is dead time), so that switching
    # instants inside a step are integrated exactly (step-centre sampling gave 15 mA / 2 W timing errors)
    def _frac(self, start):
        def M(x, p, w):                                            # measure of the periodic set [p, p+w) in [0, x]
            y = x - p
            return w * np.floor(y / 360.0) + np.clip(np.mod(y, 360.0), 0.0, w)
        e = np.arange(self.Nh + 1)[:, None] * self.dth             # step edges in [0, 180]
        w = 180.0 - self.td_deg
        up = np.diff(M(e, start + self.td_deg, w), axis=0) / self.dth
        lo = np.diff(M(e, start + 180.0 + self.td_deg, w), axis=0) / self.dth
        return up, lo

    def _schedule(self, D0, D1, D2):
        return (self._frac(np.zeros_like(D0)[None, :]), self._frac(360.0 * D1[None, :]),
                self._frac(D0[None, :]), self._frac(D0[None, :] + 360.0 * D2[None, :]))

    @staticmethod
    def _leg(fr, i_out, V, Ron, Vf):
        """step-averaged midpoint voltage (w.r.t. the negative rail) and the fraction of the step the leg connects
        its output to the + rail (for the DC current); the dead-time diode follows the sign of i_out."""
        up, lo = fr
        dead = 1.0 - up - lo
        neg = i_out < 0                                            # upper diode in the dead time
        v = up * (V - Ron * i_out) + lo * (-Ron * i_out) + dead * np.where(neg, V + Vf, -Vf)
        return v, up + dead * neg

    def half_period(self, i0, sched, V2, record=False):
        """integrate one half period from i0 (N,), returns i(Ts/2), and per-step currents if record."""
        A, B, C, Dl = sched
        n, V1 = self.n, self.V1
        i = i0.copy()
        a = self.R * self.dt / self.L
        e = np.exp(-a) if a > 0 else 1.0
        Pp = np.zeros_like(i)                                       # energy into the primary bridge (V1 * i_dc)
        Ps = np.zeros_like(i)                                       # energy out of the LV bus (V2 * i_dc2)
        rec = [] if record else None
        for k in range(self.Nh):
            fA, fB, fC, fD = (A[0][k], A[1][k]), (B[0][k], B[1][k]), (C[0][k], C[1][k]), (Dl[0][k], Dl[1][k])
            vA, uA = self._leg(fA, i, V1, self.Ron1, self.Vf1)
            vB, uB = self._leg(fB, -i, V1, self.Ron1, self.Vf1)
            vC, uC = self._leg(fC, -n * i, V2, self.Ron2, self.Vf2)
            vD, uD = self._leg(fD, n * i, V2, self.Ron2, self.Vf2)
            v = vA - vB - n * (vC - vD)
            if a > 0:
                i_new = i * e + (v / self.R) * (1.0 - e)
            else:
                i_new = i + v * self.dt / self.L
            im = 0.5 * (i + i_new)
            # DC-side currents: a leg draws i_out from the + rail while its upper switch/diode conducts
            Pp += V1 * im * (uA - uB)
            Ps += V2 * n * im * (uD - uC)
            i = i_new
            if record:
                rec.append(i.copy())
        return i, Pp / self.Nh, Ps / self.Nh, (np.array(rec) if record else None)

    def steady(self, D0, D1, D2, V2, i_guess=None, iters=40, tol=1e-7):
        """periodic steady state for fixed (D0, D1, D2): dict(Ipk, Irms, P_LV (dataset sign), P_HV, i0, wave)."""
        D0, D1, D2, V2 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(D0, D1, D2, V2)]
        sched = self._schedule(D0, D1, D2)
        if i_guess is None:
            r = tm.evaluate(D0, D1, D2, V2, self.V1, self.n, self.fs, self.L, details=True)
            i_guess = r["I_edges"][:, 0]
        lo, hi = i_guess - 5.0, i_guess + 5.0
        g = lambda x: self.half_period(x, sched, V2)[0] + x          # noqa: E731
        glo, ghi = g(lo), g(hi)
        for _ in range(8):                                           # widen until bracketed
            bad = (glo > 0) | (ghi < 0)
            if not bad.any():
                break
            lo = np.where(glo > 0, lo - 10.0, lo)
            hi = np.where(ghi < 0, hi + 10.0, hi)
            glo, ghi = g(lo), g(hi)
        x = lo - glo * (hi - lo) / (ghi - glo)
        side = 0
        for _ in range(iters):                                       # Illinois regula falsi
            gx = g(x)
            left = gx < 0
            lo = np.where(left, x, lo)
            hi = np.where(left, hi, x)
            glo = np.where(left, gx, glo * np.where(side == 1, 0.5, 1.0))
            ghi = np.where(left, ghi * np.where(side == -1, 0.5, 1.0), gx)
            side = np.where(left, 1, -1)
            x_new = lo - glo * (hi - lo) / (ghi - glo)
            if np.max(np.abs(x_new - x)) < tol:
                x = x_new
                break
            x = x_new
        iend, Pp, Ps, wave = self.half_period(x, sched, V2, record=True)
        wave = np.concatenate([x[None, :], wave], axis=0)            # (Nh + 1, N): i at step edges over a half period
        return dict(Ipk=np.abs(wave).max(axis=0), Irms=np.sqrt(np.mean(0.5 * (wave[1:] ** 2 + wave[:-1] ** 2), axis=0)),
                    P_LV=Ps, P_HV=Pp, i0=x, resid=np.abs(iend + x), wave=wave)

    def operating_point(self, P_abs, D1, D2, V2, width=12.0, iters=22):
        """PI steady state on pi_steady's branch (phi = D0 + 180 (D2 - D1) >= 0, power HV -> LV): D0 such that the
        power delivered to the LV bus equals P_abs, bracketed around the ideal model's first crossing.
        Returns dict as steady() plus D0, ok and the ideal-model D0 / Ipk."""
        P_abs, D1, D2, V2 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(P_abs, D1, D2, V2)]
        ideal = ps.operating_point(P_abs, D1, D2, V2, self.V1)
        D0i = np.nan_to_num(ideal["D0"], nan=60.0)
        lo, hi = np.maximum(D0i - width, -180.0 * (D2 - D1)), D0i + width     # phi >= 0; larger D0 -> more power
        f = lambda d0: -self.steady(d0, D1, D2, V2)["P_LV"]  # noqa: E731
        plo, phi_ = f(lo) - P_abs, f(hi) - P_abs
        ok = (plo <= 0) & (phi_ >= 0)
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            pm = f(mid) - P_abs
            up = pm >= 0
            hi = np.where(up, mid, hi)
            lo = np.where(up, lo, mid)
        D0 = 0.5 * (lo + hi)
        r = self.steady(D0, D1, D2, V2)
        r.update(D0=D0, ok=ok, D0_ideal=ideal["D0"], Ipk_ideal=ideal["Ipk"])
        return r


def selftest():
    rng = np.random.default_rng(3)
    N = 200
    D0, D1, D2 = rng.uniform(-70, 70, N), rng.uniform(0.15, 0.5, N), rng.uniform(0.15, 0.5, N)
    V2 = rng.uniform(84, 109.2, N)
    ideal = tm.evaluate(D0, D1, D2, V2)
    p = Plant(steps_half=1800)
    r = p.steady(D0, D1, D2, V2)
    eI = np.abs(r["Ipk"] - ideal["Ipk"]).max()
    eP = np.abs(r["P_LV"] - ideal["P"]).max()
    eH = np.abs(r["P_HV"] + ideal["P"]).max()
    print(f"ideal plant vs tps_model on {N} random points: max |dIpk| {eI * 1e3:.2f} mA, max |dP_LV| {eP:.3f} W, "
          f"max |P_HV + P_LV| {eH:.3f} W (lossless), shooting residual {r['resid'].max():.1e} A")
    # SPS with R: the textbook DC-free current, peak slightly reduced; power balance P_HV - P_LV = losses > 0
    pr = Plant(R=0.2, steps_half=1800)
    r2 = pr.steady(D0, 0.5, 0.5, V2)
    loss = r2["P_HV"] + r2["P_LV"]
    print(f"R = 0.2 ohm, SPS: losses min {loss.min():.3f} W (must be >= 0), mean I^2R check "
          f"{np.abs(loss - 0.2 * r2['Irms'] ** 2).max():.3f} W max deviation from Irms^2 R")
    ok = eI < 5e-3 and eP < 0.5 and eH < 0.5 and loss.min() > -1e-6 and np.abs(loss - 0.2 * r2["Irms"] ** 2).max() < 0.2
    print("SELFTEST", "PASS" if ok else "FAIL")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        selftest()
