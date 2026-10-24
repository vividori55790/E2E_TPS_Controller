"""
plant_full.py -- Simulink-faithful steady state of the dataset-era Control_Logic.slx power stage, in Python
(T6 fallback; parameters from out/plant_params.md, extracted from the .slx XML without MATLAB).

What it adds to plant_sim.py (half-period, symmetric):
  * full-period shooting with the series DC-blocking capacitors as a state (C3 = 1000 uF primary, C4 = 1000 uF
    secondary -> C_eq = (1/C3 + n^2/C4)^-1 primary-referred), so asymmetric gating has a steady state;
  * the controller's PWM: 100 ns sample grid (2.88 deg), every command edge rounded UP to the grid
    (phase counter logic of DAB_Controller_System.m: high side on iff mod(cp - p, 360) in [0, 180)), so on-times are
    62 or 63 samples;
  * the half-bridge driver: incoming device turns on td (100 ns) after the command edge; zero Coss, so in the dead
    time the body diode set by the current sign conducts;
  * MOSFET channel Ron in both directions, body diode exponential (Is 1e-9 A, N 1.2, 25 C) in parallel: reverse
    conduction drop = min(Ron |i|, Vd(|i|));
  * DC droop of the sources: V2_bus = V2 - R_lv * P_LV / V2 (R_lv 0.05 ohm), V1_bus = V1 + R_hv * P_HV / V1 (0.06),
    solved by fixed point (the dataset's V2 label is the source EMF; P_LV is measured at the bus);
  * Ipk both continuous and as the dataset measures it: max |i_L| on the 100 ns sample grid (+ optional offset).
Integration: 1250 steps per period (10 ns; 10 per controller sample), exact RL-C step with v constant per sub-step
fraction (fractions of each step spent upper-on / lower-on / dead are integrated exactly).
Shooting: Newton on x0 = (i0, vc0) with a finite-difference 2x2 Jacobian per point.
"""
import numpy as np

V1_NOM, N_TURNS, FS, L_IND = 380.0, 4.0, 80e3, 90e-6


def vdiode(i, Is=1e-9, N=1.2, Vt=0.025693):
    return N * Vt * np.log1p(np.abs(i) / Is)


class FullPlant:
    def __init__(self, Ron=0.040, td=100e-9, quant=True, grid_ns=100.0, diode=True, Vf_const=0.7,
                 C3=1000e-6, C4=1000e-6, R_lv=0.05, R_hv=0.06, droop=True, R_extra=0.0, steps=1250,
                 V1=V1_NOM, n=N_TURNS, fs=FS, L=L_IND, sample_offset_ns=0.0, Rs_dev=0.0, rev_clamp=True):
        # Rs_dev: device terminal resistance (RS + RD = 12 mOhm in the block) in series with BOTH the channel and the
        # body diode; rev_clamp False = the body diode does not share reverse current while the channel is on
        self.Ron, self.td, self.quant, self.diode, self.Vf_const = Ron, td, quant, diode, Vf_const
        self.Rs_dev, self.rev_clamp = Rs_dev, rev_clamp
        self.C3, self.C4, self.R_lv, self.R_hv, self.droop, self.R = C3, C4, R_lv, R_hv, droop, R_extra
        self.V1, self.n, self.fs, self.L, self.Ns = V1, n, fs, L, steps
        self.g = 360.0 * fs * grid_ns * 1e-9                  # grid in deg (2.88)
        self.td_deg = 360.0 * fs * td
        self.dth = 360.0 / steps
        self.dt = 1.0 / (fs * steps)
        self.Ceq = 1.0 / (1.0 / C3 + n * n / C4)
        self.sample_offset = 360.0 * fs * sample_offset_ns * 1e-9

    def __repr__(self):
        return (f"FullPlant(Ron={self.Ron * 1e3:.0f} mOhm, td={self.td * 1e9:.0f} ns, quant={self.quant}, "
                f"diode={'exp' if self.diode else self.Vf_const}, Ceq={self.Ceq * 1e6:.1f} uF, droop={self.droop})")

    # ---------------------------------------------------------------- gating
    def _q(self, p):
        return np.ceil(np.round(p / self.g, 9)) * self.g if self.quant else p

    def _frac_leg(self, p):
        """(Ns, N) fractions of each step with the upper / lower device on, for a leg of command phase p (N,)."""
        p = np.mod(p, 360.0)
        on_cmd = self._q(p)                                    # command edges (rounded up to the sample grid)
        off_cmd = self._q(p + 180.0)
        e = np.arange(self.Ns + 1)[:, None] * self.dth          # step edges [0, 360]

        def meas(x, a, b):                                     # measure of the periodic set [a, b) (mod 360) in [0, x]
            w = b - a
            y = x - a
            return w * np.floor(y / 360.0) + np.clip(np.mod(y, 360.0), 0.0, w)
        up = np.diff(meas(e, (on_cmd + self.td_deg)[None, :], off_cmd[None, :]), axis=0) / self.dth
        lo = np.diff(meas(e, (off_cmd + self.td_deg)[None, :], (on_cmd + 360.0)[None, :]), axis=0) / self.dth
        return up, lo

    def schedule(self, D0, D1, D2):
        z = np.zeros_like(D0)
        return [self._frac_leg(z), self._frac_leg(360.0 * D1), self._frac_leg(D0), self._frac_leg(D0 + 360.0 * D2)]

    # ---------------------------------------------------------------- devices
    def _vd(self, i):
        return vdiode(i) if self.diode else np.full_like(i, self.Vf_const)

    def _leg(self, fr, i_out, V):
        up, lo = fr
        dead = 1.0 - up - lo
        a = np.abs(i_out)
        rs = self.Rs_dev * a
        fwd = self.Ron * a + rs
        # reverse conduction with the gate on: channel || body diode (behind the terminal resistance)
        rev = (np.minimum(self.Ron * a, self._vd(a)) if self.rev_clamp else self.Ron * a) + rs
        vdd = self._vd(a) + rs
        pos = i_out > 0
        v_up = np.where(pos, V - fwd, V + rev)                # upper: forward when i_out > 0
        v_lo = np.where(pos, -rev, fwd)                       # lower: forward when i_out < 0
        v_dd = np.where(pos, -vdd, V + vdd)                   # dead time: body diode by current sign
        v = up * v_up + lo * v_lo + dead * v_dd
        conn_up = up + dead * (~pos)                           # fraction connected to the + rail (DC current)
        return v, conn_up

    # ---------------------------------------------------------------- one period
    def period(self, i0, vc0, sch, V1b, V2b, record=False):
        n = self.n
        i, vc = i0.copy(), vc0.copy()
        E_hv = np.zeros_like(i)
        E_lv = np.zeros_like(i)
        rec = [i.copy()] if record else None
        A, B, C, D = sch
        L, Ceq, dt, R = self.L, self.Ceq, self.dt, self.R
        for k in range(self.Ns):
            vA, uA = self._leg((A[0][k], A[1][k]), i, V1b)
            vB, uB = self._leg((B[0][k], B[1][k]), -i, V1b)
            vC, uC = self._leg((C[0][k], C[1][k]), -n * i, V2b)
            vD, uD = self._leg((D[0][k], D[1][k]), n * i, V2b)
            v = vA - vB - n * (vC - vD)
            # semi-implicit (symplectic) RL-C step: the LC period (0.46 ms) is 37x the switching period
            i_new = i + (v - vc - R * i) * dt / L
            im = 0.5 * (i + i_new)
            vc = vc + im * dt / Ceq
            E_hv += V1b * im * (uA - uB)
            E_lv += V2b * n * im * (uD - uC)
            i = i_new
            if record:
                rec.append(i.copy())
        return i, vc, E_hv / self.Ns, E_lv / self.Ns, (np.array(rec) if record else None)

    def steady_fixed(self, D0, D1, D2, V1b, V2b, iters=8, x0=None):
        """periodic steady state at fixed bus voltages; returns dict."""
        sch = self.schedule(D0, D1, D2)
        N = D0.size
        if x0 is None:
            import tps_model as tm
            r = tm.evaluate(D0, D1, D2, V2b, V1b, details=True)
            i0 = r["I_edges"][:, 0]
            vc0 = np.zeros(N)
        else:
            i0, vc0 = x0
        h = np.array([1e-3, 1e-2])
        for _ in range(iters):
            i1, v1, _, _, _ = self.period(i0, vc0, sch, V1b, V2b)
            ia, va, _, _, _ = self.period(i0 + h[0], vc0, sch, V1b, V2b)
            ib, vb, _, _, _ = self.period(i0, vc0 + h[1], sch, V1b, V2b)
            F = np.stack([i1 - i0, v1 - vc0], axis=1)                       # residual
            J = np.empty((N, 2, 2))
            J[:, 0, 0] = (ia - i1) / h[0] - 1.0
            J[:, 1, 0] = (va - v1) / h[0]
            J[:, 0, 1] = (ib - i1) / h[1]
            J[:, 1, 1] = (vb - v1) / h[1] - 1.0
            dx = np.linalg.solve(J, -F[:, :, None])[:, :, 0]
            i0, vc0 = i0 + dx[:, 0], vc0 + dx[:, 1]
            if np.abs(F[:, 0]).max() < 1e-6:
                break
        iT, vT, Ph, Pl, wave = self.period(i0, vc0, sch, V1b, V2b, record=True)
        cont = np.abs(wave).max(axis=0)
        # dataset measurement: max |i| on the 100 ns sample grid (every 10th step when steps = 1250), + offset
        stride = int(round(self.g / self.dth))
        off = int(round(self.sample_offset / self.dth))
        samp = np.abs(wave[off::stride]).max(axis=0)
        return dict(Ipk=cont, Ipk_sampled=samp, Irms=np.sqrt(np.mean(wave[:-1] ** 2, axis=0)), P_HV=Ph, P_LV=Pl,
                    i0=i0, vc0=vc0, resid=np.abs(iT - i0), wave=wave)

    def operating_point(self, P_abs, D1, D2, V2, width=15.0, iters=18):
        """PI steady state, power HV -> LV (pi_steady's branch, phi = D0 + 180 (D2 - D1) >= 0): D0 such that the
        power delivered to the LV bus is P_abs. Bus voltages are known at the operating point (|P_LV| = P_abs), so the
        droop is applied up front; bisection on D0 around the ideal first crossing, warm-started shooting."""
        import pi_steady as ps
        P_abs, D1, D2, V2 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(P_abs, D1, D2, V2)]
        V2b = V2 + (self.R_lv * P_abs / V2 if self.droop else 0.0)
        V1b = np.full_like(V2, self.V1) - (self.R_hv * 1.05 * P_abs / self.V1 if self.droop else 0.0)
        ideal = ps.operating_point(P_abs, D1, D2, V2b, V1b)
        D0i = np.nan_to_num(ideal["D0"], nan=60.0)
        # no phi >= 0 clamp: with dead time the zero-power phase moves (SPS at V2 = 84 V delivers 150 W at D0 = 0)
        lo, hi = D0i - width, D0i + width
        x0 = None

        def pw(d0, x0):
            r = self.steady_fixed(d0, D1, D2, V1b, V2b, x0=x0)
            return -r["P_LV"], r

        plo, _ = pw(lo, None)
        phi_, r = pw(hi, None)
        ok = (plo <= P_abs) & (phi_ >= P_abs)
        x0 = (r["i0"], r["vc0"])
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            pm, r = pw(mid, x0)
            x0 = (r["i0"], r["vc0"])
            up = pm >= P_abs
            hi = np.where(up, mid, hi)
            lo = np.where(up, lo, mid)
        D0 = 0.5 * (lo + hi)
        r = self.steady_fixed(D0, D1, D2, V1b, V2b, x0=x0)
        r.update(D0=D0, ok=ok, D0_ideal=ideal["D0"], Ipk_ideal=ideal["Ipk"], V1_bus=V1b, V2_bus=V2b)
        return r

    def steady(self, D0, D1, D2, V2, droop_iters=3):
        """steady state with source droop: V2 = LV source EMF (the dataset label), V1 = HV source."""
        D0, D1, D2, V2 = [np.asarray(x, float).ravel() for x in np.broadcast_arrays(D0, D1, D2, V2)]
        V1b, V2b = np.full_like(V2, self.V1), V2.copy()
        r, x0 = None, None
        for _ in range(droop_iters if self.droop else 1):
            r = self.steady_fixed(D0, D1, D2, V1b, V2b, x0=x0)
            x0 = (r["i0"], r["vc0"])
            if not self.droop:
                break
            # P_LV > 0: LV -> HV, LV source supplies current -> bus sags; P_HV (into the primary bridge from the HV
            # bus) < 0 then, current flows into the HV source -> HV bus rises
            V2b = V2 - self.R_lv * r["P_LV"] / V2
            V1b = self.V1 - self.R_hv * r["P_HV"] / self.V1
        r.update(V1_bus=V1b, V2_bus=V2b)
        return r
