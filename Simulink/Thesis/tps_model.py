"""
tps_model.py -- exact steady-state model of the DAB converter under TPS modulation.

PWM convention (identical to DAB_Controller_System.m, STAGE 6/7):
    primary   leg A high on [0, 180) deg, leg B high on [360*D1, 360*D1 + 180)
              -> v_p = V1 * (A - B): +V1 pulse of width D1*Ts starting at 0 deg
    secondary leg C high on [D0, D0 + 180), leg D high on [D0 + 360*D2, ...)
              -> v_s = V2 * (C - D): +V2 pulse of width D2*Ts starting at D0 deg
    D1 = D2 = 0.5 is SPS. D0 is the LEADING-EDGE shift in degrees.
Inductor L sits on the primary side: L di/dt = v_p - n*v_s.

Power sign follows the Simulink dataset (P_act_W, measured on the LV side):
    P > 0  <=>  D0 < 0  (power flows LV -> HV).
Internally P_pri is the power entering the primary bridge; P = -P_pri.

Steady state: both bridge voltages are half-wave antisymmetric, so i(t + Ts/2) = -i(t)
and the current has no DC component. Everything is evaluated exactly at the (at most four)
switching instants of a half period, so there is no sampling error.
"""
import numpy as np

V1_NOM, N_TURNS, FS, L_IND = 380.0, 4.0, 80e3, 90e-6


def _bridge(theta, start, width):
    """+1 / 0 / -1 three-level bridge voltage at angle theta (deg) for a pulse at `start` of `width` deg."""
    a = np.mod(theta - start, 360.0) < 180.0
    b = np.mod(theta - start - width, 360.0) < 180.0
    return a.astype(float) - b.astype(float)


def evaluate(D0, D1, D2, V2, V1=V1_NOM, n=N_TURNS, fs=FS, L=L_IND, details=False):
    """Vectorised exact evaluation. All inputs broadcast. Returns dict with
    P   [W]  LV-side power (dataset sign convention)
    Ipk [A]  peak |i_L| (primary side)
    Irms[A]  RMS of i_L
    and, with details=True, the current at every switching edge (for ZVS checks)."""
    D0, D1, D2, V2, V1 = np.broadcast_arrays(*[np.asarray(x, float) for x in (D0, D1, D2, V2, V1)])
    shp = D0.shape
    D0, D1, D2, V2, V1 = [x.ravel() for x in (D0, D1, D2, V2, V1)]
    w1, w2 = 360.0 * D1, 360.0 * D2
    # switching instants folded into one half period [0, 180)
    bp = np.stack([np.zeros_like(D0), np.mod(w1, 180.0), np.mod(D0, 180.0), np.mod(D0 + w2, 180.0)], axis=1)
    bp = np.sort(bp, axis=1)
    edges = np.concatenate([bp, np.full((bp.shape[0], 1), 180.0)], axis=1)       # (M, 5)
    seg = np.diff(edges, axis=1)                                                  # (M, 4) deg
    mid = edges[:, :-1] + 0.5 * seg
    vp = V1[:, None] * _bridge(mid, 0.0, w1[:, None])
    vs = n * V2[:, None] * _bridge(mid, D0[:, None], w2[:, None])
    vL = vp - vs
    k = 1.0 / (360.0 * fs * L)                                                    # A per (V*deg)
    dI = vL * seg * k
    J = np.concatenate([np.zeros((dI.shape[0], 1)), np.cumsum(dI, axis=1)], axis=1)  # (M, 5)
    i0 = -0.5 * J[:, -1]
    I = i0[:, None] + J                                                           # current at the 5 edges
    Ipk = np.abs(I).max(axis=1)
    ia, ib = I[:, :-1], I[:, 1:]
    # half-period averages (the full period gives the same by symmetry)
    P_pri = np.sum(vp * 0.5 * (ia + ib) * seg, axis=1) / 180.0
    Irms = np.sqrt(np.sum((ia * ia + ia * ib + ib * ib) / 3.0 * seg, axis=1) / 180.0)
    out = dict(P=(-P_pri).reshape(shp), Ipk=Ipk.reshape(shp), Irms=Irms.reshape(shp))
    if details:
        out["edges"] = edges.reshape(shp + (5,))
        out["I_edges"] = I.reshape(shp + (5,))
    return out


def current_at(theta, D0, D1, D2, V2, V1=V1_NOM, n=N_TURNS, fs=FS, L=L_IND):
    """i_L at arbitrary angles theta (deg); theta has one more trailing axis than the parameters."""
    r = evaluate(D0, D1, D2, V2, V1, n, fs, L, details=True)
    edges, I = r["edges"], r["I_edges"]
    th = np.mod(np.asarray(theta, float), 360.0)
    sign = np.where(th >= 180.0, -1.0, 1.0)
    th = np.where(th >= 180.0, th - 180.0, th)
    out = np.empty(th.shape)
    flat_e = edges.reshape(-1, 5)
    flat_I = I.reshape(-1, 5)
    flat_t = th.reshape(flat_e.shape[0], -1)
    res = out.reshape(flat_t.shape)
    for m in range(flat_e.shape[0]):
        res[m] = np.interp(flat_t[m], flat_e[m], flat_I[m])
    return sign * out


def sps_power(D0, V2, V1=V1_NOM, n=N_TURNS, fs=FS, L=L_IND):
    """Textbook SPS power, same sign convention (P > 0 for D0 < 0)."""
    d = np.asarray(D0, float) / 180.0
    return -(n * V1 * np.asarray(V2, float) / (2.0 * fs * L)) * d * (1.0 - np.abs(d))


def waveform(D0, D1, D2, V2, N=3600, V1=V1_NOM, n=N_TURNS, fs=FS, L=L_IND):
    """Brute-force sampled waveform of one operating point (for plots and for testing `evaluate`).
    Returns angle (deg), v_p, reflected secondary voltage n*v_s, and i_L at the cell centres."""
    th = (np.arange(N) + 0.5) / N * 360.0
    vp = V1 * _bridge(th, 0.0, 360.0 * D1)
    vs_ref = n * V2 * _bridge(th, D0, 360.0 * D2)
    di = (vp - vs_ref) * (360.0 / N) / (360.0 * fs * L)
    i = np.cumsum(di) - 0.5 * di
    i = i - np.mean(i)                       # steady state has no DC component
    return th, vp, vs_ref, i
