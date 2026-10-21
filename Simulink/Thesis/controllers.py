"""
controllers.py -- every controller compared in the thesis, behind one interface:
    ctrl.predict(P_abs, V2) -> (D1, D2)          D0 is left to the PI loop (pi_steady.py)
    ctrl.memory_bytes(), ctrl.ops()               deployment cost on the MCU

SPS             D1 = D2 = 0.5
GridLUT         bilinear interpolation of a (V2, |P|) table -- the classic look-up-table controller,
                and also the way an existing dataset (e.g. the ICROS2026 training surface) is replayed
LegacyRegHDC    the ICROS2026 RegHDC: ID x quantised-level encoding, K prototypes, softmax(gamma),
                Delta = 0.5 - D normalised to [-1, 1]; trained by the same per-sample rule
                as Simulink/Data/RegHDCTrainer.m
SmoothRegHDC    proposed: continuous nonlinear encoding h = sqrt(2) cos(W x + b) (no quantisation, so
                the control surface is C-infinity), one linear read-out, Delta = softplus_tau(y) so the
                SPS region is reached continuously instead of by a classifier
"""
import numpy as np

P_MAX = 1200.0
V2_LO, V2_HI = 84.0, 109.2
D_MIN = 0.10


def norm_inputs(P_abs, V2):
    u = np.clip(np.asarray(P_abs, float) / P_MAX, 0.0, 1.0)
    v = np.clip((np.asarray(V2, float) - V2_LO) / (V2_HI - V2_LO), 0.0, 1.0)
    return np.stack(np.broadcast_arrays(u, v), axis=-1)


class SPS:
    name = "SPS"

    def predict(self, P_abs, V2):
        s = np.broadcast(np.asarray(P_abs), np.asarray(V2)).shape
        return np.full(s, 0.5), np.full(s, 0.5)

    def memory_bytes(self):
        return 0

    def ops(self):
        return 0


class GridLUT:
    """Bilinear (V2, |P|) table. D tables are clipped to [D_MIN, 0.5]."""

    def __init__(self, P_axis, V2_axis, D1_tab, D2_tab, name="LUT"):
        self.P, self.V = np.asarray(P_axis, float), np.asarray(V2_axis, float)
        self.T1, self.T2 = np.asarray(D1_tab, float), np.asarray(D2_tab, float)   # (nV, nP)
        self.name = name

    def _interp(self, T, P_abs, V2):
        p = np.clip(np.asarray(P_abs, float), self.P[0], self.P[-1])
        v = np.clip(np.asarray(V2, float), self.V[0], self.V[-1])
        ip = np.clip(np.searchsorted(self.P, p) - 1, 0, self.P.size - 2)
        iv = np.clip(np.searchsorted(self.V, v) - 1, 0, self.V.size - 2)
        tp = (p - self.P[ip]) / (self.P[ip + 1] - self.P[ip])
        tv = (v - self.V[iv]) / (self.V[iv + 1] - self.V[iv])
        a = T[iv, ip] * (1 - tp) + T[iv, ip + 1] * tp
        b = T[iv + 1, ip] * (1 - tp) + T[iv + 1, ip + 1] * tp
        return a * (1 - tv) + b * tv

    def predict(self, P_abs, V2):
        d1 = np.clip(self._interp(self.T1, P_abs, V2), D_MIN, 0.5)
        d2 = np.clip(self._interp(self.T2, P_abs, V2), D_MIN, 0.5)
        return d1, d2

    def memory_bytes(self):
        return 4 * (self.T1.size + self.T2.size + self.P.size + self.V.size)

    def ops(self):
        return 2 * 8


class LegacyRegHDC:
    """ICROS2026 RegHDC. Inputs are the SIGNED power and V2 on the paper's ranges."""

    def __init__(self, D=32, K=8, Q=32, gamma=8.0, seed=0, name="RegHDC (ICROS2026)"):
        self.D, self.K, self.Q, self.gamma, self.name = D, K, Q, gamma, name
        rng = np.random.default_rng(seed)
        self.ID = rng.choice([-1.0, 1.0], size=(2, D))
        L = np.zeros((Q, D))
        L[0] = rng.choice([-1.0, 1.0], size=D)
        flips = max(1, int(np.floor(D / 2 / (Q - 1))))
        for q in range(1, Q):
            L[q] = L[q - 1]
            idx = rng.permutation(D)[:flips]
            L[q, idx] = -L[q, idx]
        self.L = L
        self.min_X = np.array([-1200.0, 80.0])
        self.max_X = np.array([1200.0, 110.0])
        self.max_Y = 0.4
        self.C = None
        self.M = None

    def encode(self, P, V2):
        X = np.stack(np.broadcast_arrays(np.asarray(P, float), np.asarray(V2, float)), axis=-1)
        q = np.clip(np.round((X - self.min_X) / (self.max_X - self.min_X + 1e-9) * (self.Q - 1)), 0, self.Q - 1).astype(int)
        return self.ID[0] * self.L[q[..., 0]] + self.ID[1] * self.L[q[..., 1]]           # (..., D)

    def _weights(self, S):
        sim = (S @ self.C.T) / (np.linalg.norm(self.C, axis=1) * np.linalg.norm(S, axis=-1, keepdims=True) + 1e-9)
        z = sim * self.gamma
        z = z - z.max(axis=-1, keepdims=True)
        w = np.exp(z)
        return sim, w / w.sum(axis=-1, keepdims=True)

    def _out(self, S):
        _, w = self._weights(S)
        MS = np.einsum("kod,...d->...ko", self.M, S) / self.D                            # (..., K, 2)
        return np.einsum("...k,...ko->...o", w, MS)

    def fit(self, P, V2, D1, D2, epochs=400, patience=40, seed=0, sample_w=None, val_frac=0.2, verbose=False):
        rng = np.random.default_rng(seed)
        S = self.encode(P, V2)
        Y = 2.0 * (np.stack([0.5 - np.asarray(D1), 0.5 - np.asarray(D2)], axis=-1) / self.max_Y) - 1.0
        N = S.shape[0]
        perm = rng.permutation(N)
        nv = int(round(val_frac * N))
        va, tr = perm[:nv], perm[nv:]
        W = np.ones(N) if sample_w is None else np.asarray(sample_w, float)
        init = tr[np.round(np.linspace(0, tr.size - 1, self.K)).astype(int)]
        self.C = S[init].astype(float).copy()
        self.M = np.zeros((self.K, 2, self.D))
        best, best_state, bad = np.inf, None, 0
        for ep in range(1, epochs + 1):
            lr = 0.05 * (1 + np.cos(np.pi * ep / epochs)) / 2 + 0.005
            for i in rng.permutation(tr):
                s = S[i]
                sim = (self.C @ s) / (np.linalg.norm(self.C, axis=1) * np.linalg.norm(s) + 1e-9)
                k = int(np.argmax(sim))
                self.C[k] += (1 - sim[k]) * s
                z = sim * self.gamma
                w = np.exp(z - z.max())
                w /= w.sum()
                yhat = w @ (self.M @ s / self.D)                                         # (2,)
                err = Y[i] - yhat
                self.M += (lr * w * W[i])[:, None, None] * err[None, :, None] * s[None, None, :]
            pred = self._out(S[va])
            rmse = np.sqrt(np.mean(((pred - Y[va]) * self.max_Y / 2) ** 2))
            if rmse < best - 1e-6:
                best, best_state, bad = rmse, (self.C.copy(), self.M.copy()), 0
            else:
                bad += 1
                if bad >= patience:
                    break
            if verbose and ep % 20 == 0:
                print(f"    legacy ep {ep}: val RMSE(D) {rmse:.4f}")
        self.C, self.M = best_state
        self.val_rmse = best
        return self

    def predict(self, P_abs, V2, sign=+1.0):
        y = self._out(self.encode(sign * np.asarray(P_abs, float), V2))
        delta = (y + 1.0) / 2.0 * self.max_Y
        d1 = np.clip(0.5 - delta[..., 0], D_MIN, 0.5)
        d2 = np.clip(0.5 - delta[..., 1], D_MIN, 0.5)
        return d1, d2

    def memory_bytes(self):
        # int32 prototypes and read-out, int8 bipolar ID / level vectors (as in RegHDCTrainer's estimate)
        return self.K * self.D * 4 + self.K * 2 * self.D * 4 + self.Q * self.D + 2 * self.D

    def ops(self):
        return self.K * self.D + self.K * 2 * self.D + 2 * self.D


def softplus(y, tau):
    return tau * np.logaddexp(0.0, y / tau)


def sigmoid(z):
    return 0.5 * (1.0 + np.tanh(0.5 * z))


class SmoothRegHDC:
    """Proposed E2E regression controller.
    x = (|P|/P_MAX, (V2 - V2_LO)/(V2_HI - V2_LO)),  h = sqrt(2) cos(x W + b)  (D dims),
    y = h M / sqrt(D) + c,   Delta = min(softplus_tau(y), 0.5 - D_MIN),  D = 0.5 - Delta."""

    def __init__(self, D=128, sigma=0.25, tau=0.004, seed=0, name="Smooth RegHDC"):
        rng = np.random.default_rng(seed)
        self.Dh, self.sigma, self.tau, self.name = D, sigma, tau, name
        self.W = rng.normal(0.0, 1.0 / sigma, size=(2, D))
        self.b = rng.uniform(0.0, 2 * np.pi, size=D)
        self.M = np.zeros((D, 2))
        self.c = np.zeros(2)

    def features(self, P_abs, V2):
        x = norm_inputs(P_abs, V2)
        return np.sqrt(2.0) * np.cos(x @ self.W + self.b)

    def raw(self, H):
        return H @ self.M / np.sqrt(self.Dh) + self.c

    def delta(self, y):
        return np.minimum(softplus(y, self.tau), 0.5 - D_MIN)

    def predict(self, P_abs, V2):
        H = self.features(P_abs, V2)
        d = self.delta(self.raw(H))
        return 0.5 - d[..., 0], 0.5 - d[..., 1]

    def fit_ridge(self, P_abs, V2, D1, D2, alpha=1e-4, sample_w=None):
        """Least squares on the pre-activation: the target y is the softplus inverse of Delta,
        floored so SPS points (Delta = 0) ask for a clearly negative y."""
        H = self.features(P_abs, V2) / np.sqrt(self.Dh)
        delta = np.stack([0.5 - np.asarray(D1, float), 0.5 - np.asarray(D2, float)], axis=-1)
        y = np.where(delta > 4 * self.tau, delta, self.tau * np.log(np.expm1(np.clip(delta, 1e-6, None) / self.tau)))
        y = np.maximum(y, -6 * self.tau)
        w = np.ones(H.shape[0]) if sample_w is None else np.asarray(sample_w, float)
        Ha = np.concatenate([H, np.ones((H.shape[0], 1))], axis=1)
        A = Ha.T @ (Ha * w[:, None]) + alpha * np.eye(Ha.shape[1])
        sol = np.linalg.solve(A, Ha.T @ (y * w[:, None]))
        self.M, self.c = sol[:-1], sol[-1]
        return self

    def jacobian(self, P_abs, V2):
        """dDelta/dx (normalised inputs) -- used for the smoothness penalty and reported roughness."""
        x = norm_inputs(P_abs, V2)
        z = x @ self.W + self.b
        dH = -np.sqrt(2.0) * np.sin(z)[..., None, :] * self.W[None, :, :]                 # (N, 2in, D)
        y = np.sqrt(2.0) * np.cos(z) @ self.M / np.sqrt(self.Dh) + self.c
        dy = dH @ self.M / np.sqrt(self.Dh)                                                 # (N, 2in, 2out)
        g = sigmoid(y / self.tau) * (softplus(y, self.tau) < 0.5 - D_MIN)
        return dy * g[:, None, :]

    def memory_bytes(self, bytes_per=4):
        return bytes_per * (self.W.size + self.b.size + self.M.size + self.c.size)

    def ops(self):
        return 2 * self.Dh + self.Dh + 2 * self.Dh     # projection, cos, read-out

    def to_json(self, path, **meta):
        import json
        d = dict(kind="SmoothRegHDC", name=self.name, D=self.Dh, sigma=self.sigma, tau=self.tau,
                 P_MAX=P_MAX, V2_LO=V2_LO, V2_HI=V2_HI, D_MIN=D_MIN,
                 W=self.W.tolist(), b=self.b.tolist(), M=self.M.tolist(), c=self.c.tolist(), meta=meta)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(d, f)

    @classmethod
    def from_json(cls, path):
        import json
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        m = cls(D=d["D"], sigma=d["sigma"], tau=d["tau"], name=d["name"])
        m.W, m.b, m.M, m.c = (np.asarray(d[k], float) for k in ("W", "b", "M", "c"))
        m.meta = d.get("meta", {})
        return m


class SmoothRegHDCQ(SmoothRegHDC):
    """Fixed-point deployment view of a trained SmoothRegHDC: int16 W/b/M, the cosine from a table with
    linear interpolation between entries (as CMSIS-DSP arm_cos_q15 / arm_cos_f32 do). Nearest-entry lookup is
    NOT good enough: its +-pi/N phase error is amplified by sum|M| (measured 0.033 in D for alpha = 1e-6).
    Used to report the accuracy cost of the MCU representation."""

    def __init__(self, base, bits=16, cos_table=1024):
        self.__dict__.update({k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in base.__dict__.items()})
        self.name = base.name + f" (int{bits})"
        self.bits, self.cos_table = bits, cos_table
        for k in ("W", "b", "M"):
            a = getattr(self, k)
            s = np.abs(a).max() / (2 ** (bits - 1) - 1)
            setattr(self, k, np.round(a / s) * s)

    def features(self, P_abs, V2):
        x = norm_inputs(P_abs, V2)
        z = np.mod(x @ self.W + self.b, 2 * np.pi) / (2 * np.pi) * self.cos_table
        i0 = np.floor(z).astype(int)
        fr = z - i0
        tab = np.round(np.cos(np.arange(self.cos_table + 1) * 2 * np.pi / self.cos_table) * 32767) / 32767   # q15
        c = tab[i0 % self.cos_table] * (1 - fr) + tab[i0 % self.cos_table + 1] * fr
        return np.sqrt(2.0) * c

    def memory_bytes(self, bytes_per=2):
        return bytes_per * (self.W.size + self.b.size + self.M.size + self.c.size) + 2 * (self.cos_table + 1)
