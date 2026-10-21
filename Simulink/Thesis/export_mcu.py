"""
export_mcu.py -- T9: Smooth RegHDC for the STM32 (float32 path and int16 path) + bit-exact Python emulation.

  python export_mcu.py [--model out/model_smooth_reghdc_bX.json]
writes out/SmoothRegHDC_STM32.h (parameters), out/smooth_reghdc.c / .h (inference), out/export_mcu.log.

float32 path  u = P/P_MAX, v = (V2 - V2_LO)/(V2_HI - V2_LO) clipped to [0, 1]; h_d = cos(u W0_d + v W1_d + b_d);
              y_o = sum_d h_d M'_do + c_o with M' = sqrt(2) M / sqrt(D) folded in; Delta = softplus_tau(y);
              D = 0.5 - min(Delta, 0.5 - D_MIN).
int16 path    inputs in Q15; W stored as int16 in 2^-k TURNS, b as uint16 in 2^-16 turns, so the phase
              u_q15*W + v_q15*W + (b << (k-1)) is a uint32 in 2^-(k+15) turns and its wrap-around is exactly mod 2 pi;
              cos from a 1025-entry Q15 table with linear interpolation (CMSIS arm_cos_q15 style); read-out M' as
              int16 * 2^-m with a 64-bit accumulator (SMLAL on Cortex-M4); the two softplus calls stay in float.
SmoothRegHDCFixed below reproduces the int16 C code operation by operation (uint32 wrap, arithmetic shifts,
int64 accumulation) -- it is what evaluate.py reports as the int16 controller. No C compiler is installed on this
PC, so the C file itself is not compiled here; the emulation follows it line by line.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import controllers as ct  # noqa: E402

OUT = os.path.join(HERE, "out")
COS_N = 1024


class SmoothRegHDCFixed:
    """Bit-exact emulation of smooth_reghdc_q15() for a trained SmoothRegHDC."""

    def __init__(self, base, name=None):
        self.base = base
        self.name = name or base.name + " int16 (C)"
        self.Dh, self.tau = base.Dh, base.tau
        turns = base.W / (2 * np.pi)
        self.k = int(np.floor(np.log2(32767.0 / np.abs(turns).max())))
        if self.k < 1:
            raise ValueError("W too large for the int16 turn format")
        self.WQ = np.round(turns * 2 ** self.k).astype(np.int64)                       # (2, D) int16
        self.BQ = np.round(np.mod(base.b / (2 * np.pi), 1.0) * 65536).astype(np.int64) % 65536   # uint16
        Mp = np.sqrt(2.0) * base.M / np.sqrt(base.Dh)                                  # (D, 2)
        self.m = int(np.floor(np.log2(32767.0 / np.abs(Mp).max())))
        self.MQ = np.round(Mp * 2 ** self.m).astype(np.int64)                          # int16
        self.C = base.c.astype(np.float32)
        self.COS = np.round(np.cos(np.arange(COS_N + 1) * 2 * np.pi / COS_N) * 32767).astype(np.int64)
        assert np.abs(self.WQ).max() <= 32767 and np.abs(self.MQ).max() <= 32767

    def raw_q(self, u_q15, v_q15):
        fb = self.k + 15                                     # phase unit = 2^-fb turn
        ph = (u_q15[:, None] * self.WQ[0][None] + v_q15[:, None] * self.WQ[1][None]
              + (self.BQ << (self.k - 1))[None]) & 0xFFFFFFFF  # uint32 wrap-around
        frac = ph & ((1 << fb) - 1)
        sh = fb - 10                                         # 1024-entry table
        idx = frac >> sh
        fr = (frac >> (sh - 15)) & 0x7FFF if sh >= 15 else (frac << (15 - sh)) & 0x7FFF
        t0, t1 = self.COS[idx], self.COS[idx + 1]
        c = t0 + (((t1 - t0) * fr) >> 15)                    # arithmetic shift, as on ARM
        acc = c @ self.MQ                                    # int64 accumulator, (N, 2)
        return acc.astype(np.float32) * np.float32(2.0 ** -(15 + self.m)) + self.C

    def predict(self, P_abs, V2):
        x = ct.norm_inputs(P_abs, V2)
        shp = x.shape[:-1]
        x = x.reshape(-1, 2)
        uq = np.round(x[:, 0] * 32767).astype(np.int64)
        vq = np.round(x[:, 1] * 32767).astype(np.int64)
        y = self.raw_q(uq, vq).astype(np.float64)
        sp = np.where(y > 20 * self.tau, y, self.tau * np.log1p(np.exp(np.minimum(y, 20 * self.tau) / self.tau)))
        d = np.minimum(sp, 0.5 - ct.D_MIN)
        return (0.5 - d[:, 0]).reshape(shp), (0.5 - d[:, 1]).reshape(shp)

    def memory_bytes(self):
        return 2 * (self.WQ.size + self.BQ.size + self.MQ.size + self.COS.size) + 4 * self.C.size

    def ops(self):
        # per dimension: 2 MAC phase, table interpolation (~4), 2 MAC read-out
        return 8 * self.Dh


def float32_predict(base, P_abs, V2):
    """The float32 C path in numpy float32 (accumulation order differs from the MCU only at the 1e-7 level)."""
    x = ct.norm_inputs(P_abs, V2).astype(np.float32).reshape(-1, 2)
    W, b = base.W.astype(np.float32), base.b.astype(np.float32)
    Mp = (np.sqrt(2.0) * base.M / np.sqrt(base.Dh)).astype(np.float32)
    h = np.cos(x[:, :1] * W[0] + x[:, 1:] * W[1] + b)
    y = (h @ Mp + base.c.astype(np.float32)).astype(np.float64)
    tau = base.tau
    sp = np.where(y > 20 * tau, y, tau * np.log1p(np.exp(np.minimum(y, 20 * tau) / tau)))
    d = np.minimum(sp, 0.5 - ct.D_MIN)
    return 0.5 - d[:, 0], 0.5 - d[:, 1]


# ------------------------------------------------------------------ C emission
def _arr(name, ctype, a, fmt, per=8):
    a = np.asarray(a).ravel()
    body = ",\n    ".join(", ".join(fmt(x) for x in a[i:i + per]) for i in range(0, a.size, per))
    return f"static const {ctype} {name} = {{\n    {body}\n}};\n"


def emit(base, fx, path_h, path_c, path_ch, src):
    D = base.Dh
    Mp = np.sqrt(2.0) * base.M / np.sqrt(D)
    def f(x):                                   # C float literal: "1200f" is not valid C, "1200.0f" is
        s = f"{float(x):.9g}"
        return (s if any(ch in s for ch in ".en") else s + ".0") + "f"
    i = lambda x: f"{int(x)}"  # noqa: E731
    h = [f"/* SmoothRegHDC_STM32.h -- generated by Simulink/Thesis/export_mcu.py from {os.path.basename(src)}.",
         " * Smooth RegHDC: (|P|, V2) -> (D1, D2) for the DAB TPS modulator; D0 is closed by the PI loop.",
         f" * D = {D}, sigma = {base.sigma:g}, tau = {base.tau:g}. float32 memory {4 * (2 * D + D + 2 * D + 2)} B,"
         f" int16 memory {fx.memory_bytes()} B. */",
         "#ifndef SMOOTH_REGHDC_STM32_H", "#define SMOOTH_REGHDC_STM32_H", "", "#include <stdint.h>", "",
         f"#define SRHDC_D        {D}", f"#define SRHDC_TAU      {f(base.tau)}", f"#define SRHDC_P_MAX    {f(ct.P_MAX)}",
         f"#define SRHDC_V2_LO    {f(ct.V2_LO)}", f"#define SRHDC_V2_HI    {f(ct.V2_HI)}",
         f"#define SRHDC_D_MIN    {f(ct.D_MIN)}",
         f"#define SRHDC_WQ_SHIFT {fx.k}   /* WQ in 2^-{fx.k} turns; phase unit 2^-{fx.k + 15} turn */",
         f"#define SRHDC_MQ_SHIFT {fx.m}   /* MQ = M' * 2^{fx.m} */", "#define SRHDC_COS_N    1024", "",
         "/* ---- float32 path: rad per normalised input, rad, read-out with sqrt(2)/sqrt(D) folded in ---- */",
         _arr("SRHDC_W[2][SRHDC_D]", "float", base.W, f),
         _arr("SRHDC_B[SRHDC_D]", "float", base.b, f),
         _arr("SRHDC_M[SRHDC_D][2]", "float", Mp, f),
         _arr("SRHDC_C[2]", "float", base.c, f),
         "/* ---- int16 path ---- */",
         _arr("SRHDC_WQ[2][SRHDC_D]", "int16_t", fx.WQ, i, 16),
         _arr("SRHDC_BQ[SRHDC_D]", "uint16_t", fx.BQ, i, 16),
         _arr("SRHDC_MQ[SRHDC_D][2]", "int16_t", fx.MQ, i, 16),
         _arr("SRHDC_COS_Q15[SRHDC_COS_N + 1]", "int16_t", fx.COS, i, 16),
         "#endif"]
    txt = "\n".join(h)
    open(path_h, "w", encoding="utf-8", newline="\n").write(txt + "\n")
    open(path_ch, "w", encoding="utf-8", newline="\n").write(C_HEADER)
    open(path_c, "w", encoding="utf-8", newline="\n").write(C_SOURCE)


C_HEADER = """/* smooth_reghdc.h -- Smooth RegHDC inference (generated by export_mcu.py). */
#ifndef SMOOTH_REGHDC_H
#define SMOOTH_REGHDC_H
#include <stdint.h>
/* |P| in W, V2 in V -> D1, D2 (pulse widths in periods, 0.5 = SPS). D0 stays with the PI loop. */
void smooth_reghdc_f32(float p_abs, float v2, float *d1, float *d2);
/* same with the int16 parameter set; u_q15 = |P|/P_MAX, v_q15 = (V2-V2_LO)/(V2_HI-V2_LO), both in [0, 32767] */
void smooth_reghdc_q15(int16_t u_q15, int16_t v_q15, float *d1, float *d2);
int16_t smooth_reghdc_u_q15(float p_abs);
int16_t smooth_reghdc_v_q15(float v2);
#endif
"""

C_SOURCE = r"""/* smooth_reghdc.c -- Smooth RegHDC inference for the DAB TPS modulator (generated by export_mcu.py).
 * Bit-exact Python emulation of smooth_reghdc_q15(): export_mcu.SmoothRegHDCFixed. */
#include <math.h>
#include "smooth_reghdc.h"
#include "SmoothRegHDC_STM32.h"

static float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

static void finish(const float y[2], float *d1, float *d2)
{
    float d[2];
    for (int o = 0; o < 2; ++o) {
        float t = y[o] / SRHDC_TAU;
        float sp = (t > 20.0f) ? y[o] : SRHDC_TAU * log1pf(expf(t));      /* softplus_tau */
        if (sp > 0.5f - SRHDC_D_MIN) sp = 0.5f - SRHDC_D_MIN;
        d[o] = 0.5f - sp;
    }
    *d1 = d[0];
    *d2 = d[1];
}

void smooth_reghdc_f32(float p_abs, float v2, float *d1, float *d2)
{
    const float u = clamp01(p_abs / SRHDC_P_MAX);
    const float v = clamp01((v2 - SRHDC_V2_LO) / (SRHDC_V2_HI - SRHDC_V2_LO));
    float y[2] = {SRHDC_C[0], SRHDC_C[1]};
    for (int d = 0; d < SRHDC_D; ++d) {
        const float h = cosf(u * SRHDC_W[0][d] + v * SRHDC_W[1][d] + SRHDC_B[d]);   /* or arm_cos_f32 */
        y[0] += h * SRHDC_M[d][0];
        y[1] += h * SRHDC_M[d][1];
    }
    finish(y, d1, d2);
}

int16_t smooth_reghdc_u_q15(float p_abs) { return (int16_t)lrintf(clamp01(p_abs / SRHDC_P_MAX) * 32767.0f); }
int16_t smooth_reghdc_v_q15(float v2)
{
    return (int16_t)lrintf(clamp01((v2 - SRHDC_V2_LO) / (SRHDC_V2_HI - SRHDC_V2_LO)) * 32767.0f);
}

void smooth_reghdc_q15(int16_t u_q15, int16_t v_q15, float *d1, float *d2)
{
    enum { FB = SRHDC_WQ_SHIFT + 15, SH = FB - 10 };          /* phase unit 2^-FB turn, 1024-entry table */
    const uint32_t frac_mask = (FB >= 32) ? 0xFFFFFFFFu : ((1u << FB) - 1u);
    int64_t acc0 = 0, acc1 = 0;
    for (int d = 0; d < SRHDC_D; ++d) {
        /* uint32 wrap-around == phase mod one turn, because one turn is 2^FB units */
        uint32_t ph = (uint32_t)((int32_t)u_q15 * SRHDC_WQ[0][d]) + (uint32_t)((int32_t)v_q15 * SRHDC_WQ[1][d])
                    + ((uint32_t)SRHDC_BQ[d] << (SRHDC_WQ_SHIFT - 1));
        const uint32_t frac = ph & frac_mask;
        const uint32_t idx = frac >> SH;
        const int32_t fr = (SH >= 15) ? (int32_t)((frac >> (SH - 15)) & 0x7FFF)
                                      : (int32_t)((frac << (15 - SH)) & 0x7FFF);
        const int32_t t0 = SRHDC_COS_Q15[idx], t1 = SRHDC_COS_Q15[idx + 1];
        const int32_t c = t0 + (((t1 - t0) * fr) >> 15);       /* arithmetic shift (ARM GCC / armclang) */
        acc0 += (int64_t)c * SRHDC_MQ[d][0];                    /* SMLAL */
        acc1 += (int64_t)c * SRHDC_MQ[d][1];
    }
    const float s = ldexpf(1.0f, -(15 + SRHDC_MQ_SHIFT));
    float y[2] = {(float)acc0 * s + SRHDC_C[0], (float)acc1 * s + SRHDC_C[1]};
    finish(y, d1, d2);
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None, help="model json (default: b* chosen by evaluate.build)")
    a = ap.parse_args()
    if a.model:
        base = ct.SmoothRegHDC.from_json(a.model)
        src = a.model
    else:
        import evaluate
        _, base = evaluate.build()
        src = f"{base.name}"
    fx = SmoothRegHDCFixed(base)
    rng = np.random.default_rng(5)
    P, V = rng.uniform(0, ct.P_MAX, 20000), rng.uniform(ct.V2_LO, ct.V2_HI, 20000)
    r1, r2 = base.predict(P, V)
    q1, q2 = fx.predict(P, V)
    f1, f2 = float32_predict(base, P, V)
    lines = [f"model {src}: D {base.Dh}, sigma {base.sigma}, tau {base.tau}",
             f"int16 format: W in 2^-{fx.k} turns (max |WQ| {np.abs(fx.WQ).max()}), M' * 2^{fx.m} "
             f"(max |MQ| {np.abs(fx.MQ).max()}), memory {fx.memory_bytes()} B; float32 memory "
             f"{4 * (5 * base.Dh + 2)} B",
             f"float32 path vs float64 model: max |dD| {max(np.abs(f1 - r1).max(), np.abs(f2 - r2).max()):.2e}",
             f"int16 path (C emulation) vs float64 model: max |dD| {max(np.abs(q1 - r1).max(), np.abs(q2 - r2).max()):.2e}"
             f", RMS {np.sqrt(np.mean(np.concatenate([(q1 - r1) ** 2, (q2 - r2) ** 2]))):.2e} (20 000 random points)"]
    emit(base, fx, os.path.join(OUT, "SmoothRegHDC_STM32.h"), os.path.join(OUT, "smooth_reghdc.c"),
         os.path.join(OUT, "smooth_reghdc.h"), src)
    lines.append("wrote out/SmoothRegHDC_STM32.h, out/smooth_reghdc.c, out/smooth_reghdc.h")
    print("\n".join(lines))
    open(os.path.join(OUT, "export_mcu.log"), "w", encoding="utf-8").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
