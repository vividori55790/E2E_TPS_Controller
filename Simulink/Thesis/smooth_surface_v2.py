"""
smooth_surface_v2.py -- T1: diagnose the roughness of the v1 surface and replace its smoother.

Diagnosis (v1): every adjacent step > 0.04 sits at |P| <= 175 W, and every step > 0.1 at |P| <= 75 W with
V2 in 92.4-97.2 V, i.e. around the corner (|P| = 0, k = n V2 / V1 = 1). There the optimum is singular: for k != 1 the light-load optimum
goes to D_min as |P| -> 0, while at k = 1 SPS is optimal for every |P| > 0, so the transition power
shrinks to zero as k -> 1. A hard admissible set (gap <= eps) forces the surface to follow that.

v2 trades a little of the (small) gain near the corner for a bounded gradient:
    E = sum_i gap_i(D_i) + mu * Irms_i / Irms_sps_i
        + lam * [ sum_P-edges |dD|^2 / hP^2 + sum_V-edges |dD|^2 / hV^2 ]
gap_i = Ipk_i(D_i) - Ipk_opt_i in A, hard constraint Ipk_i <= Ipk_SPS_i (no degradation at any node),
hP, hV = grid steps in the normalised inputs of the regression model (|P|/1200, (V2-84)/25.2), so the
pairwise term is a discretised Dirichlet energy  integral |grad D|^2  in model coordinates.
Optimised over the 0.005 lattice of landscape_v1 by red-black (checkerboard) ICM with lam continuation.

Every surface is then re-evaluated EXACTLY (pi_steady.operating_point) on the grid nodes and, through
bilinear interpolation, on a dense grid (|P| 10 W, V2 0.4 V), which is what a LUT/regression controller
built on the surface will see between the nodes.

  python -I -X utf8 Simulink/Thesis/smooth_surface_v2.py [--lams ...] [--pick-gap 0.005]
"""
import argparse
import csv
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import pi_steady as ps  # noqa: E402

OUT = os.path.join(HERE, "out")
P_MAX, V2_LO, V2_HI = 1200.0, 84.0, 109.2


# ------------------------------------------------------------------ metrics
def steps(D1, D2):
    """Largest |dD| (over D1, D2) between grid neighbours, per direction, and per node (for maps)."""
    dP = np.maximum(np.abs(np.diff(D1, axis=1)), np.abs(np.diff(D2, axis=1)))     # (nV, nP-1)
    dV = np.maximum(np.abs(np.diff(D1, axis=0)), np.abs(np.diff(D2, axis=0)))     # (nV-1, nP)
    node = np.zeros(D1.shape)
    node[:, :-1] = np.maximum(node[:, :-1], dP)
    node[:, 1:] = np.maximum(node[:, 1:], dP)
    node[:-1, :] = np.maximum(node[:-1, :], dV)
    node[1:, :] = np.maximum(node[1:, :], dV)
    tv = sum(np.abs(np.diff(A, axis=k)).sum() for A in (D1, D2) for k in (0, 1))
    # largest second difference (kink) per direction, in grid units
    d2P = max(np.abs(np.diff(A, 2, axis=1)).max() for A in (D1, D2))
    d2V = max(np.abs(np.diff(A, 2, axis=0)).max() for A in (D1, D2))
    return dict(max_dP=float(dP.max()), max_dV=float(dV.max()), max_step=float(max(dP.max(), dV.max())),
                max_d2P=float(d2P), max_d2V=float(d2V), tv=float(tv), node=node)


def exact_eval(P_abs, V2, D1, D2):
    r = ps.operating_point(P_abs, D1, D2, V2)
    s = ps.sps_reference(P_abs, V2)
    return r["Ipk"], s["Ipk"], r["ok"]


def bilinear(T, P_axis, V_axis, p, v):
    ip = np.clip(np.searchsorted(P_axis, p) - 1, 0, P_axis.size - 2)
    iv = np.clip(np.searchsorted(V_axis, v) - 1, 0, V_axis.size - 2)
    tp = (p - P_axis[ip]) / (P_axis[ip + 1] - P_axis[ip])
    tv = (v - V_axis[iv]) / (V_axis[iv + 1] - V_axis[iv])
    a = T[iv, ip] * (1 - tp) + T[iv, ip + 1] * tp
    b = T[iv + 1, ip] * (1 - tp) + T[iv + 1, ip + 1] * tp
    return a * (1 - tv) + b * tv


def dense_grid():
    p = np.arange(0.0, P_MAX + 1e-9, 10.0)
    v = np.round(np.arange(V2_LO, V2_HI + 1e-9, 0.4), 4)
    PP, VV = np.meshgrid(p, v)
    return p, v, PP, VV


def surface_report(name, D1, D2, P_axis, V_axis, opt_Ipk, dense):
    """Exact node metrics + dense bilinear metrics. dI = Ipk_SPS - Ipk (> 0 is a reduction)."""
    PP, VV = np.meshgrid(P_axis, V_axis)
    ipk, isps, ok = exact_eval(PP.ravel(), VV.ravel(), D1.ravel(), D2.ravel())
    ipk, isps = ipk.reshape(D1.shape), isps.reshape(D1.shape)
    gap = ipk - opt_Ipk
    dI = isps - ipk
    p, v, DP, DV = dense
    d1 = bilinear(D1, P_axis, V_axis, DP.ravel(), DV.ravel())
    d2 = bilinear(D2, P_axis, V_axis, DP.ravel(), DV.ravel())
    jd, jsps, jok = exact_eval(DP.ravel(), DV.ravel(), d1, d2)
    ddI = (jsps - jd).reshape(DP.shape)
    st = steps(D1, D2)
    hP, hV = 25.0 / P_MAX, 1.2 / (V2_HI - V2_LO)
    rep = dict(name=name, node_infeasible=int((~ok).sum()), node_mean_dI=float(np.nanmean(dI)),
               node_max_dI=float(np.nanmax(dI)), node_min_dI=float(np.nanmin(dI)),
               node_degraded=int(np.sum(dI < -0.005)), node_gap_mean=float(np.nanmean(gap)),
               node_gap_max=float(np.nanmax(gap)),
               dense_infeasible=int((~jok).sum()), dense_mean_dI=float(np.nanmean(ddI)),
               dense_min_dI=float(np.nanmin(ddI)), dense_degraded=int(np.sum(ddI < -0.005)),
               dense_degraded_any=int(np.sum(ddI < -1e-4)),
               max_dP=st["max_dP"], max_dV=st["max_dV"], max_step=st["max_step"],
               max_grad_P=st["max_dP"] / hP, max_grad_V=st["max_dV"] / hV,
               max_d2P=st["max_d2P"], max_d2V=st["max_d2V"], tv=st["tv"])
    return rep, dict(dI=dI, gap=gap, node_step=st["node"], dense_dI=ddI)


# ------------------------------------------------------------------ optimiser
def build_unary(L, opt, mu=0.002, sps_tol=1e-6):
    Ipk = L["Ipk"].astype(np.float64)                              # (nV, nP, C)
    Irms = L["Irms"].astype(np.float64)
    gap = Ipk - opt["Ipk"][..., None]
    U = gap + mu * Irms / opt["Irms_sps"][..., None]
    sps_c = int(np.flatnonzero((L["D1"] == 0.5) & (L["D2"] == 0.5))[0])
    bad = ~np.isfinite(Ipk) | (Ipk > opt["Ipk_sps"][..., None] + sps_tol)
    bad[..., sps_c] = False                                         # SPS is always admissible
    U[bad] = np.inf
    return U, sps_c


def make_stencils(order, hP, hV):
    """Quadratic roughness  sum_stencils w * |sum_k c_k D[iv+dv_k, ip+dp_k]|^2  in model coordinates.
    order 1: Dirichlet  |D_u|^2 + |D_v|^2      (penalises every slope, also the natural light-load ramp)
    order 2: thin plate |D_uu|^2 + 2|D_uv|^2 + |D_vv|^2   (linear ramps are free, kinks and jumps are not)"""
    if order == 1:
        return [([(0, 0, -1.0), (0, 1, 1.0)], 1.0 / hP ** 2),
                ([(0, 0, -1.0), (1, 0, 1.0)], 1.0 / hV ** 2)]
    return [([(0, 0, 1.0), (0, 1, -2.0), (0, 2, 1.0)], 1.0 / hP ** 4),
            ([(0, 0, 1.0), (1, 0, -2.0), (2, 0, 1.0)], 1.0 / hV ** 4),
            ([(0, 0, 1.0), (0, 1, -1.0), (1, 0, -1.0), (1, 1, 1.0)], 2.0 / (hP * hV) ** 2)]


def quad_coeffs(D, stencils):
    """Penalty as a function of one node's D with all others fixed:  A_i |D_i|^2 - 2 B_i . D_i + const."""
    nV, nP, _ = D.shape
    A, B = np.zeros((nV, nP)), np.zeros((nV, nP, 2))
    for st, w in stencils:
        ev = max(o[0] for o in st) + 1
        ep = max(o[1] for o in st) + 1
        mv, mp = nV - ev + 1, nP - ep + 1
        T = sum(c * D[dv:dv + mv, dp:dp + mp] for dv, dp, c in st)
        for dv, dp, c in st:
            Dn = D[dv:dv + mv, dp:dp + mp]
            A[dv:dv + mv, dp:dp + mp] += w * c * c
            B[dv:dv + mv, dp:dp + mp] -= w * c * (T - c * Dn)
    return A, B


def penalty(D, stencils):
    nV, nP, _ = D.shape
    tot = 0.0
    for st, w in stencils:
        ev = max(o[0] for o in st) + 1
        ep = max(o[1] for o in st) + 1
        T = sum(c * D[dv:dv + nV - ev + 1, dp:dp + nP - ep + 1] for dv, dp, c in st)
        tot += w * float((T ** 2).sum())
    return tot


def icm(U, Dc, sel, lam, stencils, max_sweeps=300):
    """ICM with a 3x3 colouring (nodes of one colour never share a stencil, extent <= 3), so each colour
    is updated exactly and in parallel. Dc (C, 2) candidate D values; sel (nV, nP) initial labels."""
    nV, nP, C = U.shape
    sq = (Dc ** 2).sum(1)                                           # (C,)
    colour = (np.arange(nV)[:, None] % 3) * 3 + (np.arange(nP)[None, :] % 3)
    for sweep in range(max_sweeps):
        changed = 0
        for col in range(9):
            A, B = quad_coeffs(Dc[sel], stencils)
            m = colour == col
            E = U[m] + lam * (A[m][:, None] * sq[None, :] - 2.0 * B[m] @ Dc.T)
            new = np.argmin(E, axis=1)
            changed += int(np.sum(new != sel[m]))
            sel[m] = new
        if changed == 0:
            break
    return sel, sweep + 1


def energy(U, Dc, sel, lam, stencils):
    un = np.take_along_axis(U, sel[..., None], 2)[..., 0].sum()
    return float(un + lam * penalty(Dc[sel], stencils))


# ------------------------------------------------------------------ figures
def fig_surfaces(path, P, V, surfs, maps):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(surfs)
    fig, ax = plt.subplots(n, 4, figsize=(17, 3.6 * n), constrained_layout=True, squeeze=False)
    for r, (name, D1, D2) in enumerate(surfs):
        m = maps[r]
        for c, (A, title, cmap, vmin, vmax) in enumerate((
                (D1, "D1", "Blues_r", 0.1, 0.5), (D2, "D2", "Blues_r", 0.1, 0.5),
                (m["node_step"], "max |dD| to a neighbour", "Oranges", 0.0, 0.25),
                (m["dI"], "Ipk reduction vs SPS [A]", "Greens", 0.0, 1.6))):
            im = ax[r, c].pcolormesh(P, V, A, cmap=cmap, vmin=vmin, vmax=vmax, shading="nearest")
            ax[r, c].axhline(95.0, color="0.4", lw=0.8, ls="--")
            ax[r, c].set_title(f"{name}: {title}", fontsize=10)
            ax[r, c].set_xlabel("|P| [W]")
            ax[r, c].set_ylabel("V2 [V]")
            fig.colorbar(im, ax=ax[r, c], shrink=0.85)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_pareto(path, fronts):
    """fronts: [(label, rows, picked_lam or None)]; x = mean gap, y = max step / max kink / max gap."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colours = ["#2a6fb0", "#c26a1b", "#4a8a3a"]
    markers = ["o", "s", "^"]
    fig, ax = plt.subplots(1, 3, figsize=(15, 4), constrained_layout=True)
    panels = (("max_step", "max adjacent |dD| (grid step)"),
              ("max_d2", "max |second difference of D| (grid)"),
              ("node_gap_max", "max gap to pointwise optimum [A]"))
    for (label, rows, picked), col, mk in zip(fronts, colours, markers):
        g = np.array([r["node_gap_mean"] for r in rows]) * 1e3
        lam = [r["lam"] for r in rows]
        for a, (key, _) in zip(ax, panels):
            y = np.array([max(r["max_d2P"], r["max_d2V"]) if key == "max_d2" else r[key] for r in rows])
            a.plot(g, y, "-" + mk, color=col, lw=2, ms=6, label=label)
            if picked is not None:
                ip = lam.index(picked)
                a.plot(g[ip], y[ip], "o", ms=13, mfc="none", mec="0.15", mew=1.5)
                a.annotate(f"lam={picked:g}", (g[ip], y[ip]), textcoords="offset points", xytext=(8, 6),
                           fontsize=8, color="0.25")
    for a, (_, yl) in zip(ax, panels):
        a.set_xscale("symlog", linthresh=0.1)
        a.set_xlabel("mean gap to pointwise optimum [mA]")
        a.set_ylabel(yl)
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8, frameon=False)
    ax[0].set_title("smoothness vs optimality, one marker per lambda", fontsize=9)
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_dense(path, p, v, dI_list, names):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, len(dI_list), figsize=(5.6 * len(dI_list), 4), constrained_layout=True, squeeze=False)
    for a, dI, nm in zip(ax[0], dI_list, names):
        lim = 0.05
        im = a.pcolormesh(p, v, np.clip(dI, -lim, lim), cmap="RdBu", vmin=-lim, vmax=lim, shading="nearest")
        neg = dI < -0.005
        if neg.any():
            PP, VV = np.meshgrid(p, v)
            a.plot(PP[neg], VV[neg], "kx", ms=3)
        a.set_title(f"{nm}: dense dI = Ipk_SPS - Ipk, clipped +-{lim} A\n(x: worse than SPS by > 5 mA)",
                    fontsize=9)
        a.set_xlabel("|P| [W]")
        a.set_ylabel("V2 [V]")
        fig.colorbar(im, ax=a, shrink=0.85)
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default="v1")
    ap.add_argument("--tag", default="v2")
    ap.add_argument("--lams", default="1e-6,3e-6,1e-5,3e-5,1e-4,2e-4,5e-4,1e-3,2e-3,5e-3")
    ap.add_argument("--mu", type=float, default=0.002)
    ap.add_argument("--order", type=int, default=2, choices=(1, 2))
    ap.add_argument("--pick-gap", type=float, default=0.005, help="largest lam with node mean gap <= this [A]")
    ap.add_argument("--pick-lam", type=float, default=None, help="override the pick rule")
    ap.add_argument("--compare", default=None, help="TAG of another sweep to overlay on the Pareto figure")
    a = ap.parse_args()
    t0 = time.time()
    L = dict(np.load(os.path.join(OUT, f"landscape_{a.src}.npz")))
    S = dict(np.load(os.path.join(OUT, f"surface_{a.src}.npz")))
    opt = {k[4:]: v for k, v in S.items() if k.startswith("opt_")}
    P, V = S["P"], S["V2"]
    hP, hV = (P[1] - P[0]) / P_MAX, (V[1] - V[0]) / (V2_HI - V2_LO)
    stencils = make_stencils(a.order, hP, hV)
    U, sps_c = build_unary(L, opt, a.mu)
    Dc = np.stack([L["D1"], L["D2"]], 1)
    dense = dense_grid()
    print(f"grid {V.size} x {P.size}, lattice {Dc.shape[0]} candidates, roughness order {a.order}", flush=True)

    rows, surfs = [], {}
    rep, mp = surface_report("v1-opt", S["opt_D1"], S["opt_D2"], P, V, opt["Ipk"], dense)
    rep["lam"] = 0.0
    rows.append(rep)
    rep, mp = surface_report("v1-smooth", S["smooth_D1"], S["smooth_D2"], P, V, opt["Ipk"], dense)
    rep["lam"] = -1.0
    rows.append(rep)
    surfs["v1-smooth"] = (S["smooth_D1"], S["smooth_D2"], mp)
    # continuation: start from the unary argmin, increase lam
    sel = np.argmin(U, axis=2)
    for lam in [float(x) for x in a.lams.split(",")]:
        # also try the v1 smooth field as a start; keep the lower energy
        cand = [sel.copy()]
        v1sel = np.argmin((Dc[None, None, :, 0] - S["smooth_D1"][..., None]) ** 2
                          + (Dc[None, None, :, 1] - S["smooth_D2"][..., None]) ** 2, axis=2)
        cand.append(v1sel)
        best = None
        for s0 in cand:
            s1, nsw = icm(U, Dc, s0, lam, stencils)
            e = energy(U, Dc, s1, lam, stencils)
            if best is None or e < best[0]:
                best = (e, s1, nsw)
        e, sel, nsw = best
        D1, D2 = Dc[sel, 0], Dc[sel, 1]
        rep, mp = surface_report(f"lam={lam:g}", D1, D2, P, V, opt["Ipk"], dense)
        rep["lam"] = lam
        rep["energy"] = e
        rows.append(rep)
        surfs[lam] = (D1, D2, mp)
        print(f"lam {lam:8.1e}: E {e:9.3f} ({nsw} sweeps)  gap mean {rep['node_gap_mean'] * 1e3:6.2f} mA "
              f"max {rep['node_gap_max'] * 1e3:6.1f} mA | step max {rep['max_step']:.3f} (P {rep['max_dP']:.3f}, "
              f"V {rep['max_dV']:.3f}) kink P {rep['max_d2P']:.3f} V {rep['max_d2V']:.3f} TV {rep['tv']:6.2f} "
              f"| node dI mean {rep['node_mean_dI']:.4f} "
              f"min {rep['node_min_dI'] * 1e3:+.2f} mA | dense dI mean {rep['dense_mean_dI']:.4f} "
              f"min {rep['dense_min_dI'] * 1e3:+.2f} mA, degraded {rep['dense_degraded']} "
              f"({time.time() - t0:.0f} s)", flush=True)

    keys = ["name", "lam", "node_gap_mean", "node_gap_max", "node_mean_dI", "node_max_dI", "node_min_dI",
            "node_degraded", "node_infeasible", "dense_mean_dI", "dense_min_dI", "dense_degraded",
            "dense_degraded_any", "dense_infeasible", "max_step", "max_dP", "max_dV", "max_grad_P",
            "max_grad_V", "max_d2P", "max_d2V", "tv"]
    with open(os.path.join(OUT, f"surface_{a.tag}_sweep.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for r in rows:
            w.writerow([r[k] if isinstance(r[k], str) else f"{r[k]:.6g}" for k in keys])

    lam_rows = [r for r in rows if r["lam"] > 0]
    ok = [r for r in lam_rows if r["node_gap_mean"] <= a.pick_gap and r["node_degraded"] == 0]
    picked = (ok[-1] if ok else lam_rows[0])["lam"] if a.pick_lam is None else a.pick_lam
    D1, D2, mp = surfs[picked]
    print(f"\npicked lam = {picked:g}")
    # every lam surface, so the regression stage can choose the target by learnability
    lams = np.array([r["lam"] for r in lam_rows])
    np.savez_compressed(os.path.join(OUT, f"surface_{a.tag}_sweep.npz"), P=P, V2=V, lam=lams,
                        D1=np.stack([surfs[l][0] for l in lams]), D2=np.stack([surfs[l][1] for l in lams]),
                        dI=np.stack([surfs[l][2]["dI"] for l in lams]), gap=np.stack([surfs[l][2]["gap"] for l in lams]))
    np.savez_compressed(os.path.join(OUT, f"surface_{a.tag}.npz"), P=P, V2=V,
                        **{f"opt_{k}": v for k, v in opt.items()},
                        smooth_D1=D1, smooth_D2=D2, smooth_dI=mp["dI"], smooth_gap=mp["gap"], lam=picked,
                        mu=a.mu, order=a.order)
    fig_surfaces(os.path.join(OUT, f"fig_surface_v1_vs_{a.tag}.png"), P, V,
                 [("v1", S["smooth_D1"], S["smooth_D2"]), (f"{a.tag} (lam={picked:g})", D1, D2)],
                 [surfs["v1-smooth"][2], mp])
    fronts = [(f"{a.tag} (order {a.order})", lam_rows, picked)]
    if a.compare:
        with open(os.path.join(OUT, f"surface_{a.compare}_sweep.csv"), encoding="utf-8") as f:
            other = [{k: (v if k == "name" else float(v)) for k, v in r.items()} for r in csv.DictReader(f)]
        other = [r for r in other if r["lam"] > 0]
        fronts.append((a.compare, other, None))
    fig_pareto(os.path.join(OUT, f"fig_surface_{a.tag}_pareto.png"), fronts)
    p, v, _, _ = dense
    fig_dense(os.path.join(OUT, f"fig_surface_{a.tag}_dense_dI.png"), p, v,
              [surfs["v1-smooth"][2]["dense_dI"], mp["dense_dI"]], ["v1 smooth (bilinear)", f"{a.tag} (bilinear)"])
    for r in rows:
        if r["lam"] in (-1.0, 0.0, picked):
            print({k: (round(r[k], 5) if not isinstance(r[k], str) else r[k]) for k in keys})
    print(f"done in {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
