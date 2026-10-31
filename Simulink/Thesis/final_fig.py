"""
final_fig.py -- the figures of FINAL_REPORT.md, from the map's and the edge runs' tables (no new values, no fit).

  out/r13_points_med.csv   25 ns map, 7 V2 x 8 |P| (100-1000 W), two 2.88 deg levels; max_* = largest of the 80 period peaks, osc_* = |mean i_L| of the worse level
  out/edge25_check.csv     25 ns, 25, 50, 75, 1200 W, two 0.72 deg levels (set C = its control points on the map, left out here)
  "not clean" = |mean i_L| > 20 mA in the SPS or the proposed run, as every rule of the study: no value is drawn.

    python -I -X utf8 Simulink/Thesis/final_fig.py   -> out/fig_final_map.png, out/fig_final_ipk.png, out/fig_final.log
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SPS_C, PROP_C = "#2a78d6", "#eb6834"
MID = "#f0efec"
OSC = 0.020
V2S = (84.0, 88.2, 92.4, 96.6, 100.8, 105.0, 109.2)

plt.rcParams.update({"font.family": ["Malgun Gothic", "DejaVu Sans"], "axes.unicode_minus": False, "font.size": 9,
                     "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "text.color": INK, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE})


def table():
    """rows: V2, P, SPS, B, med_SPS, med_B, clean, grid"""
    m = pd.read_csv(os.path.join(OUT, "r13_points_med.csv"))
    a = pd.DataFrame(dict(V2=m.V2, P=m.P, SPS=m.max_SPS, B=m.max_MB, med_SPS=m.med_SPS, med_B=m.med_MB,
                          clean=(m.osc_SPS <= OSC) & (m.osc_MB <= OSC), levels="2.88"))
    e = pd.read_csv(os.path.join(OUT, "edge25_check.csv"))
    e = e[e["set"] != "C"]
    b = pd.DataFrame(dict(V2=e.V2, P=e.P, SPS=e.SPS, B=e.MB, med_SPS=e.med_SPS, med_B=e.med_MB,
                          clean=(e.osc_SPS <= OSC) & (e.osc_MB <= OSC), levels="0.72"))
    return pd.concat([a, b], ignore_index=True).sort_values(["V2", "P"]).reset_index(drop=True)


def fig_map(T, log):
    Ps = sorted(T.P.unique())
    Z = np.full((len(V2S), len(Ps)), np.nan)
    for _, r in T.iterrows():
        if r.clean:
            Z[V2S.index(r.V2), Ps.index(r.P)] = 100.0 * (r.SPS - r.B) / r.SPS
    cmap = LinearSegmentedColormap.from_list("rb", ["#e34948", MID, "#2a78d6"])
    norm = TwoSlopeNorm(vmin=-70, vcenter=0, vmax=70)
    fig, ax = plt.subplots(figsize=(7.4, 3.3), dpi=200)
    im = ax.imshow(Z, cmap=cmap, norm=norm, aspect="auto", origin="lower")
    for i, v in enumerate(V2S):
        for j, p in enumerate(Ps):
            if np.isfinite(Z[i, j]):
                ax.text(j, i, f"{Z[i, j]:+.0f}" if abs(Z[i, j]) >= 0.5 else "0", ha="center", va="center", fontsize=8, color="#ffffff" if abs(Z[i, j]) >= 55 else INK)
            else:
                ax.text(j, i, "불가", ha="center", va="center", fontsize=7, color=MUTED)
    ax.set_xticks(range(len(Ps)), [f"{p:g}" for p in Ps])
    ax.set_yticks(range(len(V2S)), [f"{v:g} V (k {4 * v / 380:.3f})" for v in V2S])
    ax.set_xlabel("|P| [W]")
    ax.tick_params(length=0)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_xticks(np.arange(-0.5, len(Ps)), minor=True)
    ax.set_yticks(np.arange(-0.5, len(V2S)), minor=True)
    ax.grid(which="minor", color=SURFACE, lw=1.5)
    ax.tick_params(which="minor", length=0)
    cb = fig.colorbar(im, ax=ax, fraction=0.03, pad=0.015, ticks=[-70, -35, 0, 35, 70])
    cb.set_label("피크 저감 [%], 양수 = 제안이 낮음", color=INK2)
    cb.outline.set_visible(False)
    cb.ax.tick_params(colors=MUTED, length=0)
    fig.tight_layout()
    p = os.path.join(OUT, "fig_final_map.png")
    fig.savefig(p)
    plt.close(fig)
    log.append(f"fig_final_map: {np.isfinite(Z).sum()} values, {np.isnan(Z).sum()} not clean; min {np.nanmin(Z):+.1f} %, max {np.nanmax(Z):+.1f} %")


def fig_ipk(T, log):
    show = (84.0, 96.6, 109.2)
    fig, axs = plt.subplots(1, 3, figsize=(7.4, 2.5), dpi=200, sharey=True)
    for ax, v in zip(axs, show):
        g = T[(T.V2 == v) & T.clean].sort_values("P")
        ax.plot(g.P, g.SPS, color=SPS_C, lw=2, marker="o", ms=4, mec=SURFACE, mew=1, label="SPS")
        ax.plot(g.P, g.B, color=PROP_C, lw=2, marker="o", ms=4, mec=SURFACE, mew=1, label="제안 제어기")
        ax.set_xscale("log")
        ax.set_xticks([25, 100, 300, 1200], ["25", "100", "300", "1200"])
        ax.minorticks_off()
        ax.set_xlabel("|P| [W], 로그 축")
        ax.set_title(f"V2 {v:g} V (k {4 * v / 380:.3f})", fontsize=9, color=INK2, loc="left")
        ax.grid(axis="y", color=GRID, lw=0.6)
        ax.set_ylim(0, 5.8)
        ax.tick_params(length=0)
        for s in ("top", "right", "left"):
            ax.spines[s].set_visible(False)
        log.append(f"fig_final_ipk {v:g} V: " + ", ".join(f"{r.P:g} W {r.SPS:.3f}/{r.B:.3f}" for _, r in g.iterrows()))
    axs[0].set_ylabel("Ipk [A], 80주기 최대")
    axs[0].legend(frameon=False, loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_final_ipk.png"))
    plt.close(fig)


def main():
    T = table()
    log = [f"final_fig: {len(T)} points, not clean: " + ", ".join(f"{r.V2:g} V {r.P:g} W" for _, r in T[~T.clean].iterrows())]
    fig_map(T, log)
    fig_ipk(T, log)
    open(os.path.join(OUT, "fig_final.log"), "w", encoding="utf-8").write("\n".join(log) + "\n")
    print("\n".join(log))


if __name__ == "__main__":
    main()
