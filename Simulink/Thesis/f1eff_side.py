"""
f1eff_side.py -- (2026-11-01) values beside the f1eff verdict, NOT a verdict; the rules in f1_eff.py are untouched and nothing here changes their answers.
What it adds: Q5 at 6.25 ns per controller, the same band on L(0); the same-input Ipk, rms, |P| per grid; the Efficiency output per grid and in each run's real power direction; the analyzer's device V x I sum per grid and against L(0);
the two-level interpolation width and a chord-against-parabola estimate; de from the conduction part with 40 and 52 mohm; exact de values.

    python -I -X utf8 Simulink/Thesis/f1eff_side.py     -> out/f1eff_side.log
"""
import builtins
import os
import sys

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)
import f1_eff as fe  # noqa: E402

_log = open(os.path.join(ROOT, "out", "f1eff_side.log"), "w", encoding="utf-8")


def print(*a, **k):  # noqa: A001
    builtins.print(*a, **k)
    builtins.print(*a, file=_log, **k)


# ---------------------------------------------------------------- part: f1eff_side.py
d = fe.load()
waves, spread = fe._waves(), fe._spread()
S = d[np.isclose(d["Ts"], d["Ts_solver"], rtol=1e-6, atol=0)].copy()
S = S.assign(P_D=[fe.diode_power(waves.get(str(r.label)), r.D0, r.D1, r.D2) for r in S.itertuples()], spread=[spread.get(str(r.label), np.nan) for r in S.itertuples()])
S = S.assign(P_R40=fe.R40 * S["iL_rms"] ** 2, P_R52=fe.R52 * S["iL_rms"] ** 2)
print("columns", list(d.columns))
print("runs total", len(d), "S", len(S), "per grid", {fe.GRID_NAME[t]: len(fe.at_grid(S, t)) for t in fe.GRIDS}, "X", len(d) - len(S))

# ---- Q5 at 6.25 ns exactly, per controller and power; L(0) against P_R + P_D
key = ["ctrl", "V2", "P_target", "dirn", "level"]
g6 = fe.at_grid(S, fe.T6).set_index(key)
g12 = fe.at_grid(S, fe.T12).set_index(key)
J = g6.join(g12[["L", "iL_rms"]], rsuffix="_12")
J = J.assign(L0=2 * J["L"] - J["L_12"], PRD40=J["P_R40"] + J["P_D"], PRD52=J["P_R52"] + J["P_D"])
J = J.assign(r6_40=J["L"] / J["PRD40"], r6_52=J["L"] / J["PRD52"], r0_40=J["L0"] / J["PRD40"], r0_52=J["L0"] / J["PRD52"])
J = J.assign(ok6=(J["L"] >= 0.9 * J["PRD40"]) & (J["L"] <= 1.1 * J["PRD52"]), ok0=(J["L0"] >= 0.9 * J["PRD40"]) & (J["L0"] <= 1.1 * J["PRD52"]))
Q = J[J["L"] > 0.5].reset_index()
print("\nQ5 at 6.25 ns", int(Q["ok6"].sum()), "of", len(Q), "| same band applied to L(0) (reported, not the rule):", int(Q["ok0"].sum()), "of", len(Q))
for (c, p), x in Q.groupby(["ctrl", "P_target"]):
    print(f"  {c:5s} {p:5.0f} W: ok {int(x['ok6'].sum())}/{len(x)}; L/(R40+D) {x['r6_40'].min():.2f}...{x['r6_40'].max():.2f}; L/(R52+D) {x['r6_52'].min():.2f}...{x['r6_52'].max():.2f}; L0/(R40+D) {x['r0_40'].min():.2f}...{x['r0_40'].max():.2f}; L0/(R52+D) {x['r0_52'].min():.2f}...{x['r0_52'].max():.2f}")
print("  all L0/(R40+D) median", round(Q["r0_40"].median(), 3), "range", round(Q["r0_40"].min(), 3), round(Q["r0_40"].max(), 3), "; L0/(R52+D) median", round(Q["r0_52"].median(), 3), "range", round(Q["r0_52"].min(), 3), round(Q["r0_52"].max(), 3))
print("  all L/(R52+D) at 6.25 median", round(Q["r6_52"].median(), 3), " L/(R40+D) median", round(Q["r6_40"].median(), 3))

# ---- the Q1 rows that stay above 15 %
T = pd.read_csv(os.path.join(ROOT, "out", "f1eff_check.csv"))
x = T[T["L12.5"] > 0.5].copy()
x["r"] = x["L6.25"] / x["L12.5"]
print("\nQ1 rows, smallest r:")
print(x.sort_values("r")[["ctrl", "V2", "P_target", "dirn", "level", "L12.5", "L6.25", "r"]].head(6).to_string(index=False))
print("  by controller and power, median r:")
print(x.groupby(["ctrl", "P_target"])["r"].median().round(3).to_string())
# differences L(25) - L(12.5) and L(12.5) - L(6.25)
y = T[(T["L12.5"] - T["L6.25"]).abs() >= 0.1]
a = (y["L25"] - y["L12.5"]) / (y["L12.5"] - y["L6.25"])
print("Q7 a: median", round(a.median(), 3), "range", round(a.min(), 3), round(a.max(), 3), "n", len(a))

# ---- Ipk, rms on the same input on three grids (the same gate timing at 25, 12.5, 6.25 ns)
print("\nsame input, the same timing at 25, 12.5, 6.25 ns (P at 6.25 ns as written in the table):")
z = T[T["L25"].notna() & T["L6.25"].notna()]
for col, name in (("Ipk", "Ipk [A]"), ("rms", "i_L rms [A]"), ("Pd", "|P| [W]")):
    a, b, c = z[col + "25"], z[col + "12.5"], z[col + "6.25"]
    print(f"  {name}: |25 - 6.25| median {abs(a - c).median():.4f} max {abs(a - c).max():.4f}; |12.5 - 6.25| median {abs(b - c).median():.4f} max {abs(b - c).max():.4f}; relative |25 - 6.25| / 6.25 median {(abs(a - c) / abs(c)).median() * 100:.2f} % max {(abs(a - c) / abs(c)).max() * 100:.2f} %")
print("  rows", len(z))
print(z.assign(dI=z["Ipk25"] - z["Ipk6.25"], dR=z["rms25"] - z["rms6.25"], dP=z["Pd25"] - z["Pd6.25"]).sort_values("dI", key=abs, ascending=False)[["ctrl", "V2", "P_target", "dirn", "level", "Ipk25", "Ipk12.5", "Ipk6.25", "rms25", "rms6.25", "Pd25", "Pd6.25"]].head(6).round(3).to_string(index=False))

# ---- the Efficiency output
print("\nEff_out against the direct ratio, per grid and direction")
S2 = S.assign(dEff=S["Eff_out"] - 100 * S["eta_m"])
for t in fe.GRIDS:
    for dn in fe.DIRS:
        Y = fe.at_grid(S2, t)
        Y = Y[Y["dirn"] == dn]
        Pa = np.abs(Y["P_LV"]) if dn > 0 else Y["P_HV"]
        est = np.minimum(99.9, 100.0 * (Pa - Y["Loss_out"]) / Pa)
        print(f"  {fe.GRID_NAME[t]:>5s} ns {fe.DIR_NAME[dn]}: n {len(Y):3d}, Eff_out - eta_m median {Y['dEff'].median():+.2f}, range {Y['dEff'].min():+.2f} ... {Y['dEff'].max():+.2f} pp; |.| > 0.5 pp: {int((Y['dEff'].abs() > 0.5).sum())}; matches the estimate within 0.1 pp: {int((np.abs(Y['Eff_out'] - est) <= 0.1).sum())}; Eff_out range {Y['Eff_out'].min():.2f}...{Y['Eff_out'].max():.2f}")
Y = S2[~np.isclose(S2["Ts"], fe.T100, rtol=1e-6, atol=0)]
for dn in fe.DIRS:
    W = Y[Y["dirn"] == dn]
    Pa = np.abs(W["P_LV"]) if dn > 0 else W["P_HV"]
    est = np.minimum(99.9, 100.0 * (Pa - W["Loss_out"]) / Pa)
    off = W[np.abs(W["Eff_out"] - est) > 0.1]
    print("  not the estimate,", fe.DIR_NAME[dn])
    print(off.assign(est=est[off.index])[["label", "P_LV", "P_HV", "Loss_out", "Eff_out", "est", "eta_m"]].round(3).to_string(index=False))
    far = W[W["dEff"].abs() > 2]
    print("  more than 2 pp off:")
    print(far[["label", "P_LV", "P_HV", "Loss_out", "Eff_out", "eta_m", "dEff"]].round(3).to_string(index=False))
# Eff_out at the 6 points as a user would read it (interpolated at |P|, 100 ns and 6.25 ns)
print("\nEff_out at |P| against eta from L, per grid (interpolated on the same two levels)")
for V2, P in fe.PTS:
    for dn in fe.DIRS:
        row = []
        for c in fe.CTRLS:
            cfg = [q for q in fe.configs() if q["ctrl"] == c and q["V2"] == V2 and q["P"] == P and q["dirn"] == dn][0]
            for t in (fe.T100, fe.T25, fe.T6):
                g = fe.runs(S2, cfg, t)
                e_o = fe.at_power(g, P, "Eff_out")
                e_m = fe.at_power(g, P, "eta_m") * 100
                row.append(f"{c} {fe.GRID_NAME[t]}: {e_o:6.2f} vs {e_m:6.2f}")
        print(f"  {fe.DIR_NAME[dn]} {V2:6.1f} V {P:5.0f} W | " + " | ".join(row))

# ---- level spacing used for eta at |P|: how wide the interpolation is at 6.25 ns
print("\ninterpolation width at 6.25 ns (the two levels used) and the chord error if L = a + c P^2 between them (reported only)")
for V2, P in fe.PTS:
    for dn in fe.DIRS:
        for c in fe.CTRLS:
            cfg = [q for q in fe.configs() if q["ctrl"] == c and q["V2"] == V2 and q["P"] == P and q["dirn"] == dn][0]
            g = fe.runs(S, cfg, fe.T6)
            m, Pd, L = g["m"].to_numpy(), g["Pd"].to_numpy(), g["L"].to_numpy()
            for i in range(len(g) - 1):
                if m[i + 1] == m[i] + 1 and min(Pd[i], Pd[i + 1]) <= P <= max(Pd[i], Pd[i + 1]):
                    chord = L[i] + (P - Pd[i]) / (Pd[i + 1] - Pd[i]) * (L[i + 1] - L[i])
                    k = (L[i + 1] - L[i]) / (Pd[i + 1] ** 2 - Pd[i] ** 2)
                    para = L[i] + k * (P ** 2 - Pd[i] ** 2)
                    print(f"  {fe.DIR_NAME[dn]} {V2:6.1f} V {P:5.0f} W {c:5s}: P {Pd[i]:7.1f} ... {Pd[i + 1]:7.1f} W, L {L[i]:6.2f} ... {L[i + 1]:6.2f} W, chord {chord:6.2f}, parabola {para:6.2f} W, diff {chord - para:+.3f} W = {100 * (chord - para) / P:+.3f} pp")
                    break

# ---- de with R40 and R52 (the conduction part alone) and de from L(0)
print("\nde from the conduction part alone at 6.25 ns, R 40 mohm and 52 mohm + P_D [pp], beside de(6.25) and de(0)")
E = pd.read_csv(os.path.join(ROOT, "out", "f1eff_eta.csv"))
for r in E.itertuples():
    out = []
    for name, col in (("R40", "P_R40"), ("R52", "P_R52")):
        ll = []
        for c in fe.CTRLS:
            cfg = [q for q in fe.configs() if q["ctrl"] == c and q["V2"] == r.V2 and q["P"] == r.P and q["dirn"] == r.dirn][0]
            g = fe.runs(S.assign(X=S[col] + S["P_D"]), cfg, fe.T6)
            ll.append(fe.at_power(g, r.P, "X"))
        out.append(f"{name}: SPS {ll[0]:.2f} B {ll[1]:.2f} W, de {100 * (fe.eta_at(r.P, ll[1], r.dirn) - fe.eta_at(r.P, ll[0], r.dirn)):+.2f}")
    print(f"  {fe.DIR_NAME[r.dirn]} {r.V2:6.1f} V {r.P:5.0f} W | " + " | ".join(out) + f" | de(6.25) {getattr(r, '_' + str(list(E.columns).index('de_6.25') + 1)) if False else E.loc[r.Index, 'de_6.25']:+.2f}, de(0) {r.de_0:+.2f}, L0 SPS {r.L0_SPS:.2f} B {r.L0_B:.2f} W")
print("\nnon-periodic rows per grid, by controller (80-period spread > 5 mA):")
for t in fe.GRIDS:
    Y = fe.at_grid(S, t)
    print(" ", fe.GRID_NAME[t], {c: f"{int((Y[Y['ctrl'] == c]['spread'] > 0.005).sum())}/{int((Y['ctrl'] == c).sum())}" for c in fe.CTRLS}, "worst spread", round(float(Y["spread"].max()), 3))
print("drift |dE| per grid:", {fe.GRID_NAME[t]: round(float(fe.at_grid(S, t)["dE"].abs().max()), 3) for t in fe.GRIDS})

# ---------------------------------------------------------------- part: f1eff_side2.py
d = fe.load()
S = d[np.isclose(d["Ts"], d["Ts_solver"], rtol=1e-6, atol=0)].copy()
F = S[~np.isclose(S["Ts"], fe.T100, rtol=1e-6, atol=0)]
# the real flow of a run: P_LV < 0 HV -> LV (P_HV < 0), P_LV > 0 LV -> HV (P_HV > 0)
real = np.where(F["P_LV"] < 0, 1, -1)
rev = real != F["dirn"].to_numpy()
print("reverse-flow runs on the fine grids:", int(rev.sum()), "of", len(F))
print(F[rev][["label", "P_LV", "P_HV", "Eff_out", "Loss_out", "eta_m"]].round(3).to_string(index=False))
Pa = np.where(real > 0, np.abs(F["P_LV"]), F["P_HV"])
est = np.minimum(99.9, 100.0 * (Pa - F["Loss_out"]) / Pa)
print("Eff_out against the estimate in the run's real direction, |.| <= 0.1 pp:", int((np.abs(F["Eff_out"] - est) <= 0.1).sum()), "of", len(F), "; largest |.|", round(float(np.abs(F["Eff_out"] - est).max()), 3))
print(F[np.abs(F["Eff_out"] - est) > 0.1].assign(est=est[np.abs(F["Eff_out"] - est) > 0.1])[["label", "P_LV", "P_HV", "Eff_out", "est", "Loss_out"]].round(3).to_string(index=False))
# the direct ratio in the real direction
eta_real = np.where(real > 0, np.abs(F["P_LV_d"]) / np.abs(F["P_HV_d"]), F["P_HV_d"] / F["P_LV_d"])
off = np.abs(F["Eff_out"] - 100 * eta_real)
print("\nEff_out - direct ratio in the real direction, per grid:")
for t in fe.FINE3:
    m = np.isclose(F["Ts"], t, rtol=1e-6, atol=0)
    x = (F["Eff_out"] - 100 * eta_real)[m]
    print(f"  {fe.GRID_NAME[t]:>5s} ns: n {int(m.sum())}, median {x.median():+.2f}, range {x.min():+.2f} ... {x.max():+.2f} pp; |.| > 0.5 pp: {int((x.abs() > 0.5).sum())}")
# Q6 (ii) without the reverse-flow runs, per labelled direction
print("\nQ6 (ii) with the reverse-flow runs removed (not the rule):")
for dn in fe.DIRS:
    W = F[(F["dirn"] == dn) & ~rev]
    w = np.abs(W["Eff_out"] - 100.0 * W["eta_m"]) > 0.5
    print(f"  {fe.DIR_NAME[dn]}: {int(w.sum())} of {len(W)} = {100 * w.mean():.1f} %")
    Pb = np.abs(W["P_LV"]) if dn > 0 else W["P_HV"]
    b = np.abs(W["Eff_out"] - np.minimum(99.9, 100.0 * (Pb - W["Loss_out"]) / Pb)) <= 0.1
    print(f"     estimate: {int(b.sum())} of {len(W)}")
# what the block's outputs add up to
print("\nLoss_out = Cu + 4 SemiHV + 4 SemiLV? (reading the block's outputs)")
x = F.assign(sum4=F["Cu_out"] + 4 * F["SemiHV_out"] + 4 * F["SemiLV_out"], sum1=F["Cu_out"] + F["SemiHV_out"] + F["SemiLV_out"])
print("  |Loss - (Cu + 4 Hv + 4 Lv)| max", round(float((x["Loss_out"] - x["sum4"]).abs().max()), 4), "; |Loss - (Cu + Hv + Lv)| max", round(float((x["Loss_out"] - x["sum1"]).abs().max()), 4))
print("  Loss_out - L: median", round(float((x["Loss_out"] - x["L"]).median()), 3), "range", round(float((x["Loss_out"] - x["L"]).min()), 3), round(float((x["Loss_out"] - x["L"]).max()), 3))
print(x[np.isclose(x["Ts"], fe.T6, rtol=1e-6, atol=0)][["label", "Loss_out", "Cu_out", "SemiHV_out", "SemiLV_out", "L", "iL_rms", "I_LV_rms", "I_HV_rms"]].round(3).head(12).to_string(index=False))
# 100 ns: the same comparison, reported (the block's output at the original grid, which the user read)
H = S[np.isclose(S["Ts"], fe.T100, rtol=1e-6, atol=0)]
realH = np.where(H["P_LV"] < 0, 1, -1)
etaH = np.where(realH > 0, np.abs(H["P_LV_d"]) / np.abs(H["P_HV_d"]), H["P_HV_d"] / H["P_LV_d"])
z = H["Eff_out"] - 100 * etaH
print("\n100 ns: Eff_out - direct ratio in the real direction, n", len(H), "median", round(float(z.median()), 2), "range", round(float(z.min()), 2), round(float(z.max()), 2), "|.| > 0.5 pp:", int((z.abs() > 0.5).sum()))
print(H.assign(eta=100 * etaH)[["label", "P_LV", "P_HV", "Eff_out", "eta", "Eff_min", "Eff_max"]].round(2).head(12).to_string(index=False))
print("\nEff_min, Eff_max of the block at 100 ns: min", round(float(H["Eff_min"].min()), 2), "max", round(float(H["Eff_max"].max()), 2))

# ---------------------------------------------------------------- part: f1eff_side3.py
d = fe.load()
S = d[np.isclose(d["Ts"], d["Ts_solver"], rtol=1e-6, atol=0)].copy()
S = S.assign(Semi=S["SemiHV_out"] + S["SemiLV_out"])
print("Semi = 4 HV switch + 4 LV switch (analyzer means), same input per grid")
key = ["ctrl", "V2", "P_target", "dirn", "level"]
rows = []
for k, g in S.groupby(key):
    v = {}
    for t in fe.FINE3:
        x = fe.at_grid(g, t)
        if len(x):
            v[t] = (float(x["Semi"].iloc[0]), float(x["L"].iloc[0]))
    if len(v) == 3:
        rows.append((k, v))
dif, r0 = [], []
for k, v in rows:
    s25, s12, s6 = v[fe.T25][0], v[fe.T12][0], v[fe.T6][0]
    L12, L6 = v[fe.T12][1], v[fe.T6][1]
    L0 = 2 * L6 - L12
    dif.append((abs(s25 - s6), abs(s12 - s6), abs(s25 - s6) / s6))
    r0.append(L0 / s6)
    print(f"  {k[0]:5s} {k[1]:6.1f} V {k[2]:5.0f} W {fe.DIR_NAME[k[3]]} L{k[4]:+3d}: Semi {s25:7.3f} {s12:7.3f} {s6:7.3f} W | L {v[fe.T25][1]:7.3f} {L12:7.3f} {L6:7.3f} W | L(0) {L0:7.3f} W | L(0) / Semi {L0 / s6:5.3f} | (L6 - Semi) / L6 {(L6 - s6) / L6:5.2f}")
dif = np.array(dif)
print("  |Semi(25) - Semi(6.25)|: median", round(float(np.median(dif[:, 0])), 4), "max", round(float(dif[:, 0].max()), 4), "W; relative median", round(float(np.median(dif[:, 2])) * 100, 2), "% max", round(float(dif[:, 2].max()) * 100, 2), "%")
print("  |Semi(12.5) - Semi(6.25)|: median", round(float(np.median(dif[:, 1])), 4), "max", round(float(dif[:, 1].max()), 4), "W")
print("  L(0) / Semi: median", round(float(np.median(r0)), 3), "range", round(float(np.min(r0)), 3), round(float(np.max(r0)), 3), "n", len(r0))
print("\nde from Semi alone per grid (eta at |P| from the same two levels) [pp], beside de from L")
by = {(c["ctrl"], c["V2"], c["P"], c["dirn"]): c for c in fe.configs()}
for V2, P in fe.PTS:
    for dn in fe.DIRS:
        out = []
        for t in fe.GRIDS:
            ls, lb = (fe.at_power(fe.runs(S, by[(c, V2, P, dn)], t), P, "Semi") for c in fe.CTRLS)
            Ls, Lb = (fe.at_power(fe.runs(S, by[(c, V2, P, dn)], t), P, "L") for c in fe.CTRLS)
            out.append(f"{fe.GRID_NAME[t]}: Semi {ls:6.2f} {lb:6.2f} W de {100 * (fe.eta_at(P, lb, dn) - fe.eta_at(P, ls, dn)):+.2f} (L {100 * (fe.eta_at(P, Lb, dn) - fe.eta_at(P, Ls, dn)):+.2f})")
        print(f"  {fe.DIR_NAME[dn]} {V2:6.1f} V {P:5.0f} W | " + " | ".join(out))

# ---------------------------------------------------------------- part: f1eff_side4.py
E = pd.read_csv(os.path.join(ROOT, "out", "f1eff_eta.csv"))
for r in E.to_dict("records"):
    print(f"{fe.DIR_NAME[int(r['dirn'])]:7s} {r['V2']:6.1f} V {r['P']:5.0f} W | de25 {r['de_25']:+.3f} de12 {r['de_12.5']:+.3f} de6 {r['de_6.25']:+.3f} de0 {r['de_0']:+.3f} | |de6-de12| {abs(r['de_6.25'] - r['de_12.5']):.3f} |de25-de6| {abs(r['de_25'] - r['de_6.25']):.3f} | eta SPS 100/6.25 {r['eta_SPS_100']:.2f} {r['eta_SPS_6.25']:.2f}, L0 SPS {r['L0_SPS']:.3f} eta0 {100 * fe.eta_at(r['P'], r['L0_SPS'], int(r['dirn'])):.2f} ; B eta 100/6.25 {r['eta_B_100']:.2f} {r['eta_B_6.25']:.2f}, L0 B {r['L0_B']:.3f} eta0 {100 * fe.eta_at(r['P'], r['L0_B'], int(r['dirn'])):.2f}")
T = pd.read_csv(os.path.join(ROOT, "out", "f1eff_check.csv"))
x = T[T["L12.5"] > 0.5]
print("rows L12.5 > 0.5:", len(x), " 1 - median r:", round(1 - float((x["L6.25"] / x["L12.5"]).median()), 3), " largest drop:", round(1 - float((x["L6.25"] / x["L12.5"]).min()), 3))
print("L(6.25)/L(100) on the same rows: median", round(float((T["L6.25"] / T["L100"]).median()), 3), "range", round(float((T["L6.25"] / T["L100"]).min()), 3), round(float((T["L6.25"] / T["L100"]).max()), 3), "n", int((T["L6.25"] / T["L100"]).notna().sum()))
d = fe.load()
X = d[np.isclose(d["Ts"], fe.T25, rtol=1e-6, atol=0) & np.isclose(d["Ts_solver"], fe.T6, rtol=1e-6, atol=0)]
dif = []
for r in X.itertuples():
    a = T[(T["ctrl"] == r.ctrl) & (T["V2"] == r.V2) & (T["dirn"] == r.dirn) & (T["level"] == r.level) & (np.abs(T["D1"] - r.D1) <= 1e-6) & (np.abs(T["D2"] - r.D2) <= 1e-6)]
    dif.append(abs(r.L - a["L6.25"].iloc[0]))
print("X runs |L(25c,6.25s) - L(6.25,6.25)| W: max", round(max(dif), 4), "median", round(float(np.median(dif)), 4), "n", len(dif))
print("Ipk (25c, 6.25s) vs (6.25, 6.25):")
for r in X.itertuples():
    a = T[(T["ctrl"] == r.ctrl) & (T["V2"] == r.V2) & (T["dirn"] == r.dirn) & (T["level"] == r.level)]
    print(f"   {r.label}: Ipk {r.Ipk_sim:.4f} vs {a['Ipk6.25'].iloc[0]:.4f} vs 25/25 {a['Ipk25'].iloc[0]:.4f}; |P| {abs(r.P_LV_d):.2f} vs {a['Pd6.25'].iloc[0]:.2f} vs 25/25 {a['Pd25'].iloc[0]:.2f}")
print("runs at 100 ns that are not periodic: M-B*, SPS; first level pairs:")
print("non-periodic at fine grids:")
sp = fe._spread()
S = d[np.isclose(d["Ts"], d["Ts_solver"], rtol=1e-6, atol=0)].copy()
S = S.assign(spread=[sp.get(str(r.label), np.nan) for r in S.itertuples()])
print(S[(S["spread"] > 0.005) & ~np.isclose(S["Ts"], fe.T100, rtol=1e-6, atol=0)][["label", "spread"]].round(3).to_string(index=False))
_log.close()
