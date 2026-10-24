# Simulink plant of `Modules/Control_Logic.slx`: parameters for a Python switching re-implementation

Extracted without MATLAB from the .slx XML (blocks, `<Line>`/`<Branch>` connectivity with subsystem PMIOPort
mapping, config sets), on 2026-10-24. Simscape foundation `.ssc` sources (readable in
`C:\Program Files\MATLAB\R2025b\toolbox\physmod\simscape\library\m\+foundation\...`) were used for the equations of the
transformer, L, C, and switch. Simscape Electrical blocks (N-Channel MOSFET, Half-Bridge Driver) are protected `.sscp`, so
their internal semantics are taken from parameter names and MathWorks documentation (flagged where it matters).

## 0. Which model version (read this first)

| | **dataset-era** (git `35d5a78`, 2026-03-20) | **HEAD** `504cc7d` (= working copy, "Debuging Phase Shift") |
|---|---|---|
| Used for | `Data/TPS_Dataset_20260316_021229.csv` (312 000 rows, committed with 35d5a78). Power stage is identical to `380607b` (2026-03-15), the committed model when the run started. | nothing dataset-related |
| D0/D1/D2/V2 inputs | From Workspace `D0`, `D1`, `D2`, `Ref_VL`, `Ref_VH`, `Ref_Power` (active) | **all From Workspace blocks are commented out.** Hard-coded constants: Phi_ext = 70 deg, D1 = D2 = 0.5, Ref_VL = 96 V, V_dc = 380 V |
| PWM | inside the `DAB_Controller_System` MATLAB System (sample-based, 100 ns) | separate sawtooth-compare subsystems (`Subsystem`..`Subsystem7`) |
| Logged names | `Inductor_Current1`, `Power_LV`, `Power_HV`, ... | `I_L`, `P_LV`, `P_HV`, ... |

**Consequence:** on HEAD, `Scripts/Generate_TPS_Dataset.m` and `Thesis/verify_steady_state.m` would run every point at
70 deg / SPS / 96 V, whatever D0, D1, D2, V2 they set. They also would not find `Power_LV`/`Power_HV`, and the fallbacks
`V_LV1`/`I_LV1` are not logged. **Everything below is the dataset-era model.** HEAD differences are listed in
the "HEAD" column or noted.

No model callbacks (no PreLoadFcn/InitFcn/...), no linked data dictionary (`Data/DAB_Parameters.sldd` is not
referenced), no model workspace. **No block parameter uses a workspace variable.** I searched every XML part of both
versions. `C_HV`=1000e-6, `R_ESR_HV`=0.05, `R_ESR_LV`=0.01, `R_batt_int`=0.15 (Generate_TPS_Dataset.m:338-341),
`V_init_LV`/`V_batt` (:182-183), `fs`/`n`/`V_in`/`Kp_ext`/... (:342-350), and `Mode_Select`, `Ctrl_Mode_Ext`,
`PWM_Enable`, `Phi_Manual`, `V_Grid`, `Ref_I_ess_cmd`, `D1_ext_in`, `D2_ext_in` (:149-158, :187-188) are all **dead**.
Only the timeseries read by the active From Workspace blocks matter: `Ref_VH` = 380 V (:149), `Ref_Power` = 0 (:151),
`D0` staircase (:78-93, :157), `Ref_VL` = V2, `D1`, `D2` (:184-186).

`Modules/Circuit_Model.slx` is a different, older circuit and is not used: L = 50 uH, `MOSFET (Ideal, Switching)` with
Rds 0.04 / Goff 1e-8 / Vth 2.7, caps 250/330/1/1000/1000 uF, 1 ohm load, no local solver.

## 1. Topology

| Element | Block (library) | Notes |
|---|---|---|
| 8 switches, 2 full bridges | subsystem `ROHM SCT3040KR`..`KR7` = `ee_lib/Semiconductors & Converters/N-Channel MOSFET` (ee.semiconductors.n_mosfet, 3-terminal, no thermal port) + 3 nH source inductor | Same device on the HV and LV bridges. The 8 `MOSFET (Ideal, Switching)` blocks are commented out. |
| Gate drivers | `UCC0..UCC3` = `ee_lib/.../Half-Bridge Driver`, one per leg, PS control input | Each driver makes its own complementary low-side gate plus dead time from ONE command signal. |
| Transformer | `Ideal Transformer1` (foundation.electrical.elements.ideal_transformer), n = 4 | `v1 = 4*v2`, `i2 = 4*i1` (i2 flows out of the secondary +). No leakage, magnetizing inductance, or winding R. |
| Series inductor | `Inductor3` (foundation), primary side | 90 uH, r = 0, parallel g = 1e-9 S |
| Series DC-blocking caps | `Capacitor3` (primary) and `Capacitor4` (secondary) | 1000 uF each, ESR 1e-6 (HEAD: 100 uF) |
| HV source | `DC링크 전압` = ideal Controlled Voltage Source, value `Ref_VH` (380 V) | Then 2 uH + 0.05 ohm + Switch1 (0.01 ohm, closed) into the 47 uF bus cap |
| LV side | **ideal Controlled Voltage Source** `Controlled Voltage Source2`, value `Ref_VL` (= V2 grid value) | Then 2 uH + 0.05 ohm into the 1000 uF bus cap. **`Battery (Table-Based)` is commented out** in both versions. No load resistors (`Resistor1`, `Variable Resistor1` are commented out). |
| References | `Electrical Reference3` (HV ground), `Electrical Reference2` (LV ground), `Solver Configuration2` on HV ground | |

### Netlist (dataset-era). Node names are mine; `a -> b` gives the + to - orientation.
```
HV side (HV_GND)
 Vdc   CVS1 (in subsystem DC링크 전압)  + n_dc, - HV_GND, v = Ref_VH (S-PS converter, no filter)
 L_hv  Inductor4 (ee_lib/Passive/Inductor) 2e-6 H, r 0, g 1e-12     n_dc  -> n_a
 R_hv  Resistor  0.05 ohm                                         n_a   -> n_grid   [V_Grid = v(n_grid)]
 SW1   Switch1  R_closed 0.01, G_open 1e-8, Threshold 0.5, ctrl = Constant12 = 1 (closed)   n_grid -> n_hvm
 V_HV  전압 센서1 : v(n_hvm) - HV_GND                               -> Goto HV
 I_HV  전류 센서3 : + at n_bus, - at n_hvm (positive = bus -> source) -> Goto HI
 C_hv  Capacitor2 47e-6 F, r 1e-6, v0 380 V                       n_bus -> HV_GND
 leg A Q_A+ "ROHM SCT3040KR"  D n_bus, S A        Q_A- "KR1"  D A, S n_s1
       전류 센서4 n_s1 -> HV_GND [Q2_I_dc];  전압 센서3 A - n_s1 [Q2_V_dc]
 leg B Q_B+ "KR2"  D n_bus, S B                   Q_B- "KR3"  D B, S HV_GND
 C_bp  Capacitor3 1000e-6 F, r 1e-6, v0 0                         A     -> n_c
 L     Inductor3 90e-6 H, r 0, g 1e-9                             n_c   -> n_l   [V_Tf = 전압 센서2 = v(n_c)-v(n_l) = inductor voltage]
 I_L   전류 센서2 : n_l -> P1 (positive = into transformer primary +)  -> Goto I_L  (== transformer primary current)
 T     Ideal Transformer1 n=4: primary + = P1, primary - = B; secondary + = S1, secondary - = D
LV side (LV_GND)
 C_bs  Capacitor4 1000e-6 F, r 1e-6, v0 0                         S1    -> C
 leg C Q_C+ "KR4"  D n_lv, S C                    Q_C- "KR5"  D C, S n_s5
       전류 센서5 n_s5 -> LV_GND [Q6_I_dc];  전압 센서4 C - n_s5 [Q6_V_dc]
 leg D Q_D+ "KR6"  D n_lv, S D                    Q_D- "KR7"  D D, S LV_GND
 C_lv  Capacitor5 1000e-6 F, r 1e-6, v0 96 V                      n_lv  -> LV_GND
 I_LV  전류 센서  : + at n_lvm, - at n_lv (positive = source -> bus) -> Goto LI
 V_LV  전압 센서  : v(n_lvm) - LV_GND                                -> Goto LV
 R_lv  Resistor3 0.05 ohm                                         n_lvb -> n_lvm
 L_lv  Inductor5 (ee_lib/Passive/Inductor) 2e-6 H, r 0, g 1e-12   n_lvs -> n_lvb
 V2    CVS2  + n_lvs, - LV_GND, v = Ref_VL (S-PS converter, no filter)
Each "ROHM SCT3040KR*" subsystem: N-Channel MOSFET D -> S_int; Inductor 3e-9 H (r 0, g 1e-9) S_int -> external S;
 the gate driver's HS/LS return goes to S_int (Kelvin source); the driver's HO/LO goes to G.
Driver wiring: UCC0 -> leg A, UCC1 -> leg B, UCC2 -> leg C, UCC3 -> leg D (HO/HS = high side, LO/LS = low side).
```
HEAD differences: Inductor4, Inductor5, Resistor, and Resistor3 are commented out, so Vdc connects through Switch1 straight
to the bus and CVS2 sits straight on the LV bus. Two extra ideal current sensors (`전류 센서6/7`) sit between the bus cap
and the bridge rail.

**Logged signals (To Workspace, inherited sample time = 100 ns, timeseries):** `Inductor_Current1` = I_L,
`V_Transformer1` = V_Tf (inductor voltage), `Voltage_High1`/`Current_High1` = HV/HI, `Voltage_Low1`/`Current_Low1` =
LV/LI, `Power_HV`/`Power_LV`, plus controller internals. `Power_HV` and `Power_LV` come from MATLAB System1
(`Direct_Efficiency_Analyzer.m`). Each is a **125-sample (one Ts) moving average** of `HV*HI` / `LV*LI`, so with the
sensor signs above, `Power_LV > 0` and `Power_HV > 0` both mean LV -> HV. This matches the dataset: D0 < 0 gives P > 0.

Dataset extraction (Generate_TPS_Dataset.m:354-409): for step k the window is [t_k - 0.1 ms, t_k - 1 us), with
t_k = 6 ms + (k-1)*1 ms. `i_pk = max|I_L|` over the 100 ns samples, and `P_act = mean(Power_LV)`.

## 2. Numeric parameters (all literal block values; none come from workspace variables)

| Item | Block / param | dataset-era | HEAD |
|---|---|---|---|
| V1 source | Ref_VH (From Workspace, Generate_TPS_Dataset.m:149) | 380 V | Constant22 = 380 |
| V2 source | Ref_VL (:184) | V2 grid, 84..109.2 V | Constant19 = 96 |
| HV series L, R | Inductor4 l, Resistor R | 2 uH (r 0, g 1e-12) and 0.05 ohm | commented out |
| Switch1 | R_closed / G_open / Threshold / control | 0.01 ohm / 1e-8 S / 0.5 / 1 | same |
| HV bus cap | Capacitor2 c / r / g / v0 | 47 uF / 1e-6 / 0 / 380 V | **470 uF** |
| Primary DC-block cap | Capacitor3 c / r / v0 | 1000 uF / 1e-6 / 0 | **100 uF** |
| Series inductor | Inductor3 l / r / g / i0 | 90 uH / 0 / 1e-9 S / 0 | same |
| Transformer | Ideal Transformer1 n | 4 (primary:secondary) | same |
| Secondary DC-block cap | Capacitor4 c / r / v0 | 1000 uF / 1e-6 / 0 | **100 uF** |
| LV bus cap | Capacitor5 c / r / v0 | 1000 uF / 1e-6 / **96 V (fixed, not V2)** | same |
| LV series L, R | Inductor5 l, Resistor3 R | 2 uH and 0.05 ohm | commented out |
| Source inductance per switch | ROHM*/Inductor2 l / r / g | 3 nH / 0 / 1e-9 | same |
| Driver propagation delay | UCC* tpLH = tpHL | **30 ns** | 10 ns |
| Driver dead time (see §4) | UCC* tdLH = tdHL | **100 ns** | 200 ns |
| Controller | Fs / Ts_ctrl / V_Gate_ON / V_Gate_OFF | 80e3 / 1e-7 / 18 / -4 | same |
| Controller inputs | Mode_Select / Ctrl_Mode / PWM_En / DeadTime_ns | Constant21 = 1 / Constant8 = 3 / Constant13 = 1 / Constant7 = 0 | Mode_Select = Constant16 = 1, Ctrl_Mode = Constant15 = 0 |
| Run timing | SIM_TIME / T_STARTUP / T_STEP / T_WINDOW | 44 ms / 6 ms (:75 overrides :44) / 1 ms / 0.1 ms | - |
| Grids | V2, D1, D2 (:54-56); D0 (:59-60) | 20 x linspace each; D0 = sign(x)*90*(1-sqrt(1-abs(x))), x = linspace(-1,1,39) | - |

Battery (Table-Based) is commented out (V0_mat starts at 75 V, R0_mat 0.30, AH = 2e-4, SOC 1) and has no effect.

## 3. Switch, body-diode, and driver parameters (all 8 devices identical)

N-Channel MOSFET, `parameterization = datasheet`:

| Parameter | Value | Note |
|---|---|---|
| R_DS(on) | 0.04 ohm at Id = 15 A, Vgs = 18 V | Square-law channel. beta = 1/(Rds(Vgs-Vth)) ~ 1.79 A/V^2, so channel saturation (~175 A) is never reached. The channel conducts in both directions with the gate on. |
| Vth | 4 V | Gate is driven to +18 / -4 V, so the device is fully on or off. |
| RS / RD (source / drain ohmic R) | 2 mOhm / 10 mOhm | UNKNOWN whether the block adds these to R_DS(on) or includes them. Per-device on-resistance is 40-52 mOhm. |
| LS / LD / LG, lead R | 0 | The 3 nH is outside, in the subsystem. |
| Capacitances | C_param = fixed Ciss/Crss/Coss with values **1180e-12, 15e-12, 80e-12 entered in unit pF** | So ~1e-21 F, effectively **zero device capacitance** (the values look like they were typed in farads). No Coss, no resonant ZVS transition, and gate charging is instantaneous. |
| Body diode | exponential: Is = 1e-9 A, N = 1.2, TT = 16 ns (transit time, gives reverse-recovery charge ~TT*I), Cj0 = 0 | Vf ~ 0.64 V at 1 A, 0.71 V at 10 A, 0.75 V at 40 A. The diode parallels the channel, so in reverse conduction above ~16 A (I*0.04 > 0.65 V) it clamps the drop. |
| Temperature | 25 deg C. KR1..KR7: prmDatasheet = simulation, device T = 25 deg C = Tmeas; KR (leg A high side): none | Same behavior for all devices. Rds_T2 = 0.037 at 125 deg C is unused. |
| Off state | no explicit off-conductance in this parameterization (channel off for Vgs < Vth) | Parallel leakage paths are only the 1e-9 S of the inductors. |

Half-Bridge Driver (UCC0..3), PS control input:

| Parameter | Value |
|---|---|
| Input thresholds | uHigh = 2.0, uLow = 0.8 (the input is the controller's 18 / -4 signal) |
| Output | vHigh = 18 V, vLow = -4 V, referred to HS/LS |
| Output model | outputImpedance: rHigh = 5 ohm, rLow = 1.5 ohm. tRise = tFall = 20 ns and cLoad = 10 nF are not used in this mode (UNKNOWN). |
| Timing | dataset-era: tpLH = tpHL = 30 ns, tdLH = tdHL = 100 ns. HEAD: 10 ns and 200 ns. |

## 4. Gate-signal generation (dataset-era)

`DAB_Controller_System` (git 35d5a78) runs as a discrete MATLAB System, **Ts_ctrl = 1e-7** (getSampleTimeImpl, :410),
125 samples per switching period. In manual mode (Mode_Select = 1, :273-278): phi = Phi_Manual = D0 (no slew limit, no
PI), D1 and D2 are clamped to [0.05, 0.5], and flux balancing is off (Ctrl_Mode = 3, mask false). Leg phases (:304-307):
A = 0, B = 360*D1, C = D0, D = D0 + 360*D2 (degrees).

PWM (:318-343): `PhaseCounter = mod(PhaseCounter + 2.88, 360)` is updated **before** use. It resets to 0, so at sample k
`cp = 2.88*((k+1) mod 125)`. The high side of a leg with phase p is commanded ON iff `mod(cp - p, 360)` lies in
[0, 180). The controller's own dead time is DT = 0 (DeadTime_ns = 0). Gate levels are 18 / -4 V.

Only the odd outputs G1 (A), G3 (B), G5 (C), G7 (D) are used. The path is Goto Q1/Q3/Q5/Q7, then S-PS converter
(provide signal, no filter), then the Half-Bridge Driver input. The **driver** generates the complementary low-side
signal and the dead time.

Consequences to replicate:
- **Edge timing is quantized to a 100 ns grid (2.88 deg), rounded up**: effective phase = 2.88*ceil(p/2.88). This
  applies to the D0, D1*360, and D0 + D2*360 edges. Exact grid hits (for example D1 = 0.2, 72 deg) are decided by
  floating-point round-off in the accumulated PhaseCounter.
- The high-side on-time is 63 or 62 samples (6.3 / 6.2 us), not exactly Ts/2. That leaves a small DC component in each
  bridge voltage (up to about V*0.1/12.5, i.e. 3 V primary), which the series caps C3/C4 absorb.
- Sign: a positive phase **delays** the leg. D0 > 0 means the secondary lags, power flows HV -> LV, and P_LV < 0.
- Driver semantics (UNKNOWN in detail, from the parameter IDs): the outgoing device turns off tp after the command edge,
  and the incoming device turns on td later. With Coss = 0, the midpoint moves at turn-off if the current direction
  favors it, otherwise only after td (body-diode conduction).
- Sensor feedback reaches the controller through 5 Unit Delays (inherited sample time). This does not matter in manual mode.

HEAD only: each leg uses `Sawtooth Generator` (Amplitude 360, Frequency 40000; the Units parameter is not stored, so the
built-in default applies). The Simulink library copy stores `rad/sec` explicitly, which implies the default is Hertz.
The gate is `mod(phi + saw(t), 360) <= 180`, then a switch selects 18 / -4. This is continuous-time, sampled by the
100 ns Simscape step. Adding phi to a rising ramp makes a leg **lead**, the **opposite sign** to the dataset-era
controller. The carrier frequency (80 kHz only if the sawtooth spans -360..+360) is UNKNOWN without MATLAB.

## 5. Solver

| | dataset-era | HEAD |
|---|---|---|
| Active config set | configSet4 `_TmpConfiguration2` | same |
| Global solver | VariableStepAuto, MaxStep 1/80000, RelTol 1e-3, AbsTol auto, zero-crossing "use local settings" | ode23tb, otherwise the same |
| StopTime | 0.044 (the script also sets SIM_TIME = 0.044) | 0.21 |

The model is effectively discrete at 100 ns. The MATLAB Systems are discrete at 1e-7, the Simscape network uses a local
solver, and the only continuous source is From Workspace, which has no states. The Step block is unconnected.

`Solver Configuration2`: UseLocalSolver = on, LocalSolverChoice = **Backward Euler**, LocalSolverSampleTime =
**1e-7 s**, MaxNonlinIter 3, MaxModeIter 2, EnableSwitchedLinearOptims on, EquationFormulation time, ResidualTolerance
1e-9, consistency tolerances 1e-3, DoDC off, MinStep 1e-9. AutomaticFiltering on (tau 1e-3) applies only to 1-D/3-D
connections, of which there are none. All S-PS converters use `FilteringAndDerivatives = provide` with zero derivatives,
i.e. no input filter, so gate, Vdc, and V2 commands enter unfiltered. The PS-Simulink outputs are sampled at 1e-7.

## 6. Assessment: why the Simulink i_pk differs from an ideal lossless piecewise-linear DAB

Ranked by estimated effect on i_pk. Where noted, the estimates use `tps_model.py` over the dataset grid.

1. **D0 actually applied in the measurement window (dataset artefact, inferred, verify).** From Workspace `D0` has
   "Interpolate data" at its default (on). As far as I know the block ignores the timeseries' ZOH flag
   (Generate_TPS_Dataset.m:93); I could not check this offline. If so, D0 **ramps linearly** across each 1 ms step, and
   window k sees about D0_k + 0.95*(D0_{k+1} - D0_k), not at steady state. Exceptions: label -90 sees about -69.5 deg,
   and label +90 is steady at 90 deg.
   The dataset supports this. Labels 69.35 and 90 give the same P_LV (median ratio 0.9992, p5-p95 0.964-1.041, over
   8000 (V2, D1, D2) combinations), while steady states at those phases predict a median of 0.9455 (0.60-1.26).
   Fitting an effective D0 per label (ideal model, free power scale) gives: -90 -> -72.0, -69.35 -> -64.75,
   -24.71 -> -25.6, 0 -> +0.85, 31.6 -> 37.6, 60.8 -> 73.5, 69.35 -> 93.5, 90 -> 95.75 deg.
   That pattern is linear interpolation plus an extra ~3-4 deg increase in |D0| at high power (items 3-4). Plain ZOH
   would need an implausible asymmetric offset: about 0 for D0 < 0, +6..+11 deg for D0 > 0, +18 deg at -90, +24 deg at 69.35.
   Overall, the ideal model at the labelled D0 is off by -0.87 A mean and 1.47 A RMS (model minus dataset, 40k random rows).
2. **Conduction resistance, mainly the LV-side MOSFETs.** The same 40 mOhm device carries 4x the current on the LV side.
   Loop resistance referred to the primary is 2*Ron + 16*2*Ron = 34*Ron = **1.36 ohm** (1.77 ohm if RS+RD add), so
   L/R ~ 50-66 us against Ts/2 = 6.25 us, and the power-angle shift is atan(R/wL) ~ 1.7-2.2 deg. Above ~16 A the
   reverse drop clamps at ~0.7-0.75 V (body diode), so the effect is nonlinear and direction-dependent.
   The dataset shows losses (|P_HV| - |P_LV|) of 5-6 % at ~1.2 kW and 9-10 % at ~2.4 kW (SPS, V2 ~ 96 V). These are
   mostly secondary conduction plus the LV 0.05 ohm. Expected i_pk effect: a few %, about 0.1-0.5 A.
3. **Dead time 100 ns per edge with zero Coss.** An edge where the current opposes the transition happens td late:
   dI ~ V*td/L ~ 0.42 A per primary edge, 0.43 A per secondary edge (primary-referred). ZVS edges are not delayed.
   Near zero-current commutation (light load, small D0, TPS), the midpoint is undefined during the dead time. The
   dataset shows a dead band: V2 = 95.9 V, D1 = D2 = 0.2, label D0 = -4.87 gives P = 7.6 W against 106 W ideal.
4. **100 ns gate quantization (ceil).** Over the grid, quantizing all three edges changes i_pk by 0.14 A on average
   (p95 0.30 A, max 0.42 A) and P by -27 W on average. A one-step (2.88 deg) shift of D0 alone gives 0.31 A on average.
5. **LV source series R (0.05 ohm + 2 uH).** The bridge-side V2 is Ref_VL minus I_LV*0.05, i.e. +-1.25 V at 25 A. The
   dataset's V2 label is the source EMF. A 1.25 V change in V2 changes i_pk by 0.10 A on average (p95 0.17 A). The HV
   side (0.06 ohm, ~0.4 V) is negligible.
6. **Series DC-block caps.** At 1000 uF the ripple is about 0.04 V primary and 0.15 V secondary (negligible); in HEAD
   (100 uF) it is about 0.4 V and 1.5 V (~1.5 % of V2). Their L-C resonance (2.2 kHz dataset-era, 6.9 kHz HEAD) is
   damped by item 2 (tau = 2L/R ~ 0.1-0.15 ms, longer if the diode clamp lowers the incremental R). Residue after a
   20 deg step within 1 ms is UNKNOWN, but small if R ~ 1.4 ohm. The caps also absorb the 62/63-sample duty DC (§4).
7. **Bus ripple.** HV: 47 uF behind 2 uH gives ~0.6 V (0.2 %) ripple, with lightly damped 16 kHz ringing (zeta ~ 0.15).
   LV: 1000 uF gives ~0.1 V. Negligible for i_pk. The LV cap starts at 96 V, not V2; this settles in < 1 ms.
8. **Negligible.** No magnetizing or leakage inductance or winding R (ideal transformer), so I_L is exactly the primary
   current. The 3 nH source inductors, Backward-Euler error at 100 ns (exact for piecewise-linear current with
   grid-aligned edges), the 30 ns common propagation delay (no relative phase effect), and the body-diode reverse
   recovery (TT = 16 ns) are all small.

Recommended Python plant: the netlist in §1 at a 100 ns step; the controller's exact sample/ceil PWM logic; driver
dead time = 100 ns with ideal (Coss = 0) commutation; the exponential diode in parallel with a 40 mOhm channel;
LV/HV source R-L; series caps. Run it with both ZOH and linearly interpolated D0 against the dataset to settle item 1.

## 7. UNKNOWN / not determinable offline
- Whether R_DS(on) includes or excludes RS (2 mOhm) and RD (10 mOhm) in the datasheet parameterization.
- Exact Half-Bridge Driver timing semantics (how tp/td combine, behaviour when an input pulse is shorter than td), and how
  sub-step delays (30 ns) are discretized by the 100 ns local solver.
- Whether From Workspace honours the timeseries ZOH flag. Inferred: no; see §6 item 1.
- Which solver "VariableStepAuto" picked (irrelevant if the model is all-discrete, as it appears to be).
- HEAD only: the sawtooth Units/range, and therefore the carrier frequency.
- The dataset was generated from a working copy; uncommitted edits between 380607b and 35d5a78 (same power stage)
  cannot be ruled out.
