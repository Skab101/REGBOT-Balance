---
course: "34722"
course-name: "Linear Control Design 1"
type: test-plan
tags: [LCD, regbot, tests, physical, day5-redesign]
date: 2026-04-22
---
# REGBOT Physical Test Plan — Group 47 (v3, `day5-redesign` branch)

> [!abstract] Purpose
> Phase 6 hardware re-validation of the cascade after the Day 5 on-floor redesign. Same four missions as the original campaign, but with the v3 gains from `config/regbot_group47.ini` and fresh log/figure filenames so we can compare against the pre-redesign baseline.

> [!info] Gain source
> All tests assume the REGBOT firmware has loaded the v3 gains from
> `REGBOT-Balance-Assignment/config/regbot_group47.ini` on this branch.
> If a controller misbehaves, first confirm the gains are actually on the robot before touching the design.

> [!success] Pre-hardware evidence
> Simulink sanity sims (Phase 5.B) were clean with the v3 gains:
> - 10° balance recovery: peak voltage 2.8 V, settled in ~2 s — faster than the old design
> - 2 m topos step: peak velocity ≈ 0.8 m/s (above 0.7 spec), 7.5% overshoot, voltage peak ~3 V
>
> Screenshots are embedded in the Task 2 and Task 4 verification blocks below.

---

## 0. Pre-flight — do this once per session

- [ ] **Battery charged** (check voltage reading is ≥ nominal; low battery → bad balance performance)
- [ ] **Gyro calibration** completed (hold robot still, run gyro calibration routine; zero-rate offsets must be small)
- [ ] **Tilt-offset calibration** completed (the angle the robot reads as "vertical" matches the mechanical balance point)
- [ ] **ini loaded into GUI** — for each of the four controllers (Wheel velocity, Balance, Balance velocity, Balance position):
    - Open the controller edit dialog
    - Paste into "Load from:"  `C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\config\regbot_group47.ini`
    - Click "Load from:" button — log should show `# UControl:: loading <cID> data from ...`
    - Confirm the values in the dialog match the v3 table below
- [ ] **Sent to robot** (normal GUI "send" / "OK" workflow)
- [ ] **Saved to robot flash** so values survive a power-cycle (File → save configuration to robot)
- [ ] **Test space clear** — 3 m × 3 m minimum for Test 4, 2 m × 2 m minimum for Test 3b, 1 m × 1 m OK for Test 3a
- [ ] **Catcher ready** — one teammate within arm's reach to grab the robot if a loop goes unstable

### Ini verification values — **v3 gains**

| Controller | Dialog block | Kp | τᵢ | τ_d | Post-integrator τ |
|---|---|---|---|---|---|
| `[cvel]` | Wheel Velocity | **13.2037** | 0.1000 | — | — |
| `[cbal]` | Balance | **−1.1999** | 0.2000 | **0.0442** (as `lead_back_tau_zero`) | **0.1245** |
| `[cbav]` | Balance velocity | **0.1581** | 3.0000 | — | — |
| `[cbap]` | Balance position | **0.5411** | — (disabled) | — | — |

### Simulink-predicted behaviour (Phase 5.B, v3 gains in regbot_mg.m)

![[regbot_task2_sim_recovery_10deg_v3.png]]
*Sim 1 — 10° initial-tilt recovery with v3 gains. Motor voltage spikes briefly to 2.8 V, then a few damped oscillations. Pitch returns to 0 within ~0.3 s and fully settles by t ≈ 2 s. Faster than the v1 design.*

![[regbot_task4_sim_step_v3.png]]
*Sim 2 — 2 m position step at t = 1 s with v3 gains. Peak wheel velocity ≈ 0.8 m/s (spec ≥ 0.7 ✓), peak tilt +17°, position overshoots to 2.15 m (7.5%) then settles at 2.00 m by t ≈ 14 s. Motor voltage peaks ~3 V (no saturation).*

---

## Test 0 — Inner wheel-speed loop only (pre-validate Task 1)

> [!note] Why this matters especially on the v3 branch
> Kpwv jumped from 3.31 to 13.20 (4×). The controller is much more aggressive; if anything is wired wrong in the inner loop, this is where it would bite first. Run Test 0 before closing the balance loop.

**Mission script:**
```
bal=0, vel=0.3, log=15 : time=3
vel=0
```

**Setup:**
- Lay the robot on its side (or hold wheels off the ground) so it can't fall.
- Balance must be **disabled** (`bal=0`).

**Signals to log** (`log=15` = 15 ms interval):
- Time, motor voltage (both wheels), wheel velocity (both), commanded velocity

**Pass criteria:**
- [x] Wheel velocity reaches 0.27 m/s within ~0.10 s (vs 0.33 s in v1) — **0.012 s measured**
- [x] Zero steady-state error — **1% err, within noise**
- [x] Both wheels agree within ~5% — **0.76%**
- [x] Motor voltage stays within ±8 V — **peak 2.60 V**

**Log file — full absolute path to paste into the GUI:**
```
C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\logs\test0_wheel_speed_v3_onfloor_2026-04-22.txt
```

**Notes (post-test):** ✅ **PASS (2026-04-22)**

| Metric | Spec / v1 baseline | v3 measured |
|---|---|---|
| Rise time to 0.27 m/s | ~0.08 s target, v1 0.329 s | **0.012 s** (27× faster than v1; within one 15 ms log sample) |
| L mean (0.5–2.9 s) | 0.3 m/s | 0.2997 m/s (err −0.1%) |
| R mean | 0.3 m/s | 0.2975 m/s (err −0.8%) |
| L vs R diff | <5% | 0.76% |
| Voltage peak | <±8 V | 2.60 V (v1 peak 1.93 V) |
| Initial voltage dip | — | −0.66 V (transient reaction to the step) |

**Observations:**
- The 4× higher Kp (3.31 → 13.20) produces visibly more voltage ripple in steady state (~0.6 V peak-to-peak vs ~0.3 V on v1). This is encoder quantisation noise amplified by the higher loop gain. Wheel velocity still tracks the mean perfectly; no saturation; stability not affected. Classic speed/noise tradeoff — worth a note in the report.
- Rise time is now below the log sampling resolution, consistent with the designed 30 rad/s crossover actually materialising on hardware (v1 hidden effective wc was ≈ 9 rad/s because of the plant-model mismatch).

![[test0_wheel_speed_v3_onfloor_2026-04-22.png]]
*Test 0 v3 with v1 faint overlay. Top: wheel velocities vs reference — v3 hits target in one log sample, v1 took ~0.33 s. Middle: motor voltages — v3 has higher peak and more ripple. Bottom: tracking error.*

---

## Test 3a — Stationary balance (Task 2 verification)

**Mission script:**
```
vel=0, bal=1, log=15 : time=10
```

**Setup:**
- Hold robot upright near balance point, start mission, release gently.
- Sign on `[cbal] kp` must be `−1.1999` (negative feedback). Positive Kp → immediate runaway, as we learned in v1.

**Signals:** time, pitch, gyro, motor voltage (both), wheel velocity, x_position.

**Pass criteria:**
- [ ] Stays upright for the full 10 s
- [ ] Drift ≤ 0.5 m (previous v2 run: 0.343 m)
- [ ] Calm-period pitch ≤ ±2° (previous v2: ~±1° during the calm middle interval)
- [ ] Linear-model settling prediction: 1.34 s (was 1.55 s under v1)

**Log file — full absolute path to paste into the GUI:**
```
C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\logs\test3a_balance_rest_v3_onfloor_2026-04-22.txt
```

**Notes (post-test):**
- Balance hold time:
- Drift:
- Peak pitch:
- Motor voltage peak:
- Mean tilt offset (indicator of calibration):
- Compare to v2 (0.343 m drift, 10° late oscillation):
- Plot: `figures/test3a_balance_rest_v3_onfloor_2026-04-22.png`

---

## Test 3b — Square run at 0.8 m/s (Task 2 + 3 verification)

**Mission script:**
```
vel=0, bal=1, log=15 : time=2
vel=0.8 : dist=1
vel=0.8, tr=0.2 : turn=90
vel=0.8 : dist=1
vel=0.8, tr=0.2 : turn=90
vel=0.8 : dist=1
vel=0.8, tr=0.2 : turn=90
vel=0.8 : dist=1
vel=0
```

**Pass criteria:**
- [ ] Completes 4 sides + 3 turns without falling
- [ ] Cumulative heading ≈ 360° (previous v2 run: 359.8°)
- [ ] Peak wheel velocity > 0.8 m/s on turn (v2 baseline: 1.07 m/s)
- [ ] Motor voltage within ±8 V (v2 baseline peak: 4.67 V)
- [ ] No visible limit-cycle growth

**Log file — full absolute path to paste into the GUI:**
```
C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\logs\test3b_square_0.8ms_v3_onfloor_2026-04-22.txt
```

**Notes (post-test):**
- Speed used:
- Completed square? Y/N
- Side length measured:
- Peak tilt:
- Peak voltage:
- Compare to v2 square (+22° tilt, 4.67 V):
- Plots:
    - XY trajectory: `figures/test3b_xy_v3_onfloor_2026-04-22.png` ← updated "cool figure" for the report
    - Time series: `figures/test3b_timeseries_v3_onfloor_2026-04-22.png`

---

## Test 4 — 2 m position move

**Mission script:**
```
vel=0, bal=1, log=15 : time=2
topos=2, vel=1.2 : time=10
```

**Pass criteria:**
- [ ] Reaches 2 m ± ~5 cm (v2: reached 1.97 m peak, 1.89 m final — within spec, limit-cycle tail)
- [ ] Stays balanced throughout
- [ ] **Peak velocity ≥ 0.7 m/s** (sim predicted 0.8 m/s; v2 hardware: 1.01 m/s)
- [ ] Completes inside 10 s mission window
- [ ] No motor saturation

**Log file — full absolute path to paste into the GUI:**
```
C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\logs\test4_position_2m_v3_onfloor_2026-04-22.txt
```

**Notes (post-test):**
- Final / peak position:
- Peak velocity:
- Settling time:
- Peak tilt:
- Does the late limit-cycle still appear? (v2 pattern after target reached):
- Plot: `figures/test4_position_2m_v3_onfloor_2026-04-22.png`

---

## Post-test review — mapping to report sections

| Test | Report section (Report submodule) | Figure(s) needed |
|---|---|---|
| Sim 1 (10° recovery) | `balance-controller.tex` → Simulation Results | `regbot_task2_sim_recovery_10deg_v3.png` |
| Sim 2 (2 m step) | `position-controller.tex` → Simulation Results | `regbot_task4_sim_step_v3.png` |
| Test 0 | `wheel-speed-controller.tex` → Experiment | time-series plot |
| Test 3a | `balance-controller.tex` → Experiment | pitch/voltage/x vs. time |
| Test 3b | `velocity-controller.tex` → Experiment + "cool XY figure" | XY + time series |
| Test 4 | `position-controller.tex` → Experiment | position/velocity/pitch/voltage vs. time |

---

## Running list of issues hit during this campaign

| Test | Issue | Root cause | Fix applied | Re-run passed? |
|---|---|---|---|---|
| (carried from v1/v2) | Sign error on `[cbal] kp` | Firmware Balance does not absorb Method 2 minus sign | Committed as `kp = -1.1999` in v3 ini | — |
| (carried from v1/v2) | Tilt-offset bias → 0.34 m drift | Calibration residual | Deferred to stretch goal | pending — re-check in Test 3a v3 |
|   |   |   |   |   |

---

*Document created: 2026-04-22 (day5-redesign branch). Fill post-test notes in during the session.*
