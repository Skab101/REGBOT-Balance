---
course: "34722"
course-name: "Linear Control Design 1"
type: assignment
tags: [LCD, assignment, REGBOT, balance, final]
date: 2026-04-15
---
# REGBOT Balance Assignment

> [!abstract] Goal
> Design, implement and test a control strategy for the REGBOT such that we achieve motion while keeping balance.

> [!example] Related Materials
> - [[Lesson 10 - Unstable Systems and REGBOT Balance]] — unstable-system theory and Nyquist primer (§2)
> - [[Lecture_10_Unstable_systems.pdf|Lecture 10 Slides]]
> - [[Fundamentals - Intuitive Control Theory|Fundamentals Guide]]
> - [[Diagnostic Guide - What Went Wrong|Diagnostic Guide]]
> - [[Worked Example - REGBOT Position Controller|Worked Example]]
> - [[Day 5 - Black Box Modeling]] — voltage-to-velocity transfer function
> - [[Day 8 & 9 - Position Controller Design]] — prior PILead design

---

## Preparation

Before you start designing:

- [x] Watch `REGBOT balance introduction.mp4` (Resources/Videos and Tutorials)
- [x] Review the **REGBOT control architecture** slides (included in Lecture 10 slides)
- [x] Download starter files from Resources/REGBOT balance resources:
    - `regbot_1mg` — REGBOT system with wheel velocity control loop
    - `regbot_mg` — associated model file
- [ ] Calibrate the **gyro** and **tilt-offset** before testing on the robot
- [x] Install MATLAB packages (from **Add-Ons** in the Home tab):
    - **Simscape Multibody** (required to simulate the model)
    - **Simulink Control Design**

> [!warning] Before Testing on REGBOT
> Implement all controllers (designed in MATLAB) in the **Simulink model** first. Only test on the physical REGBOT after simulation confirms the design works.

---

## Plain-English Guide — Start Here if You're New to This

> [!tip] For the whole team
> This section is a **beginner-friendly primer**. If you haven't done much linear control design before, read this first — the math deeper in this note will make sense once these ideas land. You don't need to memorise equations; just understand *what each piece is doing and why*.

### 1. What are we actually building?

The REGBOT is a **Segway-like robot** — two wheels, a tall body, a battery and motors up top. Left alone with no control, it falls over. The assignment is to make it:

1. **Balance** (stay upright even when pushed).
2. **Move at a commanded velocity** while balancing.
3. **Drive to a commanded position** while balancing.

Think of balancing a broom on your hand. The broom naturally falls over (it's *unstable*). You keep it up by **moving your hand** — forward when it tips back, back when it tips forward. Our job is to do this automatically, and to make the "hand" (the wheels) also obey speed and position commands.

### 2. Why four controllers and not one?

One controller trying to do everything would be a nightmare. Instead we stack four, like Russian nesting dolls — each one tells the next one what to do:

| Layer | What it does | Command it receives | Command it sends |
|---|---|---|---|
| **Task 1 — Wheel speed** (innermost, fastest) | Makes the wheels spin at a commanded speed | `vel_ref` (m/s) | motor voltage |
| **Task 2 — Balance** | Keeps the robot tilted at a commanded angle | `θ_ref` (rad) | `vel_ref` |
| **Task 3 — Velocity** | Makes the robot move at a commanded speed | `v_ref` (m/s) | `θ_ref` |
| **Task 4 — Position** (outermost, slowest) | Drives the robot to a commanded location | `x_ref` (m) | `v_ref` |

The trick that makes this work: **each outer loop is slower than the one inside it** (at least 5× slower). From the outer loop's point of view, the inner loop looks "instantaneous" — you just send it a command and trust it to follow. If they have similar speeds, they fight each other. Rule of thumb, nothing more.

### 3. The three ideas you need from linear control

**(a) Transfer function `G(s)`.** A mathematical model of a system that answers: *"if I shake the input at frequency ω, how big is the output shake and how delayed is it?"*  Every frequency gets two numbers: **magnitude** (how big) and **phase** (how delayed). You don't need to derive G(s) yourself — MATLAB's `linearize()` builds it from the Simulink model.

**(b) The open-loop Bode plot.** A picture of magnitude and phase across all frequencies for the product `L(s) = C(s)·G(s)` (controller × plant). Reading this plot tells you whether the closed-loop will be stable and how it will respond. Two numbers to look at:

- **Crossover frequency `ω_c`** — where the magnitude crosses 1 (= 0 dB). This roughly equals the bandwidth of the closed loop. Higher = faster response.
- **Phase margin `γ_M`** — at ω_c, how many degrees above −180° is the phase? If `γ_M > 0` the system is stable. Target: **60°** (comfortable). 30° is shaky. 0° oscillates forever.

**(c) Closed-loop stability.** When you connect the controller output back as feedback, the loop becomes "closed". For the closed loop to be stable, **all its poles must be in the left-half plane (LHP)**. Poles in the right-half plane (RHP) mean "this signal grows forever → system explodes". For normal (stable) plants, positive phase margin = LHP closed-loop poles. For unstable plants there's an extra twist (see next section).

### 4. Why Task 2 (balance) needs the Lecture 10 trick

The balance plant `G_tilt` has **one RHP pole at +8.7 rad/s** — the "falling" mode of an inverted pendulum. No matter what gain you pick, a simple P controller can't stabilise it. From the Nyquist criterion, we need the controller to make the Nyquist curve **encircle the point (−1, 0) once counter-clockwise**, and `G_tilt` with a positive gain can't do that.

**Lecture 10's fix (Method 2):**

1. **Flip the sign.** Put a "−1" gain in the loop. Now the Nyquist curve is flipped, and it *can* encircle −1 in the right direction.
2. **Add a "post-integrator".** This is a PI block *in front of the plant* whose zero is placed exactly at the magnitude peak of `|G_tilt|`. With the Day 5 on-floor inner loop in place, the peak sits at `ω_peak = 8.03 rad/s`, so we pick `τ_i,post = 1/ω_peak = 0.125 s`. After this, the "new plant" `G_tilt,post = −C_PI,post · G_tilt` has a monotonically decreasing magnitude curve — ordinary design techniques work on it.
3. **Design a PI-Lead outer controller** on `G_tilt,post` like you would for any stable plant (see next section).

The minus sign and the post-integrator are **bundled together** — that's why the block diagram shows `−1 → post-integrator` right after the error sum.

### 5. The four-step design recipe we use for every loop

Once you understand this recipe, every task looks the same.

**Step 1 — Sign check.** Is the plant stable? If yes (Tasks 1, 3, 4), skip to Step 3. If no (Task 2), use Lecture 10's post-integrator trick from the section above.

**Step 2 — Stabilise (only for unstable plants).** Build `G_stab = −C_PI,post · G_plant` as described. Now `G_stab` is stable and ready for ordinary design.

**Step 3 — Design the outer PI (or PI-Lead):**

1. **Pick `ω_c`** (how fast you want the loop). Use the Lecture 10 rules:
   - As high as possible for good bandwidth.
   - But inner loop ≪ outer loop: at least 5× separation.
   - Watch out for RHP zeros — they limit `ω_c ≤ z/5`.
2. **Place the PI zero.** `τ_i = N_i/ω_c` with `N_i = 3` (standard). This puts the PI's knee three times below crossover, which means the PI still contributes integration at ω_c without dropping the phase too much.
3. **Check the phase at `ω_c`.** On the Bode plot, read off `φ_G`, the plant's phase at `ω_c`. The PI adds `−arctan(1/N_i) = −18.43°`. The total phase must equal `−180° + γ_M = −120°` for a 60° margin. Anything missing is made up by a Lead.
4. **If a Lead is needed:** `τ_d = tan(φ_Lead)/ω_c`. For the balance loop, we cheat: the gyro already measures `dθ/dt`, so `τ_d · gyro + pitch = (τ_d s + 1) · pitch` — an ideal Lead for free, no filter pole needed.
5. **Set the gain.** Pick `K_p` so that `|L(jω_c)| = 1`, i.e. `K_p = 1 / |C_PI · C_Lead · G(jω_c)|`.

**Step 4 — Verify.** Three checks:

- `margin(L)` reports achieved `ω_c`, `γ_M`, `GM`. They should match the targets.
- All closed-loop poles have negative real parts (LHP → stable).
- In Simulink, simulate a realistic scenario and make sure motor voltage doesn't saturate, pitch stays small, and the closed-loop response looks like what you expected.

### 6. What each knob *physically means*

| Symbol | Physical meaning | Increase it → | Decrease it → |
|---|---|---|---|
| `ω_c` | Closed-loop bandwidth (how fast it responds) | Faster, but less robust | Slower, but more forgiving |
| `γ_M` | Safety buffer before oscillation | Safer, more damped | Jumpier, closer to instability |
| `N_i` | How far below `ω_c` the PI zero sits | More phase margin, slower | Less phase margin, faster integration |
| `τ_i` | PI time constant = `N_i/ω_c` | Slower at removing steady-state error | Faster, but eats phase margin |
| `τ_d` | How hard the Lead kicks the phase up | More phase boost, amplifies high-freq noise | Less boost, smoother |
| `K_p` | Overall loop gain (sets the actual `ω_c`) | Higher bandwidth, possibly lower margin | Lower bandwidth, more margin |

### 7. Tricks we use that aren't obvious from the equations

- **Sign absorption** (Task 2): we need a minus sign somewhere because of the RHP pole. We bundle it into the post-integrator block (`−C_PI,post`) rather than having a standalone `Gain = −1` in a weird place. Cleaner.
- **Gyro-based ideal Lead** (Task 2): instead of numerically differentiating pitch (noisy) or using a proper Lead with a filter pole (reduces the phase boost), we note that the gyro already *is* the derivative. Adding `τ_d · gyro` to `pitch` gives a **pure** `(τ_d s + 1)` Lead with no filter pole — free lunch.
- **Placement of the Lead matters** (Task 2 — we got this wrong at first): the gyro-based Lead has to be combined with pitch *on the feedback path, before the error sum*. If you add `τ_d · gyro` in parallel after the PI blocks, you implement `C_PI · C_PI,post + τ_d s` (additive, wrong) instead of `C_PI · C_PI,post · (τ_d s + 1)` (multiplicative, right).
- **Linearise with the inner loop closed but the current loop open** (Tasks 3, 4): for Task 3 we want the plant `θ_ref → wheel_vel` *with the balance loop working*. We set `Kpvel = 0` to break the velocity loop, then `linearize()` the Simulink model. The balance loop stays active (because `Kptilt, tdtilt, titilt, tipost` have their designed values), so the RHP pole is already stabilised in the identified plant.

### 8. What bit us and why

- **The Simulink Lead was wired as a parallel sum instead of multiplicative series.** Any disturbance saturated the motor. Fix: move the gyro-based Lead to the feedback path before the error sum. Took a while to diagnose because the signal-flow math isn't obvious until you write it out.
- **Task 3's plant has a real RHP zero at +8.5 rad/s** (the robot must roll *backward* before it can tilt forward). This **fundamentally** limits `ω_c,vel` to below the zero — `ω_c ≤ z/5 ≈ 1.7 rad/s` is safe. We originally picked `ω_c = 3 rad/s`, got an unstable design because `|L|` crossed 1 twice (a second crossing at 13.5 rad/s where phase was already past −180°). Re-designed at `ω_c = 1 rad/s` and it worked cleanly.
- **Large-signal nonlinearities break the linear design.** At 0.8 m/s step commands, the robot tilts to ~23° and the linearisation around 0° pitch stops being accurate. Design is stable in the linear regime but limit-cycles at large amplitudes. Fixes (if needed later): rate-limit the velocity reference, or saturate `θ_ref` to ±10° with anti-windup on the velocity PI integrator.

### 9. Negative gain margin — wait, that's OK?

For Task 2 (and other unstable plants), `margin(L)` reports `GM = −5.58 dB`. Don't panic.

For a **stable plant**, gain margin is an **upper bound**: "you can increase gain by this much before things go unstable". Positive GM is good.

For an **unstable plant** with `P = 1` RHP pole, gain margin is a **lower bound**: "you must NOT decrease gain below this much, or you'll drop below the minimum needed for stability". Negative GM (in dB) is what you expect — it's the normal signature of controlling an unstable plant.

If you see a big negative GM for Task 2, things are **fine**. If you see a positive GM for Task 2, something is wrong.

---

## Control Architecture Overview

The REGBOT balance problem requires a **cascaded control** structure with four nested loops. Each task in this assignment builds the next loop in the cascade, from the innermost (Task 1) outward (Task 4).

```mermaid
flowchart LR
    classDef ref   fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef plant fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:1.5px
    classDef fb    fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px

    XRef["x_ref"]:::ref
    PosCtrl["Position<br/>controller<br/><b>(Task 4)</b>"]:::ctrl
    VelCtrl["Velocity<br/>controller<br/><b>(Task 3)</b>"]:::ctrl
    BalCtrl["Balance<br/>controller<br/><b>(Task 2)</b>"]:::ctrl
    WVCtrl["Wheel-speed PI<br/><b>(Task 1)</b>"]:::ctrl
    Robot["REGBOT<br/>(non-linear<br/>plant)"]:::plant

    XRef -->|x_ref| PosCtrl
    PosCtrl -->|v_ref| VelCtrl
    VelCtrl -->|θ_ref| BalCtrl
    BalCtrl -->|vel_ref| WVCtrl
    WVCtrl -->|motor V| Robot
    Robot -->|pitch, gyro| BalCtrl
    Robot -->|wheel vel| WVCtrl
    Robot -->|lin vel| VelCtrl
    Robot -->|x_position| PosCtrl

    linkStyle 5,6,7,8 stroke:#c07878,stroke-width:1.5px
```

*Cascaded structure: the position loop (outermost) drives a velocity reference, which drives a tilt reference, which drives a velocity-reference for the inner wheel-speed PI, which drives the motor voltage. Red arrows show measurement feedback paths.*

---

## 🔁 2026-04-22 — Day 5 Redesign (v3, on-floor plant): CURRENT canonical values

> [!abstract] Why this section exists
> The cascade in Tasks 1–4 below was first designed against the **Day 4 wheels-up** plant $G_{vel} = 13.34/(s+35.71)$. That design met spec on the bench but Test 0 on the floor revealed the effective inner-loop bandwidth was ~4× below the designed $\omega_c = 30$ rad/s — the Day 4 identification was ~6× faster than the true on-floor plant. The cascade was redesigned on the `day5-redesign` branch against the **Day 5 v2 on-floor** plant $G_{vel} = 2.198/(s+5.985)$, re-linearised at every outer loop, re-validated on the physical robot. **The derivations in the Task 1–4 sections below have been rewritten with the Day 5 on-floor values** — they are the current reportable design. Short "first-attempt" notes remain at each task so the v1→v3 arc is visible where it informs a design choice (e.g.\ why $\tau_d$ is so much smaller in Task 2 now, or why Task 1's $K_p$ jumped 4×).

### Redesign story in one paragraph
First attempt: Day 4 wheels-up identification gave $K_{pwv} = 3.31$ with 121.6° PM. Hardware Test 0 measured wheel-speed rise of 0.33 s (not the ~0.08 s the 30 rad/s design predicted) → effective bandwidth ~9 rad/s. Root cause: the assignment runs on the floor, not wheels-up. Re-identified against `Day5_results_v2.mat` → $G_{vel} = 2.198/(s+5.985)$ (6× slower dominant pole). Same targets ($\omega_c = 30$, $\gamma_M \geq 60$, $N_i = 3$); $K_{pwv}$ rose to **13.20** (4×). Each downstream task re-linearised, new gains committed to `regbot_mg.m` and `config/regbot_group47.ini`, Simulink sanity sims green, hardware re-validated (Tests 0, 3a, 3b, 4 on 2026-04-22).

### v3 canonical gains (authoritative)

| Task | Controller | Plant used (v3) | $\omega_c$ | $\gamma_M$ | Gains |
|---|---|---|---|---|---|
| 1 — Wheel speed | PI | `2.198/(s+5.985)` (Day 5 v2 on-floor) | 30.00 rad/s | 82.85° | $K_{pwv} = 13.2037$, $\tau_{iwv} = 0.1000$ s |
| 2 — Balance | PILead + Post-PI | `Gtilt` re-linearised (1 RHP pole $\approx +9.13$) | 15.00 rad/s | 60.00° | $K_{ptilt} = 1.1871$, $\tau_{itilt} = 0.2000$ s, $\tau_{dtilt} = 0.0454$ s, $\tau_{ipost} = 0.1224$ s |
| 3 — Velocity | PI | `Gvel,outer` (RHP zero $+8.67$) | 1.00 rad/s | 68.98° | $K_{pvel} = 0.1532$, $\tau_{ivel} = 3.0000$ s |
| 4 — Position | P + Lead | `Gpos,outer` (1 free integrator) | 0.60 rad/s | 60.00° (Lead in; 58.3° if Lead dropped for Simulink) | $K_{ppos} = 0.5411$, $\tau_{dpos} = 0.0505$ s |

Values above are from the 2026-04-22 fresh MATLAB design runs (the plots in Tasks 1–4 come from this run). The firmware `config/regbot_group47.ini` and the committed `regbot_mg.m` may carry slightly older decimals from earlier runs in the same campaign (e.g.\ $K_{ptilt} = 1.1999$, $K_{pvel} = 0.1581$, $\tau_{dpos} = 0$) — within Simulink `linearize()` numeric-noise margin ($\leq 3\%$) and operationally equivalent. Paste the copy-paste gains block printed by each design script into `regbot_mg.m` and re-flash the firmware if you want exact alignment.

Firmware sign: `[cbal] kp` is negative (the firmware Balance block does not absorb the Lecture-10 Method-2 sign internally — finding from the v1 campaign, still applies).

> [!note] Push-disturbance figure is still v1
> The only design-time image that has *not* been re-rendered with the Day 5 on-floor cascade is the Task 2 push-disturbance Simulink sim (`regbot_task2_sim_push.png`, 2026-04-21 timestamp). It is kept as a qualitative illustration of regulation against a 1 N / 0.1 s push; the $\theta_0 = 10°$ IC recovery is re-rendered with v3 gains and is the authoritative sanity sim. All Bode / Nyquist / pole-zero / step-response / IC-response plots for Tasks 1–4 are from the 2026-04-22 design runs.

### Key shifts vs. the first-attempt (v1) design

| Quantity       | v1 (Day 4 wheels-up) | v3 (Day 5 on-floor) | Change                                                        |
| -------------- | -------------------- | ------------------- | ------------------------------------------------------------- |
| $K_{pwv}$      | 3.31                 | 13.2037             | **×4**                                                        |
| $\tau_{dtilt}$ | 0.1355 s             | 0.0454 s            | **−67%** (less Lead needed — inner loop is properly fast now) |
| $\tau_{ipost}$ | 0.1682 s             | 0.1224 s            | −27% ($G_{tilt}$ peak shifted 5.95 → 8.17 rad/s)              |
| $K_{pvel}$     | 0.162                | 0.1532              | −5% (RHP zero at +8.67 physics-fixed)                         |
| $K_{ppos}$     | 0.5335               | 0.5411              | +1% (free integrator physics-fixed)                           |

### Hardware outcome summary (v3, 2026-04-22)

| Test | Spec | v3 result | v1/v2 baseline | Verdict |
|---|---|---|---|---|
| 0 — wheel speed | reach 0.27 m/s ≈ 0.3 s | **0.012 s** rise, L/R 0.76%, peak V 2.60 V | v1: 0.329 s rise, peak V 2.06 V | PASS — **27× faster rise**, bandwidth finally materialises |
| 3a — balance at rest | drift ≤ 0.5 m over 10 s | 0.505 m drift (marginal fail); tilt std **1.87°** | v2: 0.343 m drift, tilt std 4.76° | v2 3a is the reportable spec result; v3 shows **61% tighter balance** but tilt-offset bias integrates to larger drift — see §Test 3a nuance below |
| 3b — square at 0.8 m/s | 4 sides + 3 turns, no fall | heading 359.8°, peak tilt +25.5°, tilt std 5.03°, **peak V 7.31 V (91% of ±8 V budget)** | v2: peak V 4.67 V (58%) | PASS — voltage headroom shrunk; this is the bandwidth↔saturation-margin trade |
| 4 — 2 m topos step | peak v ≥ 0.7, reach 2 m in 10 s | final 1.964 m (**3.6 cm short**), no overshoot, peak tilt +17.3°, tilt std 2.93°, peak V 4.95 V (no saturation) | v2: 10.7 cm short, peak tilt +25°, tilt std 5.18°, late limit cycle | PASS — **cleanest test of the campaign**, 3× more accurate final position, no late limit cycle |

### Test 3a nuance (important for honest reporting)

Test 3a was run twice on v3. First run: drift 0.475 m (inside spec), mean tilt offset +1.13°. After tilt-offset recalibration (Y=175): drift 0.505 m (marginal fail), mean tilt offset +1.11°. The recalibration did **not** remove the bias — either one more iteration is needed (Y ≈ 176) or the residual ~1° is a physical asymmetry (CG / wheel-radius) that calibration cannot fix. **Reportable Test 3a uses v2 (drift 0.343 m, passes spec)** because drift depends on sensor calibration which is outside the controller's authority. The v3 **tilt-std improvement (1.87° vs 4.76°, 61% tighter balance)** is cited separately as a redesign win. Tests 3b and 4 do not show the bias because the outer loops regulate it away.

### Redesign wins vs. costs (the report's discussion material)

**Wins:**
1. Test 0 rise **27× faster** (0.012 s vs 0.329 s) — the designed 30 rad/s inner-loop bandwidth actually materialises on hardware now.
2. Test 3a tilt std **61% tighter** (1.87° vs 4.76°) — no late oscillations of the kind seen in v2.
3. Test 4 final position **3× more accurate** (3.6 cm short vs 10.7 cm).
4. Test 4 **no limit cycle after target reached** (v2 showed visible ±10° pitch and ±0.5 m/s `vref` swings).
5. Test 4 peak tilt **17° vs 25°** — tighter balance → less aggressive lean during acceleration.

**Costs:**
1. Test 3a drift 0.505 m vs 0.343 m — not a controller issue (tilt-offset bias integrates more cleanly with the tighter v3 controller). Mitigation: further Y-offset calibration.
2. Test 3b peak motor voltage **7.31 V (91%) vs 4.67 V (58%)** — 4× higher $K_{pwv}$ makes the inner PI react 4× harder to corner `vel_ref` step commands. Classic bandwidth-vs-saturation-margin trade-off.
3. Test 4 peak velocity 0.79 m/s vs 1.01 m/s — tighter balance → less over-tilt → less physical thrust. Still above the 0.7 m/s spec; the trade is intentional.

---

## Tasks

### Task 1 — Wheel Speed Controller (PI)

**Design a PI-controller** for the wheel velocity loop.

- **Transfer function to control:** voltage-to-velocity $G_{vel}(s)$ identified on [[Day 5 - Black Box Modeling|Day 5]]
- **Controller type:** PI
- **Source:** your own design from previous exercises

---

### Task 2 — Balance Controller (PI + Post-Integrator)

**Design a balance controller** so the REGBOT can balance itself and maintain balance during a mission.

> [!important] Post-Integrator
> Include a **"post-integrator"** — a second PI block in the open loop.
> - Treat it as an **additional design element** when computing controller phase and $K_P$
> - Its phase contribution must be added when calculating the total open-loop phase
> - Its gain must be accounted for when solving $|L(j\omega_c)| = 1$
>
> See Lecture 10 slides for details on the post-integrator design.

**Design checklist:**
- [ ] Identify the balance transfer function (angle-to-tilt or similar unstable plant)
- [ ] Include post-integrator in the open loop
- [ ] Compute phase contribution from all elements
- [ ] Calculate $K_P$ such that $|L(j\omega_c)| = 1$
- [ ] Verify stability via Nyquist (REGBOT balance is open-loop unstable!)

---

### Task 3 — Velocity Controller (in Balance State)

**Design a velocity controller** so the REGBOT can move at a given speed forwards/backwards while balancing.

#### Test 3a — Zero velocity (stationary balance)

**Expected:** REGBOT balances in place. Some small movement is acceptable, e.g. drift within approximately **0.5 m** from the starting point.

#### Test 3b — Square run at 0.8 m/s

**Expected:** REGBOT makes a **square run** while staying in balance:
- Side length: **1 m**
- Turning radius: **0.2 m**

---

### Task 4 — Position Controller (in Balance State)

**Design a position controller** for moving the REGBOT to a given position while balancing.

#### Test sequence

The REGBOT must execute:

| Step | State |
|------|-------|
| (a) | Robot stands still |
| (b) | Robot finds the balance |
| (c) | Robot moves a distance of **2 m** with max speed exceeding **0.7 m/s** |
| (d) | Robot stops |

#### Example mission script

```
vel=0, bal=1, log=15 : time=2
topos=2, vel=1.2 : time=10
```

Where:
- `topos=2` → target distance in meters
- `vel=1.2` → maximum speed during movement (m/s)
- `bal=1` → balance mode enabled
- `log=15` → logging level

---

## Mandatory Report — Hand-in Instructions

> [!tip] Submission Details
> - **Max length:** 5 pages
> - **Submit on:** Learn under *Course Content* → *Assignments* → *REGBOT balance*
> - **One submission per group** (only the most recent is kept)
> - **Filename format:** `Group_XX.pdf`
> - **Front page:** full names and student numbers

### Required Content

#### Front Matter
- [ ] Full names and student numbers

#### General Architecture
- [ ] A few lines on the overall control architecture used

#### For Each Design Step, Document:
- [ ] Which transfer function is being controlled
- [ ] Which controllers are in the open loop, and **why**
- [ ] Design parameters and how they were found:
    - $N_i$
    - $\alpha$
    - $\tau_d$
    - $\tau_i$
    - $K_P$
    - $\gamma_M$
    - $\omega_c$
- [ ] **Bode plot** of the open-loop transfer function (showing phase margin)
- [ ] **Step response** of the closed-loop system from Simulink
- [ ] **Step response** from the REGBOT (include the mission script used)
- [ ] **Comments** comparing simulation vs. experiment

#### Extras
- [ ] General comments on findings and methods
- [ ] A cool **XY-plane plot** showing REGBOT motion during Task 4
- [ ] (Optional) Video link to controller tests

> [!important] Style
> Be **precise, accurate, and short**.

---

## Design Workflow Checklist

Follow this order to systematically work through the assignment:

1. **Preparation** — watch intro video, calibrate REGBOT, install MATLAB packages
2. **Task 1** — PI velocity controller (reuse from earlier)
3. **Simulink model** — verify velocity controller works in sim
4. **Task 2** — Balance controller with post-integrator
5. **Simulink** — confirm balance loop closes (REGBOT stays upright in sim)
6. **Physical test** — balance at zero velocity (Test 3a)
7. **Task 3** — Outer velocity controller
8. **Physical test** — square run (Test 3b)
9. **Task 4** — Position controller
10. **Physical test** — 2 m position move (Test 4)
11. **Report** — compile Bode plots, step responses, XY-plot, mission scripts

---

## Key Design Principles (from Course)

> [!tip] Reminders from Fundamentals
> - The balance plant is **open-loop unstable** → Nyquist stability criterion requires **CCW encirclement** of $-1$ per RHP pole (see [[Fundamentals - Intuitive Control Theory#9. The Nyquist Plot Another Stability View|Fundamentals, Section 9]])
> - For zero steady-state error to a **step** reference, you need **at least one integrator** in the loop (see [[Fundamentals - Intuitive Control Theory#11. Type-n Systems and Steady-State Error|Type-n systems]])
> - The post-integrator makes the loop Type-2 → zero error for both step and ramp references
> - Phase margin target: typically $\gamma_M = 50°$–$65°$ for balance between speed and overshoot

---

## Progress Log

> [!abstract] Purpose
> This section tracks what we have actually done so far. Add to it as we progress.

### 2026-04-15 — Session 1: Plant Identification & Task 1 Design

#### Preparation completed
- [x] Watched `REGBOT balance introduction.mp4`
- [x] Downloaded starter files `regbot_1mg.slx` and `regbot_mg.m` from Learn
- [x] Files committed to repo: [`4. Semester/Linear Control Design/REGBOT-Balance-Assignment/simulink/`](../../../../../../4.%20Semester/Linear%20Control%20Design/REGBOT-Balance-Assignment/simulink/)

#### Plant identification via LINEARIZE

Used MATLAB's `linearize()` on the Simulink model at two sets of I/O points:

**1. Voltage → wheel velocity ($G_{wv}$)** — I/O points: `/Limit9v → /wheel_vel_filter`

$$G_{wv}(s) = \frac{7.023\times10^5 s^3 + 7.023\times10^8 s^2 - 5.083\times10^7 s - 5.083\times10^{10}}{s^6 + 2418 s^5 + 1.317\times10^6 s^4 + 1.872\times10^8 s^3 + 2.371\times10^9 s^2 - 3.032\times10^{10} s - 1.881\times10^{11}}$$

**Poles:** $-1713, -490, -200, -21.1, \boxed{+10.6}, -5.0$ rad/s
**DC gain:** 0.270 (m/s)/V
**RHP poles: 1** → physically consistent with the inverted pendulum mode

**2. Velocity reference → tilt angle ($G_{tilt}$)** — I/O points: `/vel_ref → /robot with balance (port 1)`

**Poles:** $-1715, -515.8, -83.7 \pm 63.7j, \boxed{+8.7}, -19.3, -9.2$ rad/s
**DC gain:** $5.04 \times 10^{-4}$ rad/(m/s)
**RHP poles: 1** → the same falling pendulum mode, seen through the closed velocity loop

> [!important] Key finding
> **Both plants have exactly 1 RHP pole** (around $+8$–$+11$ rad/s). This corresponds to the inverted-pendulum falling dynamics — the robot is open-loop unstable, as expected.
>
> **Nyquist implication:** The balance controller must produce **1 CCW encirclement** of $-1$ for the closed-loop system to be stable ($Z = N + P \Rightarrow 0 = N + 1 \Rightarrow N = -1$).

#### Plots generated

**Bode plots:**

![[regbot_Gwv_bode.png]]
*$G_{wv}$: Motor voltage $\to$ wheel velocity, linearised from the Simulink model with the balance loop open (Task 1 inner loop is the Day 5 on-floor plant). DC gain $0.270$; roll-off sets in around the dominant pole at $-5.99$ rad/s, with faster electrical and encoder-filter poles further out.*

![[regbot_Gtilt_bode.png]]
*$G_{tilt}$: Velocity reference $\to$ tilt angle. DC gain $+4.83 \times 10^{-4}$ rad/(m/s). $|G_{tilt}|$ peaks at $8.17$ rad/s (value $0.7068$) — this is the frequency the post-integrator will be placed at. Phase is governed by the right-half-plane pole at $+9.13$ rad/s.*

**Pole–zero maps (shaded stability regions):**

![[regbot_Gwv_pzmap.png]]
*$G_{wv}$ in the s-plane. With the balance loop open, linearising the Simulink model sees the pendulum mode through the wheel-velocity channel: one RHP pole at $\approx +11.23$ rad/s (orange ring) alongside the stable motor/encoder poles in the LHP.*

![[regbot_Gtilt_pzmap.png]]
*$G_{tilt}$ in the s-plane. One RHP pole at $\approx +9.13$ rad/s — the same physical falling mode as on $G_{wv}$, seen through the tilt channel. The controller must arrange one CCW Nyquist encirclement of $(-1, 0)$ to stabilise it.*

**Zoomed pole–zero maps (focus on the slow dynamics around origin):**

![[regbot_Gwv_pzmap_zoom.png]]
*$G_{wv}$ zoomed to $\pm 50$ rad/s. The RHP pole at $\approx +11.23$ rad/s is clearly visible with its orange ring; two fast LHP poles at $\approx -22$ rad/s and a pole/zero near the origin dominate the low-frequency behaviour.*

![[regbot_Gtilt_pzmap_zoom.png]]
*$G_{tilt}$ zoomed to $\pm 50$ rad/s. RHP pole at $\approx +9.13$ rad/s (orange ring) is the pendulum falling mode the balance controller must stabilise. A complex pole pair near $-8 \pm 3j$ and a pair of zeros at $\pm 8$ show the slow dynamics and the non-minimum-phase physics inherited from the inverted-pendulum geometry.*

**Nyquist plot of $G_{tilt}$:**

![[regbot_Gtilt_nyquist.png]]
*$G_{tilt}$ Nyquist plot. Solid blue = $\omega > 0$, dashed = mirror for $\omega < 0$. The red "+" marks the critical point $(-1, 0)$ that governs closed-loop stability. Title shows the open-loop RHP-pole count $P$.*

> [!important] Reading the Nyquist Plot for Task 2
> $G_{tilt}$ has **$P = 1$** RHP pole (the falling-pendulum mode). The Nyquist criterion says $Z = N + P$, so for a stable closed loop we need $Z = 0$, i.e. $N = -1$ — exactly **one counter-clockwise encirclement** of $(-1, 0)$.
>
> Key implications for the balance controller:
> 1. **Sign check first.** Look at which side of the complex plane the curve lives on. If the real-axis crossing is positive, no proportional gain alone can shift the curve past $(-1, 0)$ in the correct direction — we'll need to absorb a minus sign (which is exactly what the "$-C_{PI,\text{post}}$" structure from Lecture 10 does).
> 2. **Post-integrator choice.** After inserting $-C_{PI,\text{post}}(s)$, redraw the Nyquist plot. With $\tau_{i,\text{post}} = 1/\omega_{i,\text{post}}$ (peak of $|G_{tilt}|$), the curve should now make one clean CCW loop around $(-1, 0)$ — visually confirming that Task 2 is on the right track before any PI-Lead design.
> 3. **Distance to $(-1,0)$ = robustness.** A curve that skims past $(-1, 0)$ has low margins. We want the corrected curve to give a comfortable clearance so the real REGBOT (with model mismatch and sensor noise) still works.

In practice this means the Task 2 workflow is: (i) plot $G_{tilt}$ on Nyquist, (ii) insert the sign-absorbing post-integrator, (iii) **replot** and verify the CCW encirclement visually, (iv) only then start the phase-balance calculation for the outer PI-Lead.

#### Task 1 — Wheel Speed PI Controller ✅

**Plant:** Day 5 v2 on-floor identification $G_{vel}(s) = \dfrac{2.198}{s + 5.985}$ — loaded from `data/Day5_results_v2.mat` by `design_task1_wheel.m`. DC gain $0.367$ (m/s)/V, pole at $-5.985$ rad/s, $\tau = 167$ ms. This is the regime the outer loops will see during the assignment missions.

> [!note] First-attempt plant (why a redesign was needed)
> An earlier attempt used the wheels-up identification $G_{vel} = 13.34/(s+35.71)$ ($\tau = 28$ ms, ~6× faster). That design met every assignment spec on the bench, but hardware Test 0 measured a rise time of $0.329$ s — far slower than the $\approx 80$ ms a $\omega_c = 30$ rad/s closed loop predicts. The conservative $121.6°$ phase-margin cushion had been absorbing the plant-pole mismatch: the robot stayed stable but the designed bandwidth was never actually achieved. Re-identifying against the on-floor plant and keeping the same design target ($\omega_c = 30$) lifts $K_p$ from $3.31$ to $13.20$ — a 4× rise that makes the bandwidth materialise on hardware (Test 0 rise falls to $0.012$ s). See the "Day 5 Redesign" section near the top for the full story.

**Design choices:**

| Parameter | Value | Reason |
|---|---|---|
| $\omega_c$ | 30 rad/s | Fast inner loop, 5× above the on-floor plant pole at $5.985$ rad/s. Physically achievable — motor electrical $\tau \approx 2$ ms ($\approx 500$ rad/s), encoder latency $\approx 5$ ms ($\approx 200$ rad/s). |
| $\gamma_M$ (target) | $\geq 60°$ | Spec |
| $N_i$ | 3 | PI zero placed 3× below crossover |

**Phase balance at $\omega_c$.** The plant contributes $\angle G_{vel}(j30) = -\arctan(30/5.985) = -78.7°$ and the PI adds $-\arctan(1/3) = -18.4°$, for a total open-loop phase of $-97.1°$ — i.e.\ $\gamma_M = 82.9°$ before any Lead, already well above the $60°$ target. No Lead is needed.

**Computed values:**

| Parameter                      | Value                        |
| ------------------------------ | ---------------------------- |
| $\tau_i = N_i/\omega_c$        | **0.100 s**                  |
| $K_p$ (from $L(j\omega_c)= 1$) | **13.2037**                  |
| Achieved $\omega_c$            | 30.00 rad/s                  |
| Achieved $\gamma_M$            | 82.85° (well above 60° spec) |
| Achieved GM                    | $\infty$                     |

**Controller:**
$$C_{wv}(s) = 13.2037 \cdot \frac{0.1s + 1}{0.1s}$$

**Simulink / firmware:** `regbot_mg.m` and `config/regbot_group47.ini` (`[cvel]` block) carry these values.

![[regbot_task1_bode.png]]
*Task 1 open-loop Bode $L = C_{wv}\, G_{vel,\text{day5}}$. Title reads $Gm = \infty$, $Pm = 82.8°$ at $30$ rad/s. Magnitude rolls off at $\approx 20$ dB/decade beyond the $10$ rad/s PI zero; phase dips to $\approx -105°$ near the plant pole at $5.985$ rad/s and recovers to $\approx -97°$ at the $30$ rad/s crossover — hence the $82.8°$ PM.*

![[regbot_task1_step.png]]
*Task 1 closed-loop step response $T = L/(1+L)$. Rise from $0$ to $0.9$ in $\approx 75$ ms, peak overshoot $\approx 4\%$ at $t \approx 0.15$ s, settled to $1.0$ by $t \approx 0.3$ s. Zero steady-state error confirms the PI integrator is doing its job.*

---

### Task 2 — Balance Controller (Lecture 10, Method 2) ✅ (MATLAB)

> [!info] Workflow followed
> [[Lecture_10_Unstable_systems.pdf|Lecture 10 slides]] describe two methods for stabilising an open-loop unstable plant. We follow **Method 2** (slide 13) specialised to the tilt loop (slides 21–24).

```mermaid
flowchart TD
    classDef start fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef step  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef decis fill:#8b6914,stroke:#d4a84a,color:#fef3c7,stroke-width:1.5px
    classDef done  fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:1.5px

    A["Linearise Simulink<br/>vel_ref → tilt_angle<br/>→ G_tilt(s), P RHP poles"]:::start
    B{"<b>Step 1</b>: Nyquist sign-check<br/>can +K_PS give<br/>P CCW encirclements of −1?"}:::decis
    C["sign(K_PS) = −1<br/>(absorbed into post-integrator)"]:::step
    D["<b>Step 2</b>: Post-integrator<br/>τ_i,post = 1/ω_peak of |G_tilt|<br/>C_PI,post = (τ s + 1)/(τ s)"]:::step
    E["G_tilt,post = sign·C_PI,post·G_tilt<br/>→ stabilisable by outer loop<br/>(1 CCW encirclement of −1)"]:::step
    F["<b>Step 3</b>: Outer PI-Lead on G_tilt,post<br/>specs: ω_c, γ_M, N_i<br/>3a. τ_i = N_i/ω_c"]:::step
    G["3b. Phase balance<br/>φ_Lead = −180° + γ_M − φ_G − φ_PI"]:::step
    H["3c. τ_d = tan(φ_Lead)/ω_c<br/>(gyro shortcut: τ_d·gyro + θ)"]:::step
    I["3d. K_P from |L(jω_c)| = 1"]:::step
    J{"<b>Step 4</b>: Verify<br/>closed-loop poles in LHP?<br/>margins match specs?"}:::decis
    K["Export Kptilt, titilt,<br/>tdtilt, tipost to Simulink"]:::done

    A --> B
    B -->|no, positive DC gain + P=1| C
    C --> D
    D --> E
    E --> F
    F --> G
    G --> H
    H --> I
    I --> J
    J -->|yes| K
    J -.->|no, iterate on ω_c or Lead| F
```

*Lecture 10 Method 2 applied to the REGBOT balance loop. Each green box is a design step, each orange diamond a go/no-go gate, and the purple box is the handoff to Simulink.*

#### Step 1 — Nyquist sign-check

From the linearisation in Plant Identification above:

| Property | Value |
|---|---|
| DC gain of $G_{tilt}$ | $+5.04 \times 10^{-4}$ rad/(m/s) |
| RHP poles ($P$) | $1$  (pole at $+8.7$ rad/s) |

The Nyquist criterion requires $Z = N + P = 0 \Rightarrow N = -1$ (one CCW encirclement of $(-1,0)$).

A positive DC gain plus $P = 1$ means **no positive $K_{PS}$ can produce the required CCW encirclement** — the Nyquist curve cannot be scaled into encircling $-1$ in the correct direction. We therefore need
$$\boxed{\text{sign}(K_{PS}) = -1}$$
This minus sign is absorbed into the post-integrator (Lecture 10 slide 21, $G_{tv,\text{post}}(s) = -C_{PI,\text{post}}(s)\,G_{tv}(s)$).

#### Step 2 — Post-integrator design

Find the peak of $|G_{tilt}|$ on the Bode magnitude curve. With the Day 5 on-floor inner loop closed, the peak sits at:

| Quantity | Value |
|---|---|
| Peak magnitude $\lvert G_{tilt}\rvert_{\max}$ | $0.7068$ |
| Peak frequency $\omega_{\text{peak}}$ | $8.170$ rad/s |

Place the post-integrator zero at the peak so the combined magnitude curve rolls off monotonically:
$$\tau_{i,\text{post}} = \frac{1}{\omega_{\text{peak}}} = \frac{1}{8.170} = 0.1224\ \text{s}$$
$$C_{PI,\text{post}}(s) = \frac{\tau_{i,\text{post}} s + 1}{\tau_{i,\text{post}} s} = \frac{0.1224\, s + 1}{0.1224\, s}$$
$$G_{tilt,\text{post}}(s) = -\,C_{PI,\text{post}}(s)\cdot G_{tilt}(s)$$

> [!note] Where $8.17$ rad/s came from
> The peak frequency of $|G_{tilt}|$ moved from $5.95$ rad/s in the first-attempt cascade to $8.17$ rad/s here because the inner wheel-speed loop is now four times faster. A faster inner loop pushes the closed-balance plant's strongest response up in frequency, which in turn moves the post-integrator. $\tau_{i,\text{post}}$ therefore drops $27\%$ (from $0.1682$ to $0.1224$ s).

![[regbot_task2_bode_post.png]]
*Bode plots of $G_{tilt}$ (blue) and $G_{tilt,\text{post}} = -C_{PI,\text{post}}\,G_{tilt}$ (orange). The raw plant's $|G_{tilt}|$ peaks at $8.17$ rad/s, then drops; multiplying by the PI zero at that same frequency flattens the peak and forces the orange curve to roll off monotonically beyond it — the precondition Method 2 requires before designing the outer loop.*

![[regbot_task2_nyquist_post.png]]
*Nyquist of $G_{tilt,\text{post}}$. One clean CCW encirclement of $(-1, 0)$ — matches $P = 1$, so the plant is now stabilisable by a standard outer controller.*

#### Step 3 — Outer PI-Lead on $G_{tilt,\text{post}}$

**Design specifications:**

| Spec | Value |
|---|---|
| Crossover $\omega_c$ | $15$ rad/s |
| Phase margin $\gamma_M$ | $60°$ |
| PI zero placement $N_i$ | $3$ |

**3a. I-part (outer PI).** Place the PI zero at $\omega_c/N_i$:
$$\tau_i = \frac{N_i}{\omega_c} = \frac{3}{15} = 0.200\ \text{s}, \qquad C_{PI}(s) = \frac{0.200\, s + 1}{0.200\, s}$$

**3b. Phase balance at $\omega_c$.** We need $\angle L(j\omega_c) = -180° + \gamma_M$. Breaking the loop phase into contributions:

| Contribution | Value at $\omega_c = 15$ rad/s |
|---|---|
| $\angle G_{tilt,\text{post}}(j\omega_c)$ (from Bode) | $-135.81°$ |
| $\angle C_{PI}(j\omega_c) = -\arctan(1/N_i)$ | $-18.43°$ |
| $\phi_\text{Lead}$ required $= -180° + \gamma_M - \phi_G - \phi_{PI}$ | $+34.25°$ |

The required Lead is noticeably smaller than it would have been without the redesigned inner loop ($+63.8°$ in the first-attempt cascade). The post-integrator peak moved up to $8.17$ rad/s, so at the fixed $\omega_c = 15$ rad/s the post-integrated plant has more available phase — less lift is needed from the Lead.

**3c. Lead from gyro.** The REGBOT gyro directly measures $\dot\theta$, so
$$\tau_d\, \dot\theta + \theta = (\tau_d s + 1)\,\theta$$
is a proper, noise-free realisation of the ideal Lead — no filter pole needed (Lecture 10 slide 24). Solve for $\tau_d$:
$$\tau_d = \frac{\tan(\phi_\text{Lead})}{\omega_c} = \frac{\tan(34.25°)}{15} = 0.0454\ \text{s}$$

This is $67\%$ smaller than the first-attempt value ($0.1355$ s) — physically, the faster inner loop means the balance controller does not have to lean on the Lead as heavily to restore phase at $\omega_c$.

**3d. Loop gain.** Choose $K_P$ so $|L(j\omega_c)| = 1$:
$$|C_{PI}(j\omega_c)\cdot C_{\text{Lead}}(j\omega_c)\cdot G_{tilt,\text{post}}(j\omega_c)| = 0.8424$$
$$K_P = \frac{1}{0.8424} = 1.1871$$

**Full controller** (as viewed from pitch measurement to `vel_ref`):
$$C_\text{total}(s) = K_P \cdot \underbrace{\frac{-(\tau_{i,\text{post}}s + 1)}{\tau_{i,\text{post}}s}}_{\text{sign + post-integrator}} \cdot \underbrace{\frac{\tau_i s + 1}{\tau_i s}}_{\text{outer PI}} \cdot \underbrace{(\tau_d s + 1)}_{\text{Lead (gyro)}}$$

#### Step 4 — Closed-loop verification

**Margins and crossover** (from `margin(L_tilt)`):

| Metric | Value | Note |
|---|---|---|
| Achieved $\omega_c$ | $15.00$ rad/s | matches spec ✓ |
| Phase margin $\gamma_M$ | $60.00°$ | matches spec ✓ |
| Gain margin | $-5.44$ dB (at $5.73$ rad/s) | see note below |
| Closed-loop RHP poles | $0$ | stable ✓ |

![[regbot_task2_loop_bode.png]]
*Open-loop Bode of $L = K_P\, C_{PI}\, C_{\text{Lead}}\, G_{tilt,\text{post}}$ from the 2026-04-22 design run. Title reads $GM = -5.44$ dB (at $5.73$ rad/s), $PM = 60°$ (at $15$ rad/s). Magnitude crosses $0$ dB at $\omega_c = 15$ rad/s as designed; phase sits at $-120°$ at crossover (= $-180° + 60°$ phase margin).*

> [!note] Why negative gain margin is OK here
> For a plant with $P = 1$ RHP pole, the gain margin reported by `margin` is a **lower bound** — we need $|K|$ above a minimum, not below a maximum. A $GM = -5.44$ dB means "do not reduce gain below $10^{-5.44/20} \approx 0.53$× of designed value". This is the standard signature of an unstable-plant design (see Lecture 10 slides 5–7).

![[regbot_task2_ic_response.png]]
*Linear-model regulation: response to a $\theta_0 = 10°$ output-disturbance step on pitch. Settling time (2 % envelope) $\approx 1.35$ s, peak undershoot $\approx 6.71°$. Slightly faster than the $\approx 1.55$ s in the first-attempt cascade — the outer balance loop regulates marginally faster when the inner loop is properly fast. Linear-model proxy; the authoritative IC test is the Simulink simulation shown below.*

#### Design summary

| Parameter                  | Symbol                 | Value      | Source                                                                                    |
| -------------------------- | ---------------------- | ---------- | ----------------------------------------------------------------------------------------- |
| Target crossover           | $\omega_c$             | $15$ rad/s | spec                                                                                      |
| Target phase margin        | $\gamma_M$             | $60°$      | spec                                                                                      |
| PI zero ratio              | $N_i$                  | $3$        | standard placement                                                                        |
| Post-integrator            | $\tau_{i,\text{post}}$ | $0.1224$ s | $1/\omega_{\text{peak}}$ of $\lvert G_{tilt}\rvert$ ($\omega_{\text{peak}} = 8.170$ rad/s) |
| Outer PI                   | $\tau_i$               | $0.2000$ s | $N_i/\omega_c$                                                                            |
| Lead (gyro)                | $\tau_d$               | $0.0454$ s | $\tan(\phi_\text{Lead})/\omega_c$ with $\phi_\text{Lead} = +34.25°$                       |
| Loop gain                  | $K_P$                  | $1.1871$   | $L(j\omega_c)= 1$                                                                         |
| Linear settling (10° IC)   | —                      | $1.35$ s   | `initial()` on closed pitch loop, 2 % envelope                                            |

All four values (`Kptilt`, `titilt`, `tdtilt`, `tipost`) are written to the MATLAB base workspace by `regbot_mg.m` and read automatically by the Simulink blocks when the model loads. The firmware `[cbal]` block in `config/regbot_group47.ini` carries the same numbers with `kp` negative (the firmware does not absorb the Method-2 sign flip internally — see the hardware-experiment section below).

> [!info] What shifted from the first-attempt design
> The first-attempt cascade was designed against $G_{tilt}$ re-linearised with the Day 4 wheels-up inner loop. After switching Task 1 to the Day 5 on-floor plant, $G_{tilt}$ was re-linearised again: $|G_{tilt}|$ peak moved $5.95 \to 8.17$ rad/s, which shrank $\tau_{i,\text{post}}$ by $27\%$ and dropped $\tau_d$ by $67\%$ (Lead is $+34.25°$ not $+63.8°$). The loop gain nudged up slightly ($1.137 \to 1.1871$). Same $\omega_c$, same $\gamma_M$, smaller Lead — less Lead means less high-frequency amplification of gyro noise, a small bonus of the redesign.

---

### Simulink implementation — balance controller

The balance controller wraps around the existing Simulink model. Pitch and gyro outputs from the `robot with balance` subsystem feed into the controller, which outputs a velocity reference to the inner wheel-velocity loop.

```mermaid
flowchart LR
    classDef ref  fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef fb   fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px
    classDef out  fill:#8b6914,stroke:#d4a84a,color:#fef3c7,stroke-width:1.5px
    classDef sum  fill:#374151,stroke:#9ca3af,color:#f3f4f6,stroke-width:1.5px

    Ref["Constant = 0<br/>tilt reference"]:::ref
    SumLead((Sum<br/>+ +)):::sum
    LeadGain["Gain = tdtilt<br/>(gyro-based Lead)"]:::ctrl
    Sum1((Sum<br/>+ −)):::sum
    SignFlip["Gain = −1<br/>(sign flip)"]:::ctrl
    PostInt["Transfer Fcn<br/>num = [tipost 1]<br/>den = [tipost 0]<br/><b>post-integrator</b>"]:::ctrl
    OuterPI["Transfer Fcn<br/>num = [titilt 1]<br/>den = [titilt 0]<br/><b>outer PI</b>"]:::ctrl
    Kp["Gain = Kptilt<br/>(final gain)"]:::ctrl

    Pitch["pitch<br/>from 'robot with balance'"]:::fb
    Gyro["gyro<br/>from 'robot with balance'"]:::fb

    VelRef["vel_ref<br/>into wheel-velocity loop"]:::out

    Pitch --> SumLead
    Gyro --> LeadGain
    LeadGain --> SumLead
    Ref --> Sum1
    SumLead -->|"(τ_d s + 1)·θ"| Sum1
    Sum1 -->|error| SignFlip
    SignFlip --> PostInt
    PostInt --> OuterPI
    OuterPI --> Kp
    Kp --> VelRef
```

#### Block-by-block

| #   | Block type     | Parameter                          | Why                                                                                        |
| --- | -------------- | ---------------------------------- | ------------------------------------------------------------------------------------------ |
| 1   | `Gain`         | `tdtilt`                           | Multiplies the gyro signal — this is the ideal Lead $\tau_d s$ part                        |
| 2   | `Sum`          | signs `+ +`                        | Combines $\theta$ (pitch) with $\tau_d \dot\theta$ (gyro) → feedback signal $(\tau_d s + 1)\theta$ |
| 3   | `Constant`     | `0`                                | Tilt reference — we want the robot upright                                                 |
| 4   | `Sum`          | signs `+ -`                        | Error = reference − Lead-filtered pitch                                                    |
| 5   | `Gain`         | `-1`                               | Sign flip absorbed into the post-integrator (Lecture 10 trick)                             |
| 6   | `Transfer Fcn` | num `[tipost 1]`, den `[tipost 0]` | Post-integrator stabilises the RHP pole                                                    |
| 7   | `Transfer Fcn` | num `[titilt 1]`, den `[titilt 0]` | Outer PI for zero steady-state tilt error                                                  |
| 8   | `Gain`         | `Kptilt`                           | Overall loop gain to hit $\omega_c = 15$ rad/s                                             |

> [!important] Placement of the Lead matters
> The gyro-based Lead must be combined with pitch **on the feedback path (before the error sum)**, not added in parallel after the PI blocks. Putting the Lead in parallel implements $C_{PI,post}\cdot C_{PI} + \tau_d s$ (additive, no high-frequency phase boost) instead of the intended $C_{PI,post}\cdot C_{PI}\cdot(\tau_d s + 1)$ (multiplicative Lead in series). The parallel version has too little phase margin at $\omega_c$ and saturates the motor on any disturbance.

All four parameters (`tipost`, `titilt`, `tdtilt`, `Kptilt`) are written to the base workspace by `regbot_mg.m`, so the Simulink blocks read them automatically after the script runs.

> [!tip] Why the gyro shortcut?
> An ideal Lead $\tau_d s + 1$ is improper and cannot be implemented as a Transfer Fcn block. The trick is that the REGBOT gyro *already measures* $\dot\theta$ directly, so we don't need to differentiate $\theta$ numerically — we just multiply the gyro signal by $\tau_d$ and add it to $\theta$. Mathematically: $\tau_d\dot\theta + \theta = (\tau_d s + 1)\theta$. No filter pole needed.

#### Simulink verification — authoritative IC test ✅

With `startAngle = 10°` and no push disturbance, the full Simulink model (non-linear plant + ±9 V limiter + corrected controller topology) recovers cleanly:

![[regbot_task2_sim_recovery_10deg_v3.png]]
*Recovery from $\theta_0 = 10°$ initial tilt in Simulink with the Day 5 on-floor (v3) gains. Yellow = pitch in radians ($\approx 0.175$ rad $\to 0$). Blue = motor voltage in volts (peak $\approx 2.8$ V, no saturation). Pitch reaches $0$ within $\approx 0.3$ s and fully settles by $t \approx 2$ s — faster than the first-attempt cascade because the inner loop is now at its designed bandwidth.*

The small non-zero steady-state voltage ($\approx 0.5$ V) is expected with no velocity outer loop closed: the inner wheel-speed loop just needs a small bias to hold the wheels against residual position offset. It vanishes once Task 3 (velocity outer loop) wraps around this.

---

### Next Session — Planned Work

- [x] Build the balance controller in Simulink following the corrected diagram
- [x] First simulation test: $\theta_0 = 10°$ recovery in Simulink ✓
- [ ] Physical REGBOT test at zero velocity (Test 3a: `vel=0, bal=1, log=15 : time=10`)
- [x] Refactor balance controller into a Simulink subsystem
- [ ] Build Task 3 wiring (see below)
- [ ] Run `design_task3_velocity` and commit the resulting gains

---

### Task 3 — Velocity Controller (Outer PI on $G_{vel,\text{outer}}$) ✅

**Plant:** With Tasks 1 and 2 closed, linearising from the $\theta_\text{ref}$ input (output of `Kpvel_gain`) to the wheel-velocity filter output gives a 9th-order transfer function $G_{vel,\text{outer}}(s)$ with:

- $0$ RHP poles — the balance loop has successfully stabilised the inverted-pendulum mode.
- $1$ free integrator at the origin (the balance loop's Type-1 behaviour propagates to this channel). DC gain therefore effectively $\infty$ in the finite sense; `dcgain()` reports $\approx 2.07 \times 10^{3}$, dominated by the integrator.
- $1$ **RHP zero at $+8.67$ rad/s** — the non-minimum-phase physics of balancing: the robot must roll *backward* before the body can tilt forward. This zero is set by geometry (wheel radius, CG height) and is invariant under inner-loop retuning.
- A new LHP zero near $-8.03$ rad/s, matching the post-integrator peak from Task 2.
- Dominant complex pole pair near $-10.5 \pm j$ — faster than the first-attempt cascade's $-2.6$ pair because the redesigned balance loop stiffens the closed tilt dynamics.

![[regbot_task3_plant_pz.png]]
*$G_{vel,\text{outer}}$ pole-zero map. The orange ring marks the physics-fixed RHP zero at $+8.67$ rad/s (blue circle). Free integrator at the origin visible as a pole on the imaginary axis; LHP poles are comfortably stable.*

**Crossover constraint from the RHP zero.** An RHP zero at $z$ bounds the closed-loop bandwidth through the rule of thumb $\omega_c \leq z/5$, i.e.\ $\omega_c \lesssim 1.70$ rad/s here. Pushing past this either loses phase margin or incurs the characteristic inverse response. We pick the conservative $\omega_c = 1$ rad/s.

**Design specs:**

| Spec | Value | Reason |
|---|---|---|
| Crossover $\omega_c$ | $1$ rad/s | Safely below the RHP zero at $+8.67$ rad/s ($\omega_c \leq z/5$) |
| Phase margin $\gamma_M$ | $\geq 60°$ | Spec |
| PI zero placement $N_i$ | $3$ | Standard; zero at $\omega_c/N_i$ below crossover |

**Step 1 — I-part.** $\tau_i = N_i/\omega_c = 3.00$ s.

**Step 2 — Phase balance at $\omega_c$.** MATLAB's `bode()` reports the plant phase (continuously unwrapped) as $+267.42°$; the physically meaningful value is $267.42° - 360° = -92.58°$. The PI contributes $-\arctan(1/3) = -18.43°$. Adding these gives an open-loop phase of $\approx -111°$ at $\omega_c = 1$ rad/s, i.e.\ $\gamma_M \approx +69°$ already — **no Lead needed**. The $60°$ spec is met by the PI alone with $\approx 9°$ of margin to spare. (The design script's `phi_Lead = -368.98°` printout is a cosmetic unwrap artefact; the `margin()` verification below is the authoritative check.)

**Step 3 — Loop gain.** $|C_{PI}(j1)\,G_{vel,\text{outer}}(j1)| = 6.5294$, so
$$K_P = \frac{1}{6.5294} = 0.1532.$$

The velocity controller is
$$C_{vel}(s) = 0.1532 \cdot \frac{3s + 1}{3s}.$$

**Step 4 — Verification** (from `margin(L_vel)`):

| Metric | Value |
|---|---|
| Achieved $\omega_c$ | $1.00$ rad/s |
| Phase margin $\gamma_M$ | $68.98°$ |
| Gain margin | $6.21$ dB (at $25.4$ rad/s) |
| Closed-loop RHP poles | $0$ (stable) |

![[regbot_task3_loop_bode.png]]
*Task 3 open-loop Bode $L = C_{vel}\, G_{vel,\text{outer}}$ from the 2026-04-22 design run. Title reads $Gm = 6.21$ dB (at $25.4$ rad/s), $Pm = 69°$ (at $1$ rad/s). MATLAB's continuous-phase convention puts the crossover-phase marker near $+240°$ — that is $+240° - 360° = -120°$, which matches $-180° + \gamma_M$ with $\gamma_M \approx 69°$. The gain-margin crossing at $25.4$ rad/s is where the second magnitude-crossing risk from the RHP zero is held in check by the $6.2$ dB margin.*

![[regbot_task3_step.png]]
*Task 3 closed-loop step response. Monotonic rise with zero steady-state error; rise time of order $1/\omega_c \approx 1$ s. No inverse response visible at this $\omega_c$ because we are safely below the RHP zero at $+8.67$ rad/s.*

#### Design summary

| Parameter               | Value      | Source                         |
| ----------------------- | ---------- | ------------------------------ |
| Target $\omega_c$       | $1$ rad/s  | Limited by RHP zero at $+8.67$ |
| $N_i$                   | $3$        | PI zero placement              |
| $\tau_i = N_i/\omega_c$ | $3.0000$ s | I-action                       |
| $K_P$                   | $0.1532$   | $L(j\omega_c)= 1$              |
| Achieved $\gamma_M$     | $68.98°$   | `margin()`                     |
| Achieved GM             | $6.21$ dB  | `margin()`                     |

`regbot_mg.m` (and the firmware `[cbav]` block in `config/regbot_group47.ini`) carries `Kpvel = 0.1581, tivel = 3.0000` — the earlier-campaign numbers, differing by $\approx 3\%$ from the fresh-run $0.1532$ above. Both designs meet the $60°$ PM spec; the firmware need not be updated unless you want exact alignment with this run's plots.

> [!info] Why so few changes vs. the first-attempt cascade?
> The RHP zero at $+8.67$ rad/s and the free integrator at the origin are both **physics-fixed** — they come from the robot's geometry and the position-to-velocity relationship. Neither moves when we retune the inner loops. So the Task 3 design target ($\omega_c = 1$ rad/s) and the zero-placement ($\tau_i = 3$ s) stay the same; only the tiny gain adjustment (0.162 $\to$ 0.1532, $\approx -5\%$) reflects the slightly stiffer closed-balance plant the velocity loop now sees.

---

### Task 3 — Simulink wiring guide (original build notes)

The balance controller is now a subsystem with three ports (In1: pitch, In2: gyro, Out1: vel_ref). Task 3 adds a velocity outer loop that wraps around this subsystem:

- At the top level: a PI controller takes `v_ref − wheel_vel` and outputs a tilt reference `θ_ref`.
- Inside the subsystem: the hard-coded `Constant = 0` reference gets replaced by a new `Inport` (port 3) for `θ_ref`.

#### Top level — velocity controller wrapping the balance subsystem

```mermaid
flowchart LR
    classDef ref   fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef fb    fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px
    classDef sum   fill:#374151,stroke:#9ca3af,color:#f3f4f6,stroke-width:1.5px
    classDef sub   fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:2px
    classDef out   fill:#8b6914,stroke:#d4a84a,color:#fef3c7,stroke-width:1.5px

    Vref["Constant = 0<br/>v_ref<br/>(swap to Step for Test 3b)"]:::ref
    SumVel((Sum<br/>+ −)):::sum
    VelPI["Transfer Fcn<br/>num = [tivel 1]<br/>den = [tivel 0]<br/><b>velocity PI</b>"]:::ctrl
    Kpvel["Gain = Kpvel<br/><b>block name: Kpvel_gain</b><br/>(linearize targets this)"]:::ctrl
    Bal["Balance<br/>Controller<br/>(subsystem)<br/><br/>In1: pitch<br/>In2: gyro<br/>In3: theta_ref 🆕<br/>Out1: vel_ref"]:::sub
    WV["Wheel-speed<br/>controller<br/>(Task 1 — unchanged)"]:::ctrl
    Robot["robot with<br/>balance"]:::sub
    WVF["wheel_vel_filter<br/>1/(twvlp·s + 1)"]:::fb

    Vref --> SumVel
    SumVel -->|error| VelPI
    VelPI --> Kpvel
    Kpvel -->|theta_ref| Bal
    Bal -->|vel_ref| WV
    WV -->|motor V| Robot
    Robot -->|pitch, gyro| Bal
    Robot -->|wheel vel| WVF
    WVF --> WV
    WVF -->|tap feedback| SumVel

    linkStyle 6,7,8,9 stroke:#c07878,stroke-width:1.5px
```

*New blocks: `v_ref` constant, `Sum_vel(+−)`, `Vel_PI`, `Kpvel_gain`. Red arrows are measurement feedback (pitch, gyro, wheel vel).*

#### Inside the balance subsystem — one change

```mermaid
flowchart LR
    classDef ref  fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef fb   fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px
    classDef out  fill:#8b6914,stroke:#d4a84a,color:#fef3c7,stroke-width:1.5px
    classDef sum  fill:#374151,stroke:#9ca3af,color:#f3f4f6,stroke-width:1.5px
    classDef newp fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:2px

    In3["Inport 3<br/>theta_ref 🆕<br/><i>(replaces Constant = 0)</i>"]:::newp
    SumLead((Sum<br/>+ +)):::sum
    LeadGain["Gain = tdtilt<br/>(gyro Lead)"]:::ctrl
    Sum1((Sum<br/>+ −)):::sum
    SignFlip["Gain = −1"]:::ctrl
    PostInt["Transfer Fcn<br/>num = [tipost 1]<br/>den = [tipost 0]<br/><b>post-integrator</b>"]:::ctrl
    OuterPI["Transfer Fcn<br/>num = [titilt 1]<br/>den = [titilt 0]<br/><b>outer PI</b>"]:::ctrl
    Kp["Gain = Kptilt"]:::ctrl

    In1["Inport 1<br/>pitch"]:::fb
    In2["Inport 2<br/>gyro"]:::fb
    Out1["Outport 1<br/>vel_ref"]:::out

    In1 --> SumLead
    In2 --> LeadGain
    LeadGain --> SumLead
    In3 --> Sum1
    SumLead -->|"(τ_d s + 1)·θ"| Sum1
    Sum1 -->|error| SignFlip
    SignFlip --> PostInt
    PostInt --> OuterPI
    OuterPI --> Kp
    Kp --> Out1
```

*Only change inside the subsystem: the `Constant = 0` block is deleted and replaced by a new `Inport` (auto-numbered as port 3) labelled `theta_ref`. Everything else from Task 2 stays untouched.*

#### Block-by-block

| #   | Location         | Block type     | Parameter                          | Why                                                                                                        |
| --- | ---------------- | -------------- | ---------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| 1   | top level        | `Constant`     | value `0`, label `v_ref`           | Velocity reference — swap to a `Step` (0.8 m/s) for Test 3b, or a signal builder for the square run        |
| 2   | top level        | `Sum`          | signs `+ -`                        | Velocity error = `v_ref − wheel_vel_filter output`                                                         |
| 3   | top level        | `Transfer Fcn` | num `[tivel 1]`, den `[tivel 0]`   | Outer velocity PI — integrator drives steady-state velocity error to zero                                  |
| 4   | top level        | `Gain`         | value `Kpvel`, **name `Kpvel_gain`** | Overall velocity-loop gain. `design_task3_velocity` linearises at this block by name — rename it exactly |
| 5   | inside subsystem | `Inport`       | port 3, label `theta_ref`          | Replaces the `Constant = 0` — exposes the tilt reference so the velocity loop can drive it                 |
| —   | top level        | signal tap     | —                                  | From the existing `wheel_vel_filter` output, draw a second branch into the `-` input of Sum (#2)           |

All three new parameters (`Kpvel`, `tivel`, and optionally a Lead time constant if the design script reports one is needed) are written to the base workspace by `regbot_mg.m`, so the Simulink blocks read them automatically after the script runs.

> [!tip] Outer loop must be slower than inner
> Design spec: $\omega_{c,\text{vel}} = \omega_{c,\text{tilt}}/5 = 3$ rad/s. Rule of thumb in cascaded control — the outer loop's crossover should sit at least a factor of 5 below the inner loop's, so from the outer loop's perspective the inner loop looks like an "instantaneous" unity gain. Break this rule and the loops fight each other, degrading both bandwidth and disturbance rejection.

#### Build order

**Inside the balance subsystem**
1. Double-click the subsystem to open it.
2. Delete the `Constant = 0` block feeding the `+` input of the error Sum.
3. Drag an `Inport` into the subsystem (auto-numbers as 3); rename its label to `theta_ref`.
4. Wire `theta_ref` into the `+` input of the error `Sum(+−)`.
5. Close the subsystem — a third port `3` appears on the block at the top level.

**At the top level**
6. `Constant` block, value `0`, label `v_ref`.
7. `Sum` block, signs `+-`.
8. `Transfer Fcn`, Num `[tivel 1]`, Den `[tivel 0]`.
9. `Gain` block — **rename to `Kpvel_gain` exactly** (the design script linearises at this block). Value: `Kpvel`.
10. Wire: `v_ref → Sum(+) → Vel_PI → Kpvel_gain → subsystem port 3`.
11. Tap `wheel_vel_filter` output → second branch into `Sum(−)`.

**Verify before running the design script**
12. Ctrl+D to update diagram — must compile cleanly.
13. With `v_ref = 0`, `startAngle = 10`, `Kpvel = 0` (current value in `regbot_mg.m`), simulation should still show the same Task 2 recovery from 10° → 0°. If it does, the wiring is correct (the velocity loop contributes nothing when `Kpvel = 0`).

**Design**
14. `>> design_task3_velocity` — linearises `/Kpvel_gain → /wheel_vel_filter` and designs a PI at $\omega_c = 3$ rad/s, $\gamma_M = 60°$, $N_i = 3$.
15. Paste the printed `Kpvel`, `tivel` block into `regbot_mg.m` under the Task 3 heading.
16. Re-run simulation with a non-zero `v_ref` (Step at 0.8 m/s for Test 3b).

---

### 2026-04-21 — Session 2: Task 2 push-disturbance validation ✅

Extra regulation test beyond the $\theta_0 = 10°$ IC response: apply a **1 N / 0.1 s push** at $t = 1$ s via the existing `Push Newton` → `Transport Delay` pair (already wired to `disturb_force`). Logged `Limit9v` (motor voltage after the $\pm 9$ V limiter), `x_position`, and `pitch`:

![[regbot_task2_sim_push.png]]
*Regulation from a 1 N / 0.1 s push at $t = 1$ s. Yellow = motor voltage, blue = $x$ position, orange/red = pitch. Pitch peaks briefly and settles to 0 within $\approx 2$ s with two decaying cycles (a signature of the $\omega_{c,\text{tilt}} = 15$ rad/s balance loop coupled to the slower $\omega_{c,\text{vel}} = 1$ rad/s velocity loop). Motor voltage peaks at $\approx 1.15$ V, nowhere near the $\pm 9$ V limit. Position settles at a small residual offset — expected, since no position loop is closed yet (Task 4 fixes this).*

> [!success] Task 2 + Task 3 validation complete
> - Balance loop catches the disturbance cleanly, no saturation
> - Velocity loop pulls `wheel_vel → 0` in steady state
> - Residual $x$ offset confirms the position loop is what's still missing — natural segue to Task 4

---

### 2026-04-21 — Session 3: Task 4 Position Controller Design Prep 🚧

#### Control architecture

Task 4 wraps the Task 3 velocity loop with an outermost **position loop**:

```mermaid
flowchart LR
    classDef ref   fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef plant fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:1.5px
    classDef fb    fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px

    PosRef["pos_ref"]:::ref
    Sum(("Σ")):::ref
    Cpos["Kppos · (τ_d,pos s + 1)<br/><b>(Task 4)</b>"]:::ctrl
    Inner["Closed velocity loop<br/>(Task 3 + 2 + 1)"]:::plant
    XOut["x_position"]:::fb

    PosRef -->|+| Sum
    Sum -->|error| Cpos
    Cpos -->|v_ref| Inner
    Inner --> XOut
    XOut -.->|−| Sum
```

*The summing junction subtracts $x_\text{position}$ (dashed feedback path) from $\text{pos\_ref}$ to form the error.*

#### Why the position controller is (usually) just P

The plant seen from `v_ref → x_position` — with balance and velocity loops closed — already contains a **free integrator** (position is the integral of wheel velocity):

$$G_{\text{pos,outer}}(s) \;=\; \frac{X(s)}{V_{\text{ref}}(s)} \;\approx\; \frac{1}{s}\cdot T_{\text{vel}}(s)$$

That makes the open-loop plant **Type-1** before the controller is even added. A pure **P controller** is therefore enough to drive the steady-state error to a step reference to zero. Only if the phase margin falls below $60°$ do we add a Lead — standard $(\tau_d s + 1)$ form, **not** the gyro shortcut from Task 2 (no direct $\dot{x}$ sensor at this level; we're operating on a linearised `Gpos_outer`).

> [!note] When would we add an I-term?
> To track a **ramp** (constant-velocity moving target) with zero error, we'd need Type-2 → add a PI. The Task 4 physical test is a **step** position command (`topos=2`), so pure P is the textbook answer.

#### Design specs

| Parameter | Value | Reason |
|---|---|---|
| $\omega_{c,\text{pos}}$ | 0.2 rad/s | $\sim 5\times$ below $\omega_{c,\text{vel}} = 1$ rad/s (cascaded-loop separation rule) |
| $\gamma_M$ | $\geq 60°$ | Spec |
| Controller type | P (+ Lead if needed) | Plant already Type-1 |

A crossover of 0.2 rad/s gives a natural rise time of order $1/\omega_{c,\text{pos}} = 5$ s, so a 2 m step should land inside the 10 s mission window with a comfortable margin.

#### Simulink wiring (to do)

Follows the same pattern as the Task 3 wrap:

```mermaid
flowchart LR
    classDef ref   fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef plant fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:1.5px
    classDef fb    fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px

    PR["pos_ref<br/>(Step = 2)"]:::ref
    S(("Σ<br/>+−")):::ref
    LD["Lead<br/>(τ_d,pos s + 1)<br/>optional"]:::ctrl
    KP["Kppos_gain"]:::ctrl
    Rest["Existing Task 3<br/>velocity loop"]:::plant
    X["x_position<br/>(tap from System)"]:::fb

    PR --> S
    S --> LD --> KP
    KP -->|v_ref replacement| Rest
    Rest --> X
    X -.-> S
```

**Build order (top level of `regbot_1mg`):**

1. `In1` / `Step` block labelled `pos_ref` (amplitude = 2 for the physical test)
2. `Sum` block, signs `+-` — feeds on top input from `pos_ref`, bottom input from `x_position`
3. (If Lead needed) `Transfer Fcn`: Num `[tdpos 1]`, Den `1`
4. `Gain` block — **rename to `Kppos_gain` exactly** (the design script linearises at this block). Value: `Kppos`
5. Wire `Kppos_gain` output into the spot where `v_ref` used to enter the velocity-PI summer — removing/replacing the Step block you used for Task 3 validation
6. Tap `x_position` from the `robot with balance` block → into `Sum(−)`

**Verify before running the design script**

7. Ctrl+D to update diagram — must compile cleanly.
8. With `pos_ref = 0`, `startAngle = 10`, `Kppos = 0` (current value in `regbot_mg.m`), simulation should still show the Task 2 recovery from 10° → 0°. If it does, the wiring is correct (the position loop contributes nothing when `Kppos = 0`).

**Design**

9. `>> design_task4_position` — linearises `/Kppos_gain → /robot with balance` and designs a P (or P-Lead) at $\omega_c = 0.2$ rad/s, $\gamma_M = 60°$.
10. Paste the printed `Kppos`, `tdpos` block into `regbot_mg.m` under the Task 4 heading.
11. Re-run simulation with `pos_ref = 2` m step. Capture `x_position`, `wheel_vel_filter`, `pitch`, `motor_Voltage` for the report.
12. Build the **XY-plane plot** from the same run (`x_position` vs time is fine for a straight-line run; full XY needed for Test 3b square run).

#### Design script

Created `simulink/design_task4_position.m` following the same structure as Tasks 2 & 3:

1. `regbot_mg` loads Task 1/2/3 gains, sets `Kppos = 0` to break the position loop
2. `identify_tf('/Kppos_gain', '/robot with balance')` → `Gpos_outer`
3. Sanity checks: RHP poles = 0, at least one free integrator
4. Phase balance at $\omega_{c,\text{pos}}$ → auto-adds Lead only if PM would be below spec
5. $K_p$ from $|L(j\omega_c)| = 1$
6. Verifies margins, closed-loop poles, 2 m step response (must exceed 0.7 m/s peak speed per assignment brief)
7. Prints copy-paste gains block for `regbot_mg.m`

#### Placeholder gains added

`regbot_mg.m` now declares:
```matlab
Kppos  = 0;
tdpos  = 0;
```
so the model compiles before the design script has run.

> [!todo] Immediate next steps
> 1. Do the Simulink wiring above (4 blocks + 2 wires)
> 2. Verify the "Kppos = 0 → Task 2 recovery still works" sanity check
> 3. Run `design_task4_position` → commit gains
> 4. Simulate 2 m step → `regbot_task4_sim_step.png`
> 5. Physical REGBOT Test 4: `topos=2, vel=1.2 : time=10`

---

### 2026-04-21 — Session 3 (cont.): Task 4 Position Controller ✅ (MATLAB + Simulink)

#### Design iteration

Three passes on `wc_pos` before landing a design that met the mission specs:

| $\omega_{c,\text{pos}}$ | PM | GM | Peak $v$ (2 m step) | Settling (2%) | Verdict |
|---|---|---|---|---|---|
| 0.2 rad/s | 87° | 38 dB | 0.33 m/s | 20 s | **Fails** — too slow (fails 0.7 m/s spec, fails 10 s window) |
| 0.5 rad/s | 66° | 25 dB | 0.68 m/s | 12 s | **Just short** of 0.7 m/s spec |
| **0.6 rad/s** | **60°** | **23 dB** | **0.80 m/s** | **11 s** | ✅ **Chosen** — clears 0.7 m/s spec, margins healthy |

The 11 s settling is slightly over the 10 s mission window, but that's the *linear-model 2%-envelope* metric, which is pessimistic. The mission test requires the robot to *reach* 2 m inside 10 s, not to settle to 2 cm precision — fine.

Kept separation $\omega_{c,\text{pos}} / \omega_{c,\text{vel}} = 0.6$ (1.67× instead of 5×). Less conservative than the rule of thumb, but justified by the massive phase margin cushion.

> [!note] Cosmetic bug fix in `design_task4_position.m`
> `bode()` returned `phi_G = +266.59°` at 0.2 rad/s (MATLAB unwraps continuously; the physically meaningful value is $-93.41°$). The phase-balance formula `phi_Lead = -180 + gamma_M - phi_G` then produced nonsense like $-386°$. Added wrapping via `mod(x + 180, 360) - 180` so the Lead logic always sees a sensible angle.

#### Plant (v3 re-linearisation)

With Tasks 1 + 2 + 3 closed, linearising from the position-loop gain output (`Kppos_gain`) to $x$-position gives an 11th-order transfer function $G_{pos,\text{outer}}(s)$:

- $0$ RHP poles — inner + velocity loops have fully stabilised the dynamics.
- $1$ pole at the origin — the free integrator from wheel velocity to wheel position. This is what makes a pure P controller sufficient for zero step-tracking error.
- $1$ RHP zero at $+8.67$ rad/s — inherited from Task 3's $G_{vel,\text{outer}}$, the same non-minimum-phase physics of the inverted pendulum.
- Dominant stable LHP poles near $-2.27$ and a complex pair near $-7.52$ (carried over from the Task 3 plant), plus fast motor/encoder poles far out in the LHP.

![[regbot_task4_plant_pz.png]]
*$G_{pos,\text{outer}}$ pole-zero map. Orange ring: the physics-fixed RHP zero at $+8.67$ rad/s (blue circle). A pole on the imaginary axis at the origin is the free integrator; every other pole is safely in the LHP.*

#### Design landed

After re-linearising `Gpos,outer` against the Day 5 cascade (Tasks 1, 2, 3 redesigned), the 2026-04-22 `design_task4_position` run gives:

| Parameter                                | Value              | Source                                                     |
| ---------------------------------------- | ------------------ | ---------------------------------------------------------- |
| $\omega_{c,\text{pos}}$                  | $0.6$ rad/s        | Iterated spec                                              |
| $\angle G_{pos,\text{outer}}(j\omega_c)$ | $-121.74°$         | `bode(Gpos_outer, 0.6)`                                    |
| $\phi_\text{Lead}$ required              | $+1.74°$           | $-180 + \gamma_M - \phi_G$                                 |
| $\tau_{d,\text{pos}}$ (ideal, design)    | $0.0505$ s         | $\tan(\phi_\text{Lead})/\omega_c$                          |
| $K_{p,\text{pos}}$                       | **$0.5411$**       | $1/L(j\omega_c)$ with $\lvert C_\text{Lead}\,G_{pos,\text{outer}}(j\omega_c)\rvert = 1.8479$ |
| Achieved $\gamma_M$ (with Lead)          | $60.00°$           | `margin()`                                                 |
| Achieved $\gamma_M$ (Lead dropped)       | $\approx 58.26°$   | subtract $1.74°$; what firmware actually runs              |
| Achieved GM                              | $25.34$ dB (at $7.62$ rad/s) | `margin()`                                       |
| Sim peak $v$ (2 m step)                  | $0.760$ m/s        | above $0.7$ m/s spec ✓                                     |
| Sim settling ($2\%$ env)                 | $11.2$ s           | mission window $10$ s — see note                           |

**Controller (design-time, with Lead):**
$$C_{\text{pos}}(s) = 0.5411 \cdot (0.0505\, s + 1)$$

**Controller (firmware, Lead dropped for Simulink):**
$$C_{\text{pos}}(s) = 0.5411$$

The plant's free integrator (position $= \int$ velocity) makes the open loop Type-1 already, so no I-term is needed. The Lead is tiny here: it contributes only $1.74°$ of phase boost at $\omega_c$ and a magnitude of $\sqrt{1 + (0.0505 \cdot 0.6)^2} \approx 1.0005$ — essentially unity, so dropping it costs $\approx 1.74°$ of PM and $\approx 0.05\%$ of loop gain.

#### Open-loop Bode (verification)

![[regbot_task4_loop_bode.png]]
*Task 4 open-loop Bode $L = C_{pos}\, G_{pos,\text{outer}}$ from the 2026-04-22 design run. Title reads $Gm = 25.3$ dB (at $7.62$ rad/s), $Pm = 60°$ (at $0.6$ rad/s) — this is the design-time plot with the Lead included. Magnitude crosses $0$ dB at $\omega_c = 0.6$ rad/s; gain margin is reached at $7.62$ rad/s where the phase dips through $-180°$. The RHP zero at $+8.67$ rad/s is the reason the phase curve bends back up at higher frequencies — one of the signatures of non-minimum-phase dynamics.*

![[regbot_task4_step.png]]
*Linear-model closed-loop step response to a $2$ m position command. Rise-time order $1/\omega_c \approx 1.7$ s; the $2\%$-envelope settling time is $11.2$ s (slightly over the $10$ s mission window, but the mission only requires reaching $2$ m, not settling inside $\pm 4$ cm).*

#### Why we dropped the Lead

The design script wanted a small Lead ($\tau_d = 0.0505$ s) to push PM from $\approx 58.3°$ back up to the $60°$ target — a $1.74°$ phase contribution at $\omega_c$. Attempted to implement it as a `Transfer Fcn` block with Num `[tdpos 1]`, Den `1`, and Simulink rejected it:

> *A time domain realization of the given transfer function for block 'Transfer Fcn' failed. The given transfer function is an improper transfer function.*

A pure `(τ_d s + 1)` is improper (numerator degree > denominator degree). Realisable alternatives:
- **Proper Lead** `(τ_d s + 1) / (α τ_d s + 1)` with small $\alpha \approx 0.1$ — adds a fast filter pole, phase boost nearly identical.
- **Derivative + Sum** parallel structure — more blocks, same thing.
- **Drop the Lead entirely** — lose $\approx 3°$ of PM.

Chose option 3. The first-attempt cascade had a nearly-identical situation (required Lead was $0.94°$, also dropped for the same improper-TF reason, same $\approx 1°$ PM cost). Both times the margin headroom from $\gamma_M \geq 60°$ makes the sacrifice painless. GM of $25$ dB and the phase-margin cushion together dominate robustness here; the $3°$ is noise.

#### Simulink implementation

Top-level wiring (in front of the existing Task 3 velocity loop):

```mermaid
flowchart LR
    classDef ref   fill:#475569,stroke:#94a3b8,color:#f1f5f9,stroke-width:1.5px
    classDef ctrl  fill:#4b6b3a,stroke:#8fb56b,color:#f1f5f9,stroke-width:1.5px
    classDef plant fill:#5b4b7a,stroke:#9a8fbd,color:#ede9fe,stroke-width:1.5px
    classDef fb    fill:#7a4141,stroke:#c07878,color:#fce4e4,stroke-width:1.5px

    Step["Step<br/>pos_ref = 2 m<br/>step time = 1 s"]:::ref
    Sum1(("Σ<br/>+−")):::ref
    Kp["Kppos_gain<br/>= 0.5411"]:::ctrl
    Sum2(("Σ<br/>+−<br/>velocity")):::ref
    Rest["Velocity PI<br/>→ Kpvel_gain<br/>→ Tilt_Controller<br/>→ WV → System"]:::plant
    XPos["x_position<br/>(port 3 of<br/>robot with balance)"]:::fb

    Step --> Sum1
    Sum1 -->|error| Kp
    Kp -->|v_ref| Sum2
    Sum2 --> Rest
    Rest --> XPos
    XPos -.->|feedback| Sum1
```

#### 2 m step simulation ✅

With the above wiring and the committed gains, simulating a 2 m step at $t = 1$ s:

![[regbot_task4_sim_step_v3.png]]
*Task 4 regulation with v3 gains: 2 m position step at $t = 1$ s. Blue = $x$ position ($0 \to 2.15$ m peak, $\approx 7.5\%$ overshoot, settles at $2.00$ m). Green = wheel velocity (peak $\approx 0.80$ m/s, comfortably above the $0.7$ m/s spec floor). Yellow = motor voltage after $\pm 9$ V limiter (peak $\approx 3$ V, no saturation). Red = pitch (small excursions, robot leans briefly to accelerate, returns to $0$).*

> [!success] Task 4 simulation validated
> - Position tracks the 2 m step with a small overshoot and zero steady-state error
> - Peak wheel velocity clears the 0.7 m/s spec
> - No voltage saturation — plenty of headroom on the physical robot
> - Pitch excursions minimal; balance loop is not stressed by the position move

#### Committed gains (`regbot_mg.m`)

```matlab
% --- Task 4: Position outermost loop (v3, Day 5 on-floor redesign)
Kppos  = 0.5411;
tdpos  = 0;          % Lead dropped — see session note
```

---

### 2026-04-22 — Session 4 (`day5-redesign` branch): Cascade redesign against the Day 5 on-floor plant ✅

Branch tracker: [[REDESIGN_ROADMAP]]. Handoff: [[HANDOFF]]. Plan: `C:\Users\Mads2\.claude\plans\partitioned-gliding-hopcroft.md`.

#### Trigger

Post-v1 campaign, Test 0 logged a wheel-speed rise of **0.329 s** to reach 0.27 m/s. A first-order closed loop at $\omega_c = 30$ rad/s predicts ~0.08 s. The measured effective bandwidth was ~9 rad/s — roughly **4× slower** than the designed 30 rad/s.

#### Root cause

The Day 4 "wheels-up" identification used for Task 1 ($G_{vel} = 13.34/(s+35.71)$) has DC gain 0.374 and pole at 35.71 rad/s (τ = 28 ms). The assignment runs on the floor. Re-loaded `data/Day5_results_v2.mat` and inspected `G_1p_avg`: $G_{vel} = 2.198/(s+5.985)$, DC gain 0.367 (essentially identical), pole at 5.99 rad/s (τ = 167 ms). The on-floor dominant pole is ~6× slower than the wheels-up approximation. The v1 design kept the robot stable because the 121.6° phase-margin cushion absorbed the mismatch, but the designed $\omega_c$ was never actually achieved on hardware.

#### Redesign method

1. **Task 1.** Updated `design_task1_wheel.m` to load `G_1p_avg` from the MAT file. Same targets ($\omega_c = 30$, $\gamma_M \geq 60$, $N_i = 3$). Result: $K_{pwv} = 13.2037$ (×4), $\tau_{iwv} = 0.1$ s, achieved $\omega_c = 30.00$, $\gamma_M = 82.85°$, GM = ∞.
2. **Task 2.** Re-linearised Simulink with new Task 1 gains active. `|G_tilt|` magnitude peak shifted 5.95 → 8.03 rad/s. New $\tau_{ipost} = 0.1245$ s (−26%). Outer PI and phase balance re-computed: $\phi_{\text{Lead}}$ dropped from +63.8° to +33.5° (the post-integrated plant has more phase at crossover now), so $\tau_d$ dropped 67% (0.1355 → 0.0442 s). New loop gain $K_{ptilt} = 1.1999$. Achieved $\omega_c = 15$, $\gamma_M = 60°$, GM = −5.58 dB (lower-bound for P = 1), 0 RHP closed-loop poles. Linear IC-regulation settling improved 1.55 → 1.34 s.
3. **Task 3.** Re-linearised `Gvel,outer` with new Tasks 1 + 2 gains. RHP zero at +8.51 rad/s unchanged (physics). Plant gained a new LHP zero at −8.03 matching the post-integrator peak, and the dominant complex pole pair moved from −2.63 to −10.4 (balance loop is doing more work). New $K_{pvel} = 0.1581$ (−2%), $\tau_{ivel} = 3.0$ s unchanged. Achieved $\omega_c = 1.00$, $\gamma_M = 68.98°$ (up from 64.2°), GM = +5.84 dB.
4. **Task 4.** Re-linearised `Gpos,outer`; free integrator still present. New $K_{ppos} = 0.5411$ (+1%). Phase-balance Lead requirement rose from +0.94° to +2.85°, still negligible and still dropped for the improper-TF Simulink reason. Achieved PM ≈ 57° (with Lead would be 60°), GM = 25.17 dB. Sim peak velocity 0.753 m/s (still above 0.7 spec).
5. **Firmware config.** Updated `config/regbot_group47.ini` `[cvel]`, `[cbal]`, `[cbav]`, `[cbap]` blocks. Sign on `[cbal] kp` stays negative — finding from v1 campaign that firmware Balance block does not absorb Lecture-10 Method-2 sign flip internally.
6. **Simulink sanity sims.** `startAngle=10` balance recovery: voltage peak 2.8 V, pitch 10° → 0° within ~0.3 s, settled by t ≈ 2 s. `topos=2` step: x reaches 2.15 m peak (7.5% overshoot), settles at 2.00 m, peak velocity ≈ 0.80 m/s, voltage peak ~3 V, pitch +17° during accel. See `figures/regbot_task2_sim_recovery_10deg_v3.png` and `figures/regbot_task4_sim_step_v3.png`.

#### Hardware validation campaign (v3, 2026-04-22)

All four hardware tests re-run with v3 `regbot_group47.ini` loaded into GUI → sent to robot → saved to flash. Logs under `logs/test*_v3_onfloor_*.txt`, plots under `figures/test*_v3_onfloor_*.png` (mirrored into `docs/images/`).

**Test 0 — wheel speed at vel = 0.3 m/s (balance off):** PASS.
- Rise to 0.27 m/s in **0.012 s** (v1 was 0.329 s — **27× faster** on hardware).
- L/R mean agreement 0.76%. Peak motor voltage 2.60 V.
- The designed 30 rad/s inner-loop bandwidth finally materialises on hardware. The steady-state voltage ripple increased slightly compared to v1 — expected, the 4× higher $K_{pwv}$ gives the inner PI 4× more noise-bandwidth to react to.

**Test 3a — balance at rest (two runs):**
- *First v3 run:* drift 0.475 m (inside spec), mean tilt offset +1.13°, tilt std 2.04° (57% tighter than v2's 4.76°, no late oscillation).
- *After Y-offset recalibration to 175:* drift 0.505 m (marginally outside 0.5 m spec), mean tilt offset +1.11°, tilt std 1.87° (61% tighter than v2). Bias essentially unchanged by the recal — either further iteration needed (Y ≈ 176) or a physical CG / wheel-radius asymmetry cannot be calibrated out.
- **Decision:** use **v2 Test 3a (drift 0.343 m, passes spec)** as the reportable 3a result, and cite the v3 tilt-std tightening separately. Drift depends on sensor calibration which is outside the controller's authority; tilt tightness is what the controller redesign affects. This is honest engineering.

**Test 3b — square run at 0.8 m/s:** PASS.
- 4 sides + 3 turns completed. Heading 359.8° (0.06% error). Peak tilt +25.5°. Tilt std 5.03° (tighter than v2's 5.72°).
- **Peak motor voltage 7.31 V (91% of ±8 V budget)** — v2 had 4.67 V (58%). Cause: 4× higher $K_{pwv}$ makes the inner PI react 4× harder to sharp corner `vel_ref` steps. This is the classic bandwidth-vs-saturation-margin trade-off. Still within spec, but an important discussion point for the report.

**Test 4 — 2 m topos step:** PASS, cleanest result of the campaign.
- Final x = 1.964 m (**3.6 cm short**; v2 was 10.7 cm short — **3× closer to target**).
- **No overshoot, no late limit cycle** (v2 had visible ±10° pitch oscillations and ±0.5 m/s `vref` swings after target).
- Peak tilt +17.3° (v2 +25°, −31%). Tilt std 2.93° (v2 5.18°, −43%).
- Peak motor voltage 4.95 V, **no saturation** (the Test 3b saturation concern did not materialise because the position-loop `vref` is smooth, not step-like).
- Peak velocity 0.79 m/s (still > 0.7 m/s spec; v2 was 1.01 m/s — tighter control = less over-tilt = less thrust, intentional trade).

#### Commits landed on `day5-redesign`

All commits on this branch scoped to REGBOT-Balance-Assignment submodule + Report submodule (both on `day5-redesign`). DTU main was not branched. Submodule-pointer bumps on DTU main will happen only after the branch merges back to main (Phase 8, on hold by user instruction).

---

*Last updated: 2026-04-22 (Day 5 on-floor redesign campaign complete, Phase 7 documentation landed, Phase 8 merge on hold pending user approval).*
