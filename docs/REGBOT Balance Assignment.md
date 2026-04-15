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

## Control Architecture Overview

The REGBOT balance problem requires a **cascaded control** structure with multiple nested loops:

```
Position ref ──► [Position Ctrl] ──► Velocity ref ──► [Velocity Ctrl] ──► Balance ref
                                                                               │
                                                                               ▼
                                                       [Balance Ctrl] ──► Voltage ──► REGBOT
```

Each task in this assignment builds the next inner/outer loop in this cascade.

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
*$G_{wv}$: Motor voltage → wheel velocity. Low-frequency DC gain matches physical expectation.*

![[regbot_Gtilt_bode.png]]
*$G_{tilt}$: Velocity reference → tilt angle. This is the plant the balance controller will see.*

**Pole–zero maps (shaded stability regions):**

![[regbot_Gwv_pzmap.png]]
*$G_{wv}$ in the s-plane. The RHP pole (highlighted) is the unstable pendulum mode.*

![[regbot_Gtilt_pzmap.png]]
*$G_{tilt}$ in the s-plane. One RHP pole — the same physical falling mode.*

**Zoomed pole–zero maps (focus on the slow dynamics around origin):**

![[regbot_Gwv_pzmap_zoom.png]]
*$G_{wv}$ zoomed to ±50 rad/s. The RHP pole at $\approx +10.6$ rad/s is clearly visible with its orange ring.*

![[regbot_Gtilt_pzmap_zoom.png]]
*$G_{tilt}$ zoomed to ±50 rad/s. RHP pole at $\approx +8.7$ rad/s is the pendulum falling mode the balance controller must stabilise. Nearby LHP poles and zeros show the slow dynamics.*

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

**Plant used:** Day 5 black-box identification $G_{vel}(s) = \dfrac{13.34}{s + 35.71}$
(Chosen over the linearized $G_{wv}$ because the assignment specifies the Day 5 TF, and because the inner velocity loop should be designed in isolation from the unstable tilt mode.)

**Design choices:**

| Parameter | Value | Reason |
|---|---|---|
| $\omega_c$ | 30 rad/s | Fast inner loop, still below $\omega$ of plant pole (35.71) |
| $\gamma_M$ (target) | $\geq 60°$ | Spec |
| $N_i$ | 3 | PI zero placed 3× below crossover |

**Computed values:**

| Parameter                      | Value                                |
| ------------------------------ | ------------------------------------ |
| $\tau_i = N_i/\omega_c$        | **0.10 s**                           |
| $K_p$ (from $L(j\omega_c)= 1$) | **3.31**                             |
| Achieved $\omega_c$            | ~30 rad/s                            |
| Achieved $\gamma_M$            | ~121° (well above 60° spec — robust) |

**Controller:**
$$C_{wv}(s) = 3.31 \cdot \frac{0.1s + 1}{0.1s}$$

**Simulink:** The starter model variables `Kpwv` and `tiwv` have been updated to these design values.

![[regbot_task1_bode.png]]
*Task 1 open-loop Bode with phase and gain margins marked.*

![[regbot_task1_step.png]]
*Task 1 closed-loop step response — confirms $e_{ss} = 0$ and low overshoot.*

---

### Next Session — Planned Work

- [ ] **Task 2: Balance controller design**
    - Use $G_{tilt}$ as the plant
    - Design PILead with **post-integrator** (2nd PI block)
    - Target: 1 CCW encirclement of $-1$ on Nyquist (because $P = 1$)
    - Verify phase margin $\geq 60°$ on Bode plot
- [ ] Integrate balance controller into Simulink model
- [ ] First simulation test: can the robot balance in place?
- [ ] Physical REGBOT test at zero velocity (Test 3a)

---

*Last updated: 2026-04-15*
