---
course: "34722"
course-name: "Linear Control Design 1"
type: plan
tags: [LCD, assignment, REGBOT, plan]
date: 2026-04-15
---
# REGBOT Balance Assignment — Plan

**Group 47** — Andreas (s241123), Jonas (s240324), Mads (s246132), Sigurd (s245534)
**Deadline:** 17 May 2026
**Today:** 15 April 2026 → **32 days / ~4.5 weeks available**

---

## Overall Strategy

The 4 tasks are **sequentially dependent** (each outer loop wraps the previous inner loop), but team members can parallelize:
- **Design work** — one pair designs the controller while the other pair prepares Simulink
- **Testing** — all 4 in lab together, run experiments in parallel (one runs robot, one logs, two analyze)
- **Report** — each person owns the section of the task they led, but all review

Sequence: **Task 1 → Task 2 → Task 3 → Task 4 → Report**

---

## Phase 1 — Foundation (Week 1: Apr 15–21)

**Goal:** Everyone understands the problem and environment works.

- [ ] **All:** Watch `REGBOT balance introduction.mp4`
- [ ] **All:** Review Lecture 10 slides — unstable systems, Nyquist stabilisation, cascaded control
- [ ] **All:** Install MATLAB packages: Simscape Multibody, Simulink Control Design
- [ ] **All:** Read through [[REGBOT Balance Assignment]] notes and [[Fundamentals - Intuitive Control Theory]] sections 9 (Nyquist) and 11 (Type-n)
- [ ] **1 person:** Download starter files `regbot_1mg.slx` and `regbot_mg.m` from Learn → Resources/REGBOT balance resources
- [ ] **1 person:** Calibrate gyro and tilt-offset on physical REGBOT
- [ ] **Meeting:** Team discusses control architecture and divides roles

### Suggested role split

| Person | Primary | Secondary |
|---|---|---|
| Andreas | Task 1 + Simulink | Report |
| Jonas | Task 2 + REGBOT testing | Report |
| Mads | Task 3 + Plot/figures | Report |
| Sigurd | Task 4 + Report lead | Testing |

---

## Phase 2 — Task 1: Wheel Speed Controller (Week 1: Apr 18–21)

**Goal:** Working PI velocity loop in Simulink.

- [ ] Retrieve voltage-to-velocity TF from [[Day 5 - Black Box Modeling|Day 5]]: $G_{vel}(s) = \frac{13.34}{s + 35.71}$
- [ ] Design PI: choose $\omega_c$, $\gamma_M$, $N_i$ → compute $\tau_i$, $K_P$
- [ ] MATLAB script: `src/task1_wheel_speed.m`
- [ ] Bode plot → verify $\gamma_M$ and $\omega_c$ → save `figures/task1_bode.png`
- [ ] Simulink step response → save `figures/task1_step_sim.png`
- [ ] REGBOT test → log → `figures/task1_step_regbot.png`
- [ ] Write Task 1 section in report

> [!tip] Shortcut
> You already did a similar PI design in Day 8/9. Reuse those parameters as a starting point, then tune if needed.

---

## Phase 3 — Task 2: Balance Controller (Weeks 2–3: Apr 22–May 2)

**Goal:** REGBOT stays upright at zero velocity.

This is the **hardest** task. Budget more time here.

### Sub-phase 3a: Design (Apr 22–25)
- [ ] Identify the balance plant transfer function (angle dynamics, unstable — has RHP pole)
- [ ] Design PILead + **post-integrator** (second PI block)
- [ ] Include post-integrator phase in phase balance equation
- [ ] Plot Nyquist → verify **CCW encirclement of $-1$** (Nyquist criterion: $Z = N + P$)
- [ ] MATLAB script: `src/task2_balance.m`
- [ ] Bode plot (full loop) → `figures/task2_bode.png`

### Sub-phase 3b: Simulation (Apr 26–28)
- [ ] Integrate controller into Simulink model
- [ ] Simulate with Simscape Multibody (3D dynamics) → `figures/task2_step_sim.png`
- [ ] Verify robot stays upright for $>10$ s in simulation

### Sub-phase 3c: REGBOT test (Apr 29–May 2)
- [ ] Test 3a mission: `vel=0, bal=1, log=15 : time=10`
- [ ] Check drift stays within 0.5 m
- [ ] Log and save `figures/task3a_regbot.png`
- [ ] Write Task 2 section in report

> [!warning] Expected Issue
> The post-integrator trips up many students. Treat it as a regular PI when counting phase and gain — **don't forget to include it in $|L(j\omega_c)| = 1$**.

---

## Phase 4 — Task 3: Velocity Controller (Week 3: May 3–9)

**Goal:** REGBOT drives square run at 0.8 m/s while balancing.

- [ ] Identify outer plant (closed balance loop seen from velocity side)
- [ ] Design PILead controller
- [ ] MATLAB script: `src/task3_velocity.m`
- [ ] Bode + step response plots
- [ ] Simulink test
- [ ] REGBOT Test 3b: square run
    - Mission script in `missions/task3b_square.txt`
    - Side length 1 m, turning radius 0.2 m
    - Record XY trajectory
- [ ] Write Task 3 section in report

---

## Phase 5 — Task 4: Position Controller (Week 4: May 10–14)

**Goal:** REGBOT moves 2 m at max 0.7+ m/s then stops.

- [ ] Identify outer plant (velocity loop seen from position side)
- [ ] Design PILead
- [ ] MATLAB script: `src/task4_position.m`
- [ ] Simulation
- [ ] REGBOT Test 4:
    ```
    vel=0, bal=1, log=15 : time=2
    topos=2, vel=1.2 : time=10
    ```
- [ ] Record data, make the required **cool XY-plane plot** → `figures/task4_xy.png`
- [ ] Write Task 4 section in report

---

## Phase 6 — Report Polish & Submission (Week 5: May 14–17)

**Goal:** 5-page PDF submitted on Learn.

- [ ] Each person finalizes their section
- [ ] All review and polish
- [ ] Fill in module responsibility table
- [ ] Fill in time-spent table
- [ ] Compile and proofread
- [ ] Name PDF: `Group_47.pdf`
- [ ] Submit on Learn → Course Content → Assignments → REGBOT balance
- [ ] Upload videos (optional) and add links

> [!important] Report Style
> Assignment says: **"Be precise, accurate and short."** 5 pages is a hard limit — cut aggressively. Prefer plots over prose.

---

## Risk Mitigation

| Risk | Mitigation |
|---|---|
| Balance controller doesn't work on robot | Start Phase 3c early; have fallback simpler tuning |
| Simulation doesn't match reality | Document discrepancy in report (comments section) |
| Robot hits voltage saturation ($\pm 9$ V) | Lower crossover frequency → less aggressive control |
| Group member unavailable | Each person's section written in standalone form so others can finish |
| Last-minute bug before submission | Aim for draft by May 14, leave 3 days for polish |

---

## Quick Links

- [[REGBOT Balance Assignment|Assignment Brief]]
- [[Fundamentals - Intuitive Control Theory|Fundamentals Guide]]
- [[Diagnostic Guide - What Went Wrong|Diagnostic Guide]]
- [[Worked Example - REGBOT Position Controller|Day 8-9 Worked Example]]
- MATLAB repo: `4. Semester/Linear Control Design/REGBOT-Balance-Assignment/`

---

*Last updated: 2026-04-15*
