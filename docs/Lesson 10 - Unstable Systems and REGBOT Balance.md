---
course: "34722"
course-name: "Linear Control Design 1"
type: lecture-note
lesson: 10
tags: [LCD, lecture, notes]
date: 2026-04-15
---

# Lesson 10 - Unstable Systems and REGBOT Balance

> [!abstract] Lecture Overview
> Lesson 10/13 — Teachers: Silvia Tolu & Dimitrios Papageorgiou
> Topics: What makes a system unstable, how to stabilise open-loop unstable plants via the Nyquist criterion, and the cascaded control architecture used for the REGBOT balance (mandatory) assignment.
> Schedule: 13:00–15:00 Lecture, 15:00–17:00 Exercises (REGBOT balance group work).

> [!example] Related Materials
> - Slides: [[Lecture_10_Unstable_systems.pdf]]
> - MATLAB: `Stabilisation_open_loop_unstable.m`
> - Exercises: [[Theoretical Exercises LCD1.pdf]] — problems 1, 2, 4, 5, 6, 7, 9
> - Assignment: [[REGBOT Balance Assignment]] (mandatory)
> - Previous: [[Lesson 9 - PI-Lead Design with Specifications]]

---

## Learning Objectives

- Explain the concept of instability in dynamical systems
- Use Nyquist plots to stabilise open-loop unstable systems
- Explain the concept of cascaded control for REGBOT balance

---

## 1. Unstable Systems — What They Are and Why They Matter

Up until now every plant we controlled was already **stable** on its own — we only added controllers to make it faster, more accurate, or more disturbance-tolerant. From this lecture on, we face plants that don't even stay put without a controller. The textbook example is the inverted pendulum: balance a broomstick on your palm. Nudge it slightly and it falls. The REGBOT is exactly this — a cart with a pendulum on top that wants to fall over. Our job is to design a controller that keeps it upright.

### 1.1 Three Equivalent Ways to Say "Unstable"

A system is unstable when any of these hold (they are equivalent for LTI systems):

- **Physically:** small perturbations drive the system away from equilibrium instead of back toward it.
- **Input–output:** a bounded input produces an unbounded output. Example: apply a constant motor torque to a frictionless wheel and its angular position grows without limit.
- **Pole location:** at least one pole lies in the **right half-plane (RHP)**, *or* there are repeated poles exactly on the imaginary axis.

The RHP pole condition is the one we work with day-to-day, because poles come straight out of the transfer function denominator.

### 1.2 Two Canonical Examples

**Single RHP pole:**
$$G_1(s) = \frac{1}{s - 1} \quad \Leftrightarrow \quad \dot{y} = y + u(t) \quad \Rightarrow \quad y(t) = y_0 e^t + \int_0^t e^{t-\tau} u(\tau)\, d\tau$$

The free response $y_0 e^t$ grows exponentially. Even with a bounded input the output blows up — the $e^t$ kernel inside the convolution integral amplifies past contributions more and more.

**Repeated poles on the imaginary axis:**
$$G_2(s) = \frac{1}{(s^2 + 1)^2} \quad \overset{u(s)=1}{\Longleftrightarrow} \quad y^{(4)} + \ddot{y} + y = \delta(t) \quad \Rightarrow \quad y(t) = 0.5 \sin(t) - 0.5 t \cos(t)$$

A single pair at $\pm j$ would give a pure sinusoid (marginally stable). Repeating the pair introduces the $t\cos(t)$ term — the amplitude grows linearly forever. This is why the "no repeated $j\omega$ poles" rule exists.

> [!question] The Core Question of This Lecture
> If the plant already has an RHP pole, *no choice of reference* will make it behave. The only escape is feedback. So: **how do we close a loop around an unstable plant and end up with a stable closed-loop system?**

---

## 2. Nyquist Plots — What They Are and How to Read Them

The rest of this lecture uses Nyquist plots heavily, so it's worth pausing here to build the intuition.

### 2.1 The Plot Itself

A Bode plot shows $|G(j\omega)|$ and $\angle G(j\omega)$ as **two separate graphs** against frequency. A Nyquist plot squashes them into **one graph** by treating $G(j\omega)$ as a complex number and drawing it in the complex plane.

For each frequency $\omega$:
- Compute $G(j\omega)$ — a complex number.
- Plot it: real part on the x-axis, imaginary part on the y-axis.
- Sweep $\omega$ from $-\infty$ to $+\infty$ and connect the dots.

Because $G(-j\omega)$ is the complex conjugate of $G(j\omega)$, the bottom half of the plot is always a mirror image of the top half. Most plots in the slides show both halves — it looks like a closed curve.

> [!tip] Bode vs. Nyquist — Same Information, Different View
> Distance from the origin = $|G(j\omega)|$ (the Bode magnitude). Angle from the positive real axis = $\angle G(j\omega)$ (the Bode phase). You're looking at the exact same data as a Bode plot, just projected onto the complex plane.

### 2.2 The Critical Point $(-1, 0)$

The closed-loop poles of $\dfrac{G}{1 + G}$ are the roots of
$$1 + G(s) = 0 \quad \Leftrightarrow \quad G(s) = -1$$

So the **closed loop becomes singular** exactly when the open-loop $G$ hits the value $-1$. On the Nyquist plot, the value $-1$ lives at the point $(-1, 0)$. That's why this one point gets all the attention — it's where stability is decided.

Intuitively: if the Nyquist curve **passes through** $(-1, 0)$ the closed loop has a pole on the imaginary axis (marginally stable oscillation). If the curve comes close, you have poor stability margins. If the curve stays far away from $(-1, 0)$, you have comfortable margins.

### 2.3 Encirclements: the Bookkeeping Rule

The real power of Nyquist comes from counting how many times the curve **loops around** $(-1, 0)$. Each such loop is called an **encirclement**.

- **Clockwise (CW)** encirclement = $+1$ (contributes $+1$ to $N$)
- **Counter-clockwise (CCW)** encirclement = $-1$

If the curve doesn't close around $(-1, 0)$ at all, $N = 0$.

The Nyquist stability criterion then says:
$$Z = N + P$$
- $Z$ = closed-loop RHP poles (we want this to be **zero**)
- $N$ = net CW encirclements of $(-1, 0)$
- $P$ = open-loop RHP poles (read off $G(s)$)

> [!important] Why This Rule Exists
> Nyquist's theorem is a consequence of the *argument principle* from complex analysis: the number of times a closed curve wraps around a point equals the difference between the zeros and poles of a related function enclosed by the original $s$-plane contour. You don't need the proof — just remember the bookkeeping: each RHP pole of the open loop must be "paid back" by one CCW encirclement.

### 2.4 Reading a Nyquist Plot in Practice

A typical workflow when you look at a Nyquist plot:

1. **Find the arrow.** The curve has a direction — usually $\omega$ increases from $0$ to $\infty$ along the top half, then from $-\infty$ back to $0$ along the bottom half. The slides mark this with small arrows.
2. **Count RHP poles of the open loop, $P$.** Just read them off the transfer function denominator.
3. **Count encirclements of $(-1, 0)$, $N$.** Stand at $(-1, 0)$ and watch the curve spin around you. CW counts positive, CCW counts negative.
4. **Compute $Z = N + P$.** If $Z = 0$ the closed loop is stable. If not, the closed loop has $Z$ RHP poles.

### 2.5 What Changes When You Scale the Gain?

Multiplying by a gain $K$ just **scales the whole plot** radially around the origin — points far from $0$ get proportionally farther, points near $0$ stay near $0$. Multiplying by a **negative** gain additionally **reflects the plot through the origin** (rotates by $180°$).

This is why a P-controller sweep is a geometric search: you're asking "what scaling of this fixed curve puts the right number of encirclements around $(-1, 0)$?"

> [!tip] Quick Mental Model
> - Nyquist plot in the RHP of the complex plane, crossing the real axis at $+0.5$? A positive gain can never move it left past $-1$ — you need a negative $K_{PS}$.
> - Nyquist crosses the real axis at $-0.741$ going CCW, and you need one CCW encirclement? Scale by more than $1/0.741 \approx 1.35$.

### 2.6 Gain and Phase Margin on a Nyquist Plot

For stable plants, the **gain margin** is $1/|x|$ where $x$ is the real-axis crossing nearest to $(-1, 0)$, and the **phase margin** is the angle from the negative real axis to the point where the curve crosses the unit circle. Both are visible in one picture — Bode plots need you to flip between two graphs.

---

## 3. Stabilisation by a Proportional Controller

Before reaching for anything fancy, ask: can a single gain $K_{PS}$ do the job? Often yes. Consider:
$$G(s) = \frac{0.5(s+1)}{(0.1s+1)(-0.01s+1)} = -\frac{0.5s + 0.5}{0.001 s^2 - 0.09 s - 1}$$

The factor $(-0.01s + 1)$ has its root at $s = +100$ — a clear RHP pole. We want to find the $K_{PS}$ that makes the closed loop stable.

### 3.1 Algebraic Approach

Close the loop with unit feedback:
$$G_{cl}(s) = \frac{K_{PS} G(s)}{1 + K_{PS} G(s)} = \frac{0.5 K_{PS}(s+1)}{-0.001 s^2 + (0.5 K_{PS} + 0.09) s + 0.5 K_{PS} + 1}$$

The denominator is quadratic, so stability is easy to check: all real roots must be negative. The discriminant
$$\Delta = (0.5 K_{PS} + 0.09)^2 + 4 \cdot 0.001 (0.5 K_{PS} + 1)$$
is always positive $\to$ the closed-loop poles are real. Dividing through by the leading $-0.001$ flips the signs, so we need both middle and constant coefficients to be **negative**:

$$0.5 K_{PS} + 0.09 < 0 \quad \text{and} \quad 0.5 K_{PS} + 1 < 0 \quad \Rightarrow \quad \boxed{K_{PS} < -2}$$

Notice two things that are different from the stable-plant case:
1. We needed a **negative** gain. That isn't cheating — it just means the controller inverts the sign before sending the signal to the plant.
2. The gain has to be **large enough in magnitude**. A small gain leaves an RHP pole untouched; the gain must be strong enough to drag it across the imaginary axis.

### 3.2 Nyquist Approach

The algebra only worked because the example is quadratic. For higher-order plants we use Nyquist:
$$Z = N + P$$
where $Z$ = closed-loop RHP poles (want $0$), $N$ = **clockwise (CW)** encirclements of $(-1, 0)$ by the open-loop Nyquist plot, and $P$ = open-loop RHP poles.

> [!important] The Rule in One Sentence
> For each RHP pole of $G(s)$, the open-loop Nyquist plot must make **one counter-clockwise (CCW) encirclement** of $(-1, 0)$ so that $N = -P$ and $Z = 0$.

Sweeping $K_{PS}$ just scales the Nyquist plot about the origin. So stabilising becomes a geometry problem: *can I find a scaling (possibly with sign-flip) that makes the plot encircle $(-1,0)$ CCW the right number of times?*

For our example:

| Case | What the Nyquist plot looks like | Conclusion |
|---|---|---|
| Positive $K_{PS}$: plot of $G(s)$ | Lives in the RHP of the Nyquist plane | No amount of positive gain moves the curve left past $-1$ — cannot stabilise |
| Negative $K_{PS}$: plot of $-G(s)$ crosses the real axis at $-0.5$ | One CCW loop is nearby but doesn't yet enclose $-1$ | Need $\|K_{PS}\| \geq 2$ to push the crossing past $(-1,0)$ |
| $K_{PS} = -3$: plot of $-3G(s)$ | One clean CCW encirclement of $(-1,0)$ | Stable |

Both the algebra and Nyquist agree: $K_{PS} < -2$.

> [!tip] Why Nyquist is the Right Tool Here
> Root locus and algebra give the exact boundary, but Nyquist **shows you the sign** at a glance: if the plot is entirely in the RHP of the Nyquist plane you already know you need a negative gain before you do any calculation.

![[Lecture_10_Unstable_systems.pdf#page=5]]

---

## 4. Exam 2023 Example

Given: poles at $\{0.045,\ -2,\ -3\}$ (so $P = 1$), Nyquist plot intersects the real axis at $(-0.741, 0)$ in the **CCW** direction. A P-controller $K_{PS}$ with unit feedback is applied. Which gain stabilises?

**Reasoning:**
- $P = 1$, so we need $N = -1$: one CCW encirclement of $(-1, 0)$.
- The plot already rotates CCW, so no sign-flip is needed — we just need the crossing to move past $-1$.
- Currently the crossing sits at $-0.741$. Scaling by $K_{PS}$ stretches this to $-0.741 \, K_{PS}$. For this to be left of $-1$:
$$K_{PS} > \frac{1}{0.741} \approx 1.35$$

| Option | Value | Verdict |
|---|---|---|
| a | $K_{PS} = 1.089$ | Too small |
| b | $K_{PS} = -0.741$ | Negative — misreads the crossing as a gain |
| c | $K_{PS} = 0.741$ | Confuses the crossing distance from the origin with the required gain |
| **d** | **$K_{PS} = 1.5$** | **Correct** ($> 1.35$) |
| e | $K_{PS} = 0.92$ | Too small |

> [!tip] Exam Trick
> When an exam gives an unstable plant with a CCW real-axis crossing at $x$, the stabilising gain condition is $K_{PS} > 1/|x|$ (with the sign that preserves the CCW direction). Always sanity-check the sign against the direction of the Nyquist plot.

---

## 5. Control Design After Stabilisation

Stabilisation just gives us a closed loop that doesn't diverge. We still want **performance** — bandwidth, zero steady-state error, good phase margin. So we build an outer controller on top. There are two procedures in the course, and they only differ in how they treat the inner stabilising loop.

### 5.1 Method 1: Close the Inner Loop First

Take the previous example with $K_{PS} = -3$. The stabilised system is:
$$G_s(s) = \frac{-3 G(s)}{1 - 3 G(s)} = \frac{1500 s + 1500}{s^2 + 1410 s + 500}$$

This is now the *new plant* we design against. Looking at its Bode plot:
- Magnitude has no positive slopes $\to$ well-behaved — no resonance issues.
- Phase margin is already good $\to$ no Lead needed.
- Want zero $e_{ss}$ $\to$ add a **PI**.
- Want high bandwidth $\to$ push $\omega_c$ up.

**Design:** pick $N_i = 3$ and $\omega_c = 2000$ rad/s:
$$\tau_i = \frac{N_i}{\omega_c} = 0.0015, \quad K_P = 1.5475$$

Closed-loop results: rise time $\approx 2.5$ ms, settling time $\approx 5.9$ ms, no overshoot.

The advantage of Method 1 is clarity — you forget about the instability after step 2. The downside is that the stabilised plant's Bode plot hides some of the physics. If the raw plant had a resonance or a peak, Method 2 exposes it better.

### 5.2 Method 2: Absorb Only the Sign

Instead of closing a full loop around $K_{PS}$, we use the Nyquist fact that $K_{PS}$ must be negative and absorb **only the sign** into the plant:
$$G_s(s) \triangleq -G(s) = \frac{0.5 s + 0.5}{0.001 s^2 - 0.09 s - 1}$$

Now the Bode plot of $G_s(s)$ is the plot of the *original* physics, just with a corrected sign. For the REGBOT tilt loop this matters a lot — you can see where the pendulum resonance sits, where the amplitude peaks, where the phase drops.

For this example the magnitude curve of $G_s(s)$ increases monotonically up to $\omega_i = 31.5$ rad/s. A rising magnitude is bad — it means $K_P$ has a stronger effect at higher frequencies than at DC, which harms robustness. The PI zero is placed at the peak to cancel the rise:
$$\tau_i = \frac{1}{\omega_i} = 0.0317 \quad \Rightarrow \quad C_{PI}(s) = \frac{0.0317 s + 1}{0.0317 s}$$

With the same $\omega_c = 2000$ rad/s target and a good phase margin already available ($\sim 86°$), no Lead is needed. The gain follows from the crossover condition:
$$K_P = \left. \frac{1}{|C_{PI}(s)\, G_s(s)|} \right|_{s = j\omega_c} = 4$$

Closed-loop results: rise time $\approx 0.94$ ms, settling time $\approx 66$ ms, overshoot $\approx 6\%$, bandwidth $\approx 2114$ rad/s. Slightly more oscillatory than Method 1, but it's a cleaner path to the numbers when you can read the Bode plot.

![[Lecture_10_Unstable_systems.pdf#page=11]]

### 5.3 The Two Procedures Side by Side

> [!abstract] Method 1 — Close the inner loop, then design
> 1. Find $K_{PS}$ that stabilises $G(s)$ in unit feedback.
> 2. Close the loop; set $G_s = K_{PS}G / (1 + K_{PS}G)$ as the new plant.
> 3. Choose $\omega_c$ (as large as possible).
> 4. Design the outer controller on $G_s$:
>    - Lead if phase margin is poor
>    - PI if $e_{ss}$ must be zero
> 5. Test and iterate.

> [!abstract] Method 2 — Absorb only the sign
> 1. Check via Nyquist whether $K_{PS}$ must be negative (CCW encirclements = RHP poles).
> 2. Set $G_s = \mathrm{sign}(K_{PS}) G$ and draw its Bode plot.
>    - Place $\tau_i$ to make the amplitude monotonically decreasing (I-part at the peak)
>    - Add Lead if phase margin is poor or $\omega_c$ should be pushed higher
>    - Pick $\omega_c$ and solve for $\tau_d$ and $K_P$ (keep the sign)
> 3. Test and iterate.

> [!tip] Which to Use?
> Method 2 is preferred for the REGBOT assignment because the raw tilt-loop Bode plot shows exactly where the pendulum peak lives — information that would be buried by Method 1's inner-loop closure.

---

## 6. Introduction to the Mandatory Assignment — REGBOT Balance

The REGBOT is an **inverted pendulum on a cart**. It has three things we want to control, and each depends on the one below it:

1. **Stay upright** (balance)
2. **Balance *while moving at a given velocity***
3. **Balance *while stopping at a given position***

We'll solve this with a three-level **cascaded** controller — each loop treats the inner closed loop as its new plant.

### 6.1 Balancing: the Physics

![[Lecture_10_Unstable_systems.pdf#page=16]]

Gravity acting at distance $l$ from the wheel axis produces a tilt torque $m g \sin(\theta)\, l$ that tries to make the robot fall. To cancel it we need a **horizontal force** applied at the wheels in the *same direction as the tilt* (tilt forward $\to$ drive forward, tilt backward $\to$ drive backward).

Horizontal force is a horizontal acceleration, and we don't command accelerations directly — we command **wheel velocities**. A *changing* velocity produces an acceleration, so the balance controller output is a velocity demand whose derivative gives the required correcting acceleration.

The linear velocity relation used throughout the assignment:
$$v_\text{REGBOT} = r_\text{wheel} \cdot \omega_\text{wheel} = r_\text{wheel} \cdot n_\text{gear} \cdot \omega_\text{motor}$$

> [!important] Why Constant Velocity Cannot Balance
> A constant velocity gives **zero** acceleration and therefore zero counter-torque — gravity wins. Balance is inherently a dynamic condition: you must keep accelerating in response to tilt.

### 6.2 The Cascaded Control Architecture

![[Lecture_10_Unstable_systems.pdf#page=20]]

The full signal chain from the outermost reference to the motor voltage:

| Loop | Input | Output | Purpose |
|---|---|---|---|
| **Position controller** (outer) | $x_\text{ref}$ | $v_\text{ref}$ | Drive REGBOT to a target position |
| **Velocity controller** (middle) | $v_\text{ref}$ | $\theta_\text{ref}$ | Choose a tilt that will produce the desired speed |
| **Tilt controller** (inner) | $\theta_\text{ref}$ | `vel_ref` | Keep the robot balanced at the demanded tilt |
| **Wheel-velocity controller** (given) | `vel_ref` | motor voltage | Track the inner velocity demand at the motor |

> [!important] Always Design Inner Loops First
> The velocity controller needs the tilt loop to already work, because it assumes "I ask for tilt $\theta_\text{ref}$, I get tilt $\theta$." Likewise the position controller assumes the velocity loop tracks $v_\text{ref}$. Tune inner $\to$ middle $\to$ outer.

### 6.3 Three Linearisations, Three Designs

The assignment asks you to linearise Simulink three times, once per loop:

1. **Tilt loop:** from `vel_ref` to `tilt_angle` $\to$ the transfer function $G_{tv}(s)$. **This one is unstable.**
2. **Velocity loop:** from `θ_ref` to the output of `wheel_vel_filter` (once the tilt loop is closed).
3. **Position loop:** from `v_ref` to `x_position` (once the velocity loop is closed).

---

## 7. Tips for the Tilt (Balance) Controller

### 7.1 The Post-Integrator Trick

> [!warning] $G_{tv}(s)$ is Unstable
> The tilt-loop plant from the first linearisation is open-loop unstable (as expected — it *is* the inverted pendulum). A plain PI-Lead cannot stabilise it, so before you design anything you insert an extra **post-integrator** into the open loop.

Define the new plant to control as:
$$G_{tv,\text{post}}(s) = -C_{PI,\text{post}}(s) \cdot G_{tv}(s), \qquad C_{PI,\text{post}}(s) = \frac{\tau_{i,\text{post}}\, s + 1}{\tau_{i,\text{post}}\, s}$$

The minus sign is the same sign-absorption as Method 2 in §4.2. The PI block is called "post-integrator" because it sits *after* the tilt-controller output. Together they make the Bode magnitude curve well-behaved enough to design a normal PI-Lead on top.

![[Lecture_10_Unstable_systems.pdf#page=22]]

### 7.2 Choosing $\tau_{i,\text{post}}$

Simple recipe:

1. Make a Bode plot of $G_{tv}(s)$.
2. Find the frequency $\omega_{i,\text{post}}$ where the magnitude **peaks**.
3. Select $\tau_{i,\text{post}} = 1/\omega_{i,\text{post}}$.

The idea is exactly the same as in Method 2: put the PI zero at the peak so that the combined $C_{PI,\text{post}}\, G_{tv}$ has a monotonically decreasing magnitude. Comparing the Nyquist plots of $G_{tv}$ (no CCW encirclement $\to$ unstable) and $G_{tv,\text{post}}$ (one CCW encirclement $\to$ stable) confirms that the post-integrator did its job.

![[Lecture_10_Unstable_systems.pdf#page=23]]

### 7.3 Lead from the Gyro — Skipping the Derivative

A standard Lead has the form $(\tau s + 1)/(\alpha \tau s + 1)$. The numerator is a derivative, the denominator is a low-pass filter to keep noise from blowing up.

For REGBOT the gyro **already measures** $\dot\theta$, so no numerical differentiation is needed. Then:

$$\frac{\tau s + 1}{\alpha \tau s + 1}\, \theta = \underbrace{\frac{1}{\alpha \tau s + 1}}_{\text{low-pass, not needed}} \left( \tau \underbrace{s\theta}_{\dot\theta = \text{gyro}} + \theta \right) \;\to\; \tau \cdot \text{gyro} + \theta$$

> [!tip] Practical Lead
> Because the gyro gives a clean derivative, we drop the low-pass $1/(\alpha\tau s + 1)$ and implement the Lead as a simple weighted sum $\tau \cdot \text{gyro} + \theta$. Cleaner, less phase lag, fewer tuning parameters.

---

## 8. Key Takeaways

1. An LTI system is **unstable** exactly when any pole lies in the RHP, or repeated poles sit on the imaginary axis. Physically: bounded input $\to$ unbounded output.

2. **Nyquist**: $Z = N + P$. For each RHP pole of the plant we need **one CCW encirclement** of $(-1,0)$ to stabilise the closed loop.

3. A proportional gain $K_{PS}$ often stabilises an unstable plant. The Nyquist plot tells you the required **sign** (by whether the curve is already to the left of the origin) and the required **magnitude** (by the real-axis crossing distance).

4. After stabilisation, design the outer controller on the stabilised system. Two procedures:
   - **Method 1:** substitute $G_s = K_{PS} G / (1 + K_{PS} G)$ and design normally.
   - **Method 2:** absorb only the sign, $G_s = \mathrm{sign}(K_{PS}) G$, and place the PI zero at the Bode magnitude peak.

5. **REGBOT balance** uses a three-level cascade: position $\to$ velocity $\to$ tilt $\to$ wheel-velocity. Design inner loops first.

6. The tilt loop is unstable — stabilise it with a **post-integrator** $C_{PI,\text{post}}$ and a minus sign, with $\tau_{i,\text{post}} = 1/\omega_{i,\text{post}}$ (frequency of the magnitude peak).

7. Use the **gyro** directly for the Lead part — the gyro is $\dot\theta$, so the Lead collapses to $\tau \cdot \text{gyro} + \theta$ without any low-pass filter.

---

## 9. Quick Reference

> [!abstract] Stabilisation Workflow
> | Step | Method 1 | Method 2 |
> |---|---|---|
> | 1 | Find stabilising $K_{PS}$ via Nyquist/algebra | Determine sign of $K_{PS}$ from Nyquist |
> | 2 | Close loop: $G_s = K_{PS} G / (1 + K_{PS} G)$ | Set $G_s = \mathrm{sign}(K_{PS}) G$ |
> | 3 | Design PI(-Lead) on $G_s$ | Place $\tau_i = 1/\omega_\text{peak}$, add Lead if needed |
> | 4 | Pick $\omega_c$, compute $K_P$ | Pick $\omega_c$, compute $\tau_d$ and $K_P$ (with sign) |
> | 5 | Simulate, iterate | Simulate, iterate |

> [!abstract] REGBOT Balance Cascade
> $$x_\text{ref} \xrightarrow{\text{pos.~ctrl}} v_\text{ref} \xrightarrow{\text{vel.~ctrl}} \theta_\text{ref} \xrightarrow{\text{tilt~ctrl} + C_{PI,\text{post}}} \text{vel\_ref} \xrightarrow{\text{WV~ctrl}} u_\text{motor}$$

---

## 10. Next Lecture

**Lesson 11 — Limited systems:**
- Explain rate and amplitude limitations in systems
- Use mitigation strategies in the control design of limited systems

## 11. Today's Exercises

- **Group work:** REGBOT balance (mandatory assignment) — see [[REGBOT Balance Assignment]]
- **Home alone:** Theoretical exercises LCD1 — problems 1, 2, 4, 5, 6, 7, 9

---

> [!nav]
> [[Lesson 9 - PI-Lead Design with Specifications|← Lesson 9]]
>
> [[34722 Linear Control Design 1|34722 Home]]
>
> &nbsp;
