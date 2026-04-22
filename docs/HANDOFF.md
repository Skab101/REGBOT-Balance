---
course: "34722"
course-name: "Linear Control Design 1"
type: handoff
tags: [LCD, regbot, handoff, pedagogical-rewrite]
date: 2026-04-23
---
# REGBOT Balance Assignment — Handoff to the next Claude session

> [!abstract] Where we are
> The four-loop cascade is **designed, built, and validated**. All numbers in the repo are correct. The LaTeX report compiles cleanly (26 pages) and all hardware tests passed with the v3 Day 5 on-floor gains. What's broken is *pedagogy*: `docs/REGBOT Balance Assignment.md` explains *what* we did but not *why*, and the user (who didn't author the MATLAB scripts and hasn't been following class closely) can't learn the control theory from the current doc. The rewrite goal is to turn the doc into a teaching walkthrough grounded in the 34722 Linear Control Design 1 course material.

> [!warning] Hard rules
> 1. **Do not touch the MATLAB scripts** (`simulink/design_task*.m`, `simulink/regbot_mg.m`, `simulink/lib/pick_image_dir.m`, the `.slx` files). The user did not author them and wants them unchanged unless they explicitly ask.
> 2. **Do not regenerate or touch the design-time plots** in `docs/images/` (all `regbot_*.png`). They are from the 2026-04-22 MATLAB runs and match the numbers the rewritten doc will use.
> 3. **Do not alter the hardware test results** in `docs/Test Plan.md` (experimental facts) or the log files in `logs/`.
> 4. **Do not change the numerical content** of the four task sections — `K_p = 13.2037`, `\omega_c = 30`, the phase-balance arithmetic, `\gamma_M = 60°` achievement, etc. Those are correct. The rewrite adds pedagogy *around* them.
> 5. **Commit messages must not mention Claude or AI.** Project-wide rule.
> 6. **One task at a time.** Rewrite Task 1, commit, wait for user to say "ok keep going", then Task 2/3/4. Do not batch.
> 7. **Keep the collapsible `[!tldr]+` callouts already present at the top of each task.** Extend the detail below them; do not remove or duplicate the summary.
> 8. **The DTU vault no longer mirrors regbot content.** `docs/` is the single source of truth. Cross-vault links to DTU-vault files use `obsidian://open?vault=Obsidian&file=...`.

---

## 1. Current repo state (as of 2026-04-23)

All three repos are clean and fully pushed.

| Repo | Branch | Tip | Origin |
|---|---|---|---|
| `REGBOT-Balance-Assignment` (`Skab101/REGBOT-Balance`) | `main` | `74eb90a` — Test Plan TL;DR callouts | up to date |
| `Report` submodule (`MadsRudolph/REGBOT-Balance-assignment`) | `main` | `17c2438` — Simulink figures | up to date |
| `DTU` main (`MadsRudolph/DTU`) | `main` | `d615632` — regbot submodule bump | up to date |

Backup branch `v1-day4-wheels-up` exists on both regbot and Report origins (the pre-redesign state, in case it's ever needed).

Report is now a **real submodule of the regbot repo** at `Report/` (previously a symlink pointing into the DTU vault). DTU main no longer registers Report as a submodule; the regbot repo owns it.

`docs/_DTU-vault` is **not** expected to exist — earlier in this session's work a directory-junction approach was tried and rejected. The current approach is `obsidian://` URI links inside the markdown, pointing at `vault=Obsidian&file=<path>`.

---

## 2. The rewrite task

### What's wrong with the current doc

`docs/REGBOT Balance Assignment.md` is currently 380 lines, terse, accurate, and reference-style. Per-task sections jump straight into phase-balance arithmetic with lines like:

> Place the PI zero at $\omega_c/N_i$ with $N_i = 3$. $\tau_i = 3/15 = 0.200$ s.

That states the manipulation but doesn't explain what $N_i$ *means*, why 3 is the standard choice, what "placing a zero" does to the Bode plot, or how the student would arrive at this step on their own. The user needs a *teaching walkthrough*, not a reference card.

### What to rewrite

**PRIMARY:** `docs/REGBOT Balance Assignment.md` — per-task sections (Task 1, Task 2, Task 3, Task 4) and the cascade-architecture intro.

**SECONDARY (only after user confirms main doc):** possibly `docs/Test Plan.md` — less pedagogy needed (it's procedural hardware testing), but per-test callouts could get a similar expansion. Ask the user before touching it.

### Step 1: read the course material first (before writing anything)

The user explicitly asked for the rewrite to be grounded in the 34722 course's language and framing, not generic textbook phrasing. Open these four DTU-vault lecture notes and read them carefully:

- `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Fundamentals - Intuitive Control Theory.md`
- `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Lesson 9 - PI-Lead Design with Specifications.md`
- `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Lesson 10 - Unstable Systems and REGBOT Balance.md`
- `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Worked Example - REGBOT Position Controller.md`

Relevant PDFs in the same course folder:
- `Slides\Lecture_10_Unstable_systems.pdf`

Match the course's notation (e.g.\ if the course uses $\phi_G(j\omega_c)$ instead of $\angle G(j\omega_c)$, use that), match the way the course talks about Bode plots, phase balance, N_i, the post-integrator, gyro Lead, etc. The goal is the user recognises the language when they read the rewrite.

### Step 2: add a short "How to read Bode plots in this doc" intro section

After the cascade architecture diagram, insert a ~60–80 line section titled something like "Control-design basics used in this doc". Cover (tightly, with inline links to the lecture notes above):

- **Bode plot** — magnitude (dB) and phase (deg) vs. $\omega$; why it's the right tool here (frequency-domain phase-balance method).
- **Crossover frequency $\omega_c$** — where $|L(j\omega)|$ crosses $1$ (0 dB); approximately equals the closed-loop bandwidth. Picking $\omega_c$ is picking how fast the loop should react.
- **Phase margin $\gamma_M$** — $180° + \angle L(j\omega_c)$; what $60°$ means physically, what happens at $30°$ or $0°$.
- **Why $N_i = 3$** — PI zero at $\omega_c/N_i$; the trade-off between integral action at $\omega_c$ vs. phase loss at $\omega_c$. $N_i = 3$ is the course's default.
- **The phase-balance recipe** — pick $\omega_c$, pick $N_i$, compute $\phi_G(j\omega_c)$, compute $\phi_{PI}$, solve for $\phi_\text{Lead}$, then $\tau_d$ and $K_p$. Shown step by step in each Task section below.
- **Cascaded-loop separation** — why each outer loop must be $\geq 5\times$ slower than the one inside it (otherwise the two loops fight each other and neither hits its designed bandwidth).
- **Negative gain margin on an unstable plant** — why it's *expected* for Task 2 (Lecture 10 slides 5–7 / Fundamentals §9).

Not a course in a section — just enough that the derivations below don't require the reader to context-switch to the DTU vault every sentence.

### Step 3: rewrite each Task section as a pedagogical walkthrough

Structure below the existing `[!tldr]+` callout (which stays as-is):

1. **"What this loop does"** — 2–3 sentences on the loop's job in the cascade and why this specific control structure (PI, PILead, P, etc.) is the right choice for this plant.
2. **Numbered step-by-step derivation.** Each step should have roughly these four parts:
   - **What we do.** The manipulation. ("Place the PI zero at $\omega_c/N_i = 10$ rad/s.")
   - **What that does in Bode/Nyquist terms.** The geometric meaning. ("This adds a slope change at 10 rad/s: below 10 the controller is mostly integrator (-20 dB/dec, phase near -90°), above 10 it's mostly proportional (0 dB/dec, phase near 0°).")
   - **Why we do it.** The rule from the lecture, with an inline `obsidian://` URI link to the specific lecture or worked example when possible. ("We place the zero $N_i = 3$ times below $\omega_c$ so the PI integrator has done most of its phase drop by the time we cross over — see [Lesson 9 …](obsidian://…).")
   - **What the MATLAB script does here.** The line or block from `design_task*.m` that implements this step. ("`tau_i = Ni / wc` in `design_task1_wheel.m` — this is just the formula, nothing mysterious.")
   - **What the output number means.** How to interpret it. ("`Kp = 13.2037` — the controller multiplies the error by this much. To see why, check the Bode plot below: without it the magnitude at $\omega_c$ is $0.076$, so $K_p = 1/0.076 \approx 13.2$ lifts it to exactly $1$.")
3. **Verification.** Keep the existing `margin(L)` table and plots; add a paragraph saying how to *read* each plot — what the curves show, where the crossover is, what the GM/PM markers mean, how to spot trouble.
4. **Copy-paste block for `regbot_mg.m`** stays as-is.
5. **Simulink sanity sim caption** stays as-is (or tightens). The image embed stays.

Expected per-task length: ~150–250 lines (currently ~40–80). Total doc target: **700–900 lines** (up from 380). That's the cost of being pedagogical, but with the collapsible callouts + tight paragraphs it stays navigable.

### Step 4: workflow — one task at a time

1. Propose Task 1 as a specimen. Commit it.
2. Ask the user: *"Is this the right depth, tone, and level of hand-holding? Anything to adjust before I apply the same treatment to Tasks 2/3/4?"*
3. On green-light, do Tasks 2, 3, 4 in the same style.
4. Then ask whether to extend the same treatment to the "How to read Bode plots" intro section (if not already included in the Task 1 pass) and to `docs/Test Plan.md`.

### Step 5: keep everything else working

- All 15+ `![[regbot_*.png]]` image embeds must still resolve — don't rename or move images.
- All `obsidian://open?vault=Obsidian&file=...` links must stay valid (don't accidentally URL-encode them twice or drop the `vault=` param).
- All local `[[Lesson 10 ...]]`, `[[HANDOFF]]`, `[[REDESIGN_ROADMAP]]` wikilinks must still resolve.
- Don't break existing `\boxed{}` controller-expression math or the table structure for final gains / hardware validation.
- The LaTeX report (`Report/sections/*.tex`) is **separately authored** and out of scope for this rewrite. Only touch Report if the user asks.

---

## 3. Commit workflow for each Task

1. Edit `docs/REGBOT Balance Assignment.md` for one Task section.
2. `git add docs/REGBOT Balance Assignment.md` on the regbot repo.
3. `git commit` on regbot main with a specific message (e.g.\ "Task 1 section: pedagogical rewrite with course-material framing"). **No Claude/AI mentions**.
4. `git push origin main` on regbot.
5. `cd ..` to DTU main, `git add "4. Semester/Linear Control Design/REGBOT-Balance-Assignment"`, commit bumping the submodule, `git push`.
6. Present the new Task section to the user and wait for feedback.

---

## 4. Files + paths reference

- Target file: `C:\Users\Mads2\DTU\4. Semester\Linear Control Design\REGBOT-Balance-Assignment\docs\REGBOT Balance Assignment.md`
- Lecture-material inputs (DTU vault):
  - `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Fundamentals - Intuitive Control Theory.md`
  - `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Lesson 9 - PI-Lead Design with Specifications.md`
  - `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Lesson 10 - Unstable Systems and REGBOT Balance.md`
  - `C:\Users\Mads2\DTU\Obsidian\Courses\34722 Linear Control Design 1\Lecture Notes\Worked Example - REGBOT Position Controller.md`
- Related (keep untouched unless asked):
  - `docs/Test Plan.md`
  - `docs/REDESIGN_ROADMAP.md`
  - `docs/Lesson 10 - Unstable Systems and REGBOT Balance.md` (local copy of the DTU-vault lecture note)
  - `docs/PLAN.md`
  - `docs/images/` (all plots)
  - `simulink/*` (all MATLAB)
  - `Report/` (LaTeX submodule)

---

## 5. Summary of what's been done before this handoff

Long session's worth of work already landed — just context for understanding why the repo looks how it does:

- Day 5 on-floor redesign: $K_{pwv}$ went 3.31 → 13.2037 after discovering the first campaign used wheels-up identification; all four loops retuned; hardware re-validated 2026-04-22. See `docs/REDESIGN_ROADMAP.md` for the phase tracker (all phases ✓).
- `day5-redesign` branch merged into `main` on both regbot and Report submodules; backup branch `v1-day4-wheels-up` preserved.
- DTU vault cleaned of all regbot mirrors (previously held redundant copies of HANDOFF/Test Plan/REGBOT Balance Assignment/Images).
- Regbot repo structure simplified: dropped empty `missions/`, `src/`, `scripts/`, and redundant `figures/` (every PNG was already in `docs/images/`).
- `simulink/lib/pick_image_dir.m` now always writes to `docs/images/` (previously mirrored into the DTU vault).
- Report moved from being a DTU-side submodule + symlink to being a real submodule of the regbot repo.
- Cross-vault links rewritten from `[[wikilinks]]` to `obsidian://` URI markdown links.
- Main doc rewritten from 1262 lines (sprawling) to 380 lines (lean reference). Collapsible `[!tldr]+` callouts added to the top of each Task section.
- Test Plan got per-test collapsible narrative callouts.
- Simulink top-level model + Tilt_Controller subsystem screenshots added (`docs/images/regbot_simulink_model.png`, `regbot_simulink_tilt_controller.png`) — embedded in the main doc and in the LaTeX report.

Now: the user wants the *lean* main doc turned into a *pedagogical* one, grounded in the course's own explanations.

---

*Handoff written: 2026-04-23. Pick up from Step 1: open the four lecture notes and read them before touching the doc.*
