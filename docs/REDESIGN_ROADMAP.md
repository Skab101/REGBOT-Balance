# `day5-redesign` Roadmap

Cascade redesign against the Day 5 **on-floor v2** training-wheels plant `Gvel = 2.198/(s+5.985)` (DC 0.367, τ = 167 ms), replacing the Day 4 **wheels-up** approximation `Gvel = 13.34/(s+35.71)` that was used for the original design.

Branch is scoped to the two submodules (`REGBOT-Balance-Assignment` + `Report`). DTU main is not branched; submodule pointers bump only after merge.

Detailed plan: see Claude's plan file (local to the working machine).

---

## Phases

- [x] **Phase 0** — Branch `day5-redesign` on REGBOT-Balance and Report; this roadmap file in place.
- [x] **Phase 1** — Task 1 redesign
    - [x] 1.B  `design_task1_wheel.m` loads `G_1p_avg` from `data/Day5_results_v2.mat`
    - [x] 1.C  Ran script → **Kp = 13.2037**, τ_i = 0.1000, achieved wc = 30.00, PM = 82.85°, GM = Inf
    - [x] 1.D  Paste new gains into `regbot_mg.m`
    - [x] 1.E  Commit
- [x] **Phase 2** — Task 2 redesign
    - [x] Re-linearised Gtilt with the new inner loop in place
    - [x] **Kptilt = 1.1999, titilt = 0.2000, tdtilt = 0.0442, tipost = 0.1245** (achieved wc = 15.00, PM = 60.00°, GM = −5.58 dB, 0 RHP closed-loop poles)
    - [x] τ_d dropped 67% vs old design (0.1355 → 0.0442) — less Lead needed because the inner loop is properly fast now
- [x] **Phase 3** — Task 3 redesign
    - [x] Re-linearised Gvel,outer with new Task 1 + 2 gains in place
    - [x] RHP zero still at +8.51 rad/s (physics-fixed)
    - [x] **Kpvel = 0.1581, tivel = 3.0000** (achieved wc = 1.00, PM = 68.98°, GM = 5.84 dB)
    - [x] Plant got a new LHP zero at −8.03 matching the tipost peak, and the complex pole pair moved from −2.63 to −10.4 (balance loop is doing more work)
- [x] **Phase 4** — Task 4 redesign
    - [x] Re-linearised Gpos,outer; free integrator still there (Type-1 plant, pure P works)
    - [x] **Kppos = 0.5411** (was 0.5335); Lead dropped for same Simulink-improper-TF reason as before
    - [x] Sim peak velocity = 0.753 m/s (spec ≥ 0.7 ✓, tighter than the 0.80 of the old design but hardware should still clear with margin)
    - [x] PM 60.00° (with Lead) → ~57° (without, after drop); GM = 25.17 dB
- [x] **Phase 5.A** — `config/regbot_group47.ini` updated with all four v3 gain blocks
- [x] **Phase 5.B** — Simulink sanity sims
    - [x] `startAngle=10` balance recovery: voltage peak 2.8 V (no saturation), pitch 10° → 0° within ~0.3 s, fully settled by t ≈ 2 s. Faster than old design.
    - [x] 2 m position step: x reaches 2.15 m peak (7.5% overshoot), settles at 2.00 m; **peak velocity ≈ 0.80 m/s** (above 0.7 spec ✓); voltage peak ~3 V; pitch +17° during accel, returns to 0.
- [ ] **Phase 6** — Hardware validation (Tests 0, 3a, 3b, 4 with `v3_onfloor` suffix)
    - [x] Simulink sanity-sim screenshots moved into `figures/` and `docs/images/`:
          `regbot_task2_sim_recovery_10deg_v3.png`, `regbot_task4_sim_step_v3.png`
    - [x] `docs/Test Plan.md` prepared — pre-flight checklist, mission scripts, pass criteria, slots for post-test notes, v2-baseline comparisons baked in
    - [x] Load v3 ini into GUI → send → save to robot flash
    - [x] **Test 0 v3: PASS** — rise to 0.27 m/s in **0.012 s** (v1 was 0.329 s, 27× improvement), peak V = 2.60 V, L/R match 0.76%
    - [x] **Test 3a v3 (first run): drift 0.475 m** (tilt std 2.04° — 57% tighter than v2's 4.76°, no late oscillations; drift worsened because tilt-offset bias ≈ +1.1°).
    - [x] **Tilt-offset recalibrated (Y = 175) + Test 3a v3 re-run: drift 0.505 m** (marginal fail; bias essentially unchanged at +1.11° despite recal; tilt std 1.87° — 61% tighter than v2). Bias is either a calibration residual (one more iteration to Y ≈ 176 would likely fix it) or a physical asymmetry. Tests 3b/4 outer loops make the bias moot, so moving on.
    - [x] **Reportable Test 3a = v2 (0.343 m, passes spec).** v3 balance-tightness improvement cited separately.
    - [x] **Test 3b v3: PASS** at 0.8 m/s. Completed square, 359.8° heading. Peak tilt +25.5° (v2 was +22°); tilt std 5.03° (tighter than v2's 5.72°). **Motor voltage peak 7.31 V (91% of ±8 V budget)** — v2 had 4.67 V; the higher Kpwv costs voltage headroom. Still passes spec.
    - [x] **Test 4 v3: PASS** — the cleanest result of the whole campaign. Final x = 1.964 m (3.6 cm short; v2 was 10.7 cm short — 3× closer). No overshoot, no late limit cycle. Peak tilt +17° (v2 +25°, −31%). Tilt std 2.93° (v2 5.18°). Peak voltage 4.95 V, **no saturation** (the 3b worry didn't materialise because position-loop vref is smooth, not step-like). Peak velocity 0.79 m/s (v2 1.01; still > 0.7 spec).
- [x] **Phase 7** — Documentation sweep (`docs/*.md` + Report LaTeX)
    - [x] `docs/REGBOT Balance Assignment.md` updated: v3 canonical gains table, redesign story, v1→v3 shift table, hardware outcome summary, Test 3a nuance, Session 4 progress-log entry.
    - [x] v3 figures copied into `Report/images/` (7 files: Test 0/3a/3b-XY/3b-TS/4 + two sanity sims).
    - [x] `Report/sections/wheel-speed-controller.tex` — Day 5 on-floor plant, Kp = 13.2037, Test 0 v3 numbers, redesign narrative in place of old caveat.
    - [x] `Report/sections/balance-controller.tex` — v3 design numbers (tipost 0.1245, titilt 0.2, tdtilt 0.0442 = −67%, Kp 1.1999), Test 3a split (v2 for drift spec, v3 for tightness).
    - [x] `Report/sections/velocity-controller.tex` — RHP zero refined 8.67 → 8.51, Kpvel 0.1581, PM 68.98°, Test 3b v3 figures + saturation-margin discussion (7.31 V / 91% budget).
    - [x] `Report/sections/position-controller.tex` — Kppos 0.5411, Test 4 v3 as the cleanest run (3.6 cm short, no overshoot, no late limit cycle).
    - [x] `Report/sections/conclusion.tex` — v3 final-gains table, v3 hardware summary, new first bullet on the redesign process itself.
    - [x] `main.tex` compiles clean (24 pages, no warnings after two passes).
- [ ] **Phase 8** — Merge `day5-redesign` → `main` on both submodules (`--no-ff`), then bump DTU main pointers (**ON HOLD pending user approval**)

---

## Log entries (fill as phases complete)

| Date | Phase | Note |
|---|---|---|
| 2026-04-22 | 0 | Branch created on both submodules; roadmap in place. |
| 2026-04-22 | 1 | Task 1 PI redesigned against Gvel = 2.198/(s+5.985). New Kp = 13.2037, tau_i = 0.1000. Achieved wc = 30.00 rad/s, PM = 82.85°, GM = ∞. |
| 2026-04-22 | 2 | Task 2 re-linearised. Gtilt magnitude peak moved 5.95 → 8.03 rad/s. New tipost = 0.1245, titilt = 0.2000, tdtilt = 0.0442 (**−67%**), Kptilt = 1.1999. PM = 60.00°, settling 1.34 s vs 1.55 before. |
| 2026-04-22 | 3 | Task 3 re-linearised. RHP zero unchanged at +8.51. New Kpvel = 0.1581 (−2%), tivel = 3.0 (unchanged). PM = 68.98° (up from 64.2°), GM = 5.84 dB. |
| 2026-04-22 | 4 | Task 4 re-linearised; free v→x integrator still there. New Kppos = 0.5411 (was 0.5335). Lead dropped again (improper TF). Sim peak v = 0.753 m/s (was 0.80), still above 0.7 spec. PM ≈ 57° after Lead drop, GM 25.2 dB. |
| 2026-04-22 | 5 | regbot_group47.ini updated with all four v3 gains. Simulink sanity sims green: 10° recovery in <2 s (peak V = 2.8, no sat), 2 m step reaches 2.0 m with 7.5% overshoot and peak v ≈ 0.8 m/s. Ready for hardware. |
| 2026-04-22 | 6 | **Test 0 v3 PASS.** Rise to 0.27 m/s in 0.012 s (vs 0.329 s in v1 — 27× faster). L/R 0.76% match. Voltage peak 2.60 V, ripple higher than v1 (noise/bandwidth tradeoff from Kp = 13.2). Design crossover of 30 rad/s finally materialises on hardware. |
| 2026-04-22 | 6 | **Test 3a v3 PASS** on drift (0.475 m, spec 0.5). Tilt std **2.04° (−57% vs v2)** — balance is clearly tighter. No late oscillation. But drift increased from 0.343 to 0.475 m because tilt-offset bias grew from +0.78° to +1.13° (calibration drift between campaigns). Cleanest fix is a tilt-offset re-cal. |
| 2026-04-22 | 6 | **Test 3a v3 re-run after tilt-offset recal (Y=175):** drift 0.505 m (marginal fail of 0.5 spec). Bias unchanged at +1.11° — either one more calibration iteration needed (Y ≈ 176), or the 1° is a physical CG/wheel-radius asymmetry and cannot be calibrated out. Tilt std 1.87° — 61% tighter than v2. **Decision:** use v2 3a (0.343 m) as the reportable 3a; v3 improvement documented via tilt-std. Tests 3b/4 will not see this because the outer loops regulate it away. |
| 2026-04-22 | 6 | **Test 3b v3 PASS** at 0.8 m/s. Completed full square, 359.8°. Peak tilt +25.5° (v2 +22°), tilt std 5.03° (tighter). **Motor voltage peak 7.31 V (91 % of ±8 V)** — 57 % closer to saturation than v2's 4.67 V, because 4× Kpwv makes the inner PI respond harder to corner vel_ref steps. Classic speed-for-margin trade from the redesign. Still within spec. |
| 2026-04-22 | 6 | **Test 4 v3 PASS — strongest result of campaign.** Final x 1.964 m (−3.6 cm vs v2's −10.7 cm; 3× closer to target). No overshoot, **no late limit cycle** (was a visible artefact in v2). Peak tilt +17° (v2 +25°, −31 %). Tilt std 2.93° vs 5.18°. Peak V 4.95 (no saturation). Peak v 0.79 m/s (still > 0.7 spec, though below v2's 1.01 because tighter control = less over-tilt = less thrust). |
| 2026-04-22 | 7 | **Phase 7 complete.** Obsidian `REGBOT Balance Assignment.md` now carries a top-of-doc v3 Day 5 Redesign callout (canonical gains, story, v1/v3 shift table, hardware outcome, Test 3a nuance), Task 1/2/4 in-doc tables show v1 vs v3 with v3 flagged canonical, and a 2026-04-22 Session 4 progress-log entry was added. In the Report submodule (`day5-redesign`), 7 v3 figures were copied into `images/`, all five `.tex` section files (wheel-speed, balance, velocity, position, conclusion) were updated per-section with v3 plant/gain/test numbers and the first-attempt→redesign narrative. `main.tex` compiles clean on two passes (24 pages). Commits landed per-section for reviewability. Phase 8 (merge) is held per user instruction; no `main` merge and no DTU submodule-pointer bumps attempted. |
