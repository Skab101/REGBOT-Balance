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
- [ ] **Phase 7** — Documentation sweep (`docs/*.md` + Report LaTeX)
- [ ] **Phase 8** — Merge `day5-redesign` → `main` on both submodules (`--no-ff`), then bump DTU main pointers

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
