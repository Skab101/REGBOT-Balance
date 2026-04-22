# `day5-redesign` Roadmap

Cascade redesign against the Day 5 **on-floor v2** training-wheels plant `Gvel = 2.198/(s+5.985)` (DC 0.367, τ = 167 ms), replacing the Day 4 **wheels-up** approximation `Gvel = 13.34/(s+35.71)` that was used for the original design.

Branch is scoped to the two submodules (`REGBOT-Balance-Assignment` + `Report`). DTU main is not branched; submodule pointers bump only after merge.

Detailed plan: see Claude's plan file (local to the working machine).

---

## Phases

- [x] **Phase 0** — Branch `day5-redesign` on REGBOT-Balance and Report; this roadmap file in place.
- [ ] **Phase 1** — Task 1 redesign
    - [ ] 1.B  `design_task1_wheel.m` loads `G_1p_avg` from `data/Day5_results_v2.mat`
    - [ ] 1.C  Run script → expect Kp ≈ 13.2, τ_i = 0.10, PM ≈ 83°
    - [ ] 1.D  Paste new gains into `regbot_mg.m`
    - [ ] 1.E  Commit
- [ ] **Phase 2** — Task 2 redesign (re-linearise, new Kptilt/titilt/tdtilt/tipost)
- [ ] **Phase 3** — Task 3 redesign (re-linearise Gvel,outer)
- [ ] **Phase 4** — Task 4 redesign (re-linearise Gpos,outer)
- [ ] **Phase 5** — Push gains into `config/regbot_group47.ini`, verify in Simulink (startAngle=10, topos=2)
- [ ] **Phase 6** — Hardware validation (Tests 0, 3a, 3b, 4 with `v3_onfloor` suffix)
- [ ] **Phase 7** — Documentation sweep (`docs/*.md` + Report LaTeX)
- [ ] **Phase 8** — Merge `day5-redesign` → `main` on both submodules (`--no-ff`), then bump DTU main pointers

---

## Log entries (fill as phases complete)

| Date | Phase | Note |
|---|---|---|
| 2026-04-22 | 0 | Branch created on both submodules; roadmap in place. |
