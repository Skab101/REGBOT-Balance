# REGBOT Balance Assignment

Team repository for MATLAB code, Simulink models, and mission scripts for the 34722 Linear Control Design 1 final assignment.

## Team (Group 47)

- Andreas Skånning (s241123)
- Jonas Beck Jensen (s240324)
- Mads Rudolph (s246132)
- Sigurd Hestbech Christiansen (s245534)

## Structure

| Folder | Contents |
|--------|----------|
| `src/` | MATLAB scripts for controller design (Tasks 1-4) |
| `simulink/` | Simulink models (`regbot_1mg`, balance loop, etc.) and `regbot_mg.m` |
| `data/` | Day 5 identification `.mat` files (shared so teammates can skip Day 5) |
| `docs/` | Mirror of the REGBOT-relevant Obsidian notes (openable as a vault) |
| `scripts/` | Utility scripts (e.g. `sync_docs.sh` to refresh `docs/` from the vault) |
| `figures/` | Generated plots (Bode, step responses, XY-plane) |
| `missions/` | REGBOT mission scripts (`.txt` files) |
| `logs/` | Log files recorded from REGBOT during experiments |
| `Report/` | Symlink to the Overleaf report repo (not tracked) |

## Tasks

- [ ] **Task 1** — Wheel speed PI controller (from Day 5 voltage-to-velocity TF)
- [ ] **Task 2** — Balance controller with post-integrator (stabilises RHP pole)
- [ ] **Task 3a** — Zero-velocity balance (drift within 0.5 m)
- [ ] **Task 3b** — Square run at 0.8 m/s (side 1 m, turning radius 0.2 m)
- [ ] **Task 4** — Position controller (2 m move, max speed > 0.7 m/s)

## Prerequisites

- MATLAB with **Simscape Multibody** and **Simulink Control Design** packages
- REGBOT with calibrated gyro and tilt-offset
- Starter files from Learn → Resources/REGBOT balance resources

## Running the script

Open MATLAB in `simulink/` and run `regbot_mg`. The script automatically:
- Loads all REGBOT parameters
- Defines the Day 5 plant
- Designs the Task 1 wheel-speed PI controller
- Linearizes the Simulink model to get $G_{wv}$ and $G_{tilt}$
- Prints poles, zeros, and RHP pole counts
- Generates Bode plots, pole-zero maps, and step responses

### Plot output location

The script **auto-detects** where to save plots:

- **If you have Mads's Obsidian vault** at `DTU/Obsidian/Courses/34722 Linear Control Design 1/...`, plots go there (so they embed directly in his notes).
- **Otherwise**, plots are saved to `docs/images/`. When you open `docs/` as an Obsidian vault, the REGBOT Balance Assignment and Lesson 10 notes will show **your** freshly generated plots.

> **Don't commit `docs/images/*.png` from a teammate machine** — the tracked
> images are Mads's canonical versions synced from his vault. Your local
> regenerations will show up as uncommitted diffs; leave them alone.

To force output to `docs/images/` even when the vault is available, set `FORCE_DOCS = true` at the top of the script.

## Reading the notes

`docs/` mirrors the REGBOT-related Obsidian notes from Mads's vault:

- [`docs/REGBOT Balance Assignment.md`](docs/REGBOT%20Balance%20Assignment.md) — running progress log, Tasks 1–4
- [`docs/Lesson 10 - Unstable Systems and REGBOT Balance.md`](docs/Lesson%2010%20-%20Unstable%20Systems%20and%20REGBOT%20Balance.md) — lecture theory
- [`docs/PLAN.md`](docs/PLAN.md) — early phase/role plan

Open the `docs/` folder directly in Obsidian ("Open folder as vault") for the best reading experience — wikilinks and image embeds resolve automatically.

Mads refreshes `docs/` with:

```bash
bash scripts/sync_docs.sh
```

Teammates: you never need to run this — just `git pull` and the latest notes are there.

## Report

The LaTeX report lives in a separate repo:
`git@github.com:MadsRudolph/REGBOT-Balance-assignment.git`

Accessible here via the `Report/` symlink.
