# Day 5 identification data

Black-box identification results from Day 5 (voltage → wheel velocity).
Used by `simulink/regbot_mg.m` for the Task 1 design.

| File | Contents | Used in |
|---|---|---|
| `Day5_results.mat` | First-order fits. Variables: `G_wu_L`, `G_wu_R`, `G_wu_avg`, `G_floor_L`, `G_floor_R`, `G_floor_avg`, `T_s` | The hardcoded `Gvel_day5 = 13.34/(s+35.71)` approximation comes from the `_floor_avg` variant. |
| `Day5_results_v2.mat` | Two-pole fits (updated identification). Variables: `G_2p_L`, `G_2p_R`, `G_2p_avg`, `T_s` | Higher-fidelity alternative; not currently used by `regbot_mg.m`. |

To load from MATLAB:

```matlab
load('data/Day5_results.mat');
Gvel = G_floor_avg;     % first-order, left + right averaged
```

Raw logs (`log_on_floor*.txt`, `log_wheels_up.txt`) and the identification
script (`Day5.m`, `Day5_v2.m`) live outside this repo in Mads's
`4. Semester/Linear Control Design/Day5/` folder.
