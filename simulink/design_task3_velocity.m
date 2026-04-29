%% =======================================================================
%  Task 3 — Velocity outer loop (PI)  step-by-step
%  =======================================================================
%
%  Walks through the design DECISION BY DECISION. Run the whole script and
%  read top-to-bottom: each section prints what we do and why, and saves
%  the figure that visualises it.
%
%  Plant:   Gvel,outer(s) = theta_ref -> wheel_vel_filter  (with Tasks 1+2
%           closed and active in the workspace).
%           Stable (Task 2 stabilised the pendulum), but has an RHP zero
%           at +8.5 rad/s -- a hard physics limit on bandwidth.
%
%  Specs:   wc_vel = 1 rad/s   (well below z/5 ~= 1.7 rad/s),
%           gamma_M >= 60 deg, Ni = 3.
%
%  Steps:
%    Step 0  Identify the plant         -> figure 402
%    Step 1  Pick specs                 -> console only
%    Step 2  Place PI zero              -> console only
%    Step 3  Phase balance              -> figure 403 (combined-Bode check)
%    Step 4  Solve Kp                   -> console only
%    Step 5  Verify                     -> figures 400, 401
%
%  Run this script on its own, AFTER pasting Task 1 + Task 2 gains into
%  regbot_mg.m. The Simulink model must already have:
%    - A Velocity PI block named whose output passes through Kpvel_gain
%    - The block named exactly 'Kpvel_gain' (so linearize can find it)
%  =======================================================================

close all; clear;

% --- Load workspace ------------------------------------------------------
addpath(fileparts(mfilename('fullpath')));
regbot_mg;

s       = tf('s');
model   = 'regbot_1mg';
IMG_DIR = pick_image_dir();

VEL_CTRL_OUT_BLOCK = '/Kpvel_gain';   % linearisation break point


%% ====================== STEP 0 — IDENTIFY THE PLANT ====================
% Linearise theta_ref -> wheel_vel_filter with the velocity loop broken at
% the Kpvel_gain output. Tasks 1 + 2 stay closed (they're already active in
% regbot_mg.m), so this is the plant the velocity controller sees.
%
% Defensive zeroing in case someone edited regbot_mg:
Kpvel = 0;     %#ok<NASGU>
tivel = 1;     %#ok<NASGU>

load_system(model);
open_system(model);

Gvel_outer = identify_tf(model, VEL_CTRL_OUT_BLOCK, '/wheel_vel_filter');

P_count = sum(real(pole(Gvel_outer)) > 0);
z_all   = zero(Gvel_outer);
rhp_z   = z_all(real(z_all) > 0);

fprintf('==============================================================\n');
fprintf('  STEP 0 — IDENTIFY THE PLANT  (Tasks 1+2 closed, Task 3 open)\n');
fprintf('==============================================================\n');
fprintf('  Gvel,outer(s) = theta_ref -> wheel_vel_filter\n');
print_tf('Gvel_outer', Gvel_outer);
describe_plant(Gvel_outer);
fprintf('  RHP poles    = %d   (Task 2 stabilisation worked if 0)\n', P_count);
if ~isempty(rhp_z)
    fprintf('  RHP zeros    = %d   at  ', numel(rhp_z));
    fprintf('%+7.3f  ', real(rhp_z));
    fprintf('rad/s   <-- physics: caps achievable wc\n\n');
else
    fprintf('  RHP zeros    = 0\n\n');
end

figure(402); plot_pz_stability(Gvel_outer, 'G_{vel,outer}');
xlim([-50 10]); ylim([-50 50]);
saveas(gcf, fullfile(IMG_DIR, 'regbot_task3_plant_pz.png'));


%% ====================== STEP 1 — PICK SPECS ============================
% wc -- bandwidth knob, with TWO upper bounds:
%   (a) Cascade-separation rule: outer loop should be at least ~5x slower
%       than the inner balance loop, so wc <= 15/5 = 3 rad/s.
%   (b) RHP-zero limit:          wc <= z/5 ~= 1.7 rad/s.
% (b) is the tighter of the two. We pick wc = 1 to stay well under.
% gamma_M -- 60 deg, course default.
% Ni      -- 3, course default.
wc_vel       = 1;        % target crossover [rad/s]
gamma_M_spec = 60;       % phase margin spec [deg]
Ni_vel       = 3;        % PI zero at wc/Ni

if ~isempty(rhp_z)
    z_min = min(real(rhp_z));
    wc_max = z_min / 5;
else
    z_min = NaN;
    wc_max = Inf;
end

fprintf('==============================================================\n');
fprintf('  STEP 1 — PICK SPECS\n');
fprintf('==============================================================\n');
fprintf('  wc      = %.2f rad/s   (target -- below z/5 = %.2f, RHP-zero ceiling)\n', ...
        wc_vel, wc_max);
fprintf('  gamma_M >= %.0f deg     (course default; ~10%% step overshoot)\n', gamma_M_spec);
fprintf('  Ni      = %d            (PI zero at wc/Ni; course default)\n\n', Ni_vel);


%% ====================== STEP 2 — PLACE PI ZERO =========================
% Same recipe as Task 1: tau_i = Ni / wc.
tau_i_vel = Ni_vel / wc_vel;
C_PI_vel  = (tau_i_vel*s + 1) / (tau_i_vel*s);

fprintf('==============================================================\n');
fprintf('  STEP 2 — PLACE THE PI ZERO\n');
fprintf('==============================================================\n');
fprintf('  tau_i = Ni/wc        = %.4f s\n', tau_i_vel);
fprintf('  PI zero              = %.4f rad/s   (one-third of wc)\n', 1/tau_i_vel);
fprintf('  C_PI(s)              = (%.4f s + 1) / (%.4f s)\n\n', tau_i_vel, tau_i_vel);


%% ====================== STEP 3 — PHASE BALANCE =========================
% Read phase of (C_PI * Gvel,outer) at wc and decide if a Lead is needed.
[magL_unscaled, phi_G_unwrapped] = bode(C_PI_vel * Gvel_outer, wc_vel);
magL_unscaled    = squeeze(magL_unscaled);
phi_G_unwrapped  = squeeze(phi_G_unwrapped);

% MATLAB returns continuously-unwrapped phase. For a 9th-order plant the
% unwrap can add +360 deg before reaching wc; wrap into [-180, 180] for the
% physical reading.
phi_L_phys = mod(phi_G_unwrapped + 180, 360) - 180;
gamma_M_natural = 180 + phi_L_phys;

phi_PI = -atand(1/Ni_vel);
% Plant-only phase = combined - PI contribution
phi_G_only = phi_L_phys - phi_PI;

phi_Lead = -180 + gamma_M_spec - phi_G_only - phi_PI;

fprintf('==============================================================\n');
fprintf('  STEP 3 — PHASE BALANCE: DO WE NEED A LEAD?\n');
fprintf('==============================================================\n');
fprintf('  MATLAB phase (unwrapped) = %+7.2f deg\n', phi_G_unwrapped);
fprintf('  Phase wrapped to [-180,180] = %+7.2f deg   <-- physical reading\n', phi_L_phys);
fprintf('  Plant-only phase           = %+7.2f deg\n', phi_G_only);
fprintf('  PI phase contribution      = %+7.2f deg\n', phi_PI);
fprintf('  Natural gamma_M            = %+7.2f deg     (= 180 + phase)\n', gamma_M_natural);
fprintf('  Spec                       = %+7.2f deg\n', gamma_M_spec);
if gamma_M_natural >= gamma_M_spec
    fprintf('  -> Natural gamma_M exceeds spec by %.2f deg. NO LEAD NEEDED.\n\n', ...
            gamma_M_natural - gamma_M_spec);
else
    fprintf('  -> Natural gamma_M is below spec by %.2f deg. Lead REQUIRED.\n\n', ...
            gamma_M_spec - gamma_M_natural);
end

% Visual: combined Bode with wc + PM-line markers
figure(403); clf
bode(C_PI_vel * Gvel_outer, {0.01, 100});
grid on;
ax_all = findall(gcf, 'type', 'axes');
phase_ax = ax_all(1);
mag_ax   = ax_all(2);
xline(mag_ax,   wc_vel, 'r--', sprintf('\\omega_c = %g', wc_vel));
xline(phase_ax, wc_vel, 'r--', sprintf('\\omega_c = %g', wc_vel));
yline(phase_ax, -180 + gamma_M_spec, 'g--', sprintf('-180+%d°  (PM line)', gamma_M_spec));
title(mag_ax, 'Step 3: phase at \omega_c -- above the green line means no Lead needed');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task3_phase_balance.png'));


%% ====================== STEP 4 — SOLVE Kp ==============================
% Magnitude condition |L(j wc)| = 1. Same as Task 1 but the plant is
% LOUD at wc (free integrator from Task 2's post-integrator), so Kp < 1.
Kp_vel = 1 / magL_unscaled;

fprintf('==============================================================\n');
fprintf('  STEP 4 — SOLVE K_p so |L(j wc)| = 1\n');
fprintf('==============================================================\n');
fprintf('  |C_PI * Gvel,outer|_{wc} = %.4f       (need to scale to 1.000)\n', magL_unscaled);
if magL_unscaled > 1
    fprintf('  Curve is %+.2f dB ABOVE 0 dB at wc -- attenuate to crossover.\n', 20*log10(magL_unscaled));
else
    fprintf('  Curve is %+.2f dB BELOW 0 dB at wc -- amplify to crossover.\n', 20*log10(magL_unscaled));
end
fprintf('  Kp = 1/|.|               = %.4f\n\n', Kp_vel);

C_vel = Kp_vel * C_PI_vel;
L_vel = C_vel * Gvel_outer;
T_vel = feedback(L_vel, 1);
print_tf('C_vel = Kp * (tau_i s + 1)/(tau_i s)', C_vel);


%% ====================== STEP 5 — VERIFY =================================
[GM, PM, ~, wc_ach] = margin(L_vel);

fprintf('==============================================================\n');
fprintf('  STEP 5 — VERIFY\n');
fprintf('==============================================================\n');
fprintf('  Achieved wc            = %.2f rad/s   (target %.0f)\n', wc_ach, wc_vel);
fprintf('  Phase margin           = %.2f deg     (target >= %.0f)\n', PM, gamma_M_spec);
fprintf('  Gain margin            = %.2f dB\n', 20*log10(GM));
fprintf('  Closed-loop RHP poles  = %d\n\n',   sum(real(pole(T_vel)) > 0));

save_plot(figure(400), @() margin(L_vel), ...
    'Step 5: Open-loop  L = C_{vel} G_{vel,outer}', ...
    IMG_DIR, 'regbot_task3_loop_bode.png');

save_plot(figure(401), @() step(T_vel, 5), ...
    'Step 5: Closed-loop step (v_{ref} = 1 m/s)', ...
    IMG_DIR, 'regbot_task3_step.png');


%% ------------------- Write to workspace + gains block ------------------
Kpvel = Kp_vel;
tivel = tau_i_vel;

fprintf('==============================================================\n');
fprintf('  Copy-paste this block into regbot_mg.m (Task 3 gains)\n');
fprintf('==============================================================\n');
fprintf('    Kpvel  = %.4f;\n', Kpvel);
fprintf('    tivel  = %.4f;\n\n', tivel);
