%% =======================================================================
%  Task 4 — Position outermost loop (P / P-Lead)  step-by-step
%  =======================================================================
%
%  Walks through the design DECISION BY DECISION. Run the whole script and
%  read top-to-bottom: each section prints what we do and why, and saves
%  the figure that visualises it.
%
%  Plant:   Gpos,outer(s) = pos_ref -> x_position  (with Tasks 1+2+3
%           closed and active in the workspace).
%           Type-1 (free integrator from v -> x), so a pure P controller
%           drives step-tracking error to zero -- no I-term needed.
%
%  Specs:   wc_pos = 0.6 rad/s  (iterated against mission specs, not
%                                derived from cascade rules),
%           gamma_M >= 60 deg.
%
%  Steps:
%    Step 0  Identify the plant         -> figure 502
%    Step 1  Pick wc against mission    -> console only
%    Step 2  Phase balance (Lead?)      -> figure 503 (combined-Bode check)
%    Step 3  Solve Kp                   -> console only
%    Step 4  Lead drop note (improper)  -> console only
%    Step 5  Verify                     -> figures 500, 501
%
%  Run this script on its own, AFTER pasting Tasks 1, 2, and 3 gains into
%  regbot_mg.m. The Simulink model must already have:
%    - A position-controller chain whose output passes through Kppos_gain
%    - The block named exactly 'Kppos_gain'
%    - On 'robot with balance', port 3 = x_position
%  =======================================================================

close all; clear;

% --- Load workspace ------------------------------------------------------
addpath(fileparts(mfilename('fullpath')));
regbot_mg;

s       = tf('s');
model   = 'regbot_1mg';
IMG_DIR = pick_image_dir();

POS_CTRL_OUT_BLOCK = '/Kppos_gain';
X_POS_BLOCK        = '/robot with balance';
X_POS_PORT         = 3;       % port 3 = x_position


%% ====================== STEP 0 — IDENTIFY THE PLANT ====================
% Linearise pos_ref -> x_position with the position loop broken at the
% Kppos_gain output. Tasks 1, 2, 3 stay closed.
Kppos = 0;     %#ok<NASGU>

load_system(model);
open_system(model);

% --- Linearise Gpos,outer: pos_ref -> x_position ------------------------
% Open the position loop at the Kppos_gain output and read x_position
% (port 3 of the 'robot with balance' subsystem). Tasks 1+2+3 stay closed.
io(1) = linio([model POS_CTRL_OUT_BLOCK], 1,          'openinput');
io(2) = linio([model X_POS_BLOCK],        X_POS_PORT, 'openoutput');
setlinio(model, io);
sys        = linearize(model, io, 0);
[num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gpos_outer = minreal(tf(num, den));

P_count   = sum(real(pole(Gpos_outer)) > 0);
n_int     = sum(abs(pole(Gpos_outer)) < 1e-6);   % poles ~ 0

fprintf('==============================================================\n');
fprintf('  STEP 0 — IDENTIFY THE PLANT  (Tasks 1+2+3 closed, Task 4 open)\n');
fprintf('==============================================================\n');
fprintf('  Gpos,outer(s) = pos_ref -> x_position\n');
print_tf('Gpos_outer', Gpos_outer);

% --- Describe Gpos,outer: poles, zeros, DC gain, RHP-pole count ---------
fprintf('  Poles:  '); fprintf('%7.2f  ', sort(real(pole(Gpos_outer)))); fprintf('\n');
fprintf('  Zeros:  '); fprintf('%7.2f  ', sort(real(zero(Gpos_outer)))); fprintf('\n');
fprintf('  DC gain   = %.4e\n', dcgain(Gpos_outer));
fprintf('  RHP poles = %d  (anything > 0 means the plant is unstable)\n\n', ...
        sum(real(pole(Gpos_outer))>0));

fprintf('  RHP poles                  = %d\n', P_count);
fprintf('  Integrators (poles ~ 0)    = %d   (>= 1 expected, from v -> x)\n', n_int);
if n_int >= 1
    fprintf('  -> Plant is Type-%d. Pure P alone gives e_ss = 0 on a step.\n\n', n_int);
end

figure(502); clf; zplane(zero(Gpos_outer), pole(Gpos_outer)); grid on
xlim([-10 2]); ylim([-10 10]);
title('Pole-zero map: G_{pos,outer}');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task4_plant_pz.png'));


%% ====================== STEP 1 — PICK wc =================================
% Unlike Tasks 1-3, wc is NOT picked from cascade/PM rules. It's iterated
% against the mission spec: reach 2 m, peak v >= 0.7 m/s, in <= 10 s.
%
% Iteration history (recorded in the doc table):
%   wc = 0.2  -> peak v 0.33 m/s, settle 20 s   FAIL
%   wc = 0.5  -> peak v 0.68 m/s, settle 12 s   close
%   wc = 0.6  -> peak v ~0.82 m/s, settle ~10 s CHOSEN
wc_pos       = 0.6;       % chosen against mission specs
gamma_M_spec = 60;        % course default

fprintf('==============================================================\n');
fprintf('  STEP 1 — PICK wc  (mission-driven, not cascade-driven)\n');
fprintf('==============================================================\n');
fprintf('  wc = %.2f rad/s   (iterated to clear peak v >= 0.7 m/s in <= 10 s)\n', wc_pos);
fprintf('  gamma_M >= %.0f deg\n\n', gamma_M_spec);


%% ====================== STEP 2 — PHASE BALANCE =========================
% No PI this time (plant already Type-1), so the equation simplifies:
%   gamma_M - 180 = phi_G + phi_Lead
%
% MATLAB returns a continuously-unwrapped phase. For this 11th-order plant
% the unwrap can add 360 deg before reaching wc; wrap into [-180, 180].
[~, phi_G_unwrapped] = bode(Gpos_outer, wc_pos);
phi_G_unwrapped = squeeze(phi_G_unwrapped);
phi_G    = mod(phi_G_unwrapped + 180, 360) - 180;
phi_Lead = mod(-180 + gamma_M_spec - phi_G + 180, 360) - 180;

if phi_Lead <= 0
    tau_d_pos   = 0;  C_Lead_pos = tf(1);
    lead_note   = 'no Lead needed -- phase margin already met';
elseif phi_Lead >= 89
    tau_d_pos   = NaN; C_Lead_pos = tf(1);
    lead_note   = sprintf('WARN: phi_Lead = %.1f deg too high', phi_Lead);
else
    tau_d_pos   = tand(phi_Lead) / wc_pos;
    C_Lead_pos  = tau_d_pos*s + 1;
    lead_note   = 'standard ideal Lead (tau_d s + 1)';
end

fprintf('==============================================================\n');
fprintf('  STEP 2 — PHASE BALANCE: HOW MUCH LEAD?\n');
fprintf('==============================================================\n');
fprintf('  MATLAB phase (unwrapped) = %+7.2f deg\n', phi_G_unwrapped);
fprintf('  Phase wrapped to [-180,180] = %+7.2f deg   <-- physical reading\n', phi_G);
fprintf('  Required phi_Lead          = %+7.2f deg     (= -180 + gamma_M - phi_G)\n', phi_Lead);
fprintf('  tau_d = tan(phi_Lead)/wc   = %.4f s\n', tau_d_pos);
fprintf('  C_Lead(s)                 = %.4f s + 1     (%s)\n\n', tau_d_pos, lead_note);

% Visual
figure(503); clf
bode(C_Lead_pos * Gpos_outer, {0.01, 100});
grid on;
ax_all = findall(gcf, 'type', 'axes');
phase_ax = ax_all(1);
mag_ax   = ax_all(2);
xline(mag_ax,   wc_pos, 'r--', sprintf('\\omega_c = %g', wc_pos));
xline(phase_ax, wc_pos, 'r--', sprintf('\\omega_c = %g', wc_pos));
yline(phase_ax, -180 + gamma_M_spec, 'g--', sprintf('-180+%d°  (PM line)', gamma_M_spec));
title(mag_ax, 'Step 2: phase at \omega_c with ideal Lead applied');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task4_phase_balance.png'));


%% ====================== STEP 3 — SOLVE Kp ==============================
magL    = squeeze(bode(C_Lead_pos * Gpos_outer, wc_pos));
Kp_pos  = 1 / magL;

fprintf('==============================================================\n');
fprintf('  STEP 3 — SOLVE K_p so |L(j wc)| = 1\n');
fprintf('==============================================================\n');
fprintf('  |C_Lead * Gpos,outer|_{wc} = %.4f\n', magL);
fprintf('  Kp = 1 / |.|               = %.4f\n\n', Kp_pos);


%% ====================== STEP 4 — LEAD DECISION =========================
% The ideal Lead (tau_d s + 1) is IMPROPER (numerator degree > denominator
% degree). Simulink's Transfer Fcn block refuses improper TFs. Three
% implementation options:
%   (a) Ideal Lead (tau_d s + 1)                              -- rejected
%   (b) Proper Lead (tau_d s + 1)/(alpha tau_d s + 1), alpha<1
%       -- adds a fast filter pole that costs some phase back
%   (c) Drop the Lead entirely -- accept phi_Lead deg of PM cost
%
% Rule: if the required phi_Lead is small enough that the PM cost is
% acceptable (LEAD_DROP_THRESHOLD_DEG), pick (c) for simplicity. Otherwise
% pick (b) with a default alpha so the firmware PM still meets the spec.
LEAD_DROP_THRESHOLD_DEG = 5;
ALPHA                   = 0.1;

if phi_Lead <= LEAD_DROP_THRESHOLD_DEG
    tdpos_firmware  = 0;
    C_Lead_firmware = tf(1);
    decision = sprintf(...
        'drop Lead (phi_Lead = %.2f deg <= threshold %.1f deg)', ...
        phi_Lead, LEAD_DROP_THRESHOLD_DEG);
else
    tdpos_firmware  = tau_d_pos;
    C_Lead_firmware = (tau_d_pos*s + 1) / (ALPHA*tau_d_pos*s + 1);
    decision = sprintf(...
        'proper Lead with alpha = %.2f (phi_Lead = %.2f deg > %.1f threshold)', ...
        ALPHA, phi_Lead, LEAD_DROP_THRESHOLD_DEG);
end

fprintf('==============================================================\n');
fprintf('  STEP 4 — LEAD DECISION\n');
fprintf('==============================================================\n');
fprintf('  Ideal Lead (%.4f s + 1) is improper -- Simulink rejects it.\n', tau_d_pos);
fprintf('  Required phi_Lead       = %.2f deg\n', phi_Lead);
fprintf('  Drop threshold          = %.1f deg\n', LEAD_DROP_THRESHOLD_DEG);
fprintf('  Decision                : %s\n\n', decision);


%% ====================== STEP 5 — VERIFY ================================
% Verify margins for both the design-time controller (with ideal Lead)
% and the firmware controller (whichever Lead implementation Step 4
% selected). The mission-spec checks use the firmware controller.

% Design-time (ideal Lead in series)
L_pos_design = Kp_pos * C_Lead_pos * Gpos_outer;
[GM_d, PM_d, ~, wc_d] = margin(L_pos_design);

% Firmware (Step-4-selected Lead)
L_pos_firmware = Kp_pos * C_Lead_firmware * Gpos_outer;
T_pos_firmware = feedback(L_pos_firmware, 1);
[GM_f, PM_f, ~, wc_f] = margin(L_pos_firmware);

fprintf('==============================================================\n');
fprintf('  STEP 5 — VERIFY\n');
fprintf('==============================================================\n');
fprintf('  Design-time (with ideal Lead):\n');
fprintf('    wc = %.3f rad/s    PM = %.2f deg    GM = %.2f dB\n', ...
        wc_d, PM_d, 20*log10(GM_d));
fprintf('  Firmware (%s):\n', decision);
fprintf('    wc = %.3f rad/s    PM = %.2f deg    GM = %.2f dB\n', ...
        wc_f, PM_f, 20*log10(GM_f));
fprintf('  Closed-loop RHP poles   = %d\n\n', ...
        sum(real(pole(T_pos_firmware)) > 0));

% 2 m mission step using the firmware controller (the actual deployed one)
[y_step, t_step] = step(2 * T_pos_firmware, 20);
peak_v   = max(gradient(y_step, t_step));
settle_i = find(abs(y_step - 2) > 0.02*2, 1, 'last');
settle_t = t_step(settle_i);

fprintf('  2 m step response (firmware controller):\n');
fprintf('    Peak velocity (d/dt)  = %.3f m/s    (spec: must exceed 0.7 m/s)\n', peak_v);
fprintf('    Settling time (2%%)    = %.2f s     (mission window: 10 s)\n\n', settle_t);

save_plot(figure(500), @() margin(L_pos_firmware), ...
    'Step 5: Open-loop  L = C_{pos} G_{pos,outer}  (firmware)', ...
    IMG_DIR, 'regbot_task4_loop_bode.png');

save_plot(figure(501), @() step(2 * T_pos_firmware, 20), ...
    'Step 5: Closed-loop 2 m step (pos_{ref} = 2 m, firmware)', ...
    IMG_DIR, 'regbot_task4_step.png');


%% ------------------- Write to workspace + gains block ------------------
Kppos = Kp_pos;
tdpos = tdpos_firmware;       % whatever Step 4 selected

fprintf('==============================================================\n');
fprintf('  Copy-paste this block into regbot_mg.m (Task 4 gains)\n');
fprintf('==============================================================\n');
fprintf('    Kppos  = %.4f;\n', Kppos);
fprintf('    tdpos  = %.4f;       %% Lead per Step 4 decision\n\n', tdpos);
