%% =======================================================================
%  Task 2 — Balance controller (Lecture 10 Method 2)  step-by-step
%  =======================================================================
%
%  Walks through the design DECISION BY DECISION, matching the four steps
%  of Method 2. Run the whole script and read top-to-bottom: each section
%  prints what we do and why, and saves the figure that visualises it.
%
%  Plant:   Gtilt(s) = vel_ref -> tilt angle  (from regbot_1mg.slx with the
%           Task 1 wheel-velocity loop closed)
%           7th-order, 1 RHP pole (the "falling" mode of the inverted
%           pendulum), 1 RHP zero (the wheels-must-roll-the-wrong-way-first
%           non-minimum-phase signature).
%
%  Specs:   wc = 15 rad/s, gamma_M >= 60 deg, Ni = 3
%
%  Method 2 (Lecture 10 slide 11) is four steps:
%
%    Step 0  Inspect the plant            -> figures 100, 101, 105, 106
%    Step 1  Nyquist sign-check (K_PS)    -> console only
%    Step 2  Post-integrator at peak      -> figures 300, 301
%    Step 3  Outer PI-Lead                -> figure 305 (phase-balance check)
%    Step 4  Verify                       -> figures 302, 304
%  =======================================================================

close all; clear;

% --- Load workspace ------------------------------------------------------
addpath(fileparts(mfilename('fullpath')));
regbot_mg;

s       = tf('s');
model   = 'regbot_1mg';
IMG_DIR = pick_image_dir();


%% ====================== STEP 0 — IDENTIFY THE PLANT ====================
% Open the balance loop before linearising so Gtilt is the true plant from
% vel_ref to tilt angle (not a partially-closed loop). We zero the four
% gains the balance controller uses; the wheel-velocity loop (Task 1) stays
% active.
Kptilt = 0;       % breaks balance loop at the Kptilt gain
tdtilt = 0;       % silences the gyro Lead path
titilt = 1;       % benign TF placeholder
tipost = 1;       % benign TF placeholder

load_system(model);
open_system(model);

% --- Linearise Gwv: voltage -> wheel velocity ----------------------------
% Insert an open-input at the model's voltage limiter output and an
% open-output at the wheel-velocity filter, then linearise at t = 0.
% Convert state-space -> tf and minreal-cancel matched pole/zero pairs.
io_wv(1) = linio([model '/Limit9v'],          1, 'openinput');
io_wv(2) = linio([model '/wheel_vel_filter'], 1, 'openoutput');
setlinio(model, io_wv);
sys_wv     = linearize(model, io_wv, 0);
[num, den] = ss2tf(sys_wv.A, sys_wv.B, sys_wv.C, sys_wv.D);
Gwv        = minreal(tf(num, den));

% --- Linearise Gtilt: vel_ref -> tilt angle ------------------------------
io_tilt(1) = linio([model '/vel_ref'],            1, 'openinput');
io_tilt(2) = linio([model '/robot with balance'], 1, 'openoutput');
setlinio(model, io_tilt);
sys_tilt   = linearize(model, io_tilt, 0);
[num, den] = ss2tf(sys_tilt.A, sys_tilt.B, sys_tilt.C, sys_tilt.D);
Gtilt      = minreal(tf(num, den));

P_count    = sum(real(pole(Gtilt))>0);
dc         = dcgain(Gtilt);
p_unstable = max(real(pole(Gtilt)));   % rate of the falling mode (rad/s)

fprintf('==============================================================\n');
fprintf('  STEP 0 — IDENTIFY THE PLANT\n');
fprintf('==============================================================\n');
fprintf('  Gwv(s)  = voltage -> wheel velocity   (Task 1 inner loop)\n');
print_tf('Gwv', Gwv);
fprintf('  Gtilt(s) = vel_ref -> tilt angle      (Task 2 outer plant)\n');
print_tf('Gtilt', Gtilt);

% --- Describe Gtilt: poles, zeros, DC gain, RHP-pole count ---------------
fprintf('  Poles:  '); fprintf('%7.2f  ', sort(real(pole(Gtilt)))); fprintf('\n');
fprintf('  Zeros:  '); fprintf('%7.2f  ', sort(real(zero(Gtilt)))); fprintf('\n');
fprintf('  DC gain   = %.4e\n', dcgain(Gtilt));
fprintf('  RHP poles = %d  (anything > 0 means the plant is unstable)\n\n', ...
        sum(real(pole(Gtilt))>0));

fprintf('  RHP poles of Gtilt   = %d   (P = %d for Nyquist bookkeeping)\n', P_count, P_count);
fprintf('  DC gain of Gtilt     = %+.3e\n', dc);
if P_count > 0
    fprintf('  -> Plant is UNSTABLE: tilt grows like e^(%.2f t) without feedback.\n\n', ...
            p_unstable);
else
    fprintf('  -> Plant is stable.\n\n');
end

% Plant-ID plots (used in the report + Obsidian doc)
save_plot(figure(100), @() bode(Gwv), ...
    'G_{wv}: voltage -> wheel velocity', ...
    IMG_DIR, 'regbot_Gwv_bode.png');

save_plot(figure(101), @() bode(Gtilt), ...
    'G_{tilt}: vel_{ref} -> tilt angle', ...
    IMG_DIR, 'regbot_Gtilt_bode.png');

% Pole-zero maps (full + zoomed view of each plant)
figure(102); clf; zplane(zero(Gwv),   pole(Gwv));   grid on
title('Pole-zero map: G_{wv}');
saveas(gcf, fullfile(IMG_DIR, 'regbot_Gwv_pzmap.png'));

figure(103); clf; zplane(zero(Gtilt), pole(Gtilt)); grid on
title('Pole-zero map: G_{tilt}');
saveas(gcf, fullfile(IMG_DIR, 'regbot_Gtilt_pzmap.png'));

figure(104); clf; zplane(zero(Gwv),   pole(Gwv));   grid on
xlim([-50 50]); ylim([-50 50]);
title('Pole-zero map: G_{wv} (zoomed)');
saveas(gcf, fullfile(IMG_DIR, 'regbot_Gwv_pzmap_zoom.png'));

figure(105); clf; zplane(zero(Gtilt), pole(Gtilt)); grid on
xlim([-50 50]); ylim([-50 50]);
title('Pole-zero map: G_{tilt} (zoomed)');
saveas(gcf, fullfile(IMG_DIR, 'regbot_Gtilt_pzmap_zoom.png'));

% Nyquist of Gtilt with the critical point (-1, 0) marked.
figure(106); clf; nyquist(Gtilt); grid on; hold on
plot(-1, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
title(sprintf('Nyquist: G_{tilt}  (RHP poles P = %d)', P_count));
saveas(gcf, fullfile(IMG_DIR, 'regbot_Gtilt_nyquist.png'));


%% ====================== STEP 1 — SIGN OF K_PS ==========================
% Nyquist criterion: Z = N + P. For Gtilt we have P = 1, want Z = 0, so
% need N = -1 (one CCW encirclement of -1). Positive K_PS only scales the
% Nyquist curve radially; with DC gain > 0 it cannot produce CCW. So we
% need K_PS < 0. The minus sign is bundled into the post-integrator.
w_grid = logspace(-2, 4, 4000);
[mag_g, ~] = bode(Gtilt, w_grid);
mag_g = squeeze(mag_g);

if dc > 0
    sign_K = -1;
    sign_reason = 'DC gain > 0 AND P = 1  =>  sign(K_PS) = -1';
else
    sign_K = +1;
    sign_reason = 'DC gain < 0  =>  sign(K_PS) = +1 may suffice';
end

fprintf('==============================================================\n');
fprintf('  STEP 1 — SIGN OF K_PS  (Nyquist bookkeeping)\n');
fprintf('==============================================================\n');
fprintf('  Z = N + P   want Z = 0   P = %d   ->  need N = -%d  (CCW encirclements)\n', ...
        P_count, P_count);
fprintf('  %s\n', sign_reason);
fprintf('  -> The -1 will be bundled into the post-integrator in Step 2.\n\n');


%% ====================== STEP 2 — POST-INTEGRATOR =======================
% Place the PI zero at the frequency where |Gtilt| peaks. This does two
% things: (i) magnitude reshape -- the +20 dB/dec from the zero cancels
% the rise into the resonance, so the combined |Gtilt,post| is monotonic;
% (ii) combined with the sign flip it produces the required CCW encirclement.
[mag_peak, k_peak] = max(mag_g);
w_ip       = w_grid(k_peak);
tau_ip     = 1 / w_ip;

C_PI_post  = (tau_ip*s + 1) / (tau_ip*s);
Gtilt_post = sign_K * C_PI_post * Gtilt;

fprintf('==============================================================\n');
fprintf('  STEP 2 — POST-INTEGRATOR AT THE MAGNITUDE PEAK\n');
fprintf('==============================================================\n');
fprintf('  |Gtilt|_max          = %.4f   at  w_peak = %.3f rad/s\n', mag_peak, w_ip);
fprintf('  tau_i,post = 1/w_peak = %.4f s\n', tau_ip);
print_tf('C_PI_post', C_PI_post);
fprintf('  Stabilised plant     : Gtilt,post = (%+d) * C_PI,post * Gtilt\n', sign_K);
print_tf('Gtilt_post', minreal(Gtilt_post));
fprintf('  RHP poles of Gtilt,post = %d  (outer loop will close this stably)\n\n', ...
        sum(real(pole(minreal(Gtilt_post)))>0));

save_plot(figure(300), @() bode(Gtilt, Gtilt_post, w_grid), ...
    'Step 2: G_{tilt} (blue) vs. G_{tilt,post} (orange) -- post-integrator flattens the peak', ...
    IMG_DIR, 'regbot_task2_bode_post.png');
legend('G_{tilt}(s)', '-C_{PI,post}(s) G_{tilt}(s)', 'Location', 'best');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task2_bode_post.png'));

% Nyquist of the stabilised plant: should make one CCW encirclement of (-1,0).
figure(301); clf; nyquist(Gtilt_post); grid on; hold on
plot(-1, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2);
title('Nyquist: G_{tilt,post}  (one CCW encirclement of -1)');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task2_nyquist_post.png'));


%% ====================== STEP 3 — OUTER PI-LEAD =========================
% Standard PI-Lead recipe on the well-behaved Gtilt,post -- same as Task 1
% with one extra step: the phase deficit at wc is large, so a Lead is
% needed. We use the gyro shortcut: C_Lead = (tau_d s + 1) implemented as
% tau_d * gyro + theta in Simulink, no filter pole needed.

% 3a. Specs (same defaults as Task 1)
wc_tilt  = 15;       % target crossover [rad/s]
gamma_M  = 60;       % phase margin spec [deg]
Ni_tilt  = 3;        % PI zero at wc/Ni

% 3b. Outer PI zero
tau_i_tilt = Ni_tilt / wc_tilt;
C_PI_tilt  = (tau_i_tilt*s + 1) / (tau_i_tilt*s);

% 3c. Phase balance
[~, phi_G]  = bode(Gtilt_post, wc_tilt);
phi_PI      = -atand(1/Ni_tilt);
phi_Lead    = -180 + gamma_M - phi_G - phi_PI;

% 3d. Lead via the gyro shortcut
if phi_Lead <= 0
    tau_d  = 0;  C_Lead = tf(1);
    lead_note = 'no Lead needed';
elseif phi_Lead >= 89
    tau_d  = NaN; C_Lead = tf(1);
    lead_note = sprintf('WARN: phi_Lead = %.1f deg too high', phi_Lead);
else
    tau_d  = tand(phi_Lead) / wc_tilt;
    C_Lead = tau_d*s + 1;
    lead_note = 'gyro-based ideal Lead (tau_d*gyro + theta)';
end

% 3e. Solve Kp
magL    = squeeze(bode(C_PI_tilt * C_Lead * Gtilt_post, wc_tilt));
Kp_tilt = 1 / magL;

% Full controller assembly
C_outer_tilt = Kp_tilt * C_PI_tilt * C_Lead;
L_tilt       = C_outer_tilt * Gtilt_post;
C_total_tilt = Kp_tilt * sign_K * C_PI_post * C_PI_tilt * C_Lead;

fprintf('==============================================================\n');
fprintf('  STEP 3 — OUTER PI-LEAD ON G_{tilt,post}\n');
fprintf('==============================================================\n');
fprintf('  3a. Specs:           wc = %.1f rad/s   gamma_M = %.0f deg   Ni = %d\n', ...
        wc_tilt, gamma_M, Ni_tilt);
fprintf('  3b. Outer PI zero:   tau_i = Ni/wc       = %.4f s\n', tau_i_tilt);
fprintf('  3c. Phase balance at wc:\n');
fprintf('      phi_Gtilt,post(j wc) = %+7.2f deg\n', phi_G);
fprintf('      phi_PI(j wc)         = %+7.2f deg     (= -atan(1/Ni))\n', phi_PI);
fprintf('      phi_Lead required    = %+7.2f deg     (= -180 + gamma_M - phi_G - phi_PI)\n', phi_Lead);
fprintf('  3d. Lead:            tau_d = tan(phi_Lead)/wc = %.4f s   (%s)\n', tau_d, lead_note);
fprintf('  3e. Loop gain:       |C_PI * C_Lead * Gtilt,post|_{wc} = %.4f\n', magL);
fprintf('                       Kp = 1/|.|             = %.4f\n\n', Kp_tilt);
print_tf('C_outer = Kp * C_PI * C_Lead', C_outer_tilt);
print_tf('C_total (full cascade with sign flip + post-integrator)', minreal(C_total_tilt));

% Phase-balance visual: combined loop Bode with wc + PM-line markers
figure(305); clf
bode(C_PI_tilt * C_Lead * Gtilt_post, {0.1, 1000});
grid on;
ax_all = findall(gcf, 'type', 'axes');
phase_ax = ax_all(1);
mag_ax   = ax_all(2);
xline(mag_ax,   wc_tilt, 'r--', sprintf('\\omega_c = %g', wc_tilt));
xline(phase_ax, wc_tilt, 'r--', sprintf('\\omega_c = %g', wc_tilt));
yline(phase_ax, -180 + gamma_M, 'g--', sprintf('-180+%d°  (PM line)', gamma_M));
title(mag_ax, 'Step 3: phase at \omega_c after PI+Lead -- should sit on the PM line');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task2_phase_balance.png'));


%% ====================== STEP 4 — VERIFY =================================
T_tilt   = feedback(L_tilt, 1);
cl_poles = pole(minreal(T_tilt));
rhp_cl   = sum(real(cl_poles) > 0);
[GMt, PMt, ~, wct_ach] = margin(L_tilt);

fprintf('==============================================================\n');
fprintf('  STEP 4 — VERIFY\n');
fprintf('==============================================================\n');
fprintf('  margin(L_tilt):\n');
fprintf('    Achieved wc            = %.2f rad/s   (target %.1f)\n', wct_ach, wc_tilt);
fprintf('    Phase margin           = %.2f deg     (target %.0f)\n', PMt, gamma_M);
fprintf('    Gain margin            = %.2f dB      (negative is OK on P=1 plants:\n', 20*log10(GMt));
fprintf('                                          lower bound on |K| for stability)\n');
fprintf('  Closed-loop poles (real parts, sorted):\n');
fprintf('    '); fprintf('%+7.2f  ', sort(real(cl_poles))); fprintf('\n');
if rhp_cl == 0
    stab_msg = '(stable ✓)';
else
    stab_msg = '(UNSTABLE — redesign)';
end
fprintf('  RHP closed-loop poles    = %d   %s\n\n', rhp_cl, stab_msg);

save_plot(figure(302), @() margin(L_tilt), ...
    'Step 4: Open-loop  L = K_P C_{PI} C_{Lead} G_{tilt,post}', ...
    IMG_DIR, 'regbot_task2_loop_bode.png');

save_plot(figure(303), @() step(T_tilt, 2), ...
    'Step 4: Closed-loop step (reference tracking)', ...
    IMG_DIR, 'regbot_task2_step.png');

% Linear-model regulation response (proxy for IC release)
theta0 = deg2rad(10);
t_ic   = linspace(0, 2, 2001);
S_tilt = feedback(1, L_tilt);
[y_dist, t_out] = step(theta0 * S_tilt, t_ic);

figure(304); clf;
plot(t_out, rad2deg(y_dist), 'b', 'LineWidth',1.4); grid on
xlabel('Time [s]'); ylabel('Pitch [deg]');
title('Step 4: Linear-model recovery from \theta_0 = 10° initial tilt');
yline(0,'k:');
saveas(gcf, fullfile(IMG_DIR, 'regbot_task2_ic_response.png'));

abs_env  = abs(y_dist);
settle_i = find(abs_env > 0.02*theta0, 1, 'last');
settle_t = t_out(settle_i);
peak_us  = max(-y_dist);
fprintf('  Linear-model recovery from theta_0 = 10 deg:\n');
fprintf('    Settling time (2%% env.) = %.2f s\n', settle_t);
fprintf('    Peak undershoot         = %.2f deg   (RHP-zero signature)\n', rad2deg(peak_us));
fprintf('    Authoritative IC test   : Simulink with startAngle = 10\n\n');


%% ------------------- Write to workspace + gains block ------------------
Kptilt = Kp_tilt;
titilt = tau_i_tilt;
tdtilt = tau_d;
tipost = tau_ip;

fprintf('==============================================================\n');
fprintf('  Copy-paste this block into regbot_mg.m (Task 2 gains)\n');
fprintf('==============================================================\n');
fprintf('    Kptilt = %.4f;\n', Kptilt);
fprintf('    titilt = %.4f;\n', titilt);
fprintf('    tdtilt = %.4f;\n', tdtilt);
fprintf('    tipost = %.4f;\n\n', tipost);
fprintf('  Firmware [cbal] kp must be entered as NEGATIVE (-%.4f) --\n', Kptilt);
fprintf('  the firmware does NOT absorb the Method 2 sign flip.\n\n');
