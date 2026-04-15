%% REGBOT Balance Assignment — Main MATLAB Script
% Simscape Multibody model of the REGBOT with a wheel-velocity controller.
% This script is the base for designing the balance control system.
%
% CONTENTS
%   1. Setup (paths, model name, Laplace variable)
%   2. REGBOT physical parameters (needed by the Simulink model)
%   3. Day 5 plant (voltage -> wheel velocity, from black-box measurement)
%   4. TASK 1: Wheel-speed PI controller design
%   5. Plant identification from Simulink (linearize)
%   6. Analysis — poles, zeros, stability check
%   7. Plots — Bode, pole-zero maps, step response
%   8. TASK 2: Balance (tilt) controller with post-integrator
%   9. TASK 3: Velocity controller (stub — run after tilt loop is in Simulink)
%  10. TASK 4: Position controller (stub — run after velocity loop is closed)
%  11. Local helper functions
%
% USAGE
%   Just run the script. All plots are saved to the Obsidian vault folder
%   under regbot/Images/ automatically.

close all; clear;

%% 1. Setup
s = tf('s');
model = 'regbot_1mg';

% Output directory for plots — auto-detected
%   - If Mads's Obsidian vault is reachable, plots go there (so the notes
%     in the vault render current figures for him).
%   - Otherwise plots go to docs/images/ in the team repo, so teammates
%     who open docs/ as an Obsidian vault see their own fresh plots in
%     the REGBOT Balance Assignment and Lesson 10 notes.
%   - Set FORCE_DOCS = true to always use docs/images/ (useful when you
%     want to preview a change as a teammate would see it).
FORCE_DOCS = false;

SCRIPT_DIR   = fileparts(mfilename('fullpath'));
REPO_ROOT    = fullfile(SCRIPT_DIR, '..');
OBSIDIAN_DIR = fullfile(REPO_ROOT, '..','..','..','Obsidian','Courses', ...
    '34722 Linear Control Design 1','Exercises','Work','regbot','Images');
DOCS_DIR     = fullfile(REPO_ROOT, 'docs','images');

if ~FORCE_DOCS && exist(OBSIDIAN_DIR, 'dir')
    IMG_DIR = OBSIDIAN_DIR;
    fprintf('Plots -> Obsidian vault: %s\n', IMG_DIR);
else
    IMG_DIR = DOCS_DIR;
    if ~exist(IMG_DIR, 'dir'), mkdir(IMG_DIR); end
    fprintf('Plots -> team repo docs: %s\n', IMG_DIR);
    fprintf('        (note: these are local regenerations — do NOT git add them)\n');
end

%% 2. REGBOT physical parameters (used by the Simulink model)
% Motor — electrical
RA   = 3.3/2;      % armature resistance [ohm] (two motors in parallel)
LA   = 6.6e-3/2;   % armature inductance [H]
Kemf = 0.0105;     % EMF / torque constant [V s/rad]
Km   = Kemf;       % torque constant [N m/A]

% Motor — mechanical
JA = 1.3e-6*2;     % rotor inertia [kg m^2] (two motors)
BA = 3e-6*2;       % rotor friction [N m s/rad] (two motors)
NG = 9.69;         % gear ratio

% Vehicle
WR = 0.03;         % wheel radius [m]
Bw = 0.155;        % distance between wheels [m]

% Masses and geometry
mmotor    = 0.193;                       % motor + gear [kg]
mframe    = 0.32;                        % frame + base PCB [kg]
mtopextra = 0.97 - mframe - mmotor;      % top mass (battery + charger) [kg]
mpdist    = 0.10;                        % distance to lid [m]
pushDist  = 0.10;                        % disturbance position (Z) [m]

% Simulation
startAngle = 10;    % initial tilt [deg] at t = 0
twvlp      = 0.005; % wheel-velocity filter time constant [s]

%% 3. Day 5 plant — voltage -> wheel velocity
% Identified from black-box measurements with the robot on its side,
% so the tilt dynamics are decoupled. Use this for Task 1 design.
Gvel_day5 = 13.34 / (s + 35.71);

fprintf('\n--- Day 5 plant: Gvel(s) = 13.34 / (s + 35.71) ---\n');
fprintf('  DC gain: %.3f  |  tau: %.3f s  |  pole: %.1f rad/s\n', ...
    dcgain(Gvel_day5), 1/35.71, pole(Gvel_day5));

%% 4. TASK 1 — Wheel-speed PI controller
% Design specifications:
%   wc_target  = 30 rad/s   (fast inner loop, below plant pole at 35.71)
%   gamma_M    >= 60 deg    (robust phase margin)
%   Ni         = 3          (PI zero placed Ni-times below crossover)
%
% Derived:
%   tau_i = Ni / wc = 3 / 30 = 0.10 s
%   Kp    = 3.31  (such that |L(j wc)| = 1)

Kpwv  = 3.31;    % Kp
tiwv  = 0.10;    % tau_i [s]
Kffwv = 0;       % feed-forward gain (not used)

% Build controller and verify the loop
C_wv = Kpwv * (tiwv*s + 1) / (tiwv*s);
L_wv = C_wv * Gvel_day5;
[GM, PM, ~, wc] = margin(L_wv);

fprintf('\n--- Task 1: Wheel-speed PI controller ---\n');
fprintf('  Kp = %.2f, tau_i = %.2f s\n', Kpwv, tiwv);
fprintf('  wc = %.1f rad/s  (target 30)\n', wc);
fprintf('  PM = %.1f deg    (target >= 60)\n', PM);
fprintf('  GM = %.1f dB\n', 20*log10(GM));

%% 5. Plant identification from Simulink
% Use linearize() to extract transfer functions from the Simulink model
% at two I/O point pairs. identify_tf() is a local helper at the bottom.
load_system(model);
open_system(model);

Gwv   = identify_tf(model, '/Limit9v', '/wheel_vel_filter');
Gtilt = identify_tf(model, '/vel_ref', '/robot with balance');

%% 6. Analysis — poles, zeros, stability
print_plant('Gwv (voltage -> wheel velocity)', Gwv);
print_plant('Gtilt (vel_ref -> tilt angle)',   Gtilt);

%% 7. Plots
% Task 1 — loop Bode and closed-loop step response
fig = figure(200);
margin(L_wv); grid on;
title('Task 1: Loop Bode  L = C_{wv} \cdot G_{vel,day5}');
saveas(fig, fullfile(IMG_DIR, 'regbot_task1_bode.png'));

fig = figure(201);
step(feedback(L_wv, 1), 0.5); grid on;
title('Task 1: Closed-loop step response (wheel speed)');
saveas(fig, fullfile(IMG_DIR, 'regbot_task1_step.png'));

% Plant Bode plots
fig = figure(100); bode(Gwv);   grid on;
title('G_{wv}: motor voltage -> wheel velocity');
saveas(fig, fullfile(IMG_DIR, 'regbot_Gwv_bode.png'));

fig = figure(101); bode(Gtilt); grid on;
title('G_{tilt}: velocity reference -> tilt angle');
saveas(fig, fullfile(IMG_DIR, 'regbot_Gtilt_bode.png'));

% Pole-zero maps — full and zoomed-in versions
fig = figure(102); plot_pz_stability(Gwv,   'Gwv');
saveas(fig, fullfile(IMG_DIR, 'regbot_Gwv_pzmap.png'));

fig = figure(103); plot_pz_stability(Gtilt, 'Gtilt');
saveas(fig, fullfile(IMG_DIR, 'regbot_Gtilt_pzmap.png'));

fig = figure(104); plot_pz_stability(Gwv,   'Gwv - zoomed');
xlim([-50 50]); ylim([-50 50]);
saveas(fig, fullfile(IMG_DIR, 'regbot_Gwv_pzmap_zoom.png'));

fig = figure(105); plot_pz_stability(Gtilt, 'Gtilt - zoomed');
xlim([-50 50]); ylim([-50 50]);
saveas(fig, fullfile(IMG_DIR, 'regbot_Gtilt_pzmap_zoom.png'));

% Nyquist plot of Gtilt — the plant has RHP pole(s), so Nyquist reveals
% whether a proportional gain can stabilise it. Per Nyquist criterion
% (Z = N + P), we need one CCW encirclement of (-1, 0) for each RHP pole.
fig = figure(106); plot_nyquist_critical(Gtilt, 'Gtilt');
saveas(fig, fullfile(IMG_DIR, 'regbot_Gtilt_nyquist.png'));

fprintf('\nDone (Task 1 + plant ID). All plots saved to the Obsidian Images folder.\n');


%% 8. TASK 2 — Balance (tilt) controller with post-integrator =============
% Gtilt(s) from vel_ref -> tilt_angle is open-loop UNSTABLE (one RHP pole
% around +8 rad/s, the falling-pendulum mode).
%
% Design strategy (from Lecture 10):
%   (a) Insert a post-integrator   C_PI_post(s) = (t_ip*s + 1)/(t_ip*s)
%       together with a sign-flip  -> G_tilt_post = -C_PI_post * Gtilt
%       t_ip is chosen so the PI zero cancels the peak of |Gtilt|, giving
%       a monotonically decreasing magnitude curve.
%   (b) Verify stabilisation on Nyquist: the new curve should encircle
%       (-1, 0) once counter-clockwise (P = 1).
%   (c) Design an outer PI-Lead on G_tilt_post using the standard flow.

fprintf('\n===== TASK 2: Balance controller design =====\n');

% --- (a) Find the magnitude peak of Gtilt to place the post-integrator zero
w_grid = logspace(-2, 4, 4000);
[mag_tilt, ~] = bode(Gtilt, w_grid);
mag_tilt = squeeze(mag_tilt);
[mag_peak, k_peak] = max(mag_tilt);
w_ip = w_grid(k_peak);            % frequency of magnitude peak
tau_ip = 1 / w_ip;                % post-integrator time constant

C_PI_post = (tau_ip*s + 1) / (tau_ip*s);
Gtilt_post = -C_PI_post * Gtilt;   % sign-absorbed plant to control

fprintf('  |Gtilt| peak:  %.3f  at  w_ip = %.2f rad/s\n', mag_peak, w_ip);
fprintf('  Post-integrator:  C_PI_post(s) = (%.4f*s + 1)/(%.4f*s)\n', ...
        tau_ip, tau_ip);
fprintf('  New plant Gtilt_post = -C_PI_post * Gtilt\n');
fprintf('  RHP poles of Gtilt_post: %d\n', ...
        sum(real(pole(minreal(Gtilt_post))) > 0));

% Bode comparison: Gtilt vs Gtilt_post
fig = figure(300);
bode(Gtilt, Gtilt_post, w_grid); grid on
legend('G_{tilt}(s)', 'G_{tilt,post}(s) = -C_{PI,post} G_{tilt}', ...
       'Location', 'best');
title('Task 2: Bode — original vs. post-integrated tilt plant');
saveas(fig, fullfile(IMG_DIR, 'regbot_task2_bode_post.png'));

% Nyquist comparison: confirms CCW encirclement after the post-integrator
fig = figure(301);
plot_nyquist_critical(Gtilt_post, 'Gtilt_{post} (with post-integrator)');
saveas(fig, fullfile(IMG_DIR, 'regbot_task2_nyquist_post.png'));

% --- (b) Outer PI-Lead design on Gtilt_post
%
% LEAD IMPLEMENTATION NOTE (gyro shortcut):
%   Standard Lead is (tau_d*s + 1)/(alpha*tau_d*s + 1).  The low-pass pole
%   (alpha) only exists to tame noise when we numerically differentiate
%   theta to get dtheta/dt.  REGBOT has a gyro that measures dtheta/dt
%   directly, so in Simulink the Lead reduces to the simple sum
%
%       Lead_output = tau_d * gyro + theta
%
%   i.e. an IDEAL Lead  (tau_d*s + 1)  with no filter pole (alpha = 0).
%   Design consequences:
%       phi_Lead(wc) = atan(wc*tau_d)          (NOT  asin((1-a)/(1+a)))
%       |Lead|(wc)   = 1/cos(phi_Lead) = sec(phi_Lead)
%   Solving:
%       tau_d = tan(phi_Lead) / wc
%   There is no alpha to pick — the gyro eliminates that degree of freedom.

% Specifications (tune these):
wc_tilt   = 15;      % target crossover [rad/s]  (tune: start small, push up)
gamma_M   = 60;      % desired phase margin [deg]
Ni_tilt   = 3;       % PI zero placed Ni times below wc
tau_i_tilt = Ni_tilt / wc_tilt;
C_PI_tilt  = (tau_i_tilt*s + 1) / (tau_i_tilt*s);

% Phase balance at wc:  -180 + gamma_M = phi_G + phi_PI + phi_Lead
[~, ph_w] = bode(Gtilt_post, wc_tilt);
phi_G    = ph_w;                                   % [deg]
phi_PI   = -atand(1/Ni_tilt);                      % [deg]
phi_Lead = -180 + gamma_M - phi_G - phi_PI;        % [deg]

if phi_Lead <= 0
    fprintf('  NOTE: no Lead needed (phi_Lead = %.1f deg <= 0).\n', phi_Lead);
    tau_d  = 0;
    C_Lead = tf(1);
elseif phi_Lead >= 89
    warning(['Gyro Lead requires %.1f deg — approaches singular tan().\n' ...
             'Lower wc_tilt or cascade two Lead blocks.'], phi_Lead);
    tau_d  = NaN;
    C_Lead = tf(1);
else
    % Gyro-based ideal Lead:  (tau_d*s + 1)   — no alpha pole
    tau_d  = tand(phi_Lead) / wc_tilt;
    C_Lead = tau_d*s + 1;
end

% Kp from the crossover magnitude condition:  |C_PI*C_Lead*Gtilt_post|(wc) = 1/Kp
[magL, ~] = bode(C_PI_tilt * C_Lead * Gtilt_post, wc_tilt);
Kp_tilt = 1 / magL;

C_tilt_total = Kp_tilt * C_PI_tilt * C_Lead;
L_tilt       = C_tilt_total * Gtilt_post;
[GM_t, PM_t, ~, wc_ach] = margin(L_tilt);

fprintf('  Design at wc = %.1f rad/s, gamma_M target = %.0f deg\n', ...
        wc_tilt, gamma_M);
fprintf('  PI:   tau_i = %.4f s   (Ni = %d)\n', tau_i_tilt, Ni_tilt);
fprintf('  Lead: tau_d = %.4f s   (phi_Lead = %.1f deg, gyro-based)\n', ...
        tau_d, phi_Lead);
fprintf('        -> Simulink implementation:  tau_d * gyro + theta\n');
fprintf('  Kp = %.3f\n', Kp_tilt);
fprintf('  Achieved:  wc = %.1f rad/s, PM = %.1f deg, GM = %.1f dB\n', ...
        wc_ach, PM_t, 20*log10(GM_t));

% Bode of the full open loop with margins marked
fig = figure(302);
margin(L_tilt); grid on
title('Task 2: Open loop  L = K_P C_{PI} C_{Lead} G_{tilt,post}');
saveas(fig, fullfile(IMG_DIR, 'regbot_task2_loop_bode.png'));

% Closed-loop step response of the balance loop (tilt reference tracking)
fig = figure(303);
step(feedback(L_tilt, 1), 2); grid on
title('Task 2: Closed-loop step response (tilt reference -> tilt angle)');
saveas(fig, fullfile(IMG_DIR, 'regbot_task2_step.png'));

% Simulink handoff — assign to base workspace variables used by the model.
%   NOTE: rename to match your Simulink blocks after you add the tilt
%   controller subsystem.  The Lead is implemented as  tau_d*gyro + theta,
%   so only tau_d is needed — there is no alpha parameter.
Kptilt  = Kp_tilt;        % outer Kp
titilt  = tau_i_tilt;     % outer tau_i  (PI zero)
tdtilt  = tau_d;          % Lead tau_d   (multiplied by the gyro signal)
tipost  = tau_ip;         % post-integrator tau_i

%% 9. TASK 3 — Velocity controller (outer loop) ===========================
% Run this section AFTER the tilt loop has been added to Simulink and the
% robot balances in simulation. Then:
%   - Linearise from theta_ref to the wheel_vel_filter output with the
%     tilt loop closed.  This is the new plant Gvel_outer(s).
%   - Design a PI (or PI-Lead) on Gvel_outer using the standard flow.
%
% Placeholder — uncomment when Simulink model contains the tilt controller:
%
%   Gvel_outer = identify_tf(model, '/theta_ref_input', '/wheel_vel_filter');
%   print_plant('Gvel_outer (theta_ref -> wheel velocity)', Gvel_outer);
%
%   wc_vel = 3;             % slower than the inner tilt loop (wc_tilt / ~5)
%   Ni_vel = 3;
%   tau_i_vel = Ni_vel / wc_vel;
%   C_PI_vel = (tau_i_vel*s + 1) / (tau_i_vel*s);
%   [magV, ~] = bode(C_PI_vel * Gvel_outer, wc_vel);
%   Kp_vel = 1 / magV;
%
%   L_vel = Kp_vel * C_PI_vel * Gvel_outer;
%   figure; margin(L_vel); grid on; title('Task 3: velocity open loop');
%   figure; step(feedback(L_vel, 1), 5); title('Task 3: closed-loop step');
%
%   Kpvel = Kp_vel; tivel = tau_i_vel;   % -> Simulink

%% 10. TASK 4 — Position controller (outermost loop) ======================
% Run this section AFTER the velocity loop is closed in Simulink and the
% robot tracks a commanded speed while balancing.
%   - Linearise from v_ref to x_position with the velocity loop closed.
%   - Usually a simple P-controller is enough (the velocity loop already
%     contains an integrator from the tilt loop's post-integrator).
%
% Placeholder — uncomment when Simulink model contains the velocity controller:
%
%   Gpos_outer = identify_tf(model, '/v_ref_input', '/x_position');
%   print_plant('Gpos_outer (v_ref -> x position)', Gpos_outer);
%
%   wc_pos = 0.8;           % slower than velocity loop (~wc_vel / 3)
%   Kp_pos = 1 / abs(evalfr(Gpos_outer, 1j*wc_pos));
%
%   L_pos = Kp_pos * Gpos_outer;
%   figure; margin(L_pos); grid on; title('Task 4: position open loop');
%   figure; step(feedback(L_pos, 1), 10); title('Task 4: closed-loop step');
%
%   Kppos = Kp_pos;          % -> Simulink

fprintf('\nAll tasks processed. Ready for Simulink simulation.\n');


%% 11. Local helper functions =============================================

function G = identify_tf(model, in_block, out_block)
    % Linearize a Simulink model between two block signals and return the TF.
    %   model     : model name, e.g. 'regbot_1mg'
    %   in_block  : path to input block, e.g. '/Limit9v'
    %   out_block : path to output block, e.g. '/wheel_vel_filter'
    io(1) = linio(strcat(model, in_block),  1, 'openinput');
    io(2) = linio(strcat(model, out_block), 1, 'openoutput');
    setlinio(model, io);
    sys = linearize(model, io, 0);
    [num, den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
    G = minreal(tf(num, den));
end

function print_plant(name, G)
    % Print plant summary: poles, zeros, DC gain, and RHP pole count.
    fprintf('\n--- %s ---\n', name);
    fprintf('Poles:\n'); disp(pole(G));
    fprintf('Zeros:\n'); disp(zero(G));
    fprintf('DC gain:    %.4e\n', dcgain(G));
    fprintf('RHP poles:  %d\n',   sum(real(pole(G)) > 0));
end

function plot_pz_stability(G, ttl)
    % Pole-zero map with shaded LHP (green = stable) and RHP (red = unstable).
    % RHP poles are highlighted with an orange ring.
    clf
    p = pole(G); z = zero(G);

    % Auto axis scale from pole/zero spread
    all_pts = [p; z];
    if isempty(all_pts)
        lim = 10;
    else
        lim = max(max(abs(real(all_pts))), max(abs(imag(all_pts)))) * 1.2 + 1;
    end

    hold on
    % Shaded regions
    patch([0 -lim -lim 0], [-lim -lim lim lim], [0.85 1 0.85], ...
          'EdgeColor','none','FaceAlpha',0.5);   % LHP green
    patch([0  lim  lim 0], [-lim -lim lim lim], [1 0.85 0.85], ...
          'EdgeColor','none','FaceAlpha',0.5);   % RHP red

    % Axes
    plot([-lim lim], [0 0], 'k',   'LineWidth', 0.5);
    plot([0 0], [-lim lim], 'k',   'LineWidth', 1.5);

    % Poles (red X) and zeros (blue O)
    plot(real(p), imag(p), 'rx', 'MarkerSize', 14, 'LineWidth', 2.5);
    plot(real(z), imag(z), 'bo', 'MarkerSize', 12, 'LineWidth', 2);

    % Highlight RHP poles with orange ring
    rhp = p(real(p) > 0);
    if ~isempty(rhp)
        plot(real(rhp), imag(rhp), 'o', 'MarkerSize', 20, ...
             'MarkerEdgeColor', [1 0.6 0], 'LineWidth', 2.5);
    end

    xlim([-lim lim]); ylim([-lim lim]);
    axis equal; grid on
    xlabel('Real axis'); ylabel('Imaginary axis');
    title(sprintf('Pole-zero map: %s', ttl));

    % Region annotations
    text(-lim*0.6, lim*0.85, 'LHP (stable)', ...
         'Color', [0 0.5 0], 'FontWeight', 'bold', 'FontSize', 11);
    text( lim*0.15, lim*0.85, 'RHP (unstable)', ...
         'Color', [0.7 0 0], 'FontWeight', 'bold', 'FontSize', 11);

    % Legend
    legend_items = {'Poles', 'Zeros'};
    if ~isempty(rhp), legend_items{end+1} = 'RHP poles (unstable)'; end
    legend(legend_items, 'Location', 'best');
    hold off
end

function plot_nyquist_critical(G, ttl)
    % Nyquist plot with the critical point (-1, 0) highlighted.
    % Shows the full curve (both w > 0 and w < 0) and marks RHP-pole count
    % so the CCW-encirclement requirement is easy to read off.
    clf
    P = sum(real(pole(G)) > 0);

    % Compute Nyquist data
    [re, im, w] = nyquist(G);
    re = squeeze(re); im = squeeze(im);

    hold on; grid on
    % The two branches: positive w (solid) and negative w (dashed, mirror)
    plot(re,  im,  'b-',  'LineWidth', 1.5);
    plot(re, -im,  'b--', 'LineWidth', 1.0);

    % Critical point (-1, 0) with a unit "forbidden" circle for context
    th = linspace(0, 2*pi, 200);
    plot(-1 + 0.05*cos(th), 0.05*sin(th), 'r-', 'LineWidth', 1);
    plot(-1, 0, 'r+', 'MarkerSize', 14, 'LineWidth', 2);

    % Origin axes
    xl = xlim; yl = ylim;
    plot(xl, [0 0], 'k', 'LineWidth', 0.5);
    plot([0 0], yl, 'k', 'LineWidth', 0.5);

    % Arrow showing direction of increasing w (pick a mid-sample)
    k = max(2, round(length(w)/3));
    quiver(re(k), im(k), re(k+1)-re(k), im(k+1)-im(k), 0, ...
           'Color', 'b', 'MaxHeadSize', 2, 'LineWidth', 1.2, ...
           'AutoScale', 'off');

    axis equal
    xlabel('Re\{G(j\omega)\}'); ylabel('Im\{G(j\omega)\}');
    title(sprintf('Nyquist plot: %s  (RHP poles P = %d)', ttl, P));
    legend({'\omega > 0','\omega < 0 (mirror)','critical point (-1,0)'}, ...
           'Location', 'best');
    hold off
end
