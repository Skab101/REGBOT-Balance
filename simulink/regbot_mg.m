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
%   8. Local helper functions
%
% USAGE
%   Just run the script. All plots are saved to the Obsidian vault folder
%   under regbot/Images/ automatically.

close all; clear;

%% 1. Setup
s = tf('s');
model = 'regbot_1mg';

% Obsidian Images folder — auto-resolved from script location
IMG_DIR = fullfile(fileparts(mfilename('fullpath')), ...
    '..','..','..','..','Obsidian','Courses', ...
    '34722 Linear Control Design 1','Exercises','Work','regbot','Images');
if ~exist(IMG_DIR,'dir'), mkdir(IMG_DIR); end
fprintf('Plots will be saved to: %s\n', IMG_DIR);

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

fprintf('\nDone. All plots saved to the Obsidian Images folder.\n');


%% 8. Local helper functions ==============================================

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
