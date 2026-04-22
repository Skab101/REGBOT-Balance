%% =======================================================================
%  regbot_mg.m — Workspace loader for the REGBOT Balance Assignment
%  =======================================================================
%
%  Populates the MATLAB base workspace with everything regbot_1mg.slx
%  needs to compile and simulate:
%
%    1.  Physical parameters of the REGBOT (motor, vehicle, masses).
%    2.  Controller gains for every loop that is currently designed.
%
%  This file is intentionally short and fast. It does NOT linearise the
%  model, does NOT generate plots, and does NOT do any control-design
%  math. Run it before opening the Simulink model, or let the model's
%  PreLoad/InitFcn call it automatically.
%
%  ---------------------------------------------------------------------
%  WHERE EACH GAIN COMES FROM
%  ---------------------------------------------------------------------
%  When you want to tune or redesign a controller (different wc, gamma_M,
%  Ni, ...) run the matching design script in design/. Each script
%  finishes by printing a copy-pasteable block that you drop into the
%  "Committed controller gains" section below.
%
%    design/design_task1_wheel.m     ->  Kpwv,  tiwv,   Kffwv
%    design/design_task2_balance.m   ->  Kptilt, titilt, tdtilt, tipost
%    design/design_task3_velocity.m  ->  (future)
%    design/design_task4_position.m  ->  (future)
%  =======================================================================

% Add this folder + lib/ to the path so helpers and design scripts
% resolve even when regbot_mg is called from elsewhere (e.g. Simulink's
% PreLoadFcn).
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, 'lib'));


%% ----------------------------- Physical parameters --------------------
% Motor — electrical (two motors in parallel)
RA   = 3.3/2;            % armature resistance          [ohm]
LA   = 6.6e-3/2;         % armature inductance          [H]
Kemf = 0.0105;           % EMF / torque constant        [V s/rad]
Km   = Kemf;             % torque constant              [N m/A]

% Motor — mechanical (two motors)
JA   = 1.3e-6 * 2;       % rotor inertia                [kg m^2]
BA   = 3e-6   * 2;       % rotor friction               [N m s/rad]
NG   = 9.69;             % gear ratio

% Vehicle
WR   = 0.03;             % wheel radius                 [m]
Bw   = 0.155;            % distance between wheels      [m]

% Masses and geometry
mmotor    = 0.193;                        % motor + gear                     [kg]
mframe    = 0.32;                         % frame + base PCB                 [kg]
mtopextra = 0.97 - mframe - mmotor;       % top mass (battery + charger)     [kg]
mpdist    = 0.10;                         % distance to lid                  [m]
pushDist  = 0.10;                         % disturbance application (Z)      [m]

% Simulation settings
startAngle = 10;         % initial tilt at t = 0        [deg]
twvlp      = 0.005;      % wheel-velocity filter tau    [s]


%% ----------------------------- Committed controller gains -------------
% Update these whenever a design script produces a new block you want to
% keep. Leave gains at 0 / sensible placeholders for loops not yet closed.

% --- Task 1: Wheel-speed PI (source: design/design_task1_wheel.m) -----
% Day 5 v2 on-floor training-wheels plant Gvel = 2.198/(s+5.985).
% wc = 30 rad/s, gamma_M spec >= 60 deg, Ni = 3 -> Kp = 13.20, tau_i = 0.10.
% Achieved wc = 30.00 rad/s, PM = 82.85 deg, GM = Inf dB.
Kpwv   = 13.2037;
tiwv   = 0.1000;
Kffwv  = 0;

% --- Task 2: Balance controller (source: design/design_task2_balance.m)
% Day 5 on-floor redesign: faster inner loop shifts Gtilt magnitude peak
% from 5.95 to 8.03 rad/s, drops the required Lead boost from +63.8 to
% +33.5 deg, and reduces tau_d accordingly. Same specs (wc = 15 rad/s,
% gamma_M = 60 deg, Ni = 3). Achieved wc = 15.00, PM = 60.00, GM = -5.58
% dB (lower bound, normal for P=1). Linear-model IC settling 1.34 s.
Kptilt = 1.1999;
titilt = 0.2000;
tdtilt = 0.0442;
tipost = 0.1245;

% --- Task 3: Velocity outer loop (source: design_task3_velocity.m) -----
% wc_vel = 1 rad/s (RHP-zero z/5 rule; zero still at +8.51 rad/s
% after the Day 5 redesign). Achieved wc = 1.00 rad/s, PM = 68.98 deg,
% GM = 5.84 dB. No Lead needed.
Kpvel  = 0.1581;
tivel  = 3.0000;

% --- Task 4: Position outermost loop (source: design_task4_position.m)
% Pure P on Gpos,outer. wc_pos = 0.6 rad/s (1.67x below wc_vel = 1 rad/s);
% achieves wc = 0.60 rad/s, PM = 59 deg, GM = 23.2 dB. The design script
% wanted a tiny Lead (tdpos = 0.027 s) to push PM from 59 to 60, but a
% pure-Lead TF is improper and Simulink rejects it. The 1 deg of PM isn't
% worth a proper-Lead block with a fast filter pole — drop it.
% Plant already has a free integrator (v -> x) so no I term needed.
% 2 m step response: peak velocity 0.80 m/s (spec >= 0.7 m/s).


Kppos  = 0.5335;
tdpos  = 0.0273;
