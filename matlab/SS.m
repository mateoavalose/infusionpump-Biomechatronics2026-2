clear all;
close all;
clc;

%% Motor Parameters (Estimated from Identification)
R = 27.4;                    % Armature resistance [Ohm]
K = 0.031946;                % Motor constant [V.s/rad]
b = 8.780965e-06;            % Viscous damping [N.m.s/rad]
L = 6.157915e-01;            % Armature inductance [H]
J = 5.731284e-06;            % Moment of inertia [kg.m^2]

gearRatio = 1/472.727;       % Gear reduction (motor to output)
rad2rpm = 60 / (2*pi);       % Conversion factor

%% Design Specifications
TARGET_SPEED_RPM = 1.0;
TARGET_SPEED_RAD = TARGET_SPEED_RPM / rad2rpm;

%% Step 1: Create Transfer Function Model
s_tf = tf('s');
G_num = gearRatio * K;
G_den = (J*s_tf + b)*(L*s_tf + R) + K^2;
G = G_num / G_den;

% Get natural response characteristics
info = stepinfo(G);
ts_natural = info.SettlingTime;

fprintf('=== Plant Characteristics ===\n');
fprintf('Natural settling time: %.4f s\n\n', ts_natural);

%% Step 2: Convert to State Space

A_plant = [
    -b/J,   K/J;
    -K/L,  -R/L
];

B_plant = [
    0;
    1/L
];

C_plant = [gearRatio, 0];

sys = ss(A_plant, B_plant, C_plant, 0);

fprintf('=== Plant (2-state) ===\n');
fprintf('States: [ω, i]\n');
fprintf('A_plant = \n');
disp(A_plant);
fprintf('B_plant = \n');
disp(B_plant);
fprintf('C_plant = \n');
disp(C_plant);

%% Step 3: Augment with Integrator for Servo Control
% Extended state: x_aug = [ω, i, e_int]
% where e_int = integral(ω_ref - ω)
%   de_int/dt = ω_ref - ω = -ω (error, with constant reference handled by feedforward)

A_aug = [
    A_plant,  [0; 0];
    -C_plant, 0
];

B_aug = [
    B_plant;
    0
];

C_aug = [C_plant, 0];  % Output still speed

E_aug = [0; 0; 1];

fprintf('\n=== Augmented System (3-state with Integrator) ===\n');
fprintf('States: [ω, i, e_int]\n');
fprintf('A_aug = \n');
disp(A_aug);
fprintf('B_aug = \n');
disp(B_aug);
fprintf('C_aug = \n');
disp(C_aug);

fprintf('Plant order: %d\n', size(A_plant,1));
fprintf('Augmented (with integrator): %d\n\n', size(A_aug,1));

%% Step 4: Desired Performance via Pole Placement
% Target settling time: reasonable for motor control
ts_target = ts_natural * 0.8;

% Damping ratio for minimal overshoot
zeta = -log(1/100)/sqrt(pi^2 + (log(1/100))^2);
omega_n = 4 / (zeta * ts_target);

% Closed-loop poles (2nd-order dominant + integrator)
% Place poles well into the left half-plane for stability margin
p_dominant = -zeta*omega_n + 1i*omega_n*sqrt(1-zeta^2);
p_dominant_conj = conj(p_dominant);
p_integrator = -omega_n * zeta * 10;  % Integrator pole, faster than dominant

desired_poles = [p_dominant; p_dominant_conj; p_integrator];

fprintf('=== Desired Closed-Loop Poles ===\n');
fprintf('Dominant (complex): %.4f ± j%.4f\n', real(p_dominant), imag(p_dominant));
fprintf('Integrator pole: %.4f\n', p_integrator);
fprintf('Natural frequency: %.4f rad/s\n', omega_n);
fprintf('Damping ratio: %.4f\n\n', zeta);

%% Step 5: Calculate State Feedback Gains via Pole Placement
try
    K_feedback = place(A_aug, B_aug, desired_poles);
    fprintf('=== State Feedback Gains (Pole Placement) ===\n');
    fprintf('Kω (angular velocity): %.6f\n', K_feedback(1));
    fprintf('Ki (current):          %.6f\n', K_feedback(2));
    fprintf('K_integral (error):    %.6f\n\n', K_feedback(3));
catch ME
    fprintf('ERROR in pole placement: %s\n', ME.message);
end

%% Plot Response
Af = A_aug-B_aug*K_feedback;

sys_f = ss(Af, E_aug, C_aug, 0);

figure
subplot(211)
step(TARGET_SPEED_RAD * sys)
title('Open Loop');
subplot(212)
step(TARGET_SPEED_RAD * sys_f)
title('Closed Loop');