clear all;
close all;
clc;

%% Theorical Parameters
R = 27.4; % Ohms

K = 0.0404;  % N.m/A
b = K * 0.011/297; % N.m.s

% Parameters from simulation
L = 6.157915e-01; % Henry
J = 5.731284e-06; % kg.m^2

%% Estimated Parameters
R = 27.4; % Ohm

K = 0.031946; % V.s/rad
b = 8.780965e-06; % N.m.s/rad

L = 6.157915e-01; % H
J = 5.731284e-06; % kg.m^2

%% Read Behavior
S = load('motor_identification.mat');

t_meas = S.t(:);
omega_meas = S.omega(:);
rpm_meas = omega_meas * 60/(2*pi);

%% Plant parameters
gearRatio = 1/472.727;
rad2rpm = 60 / (2*pi);
gain = rad2rpm * gearRatio;

TARGET_SPEED_RPM = 6.0;
TARGET_SPEED_RAD = TARGET_SPEED_RPM * rad2rpm^-1;
SUPPLY_VOLT      = 12.0;
PWM_PER_RPM_EST  = 40.0;
PWM_STEP         = min(max(round(TARGET_SPEED_RPM * PWM_PER_RPM_EST), 1), 255);

Va = (PWM_STEP / 255) * SUPPLY_VOLT;

%% Transfer Function
s = tf('s');
G = (K) / ((J*s + b)*(L*s + R) + K^2);

% Step Response on the measured time window
if numel(t_meas) < 2
	error('motor_identification.mat does not contain enough time samples.');
end

t_meas = t_meas - t_meas(1);
dt_meas = median(diff(t_meas));
if ~isfinite(dt_meas) || dt_meas <= 0
	error('Invalid time vector loaded from motor_identification.mat.');
end

t_sim = (0:dt_meas:t_meas(end)).';
if numel(t_sim) < 2
	t_sim = linspace(0, t_meas(end), 2).';
end

[y_model, t_model] = step(Va * gain* G, t_sim);
y_model = squeeze(y_model);
rpm_model = interp1(t_model, y_model, t_meas, 'linear', 'extrap');

figure;
plot(t_meas, rpm_meas, 'LineWidth', 2);
hold on;
plot(t_meas, rpm_model, '--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (rpm)');
title('ESP32 Read Behavior vs Transfer Function');
legend('ESP32 read behavior', 'Transfer function');
grid on;

%% PID Design
close all; clear all; clc;

% --- 1. System Parameters ---
R = 27.4; K = 0.031946; b = 8.780965e-06;
L = 6.157915e-01; J = 5.731284e-06; gearRatio = 1/472.727;

% --- 2. Calculate Natural Plant Response ---
s_tf = tf('s');
G_num = gearRatio * K;
G_den = (J*s_tf + b)*(L*s_tf + R) + K^2;
G = G_num / G_den;

% Get natural settling time (2% criterion)
info = stepinfo(G);
ts_natural = info.SettlingTime;

% --- 3. Set Desired Performance ---
ts_desired = ts_natural * 0.8; 

zd = -log(0.01)/sqrt(pi^2+(log(0.01))^2); % Damping for 1% overshoot
wd = 4 / (zd * ts_desired); 

fprintf('Natural Settling Time: %.4f s\n', ts_natural);
fprintf('Target Settling Time:  %.4f s\n\n', ts_desired);

% --- 4. Symbolic Pole Placement ---
syms s Kc Ti Td;

% Symbolic Plant
numG = G_num;
denG = (J*s + b)*(L*s + R) + K^2;

% Symbolic PID
numC = Kc*Td*s^2 + Kc*s + (Kc/Ti);
denC = s;

% Characteristic Equation
charEq = expand(denC*denG + numC*numG);
actual_coeffs = coeffs(charEq, s, 'All');
norm_actual_coeffs = actual_coeffs / actual_coeffs(1);

% Desired Polynomial (3rd order)
p_fast = 10 * (zd*wd); 
desired_poly = (s^2 + 2*zd*wd*s + wd^2) * (s + p_fast);
desired_coeffs = coeffs(expand(desired_poly), s, 'All');

% --- 5. Solve ---
eqs = [norm_actual_coeffs(2) == desired_coeffs(2), ...
       norm_actual_coeffs(3) == desired_coeffs(3), ...
       norm_actual_coeffs(4) == desired_coeffs(4)];

sol = solve(eqs, [Kc, Ti, Td]);

% --- 6. Results ---
% Pick the first real solution
Kc_val = double(sol.Kc(1));
Ti_val = double(sol.Ti(1));
Td_val = double(sol.Td(1));

fprintf('--- Calculated Gains ---\n');
fprintf('Kc: %f\n', Kc_val);
fprintf('Ti: %f\n', Ti_val);
fprintf('Td: %f\n\n', Td_val);

% --- 7. Verification ---
C = (Kc_val*Td_val*s_tf^2 + Kc_val*s_tf + (Kc_val/Ti_val))/s_tf;
T = feedback(C*G, 1);
figure;
step(T);
title(['Closed Loop Step Response (ts_{target} = ', num2str(ts_desired), 's)']);
grid on;