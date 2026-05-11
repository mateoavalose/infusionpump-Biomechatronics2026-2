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
SUPPLY_VOLT      = 12.0;
PWM_PER_RPM_EST  = 40.0;
PWM_STEP         = min(max(round(TARGET_SPEED_RPM * PWM_PER_RPM_EST), 1), 255);

Va = (PWM_STEP / 255) * SUPPLY_VOLT;

%% Transfer Function
s = tf('s');
G = gain * (K) / ((J*s + b)*(L*s + R) + K^2);

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

[y_model, t_model] = step(Va * G, t_sim);
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