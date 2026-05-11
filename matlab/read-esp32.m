%% =========================================================
%  ESP32 DC MOTOR IDENTIFICATION + TF FITTING
%  MATLAB R2025b
% ==========================================================

clear;
clc;
close all;

%% ---------------------------------------------------------
% SERIAL CONFIG
% ---------------------------------------------------------

PORT = "/dev/ttyUSB0";
BAUD = 115200;

s = serialport(PORT, BAUD);

configureTerminator(s, "LF");
flush(s);

pause(2);

disp('ESP32 Connected');

%% ---------------------------------------------------------
% USER PARAMETERS
% ---------------------------------------------------------

TARGET_SPEED_RPM = 6.0;   % desired speed setpoint shown in plots
TEST_TIME        = 3;     % seconds
SUPPLY_VOLT      = 12.0;  % motor supply voltage

TELEMETRY_MS     = 10;    % Sampling interval in milliseconds (must match ESP32)

PWM_PER_RPM_EST  = 40.0; % Open-loop feedforward guess used to convert speed setpoint into PWM.
PWM_STEP         = min(max(round(TARGET_SPEED_RPM * PWM_PER_RPM_EST), 1), 255);
disp(' ');
disp('USER PARAMETERS');
fprintf('Target speed setpoint = %.2f rpm\n', TARGET_SPEED_RPM);
fprintf('Initial feedforward PWM command = %d\n', PWM_STEP);

%% ---------------------------------------------------------
% EXPERIMENTAL MEASUREMENT
% ---------------------------------------------------------

% RESET MOTOR
writeline(s, "STOP");
pause(1);

writeline(s, "RESET");
pause(0.5);

% ---------------------------------------------------------
% START STEP RESPONSE TEST
% ---------------------------------------------------------

disp(' ');
disp('STARTING SPEED SETPOINT TEST');

writeline(s, sprintf("S %d", PWM_STEP));
pause(0.1);

writeline(s, "GO");

% ---------------------------------------------------------
% DATA STORAGE
% ---------------------------------------------------------

t       = [];
pwm     = [];
dirc    = [];
pulses  = [];
rpm     = [];
omega   = [];
current = [];
step_applied_idx = [];  % Track when STEP_APPLIED marker is received

tic;

% ---------------------------------------------------------
% ACQUIRE TELEMETRY
% ---------------------------------------------------------

while toc < TEST_TIME

    if s.NumBytesAvailable > 0

        line = readline(s);
        line = strtrim(line);

        % Check for timing marker
        if contains(line, 'STEP_APPLIED')
            if isempty(step_applied_idx) && ~isempty(t)
                step_applied_idx = length(t);  % Record index when step was applied
                disp(sprintf('[MATLAB] STEP_APPLIED marker received at index %d (t=%.3f s)', step_applied_idx, t(step_applied_idx)));
            end
            continue;
        end

        vals = split(line, ",");

        if numel(vals) == 7

            try
                t(end+1)       = double(vals(1))/1000;
                pwm(end+1)     = double(vals(2));
                dirc(end+1)    = double(vals(3));
                pulses(end+1)  = double(vals(4));
                rpm(end+1)     = double(vals(5));
                omega(end+1)   = double(vals(6));
                current(end+1) = double(vals(7));
            catch
            end

        end
    end
end

% ---------------------------------------------------------
% STOP MOTOR
% ---------------------------------------------------------

writeline(s, "STOP");

disp(' ');
disp('TEST COMPLETED');

% ---------------------------------------------------------
% CLEAN TIME VECTOR & CROP TO TEST WINDOW
% ---------------------------------------------------------

if isempty(t)
    error('No telemetry data acquired. Check serial link and test duration.');
end

% Define reference at acquisition start first, then crop using STEP_APPLIED
t = t - t(1);

if ~isempty(step_applied_idx) && step_applied_idx > 0 && step_applied_idx <= length(t)
    t_step = t(step_applied_idx);
    disp(sprintf('[MATLAB] STEP_APPLIED marker detected at t=%.4f s (acquisition reference).', t_step));
else
    t_step = 0;
    disp('[MATLAB] No STEP_APPLIED marker detected. Falling back to acquisition start as crop reference.');
end

% Keep only samples in [STEP_APPLIED, STEP_APPLIED + TEST_TIME]
window_start = t_step;
window_end   = t_step + TEST_TIME;
valid_idx = (t >= window_start) & (t <= window_end);
t       = t(valid_idx);
pwm     = pwm(valid_idx);
dirc    = dirc(valid_idx);
pulses  = pulses(valid_idx);
rpm     = rpm(valid_idx);
omega   = omega(valid_idx);
current = current(valid_idx);

if numel(t) < 2
    error('Not enough samples left after cropping to [STEP_APPLIED, STEP_APPLIED + TEST_TIME]. Increase TEST_TIME or telemetry rate.');
end

% Re-base time so t=0 is exactly STEP_APPLIED
t = t - window_start;

% Convert speed to rpm for plotting and setpoint comparison
rpm_meas = omega * 60.0 / (2.0 * pi);
rpm_setpoint = TARGET_SPEED_RPM * ones(size(t));

% Plot vectors now already cropped to acquisition window [0, TEST_TIME]
t_plot = t;
rpm_meas_plot = rpm_meas;
rpm_setpoint_plot = rpm_setpoint;
omega_plot = omega;
current_plot = current;

% ---------------------------------------------------------
% PLOT MEASURED RESPONSE VS SPEED SETPOINT
% ---------------------------------------------------------

figure;
plot(t_plot, rpm_meas_plot, 'LineWidth', 2);
hold on;
plot(t_plot, rpm_setpoint_plot, '--', 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('Speed [rpm]');
title('Measured Speed Response vs Target Setpoint');
legend('Measured speed', 'Target speed');

% ---------------------------------------------------------
% ESTIMATE STEADY STATE VALUE
% ---------------------------------------------------------

omega_ss = mean(omega(end-20:end));

% Check if system has settled (within 2% of final value for last second)
if TEST_TIME >= 1.0
    last_second_idx = find(t >= (TEST_TIME - 1.0));
    omega_ss_check = omega(last_second_idx);
    omega_variation = (max(omega_ss_check) - min(omega_ss_check)) / max(omega_ss, 0.1);
    if omega_variation > 0.05  % More than 5% variation
        warning('Motor may not have fully settled. Variation = %.2f%%', omega_variation*100);
    end
else
    warning('Test duration too short (%.1f s). Recommend at least 2-3 seconds for motor to settle.', TEST_TIME);
end

% Check for quantization artifacts in omega (oscillating between 1-2 values)
omega_unique = unique(round(omega, 4));
if length(omega_unique) <= 3 && std(omega) > 0.01 * mean(omega)
    warning(['Omega data shows significant quantization (only %d unique values detected).\n', ...
        'This indicates ENCODER RESOLUTION LIMIT at your sampling rate.\n', ...
        'Solutions: (1) Increase sampling rate to 5-10ms in ESP32 (reduce TELEMETRY_MS),\n', ...
        '           (2) Increase motor speed (higher PWM) to move more encoder pulses/window,\n', ...
        '           (3) Apply low-pass filter to smooth omega estimate.'], length(omega_unique));
    
    % Apply simple moving average filter to smooth omega
    omega_filtered = movmean(omega, 5, 'omitnan');  % 5-point moving average
    disp('[MATLAB] Applied 5-point moving average filter to omega.');
    omega = omega_filtered;
end

disp("Steady-state omega:");
disp(omega_ss);

% ---------------------------------------------------------
% ESTIMATE RISE + SETTLING TIME
% ---------------------------------------------------------

info = stepinfo(omega, t);

disp(info);

% ---------------------------------------------------------
% ELECTRICAL INPUT
% ---------------------------------------------------------

Va = (PWM_STEP / 255) * SUPPLY_VOLT;

fprintf('\nEstimated input voltage = %.3f V\n', Va);

rpm_ss_meas = omega_ss * 60.0 / (2.0 * pi);
fprintf('Measured steady-state speed = %.3f rpm\n', rpm_ss_meas);

%% =========================================================
% MOTOR PARAMETER IDENTIFICATION
% =========================================================

disp(' ');
disp('STARTING MOTOR PARAMETER IDENTIFICATION');

% ---------------------------------------------------------
% FIXED PARAMETER
% ---------------------------------------------------------

R0 = 27.4;     % Measured armature resistance [Ohm]
gearRatio = 472.7272; % Encoder gearbox ratio
gain = 60 / (2*pi*gearRatio); % Convert motor rad/s to output rpm after gearbox

% ---------------------------------------------------------
% INITIAL GUESS
%
% x = [L J K b]
% ---------------------------------------------------------

x0 = [
    1e-3      % L [H]
    1e-5      % J [kg.m^2]
    0.04      % K [V.s/rad]
    1e-5      % b [N.m.s/rad]
];

% ---------------------------------------------------------
% PARAMETER BOUNDS
% ---------------------------------------------------------

lb = [
    1e-6      % L
    1e-8      % J
    1e-4      % K
    1e-8      % b
];

ub = [
    1         % L
    1e-2      % J
    1         % K
    1         % b
];

% ---------------------------------------------------------
% COST FUNCTION
% ---------------------------------------------------------

costFun = @(x) motorCost(x, R0, t, omega, Va);

% ---------------------------------------------------------
% OPTIMIZATION OPTIONS
% ---------------------------------------------------------

opts = optimoptions( ...
    'fmincon', ...
    'Display', 'iter', ...
    'MaxFunctionEvaluations', 5000, ...
    'MaxIterations', 1000);

% ---------------------------------------------------------
% RUN OPTIMIZATION
% ---------------------------------------------------------

disp('Running optimization...');

xopt = fmincon( ...
    costFun, ...
    x0, ...
    [], [], [], [], ...
    lb, ub, ...
    [], ...
    opts);

% ---------------------------------------------------------
% EXTRACT PARAMETERS
% ---------------------------------------------------------

L = xopt(1);
J = xopt(2);
K = xopt(3);
b = xopt(4);

R = R0;

% ---------------------------------------------------------
% BUILD TRANSFER FUNCTION
% ---------------------------------------------------------

s = tf('s');

G = K / ((J*s + b)*(L*s + R) + K^2);

% ---------------------------------------------------------
% SIMULATE MODEL
% ---------------------------------------------------------

% Use a uniform simulation grid because `step` requires evenly spaced times.
% The measurement timestamps come from serial reception and are not uniform.
t_meas = t(:);
if numel(t_meas) < 2
    error('Not enough time samples to simulate the model.');
end

dt_meas = median(diff(t_meas));
if ~isfinite(dt_meas) || dt_meas <= 0
    error('Invalid time vector spacing detected.');
end

t_sim = (0:dt_meas:t_meas(end)).';
if numel(t_sim) < 2
    t_sim = linspace(0, t_meas(end), 2).';
end

[y_model, t_model] = step(Va * gain * G, t_sim);

y_model = squeeze(y_model);

rpm_model = interp1(t_model, y_model, t_meas, 'linear', 'extrap');

rpm_meas = omega(:) * 60/(2*pi);
rpm_model = rpm_model(:);

% ---------------------------------------------------------
% FIT QUALITY
% ---------------------------------------------------------

rmse = sqrt(mean((rpm_model - rpm_meas).^2));

fit_percent = 100 * ...
    (1 - norm(rpm_model - rpm_meas) / max(norm(rpm_meas - mean(rpm_meas)), eps));

%% ---------------------------------------------------------
% PLOT COMPARISON
% ---------------------------------------------------------

figure;

plot(t_meas, rpm_meas, 'LineWidth', 2);
hold on;

plot(t_meas, rpm_model, '--', 'LineWidth', 2);

plot(t_meas, TARGET_SPEED_RPM * ones(size(t_meas)), ':', 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('Speed [rpm]');

title('Measured vs Identified Motor Model');

legend( ...
    'Measured', ...
    'Identified Model', ...
    'Target');

% ---------------------------------------------------------
% DISPLAY RESULTS
% ---------------------------------------------------------

disp(' ');
disp(' IDENTIFIED PARAMETERS ');

fprintf('R = %.6f Ohm\n', R);
fprintf('K = %.6f V.s/rad\n', K);
fprintf('b = %.6e N.m.s/rad\n', b);
fprintf('L = %.6e H\n', L);
fprintf('J = %.6e kg.m^2\n', J);
fprintf('Gear ratio used in model = %.3f\n', gearRatio);

fprintf('\nRMSE = %.6f rpm\n', rmse);
fprintf('Fit = %.2f %%\n', fit_percent);

% ---------------------------------------------------------
% TIME CONSTANTS
% ---------------------------------------------------------

tau_elec = L / R;

tau_mech = J / b;

fprintf('\nElectrical tau = %.6f s\n', tau_elec);
fprintf('Mechanical tau = %.6f s\n', tau_mech);

% ---------------------------------------------------------
% SHOW TF
% ---------------------------------------------------------

disp(' ');
disp('Transfer Function:');

G

%% ---------------------------------------------------------
% SAVE DATA
% ---------------------------------------------------------

save('motor_identification.mat');

disp("Data saved.");
