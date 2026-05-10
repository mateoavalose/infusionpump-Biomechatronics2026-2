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

disp("Connected to ESP32");

%% ---------------------------------------------------------
% USER PARAMETERS
% ---------------------------------------------------------

PWM_STEP      = 255;
TEST_TIME     = 3;      % seconds (need ≥2s for motor to fully settle + capture dynamics)
SUPPLY_VOLT   = 12.0;    % motor supply voltage

% TELEMETRY CONFIG — must match ESP32 main.cpp
TELEMETRY_MS  = 10;     % Sampling interval in milliseconds

%% ---------------------------------------------------------
% RESET MOTOR
% ---------------------------------------------------------

writeline(s, "STOP");
pause(1);

writeline(s, "RESET");
pause(0.5);

% ---------------------------------------------------------
% START STEP RESPONSE TEST
% ---------------------------------------------------------

disp("Starting step test...");

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

disp("Test completed.");

% ---------------------------------------------------------
% CLEAN TIME VECTOR & ALIGN TO STEP
% ---------------------------------------------------------

t = t - t(1);  % Start from t=0 relative to first data point

% If STEP_APPLIED marker was received, re-align so t=0 is step application
if ~isempty(step_applied_idx) && step_applied_idx > 0 && step_applied_idx <= length(t)
    t_step = t(step_applied_idx);
    t = t - t_step;  % New t=0 is at step application
    disp(sprintf('[MATLAB] Time re-aligned: Step occurred at %.4f s after first sample', t_step));
    % Remove pre-step data (optional: keep it for diagnostics)
    % t(1:step_applied_idx-1) = [];
    % pwm(1:step_applied_idx-1) = [];
    % ... etc
else
    disp('[MATLAB] No STEP_APPLIED marker detected. Using first data point as t=0.');
end

% ---------------------------------------------------------
% PLOT RAW RESPONSE
% ---------------------------------------------------------

figure;
plot(t, omega, 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('\omega [rad/s]');
title('Measured Motor Step Response');

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
% ESTIMATE 63% TIME CONSTANT
% ---------------------------------------------------------

omega63 = 0.632 * omega_ss;

idx63 = find(omega >= omega63, 1);

tau_est = t(idx63);

fprintf('\nEstimated tau = %.4f s\n', tau_est);

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

%% ---------------------------------------------------------
% INITIAL PARAMETER ESTIMATES
% ---------------------------------------------------------

% Estimate R from initial transient (when omega ≈ 0, back-EMF ≈ 0)
% Use first ~50-100ms where di/dt is large but speed hasn't accelerated much
initial_window = min(10, length(current));  % First 10 samples (~200ms)
initial_idx = find(current(1:initial_window) > max(current(1:initial_window))*0.3, 1);
if isempty(initial_idx); initial_idx = 1; end
R0 = Va / mean(current(initial_idx:min(initial_idx+5, length(current))));

% K from steady state is solid
K0 = omega_ss / Va;

% Better J/b estimation: use initial acceleration
% At startup: dω/dt ≈ K*i_initial / J (since ω≈0 and b*ω ≈ 0)
accel_window = min(50, length(omega));
accel_initial = gradient(omega(1:accel_window)) / (TELEMETRY_MS/1000);
i_start = mean(current(1:min(10, length(current))));
J0 = max((K0 * i_start) / mean(accel_initial(accel_initial > 0)), 1e-6);

% Damping from steady-state equilibrium: K*i_ss = b*omega_ss + friction
% Estimate b from steady-state current and speed
i_ss = mean(current(end-20:end));
b0 = max(K0 * i_ss / max(omega_ss, 0.1), 0.001);

% Typical motor inductance 0.5-2 mH
L0 = 8e-4;

x0 = [L0 J0 K0 b0];

fprintf('\nInitial guesses:\n');
fprintf('R = %.6f Ohm\n', R0);
fprintf('L = %.6f H\n', L0);
fprintf('J = %.6e kg.m^2\n', J0);
fprintf('K = %.6f V.s/rad\n', K0);
fprintf('b = %.6e N.m.s/rad\n', b0);

%% ---------------------------------------------------------
% PARAMETER BOUNDS (Adjusted for realistic motor parameters)
%% ---------------------------------------------------------

lb = [1e-5   1e-7   0.01   1e-6];   % L, J, K, b lower bounds
ub = [10     1e-2   100    1];      % L, J, K, b upper bounds

fprintf('\nBounds:\n');
fprintf('L: [%.2e, %.2e] H\n', lb(1), ub(1));
fprintf('J: [%.2e, %.2e] kg.m^2\n', lb(2), ub(2));
fprintf('K: [%.4f, %.4f] V.s/rad\n', lb(3), ub(3));
fprintf('b: [%.2e, %.2e] N.m.s/rad\n', lb(4), ub(4));

% ---------------------------------------------------------
% COST FUNCTION
% ---------------------------------------------------------

costFun = @(x) motorCost(x, R0, t, omega, Va);

% ---------------------------------------------------------
% OPTIMIZATION
% ---------------------------------------------------------

disp("Running optimization...");

xopt = fmincon(costFun, x0, [], [], [], [], lb, ub);

L = xopt(1);
J = xopt(2);
K = xopt(3);
b = xopt(4);
R = R0;

% ---------------------------------------------------------
% BUILD TRANSFER FUNCTION
% ---------------------------------------------------------

s_tf = tf('s');

G = K / ((J*s_tf + b)*(L*s_tf + R) + K^2);

%% ---------------------------------------------------------
% SIMULATE MODEL
% ---------------------------------------------------------

[y_model, t_model] = step(Va * G, t(end));

% ---------------------------------------------------------
% COMPARE MODEL VS REAL
% ---------------------------------------------------------

figure;

plot(t, omega, 'LineWidth', 2);
hold on;

plot(t_model, y_model, '--', 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('\omega [rad/s]');

legend('Measured', 'Model');

title('Real Motor vs Estimated Transfer Function');

% ---------------------------------------------------------
% DISPLAY RESULTS
% ---------------------------------------------------------

disp(" ");
disp("=========== IDENTIFIED MOTOR PARAMETERS ===========");

fprintf('R = %.6f Ohm\n', R);
fprintf('L = %.6e H\n', L);
fprintf('J = %.6e kg.m^2\n', J);
fprintf('b = %.6e N.m.s/rad\n', b);
fprintf('K = %.6f V.s/rad (back-EMF constant)\n', K);

% Calculate fitness metrics
fit_error = norm(y_model(1:length(omega)) - omega') / norm(omega);
fprintf('\nFit quality: MSE = %.6e, RMSE = %.4f rad/s, Error = %.2f%%\n', ...
    mean((y_model(1:length(omega)) - omega').^2), ...
    sqrt(mean((y_model(1:length(omega)) - omega').^2)), ...
    fit_error*100);

% Display motor time constants
tau_elec = L / R;
tau_mech = J / b;
tau_dominant = max(tau_elec, tau_mech);

fprintf('\nDerived time constants:\n');
fprintf('  Electrical: τ_L = L/R = %.6f s\n', tau_elec);
fprintf('  Mechanical: τ_mech = J/b = %.6f s\n', tau_mech);
fprintf('  Dominant time constant = %.6f s\n', tau_dominant);

disp(" ");

%% ---------------------------------------------------------
% SAVE DATA
% ---------------------------------------------------------

save('motor_identification.mat');

disp("Data saved.");
