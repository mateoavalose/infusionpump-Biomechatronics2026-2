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

TARGET_SPEED_RPM = 6.0;   % desired speed setpoint shown in plots
TEST_TIME        = 5;     % seconds (need >=2s for motor to fully settle + capture dynamics)
SUPPLY_VOLT      = 12.0;  % motor supply voltage

% TELEMETRY CONFIG — must match ESP32 main.cpp
TELEMETRY_MS     = 10;    % Sampling interval in milliseconds (must match ESP32)

% Open-loop feedforward guess used to convert speed setpoint into PWM.
% The optimized model later computes a better PWM for the target speed.
PWM_PER_RPM_EST  = 40.0;
PWM_STEP         = min(max(round(TARGET_SPEED_RPM * PWM_PER_RPM_EST), 1), 255);

fprintf('Target speed setpoint = %.2f rpm\n', TARGET_SPEED_RPM);
fprintf('Initial feedforward PWM command = %d\n', PWM_STEP);

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

disp("Starting speed-setpoint test...");

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

% Convert speed to rpm for plotting and setpoint comparison
rpm_meas = omega * 60.0 / (2.0 * pi);
rpm_setpoint = TARGET_SPEED_RPM * ones(size(t));

% ---------------------------------------------------------
% PLOT MEASURED RESPONSE VS SPEED SETPOINT
% ---------------------------------------------------------

figure;
plot(t, rpm_meas, 'LineWidth', 2);
hold on;
plot(t, rpm_setpoint, '--', 'LineWidth', 2);
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

rpm_ss_meas = omega_ss * 60.0 / (2.0 * pi);
fprintf('Measured steady-state speed = %.3f rpm\n', rpm_ss_meas);

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
% ---------------------------------------------------------

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
% BASELINE MODEL PLOT BEFORE OPTIMIZATION
% ---------------------------------------------------------

s_tf = tf('s');
[y_base, t_base] = step(Va * (K0 / ((J0*s_tf + b0)*(L0*s_tf + R0) + K0^2)), t(end));
rpm_base = y_base * 60.0 / (2.0 * pi);
rpm_base_on_meas = interp1(t_base, rpm_base, t, 'linear', 'extrap');

figure;
plot(t, rpm_meas, 'LineWidth', 2);
hold on;
plot(t, rpm_base_on_meas, '--', 'LineWidth', 2);
plot(t, rpm_setpoint, ':', 'LineWidth', 2);
grid on;

xlabel('Time [s]');
ylabel('Speed [rpm]');
title('Pre-Optimization Model vs Measured Speed');
legend('Measured speed', 'Initial model', 'Target speed');

% ---------------------------------------------------------
% OPTIMIZATION
% ---------------------------------------------------------

disp("Running optimization...");

% Validate measured data before optimization
if length(t) < 10 || length(omega) < 10
    warning('Not enough data points for reliable optimization (need >=10). Skipping optimization.');
    xopt = x0;
else
    % Evaluate cost at initial guess to ensure objective is defined
    cost0 = costFun(x0);
    if ~isfinite(cost0) || cost0 > 1e11
        warning('Initial cost is invalid (%.3e). Replacing initial guess with safe defaults.', cost0);
        x0 = [1e-3, 1e-4, max(K0,0.01), 1e-3];
        cost0 = costFun(x0);
    end

    if ~isfinite(cost0) || cost0 > 1e11
        warning('Cost at fallback initial guess still invalid. Skipping optimization and using fallback parameters.');
        xopt = x0;
    else
        try
            opts = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',2000);
            xopt = fmincon(costFun, x0, [], [], [], [], lb, ub, [], opts);
        catch ME
            warning('fmincon failed: %s\nUsing initial guess as solution.', ME.message);
            xopt = x0;
        end
    end
end

L = xopt(1);
J = xopt(2);
K = xopt(3);
b = xopt(4);
R = R0;

% ---------------------------------------------------------
% BUILD TRANSFER FUNCTION
% ---------------------------------------------------------

G = K / ((J*s_tf + b)*(L*s_tf + R) + K^2);

%% ---------------------------------------------------------
% SIMULATE MODEL
% ---------------------------------------------------------

[y_model, t_model] = step(Va * G, t(end));

rpm_model = y_model * 60.0 / (2.0 * pi);
rpm_model_on_meas = interp1(t_model, rpm_model, t, 'linear', 'extrap');

% Feedforward PWM required by the optimized model to reach the target speed.
dc_gain = K / (b * R + K^2);
omega_target = TARGET_SPEED_RPM * (2.0 * pi / 60.0);
va_for_target = omega_target / max(dc_gain, eps);
pwm_for_target = min(max(round((va_for_target / SUPPLY_VOLT) * 255.0), 1), 255);

% ---------------------------------------------------------
% COMPARE MODEL VS REAL
% ---------------------------------------------------------

figure;

plot(t, rpm_meas, 'LineWidth', 2);
hold on;

plot(t, rpm_model_on_meas, '--', 'LineWidth', 2);
plot(t, rpm_setpoint, ':', 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('Speed [rpm]');

legend('Measured speed', 'Optimized model', 'Target speed');

title('Post-Optimization Model vs Measured Speed');

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
fprintf('\nFeedforward PWM for %.2f rpm = %d / 255\n', TARGET_SPEED_RPM, pwm_for_target);

% Calculate fitness metrics
fit_error = norm(rpm_model_on_meas - rpm_meas') / norm(rpm_meas);
fprintf('\nFit quality: MSE = %.6e, RMSE = %.4f rpm, Error = %.2f%%\n', ...
    mean((rpm_model_on_meas - rpm_meas').^2), ...
    sqrt(mean((rpm_model_on_meas - rpm_meas').^2)), ...
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
