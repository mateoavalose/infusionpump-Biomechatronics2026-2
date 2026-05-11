function err = motorCost(x, R, t, omega_meas, Va)

% Parameters
L = x(1);
J = x(2);
K = x(3);
b = x(4);
gearRatio = 472.7272;

% Reject invalid values
if any(x <= 0)

    err = 1e12;
    return;

end

try

    % Transfer function
    s = tf('s');

    % Output-shaft speed model: divide motor speed by gearbox ratio.
    G = (K / gearRatio) / ((J*s + b)*(L*s + R) + K^2);

    % Simulate on a uniform grid because `step` requires evenly spaced time.
    t_meas = t(:);
    if numel(t_meas) < 2
        err = 1e12;
        return;
    end

    dt_meas = median(diff(t_meas));
    if ~isfinite(dt_meas) || dt_meas <= 0
        err = 1e12;
        return;
    end

    t_sim = (0:dt_meas:t_meas(end)).';
    if numel(t_sim) < 2
        t_sim = linspace(0, t_meas(end), 2).';
    end

    y = step(Va * G, t_sim);

    y = squeeze(y);

    % Convert to rpm
    rpm_sim = interp1(t_sim, y * 60/(2*pi), t_meas, 'linear', 'extrap');

    rpm_meas = omega_meas * 60/(2*pi);

    % Error
    err = mean(((rpm_meas(:) - rpm_sim(:)) ./ max(abs(rpm_meas(:)), 1)).^2);

    % Penalize NaNs
    if isnan(err) || isinf(err)

        err = 1e12;

    end

catch

    err = 1e12;

end