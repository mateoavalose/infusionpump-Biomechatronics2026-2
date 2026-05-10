function err = motorCost(x, R, t_meas, omega_meas, Va)

L = x(1);
J = x(2);
K = x(3);
b = x(4);

s = tf('s');

G = K / ((J*s + b)*(L*s + R) + K^2);

% Simulate step response robustly. If simulation fails or returns NaN,
% return a large error so optimizer avoids invalid regions.
try
	[y, t] = step(Va * G, t_meas(end));
	% Ensure vectors are real and finite
	if any(~isfinite(y)) || any(~isfinite(t))
		err = 1e12;
		return;
	end
	% Interpolate model output onto measured time vector. Allow extrapolation
	% to avoid NaNs if time grids slightly differ.
	y_interp = interp1(t, y, t_meas, 'linear', 'extrap');
	if any(~isfinite(y_interp))
		err = 1e12;
		return;
	end
	err = sum((omega_meas - y_interp).^2);
	if ~isfinite(err) || isnan(err)
		err = 1e12;
	end
catch
	err = 1e12;
end

end
