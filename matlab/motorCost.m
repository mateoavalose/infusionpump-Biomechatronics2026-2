function err = motorCost(x, R, t_meas, omega_meas, Va)

L = x(1);
J = x(2);
K = x(3);
b = x(4);

s = tf('s');

G = K / ((J*s + b)*(L*s + R) + K^2);

[y, t] = step(Va * G, t_meas(end));

y_interp = interp1(t, y, t_meas);

err = sum((omega_meas - y_interp).^2);

end
