%% Parameters
R = 27.4;         % Ohms

%K = 0.0078;       % N.m/A
K = 0.0404;       % N.m/A
b = K * 0.012/297;  % N.m.s

L = 1e-3;         % Henry
J = 1e-4;         % kg.m^2

% Input Voltage
V = 1.882;

% Transfer Function
s_tf = tf('s');
G = K / ((J*s_tf + b)*(L*s_tf + R) + K^2);

% Step Response
[y, t] = step(V * G);
figure;
plot(t, y);
xlabel('Time (s)');
ylabel('Response');
title('Step Response of the System');
grid on;