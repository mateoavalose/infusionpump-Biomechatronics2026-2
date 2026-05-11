%% Theorical Parameters
R = 27.4;         % Ohms

K = 0.0404;       % N.m/A
b = K * 0.011/297;  % N.m.s

% Parameters estimated
L = 0.001;         % Henry
J = 0.001;         % kg.m^2

%% Estimated Parameters
R = 27.4;
L = 6.954437e-01;
J = 6.981318e-06;
K = 0.036730;
b = 5.334275e-06; 
gearRatio = 472.727;

%% Transfer Function
Va = 5.647;
s = tf('s');
G = (K / gearRatio) / ((J*s + b)*(L*s + R) + K^2);

% Step Response
[y, t] = step(Va * G, t_sim);
figure;
plot(t, y);
xlabel('Time (s)');
ylabel('Response');
title('Step Response of the System');
grid on;