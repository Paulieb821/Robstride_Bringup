%% Load Data
data = readtable('sys_id_data.csv');
time = data.time;
tau = data.torque;
theta = data.position;

dt = mean(diff(time));

% Create iddata object
u = tau;
y = theta;
z = iddata(y, u, dt); % 1 output (theta), 1 input (tau)

%% --- Model 1: Linear Spring-Mass-Damper (SMD) ---
% Equation: J*theta_ddot + c*theta_dot + k*theta = tau

% Define ODE function for gray-box model
linear_smd = @(p, x, u) [
    x(2);  % theta_dot
    (1/p(1)) * (u - p(2)*x(2) - p(3)*x(1))  % theta_ddot
];

% Initial parameter guesses: [J, c, k]
params0_1 = [0.01, 0.01, 0.01];

% Create idgrey model
init_sys1 = idgrey(linear_smd, params0_1, 'c', [], 0, ...
    'StateName', {'theta', 'theta_dot'}, ...
    'InputName', 'tau', ...
    'OutputName', 'theta');

% Estimate model
opt = greyestOptions('Display', 'on');
sys1 = greyest(z, init_sys1, opt);

%% --- Model 2: SMD with Coulomb Friction ---
% Equation: J*theta_ddot + c*theta_dot + k*theta = tau - f*tanh(gain*theta_dot)

coulomb_smd = @(p, x, u) [
    x(2);  % theta_dot
    (1/p(1)) * (u - p(2)*x(2) - p(3)*x(1) - p(4)*tanh(50*x(2)))  % theta_ddot with friction
];

% Initial parameter guesses: [J, c, k, f]
params0_2 = [0.01, 0.01, 0.01, 0.01];

init_sys2 = idgrey(coulomb_smd, params0_2, 'c', [], 0, ...
    'StateName', {'theta', 'theta_dot'}, ...
    'InputName', 'tau', ...
    'OutputName', 'theta');

% Estimate model
sys2 = greyest(z, init_sys2, opt);

%% --- Model Comparison ---
figure;
compare(z, sys1, sys2);
legend('Measured', 'Linear SMD', 'SMD with Coulomb Friction');
title('System Identification Comparison');
