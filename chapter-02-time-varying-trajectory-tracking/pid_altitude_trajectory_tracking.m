% PID altitude trajectory-tracking simulation
% This script defines the parameters, time-varying reference, disturbance, and controller gains.

clc;
clear;
close all;

% Physical parameters
params.m = 0.75;
params.g = 9.81;

% Simulation settings
sim.dt = 0.01;
sim.t_end = 12;
sim.t = 0:sim.dt:sim.t_end;

% Controller gains
gains.Kp = 10;
gains.Ki = 2;
gains.Kd = 6;

% Time-varying reference altitude
z_ref = 1 + 0.3*sin(1.5*sim.t);

% Disturbance force (N), applied after 5 s
disturbance = zeros(size(sim.t));
disturbance(sim.t >= 5) = 1.0;

% Run simulation
results = simulate_altitude_pid_tracking(sim, params, gains, z_ref, disturbance);

% Plot results
plot_altitude_tracking(sim.t, results, z_ref, disturbance);

% Tracking error
tracking_error = z_ref - results.z;

fprintf('Final altitude         = %.4f m\n', results.z(end));
fprintf('Final reference        = %.4f m\n', z_ref(end));
fprintf('Final tracking error   = %.4f m\n', tracking_error(end));
fprintf('Maximum altitude       = %.4f m\n', max(results.z));
fprintf('Mean absolute error    = %.4f m\n', mean(abs(tracking_error)));