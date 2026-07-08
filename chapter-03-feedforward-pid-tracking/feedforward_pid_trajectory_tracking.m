% Feedforward PID altitude trajectory-tracking simulation
% This script compares PID-only tracking with PID plus feedforward control.

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

% Reference trajectory
t = sim.t;
z_ref = 1 + 0.3*sin(0.8*t);
vz_ref = 0.24*cos(0.8*t);
az_ref = -0.192*sin(0.8*t);

% Disturbance force (N), applied after 5 s
disturbance = zeros(size(t));
disturbance(t >= 5) = 1.0;

% Run PID-only case
results_pid = simulate_altitude_pid_tracking(sim, params, gains, z_ref, disturbance);

% Run PID + feedforward case
results_ff = simulate_altitude_feedforward_pid(sim, params, gains, z_ref, vz_ref, az_ref, disturbance);

% Plot comparison
plot_altitude_ff_comparison(t, z_ref, results_pid, results_ff);

% Display tracking errors
fprintf('PID only mean abs error   = %.4f m\n', mean(abs(z_ref - results_pid.z)));
fprintf('PID + FF mean abs error   = %.4f m\n', mean(abs(z_ref - results_ff.z)));
fprintf('PID only final error      = %.4f m\n', z_ref(end) - results_pid.z(end));
fprintf('PID + FF final error      = %.4f m\n', z_ref(end) - results_ff.z(end));