% Saturation and anti-windup altitude-control simulation
% This script compares saturated altitude tracking with and without anti-windup.

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
z_ref  = 1 + 0.3*sin(2*t);
vz_ref = 0.6*cos(2*t);
az_ref = -1.2*sin(2*t);

% Strong downward disturbance, applied after 4 s
disturbance = zeros(size(t));
disturbance(t >= 4) = 5;

% Actuator limits
limits.T_min = 0;
limits.T_max = 13;

% Run saturated case without anti-windup
results_no_aw = simulate_altitude_saturation_no_aw(sim, params, gains, z_ref, vz_ref, az_ref, disturbance, limits);

% Run saturated case with anti-windup
results_aw = simulate_altitude_saturation_aw(sim, params, gains, z_ref, vz_ref, az_ref, disturbance, limits);

% Plot comparison
plot_saturation_comparison(t, z_ref, disturbance, results_no_aw, results_aw, limits);

% Print performance metrics
fprintf('--- Without anti-windup ---\n');
fprintf('Mean absolute error  = %.4f m\n', mean(abs(z_ref - results_no_aw.z)));
fprintf('Final error          = %.4f m\n', z_ref(end) - results_no_aw.z(end));
fprintf('Maximum thrust used  = %.4f N\n', max(results_no_aw.thrust));

fprintf('--- With anti-windup ---\n');
fprintf('Mean absolute error  = %.4f m\n', mean(abs(z_ref - results_aw.z)));
fprintf('Final error          = %.4f m\n', z_ref(end) - results_aw.z(end));
fprintf('Maximum thrust used  = %.4f N\n', max(results_aw.thrust));