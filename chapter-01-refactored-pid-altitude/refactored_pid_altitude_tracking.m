% Refactored PID altitude-tracking simulation
% This script defines the parameters, reference signal, disturbance, and controller gains.
clc;
clear;
close all;

% Physical parameters
params.m = 0.75;
params.g = 9.81;

% Simulation settings
sim.dt = 0.01;
sim.t_end = 10;
sim.t = 0:sim.dt:sim.t_end;

% Controller gains
gains.Kp = 10;
gains.Ki = 2;
gains.Kd = 6;

% Reference altitude
% z_ref = 1.5;
% a time-varying reference
z_ref = 1 + 0.3*sin(0.8*sim.t);

% Disturbance force (N); applied after 4 s
disturbance = zeros(size(sim.t));
disturbance(sim.t >= 4) = 1.2;

% Run simulation
results = simulate_altitude_pid(sim, params, gains, z_ref, disturbance);

% Plot results
plot_altitude_response(sim.t, results, z_ref, disturbance);

% Display final error
% final_error = z_ref - results.z(end);
final_error = z_ref(end) - results.z(end); % Time-varying reference

fprintf('Final altitude = %.4f m\n', results.z(end));
fprintf('Final error    = %.4f m\n', final_error);

% Missing validation print (good engineering habit)
fprintf('Max altitude   = %.4f m\n', max(results.z));

% Optional sanity check
if any(results.z < -0.1)
    warning('Altitude went below ground level — check dynamics!');
end