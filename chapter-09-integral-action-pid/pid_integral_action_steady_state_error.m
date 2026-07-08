% Adding integral action for full PID altitude control
% This script compares PD and PID altitude control under constant disturbance.

clc;
clear;
close all;

% Physical parameters
m = 0.75;
g = 9.81;

% Simulation settings
dt = 0.01;
t_end = 10;
t = 0:dt:t_end;

% Reference altitude
z_ref = 1;

% Design parameters
wn = 3;       % natural frequency in rad/s
zeta = 0.7;  % damping ratio

% Convert design parameters to PD gains
Kp = m*wn^2;
Kd = 2*m*zeta*wn;

% Integral gain for PID controller
Ki = 0.8;

% Constant downward disturbance
disturbance_value = 0.5;

%% Case 1: PD controller with disturbance

% Initial states
z_pd = zeros(size(t));
vz_pd = zeros(size(t));
thrust_pd = zeros(size(t));
error_pd = zeros(size(t));
disturbance = disturbance_value*ones(size(t));

prev_error = z_ref - z_pd(1);

for k = 1:length(t)-1

    % Altitude error
    error = z_ref - z_pd(k);
    error_pd(k) = error;

    % Derivative of error
    if k == 1
        error_dot = 0;
    else
        error_dot = (error - prev_error)/dt;
    end

    % PD control law
    u_pd = Kp*error + Kd*error_dot;

    % Effective thrust after disturbance
    thrust_pd(k) = m*g + u_pd - disturbance(k);

    % Altitude dynamics
    az = (thrust_pd(k) - m*g)/m;

    % Euler integration
    vz_pd(k+1) = vz_pd(k) + az*dt;
    z_pd(k+1) = z_pd(k) + vz_pd(k+1)*dt;

    % Update previous error
    prev_error = error;
end

% Final stored values
thrust_pd(end) = thrust_pd(end-1);
error_pd(end) = z_ref - z_pd(end);

%% Case 2: PID controller with disturbance

% Initial states
z_pid = zeros(size(t));
vz_pid = zeros(size(t));
thrust_pid = zeros(size(t));
error_pid = zeros(size(t));
error_int_hist = zeros(size(t));

% Integral memory
error_int = 0;

prev_error = z_ref - z_pid(1);

for k = 1:length(t)-1

    % Altitude error
    error = z_ref - z_pid(k);
    error_pid(k) = error;

    % Derivative of error
    if k == 1
        error_dot = 0;
    else
        error_dot = (error - prev_error)/dt;
    end

    % Integral of error
    error_int = error_int + error*dt;
    error_int_hist(k+1) = error_int;

    % PID control law
    u_pid = Kp*error + Kd*error_dot + Ki*error_int;

    % Effective thrust after disturbance
    thrust_pid(k) = m*g + u_pid - disturbance(k);

    % Altitude dynamics
    az = (thrust_pid(k) - m*g)/m;

    % Euler integration
    vz_pid(k+1) = vz_pid(k) + az*dt;
    z_pid(k+1) = z_pid(k) + vz_pid(k+1)*dt;

    % Update previous error
    prev_error = error;
end

% Final stored values
thrust_pid(end) = thrust_pid(end-1);
error_pid(end) = z_ref - z_pid(end);
error_int_final = error_int;

%% Calculate response results

% PD results
max_alt_pd = max(z_pd);
overshoot_pd = max(0, (max_alt_pd - z_ref)/z_ref*100);
final_alt_pd = z_pd(end);
final_error_pd = z_ref - z_pd(end);
mae_pd = mean(abs(z_ref - z_pd));

idx_10_pd = find(z_pd >= 0.1*z_ref, 1, 'first');
idx_90_pd = find(z_pd >= 0.9*z_ref, 1, 'first');

if ~isempty(idx_10_pd) && ~isempty(idx_90_pd)
    rise_time_pd = t(idx_90_pd) - t(idx_10_pd);
else
    rise_time_pd = NaN;
end

% PID results
max_alt_pid = max(z_pid);
overshoot_pid = max(0, (max_alt_pid - z_ref)/z_ref*100);
final_alt_pid = z_pid(end);
final_error_pid = z_ref - z_pid(end);
mae_pid = mean(abs(z_ref - z_pid));

idx_10_pid = find(z_pid >= 0.1*z_ref, 1, 'first');
idx_90_pid = find(z_pid >= 0.9*z_ref, 1, 'first');

if ~isempty(idx_10_pid) && ~isempty(idx_90_pid)
    rise_time_pid = t(idx_90_pid) - t(idx_10_pid);
else
    rise_time_pid = NaN;
end

%% Plot altitude response

figure;
plot(t, z_pd, 'LineWidth', 2);
hold on;
plot(t, z_pid, 'LineWidth', 2);
yline(z_ref, '--r', 'Reference');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('PD and PID Altitude Response with Constant Disturbance');
legend('PD with disturbance', 'PID with disturbance', 'Reference', 'Location', 'best');

%% Plot tracking error

figure;
plot(t, error_pd, 'LineWidth', 2);
hold on;
plot(t, error_pid, 'LineWidth', 2);
yline(0, '--k');
grid on;
xlabel('Time (s)');
ylabel('Tracking error (m)');
title('Tracking Error: PD vs PID');
legend('PD error', 'PID error', 'Zero error', 'Location', 'best');

%% Plot integral error

figure;
plot(t, error_int_hist, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Integral error (m·s)');
title('Integral Error Growth in PID Control');

%% Plot disturbance

figure;
plot(t, disturbance, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Disturbance force (N)');
title('Constant Disturbance');

%% Display results

fprintf('--- Controller design parameters ---\n');
fprintf('wn = %.2f rad/s\n', wn);
fprintf('zeta = %.2f\n', zeta);
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n', Kd);
fprintf('Ki = %.4f\n', Ki);
fprintf('Disturbance = %.4f N\n', disturbance_value);

fprintf('\n--- PD with disturbance ---\n');
fprintf('Rise time      = %.4f s\n', rise_time_pd);
fprintf('Max altitude   = %.4f m\n', max_alt_pd);
fprintf('Overshoot      = %.2f %%\n', overshoot_pd);
fprintf('Final altitude = %.4f m\n', final_alt_pd);
fprintf('Final error    = %.4f m\n', final_error_pd);
fprintf('Mean abs error = %.4f m\n', mae_pd);

fprintf('\n--- PID with disturbance ---\n');
fprintf('Rise time      = %.4f s\n', rise_time_pid);
fprintf('Max altitude   = %.4f m\n', max_alt_pid);
fprintf('Overshoot      = %.2f %%\n', overshoot_pid);
fprintf('Final altitude = %.4f m\n', final_alt_pid);
fprintf('Final error    = %.4f m\n', final_error_pid);
fprintf('Mean abs error = %.4f m\n', mae_pid);
fprintf('Final integral error = %.4f m.s\n', error_int_final);