% From PD tuning to control design using natural frequency and damping ratio
% This script designs a PD altitude controller using wn and zeta,
% then compares the response with and without disturbance.

clc;
clear;
close all;

% Physical parameters
m = 0.75;
g = 9.81;

% Simulation settings
dt = 0.01;
t_end = 6;
t = 0:dt:t_end;

% Reference altitude
z_ref = 1;

% Design parameters
wn = 3;       % natural frequency in rad/s
zeta = 0.7;  % damping ratio

% Convert design parameters to controller gains
Kp = m*wn^2;
Kd = 2*m*zeta*wn;

%% Case 1: Designed PD controller without disturbance

% Initial states
z_no_dist = zeros(size(t));
vz_no_dist = zeros(size(t));
thrust_no_dist = zeros(size(t));
error_no_dist = zeros(size(t));

prev_error = z_ref - z_no_dist(1);

for k = 1:length(t)-1

    % Altitude error
    error = z_ref - z_no_dist(k);
    error_no_dist(k) = error;

    % Derivative of error
    if k == 1
        error_dot = 0;
    else
        error_dot = (error - prev_error)/dt;
    end

    % Designed PD controller
    u = Kp*error + Kd*error_dot;

    % Total thrust without disturbance
    thrust_no_dist(k) = m*g + u;

    % Altitude dynamics
    az = (thrust_no_dist(k) - m*g)/m;

    % Euler integration
    vz_no_dist(k+1) = vz_no_dist(k) + az*dt;
    z_no_dist(k+1) = z_no_dist(k) + vz_no_dist(k+1)*dt;

    % Update previous error
    prev_error = error;
end

% Final stored values
thrust_no_dist(end) = thrust_no_dist(end-1);
error_no_dist(end) = z_ref - z_no_dist(end);

%% Case 2: Designed PD controller with sinusoidal disturbance

% Initial states
z_dist = zeros(size(t));
vz_dist = zeros(size(t));
thrust_dist = zeros(size(t));
error_dist = zeros(size(t));
disturbance = zeros(size(t));

prev_error = z_ref - z_dist(1);

for k = 1:length(t)-1

    % Altitude error
    error = z_ref - z_dist(k);
    error_dist(k) = error;

    % Derivative of error
    if k == 1
        error_dot = 0;
    else
        error_dot = (error - prev_error)/dt;
    end

    % Designed PD controller
    u = Kp*error + Kd*error_dot;

    % Sinusoidal disturbance
    disturbance(k) = 0.5*sin(2*pi*0.5*t(k));

    % Effective thrust after disturbance
    thrust_dist(k) = m*g + u - disturbance(k);

    % Altitude dynamics
    az = (thrust_dist(k) - m*g)/m;

    % Euler integration
    vz_dist(k+1) = vz_dist(k) + az*dt;
    z_dist(k+1) = z_dist(k) + vz_dist(k+1)*dt;

    % Update previous error
    prev_error = error;
end

% Final stored values
thrust_dist(end) = thrust_dist(end-1);
error_dist(end) = z_ref - z_dist(end);
disturbance(end) = disturbance(end-1);

%% Calculate response results

% No-disturbance case
max_alt_no_dist = max(z_no_dist);
overshoot_no_dist = max(0, (max_alt_no_dist - z_ref)/z_ref*100);
final_alt_no_dist = z_no_dist(end);
final_error_no_dist = z_ref - z_no_dist(end);
mae_no_dist = mean(abs(z_ref - z_no_dist));

idx_10_no_dist = find(z_no_dist >= 0.1*z_ref, 1, 'first');
idx_90_no_dist = find(z_no_dist >= 0.9*z_ref, 1, 'first');

if ~isempty(idx_10_no_dist) && ~isempty(idx_90_no_dist)
    rise_time_no_dist = t(idx_90_no_dist) - t(idx_10_no_dist);
else
    rise_time_no_dist = NaN;
end

% Disturbance case
max_alt_dist = max(z_dist);
overshoot_dist = max(0, (max_alt_dist - z_ref)/z_ref*100);
final_alt_dist = z_dist(end);
final_error_dist = z_ref - z_dist(end);
mae_dist = mean(abs(z_ref - z_dist));

idx_10_dist = find(z_dist >= 0.1*z_ref, 1, 'first');
idx_90_dist = find(z_dist >= 0.9*z_ref, 1, 'first');

if ~isempty(idx_10_dist) && ~isempty(idx_90_dist)
    rise_time_dist = t(idx_90_dist) - t(idx_10_dist);
else
    rise_time_dist = NaN;
end

%% Plot altitude response

figure;
plot(t, z_no_dist, 'LineWidth', 2);
hold on;
plot(t, z_dist, 'LineWidth', 2);
yline(z_ref, '--r', 'Reference');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Designed PD Controller Using Natural Frequency and Damping Ratio');
legend('Without disturbance', 'With disturbance', 'Reference', 'Location', 'best');

%% Plot tracking error

figure;
plot(t, error_no_dist, 'LineWidth', 2);
hold on;
plot(t, error_dist, 'LineWidth', 2);
yline(0, '--k');
grid on;
xlabel('Time (s)');
ylabel('Tracking error (m)');
title('Tracking Error Comparison');
legend('Without disturbance', 'With disturbance', 'Zero error', 'Location', 'best');

%% Plot disturbance

figure;
plot(t, disturbance, 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Disturbance force (N)');
title('Sinusoidal Disturbance');

%% Display results

fprintf('--- Controller design parameters ---\n');
fprintf('wn = %.2f rad/s\n', wn);
fprintf('zeta = %.2f\n', zeta);
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n', Kd);

fprintf('\n--- Without disturbance ---\n');
fprintf('Rise time      = %.4f s\n', rise_time_no_dist);
fprintf('Max altitude   = %.4f m\n', max_alt_no_dist);
fprintf('Overshoot      = %.2f %%\n', overshoot_no_dist);
fprintf('Final altitude = %.4f m\n', final_alt_no_dist);
fprintf('Final error    = %.4f m\n', final_error_no_dist);
fprintf('Mean abs error = %.4f m\n', mae_no_dist);

fprintf('\n--- With disturbance ---\n');
fprintf('Rise time      = %.4f s\n', rise_time_dist);
fprintf('Max altitude   = %.4f m\n', max_alt_dist);
fprintf('Overshoot      = %.2f %%\n', overshoot_dist);
fprintf('Final altitude = %.4f m\n', final_alt_dist);
fprintf('Final error    = %.4f m\n', final_error_dist);
fprintf('Mean abs error = %.4f m\n', mae_dist);