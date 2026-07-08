% Controller tuning using natural frequency and damping ratio
% This script compares altitude responses for different damping ratios
% and different natural frequencies, and prints response results.

clc;
clear;
close all;

% Physical parameters
m = 0.75;       % drone mass in kg
g = 9.81;      % gravitational acceleration in m/s^2

% Simulation settings
dt = 0.01;
t_end = 6;
t = 0:dt:t_end;

% Reference altitude
z_ref = 1.0;

%% Part 1: Effect of damping ratio

% Fixed natural frequency
omega_n = 3;

% Different damping ratios
zeta_list = [0.3; 0.7; 1.2];

% Store altitude responses
z_all_damping = zeros(length(zeta_list), length(t));

% Store gain values
Kp_damping = zeros(length(zeta_list), 1);
Kd_damping = zeros(length(zeta_list), 1);

% Store response results
max_alt_damping = zeros(length(zeta_list), 1);
final_alt_damping = zeros(length(zeta_list), 1);
final_error_damping = zeros(length(zeta_list), 1);
overshoot_damping = zeros(length(zeta_list), 1);

for i = 1:length(zeta_list)

    zeta = zeta_list(i);

    % Tune gains from desired second-order dynamics
    Kp = m*omega_n^2;
    Kd = 2*m*zeta*omega_n;

    % Store gains
    Kp_damping(i) = Kp;
    Kd_damping(i) = Kd;

    % Initial states
    z = zeros(1, length(t));
    vz = zeros(1, length(t));
    thrust = zeros(1, length(t));

    for k = 1:length(t)-1

        % Altitude and velocity errors
        e = z_ref - z(k);
        ev = -vz(k);

        % PD control around hover
        u = Kp*e + Kd*ev;

        % Total thrust
        thrust(k) = m*g + u;

        % Prevent negative thrust
        thrust(k) = max(0, thrust(k));

        % Vertical dynamics
        az = thrust(k)/m - g;

        % Euler integration
        vz(k+1) = vz(k) + az*dt;
        z(k+1) = z(k) + vz(k+1)*dt;
    end

    % Store response for this damping ratio
    z_all_damping(i,:) = z;

    % Calculate response results
    max_alt_damping(i) = max(z);
    final_alt_damping(i) = z(end);
    final_error_damping(i) = z_ref - z(end);
    overshoot_damping(i) = max(0, (max(z) - z_ref)/z_ref*100);
end

% Plot damping-ratio comparison
figure;
plot(t, z_all_damping(1,:), 'LineWidth', 1.8);
hold on;
plot(t, z_all_damping(2,:), 'LineWidth', 1.8);
plot(t, z_all_damping(3,:), 'LineWidth', 1.8);
yline(z_ref, '--k', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Altitude z (m)');
title('Altitude Response for Different Damping Ratios');
legend('\zeta = 0.3', '\zeta = 0.7', '\zeta = 1.2', ...
       'Reference', 'Location', 'best');

%% Part 2: Effect of natural frequency

% Fixed damping ratio
zeta = 0.7;

% Different natural frequencies
omega_n_list = [2, 4, 6];

% Store altitude responses
z_all_frequency = zeros(length(omega_n_list), length(t));

% Store gain values
Kp_frequency = zeros(length(omega_n_list), 1);
Kd_frequency = zeros(length(omega_n_list), 1);

% Store response results
max_alt_frequency = zeros(length(omega_n_list), 1);
final_alt_frequency = zeros(length(omega_n_list), 1);
final_error_frequency = zeros(length(omega_n_list), 1);
overshoot_frequency = zeros(length(omega_n_list), 1);

for i = 1:length(omega_n_list)

    omega_n = omega_n_list(i);

    % Tune gains from desired second-order dynamics
    Kp = m*omega_n^2;
    Kd = 2*m*zeta*omega_n;

    % Store gains
    Kp_frequency(i) = Kp;
    Kd_frequency(i) = Kd;

    % Initial states
    z = zeros(1, length(t));
    vz = zeros(1, length(t));
    thrust = zeros(1, length(t));

    for k = 1:length(t)-1

        % Altitude and velocity errors
        e = z_ref - z(k);
        ev = -vz(k);

        % PD control around hover
        u = Kp*e + Kd*ev;

        % Total thrust
        thrust(k) = m*g + u;

        % Prevent negative thrust
        thrust(k) = max(0, thrust(k));

        % Vertical dynamics
        az = thrust(k)/m - g;

        % Euler integration
        vz(k+1) = vz(k) + az*dt;
        z(k+1) = z(k) + vz(k+1)*dt;
    end

    % Store response for this natural frequency
    z_all_frequency(i,:) = z;

    % Calculate response results
    max_alt_frequency(i) = max(z);
    final_alt_frequency(i) = z(end);
    final_error_frequency(i) = z_ref - z(end);
    overshoot_frequency(i) = max(0, (max(z) - z_ref)/z_ref*100);
end

% Plot natural-frequency comparison
figure;
plot(t, z_all_frequency(1,:), 'LineWidth', 1.8);
hold on;
plot(t, z_all_frequency(2,:), 'LineWidth', 1.8);
plot(t, z_all_frequency(3,:), 'LineWidth', 1.8);
yline(z_ref, '--k', 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Altitude z (m)');
title('Altitude Response for Different Natural Frequencies');
legend('\omega_n = 2 rad/s', '\omega_n = 4 rad/s', '\omega_n = 6 rad/s', ...
       'Reference', 'Location', 'best');

%% Display gain values and response results

fprintf('--- Damping-ratio comparison ---\n');
for i = 1:length(zeta_list)
    fprintf(['zeta = %.1f, omega_n = %.1f, Kp = %.4f, Kd = %.4f, ', ...
             'Max altitude = %.4f m, Overshoot = %.2f %%, Final altitude = %.4f m, Final error = %.4f m\n'], ...
        zeta_list(i), 3, Kp_damping(i), Kd_damping(i), ...
        max_alt_damping(i), overshoot_damping(i), ...
        final_alt_damping(i), final_error_damping(i));
end

fprintf('\n--- Natural-frequency comparison ---\n');
for i = 1:length(omega_n_list)
    fprintf(['omega_n = %.1f, zeta = %.1f, Kp = %.4f, Kd = %.4f, ', ...
             'Max altitude = %.4f m, Overshoot = %.2f %%, Final altitude = %.4f m, Final error = %.4f m\n'], ...
        omega_n_list(i), 0.7, Kp_frequency(i), Kd_frequency(i), ...
        max_alt_frequency(i), overshoot_frequency(i), ...
        final_alt_frequency(i), final_error_frequency(i));
end