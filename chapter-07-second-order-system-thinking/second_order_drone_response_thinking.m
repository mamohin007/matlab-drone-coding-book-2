% From PD to second-order system thinking for drone control
% This script compares standard second-order step responses
% for different damping ratios and natural frequencies.

clc;
clear;
close all;

% Time vector
t = 0:0.01:5;

% Unit step reference
ref = 1;

%% Part 1: Effect of damping ratio

% Fixed natural frequency
wn = 3;   % rad/s

% Different damping ratios
zeta_values = [0.2, 0.7, 1.0, 1.5];

% Store responses and response results
response_damping = zeros(length(zeta_values), length(t));
rise_time_damping = zeros(length(zeta_values), 1);
max_response_damping = zeros(length(zeta_values), 1);
overshoot_damping = zeros(length(zeta_values), 1);
final_value_damping = zeros(length(zeta_values), 1);
final_error_damping = zeros(length(zeta_values), 1);
settling_time_damping = zeros(length(zeta_values), 1);

figure;
hold on;

for i = 1:length(zeta_values)

    zeta = zeta_values(i);

    % Standard second-order transfer function
    num = wn^2;
    den = [1 2*zeta*wn wn^2];

    sys = tf(num, den);

    % Step response
    [y, t_out] = step(sys, t);

    % Store response
    response_damping(i,:) = y;

    % Calculate rise time using 10 percent to 90 percent levels
    idx_10 = find(y >= 0.1*ref, 1, 'first');
    idx_90 = find(y >= 0.9*ref, 1, 'first');

    if ~isempty(idx_10) && ~isempty(idx_90)
        rise_time_damping(i) = t_out(idx_90) - t_out(idx_10);
    else
        rise_time_damping(i) = NaN;
    end

    % Calculate response results
    max_response_damping(i) = max(y);
    overshoot_damping(i) = max(0, (max(y) - ref)/ref*100);
    final_value_damping(i) = y(end);
    final_error_damping(i) = ref - y(end);

    % Estimate settling time using 2 percent band
    error_band = abs(y - ref) <= 0.02*ref;
    settling_time_damping(i) = NaN;

    for k = 1:length(error_band)
        if error_band(k) && all(error_band(k:end))
            settling_time_damping(i) = t_out(k);
            break;
        end
    end

    % Plot response
    plot(t_out, y, 'LineWidth', 2);
end

yline(ref, '--k', 'Reference');

legend('\zeta = 0.2', '\zeta = 0.7', '\zeta = 1.0', '\zeta = 1.5', ...
       'Reference', 'Location', 'best');
xlabel('Time (s)');
ylabel('Altitude response');
title('Effect of Damping Ratio on Drone Altitude Response');
grid on;

%% Part 2: Effect of natural frequency

% Different natural frequencies
wn_values = [2, 4, 6];   % rad/s

% Fixed damping ratio
zeta = 0.7;

% Store responses and response results
response_frequency = zeros(length(wn_values), length(t));
rise_time_frequency = zeros(length(wn_values), 1);
max_response_frequency = zeros(length(wn_values), 1);
overshoot_frequency = zeros(length(wn_values), 1);
final_value_frequency = zeros(length(wn_values), 1);
final_error_frequency = zeros(length(wn_values), 1);
settling_time_frequency = zeros(length(wn_values), 1);

figure;
hold on;

for i = 1:length(wn_values)

    wn = wn_values(i);

    % Standard second-order transfer function
    num = wn^2;
    den = [1 2*zeta*wn wn^2];

    sys = tf(num, den);

    % Step response
    [y, t_out] = step(sys, t);

    % Store response
    response_frequency(i,:) = y;

    % Calculate rise time using 10 percent to 90 percent levels
    idx_10 = find(y >= 0.1*ref, 1, 'first');
    idx_90 = find(y >= 0.9*ref, 1, 'first');

    if ~isempty(idx_10) && ~isempty(idx_90)
        rise_time_frequency(i) = t_out(idx_90) - t_out(idx_10);
    else
        rise_time_frequency(i) = NaN;
    end

    % Calculate response results
    max_response_frequency(i) = max(y);
    overshoot_frequency(i) = max(0, (max(y) - ref)/ref*100);
    final_value_frequency(i) = y(end);
    final_error_frequency(i) = ref - y(end);

    % Estimate settling time using 2 percent band
    error_band = abs(y - ref) <= 0.02*ref;
    settling_time_frequency(i) = NaN;

    for k = 1:length(error_band)
        if error_band(k) && all(error_band(k:end))
            settling_time_frequency(i) = t_out(k);
            break;
        end
    end

    % Plot response
    plot(t_out, y, 'LineWidth', 2);
end

yline(ref, '--k', 'Reference');

legend('\omega_n = 2 rad/s', '\omega_n = 4 rad/s', '\omega_n = 6 rad/s', ...
       'Reference', 'Location', 'best');
xlabel('Time (s)');
ylabel('Altitude response');
title('Effect of Natural Frequency on Drone Altitude Response');
grid on;

%% Display response results

fprintf('--- Damping-ratio comparison ---\n');
for i = 1:length(zeta_values)
    fprintf(['zeta = %.1f, wn = %.1f rad/s, Rise time = %.2f s, ', ...
             'Max response = %.4f, Overshoot = %.2f %%, ', ...
             'Final value = %.4f, Final error = %.4f, ', ...
             'Settling time = %.2f s\n'], ...
        zeta_values(i), 3, rise_time_damping(i), ...
        max_response_damping(i), overshoot_damping(i), ...
        final_value_damping(i), final_error_damping(i), ...
        settling_time_damping(i));
end

fprintf('\n--- Natural-frequency comparison ---\n');
for i = 1:length(wn_values)
    fprintf(['wn = %.1f rad/s, zeta = %.1f, Rise time = %.2f s, ', ...
             'Max response = %.4f, Overshoot = %.2f %%, ', ...
             'Final value = %.4f, Final error = %.4f, ', ...
             'Settling time = %.2f s\n'], ...
        wn_values(i), 0.7, rise_time_frequency(i), ...
        max_response_frequency(i), overshoot_frequency(i), ...
        final_value_frequency(i), final_error_frequency(i), ...
        settling_time_frequency(i));
end