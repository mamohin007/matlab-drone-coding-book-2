% Saturation and anti-windup for realistic PID altitude control
% This script compares PID control without anti-windup,
% PID control with conditional integration, and PID control
% with a clamped integrator.

clc;
clear;
close all;

%% Parameters
m = 0.75;
g = 9.81;
dt = 0.01;
t_end = 6;
t = 0:dt:t_end;

%% Reference
z_ref = 2;

%% Controller design
wn = 3;
zeta = 0.7;

Kp = m*wn^2;
Kd = 2*m*zeta*wn;
Ki = 0.5;

%% Thrust limits
T_min = 0;
T_max = 8;

%% Clamp limit for clamped integrator
int_limit = 2;

%% Storage
z_noAW = zeros(size(t));
vz_noAW = zeros(size(t));

z_condAW = zeros(size(t));
vz_condAW = zeros(size(t));

z_clampAW = zeros(size(t));
vz_clampAW = zeros(size(t));

% Integral states
int_noAW = 0;
int_condAW = 0;
int_clampAW = 0;

% Integrator history
int_hist_noAW = zeros(size(t));
int_hist_condAW = zeros(size(t));
int_hist_clampAW = zeros(size(t));

% Thrust histories
T1_unsat_hist = zeros(size(t));
T1_hist = zeros(size(t));

T2_unsat_hist = zeros(size(t));
T2_hist = zeros(size(t));

T3_unsat_hist = zeros(size(t));
T3_hist = zeros(size(t));

%% Simulation loop
for k = 1:length(t)-1

    % Common disturbance
    d = 0.5*sin(2*pi*0.5*t(k));

    %% Case 1: No Anti-Windup
    e1 = z_ref - z_noAW(k);
    e1_dot = -vz_noAW(k);

    % Always integrate
    int_noAW = int_noAW + e1*dt;
    int_hist_noAW(k+1) = int_noAW;

    % PID control
    u1 = Kp*e1 + Kd*e1_dot + Ki*int_noAW;

    % Unsaturated and saturated thrust
    T1_unsat = m*g + u1 - d;
    T1 = max(T_min, min(T1_unsat, T_max));

    % Store thrust history
    T1_unsat_hist(k) = T1_unsat;
    T1_hist(k) = T1;

    % Dynamics
    az1 = (T1 - m*g)/m;
    vz_noAW(k+1) = vz_noAW(k) + az1*dt;
    z_noAW(k+1) = z_noAW(k) + vz_noAW(k+1)*dt;

    %% Case 2: Conditional Integration
    e2 = z_ref - z_condAW(k);
    e2_dot = -vz_condAW(k);

    % PID control using current integrator
    u2 = Kp*e2 + Kd*e2_dot + Ki*int_condAW;

    % Unsaturated and saturated thrust
    T2_unsat = m*g + u2 - d;
    T2 = max(T_min, min(T2_unsat, T_max));

    % Integrate only if not saturated
    if T2 == T2_unsat
        int_condAW = int_condAW + e2*dt;
    end
    int_hist_condAW(k+1) = int_condAW;

    % Store thrust history
    T2_unsat_hist(k) = T2_unsat;
    T2_hist(k) = T2;

    % Dynamics
    az2 = (T2 - m*g)/m;
    vz_condAW(k+1) = vz_condAW(k) + az2*dt;
    z_condAW(k+1) = z_condAW(k) + vz_condAW(k+1)*dt;

    %% Case 3: Clamped Integrator
    e3 = z_ref - z_clampAW(k);
    e3_dot = -vz_clampAW(k);

    % Integrate always, then clamp
    int_clampAW = int_clampAW + e3*dt;
    int_clampAW = max(-int_limit, min(int_clampAW, int_limit));
    int_hist_clampAW(k+1) = int_clampAW;

    % PID control
    u3 = Kp*e3 + Kd*e3_dot + Ki*int_clampAW;

    % Unsaturated and saturated thrust
    T3_unsat = m*g + u3 - d;
    T3 = max(T_min, min(T3_unsat, T_max));

    % Store thrust history
    T3_unsat_hist(k) = T3_unsat;
    T3_hist(k) = T3;

    % Dynamics
    az3 = (T3 - m*g)/m;
    vz_clampAW(k+1) = vz_clampAW(k) + az3*dt;
    z_clampAW(k+1) = z_clampAW(k) + vz_clampAW(k+1)*dt;
end

%% Fill last thrust values for cleaner plots
T1_unsat_hist(end) = T1_unsat_hist(end-1);
T1_hist(end) = T1_hist(end-1);

T2_unsat_hist(end) = T2_unsat_hist(end-1);
T2_hist(end) = T2_hist(end-1);

T3_unsat_hist(end) = T3_unsat_hist(end-1);
T3_hist(end) = T3_hist(end-1);

%% Calculate response results

% No anti-windup
max_alt_noAW = max(z_noAW);
overshoot_noAW = max(0, (max_alt_noAW - z_ref)/z_ref*100);
final_alt_noAW = z_noAW(end);
final_error_noAW = z_ref - final_alt_noAW;
mae_noAW = mean(abs(z_ref - z_noAW));
final_int_noAW = int_hist_noAW(end);

% Conditional integration
max_alt_condAW = max(z_condAW);
overshoot_condAW = max(0, (max_alt_condAW - z_ref)/z_ref*100);
final_alt_condAW = z_condAW(end);
final_error_condAW = z_ref - final_alt_condAW;
mae_condAW = mean(abs(z_ref - z_condAW));
final_int_condAW = int_hist_condAW(end);

% Clamped integrator
max_alt_clampAW = max(z_clampAW);
overshoot_clampAW = max(0, (max_alt_clampAW - z_ref)/z_ref*100);
final_alt_clampAW = z_clampAW(end);
final_error_clampAW = z_ref - final_alt_clampAW;
mae_clampAW = mean(abs(z_ref - z_clampAW));
final_int_clampAW = int_hist_clampAW(end);

%% Plot altitude responses
figure;
plot(t, z_noAW, '--', 'LineWidth', 1.8);
hold on;
plot(t, z_condAW, '-', 'LineWidth', 2.0);
plot(t, z_clampAW, '-.', 'LineWidth', 2.0);
yline(z_ref, '--k', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Comparison of Anti-Windup Methods');
legend('No Anti-Windup', 'Conditional Integration', ...
       'Clamped Integrator', 'Reference', 'Location', 'best');

%% Plot integrator histories
figure;
plot(t, int_hist_noAW, '--', 'LineWidth', 1.8);
hold on;
plot(t, int_hist_condAW, '-', 'LineWidth', 2.0);
plot(t, int_hist_clampAW, '-.', 'LineWidth', 2.0);
yline(int_limit, ':k', 'Upper Clamp');
yline(-int_limit, ':k', 'Lower Clamp');
grid on;
xlabel('Time (s)');
ylabel('Integral state (m⋅s)');
title('Integrator State Comparison');
legend('No Anti-Windup', 'Conditional Integration', ...
       'Clamped Integrator', 'Upper Clamp', 'Lower Clamp', ...
       'Location', 'best');

%% Plot thrust saturation check for No Anti-Windup
figure;
plot(t, T1_unsat_hist, '--', 'LineWidth', 1.8);
hold on;
plot(t, T1_hist, '-', 'LineWidth', 2.0);
yline(T_max, '--r', 'T_{max}');
yline(T_min, '--k', 'T_{min}');
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust Saturation Check: No Anti-Windup');
legend('T_{unsat,noAW}', 'T_{sat,noAW}', 'T_{max}', 'T_{min}', ...
       'Location', 'best');

%% Plot thrust saturation check for Conditional Integration
figure;
plot(t, T2_unsat_hist, '--', 'LineWidth', 1.8);
hold on;
plot(t, T2_hist, '-', 'LineWidth', 2.0);
yline(T_max, '--r', 'T_{max}');
yline(T_min, '--k', 'T_{min}');
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust Saturation Check: Conditional Integration');
legend('T_{unsat,condAW}', 'T_{sat,condAW}', 'T_{max}', 'T_{min}', ...
       'Location', 'best');

%% Plot thrust saturation check for Clamped Integrator
figure;
plot(t, T3_unsat_hist, '--', 'LineWidth', 1.8);
hold on;
plot(t, T3_hist, '-', 'LineWidth', 2.0);
yline(T_max, '--r', 'T_{max}');
yline(T_min, '--k', 'T_{min}');
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Thrust Saturation Check: Clamped Integrator');
legend('T_{unsat,clampAW}', 'T_{sat,clampAW}', 'T_{max}', 'T_{min}', ...
       'Location', 'best');

%% Display results
fprintf('--- Controller and actuator settings ---\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n', Kd);
fprintf('Ki = %.4f\n', Ki);
fprintf('T_min = %.2f N\n', T_min);
fprintf('T_max = %.2f N\n', T_max);
fprintf('Integrator clamp limit = %.2f m⋅s\n', int_limit);

fprintf('\n--- No Anti-Windup ---\n');
fprintf('Max altitude = %.4f m\n', max_alt_noAW);
fprintf('Overshoot = %.2f %%\n', overshoot_noAW);
fprintf('Final altitude = %.4f m\n', final_alt_noAW);
fprintf('Final error = %.4f m\n', final_error_noAW);
fprintf('Mean abs error = %.4f m\n', mae_noAW);
fprintf('Final integral state = %.4f m⋅s\n', final_int_noAW);

fprintf('\n--- Conditional Integration ---\n');
fprintf('Max altitude = %.4f m\n', max_alt_condAW);
fprintf('Overshoot = %.2f %%\n', overshoot_condAW);
fprintf('Final altitude = %.4f m\n', final_alt_condAW);
fprintf('Final error = %.4f m\n', final_error_condAW);
fprintf('Mean abs error = %.4f m\n', mae_condAW);
fprintf('Final integral state = %.4f m⋅s\n', final_int_condAW);

fprintf('\n--- Clamped Integrator ---\n');
fprintf('Max altitude = %.4f m\n', max_alt_clampAW);
fprintf('Overshoot = %.2f %%\n', overshoot_clampAW);
fprintf('Final altitude = %.4f m\n', final_alt_clampAW);
fprintf('Final error = %.4f m\n', final_error_clampAW);
fprintf('Mean abs error = %.4f m\n', mae_clampAW);
fprintf('Final integral state = %.4f m⋅s\n', final_int_clampAW);