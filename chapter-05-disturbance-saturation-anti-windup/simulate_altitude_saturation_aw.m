function results = simulate_altitude_saturation_aw(sim, params, gains, z_ref, vz_ref, az_ref, disturbance, limits)

dt = sim.dt;
m = params.m;
g = params.g;

n = length(sim.t);
z = zeros(1, n);
vz = zeros(1, n);
thrust = zeros(1, n);
thrust_cmd = zeros(1, n);
tracking_error = zeros(1, n);
error_int_hist = zeros(1, n);

% Start on trajectory
z(1) = z_ref(1);
vz(1) = vz_ref(1);

% Actuator limits
T_min = limits.T_min;
T_max = limits.T_max;

% Controller memory
error_int = 0;

for k = 1:n-1

    % Position and velocity errors
    pos_error = z_ref(k) - z(k);
    vel_error = vz_ref(k) - vz(k);
    tracking_error(k) = pos_error;

    % Temporary control using current integral value
    u_temp = gains.Kp*pos_error + gains.Kd*vel_error + gains.Ki*error_int;
    u_ff = m*az_ref(k);
    thrust_cmd_temp = m*g + u_ff + u_temp;

    % Conditional integration anti-windup
    if thrust_cmd_temp > T_min && thrust_cmd_temp < T_max
        error_int = error_int + pos_error*dt;
    end

    % Additional integral clamp
    error_int = max(min(error_int, 1.5), -1.5);
    error_int_hist(k) = error_int;

    % Final feedback control
    u = gains.Kp*pos_error + gains.Kd*vel_error + gains.Ki*error_int;

    % Final commanded thrust before saturation
    thrust_cmd(k) = m*g + u_ff + u;

    % Saturated thrust
    thrust(k) = min(max(thrust_cmd(k), T_min), T_max);

    % Altitude dynamics with external downward disturbance
    az = (thrust(k) - disturbance(k))/m - g;

    % Euler integration
    vz(k+1) = vz(k) + az*dt;
    z(k+1) = z(k) + vz(k)*dt;
end

% Final values
thrust(end) = thrust(end-1);
thrust_cmd(end) = thrust_cmd(end-1);
tracking_error(end) = z_ref(end) - z(end);
error_int_hist(end) = error_int;

% Store outputs
results.z = z;
results.vz = vz;
results.thrust = thrust;
results.thrust_cmd = thrust_cmd;
results.error = tracking_error;
results.error_int = error_int_hist;

end