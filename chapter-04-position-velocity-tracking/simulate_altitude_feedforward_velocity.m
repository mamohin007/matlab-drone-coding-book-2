function results = simulate_altitude_feedforward_velocity(sim, params, gains, z_ref, vz_ref, az_ref, disturbance)

t = sim.t;
dt = sim.dt;
m = params.m;
g = params.g;

% Preallocate arrays
n = length(t);
z = zeros(1, n);
vz = zeros(1, n);
thrust = zeros(1, n);
tracking_error = zeros(1, n);

% Start on trajectory
z(1) = z_ref(1);
vz(1) = vz_ref(1);

% Integral memory
error_int = 0;

for k = 1:n-1

    % Position and velocity errors
    pos_error = z_ref(k) - z(k);
    vel_error = vz_ref(k) - vz(k);
    tracking_error(k) = pos_error;

    % Integral of position error
    error_int = error_int + pos_error*dt;
    error_int = max(min(error_int, 2), -2);

    % Position and velocity feedback control
    u_fb = gains.Kp*pos_error + gains.Ki*error_int + gains.Kd*vel_error;

    % Feedforward acceleration term
    u_ff = m*az_ref(k);

    % Total commanded thrust
    thrust(k) = m*g + u_fb + u_ff;

    % Thrust saturation
    thrust(k) = max(0, thrust(k));

    % Altitude dynamics with external downward disturbance
    az = (thrust(k) - disturbance(k))/m - g;

    % Euler integration
    vz(k+1) = vz(k) + az*dt;
    z(k+1) = z(k) + vz(k)*dt;
end

% Final values
thrust(end) = thrust(end-1);
tracking_error(end) = z_ref(end) - z(end);

% Store outputs
results.z = z;
results.vz = vz;
results.thrust = thrust;
results.error = tracking_error;

end