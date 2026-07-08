function results = simulate_altitude_feedforward_pid(sim, params, gains, z_ref, vz_ref, az_ref, disturbance)

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

% Initial conditions
z(1) = z_ref(1);
vz(1) = vz_ref(1);

% PID memory
error_int = 0;
prev_error = z_ref(1) - z(1);

for k = 1:n-1

    % Tracking error
    error = z_ref(k) - z(k);
    tracking_error(k) = error;

    % PID feedback controller
    [u_pid, error_int] = pid_controller(error, prev_error, error_int, dt, gains);

    % Feedforward acceleration term
    u_ff = m*az_ref(k);

    % Total commanded thrust with feedforward
    thrust(k) = m*g + u_ff + u_pid;

    % Thrust saturation
    thrust(k) = max(0, thrust(k));

    % Altitude dynamics with external downward disturbance
    az = (thrust(k) - disturbance(k))/m - g;

    % Euler integration
    vz(k+1) = vz(k) + az*dt;
    z(k+1) = z(k) + vz(k)*dt;

    % Update previous error
    prev_error = error;
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