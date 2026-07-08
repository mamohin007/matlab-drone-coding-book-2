function results = simulate_altitude_pid(sim, params, gains, z_ref, disturbance)

t = sim.t;
dt = sim.dt;
m = params.m;
g = params.g;

% Preallocate
n = length(t);
z = zeros(1, n);
vz = zeros(1, n);
thrust = zeros(1, n);
error_int = 0;
prev_error = 0;

for k = 1:n-1

    % Current error
    error = z_ref(k) - z(k); % The reference at the current time step only

    % PID controller
    [u, error_int] = pid_controller(error, prev_error, error_int, dt, gains);

    % Total commanded thrust
    thrust(k) = m*g + u;

    % Optional saturation
    thrust(k) = max(0, thrust(k));

    % Altitude dynamics with external downward disturbance
    az = (thrust(k) - disturbance(k))/m - g;

    % Euler integration
    vz(k+1) = vz(k) + az*dt;
    z(k+1) = z(k) + vz(k+1)*dt;

    % Update previous error
    prev_error = error;
end

thrust(end) = thrust(end-1);

% Package outputs
results.z = z;
results.vz = vz;
results.thrust = thrust;

end
