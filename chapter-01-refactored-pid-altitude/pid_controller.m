function [u, error_int] = pid_controller(error, prev_error, error_int, dt, gains)

error_int = error_int + error*dt;

% Anti-windup (simple clamp)
error_int = max(min(error_int, 2), -2);

% Derivative calculation
if prev_error == 0
    error_der = 0;
else
    error_der = (error - prev_error)/dt;
end

u = gains.Kp*error + gains.Ki*error_int + gains.Kd*error_der;

end