function [u, error_int] = pid_controller(error, prev_error, error_int, dt, gains)

% Integral update
error_int = error_int + error*dt;

% Simple anti-windup clamp
error_int = max(min(error_int, 2), -2);

% Derivative term
error_der = (error - prev_error)/dt;

% PID control law
u = gains.Kp*error + gains.Ki*error_int + gains.Kd*error_der;

end
