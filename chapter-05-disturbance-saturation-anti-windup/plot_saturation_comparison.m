function plot_saturation_comparison(t, z_ref, disturbance, results_no_aw, results_aw, limits)

figure;

subplot(4,1,1);
plot(t, z_ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, results_no_aw.z, 'b', 'LineWidth', 1.8);
plot(t, results_aw.z, 'r', 'LineWidth', 1.8);
grid on;
ylabel('Altitude (m)');
title('Altitude Tracking with Saturation');
legend('Reference', 'No anti-windup', 'With anti-windup', 'Location', 'best');

subplot(4,1,2);
plot(t, results_no_aw.error, 'b', 'LineWidth', 1.8);
hold on;
plot(t, results_aw.error, 'r', 'LineWidth', 1.8);
yline(0, '--');
grid on;
ylabel('Error (m)');
title('Tracking Error');
legend('No anti-windup', 'With anti-windup', 'Zero error', 'Location', 'best');

subplot(4,1,3);
plot(t, results_no_aw.thrust, 'b', 'LineWidth', 1.8);
hold on;
plot(t, results_aw.thrust, 'r', 'LineWidth', 1.8);
yline(limits.T_max, '--k', 'T_{max}');
plot(t, disturbance, 'g--', 'LineWidth', 1.2);
grid on;
ylabel('Thrust (N)');
title('Saturated Thrust and Disturbance');
legend('No anti-windup thrust', 'With anti-windup thrust', 'Max thrust', 'Disturbance', 'Location', 'best');

subplot(4,1,4);
plot(t, results_no_aw.error_int, 'b', 'LineWidth', 1.8);
hold on;
plot(t, results_aw.error_int, 'r', 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Integral error');
title('Integral Term Growth');
legend('No anti-windup', 'With anti-windup', 'Location', 'best');

end