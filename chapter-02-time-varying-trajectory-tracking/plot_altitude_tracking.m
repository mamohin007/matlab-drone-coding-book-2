function plot_altitude_tracking(t, results, z_ref, disturbance)

figure;

subplot(3,1,1);
plot(t, results.z, 'LineWidth', 1.8);
hold on;
plot(t, z_ref, '--', 'LineWidth', 1.5);
grid on;
ylabel('Altitude (m)');
title('Altitude Trajectory Tracking');
legend('Actual altitude', 'Reference altitude', 'Location', 'best');

subplot(3,1,2);
plot(t, results.error, 'LineWidth', 1.8);
hold on;
yline(0, '--');
grid on;
ylabel('Error (m)');
title('Tracking Error');
legend('Tracking error', 'Zero error', 'Location', 'best');

subplot(3,1,3);
plot(t, results.thrust, 'LineWidth', 1.5);
hold on;
plot(t, disturbance, '--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Thrust Response and External Disturbance');
legend('Commanded thrust', 'External disturbance (downward)', 'Location', 'best');

end