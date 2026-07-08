function plot_altitude_response(t, results, z_ref, disturbance)

figure;
plot(t, results.z, 'b', 'LineWidth', 1.8);
hold on;
plot(t, z_ref, 'r--', 'LineWidth', 1.5);
xline(4, '--k', 'Disturbance start');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Response');
legend('Altitude', 'Reference');

figure;
plot(t, results.thrust, 'LineWidth', 1.8);
hold on;
plot(t, disturbance, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Force (N)');
title('Thrust Response and External Disturbance');
legend('Commanded thrust', ...
       'External disturbance (downward)', ...
       'Location','best');

end