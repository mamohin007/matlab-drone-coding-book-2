function plot_altitude_ff_comparison(t, z_ref, results_pid, results_ff)

figure;

subplot(4,1,1);
plot(t, z_ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, results_pid.z, 'b', 'LineWidth', 1.8);
plot(t, results_ff.z, 'r', 'LineWidth', 1.8);
grid on;
ylabel('Altitude (m)');
title('Trajectory Tracking Comparison');
legend('Reference', 'PID only', 'PID + feedforward', 'Location', 'best');

subplot(4,1,2);
plot(t, z_ref - results_pid.z, 'LineWidth', 1.8);
hold on;
plot(t, z_ref - results_ff.z, 'LineWidth', 1.8);
yline(0, '--');
grid on;
ylabel('Error (m)');
title('Tracking Error Comparison');
legend('PID only error', 'PID + feedforward error', 'Zero error', 'Location', 'best');

subplot(4,1,3);
plot(t, results_pid.thrust, 'LineWidth', 1.8);
hold on;
plot(t, results_ff.thrust, 'LineWidth', 1.8);
grid on;
ylabel('Thrust (N)');
title('Commanded Thrust Comparison');
legend('PID only commanded thrust', 'PID + feedforward commanded thrust', 'Location', 'best');

subplot(4,1,4);
plot(t, results_pid.z - results_ff.z, 'LineWidth', 1.8);
grid on;
xlabel('Time (s)');
ylabel('Altitude difference (m)');
title('Difference Between PID Only and PID + Feedforward');
legend('z_{PID} - z_{FF}', 'Location', 'best');

end