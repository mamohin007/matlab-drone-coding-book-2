function plot_position_velocity_comparison(t, z_ref, vz_ref, results_pid, results_ff, results_vc)

figure;

subplot(4,1,1);
plot(t, z_ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, results_pid.z, 'b', 'LineWidth', 1.8);
plot(t, results_ff.z, 'r', 'LineWidth', 1.8);
plot(t, results_vc.z, 'g', 'LineWidth', 1.8);
grid on;
ylabel('Altitude (m)');
title('Altitude Tracking Comparison');
legend('Reference', 'PID only', 'PID + feedforward', 'FF + velocity control', 'Location', 'best');

subplot(4,1,2);
plot(t, z_ref - results_pid.z, 'b', 'LineWidth', 1.8);
hold on;
plot(t, z_ref - results_ff.z, 'r', 'LineWidth', 1.8);
plot(t, z_ref - results_vc.z, 'g', 'LineWidth', 1.8);
yline(0, '--');
grid on;
ylabel('Error (m)');
title('Tracking Error Comparison');
legend('PID only', 'PID + feedforward', 'FF + velocity control', 'Location', 'best');

subplot(4,1,3);
plot(t, vz_ref, 'k--', 'LineWidth', 1.5);
hold on;
plot(t, results_pid.vz, 'b', 'LineWidth', 1.8);
plot(t, results_ff.vz, 'r', 'LineWidth', 1.8);
plot(t, results_vc.vz, 'g', 'LineWidth', 1.8);
grid on;
ylabel('Velocity (m/s)');
title('Velocity Tracking Comparison');
legend('Reference velocity', 'PID only', 'PID + feedforward', 'FF + velocity control', 'Location', 'best');

subplot(4,1,4);
plot(t, results_pid.thrust, 'b', 'LineWidth', 1.5);
hold on;
plot(t, results_ff.thrust, 'r', 'LineWidth', 1.5);
plot(t, results_vc.thrust, 'g', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Thrust (N)');
title('Commanded Thrust Comparison');
legend('PID only', 'PID + feedforward', 'FF + velocity control', 'Location', 'best');

end