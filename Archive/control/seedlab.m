close all; clear;
k = 2;
sigma = 14;
kp = 2.5;
load motordat.mat
out = sim('seed.slx');
figure
subplot(2, 1, 1);
plot(out.voltage);
hold on;
plot(data(:, 1)/1000, data(:, 2));
legend('Simulated', 'Experimental');
hold off
xlabel('Time (s)');
ylabel('Voltage (V)');
subplot(2, 1, 2);
plot(out.velocity);
hold on;
plot(out.desiredvelocity);
legend('Simulated', 'Experimental');
hold off
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
