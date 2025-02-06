clc; clear; close all;

softTrajectory = SoftTrajectoryGenerator();
real_target_pos = [2; 3; -1]; 
chargeHeight = 0.5;        

softTrajectory.addWaypoint([0; 0; -1], 0);
softTrajectory.addWaypoint([0; 0; -1], 5);
softTrajectory.addWaypoint([real_target_pos(1); real_target_pos(2); real_target_pos(3) - chargeHeight], 10);
softTrajectory.addWaypoint(real_target_pos, 15);
softTrajectory.addWaypoint([real_target_pos(1); real_target_pos(2); real_target_pos(3) - chargeHeight], 20);
softTrajectory.addWaypoint([0; 0; -2], 25);
softTrajectory.addWaypoint([0; 0; -1], 30);

softTrajectory.generateTrajectories();

dt = 0.1;
timeSteps = 0:dt:30;
positions = zeros(3, length(timeSteps));
velocities = zeros(3, length(timeSteps));

for i = 1:length(timeSteps)
    [pos, vel] = softTrajectory.getNextState(dt);
    positions(:, i) = pos;
    velocities(:, i) = vel;
end

figure;
subplot(3,1,1);
plot(timeSteps, positions(1,:), 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position'); title('X Position vs Time'); grid on;

subplot(3,1,2);
plot(timeSteps, positions(2,:), 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position'); title('Y Position vs Time'); grid on;

subplot(3,1,3);
plot(timeSteps, positions(3,:), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Position'); title('Z Position vs Time'); grid on;

figure;
subplot(3,1,1);
plot(timeSteps, velocities(1,:), 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Velocity'); title('X Velocity vs Time'); grid on;

subplot(3,1,2);
plot(timeSteps, velocities(2,:), 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Velocity'); title('Y Velocity vs Time'); grid on;

subplot(3,1,3);
plot(timeSteps, velocities(3,:), 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Velocity'); title('Z Velocity vs Time'); grid on;
