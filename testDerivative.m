dt = 0.01; 

disturbance_measure_rot = droneSuperTwist.disturbance_measure_rot;
integral1 = zeros(size(disturbance_measure_rot));  % 3xN matrix for the first integral
integral2 = zeros(size(disturbance_measure_rot));  % 3xN matrix for the second integral
for row = 1:size(disturbance_measure_rot, 1)
    integral1(row, 1) = disturbance_measure_rot(row, 1) * dt;  
    for i = 2:size(disturbance_measure_rot, 2)
        integral1(row, i) = integral1(row, i-1) + 0.5 * (disturbance_measure_rot(row, i) + disturbance_measure_rot(row, i-1)) * dt;
    end
    integral2(row, 1) = integral1(row, 1) * dt;  
    for i = 2:size(integral1, 2)
        integral2(row, i) = integral2(row, i-1) + 0.5 * (integral1(row, i) + integral1(row, i-1)) * dt;
    end
end
figure;

subplot(3, 1, 1);
plot(disturbance_measure_rot(1, :), 'o-', 'DisplayName', 'X');
hold on;
plot(disturbance_measure_rot(2, :), 'o-', 'DisplayName', 'Y');
plot(disturbance_measure_rot(3, :), 'o-', 'DisplayName', 'Z');
title('Original disturbance_measure_rot (3D vector over time)');
xlabel('Time Step');
ylabel('Value');
legend;
grid on;

subplot(3, 1, 2);
plot(integral1(1, :), 'o-', 'DisplayName', 'Integral of X');
hold on;
plot(integral1(2, :), 'o-', 'DisplayName', 'Integral of Y');
plot(integral1(3, :), 'o-', 'DisplayName', 'Integral of Z');
title('First Integral (Cumulative sum of disturbance_measure_rot)');
xlabel('Time Step');
ylabel('Integral Value');
legend;
grid on;

subplot(3, 1, 3);
plot(integral2(1, :), 'o-', 'DisplayName', 'Second Integral of X');
hold on;
plot(integral2(2, :), 'o-', 'DisplayName', 'Second Integral of Y');
plot(integral2(3, :), 'o-', 'DisplayName', 'Second Integral of Z');
title('Second Integral (Cumulative sum of the first integral)');
xlabel('Time Step');
ylabel('Second Integral Value');
legend;
grid on;
