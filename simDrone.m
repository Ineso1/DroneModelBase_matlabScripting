% close all
% clearvars
% clc

% mass = 0.405;
% q = quaternion(1, 0, 0, 0);
% x0 = 0;
% y0 = 0;
% z0 = 0;
% dt = 0.01;
% sim_time = 8;

% % Trajectory: radius 2m, center (0,0,2), and frequency 0.5 Hz
% trajectory_type = 'circle';
% trajectory_params = [2, 0, 0, 2, 0.5];

% % Create TrajectoryFollower drone
% drone = TrajectoryFollower(mass, q, x0, y0, z0, dt, trajectory_type, trajectory_params);
% drone.obs_num = 2;
% drone = drone.setControlGains(4, 1, 2, 20, 2, 6);

% drone = drone.generateTrajectory(sim_time);

% for i = 1:(sim_time / dt)
%     drone = drone.update();
% end

% drone.plotPosition();
% drone.plotErrors();
% drone.plotOrientation();
% pause(3);
% drone.animateDroneTrajectory();




close all
clearvars
clc

mass = 0.405;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;
sim_time = 8; % seconds
iterations = sim_time / dt;

drone = Drone(mass, q, x0, y0, z0, dt);
drone.obs_num = 2;
drone = drone.setControlGains(4, 1, 2, 20, 2, 6);
drone = drone.setAimPoint(1, 2, 3);
drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]);

disturbance_trans = [0; 0; 0];
disturbance_rot = [0; 0; 0];

for i = 1:iterations
    time = i * dt;
    if time >= 2 && time <= 3
        
        disturbance_trans = [0.5 * sin(pi * (time - 2)); 1 * sin(pi * (time - 2)); 6 * sin(pi * (time - 2))];
        %disturbance_rot = [3.5 * sin(pi * (time - 2)); 2.5 * sin(pi * (time - 2)); 6.5 * sin(pi * (time - 2))];
        drone = drone.setDisturbance(disturbance_trans, disturbance_rot);
    else
        drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]); 
    end
    
    drone = drone.update();
end

drone.plotPosition();
drone.plotErrors();
drone.plotOrientation();
drone.plotDisturbance_trans();
pause(3);
drone.animateDroneTrajectory();