

%%% CASO DE UDE
close all
clearvars
clc

mass = 0.405;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;
sim_time = 20; % seconds
iterations = sim_time / dt;

drone = Drone(mass, q, x0, y0, z0, dt);
drone.obs_num = 3;
drone = drone.setControlGains(1, 0.5, 0, 25.5, 4, 0);
drone = drone.setAimPoint(0, 0, 1);
drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]);

disturbance_trans = [0; 0; 0];
disturbance_rot = [0; 0; 0];

drone.disturbanceRejection_trans = diag([0,0,0]);
drone.disturbanceRejection_rot = diag([0,0,0]);

for i = 1:iterations
    time = i * dt;
    if time >= 4 
        drone.disturbanceRejection_trans = diag([0,0,0]);
        drone.disturbanceRejection_rot = diag([0,0,0]);
    end
    if time >= 5 
        drone = drone.setAimPoint(1, 2, 2);
    end
    if time >= 6 && time <= 50
        %disturbance_trans = [4 * sin(pi * (time - 2)); 2 * sin(pi * (time - 2)); 3 * sin(pi * (time - 2))];
        disturbance_trans = [1;2;5];
        drone = drone.setDisturbance(disturbance_trans, disturbance_rot);
    else
        %drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]); 
        disturbance_trans = [0;0;0];
        drone = drone.setDisturbance(disturbance_trans, disturbance_rot);
    end
    
    drone = drone.update();
end

drone.plotControl();
% drone.plotFilterPos();
% drone.plotPosition();
drone.plotErrors();
%drone.plotOrientation();
drone.plotDisturbance_trans();
drone.plotDisturbance_rot();

%drone.plotOrientationError();
%pause(3);
%drone.animateDroneTrajectory();