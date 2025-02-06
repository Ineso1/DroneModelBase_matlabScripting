

%%% CASO DE Luenberger
close all
clearvars
clc

mass = 0.405;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;
sim_time = 50; % seconds
iterations = sim_time / dt;

drone = Drone(mass, q, x0, y0, z0, dt);
drone.obs_num = 2;
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
        drone.disturbanceRejection_trans = diag([1,1,1]);
        drone.disturbanceRejection_rot = diag([1,1,1]);
    end
    if time >= 5 
        drone = drone.setAimPoint(1, 2, 2);
    end
    if time >= 6 && time <= 50
        w = 0.3;
        disturbance_trans = [1 * sin(w * time); 2 * cos(w*time); 5 * sin(w*time)];
        disturbance_rot = [0; 0; 0];
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