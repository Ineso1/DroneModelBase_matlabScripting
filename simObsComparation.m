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

disturbance_trans = [0; 0; 0];
disturbance_rot = [0; 0; 0];

x_comp_trans = 0;
y_comp_trans = 0;
z_comp_trans = 1;

% Drone UDE
droneUDE = Drone(mass, q, x0, y0, z0, dt);
droneUDE.obs_num = 1;
droneUDE = droneUDE.setControlGains(4, 1, 2, 20, 2, 6);
droneUDE = droneUDE.setAimPoint(1, 2, 3);
droneUDE = droneUDE.setDisturbance([0; 0; 0], [0; 0; 0]);
droneUDE.disturbanceRejection_trans = diag([x_comp_trans,y_comp_trans,z_comp_trans]);
droneUDE.disturbanceRejection_rot = diag([0,0,0]);

% Drone Luenberger
droneLuenberger = Drone(mass, q, x0, y0, z0, dt);
droneLuenberger.obs_num = 2;
droneLuenberger = droneLuenberger.setControlGains(4, 1, 2, 20, 2, 6);
droneLuenberger = droneLuenberger.setAimPoint(1, 2, 3);
droneLuenberger = droneLuenberger.setDisturbance([0; 0; 0], [0; 0; 0]);
droneLuenberger.disturbanceRejection_trans = diag([x_comp_trans,y_comp_trans,z_comp_trans]);
droneLuenberger.disturbanceRejection_rot = diag([0,0,0]);

% Drone SlidingMode
droneSlidingMode = Drone(mass, q, x0, y0, z0, dt);
droneSlidingMode.obs_num = 4;
droneSlidingMode = droneSlidingMode.setControlGains(4, 1, 2, 20, 2, 6);
droneSlidingMode = droneSlidingMode.setAimPoint(1, 2, 3);
droneSlidingMode = droneSlidingMode.setDisturbance([0; 0; 0], [0; 0; 0]);
droneSlidingMode.disturbanceRejection_trans = diag([x_comp_trans,y_comp_trans,z_comp_trans]);
droneSlidingMode.disturbanceRejection_rot = diag([0,0,0]);

% Drone SuperTwist
droneSuperTwist = Drone(mass, q, x0, y0, z0, dt);
droneSuperTwist.obs_num = 3;
droneSuperTwist = droneSuperTwist.setControlGains(4, 1, 2, 20, 2, 6);
droneSuperTwist = droneSuperTwist.setAimPoint(1, 2, 3);
droneSuperTwist = droneSuperTwist.setDisturbance([0; 0; 0], [0; 0; 0]);
droneSuperTwist.disturbanceRejection_trans = diag([x_comp_trans,y_comp_trans,z_comp_trans]);
droneSuperTwist.disturbanceRejection_rot = diag([0,0,0.1]);

for i = 1:iterations
    time = i * dt;
    if time >= 2 && time <= 9        
        disturbance_trans = [0;0;5];
        droneUDE = droneUDE.setDisturbance(disturbance_trans, disturbance_rot);
        droneLuenberger = droneLuenberger.setDisturbance(disturbance_trans, disturbance_rot);
        droneSlidingMode = droneSlidingMode.setDisturbance(disturbance_trans, disturbance_rot);
        droneSuperTwist = droneSuperTwist.setDisturbance(disturbance_trans, disturbance_rot);
    else
        disturbance_trans = [0;0;0];
        droneUDE = droneUDE.setDisturbance(disturbance_trans, disturbance_rot);
        droneLuenberger = droneLuenberger.setDisturbance(disturbance_trans, disturbance_rot);
        droneSlidingMode = droneSlidingMode.setDisturbance(disturbance_trans, disturbance_rot);
        droneSuperTwist = droneSuperTwist.setDisturbance(disturbance_trans, disturbance_rot);
    end
    
    droneUDE = droneUDE.update();
    droneLuenberger = droneLuenberger.update();
    droneSuperTwist = droneSuperTwist.update();
    droneSlidingMode = droneSlidingMode.update();
end

droneUDE.plotErrors();
droneUDE.plotDisturbance_trans();
droneUDE.plotDisturbance_rot();

droneLuenberger.plotErrors();
droneLuenberger.plotDisturbance_trans();
droneLuenberger.plotDisturbance_rot();

droneSuperTwist.plotErrors();
droneSuperTwist.plotDisturbance_trans();
droneSuperTwist.plotDisturbance_rot();

droneSlidingMode.plotErrors();
droneSlidingMode.plotDisturbance_trans();
droneSlidingMode.plotDisturbance_rot();