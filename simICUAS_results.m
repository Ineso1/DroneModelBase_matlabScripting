%% Workspace preparing.
clearvars; close all; clc

% UAV parameters and initial conditions
mass = 0.405;
q = quaternion(1, 0, 0, 0);
x0 = 0;
y0 = 0;
z0 = 0;
dt = 0.01;

% Simulation settings
sim_time = 30; % seconds
iterations = sim_time / dt;

% Control base para todas las simulaciones. Mantener igual para todos para
% justa comparación. 
pid_tht = [2 1 0];
pid_trq = [40 8 0];

print_settings.save = true;
print_settings.path = './pictures/';
print_settings.code = '_sim';


%% Set simulators. 
% Without observers, only PD controller. 
normal = Drone(mass, q, x0, y0, z0, dt);
normal.obs_num = 0;
normal.name = 'Without';
normal = normal.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
normal = normal.setAimPoint(1,2,2);
normal = normal.setDisturbance([0; 0; 0], [0; 0; 0]);

normal.disturbanceRejection_trans = diag([0,0,0]);
normal.disturbanceRejection_rot = diag([0,0,0]);

pid.disturbanceRejection_trans = diag([0,0,0]);
pid.disturbanceRejection_rot = diag([0,0,0]);

% UDE rot + trans
rot = Drone(mass, q, x0, y0, z0, dt);
rot.obs_num = 1;
rot.name = 'Full UDE';
rot = rot.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
rot = rot.setAimPoint(1,2,2);
rot = rot.setDisturbance([0; 0; 0], [0; 0; 0]);

rot.disturbanceRejection_trans = diag([0,0,0]);
rot.disturbanceRejection_rot = diag([0,0,0]);

% UDE (only translational)
trans = Drone(mass, q, x0, y0, z0, dt);
trans.obs_num = 1;
trans.name = 'Translational UDE';
trans = trans.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
trans = trans.setAimPoint(1,2,2);
trans = trans.setDisturbance([0; 0; 0], [0; 0; 0]);

trans.disturbanceRejection_trans = diag([0,0,0]);
trans.disturbanceRejection_rot = diag([0,0,0]);

% Luenberger
luenberger = Drone(mass, q, x0, y0, z0, dt);
luenberger.obs_num = 2;
luenberger.name = 'Luenberger';
luenberger = luenberger.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
luenberger = luenberger.setAimPoint(1,2,2);
luenberger = luenberger.setDisturbance([0; 0; 0], [0; 0; 0]);

luenberger.disturbanceRejection_trans = diag([0,0,0]);
luenberger.disturbanceRejection_rot = diag([0,0,0]);

%% Run simulations.
for i = 1:iterations
    time = i * dt;

    % ¿podría quitarse este if? dejar tal vez esas ganancias desde el comienzo. 
    % se me hace un poco mal tener que poner el observador "justo antes" de
    % la perturbación. 
    if time >= 10  
        rot.disturbanceRejection_trans = diag([1,1,1]);
        rot.disturbanceRejection_rot = diag([15,15,15]);
        
        trans.disturbanceRejection_trans = 0.55*diag([1,1,1]);

        luenberger.disturbanceRejection_trans = 0.5*diag([1,1,1]);
        luenberger.disturbanceRejection_rot = 2*diag([1,1,1]);
    end

    if time >= 10
        w = 0.5; % En el paper es 0.1, por eso se ve descente. Idealmente w = 0.5 o más :)
        %disturbance_trans = [1;1;2];
        disturbance_trans = [1 * sin(w * time); 1 * cos(w*time); 2 * sin(w*time)];
        a = 1e-2; b = 0.5;
        disturbance_rot = [0; 0; 0]; %[a*sin(b*w*time); a*sin(b*w*time); a*sin(b*w*time)];

    else
        disturbance_trans = [0;0;0];
        disturbance_rot = [0; 0; 0];
    end

    rot = rot.setDisturbance(disturbance_trans, disturbance_rot);
    trans = trans.setDisturbance(disturbance_trans, disturbance_rot);
    normal = normal.setDisturbance(disturbance_trans, disturbance_rot);
    luenberger = luenberger.setDisturbance(disturbance_trans, disturbance_rot);
    
    rot = rot.update();
    trans = trans.update();
    normal = normal.update();
    luenberger = luenberger.update();
end

%% Comparison
close all
% Set colors for each simulation and build the array for comparison. 
colors = ["#D81E5B","#5E8C61","#02dcc8","#22577A",];
simulations = [normal, luenberger, trans, rot];

% Compute evaluation and show results. 
results = ResultsAnalysis(simulations, colors , print_settings);
results.showNormComparison();