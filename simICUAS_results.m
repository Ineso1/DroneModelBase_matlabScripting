

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
sim_time = 25; % seconds
iterations = sim_time / dt;

pid_tht = [2 1 0];
pid_trq = [40 8 0];

% UDE
ude = Drone(mass, q, x0, y0, z0, dt);
ude.obs_num = 1;
ude.name = 'UDE';
%ude = ude.setControlGains(1, 0.5, 0, 25.5, 4, 0);
ude = ude.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
ude = ude.setAimPoint(1,2,2);
ude = ude.setDisturbance([0; 0; 0], [0; 0; 0]);

disturbance_trans = [0; 0; 0];
disturbance_rot = [0; 0; 0];

ude.disturbanceRejection_trans = diag([0,0,0]);
ude.disturbanceRejection_rot = diag([0,0,0]);

% normal
normal = Drone(mass, q, x0, y0, z0, dt);
normal.obs_num = 0;
normal.name = 'Without';
%normal = normal.setControlGains(1, 0.5, 0, 25.5, 4, 0);
normal = normal.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
normal = normal.setAimPoint(1,2,2);
normal = normal.setDisturbance([0; 0; 0], [0; 0; 0]);

normal.disturbanceRejection_trans = diag([0,0,0]);
normal.disturbanceRejection_rot = diag([0,0,0]);

% Luenberger
luenberger = Drone(mass, q, x0, y0, z0, dt);
luenberger.obs_num = 2;
luenberger.name = 'Luenberger';
%luenberger = luenberger.setControlGains(1, 0.5, 0, 25.5, 4, 0);
luenberger = luenberger.setControlGains(pid_tht(1), pid_tht(2), pid_tht(3), pid_trq(1), pid_trq(2), pid_trq(3));
luenberger = luenberger.setAimPoint(1,2,2);
luenberger = luenberger.setDisturbance([0; 0; 0], [0; 0; 0]);

luenberger.disturbanceRejection_trans = diag([0,0,0]);
luenberger.disturbanceRejection_rot = diag([0,0,0]);

for i = 1:iterations
    time = i * dt;
    if time >= 10 
        ude.disturbanceRejection_trans = diag([1,1,1]);
        ude.disturbanceRejection_rot = diag([15,15,15]);

        luenberger.disturbanceRejection_trans = diag([0.9,1,1]);
        luenberger.disturbanceRejection_rot = diag([2,2,1]);
    end
    % if time >= 5 
    %     normal = normal.setAimPoint(1,2,2);
    %     ude = ude.setAimPoint(1, 2, 2);
    %     luenberger = luenberger.setAimPoint(1, 2, 2);
    % end
    if time >= 10 %&& time <= 15
        %disturbance_trans = [4 * sin(pi * (time - 2)); 2 * sin(pi * (time - 2)); 3 * sin(pi * (time - 2))];
        disturbance_trans = [1;2;5];
        ude = ude.setDisturbance(disturbance_trans, disturbance_rot);
        normal = normal.setDisturbance(disturbance_trans, disturbance_rot);
        luenberger = luenberger.setDisturbance(disturbance_trans, disturbance_rot);
    else
        %drone = drone.setDisturbance([0; 0; 0], [0; 0; 0]); 
        disturbance_trans = [0;0;0];
        ude = ude.setDisturbance(disturbance_trans, disturbance_rot);
        normal = normal.setDisturbance(disturbance_trans, disturbance_rot);
        luenberger = luenberger.setDisturbance(disturbance_trans, disturbance_rot);
    end
    
    ude = ude.update();
    normal = normal.update();
    luenberger = luenberger.update();
end
%%
%drone.plotControl();
clc; close all;

print_settings.save = true;
print_settings.path = './pictures/';
print_settings.code = '_sim';

colors = ["#D81E5B", "#22577A","#5E8C61"];

% {"#1E3E62","#FF6500"}

results = ResultsAnalysis([normal, ude, luenberger],colors , print_settings);

results.showNormComparison();

%drone.plotDisturbance_trans();
%drone.plotDisturbance_rot();
