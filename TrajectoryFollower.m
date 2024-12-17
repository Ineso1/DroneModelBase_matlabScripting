classdef TrajectoryFollower < Drone
    properties
        trajectory_type                 % Type of trajectory: 'line', 'circle', 'custom'
        trajectory_params               % Parameters for the chosen trajectory
        time_array_trajectory_follower  % Simulation time array
        reference_position_array        % Precomputed trajectory positions
        current_time_index              % Keeps track of the time index
    end

    methods
        function obj = TrajectoryFollower(mass, q, x0, y0, z0, dt, trajectory_type, trajectory_params)
            obj@Drone(mass, q, x0, y0, z0, dt);
            obj.trajectory_type = trajectory_type;
            obj.trajectory_params = trajectory_params;
            obj.time_array_trajectory_follower = [];
            obj.reference_position_array = [];
            obj.current_time_index = 1;
        end

        function obj = generateTrajectory(obj, sim_time)
            obj.time_array_trajectory_follower = 0:obj.dt:sim_time;
            obj.reference_position_array = zeros(3, length(obj.time_array_trajectory_follower));

            switch obj.trajectory_type
                case 'line'
                    % Straight-line trajectory: params = [x_f, y_f, z_f]
                    x_f = obj.trajectory_params(1);
                    y_f = obj.trajectory_params(2);
                    z_f = obj.trajectory_params(3);
                    for k = 1:length(obj.time_array_trajectory_follower)
                        obj.reference_position_array(:, k) = [(x_f / sim_time) * obj.time_array_trajectory_follower(k);
                                                               (y_f / sim_time) * obj.time_array_trajectory_follower(k);
                                                               (z_f / sim_time) * obj.time_array_trajectory_follower(k)];
                    end
                case 'circle'
                    % Circular trajectory: params = [radius, center_x, center_y, center_z, frequency]
                    radius = obj.trajectory_params(1);
                    center_x = obj.trajectory_params(2);
                    center_y = obj.trajectory_params(3);
                    center_z = obj.trajectory_params(4);
                    frequency = obj.trajectory_params(5);
                    for k = 1:length(obj.time_array_trajectory_follower)
                        t = obj.time_array_trajectory_follower(k);
                        obj.reference_position_array(:, k) = [center_x + radius * cos(2 * pi * frequency * t);
                                                               center_y + radius * sin(2 * pi * frequency * t);
                                                               center_z];
                    end
                case 'custom'
                    % Custom trajectory: custom literal XD
                    custom_func = obj.trajectory_params;
                    for k = 1:length(obj.time_array_trajectory_follower)
                        obj.reference_position_array(:, k) = custom_func(obj.time_array_trajectory_follower(k));
                    end
                otherwise
                    error('Unsupported trajectory type! Use "line", "circle", or "custom".');
            end
        end

        function obj = followTrajectory(obj)
            if obj.current_time_index <= length(obj.time_array_trajectory_follower)
                obj.p_d = obj.reference_position_array(:, obj.current_time_index);
                obj.current_time_index = obj.current_time_index + 1;
            end
        end

        function obj = update(obj)
            obj = obj.followTrajectory();
            obj = obj.applyControl();
            obj = obj.updateState();
            obj.iterations = obj.iterations + 1;
            obj = updateDroneDataExtention(obj);
        end
    end
end