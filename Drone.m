classdef Drone < DroneDynamic
    properties

        % Translational vars
        p_d
        dp_d
        ep
        edp

        % Rotational vars
        q_d
        eq
        eq_prev

        % Control vars
        u_thrust
        u_torque
        
        % Control constants
        kp_thrust
        kd_thrust_1
        kd_thrust_2
        kp_torque
        kd_torque_1
        kd_torque_2
        max_thrust
        max_torque

        UDE_obs
        Luenberger_obs
        SuperTwist_obs
        SlidingMode_obs

        name

        KalmanFilter

        obs_num
        disturbanceRejection_trans
        disturbanceRejection_rot
        rejection_trans
        rejection_rot

        % Gaussian error properties
        noiseRangeMin
        noiseRangeMax
        noiseMean
        noiseStdDev
        p_noisy
    end
    methods
        function obj = Drone(mass, q, x0, y0, z0, dt, name)
            if (nargin < 7)
                name = 'standard';
            end

            obj@DroneDynamic(mass, q, x0, y0, z0, dt);
            obj.obs_num = 0;
            obj.p_d = [0; 0; 0;];
            obj.dp_d = [0; 0; 0;];
            obj.q_d = quaternion(1, 0, 0, 0);
            obj.ep = [0; 0; 0;];
            obj.edp = [0; 0; 0;];
            obj.eq = obj.q;
            obj.name = name;
            
            % Initialize control vars and consts
            obj.u_thrust = [0; 0; 0];
            obj.u_torque = [0; 0; 0];
            obj.kp_thrust = 0;
            obj.kd_thrust_1 = 0;
            obj.kd_thrust_2 = 0;
            obj.kp_torque = 0;
            obj.kd_torque_1 = 0;
            obj.kd_torque_2 = 0;
            obj.max_thrust = 10;
            obj.max_torque = 10;
            
            % Observer
            obj.UDE_obs = UDE(obj.mass, obj.J, obj.p, obj.dp, obj.q, obj.omega);
            obj.Luenberger_obs = Luenberger(obj.mass, obj.J, obj.p, obj.dp, obj.q, obj.omega);
            obj.KalmanFilter = Kalman(obj.mass, obj.J, obj.p, obj.dp, obj.q, obj.omega);
            obj.SuperTwist_obs = SuperTwistEstimator(obj.mass, obj.J, obj.p, obj.dp, obj.q, obj.omega);
            obj.SlidingMode_obs = SlidingModeEstimator(obj.mass, obj.J, obj.p, obj.dp, obj.q, obj.omega);

            obj.disturbanceRejection_trans = diag([1,1,1]);
            obj.disturbanceRejection_rot = diag([1,1,1]);

            obj.rejection_trans = [0;0;0];
            obj.rejection_rot = [0;0;0];

            obj.noiseRangeMin = -1.5;
            obj.noiseRangeMax = 1.5;
            obj.noiseMean = 0;
            obj.noiseStdDev = 0.1;
            obj.p_noisy = [0; 0; 0;];
        end

        function obj = setControlGains(obj, kp_thrust, kd_thrust_1, kd_thrust_2, kp_torque, kd_torque_1, kd_torque_2)
            obj.kp_thrust = kp_thrust;
            obj.kd_thrust_1 = kd_thrust_1;
            obj.kd_thrust_2 = kd_thrust_2;
            obj.kp_torque = kp_torque;
            obj.kd_torque_1 = kd_torque_1;
            obj.kd_torque_2 = kd_torque_2;             
        end

        function obj = setAimPoint(obj, x_d, y_d, z_d)
            obj.p_d = [x_d; y_d; z_d];
        end

        function saveNoisyData(obj, filename)
            % Guardar los datos de p_noisy en un archivo CSV
            if exist('p_noisy.csv', 'file') == 2
                % Si el archivo existe, agrega los datos nuevos
                writematrix(obj.p_noisy', filename, 'WriteMode', 'append');
            else
                % Si el archivo no existe, crea uno nuevo
                writematrix(obj.p_noisy', filename);
            end
        end 

        function obj = applyControl(obj)

            if obj.obs_num == 1
            obj.UDE_obs.calculateStateUDE_trans(obj.u_thrust, obj.p, obj.dp, obj.dt);
            obj.UDE_obs.calculateStateUDE_rot(obj.u_torque, obj.q, obj.omega, obj.dt);
            obj.rejection_trans = obj.disturbanceRejection_trans * obj.UDE_obs.w_hat_trans;
            obj.rejection_rot = obj.disturbanceRejection_rot * obj.UDE_obs.w_hat_rot;
            elseif obj.obs_num == 2
            obj.Luenberger_obs.calculateDisturbanceL_trans(obj.u_thrust, obj.p, obj.dp, obj.dt);
            obj.Luenberger_obs.calculateDisturbanceL_rot(obj.u_torque, obj.q, obj.omega, obj.dt);
            obj.rejection_trans = obj.disturbanceRejection_trans * obj.Luenberger_obs.w_hat_trans;
            obj.rejection_rot = obj.disturbanceRejection_rot * obj.Luenberger_obs.w_hat_rot;
            elseif obj.obs_num == 3
            obj.SuperTwist_obs.calculateDisturbanceST_trans(obj.u_thrust, obj.p, obj.dp, obj.dt);
            obj.SuperTwist_obs.calculateDisturbanceST_rot(obj.u_torque, obj.q, obj.omega, obj.dt);
            obj.rejection_trans = obj.disturbanceRejection_trans * obj.SuperTwist_obs.w_hat_trans;
            obj.rejection_rot = obj.disturbanceRejection_rot * obj.SuperTwist_obs.w_hat_rot;
            elseif obj.obs_num == 4
            obj.SlidingMode_obs.calculateDisturbanceSM_trans(obj.u_thrust, obj.p, obj.dp, obj.dt);    
            obj.SlidingMode_obs.calculateDisturbanceSM_rot(obj.u_torque, obj.q, obj.omega, obj.dt);
            obj.rejection_trans = obj.disturbanceRejection_trans * obj.SlidingMode_obs.w_hat_trans;
            obj.rejection_rot = obj.disturbanceRejection_rot * obj.SlidingMode_obs.w_hat_rot;
            end

            if any(isnan(obj.rejection_trans))
                warning('rot contains NaN values. Resetting to zero.');
                obj.rejection_trans = [0;0;0];
            end

            if any(isnan(obj.rejection_rot))
                warning('rot contains NaN values. Resetting to zero.');
                obj.rejection_rot = [0;0;0];
            end

            noise = obj.generateGaussianError(obj.noiseRangeMin, obj.noiseRangeMax, obj.noiseMean, obj.noiseStdDev);
            obj.p_noisy = obj.p + noise;

            % Llamada al método para guardar los datos de p_noisy
            obj.saveNoisyData('p_noisy.csv');

            obj.KalmanFilter.kalman_estimate(obj.p_noisy, obj.dp, obj.u_thrust);

            % Translational control
            obj.ep = obj.p_d - obj.p;
            obj.edp = obj.dp_d - obj.dp;

            obj.u_thrust = obj.kp_thrust * obj.ep + obj.kd_thrust_1 * obj.edp;
            if norm(obj.u_thrust)~=0
                obj.u_thrust = obj.max_thrust * tanh(norm(obj.u_thrust) / obj.max_thrust) * obj.u_thrust / norm(obj.u_thrust);
            end
            obj.u_thrust = obj.u_thrust + [0; 0; obj.mass * obj.g] - obj.rejection_trans;
            obj.F_bf = norm(obj.u_thrust);

            if obj.F_bf < 0
                obj.F_bf = 0;
            end

            if norm(obj.F_bf) ~= 0
                uz_uvec = obj.u_thrust / norm(obj.u_thrust); % unit vector on thrust force direction :)
                obj.q_d = exp(0.5*log(quaternion([dot([0;0;1],uz_uvec);[cross([0;0;1],uz_uvec)]]')));
                obj.q_d = normalize(obj.q_d);

            else
                obj.q_d = obj.q;
                obj.q_d = normalize(obj.q_d);
            end

            % Rotation control

            obj.eq_prev = obj.eq;
            obj.eq = obj.q * (obj.q_d');
            eomega = rotvec(obj.eq * obj.eq_prev')'/obj.dt;

            if norm(rotvec(obj.eq)) > pi || norm(rotvec(obj.eq))< -pi
                obj.q_d = -obj.q_d;
                obj.eq = (obj.q_d') * obj.q;
            end

            obj.u_torque = -obj.kp_torque * rotvec(obj.eq)' - obj.kd_torque_1 * obj.omega;
            if norm(obj.u_torque) ~= 0
                obj.u_torque = obj.max_torque * tanh(norm(obj.u_torque)/obj.max_torque) * obj.u_torque/norm(obj.u_torque);
            else
                obj.u_torque = [0;0;0];
            end
            obj.tau = obj.J * obj.u_torque - obj.J * obj.rejection_rot;
        end

        function obj = updateDroneDataExtention(obj)
            obj.thrust_array(:, obj.iterations + 1) = obj.F_bf;
            obj.torque_array(:, obj.iterations + 1) = obj.tau;
            obj.ep_array(:, obj.iterations + 1) = obj.ep;
            obj.eq_array(:, obj.iterations + 1) = rotvec(obj.eq)';
            obj.p_array(:, obj.iterations + 1) = obj.p;
            obj.p_array_noisy(:, obj.iterations + 1) = obj.p_noisy;
            obj.p_hat_array(:, obj.iterations + 1) = obj.KalmanFilter.Xk;
            obj.dp_array(:, obj.iterations + 1) = obj.dp;
            obj.ddp_array(:, obj.iterations + 1) = obj.ddp;
            obj.p_d_array(:, obj.iterations + 1) = obj.p_d;
            q_components = compact(obj.q);
            obj.q_array(:, obj.iterations + 1) = q_components;
            dq_components = compact(obj.dq);
            obj.dq_array(:, obj.iterations + 1) = dq_components;
            q_d_components = compact(obj.q_d);
            obj.q_d_array(:, obj.iterations + 1) = q_d_components;
            obj.omega_array(:, obj.iterations + 1) = obj.omega;
            obj.domega_array(:, obj.iterations + 1) = obj.domega;
            obj.dx_state_trans(:, obj.iterations + 1) = obj.dx_sys_trans;
            obj.dx_state_rot(:, obj.iterations + 1) = obj.dx_sys_rot;
            obj.time_array(obj.iterations + 1) = obj.iterations * obj.dt;
            obj = obj.updateDisturbanceArray(obj.disturbance_trans, obj.disturbance_rot);
            if obj.obs_num == 1
                obj.disturbance_measure_trans(:,obj.iterations + 1) = obj.UDE_obs.w_hat_trans;
                obj.disturbance_measure_rot(:,obj.iterations + 1) = obj.UDE_obs.w_hat_rot;
            elseif obj.obs_num == 2
                obj.disturbance_measure_trans(:,obj.iterations + 1) = obj.Luenberger_obs.w_hat_trans;
                obj.disturbance_measure_rot(:,obj.iterations + 1) = obj.Luenberger_obs.w_hat_rot;
            elseif obj.obs_num == 3
                obj.disturbance_measure_trans(:,obj.iterations + 1) = obj.SuperTwist_obs.w_hat_trans;
                obj.disturbance_measure_rot(:,obj.iterations + 1) = obj.SuperTwist_obs.w_hat_rot;
            elseif obj.obs_num == 4
                obj.disturbance_measure_trans(:,obj.iterations + 1) = obj.SlidingMode_obs.w_hat_trans;
                obj.disturbance_measure_rot(:,obj.iterations + 1) = obj.SlidingMode_obs.w_hat_rot;
            end
        end

        function obj = setDisturbance(obj, disturbance_vector_trans, disturbance_vector_rot)
            obj.disturbance_trans = disturbance_vector_trans;
            obj.disturbance_rot = disturbance_vector_rot;
        end

        function errorValue = generateGaussianError(obj, rangeMin, rangeMax, meanValue, stdDev)
            while true
                value = meanValue + stdDev * randn();
                if value >= rangeMin && value <= rangeMax
                    errorValue = value;
                    return;
                end
            end
        end      

        function obj = update(obj)
            obj = obj.applyControl();
            obj = obj.updateState();
            obj.iterations = obj.iterations + 1;
            obj = updateDroneDataExtention(obj);
        end
    end
end