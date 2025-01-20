classdef SlidingModeEstimator < ObserverBase & handle
    properties
        rho_trans
        rho_rot

        epsilon_trans
        epsilon_rot

        tau_trans
        tau_rot

        c_p_trans
        c_dp_trans
        c_q_rot
        c_omega_rot

        d_est_filtered_trans  
        d_est_filtered_rot  
    end

    methods
        function obj = SlidingModeEstimator(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0);

            L_trans = 4; % Upper bound of disturbance 
            L_rot = 5;   % Upper bound of disturbance

            obj.epsilon_trans = 0.1;
            obj.epsilon_rot = 0.1;

            obj.tau_trans = 0.3;
            obj.tau_rot = 0.01;

            obj.rho_trans = (L_trans + obj.epsilon_trans); 
            obj.rho_rot = L_rot + obj.epsilon_rot;     

            obj.c_p_trans = 1;
            obj.c_dp_trans = 1.2;
            obj.c_q_rot = 10;
            obj.c_omega_rot = 10;

            obj.d_est_filtered_trans = [0; 0; 0];
            obj.d_est_filtered_rot = [0; 0; 0]; 
            
        end

        function obj = calculateDisturbanceSM_trans(obj, u_thrust, p, dp, dt)
            e_p = p - obj.x_hat_trans(1:3);
            e_dp = dp - obj.x_hat_trans(4:6);
            s = obj.c_p_trans * e_p + obj.c_dp_trans * e_dp; 
            omega = obj.rho_trans * sign(s);
            dx_hat = obj.A_trans * obj.x_hat_trans + obj.B_trans * (u_thrust - [0; 0; obj.g * obj.mass_ObserverBase]) + obj.B_trans * omega;
            obj.x_hat_trans = obj.x_hat_trans + dt * dx_hat;
            obj.d_est_filtered_trans = obj.d_est_filtered_trans + (1/obj.tau_trans) * (-obj.d_est_filtered_trans + omega) * dt;
            obj.w_hat_trans = obj.d_est_filtered_trans;
        end

        function obj = calculateDisturbanceSM_rot(obj, u_torque, q, omega, dt)
            q.normalize();
            q_vec = rotvec(q)';
            e_q = q_vec - obj.x_hat_rot(1:3);
            e_omega = omega - obj.x_hat_rot(4:6);
            s = obj.c_q_rot * e_q + obj.c_omega_rot * e_omega; 
            omega_sm = obj.rho_rot * tanh(s);
            dx_hat = obj.A_rot * obj.x_hat_rot + obj.B_rot * (u_torque - cross(omega, obj.J_ObserverBase * omega)) + obj.B_rot * omega_sm;
            obj.x_hat_rot = obj.x_hat_rot + dt * dx_hat;
            obj.d_est_filtered_rot = obj.d_est_filtered_rot + (1/obj.tau_rot) * (-obj.d_est_filtered_rot + omega) * dt;
            obj.w_hat_rot = obj.d_est_filtered_rot;
        end
    end
end
