classdef SuperTwistEstimator < ObserverBase & handle
    properties
        lambda0_trans
        lambda1_trans
        lambda0_rot
        lambda1_rot

        eta_trans
        eta_rot

        c_p_trans
        c_dp_trans
        c_q_rot
        c_omega_rot
    end

    methods
        function obj = SuperTwistEstimator(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0);
            L_trans = 5; % Upper bound of disturbance 
            L_rot = 1.5;   % Upper bound of disturbance 

            obj.lambda0_trans = 1.5 * sqrt(L_trans);
            obj.lambda1_trans = 1.1 * L_trans;
            obj.lambda0_rot = 1.5 * sqrt(L_rot);
            obj.lambda1_rot = 1.1 * L_rot;
            obj.eta_trans = zeros(3, 1);
            obj.eta_rot = zeros(3, 1);

            obj.c_p_trans = 1;
            obj.c_dp_trans = 1.2;
            obj.c_q_rot = 0.1;
            obj.c_omega_rot = 0.1;
        end

        function obj = calculateDisturbanceST_trans(obj, u_thrust, p, dp, dt)
            e_p = p - obj.x_hat_trans(1:3);
            e_dp = dp - obj.x_hat_trans(4:6);
            s = obj.c_p_trans * e_p + obj.c_dp_trans * e_dp; 
            omega = obj.lambda0_trans * abs(s).^(1/2) .* sign(s) + obj.eta_trans;
            deta = obj.lambda1_trans * sign(s);
            obj.eta_trans = obj.eta_trans + dt * deta;
            dx_hat = obj.A_trans * obj.x_hat_trans + obj.B_trans * (u_thrust - [0; 0; obj.g * obj.mass_ObserverBase]) + obj.B_trans * omega;
            obj.x_hat_trans = obj.x_hat_trans + dt * dx_hat;
            obj.w_hat_trans = omega;
        end

        function obj = calculateDisturbanceST_rot(obj, u_torque, q, omega, dt)
            q.normalize();
            q_vec = rotvec(q)';
            e_q = q_vec - obj.x_hat_rot(1:3);
            e_omega = omega - obj.x_hat_rot(4:6);
            s = obj.c_q_rot * e_q + obj.c_q_rot * e_omega; 
            omega_st = obj.lambda0_rot * abs(s).^(1/2) .* sign(s) + obj.eta_rot;
            deta = obj.lambda1_rot * sign(s);
            obj.eta_rot = obj.eta_rot + dt * deta;
            dx_hat = obj.A_rot * obj.x_hat_rot + obj.B_rot * (u_torque - cross(omega, obj.J_ObserverBase * omega)) + obj.B_rot * omega_st;
            obj.x_hat_rot = obj.x_hat_rot + dt * dx_hat;
            obj.w_hat_rot = omega_st;
        end
    end
end
