classdef UDE < ObserverBase & handle
    properties
        B_pinv_trans
        Omega_UDE_trans
        xi_UDE_trans

        B_pinv_rot
        Omega_UDE_rot
        xi_UDE_rot
    end
    methods
        function obj = UDE(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0)
            obj.B_pinv_trans = pinv(obj.B_trans);            
            obj.Omega_UDE_trans = diag([10, 10, 10]);
            obj.xi_UDE_trans = -obj.Omega_UDE_trans * (obj.B_pinv_trans * obj.x_t);

            obj.B_pinv_rot = pinv(obj.B_rot);               
            obj.Omega_UDE_rot = diag([10, 10, 10]);
            obj.xi_UDE_rot = -obj.Omega_UDE_rot * (obj.B_pinv_rot * obj.x_r);
        end

        function obj = calculateStateUDE_trans(obj, u_thrust, p, dp, dt)
            x_t = [p; dp];
            xi_dot_trans = -obj.Omega_UDE_trans * obj.xi_UDE_trans - (obj.Omega_UDE_trans^2 * (obj.B_pinv_trans * x_t) + obj.Omega_UDE_trans * (obj.B_pinv_trans * obj.A_trans * x_t)) - obj.Omega_UDE_trans * (u_thrust - [0; 0; obj.g * obj.mass_ObserverBase]);
            obj.xi_UDE_trans = obj.xi_UDE_trans + dt * xi_dot_trans;
            obj.w_hat_trans = obj.xi_UDE_trans + obj.Omega_UDE_trans * obj.B_pinv_trans * x_t;
            obj.dx_hat_trans = obj.A_trans * x_t + obj.B_trans * u_thrust + obj.B_trans * obj.w_hat_trans;
            obj.x_hat_trans = obj.x_hat_trans + dt * obj.dx_hat_trans;
        end

        function obj = calculateStateUDE_rot(obj, u_torque, q, omega, dt)
            %obj.domega = obj.J\(obj.tau - cross(obj.omega, obj.J * obj.omega)) + obj.disturbance_rot;
            controlVar_test = obj.J_ObserverBase * u_torque + cross(omega, obj.J_ObserverBase * omega);
            [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(q);
            x_r = [q_vec_1; q_vec_2; q_vec_3; omega];
            xi_dot_rot = -obj.Omega_UDE_rot * obj.xi_UDE_rot - (obj.Omega_UDE_rot^2 * (obj.B_pinv_rot * x_r) + obj.Omega_UDE_rot * (obj.B_pinv_rot * obj.A_rot * x_r)) - obj.Omega_UDE_rot * (controlVar_test);
            obj.xi_UDE_rot = obj.xi_UDE_rot + dt * xi_dot_rot;
            obj.w_hat_rot = obj.xi_UDE_rot + obj.Omega_UDE_rot * obj.B_pinv_rot * x_r;
            obj.dx_hat_rot = obj.A_rot * x_r + obj.B_rot * controlVar_test + obj.B_rot * obj.w_hat_rot;
            obj.x_hat_rot = obj.x_hat_rot + dt * obj.dx_hat_rot;
        end
    end
end