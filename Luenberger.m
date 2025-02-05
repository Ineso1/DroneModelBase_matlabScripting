classdef Luenberger < ObserverBase & handle
    properties

        A_ext_trans
        B_ext_trans
        C_ext_trans

        A_ext_rot
        B_ext_rot
        C_ext_rot

        desired_eigenvalues_ext_trans
        desired_eigenvalues_trans

        L_ext_trans
        L_trans

        desired_eigenvalues_ext_rot
        desired_eigenvalues_rot

        L_ext_rot
        L_rot

        x_hat_ext_L_trans
        x_hat_ext_L_rot

    end
    methods
        function obj = Luenberger(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0)

            obj.A_ext_trans = [obj.A_trans, obj.B_trans, zeros(6,3); zeros(3,9), eye(3,3); zeros(3,12)];
            obj.B_ext_trans = [obj.B_trans; zeros(6,3)];
            obj.C_ext_trans = [obj.C_trans, zeros(3,6)];

            obj.A_ext_rot = [obj.A_rot, obj.B_rot, zeros(6,3); zeros(3,9), eye(3,3); zeros(3,12)];
            obj.B_ext_rot = [obj.B_rot; zeros(6,3)];
            obj.C_ext_rot = [obj.C_rot, zeros(3,6)];
            
            obj.desired_eigenvalues_trans = [-0.1, -0.1, -1, -0.5, -0.5, -2];
            obj.desired_eigenvalues_ext_trans = [obj.desired_eigenvalues_trans, -0.7, -0.7, -3, -0.8, -0.8, -4];

            obj.L_trans = place(obj.A_trans', obj.C_trans', obj.desired_eigenvalues_trans)';
            obj.L_ext_trans = place(obj.A_ext_trans', obj.C_ext_trans', obj.desired_eigenvalues_ext_trans)';

            obj.desired_eigenvalues_rot = [-0.7, -0.7, -0.7, -0.8, -0.8, -0.8];
            obj.desired_eigenvalues_ext_rot = [obj.desired_eigenvalues_rot, -0.9, -0.9, -0.9, -0.95, -0.95, -0.95];

            obj.L_rot = place(obj.A_rot', obj.C_rot', obj.desired_eigenvalues_rot)';
            obj.L_ext_rot = place(obj.A_ext_rot', obj.C_ext_rot', obj.desired_eigenvalues_ext_rot)';

            obj.x_hat_ext_L_trans = zeros(12,1);
            obj.x_hat_ext_L_rot = zeros(12,1);
        end

        function obj = calculateDisturbanceL_trans(obj, u_thrust, p, dp, dt)
            x_t = [p; dp; zeros(6, 1)];
            y = obj.C_ext_trans * x_t;
            obj.dx_hat_trans = obj.A_ext_trans * obj.x_hat_ext_L_trans + obj.B_ext_trans * (u_thrust - [0; 0; obj.g * obj.mass_ObserverBase]) + obj.L_ext_trans * (y - obj.C_ext_trans * obj.x_hat_ext_L_trans);
            obj.x_hat_ext_L_trans = obj.x_hat_ext_L_trans + dt * obj.dx_hat_trans;
            obj.w_hat_trans = obj.x_hat_ext_L_trans(7:9);
        end

        function obj = calculateDisturbanceL_rot(obj, u_torque, q, omega, dt)
            q.normalize();
            q_vec = rotvec(q)';
            x_r = [q_vec; omega; zeros(6, 1)];
            % [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(q);
            % x_r = [q_vec_1; q_vec_2; q_vec_3; omega; zeros(6, 1)];
            y = obj.C_ext_rot * x_r;
            obj.dx_hat_rot = obj.A_ext_rot * obj.x_hat_ext_L_rot + obj.B_ext_rot * (u_torque - cross(omega, obj.J_ObserverBase * omega)) + obj.L_ext_rot * (y - obj.C_ext_rot * obj.x_hat_ext_L_rot);
            obj.x_hat_ext_L_rot = obj.x_hat_ext_L_rot + dt * obj.dx_hat_rot;
            obj.w_hat_rot = obj.x_hat_ext_L_rot(7:9);
        end

    end
end