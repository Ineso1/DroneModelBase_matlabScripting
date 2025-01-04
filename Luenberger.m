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
            
            obj.desired_eigenvalues_trans = [-5, -5, -5, -6, -6, -6];
            obj.desired_eigenvalues_ext_trans = [obj.desired_eigenvalues_trans, -7, -7, -7, -8, -8, -8];

            obj.L_trans = place(obj.A_trans', obj.C_trans', obj.desired_eigenvalues_trans)';
            obj.L_ext_trans = place(obj.A_ext_trans', obj.C_ext_trans', obj.desired_eigenvalues_ext_trans)';

            obj.desired_eigenvalues_rot = [-5, -5, -5, -6, -6, -6];
            obj.desired_eigenvalues_ext_rot = [obj.desired_eigenvalues_rot, -7, -7, -7, -8, -8, -8];

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
            q_vec = [q.q1; q.q2; q.q3; q.q4];  % Include q4 (scalar part of the quaternion)
            x_r = [q_vec; omega; zeros(6, 1)];
            y = obj.C_ext_rot * x_r;
            obj.dx_hat_rot = obj.A_ext_rot * obj.x_hat_ext_L_rot + obj.B_ext_rot * u_torque + obj.L_ext_rot * (y - obj.C_ext_rot * obj.x_hat_ext_L_rot);
            obj.x_hat_ext_L_rot = obj.x_hat_ext_L_rot + dt * obj.dx_hat_rot;
            obj.w_hat_rot = obj.x_hat_ext_L_rot(7:9);
        end

    end
end