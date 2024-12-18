classdef ObserverBase < handle
    properties
        g

        mass_ObserverBase
        J_ObserverBase

        A_trans
        B_trans
        C_trans

        A_rot
        B_rot
        C_rot

        x_hat_trans
        x_hat_rot
        
        dx_hat_trans
        dx_hat_rot

        w_hat_trans
        w_hat_rot

        x_t
        x_r
    end
    methods
        function obj = ObserverBase(mass, J, p_0, dp_0, q_0, omega_0)
            obj.g = 9.81;
            obj.mass_ObserverBase = mass;
            obj.J_ObserverBase = J;
            obj.A_trans = [zeros(3,3), eye(3); zeros(3,6)];  % 6x6 matrix
            obj.B_trans = [zeros(3,3); eye(3)/obj.mass_ObserverBase];     % 6x3 matrix
            obj.C_trans = [eye(3), zeros(3,3)]; 
            obj.A_rot = [zeros(3,3), eye(3); zeros(3,6)];    % 6x6 matrix
            obj.B_rot = [zeros(3,3); inv(J)];            % 6x3 matrix
            obj.C_rot = [eye(3), zeros(3,3)];      
            obj.x_hat_trans = [0; 0; 0; 0; 0; 0;];
            obj.x_hat_rot = [0; 0; 0; 0; 0; 0;];
            obj.w_hat_trans = zeros(3,1);
            obj.w_hat_rot = zeros(3,1);
            obj.x_t = [p_0; dp_0];
            [q_vec_0, q_vec_1, q_vec_2, q_vec_3] = parts(q_0);
            obj.x_r = [q_vec_1; q_vec_2; q_vec_3; omega_0];
        end
    end
end