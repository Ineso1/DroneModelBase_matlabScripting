classdef DroneDataExtention
    properties
        % Errors
        ep_array
        eq_array

        % Dynamics
        p_array
        dp_array
        ddp_array
        q_array
        dq_array 
        omega_array
        domega_array

        % Aim
        p_d_array
        q_d_array

        % Model
        drone_model
        
        % Sim var
        time_array

        % Disturbance
        disturbance_trans_array
        disturbance_rot_array
        disturbance_measure_trans
        disturbance_measure_rot

        % State
        dx_state_trans
        dx_state_hat_trans
        dx_state_rot
        dx_state_hat_rot

    end
    methods
        function obj = DroneDataExtention()
            obj.ep_array = [];
            obj.eq_array = [];
            obj.p_array = [];
            obj.dp_array = [];
            obj.ddp_array = [];
            obj.q_array = [];
            obj.dq_array = [];
            obj.omega_array = [];
            obj.domega_array = [];
            obj.p_d_array = [];
            obj.q_d_array = [];
            obj.drone_model = DroneModel();
            obj.time_array = [];
            obj.disturbance_trans_array = [];
            obj.disturbance_rot_array = [];
            obj.disturbance_measure_trans = [];
            obj.disturbance_measure_rot = [];
            obj.dx_state_trans = [];
            obj.dx_state_hat_trans = [];
            obj.dx_state_rot = [];
            obj.dx_state_hat_rot = [];
        end

        function obj = updateDisturbanceArray(obj, disturbance_trans, disturbance_rot)
            obj.disturbance_trans_array(:, end+1) = disturbance_trans; % Append disturbance for each time step
            obj.disturbance_rot_array(:, end+1) = disturbance_rot;
        end

        function ep_array = getPositionErrorArray(obj)
            ep_array = obj.ep_array;
        end

        function eq_array = getOrientationErrorArray(obj)
            eq_array = obj.eq_array;
        end
        
        function p_array = getPositionArray(obj)
            p_array = obj.p_array;
        end

        function plotPosition(obj)
            figure;
            plot3(obj.p_array(1,:), obj.p_array(2,:), obj.p_array(3,:), 'bo-');
            title('Drone Position over Time');
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            grid on;
        end
        
        function plotOrientation(obj)
            orientation_euler = eulerd(quaternion(obj.q_array(1,:), obj.q_array(2,:), obj.q_array(3,:), obj.q_array(4,:)), 'ZYX', 'frame');
            figure;
            plot(orientation_euler);
            title('Drone Orientation (Euler Angles) over Time');
            xlabel('Time Step');
            ylabel('Angle (degrees)');
            legend('Yaw', 'Pitch', 'Roll');
            grid on;
        end

        function plotErrors(obj)
            figure;
            subplot(2,1,1);
            plot(obj.time_array, obj.ep_array(1,:), 'r-', 'DisplayName', 'Error X');
            hold on;
            plot(obj.time_array, obj.ep_array(2,:), 'g-', 'DisplayName', 'Error Y');
            plot(obj.time_array, obj.ep_array(3,:), 'b-', 'DisplayName', 'Error Z');
            title('Position Error over Time');
            xlabel('Time');
            ylabel('Position Error');
            legend('show');
            grid on;

            subplot(2,1,2);
            plot(obj.time_array, obj.eq_array(1,:), 'r-', 'DisplayName', 'Error Yaw');
            hold on;
            plot(obj.time_array, obj.eq_array(2,:), 'g-', 'DisplayName', 'Error Pitch');
            plot(obj.time_array, obj.eq_array(3,:), 'b-', 'DisplayName', 'Error Roll');
            title('Orientation Error over Time');
            xlabel('Time');
            ylabel('Orientation Error');
            legend('show');
            grid on;
        end

        function plotObserverStates_trans(obj)
            figure;
            subplot(3,1,1);
            title('dp over time');
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(1,:), 'r--', 'DisplayName', 'Raw State dx');
            plot(obj.time_array, obj.dp_array(1,:), 'k', 'DisplayName', 'Real State dx');
            plot(obj.time_array, obj.dx_state_hat_trans(1,:), 'g-', 'DisplayName', 'UDE state dx');
            legend('show');
            subplot(3,1,2);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(2,:), 'r--', 'DisplayName', 'Raw State dy');
            plot(obj.time_array, obj.dp_array(2,:), 'k', 'DisplayName', 'Real State dy');
            plot(obj.time_array, obj.dx_state_hat_trans(2,:), 'g-', 'DisplayName', 'UDE state dy');
            legend('show');
            subplot(3,1,3);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(3,:), 'r--', 'DisplayName', 'Raw State dz');
            plot(obj.time_array, obj.dp_array(3,:), 'k', 'DisplayName', 'Real State dz');
            plot(obj.time_array, obj.dx_state_hat_trans(3,:), 'g-', 'DisplayName', 'UDE state dz');
            legend('show');

            figure;
            subplot(3,1,1);
            title('ddp over time');
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(4,:), 'r--', 'DisplayName', 'Raw State ddx');
            plot(obj.time_array, obj.ddp_array(1,:), 'k', 'DisplayName', 'Real State ddx');
            plot(obj.time_array, obj.dx_state_hat_trans(4,:), 'g-', 'DisplayName', 'UDE state ddx');
            legend('show');
            subplot(3,1,2);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(5,:), 'r--', 'DisplayName', 'Raw State ddy');
            plot(obj.time_array, obj.ddp_array(2,:), 'k', 'DisplayName', 'Real State ddy');
            plot(obj.time_array, obj.dx_state_hat_trans(5,:), 'g-', 'DisplayName', 'UDE state ddy');
            legend('show');
            subplot(3,1,3);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_trans(6,:), 'r--', 'DisplayName', 'Raw State ddz');
            plot(obj.time_array, obj.ddp_array(3,:), 'k', 'DisplayName', 'Real State ddz');
            plot(obj.time_array, obj.dx_state_hat_trans(6,:), 'g-', 'DisplayName', 'UDE state ddz');
            legend('show');
        end

        function plotObserverStates_rot(obj)
            figure;
            subplot(3,1,1);
            title('dq over time');
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(1,:), 'r--', 'DisplayName', 'Raw State dq1');
            plot(obj.time_array, obj.dq_array(1,:), 'k', 'DisplayName', 'Real State dq1');
            plot(obj.time_array, obj.dx_state_hat_rot(1,:), 'g-', 'DisplayName', 'UDE state dq1');
            legend('show');
            subplot(3,1,2);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(2,:), 'r--', 'DisplayName', 'Raw State dq2');
            plot(obj.time_array, obj.dq_array(2,:), 'k', 'DisplayName', 'Real State dq2');
            plot(obj.time_array, obj.dx_state_hat_rot(2,:), 'g-', 'DisplayName', 'UDE state dq2');
            legend('show');
            subplot(3,1,3);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(3,:), 'r--', 'DisplayName', 'Raw State dq3');
            plot(obj.time_array, obj.dq_array(3,:), 'k', 'DisplayName', 'Real State dq3');
            plot(obj.time_array, obj.dx_state_hat_rot(3,:), 'g-', 'DisplayName', 'UDE state dq3');
            legend('show');

            figure;
            subplot(3,1,1);
            title('domega over time');
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(4,:), 'r--', 'DisplayName', 'Raw State domegaX');
            plot(obj.time_array, obj.domega_array(1,:), 'k', 'DisplayName', 'Real State domegaX');
            plot(obj.time_array, obj.dx_state_hat_rot(4,:), 'g-', 'DisplayName', 'UDE state domegaX');
            legend('show');
            subplot(3,1,2);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(5,:), 'r--', 'DisplayName', 'Raw State domegaY');
            plot(obj.time_array, obj.domega_array(2,:), 'k', 'DisplayName', 'Real State domegaY');
            plot(obj.time_array, obj.dx_state_hat_rot(5,:), 'g-', 'DisplayName', 'UDE state domegaY');
            legend('show');
            subplot(3,1,3);
            xlabel('Time (s)');
            ylabel('state');
            grid on;
            hold on;
            plot(obj.time_array, obj.dx_state_rot(6,:), 'r--', 'DisplayName', 'Raw State domegaZ');
            plot(obj.time_array, obj.domega_array(3,:), 'k', 'DisplayName', 'Real State domegaZ');
            plot(obj.time_array, obj.dx_state_hat_rot(6,:), 'g-', 'DisplayName', 'UDE state domegaZ');
            legend('show');
        end
        
        function animateDroneTrajectory(obj)
            figure;
            hold on;
            grid on;
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            title('sfnjansfkjaskjfnkjsanfakjsnkj');
            axis equal;
            view(3);  % isometric
            plot3(obj.p_array(1,:), obj.p_array(2,:), obj.p_array(3,:), 'k--');
            [xs, ys, zs] = sphere;
            drone_center_surf = surf(xs * 0.05, ys * 0.05, zs * 0.05, 'EdgeColor', 'none', 'FaceColor', 'g');
            rotor_cylinders = gobjects(1, 4);
            rotor_caps = gobjects(2, 4);  % For top and bottom 
            rotor_radius = 0.05;
            rotor_height = 0.05; 
            [xc, yc, zc] = cylinder(rotor_radius);
            
            rotor_colors = {'r', 'b', 'm', 'c'};
            for i = 1:4
                rotor_cylinders(i) = surf(xc, yc, zc * rotor_height, 'EdgeColor', 'none', 'FaceColor', rotor_colors{i});
                rotor_caps(1, i) = fill3(xc(1,:), yc(1,:), zc(1,:) * rotor_height, rotor_colors{i}, 'EdgeColor', 'none');
                rotor_caps(2, i) = fill3(xc(2,:), yc(2,:), zc(2,:) * rotor_height, rotor_colors{i}, 'EdgeColor', 'none');
            end
            
            arm_lines = gobjects(1, 2);
            for i = 1:2
                arm_lines(i) = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);
            end
            
            for i = 1:length(obj.time_array)
                drone_position = obj.p_array(:, i);
                drone_orientation = quaternion(obj.q_array(1,i), obj.q_array(2,i), obj.q_array(3,i), obj.q_array(4,i));
                obj.drone_model = obj.drone_model.setPosition(drone_position);
                obj.drone_model = obj.drone_model.setOrientation(drone_orientation);
                [rotor_positions, center_position] = obj.drone_model.getDroneData();
                set(drone_center_surf, 'XData', xs * 0.05 + center_position(1), 'YData', ys * 0.05 + center_position(2), 'ZData', zs * 0.05 + center_position(3));
                for j = 1:4
                    set(rotor_cylinders(j), 'XData', xc + rotor_positions(1, j), 'YData', yc + rotor_positions(2, j), 'ZData', zc * rotor_height + rotor_positions(3, j));
                    set(rotor_caps(1, j), 'XData', xc(1,:) + rotor_positions(1, j), 'YData', yc(1,:) + rotor_positions(2, j), 'ZData', zc(1,:) * rotor_height + rotor_positions(3, j));
                    
                    set(rotor_caps(2, j), 'XData', xc(2,:) + rotor_positions(1, j), 'YData', yc(2,:) + rotor_positions(2, j), 'ZData', zc(2,:) * rotor_height + rotor_positions(3, j));
                end
                set(arm_lines(1), 'XData', [rotor_positions(1, 1), rotor_positions(1, 2)], 'YData', [rotor_positions(2, 1), rotor_positions(2, 2)], 'ZData', [rotor_positions(3, 1), rotor_positions(3, 2)]);
                set(arm_lines(2), 'XData', [rotor_positions(1, 3), rotor_positions(1, 4)], 'YData', [rotor_positions(2, 3), rotor_positions(2, 4)], 'ZData', [rotor_positions(3, 3), rotor_positions(3, 4)]);
                pause(0.05);
            end
            hold off;
        end

        function plotDisturbance_trans(obj)
            figure;
            plot(obj.time_array, obj.disturbance_trans_array(1,:), 'r-', 'DisplayName', 'Disturbance X');
            hold on;
            plot(obj.time_array, obj.disturbance_trans_array(2,:), 'g-', 'DisplayName', 'Disturbance Y');
            plot(obj.time_array, obj.disturbance_trans_array(3,:), 'b-', 'DisplayName', 'Disturbance Z');

            plot(obj.time_array, obj.disturbance_measure_trans(1,:), 'r*-', 'DisplayName', 'Disturbance Measured X');
            plot(obj.time_array, obj.disturbance_measure_trans(2,:), 'g*-', 'DisplayName', 'Disturbance Measured Y');
            plot(obj.time_array, obj.disturbance_measure_trans(3,:), 'b*-', 'DisplayName', 'Disturbance Measured Z');
            title('Disturbance trans Over Time');
            xlabel('Time (s)');
            ylabel('Disturbance');
            legend('show');
            grid on;
        end

        function plotDisturbance_rot(obj)
            figure;
            plot(obj.time_array, obj.disturbance_rot_array(1,:), 'r-', 'DisplayName', 'Disturbance X');
            hold on;
            plot(obj.time_array, obj.disturbance_rot_array(2,:), 'g-', 'DisplayName', 'Disturbance Y');
            plot(obj.time_array, obj.disturbance_rot_array(3,:), 'b-', 'DisplayName', 'Disturbance Z');

            plot(obj.time_array, obj.disturbance_measure_rot(1,:), 'r*-', 'DisplayName', 'Disturbance Measured X');
            plot(obj.time_array, obj.disturbance_measure_rot(2,:), 'g*-', 'DisplayName', 'Disturbance Measured Y');
            plot(obj.time_array, obj.disturbance_measure_rot(3,:), 'b*-', 'DisplayName', 'Disturbance Measured Z');
            title('Disturbance Rot Over Time');
            xlabel('Time (s)');
            ylabel('Disturbance');
            legend('show');
            grid on;
        end
    end
end