classdef DroneModel
    properties
        rotor_distance % Distance from center to rotors
        rotor_size     % Size of the rotors
        rotor_colors   % Colors for the rotors
        center_color   % Color for the drone's center
        position       % Position of the drone
        orientation    % Quaternion orientation
    end
    
    methods
        function obj = DroneModel(rotor_distance, rotor_size)
            if nargin == 0
                rotor_distance = 0.3;
                rotor_size = 0.08;
            end
            obj.rotor_distance = rotor_distance;
            obj.rotor_size = rotor_size;
            obj.rotor_colors = {'r', 'b', 'm', 'c'};
            obj.center_color = 'g';
            obj.position = [0; 0; 0]; 
            obj.orientation = quaternion(1, 0, 0, 0);
        end
        
        % Method to set the drone's position
        function obj = setPosition(obj, new_position)
            obj.position = new_position;
        end
        
        % Method to set the drone's orientation (as a quaternion)
        function obj = setOrientation(obj, new_orientation)
            obj.orientation = new_orientation;
        end
        
        % Method to return rotor positions and center for external use
        function [global_rotor_positions, center_position] = getDroneData(obj)
            local_rotor_positions = [
                obj.rotor_distance,  0,              0;  % Rotor 1 (right)
                -obj.rotor_distance, 0,              0;  % Rotor 2 (left)
                0,              obj.rotor_distance,  0;  % Rotor 3 (front)
                0,             -obj.rotor_distance,  0;  % Rotor 4 (back)
            ];
            
            rotated_rotor_positions = rotateframe(obj.orientation, local_rotor_positions);
            global_rotor_positions = rotated_rotor_positions' + obj.position; % Transpose to get 4x3 format
            center_position = obj.position;
        end
    end
end
