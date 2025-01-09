classdef Kalman < ObserverBase & handle
    properties
        Fk      % State transition matrix (Discrete-time A)
        Bk      % Input matrix (Discrete-time B)
        Hk      % Observation matrix (C)
        Zk      % Measurement vector

        Xk      % State estimate vector
        Rk      % Measurement noise covariance
        Qk      % Process noise covariance
        Pk      % State covariance matrix
        Sk      % Innovation covariance
        Kk      % Kalman gain

        dt      % Sampling time
        firstUpdate % Flag for first update
    end

    methods
        % Constructor
        function obj = Kalman(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0);
            obj.dt = 0.01;      % 0.01

            obj.Fk = eye(6) + obj.A_trans * obj.dt + 0.5 * (obj.A_trans * obj.dt)^2; % Discretized A
            obj.Bk = obj.dt * obj.B_trans; % Discretized B
            obj.Hk = [eye(6)];    % Observation matrix

            obj.Qk = eye(6) * 1.5; % Process noise covariance
            obj.Rk = eye(6) * 25;  % Measurement noise covariance

            obj.Pk = eye(6) * 60; % Large initial uncertainty

            obj.Xk = [p_0; dp_0];

            obj.firstUpdate = true;
        end

        % Kalman filter step
        function obj = kalman_estimate(obj, p, dp, u_thrust)
            X = [p; dp];

            % Handle missing or invalid measurements
            if isempty(p) || all(p == 0)
                obj.Qk = eye(6) * 150; % Increase process noise
                obj.Rk = eye(6) * 1e10; % High measurement noise
                X = obj.Xk; % Use previous estimate
                %disp("MAMA ESCUCHO BORROSO!");
                %disp("Missing data: using model prediction");
            else
                obj.Qk = eye(6) * 1.5; % Moderate process noise
                obj.Rk = eye(6) * 25;  % Moderate measurement noise
                %disp("ia sirbo pero oygo borrozo!");
            end

            %%%%%%%%%%%%%%%%%%%%%%%% PREDICT STEP %%%%%%%%%%%%%%%%%%%%%%%%
            if obj.firstUpdate && all(dp == 0) && all(u_thrust == 0)
                obj.Xk = obj.Fk * obj.Xk;
                obj.firstUpdate = false;
            else
                obj.Xk = obj.Fk * obj.Xk + obj.Bk * (u_thrust - [0; 0; obj.g * obj.mass_ObserverBase]);
            end
            obj.Pk = obj.Fk * obj.Pk * obj.Fk' + obj.Qk;

            %%%%%%%%%%%%%%%%%%%%%%%% UPDATE STEP %%%%%%%%%%%%%%%%%%%%%%%%%
            obj.Zk = obj.Hk * X;    
            Zest = obj.Hk * obj.Xk; 
            Yk = obj.Zk - Zest;     
            obj.Sk = obj.Hk * obj.Pk * obj.Hk' + obj.Rk;
            obj.Kk = obj.Pk * obj.Hk' / obj.Sk;
            obj.Xk = obj.Xk + obj.Kk * Yk;
            obj.Pk = (eye(size(obj.Pk)) - obj.Kk * obj.Hk) * obj.Pk;
        end

        % salida chida
        function [p, dp] = getState(obj)
            p = obj.Xk(1:3);      % Position estimate
            dp = obj.Xk(4:6);     % Velocity estimate
        end
    end
end
