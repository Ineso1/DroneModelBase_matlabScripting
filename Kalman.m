classdef Kalman < ObserverBase & handle
    properties

    Fk      % State vector  = A
    Bk      % Input Matrix = B
    Uk      % input vector = U
    Hk      % H_k = C = Observation Matrix
    Zk      % measurement

    Xk      % State Estimate  ------------>Xk(1) = X(1)
    Rk      % Variance associated
    Qk      % Covariance associated with noise 
    Pk      % Covariance matrix
    Sk      % Innovation covariance
    Kk      % Kalman gain


end

    methods
        function obj = Kalman(mass, J, p_0, dp_0, q_0, omega_0)
            obj@ObserverBase(mass, J, p_0, dp_0, q_0, omega_0);
            obj.Fk = obj.A_trans;
            obj.Bk = obj.B_trans;

            % Initialize control vars and consts
            obj.Pk = eye(6)*10;
            obj.Qk = eye(6)*1.5;
            obj.Rk= eye(6)*25;
            obj.Hk = eye(6);
            obj.Xk;
        end

        function obj = KalmanFC(X,Uk, i)
        %%%%%%%%%%%%%%%%%%%%%%%% Extended Kalman Filter %%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%% In case of losing data %%%%%%%%%%%%%%%%%%%%%%%%%%%
            if X(:,i)==0
                %Covariance associated with the noise
                obj.Qk = [1 0 0;0 1 0;0 0 1]*150;
                %Variance associated
                obj.Rk = [1 0 0;0 1 0;0 0 1]*1e10;
            else
                %Covariance associated with the noise
                obj.Qk = eye(6)*15;
                %Variance associated
                obj.Rk = eye(3)*20;
            end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % %Covariance associated with the noise
            % obj.Qk = [1 0 0;0 1 0;0 0 1]*15;
            % %Variance associated
            % obj.Rk = [1 0 0;0 1 0;0 0 1]*20;
            % 
            % IF Qk >> Rk The filter relies more on measurements and less on model predictions.
            % IF Rk >> Qk The filter relies more on model predictions and less on measurements.

            % Qk > Rk
            %   If my uncertainty in my model at time K > uncertainty in my
            %   measurement at time K, my measurement will be trusted more than my
            %   model.

            % Qk < Rk
            %   If my uncertainty in my model at time K < uncertainty in my
            %   measurement at time K, the model will be trusted more than the measurement.

        %%%%%%%%%%%%%%%%%%%%% Update state from X,Y,Theta %%%%%%%%%%%%%%%%%%%%%%%%%
            %Update the next X
            obj.Xk(:,i+1) = obj.Fk * X(:,i) + obj.Bk * Uk;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PREDICT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Predicted estimate covariance
            obj.Pk = obj.Fk*obj.Pk*obj.Fk' + obj.Qk;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Keep measurement
            Zk = Hk*X(:,i);
            %Keep value estimated from X,Y,Theta
            Zest = Hk*obj.Xk(:,i+1);
            %The estimated measure taking into account the prediction
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Innovation on measurement pre-fit residual
            Yk = Zk - Zest;  % La diferencia completa de las observaciones
            %Innovation covariance
            Sk = Hk*obj.Pk*Hk'+obj.Rk;
            %Optimal Kalman gain
            Kk = obj.Pk*Hk'*inv(Sk);
            %Update state estimate
            obj.Xk(:,i+1) = Xk(:,i+1) + Kk*Yk;
            %Updated estimated covariance
            obj.Pk = (eye(length(Hk)) - Kk*Hk)*obj.Pk;

            %
            sigmax(i)=sqrt(Pk(1,1));
            obj@ObserverBase()


        end

    end
end