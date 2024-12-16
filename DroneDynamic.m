classdef DroneDynamic < DroneDataExtention
    properties
        g
        mass
        J

        % Translational dynamics
        p
        dp
        ddp

        % Rotational dynamics
        q
        dq
        omega
        domega

        % Control
        F_bf
        tau

        % State space estimation
        dx_sys_trans
        dx_sys_rot

        % General simulation variables
        dt
        iterations
        disturbance_trans
        disturbance_rot

    end
    methods
        function obj = DroneDynamic(mass, q, x0, y0, z0, dt)
            obj@DroneDataExtention();
            if nargin >= 1
                obj.mass = mass;
                obj.q = q;
                obj.p = [x0; y0; z0];
                obj.dt = dt;
            else
                obj.mass = 0.405;
                obj.q = quaternion(1, 0, 0, 0);
                obj.p = [0; 0; 0];
                obj.dt = 0.001;
            end
            obj.J = [2098e-6, 63.577538e-6, -2.002648e-6; 
                    63.577538e-6, 2102e-6, 0.286186e-6; 
                    -2.002648e-6, 0.286186e-6, 4068e-6];

            obj.dp = [0; 0; 0];
            obj.ddp = [0; 0; 0];
            obj.dq = quaternion(1, 0, 0, 0);

            obj.omega = [0; 0; 0;];
            obj.domega = [0; 0; 0;];
            obj.iterations = 0;

            obj.tau = [0; 0; 0;];
            obj.F_bf = 0;
            
            obj.g = 9.81;
            obj.disturbance_trans = [0; 0; 0];
            obj.disturbance_rot = [0; 0; 0];

            obj.dx_sys_trans = [0; 0; 0; 0; 0; 0;];
            obj.dx_sys_rot = [0; 0; 0; 0; 0; 0;];

            obj = obj.updateDisturbanceArray(obj.disturbance_trans, obj.disturbance_rot);
        end

        function obj = updateState(obj)

            % dot{X}
            [somethingWeDontCareAbout, ax, ay, az] = parts(obj.q * quaternion( 0, 0, 0, obj.F_bf/obj.mass) * (obj.q'));
            obj.ddp = [ax; ay; az] + obj.disturbance_trans;
            obj.dp = obj.dp + ([0; 0; -obj.g] +  obj.ddp) * obj.dt;
            obj.domega = obj.J\(obj.tau - cross(obj.omega, obj.J * obj.omega)) + obj.disturbance_rot;
            obj.dq = 0.5*quaternion([0,obj.omega'])*obj.q;
            obj.dx_sys_trans = [obj.dp; obj.ddp - obj.disturbance_trans];
            [dq_vec_0, dq_vec_1, dq_vec_2, dq_vec_3] = parts(obj.dq);
            obj.dx_sys_rot = [dq_vec_1; dq_vec_2; dq_vec_3; obj.domega - obj.disturbance_rot];

            % X
            obj.p = obj.p + obj.dp * obj.dt + 0.5 * obj.ddp * obj.dt^2;
            obj.q = obj.q + obj.dq * obj.dt;
            obj.q = normalize(obj.q);
            obj.omega = obj.omega + obj.domega * obj.dt;
        end
        
    end
end