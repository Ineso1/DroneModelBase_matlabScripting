classdef SoftTrajectoryGenerator < handle
    properties
        waypoints = [];
        trajectorySegments = [];
        currentTime = 0;
        currentSegmentIndex = 1; 
    end
    
    methods
        function obj = SoftTrajectoryGenerator()
            obj.currentTime = 0;
            obj.currentSegmentIndex = 1;
        end
        
        function addWaypoint(obj, position, time)
            obj.waypoints = [obj.waypoints; struct('position', position, 'time', time)];
        end
        
        function generateTrajectories(obj)
            obj.trajectorySegments = [];
            numWaypoints = length(obj.waypoints);
            for i = 1:numWaypoints-1
                wp0 = obj.waypoints(i);
                wp1 = obj.waypoints(i+1);
                
                coeffs_x = obj.computeQuinticCoefficients(wp0.time, wp1.time, wp0.position(1), wp1.position(1), 0, 0, 0, 0);
                coeffs_y = obj.computeQuinticCoefficients(wp0.time, wp1.time, wp0.position(2), wp1.position(2), 0, 0, 0, 0);
                coeffs_z = obj.computeQuinticCoefficients(wp0.time, wp1.time, wp0.position(3), wp1.position(3), 0, 0, 0, 0);
                
                seg = struct('coeffs_x', coeffs_x, 'coeffs_y', coeffs_y, ...
                             'coeffs_z', coeffs_z, 'startTime', wp0.time, 'endTime', wp1.time);
                obj.trajectorySegments = [obj.trajectorySegments; seg];
            end
        end
        
        function [position, velocity] = getNextState(obj, dt)
            obj.currentTime = obj.currentTime + dt;
            position = [0; 0; 0];
            velocity = [0; 0; 0];
            
            for i = obj.currentSegmentIndex:length(obj.trajectorySegments)
                seg = obj.trajectorySegments(i); 
                if obj.currentTime >= seg.startTime && obj.currentTime <= seg.endTime
                    obj.currentSegmentIndex = i;
                    t = obj.currentTime;
                    
                    position(1) = obj.evaluatePolynomial(seg.coeffs_x, t);
                    position(2) = obj.evaluatePolynomial(seg.coeffs_y, t);
                    position(3) = obj.evaluatePolynomial(seg.coeffs_z, t);
                    
                    vel_x = polyder(seg.coeffs_x);
                    vel_y = polyder(seg.coeffs_y);
                    vel_z = polyder(seg.coeffs_z);
                    
                    velocity(1) = obj.evaluatePolynomial(vel_x, t);
                    velocity(2) = obj.evaluatePolynomial(vel_y, t);
                    velocity(3) = obj.evaluatePolynomial(vel_z, t);
                    return;
                end
            end
        end
        
        function coeffs = computeQuinticCoefficients(~, t0, tf, p0, pf, v0, vf, a0, af)
            A = [1, t0, t0^2, t0^3, t0^4, t0^5;
                 1, tf, tf^2, tf^3, tf^4, tf^5;
                 0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                 0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                 0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
                 0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
            b = [p0; pf; v0; vf; a0; af];
            coeffs = A\b;
            coeffs = flipud(coeffs); 
        end
        
        function val = evaluatePolynomial(~, coeffs, t)
            val = polyval(coeffs, t);
        end
    end
end
