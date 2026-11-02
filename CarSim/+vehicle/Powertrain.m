classdef Powertrain
    %POWERTRAIN Torque / power limited traction, single fixed gear ratio.
    properties (SetAccess = immutable)
        maxTorque (1,1) double   % N*m (motor)
        gearRatio (1,1) double   % -
        maxPower (1,1) double    % W
        wheelRadius (1,1) double % m
    end
    properties (Constant)
        minSpeed = 1             % m/s, avoids Pmax/v -> Inf at standstill
    end
    methods
        function obj = Powertrain(maxTorque, gearRatio, maxPower, wheelRadius)
            obj.maxTorque = maxTorque;
            obj.gearRatio = gearRatio;
            obj.maxPower = maxPower;
            obj.wheelRadius = wheelRadius;
        end

        function F = maxForce(obj, v)
            %MAXFORCE Maximum traction force at the wheels [N].
            Ftorque = obj.maxTorque * obj.gearRatio / obj.wheelRadius;
            Fpower = obj.maxPower / max(v, obj.minSpeed);
            F = min(Ftorque, Fpower);
        end

        function [Fdrive, motorTorque, wheelTorque] = apply(obj, Frequest, v)
            %APPLY Saturate a requested traction force (>= 0).
            Fdrive = min(max(Frequest, 0), obj.maxForce(v));
            [motorTorque, wheelTorque] = obj.torquesFromForce(Fdrive);
        end

        function [motorTorque, wheelTorque] = torquesFromForce(obj, F)
            wheelTorque = F * obj.wheelRadius;
            motorTorque = wheelTorque / obj.gearRatio;
        end
    end
end
