classdef WheelLoadModel
    %WHEELLOADMODEL Static + longitudinal + lateral load transfer.
    %   Wheel order everywhere: [FL FR RL RR].
    %   ax > 0 : accelerating (load moves to the rear)
    %   ay > 0 : turning left (load moves to the right wheels)
    properties (SetAccess = immutable)
        mass (1,1) double
        wheelbase (1,1) double
        trackWidth (1,1) double
        cgFrontFraction (1,1) double
        cgHeight (1,1) double
        gravity (1,1) double
    end
    methods
        function obj = WheelLoadModel(mass, wheelbase, trackWidth, cgFrontFraction, cgHeight, gravity)
            obj.mass = mass;
            obj.wheelbase = wheelbase;
            obj.trackWidth = trackWidth;
            obj.cgFrontFraction = cgFrontFraction;
            obj.cgHeight = cgHeight;
            obj.gravity = gravity;
        end

        function Fz = loads(obj, ax, ay)
            %LOADS Vertical load on each wheel [N], 1x4 [FL FR RL RR].
            f = obj.cgFrontFraction;
            W = obj.mass * obj.gravity;
            dLong = obj.mass * ax * obj.cgHeight / obj.wheelbase;
            dLat  = obj.mass * ay * obj.cgHeight / obj.trackWidth;
            Ffront = f * W - dLong;
            Frear  = (1 - f) * W + dLong;
            dLatF = f * dLat;           % lateral transfer shared like the static weight
            dLatR = (1 - f) * dLat;
            Fz = [Ffront/2 - dLatF, Ffront/2 + dLatF, ...
                  Frear/2  - dLatR, Frear/2  + dLatR];
            Fz = max(Fz, 0);
        end

        function a = maxDriveAccel(obj, muEff, dragAccel)
            %MAXDRIVEACCEL Tyre-limited tractive force / mass [m/s^2] (rear drive, with load transfer).
            %   muEff     : friction left after cornering (friction circle)
            %   dragAccel : drag / mass, which also unloads the transfer term
            if nargin < 3, dragAccel = 0; end
            k = muEff * obj.cgHeight / obj.wheelbase;
            a = (muEff * obj.gravity * (1 - obj.cgFrontFraction) - k * dragAccel) / max(1 - k, 0.1);
            a = max(a, 0);
        end

        function a = maxBrakeDecel(obj, muEff, bias)
            %MAXBRAKEDECEL Largest deceleration [m/s^2] before a brake axle saturates.
            k = muEff * obj.cgHeight / obj.wheelbase;
            f = obj.cgFrontFraction;
            aRear = muEff * obj.gravity * (1 - f) / ((1 - bias) + k);
            dF = bias - k;
            if dF > 0
                aFront = muEff * obj.gravity * f / dF;
            else
                aFront = Inf;
            end
            a = min(aRear, aFront);
        end
    end
end
