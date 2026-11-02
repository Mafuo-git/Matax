classdef SpeedProfile < handle
    %SPEEDPROFILE Quasi-static target speed along the lap.
    %   1. corner limit      v <= sqrt(mu_lat * g / |kappa|)
    %   2. forward pass      power / torque / rear grip (with load transfer) / drag
    %   3. backward pass     brake force / axle grip (with load transfer)
    %   The physics objects are the ones used by the vehicle, so the profile
    %   follows any change of the car setup.

    properties (SetAccess = private)
        s (:,1) double
        v (:,1) double             % target speed [m/s]
        a (:,1) double             % target acceleration v*dv/ds [m/s^2]
        ds (1,1) double
        totalLength (1,1) double
        n (1,1) double
        lateralMargin (1,1) double % fraction of mu used in corners
        brakeMargin (1,1) double   % fraction of the braking limit used
        vCap (1,1) double = 100    % m/s
    end

    properties (Access = private)
        mass, grip, g
        powertrain, brakes, aero, wheelLoads
    end

    methods
        function obj = SpeedProfile(trk, params, powertrain, brakes, aero, wheelLoads, lateralMargin, brakeMargin)
            if nargin < 7, lateralMargin = 0.70; end
            if nargin < 8, brakeMargin = 0.90; end
            obj.mass = params.mass;
            obj.grip = params.grip;
            obj.g = wheelLoads.gravity;
            obj.powertrain = powertrain;
            obj.brakes = brakes;
            obj.aero = aero;
            obj.wheelLoads = wheelLoads;
            obj.lateralMargin = lateralMargin;
            obj.brakeMargin = brakeMargin;
            obj.compute(trk);
        end

        function vt = targetSpeed(obj, sq)
            vt = obj.sample(obj.v, sq);
        end

        function at = targetAccel(obj, sq)
            at = obj.sample(obj.a, sq);
        end

        function F = feedforwardForce(obj, sq, vNow)
            %FEEDFORWARDFORCE Force needed to follow the profile (> 0 traction, < 0 brake).
            F = obj.mass * obj.sample(obj.a, sq) + obj.aero.drag(vNow);
        end
    end

    methods (Access = private)
        function compute(obj, trk)
            N = trk.n;
            ds = trk.ds;
            kappa = abs(trk.curvature);
            muLat = obj.lateralMargin * obj.grip;

            vLat = min(sqrt(muLat * obj.g ./ max(kappa, 1e-9)), obj.vCap);

            % start at the slowest point: its speed is certainly the corner limit
            [~, i0] = min(vLat);
            rot = circshift((1:N)', -(i0 - 1));
            kr = kappa(rot);
            vv = vLat(rot);

            for i = 1:N                                   % forward pass (acceleration)
                j = i + 1; if j > N, j = 1; end
                a = obj.accelMax(vv(i), kr(i));
                vn = sqrt(max(vv(i)^2 + 2 * a * ds, 0));
                vv(j) = min(vv(j), vn);
            end
            for i = N:-1:1                                % backward pass (braking)
                j = i + 1; if j > N, j = 1; end
                a = obj.decelMax(vv(j), max(kr(i), kr(j)));
                vv(i) = min(vv(i), sqrt(vv(j)^2 + 2 * a * ds));
            end

            vOut = zeros(N, 1);
            vOut(rot) = vv;

            v2 = vOut .^ 2;
            obj.s = trk.s;
            obj.v = vOut;
            obj.a = (v2([2:N, 1]) - v2([N, 1:N-1])) / (4 * ds);   % d(v^2/2)/ds
            obj.ds = ds;
            obj.totalLength = trk.totalLength;
            obj.n = N;
        end

        function a = accelMax(obj, v, kappa)
            ay = v * v * kappa;
            muEff = sqrt(max(obj.grip^2 - (ay / obj.g)^2, 0));
            drag = obj.aero.drag(v);
            aGrip = obj.wheelLoads.maxDriveAccel(muEff, drag / obj.mass);
            Ftire = min(obj.powertrain.maxForce(v), obj.mass * aGrip);
            a = (Ftire - drag) / obj.mass;
        end

        function a = decelMax(obj, v, kappa)
            ay = v * v * kappa;
            muEff = sqrt(max(obj.grip^2 - (ay / obj.g)^2, 0));
            aGrip = obj.wheelLoads.maxBrakeDecel(muEff, obj.brakes.bias);
            a = obj.brakeMargin * min(aGrip, obj.brakes.maxForce / obj.mass);
        end

        function val = sample(obj, arr, sq)
            f = mod(sq, obj.totalLength) / obj.ds;
            k = floor(f);
            t = f - k;
            i0 = k + 1;
            if i0 > obj.n
                i0 = obj.n;
                t = 1;
            end
            i1 = i0 + 1;
            if i1 > obj.n, i1 = 1; end
            val = (1 - t) * arr(i0) + t * arr(i1);
        end
    end
end
