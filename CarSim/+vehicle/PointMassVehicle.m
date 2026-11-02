classdef PointMassVehicle < vehicle.VehicleModel
    %POINTMASSVEHICLE Point mass + kinematic bicycle + per-wheel force model.
    %   Reference point (x, y) = centre of the rear axle.
    %   yaw is the heading angle (rad, counter-clockwise from +x, wrapped to [-pi, pi]).
    %
    %   Tyre model: friction circle per wheel, sqrt(Fx^2 + Fy^2) <= mu*Fz.
    %   Lateral force has priority; what is left is available for traction / braking.
    %   Wheel loads use the previous step's ax (no algebraic loop).

    properties (SetAccess = private)
        params          % vehicle.VehicleParams
        powertrain      % vehicle.Powertrain
        brakes          % vehicle.BrakeSystem
        aero            % vehicle.Aero
        wheelLoads      % vehicle.WheelLoadModel
        gravity (1,1) double = 9.81
    end

    methods
        function obj = PointMassVehicle(params, config)
            params.validate();
            obj.params = params;
            obj.gravity = config.gravity;
            obj.powertrain = vehicle.Powertrain(params.maxMotorTorque, params.gearRatio, ...
                params.maxPower, params.rearWheelRadius);
            obj.brakes = vehicle.BrakeSystem(params.maxBrakeForce, params.brakeBias);
            obj.aero = vehicle.Aero(params.CdA, config.airDensity);
            obj.wheelLoads = vehicle.WheelLoadModel(params.mass, params.wheelbase, ...
                params.trackWidth, params.cgFrontFraction, params.cgHeight, config.gravity);
        end

        function [state, out] = step(obj, state, cmd, dt)
            p = obj.params;
            g = obj.gravity;
            m = p.mass;
            v = state.v;

            % --- steering: kinematic bicycle, reference point on the rear axle
            delta = min(max(cmd.steerAngle, -p.maxSteerAngle), p.maxSteerAngle);
            yawRate = v * tan(delta) / p.wheelbase;
            ay = v * yawRate;
            ayMax = p.grip * g;
            if abs(ay) > ayMax                 % lateral grip exhausted -> understeer
                ay = sign(ay) * ayMax;
                yawRate = ay / v;
            end

            % --- wheel loads and per-wheel longitudinal capacity (friction circle)
            Fz = obj.wheelLoads.loads(state.ax, ay);
            kLong = sqrt(max(p.grip^2 - (ay / g)^2, 0));
            cap = kLong * Fz;                  % 1x4 [N]

            % --- traction (rear wheels, equal split, per-wheel cap)
            Fd = obj.powertrain.apply(cmd.driveForce, v);
            driveW = [0, 0, min(Fd/2, cap(3)), min(Fd/2, cap(4))];

            % --- braking (all wheels, bias split, per-wheel cap; no reverse)
            if v > 0.01
                [~, brakeReq] = obj.brakes.apply(cmd.brakeForce);
                brakeW = min(brakeReq, cap);
            else
                brakeW = zeros(1, 4);
            end

            FxW = driveW - brakeW;
            FyW = ay * Fz / g;                 % Fy_i = m*ay*Fz_i/(m*g)
            Fdrive = sum(driveW);
            Fbrake = sum(brakeW);

            % --- longitudinal dynamics
            ax = (sum(FxW) - obj.aero.drag(v)) / m;

            % --- semi-implicit Euler: v, then yaw, then position
            v = max(v + ax * dt, 0);
            yaw = state.yaw + yawRate * dt;
            yaw = atan2(sin(yaw), cos(yaw));
            state.x = state.x + v * cos(yaw) * dt;
            state.y = state.y + v * sin(yaw) * dt;
            state.v = v;
            state.yaw = yaw;
            state.ax = ax;
            state.ay = ay;
            state.yawRate = yawRate;

            [Tm, Tw] = obj.powertrain.torquesFromForce(Fdrive);
            out = struct('steerAngle', delta, 'driveForce', Fdrive, 'brakeForce', Fbrake, ...
                'motorTorque', Tm, 'wheelTorque', Tw, ...
                'Fx', FxW, 'Fy', FyW, 'Fz', Fz, 'yawRate', yawRate);
        end

        function state = initialState(obj, trk, speedLimit)
            p0 = trk.pointAt(0);
            state = struct('x', p0(1), 'y', p0(2), 'yaw', trk.headingAt(0), ...
                'v', min(obj.params.initialSpeed, speedLimit), ...
                's', 0, 'lateralOffset', 0, 'idx', 1, ...
                'ax', 0, 'ay', 0, 'yawRate', 0, 't', 0, 'lap', 1);
        end

        function s = describe(obj)
            s = struct('model', 'PointMassVehicle', 'params', obj.params.toStruct());
        end
    end
end
