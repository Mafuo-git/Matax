classdef PurePursuitDriver < driver.Driver
    %PUREPURSUITDRIVER Pure Pursuit steering + (feed-forward + PI) speed control.
    %   delta = atan(2*L*sin(alpha) / ld),   ld = max(lookaheadMin, lookaheadGain*v)
    %   F     = m*a_ff + F_drag + Kp*e + I      (F > 0 traction, F < 0 brake)

    properties
        lookaheadMin (1,1) double = 4        % m
        lookaheadGain (1,1) double = 0.25    % s
        Kp (1,1) double = 2000               % N / (m/s)
        Ki (1,1) double = 500                % N / (m)   (integral of the speed error)
        maxIntegral (1,1) double = 1000      % N, anti-windup clamp
        integralLeakTime (1,1) double = 1    % s, the integral decays so it cannot delay braking after a long acceleration
    end

    properties (SetAccess = private)
        profile                 % driver.SpeedProfile
        wheelbase (1,1) double
        maxSteer (1,1) double
    end

    properties (Access = private)
        integ (1,1) double = 0
    end

    methods
        function obj = PurePursuitDriver(params, profile)
            obj.profile = profile;
            obj.wheelbase = params.wheelbase;
            obj.maxSteer = params.maxSteerAngle;
        end

        function reset(obj)
            obj.integ = 0;
        end

        function v = startSpeedLimit(obj, sq)
            v = obj.profile.targetSpeed(sq);
        end

        function cmd = command(obj, state, trk, dt)
            v = state.v;
            sq = state.s;

            % --- lateral: pure pursuit on the centre line
            ld = max(obj.lookaheadMin, obj.lookaheadGain * v);
            tgt = trk.lookaheadPoint(sq, ld);
            dx = tgt(1) - state.x;
            dy = tgt(2) - state.y;
            alpha = atan2(dy, dx) - state.yaw;
            alpha = atan2(sin(alpha), cos(alpha));
            dist = max(hypot(dx, dy), 1e-3);
            delta = atan2(2 * obj.wheelbase * sin(alpha), dist);
            delta = min(max(delta, -obj.maxSteer), obj.maxSteer);

            % --- longitudinal: feed-forward + PI
            e = obj.profile.targetSpeed(sq) - v;
            obj.integ = obj.integ * (1 - dt / obj.integralLeakTime) + obj.Ki * e * dt;
            obj.integ = min(max(obj.integ, -obj.maxIntegral), obj.maxIntegral);
            F = obj.profile.feedforwardForce(sq, v) + obj.Kp * e + obj.integ;

            cmd.driveForce = max(F, 0);
            cmd.brakeForce = max(-F, 0);
            cmd.steerAngle = delta;
        end

        function s = describe(obj)
            s = struct('driver', 'PurePursuitDriver', ...
                'lookaheadMin', obj.lookaheadMin, 'lookaheadGain', obj.lookaheadGain, ...
                'Kp', obj.Kp, 'Ki', obj.Ki, 'maxIntegral', obj.maxIntegral, ...
                'integralLeakTime', obj.integralLeakTime, ...
                'lateralMargin', obj.profile.lateralMargin, 'brakeMargin', obj.profile.brakeMargin);
        end
    end
end
