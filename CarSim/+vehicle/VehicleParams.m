classdef VehicleParams
    %VEHICLEPARAMS Car setup (value object, SI units everywhere).
    %   Edit the numbers in +vehicle/+presets/defaultCar.m, not in the UI.

    properties
        mass (1,1) double {mustBePositive, mustBeFinite} = 1200               % kg
        wheelbase (1,1) double {mustBePositive, mustBeFinite} = 2.6           % m
        length (1,1) double {mustBePositive, mustBeFinite} = 4.2              % m (body length)
        trackWidth (1,1) double {mustBePositive, mustBeFinite} = 1.5          % m (also body width in the UI)
        rearWheelDiameter (1,1) double {mustBePositive, mustBeFinite} = 0.65  % m
        cgFrontFraction (1,1) double {mustBeFinite} = 0.48                    % - static weight fraction on the front axle
        cgHeight (1,1) double {mustBeNonnegative, mustBeFinite} = 0.5         % m
        maxSteerAngle (1,1) double {mustBePositive, mustBeFinite} = deg2rad(30) % rad
        grip (1,1) double {mustBePositive, mustBeFinite} = 1.2                % - friction coefficient mu
        CdA (1,1) double {mustBeNonnegative, mustBeFinite} = 0.65             % m^2 (drag coefficient * frontal area)
        maxMotorTorque (1,1) double {mustBePositive, mustBeFinite} = 350      % N*m
        gearRatio (1,1) double {mustBePositive, mustBeFinite} = 9             % - motor -> wheel
        maxPower (1,1) double {mustBePositive, mustBeFinite} = 150e5          % W
        maxBrakeForce (1,1) double {mustBePositive, mustBeFinite} = 14e3      % N (total)
        brakeBias (1,1) double {mustBeFinite} = 0.6                           % - fraction of braking on the front axle
        initialSpeed (1,1) double {mustBeNonnegative, mustBeFinite} = 20      % m/s (flying start)
    end

    properties (Dependent)
        rearWheelRadius     % m
        minTurnRadius       % m, wheelbase / tan(maxSteerAngle)
    end

    methods
        function r = get.rearWheelRadius(obj)
            r = obj.rearWheelDiameter / 2;
        end

        function r = get.minTurnRadius(obj)
            r = obj.wheelbase / tan(obj.maxSteerAngle);
        end

        function validate(obj)
            %VALIDATE Cross-field checks (single-field checks are in the property definitions).
            assert(obj.cgFrontFraction > 0 && obj.cgFrontFraction < 1, ...
                'VehicleParams:cg', 'cgFrontFraction must be in (0, 1).');
            assert(obj.brakeBias > 0 && obj.brakeBias < 1, ...
                'VehicleParams:bias', 'brakeBias must be in (0, 1).');
            assert(obj.maxSteerAngle < pi/2, ...
                'VehicleParams:steer', 'maxSteerAngle must be below 90 deg.');
            assert(obj.length >= obj.wheelbase, ...
                'VehicleParams:length', 'length must be >= wheelbase.');
        end

        function s = toStruct(obj)
            %TOSTRUCT Export every public property (including dependent ones).
            s = struct();
            for name = string(properties(obj))'
                s.(name) = obj.(name);
            end
        end
    end
end
