classdef SimConfig
    %SIMCONFIG Numerical and environment settings.
    properties
        dt (1,1) double {mustBePositive} = 1e-3          % s, fixed step = recording step
        airDensity (1,1) double {mustBePositive} = 1.225 % kg/m^3
        gravity (1,1) double {mustBePositive} = 9.81     % m/s^2
        frameRate (1,1) double {mustBePositive} = 30     % Hz, UI refresh
        maxCatchUp (1,1) double {mustBePositive} = 0.1   % s, max simulated time computed per frame
    end
    methods
        function s = toStruct(obj)
            s = struct();
            for name = string(properties(obj))'
                s.(name) = obj.(name);
            end
        end
    end
end
