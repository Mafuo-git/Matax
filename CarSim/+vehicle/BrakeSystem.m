classdef BrakeSystem
    %BRAKESYSTEM Total brake force limit with fixed front/rear bias.
    properties (SetAccess = immutable)
        maxForce (1,1) double   % N (total)
        bias (1,1) double       % - fraction on the front axle
    end
    methods
        function obj = BrakeSystem(maxForce, bias)
            obj.maxForce = maxForce;
            obj.bias = bias;
        end

        function [Fbrake, perWheel] = apply(obj, Frequest)
            %APPLY Brake force magnitude (>= 0) and its split [FL FR RL RR].
            Fbrake = min(max(Frequest, 0), obj.maxForce);
            Ff = Fbrake * obj.bias;
            Fr = Fbrake - Ff;
            perWheel = [Ff/2, Ff/2, Fr/2, Fr/2];
        end
    end
end
