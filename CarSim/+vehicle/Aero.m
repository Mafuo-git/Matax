classdef Aero
    %AERO Aerodynamic drag only (no downforce).
    properties (SetAccess = immutable)
        CdA (1,1) double        % m^2
        airDensity (1,1) double % kg/m^3
    end
    methods
        function obj = Aero(CdA, airDensity)
            obj.CdA = CdA;
            obj.airDensity = airDensity;
        end

        function F = drag(obj, v)
            %DRAG Drag force magnitude [N] at speed v [m/s].
            F = 0.5 * obj.airDensity * obj.CdA * v * v;
        end
    end
end
