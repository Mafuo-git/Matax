classdef (Abstract) Driver < handle
    %DRIVER Interface of every driver (automatic, keyboard, replay, ...).
    methods (Abstract)
        cmd = command(obj, state, trk, dt)   % cmd.driveForce, cmd.brakeForce, cmd.steerAngle
        reset(obj)
    end
    methods
        function v = startSpeedLimit(~, ~)
            %STARTSPEEDLIMIT Highest speed the driver accepts at arc length s (default: none).
            v = Inf;
        end

        function s = describe(~)
            s = struct();
        end
    end
end
