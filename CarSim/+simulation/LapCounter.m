classdef LapCounter < handle
    %LAPCOUNTER Detects start/finish line crossings from the arc length s.
    properties (SetAccess = private)
        lap (1,1) double = 1
        lapStartTime (1,1) double = 0
        lapHistory table = table(zeros(0,1), zeros(0,1), 'VariableNames', {'lap', 'lapTime'})
    end
    properties (SetAccess = private)
        trackLength (1,1) double
        sPrev (1,1) double = 0
    end
    properties
        minLapTime (1,1) double = 1     % s, ignores spurious crossings right after the start
    end

    methods
        function obj = LapCounter(trackLength)
            obj.trackLength = trackLength;
        end

        function reset(obj, s0)
            obj.lap = 1;
            obj.lapStartTime = 0;
            obj.lapHistory = table(zeros(0,1), zeros(0,1), 'VariableNames', {'lap', 'lapTime'});
            if s0 > obj.trackLength / 2
                s0 = s0 - obj.trackLength;      % just before the line
            end
            obj.sPrev = s0;
        end

        function [completed, lapTime] = update(obj, s, t, v)
            %UPDATE Call once per step. The crossing time is interpolated with s and v.
            completed = false;
            lapTime = 0;
            if s - obj.sPrev < -0.5 * obj.trackLength && (t - obj.lapStartTime) > obj.minLapTime
                tc = t - s / max(v, 0.1);
                lapTime = tc - obj.lapStartTime;
                obj.lapHistory(end+1, :) = {obj.lap, lapTime};
                obj.lap = obj.lap + 1;
                obj.lapStartTime = tc;
                completed = true;
            end
            obj.sPrev = s;
        end

        function lt = currentLapTime(obj, t)
            lt = max(t - obj.lapStartTime, 0);
        end
    end
end
