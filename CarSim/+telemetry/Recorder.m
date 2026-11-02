classdef Recorder < handle
    %RECORDER Pre-allocated buffer (grows by doubling) filled one row per simulation step.
    properties (SetAccess = private)
        isRecording (1,1) logical = false
        nRows (1,1) double = 0
    end
    properties (Constant)
        chunk = 60000                       % rows allocated at start (= 60 s at 1 kHz)
    end
    properties (Access = private)
        data double = []
        setup struct = struct()
        startTime datetime
        startLap (1,1) double = 1
        atLapStart (1,1) logical = false
        lapLog (:,3) double = zeros(0, 3)   % [lap lapTime complete]
    end

    methods
        function start(obj, setup, lap, atLapStart)
            assert(~obj.isRecording, 'Recorder:busy', 'Already recording.');
            obj.data = nan(obj.chunk, telemetry.SignalCatalog.count());
            obj.nRows = 0;
            obj.setup = setup;
            obj.startTime = datetime('now');
            obj.startLap = lap;
            obj.atLapStart = atLapStart;
            obj.lapLog = zeros(0, 3);
            obj.isRecording = true;
        end

        function push(obj, row)
            n = obj.nRows + 1;
            if n > size(obj.data, 1)
                obj.data = [obj.data; nan(size(obj.data))];
            end
            obj.data(n, :) = row;
            obj.nRows = n;
        end

        function lapCompleted(obj, lap, lapTime)
            complete = obj.atLapStart || lap > obj.startLap;
            obj.lapLog(end+1, :) = [lap, lapTime, complete];
        end

        function session = stop(obj, currentLap, currentLapTime)
            %STOP Close the session and return it as a struct.
            %   session.telemetry : timetable (RowTimes = t, with VariableUnits)
            %   session.laps      : table (lap, lapTime, complete)
            %   session.setup     : car / track / driver / simulation setup
            %   session.meta      : date, version, units
            assert(obj.isRecording, 'Recorder:idle', 'Not recording.');
            n = obj.nRows;
            names = telemetry.SignalCatalog.names();
            units = telemetry.SignalCatalog.units();

            d = obj.data(1:n, :);
            tt = array2timetable(d(:, 2:end), 'RowTimes', seconds(d(:, 1)), ...
                'VariableNames', cellstr(names(2:end)));
            tt.Properties.DimensionNames{1} = 't';
            tt.Properties.VariableUnits = cellstr(units(2:end));

            if n > 0
                obj.lapLog(end+1, :) = [currentLap, currentLapTime, 0];   % lap in progress
            end
            L = obj.lapLog;
            laps = table(L(:, 1), L(:, 2), logical(L(:, 3)), ...
                'VariableNames', {'lap', 'lapTime', 'complete'});

            unitStruct = struct();
            for i = 1:numel(names)
                unitStruct.(names(i)) = units(i);
            end
            meta = struct('startTime', obj.startTime, 'endTime', datetime('now'), ...
                'version', "1.0", 'rows', n, 'units', unitStruct);

            session = struct('telemetry', tt, 'laps', laps, 'setup', obj.setup, 'meta', meta);

            obj.isRecording = false;
            obj.data = [];
            obj.nRows = 0;
        end
    end
end
