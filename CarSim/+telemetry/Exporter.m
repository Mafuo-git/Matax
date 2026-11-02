classdef Exporter
    %EXPORTER Writes a recorded session to disk.
    %   <folder>/session_yyyyMMdd_HHmmss.mat        telemetry, laps, setup, meta
    %   <folder>/session_yyyyMMdd_HHmmss.csv        same table + speed_kmh, steer_deg, yaw_deg, yawRate_degps
    %   <folder>/session_yyyyMMdd_HHmmss_laps.csv   lap summary
    %   <folder>/session_yyyyMMdd_HHmmss_setup.json car / track / driver / simulation setup + units
    methods (Static)
        function files = write(session, folder, formats)
            arguments
                session (1,1) struct
                folder (1,1) string
                formats (1,:) string = ["mat", "csv"]
            end
            if height(session.telemetry) == 0
                error('Exporter:empty', 'The session contains no sample.');
            end
            if ~isfolder(folder), mkdir(folder); end

            t0 = session.meta.startTime;
            t0.Format = 'yyyyMMdd_HHmmss';
            base = fullfile(folder, "session_" + string(t0));
            formats = lower(formats);
            files = strings(1, 0);

            if any(formats == "mat")
                S = struct('telemetry', session.telemetry, 'laps', session.laps, ...
                    'setup', session.setup, 'meta', session.meta); %#ok<NASGU>
                path = char(base + ".mat");
                save(path, '-struct', 'S');
                files(end+1) = string(path);
            end

            if any(formats == "csv")
                T = timetable2table(session.telemetry);
                T.Properties.VariableUnits = {};
                T.t = seconds(T.t);
                T.speed_kmh = T.speed * 3.6;
                T.steer_deg = rad2deg(T.steerAngle);
                T.yaw_deg = rad2deg(T.yaw);
                T.yawRate_degps = rad2deg(T.yawRate);
                path = char(base + ".csv");
                writetable(T, path);
                files(end+1) = string(path);

                path = char(base + "_laps.csv");
                writetable(session.laps, path);
                files(end+1) = string(path);

                payload = struct('meta', session.meta, 'setup', session.setup, ...
                    'laps', table2struct(session.laps));
                payload.meta.startTime = char(session.meta.startTime);
                payload.meta.endTime = char(session.meta.endTime);
                path = char(base + "_setup.json");
                fid = fopen(path, 'w', 'n', 'UTF-8');
                fprintf(fid, '%s', jsonencode(payload, 'PrettyPrint', true));
                fclose(fid);
                files(end+1) = string(path);
            end
        end
    end
end
