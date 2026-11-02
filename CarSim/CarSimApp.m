classdef CarSimApp < matlab.apps.AppBase
    % CARSIMAPP Real-time 2D lap simulator (top view, fixed circuit).
    %
    % UI only: all the physics lives in the packages (+simulation, +vehicle, ...).
    % The file follows the structure that App Designer generates (component
    % properties, createComponents, callbacks named <Component><Event>), so the
    % callbacks can be copied one-to-one into a .mlapp.
    %
    % Run:  app = CarSimApp;   (after  setupProject  and  makeTestTracks)

    % ------------------------------------------------------------- components
    properties (Access = public)
        UIFigure        matlab.ui.Figure
        TrackAxes       matlab.ui.control.UIAxes
        TrackDropDown   matlab.ui.control.DropDown
        StartButton     matlab.ui.control.Button
        StopButton      matlab.ui.control.Button
        EndButton       matlab.ui.control.Button
        RecordButton    matlab.ui.control.StateButton
        SpeedValue      matlab.ui.control.Label
        SteerValue      matlab.ui.control.Label
        BrakeValue      matlab.ui.control.Label
        DriveValue      matlab.ui.control.Label
        LapValue        matlab.ui.control.Label
        LapTimeValue    matlab.ui.control.Label
        LastLapValue    matlab.ui.control.Label
        StatusLabel     matlab.ui.control.Label
        FileLabel       matlab.ui.control.Label
    end

    % ------------------------------------------------------------- app data
    properties (Access = private)
        Sim                                  % simulation.Simulator
        Params                               % vehicle.VehicleParams
        Trk                                  % track.Track
        State (1,1) string = "idle"          % idle | running | paused | closing
        CarPatch                             % patch handle (updated, never recreated)
        CarLocal (4,2) double = zeros(4, 2)  % body corners in the car frame
        SessionFolder (1,1) string
        Lagging (1,1) logical = false
    end

    % ------------------------------------------------------------- helpers
    methods (Access = private)

        function startupFcn(app)
            root = fileparts(mfilename('fullpath'));
            app.SessionFolder = string(fullfile(root, 'data', 'sessions'));
            app.Params = vehicle.presets.defaultCar();

            names = track.TrackLibrary.list();
            if isempty(names)
                app.TrackDropDown.Items = {'(no track)'};
                uialert(app.UIFigure, ...
                    'No track in data/tracks. Run scripts/makeTestTracks.m first.', 'No track');
                app.updateButtons();
                return
            end
            app.TrackDropDown.Items = cellstr(names);
            app.TrackDropDown.Value = char(names(1));
            app.loadTrack(names(1));
        end

        function loadTrack(app, name)
            try
                app.Trk = track.TrackLibrary.load(name);
                app.Sim = simulation.Simulator.build(app.Params, app.Trk, simulation.SimConfig());
            catch err
                app.Sim = [];
                uialert(app.UIFigure, err.message, 'Cannot build the simulation');
                app.updateButtons();
                return
            end
            app.State = "idle";
            app.Lagging = false;
            app.drawTrack();
            app.updateGraphics();
            app.updateButtons();
        end

        function drawTrack(app)
            ax = app.TrackAxes;
            cla(ax);
            P = app.Trk.points;
            hw = app.Trk.width / 2;
            nx = -sin(app.Trk.heading);
            ny = cos(app.Trk.heading);
            Lx = P(:, 1) + hw * nx;  Ly = P(:, 2) + hw * ny;
            Rx = P(:, 1) - hw * nx;  Ry = P(:, 2) - hw * ny;

            hold(ax, 'on');
            plot(ax, [Lx; Lx(1)], [Ly; Ly(1)], '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.2);
            plot(ax, [Rx; Rx(1)], [Ry; Ry(1)], '-', 'Color', [0.25 0.25 0.25], 'LineWidth', 1.2);
            plot(ax, [P(:, 1); P(1, 1)], [P(:, 2); P(1, 2)], '--', 'Color', [0.75 0.75 0.75]);
            plot(ax, [Lx(1) Rx(1)], [Ly(1) Ry(1)], '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 3);
            app.CarPatch = patch(ax, 'XData', nan(4, 1), 'YData', nan(4, 1), ...
                'FaceColor', [0.1 0.4 0.9], 'EdgeColor', 'k');
            hold(ax, 'off');

            allX = [Lx; Rx];
            allY = [Ly; Ry];
            span = max(max(allX) - min(allX), max(allY) - min(allY));
            pad = 0.05 * span;
            xlim(ax, [min(allX) - pad, max(allX) + pad]);
            ylim(ax, [min(allY) - pad, max(allY) + pad]);
            axis(ax, 'equal');
            grid(ax, 'on');
            box(ax, 'on');
            xlabel(ax, 'x (m)');
            ylabel(ax, 'y (m)');
            title(ax, app.Trk.name, 'Interpreter', 'none');
            disableDefaultInteractivity(ax);

            % the car is drawn larger than life so that it stays visible on the whole circuit
            p = app.Params;
            scale = max(1, 0.025 * span / p.length);
            ro = (p.length - p.wheelbase) / 2;           % rear overhang (reference = rear axle)
            body = [-ro, -p.trackWidth/2; p.length - ro, -p.trackWidth/2; ...
                     p.length - ro, p.trackWidth/2; -ro, p.trackWidth/2];
            app.CarLocal = scale * body;
        end

        function updateGraphics(app)
            if isempty(app.Sim), return, end
            st = app.Sim.state;
            out = app.Sim.lastOut;

            c = cos(st.yaw);  sn = sin(st.yaw);
            V = app.CarLocal * [c, -sn; sn, c]' + [st.x, st.y];
            app.CarPatch.XData = V(:, 1);
            app.CarPatch.YData = V(:, 2);

            app.SpeedValue.Text = sprintf('%.1f km/h', st.v * 3.6);
            app.SteerValue.Text = sprintf('%+.1f deg', rad2deg(out.steerAngle));
            app.BrakeValue.Text = sprintf('%.0f N', out.brakeForce);
            app.DriveValue.Text = sprintf('%.0f N', out.driveForce);
            app.LapValue.Text = sprintf('%d', st.lap);
            app.LapTimeValue.Text = sprintf('%.3f s', app.Sim.lapCounter.currentLapTime(st.t));
            hist = app.Sim.lapCounter.lapHistory;
            if height(hist) > 0
                app.LastLapValue.Text = sprintf('%.3f s', hist.lapTime(end));
            else
                app.LastLapValue.Text = '--';
            end

            parts = upper(app.State);
            if app.Sim.recorder.isRecording, parts = parts + " | REC"; end
            if app.Lagging, parts = parts + " | SIM LAG"; end
            if app.Sim.offTrack, parts = parts + " | OFF TRACK"; end
            app.StatusLabel.Text = char(parts);
        end

        function updateButtons(app)
            hasSim = ~isempty(app.Sim);
            recording = hasSim && app.Sim.recorder.isRecording;
            app.StartButton.Enable = onoff(hasSim && app.State ~= "running");
            app.StopButton.Enable = onoff(hasSim && app.State == "running");
            app.EndButton.Enable = onoff(hasSim && (app.State ~= "idle" || recording));
            app.RecordButton.Enable = onoff(hasSim);
            app.TrackDropDown.Enable = onoff(app.State == "idle" && ~recording);
        end

        function runLoop(app)
            % Real-time loop: the simulation is advanced by the wall-clock time elapsed
            % since the last frame (fixed 1 ms steps); the UI is refreshed once per frame.
            cfg = app.Sim.config;
            dt = cfg.dt;
            frame = 1 / cfg.frameRate;
            maxSteps = floor(cfg.maxCatchUp / dt);
            clk = tic;
            anchorWall = 0;
            anchorSim = app.Sim.simTime;

            while isvalid(app) && app.State == "running"
                iterStart = toc(clk);
                due = floor((anchorSim + (iterStart - anchorWall) - app.Sim.simTime) / dt);
                lagging = due > maxSteps;
                if lagging, due = maxSteps; end

                app.Sim.advance(due);

                app.Lagging = lagging;
                if lagging                                % cannot keep up: re-anchor the clock
                    anchorWall = toc(clk);
                    anchorSim = app.Sim.simTime;
                end

                app.updateGraphics();
                drawnow;                                  % also processes Stop / End / Record clicks
                if ~isvalid(app), return, end

                remaining = frame - (toc(clk) - iterStart);
                if remaining > 0.001, pause(remaining); end
            end
        end

        function finishRecording(app)
            session = app.Sim.stopRecording();
            app.RecordButton.Value = false;
            if height(session.telemetry) == 0
                app.FileLabel.Text = 'Recording was empty: nothing saved.';
                return
            end
            try
                files = telemetry.Exporter.write(session, app.SessionFolder, ["mat", "csv"]);
                [~, nm, ext] = fileparts(files(1));
                app.FileLabel.Text = char("Saved: " + nm + ext);
            catch err
                uialert(app.UIFigure, err.message, 'Cannot save the session');
            end
        end

        function lbl = addReadout(~, grid, row, caption)
            c = uilabel(grid);
            c.Layout.Row = row;  c.Layout.Column = 1;
            c.Text = caption;
            c.FontColor = [0.4 0.4 0.4];
            lbl = uilabel(grid);
            lbl.Layout.Row = row;  lbl.Layout.Column = 2;
            lbl.Text = '--';
            lbl.FontSize = 18;
            lbl.FontWeight = 'bold';
            lbl.HorizontalAlignment = 'right';
        end
    end

    % ------------------------------------------------------------- callbacks
    methods (Access = private)

        function TrackDropDownValueChanged(app, ~)
            if app.State == "idle"
                app.loadTrack(string(app.TrackDropDown.Value));
            end
        end

        function StartButtonPushed(app, ~)
            if isempty(app.Sim) || app.State == "running", return, end
            app.State = "running";
            app.updateButtons();
            app.runLoop();
            if isvalid(app)
                app.updateGraphics();
                app.updateButtons();
            end
        end

        function StopButtonPushed(app, ~)
            if app.State == "running"
                app.State = "paused";
                app.updateButtons();
                app.updateGraphics();
            end
        end

        function EndButtonPushed(app, ~)
            if isempty(app.Sim), return, end
            app.State = "idle";
            if app.Sim.recorder.isRecording
                app.finishRecording();
            end
            app.Sim.reset();
            app.Lagging = false;
            app.updateGraphics();
            app.updateButtons();
        end

        function RecordButtonValueChanged(app, ~)
            if isempty(app.Sim)
                app.RecordButton.Value = false;
                return
            end
            if app.RecordButton.Value
                app.Sim.startRecording();
                app.FileLabel.Text = 'Recording...';
            else
                app.finishRecording();
            end
            app.updateGraphics();
            app.updateButtons();
        end

        function UIFigureCloseRequest(app, ~)
            app.State = "closing";
            try
                if ~isempty(app.Sim) && app.Sim.recorder.isRecording
                    app.finishRecording();
                end
            catch
            end
            delete(app);
        end
    end

    % ------------------------------------------------------------- layout
    methods (Access = private)

        function createComponents(app)
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 1150 680];
            app.UIFigure.Name = 'CarSim - 2D lap simulator';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);

            main = uigridlayout(app.UIFigure, [1 2]);
            main.ColumnWidth = {'1x', 330};

            app.TrackAxes = uiaxes(main);
            app.TrackAxes.Layout.Row = 1;
            app.TrackAxes.Layout.Column = 1;

            side = uigridlayout(main, [5 1]);
            side.Layout.Row = 1;
            side.Layout.Column = 2;
            side.RowHeight = {28, 80, '1x', 28, 28};

            app.TrackDropDown = uidropdown(side);
            app.TrackDropDown.Layout.Row = 1;
            app.TrackDropDown.Items = {'(no track)'};
            app.TrackDropDown.ValueChangedFcn = createCallbackFcn(app, @TrackDropDownValueChanged, true);

            buttons = uigridlayout(side, [2 2]);
            buttons.Layout.Row = 2;
            buttons.Padding = [0 0 0 0];

            app.StartButton = uibutton(buttons, 'push');
            app.StartButton.Layout.Row = 1;
            app.StartButton.Layout.Column = 1;
            app.StartButton.Text = 'Start';
            app.StartButton.ButtonPushedFcn = createCallbackFcn(app, @StartButtonPushed, true);

            app.StopButton = uibutton(buttons, 'push');
            app.StopButton.Layout.Row = 1;
            app.StopButton.Layout.Column = 2;
            app.StopButton.Text = 'Stop';
            app.StopButton.ButtonPushedFcn = createCallbackFcn(app, @StopButtonPushed, true);

            app.EndButton = uibutton(buttons, 'push');
            app.EndButton.Layout.Row = 2;
            app.EndButton.Layout.Column = 1;
            app.EndButton.Text = 'End';
            app.EndButton.ButtonPushedFcn = createCallbackFcn(app, @EndButtonPushed, true);

            app.RecordButton = uibutton(buttons, 'state');
            app.RecordButton.Layout.Row = 2;
            app.RecordButton.Layout.Column = 2;
            app.RecordButton.Text = 'Record';
            app.RecordButton.ValueChangedFcn = createCallbackFcn(app, @RecordButtonValueChanged, true);

            readouts = uigridlayout(side, [7 2]);
            readouts.Layout.Row = 3;
            readouts.Padding = [0 0 0 0];
            readouts.ColumnWidth = {'1x', '1x'};
            readouts.RowHeight = repmat({'fit'}, 1, 7);
            app.SpeedValue   = app.addReadout(readouts, 1, 'Speed');
            app.SteerValue   = app.addReadout(readouts, 2, 'Steering angle');
            app.BrakeValue   = app.addReadout(readouts, 3, 'Brake force');
            app.DriveValue   = app.addReadout(readouts, 4, 'Drive force');
            app.LapValue     = app.addReadout(readouts, 5, 'Lap');
            app.LapTimeValue = app.addReadout(readouts, 6, 'Lap time');
            app.LastLapValue = app.addReadout(readouts, 7, 'Last lap');

            app.StatusLabel = uilabel(side);
            app.StatusLabel.Layout.Row = 4;
            app.StatusLabel.Text = 'IDLE';
            app.StatusLabel.FontWeight = 'bold';

            app.FileLabel = uilabel(side);
            app.FileLabel.Layout.Row = 5;
            app.FileLabel.Text = '';
            app.FileLabel.FontColor = [0.3 0.3 0.3];

            app.UIFigure.Visible = 'on';
        end
    end

    % ------------------------------------------------------------- lifecycle
    methods (Access = public)

        function app = CarSimApp
            createComponents(app)
            registerApp(app, app.UIFigure)
            runStartupFcn(app, @startupFcn)
            if nargout == 0
                clear app
            end
        end

        function delete(app)
            delete(app.UIFigure)
        end
    end
end

function s = onoff(tf)
    if tf, s = 'on'; else, s = 'off'; end
end
