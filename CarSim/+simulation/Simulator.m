classdef Simulator < handle
    %SIMULATOR Fixed-step loop: driver -> vehicle -> track projection -> laps -> recorder.
    %   It knows nothing about the UI. The step is dt = config.dt (1 ms) and
    %   recording happens at every step, in simulated time.

    properties (SetAccess = private)
        vehicleModel
        driverModel
        trackModel
        config
        state
        lastOut
        simTime (1,1) double = 0
        lapCounter
        recorder
        offTrack (1,1) logical = false
    end

    properties (Access = private)
        offTrackWarned (1,1) logical = false
    end

    methods (Static)
        function sim = build(params, trk, config)
            %BUILD Wire the default objects together (point mass + pure pursuit).
            if nargin < 3, config = simulation.SimConfig(); end
            trk.assertDrivable(params.minTurnRadius);
            veh = vehicle.PointMassVehicle(params, config);
            prof = driver.SpeedProfile(trk, params, veh.powertrain, veh.brakes, veh.aero, veh.wheelLoads);
            drv = driver.PurePursuitDriver(params, prof);
            sim = simulation.Simulator(veh, drv, trk, config);
        end
    end

    methods
        function obj = Simulator(vehicleModel, driverModel, trackModel, config)
            obj.vehicleModel = vehicleModel;
            obj.driverModel = driverModel;
            obj.trackModel = trackModel;
            obj.config = config;
            obj.lapCounter = simulation.LapCounter(trackModel.totalLength);
            obj.recorder = telemetry.Recorder();
            obj.reset();
        end

        function reset(obj)
            %RESET Put the car back on the line at its initial speed.
            assert(~obj.recorder.isRecording, 'Simulator:recording', ...
                'Stop the recording before resetting the simulator.');
            obj.driverModel.reset();
            vLim = obj.driverModel.startSpeedLimit(0);
            st = obj.vehicleModel.initialState(obj.trackModel, vLim);
            [st.s, st.lateralOffset, st.idx] = obj.trackModel.project([st.x, st.y], []);
            obj.lapCounter.reset(st.s);
            obj.state = st;
            obj.lastOut = struct('steerAngle', 0, 'driveForce', 0, 'brakeForce', 0, ...
                'motorTorque', 0, 'wheelTorque', 0, 'Fx', zeros(1,4), 'Fy', zeros(1,4), ...
                'Fz', zeros(1,4), 'yawRate', 0);
            obj.simTime = 0;
            obj.offTrack = false;
            obj.offTrackWarned = false;
        end

        function advance(obj, nSteps)
            %ADVANCE Run nSteps fixed steps (hot loop: no object creation).
            dt = obj.config.dt;
            st = obj.state;
            out = obj.lastOut;
            trk = obj.trackModel;
            halfWidth = trk.width / 2;
            rec = obj.recorder.isRecording;

            for k = 1:nSteps
                cmd = obj.driverModel.command(st, trk, dt);
                [st, out] = obj.vehicleModel.step(st, cmd, dt);
                [st.s, st.lateralOffset, st.idx] = trk.project([st.x, st.y], st.idx);
                st.t = st.t + dt;

                [done, lapTime] = obj.lapCounter.update(st.s, st.t, st.v);
                if done
                    if rec, obj.recorder.lapCompleted(st.lap, lapTime); end
                    st.lap = obj.lapCounter.lap;
                end

                if abs(st.lateralOffset) > halfWidth
                    obj.offTrack = true;
                end

                if rec
                    row = [st.t, st.lap, obj.lapCounter.currentLapTime(st.t), ...
                        st.x, st.y, st.s, st.lateralOffset, ...
                        st.yaw, out.yawRate, st.v, st.ax, st.ay, ...
                        out.steerAngle, cmd.driveForce, out.driveForce, out.brakeForce, ...
                        out.motorTorque, out.wheelTorque, out.Fx, out.Fy, out.Fz];
                    obj.recorder.push(row);
                end
            end

            obj.state = st;
            obj.lastOut = out;
            obj.simTime = st.t;

            if obj.offTrack && ~obj.offTrackWarned
                obj.offTrackWarned = true;
                warning('Simulator:offTrack', ...
                    'The reference point left the track (|offset| = %.2f m > %.2f m).', ...
                    abs(st.lateralOffset), halfWidth);
            end
        end

        function startRecording(obj)
            atLapStart = obj.lapCounter.currentLapTime(obj.state.t) < 0.05;
            obj.recorder.start(obj.setupStruct(), obj.state.lap, atLapStart);
        end

        function session = stopRecording(obj)
            session = obj.recorder.stop(obj.state.lap, ...
                obj.lapCounter.currentLapTime(obj.state.t));
        end

        function s = setupStruct(obj)
            %SETUPSTRUCT Everything needed to reproduce / interpret a session.
            trk = obj.trackModel;
            s = struct();
            s.vehicle = obj.vehicleModel.describe();
            s.track = struct('name', trk.name, 'width', trk.width, 'length', trk.totalLength);
            s.driver = obj.driverModel.describe();
            s.simulation = obj.config.toStruct();
            s.conventions = struct( ...
                'referencePoint', 'centre of the rear axle', ...
                'yaw', 'rad, counter-clockwise from +x, wrapped to [-pi, pi]', ...
                'lateralOffset', 'm, positive on the left of the centre line', ...
                'wheelOrder', 'FL, FR, RL, RR', ...
                'Fx', 'N, positive forward', ...
                'Fy', 'N, positive to the left', ...
                'Fz', 'N, vertical load', ...
                'brakeForce', 'N, magnitude (positive)', ...
                'launch', 'flying start at s = 0, t = 0');
        end
    end
end
