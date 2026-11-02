% RUNHEADLESS Run the simulator without the UI, record 2 laps, export and plot.
%   Use it to validate the model and to measure the cost of one step
%   (it must stay well below 1 ms to run in real time).
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
setupProject;

trackName = "stadium";
nLaps = 2;

params = vehicle.presets.defaultCar();
trk = track.TrackLibrary.load(trackName);
simulator = simulation.Simulator.build(params, trk);
dt = simulator.config.dt;

simulator.startRecording();
tStart = tic;
while simulator.state.lap <= nLaps && simulator.simTime < 600
    simulator.advance(1000);
end
wall = toc(tStart);
session = simulator.stopRecording();

steps = round(simulator.simTime / dt);
fprintf('Simulated %.1f s in %.2f s of wall time -> %.1f us/step (budget: %.0f us), x%.1f real time\n', ...
    simulator.simTime, wall, 1e6 * wall / steps, 1e6 * dt, simulator.simTime / wall);
disp(session.laps);
if simulator.offTrack
    fprintf('WARNING: the car left the track.\n');
end

files = telemetry.Exporter.write(session, fullfile(root, 'data', 'sessions'));
disp(files');

tt = session.telemetry;
figure('Name', 'Headless run', 'Color', 'w');
subplot(2, 2, 1);
scatter(tt.x, tt.y, 4, tt.speed * 3.6, 'filled'); axis equal; grid on;
colorbar; title('Trajectory coloured by speed [km/h]');
subplot(2, 2, 2);
plot(tt.s, tt.speed * 3.6); grid on; xlabel('s [m]'); ylabel('speed [km/h]');
subplot(2, 2, 3);
plot(seconds(tt.t), rad2deg(tt.steerAngle)); grid on; xlabel('t [s]'); ylabel('steer [deg]');
subplot(2, 2, 4);
plot(seconds(tt.t), tt.driveForce, seconds(tt.t), -tt.brakeForce); grid on;
xlabel('t [s]'); ylabel('force [N]'); legend('drive', 'brake');
