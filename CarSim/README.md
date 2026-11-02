# CarSim - 2D lap simulator (MATLAB)

Point-mass car with per-wheel forces, automatic driver, real-time top-view app, 1 kHz telemetry export.

## Quick start

```matlab
setupProject            % adds the project to the path
makeTestTracks          % creates data/tracks/stadium.mat and complex.mat
runHeadless             % script run: 2 laps, timing per step, export, plots
app = CarSimApp;        % real-time UI
```

`runHeadless` prints the cost of one step. It must stay far below 1000 us to run in real time.

## Layout

| Folder | Role |
|---|---|
| `CarSimApp.m` | UI only, no physics |
| `+vehicle` | `VehicleParams`, `VehicleModel` (abstract), `PointMassVehicle`, `Powertrain`, `BrakeSystem`, `Aero`, `WheelLoadModel`, `+presets/defaultCar.m` |
| `+track` | `Track` (resampling, s / heading / curvature, projection), `TrackLibrary` |
| `+driver` | `Driver` (abstract), `SpeedProfile`, `PurePursuitDriver` |
| `+simulation` | `Simulator`, `SimConfig`, `LapCounter` |
| `+telemetry` | `SignalCatalog` (columns + units), `Recorder`, `Exporter` |
| `scripts` | `makeTestTracks`, `runHeadless` |
| `data/tracks` | one `.mat` per circuit: `centerline` (Nx2, m, closed, counter-clockwise), `width` (m), `name` |
| `data/sessions` | recorded sessions |

Dependencies point one way only: `CarSimApp -> Simulator -> {VehicleModel, Driver, Track, LapCounter, Recorder}`.

## Change the car

Edit `+vehicle/+presets/defaultCar.m` only. `initialSpeed` is capped by the target speed at the line.

## Add a circuit

Save a `.mat` with `centerline`, `width`, `name` in `data/tracks/`. The first point is the start/finish line.
Every bend must be wider than `wheelbase / tan(maxSteerAngle)` or loading fails with an explicit error.

## Add another vehicle model

Subclass `vehicle.VehicleModel` (`step`, `initialState`, `describe`) and pass it to `simulation.Simulator`.
Nothing else changes.

## Conventions

* SI units internally. `speed_kmh`, `steer_deg`, `yaw_deg`, `yawRate_degps` are added in the CSV only.
* Reference point (x, y): centre of the rear axle. yaw: rad, counter-clockwise from +x.
* Wheel order: FL, FR, RL, RR. Fx > 0 forward, Fy > 0 left, lateralOffset > 0 left of the centre line.
* Step = recording step = 1 ms, simulated time (not wall time).

## Session files (`data/sessions`)

| File | Content |
|---|---|
| `session_yyyyMMdd_HHmmss.mat` | `telemetry` (timetable with units), `laps`, `setup`, `meta` |
| `session_..csv` | telemetry table (30 SI columns + 4 converted columns) |
| `session_.._laps.csv` | `lap`, `lapTime`, `complete` |
| `session_.._setup.json` | car / track / driver / simulation setup, conventions, units |

A lap is `complete = false` when the recording started or stopped in the middle of it.

## Model summary

* Steering: kinematic bicycle, `yawRate = v tan(delta) / L`, `ay = v * yawRate`.
* Tyres: friction circle per wheel, `sqrt(Fx^2 + Fy^2) <= mu Fz`, lateral first.
* Loads: static split + `m ax h / L` + `m ay h / t`; computed from the previous step's `ax`.
* Traction on the rear wheels only, limited by torque, power and wheel grip. Braking on four wheels with a fixed bias.
* Driver: quasi-static speed profile (corner limit, forward and backward passes) + Pure Pursuit + feed-forward / PI.
* Lap time resolution is better than 1 ms (the line crossing time is interpolated).
