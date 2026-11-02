function p = defaultCar()
%DEFAULTCAR Default car setup. Edit the values below to change the car.
    p = vehicle.VehicleParams();

    % --- geometry / mass -------------------------------------------------
    p.mass              = 1200;        % kg
    p.wheelbase         = 2.6;         % m
    p.length            = 4.2;         % m
    p.trackWidth        = 1.5;         % m
    p.rearWheelDiameter = 0.65;        % m
    p.cgFrontFraction   = 0.48;        % -
    p.cgHeight          = 0.5;         % m

    % --- steering / tyres / aero ------------------------------------------
    p.maxSteerAngle     = deg2rad(30); % rad
    p.grip              = 1.2;         % -
    p.CdA               = 0.65;        % m^2

    % --- powertrain / brakes ----------------------------------------------
    p.maxMotorTorque    = 350;         % N*m
    p.gearRatio         = 9;           % -
    p.maxPower          = 150e3;       % W
    p.maxBrakeForce     = 14e3;        % N
    p.brakeBias         = 0.6;         % -

    % --- start ------------------------------------------------------------
    p.initialSpeed      = 20;          % m/s (limited to the target speed at the line)

    p.validate();
end
