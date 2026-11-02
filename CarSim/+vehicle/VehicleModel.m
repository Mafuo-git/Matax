classdef (Abstract) VehicleModel < handle
    %VEHICLEMODEL Interface every vehicle dynamics model must implement.
    %   Swap PointMassVehicle for a bicycle / 4-wheel model without touching
    %   the driver, the simulator or the app.
    %
    %   state : struct (x y yaw v s lateralOffset idx ax ay yawRate t lap)
    %   cmd   : struct (driveForce brakeForce steerAngle)
    %   out   : struct (steerAngle driveForce brakeForce motorTorque wheelTorque
    %                   Fx Fy Fz yawRate)   -- Fx/Fy/Fz are 1x4 [FL FR RL RR]
    methods (Abstract)
        [state, out] = step(obj, state, cmd, dt)
        state = initialState(obj, trk, speedLimit)
        s = describe(obj)
    end
end
