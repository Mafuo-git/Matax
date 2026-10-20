function [D, meta] = parseSimLap(filePath)
%PARSESIMLAP  Lit un fichier SimLap (.mat, variable 'Data') et renvoie les
%   signaux au format commun du dashboard (voir DataParserMain).
%
%   Source : struct Data {Time, PositionX/Y, SpeedX/Y, AccelerationX/Y,
%            LatAcceleration, Yaw, SteeringAngle, Torque}, repere monde.
%
%   Conversions :
%     t          = Time - Time(1)
%     lap        = 1 (un seul tour dans le fichier)
%     lapTime    = t
%     x, y       = PositionX, PositionY
%     s          = longueur de trajectoire cumulee (a partir de x, y)
%     yaw        = Yaw
%     yawRate    = d(yaw)/dt (non fourni, derive)
%     speed      = hypot(SpeedX, SpeedY)
%     ax, ay     = AccelerationX/Y (monde) tournees dans le repere vehicule
%                  (ay > 0 a gauche). LatAcceleration n'est pas utilise :
%                  elle est saturee a +-20 m/s^2.
%     steerAngle = SteeringAngle (rad, suppose angle de roue)
%     driveForceCmd = Torque, SANS conversion (unite d'origine, non precisee)
%     brakeForce, driveForce, motorTorque, wheelTorque : non disponibles

    S = load(filePath, 'Data');
    R = S.Data;

    t   = R.Time(:);
    x   = R.PositionX(:);
    y   = R.PositionY(:);
    yaw = R.Yaw(:);

    D = struct();
    D.t       = t - t(1);
    D.lap     = ones(size(t));
    D.lapTime = D.t;
    D.x       = x;
    D.y       = y;
    D.s       = [0; cumsum(hypot(diff(x), diff(y)))];
    D.yaw     = yaw;
    D.yawRate = gradient(unwrap(yaw), D.t);
    D.speed   = hypot(R.SpeedX(:), R.SpeedY(:));

    % Acceleration monde -> repere vehicule
    axW = R.AccelerationX(:);
    ayW = R.AccelerationY(:);
    D.ax = axW .* cos(yaw) + ayW .* sin(yaw);
    D.ay = -axW .* sin(yaw) + ayW .* cos(yaw);

    D.steerAngle    = R.SteeringAngle(:);
    D.driveForceCmd = R.Torque(:);

    meta = struct();
    meta.t0 = t(1);
    meta.notes = { ...
        'ax/ay : acceleration monde tournee par yaw (repere vehicule)', ...
        'yawRate : derive numerique de yaw', ...
        's : longueur de trajectoire cumulee depuis le 1er echantillon', ...
        'driveForceCmd : canal Torque brut (unite inconnue, aucune conversion)', ...
        'brakeForce : absent de la source'};
end
