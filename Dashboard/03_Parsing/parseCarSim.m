function [D, meta] = parseCarSim(filePath)
%PARSECARSIM  Lit un fichier CarSim (.mat : telemetry + setup + meta) et
%   renvoie les signaux au format commun du dashboard (voir DataParserMain).
%
%   'telemetry' est un timetable dont les variables portent deja les noms
%   attendus (lap, lapTime, x, y, s, yaw, yawRate, speed, ax, ay,
%   steerAngle, driveForceCmd, brakeForce, driveForce, motorTorque,
%   wheelTorque) : aucune conversion d'unite, seul t est reconstruit a
%   partir des RowTimes (en secondes, ramene a 0 au premier echantillon).
%
%   Necessite MATLAB (lecture d'un timetable).

    S = load(filePath, 'telemetry', 'setup', 'meta');
    TT = S.telemetry;

    if istimetable(TT)
        rt = TT.Properties.RowTimes;
    else
        % table classique : on cherche une colonne temps
        if ~ismember('t', TT.Properties.VariableNames)
            error('parseCarSim:noTime', 'Ni timetable ni colonne "t" dans telemetry.');
        end
        rt = TT.t;
    end

    if isduration(rt)
        tAbs = seconds(rt);
    elseif isdatetime(rt)
        tAbs = seconds(rt - rt(1));
    else
        tAbs = double(rt);
    end
    tAbs = tAbs(:);

    D = struct();
    D.t = tAbs - tAbs(1);

    wanted = {'lap','lapTime','x','y','s','yaw','yawRate','speed','ax','ay', ...
              'steerAngle','driveForceCmd','brakeForce', ...
              'driveForce','motorTorque','wheelTorque'};
    available = TT.Properties.VariableNames;
    for i = 1:numel(wanted)
        f = wanted{i};
        if ismember(f, available)
            D.(f) = double(TT.(f)(:));
        end
    end

    meta = struct();
    meta.t0    = tAbs(1);
    meta.setup = S.setup;      % parametres vehicule / piste / pilote
    meta.info  = S.meta;       % version, dates, nb de lignes, unites
    meta.notes = { ...
        't : RowTimes en secondes, decale pour commencer a 0 (t0 dans meta.t0)', ...
        'autres champs : variables du timetable telles quelles'};
end
