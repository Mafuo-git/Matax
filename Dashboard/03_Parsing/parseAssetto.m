function [D, meta] = parseAssetto(filePath)
%PARSEASSETTO  Lit un export Assetto Corsa (.mat ou .csv MoTeC) et renvoie
%   les signaux au format commun du dashboard (voir DataParserMain).
%
%   .mat : une struct {Time, Value, Units} par canal, frequences mixtes
%          (1/10/30/50 Hz) -> reechantillonnage lineaire sur la grille du
%          canal le plus rapide.
%   .csv : export MoTeC, tous les canaux deja sur la meme grille (50-80 Hz).
%
%   Les exports n'ont pas tous les memes canaux (selon la voiture et le
%   logger). Indispensables : Ground_Speed, Car_Coord_X/Y, Chassis_Velocity_X/Y,
%   Chassis_Yaw_Rate, CG_Accel_Longitudinal/Lateral, Steering_Angle, Lap_Time.
%   Le reste a un repli (voir ci-dessous) ; chaque repli utilise est
%   signale dans la console et dans meta.fallbacks.
%
%   Conversions (D.<champ> <- canal Assetto) :
%     t          <- grille maitre, ramenee a 0
%     lap        <- Lap_Number                  | repli : Session_Lap_Count + 1
%     lapTime    <- Lap_Time
%     s          <- Lap_Distance                | repli : Distance - Distance
%                                                 au debut du tour (note 4)
%     x, y       <- Car_Coord_X, Car_Coord_Y    (Car_Coord_Z = altitude)
%     speed      <- Ground_Speed / 3.6
%     yaw        <- cap de la trajectoire + angle de derive (voir note 1)
%     yawRate    <- -Chassis_Yaw_Rate (deg/s -> rad/s, signe inverse)
%     ax         <- CG_Accel_Longitudinal * g
%     ay         <- CG_Accel_Lateral * g        (> 0 a gauche)
%     steerAngle <- -Steering_Angle / rapport de direction  (note 2)
%     brakeForce <- somme_roues( Brake_Torque / rayon pneu charge )
%                                               | repli : Brake_Pos (en %, note 5)
%     driveForce <- Puissance moteur / vitesse circonferentielle roues AR
%                   (seulement si Drivetrain_Torque/Power sont presents)
%     motorTorque<- max(Drivetrain_Torque, 0)   (idem)
%     wheelTorque<- driveForce * rayon roues AR (idem)
%     driveForceCmd <- Throttle_Pos/100 * max(driveForce)   (note 3)
%                                               | repli : Throttle_Pos (en %)
%     rpm        <- Engine_RPM                  (champ optionnel, tr/min)
%
%   Notes :
%     1) Le canal Car_Heading contient des echantillons aberrants (sauts
%        de plus de 100 deg) et n'existe pas dans tous les exports : le
%        lacet est reconstruit a partir de la direction de deplacement
%        (derivee de x, y, lissee sur ~0.15 s) + atan2(Vy, Vx) du chassis.
%        Equivalent a -Car_Heading - 90 deg a ~2 deg pres.
%     2) Steering_Angle est l'angle VOLANT (positif a droite). Le rapport
%        de direction est estime par regression sur l'angle de pincement
%        (Toe_In_FL, Toe_In_FR) ; repli : DEFAULT_STEER_RATIO.
%     3) Assetto n'a pas de consigne en newtons : la consigne est la
%        position de la pedale mise a l'echelle de la force motrice maximale
%        observee. Propulsion supposee (roues arriere).
%     4) Session_Lap_Count vaut 0 pendant le premier (demi-)tour : on ajoute 1
%        pour numeroter les tours a partir de 1. Un tour qui ne commence pas
%        a la ligne (depart de session) sera marque "incomplete" par le
%        dashboard.
%     5) Sans couple de frein dans l'export, brakeForce est la position de
%        la pedale en % (pas des newtons). Le dashboard la normalise par son
%        maximum, seule l'allure de la courbe est donc significative.

    DEFAULT_STEER_RATIO = 13.5;
    G = 9.80665;
    HP_TO_W = 745.699872;
    fallbacks = {};

    % ---- lecture -> canaux {t, v} --------------------------------------
    [~, ~, ext] = fileparts(char(filePath));
    switch lower(ext)
        case '.mat'
            ch = readAssettoMat(filePath);
        case '.csv'
            ch = readAssettoCsv(filePath);
        otherwise
            error('parseAssetto:badExtension', 'Extension "%s" non geree.', ext);
    end

    names = fieldnames(ch);
    nPts  = cellfun(@(f) numel(ch.(f).t), names);
    [~, kMax] = max(nPts);
    tM  = ch.(names{kMax}).t;          % grille maitre (canal le plus rapide)
    t   = tM - tM(1);
    N   = numel(tM);
    fs  = 1 / median(diff(t));

    g = @(name) getChannel(ch, name, tM);
    hasAll = @(list) all(isfield(ch, list));

    need = {'Ground_Speed','Car_Coord_X','Car_Coord_Y','Chassis_Velocity_X', ...
            'Chassis_Velocity_Y','Chassis_Yaw_Rate','CG_Accel_Longitudinal', ...
            'CG_Accel_Lateral','Steering_Angle','Lap_Time'};
    lacks = need(~isfield(ch, need));
    if ~isempty(lacks)
        error('parseAssetto:missingChannel', ...
              'Canaux indispensables absents : %s', strjoin(lacks, ', '));
    end

    % ---- tours : grandeurs qui se remettent a zero ---------------------
    D = struct();
    D.t       = t;
    D.lapTime = resampleWrapped(ch.Lap_Time, tM);

    if isfield(ch, 'Lap_Number')
        D.lap = round(resamplePrevious(ch.Lap_Number, tM));
    elseif isfield(ch, 'Session_Lap_Count')
        D.lap = round(resamplePrevious(ch.Session_Lap_Count, tM)) + 1;
        fallbacks{end+1} = 'lap <- Session_Lap_Count + 1 (Lap_Number absent)';
    else
        error('parseAssetto:noLapInfo', ...
              'Aucun canal de numero de tour (Lap_Number ou Session_Lap_Count).');
    end

    if isfield(ch, 'Lap_Distance')
        D.s = resampleWrapped(ch.Lap_Distance, tM);
    elseif isfield(ch, 'Distance')
        dist = g('Distance');
        D.s = dist;
        ids = unique(D.lap);
        for k = 1:numel(ids)
            m = D.lap == ids(k);
            D.s(m) = dist(m) - dist(find(m, 1));
        end
        fallbacks{end+1} = 's <- Distance - Distance au debut du tour (Lap_Distance absent)';
    else
        error('parseAssetto:noDistance', ...
              'Aucun canal de distance (Lap_Distance ou Distance).');
    end

    % ---- position, vitesse ---------------------------------------------
    x     = g('Car_Coord_X');
    y     = g('Car_Coord_Y');
    speed = g('Ground_Speed') / 3.6;
    D.x = x;
    D.y = y;
    D.speed = speed;

    % ---- lacet ---------------------------------------------------------
    % Derivee de la position bruitee (quantification) : lissage ~0.15 s.
    w      = max(1, round(0.15 * fs));
    vxw    = movmean(gradient(x, t), w);
    vyw    = movmean(gradient(y, t), w);
    course = unwrap(atan2(vyw, vxw));
    beta   = atan2(g('Chassis_Velocity_Y'), g('Chassis_Velocity_X'));
    yaw    = course + beta;
    yaw(speed < 1) = NaN;                       % direction indefinie a l'arret
    yaw    = fillmissing(yaw, 'nearest');
    D.yaw  = atan2(sin(yaw), cos(yaw));         % ramene dans [-pi, pi]
    D.yawRate = -g('Chassis_Yaw_Rate') * pi/180;

    % ---- accelerations -------------------------------------------------
    D.ax = g('CG_Accel_Longitudinal') * G;
    D.ay = g('CG_Accel_Lateral') * G;

    % ---- direction -----------------------------------------------------
    steerWheelDeg = g('Steering_Angle');
    ratio = DEFAULT_STEER_RATIO;
    ratioSource = 'valeur par defaut';
    if hasAll({'Toe_In_FL','Toe_In_FR'})
        roadDeg = (g('Toe_In_FL') - g('Toe_In_FR')) / 2;
        if std(roadDeg) > 0
            c = corrcoef(roadDeg, steerWheelDeg);
            p = polyfit(roadDeg, steerWheelDeg, 1);
            if abs(c(1,2)) > 0.95 && isfinite(p(1)) && abs(p(1)) > 1
                ratio = abs(p(1));
                ratioSource = 'regression sur Toe_In_FL/FR';
            end
        end
    end
    D.steerAngle = -steerWheelDeg * pi/180 / ratio;

    % ---- freinage ------------------------------------------------------
    wheels = {'FL','FR','RL','RR'};
    brakeNeed = [strcat('Brake_Torque_', wheels), strcat('Tire_Loaded_Radius_', wheels)];
    if hasAll(brakeNeed)
        bf = zeros(N, 1);
        for i = 1:numel(wheels)
            wh = wheels{i};
            bf = bf + g(['Brake_Torque_' wh]) ./ (g(['Tire_Loaded_Radius_' wh]) / 1000);
        end
        D.brakeForce = bf;
    elseif isfield(ch, 'Brake_Pos')
        D.brakeForce = g('Brake_Pos');
        fallbacks{end+1} = 'brakeForce <- Brake_Pos en % (Brake_Torque absent)';
    end

    % ---- motricite -----------------------------------------------------
    driveNeed = {'Drivetrain_Torque','Drivetrain_Power','Wheel_Angular_Speed_RL', ...
                 'Wheel_Angular_Speed_RR','Tire_Loaded_Radius_RL','Tire_Loaded_Radius_RR'};
    if hasAll(driveNeed)
        rRear  = (g('Tire_Loaded_Radius_RL') + g('Tire_Loaded_Radius_RR')) / 2000;   % m
        wRear  = (g('Wheel_Angular_Speed_RL') + g('Wheel_Angular_Speed_RR')) / 2;    % rad/s
        vWheel = max(wRear .* rRear, 1);                                             % m/s
        power  = max(g('Drivetrain_Power'), 0) * HP_TO_W;                            % W

        D.motorTorque = max(g('Drivetrain_Torque'), 0);
        D.driveForce  = power ./ vWheel;
        D.wheelTorque = D.driveForce .* rRear;
    else
        fallbacks{end+1} = 'driveForce, motorTorque, wheelTorque non calcules (Drivetrain_Torque/Power absents)';
    end

    % ---- regime moteur (mesure directe, independante des couples) --------
    if isfield(ch, 'Engine_RPM')
        D.rpm = g('Engine_RPM');
    else
        fallbacks{end+1} = 'rpm non disponible (Engine_RPM absent)';
    end

    if isfield(ch, 'Throttle_Pos')
        throttle = min(max(g('Throttle_Pos'), 0), 100) / 100;
        if isfield(D, 'driveForce')
            D.driveForceCmd = throttle * max(D.driveForce);
        else
            D.driveForceCmd = 100 * throttle;
            fallbacks{end+1} = 'driveForceCmd <- Throttle_Pos en % (pas de force motrice)';
        end
    end

    % ---- infos -----------------------------------------------------------
    for i = 1:numel(fallbacks)
        fprintf('  [parseAssetto] repli : %s\n', fallbacks{i});
    end

    meta = struct();
    meta.t0 = tM(1);
    meta.steerRatio = ratio;
    meta.steerRatioSource = ratioSource;
    meta.fallbacks = fallbacks;
    meta.notes = { ...
        'grille temporelle : celle du canal le plus rapide, t ramene a 0', ...
        'yaw : trajectoire lissee + angle de derive', ...
        'yawRate, steerAngle : signes inverses vs Assetto (positif a gauche)', ...
        sprintf('steerAngle : rapport de direction %.2f (%s)', ratio, ratioSource), ...
        'driveForce : puissance / vitesse roues AR (propulsion supposee)', ...
        'driveForceCmd : pedale * force motrice max observee'};
end


%% ========================================================================
%  Lecture
%% ========================================================================
function ch = readAssettoMat(filePath)
% ch.(nom) = struct('t', temps(:), 'v', valeurs(:)) pour chaque canal.

    S = load(filePath);
    ch = struct();
    f = fieldnames(S);
    for i = 1:numel(f)
        c = S.(f{i});
        if isstruct(c) && all(isfield(c, {'Time', 'Value'}))
            ch.(f{i}) = struct('t', double(c.Time(:)), 'v', double(c.Value(:)));
        end
    end
    if isempty(fieldnames(ch))
        error('parseAssetto:noChannel', 'Aucun canal {Time, Value} dans %s.', filePath);
    end
end


function ch = readAssettoCsv(filePath)
% Export MoTeC : en-tete (metadonnees), ligne des noms ("Time","Distance",...),
% ligne des unites, lignes vides, puis les donnees entre guillemets.
%
% Lecture par octets + sscanf : un export de 160 Mo (130 000 lignes x 170
% canaux) reste lisible, ce qui n'est pas le cas d'un decoupage ligne par ligne.

    fid = fopen(filePath, 'r');
    if fid < 0
        error('parseAssetto:cannotOpen', 'Impossible d''ouvrir %s.', filePath);
    end
    raw = fread(fid, Inf, '*uint8')';
    fclose(fid);

    % -- en-tete : ligne des noms puis ligne des unites ------------------
    head = char(raw(1:min(numel(raw), 100000)));
    p = regexp(head, '(?<=\n)"Time"', 'once');
    if isempty(p)
        error('parseAssetto:badCsv', 'Ligne d''en-tete "Time" introuvable dans %s.', filePath);
    end
    e1 = p - 1 + find(raw(p:end) == 10, 1);          % fin de la ligne des noms
    e2 = e1 + find(raw(e1+1:end) == 10, 1);          % fin de la ligne des unites
    tok   = regexp(char(raw(p:e1-1)), '"([^"]*)"', 'tokens');
    names = [tok{:}];
    nC    = numel(names);

    % -- donnees : on retire guillemets et CR, les fins de ligne deviennent
    %    des virgules, puis un seul sscanf lit tout -----------------------
    body = raw(e2+1:end);
    clear raw
    body = body(find(body ~= 10 & body ~= 13, 1):end);   % saute les lignes vides
    body(body == 34 | body == 13) = [];
    body(body == 10) = 44;
    vals = sscanf(char(body), '%f,');
    clear body

    if isempty(vals) || mod(numel(vals), nC) ~= 0
        error('parseAssetto:badCsv', ...
              ['Lignes de donnees incoherentes dans %s (%d valeurs lues pour %d colonnes). ' ...
               'Valeur vide ou ligne tronquee ?'], filePath, numel(vals), nC);
    end
    M = reshape(vals, nC, []).';
    clear vals

    t  = M(:, 1);
    ch = struct();
    for j = 2:nC                                       % col 2 = Distance cumulee, gardee
        fld = matlab.lang.makeValidName(strrep(names{j}, ' ', '_'));
        ch.(fld) = struct('t', t, 'v', M(:, j));
    end
end


%% ========================================================================
%  Reechantillonnage
%% ========================================================================
function v = getChannel(ch, name, tM)
% Canal sur la grille maitre (interpolation lineaire).
    if ~isfield(ch, name)
        error('parseAssetto:missingChannel', 'Canal "%s" absent.', name);
    end
    c = ch.(name);
    if sameGrid(c.t, tM)
        v = c.v;
    else
        v = interp1(c.t, c.v, tM, 'linear', 'extrap');
    end
end


function v = resamplePrevious(c, tM)
% Grandeur par paliers (numero de tour) : derniere valeur connue.
    if sameGrid(c.t, tM)
        v = c.v;
        return
    end
    v = interp1(c.t, c.v, tM, 'previous', 'extrap');
    v = fillmissing(v, 'nearest');
end


function v = resampleWrapped(c, tM)
% Grandeur qui retombe a ~0 a chaque tour (Lap_Time, Lap_Distance) : une
% interpolation lineaire directe creerait des valeurs absurdes au passage
% de la ligne. On "deroule" les remises a zero, on interpole, puis on
% remet les decalages.
    if sameGrid(c.t, tM)
        v = c.v;
        return
    end
    tS = c.t;
    vS = c.v;
    off = zeros(size(vS));
    resets = find(diff(vS) < -0.5 * max(abs(vS)));
    for k = resets(:)'
        off(k+1:end) = off(k+1:end) + vS(k);
    end
    uq   = interp1(tS, vS + off, tM, 'linear', 'extrap');
    offq = interp1(tS, off, tM, 'previous', 'extrap');
    offq = fillmissing(offq, 'nearest');
    v = uq - offq;
end


function tf = sameGrid(t1, t2)
    tf = numel(t1) == numel(t2) && all(abs(t1 - t2) < 1e-9);
end
