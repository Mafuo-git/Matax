function [D, meta] = DataParserMain(filePath)
%DATAPARSERMAIN  Point d'entree unique : choix du fichier, detection du format, parsing.
%
%   [D, meta] = DataParserMain()           % ouvre une fenetre uigetfile (callback)
%   [D, meta] = DataParserMain(filePath)   % fichier donne directement
%
%   Formats reconnus (voir detectFileFormat plus bas) :
%     SimLap       -> parseSimLap     (.mat)
%     CarSim       -> parseCarSim     (.mat)
%     AssettoCorsa -> parseAssetto    (.mat ou .csv MoTeC)
%
%   Sortie D : struct de vecteurs colonnes (meme longueur N, SI, repere
%   vehicule : x vers l'avant, y vers la gauche, angles/lacet positifs a
%   gauche, t demarre a 0) :
%
%     Requis   : t, lap, lapTime, x, y, s, yaw, yawRate, speed, ax, ay,
%                steerAngle, driveForceCmd, brakeForce
%     Optionnel: driveForce, motorTorque, wheelTorque  (present seulement
%                si la source permet de le calculer)
%
%   Un champ requis non disponible dans la source est rempli par NaN et
%   signale dans la console. meta (2e sortie) decrit la source et les
%   hypotheses de conversion ; il est separe de D pour que D ne contienne
%   que des signaux.
%
%   Unites : t,lapTime [s] ; x,y,s [m] ; yaw [rad] ; yawRate [rad/s] ;
%   speed [m/s] ; ax,ay [m/s^2] ; steerAngle [rad] (roue) ;
%   driveForceCmd, brakeForce, driveForce [N] ; motorTorque, wheelTorque [N*m].

    persistent lastFolder

    % ---- 1) Choix du fichier ------------------------------------------
    if nargin < 1 || isempty(filePath)
        if isempty(lastFolder) || ~isfolder(lastFolder)
            lastFolder = pwd;
        end
        [fileName, folder] = uigetfile( ...
            {'*.mat;*.csv', 'Fichiers de telemetrie (*.mat, *.csv)'; ...
             '*.mat',       'Fichiers MATLAB (*.mat)'; ...
             '*.csv',       'Fichiers CSV (*.csv)'}, ...
            'Selectionner un fichier de donnees', lastFolder);
        if isequal(fileName, 0)
            fprintf('Aucun fichier selectionne.\n');
            D = [];
            meta = [];
            return
        end
        lastFolder = folder;
        filePath = fullfile(folder, fileName);
    end
    filePath = char(filePath);

    % ---- 2) Detection du format ---------------------------------------
    fmt = detectFileFormat(filePath);

    % ---- 3) Parsing ----------------------------------------------------
    switch fmt
        case "SimLap"
            [D, meta] = parseSimLap(filePath);
        case "CarSim"
            [D, meta] = parseCarSim(filePath);
        case "AssettoCorsa"
            [D, meta] = parseAssetto(filePath);
        otherwise
            error('DataParserMain:unknownFormat', 'Format non gere : %s', fmt);
    end

    % ---- 4) Controle / mise en forme commune ---------------------------
    [D, meta] = completeAndCheck(D, meta, fmt, filePath);
end


%% ========================================================================
%  Mise en forme commune des sorties des parseurs
%% ========================================================================
function [D, meta] = completeAndCheck(Din, meta, fmt, filePath)

    req = {'t','lap','lapTime','x','y','s','yaw','yawRate','speed','ax','ay', ...
           'steerAngle','driveForceCmd','brakeForce'};
    opt = {'driveForce','motorTorque','wheelTorque'};

    if ~isfield(Din, 't') || isempty(Din.t)
        error('DataParserMain:noTime', 'Le parseur %s n''a pas fourni de vecteur temps.', fmt);
    end
    N = numel(Din.t);

    D = struct();
    missing = {};
    for i = 1:numel(req)
        f = req{i};
        if isfield(Din, f) && ~isempty(Din.(f))
            v = double(Din.(f)(:));
            if numel(v) ~= N
                error('DataParserMain:badLength', ...
                      'Champ "%s" : %d valeurs au lieu de %d.', f, numel(v), N);
            end
            D.(f) = v;
        else
            D.(f) = NaN(N, 1);
            missing{end+1} = f; %#ok<AGROW>
        end
    end

    optPresent = {};
    for i = 1:numel(opt)
        f = opt{i};
        if isfield(Din, f) && ~isempty(Din.(f))
            D.(f) = double(Din.(f)(:));
            optPresent{end+1} = f; %#ok<AGROW>
        end
    end

    if any(diff(D.t) <= 0)
        warning('DataParserMain:timeNotIncreasing', ...
                'Le vecteur temps n''est pas strictement croissant.');
    end

    % ---- meta ----------------------------------------------------------
    meta.format      = char(fmt);
    meta.file        = filePath;
    meta.nSamples    = N;
    meta.duration    = D.t(end) - D.t(1);
    meta.fs          = 1 / median(diff(D.t));
    meta.laps        = unique(D.lap(~isnan(D.lap)))';
    meta.missingReq  = missing;
    meta.optPresent  = optPresent;

    % ---- resume console ------------------------------------------------
    [~, name, ext] = fileparts(filePath);
    fprintf('[%s%s] %d echantillons | %.2f s | ~%.0f Hz | tour(s) : %s\n', ...
            name, ext, N, meta.duration, meta.fs, num2str(meta.laps));
    if ~isempty(missing)
        fprintf('  Champs requis absents de la source (remplis par NaN) : %s\n', ...
                strjoin(missing, ', '));
    end
    if isempty(optPresent)
        fprintf('  Champs optionnels disponibles : aucun\n');
    else
        fprintf('  Champs optionnels disponibles : %s\n', strjoin(optPresent, ', '));
    end
end


%% ========================================================================
%  Detection du format (ex-fonction detectFileFormat)
%% ========================================================================
function fmt = detectFileFormat(filePath)
%   .csv                       -> "AssettoCorsa"  (export MoTeC CSV)
%   .mat, variable 'Data'      -> "SimLap"
%   .mat, telemetry/setup/meta -> "CarSim"
%   .mat, structs Time/Value/Units (Ground_Speed, Engine_RPM, ...)
%                              -> "AssettoCorsa"

    filePath = string(filePath);
    if ~isscalar(filePath) || ~isfile(filePath)
        error('DataParserMain:fileNotFound', 'Fichier introuvable : %s', filePath);
    end

    [~, name, ext] = fileparts(char(filePath));

    switch lower(ext)
        case '.csv'
            fmt = "AssettoCorsa";
        case '.mat'
            fmt = detectMatFormat(filePath);
        otherwise
            error('DataParserMain:unsupportedExtension', ...
                  'Extension "%s" non supportee (attendu : .csv ou .mat) : %s', ...
                  ext, filePath);
    end

    fprintf('[%s%s] Format detecte : %s\n', name, ext, fmt);
end


function fmt = detectMatFormat(filePath)
% Identifie le format d'un .mat a partir de la liste des variables (whos,
% sans charger le fichier), puis verifie la structure d'une variable cle.

    try
        info = whos('-file', char(filePath));
    catch ME
        error('DataParserMain:unreadableMat', ...
              'Fichier .mat illisible (%s) : %s', filePath, ME.message);
    end

    names = {info.name};

    % --- CarSim : telemetry (timetable/table) + setup + meta ---------------
    if all(ismember({'telemetry', 'setup', 'meta'}, names)) && ...
            ismember(classOf(info, 'telemetry'), {'timetable', 'table'})
        fmt = "CarSim";
        return
    end

    % --- SimLap : une unique struct 'Data' (Time, PositionX, ...) ----------
    if isequal(names, {'Data'}) && strcmp(classOf(info, 'Data'), 'struct')
        s = load(char(filePath), 'Data');
        if hasFields(s.Data, {'Time', 'PositionX', 'PositionY', 'SteeringAngle'})
            fmt = "SimLap";
            return
        end
    end

    % --- Assetto Corsa : 1 struct {Time, Value, Units} par canal -----------
    keyChannels = {'Ground_Speed', 'Engine_RPM', 'Throttle_Pos', 'Brake_Pos'};
    if all(ismember(keyChannels, names))
        s = load(char(filePath), keyChannels{1});
        if hasFields(s.(keyChannels{1}), {'Time', 'Value', 'Units'})
            fmt = "AssettoCorsa";
            return
        end
    end

    error('DataParserMain:unknownMatFormat', ...
          ['Format .mat non reconnu (%s).\n' ...
           'Variables trouvees : %s'], ...
          filePath, strjoin(names(1:min(end, 10)), ', '));
end


function tf = hasFields(s, requiredFields)
    tf = isstruct(s) && all(isfield(s, requiredFields));
end


function c = classOf(info, varName)
    c = info(find(strcmp({info.name}, varName), 1)).class;
end
