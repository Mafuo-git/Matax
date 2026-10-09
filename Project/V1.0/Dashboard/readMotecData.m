function D = readMotecData(input)
% READMOTECDATA Import a local MoTeC .ld file or a DataMaster MAT export.

if ischar(input) || (isstring(input) && isscalar(input))
    file = char(input);
    [~,~,ext] = fileparts(file);
    if strcmpi(ext,'.ld')
        if ~ispc
            error('MoTeC .ld import requires Windows and the MoTeC i2 COM interface.');
        end
        try
            i2 = actxserver('MoTeC.i2Application');
        catch ME
            error('Could not start MoTeC i2. Install MoTeC i2 and enable its COM interface. (%s)',ME.message);
        end
        matFile = [tempname '.mat'];
        save(matFile,'matFile');
        cleanup = onCleanup(@() cleanupExport(i2,matFile)); %#ok<NASGU>
        i2.Visible = 0;
        i2.DataSources.Open(file);
        i2.DataSources.ExportMainAsMAT(matFile);
        i2.DataSources.CloseAll;
        S = load(matFile);
    else
        S = load(file);
    end
else
    S = input;
end

required = {'t','lap','lapTime','x','y','s','yaw','yawRate','speed','ax','ay', ...
    'steerAngle','driveForceCmd','brakeForce'};
if all(isfield(S,required))
    D = struct();
    for k = 1:numel(required), D.(required{k}) = double(S.(required{k})(:)); end
    optional = {'driveForce','motorTorque','wheelTorque'};
    for k = 1:numel(optional)
        if isfield(S,optional{k}), D.(optional{k}) = double(S.(optional{k})(:)); end
    end
    return
end
names = fieldnames(S);
for k = 1:numel(names)
    value = S.(names{k});
    if istable(value), value = table2struct(value,'ToScalar',true); end
    if isstruct(value) && isscalar(value) && all(isfield(value,required))
        D = struct();
        for j = 1:numel(required), D.(required{j}) = double(value.(required{j})(:)); end
        optional = {'driveForce','motorTorque','wheelTorque'};
        for j = 1:numel(optional)
            if isfield(value,optional{j}), D.(optional{j}) = double(value.(optional{j})(:)); end
        end
        return
    end
end

channels = collectChannels(S);
if isempty(channels)
    error('No DataMaster channels found. Expected channels with Time, Value, and Units fields.');
end

clockIndex = find(ismember(cellfun(@normalise,{channels.Name},'UniformOutput',false), ...
    {'time','sessiontime','elapsedtime'}),1);
if ~isempty(clockIndex) && numel(channels(clockIndex).Value) > 1
    baseTime = channels(clockIndex).Value(:);
else
    [~,clockIndex] = max(arrayfun(@(c) numel(c.Time),channels));
    baseTime = channels(clockIndex).Time(:);
end
baseTime = baseTime(isfinite(baseTime));
baseTime = unique(baseTime,'sorted');
if numel(baseTime) < 20
    error('The MoTeC export does not contain a usable time base.');
end
baseTime = baseTime - baseTime(1);

D = struct();
D.t = baseTime;
[D.lap,lapUnit] = channel(channels,{'Lap','Lap Number','Lap No','Lap Count'},baseTime,false); %#ok<ASGLU>
[D.lapTime,lapTimeUnit] = channel(channels,{'Lap Time','Current Lap Time'},baseTime,false);
[D.s,sUnit] = channel(channels,{'Lap Distance','Distance','Distance Travelled','Lap Dist'},baseTime,false);
[D.speed,speedUnit] = channel(channels,{'Ground Speed','GPS Speed','Vehicle Speed','Speed'},baseTime,false);
[D.steerAngle,steerUnit] = channel(channels,{'Steering Angle','Steer Angle','Steering Pos','Steering'},baseTime,false);
[D.driveForceCmd,~] = channel(channels,{'Throttle Pos','Throttle Position','Throttle','Accelerator','Throttle Pedal'},baseTime,false);
[D.brakeForce,~] = channel(channels,{'Brake Pos','Brake Position','Brake Pressure','Brake','Brake Pedal'},baseTime,false);
[D.ax,axUnit] = channel(channels,{'Longitudinal G','Longitudinal Acceleration','Accel X','G Force Longitudinal'},baseTime,false);
[D.ay,ayUnit] = channel(channels,{'Lateral G','Lateral Acceleration','Accel Y','G Force Lateral'},baseTime,false);
[D.yaw,yawUnit] = channel(channels,{'Yaw','Heading','GPS Heading','Vehicle Heading'},baseTime,false);
[D.yawRate,yawRateUnit] = channel(channels,{'Yaw Rate','Yaw Rate (deg/s)','Yaw Velocity'},baseTime,false);
[D.x,xUnit] = channel(channels,{'GPS X','Position X','World X','GPS Position X','Vehicle Position X'},baseTime,false);
[D.y,yUnit] = channel(channels,{'GPS Y','Position Y','World Y','GPS Position Y','Vehicle Position Y'},baseTime,false);
[longitude,longitudeUnit] = channel(channels,{'GPS Longitude','Longitude','GPS Long'},baseTime,false); %#ok<ASGLU>
[latitude,latitudeUnit] = channel(channels,{'GPS Latitude','Latitude','GPS Lat'},baseTime,false); %#ok<ASGLU>

if all(isfinite(longitude)) && all(isfinite(latitude)) && (~all(isfinite(D.x)) || ~all(isfinite(D.y)))
    lon0 = longitude(1); lat0 = latitude(1);
    D.x = (longitude-lon0)*111320*cosd(lat0);
    D.y = (latitude-lat0)*110540;
    xUnit = 'm'; yUnit = 'm';
end
D.x = distanceToMetres(D.x,xUnit);
D.y = distanceToMetres(D.y,yUnit);
D.s = distanceToMetres(D.s,sUnit);
D.speed = speedToMetresPerSecond(D.speed,speedUnit);
D.steerAngle = angleToRadians(D.steerAngle,steerUnit);
D.yaw = angleToRadians(D.yaw,yawUnit);
D.yawRate = rateToRadiansPerSecond(D.yawRate,yawRateUnit);
D.ax = accelerationToMetresPerSecondSquared(D.ax,axUnit);
D.ay = accelerationToMetresPerSecondSquared(D.ay,ayUnit);

if ~all(isfinite(D.yawRate)) && all(isfinite(D.speed)) && all(isfinite(D.steerAngle))
    D.yawRate = D.speed.*tan(D.steerAngle)/2.7;
end
if ~all(isfinite(D.yaw)) && all(isfinite(D.yawRate))
    D.yaw = cumtrapz(baseTime,D.yawRate);
end
if ~all(isfinite(D.x)) || ~all(isfinite(D.y))
    if all(isfinite(D.yaw)) && all(isfinite(D.speed))
        D.x = cumtrapz(baseTime,D.speed.*cos(D.yaw));
        D.y = cumtrapz(baseTime,D.speed.*sin(D.yaw));
    end
end
if ~all(isfinite(D.yaw)) && all(isfinite(D.x)) && all(isfinite(D.y))
    D.yaw = unwrap(atan2(gradient(D.y,baseTime),gradient(D.x,baseTime)));
end
if ~all(isfinite(D.yawRate)) && all(isfinite(D.yaw))
    D.yawRate = gradient(D.yaw,baseTime);
end
if ~all(isfinite(D.s)) && all(isfinite(D.x)) && all(isfinite(D.y))
    D.s = [0; cumsum(hypot(diff(D.x),diff(D.y)))];
end
if ~all(isfinite(D.speed)) && all(isfinite(D.s))
    D.speed = abs(gradient(D.s,baseTime));
end
if ~all(isfinite(D.ax)) && all(isfinite(D.speed))
    D.ax = gradient(D.speed,baseTime);
end
if ~all(isfinite(D.ay) ) && all(isfinite(D.speed)) && all(isfinite(D.yawRate))
    D.ay = D.speed.*D.yawRate;
end

if all(isfinite(D.lapTime)) && any(diff(D.lapTime) < -0.5) && ~all(isfinite(D.lap))
    D.lap = 1 + cumsum([false; diff(D.lapTime) < -0.5]);
elseif all(isfinite(D.s)) && any(diff(D.s) < -5) && ~all(isfinite(D.lap))
    D.lap = 1 + cumsum([false; diff(D.s) < -5]);
end
if ~all(isfinite(D.lap))
    error('Missing MoTeC channel: Lap/Lap Number (or a lap-time/distance reset to detect laps).');
end
D.lap = round(D.lap);
if ~all(isfinite(D.lapTime))
    ids = unique(D.lap);
    D.lapTime = zeros(size(D.lap));
    for k = 1:numel(ids)
        idx = D.lap == ids(k);
        D.lapTime(idx) = baseTime(idx) - baseTime(find(idx,1));
    end
elseif contains(lower(lapTimeUnit),'ms')
    D.lapTime = D.lapTime/1000;
end

missing = {};
for name = {'s','x','y','yaw','yawRate','speed','ax','ay','steerAngle','driveForceCmd','brakeForce'}
    if ~isfield(D,name{1}) || numel(D.(name{1})) ~= numel(baseTime) || any(~isfinite(D.(name{1})))
        missing{end+1} = name{1}; %#ok<AGROW>
    end
end
if ~isempty(missing)
    channelNames = strjoin({channels.Name},', ');
    error('Could not build dashboard data from this MoTeC log. Missing/unusable fields: %s. Available channels: %s', ...
        strjoin(missing,', '),channelNames);
end
end

function channels = collectChannels(S)
channels = struct('Name',{},'Time',{},'Value',{},'Unit',{});
names = fieldnames(S);
for k = 1:numel(names)
    value = S.(names{k});
    if isstruct(value) && isscalar(value) && all(isfield(value,{'Time','Value'}))
        unit = '';
        if isfield(value,'Units'), unit = char(value.Units); end
        channels(end+1) = struct('Name',names{k},'Time',double(value.Time(:)), ...
            'Value',double(value.Value(:)),'Unit',unit); %#ok<AGROW>
    elseif isstruct(value) && isscalar(value)
        nested = collectChannels(value);
        channels = [channels nested]; %#ok<AGROW>
    end
end
end

function [value,unit] = channel(channels,aliases,t,required)
names = cellfun(@normalise,{channels.Name},'UniformOutput',false);
aliasNames = cellfun(@normalise,aliases,'UniformOutput',false);
index = find(ismember(names,aliasNames),1);
value = nan(size(t)); unit = '';
if isempty(index)
    if required, error('Missing MoTeC channel: %s',strjoin(aliases,', ')); end
    return
end
c = channels(index); unit = c.Unit;
valid = isfinite(c.Time) & isfinite(c.Value);
ct = c.Time(valid); cv = c.Value(valid);
[ct,order] = sort(ct); cv = cv(order);
[ct,uniqueIndex] = unique(ct,'stable'); cv = cv(uniqueIndex);
if numel(ct) >= 2
    value = interp1(ct,cv,t,'linear','extrap');
elseif numel(ct) == 1
    value(:) = cv;
end
end

function value = normalise(text)
value = lower(regexprep(char(text),'[^a-zA-Z0-9]',''));
end

function value = distanceToMetres(value,unit)
u = lower(unit);
if contains(u,'ft'), value = value*0.3048;
elseif contains(u,'km') && ~contains(u,'/'), value = value*1000;
end
end

function value = speedToMetresPerSecond(value,unit)
u = lower(unit);
if contains(u,'km/h') || contains(u,'kmh'), value = value/3.6;
elseif contains(u,'mph'), value = value*0.44704;
end
end

function value = angleToRadians(value,unit)
if contains(lower(unit),'deg') || max(abs(value),[],'omitnan') > 2*pi
    value = value*pi/180;
end
end

function value = rateToRadiansPerSecond(value,unit)
if contains(lower(unit),'deg') || max(abs(value),[],'omitnan') > 2*pi
    value = value*pi/180;
end
end

function value = accelerationToMetresPerSecondSquared(value,unit)
u = lower(unit);
if contains(u,'g') && ~contains(u,'m/s'), value = value*9.81; end
end

function cleanupExport(i2,matFile)
try, i2.DataSources.CloseAll; catch, end
try, i2.Quit; catch, end
try, delete(i2); catch, end
if exist(matFile,'file'), delete(matFile); end
end