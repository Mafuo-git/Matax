function Sess = loadTelemetrySession(file, opts)
% LOADTELEMETRYSESSION  Read a simulator .mat or MoTeC .ld file for the dashboard.
%
%   Sess = loadTelemetrySession('session.mat')
%   Sess = loadTelemetrySession('session.ld')
%   Sess = loadTelemetrySession('session.mat', struct('Decim',5))   % override a setting
%
% Sess.Laps{k} holds one decimated lap (columns, all same length):
%   id, tl (lap time, s), s (distance, m), x, y, yaw, vk (km/h), thr, brk (%),
%   steer (deg), glon, glat (g, glat < 0 = left), rA (yaw rate, deg/s), rpm,
%   rE (yaw rate expected from steering, deg/s), e (balance error, deg/s,
%   <0 understeer, >0 oversteer), bvalid, complete

P = struct('Decim',10,'WheelBase',NaN,'UndersteerGradient',NaN,'SteeringRatio',1, ...
           'WheelRadius',NaN,'GearRatio',NaN,'CornerCurvature',1/250, ...
           'CornerMinLength',20,'CornerMergeGap',40);
if nargin > 1 && isstruct(opts)
    fn = fieldnames(opts);
    for i = 1:numel(fn), P.(fn{i}) = opts.(fn{i}); end
end

D = readMotecData(file);
v = D.speed;

% ---- sign conventions, relative to yaw rate (yaw rate > 0 = left turn) ----
s1 = sign(sum(D.ay .* v .* D.yawRate));         Q.signAy    = s1 + (s1 == 0);
s2 = sign(sum(D.steerAngle .* v .* D.yawRate)); Q.signSteer = s2 + (s2 == 0);
Q.ratio = P.SteeringRatio;

% ---- pedal normalisation ----
N.thrMax = max(max(D.driveForceCmd), eps);
N.brkMax = max(max(abs(D.brakeForce)), eps);

% ---- motor RPM (derived from torques unless given) ----
R = P.WheelRadius;
if isnan(R) && isfield(D,'wheelTorque') && isfield(D,'driveForce')
    m = abs(D.driveForce) > 100;
    if any(m), R = median(abs(D.wheelTorque(m) ./ D.driveForce(m))); end
end
if isnan(R) || R < 0.15 || R > 0.7, R = 0.33; end
G = P.GearRatio;
if isnan(G) && isfield(D,'motorTorque') && isfield(D,'wheelTorque')
    m = D.motorTorque > 1 & D.wheelTorque > 1;
    if any(m), G = median(D.wheelTorque(m) ./ D.motorTorque(m)); end
end
if ~isnan(G) && G <= 0, G = NaN; end
N.R = R; N.G = G;

% ---- neutral steering response: yaw rate = v*delta / (L + K*v^2) ----
d = Q.signSteer * D.steerAngle / Q.ratio;
y = v .* d ./ D.yawRate;
m = abs(D.yawRate) > 0.02 & abs(d) > 0.003 & v > 5 & abs(D.ay) < 0.3*max(abs(D.ay));
m = m & y > 0.5 & y < 10;
Lw = P.WheelBase; K = P.UndersteerGradient;
if isnan(Lw) && isnan(K) && nnz(m) > 50
    p = [ones(nnz(m),1) v(m).^2] \ y(m);
    if p(1) > 1.5 && p(1) < 4.5, Lw = p(1); K = p(2); end
end
if isnan(Lw), Lw = 2.7; end
if isnan(K),  K  = 0;   end
Q.Lw = Lw; Q.K = K;

% ---- split into laps (decimated) ----
laps = {};
ids = unique(D.lap(~isnan(D.lap)));
for i = 1:numel(ids)
    idx = find(D.lap == ids(i));
    if numel(idx) < 20, continue; end
    keep = unique([1:P.Decim:numel(idx), numel(idx)]);
    laps{end+1} = buildLap(D, idx(keep), N, Q); %#ok<AGROW>
end
if isempty(laps), error('No usable lap found in the file.'); end

lens = cellfun(@(l) l.s(end), laps);
for k = 1:numel(laps), laps{k}.complete = lens(k) >= 0.97*max(lens); end
times = cellfun(@(l) l.tl(end), laps);
times(~cellfun(@(l) l.complete, laps)) = inf;
[~,best] = min(times);
sel = find((1:numel(laps)) ~= best, 1);
if isempty(sel), sel = best; end

% ---- corners (detected once, on the best lap) ----
Bp = laps{best};
C = detectCorners(Bp, P);
cxy = zeros(size(C,1),2);
for c = 1:size(C,1)
    idx = find(Bp.s >= C(c,1) & Bp.s <= C(c,2));
    [~,j] = min(Bp.vk(idx)); k = idx(j);
    cxy(c,:) = [Bp.x(k) Bp.y(k)];
end

% ---- session-wide limits ----
vmax   = max(cellfun(@(l) max(l.vk), laps));
smax   = max(cellfun(@(l) max(abs(l.steer)), laps));
rmax   = max(cellfun(@(l) max(abs([l.rA; l.rE])), laps));
gmax   = max(cellfun(@(l) max(hypot(l.glat,l.glon)), laps));
rpmmax = max(cellfun(@(l) max(l.rpm), laps));
X = cell2mat(cellfun(@(l) [min(l.x) max(l.x) min(l.y) max(l.y)], laps(:), 'UniformOutput', false));
pad = 0.05*max(max(X(:,2))-min(X(:,1)), max(X(:,4))-min(X(:,3)));

Lim.vtop     = max(50, ceil(vmax/50)*50);
if isnan(rpmmax), Lim.rtop = 1000; else, Lim.rtop = max(1000, ceil(rpmmax/1000)*1000); end
Lim.steerMax = max(1, ceil(smax*1.1));
Lim.rateMax  = max(5, ceil(rmax*1.1));
Lim.gm       = max(1, ceil(gmax*2)/2);
Lim.x        = [min(X(:,1)) max(X(:,2))] + pad*[-1 1];
Lim.y        = [min(X(:,3)) max(X(:,4))] + pad*[-1 1];
Lim.headLen  = 0.6*pad;

ae = sort(abs(Bp.e(Bp.bvalid)));
if isempty(ae), balScale = 10; else, balScale = max(5, ae(max(1,round(0.98*numel(ae))))); end

Sess.File     = file;
Sess.Laps     = laps;
Sess.BestIdx  = best;
Sess.SelIdx   = sel;
Sess.RefIdx   = best;
Sess.TrackLen = Bp.s(end);
Sess.CornerS  = C;
Sess.CornerXY = cxy;
Sess.BalScale = balScale;
Sess.Lim      = Lim;
Sess.Params   = P;
Sess.Calib    = struct('WheelBase',Lw,'UndersteerGradient',K,'WheelRadius',R,'GearRatio',G, ...
                       'SignAy',Q.signAy,'SignSteer',Q.signSteer);
end

%% ------------------------------------------------------------------------
function Lp = buildLap(D, idx, N, Q)
n = numel(idx); r = (1:n)'*1e-9;
Lp.id = D.lap(idx(1));
tl = D.lapTime(idx) - D.lapTime(idx(1));
if any(diff(tl) < 0), tl = D.t(idx) - D.t(idx(1)); end
Lp.tl = tl + r;
Lp.s  = cummax(D.s(idx) - D.s(idx(1))) + r;
Lp.x = D.x(idx); Lp.y = D.y(idx); Lp.yaw = D.yaw(idx);
v = D.speed(idx);
Lp.vk = v*3.6;
Lp.thr = 100*max(D.driveForceCmd(idx),0)/N.thrMax;
Lp.brk = 100*abs(D.brakeForce(idx))/N.brkMax;
Lp.steer = D.steerAngle(idx)*180/pi;
Lp.glon = D.ax(idx)/9.81;
Lp.glat = -Q.signAy*D.ay(idx)/9.81;          % negative = left turn on the G circle
Lp.rA = D.yawRate(idx)*180/pi;
Lp.rpm = v/N.R*N.G*30/pi;

dEff = Q.signSteer*Lp.steer/Q.ratio;
rE = v.*dEff ./ max(Q.Lw + Q.K*v.^2, 0.5*Q.Lw);
rA = Lp.rA;
dirS = sign(rE);
opp = (rA.*rE < 0) & (abs(rA) > abs(rE));
swap = opp | abs(rE) < 1;
dirS(swap) = sign(rA(swap));
valid = abs(rE) > 1 | abs(rA) > 3;
e = zeros(n,1);
e(valid) = dirS(valid).*(rA(valid) - rE(valid));   % <0 understeer, >0 oversteer
Lp.rE = rE;
Lp.e = movmean(e,15);
Lp.bvalid = movmean(double(valid),15) > 0.5;
Lp.complete = true;
end

%% ------------------------------------------------------------------------
function C = detectCorners(Lp, P)
kap = movmean(abs(Lp.rA)*pi/180 ./ max(Lp.vk/3.6,1), 25);
m = kap > P.CornerCurvature;
d = diff([0; m(:); 0]);
i0 = find(d == 1); i1 = find(d == -1) - 1;
if isempty(i0), C = zeros(0,2); return; end
S = [Lp.s(i0), Lp.s(i1)];
out = S(1,:);
for k = 2:size(S,1)
    if S(k,1) - out(end,2) < P.CornerMergeGap
        out(end,2) = S(k,2);
    else
        out(end+1,:) = S(k,:); %#ok<AGROW>
    end
end
C = out((out(:,2)-out(:,1)) >= P.CornerMinLength, :);
end
