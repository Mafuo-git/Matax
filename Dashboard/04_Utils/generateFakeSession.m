function generateFakeSession(file, nLaps)
% generateFakeSession  Create a synthetic 1 kHz session with the same variables
% as the simulator, to test the dashboard before real data is available.
%
%   generateFakeSession                      -> fake_session.mat, 3 laps
%   generateFakeSession('test.mat', 5)
%
% Conventions: x/y/yaw in world frame, ax/ay in vehicle frame (ay > 0 = left turn),
% steerAngle > 0 = left, brakeForce >= 0.

if nargin < 1, file = 'fake_session.mat'; end
if nargin < 2, nLaps = 3; end

% ---------------- closed track (polar "bean") ----------------
th  = linspace(0,2*pi,3001)';
rad = 1.6*(220 + 70*cos(2*th) + 30*sin(3*th) + 18*cos(5*th));
xc = rad.*cos(th); yc = rad.*sin(th);
s0 = [0; cumsum(hypot(diff(xc),diff(yc)))];
L  = s0(end);
hd = unwrap(atan2(gradient(yc,s0), gradient(xc,s0)));
kap0 = movmean(gradient(hd,s0), 31);

ds = 1; n = floor(L/ds);
se = (0:n)'*ds;                              % distance grid (n+1 points)
kg  = interp1(s0,kap0,se);
hdg = interp1(s0,hd,se);
xg  = interp1(s0,xc,se);
yg  = interp1(s0,yc,se);

% ---------------- vehicle parameters ----------------
m = 800; W = m*9.81; h = 0.30; Tw = 1.6; Lwb = 2.7; Kus = 0.0003;
aLat = 20; vmax = 85; aAcc = 8; aBrk = 20;
Rw = 0.33; Gr = 6; eff = 0.95;
dt = 0.001;

laps = cell(1,nLaps); tOffset = 0;
for i = 1:nLaps
    f = min(1, 0.93 + 0.035*(i-1));          % lap 1 slowest, then improving

    % --- speed profile (forward / backward / forward passes)
    vp = min(vmax, sqrt(f*aLat./max(abs(kg),1e-4)));
    for k = 2:n+1,   vp(k) = min(vp(k), sqrt(vp(k-1)^2 + 2*f*aAcc*ds)); end
    for k = n:-1:1,  vp(k) = min(vp(k), sqrt(vp(k+1)^2 + 2*aBrk*ds)); end
    for k = 2:n+1,   vp(k) = min(vp(k), sqrt(vp(k-1)^2 + 2*f*aAcc*ds)); end

    tt = [0; cumsum(ds ./ (0.5*(vp(1:end-1) + vp(2:end))))];
    T  = tt(end);
    tq = (0:dt:T)';
    sq = interp1(tt,se,tq);
    vq = interp1(se,vp,sq);
    kq = interp1(se,kg,sq);
    hq = interp1(se,hdg,sq);
    lo = 0.8*(i-1)*sin(2*pi*3*sq/L);         % small lateral offset between laps
    xq = interp1(se,xg,sq) - lo.*sin(hq);
    yq = interp1(se,yg,sq) + lo.*cos(hq);

    % --- dynamics
    axq = movmean(gradient(vq,dt),100);
    ayq = vq.^2 .* kq;
    rq  = vq .* kq;
    Fx  = m*axq + 0.55*vq.^2 + 120;
    cmd = max(Fx,0);
    brk = max(-Fx,0);
    drv = movmean(cmd,50);

    % --- steering: kinematic + understeer near the limit, a bit less on power-on exit
    amax  = f*aLat;
    sat   = min(abs(ayq)/amax,1.2);
    delta = (Lwb + Kus*vq.^2).*kq .* (1 + 0.35*sat.^4);
    os = (cmd > 0.5*max(cmd)) & (abs(ayq) > 0.6*amax);
    delta(os) = delta(os)*0.85;
    delta = movmean(delta + 0.0005*randn(size(delta)),50);

    % --- wheel loads and forces
    down = 0.9*vq.^2;
    Fz_FL = 0.45*W/2 + 0.4*down/2 - m*axq*h/Lwb/2 - 0.45*m*ayq*h/Tw;
    Fz_FR = 0.45*W/2 + 0.4*down/2 - m*axq*h/Lwb/2 + 0.45*m*ayq*h/Tw;
    Fz_RL = 0.55*W/2 + 0.6*down/2 + m*axq*h/Lwb/2 - 0.55*m*ayq*h/Tw;
    Fz_RR = 0.55*W/2 + 0.6*down/2 + m*axq*h/Lwb/2 + 0.55*m*ayq*h/Tw;
    Fz_FL = max(Fz_FL,50); Fz_FR = max(Fz_FR,50); Fz_RL = max(Fz_RL,50); Fz_RR = max(Fz_RR,50);
    Fyf = 0.45*m*ayq; Fyr = 0.55*m*ayq;

    Q = struct();
    Q.t = tq + tOffset;
    Q.lap = i*ones(size(tq));
    Q.lapTime = tq;
    Q.x = xq; Q.y = yq; Q.s = sq;
    Q.lateralOffset = lo;
    Q.yaw = hq; Q.yawRate = rq;
    Q.speed = vq; Q.ax = axq; Q.ay = ayq;
    Q.steerAngle = delta;
    Q.driveForceCmd = cmd; Q.driveForce = drv; Q.brakeForce = brk;
    Q.wheelTorque = drv*Rw;
    Q.motorTorque = Q.wheelTorque/(Gr*eff);
    Q.Fx_FL = -0.3*brk;            Q.Fx_FR = -0.3*brk;
    Q.Fx_RL = -0.2*brk + drv/2;    Q.Fx_RR = -0.2*brk + drv/2;
    Q.Fy_FL = Fyf.*Fz_FL./(Fz_FL+Fz_FR); Q.Fy_FR = Fyf.*Fz_FR./(Fz_FL+Fz_FR);
    Q.Fy_RL = Fyr.*Fz_RL./(Fz_RL+Fz_RR); Q.Fy_RR = Fyr.*Fz_RR./(Fz_RL+Fz_RR);
    Q.Fz_FL = Fz_FL; Q.Fz_FR = Fz_FR; Q.Fz_RL = Fz_RL; Q.Fz_RR = Fz_RR;
    laps{i} = Q;
    tOffset = Q.t(end) + dt;
    fprintf('Lap %d: %.3f s\n',i,T);
end

% ---------------- concatenate and save ----------------
fn = fieldnames(laps{1});
S = struct();
for k = 1:numel(fn)
    S.(fn{k}) = cell2mat(cellfun(@(c) c.(fn{k}), laps(:), 'UniformOutput', false));
end
save(file,'-struct','S');
fprintf('Saved %s (%d samples)\n',file,numel(S.t));
end
