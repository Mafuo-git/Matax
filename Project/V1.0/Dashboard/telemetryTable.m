function [rows, cols, dv] = telemetryTable(mode, Sess, selIdx, refIdx, numSectors)
% TELEMETRYTABLE  Build the corner or sector analysis table (cell array of text).
%   [rows, cols, dv] = telemetryTable('Corners', Sess, selIdx, refIdx)
%   [rows, cols, dv] = telemetryTable('Sectors', Sess, selIdx, refIdx, 3)
% dv = numeric delta vs reference per row (used to colour the Delta column).

if nargin < 5, numSectors = 3; end
Lp = Sess.Laps{selIdx}; Rp = [];
if refIdx > 0, Rp = Sess.Laps{refIdx}; end
if strcmpi(mode,'Corners')
    [rows,cols,dv] = cornerRows(Sess,Lp,Rp);
else
    [rows,cols,dv] = sectorRows(Sess,Lp,Rp,numSectors);
end
end

function t = segTime(L, s0, s1)
% time spent between distances s0 and s1 on lap L (NaN if the lap does not cover it)
if L.s(end) < s1 && ~L.complete, t = NaN; return; end
s1 = min(s1,L.s(end));
t = interp1(L.s,L.tl,s1,'linear','extrap') - interp1(L.s,L.tl,s0,'linear','extrap');
end

function [rows, cols, dv] = cornerRows(Sess, Lp, Rp)
cols = {'Corner','Start (m)','Entry (km/h)','Apex (km/h)','Exit (km/h)', ...
        'Brake pt (m)','Max lat G','Time (s)','Delta (s)'};
C = Sess.CornerS; n = size(C,1);
rows = cell(n,9); dv = nan(n,1);
for c = 1:n
    s0 = C(c,1); s1 = C(c,2); s1e = min(s1,Lp.s(end));
    mk = Lp.s >= s0 & Lp.s <= s1e;
    tc = segTime(Lp,s0,s1);
    if isnan(tc) || nnz(mk) < 2
        rows(c,:) = [{sprintf('T%d',c),sprintf('%.0f',s0)}, repmat({'-'},1,7)];
        continue
    end
    sk = Lp.s(mk); vk = Lp.vk(mk);
    [vmin,im] = min(vk); sApex = sk(im);
    entry = interp1(Lp.s,Lp.vk,s0,'linear','extrap');
    exitv = interp1(Lp.s,Lp.vk,s1e,'linear','extrap');
    sPrev = -inf; if c > 1, sPrev = C(c-1,2); end
    w = find(Lp.s >= max(s0-250,sPrev) & Lp.s <= sApex & Lp.brk > 5,1);
    if isempty(w), bp = '-'; else, bp = sprintf('%.0f',s0 - Lp.s(w)); end
    if isempty(Rp)
        dstr = '-';
    else
        d = tc - segTime(Rp,s0,s1); dv(c) = d;
        if isnan(d), dstr = '-'; else, dstr = sprintf('%+.3f',d); end
    end
    rows(c,:) = {sprintf('T%d',c),sprintf('%.0f',s0),sprintf('%.0f',entry), ...
        sprintf('%.0f',vmin),sprintf('%.0f',exitv),bp, ...
        sprintf('%.2f',max(abs(Lp.glat(mk)))),sprintf('%.3f',tc),dstr};
end
end

function [rows, cols, dv] = sectorRows(Sess, Lp, Rp, n)
cols = {'Sector','Start (m)','End (m)','Time (s)','Ref (s)','Delta (s)', ...
        'Avg (km/h)','Max (km/h)','Min (km/h)'};
ed = linspace(0,Sess.TrackLen,n+1);
rows = cell(n,9); dv = nan(n,1);
for k = 1:n
    s0 = ed(k); s1 = ed(k+1);
    head = {sprintf('S%d',k),sprintf('%.0f',s0),sprintf('%.0f',s1)};
    tc = segTime(Lp,s0,s1);
    mk = Lp.s >= s0 & Lp.s <= min(s1,Lp.s(end));
    if isnan(tc) || nnz(mk) < 2
        rows(k,:) = [head, repmat({'-'},1,6)]; continue
    end
    if isempty(Rp), tr = NaN; else, tr = segTime(Rp,s0,s1); end
    if isnan(tr), rs = '-'; ds = '-';
    else, rs = sprintf('%.3f',tr); dv(k) = tc - tr; ds = sprintf('%+.3f',tc-tr); end
    rows(k,:) = [head,{sprintf('%.3f',tc),rs,ds,sprintf('%.0f',mean(Lp.vk(mk))), ...
        sprintf('%.0f',max(Lp.vk(mk))),sprintf('%.0f',min(Lp.vk(mk)))}];
end
end
