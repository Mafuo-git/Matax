function str = fmtTime(t)
% FMTTIME  Seconds -> 'm:ss.mmm'
if isnan(t), str = '-'; return; end
m = floor(t/60);
str = sprintf('%d:%06.3f', m, t - 60*m);
end
