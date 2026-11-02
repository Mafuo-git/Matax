function makeTestTracks()
%MAKETESTTRACKS Generate data/tracks/stadium.mat and data/tracks/complex.mat.
%   A track file holds: centerline (Nx2, metres, counter-clockwise), width (m), name.
%   The first point of centerline is the start/finish line.
    root = fileparts(fileparts(mfilename('fullpath')));
    folder = fullfile(root, 'data', 'tracks');
    if ~isfolder(folder), mkdir(folder); end

    % ---------------- stadium: two 300 m straights + two 70 m radius bends
    Ls = 300;  R = 70;  step = 0.5;
    bottom = [linspace(0, Ls, round(Ls/step) + 1)', -R * ones(round(Ls/step) + 1, 1)];
    th = linspace(-pi/2, pi/2, round(pi*R/step) + 1)';
    right = [Ls + R*cos(th), R*sin(th)];
    top = [linspace(Ls, 0, round(Ls/step) + 1)', R * ones(round(Ls/step) + 1, 1)];
    th = linspace(pi/2, 3*pi/2, round(pi*R/step) + 1)';
    left = [R*cos(th), R*sin(th)];
    P = [bottom(1:end-1, :); right(1:end-1, :); top(1:end-1, :); left(1:end-1, :)];
    [~, k] = min(hypot(P(:,1) - Ls/2, P(:,2) + R));       % start line in the middle of the bottom straight
    centerline = circshift(P, -(k - 1), 1);                %#ok<NASGU>
    width = 12;                                            %#ok<NASGU>
    name = "stadium";                                      %#ok<NASGU>
    save(fullfile(folder, 'stadium.mat'), 'centerline', 'width', 'name');

    % ---------------- complex: smooth closed curve with a hairpin (R ~ 25 m)
    theta = linspace(-pi/2, 3*pi/2, 6001)';
    theta(end) = [];
    centerline = [250*cos(theta) + 25*cos(3*theta), 150*sin(theta) + 20*sin(2*theta)];
    width = 12;
    name = "complex";
    save(fullfile(folder, 'complex.mat'), 'centerline', 'width', 'name');

    names = track.TrackLibrary.list();
    for n = names
        trk = track.TrackLibrary.load(n);
        fprintf('%-10s length %7.1f m, tightest bend radius %5.1f m\n', ...
            n, trk.totalLength, 1 / max(abs(trk.curvature)));
    end
end
