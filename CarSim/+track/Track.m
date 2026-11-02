classdef Track < handle
    %TRACK Closed 2D circuit described by a centre line.
    %   The centre line is resampled at a regular step so that arc length s,
    %   heading and curvature can be looked up by direct indexing (constant cost).
    %
    %   Conventions: s in [0, totalLength), s = 0 is the start/finish line (first point).
    %   lateralOffset > 0 means the car is on the LEFT of the centre line.

    properties (SetAccess = private)
        name (1,1) string = ""
        width (1,1) double = 10          % m (constant)
        points (:,2) double              % n x 2, regularly spaced
        s (:,1) double                   % arc length of each point [m]
        heading (:,1) double             % tangent angle [rad]
        curvature (:,1) double           % signed curvature [1/m], > 0 = left turn
        totalLength (1,1) double = 0     % m
        ds (1,1) double = 1              % m, spacing between points
        n (1,1) double = 0               % number of points
    end

    properties (Constant)
        resampleStep = 1                 % m
        curvatureSmoothing = 7           % points (moving average, circular)
        searchRadius = 30                % m, local window used by project()
    end

    properties (Access = private)
        searchOffsets (1,:) double = 0
    end

    methods (Static)
        function obj = fromFile(path)
            %FROMFILE Load a .mat file with centerline (Nx2), width (scalar) and name.
            S = load(path);
            for field = ["centerline", "width"]
                assert(isfield(S, field), 'Track:badFile', ...
                    'Track file "%s" must contain "%s".', path, field);
            end
            if isfield(S, 'name')
                nm = string(S.name);
            else
                [~, nm] = fileparts(path);
                nm = string(nm);
            end
            obj = track.Track();
            obj.init(S.centerline, S.width, nm);
        end
    end

    methods
        function [sOut, lat, idx] = project(obj, xy, hintIdx)
            %PROJECT Closest point of the centre line.
            %   [s, lateralOffset, idx] = project(xy, hintIdx)
            %   Searches only around hintIdx (constant cost). Use [] for a full search.
            N = obj.n;
            P = obj.points;
            if nargin < 3 || isempty(hintIdx)
                cand = 1:N;
            else
                cand = mod(hintIdx - 1 + obj.searchOffsets, N) + 1;
            end
            dx = P(cand, 1) - xy(1);
            dy = P(cand, 2) - xy(2);
            [~, k] = min(dx .* dx + dy .* dy);
            j = cand(k);
            jm = j - 1; if jm < 1, jm = N; end
            jp = j + 1; if jp > N, jp = 1; end
            [t1, d1, l1] = segProject(P(jm, :), P(j, :), xy);
            [t2, d2, l2] = segProject(P(j, :), P(jp, :), xy);
            if d1 < d2
                sOut = (jm - 1) * obj.ds + t1 * obj.ds;
                lat = l1;
            else
                sOut = (j - 1) * obj.ds + t2 * obj.ds;
                lat = l2;
            end
            sOut = mod(sOut, obj.totalLength);
            idx = j;
        end

        function p = pointAt(obj, sq)
            [i0, i1, t] = obj.locate(sq);
            p = (1 - t) * obj.points(i0, :) + t * obj.points(i1, :);
        end

        function psi = headingAt(obj, sq)
            [i0, i1, t] = obj.locate(sq);
            h0 = obj.heading(i0);
            psi = h0 + t * wrapAngle(obj.heading(i1) - h0);
        end

        function k = curvatureAt(obj, sq)
            [i0, i1, t] = obj.locate(sq);
            k = (1 - t) * obj.curvature(i0) + t * obj.curvature(i1);
        end

        function p = lookaheadPoint(obj, sq, dist)
            p = obj.pointAt(sq + dist);
        end

        function assertDrivable(obj, minTurnRadius)
            %ASSERTDRIVABLE Error if the tightest bend is below the car's turning radius.
            kMax = max(abs(obj.curvature));
            if kMax > 1 / minTurnRadius
                error('Track:notDrivable', ...
                    ['Track "%s" has a bend of radius %.1f m but the car cannot ' ...
                     'turn tighter than %.1f m.'], obj.name, 1 / kMax, minTurnRadius);
            end
        end
    end

    methods (Access = private)
        function init(obj, P, width, nm)
            validateattributes(P, {'numeric'}, {'2d', 'ncols', 2, 'finite'});
            validateattributes(width, {'numeric'}, {'scalar', 'positive', 'finite'});
            P = double(P);
            if norm(P(1, :) - P(end, :)) < 1e-9
                P(end, :) = [];                          % drop the duplicated closing point
            end
            assert(size(P, 1) >= 8, 'Track:tooShort', 'Centre line needs at least 8 points.');

            % two passes -> spacing is regular to ~1e-4
            P = resampleClosed(P, obj.resampleStep);
            P = resampleClosed(P, obj.resampleStep);

            N = size(P, 1);
            seg = diff([P; P(1, :)]);
            L = sum(hypot(seg(:, 1), seg(:, 2)));
            dsLocal = L / N;

            d = P([2:N, 1], :) - P([N, 1:N-1], :);       % central differences
            psi = atan2(d(:, 2), d(:, 1));
            dpsi = wrapAngle(psi([2:N, 1]) - psi([N, 1:N-1]));
            kappa = circMovMean(dpsi / (2 * dsLocal), obj.curvatureSmoothing);

            obj.name = nm;
            obj.width = width;
            obj.points = P;
            obj.n = N;
            obj.ds = dsLocal;
            obj.totalLength = N * dsLocal;
            obj.s = (0:N-1)' * dsLocal;
            obj.heading = psi;
            obj.curvature = kappa;
            W = ceil(obj.searchRadius / dsLocal);
            obj.searchOffsets = -W:W;
        end

        function [i0, i1, t] = locate(obj, sq)
            f = mod(sq, obj.totalLength) / obj.ds;
            k = floor(f);
            t = f - k;
            i0 = k + 1;
            if i0 > obj.n
                i0 = obj.n;
                t = 1;
            end
            i1 = i0 + 1;
            if i1 > obj.n, i1 = 1; end
        end
    end
end

% ---------------------------------------------------------------- local helpers
function Q = resampleClosed(P, step)
    n = size(P, 1);
    Pc = [P; P(1, :)];
    u = [0; cumsum(hypot(diff(Pc(:, 1)), diff(Pc(:, 2))))];
    L = u(end);
    m = max(round(L / step), 16);
    uq = (0:m-1)' * (L / m);
    k = min(10, n - 1);                                  % periodic extension on both sides
    ue = [u(n-k+1:n) - L; u(1:n); u(1:k) + L];
    Pe = [P(n-k+1:n, :); P; P(1:k, :)];
    Q = interp1(ue, Pe, uq, 'spline');
end

function a = wrapAngle(a)
    a = mod(a + pi, 2 * pi) - pi;
end

function y = circMovMean(x, w)
    if w <= 1, y = x; return; end
    h = floor(w / 2);
    xe = [x(end-h+1:end); x; x(1:h)];
    ye = movmean(xe, 2 * h + 1, 'Endpoints', 'shrink');
    y = ye(h+1:end-h);
end

function [t, d2, lat] = segProject(pa, pb, xy)
    ex = pb(1) - pa(1);  ey = pb(2) - pa(2);
    rx = xy(1) - pa(1);  ry = xy(2) - pa(2);
    e2 = ex * ex + ey * ey;
    t = min(max((rx * ex + ry * ey) / e2, 0), 1);
    qx = xy(1) - (pa(1) + t * ex);
    qy = xy(2) - (pa(2) + t * ey);
    d2 = qx * qx + qy * qy;
    lat = (ex * qy - ey * qx) / sqrt(e2);                % > 0 : left of travel direction
end
