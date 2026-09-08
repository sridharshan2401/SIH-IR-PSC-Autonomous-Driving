function [s, d, idx, foot] = projectPointOnPath(P, pt)
%PROJECTPOINTONPATH Project a point onto a polyline (Frenet coordinates).
%
%   COMPONENT STATUS: REAL
%
%   [s, d, idx, foot] = PROJECTPOINTONPATH(P, pt) finds the closest point on
%   polyline P to the query point pt, and returns its position in path
%   coordinates.
%
%   This is the bridge between world coordinates and the road-aligned frame
%   the IR-PSC planner works in. The lateral coordinate d is what lets the
%   planner say "this obstacle is 1.2 m to the left of my corridor centre"
%   without ever needing a lane marking.
%
%   Inputs:
%       P  - Nx2 polyline, N >= 2
%       pt - 1x2 query point [x y]
%
%   Outputs:
%       s    - arc length along P of the closest point (metres from P(1,:))
%       d    - signed lateral offset in metres. Positive = LEFT of the path
%              direction of travel, negative = RIGHT.
%       idx  - index of the segment start containing the projection
%       foot - 1x2 coordinates of the closest point itself
%
%   The search is exhaustive over segments. For the path lengths used here
%   (tens of points) that is both fast enough and immune to the local-minimum
%   traps a gradient search would hit on a curved corridor.
%
%   Example:
%       [s, d] = projectPointOnPath([0 0; 10 0], [4 2]);   % s = 4, d = 2
%
%   Requires: base MATLAB only.
%
%   See also PATHARCLENGTH, FRENETTOCARTESIAN.

validateattributes(P, {'numeric'}, {'2d','ncols',2,'finite','real'}, mfilename, 'P');
validateattributes(pt, {'numeric'}, {'vector','numel',2,'finite','real'}, mfilename, 'pt');
pt = pt(:).';

N = size(P,1);
if N < 2
    error('projectPointOnPath:tooShort', 'Need at least 2 path points.');
end

sCum  = pathArcLength(P);
bestD2   = Inf;
idx      = 1;
foot     = P(1,:);
s        = 0;
bestSide = 0;

for i = 1:N-1
    a   = P(i,:);
    b   = P(i+1,:);
    ab  = b - a;
    L2  = ab(1)^2 + ab(2)^2;
    if L2 < 1e-12
        t = 0;
    else
        t = ((pt - a) * ab.') / L2;
        t = min(max(t, 0), 1);          % clamp to the segment
    end
    f  = a + t * ab;
    d2 = (pt(1)-f(1))^2 + (pt(2)-f(2))^2;

    if d2 < bestD2
        bestD2 = d2;
        idx    = i;
        foot   = f;
        s      = sCum(i) + t * sqrt(L2);
        % Sign convention: z-component of cross(tangent, a->pt).
        % Positive means pt lies to the LEFT of the direction of travel.
        bestSide = ab(1)*(pt(2)-a(2)) - ab(2)*(pt(1)-a(1));
    end
end

d = sqrt(bestD2) * sign(bestSide);
if bestSide == 0
    d = 0;
end
end
