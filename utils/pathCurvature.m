function k = pathCurvature(P)
%PATHCURVATURE Signed curvature at every point of a polyline.
%
%   COMPONENT STATUS: REAL
%
%   k = PATHCURVATURE(P) estimates the signed curvature (1/m) at each point
%   using the circumscribed-circle formula on consecutive point triples.
%   Positive curvature is a left turn, negative a right turn.
%
%   The triangle form is used rather than finite-difference derivatives
%   because it stays numerically stable on unevenly spaced points, which is
%   exactly what corridor extraction produces.
%
%       k = 2 * cross(v1, v2) / (|v1| * |v2| * |v2 - v1|)
%
%   Inputs:
%       P - Nx2 array of [x y] points, N >= 3
%
%   Outputs:
%       k - Nx1 vector of signed curvature in 1/m. Endpoints copy their
%           nearest interior neighbour. Degenerate (coincident) triples
%           yield 0 rather than Inf or NaN.
%
%   Example:
%       k = pathCurvature(circlePoints);   % ~ 1/R everywhere
%
%   Requires: base MATLAB only.
%
%   See also PATHHEADING, CHECKFEASIBILITY.

validateattributes(P, {'numeric'}, {'2d','ncols',2,'finite','real'}, mfilename, 'P');
N = size(P,1);
k = zeros(N,1);
if N < 3
    return;
end

for i = 2:N-1
    v1 = P(i,:)   - P(i-1,:);
    v2 = P(i+1,:) - P(i,:);
    n1 = hypot(v1(1), v1(2));
    n2 = hypot(v2(1), v2(2));
    v3 = P(i+1,:) - P(i-1,:);
    n3 = hypot(v3(1), v3(2));

    denom = n1 * n2 * n3;
    if denom < 1e-9
        k(i) = 0;              % coincident points: no meaningful curvature
        continue;
    end
    crossZ = v1(1)*v2(2) - v1(2)*v2(1);
    k(i)   = 2 * crossZ / denom;
end

k(1) = k(2);
k(N) = k(N-1);
end
