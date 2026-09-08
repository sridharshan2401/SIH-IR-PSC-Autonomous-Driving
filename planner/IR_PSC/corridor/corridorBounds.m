function [dMin, dMax] = corridorBounds(corridor, egoHalfWidth, margin)
%CORRIDORBOUNDS Allowed lateral-offset band at each corridor station.
%
%   COMPONENT STATUS: REAL
%
%   [dMin, dMax] = CORRIDORBOUNDS(corridor, egoHalfWidth, margin) returns,
%   for every station, how far the trajectory may move left (dMax, positive)
%   and right (dMin, negative) from the corridor centreline while keeping
%   the whole vehicle body inside the drivable corridor.
%
%   IR-PSC step 8: preserve road-boundary safety margins. This is the
%   constraint that the deformation stage is not permitted to violate --
%   dodging an obstacle must never push the vehicle off the road.
%
%   Inputs:
%       corridor     - struct from extractCorridor()
%       egoHalfWidth - m, half the ego vehicle width
%       margin       - m, extra clearance to keep from each boundary
%
%   Outputs:
%       dMin - Nx1, most negative (rightward) offset allowed, <= 0
%       dMax - Nx1, most positive (leftward) offset allowed, >= 0
%
%   Where the corridor is too narrow for the vehicle plus margins, both
%   bounds collapse to 0. That is reported honestly rather than being
%   widened to something drivable: a corridor narrower than the car is a
%   fact the decision logic needs to see so it can slow down or stop.
%
%   Example:
%       [dMin, dMax] = corridorBounds(corr, 0.84, 0.35);
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, DEFORMTRAJECTORY, CHECKCLEARANCE.

validateattributes(egoHalfWidth, {'numeric'}, {'scalar','positive','finite'}, ...
                   mfilename, 'egoHalfWidth');
validateattributes(margin, {'numeric'}, {'scalar','nonnegative','finite'}, ...
                   mfilename, 'margin');

need = egoHalfWidth + margin;

dMax = corridor.leftDist  - need;    % room to move left
dMin = -(corridor.rightDist - need); % room to move right (negative)

% Never allow an inverted band.
dMax = max(dMax, 0);
dMin = min(dMin, 0);

tooNarrow = (corridor.width < 2 * need);
dMax(tooNarrow) = 0;
dMin(tooNarrow) = 0;
end
