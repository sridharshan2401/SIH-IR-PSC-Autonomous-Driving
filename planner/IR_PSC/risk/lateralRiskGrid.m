function [R, offsets, stationTimes] = lateralRiskGrid(corridor, preds, ego, cfg, vp, dMin, dMax)
%LATERALRISKGRID Risk of each lateral offset at each corridor station.
%
%   COMPONENT STATUS: REAL
%
%   [R, offsets, stationTimes] = LATERALRISKGRID(corridor, preds, ego, cfg,
%   vp, dMin, dMax) builds the station-by-offset risk table that the
%   trajectory deformation stage searches.
%
%   This is the bridge between "where are the hazards predicted to be" and
%   "which way should the trajectory bend". Every cell R(i,j) answers a
%   single concrete question:
%
%       If the vehicle passes station i at lateral offset offsets(j),
%       at the time it is predicted to arrive there, how dangerous is that?
%
%   Time matters and is handled explicitly. Station i is reached at
%   approximately s(i)/v seconds, so a road user predicted to cross the road
%   in two seconds only makes the stations the ego reaches at around two
%   seconds dangerous. A purely spatial obstacle map cannot express that,
%   and would brake for a hazard that will have cleared the road long before
%   the vehicle arrives.
%
%   Inputs:
%       corridor - corridor struct from extractCorridor()
%       preds    - prediction struct array from predictObstacles()
%       ego      - ego state struct from makeEgoState()
%       cfg      - config struct from irpscConfig()
%       vp       - vehicle params struct from vehicleParams()
%       dMin     - Nx1 most negative allowed offset per station
%       dMax     - Nx1 most positive allowed offset per station
%
%   Outputs:
%       R            - NxK risk matrix in [0,1]. Cells outside the allowed
%                      lateral band are set to Inf so the search can never
%                      choose them.
%       offsets      - 1xK lateral offsets sampled (m, positive = left)
%       stationTimes - Nx1 predicted arrival time at each station (s)
%
%   Example:
%       [R, off] = lateralRiskGrid(corr, preds, ego, cfg, vp, dMin, dMax);
%
%   Requires: base MATLAB only.
%
%   See also DEFORMTRAJECTORY, PREDICTEDOCCUPANCYRISK, CORRIDORBOUNDS.

N = size(corridor.center, 1);

% --- Lateral offset samples ------------------------------------------
maxShift = cfg.deform.maxLateralShift;
step     = 0.25;                                   % m, offset resolution
offsets  = -maxShift:step:maxShift;
if isempty(offsets)
    offsets = 0;
end
K = numel(offsets);

% --- Arrival time at each station ------------------------------------
% Planning speed is the current speed, floored so a stopped vehicle still
% produces finite arrival times instead of Inf everywhere.
vPlan        = max(ego.speed, 1.0);
stationTimes = corridor.s(:) / vPlan;

tHorizon = cfg.prediction.horizon;
stationTimes = min(stationTimes, tHorizon);   % clamp, never extrapolate

% --- Evaluate ---------------------------------------------------------
R = zeros(N, K);

if isempty(preds)
    R = applyBounds(R, offsets, dMin, dMax);
    return;
end

for j = 1:K
    d = offsets(j);
    P = frenetToCartesian(corridor.center, corridor.s, d);

    % Vehicle heading at each station follows the corridor.
    th = corridor.heading(:);

    colRisk = zeros(N,1);
    for dIdx = 1:numel(vp.discOffsets)
        off = vp.discOffsets(dIdx);
        discPos = [P(:,1) + off .* cos(th), P(:,2) + off .* sin(th)];
        r = predictedOccupancyRisk(discPos, stationTimes, preds, vp.discRadius, cfg);
        colRisk = max(colRisk, r);
    end
    R(:,j) = colRisk;
end

R = applyBounds(R, offsets, dMin, dMax);
end

% =====================================================================
function R = applyBounds(R, offsets, dMin, dMax)
%APPLYBOUNDS Mark offsets outside the drivable corridor as forbidden.
%   Inf rather than a large finite penalty, so no weighting of the other
%   cost terms can ever buy a path off the road.
N = size(R,1);
for i = 1:N
    bad = offsets < dMin(i) | offsets > dMax(i);
    R(i, bad) = Inf;
end
end
