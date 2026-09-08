function [totalRisk, perSample, worstId] = conflictRisk(traj, preds, cfg, vp)
%CONFLICTRISK Aggregate collision risk along a candidate ego trajectory.
%
%   COMPONENT STATUS: REAL
%
%   [totalRisk, perSample, worstId] = CONFLICTRISK(traj, preds, cfg, vp)
%   scores a proposed ego trajectory against the predicted occupancy of
%   every tracked road user.
%
%   IR-PSC step 6, evaluated on the trajectory the planner actually intends
%   to drive (as opposed to TIMETOCONFLICT, which evaluates the do-nothing
%   trajectory).
%
%   The ego footprint is represented by the three covering discs from
%   VEHICLEPARAMS rather than a single point, so a long vehicle body
%   swinging past an obstacle is scored correctly rather than only its rear
%   axle being checked.
%
%   The trajectory total is the MAXIMUM per-sample risk, not the mean.
%   Averaging would let a long safe trajectory hide one lethal instant,
%   which is exactly the failure mode a safety score must not have.
%
%   Inputs:
%       traj  - trajectory struct with fields:
%               .pos     Nx2 positions
%               .times   Nx1 times from the planning instant (s)
%               .heading Nx1 headings (rad)
%       preds - prediction struct array from predictObstacles()
%       cfg   - config struct from irpscConfig()
%       vp    - vehicle params struct from vehicleParams()
%
%   Outputs:
%       totalRisk - scalar in [0,1], the peak risk over the trajectory
%       perSample - Nx1 risk at each trajectory sample
%       worstId   - track id of the largest contributor at the peak, or NaN
%
%   Example:
%       [R, rs] = conflictRisk(traj, preds, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTEDOCCUPANCYRISK, SCORETRAJECTORY, TIMETOCONFLICT.

N = size(traj.pos, 1);
perSample = zeros(N,1);
worstId   = NaN;
totalRisk = 0;

if isempty(preds) || N == 0
    return;
end

th = traj.heading(:);
t  = traj.times(:);

sampleWorst = nan(N,1);

for dIdx = 1:numel(vp.discOffsets)
    off = vp.discOffsets(dIdx);
    discPos = [traj.pos(:,1) + off .* cos(th), ...
               traj.pos(:,2) + off .* sin(th)];

    [r, wid] = predictedOccupancyRisk(discPos, t, preds, vp.discRadius, cfg);

    better = r > perSample;
    perSample(better)   = r(better);
    sampleWorst(better) = wid(better);
end

[totalRisk, iPeak] = max(perSample);
if isfinite(iPeak) && iPeak >= 1
    worstId = sampleWorst(iPeak);
end
end
