function corridor = extractCorridor(grid, ego, cfg, priorCenter)
%EXTRACTCORRIDOR Build the drivable safety corridor from free space.
%
%   COMPONENT STATUS: REAL
%
%   corridor = EXTRACTCORRIDOR(grid, ego, cfg, priorCenter) estimates the
%   left and right drivable boundaries ahead of the vehicle and the
%   continuous centreline between them.
%
%   This function is the centre of the IR-PSC contribution. It implements
%   the principle that gives the project its name:
%
%       "Lane is an optional cue; drivable space is the primary
%        planning constraint."
%
%   Nothing here looks for a painted line. Boundaries are found by casting
%   rays perpendicular to a seed path until free space ends. On a village
%   road with no markings at all, this still produces a usable corridor.
%
%   IR-PSC steps 1 and 2: estimate drivable space, identify boundaries.
%
%   Algorithm
%   ---------
%     1. SEEDCENTERLINE marches through free space to guess the road course.
%     2. Resample the seed to uniform stations.
%     3. At each station cast one ray left and one ray right, perpendicular
%        to the seed heading, until free space ends.
%     4. Erode both boundaries inward by cfg.corridor.boundaryErode so the
%        corridor never touches the exact edge of measured free space.
%     5. Recompute the centreline as the midpoint of the eroded boundaries,
%        then smooth it. This corrects the seed's greedy lateral bias.
%     6. Blend with the previous frame's centreline for temporal stability.
%
%   Why step 5 matters: the seed hugs whichever side had more space. Taking
%   the midpoint of the measured boundaries recentres the corridor, which is
%   what makes the preferred trajectory road-following rather than
%   free-space-hugging.
%
%   Inputs:
%       grid        - occupancy grid struct from makeOccupancyGrid()
%       ego         - ego state struct from makeEgoState()
%       cfg         - config struct from irpscConfig()
%       priorCenter - (optional) Nx2 centreline from the previous planning
%                     cycle, used for temporal smoothing. Pass [] on the
%                     first cycle.
%
%   Outputs:
%       corridor - struct with fields:
%           .center      Nx2 centreline points (world frame)
%           .left        Nx2 left boundary points
%           .right       Nx2 right boundary points
%           .s           Nx1 arc length along the centreline
%           .heading     Nx1 centreline heading (rad)
%           .width       Nx1 corridor width at each station (m)
%           .leftDist    Nx1 distance from centreline to left boundary
%           .rightDist   Nx1 distance from centreline to right boundary
%           .leftObserved  Nx1 logical, false where the ray found no edge
%           .rightObserved Nx1 logical, same for the right side
%           .minWidth    scalar, narrowest point (m)
%           .valid       logical, true if the corridor is usable
%           .quality     0..1 corridor confidence (see below)
%           .length      scalar, corridor length in m
%           .stalled     logical, from the seed march
%
%   Quality score
%   -------------
%   quality combines three things that are all genuinely measurable here:
%     - how much of the requested lookahead distance was achieved,
%     - what fraction of stations are at least minWidth wide,
%     - what fraction of boundary rays actually found an edge.
%   A ray that runs to maxRayLength without hitting anything means the road
%   edge was NOT observed (open field, missing return, sensor gap). Treating
%   that as a confident boundary would be dishonest, so it lowers quality
%   and, through it, the planner's confidence. This is what feeds
%   confidence-aware behaviour downstream.
%
%   Example:
%       corridor = extractCorridor(grid, ego, irpscConfig('village'), []);
%
%   Requires: base MATLAB only.
%
%   See also SEEDCENTERLINE, EXTRACTCENTERLINE, IRPSCPLANNER, COMPUTECONFIDENCE.

if nargin < 4, priorCenter = []; end

c = cfg.corridor;

% --- 1. Seed through free space -------------------------------------
[seed, seedInfo] = seedCenterline(grid, ego, cfg);

if size(seed,1) < 2
    corridor = emptyCorridor(ego, seedInfo);
    return;
end

% --- 2. Uniform stations --------------------------------------------
sSeed  = pathArcLength(seed);
nSta   = max(2, floor(sSeed(end) / c.stationStep) + 1);
seedR  = resamplePath(seed, nSta);
seedR  = smoothPath(seedR, 2, 1, true);       % light pre-smoothing
thSeed = pathHeading(seedR);

% --- 3. Cast boundary rays ------------------------------------------
N          = size(seedR,1);
leftPt     = zeros(N,2);
rightPt    = zeros(N,2);
leftD      = zeros(N,1);
rightD     = zeros(N,1);
leftObs    = false(N,1);
rightObs   = false(N,1);

for i = 1:N
    hL = thSeed(i) + pi/2;    % left normal
    hR = thSeed(i) - pi/2;    % right normal

    [dL, pL] = rayCastGrid(grid, seedR(i,:), hL, c.maxRayLength, c.rayStep);
    [dR, pR] = rayCastGrid(grid, seedR(i,:), hR, c.maxRayLength, c.rayStep);

    % A ray that ran the full length found no edge: not an observation.
    leftObs(i)  = dL < c.maxRayLength - 1e-6;
    rightObs(i) = dR < c.maxRayLength - 1e-6;

    % Erode inward so the corridor never sits exactly on measured free space.
    dLe = max(dL - c.boundaryErode, 0);
    dRe = max(dR - c.boundaryErode, 0);

    leftD(i)  = dLe;
    rightD(i) = dRe;
    leftPt(i,:)  = seedR(i,:) + dLe * [cos(hL), sin(hL)];
    rightPt(i,:) = seedR(i,:) + dRe * [cos(hR), sin(hR)];
end

% --- 4. Recentre and smooth -----------------------------------------
center = extractCenterline(leftPt, rightPt, c.centerlineSmooth);

% --- 5. Temporal blending -------------------------------------------
% Corridor estimates jitter frame to frame because free space jitters.
% Blending against the previous centreline keeps the preferred trajectory
% stable without hiding a genuine change in the road.
if ~isempty(priorCenter) && size(priorCenter,1) >= 2 && cfg.ablation.useTemporalSmoothing
    priorR = resamplePath(priorCenter, size(center,1));
    a      = cfg.deform.temporalAlpha;      % 1 = ignore history
    center = a * center + (1 - a) * priorR;
end

% --- 6. Geometry and quality ----------------------------------------
center  = resamplePath(center, N);
sCen    = pathArcLength(center);
thCen   = pathHeading(center);

% Re-measure widths against the final centreline so .width is consistent
% with .center rather than with the seed.
lDist = zeros(N,1);
rDist = zeros(N,1);
for i = 1:N
    lDist(i) = norm(leftPt(i,:)  - center(i,:));
    rDist(i) = norm(rightPt(i,:) - center(i,:));
end
width = lDist + rDist;

corridor.center        = center;
corridor.left          = leftPt;
corridor.right         = rightPt;
corridor.s             = sCen;
corridor.heading       = thCen;
corridor.width         = width;
corridor.leftDist      = lDist;
corridor.rightDist     = rDist;
corridor.leftObserved  = leftObs;
corridor.rightObserved = rightObs;
corridor.minWidth      = min(width);
corridor.length        = sCen(end);
corridor.stalled       = seedInfo.stalled;

lengthScore   = min(1, corridor.length / max(c.lookaheadDist, eps));
widthScore    = mean(width >= c.minWidth);
observedScore = mean([leftObs; rightObs]);

corridor.quality = max(0, min(1, ...
    0.40 * lengthScore + 0.35 * widthScore + 0.25 * observedScore));

corridor.valid = corridor.minWidth >= c.minWidth && ...
                 corridor.length   >= 2 * c.stationStep;
end

% =====================================================================
function corridor = emptyCorridor(ego, seedInfo)
%EMPTYCORRIDOR Degenerate corridor for when no drivable space was found.
%   Returned rather than throwing, so the planner can respond with a safe
%   stop instead of the whole simulation aborting.
p = ego.pos(:).';
corridor.center        = [p; p];
corridor.left          = [p; p];
corridor.right         = [p; p];
corridor.s             = [0; 0];
corridor.heading       = [ego.heading; ego.heading];
corridor.width         = [0; 0];
corridor.leftDist      = [0; 0];
corridor.rightDist     = [0; 0];
corridor.leftObserved  = [false; false];
corridor.rightObserved = [false; false];
corridor.minWidth      = 0;
corridor.length        = 0;
corridor.quality       = 0;
corridor.valid         = false;
corridor.stalled       = seedInfo.stalled;
end
