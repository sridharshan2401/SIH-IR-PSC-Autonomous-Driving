function [seed, info] = seedCenterline(grid, ego, cfg)
%SEEDCENTERLINE Greedy maximum-clearance march through drivable space.
%
%   COMPONENT STATUS: REAL
%
%   [seed, info] = SEEDCENTERLINE(grid, ego, cfg) produces a first rough
%   guess at where the road goes, using ONLY free space. No lane markings,
%   no map, no road model.
%
%   IR-PSC step 1: estimate drivable space.
%
%   How it works
%   ------------
%   Starting at the ego vehicle, the march repeatedly:
%     1. fans out a set of candidate headings around the current heading,
%     2. ray-casts each one to find how far it stays in free space,
%     3. scores each candidate on free distance minus a turn penalty,
%     4. steps one station along the winner.
%
%   The turn penalty matters: without it the march oscillates between two
%   near-equal openings and produces a zig-zag seed that later smoothing
%   cannot fully repair. With it, the march prefers to keep going straight
%   unless a turn buys real clearance.
%
%   This is a greedy local search, NOT a global planner. It can be fooled by
%   a wide dead end that looks better than the narrow correct road. That is
%   acceptable here because the seed only has to be roughly right: the
%   boundary ray-casting in EXTRACTCORRIDOR corrects the lateral placement,
%   and replanning happens several times per second.
%
%   Inputs:
%       grid - occupancy grid struct from makeOccupancyGrid()
%       ego  - ego state struct from makeEgoState()
%       cfg  - config struct from irpscConfig()
%
%   Outputs:
%       seed - Kx2 polyline of seed points, starting at ego.pos
%       info - struct with fields:
%              .stalled       logical, true if the march could not advance
%              .reachedLength m actually achieved (may be < lookaheadDist)
%              .steps         number of stations produced
%
%   Example:
%       [seed, info] = seedCenterline(grid, ego, irpscConfig('village'));
%
%   Requires: base MATLAB only.
%
%   See also EXTRACTCORRIDOR, RAYCASTGRID.

c        = cfg.corridor;
stepLen  = c.stationStep;
nSteps   = max(1, ceil(c.lookaheadDist / stepLen));
fanHalf  = c.seedHeadingFan;
nFan     = max(3, c.seedFanCount);
probeLen = min(c.lookaheadDist, 15.0);   % how far each candidate looks ahead

% Turn penalty in metres of "equivalent clearance" per radian of heading
% change. Tuned so a 0.3 rad turn must buy about 1 m of extra clearance.
turnPenalty = 3.0;

seed        = zeros(nSteps + 1, 2);
seed(1,:)   = ego.pos(:).';
curPos      = ego.pos(:).';
curHeading  = ego.heading;
stalled     = false;
k           = 1;

fanOffsets = linspace(-fanHalf, fanHalf, nFan);

for i = 1:nSteps
    bestScore   = -Inf;
    bestHeading = curHeading;
    bestFree    = 0;

    for j = 1:nFan
        hTest = curHeading + fanOffsets(j);
        d     = rayCastGrid(grid, curPos, hTest, probeLen, c.rayStep);
        score = d - turnPenalty * abs(fanOffsets(j));
        if score > bestScore
            bestScore   = score;
            bestHeading = hTest;
            bestFree    = d;
        end
    end

    % If even the best direction cannot fit one more station, stop marching.
    % A short seed is honest information: it tells the planner the corridor
    % is blocked, which is what triggers conservative mode or a safe stop.
    if bestFree < stepLen * 1.5
        stalled = true;
        break;
    end

    nextPos = curPos + stepLen * [cos(bestHeading), sin(bestHeading)];

    if isOccupiedAt(grid, nextPos)
        stalled = true;
        break;
    end

    k          = k + 1;
    seed(k,:)  = nextPos;
    curPos     = nextPos;
    curHeading = bestHeading;
end

seed = seed(1:k, :);

info.stalled       = stalled;
info.steps         = k;
if k >= 2
    sArc = pathArcLength(seed);
    info.reachedLength = sArc(end);
else
    info.reachedLength = 0;
end
end
