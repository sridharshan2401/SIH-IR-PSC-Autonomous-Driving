function out = baselinePlanner(grid, ego, obstacles, cfg, state)
%BASELINEPLANNER Fixed-candidate reactive planner, for comparison only.
%
%   COMPONENT STATUS: REAL (a faithful implementation of the approach it
%   represents -- deliberately NOT an improved version of it)
%
%   out = BASELINEPLANNER(grid, ego, obstacles, cfg, state) plans using the
%   conventional structured-road recipe: a small fixed set of lateral lane
%   offsets, each evaluated against CURRENT obstacle positions, pick the
%   best. This is the approach IR-PSC is claimed to improve upon, so it
%   exists here to make that claim measurable rather than rhetorical.
%
%   WHY THIS FILE MATTERS
%   ---------------------
%   A contribution that is never compared against the obvious alternative is
%   an assertion, not a result. This baseline is what makes the IR-PSC
%   comparison honest. It is implemented properly -- same corridor input,
%   same vehicle limits, same scoring function, same safe-stop fallback --
%   so any difference in the results comes from the planning approach and
%   not from one planner being handicapped.
%
%   THE THREE DIFFERENCES, WHICH ARE THE WHOLE POINT
%   ------------------------------------------------
%     1. FIXED CANDIDATES. Five discrete lateral offsets instead of a
%        continuously deformable path. On a road whose usable width varies,
%        the right offset is usually not one of the five.
%
%     2. NO PREDICTION. Obstacles are treated as static at their current
%        positions. A pedestrian who will be in the road in 1.5 seconds is
%        invisible to this planner until they are already there.
%
%     3. NO CONFIDENCE AWARENESS. It plans at full commitment regardless of
%        how poor its inputs are. There is no mechanism by which uncertainty
%        can change its behaviour.
%
%   It DOES still use the drivable-space corridor for its reference path and
%   its lateral limits. That is a deliberate choice: giving the baseline the
%   corridor isolates the effect of prediction, continuous deformation and
%   confidence, rather than conflating them with the separate benefit of not
%   needing lane markings. The comparison is therefore conservative -- it
%   understates the full IR-PSC advantage rather than inflating it.
%
%   Inputs and Outputs match IRPSCPLANNER exactly, so the two are drop-in
%   interchangeable in the simulation loop and in the metrics code.
%   Additional output field:
%       .chosenOffset - the lateral offset selected, in metres
%
%   Example:
%       out = baselinePlanner(grid, ego, obstacles, cfg, []);
%
%   Requires: base MATLAB only.
%
%   See also IRPSCPLANNER, SCORETRAJECTORY, RUNCOMPARISON.

if nargin < 5, state = []; end

vp     = vehicleParams(cfg);
timing = struct();
tAll   = tic;

out            = struct();
out.isSafeStop = false;
out.status     = 'ok';

% --- Corridor (shared with IR-PSC, see note above) ---------------------
t0 = tic;
corridor = extractCorridor(grid, ego, cfg, []);   % no temporal smoothing
timing.corridor = toc(t0);

% --- No prediction: obstacles frozen where they are --------------------
t0 = tic;
cfgNoPred = cfg;
cfgNoPred.ablation.usePrediction  = false;
cfgNoPred.ablation.useUncertainty = false;
preds = predictObstacles(obstacles, cfgNoPred, corridor);
timing.prediction = toc(t0);

% --- No confidence awareness: always fully confident -------------------
conf = 1.0;
confBreak = struct('note', 'baseline planner does not model confidence');

[ttc, ~, ttcDetails] = timeToConflict(ego, preds, cfg, vp);

out.corridor            = corridor;
out.preds               = preds;
out.confidence          = conf;
out.confidenceBreakdown = confBreak;
out.ttc                 = ttc;
out.ttcLevel            = ttcDetails.level;

if ~corridor.valid
    out.status = 'no_corridor';
    out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, obstacles, ...
                           grid, timing, tAll);
    return;
end

% --- Five fixed lateral candidates -------------------------------------
% The classic structured-road candidate set: centre, plus two offsets each
% side at roughly half a lane and a full lane.
candidateOffsets = [-2.0, -1.0, 0.0, 1.0, 2.0];

[dMin, dMax] = corridorBounds(corridor, vp.halfWidth, cfg.deform.boundaryMargin);

t0 = tic;
N = size(corridor.center,1);
bestScore  = Inf;
bestTraj   = [];
bestOffset = 0;

for c = 1:numel(candidateOffsets)
    d = candidateOffsets(c);

    % Reject a candidate that leaves the corridor anywhere along its length.
    if any(d < dMin) || any(d > dMax)
        continue;
    end

    dProfile = repmat(d, N, 1);
    traj = generateTrajectory(corridor, dProfile, ego, cfg, zeros(N,1));
    if ~traj.valid
        continue;
    end

    % Reject on the same clearance and feasibility rules IR-PSC obeys.
    clearOk = checkClearance(traj, grid, obstacles, cfg, vp);
    feasOk  = checkFeasibility(traj, cfg, vp);
    if ~clearOk || ~feasOk
        continue;
    end

    s = scoreTrajectory(traj, corridor, preds, cfg, vp);
    if s < bestScore
        bestScore  = s;
        bestTraj   = traj;
        bestOffset = d;
    end
end
timing.candidates = toc(t0);

if isempty(bestTraj)
    % Every fixed candidate was rejected. This is the characteristic
    % failure of a fixed-candidate planner on a narrow or irregular road:
    % a perfectly drivable gap exists, but it does not happen to line up
    % with one of the five offsets.
    out.status = 'no_feasible_candidate';
    out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, obstacles, ...
                           grid, timing, tAll);
    return;
end

[clearOk, minClear] = checkClearance(bestTraj, grid, obstacles, cfg, vp);
feasOk = checkFeasibility(bestTraj, cfg, vp);

out.traj         = bestTraj;
out.chosenOffset = bestOffset;
out.risk         = conflictRisk(bestTraj, preds, cfg, vp);
out.score        = bestScore;
out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = feasOk;
out.state        = struct('center', corridor.center, 'profile', []);

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function out = baselineSafeStop(out, ego, corridor, cfg, vp, preds, ...
                                obstacles, grid, timing, tAll)
%BASELINESAFESTOP Emergency stop fallback, matching the IR-PSC fallback.
traj = safeStopTrajectory(ego, corridor, cfg, true);

out.traj         = traj;
out.isSafeStop   = true;
out.chosenOffset = 0;
out.risk         = conflictRisk(traj, preds, cfg, vp);
out.score        = scoreTrajectory(traj, corridor, preds, cfg, vp);

[clearOk, minClear] = checkClearance(traj, grid, obstacles, cfg, vp);
out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = checkFeasibility(traj, cfg, vp);
out.state        = struct('center', corridor.center, 'profile', []);

timing.total = toc(tAll);
out.timing   = timing;
end
