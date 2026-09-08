function out = irpscPlanner(grid, ego, obstacles, cfg, state)
%IRPSCPLANNER Indian-Road Predictive Safety Corridor planner.
%
%   COMPONENT STATUS: REAL (base MATLAB; no toolbox required)
%
%   out = IRPSCPLANNER(grid, ego, obstacles, cfg, state) runs one complete
%   planning cycle and returns the trajectory to drive, together with the
%   confidence and risk information the decision logic needs.
%
%   ============================================================
%   WHAT IS AND IS NOT CLAIMED AS THE CONTRIBUTION
%   ============================================================
%   The individual techniques used here -- ray casting, Frenet-style
%   road-aligned coordinates, Gaussian uncertainty propagation, dynamic
%   programming, moving-average smoothing -- are all standard and are NOT
%   claimed as novel.
%
%   The contribution is the HIERARCHY: which quantity is treated as primary,
%   and what the system does when it is unsure. Specifically:
%
%     - Drivable space, not lane markings, is the primary planning
%       constraint. The planner never asks where the lane is. On an unmarked
%       village road it behaves no differently in principle than on a marked
%       highway.
%
%     - The preferred path is CONTINUOUS -- the corridor centreline, which
%       moves smoothly as the road bends and narrows. It is not a choice
%       among a handful of fixed lane-offset candidates.
%
%     - Predicted occupancy with explicit uncertainty drives avoidance, and
%       uncertainty changes behaviour rather than merely being reported: a
%       vague prediction inflates the hazard ellipse and the planner gives
%       it more room, with no special-case code.
%
%     - Confidence in the system's own inputs is measured and acted upon,
%       degrading autonomy before a failure rather than after one.
%
%   ============================================================
%   THE 13 IR-PSC STEPS, AND WHERE EACH ONE LIVES
%   ============================================================
%     1. Estimate drivable space        -> seedCenterline
%     2. Identify road boundaries       -> extractCorridor
%     3. Track surrounding road users   -> (input; perception/tracking)
%     4. Predict short-term occupancy   -> predictObstacles
%     5. Represent uncertainty          -> predictionUncertainty
%     6. Estimate collision risk        -> lateralRiskGrid, conflictRisk
%     7. Continuous preferred path      -> extractCenterline
%     8. Deform around hazards          -> deformTrajectory
%     9. Preserve boundary margins      -> corridorBounds, checkClearance
%    10. Smooth spatially/temporally    -> generateTrajectory, smoothPath
%    11. Check vehicle feasibility      -> checkFeasibility
%    12. Confidence-aware behaviour     -> computeConfidence
%    13. Degrade or stop safely         -> safeStopTrajectory
%
%   ============================================================
%   SAFETY STATEMENT
%   ============================================================
%   This is simulation research code. It is NOT safety certified, has NOT
%   been validated on a real vehicle, and makes NO claim of guaranteed
%   collision avoidance. The checks below reduce risk within the modelled
%   world; they cannot bound behaviour in the real one.
%
%   Inputs:
%       grid      - occupancy grid struct from makeOccupancyGrid(), giving
%                   drivable space. Non-drivable cells include road edges,
%                   buildings, parked vehicles and (where modelled) potholes.
%       ego       - ego state struct from makeEgoState()
%       obstacles - obstacle struct array from makeObstacle(), may be empty
%       cfg       - config struct from irpscConfig()
%       state     - (optional) planner memory from the previous cycle, for
%                   temporal smoothing. Pass [] on the first cycle.
%
%   Outputs:
%       out - struct with fields:
%           .traj        trajectory struct to drive (see generateTrajectory)
%           .corridor    corridor struct from extractCorridor()
%           .preds       prediction struct array
%           .confidence  scalar in [0,1]
%           .confidenceBreakdown  struct explaining the confidence value
%           .risk        peak predicted risk along the chosen trajectory
%           .ttc         time to conflict on the do-nothing trajectory (s)
%           .ttcLevel    'none' | 'warning' | 'critical'
%           .feasible    logical, trajectory passed the feasibility check
%           .clearanceOk logical, trajectory passed the clearance check
%           .minClearance scalar, tightest clearance found (m)
%           .status      char, one of:
%                        'ok' | 'no_corridor' | 'deformation_infeasible' |
%                        'clearance_failed' | 'feasibility_failed'
%           .isSafeStop  logical, a safe-stop trajectory was returned
%           .score       trajectory score from scoreTrajectory()
%           .state       planner memory to pass into the next cycle
%           .timing      struct of per-stage elapsed seconds
%
%   Example:
%       cfg   = irpscConfig('village');
%       state = [];
%       out   = irpscPlanner(grid, ego, obstacles, cfg, state);
%       state = out.state;   % feed forward for temporal smoothing
%
%   Requires: base MATLAB only. No toolbox is used anywhere in this call
%   tree, so the planner can be unit-tested on any MATLAB installation.
%
%   See also EXTRACTCORRIDOR, PREDICTOBSTACLES, DEFORMTRAJECTORY,
%            COMPUTECONFIDENCE, SAFESTOPTRAJECTORY, DECISIONLOGIC.

if nargin < 5, state = []; end

vp = vehicleParams(cfg);
timing = struct();
tAll = tic;

% Previous-cycle memory for temporal smoothing.
if isempty(state) || ~isstruct(state)
    priorCenter  = [];
    priorProfile = [];
else
    priorCenter  = getFieldOr(state, 'center',  []);
    priorProfile = getFieldOr(state, 'profile', []);
end

out = struct();
out.isSafeStop = false;
out.status     = 'ok';

% =====================================================================
% STEPS 1-2: drivable space -> corridor
% =====================================================================
t0 = tic;
corridor = extractCorridor(grid, ego, cfg, priorCenter);
timing.corridor = toc(t0);

% =====================================================================
% STEPS 3-5: predict road users, with uncertainty
% =====================================================================
t0 = tic;
preds = predictObstacles(obstacles, cfg, corridor);
timing.prediction = toc(t0);

% =====================================================================
% STEP 12: confidence in our own inputs
% =====================================================================
t0 = tic;
[conf, confBreak] = computeConfidence(corridor, preds, obstacles, cfg);
timing.confidence = toc(t0);

% Time to conflict on the do-nothing trajectory. Computed regardless of the
% planning outcome, because the decision logic needs it even when planning
% succeeds.
[ttc, ~, ttcDetails] = timeToConflict(ego, preds, cfg, vp);

out.corridor             = corridor;
out.preds                = preds;
out.confidence           = conf;
out.confidenceBreakdown  = confBreak;
out.ttc                  = ttc;
out.ttcLevel             = ttcDetails.level;

% --- Bail out early if there is no usable corridor ---------------------
if ~corridor.valid
    out.status  = 'no_corridor';
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                             obstacles, grid, timing, tAll, ...
                             ttc > cfg.risk.ttcCritical);
    return;
end

% =====================================================================
% STEPS 6-8: risk grid, then deform the preferred path around hazards
% =====================================================================
t0 = tic;
[dMin, dMax] = corridorBounds(corridor, vp.halfWidth, cfg.deform.boundaryMargin);
[R, offsets] = lateralRiskGrid(corridor, preds, ego, cfg, vp, dMin, dMax);
timing.riskGrid = toc(t0);

% Ego's current lateral offset, so the profile starts where the car is.
[~, d0] = projectPointOnPath(corridor.center, ego.pos);

t0 = tic;
[dProfile, defInfo] = deformTrajectory(R, offsets, cfg, d0, priorProfile);
timing.deformation = toc(t0);

if ~defInfo.feasible
    out.status = 'deformation_infeasible';
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                             obstacles, grid, timing, tAll, true);
    return;
end

% =====================================================================
% STEPS 9-10: build and smooth the trajectory
% =====================================================================
t0 = tic;
stationRisk = zeros(size(R,1),1);
for i = 1:size(R,1)
    [~, j] = min(abs(offsets - dProfile(i)));
    r = R(i,j);
    if ~isfinite(r), r = 1; end
    stationRisk(i) = r;
end

traj = generateTrajectory(corridor, dProfile, ego, cfg, stationRisk);
timing.trajectory = toc(t0);

% =====================================================================
% STEPS 9 and 11: clearance and feasibility
% =====================================================================
t0 = tic;
[clearOk, minClear] = checkClearance(traj, grid, obstacles, cfg, vp);
[feasOk, ~]         = checkFeasibility(traj, cfg, vp);
timing.checks = toc(t0);

out.clearanceOk  = clearOk;
out.minClearance = minClear;
out.feasible     = feasOk;

if ~clearOk
    out.status = 'clearance_failed';
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                             obstacles, grid, timing, tAll, true);
    return;
end

if ~feasOk
    out.status = 'feasibility_failed';
    out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                             obstacles, grid, timing, tAll, false);
    return;
end

% =====================================================================
% STEP 11 (scoring) and hand-off
% =====================================================================
out.traj  = traj;
out.risk  = conflictRisk(traj, preds, cfg, vp);
out.score = scoreTrajectory(traj, corridor, preds, cfg, vp);

out.state = struct('center', corridor.center, 'profile', dProfile);

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function out = finishWithSafeStop(out, ego, corridor, cfg, vp, preds, ...
                                  obstacles, grid, timing, tAll, gentle)
%FINISHWITHSAFESTOP Populate the output with a safe-stop trajectory.
%   `gentle` true uses service braking, false uses emergency braking.
traj = safeStopTrajectory(ego, corridor, cfg, ~gentle);

out.traj       = traj;
out.isSafeStop = true;
out.risk       = conflictRisk(traj, preds, cfg, vp);
out.score      = scoreTrajectory(traj, corridor, preds, cfg, vp);

if ~isfield(out, 'clearanceOk')
    [clearOk, minClear]  = checkClearance(traj, grid, obstacles, cfg, vp);
    out.clearanceOk      = clearOk;
    out.minClearance     = minClear;
end
if ~isfield(out, 'feasible')
    out.feasible = checkFeasibility(traj, cfg, vp);
end

% Do not carry a deformation profile forward out of a failed cycle; the next
% cycle should start clean rather than inherit a plan that did not work.
out.state = struct('center', corridor.center, 'profile', []);

timing.total = toc(tAll);
out.timing   = timing;
end

% =====================================================================
function v = getFieldOr(s, name, defaultVal)
%GETFIELDOR Read a struct field, or return a default if it is absent.
if isstruct(s) && isfield(s, name)
    v = s.(name);
else
    v = defaultVal;
end
end
