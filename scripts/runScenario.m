function [log, M] = runScenario(scenarioName, plannerFn, cfg, opts)
%RUNSCENARIO Closed-loop simulation of one scenario with one planner.
%
%   COMPONENT STATUS: REAL (integration) over SIMPLIFIED sensor and vehicle
%   models. See docs/COMPONENT_REGISTER.md.
%
%   [log, M] = RUNSCENARIO(scenarioName, plannerFn, cfg, opts) runs the full
%   closed loop and returns the log and its metrics.
%
%   ============================================================
%   THE CLOSED LOOP
%   ============================================================
%     scenario ground truth
%         -> camera / LiDAR / radar simulation   (SIMPLIFIED)
%         -> multi-sensor fusion                 (SIMPLIFIED)
%         -> multi-object tracking               (FALLBACK)
%         -> short-term prediction               (REAL over SIMPLIFIED)
%         -> planner (IR-PSC or baseline)        (REAL)
%         -> decision logic                      (REAL)
%         -> controller                          (FALLBACK)
%         -> vehicle dynamics                    (SIMPLIFIED)
%         -> new ego state, feeding back into the sensors
%
%   The loop is genuinely closed: the ego state produced by the dynamics
%   model determines what the sensors see on the next step. Nothing is
%   fed back from ground truth to the planner.
%
%   ============================================================
%   REPRODUCIBILITY
%   ============================================================
%   All randomness comes from one seeded RandStream created here. Two runs
%   with the same seed, scenario and configuration produce byte-identical
%   logs. This is what makes the baseline comparison and the ablation study
%   valid -- both planners face exactly the same noise, the same missed
%   detections and the same false positives.
%
%   ============================================================
%   WHAT THE RESULTS MEAN
%   ============================================================
%   Simulation results only. The sensor model is geometric, not a real
%   detector. The vehicle model is kinematic, with no tyre or suspension
%   dynamics. Results are valid for comparing planning approaches under
%   identical conditions. They are NOT real-world safety evidence, and no
%   collision count from this function may be presented as such.
%
%   Inputs:
%       scenarioName - char, one of the names accepted by buildScenario()
%       plannerFn    - function handle, @irpscPlanner or @baselinePlanner.
%                      Must accept (grid, ego, obstacles, cfg, state).
%       cfg          - (optional) config struct. Default: the profile the
%                      scenario declares.
%       opts         - (optional) struct with fields:
%                      .seed        integer random seed (default 0)
%                      .verbose     logical, print progress (default false)
%                      .usePerfectPerception  logical (default false). When
%                                   true the sensor and tracking chain is
%                                   bypassed and ground truth goes straight
%                                   to the planner. Useful for isolating
%                                   planner behaviour from perception noise.
%                      .maxTime     override the scenario time limit
%
%   Outputs:
%       log - simulation log struct, as documented in computeMetrics()
%       M   - metric struct from computeMetrics()
%
%   Example:
%       [log, M] = runScenario('village', @irpscPlanner, [], struct('seed',42));
%       fprintf('min clearance %.2f m\n', M.minClearance);
%
%   Requires: base MATLAB only.
%
%   See also BUILDSCENARIO, IRPSCPLANNER, BASELINEPLANNER, COMPUTEMETRICS.

if nargin < 3, cfg  = []; end
if nargin < 4, opts = struct(); end

seed       = getOpt(opts, 'seed', 0);
verbose    = getOpt(opts, 'verbose', false);
perfectPer = getOpt(opts, 'usePerfectPerception', false);

% --- Build the scenario --------------------------------------------------
scn = buildScenario(scenarioName, seed);

if isempty(cfg)
    cfg = irpscConfig(scn.profile);
end

maxTime = getOpt(opts, 'maxTime', min(scn.duration, cfg.sim.maxTime));

vp = vehicleParams(cfg);
sc = sensorConfig();

% --- Single seeded stream drives every random draw ----------------------
stream = RandStream('mt19937ar', 'Seed', seed);

% --- Initial state -------------------------------------------------------
ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);
actors      = scn.actors;
plannerState = [];
trackerState = [];
decisionState = [];
controlState  = [];

dt     = cfg.sim.dt;
nSteps = max(1, floor(maxTime / dt));

% --- Preallocate the log -------------------------------------------------
log.t         = zeros(nSteps,1);
log.egoPos    = zeros(nSteps,2);
log.egoSpeed  = zeros(nSteps,1);
log.egoHeading= zeros(nSteps,1);
log.minClear  = inf(nSteps,1);
log.planTime  = nan(nSteps,1);
log.risk      = zeros(nSteps,1);
log.confidence= ones(nSteps,1);
log.ttc       = inf(nSteps,1);
log.state     = cell(nSteps,1);
log.status    = cell(nSteps,1);
log.collided  = false(nSteps,1);
log.traj      = cell(nSteps,1);
log.obstacles = cell(nSteps,1);
log.preds     = cell(nSteps,1);
log.tracks    = cell(nSteps,1);
log.nDetections = zeros(nSteps,1);
% Corridor is cached per step purely so demoScenario() can draw the
% extracted road boundaries during replay. Nothing in the control loop or
% the metrics reads it.
log.corridorCache = cell(nSteps,1);

lastPlan = [];
lastTraj = [];
goalReached = false;

% =====================================================================
% MAIN LOOP
% =====================================================================
for k = 1:nSteps
    t = (k-1) * dt;

    % --- 1. Advance the world ----------------------------------------
    [actors, groundTruth] = stepScenario(actors, t, dt);

    % --- 2. Perception ------------------------------------------------
    if perfectPer
        obstacles = groundTruth;
        nDet = numel(groundTruth);
    else
        camDets   = simulateDetections(groundTruth, ego, sc.camera, stream);
        lidarDets = simulateDetections(groundTruth, ego, sc.lidar,  stream);
        radarDets = simulateDetections(groundTruth, ego, sc.radar,  stream);
        nDet = numel(camDets) + numel(lidarDets) + numel(radarDets);

        fused = fuseDetections({camDets, lidarDets, radarDets}, cfg);
        [obstacles, trackerState] = multiObjectTracker(fused, dt, cfg, trackerState);
    end

    % --- 3. Plan (only on replan steps) --------------------------------
    if mod(k-1, cfg.sim.planEvery) == 0 || isempty(lastPlan)
        tPlan = tic;
        plan  = plannerFn(scn.grid, ego, obstacles, cfg, plannerState);
        log.planTime(k) = toc(tPlan);
        plannerState = plan.state;
        lastPlan = plan;
        lastTraj = plan.traj;
    else
        plan = lastPlan;
    end

    % --- 4. Decision logic ---------------------------------------------
    [action, decisionState] = decisionLogic(plan, cfg, decisionState);

    % --- 5. Select the trajectory to drive -----------------------------
    if action.useSafeStop
        traj = safeStopTrajectory(ego, plan.corridor, cfg, true);
    else
        traj = lastTraj;
    end

    % --- 6. Control -----------------------------------------------------
    if isempty(traj) || ~isfield(traj,'pos') || size(traj.pos,1) < 2
        steerCmd = 0;
        accelCmd = -cfg.ego.emergencyDecel;
    else
        steerCmd = purePursuitControl(traj, ego, cfg, vp);
        [accelCmd, controlState] = longitudinalControl(traj, ego, ...
                                        action.speedLimit, cfg, controlState);
    end

    % --- 7. Vehicle dynamics ---------------------------------------------
    ego = bicycleModelStep(ego, steerCmd, accelCmd, cfg);

    % --- 8. Collision check against ground truth -------------------------
    [collided, clearance] = checkCollisionAgainstTruth(ego, groundTruth, vp);

    % --- 9. Log -----------------------------------------------------------
    log.t(k)          = t;
    log.egoPos(k,:)   = ego.pos;
    log.egoSpeed(k)   = ego.speed;
    log.egoHeading(k) = ego.heading;
    log.minClear(k)   = clearance;
    log.risk(k)       = getFieldOr(plan, 'risk', 0);
    log.confidence(k) = getFieldOr(plan, 'confidence', 1);
    log.ttc(k)        = getFieldOr(plan, 'ttc', Inf);
    log.state{k}      = action.state;
    log.status{k}     = getFieldOr(plan, 'status', 'ok');
    log.collided(k)   = collided;
    log.traj{k}       = traj;
    log.obstacles{k}  = groundTruth;
    log.preds{k}      = getFieldOr(plan, 'preds', []);
    log.tracks{k}     = obstacles;
    log.nDetections(k)= nDet;
    log.corridorCache{k} = getFieldOr(plan, 'corridor', struct('valid', false));

    if verbose && mod(k, 20) == 0
        fprintf('t=%5.2f  v=%5.2f  state=%-20s risk=%.2f conf=%.2f\n', ...
                t, ego.speed, action.state, log.risk(k), log.confidence(k));
    end

    % --- 10. Termination --------------------------------------------------
    if hypot(ego.pos(1)-scn.goal(1), ego.pos(2)-scn.goal(2)) <= scn.goalRadius
        goalReached = true;
        log = truncateLog(log, k);
        break;
    end
end

if ~goalReached && numel(log.t) > nSteps
    log = truncateLog(log, nSteps);
end

log.goalReached = goalReached;
log.scenario    = scn.name;
log.sihScenario = scn.sihScenario;
log.seed        = seed;
log.planner     = func2str(plannerFn);
log.profile     = cfg.profile;
log.perfectPerception = perfectPer;

M = computeMetrics(log, cfg);
end

% =====================================================================
function [collided, clearance] = checkCollisionAgainstTruth(ego, truth, vp)
%CHECKCOLLISIONAGAINSTTRUTH Ground-truth collision and clearance check.
%   Uses the three covering discs and true obstacle extents. This is the
%   independent referee: it uses ground truth, never the planner's beliefs,
%   so a perception failure shows up as a collision rather than being
%   hidden by the same error that caused it.
collided  = false;
clearance = Inf;

for i = 1:numel(truth)
    o    = truth(i);
    oRad = hypot(o.length, o.width) / 2;
    for d = 1:numel(vp.discOffsets)
        off = vp.discOffsets(d);
        c   = [ego.pos(1) + off*cos(ego.heading), ...
               ego.pos(2) + off*sin(ego.heading)];
        gap = hypot(c(1)-o.pos(1), c(2)-o.pos(2)) - vp.discRadius - oRad;
        clearance = min(clearance, gap);
        if gap <= 0
            collided = true;
        end
    end
end
end

% =====================================================================
function log = truncateLog(log, n)
%TRUNCATELOG Trim every logged array to n entries.
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if isnumeric(v) || islogical(v)
        if size(v,1) >= n
            log.(f{i}) = v(1:n, :);
        end
    elseif iscell(v)
        if numel(v) >= n
            log.(f{i}) = v(1:n);
        end
    end
end
end

% =====================================================================
function v = getOpt(opts, name, defaultVal)
if isstruct(opts) && isfield(opts, name) && ~isempty(opts.(name))
    v = opts.(name);
else
    v = defaultVal;
end
end

% =====================================================================
function v = getFieldOr(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
