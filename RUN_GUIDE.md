# Run Guide

Every command, what it does, and what to expect.

**Before anything:** `setupPaths` — once per MATLAB session.

---

## Quick reference

| Command | What it does | Time |
|---|---|---|
| `setupPaths` | Adds source folders to the path | instant |
| `runAllTests('unit')` | Unit tests | ~1 min |
| `runAllTests('all')` | Unit + integration | ~5 min |
| `demoScenario('village')` | Animated single run | ~1 min |
| `runScenario('village', @irpscPlanner)` | One run, no animation | seconds |
| `runComparison({'village'}, 1:5)` | Baseline vs IR-PSC | ~2 min |
| `runAblation({'market'}, 1:5)` | Five ablations | ~6 min |
| `runAllExperiments(1:10)` | Everything | ~1 hour |

---

## Testing

```matlab
runAllTests('unit')          % fast, pure numerics
runAllTests('integration')   % full closed-loop runs
runAllTests('all')
```

**Expect failures on first run.** Nothing here has ever been executed.

Run a single suite while fixing something:

```matlab
runtests('testGeometry')
runtests('testCorridor')
runtests('testPredictionAndRisk')
runtests('testPlannerCore')
runtests('testDecisionAndVehicle')
runtests('testClosedLoop')
```

Or a single test:

```matlab
runtests('testGeometry/testCurvatureOfCircle')
```

---

## Running one scenario

```matlab
[log, M] = runScenario('village', @irpscPlanner);
```

Scenario names: `'village'` · `'urban'` · `'highway'` · `'market'` · `'cattle'`

Planners: `@irpscPlanner` · `@baselinePlanner`

### Options

```matlab
opts = struct( ...
    'seed',                42, ...   % reproducibility — always set this
    'verbose',             true, ... % print progress
    'maxTime',             20.0, ... % cut the run short
    'usePerfectPerception', true);   % bypass sensors and tracking

[log, M] = runScenario('market', @irpscPlanner, [], opts);
```

`usePerfectPerception` is the diagnostic switch worth knowing. It feeds ground truth straight to the planner. If a problem disappears with it on, the problem is in perception, not planning.

### Custom configuration

```matlab
cfg = irpscConfig('village');
cfg.ego.maxSpeed = 8.0;
cfg.deform.maxLateralShift = 1.0;
[log, M] = runScenario('village', @irpscPlanner, cfg, opts);
```

### Reading the result

```matlab
M.collisionCount        % distinct collision events
M.minClearance          % tightest clearance, metres (can be negative)
M.completed             % goal reached AND no collision
M.averageSpeed          % m/s
M.emergencyStops        % SAFE_STOP episodes
M.plannerFailures       % failed planning cycles
M.meanLatency           % seconds per replan
M.p95Latency            % 95th percentile — the tail governs deadlines
M.smoothness            % RMS d(curvature)/ds, lower is smoother
M.predictionError       % RMS prediction error, metres
M.lateralTrackingError  % how well the controller followed the plan
```

**Always read `M.provenance`.** It records that these are simulation results under simplified models, and it travels with the numbers so they cannot be quoted without their caveat.

---

## Demonstrations

```matlab
demoScenario('village')                  % IR-PSC, seed 1
demoScenario('market', 'baseline')       % the conventional planner
demoScenario('cattle', 'irpsc', struct('seed', 7, 'pauseTime', 0.03))
```

Options: `.seed` · `.pauseTime` · `.frameStep` · `.maxTime` · `.record`

### The side-by-side that makes the argument

```matlab
demoScenario('market', 'baseline', struct('seed', 3));
demoScenario('market', 'irpsc',    struct('seed', 3));
```

Same seed means the same world: identical actor timings, identical sensor noise, identical missed detections. Any difference is the planner.

### What to look for in the animation

| Element | Meaning |
|---|---|
| Grey dots | Drivable space. **No lane markings — there are none** |
| Dashed blue | Corridor boundaries, inferred from free space |
| Dotted grey | Corridor centreline: the preferred path |
| **Solid green** | The planned trajectory |
| **Gap between dotted and solid** | **The deformation — the contribution, visible** |
| Solid red | A safe-stop trajectory |
| Orange ellipses | Predicted occupancy; larger = less certain |
| Red boxes | Vulnerable road users |
| Blue boxes | Vehicles |

---

## Experiments

### Baseline comparison

```matlab
r = runComparison({'village','market'}, 1:10, struct('verbose', true));
printComparison(r);
```

Both planners get identical seeds, enforced by construction.

### Ablation study

```matlab
r = runAblation({'market'}, 1:10, struct('verbose', true));
```

Variants: `full` · `no_prediction` · `no_uncertainty` · `no_confidence` · `no_deformation` · `no_smoothing`

A variant showing near-zero deltas means that component is not earning its complexity in that scenario. **Report that honestly.**

### Everything

```matlab
r = runAllExperiments(1:10);
```

Saves to `results/full_evaluation_<timestamp>.mat`.

---

## Tuning

All tunables live in `config/irpscConfig.m`. Nothing else contains a magic constant.

```matlab
cfg = irpscConfig('market');

cfg.corridor.lookaheadDist  = 30;    % see further ahead
cfg.prediction.horizon      = 4.0;   % predict further
cfg.deform.maxLateralShift  = 3.0;   % allow bigger dodges
cfg.safety.lateralClearance = 0.8;   % keep more room
cfg.decision.confLow        = 0.5;   % become conservative sooner
```

### Effects worth knowing

| Change | Effect |
|---|---|
| ↑ `corridor.lookaheadDist` | Earlier reactions, more computation |
| ↑ `prediction.horizon` | Sees hazards sooner, but far-future predictions are vague |
| ↑ `prediction.lateralSpread.*` | Wider hazard ellipses, more cautious, may become timid |
| ↑ `deform.maxLateralShift` | Bigger avoidance manoeuvres, may leave the comfortable centre |
| ↑ `risk.nSigmaOccupancy` | Bigger hazard ellipses, more conservative |
| ↓ `decision.debounceEnter` | Reacts faster, risks chattering |
| ↑ `decision.debounceExit` | Stays cautious longer after a hazard clears |

---

## Debugging

### Inspect one planning cycle

```matlab
setupPaths;
scn = buildScenario('village', 1);
cfg = irpscConfig('village');
ego = makeEgoState(scn.egoStart.pos, scn.egoStart.heading, scn.egoStart.speed);

out = irpscPlanner(scn.grid, ego, [], cfg, []);

out.status              % 'ok' or the failure reason
out.corridor.valid
out.corridor.minWidth
out.corridor.quality
out.confidence
out.confidenceBreakdown % which component dropped it
out.timing              % seconds per stage
```

### Check the corridor alone

```matlab
corr = extractCorridor(scn.grid, ego, cfg, []);
figure; hold on; axis equal;
plot(corr.left(:,1),   corr.left(:,2),   'b--');
plot(corr.right(:,1),  corr.right(:,2),  'b--');
plot(corr.center(:,1), corr.center(:,2), 'k:');
plot(ego.pos(1), ego.pos(2), 'go', 'MarkerSize', 10);
```

If the corridor is wrong, everything downstream is wrong. Check this first.

### Find where a run went bad

```matlab
[log, M] = runScenario('market', @irpscPlanner, [], struct('seed',1));

find(log.collided)                    % which steps collided
unique(log.status)                    % which failures occurred
[~, k] = min(log.minClear);           % tightest moment
log.state{k}                          % what state it was in
log.risk(k)
log.confidence(k)
```

### Common failure statuses

| Status | Meaning | Look at |
|---|---|---|
| `no_corridor` | No drivable space found | The occupancy grid; is the ego inside it? |
| `deformation_infeasible` | Corridor exists, every offset blocked | Genuinely blocked, or margins too strict? |
| `clearance_failed` | Path produced, body would not fit | `cfg.safety` margins, vehicle width |
| `feasibility_failed` | Path produced, vehicle cannot drive it | Curvature limits, speed profile |
| `no_feasible_candidate` | Baseline only — none of its five offsets worked | **This is the fixed-candidate failure mode** |

---

## Reproducibility

Every run takes a seed. All randomness flows from one `RandStream`.

```matlab
log1 = runScenario('village', @irpscPlanner, [], struct('seed',42));
log2 = runScenario('village', @irpscPlanner, [], struct('seed',42));
isequal(log1.egoPos, log2.egoPos)     % must be true
```

If that is ever false, something is reading global random state and **every comparison in this project becomes invalid.** `testClosedLoop/testRunIsReproducibleWithSameSeed` guards it.

---

## What you cannot do

- **Get a metric without running something.** `computeMetrics` throws on an empty log, by design.
- **Open a `.slx`, `.sfx`, `.rrscene` or `.prj`.** None exists. Generate them with the builder scripts.
- **Quote real sensor or detector performance.** The sensor model is geometric; no detector has been trained.
- **Claim real-time performance.** Latency numbers are MATLAB interpreted timings on a development machine.
