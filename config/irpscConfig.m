function cfg = irpscConfig(profile)
%IRPSCCONFIG Master configuration for the IR-PSC planning system.
%
%   COMPONENT STATUS: REAL
%
%   cfg = IRPSCCONFIG() returns the default ('urban') configuration.
%   cfg = IRPSCCONFIG(profile) returns a scenario-tuned configuration.
%
%   Every tunable number in the IR-PSC pipeline lives here. Nothing else in
%   the project should contain a magic constant. This makes ablation studies
%   straightforward: an ablation is just a modified copy of this struct.
%
%   Inputs:
%       profile - char, one of 'urban' (default), 'village', 'highway',
%                 'market', 'cattle'. Case-insensitive.
%
%   Outputs:
%       cfg - struct with fields .ego .corridor .prediction .risk .deform
%             .safety .traj .decision .control .sim .ablation .profile
%
%   Example:
%       cfg = irpscConfig('village');
%
%   Requires: base MATLAB only.
%
%   See also VEHICLEPARAMS, OBJECTCLASSES, IRPSCPLANNER.

if nargin < 1 || isempty(profile)
    profile = 'urban';
end
profile = lower(char(profile));

% ---------------------------------------------------------------------
% Ego vehicle: a representative compact Indian passenger car.
% SOURCE: nominal values chosen for simulation. NOT measured from a real
% vehicle. Replace with measured values before making any physical claim.
% ---------------------------------------------------------------------
cfg.ego.length         = 3.99;   % m, overall length
cfg.ego.width          = 1.68;   % m, overall width
cfg.ego.wheelbase      = 2.45;   % m
cfg.ego.rearOverhang   = 0.70;   % m, rear axle to rear bumper
cfg.ego.maxSpeed       = 16.7;   % m/s (~60 km/h)
cfg.ego.maxAccel       = 2.0;    % m/s^2, comfortable acceleration
cfg.ego.maxDecel       = 4.5;    % m/s^2, service braking magnitude
cfg.ego.emergencyDecel = 7.0;    % m/s^2, emergency braking magnitude
cfg.ego.maxSteer       = 0.61;   % rad (~35 deg) road-wheel angle
cfg.ego.maxSteerRate   = 0.50;   % rad/s
cfg.ego.maxLatAccel    = 3.0;    % m/s^2, comfort limit
cfg.ego.maxJerk        = 3.0;    % m/s^3

% ---------------------------------------------------------------------
% Corridor extraction. Drivable space is the PRIMARY planning constraint;
% lane markings are treated as an optional cue only.
% ---------------------------------------------------------------------
cfg.corridor.lookaheadDist    = 40.0;  % m, corridor length ahead of ego
cfg.corridor.stationStep      = 1.0;   % m, spacing of corridor stations
cfg.corridor.maxRayLength     = 12.0;  % m, max lateral search for a boundary
cfg.corridor.rayStep          = 0.10;  % m, ray-march resolution
cfg.corridor.minWidth         = 2.6;   % m, below this the corridor is unusable
cfg.corridor.seedHeadingFan   = 0.70;  % rad, half-angle of forward search fan
cfg.corridor.seedFanCount     = 21;    % headings tested per seed step
cfg.corridor.centerlineSmooth = 5;     % moving-average half-window (stations)
cfg.corridor.boundaryErode    = 0.15;  % m, shrink boundaries inward

% ---------------------------------------------------------------------
% Short-term prediction
% ---------------------------------------------------------------------
cfg.prediction.horizon     = 3.0;   % s, prediction horizon
cfg.prediction.dt          = 0.20;  % s, prediction time step
cfg.prediction.posNoiseStd = 0.30;  % m, initial position std dev
cfg.prediction.velNoiseStd = 0.40;  % m/s, initial velocity std dev
cfg.prediction.accelStd    = 1.20;  % m/s^2, process noise (drives growth)
cfg.prediction.headingStd  = 0.15;  % rad, initial heading std dev
cfg.prediction.maxSigma    = 6.0;   % m, cap on predicted std dev

% Irregular-motion allowance: EXTRA lateral uncertainty per class, expressed
% in metres of std dev added per second of horizon. On unstructured Indian
% roads, road users do not follow lanes, so lateral spread must grow faster
% for agile and erratic classes than a lane-following model would assume.
cfg.prediction.lateralSpread = struct( ...
    'car',          0.25, ...
    'bus',          0.15, ...
    'truck',        0.15, ...
    'motorcycle',   0.70, ...
    'bicycle',      0.55, ...
    'autorickshaw', 0.55, ...
    'pedestrian',   0.90, ...
    'pushcart',     0.45, ...
    'animal',       1.10, ...
    'unknown',      0.80);

% ---------------------------------------------------------------------
% Risk / conflict model
% ---------------------------------------------------------------------
cfg.risk.sigmaScale      = 1.0;   % multiplier on predicted uncertainty
cfg.risk.nSigmaOccupancy = 2.0;   % occupancy footprint = mean + n*sigma
cfg.risk.ttcCritical     = 1.5;   % s, below this TTC counts as critical
cfg.risk.ttcWarning      = 3.5;   % s, below this TTC counts as a warning
cfg.risk.riskFloor       = 1e-4;  % ignore risk contributions below this

% Vulnerable road users are weighted higher so the planner leaves more room.
% This is a DESIGN CHOICE reflecting Indian-road priorities. It is not a
% validated safety guarantee and confers no safety certification.
cfg.risk.classWeight = struct( ...
    'car',          1.00, ...
    'bus',          1.10, ...
    'truck',        1.10, ...
    'motorcycle',   1.15, ...
    'bicycle',      1.30, ...
    'autorickshaw', 1.10, ...
    'pedestrian',   1.60, ...
    'pushcart',     1.25, ...
    'animal',       1.50, ...
    'unknown',      1.30);

% ---------------------------------------------------------------------
% Trajectory deformation
% ---------------------------------------------------------------------
cfg.deform.maxLateralShift = 2.5;   % m, max deviation from corridor centre
cfg.deform.influenceRadius = 6.0;   % m, longitudinal reach of one hazard
cfg.deform.gain            = 1.0;   % overall deformation strength
cfg.deform.iterations      = 3;     % relaxation sweeps
cfg.deform.boundaryMargin  = 0.35;  % m, keep clear of corridor edges
cfg.deform.temporalAlpha   = 0.65;  % temporal blend (0 = frozen, 1 = no memory)

% ---------------------------------------------------------------------
% Safety thresholds
% ---------------------------------------------------------------------
cfg.safety.lateralClearance    = 0.50;  % m, desired side clearance
cfg.safety.minLateralClearance = 0.25;  % m, hard minimum
cfg.safety.longitudinalGap     = 2.0;   % m, standoff at zero speed
cfg.safety.timeGap             = 1.2;   % s, speed-dependent following gap
cfg.safety.maxCurvature        = 0.25;  % 1/m, tightest allowed path curvature
cfg.safety.minCorridorWidth    = 2.6;   % m, must exceed ego width + margins

% ---------------------------------------------------------------------
% Trajectory generation and smoothing
% ---------------------------------------------------------------------
cfg.traj.numPoints        = 41;    % samples in the output trajectory
cfg.traj.smoothWindow     = 5;     % moving-average half-window (samples)
cfg.traj.smoothPasses     = 2;     % repeated smoothing passes
cfg.traj.minSpeed         = 0.0;   % m/s
cfg.traj.speedComfortJerk = 1.5;   % m/s^3 used in speed profiling

% ---------------------------------------------------------------------
% Decision logic (see decision/decisionLogic.m and the Stateflow spec)
% ---------------------------------------------------------------------
cfg.decision.confHigh          = 0.70;  % above -> normal driving allowed
cfg.decision.confLow           = 0.40;  % below -> conservative mode
cfg.decision.riskHazard        = 0.25;  % risk above -> hazard assessment
cfg.decision.riskAvoid         = 0.50;  % risk above -> predictive avoidance
cfg.decision.riskStop          = 0.85;  % risk above -> safe stop
cfg.decision.debounceEnter     = 3;     % consecutive frames to ENTER a state
cfg.decision.debounceExit      = 8;     % consecutive frames to LEAVE a state
cfg.decision.minDwellFrames    = 5;     % minimum frames spent in any state
cfg.decision.conservativeSpeed = 4.0;   % m/s speed cap in conservative mode
cfg.decision.recoveryFrames    = 10;    % frames of good health before recovery

% ---------------------------------------------------------------------
% Controller
% ---------------------------------------------------------------------
cfg.control.lookaheadMin  = 3.0;   % m
cfg.control.lookaheadGain = 0.6;   % m per (m/s) of speed
cfg.control.kpSpeed       = 1.2;
cfg.control.kiSpeed       = 0.15;
cfg.control.iMax          = 2.0;   % integrator clamp

% ---------------------------------------------------------------------
% Simulation timing
% ---------------------------------------------------------------------
cfg.sim.dt        = 0.05;  % s, 20 Hz closed loop
cfg.sim.planEvery = 4;     % replan every N steps (=> 5 Hz planning)
cfg.sim.maxTime   = 60.0;  % s, per-scenario cap

% ---------------------------------------------------------------------
% Ablation switches. All true = full IR-PSC. Flipping one to false disables
% that stage so its contribution can be measured. See experiments/.
% ---------------------------------------------------------------------
cfg.ablation.usePrediction        = true;
cfg.ablation.useUncertainty       = true;
cfg.ablation.useConfidenceAware   = true;
cfg.ablation.useDeformation       = true;
cfg.ablation.useTemporalSmoothing = true;

% ---------------------------------------------------------------------
% Scenario profile overrides
% ---------------------------------------------------------------------
switch profile
    case 'urban'
        % Defaults above are tuned for the urban intersection case.
    case 'village'
        cfg.ego.maxSpeed            = 11.1;  % ~40 km/h
        cfg.corridor.maxRayLength   = 8.0;   % narrow road, boundaries close
        cfg.corridor.minWidth       = 2.8;
        cfg.deform.maxLateralShift  = 1.5;   % little room to move sideways
        cfg.safety.lateralClearance = 0.60;
    case 'highway'
        cfg.ego.maxSpeed           = 22.2;   % ~80 km/h
        cfg.corridor.lookaheadDist = 70.0;   % faster, so look further ahead
        cfg.prediction.horizon     = 4.0;
        cfg.deform.maxLateralShift = 3.5;
        cfg.risk.ttcWarning        = 5.0;
    case 'market'
        cfg.ego.maxSpeed            = 5.6;   % ~20 km/h
        cfg.corridor.lookaheadDist  = 20.0;
        cfg.prediction.horizon      = 2.5;
        cfg.safety.lateralClearance = 0.70;  % crowded, give more space
        cfg.decision.confLow        = 0.50;  % become conservative sooner
        cfg.decision.riskHazard     = 0.18;
    case 'cattle'
        cfg.ego.maxSpeed       = 11.1;
        cfg.prediction.horizon = 3.5;
        cfg.risk.ttcCritical   = 2.2;        % react earlier
        cfg.decision.riskStop  = 0.75;
    otherwise
        error('irpscConfig:unknownProfile', ...
              'Unknown profile "%s". Use urban|village|highway|market|cattle.', ...
              profile);
end

cfg.profile = profile;
end
