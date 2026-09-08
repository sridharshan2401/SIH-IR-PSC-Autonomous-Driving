function [ok, details] = checkFeasibility(traj, cfg, vp)
%CHECKFEASIBILITY Verify a trajectory is drivable by this vehicle.
%
%   COMPONENT STATUS: REAL
%
%   [ok, details] = CHECKFEASIBILITY(traj, cfg, vp) checks a proposed
%   trajectory against the vehicle's kinematic and comfort limits.
%
%   IR-PSC step 10: consider vehicle feasibility.
%
%   A planner that ignores this produces beautiful paths the car cannot
%   drive. The controller then cuts the corner, and the actual driven path
%   is nothing like the checked one -- so every safety check performed on
%   the plan becomes meaningless. Checking feasibility is what keeps the
%   plan and the reality connected.
%
%   Checks performed:
%     1. Curvature <= the vehicle's minimum turning radius allows, and
%        <= cfg.safety.maxCurvature.
%     2. Lateral acceleration v^2 * |k| <= cfg.ego.maxLatAccel.
%     3. Longitudinal acceleration within [-maxDecel, +maxAccel].
%     4. Longitudinal jerk within +/- cfg.ego.maxJerk.
%     5. Implied steering angle within +/- cfg.ego.maxSteer.
%
%   Inputs:
%       traj - trajectory struct from generateTrajectory()
%       cfg  - config struct from irpscConfig()
%       vp   - vehicle params struct from vehicleParams()
%
%   Outputs:
%       ok      - logical, true only if every check passes
%       details - struct with fields:
%                 .maxCurvature, .maxLatAccel, .maxAccel, .maxDecel,
%                 .maxJerk, .maxSteer  - the worst value observed
%                 .curvatureOk, .latAccelOk, .accelOk, .jerkOk, .steerOk
%                 .violations - cellstr naming each failed check
%
%   Example:
%       [ok, d] = checkFeasibility(traj, cfg, vp);
%       if ~ok, disp(d.violations); end
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, CHECKCLEARANCE, PATHCURVATURE.

k = traj.curvature(:);
v = traj.speed(:);
t = traj.times(:);
N = numel(v);

details.violations = {};

% --- 1. Curvature ------------------------------------------------------
maxK   = max(abs(k));
kLimit = min(vp.maxCurvature, cfg.safety.maxCurvature);
curvatureOk = maxK <= kLimit + 1e-6;

% --- 2. Lateral acceleration -------------------------------------------
latAcc     = v.^2 .* abs(k);
maxLat     = max(latAcc);
latAccelOk = maxLat <= cfg.ego.maxLatAccel + 1e-6;

% --- 3. Longitudinal acceleration --------------------------------------
if N >= 2
    dt = diff(t);
    dt(dt <= 0) = eps;
    aLon = diff(v) ./ dt;
else
    aLon = 0;
end
maxAcc  = max([aLon; 0]);
maxDec  = -min([aLon; 0]);
accelOk = maxAcc <= cfg.ego.maxAccel + 1e-3 && ...
          maxDec <= cfg.ego.maxDecel + 1e-3;

% --- 4. Jerk -----------------------------------------------------------
if numel(aLon) >= 2
    dt2 = diff(t(1:end-1));
    dt2(dt2 <= 0) = eps;
    jerk = diff(aLon) ./ dt2;
else
    jerk = 0;
end
maxJerk = max(abs([jerk; 0]));
jerkOk  = maxJerk <= cfg.ego.maxJerk + 1e-2;

% --- 5. Steering angle -------------------------------------------------
steer    = atan(vp.wheelbase * k);
maxSteer = max(abs(steer));
steerOk  = maxSteer <= cfg.ego.maxSteer + 1e-6;

% --- Collect -----------------------------------------------------------
details.maxCurvature = maxK;
details.maxLatAccel  = maxLat;
details.maxAccel     = maxAcc;
details.maxDecel     = maxDec;
details.maxJerk      = maxJerk;
details.maxSteer     = maxSteer;
details.curvatureOk  = curvatureOk;
details.latAccelOk   = latAccelOk;
details.accelOk      = accelOk;
details.jerkOk       = jerkOk;
details.steerOk      = steerOk;

if ~curvatureOk, details.violations{end+1} = 'curvature'; end
if ~latAccelOk,  details.violations{end+1} = 'lateralAcceleration'; end
if ~accelOk,     details.violations{end+1} = 'longitudinalAcceleration'; end
if ~jerkOk,      details.violations{end+1} = 'jerk'; end
if ~steerOk,     details.violations{end+1} = 'steeringAngle'; end

ok = isempty(details.violations);
end
