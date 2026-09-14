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

% Only samples the vehicle actually reaches are checked. Geometry beyond a
% planned stop point (traj.reachable == false) is never driven.
if isfield(traj, 'reachable') && numel(traj.reachable) == numel(traj.speed)
    use = logical(traj.reachable(:));
else
    use = true(numel(traj.speed), 1);
end
k = traj.curvature(use);
v = traj.speed(use);
t = traj.times(use);
s = traj.s(use);
k = k(:);  v = v(:);  t = t(:);  s = s(:);
N = numel(v);

details.violations = {};

% --- 1. Curvature ------------------------------------------------------
maxK   = max([abs(k); 0]);
kLimit = min(vp.maxCurvature, cfg.safety.maxCurvature);
curvatureOk = maxK <= kLimit + 1e-6;

% --- 2. Lateral acceleration -------------------------------------------
% A sample only counts as a violation if a slower speed was physically
% attainable there. If the vehicle is ALREADY too fast for the bend under
% its nose, full braking cannot help at that sample; that is reported as
% an unavoidable transient instead of silently passing or rejecting the
% only plan that brakes as hard as possible.
latAcc = v.^2 .* abs(k);
maxLat = max([latAcc; 0]);
if N >= 1
    vFloor    = sqrt(max(v(1)^2 - 2 * cfg.ego.maxDecel * s, 0));
    avoidable = v > vFloor + 0.3;     % a slower speed was attainable here
else
    avoidable = false(0,1);
end
over = latAcc > cfg.ego.maxLatAccel + 1e-6;
latAccelOk = ~any(over & avoidable);
details.unavoidableLatAccel = any(over & ~avoidable);

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
% aLon(i) is the mean acceleration over [t(i), t(i+1)], centred at the
% interval midpoint, so consecutive values are separated by the MIDPOINT
% spacing. (The earlier code divided by t(i+1)-t(i), which is wrong for
% non-uniform sample times.)
if numel(aLon) >= 2
    tm  = 0.5 * (t(1:end-1) + t(2:end));
    dt2 = diff(tm);
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
