function traj = safeStopTrajectory(ego, corridor, cfg, emergency)
%SAFESTOPTRAJECTORY Controlled stop along the best available path.
%
%   COMPONENT STATUS: REAL
%
%   traj = SAFESTOPTRAJECTORY(ego, corridor, cfg, emergency) produces a
%   trajectory that brings the vehicle to a standstill.
%
%   IR-PSC step 13: stop safely when no feasible trajectory exists.
%
%   This is the fallback the whole confidence-aware design exists to reach
%   gracefully. It is invoked when the corridor is invalid, when deformation
%   finds no admissible path, when clearance or feasibility checks fail, or
%   when the decision logic escalates to SAFE_STOP.
%
%   The stop follows the corridor centreline where one is available, so the
%   vehicle decelerates along the road rather than braking in a straight
%   line off a bend. Where no corridor exists it falls back to the current
%   heading, which is the best available guess.
%
%   Deceleration is constant, giving the classic v^2 = v0^2 - 2*a*s profile.
%   Constant deceleration is used rather than a smoother jerk-limited
%   profile because in this situation stopping distance matters more than
%   passenger comfort.
%
%   HONESTY NOTE: "safe stop" names the INTENT and the manoeuvre. It does
%   not assert that stopping is always safe, that the vehicle will always
%   stop in time, or that a following vehicle will not strike it. Those are
%   outcomes this project has not measured and does not claim.
%
%   Inputs:
%       ego       - ego state struct from makeEgoState()
%       corridor  - corridor struct from extractCorridor(), or [] if none
%       cfg       - config struct from irpscConfig()
%       emergency - logical; true uses cfg.ego.emergencyDecel, false uses
%                   cfg.ego.maxDecel. Default false.
%
%   Outputs:
%       traj - trajectory struct in the same format as generateTrajectory(),
%              with an extra field .isSafeStop = true
%
%   Example:
%       traj = safeStopTrajectory(ego, corridor, cfg, true);
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, DECISIONLOGIC.

if nargin < 4 || isempty(emergency), emergency = false; end

if emergency
    decel = cfg.ego.emergencyDecel;
else
    decel = cfg.ego.maxDecel;
end

v0       = max(ego.speed, 0);
stopDist = v0^2 / (2 * max(decel, 0.1));
stopDist = max(stopDist, 0.5);     % always emit a path of non-zero length

M = cfg.traj.numPoints;

% --- Geometry -----------------------------------------------------------
useCorridor = ~isempty(corridor) && isfield(corridor,'center') && ...
              size(corridor.center,1) >= 2 && corridor.length > 0.5;

if useCorridor
    sStop = linspace(0, min(stopDist, corridor.length), M).';
    P     = frenetToCartesian(corridor.center, sStop, 0);
    P(1,:) = ego.pos(:).';
else
    fwd = [cos(ego.heading), sin(ego.heading)];
    sStop = linspace(0, stopDist, M).';
    P = ego.pos(:).' + sStop .* fwd;
end

P = resamplePath(P, M);

sArc = pathArcLength(P);
th   = pathHeading(P);
k    = pathCurvature(P);

% --- Speed: constant deceleration to zero -------------------------------
L = max(sArc(end), 1e-3);
v = sqrt(max(v0^2 - 2 * decel * sArc, 0));
v(sArc >= L) = 0;
v(end)       = 0;

% --- Timing -------------------------------------------------------------
times = zeros(M,1);
for i = 2:M
    ds   = sArc(i) - sArc(i-1);
    vAvg = 0.5 * (v(i) + v(i-1));
    if vAvg < 0.05
        % Vehicle has stopped: remaining samples share the final instant.
        times(i) = times(i-1);
    else
        times(i) = times(i-1) + ds / vAvg;
    end
end

traj.pos        = P;
traj.heading    = th;
traj.curvature  = k;
traj.speed      = v;
traj.s          = sArc;
traj.times      = times;
traj.valid      = true;
traj.isSafeStop = true;
traj.emergency  = emergency;
end
