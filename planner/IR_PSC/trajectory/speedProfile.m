function v = speedProfile(sArc, curvature, vTarget, vCurrent, cfg, riskProfile)
%SPEEDPROFILE Feasible, comfortable speed along a planned path.
%
%   COMPONENT STATUS: REAL
%
%   v = SPEEDPROFILE(sArc, curvature, vTarget, vCurrent, cfg, riskProfile)
%   assigns a speed to every point of a planned path, respecting curvature,
%   acceleration limits and predicted risk.
%
%   Three limits are applied, then the tightest wins at each point:
%
%     1. Curvature limit. Lateral acceleration v^2 * |k| must not exceed
%        cfg.ego.maxLatAccel, so v <= sqrt(aLat / |k|). This is what makes
%        the vehicle slow for a bend rather than trying to take it flat out.
%
%     2. Risk limit. Where predicted risk is high the speed is scaled down,
%        so the vehicle passes a hazard slowly even when it has room to pass.
%
%     3. Acceleration limits, applied as a forward pass (do not accelerate
%        harder than maxAccel) followed by a BACKWARD pass (arrive at each
%        point slow enough to have braked legally). The backward pass is the
%        important one: it is what guarantees the vehicle starts braking
%        early enough for a slow point further ahead, instead of discovering
%        the limit when it is already too late to respect it.
%
%   Inputs:
%       sArc        - Nx1 arc length along the path (m)
%       curvature   - Nx1 signed curvature (1/m)
%       vTarget     - scalar desired cruising speed (m/s)
%       vCurrent    - scalar current ego speed (m/s)
%       cfg         - config struct from irpscConfig()
%       riskProfile - (optional) Nx1 risk in [0,1] at each point. Default 0.
%
%   Outputs:
%       v - Nx1 speed profile (m/s), non-negative
%
%   Example:
%       v = speedProfile(s, k, 8.0, 6.0, cfg, riskPerSample);
%
%   Requires: base MATLAB only.
%
%   See also GENERATETRAJECTORY, CHECKFEASIBILITY.

N = numel(sArc);
if nargin < 6 || isempty(riskProfile)
    riskProfile = zeros(N,1);
end
sArc        = sArc(:);
curvature   = curvature(:);
riskProfile = riskProfile(:);

% --- 1. Curvature limit ------------------------------------------------
aLat  = cfg.ego.maxLatAccel;
kAbs  = max(abs(curvature), 1e-6);
vCurv = sqrt(aLat ./ kAbs);

% --- 2. Risk limit -----------------------------------------------------
% Scale from full speed at zero risk down to 25% at maximum risk. Speed is
% never driven to zero here: stopping is a decision-logic action, not a
% side effect of the speed profiler.
vRisk = vTarget * (1 - 0.75 * min(max(riskProfile, 0), 1));

v = min([repmat(vTarget, N, 1), vCurv, vRisk], [], 2);
v = max(v, cfg.traj.minSpeed);

% --- 3a. Forward pass: acceleration limit ------------------------------
v(1) = min(v(1), max(vCurrent, 0));
for i = 2:N
    ds     = max(sArc(i) - sArc(i-1), 1e-6);
    vAccel = sqrt(max(v(i-1)^2 + 2 * cfg.ego.maxAccel * ds, 0));
    v(i)   = min(v(i), vAccel);
end

% --- 3b. Backward pass: braking limit ----------------------------------
for i = N-1:-1:1
    ds     = max(sArc(i+1) - sArc(i), 1e-6);
    vBrake = sqrt(max(v(i+1)^2 + 2 * cfg.ego.maxDecel * ds, 0));
    v(i)   = min(v(i), vBrake);
end

v = max(v, 0);
end
