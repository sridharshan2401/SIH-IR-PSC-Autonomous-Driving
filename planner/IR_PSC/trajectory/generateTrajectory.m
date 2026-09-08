function traj = generateTrajectory(corridor, dProfile, ego, cfg, riskProfile)
%GENERATETRAJECTORY Build the final continuous trajectory from an offset profile.
%
%   COMPONENT STATUS: REAL
%
%   traj = GENERATETRAJECTORY(corridor, dProfile, ego, cfg, riskProfile)
%   converts the chosen lateral-offset profile into a world-frame,
%   time-parameterised trajectory the controller can follow.
%
%   IR-PSC steps 7 and 9: continuous road-following trajectory, spatially
%   and temporally smoothed.
%
%   Steps:
%     1. Smooth the offset profile. The dynamic-programming solution is
%        optimal on a discrete grid, so it can contain 0.25 m steps between
%        adjacent stations. Smoothing removes the quantisation without
%        materially changing the route chosen.
%     2. Map (station, offset) back to world coordinates.
%     3. Anchor the first point to the ego's actual position, so the
%        controller is never handed a path that starts somewhere the vehicle
%        is not.
%     4. Resample to a fixed number of points and smooth the geometry.
%     5. Compute heading and curvature.
%     6. Assign a speed profile, then integrate it to get arrival times.
%
%   Inputs:
%       corridor    - corridor struct from extractCorridor()
%       dProfile    - Nx1 lateral offsets from deformTrajectory()
%       ego         - ego state struct from makeEgoState()
%       cfg         - config struct from irpscConfig()
%       riskProfile - (optional) Nx1 risk per station, used for speed
%
%   Outputs:
%       traj - struct with fields:
%           .pos       Mx2 world positions
%           .heading   Mx1 heading (rad)
%           .curvature Mx1 signed curvature (1/m)
%           .speed     Mx1 speed (m/s)
%           .s         Mx1 arc length (m)
%           .times     Mx1 arrival time from now (s)
%           .valid     logical
%
%   Example:
%       traj = generateTrajectory(corr, dProfile, ego, cfg, riskPerStation);
%
%   Requires: base MATLAB only.
%
%   See also DEFORMTRAJECTORY, SPEEDPROFILE, SCORETRAJECTORY.

if nargin < 5, riskProfile = []; end

N = size(corridor.center, 1);
if N < 2 || ~corridor.valid
    traj = emptyTraj(ego);
    return;
end

dProfile = dProfile(:);
if numel(dProfile) ~= N
    dProfile = resampleSeries(dProfile, N);
end

% --- 1. Smooth the offset profile -------------------------------------
if cfg.traj.smoothWindow > 0
    dSmooth = smoothSeries(dProfile, cfg.traj.smoothWindow, cfg.traj.smoothPasses);
else
    dSmooth = dProfile;
end

% --- 2. Back to world coordinates -------------------------------------
P = frenetToCartesian(corridor.center, corridor.s, dSmooth);

% --- 3. Anchor to the ego position ------------------------------------
P(1,:) = ego.pos(:).';

% --- 4. Resample and smooth geometry ----------------------------------
M = cfg.traj.numPoints;
P = resamplePath(P, M);
P = smoothPath(P, cfg.traj.smoothWindow, cfg.traj.smoothPasses, true);

% --- 5. Geometry -------------------------------------------------------
sArc = pathArcLength(P);
th   = pathHeading(P);
k    = pathCurvature(P);

% --- 6. Speed and timing ----------------------------------------------
if isempty(riskProfile)
    riskM = zeros(M,1);
else
    riskM = resampleSeries(riskProfile(:), M);
end

vTarget = cfg.ego.maxSpeed;
v = speedProfile(sArc, k, vTarget, ego.speed, cfg, riskM);

% Integrate ds / v to arrival times, guarding against division by zero when
% the profile brings the vehicle to a stop.
times = zeros(M,1);
for i = 2:M
    ds    = sArc(i) - sArc(i-1);
    vAvg  = max(0.5 * (v(i) + v(i-1)), 0.1);
    times(i) = times(i-1) + ds / vAvg;
end

traj.pos       = P;
traj.heading   = th;
traj.curvature = k;
traj.speed     = v;
traj.s         = sArc;
traj.times     = times;
traj.valid     = true;
end

% =====================================================================
function traj = emptyTraj(ego)
%EMPTYTRAJ Degenerate single-point trajectory at the ego position.
p = ego.pos(:).';
traj.pos       = [p; p];
traj.heading   = [ego.heading; ego.heading];
traj.curvature = [0; 0];
traj.speed     = [0; 0];
traj.s         = [0; 0];
traj.times     = [0; 0];
traj.valid     = false;
end

% =====================================================================
function y = resampleSeries(x, n)
%RESAMPLESERIES Linearly resample a series to n samples.
x = x(:);
m = numel(x);
if m == n
    y = x;
elseif m < 2
    y = repmat(x(1), n, 1);
else
    y = interp1(linspace(0,1,m).', x, linspace(0,1,n).', 'linear');
end
end
