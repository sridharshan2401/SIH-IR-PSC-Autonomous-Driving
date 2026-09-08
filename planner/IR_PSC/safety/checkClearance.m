function [ok, minClear, details] = checkClearance(traj, grid, obstacles, cfg, vp)
%CHECKCLEARANCE Verify a trajectory keeps clear of static space and obstacles.
%
%   COMPONENT STATUS: REAL
%
%   [ok, minClear, details] = CHECKCLEARANCE(traj, grid, obstacles, cfg, vp)
%   measures the smallest clearance the ego vehicle body would have along
%   the trajectory, against both the occupancy grid (road edges, buildings,
%   parked vehicles, potholes marked as non-drivable) and the current
%   positions of tracked road users.
%
%   IR-PSC step 8: preserve road-boundary and obstacle safety margins.
%
%   This is a CURRENT-STATE geometric check, deliberately separate from the
%   predictive risk assessment. Prediction can be wrong; the geometry of
%   where things are right now is the last line of defence, and it is
%   checked independently so that a prediction failure cannot silently
%   disable it.
%
%   The ego body is represented by the three covering discs from
%   VEHICLEPARAMS. Clearance for a disc is measured by ray-casting outward
%   in eight directions and taking the shortest free distance, minus the
%   disc radius. Eight rays is a compromise: it is cheap, and combined with
%   the conservative disc cover it reliably catches a body that is about to
%   clip a boundary.
%
%   Inputs:
%       traj      - trajectory struct from generateTrajectory()
%       grid      - occupancy grid struct, or [] to skip the static check
%       obstacles - obstacle struct array (current positions), may be empty
%       cfg       - config struct from irpscConfig()
%       vp        - vehicle params struct from vehicleParams()
%
%   Outputs:
%       ok       - logical, true if minClear >= cfg.safety.minLateralClearance
%       minClear - scalar smallest clearance found (m). Can be negative,
%                  meaning the body overlaps something.
%       details  - struct with fields:
%                  .perSample   Nx1 clearance at each trajectory sample
%                  .worstIndex  index of the tightest sample
%                  .staticMin   tightest clearance against the grid
%                  .dynamicMin  tightest clearance against road users
%
%   Example:
%       [ok, mc] = checkClearance(traj, grid, obstacles, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also CHECKFEASIBILITY, VEHICLEPARAMS, RAYCASTGRID.

N = size(traj.pos,1);
perSample  = inf(N,1);
staticMin  = Inf;
dynamicMin = Inf;

rayDirs = (0:7) * (pi/4);
probeLen = 6.0;                       % m, far enough to prove ample room

th = traj.heading(:);

for i = 1:N
    for dIdx = 1:numel(vp.discOffsets)
        off = vp.discOffsets(dIdx);
        c   = [traj.pos(i,1) + off*cos(th(i)), traj.pos(i,2) + off*sin(th(i))];

        % --- static occupancy -------------------------------------
        if ~isempty(grid)
            dMin = probeLen;
            for a = 1:numel(rayDirs)
                d = rayCastGrid(grid, c, rayDirs(a), probeLen, ...
                                cfg.corridor.rayStep);
                dMin = min(dMin, d);
            end
            clr = dMin - vp.discRadius;
            staticMin      = min(staticMin, clr);
            perSample(i)   = min(perSample(i), clr);
        end

        % --- dynamic obstacles ------------------------------------
        for k = 1:numel(obstacles)
            o    = obstacles(k);
            oRad = hypot(o.length, o.width) / 2;
            d    = hypot(c(1)-o.pos(1), c(2)-o.pos(2)) - vp.discRadius - oRad;
            dynamicMin   = min(dynamicMin, d);
            perSample(i) = min(perSample(i), d);
        end
    end
end

[minClear, worstIndex] = min(perSample);
ok = minClear >= cfg.safety.minLateralClearance;

details.perSample  = perSample;
details.worstIndex = worstIndex;
details.staticMin  = staticMin;
details.dynamicMin = dynamicMin;
end
