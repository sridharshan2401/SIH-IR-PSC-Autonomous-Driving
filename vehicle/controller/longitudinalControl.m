function [accelCmd, cstate] = longitudinalControl(traj, ego, speedLimit, cfg, cstate)
%LONGITUDINALCONTROL PI speed controller with anti-windup.
%
%   COMPONENT STATUS: DOCUMENTED FALLBACK
%   (Substitute for MPC longitudinal control; see fallback F7 in
%   docs/COMPONENT_REGISTER.md. This is a real working controller.)
%
%   [accelCmd, cstate] = LONGITUDINALCONTROL(traj, ego, speedLimit, cfg,
%   cstate) computes the longitudinal acceleration command that tracks the
%   planned speed, capped by the decision logic's speed limit.
%
%   The target speed is read from the trajectory at the vehicle's current
%   arc-length position, so the controller follows the planned speed PROFILE
%   rather than a single set point. That matters because the profile already
%   encodes slowing for curvature and for risk.
%
%   Anti-windup
%   -----------
%   The integrator is clamped to +/- cfg.control.iMax, and integration is
%   suspended whenever the output is saturated and the error would push it
%   further into saturation. Without this, a long period of commanded
%   braking against an acceleration limit winds up a large integral term
%   that then causes a surge the moment the road clears -- exactly the wrong
%   behaviour after a hazard has passed.
%
%   Inputs:
%       traj       - trajectory struct from generateTrajectory()
%       ego        - ego state struct from makeEgoState()
%       speedLimit - m/s cap from decisionLogic()
%       cfg        - config struct from irpscConfig()
%       cstate     - controller state from the previous call, or [] to init
%
%   Outputs:
%       accelCmd - commanded longitudinal acceleration (m/s^2), saturated to
%                  [-cfg.ego.maxDecel, +cfg.ego.maxAccel]
%       cstate   - updated controller state, pass into the next call
%
%   Example:
%       cstate = [];
%       [a, cstate] = longitudinalControl(traj, ego, 8.0, cfg, cstate);
%
%   Requires: base MATLAB only.
%
%   See also PUREPURSUITCONTROL, DECISIONLOGIC, SPEEDPROFILE.

if isempty(cstate) || ~isstruct(cstate)
    cstate.integral = 0;
    cstate.lastErr  = 0;
end

dt = cfg.sim.dt;

% --- Target speed from the profile at the vehicle's position ------------
if size(traj.pos,1) >= 2
    sEgo = projectPointOnPath(traj.pos, ego.pos);
    sP   = traj.s;
    keep = [true; diff(sP) > 1e-9];
    if sum(keep) >= 2
        vTarget = interp1(sP(keep), traj.speed(keep), ...
                          min(max(sEgo, sP(1)), sP(end)), 'linear');
    else
        vTarget = traj.speed(1);
    end
else
    vTarget = 0;
end

vTarget = min(vTarget, speedLimit);
vTarget = max(vTarget, 0);

% --- PI ------------------------------------------------------------------
err = vTarget - ego.speed;

aMax = cfg.ego.maxAccel;
aMin = -cfg.ego.maxDecel;

unsaturated = cfg.control.kpSpeed * err + cfg.control.kiSpeed * cstate.integral;

% Integrate only when doing so will not deepen an existing saturation.
willSaturateHigh = unsaturated >= aMax && err > 0;
willSaturateLow  = unsaturated <= aMin && err < 0;
if ~willSaturateHigh && ~willSaturateLow
    cstate.integral = cstate.integral + err * dt;
    cstate.integral = max(min(cstate.integral, cfg.control.iMax), -cfg.control.iMax);
end

accelCmd = cfg.control.kpSpeed * err + cfg.control.kiSpeed * cstate.integral;
accelCmd = max(min(accelCmd, aMax), aMin);

cstate.lastErr = err;
end
