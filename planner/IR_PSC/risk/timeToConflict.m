function [ttc, criticalId, details] = timeToConflict(ego, preds, cfg, vp)
%TIMETOCONFLICT Shortest predicted time to a spatial conflict.
%
%   COMPONENT STATUS: REAL
%
%   [ttc, criticalId, details] = TIMETOCONFLICT(ego, preds, cfg, vp) finds
%   the earliest time at which the ego vehicle and any predicted road user
%   would come within their combined safety radius, assuming the ego
%   continues at its current speed and heading.
%
%   IR-PSC step 6, and the primary trigger for the decision logic.
%
%   Why a constant-heading ego assumption
%   -------------------------------------
%   TTC here answers "if I do nothing, when does this become a problem?".
%   That is precisely the question the decision logic needs in order to
%   decide whether to intervene, and it must NOT depend on the trajectory
%   the planner is currently proposing -- otherwise a planner that has
%   already dodged would report no danger and the system would never
%   escalate. Risk along the actual proposed trajectory is evaluated
%   separately by CONFLICTRISK.
%
%   Detection uses the predicted mean positions, sampled at the prediction
%   time step, and the combined footprint radii inflated by one standard
%   deviation. Sub-step linear interpolation refines the crossing instant.
%
%   Inputs:
%       ego   - ego state struct from makeEgoState()
%       preds - prediction struct array from predictObstacles()
%       cfg   - config struct from irpscConfig()
%       vp    - vehicle params struct from vehicleParams()
%
%   Outputs:
%       ttc        - seconds to the earliest conflict, or Inf if none is
%                    predicted within the horizon
%       criticalId - track id responsible for that earliest conflict, NaN if
%                    there is none
%       details    - struct with fields:
%                    .perObstacle  1xM vector of individual TTC values
%                    .ids          1xM corresponding track ids
%                    .minGap       m, closest predicted approach distance
%                    .level        char, 'none' | 'warning' | 'critical'
%
%   Example:
%       [ttc, id, d] = timeToConflict(ego, preds, cfg, vp);
%
%   Requires: base MATLAB only.
%
%   See also PREDICTEDOCCUPANCYRISK, DECISIONLOGIC.

ttc        = Inf;
criticalId = NaN;
details    = struct('perObstacle', [], 'ids', [], 'minGap', Inf, 'level', 'none');

if isempty(preds)
    return;
end

M       = numel(preds);
perObs  = inf(1, M);
ids     = nan(1, M);
minGap  = Inf;

fwd = [cos(ego.heading), sin(ego.heading)];

for k = 1:M
    p  = preds(k);
    tp = p.times(:);
    if isempty(tp)
        continue;
    end

    % Ego straight-line propagation at constant speed.
    egoPos = ego.pos(:).' + (ego.speed * tp) .* fwd;

    d   = sqrt(sum((egoPos - p.pos).^2, 2));
    % Conflict threshold, inflated by one sigma of the prediction so that a
    % vague prediction triggers earlier than a confident one.
    sig = 0.5 * (p.sigmaLong + p.sigmaLat);
    R   = vp.circumRadius + p.radius + sig;

    gap = d - R;
    minGap = min(minGap, min(gap));

    hit = find(gap <= 0, 1, 'first');
    if isempty(hit)
        continue;
    end

    if hit == 1
        tk = 0;                     % already in conflict at t = 0
    else
        % Linear interpolation of the zero crossing between two samples.
        g0 = gap(hit-1);
        g1 = gap(hit);
        frac = g0 / max(g0 - g1, eps);
        tk = tp(hit-1) + frac * (tp(hit) - tp(hit-1));
    end

    perObs(k) = tk;
    ids(k)    = p.id;

    if tk < ttc
        ttc        = tk;
        criticalId = p.id;
    end
end

details.perObstacle = perObs;
details.ids         = ids;
details.minGap      = minGap;

if ttc <= cfg.risk.ttcCritical
    details.level = 'critical';
elseif ttc <= cfg.risk.ttcWarning
    details.level = 'warning';
else
    details.level = 'none';
end
end
