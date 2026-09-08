function [actors, groundTruth] = stepScenario(actors, t, dt)
%STEPSCENARIO Advance all scripted actors and emit ground truth.
%
%   COMPONENT STATUS: REAL
%
%   [actors, groundTruth] = STEPSCENARIO(actors, t, dt) moves every active
%   actor along its waypoint path and returns the current true state of all
%   of them in the standard obstacle format.
%
%   An actor becomes active when the simulation time reaches its startTime.
%   That single mechanism is what produces the sudden appearances these
%   scenarios need: cattle stepping into the road, a motorcycle emerging
%   from a side lane, a pedestrian leaving the verge. The road user simply
%   does not exist until its moment, and then it does.
%
%   Velocity is taken from the path tangent, so heading and velocity always
%   agree. Perception noise is applied downstream in SIMULATEDETECTIONS --
%   the values here are exact ground truth, which is what the prediction
%   error metric needs to compare against.
%
%   Inputs:
%       actors - actor struct array from makeActor()
%       t      - current simulation time (s)
%       dt     - time step (s)
%
%   Outputs:
%       actors      - updated actor struct array (pass back in next step)
%       groundTruth - obstacle struct array of currently active actors, in
%                     makeObstacle() format
%
%   Example:
%       [actors, gt] = stepScenario(actors, t, 0.05);
%
%   Requires: base MATLAB only.
%
%   See also MAKEACTOR, MAKEOBSTACLE, SIMULATEDETECTIONS.

groundTruth = struct('id',{},'class',{},'pos',{},'vel',{},'accel',{}, ...
                     'heading',{},'speed',{},'length',{},'width',{}, ...
                     'confidence',{},'age',{},'posCov',{},'vulnerable',{}, ...
                     'agility',{});

for i = 1:numel(actors)
    a = actors(i);

    % --- Activation ---------------------------------------------------
    if ~a.active
        if t >= a.startTime
            a.active = true;
        else
            actors(i) = a;
            continue;                    % not yet in the scene
        end
    end

    % --- Advance along the waypoint path ------------------------------
    if ~a.finished
        a.s = a.s + a.speed * dt;

        if a.s >= a.pathLength
            if a.loop
                a.s = mod(a.s, max(a.pathLength, eps));
            elseif a.stopAtEnd
                a.s = a.pathLength;
                a.finished = true;
            else
                a.active = false;        % leaves the scene entirely
                actors(i) = a;
                continue;
            end
        end
    end

    % --- Position and heading from the path ---------------------------
    sPath = pathArcLength(a.waypoints);
    keep  = [true; diff(sPath) > 1e-9];
    sU    = sPath(keep);
    wU    = a.waypoints(keep,:);

    if numel(sU) < 2
        pos = a.waypoints(1,:);
        hdg = a.heading;
    else
        sq  = min(max(a.s, 0), sU(end));
        pos = [interp1(sU, wU(:,1), sq, 'linear'), ...
               interp1(sU, wU(:,2), sq, 'linear')];

        % Heading from a short finite difference along the path.
        ds  = min(0.5, sU(end) / 20);
        sA  = min(max(sq - ds, 0), sU(end));
        sB  = min(max(sq + ds, 0), sU(end));
        pA  = [interp1(sU, wU(:,1), sA, 'linear'), interp1(sU, wU(:,2), sA, 'linear')];
        pB  = [interp1(sU, wU(:,1), sB, 'linear'), interp1(sU, wU(:,2), sB, 'linear')];
        if hypot(pB(1)-pA(1), pB(2)-pA(2)) > 1e-6
            hdg = atan2(pB(2)-pA(2), pB(1)-pA(1));
        else
            hdg = a.heading;
        end
    end

    if a.finished
        spd = 0;
    else
        spd = a.speed;
    end

    a.pos     = pos;
    a.heading = hdg;
    a.vel     = spd * [cos(hdg), sin(hdg)];

    actors(i) = a;

    groundTruth(end+1) = makeObstacle(a.id, a.class, a.pos, a.vel, ...
                                      'Heading', a.heading, ...
                                      'Confidence', 1.0, ...
                                      'Age', 100); %#ok<AGROW>
end
end
