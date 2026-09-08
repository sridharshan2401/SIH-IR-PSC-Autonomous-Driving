function a = makeActor(id, class, waypoints, speed, varargin)
%MAKEACTOR Define a scripted road user for a scenario.
%
%   COMPONENT STATUS: REAL
%
%   a = MAKEACTOR(id, class, waypoints, speed, ...) defines one road user
%   that moves along a waypoint path at a given speed.
%
%   Actors are SCRIPTED, not intelligent. They follow their waypoints
%   regardless of what the ego vehicle does. That is deliberate for
%   evaluation: a reactive actor that politely yields would quietly make the
%   ego planner look better than it is, and would make runs irreproducible.
%   Scripted actors give every planner an identical, repeatable challenge,
%   which is what the baseline comparison requires.
%
%   The consequence, stated plainly: these scenarios cannot demonstrate
%   negotiation or mutual yielding, because the other road users cannot
%   negotiate. Interactive actor behaviour is future work, and is where
%   RoadRunner Scenario would be used on the destination machine.
%
%   Inputs:
%       id        - numeric scalar, unique actor identifier
%       class     - char, a class name from objectClasses()
%       waypoints - Kx2 path the actor follows, K >= 2
%       speed     - scalar travel speed (m/s)
%
%   Name-value pairs:
%       'StartTime' - s, when the actor becomes active. Default 0.
%                     This is how a sudden appearance is scripted -- for
%                     example cattle stepping out at t = 4 s.
%       'StopAtEnd' - logical, halt at the last waypoint rather than
%                     vanishing. Default true.
%       'Loop'      - logical, restart at the first waypoint. Default false.
%
%   Outputs:
%       a - actor struct with the fields above plus runtime state
%           (.pos, .vel, .heading, .s, .active, .finished)
%
%   Example:
%       a = makeActor(3, 'animal', [10 -8; 10 8], 1.2, 'StartTime', 4.0);
%
%   Requires: base MATLAB only.
%
%   See also STEPSCENARIO, OBJECTCLASSES, MAKEOBSTACLE.

validateattributes(waypoints, {'numeric'}, {'2d','ncols',2,'finite','real'}, ...
                   mfilename, 'waypoints');
if size(waypoints,1) < 2
    error('makeActor:tooFewWaypoints', 'Need at least 2 waypoints.');
end

p = inputParser;
p.FunctionName = mfilename;
addParameter(p, 'StartTime', 0,    @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'StopAtEnd', true, @(x) islogical(x) || isnumeric(x));
addParameter(p, 'Loop',      false,@(x) islogical(x) || isnumeric(x));
parse(p, varargin{:});

a.id        = id;
a.class     = char(class);
a.waypoints = waypoints;
a.speed     = speed;
a.startTime = p.Results.StartTime;
a.stopAtEnd = logical(p.Results.StopAtEnd);
a.loop      = logical(p.Results.Loop);

% Runtime state
a.s        = 0;
a.pos      = waypoints(1,:);
a.heading  = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
a.vel      = speed * [cos(a.heading), sin(a.heading)];
a.active   = (a.startTime <= 0);
a.finished = false;

sArc = pathArcLength(waypoints);
a.pathLength = sArc(end);
end
