function scn = buildScenario(name, seed)
%BUILDSCENARIO Construct one of the five required SIH validation scenarios.
%
%   COMPONENT STATUS: SIMPLIFIED SIMULATION
%
%   scn = BUILDSCENARIO(name, seed) returns a complete scenario definition:
%   drivable space, ego start and goal, and the scripted road users.
%
%   ============================================================
%   RELATIONSHIP TO ROADRUNNER -- READ THIS
%   ============================================================
%   These are MATLAB-defined scenarios, not RoadRunner scenes. They are
%   geometric abstractions: a centreline, a varying width, some blocked
%   regions, and scripted actors. They exercise the planner correctly, and
%   they run today with base MATLAB and nothing else.
%
%   They are NOT the detailed RoadRunner scenes the SIH problem statement
%   requires, and they must never be presented as such. The problem
%   statement asks for at least two detailed RoadRunner scenes -- a village
%   road and an urban intersection. Those must be authored in RoadRunner on
%   the destination machine. See roadrunner/ROADRUNNER_PLAN.md for the
%   specifications, and docs/COMPONENT_REGISTER.md fallback F9.
%
%   The five scenarios
%   ------------------
%     'village'      Unmarked village road. Narrow, irregular width, no
%                    markings of any kind, oncoming motorcycle, a parked
%                    truck narrowing the usable width, road-edge debris.
%     'urban'        Busy unsignalized intersection. Cross traffic from
%                    both directions with no signal and no right of way,
%                    plus an auto-rickshaw merging informally.
%     'highway'      Highway merge with slow-moving vehicles. A slow truck
%                    ahead and a merging vehicle entering without signalling.
%     'market'       Dense market area. Very narrow drivable space,
%                    pedestrians, pushcarts, a stationary bicycle, and
%                    unmarked crossing behaviour.
%     'cattle'       Sudden cattle crossing. An open road on which cattle
%                    step out at a scripted moment, plus a following
%                    motorcycle to make braking consequential.
%
%   Inputs:
%       name - char, one of 'village','urban','highway','market','cattle'
%       seed - (optional) integer random seed for reproducibility.
%              Default 0. Every experiment MUST pass an explicit seed so
%              results can be reproduced exactly.
%
%   Outputs:
%       scn - struct with fields:
%           .name       char, canonical scenario name
%           .profile    char, the irpscConfig profile to use
%           .grid       occupancy grid struct (drivable space)
%           .centerline Kx2 true road centreline (ground truth, for
%                       reference and plotting only -- the planner never
%                       receives this)
%           .egoStart   struct with .pos, .heading, .speed
%           .goal       1x2 goal position
%           .goalRadius m, how close counts as reaching the goal
%           .actors     actor struct array from makeActor()
%           .duration   s, scenario time limit
%           .seed       the seed used
%           .description char, human-readable summary
%           .sihScenario char, which official SIH scenario this covers
%
%   Example:
%       scn = buildScenario('village', 42);
%
%   Requires: base MATLAB only.
%
%   See also MAKEACTOR, BUILDROADGRID, RUNSCENARIO.

if nargin < 2 || isempty(seed), seed = 0; end

name = lower(char(name));
res  = 0.20;                      % m per cell

switch name
    case 'village'
        scn = scenarioVillage(res);
    case {'urban','urban_intersection','intersection'}
        scn = scenarioUrbanIntersection(res);
    case {'highway','highway_merge','merge'}
        scn = scenarioHighwayMerge(res);
    case 'market'
        scn = scenarioMarket(res);
    case {'cattle','cattle_crossing'}
        scn = scenarioCattleCrossing(res);
    otherwise
        error('buildScenario:unknownScenario', ...
              ['Unknown scenario "%s". Use one of: village, urban, ' ...
               'highway, market, cattle.'], name);
end

scn.seed = seed;
end
