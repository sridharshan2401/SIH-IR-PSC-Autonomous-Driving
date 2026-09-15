function V = createDemoViewer(scn, cfg, opts)
%CREATEDEMOVIEWER Build the 3D judge-demonstration viewer for a scenario.
%
%   COMPONENT STATUS: REAL (visualisation). Base MATLAB graphics only.
%
%   V = CREATEDEMOVIEWER(scn, cfg, opts) creates the figure, draws the
%   static world once (terrain, drivable surface taken from the scenario's
%   occupancy grid, optional painted markings, roadside buildings and trees,
%   parked vehicles and stalls, pothole depressions, goal) and creates the
%   graphics objects that UPDATEDEMOVIEWER moves every frame.
%
%   HONESTY NOTE: this is a clear, schematic 3D rendering made with MATLAB
%   patch/line/text objects. It is NOT RoadRunner, NOT Unreal Engine and
%   NOT photorealistic. Nothing in it is animated independently: every
%   pose, trajectory, corridor, risk cell and label is taken from the
%   simulation state passed to UPDATEDEMOVIEWER. A photorealistic scene
%   would need RoadRunner (scene authoring) and Automated Driving Toolbox /
%   Simulink 3D Animation or the Unreal Engine co-simulation blocks, none of
%   which this project has installed or uses.
%
%   Layout
%   ------
%     left   : 3D world view (camera modes: chase / overview / top)
%     right  : mini-map, then the decision and perception panel
%     bottom : speed, risk and confidence over time
%
%   Keyboard (click the figure first): 1 chase, 2 overview, 3 top-down,
%   space pause/resume, q stop the run.
%
%   Inputs:
%       scn  - scenario struct from buildScenario()
%       cfg  - config struct from irpscConfig()
%       opts - (optional) struct: .camera ('chase'|'overview'|'top'),
%              .visible (true), .position ([x y w h] pixels),
%              .title (char)
%
%   Outputs:
%       V - viewer struct (graphics handles and viewer state)
%
%   Requires: base MATLAB only.
%
%   See also UPDATEDEMOVIEWER, RUNDEMO, REPLAYDEMO, ACTORMODEL3D.

if nargin < 3, opts = struct(); end
camMode  = getOpt(opts, 'camera', 'chase');
visible  = getOpt(opts, 'visible', true);
figPos   = getOpt(opts, 'position', [40 40 1500 860]);
titleStr = getOpt(opts, 'title', 'IR-PSC autonomous driving demonstration');

V = struct();
V.scn = scn;
V.cfg = cfg;
V.vp  = vehicleParams(cfg);
V.pal = palette();

if visible, vis = 'on'; else, vis = 'off'; end
V.fig = figure('Name', titleStr, 'NumberTitle', 'off', 'Color', V.pal.bg, ...
               'Position', figPos, 'Visible', vis, 'MenuBar', 'none', ...
               'InvertHardcopy', 'off', 'KeyPressFcn', @onKey);
setappdata(V.fig, 'viewerCtl', struct('camera', camMode, 'paused', false, 'quit', false));

% ---------------------------------------------------------------------
% Axes
% ---------------------------------------------------------------------
V.ax = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.0 0.19 0.705 0.81], ...
            'Color', V.pal.sky, 'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
hold(V.ax, 'on');
axis(V.ax, 'equal');
% Keep the axes visible (so its sky-coloured background is drawn) but hide
% ticks and rulers.
set(V.ax, 'XTick', [], 'YTick', [], 'ZTick', [], 'Box', 'off', ...
          'Projection', 'perspective', 'CameraViewAngleMode', 'manual', ...
          'CameraViewAngle', 38, 'Clipping', 'off');

V.axMap = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.715 0.68 0.28 0.31], ...
               'Color', [0.12 0.14 0.16], 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6], ...
               'FontSize', 7, 'Box', 'on');
hold(V.axMap, 'on');
axis(V.axMap, 'equal');

V.axHud = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.715 0.0 0.28 0.67], ...
               'XLim', [0 1], 'YLim', [0 1], 'Visible', 'off');
hold(V.axHud, 'on');

V.axTs = axes('Parent', V.fig, 'Units', 'normalized', 'Position', [0.045 0.035 0.64 0.13], ...
              'Color', [0.10 0.11 0.13], 'XColor', [0.75 0.75 0.75], 'YColor', [0.75 0.75 0.75], ...
              'FontSize', 8, 'Box', 'on', 'YLim', [0 1.05]);
hold(V.axTs, 'on');
xlabel(V.axTs, 'time (s)');

% ---------------------------------------------------------------------
% Static world
% ---------------------------------------------------------------------
drawTerrainAndRoad(V);
drawMarkings(V);
drawStaticObjects(V);
V = drawPotholeGround(V);
drawGoal(V);

light('Parent', V.ax, 'Position', [0.3 -0.5 1], 'Style', 'infinite');
light('Parent', V.ax, 'Position', [-0.6 0.4 0.8], 'Style', 'infinite', 'Color', [0.45 0.45 0.5]);

% ---------------------------------------------------------------------
% Dynamic objects (created once, updated every frame)
% ---------------------------------------------------------------------
a = V.ax;
V.h.corridor    = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                        'FaceColor', V.pal.corridor, 'FaceAlpha', 0.22, 'EdgeColor', 'none');
V.h.corrLeft    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.corridorEdge, 'LineWidth', 2.0);
V.h.corrRight   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.corridorEdge, 'LineWidth', 2.0);
V.h.riskCells   = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                        'FaceVertexCData', [1 0 0], 'FaceColor', 'flat', ...
                        'FaceAlpha', 0.50, 'EdgeColor', 'none');
V.h.potholeCells = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                        'FaceColor', V.pal.potholeCost, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
for i = 1:3
    V.h.predOcc(i) = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                           'FaceColor', V.pal.predOcc, 'FaceAlpha', 0.34 - 0.09*i, ...
                           'EdgeColor', V.pal.predOcc * 0.8, 'EdgeAlpha', 0.5);
end
V.h.predPaths   = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.predOcc, 'LineStyle', ':', 'LineWidth', 1.5);
V.h.prefPath    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.preferred, 'LineStyle', '--', 'LineWidth', 1.6);
V.h.prevTraj    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', [0.55 0.75 0.85], 'LineWidth', 1.0);
V.h.rejected    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', [1 0.25 0.25], 'LineStyle', '--', 'LineWidth', 1.8);
V.h.trajRibbon  = patch('Parent', a, 'Vertices', nan(3,3), 'Faces', [1 2 3], ...
                        'FaceColor', V.pal.traj, 'FaceAlpha', 0.85, 'EdgeColor', 'none');
V.h.trajLine    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.traj, 'LineWidth', 3.5);
V.h.trajDots    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 4, ...
                       'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', V.pal.traj);
V.h.stopWall    = patch('Parent', a, 'Vertices', nan(4,3), 'Faces', [1 2 3 4], ...
                        'FaceColor', [0.95 0.1 0.1], 'FaceAlpha', 0.45, 'EdgeColor', [1 0.2 0.2]);
V.h.egoRing     = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', [0.2 0.9 1.0], 'LineWidth', 2);
V.h.tracks      = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', V.pal.track, 'LineWidth', 1.6);
V.h.detCam      = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                       'Marker', 's', 'MarkerSize', 4, 'MarkerFaceColor', [1.0 0.9 0.2], 'MarkerEdgeColor', 'none');
V.h.detLidar    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                       'Marker', 'o', 'MarkerSize', 3, 'MarkerFaceColor', [0.3 1.0 0.5], 'MarkerEdgeColor', 'none');
V.h.detRadar    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                       'Marker', 'd', 'MarkerSize', 4, 'MarkerFaceColor', [1.0 0.4 0.9], 'MarkerEdgeColor', 'none');
V.h.potDets     = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, 'LineStyle', 'none', ...
                       'Marker', 'x', 'MarkerSize', 6, 'LineWidth', 1.5, 'Color', [1 0.6 0.1]);
V.h.potRings    = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', [1 0.6 0.1], 'LineWidth', 2.5);
V.h.potTentative = line('Parent', a, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                       'Color', [0.8 0.8 0.8], 'LineWidth', 1.2, 'LineStyle', '--');
V.h.banner      = text(0.5, 0.95, '', 'Parent', V.ax, 'Units', 'normalized', ...
                       'HorizontalAlignment', 'center', 'FontSize', 20, 'FontWeight', 'bold', ...
                       'Color', [1 1 1], 'BackgroundColor', [0.8 0 0], 'Margin', 6, 'Visible', 'off');
V.h.eventText   = text(0.01, 0.97, '', 'Parent', V.ax, 'Units', 'normalized', ...
                       'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', ...
                       'FontSize', 11, 'Color', [1 1 1], 'BackgroundColor', [0 0 0], 'Margin', 4);
V.h.camText     = text(0.99, 0.02, '', 'Parent', V.ax, 'Units', 'normalized', ...
                       'HorizontalAlignment', 'right', 'FontSize', 8, 'Color', [0.9 0.9 0.9]);

egoModel = actorModel3D('ego', V.vp.length, V.vp.width, true);
V.egoModel = egoModel;
V.h.ego = patch('Parent', a, 'Vertices', egoModel.V, 'Faces', egoModel.F, ...
                'FaceVertexCData', egoModel.C, 'FaceColor', 'flat', 'EdgeColor', [0.1 0.1 0.1], ...
                'EdgeAlpha', 0.3, 'FaceLighting', 'gouraud');
V.h.egoLabel = text(0, 0, 0, 'EGO', 'Parent', a, 'Color', [0.2 0.9 1], 'FontWeight', 'bold', ...
                    'FontSize', 9, 'HorizontalAlignment', 'center');

% Pools, grown on demand
V.actorIds = [];  V.actorH = [];  V.actorLbl = [];  V.actorModels = {};
V.trackLbl = [];
V.potLbl   = [];

% ---------------------------------------------------------------------
% Mini-map (static part)
% ---------------------------------------------------------------------
g = scn.grid;
step = max(1, round(0.6 / g.resolution));
free = ~g.occ(1:step:end, 1:step:end);
xm = g.origin(1) + (0:size(free,2)-1) * step * g.resolution;
ym = g.origin(2) + (0:size(free,1)-1) * step * g.resolution;
imagesc(V.axMap, xm, ym, double(free));
colormap(V.axMap, [0.18 0.25 0.16; 0.42 0.42 0.45]);
set(V.axMap, 'YDir', 'normal');
plot(V.axMap, scn.goal(1), scn.goal(2), 'p', 'MarkerSize', 12, 'MarkerFaceColor', [1 0.85 0.1], ...
     'MarkerEdgeColor', 'k');
for p = 1:numel(scn.potholes)
    plot(V.axMap, scn.potholes(p).pos(1), scn.potholes(p).pos(2), '.', 'Color', [0.1 0.1 0.1], 'MarkerSize', 10);
end
V.h.mapTraj  = plot(V.axMap, NaN, NaN, '-', 'Color', V.pal.traj, 'LineWidth', 2);
V.h.mapTrail = plot(V.axMap, NaN, NaN, '-', 'Color', [1 1 1], 'LineWidth', 1);
V.h.mapActors = plot(V.axMap, NaN, NaN, 'o', 'MarkerSize', 3, 'MarkerFaceColor', [1 0.3 0.3], 'MarkerEdgeColor', 'none');
V.h.mapEgo   = plot(V.axMap, NaN, NaN, 's', 'MarkerSize', 7, 'MarkerFaceColor', [0.2 0.9 1], 'MarkerEdgeColor', 'k');
title(V.axMap, 'Map (drivable space from occupancy grid)', 'Color', [0.85 0.85 0.85], 'FontSize', 8);
V.mapHalfWindow = 60;

% ---------------------------------------------------------------------
% Time series
% ---------------------------------------------------------------------
V.h.tsSpeed = plot(V.axTs, NaN, NaN, '-', 'Color', [0.2 0.9 1.0], 'LineWidth', 1.8);
V.h.tsRisk  = plot(V.axTs, NaN, NaN, '-', 'Color', [1.0 0.35 0.25], 'LineWidth', 1.5);
V.h.tsConf  = plot(V.axTs, NaN, NaN, '-', 'Color', [0.5 1.0 0.5], 'LineWidth', 1.2);
V.h.tsStop  = plot(V.axTs, NaN, NaN, 'LineStyle', 'none', 'Marker', '.', 'Color', [1 0 0], 'MarkerSize', 8);
title(V.axTs, 'speed / max speed (cyan)    risk (red)    confidence (green)    safe stop (red dots)', ...
      'Color', [0.85 0.85 0.85], 'FontSize', 8, 'FontWeight', 'normal');

% ---------------------------------------------------------------------
% HUD
% ---------------------------------------------------------------------
V = createHud(V);

V.camPos = [];  V.camTgt = [];
V.prevTrajPos = zeros(0,2);
V.frameCount = 0;
end

% =========================================================================
function drawTerrainAndRoad(V)
g = V.scn.grid;
res = g.resolution;
x0 = g.origin(1) - res/2;  x1 = g.origin(1) + (g.nCols - 0.5) * res;
y0 = g.origin(2) - res/2;  y1 = g.origin(2) + (g.nRows - 0.5) * res;
pad = 60;
% Terrain as a coarse tiled mesh (a single huge quad is not reliably
% rendered with perspective projection).
[TX, TY] = meshgrid(linspace(x0-pad, x1+pad, 24), linspace(y0-pad, y1+pad, 12));
Vt = [TX(:), TY(:), -0.06*ones(numel(TX),1)];
[nr, nc] = size(TX);
Ft = zeros((nr-1)*(nc-1), 4);  q = 0;
for c = 1:nc-1
    for r = 1:nr-1
        q = q + 1;
        i1 = (c-1)*nr + r;
        Ft(q,:) = [i1, i1 + nr, i1 + nr + 1, i1 + 1];
    end
end
patch('Parent', V.ax, 'Vertices', Vt, 'Faces', Ft, 'FaceColor', V.pal.terrain, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');

% Drivable surface: one quad per horizontal run of free cells.
free = ~g.occ;
Vv = zeros(0,3);  Ff = zeros(0,4);
for r = 1:g.nRows
    row = free(r, :);
    if ~any(row), continue; end
    d = diff([false, row, false]);
    starts = find(d == 1);
    stops  = find(d == -1) - 1;
    y = g.origin(2) + (r-1) * res;
    for q = 1:numel(starts)
        xa = g.origin(1) + (starts(q)-1) * res - res/2;
        xb = g.origin(1) + (stops(q)-1)  * res + res/2;
        n0 = size(Vv,1);
        Vv = [Vv; xa y-res/2 0; xb y-res/2 0; xb y+res/2 0; xa y+res/2 0]; %#ok<AGROW>
        Ff = [Ff; n0+1 n0+2 n0+3 n0+4]; %#ok<AGROW>
    end
end
patch('Parent', V.ax, 'Vertices', Vv, 'Faces', Ff, 'FaceColor', V.pal.road, ...
      'EdgeColor', 'none', 'FaceLighting', 'none');

% Unpaved shoulders along the scenario road, where its width is known.
hw = V.scn.roadHalfWidth(:);
if all(isfinite(hw))
    cl = V.scn.centerline;
    th = pathHeading(cl);
    n  = [-sin(th), cos(th)];
    for side = [-1 1]
        inner = cl + side * repmat(hw, 1, 2) .* n;
        outer = cl + side * repmat(hw + 1.1, 1, 2) .* n;
        K = size(cl,1);
        Vs = [inner, -0.03*ones(K,1); outer, -0.03*ones(K,1)];
        Fs = [(1:K-1).', (2:K).', K + (2:K).', K + (1:K-1).'];
        patch('Parent', V.ax, 'Vertices', Vs, 'Faces', Fs, 'FaceColor', V.pal.shoulder, ...
              'EdgeColor', 'none', 'FaceLighting', 'none');
    end
end
end

% =========================================================================
function drawMarkings(V)
mk = V.scn.markings;
hw = V.scn.roadHalfWidth(:);
if ~isstruct(mk) || mk.sTo <= mk.sFrom || ~all(isfinite(hw))
    return;
end
cl = V.scn.centerline;
sCl = pathArcLength(cl);
z = 0.012;
Vv = zeros(0,3);  Ff = zeros(0,4);
% Centre dashes
for s0 = mk.sFrom:6:mk.sTo-3
    [Vv, Ff] = addStrip(Vv, Ff, cl, s0, s0 + 3, 0, 0.12, z);
end
% Edge lines
for sideOff = [-1 1]
    for s0 = mk.sFrom:2:mk.sTo-2
        hwAt = interp1(sCl, hw, s0 + 1, 'linear', 'extrap');
        [Vv, Ff] = addStrip(Vv, Ff, cl, s0, s0 + 2.05, sideOff * (hwAt - 0.25), 0.12, z);
    end
end
patch('Parent', V.ax, 'Vertices', Vv, 'Faces', Ff, 'FaceColor', [0.95 0.95 0.92], ...
      'EdgeColor', 'none', 'FaceLighting', 'none');
% Where markings end, say so on the road.
pEnd = frenetToCartesian(cl, mk.sTo + 4, 0);
text(pEnd(1), pEnd(2), 0.3, 'markings end - planner never used them', 'Parent', V.ax, ...
     'Color', [1 1 0.7], 'FontSize', 8, 'HorizontalAlignment', 'center');
end

function [Vv, Ff] = addStrip(Vv, Ff, cl, sA, sB, d, w, z)
s  = [sA; sB];
pc = frenetToCartesian(cl, s, d);
th = atan2(pc(2,2) - pc(1,2), pc(2,1) - pc(1,1));
n  = [-sin(th), cos(th)] * w / 2;
n0 = size(Vv,1);
Vv = [Vv; pc(1,:) - n, z; pc(2,:) - n, z; pc(2,:) + n, z; pc(1,:) + n, z];
Ff = [Ff; n0+1 n0+2 n0+3 n0+4];
end

% =========================================================================
function drawStaticObjects(V)
so = V.scn.staticObjects;
for i = 1:numel(so)
    o = so(i);
    switch o.type
        case 'building'
            col = [0.80 0.66 0.52] + 0.08 * mod(i, 3);
            col = min(col, 1);
            m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), o.height], col);
            r = boxModel([2*o.halfSize(1)*1.05, 2*o.halfSize(2)*1.05, 0.3], [0.60 0.25 0.20]);
            r.V(:,3) = r.V(:,3) + o.height;
            m = mergeModels(m, r);
        case 'tree'
            m = boxModel([0.35, 0.35, o.height*0.55], [0.40 0.28 0.18]);
            c = boxModel([2*o.radius*1.6, 2*o.radius*1.6, o.height*0.55], [0.20 0.50 0.22]);
            c.V(:,3) = c.V(:,3) + o.height*0.45;
            m = mergeModels(m, c);
        case 'truck'
            m = actorModel3D('truck', 2*o.halfSize(1), 2*o.halfSize(2));
        case 'cart'
            m = actorModel3D('pushcart', 2*o.halfSize(1), 2*o.halfSize(2));
        case 'stall'
            m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), 0.9], [0.55 0.40 0.25]);
            c = boxModel([2*o.halfSize(1)*1.2, 2*o.halfSize(2)*1.3, 0.12], [0.85 0.25 0.20]);
            c.V(:,3) = c.V(:,3) + o.height - 0.12;
            m = mergeModels(m, c);
            for sx = [-1 1]
                for sy = [-1 1]
                    p = boxModel([0.08 0.08 o.height], [0.3 0.3 0.3]);
                    p.V(:,1) = p.V(:,1) + sx * o.halfSize(1);
                    p.V(:,2) = p.V(:,2) + sy * o.halfSize(2);
                    m = mergeModels(m, p);
                end
            end
        case 'debris'
            m = boxModel([2*o.radius, 2*o.radius, o.height], [0.55 0.30 0.20]);
        otherwise
            if ~isempty(o.halfSize)
                m = boxModel([2*o.halfSize(1), 2*o.halfSize(2), max(o.height, 0.5)], [0.5 0.5 0.5]);
            else
                m = boxModel([2*o.radius, 2*o.radius, max(o.height, 0.5)], [0.5 0.5 0.5]);
            end
    end
    Vw = placeModel(m.V, o.center, o.yaw);
    patch('Parent', V.ax, 'Vertices', Vw, 'Faces', m.F, 'FaceVertexCData', m.C, ...
          'FaceColor', 'flat', 'EdgeColor', [0.15 0.15 0.15], 'EdgeAlpha', 0.25, ...
          'FaceLighting', 'gouraud');
end
end

% =========================================================================
function V = drawPotholeGround(V)
ph = V.scn.potholes;
for p = 1:numel(ph)
    t = linspace(0, 2*pi, 28).';
    e = [ph(p).length/2 * cos(t), ph(p).width/2 * sin(t)];
    P = placeModel([e, zeros(size(t))], ph(p).pos, ph(p).yaw);
    % Depression: dark floor at -depth and a sloped rim.
    floorV = [P(:,1:2)*0.999 + 0.001*repmat(ph(p).pos,numel(t),1), -ph(p).depth*ones(numel(t),1)];
    patch('Parent', V.ax, 'XData', floorV(:,1), 'YData', floorV(:,2), 'ZData', floorV(:,3), ...
          'FaceColor', [0.10 0.09 0.08], 'EdgeColor', 'none', 'FaceLighting', 'none');
    inner = [ph(p).pos + 0.75*(P(:,1:2) - ph(p).pos), -ph(p).depth*ones(numel(t),1)];
    rim   = [P(:,1:2), 0.005*ones(numel(t),1)];
    nT = numel(t);
    Vr = [rim; inner];
    Fr = [(1:nT-1).', (2:nT).', nT + (2:nT).', nT + (1:nT-1).'];
    patch('Parent', V.ax, 'Vertices', Vr, 'Faces', Fr, 'FaceColor', [0.20 0.18 0.16], ...
          'EdgeColor', 'none', 'FaceLighting', 'none');
end
end

% =========================================================================
function drawGoal(V)
g = V.scn.goal;
t = linspace(0, 2*pi, 40);
r = V.scn.goalRadius;
patch('Parent', V.ax, 'XData', g(1) + r*cos(t), 'YData', g(2) + r*sin(t), 'ZData', 0.02*ones(size(t)), ...
      'FaceColor', [1 0.85 0.1], 'FaceAlpha', 0.18, 'EdgeColor', [1 0.85 0.1], 'LineWidth', 2);
line('Parent', V.ax, 'XData', [g(1) g(1)], 'YData', [g(2) g(2)], 'ZData', [0 5], ...
     'Color', [0.9 0.9 0.9], 'LineWidth', 3);
patch('Parent', V.ax, 'XData', g(1) + [0 0 2.2], 'YData', g(2) + [0 0 0], 'ZData', [5 3.8 4.4], ...
      'FaceColor', [1 0.85 0.1], 'EdgeColor', 'k');
text(g(1), g(2), 5.6, 'GOAL', 'Parent', V.ax, 'Color', [1 0.9 0.2], 'FontWeight', 'bold', ...
     'HorizontalAlignment', 'center', 'FontSize', 10);
end

% =========================================================================
function V = createHud(V)
a = V.axHud;
txt = @(x, y, s, sz, col, w) text(x, y, s, 'Parent', a, 'Units', 'data', 'FontSize', sz, ...
            'Color', col, 'FontWeight', w, 'VerticalAlignment', 'middle', 'Interpreter', 'none');
V.hud.title   = txt(0.02, 0.975, 'IR-PSC  |  LIVE PLANNER OUTPUT', 11, [0.2 0.9 1], 'bold');
V.hud.scen    = txt(0.02, 0.945, '', 8, [0.85 0.85 0.85], 'normal');
V.hud.time    = txt(0.02, 0.915, '', 8, [0.85 0.85 0.85], 'normal');
V.hud.speed   = txt(0.02, 0.865, '', 20, [1 1 1], 'bold');
V.hud.limit   = txt(0.50, 0.872, '', 7, [0.8 0.8 0.8], 'normal');
V.hud.stateBox = rectangle('Parent', a, 'Position', [0.02 0.795 0.96 0.045], ...
                           'FaceColor', [0.2 0.6 0.2], 'EdgeColor', 'none');
V.hud.state   = txt(0.04, 0.818, '', 12, [1 1 1], 'bold');
V.hud.reasonH = txt(0.02, 0.765, 'Decision', 8, [0.6 0.8 1], 'bold');
V.hud.reason  = txt(0.02, 0.735, '', 8, [1 1 1], 'normal');
V.hud.behH    = txt(0.02, 0.700, 'Planner behaviour', 8, [0.6 0.8 1], 'bold');
V.hud.beh     = txt(0.02, 0.670, '', 8, [1 1 1], 'normal');
V.hud.status  = txt(0.02, 0.640, '', 8, [0.85 0.85 0.85], 'normal');

V.hud.riskLbl = txt(0.02, 0.600, 'Risk', 8, [0.9 0.9 0.9], 'bold');
rectangle('Parent', a, 'Position', [0.25 0.588 0.73 0.024], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
V.hud.riskBar = rectangle('Parent', a, 'Position', [0.25 0.588 0.001 0.024], 'FaceColor', [1 0.3 0.2], 'EdgeColor', 'none');
V.hud.riskVal = txt(0.87, 0.600, '', 8, [1 1 1], 'bold');
V.hud.confLbl = txt(0.02, 0.565, 'Confidence', 8, [0.9 0.9 0.9], 'bold');
rectangle('Parent', a, 'Position', [0.25 0.553 0.73 0.024], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none');
V.hud.confBar = rectangle('Parent', a, 'Position', [0.25 0.553 0.001 0.024], 'FaceColor', [0.4 0.9 0.4], 'EdgeColor', 'none');
V.hud.confVal = txt(0.87, 0.565, '', 8, [1 1 1], 'bold');
V.hud.ttc     = txt(0.02, 0.525, '', 8, [0.9 0.9 0.9], 'normal');

V.hud.percH   = txt(0.02, 0.485, 'Perception (simulated sensors -> fusion -> tracking)', 8, [0.6 0.8 1], 'bold');
V.hud.perc1   = txt(0.02, 0.455, '', 8, [1 1 1], 'normal');
V.hud.perc2   = txt(0.02, 0.428, '', 8, [1 1 1], 'normal');
V.hud.potH    = txt(0.02, 0.390, 'Potholes (detected -> tracked -> planned action)', 8, [0.6 0.8 1], 'bold');
for i = 1:5
    V.hud.pot(i) = txt(0.02, 0.362 - 0.026*(i-1), '', 8, [1 1 1], 'normal');
end
V.hud.eventsH = txt(0.02, 0.215, 'Event log', 8, [0.6 0.8 1], 'bold');
for i = 1:4
    V.hud.ev(i) = txt(0.02, 0.188 - 0.024*(i-1), '', 7, [0.9 0.9 0.9], 'normal');
end

% Legend
y = 0.075;
items = {V.pal.traj, 'planned trajectory'; V.pal.preferred, 'preferred path'; ...
         V.pal.corridorEdge, 'drivable corridor'; [1 0.35 0.2], 'predicted risk'; ...
         V.pal.predOcc, 'predicted occupancy'; V.pal.potholeCost, 'pothole cost'; ...
         V.pal.track, 'tracked object'};
for i = 1:size(items,1)
    col = mod(i-1, 2);  rowI = floor((i-1)/2);
    xx = 0.02 + 0.49*col;  yy = y - 0.022*rowI + 0.02;
    rectangle('Parent', a, 'Position', [xx yy-0.007 0.05 0.014], 'FaceColor', items{i,1}, 'EdgeColor', 'none');
    txt(xx + 0.065, yy, items{i,2}, 7, [0.85 0.85 0.85], 'normal');
end
txt(0.02, -0.01, 'Schematic MATLAB 3D graphics - not RoadRunner. Simulation only.', 7, [0.7 0.7 0.7], 'normal');
V.eventLog = {};
end

% =========================================================================
function onKey(src, evt)
ctl = getappdata(src, 'viewerCtl');
switch lower(evt.Key)
    case {'1'}, ctl.camera = 'chase';
    case {'2'}, ctl.camera = 'overview';
    case {'3'}, ctl.camera = 'top';
    case {'space'}, ctl.paused = ~ctl.paused;
    case {'q', 'escape'}, ctl.quit = true;
end
setappdata(src, 'viewerCtl', ctl);
end

% =========================================================================
function m = boxModel(sz, col)
hx = sz(1)/2;  hy = sz(2)/2;  hz = sz(3);
m.V = [-hx -hy 0; hx -hy 0; hx hy 0; -hx hy 0; -hx -hy hz; hx -hy hz; hx hy hz; -hx hy hz];
m.F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
m.C = repmat(col, 6, 1) .* repmat([0.7; 1.0; 0.85; 0.95; 0.85; 0.95], 1, 3);
end

function m = mergeModels(a, b)
m.V = [a.V; b.V];
m.F = [a.F; b.F + size(a.V,1)];
m.C = [a.C; b.C];
end

function W = placeModel(Vb, pos, yaw)
c = cos(yaw);  s = sin(yaw);
W = [Vb(:,1)*c - Vb(:,2)*s + pos(1), Vb(:,1)*s + Vb(:,2)*c + pos(2), Vb(:,3)];
end

function p = palette()
p.bg          = [0.07 0.08 0.10];
p.sky         = [0.62 0.76 0.90];
p.terrain     = [0.40 0.52 0.30];
p.road        = [0.33 0.33 0.35];
p.shoulder    = [0.56 0.47 0.35];
p.corridor    = [0.20 0.95 0.45];
p.corridorEdge= [0.10 0.85 0.35];
p.traj        = [0.00 0.95 1.00];
p.preferred   = [1.00 1.00 1.00];
p.predOcc     = [1.00 0.25 0.25];
p.potholeCost = [0.75 0.30 1.00];
p.track       = [1.00 0.95 0.20];
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
