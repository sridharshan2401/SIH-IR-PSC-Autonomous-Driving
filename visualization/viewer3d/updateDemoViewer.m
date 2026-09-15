function V = updateDemoViewer(V, frame)
%UPDATEDEMOVIEWER Draw one simulation frame in the 3D demonstration viewer.
%
%   COMPONENT STATUS: REAL (visualisation of simulation state; no simulation)
%
%   V = UPDATEDEMOVIEWER(V, frame) moves every dynamic graphics object to
%   the state carried by `frame` (see RUNSCENARIO, opts.onStep, or
%   REPLAYDEMO). It does not compute anything the simulation did not:
%
%     ego pose            <- frame.ego (bicycle model output)
%     road users (3D)     <- frame.truth (scenario ground truth)
%     yellow boxes/labels <- frame.tracks (multi-object tracker output)
%     sensor dots         <- frame.detections (simulated camera/LiDAR/radar)
%     cyan trajectory     <- frame.traj (what the controller is following:
%                            the IR-PSC plan, or the safe-stop trajectory)
%     dashed white path   <- frame.plan.preferredPath (undeformed preference)
%     green corridor      <- frame.plan.corridor (drivable-space corridor)
%     red/amber cells     <- frame.plan.riskGrid.R (the DP's risk table)
%     purple cells        <- frame.plan.riskGrid.potholeCost
%     red ellipses        <- frame.plan.preds (predicted occupancy, 1/2/3 s)
%     pothole rings       <- frame.potholeTracks (pothole tracker output)
%     HUD                 <- frame.action (decision logic), frame.plan
%
%   Requires: base MATLAB only.
%
%   See also CREATEDEMOVIEWER, RUNDEMO, REPLAYDEMO.

if ~ishandle(V.fig)
    return;
end
V.frameCount = V.frameCount + 1;
ego  = frame.ego;
plan = frame.plan;
pal  = V.pal;
vp   = V.vp;

% ---------------------------------------------------------------------
% Ego
% ---------------------------------------------------------------------
[~, egoC] = egoFootprint(ego.pos, ego.heading, vp);
set(V.h.ego, 'Vertices', place(V.egoModel.V, egoC, ego.heading, 0));
set(V.h.egoLabel, 'Position', [egoC, 2.3]);
t = linspace(0, 2*pi, 30);
set(V.h.egoRing, 'XData', egoC(1) + 2.6*cos(t), 'YData', egoC(2) + 2.6*sin(t), 'ZData', 0.04*ones(size(t)));

% ---------------------------------------------------------------------
% Road users: ground-truth 3D models
% ---------------------------------------------------------------------
truth = frame.truth;
seen = false(size(V.actorIds));
for i = 1:numel(truth)
    o = truth(i);
    j = find(V.actorIds == o.id, 1);
    if isempty(j)
        m = actorModel3D(o.class, o.length, o.width);
        h = patch('Parent', V.ax, 'Vertices', m.V, 'Faces', m.F, 'FaceVertexCData', m.C, ...
                  'FaceColor', 'flat', 'EdgeColor', [0.1 0.1 0.1], 'EdgeAlpha', 0.3, ...
                  'FaceLighting', 'gouraud');
        V.actorIds(end+1) = o.id;
        V.actorH(end+1)   = h;
        V.actorModels{end+1} = m;
        seen(end+1) = false; %#ok<AGROW>
        j = numel(V.actorIds);
    end
    set(V.actorH(j), 'Vertices', place(V.actorModels{j}.V, o.pos, o.heading, 0), 'Visible', 'on');
    seen(j) = true;
end
for j = find(~seen)
    set(V.actorH(j), 'Visible', 'off');
end

% ---------------------------------------------------------------------
% Perception: tracks (wire boxes + labels) and raw detections
% ---------------------------------------------------------------------
tracks = frame.tracks;
[X, Y, Z] = deal([]);
for i = 1:numel(tracks)
    o = tracks(i);
    C = boxCorners(o.pos, o.heading, o.length + 0.3, o.width + 0.3);
    hgt = 2.0;
    for lev = [0.05 hgt]
        X = [X; C(:,1); C(1,1); NaN]; %#ok<AGROW>
        Y = [Y; C(:,2); C(1,2); NaN]; %#ok<AGROW>
        Z = [Z; repmat(lev, 5, 1); NaN]; %#ok<AGROW>
    end
    for c = 1:4
        X = [X; C(c,1); C(c,1); NaN]; %#ok<AGROW>
        Y = [Y; C(c,2); C(c,2); NaN]; %#ok<AGROW>
        Z = [Z; 0.05; hgt; NaN]; %#ok<AGROW>
    end
end
setLine(V.h.tracks, X, Y, Z);
V.trackLbl = ensureTextPool(V, V.trackLbl, numel(tracks), [1 0.95 0.3], 8);
for i = 1:numel(tracks)
    o = tracks(i);
    set(V.trackLbl(i), 'Position', [o.pos, 2.6], 'Visible', 'on', ...
        'String', sprintf('T%d %s %.1fm/s', o.id, o.class, o.speed));
end
for i = numel(tracks)+1:numel(V.trackLbl)
    set(V.trackLbl(i), 'Visible', 'off');
end

det = frame.detections;
setPoints(V.h.detCam,   det.camera, 0.9);
setPoints(V.h.detLidar, det.lidar,  0.6);
setPoints(V.h.detRadar, det.radar,  1.2);

% ---------------------------------------------------------------------
% Planner: corridor, risk grid, pothole cost, predictions, paths
% ---------------------------------------------------------------------
corr = getf(plan, 'corridor', []);
if isstruct(corr) && isfield(corr, 'valid') && isfield(corr, 'left') && size(corr.left,1) >= 2
    n = size(corr.left,1);
    if isfield(corr, 'blockedIdx') && ~isempty(corr.blockedIdx)
        n = max(2, corr.blockedIdx - 1);
    end
    L = corr.left(1:n,:);  R = corr.right(1:n,:);
    Vc = [L, 0.03*ones(n,1); R, 0.03*ones(n,1)];
    Fc = [(1:n-1).', (2:n).', n + (2:n).', n + (1:n-1).'];
    if corr.valid
        set(V.h.corridor, 'Vertices', Vc, 'Faces', Fc, 'FaceColor', pal.corridor);
    else
        set(V.h.corridor, 'Vertices', Vc, 'Faces', Fc, 'FaceColor', [0.9 0.2 0.2]);
    end
    setLine(V.h.corrLeft,  L(:,1), L(:,2), 0.06*ones(n,1));
    setLine(V.h.corrRight, R(:,1), R(:,2), 0.06*ones(n,1));
else
    set(V.h.corridor, 'Vertices', nan(3,3), 'Faces', [1 2 3]);
    setLine(V.h.corrLeft, NaN, NaN, NaN);
    setLine(V.h.corrRight, NaN, NaN, NaN);
end

rg = getf(plan, 'riskGrid', []);
if isstruct(rg) && isfield(rg, 'R') && ~isempty(rg.R) && size(rg.center,1) >= 2
    [Vr, Fr, Cr] = gridCells(rg, rg.R, 0.08, 0.05);
    if isempty(Fr)
        set(V.h.riskCells, 'Vertices', nan(3,3), 'Faces', [1 2 3], 'FaceVertexCData', [1 0 0]);
    else
        set(V.h.riskCells, 'Vertices', Vr, 'Faces', Fr, 'FaceVertexCData', Cr);
    end
    if isfield(rg, 'potholeCost') && ~isempty(rg.potholeCost) && any(rg.potholeCost(:) > 0)
        [Vp, Fp] = gridCells(rg, double(rg.potholeCost > 0), 0.5, 0.045);
        set(V.h.potholeCells, 'Vertices', Vp, 'Faces', Fp);
    else
        set(V.h.potholeCells, 'Vertices', nan(3,3), 'Faces', [1 2 3]);
    end
else
    set(V.h.riskCells, 'Vertices', nan(3,3), 'Faces', [1 2 3], 'FaceVertexCData', [1 0 0]);
    set(V.h.potholeCells, 'Vertices', nan(3,3), 'Faces', [1 2 3]);
end

preds = getf(plan, 'preds', []);
horizonsWanted = [1 2 3];
[PX, PY] = deal([]);
for hIdx = 1:3
    Ve = zeros(0,3);  Fe = zeros(0,24);
    for k = 1:numel(preds)
        p = preds(k);
        tp = p.times(:);
        q = find(tp >= horizonsWanted(hIdx) - 1e-6, 1);
        if isempty(q), continue; end
        if isfield(p, 'halfLength') && ~isempty(p.halfLength)
            aL = p.halfLength;  aW = p.halfWidth;
        else
            aL = p.radius;  aW = p.radius;
        end
        % Display the 1-sigma occupancy region (the risk model itself uses
        % cfg.risk.nSigmaOccupancy). Very vague predictions (new or false
        % tracks) are drawn only at 1 s, so they do not flood the view.
        a1 = aL + p.sigmaLong(q);
        b1 = aW + p.sigmaLat(q);
        if hIdx > 1 && max(a1, b1) > 5
            continue;
        end
        tt = linspace(0, 2*pi, 25);  tt(end) = [];
        E  = [a1*cos(tt).', b1*sin(tt).', zeros(24,1)];
        n0 = size(Ve,1);
        Ve = [Ve; place(E, p.pos(q,:), p.heading(q), 0.08 + 0.01*hIdx)]; %#ok<AGROW>
        Fe = [Fe; n0 + (1:24)]; %#ok<AGROW>
    end
    if isempty(Fe)
        set(V.h.predOcc(hIdx), 'Vertices', nan(3,3), 'Faces', [1 2 3]);
    else
        set(V.h.predOcc(hIdx), 'Vertices', Ve, 'Faces', Fe);
    end
end
for k = 1:numel(preds)
    PX = [PX; preds(k).pos(:,1); NaN]; %#ok<AGROW>
    PY = [PY; preds(k).pos(:,2); NaN]; %#ok<AGROW>
end
if isempty(PX), PX = NaN; PY = NaN; end
setLine(V.h.predPaths, PX, PY, 0.15*ones(size(PX)));

pp = getf(plan, 'preferredPath', zeros(0,2));
if size(pp,1) >= 2
    setLine(V.h.prefPath, pp(:,1), pp(:,2), 0.09*ones(size(pp,1),1));
else
    setLine(V.h.prefPath, NaN, NaN, NaN);
end

rej = getf(plan, 'rejectedTraj', []);
if isstruct(rej) && isfield(rej, 'pos') && size(rej.pos,1) >= 2
    setLine(V.h.rejected, rej.pos(:,1), rej.pos(:,2), 0.14*ones(size(rej.pos,1),1));
else
    setLine(V.h.rejected, NaN, NaN, NaN);
end

% The trajectory actually being followed ------------------------------------
tr = frame.traj;
isStop = strcmp(frame.action.state, 'SAFE_STOP');
beh = getf(plan, 'behaviour', struct());
if isStop
    tcol = [1 0.15 0.15];
elseif flag(beh, 'yielding') || flag(beh, 'following') || flag(beh, 'blockedAhead')
    tcol = [1 0.65 0.1];
else
    tcol = pal.traj;
end
if isstruct(tr) && isfield(tr, 'pos') && size(tr.pos,1) >= 2
    if isfield(tr, 'reachable') && numel(tr.reachable) == size(tr.pos,1)
        use = logical(tr.reachable(:));
    else
        use = true(size(tr.pos,1),1);
    end
    P = tr.pos(use,:);
    if size(P,1) >= 2
        th = pathHeading(P);
        nrm = [-sin(th), cos(th)] * 0.45;
        m = size(P,1);
        Vt = [P + nrm, 0.10*ones(m,1); P - nrm, 0.10*ones(m,1)];
        Ft = [(1:m-1).', (2:m).', m + (2:m).', m + (1:m-1).'];
        set(V.h.trajRibbon, 'Vertices', Vt, 'Faces', Ft, 'FaceColor', tcol);
        setLine(V.h.trajLine, P(:,1), P(:,2), 0.16*ones(m,1));
        set(V.h.trajLine, 'Color', tcol);
        set(V.h.trajDots, 'XData', P(1:4:end,1), 'YData', P(1:4:end,2), ...
            'ZData', 0.2*ones(numel(1:4:m),1), 'MarkerEdgeColor', tcol);
        % Planned stop point
        vEnd = tr.speed(find(use, 1, 'last'));
        if vEnd <= 0.05 && m >= 2
            pS = P(end,:);
            hS = th(end);
            nS = [-sin(hS), cos(hS)] * 1.6;
            fS = [cos(hS), sin(hS)] * (vp.frontOverhang + 0.3);
            q1 = pS + fS - nS;  q2 = pS + fS + nS;
            set(V.h.stopWall, 'Vertices', [q1 0; q2 0; q2 1.4; q1 1.4], 'Faces', [1 2 3 4]);
        else
            set(V.h.stopWall, 'Vertices', nan(4,3));
        end
    end
    if size(V.prevTrajPos,1) >= 2
        setLine(V.h.prevTraj, V.prevTrajPos(:,1), V.prevTrajPos(:,2), 0.07*ones(size(V.prevTrajPos,1),1));
    end
    if isfield(frame, 'replanned') && frame.replanned
        V.prevTrajPos = P;
    elseif isempty(V.prevTrajPos)
        V.prevTrajPos = P;
    end
end

% ---------------------------------------------------------------------
% Potholes: detections and tracks
% ---------------------------------------------------------------------
pd = frame.potholeDets;
if isempty(pd)
    setLine(V.h.potDets, NaN, NaN, NaN);
else
    setLine(V.h.potDets, pd(:,1), pd(:,2), 0.25*ones(size(pd,1),1));
end
pt = frame.potholeTracks;
[RX, RY, RZ, TX, TY, TZ] = deal([]);
V.potLbl = ensureTextPool(V, V.potLbl, numel(pt), [1 0.7 0.2], 8);
plannerPot = getf(plan, 'potholes', []);
for i = 1:numel(pt)
    tt = linspace(0, 2*pi, 30).';
    E = [(pt(i).length/2 + 0.25) * cos(tt), (pt(i).width/2 + 0.25) * sin(tt), zeros(30,1)];
    W = place(E, pt(i).pos, pt(i).yaw, 0.06);
    if pt(i).confirmed
        RX = [RX; W(:,1); NaN]; RY = [RY; W(:,2); NaN]; RZ = [RZ; W(:,3); NaN]; %#ok<AGROW>
        action = '';
        if ~isempty(plannerPot)
            li = find([plannerPot.id] == pt(i).id, 1);
            if ~isempty(li) && ~strcmp(plannerPot(li).action, 'none')
                action = [' -> ' upper(plannerPot(li).action)];
            end
        end
        set(V.potLbl(i), 'Position', [pt(i).pos, 1.2], 'Visible', 'on', 'Color', sevColor(pt(i).severity), ...
            'String', sprintf('POTHOLE %s %.0f cm%s', upper(pt(i).severity), 100*pt(i).depth, action));
    else
        TX = [TX; W(:,1); NaN]; TY = [TY; W(:,2); NaN]; TZ = [TZ; W(:,3); NaN]; %#ok<AGROW>
        set(V.potLbl(i), 'Position', [pt(i).pos, 0.8], 'Visible', 'on', 'Color', [0.8 0.8 0.8], ...
            'String', sprintf('pothole? (%d hits)', pt(i).hits));
    end
end
for i = numel(pt)+1:numel(V.potLbl)
    set(V.potLbl(i), 'Visible', 'off');
end
if isempty(RX), RX = NaN; RY = NaN; RZ = NaN; end
if isempty(TX), TX = NaN; TY = NaN; TZ = NaN; end
setLine(V.h.potRings, RX, RY, RZ);
setLine(V.h.potTentative, TX, TY, TZ);

% ---------------------------------------------------------------------
% Banner and event text
% ---------------------------------------------------------------------
if frame.action.emergencyBrake
    set(V.h.banner, 'String', 'EMERGENCY BRAKING', 'Visible', 'on', 'BackgroundColor', [0.8 0 0]);
elseif isStop
    set(V.h.banner, 'String', 'SAFE STOP', 'Visible', 'on', 'BackgroundColor', [0.75 0.25 0.05]);
elseif frame.collided
    set(V.h.banner, 'String', 'CONTACT RECORDED', 'Visible', 'on', 'BackgroundColor', [0.5 0 0.5]);
else
    set(V.h.banner, 'Visible', 'off');
end
set(V.h.eventText, 'String', sprintf(' %s ', getf(beh, 'reason', '')));

% ---------------------------------------------------------------------
% Camera
% ---------------------------------------------------------------------
ctl = getappdata(V.fig, 'viewerCtl');
fwd = [cos(ego.heading), sin(ego.heading)];
switch ctl.camera
    case 'overview'
        pos = [egoC - 38*fwd, 48];  tgt = [egoC + 22*fwd, 0];  up = [0 0 1];  va = 45;
    case 'top'
        pos = [egoC + 12*fwd, 95];  tgt = [egoC + 12*fwd, 0];  up = [fwd, 0];  va = 40;
    otherwise
        pos = [egoC - 15*fwd, 7.5]; tgt = [egoC + 16*fwd, 0.5]; up = [0 0 1];  va = 50;
end
if isempty(V.camPos) || ~strcmp(getf(V, 'camMode', ''), ctl.camera)
    V.camPos = pos;  V.camTgt = tgt;
else
    V.camPos = V.camPos + 0.35 * (pos - V.camPos);   % gentle follow, no jitter
    V.camTgt = V.camTgt + 0.35 * (tgt - V.camTgt);
end
V.camMode = ctl.camera;
set(V.ax, 'CameraPosition', V.camPos, 'CameraTarget', V.camTgt, 'CameraUpVector', up, ...
          'CameraViewAngle', va);
set(V.h.camText, 'String', sprintf('camera: %s   [1] chase  [2] overview  [3] top  [space] pause  [q] quit', ctl.camera));

% ---------------------------------------------------------------------
% Mini-map
% ---------------------------------------------------------------------
set(V.h.mapEgo, 'XData', egoC(1), 'YData', egoC(2));
if isstruct(tr) && isfield(tr, 'pos')
    set(V.h.mapTraj, 'XData', tr.pos(:,1), 'YData', tr.pos(:,2));
end
lg = frame.log;
k = frame.k;
if isstruct(lg) && isfield(lg, 'egoPos')
    set(V.h.mapTrail, 'XData', lg.egoPos(1:k,1), 'YData', lg.egoPos(1:k,2));
end
if ~isempty(truth)
    tp = reshape([truth.pos], 2, []).';
    set(V.h.mapActors, 'XData', tp(:,1), 'YData', tp(:,2));
else
    set(V.h.mapActors, 'XData', NaN, 'YData', NaN);
end
w = V.mapHalfWindow;
set(V.axMap, 'XLim', egoC(1) + [-w/2, w*1.5], 'YLim', egoC(2) + [-w*0.55, w*0.55]);

% ---------------------------------------------------------------------
% Time series
% ---------------------------------------------------------------------
if isstruct(lg) && isfield(lg, 't')
    t0 = max(0, frame.t - 30);
    idx = find(lg.t(1:k) >= t0);
    tt = lg.t(idx);
    set(V.h.tsSpeed, 'XData', tt, 'YData', lg.egoSpeed(idx) / max(V.cfg.ego.maxSpeed, eps));
    set(V.h.tsRisk,  'XData', tt, 'YData', lg.risk(idx));
    set(V.h.tsConf,  'XData', tt, 'YData', lg.confidence(idx));
    stopMask = strcmp(lg.state(idx), 'SAFE_STOP');
    if any(stopMask)
        set(V.h.tsStop, 'XData', tt(stopMask), 'YData', 0.02*ones(sum(stopMask),1));
    else
        set(V.h.tsStop, 'XData', NaN, 'YData', NaN);
    end
    set(V.axTs, 'XLim', [t0, max(t0 + 30, frame.t + 0.1)]);
end

% ---------------------------------------------------------------------
% HUD
% ---------------------------------------------------------------------
V = updateHud(V, frame, plan, beh);
end

% =========================================================================
function V = updateHud(V, frame, plan, beh)
h = V.hud;
act = frame.action;
set(h.scen, 'String', sprintf('%s   (seed %d)', V.scn.sihScenario, V.scn.seed));
set(h.time, 'String', sprintf('t = %5.1f s    step %d    planner %s', frame.t, frame.k, getf(frame, 'plannerName', 'IR-PSC')));
set(h.speed, 'String', sprintf('%4.1f km/h', 3.6 * frame.ego.speed));
set(h.limit, 'String', sprintf('limit %3.0f km/h\nsteer %+5.1f deg\naccel %+4.1f m/s^2', ...
    3.6 * act.speedLimit, rad2deg(frame.ego.steer), frame.accel));
[col, label] = stateStyle(act.state);
set(h.stateBox, 'FaceColor', col);
set(h.state, 'String', label);
set(h.reason, 'String', clip(act.reason, 62));
set(h.beh, 'String', clip(getf(beh, 'reason', ''), 62));
set(h.status, 'String', sprintf('planner status: %s', strrep(getf(plan, 'status', 'ok'), '_', ' ')));

risk = getf(plan, 'risk', 0);  conf = getf(plan, 'confidence', 1);
set(h.riskBar, 'Position', [0.25 0.588 max(0.001, 0.73*min(risk,1)) 0.024]);
set(h.riskVal, 'String', sprintf('%.2f', risk));
set(h.confBar, 'Position', [0.25 0.553 max(0.001, 0.73*min(conf,1)) 0.024]);
set(h.confVal, 'String', sprintf('%.2f', conf));
set(h.ttc, 'String', sprintf('TTC nominal %s    TTC planned %s    min clearance %s', ...
    fmtT(getf(plan, 'ttc', Inf)), fmtT(getf(plan, 'ttcPlanned', Inf)), fmtM(getf(plan, 'minClearance', Inf))));

tracks = frame.tracks;
if isempty(tracks)
    set(h.perc1, 'String', 'confirmed tracks: none');
else
    cls = {tracks.class};
    u = unique(cls);
    parts = cell(1, numel(u));
    for i = 1:numel(u)
        parts{i} = sprintf('%s x%d', u{i}, sum(strcmp(cls, u{i})));
    end
    set(h.perc1, 'String', clip(['confirmed tracks: ' strjoin(parts, ', ')], 62));
end
d = frame.detections;
set(h.perc2, 'String', sprintf('detections: camera %d  lidar %d  radar %d', ...
    size(d.camera,1), size(d.lidar,1), size(d.radar,1)));

pt = frame.potholeTracks;
pp = getf(plan, 'potholes', []);
lines = {};
for i = 1:numel(pt)
    act2 = '';
    if ~isempty(pp)
        li = find([pp.id] == pt(i).id, 1);
        if ~isempty(li), act2 = pp(li).action; end
    end
    dist = hypot(pt(i).pos(1) - frame.ego.pos(1), pt(i).pos(2) - frame.ego.pos(2));
    if dist > 60, continue; end
    lines{end+1} = sprintf('#%d %-9s %-8s d=%4.1fcm %4.0fm %s', pt(i).id, pt(i).state, ...
        pt(i).severity, 100*pt(i).depth, dist, upper(act2)); %#ok<AGROW>
end
for i = 1:numel(h.pot)
    if i <= numel(lines)
        set(h.pot(i), 'String', lines{i});
    else
        set(h.pot(i), 'String', '');
    end
end
if isempty(lines)
    set(h.pot(1), 'String', 'none detected nearby');
end

% Event log: record state changes and new behaviours.
evt = '';
if act.stateChanged
    evt = sprintf('%5.1fs  -> %s', frame.t, act.state);
end
if ~isempty(evt)
    V.eventLog{end+1} = evt;
end
n = numel(V.eventLog);
for i = 1:numel(h.ev)
    j = n - i + 1;
    if j >= 1
        set(h.ev(i), 'String', V.eventLog{j});
    else
        set(h.ev(i), 'String', '');
    end
end
end

% =========================================================================
function [Vv, Ff, Cc] = gridCells(rg, M, thresh, z)
% Quads for cells of the station x offset table M above thresh.
[I, J] = find(isfinite(M) & M > thresh);
Vv = zeros(0,3);  Ff = zeros(0,4);  Cc = zeros(0,3);
if isempty(I), return; end
s = rg.s(:);  off = rg.offsets(:).';
N = numel(s);
if N >= 2, ds = median(diff(s)); else, ds = 1; end
if numel(off) >= 2, dd = off(2) - off(1); else, dd = 0.25; end
sA = s(I) - ds/2;  sB = s(I) + ds/2;
oA = off(J).' - dd/2;  oB = off(J).' + dd/2;
n = numel(I);
sq = [sA; sB; sB; sA];
oq = [oA; oA; oB; oB];
P = frenetToCartesian(rg.center, max(min(sq, s(end)), s(1)), oq);
Vv = [P, z*ones(4*n,1)];
Ff = [(1:n).', n + (1:n).', 2*n + (1:n).', 3*n + (1:n).'];
val = min(max(M(sub2ind(size(M), I, J)), 0), 1);
Cc = [ones(n,1), 0.85 - 0.75*val, 0.15*ones(n,1)];
end

function W = place(Vb, pos, yaw, dz)
c = cos(yaw);  s = sin(yaw);
W = [Vb(:,1)*c - Vb(:,2)*s + pos(1), Vb(:,1)*s + Vb(:,2)*c + pos(2), Vb(:,3) + dz];
end

function setLine(h, x, y, z)
if isempty(x), x = NaN; y = NaN; z = NaN; end
set(h, 'XData', x(:), 'YData', y(:), 'ZData', z(:));
end

function setPoints(h, P, z)
if isempty(P)
    setLine(h, NaN, NaN, NaN);
else
    setLine(h, P(:,1), P(:,2), z*ones(size(P,1),1));
end
end

function pool = ensureTextPool(V, pool, n, col, sz)
while numel(pool) < n
    pool(end+1) = text(0, 0, 0, '', 'Parent', V.ax, 'Color', col, 'FontSize', sz, ...
                       'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
                       'Interpreter', 'none', 'Visible', 'off'); %#ok<AGROW>
end
end

function [col, label] = stateStyle(state)
switch state
    case 'NORMAL_DRIVING',       col = [0.15 0.55 0.20]; label = 'NORMAL DRIVING';
    case 'HAZARD_ASSESSMENT',    col = [0.75 0.55 0.05]; label = 'HAZARD ASSESSMENT';
    case 'PREDICTIVE_AVOIDANCE', col = [0.85 0.40 0.05]; label = 'PREDICTIVE AVOIDANCE';
    case 'CONSERVATIVE_DRIVING', col = [0.15 0.35 0.75]; label = 'CONSERVATIVE DRIVING';
    case 'SAFE_STOP',            col = [0.80 0.08 0.08]; label = 'SAFE STOP';
    case 'RECOVERY',             col = [0.10 0.55 0.55]; label = 'RECOVERY';
    otherwise,                   col = [0.4 0.4 0.4];    label = state;
end
end

function c = sevColor(sev)
switch sev
    case 'severe',   c = [1.0 0.2 0.2];
    case 'moderate', c = [1.0 0.6 0.1];
    otherwise,       c = [1.0 0.95 0.3];
end
end

function s = clip(s, n)
if numel(s) > n
    s = [s(1:n-3) '...'];
end
end

function s = fmtT(t)
if isfinite(t), s = sprintf('%.1f s', t); else, s = 'none'; end
end

function s = fmtM(m)
if isfinite(m), s = sprintf('%.2f m', m); else, s = '-'; end
end

function tf = flag(s, name)
tf = isstruct(s) && isfield(s, name) && ~isempty(s.(name)) && logical(s.(name));
end

function v = getf(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
