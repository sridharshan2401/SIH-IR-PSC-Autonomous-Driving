function V = replayDemo(logOrFile, opts)
%REPLAYDEMO Replay a recorded simulation log in the 3D viewer.
%
%   COMPONENT STATUS: REAL (visualisation of a recorded run)
%
%   REPLAYDEMO(log) replays a log returned by RUNDEMO or RUNSCENARIO.
%   REPLAYDEMO('results/run.mat') loads a .mat file containing `log`.
%   V = REPLAYDEMO(..., opts) returns the viewer.
%
%   Replay is for presenting a run again (for example on a slower laptop,
%   or to pause on an interesting moment). Every frame shows exactly what
%   the planner, perception and decision logic produced at that step of the
%   recorded run -- the log stores the trajectory, corridor, risk grid,
%   predictions, tracks, detections and pothole tracks of every step. It is
%   not a re-simulation and computes no new plans.
%
%   Options:
%       .camera      'chase' | 'overview' | 'top'
%       .fromTime    s, start of replay (default 0)
%       .toTime      s, end of replay (default end of log)
%       .renderEvery draw every N logged steps (default 2)
%       .pauseTime   s between frames (default 0.02)
%       .videoFile   record as in RUNDEMO
%       .visible     false for off-screen rendering
%       .snapshotTimes  vector of times at which to save PNG snapshots
%       .snapshotDir    folder for snapshots (default 'results/snapshots')
%
%   Example:
%       [log, M] = runDemo(struct('visible', false));
%       save('results/demo_run.mat', 'log', 'M');
%       replayDemo('results/demo_run.mat', struct('camera', 'overview'))
%
%   Requires: base MATLAB only.
%
%   See also RUNDEMO, UPDATEDEMOVIEWER.

if nargin < 2, opts = struct(); end
if ischar(logOrFile)
    S = load(logOrFile);
    log = S.log;
else
    log = logOrFile;
end

scn = buildScenario(log.scenario, log.seed);
cfg = irpscConfig(log.profile);
V = createDemoViewer(scn, cfg, struct('camera', getOpt(opts, 'camera', 'chase'), ...
        'visible', getOpt(opts, 'visible', true), ...
        'title', sprintf('REPLAY -- %s -- %s', log.planner, scn.sihScenario)));

t0    = getOpt(opts, 'fromTime', 0);
t1    = getOpt(opts, 'toTime', log.t(end));
every = max(1, round(getOpt(opts, 'renderEvery', 2)));
pauseTime = getOpt(opts, 'pauseTime', 0.02);
snapT = getOpt(opts, 'snapshotTimes', []);
snapDir = getOpt(opts, 'snapshotDir', fullfile('results', 'snapshots'));
if ~isempty(snapT) && exist(snapDir, 'dir') ~= 7
    mkdir(snapDir);
end

rec = struct('mode', 'none');
videoFile = getOpt(opts, 'videoFile', '');
if ~isempty(videoFile)
    if exist('VideoWriter', 'class') == 8 || exist('VideoWriter', 'file') == 2
        rec.writer = VideoWriter(videoFile);
        rec.writer.FrameRate = 10;
        open(rec.writer);
        rec.mode = 'video';
    else
        warning('replayDemo:noVideoWriter', 'VideoWriter not available; no video written.');
    end
end

ks = find(log.t >= t0 & log.t <= t1);
snapDone = false(size(snapT));
for n = 1:numel(ks)
    k = ks(n);
    isSnap = false;
    for q = 1:numel(snapT)
        if ~snapDone(q) && log.t(k) >= snapT(q)
            isSnap = true;  snapDone(q) = true;
        end
    end
    if mod(n-1, every) ~= 0 && ~isSnap
        continue;
    end
    if ~ishandle(V.fig), break; end
    ctl = getappdata(V.fig, 'viewerCtl');
    while ctl.paused && ishandle(V.fig) && ~ctl.quit
        pause(0.1);
        ctl = getappdata(V.fig, 'viewerCtl');
    end
    if ctl.quit, break; end

    frame = frameFromLog(log, k, scn, cfg);
    V = updateDemoViewer(V, frame);
    drawnow;
    if strcmp(rec.mode, 'video')
        writeVideo(rec.writer, getframe(V.fig));
    end
    if isSnap
        ctlNow = getappdata(V.fig, 'viewerCtl');
        f = fullfile(snapDir, sprintf('%s_t%05.1f_%s.png', log.scenario, log.t(k), ctlNow.camera));
        print(V.fig, f, '-dpng', '-r90');
        fprintf('snapshot %s\n', f);
    end
    if pauseTime > 0
        pause(pauseTime);
    end
end
if strcmp(rec.mode, 'video')
    close(rec.writer);
end
end

% =========================================================================
function frame = frameFromLog(log, k, scn, cfg)
plan = struct();
plan.corridor      = log.corridorCache{k};
plan.preds         = log.preds{k};
plan.riskGrid      = log.riskGrid{k};
plan.preferredPath = log.preferredPath{k};
plan.potholes      = log.plannerPotholes{k};
plan.behaviour     = log.behaviour{k};
plan.risk          = log.risk(k);
plan.confidence    = log.confidence(k);
plan.ttc           = log.ttc(k);
plan.ttcPlanned    = log.ttcPlanned(k);
plan.status        = log.status{k};
plan.minClearance  = log.minClear(k);
plan.rejectedTraj  = getCell(log, 'rejectedTraj', k);

action = struct('state', log.state{k}, 'reason', log.decisionReason{k}, ...
                'emergencyBrake', log.emergencyBrake(k), ...
                'speedLimit', getNum(log, 'speedLimit', k, cfg.ego.maxSpeed), ...
                'stateChanged', getNum(log, 'stateChanged', k, false));

ego = makeEgoState(log.egoPos(k,:), log.egoHeading(k), log.egoSpeed(k), ...
                   'Steer', log.egoSteer(k), 'Accel', log.egoAccel(k), 'Time', log.t(k));

frame = struct('k', k, 't', log.t(k), 'scn', scn, 'cfg', cfg, 'ego', ego, ...
               'plan', plan, 'action', action, 'traj', log.traj{k}, ...
               'tracks', log.tracks{k}, 'truth', log.obstacles{k}, ...
               'detections', log.detections{k}, 'potholeTracks', log.potholeTracks{k}, ...
               'potholeDets', log.potholeDets{k}, 'steer', log.egoSteer(k), ...
               'accel', log.egoAccel(k), 'collided', log.collided(k), ...
               'replanned', log.replanned(k), 'plannerName', log.planner, 'log', log);
if isempty(frame.detections)
    frame.detections = struct('camera', zeros(0,2), 'lidar', zeros(0,2), 'radar', zeros(0,2));
end
end

function v = getCell(log, name, k)
if isfield(log, name) && numel(log.(name)) >= k
    v = log.(name){k};
else
    v = [];
end
end

function v = getNum(log, name, k, defaultVal)
if isfield(log, name) && numel(log.(name)) >= k
    v = log.(name)(k);
else
    v = defaultVal;
end
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
