function [log, M] = runDemo(opts)
%RUNDEMO Main judge demonstration: live closed-loop IR-PSC simulation in 3D.
%
%   COMPONENT STATUS: REAL (entry point) over SIMPLIFIED sensor, vehicle and
%   scenario models.
%
%   THE ONE COMMAND TO RUN:
%
%       >> setupPaths
%       >> runDemo
%
%   runDemo runs the full closed loop (RUNSCENARIO) on the primary demo
%   scenario and draws every simulation step in the 3D viewer as it
%   happens. The vehicle's motion comes from the controller and the
%   kinematic bicycle model; the trajectory you see is the IR-PSC planner's
%   output at that step. Nothing is precomputed: the viewer is called from
%   inside the loop through RUNSCENARIO's opts.onStep hook.
%
%   [log, M] = RUNDEMO(opts) returns the simulation log and metrics.
%
%   Options (all optional):
%       .scenario     'demo' (default) | 'village' | 'urban' | 'highway' |
%                     'market' | 'cattle'
%       .planner      'irpsc' (default) | 'baseline'
%       .seed         random seed (default 1)
%       .camera       'chase' (default) | 'overview' | 'top'
%       .renderEvery  draw every N simulation steps (default 2 = 10 Hz)
%       .realTime     true to slow the display to simulated time (default false)
%       .videoFile    e.g. 'results/demo.mp4' to record (default: no video).
%                     Uses VideoWriter (base MATLAB). If VideoWriter is not
%                     available (e.g. GNU Octave) frames are written as PNG
%                     files into a folder of that name instead.
%       .maxTime      stop after this many simulated seconds
%       .usePerfectPerception  bypass simulated sensors (diagnostic)
%       .visible      false to render off-screen (default true)
%
%   Keyboard, once the figure has focus: 1 chase camera, 2 overview,
%   3 top-down, space pause/resume, q stop.
%
%   Examples:
%       runDemo                                            % the demo
%       runDemo(struct('camera', 'overview'))
%       runDemo(struct('videoFile', 'results/irpsc_demo.mp4'))
%       runDemo(struct('scenario', 'market', 'planner', 'baseline'))
%
%   Requires: base MATLAB only (VideoWriter is part of base MATLAB).
%
%   See also RUNSCENARIO, CREATEDEMOVIEWER, UPDATEDEMOVIEWER, REPLAYDEMO.

if nargin < 1, opts = struct(); end
scenario  = getOpt(opts, 'scenario', 'demo');
planner   = lower(getOpt(opts, 'planner', 'irpsc'));
seed      = getOpt(opts, 'seed', 1);
every     = max(1, round(getOpt(opts, 'renderEvery', 2)));
realTime  = getOpt(opts, 'realTime', false);
videoFile = getOpt(opts, 'videoFile', '');

switch planner
    case {'irpsc', 'ir-psc'}
        plannerFn = @irpscPlanner;  plannerName = 'IR-PSC';
    case {'baseline', 'fixed'}
        plannerFn = @baselinePlanner;  plannerName = 'Baseline (fixed candidates)';
    otherwise
        error('runDemo:unknownPlanner', 'Unknown planner "%s". Use irpsc or baseline.', planner);
end

scn = buildScenario(scenario, seed);
cfg = irpscConfig(scn.profile);

V = createDemoViewer(scn, cfg, struct('camera', getOpt(opts, 'camera', 'chase'), ...
        'visible', getOpt(opts, 'visible', true), ...
        'title', sprintf('%s -- %s', plannerName, scn.sihScenario)));
fig = V.fig;

% Viewer, recorder and pacing state live in the figure, so the per-step
% callback is an ordinary sub-function (portable; no nested-function state).
st = struct('V', V, 'rec', startRecorder(videoFile), 'every', every, ...
            'realTime', realTime, 'wallStart', tic, 'plannerName', plannerName);
setappdata(fig, 'demoState', st);

runOpts = struct('seed', seed, 'onStep', @(frame) demoStep(fig, frame), ...
                 'usePerfectPerception', getOpt(opts, 'usePerfectPerception', false));
if isfield(opts, 'maxTime'), runOpts.maxTime = opts.maxTime; end

fprintf('Running %s on "%s" (seed %d). Close the window or press q to stop.\n', ...
        plannerName, scn.sihScenario, seed);
[log, M] = runScenario(scenario, plannerFn, cfg, runOpts);

if ishandle(fig)
    st = getappdata(fig, 'demoState');
    stopRecorder(st.rec);
end
printSummary(log, M, plannerName, scn);
end

% =========================================================================
function keepGoing = demoStep(fig, frame)
%DEMOSTEP Called by runScenario after every simulation step.
keepGoing = true;
if ~ishandle(fig)
    keepGoing = false;              % window closed
    return;
end
ctl = getappdata(fig, 'viewerCtl');
while ctl.paused && ishandle(fig) && ~ctl.quit
    pause(0.1);
    ctl = getappdata(fig, 'viewerCtl');
end
if ~ishandle(fig) || ctl.quit
    keepGoing = false;
    return;
end
st = getappdata(fig, 'demoState');
if mod(frame.k, st.every) ~= 0 && frame.k ~= 1
    return;
end
frame.replanned   = frame.log.replanned(frame.k);
frame.plannerName = st.plannerName;
st.V   = updateDemoViewer(st.V, frame);
drawnow;
st.rec = writeFrame(st.rec, fig);
if st.realTime
    lag = frame.t - toc(st.wallStart);
    if lag > 0, pause(lag); end
end
setappdata(fig, 'demoState', st);
end

% =========================================================================
function rec = startRecorder(videoFile)
rec = struct('mode', 'none', 'writer', [], 'folder', '', 'n', 0);
if isempty(videoFile), return; end
outDir = fileparts(videoFile);
if ~isempty(outDir) && exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end
if exist('VideoWriter', 'class') == 8 || exist('VideoWriter', 'file') == 2
    try
        w = VideoWriter(videoFile, 'MPEG-4');
    catch
        w = VideoWriter(videoFile);        % e.g. Motion JPEG AVI where MP4 is unsupported
    end
    w.FrameRate = 10;
    open(w);
    rec.mode = 'video';
    rec.writer = w;
    fprintf('Recording video to %s\n', videoFile);
else
    [p, n] = fileparts(videoFile);
    rec.folder = fullfile(p, [n '_frames']);
    if exist(rec.folder, 'dir') ~= 7, mkdir(rec.folder); end
    rec.mode = 'png';
    fprintf('VideoWriter not available; saving PNG frames to %s\n', rec.folder);
end
end

function rec = writeFrame(rec, fig)
switch rec.mode
    case 'video'
        writeVideo(rec.writer, getframe(fig));
        rec.n = rec.n + 1;
    case 'png'
        rec.n = rec.n + 1;
        print(fig, fullfile(rec.folder, sprintf('frame_%05d.png', rec.n)), '-dpng', '-r80');
end
end

function stopRecorder(rec)
if strcmp(rec.mode, 'video')
    close(rec.writer);
    fprintf('Video closed (%d frames).\n', rec.n);
elseif strcmp(rec.mode, 'png')
    fprintf('Saved %d PNG frames.\n', rec.n);
end
end

function printSummary(log, M, plannerName, scn)
fprintf('\n=====================================================================\n');
fprintf('  %s on %s\n', plannerName, scn.sihScenario);
fprintf('=====================================================================\n');
fprintf('  goal reached        : %d\n', log.goalReached);
fprintf('  simulated time      : %.1f s\n', log.t(end));
fprintf('  collisions          : %d (static contacts %d steps, dynamic %d steps)\n', ...
        M.collisionCount, sum(log.collidedStatic), sum(log.collidedDynamic));
fprintf('  worst clearance     : %.2f m\n', M.minClearance);
fprintf('  average speed       : %.2f m/s\n', M.averageSpeed);
fprintf('  safe-stop episodes  : %d\n', M.emergencyStops);
fprintf('  emergency braking   : %d steps\n', sum(log.emergencyBrake));
fprintf('  pothole wheel entries: %d\n', numel(log.potholeEvents));
for e = 1:numel(log.potholeEvents)
    ev = log.potholeEvents(e);
    fprintf('      t=%5.1f s pothole %d (%s) at %.1f m/s\n', ev.t, ev.id, ev.severity, ev.speed);
end
fprintf('---------------------------------------------------------------------\n');
fprintf('  Simulation result under SIMPLIFIED sensor, vehicle and scenario\n');
fprintf('  models. Not real-world safety evidence. Not safety certified.\n');
fprintf('=====================================================================\n\n');
end

function v = getOpt(s, name, defaultVal)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    v = s.(name);
else
    v = defaultVal;
end
end
