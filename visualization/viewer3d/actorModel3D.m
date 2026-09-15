function model = actorModel3D(className, len, wid, isEgo)
%ACTORMODEL3D Simple low-polygon 3D model for a road user (display only).
%
%   COMPONENT STATUS: REAL (visualisation geometry; carries no simulation state)
%
%   model = ACTORMODEL3D(className, len, wid) returns a model in the body
%   frame (x forward, y left, z up, origin at the body centre on the
%   ground), built from boxes and prisms, sized to the object's length and
%   width. model = ACTORMODEL3D(..., true) builds the ego vehicle, which
%   uses a distinct colour scheme.
%
%   The models exist to make a scene READABLE for a judge -- a bus looks
%   like a bus, a cow like an animal -- not to be realistic. Every model is
%   placed each frame at the pose the simulation reports; nothing here moves
%   anything.
%
%   Outputs:
%       model - struct with fields
%           .V      Kx3 vertices (body frame)
%           .F      Fx4 faces (quads; triangles repeat a vertex)
%           .C      Fx3 face colours (RGB)
%           .height scalar, top of the model (m)
%
%   Requires: base MATLAB only.
%
%   See also CREATEDEMOVIEWER, UPDATEDEMOVIEWER.

if nargin < 4, isEgo = false; end
V = zeros(0,3);  F = zeros(0,4);  C = zeros(0,3);

switch className
    case 'ego'
        body  = [0.93 0.94 0.96];  cabin = [0.10 0.45 0.85];
        [V,F,C] = addBox(V,F,C, [0 0 0.50], [len, wid, 0.70], body);
        [V,F,C] = addBox(V,F,C, [-0.15*len 0 1.15], [0.52*len, 0.88*wid, 0.60], cabin);
        [V,F,C] = addBox(V,F,C, [0.49*len 0 0.45], [0.04, 0.9*wid, 0.25], [1 0.95 0.6]); % lights
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 1.45;
    case {'car', 'unknown'}
        body = [0.80 0.20 0.20];  cabin = [0.25 0.25 0.30];
        if strcmp(className, 'unknown'), body = [0.6 0.6 0.6]; end
        [V,F,C] = addBox(V,F,C, [0 0 0.50], [len, wid, 0.70], body);
        [V,F,C] = addBox(V,F,C, [-0.1*len 0 1.12], [0.5*len, 0.86*wid, 0.55], cabin);
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 1.4;
    case 'bus'
        [V,F,C] = addBox(V,F,C, [0 0 1.70], [len, wid, 2.70], [0.95 0.55 0.10]);
        [V,F,C] = addBox(V,F,C, [0 0 2.10], [len*1.001, wid*1.01, 0.8], [0.20 0.30 0.40]); % windows
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 3.1;
    case 'truck'
        [V,F,C] = addBox(V,F,C, [-0.12*len 0 1.75], [0.72*len, wid, 2.5], [0.20 0.45 0.25]);
        [V,F,C] = addBox(V,F,C, [0.38*len 0 1.35], [0.24*len, wid, 1.9], [0.85 0.75 0.15]);
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 3.0;
    case 'autorickshaw'
        [V,F,C] = addBox(V,F,C, [0 0 0.75], [len, wid, 1.0], [0.15 0.55 0.20]);
        [V,F,C] = addBox(V,F,C, [-0.05*len 0 1.60], [0.9*len, wid, 0.12], [0.95 0.85 0.15]); % canopy
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 1.7;
    case 'motorcycle'
        [V,F,C] = addBox(V,F,C, [0 0 0.55], [len, 0.35, 0.45], [0.15 0.15 0.15]);
        [V,F,C] = addBox(V,F,C, [-0.1*len 0 1.25], [0.35, 0.40, 0.75], [0.90 0.30 0.10]); % rider
        [V,F,C] = addBox(V,F,C, [-0.1*len 0 1.75], [0.25, 0.25, 0.25], [0.95 0.80 0.60]);
        h = 1.9;
    case 'bicycle'
        [V,F,C] = addBox(V,F,C, [0 0 0.50], [len, 0.08, 0.60], [0.10 0.35 0.75]);
        [V,F,C] = addBox(V,F,C, [-0.05*len 0 1.20], [0.30, 0.35, 0.70], [0.20 0.65 0.30]);
        [V,F,C] = addBox(V,F,C, [-0.05*len 0 1.68], [0.22, 0.22, 0.24], [0.95 0.80 0.60]);
        h = 1.8;
    case 'pedestrian'
        [V,F,C] = addBox(V,F,C, [0 0 0.45], [0.25, 0.35, 0.90], [0.20 0.20 0.45]);
        [V,F,C] = addBox(V,F,C, [0 0 1.20], [0.28, 0.45, 0.60], [0.85 0.25 0.45]);
        [V,F,C] = addBox(V,F,C, [0 0 1.62], [0.22, 0.22, 0.24], [0.95 0.78 0.60]);
        h = 1.75;
    case 'animal'
        coat = [0.92 0.90 0.85];
        [V,F,C] = addBox(V,F,C, [0 0 1.00], [0.70*len, wid, 0.70], coat);                 % body
        [V,F,C] = addBox(V,F,C, [0.45*len 0 1.25], [0.28*len, 0.55*wid, 0.45], coat);      % head
        [V,F,C] = addBox(V,F,C, [0.50*len 0 1.52], [0.05, 0.75*wid, 0.08], [0.5 0.4 0.3]); % horns
        for sx = [-0.25 0.25]
            for sy = [-0.3 0.3]
                [V,F,C] = addBox(V,F,C, [sx*len sy*wid 0.33], [0.12, 0.12, 0.66], coat*0.8);
            end
        end
        h = 1.6;
    case 'pushcart'
        [V,F,C] = addBox(V,F,C, [0 0 0.85], [len, wid, 0.25], [0.55 0.35 0.15]);
        [V,F,C] = addBox(V,F,C, [0 0 1.05], [0.8*len, 0.8*wid, 0.25], [0.30 0.65 0.20]); % produce
        [V,F,C] = addWheels(V,F,C, len, wid);
        h = 1.3;
    otherwise
        [V,F,C] = addBox(V,F,C, [0 0 0.6], [len, wid, 1.2], [0.6 0.6 0.6]);
        h = 1.2;
end

if isEgo && ~strcmp(className, 'ego')
    C = 0.5 * C + 0.5 * repmat([0.2 0.6 1.0], size(C,1), 1);
end

model.V = V;  model.F = F;  model.C = C;  model.height = h;
end

% -------------------------------------------------------------------------
function [V,F,C] = addBox(V,F,C, c, sz, col)
hx = sz(1)/2;  hy = sz(2)/2;  hz = sz(3)/2;
v = [-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz; ...
     -hx -hy  hz; hx -hy  hz; hx hy  hz; -hx hy  hz] + repmat(c, 8, 1);
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
F = [F; f + size(V,1)];
V = [V; v];
shade = [0.75; 1.0; 0.85; 0.95; 0.85; 0.95];     % crude face shading
C = [C; repmat(col, 6, 1) .* repmat(shade, 1, 3)];
end

function [V,F,C] = addWheels(V,F,C, len, wid)
r = 0.33;
for sx = [-0.32 0.32]
    for sy = [-0.5 0.5]
        [V,F,C] = addBox(V,F,C, [sx*len, sy*wid, r], [2*r, 0.22, 2*r], [0.08 0.08 0.08]);
    end
end
end
