function y = smoothSeries(x, halfWin, passes)
%SMOOTHSERIES Moving-average smoothing of a scalar series (shrinking window).
%
%   COMPONENT STATUS: REAL
%
%   y = SMOOTHSERIES(x, halfWin, passes) smooths the column vector x. Used
%   for lateral-offset profiles and speed profiles, where SMOOTHPATH would
%   be the wrong shape.
%
%   Inputs:
%       x       - Nx1 numeric vector
%       halfWin - non-negative integer half-window (0 = no smoothing)
%       passes  - positive integer number of passes (default 1)
%
%   Outputs:
%       y - Nx1 smoothed vector
%
%   Requires: base MATLAB only.
%
%   See also SMOOTHPATH.

if nargin < 3 || isempty(passes), passes = 1; end

validateattributes(x, {'numeric'}, {'vector','finite','real'}, mfilename, 'x');
validateattributes(halfWin, {'numeric'}, {'scalar','integer','nonnegative'}, ...
                   mfilename, 'halfWin');

wasRow = isrow(x);
y = x(:);
N = numel(y);
if halfWin == 0 || N < 3
    if wasRow, y = y.'; end
    return;
end

for p = 1:passes
    s = y;
    for i = 1:N
        lo = max(1, i - halfWin);
        hi = min(N, i + halfWin);
        s(i) = mean(y(lo:hi));
    end
    y = s;
end

if wasRow, y = y.'; end
end
