function verifySize(tc, varargin) %#ok<INUSL>
%VERIFYSIZE Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
if ~isequal(size(varargin{1}), varargin{2}), fail(varargin, 3, sprintf('verifySize: got %s', mat2str(size(varargin{1})))); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
