function verifyTrue(tc, varargin) %#ok<INUSL>
%VERIFYTRUE Octave compatibility shim for matlab.unittest verification.
%   OCTAVE ONLY. Throws 'octaveShim:verifyFailed' on failure.
cond = varargin{1}; if ~(islogical(cond) || isnumeric(cond)) || isempty(cond) || ~all(cond(:)), fail(varargin, 2, 'verifyTrue'); end
end

function fail(args, msgIdx, what)
msg = '';
if numel(args) >= msgIdx && ischar(args{msgIdx})
    msg = args{msgIdx};
end
error('octaveShim:verifyFailed', '%s failed. %s', what, msg);
end
