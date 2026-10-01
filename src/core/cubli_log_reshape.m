function D = cubli_log_reshape(ts,n)
%CUBLI_LOG_RESHAPE Normalise a To Workspace timeseries to n x N.
%
% To Workspace stores an n-component physical signal as n x 1 x N, and MATLAB
% reshape is column-major, so the tempting `reshape(ts.Data, N, [])` interleaves
% the components and silently returns nonsense -- every row becomes a repeat of
% the n components instead of one sample. That bug produced two false findings
% on this project (a "COM cycling between axes" and a "contact chatter" that
% did not exist). Always go through this function.
%
% D is n x N; use D(k,:) for component k over time.

N = numel(ts.Time);
D = reshape(ts.Data,n,N);
assert(size(D,1) == n,'CubliClean:UnexpectedLogShape', ...
    'Log has %d components, expected %d.',numel(ts.Data)/N,n);
end
