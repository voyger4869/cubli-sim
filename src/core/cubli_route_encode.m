function codes = cubli_route_encode(sequence,maxSteps)
%CUBLI_ROUTE_ENCODE Validate R/L/U/D and return fixed-size numeric commands.
% The letters refer to the fixed world ground plane, independent of cube pose.
% Codes: R=1 (+X), L=2 (-X), U=3 (+Y), D=4 (-Y).

if nargin < 2, maxSteps = 64; end
assert(isnumeric(maxSteps) && isscalar(maxSteps) && ...
    isfinite(maxSteps) && maxSteps >= 1 && fix(maxSteps) == maxSteps, ...
    'CubliClean:RouteCapacity','maxSteps must be a positive integer.');
assert((isstring(sequence) && isscalar(sequence)) || ...
    (ischar(sequence) && isrow(sequence)), ...
    'CubliClean:RouteType','Route must be one string or a character row.');
route = upper(char(sequence));
assert(~isempty(route) && numel(route) <= maxSteps, ...
    'CubliClean:RouteLength','Route must contain 1 to %d commands.',maxSteps);
assert(all(ismember(route,'RLUD')), ...
    'CubliClean:RouteCharacter','Route may contain only R, L, U and D.');

codes = zeros(maxSteps,1);
for k = 1:numel(route)
    switch route(k)
        case 'R', codes(k) = 1;
        case 'L', codes(k) = 2;
        case 'U', codes(k) = 3;
        case 'D', codes(k) = 4;
    end
end
end
