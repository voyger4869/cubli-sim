function verify_preset_compare(tagA,tagB,tol)
%VERIFY_PRESET_COMPARE Difference two verify_preset_snapshot files, field by
%field, and report the worst absolute difference.
%
%     verify_preset_compare("before","after")        % tol default 0
%     verify_preset_compare("before","after",1e-9)
%
% The default tolerance is ZERO, i.e. bit-exact. That is the right default for
% this project: the verified preset is supposed to be untouched by edits that
% do not mean to touch it, and every legitimate change to it has so far been
% bit-exact after all (five consecutive snapshots compared at tol 0 with a
% worst difference of exactly 0).
%
% A NaN on one side only counts as a difference, not as agreement.

if nargin < 3 || isempty(tol), tol = 0; end
here = fileparts(mfilename('fullpath'));
A = load(fullfile(here,sprintf('verify_preset_%s.mat',tagA)));
B = load(fullfile(here,sprintf('verify_preset_%s.mat',tagB)));

numeric = {'tEnd','ok','rotorEnd','rotorMax','penetrationMax','comZEnd', ...
           'comZMin','settledMin','tiltEndDeg','facesDone','sideTravel'};
worst = 0; nBad = 0;
for f = fieldnames(A).'
    a = A.(f{1}); b = B.(f{1});
    if ~strcmp(a.failed,b.failed)
        fprintf('%-24s FAILED CHECKS CHANGED: [%s] -> [%s]\n',f{1},a.failed,b.failed);
        nBad = nBad + 1;
    end
    for g = numeric
        d = abs(a.(g{1}) - b.(g{1}));
        if any(isnan(d)), d = Inf; end      % NaN vs a number IS a difference
        if max(d) > tol
            fprintf('%-24s %-16s %14.6g -> %14.6g  (|diff| %.3g)\n', ...
                f{1},g{1},a.(g{1}),b.(g{1}),max(d));
            nBad = nBad + 1;
        end
        worst = max(worst,max(d));
    end
end

fprintf('\nworst |diff| = %.6g over %d mismatched fields (tol %.3g)\n',worst,nBad,tol);
if nBad == 0
    fprintf('PASS: the two snapshots agree.\n');
else
    fprintf('FAIL: the verified preset moved.\n');
end
end
