function verify_preset_snapshot(tag)
%VERIFY_PRESET_SNAPSHOT Record every mode's numbers so a later change can be
%differenced instead of eyeballed.
%
%     verify_preset_snapshot("before")     % ...edit something...
%     verify_preset_snapshot("after")
%     verify_preset_compare("before","after")
%
% WHY THIS EXISTS. The "fast" balance preset is the verified behaviour and is
% not allowed to move. "It still looks the same" is not a check: this project
% has more than once concluded a setting worked from a single run, and had a
% whole conclusion invalidated by a state variable that turned out to be dead.
% So the gate is numeric, and verify_preset_compare runs it at tolerance ZERO.
%
% The mode list is fixed and every field is compared by name, so a mode that
% silently stops running shows up as a mismatch rather than as a pass.
%
% Runtime: about 3-5 minutes (8 modes, one model build each).
% Snapshots are written next to this file as verify_preset_<tag>.mat, which the
% .gitignore excludes -- they are build artefacts, not sources.

if nargin < 1 || isempty(tag), tag = "snap"; end
setup_cubli();

modes = ["edge_balance","stand_to_edge","point_balance","walk", ...
         "stand_to_point","edge_to_point","flat_to_point","flat_to_point_direct"];
S = struct();
fprintf('\n%-24s %6s %10s %10s  %s\n','mode','ok','rotorEnd','|w| peak','failed');
fprintf('%s\n',repmat('-',1,78));
for m = modes
    P = cubli_mode(m);
    model = build_cubli_clean_cubli();
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');

    wn  = localRotorNorm(out);
    rep = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
        out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
        out.get('cubli_penetration'),P,out.get('cubli_R'));

    e = struct('tEnd',rep.tEnd,'ok',double(rep.ok), ...
        'failed',strjoin(cellstr(rep.failedChecks),'|'), ...
        'rotorEnd',wn(end),'rotorMax',max(wn), ...
        'penetrationMax',localNum(rep,'penetrationMax',NaN), ...
        'comZEnd',localNum(rep,'comZEnd',NaN), ...
        'comZMin',localNum(rep,'comZMin',NaN), ...
        'settledMin',localNum(rep,'settledMin',NaN), ...
        'tiltEndDeg',localNum(rep,'tiltEndDeg',NaN), ...
        'facesDone',localNum(rep,'facesDone',NaN), ...
        'sideTravel',localNum(rep,'sideTravel',NaN));
    S.(matlab.lang.makeValidName(m)) = e;
    failed = e.failed; if isempty(failed), failed = '-'; end
    fprintf('%-24s %6d %10.1f %10.1f  %s\n',m,rep.ok,wn(end),max(wn),failed);
end

file = fullfile(fileparts(mfilename('fullpath')),sprintf('verify_preset_%s.mat',tag));
save(file,'-struct','S');
fprintf('\nsaved %s\n',file);
end

function v = localNum(s,f,d)
if isfield(s,f), v = s.(f); else, v = d; end
end

function wn = localRotorNorm(out)
% Three-wheel norm. Reading only w_y once made a 1412 rad/s windup on the X/Z
% pair look like a success -- see docs/03.
n  = numel(out.get('cubli_com').Time);
g  = @(nm) interp1(out.get(nm).Time(:).', ...
        cubli_log_reshape(out.get(nm),1),linspace( ...
        out.get(nm).Time(1),out.get(nm).Time(end),n));
wn = sqrt(g('cubli_wx').^2 + g('cubli_wy').^2 + g('cubli_wz').^2);
end
