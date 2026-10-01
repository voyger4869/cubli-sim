function verify_cubli()
%VERIFY_CUBLI Run every implemented mode and print the acceptance verdict.
%
%     verify_cubli
%
% This is the acceptance gate. Each mode builds the unified plant, runs for its
% own duration with the verified "fast" preset, and is judged by
% cubli_run_report against thresholds taken from cubli_clean_parameters.
%
% READ THE RESULT HONESTLY. Three of these modes are expected to FAIL and are
% documented as failing:
%   walk             -- gait works at the default drive (6.97 faces from 8
%                       commanded) but the face count and penetration checks
%                       are not met, and the count is not robust to the drive.
%   stand_to_point   -- the rotor runs to 3129 rad/s against an 1800 limit.
%   (everything else passes)
% A failure here is information, not a regression, unless it used to pass.
%
% The "fast" preset is used because it is the verified behaviour. The two
% rotor-loop presets change what the answer means and need the contact solver
% at 0.25 ms -- see docs/03 and docs/02.
%
% Runtime: about 3-5 minutes (8 modes, one model build each).

setup_cubli();

modes = ["edge_balance","stand_to_edge","point_balance","edge_to_point", ...
         "flat_to_point","flat_to_point_direct","walk","stand_to_point"];

fprintf('\n%-24s %8s %10s %10s  %s\n','mode','ok','rotor end','|w| peak','failed checks');
fprintf('%s\n',repmat('-',1,78));
nPass = 0;
for m = modes
    P = cubli_mode(m);
    model = build_cubli_clean_cubli();
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');

    wn = localRotorNorm(out);
    rep = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
        out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
        out.get('cubli_penetration'),P,out.get('cubli_R'));

    failed = strjoin(cellstr(rep.failedChecks),', ');
    if isempty(failed), failed = '-'; end
    fprintf('%-24s %8d %10.1f %10.1f  %s\n',m,rep.ok,wn(end),max(wn),failed);
    nPass = nPass + rep.ok;
end

fprintf('%s\n',repmat('-',1,78));
fprintf('%d of %d modes pass. Expected failures: walk, stand_to_point.\n', ...
    nPass,numel(modes));
fprintf(['See docs/03 for why those two fail, and docs/02 for the full evidence\n' ...
         'behind every threshold.\n']);
end

function wn = localRotorNorm(out)
% The THREE-WHEEL norm, not a single channel. Reading only w_y once made a
% 1412 rad/s windup on the X/Z pair look like a success -- see docs/03.
n  = numel(out.get('cubli_com').Time);
g  = @(nm) interp1(out.get(nm).Time(:).', ...
        cubli_log_reshape(out.get(nm),1),linspace( ...
        out.get(nm).Time(1),out.get(nm).Time(end),n));
wn = sqrt(g('cubli_wx').^2 + g('cubli_wy').^2 + g('cubli_wz').^2);
end
