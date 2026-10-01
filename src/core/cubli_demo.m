function cubli_demo()
%CUBLI_DEMO Run several capabilities of the unified plant from ONE built model.
%
% This is the point of build_cubli_clean_cubli: the model is built once and
% each capability is selected by P.run.mode. The .slx is not regenerated
% between runs -- only the workspace variables change.
%
% Add modes to the list below as they come online.

root = cubli_root();
setup_cubli();   % 源码分在四个文件夹，全部上路径

model = build_cubli_clean_cubli();
set_param(model,'SimMechanicsOpenEditorOnUpdate','off');
before = dir(fullfile(root,[model '.slx']));
fprintf('built once: %s (%.0f bytes)\n',before.name,before.bytes);

for m = ["edge_balance","stand_to_edge"]
    P = cubli_mode(m);
    assignin('base','P',P);   % the model reads its IC/ctrl/mode from here
    t0 = tic;
    out = sim(model,'ReturnWorkspaceOutputs','on');
    fprintf('\n===== %s (%.1f s wall) =====\n',m,toc(t0));
    cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
        out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
        out.get('cubli_penetration'),P);
end

after = dir(fullfile(root,[model '.slx']));
fprintf('\n.slx before %.0f bytes / %s\n.slx after  %.0f bytes / %s\n', ...
    before.bytes,before.date,after.bytes,after.date);
fprintf('regenerated? %d  (0 = the file was NOT touched)\n', ...
    ~strcmp(before.date,after.date) || before.bytes ~= after.bytes);
close_system(model,0);
end
