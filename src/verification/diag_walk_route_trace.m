function diag_walk_route_trace(route)
%DIAG_WALK_ROUTE_TRACE Export synchronized numeric logs for a route failure.
if nargin < 1, route = "RULD"; end
setup_cubli();
P = cubli_mode("walk_route",{'run.route.sequence',string(route)});
model = build_cubli_clean_cubli();
assignin('base','P',P);
out = sim(model,'ReturnWorkspaceOutputs','on');
signals = {'cubli_com','cubli_rate','cubli_R','cubli_route_state', ...
    'cubli_wx','cubli_wy','cubli_wz','cubli_tau_x','cubli_tau_y','cubli_tau_z', ...
    'cubli_penetration'};
log = struct();
for k = 1:numel(signals)
    ts = out.get(signals{k});
    log.(signals{k}).t = ts.Time(:).';
    log.(signals{k}).x = cubli_log_reshape(ts,numel(ts.Data)/numel(ts.Time));
end
save('diag_walk_route_trace.mat','log','P');
end
