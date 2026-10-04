function results = verify_walk_route(routes)
%VERIFY_WALK_ROUTE Exercise the requested mixed and repeated route examples.
% The controller logs [completedSteps; phase; fault] as cubli_route_state.
% A simulation completing is not a pass: inspect each report.ok and failedChecks.

setup_cubli();
model = build_cubli_clean_cubli();
if nargin < 1, routes = ["RLUD","RRRRRR","RU","RULD","RLUDRLUD"]; end
routes = string(routes);
results = repmat(struct('route',"",'report',[]),1,numel(routes));
for k = 1:numel(routes)
    P = cubli_mode("walk_route",{'run.route.sequence',routes(k)});
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');
    report = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
        out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
        out.get('cubli_penetration'),P,out.get('cubli_R'),out.get('cubli_wy'), ...
        out.get('cubli_route_state'),out.get('cubli_wx'),out.get('cubli_wz'));
    results(k).route = routes(k);
    results(k).report = report;
    assert(report.completedOk && report.faultOk && report.stepsOk && ...
        report.crossTrackOk && ...
        report.positionOk && report.settledOk && report.wheelSpeedOk, ...
        'CubliClean:RouteMotion','Route %s failed a motion or landing check.', ...
        routes(k));
end
end
