function rows = diag_walk_route_sweep(routes)
%DIAG_WALK_ROUTE_SWEEP Probe direction changes and wheel-speed limits.
setup_cubli();
model = build_cubli_clean_cubli();
if nargin < 1
    routes = ["R","L","U","D","RL","LR","RU","UR","RD","DR", ...
        "UL","LU","UD","DU","RR","LL","UU","DD", ...
        "RUR","URU","RULD","RURD","RRUU", ...
        "RURURURU","RLUDRLUD","RRUULLDD","RULDURDL","RRRRRRRR"];
end
routes = string(routes);
rows = repmat(struct('route',"",'requested',0,'completed',0,'phase',0,'fault',0, ...
    'stepProjection',[],'stepCross',[],'actual',[],'expected',[], ...
    'wheelPeak',[],'penetration',0),1,numel(routes));
fprintf('route     done phase fault    ex     ey    wx    wy    wz    pen\n');
for k = 1:numel(routes)
    route = routes(k);
    P = cubli_mode("walk_route",{'run.route.sequence',route});
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');
    rep = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
        out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
        out.get('cubli_penetration'),P,out.get('cubli_R'),out.get('cubli_wy'), ...
        out.get('cubli_route_state'),out.get('cubli_wx'),out.get('cubli_wz'));
    peak = zeros(1,3);
    for j = 1:3
        name = ["cubli_wx","cubli_wy","cubli_wz"];
        peak(j) = max(abs(out.get(name(j)).Data(:)));
    end
    rows(k).route = route;
    rows(k).requested = numel(char(route));
    rows(k).completed = rep.routeCompleted;
    rows(k).phase = rep.routePhase;
    rows(k).fault = rep.routeFault;
    rows(k).stepProjection = rep.stepProjectionSides;
    rows(k).stepCross = rep.stepCrossSides;
    rows(k).actual = rep.actualXY;
    rows(k).expected = rep.expectedXY;
    rows(k).wheelPeak = peak;
    rows(k).penetration = rep.penetrationMax;
    err = (rep.actualXY-rep.expectedXY)/P.cube.side;
    fprintf('%-8s %2d/%-2d  %2d    %2d  %+5.2f %+5.2f  %5.0f %5.0f %5.0f  %.2f\n', ...
        route,rep.routeCompleted,numel(char(route)),rep.routePhase,rep.routeFault, ...
        err(1),err(2),peak,1000*rep.penetrationMax);
end
end
