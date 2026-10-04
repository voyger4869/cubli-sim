function rows = diag_walk_route_margin()
%DIAG_WALK_ROUTE_MARGIN Compare speed budget with route accuracy.
setup_cubli();
model = build_cubli_clean_cubli();
routes = ["RLUD","RULD","RLUDRLUD","RRRRRR","RU"];
margins = [20 24 27 30 35];
rows = repmat(struct('margin',0,'route',"",'completed',0,'fault',0, ...
    'wheelPeak',0,'maxCross',0,'positionError',0),1,numel(routes)*numel(margins));
k = 0;
for margin = margins
    for route = routes
        P = cubli_mode("walk_route", ...
            {'run.route.sequence',route; 'run.route.speedGuardMargin',margin});
        assignin('base','P',P);
        out = sim(model,'ReturnWorkspaceOutputs','on');
        rep = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
            out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
            out.get('cubli_penetration'),P,out.get('cubli_R'),out.get('cubli_wy'), ...
            out.get('cubli_route_state'),out.get('cubli_wx'),out.get('cubli_wz'));
        k = k+1;
        rows(k).margin = margin;
        rows(k).route = route;
        rows(k).completed = rep.routeCompleted;
        rows(k).fault = rep.routeFault;
        rows(k).wheelPeak = max(rep.wheelPeak);
        rows(k).maxCross = max(rep.stepCrossSides);
        rows(k).positionError = rep.positionErrorSides;
        fprintf('MARGIN %2g  %-8s %d/%d fault%d wPeak%.1f cross%.3f endErr%.3f\n', ...
            margin,route,rep.routeCompleted,numel(char(route)),rep.routeFault, ...
            rows(k).wheelPeak,rows(k).maxCross,rows(k).positionError);
    end
end
end
