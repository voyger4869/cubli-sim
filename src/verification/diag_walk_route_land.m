function rows = diag_walk_route_land()
%DIAG_WALK_ROUTE_LAND Check whether earlier braking reduces landing impact.
setup_cubli();
model = build_cubli_clean_cubli();
routes = ["R","RLUD"];
lands = [70 75 80 84 88 92];
rows = zeros(numel(routes)*numel(lands),6);
k = 0;
for route = routes
    for land = lands
        P = cubli_mode("walk_route", ...
            {'run.route.sequence',route; 'run.walk.landDeg',land});
        assignin('base','P',P);
        out = sim(model,'ReturnWorkspaceOutputs','on');
        state = cubli_log_reshape(out.get('cubli_route_state'),3);
        pen = out.get('cubli_penetration');
        c = cubli_log_reshape(out.get('cubli_com'),3);
        k = k+1;
        rows(k,:) = [numel(char(route)),land,state(1,end),state(3,end), ...
            max(pen.Data(:)),norm(c(1:2,end)-c(1:2,1))];
        fprintf('%-4s land%2g: %d/%d fault%d penetration%.3f mm\n', ...
            route,land,rows(k,3),rows(k,1),rows(k,4),1000*rows(k,5));
    end
end
end
