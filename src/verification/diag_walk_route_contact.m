function rows = diag_walk_route_contact()
%DIAG_WALK_ROUTE_CONTACT Check whether contact penetration converges by step.
setup_cubli();
P = cubli_mode("walk_route",{'run.route.sequence',"R"});
model = build_cubli_clean_cubli();
assignin('base','P',P);
choices = ["ODE3","ODE5","ODE5","ODE5"];
steps = [0.001,0.0005,0.00025,0.000125];
rows = zeros(numel(steps),4);
for k = 1:numel(steps)
    set_param([model '/Solver Configuration'], ...
        'MultibodyLocalSolverChoice',char(choices(k)), ...
        'MultibodyLocalSolverSampleTime',num2str(steps(k)));
    out = sim(model,'ReturnWorkspaceOutputs','on');
    pen = out.get('cubli_penetration');
    state = cubli_log_reshape(out.get('cubli_route_state'),3);
    wy = cubli_log_reshape(out.get('cubli_wy'),1);
    rows(k,:) = [steps(k),max(pen.Data(:)),state(1,end),max(abs(wy))];
    fprintf('%s @ %.6f s: penetration %.3f mm, completed %.0f, wY peak %.1f\n', ...
        choices(k),steps(k),1000*rows(k,2),rows(k,3),rows(k,4));
end
end
