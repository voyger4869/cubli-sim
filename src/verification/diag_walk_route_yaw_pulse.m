function rows = diag_walk_route_yaw_pulse()
%DIAG_WALK_ROUTE_YAW_PULSE Explore a bounded spin-and-brake flat yaw action.
setup_cubli();
P = cubli_mode("walk_route",{'run.route.sequence',"R"});
P.run.modeCode = 0;
P.run.duration = 4;
model = build_cubli_clean_cubli();
delete_line(model,'Torque Demux/3','Torque Z Hold/1');
add_block('simulink/Sources/From Workspace',[model '/Yaw Pulse'], ...
    'VariableName','yawPulse','Position',[1000 590 1100 620]);
add_line(model,'Yaw Pulse/1','Torque Z Hold/1','autorouting','on');
torques = [0.10 0.15 0.20];
rows = zeros(numel(torques),5);
for k = 1:numel(torques)
    tq = torques(k);
    rise = 1.5*0.1/tq;
    t = [0 rise rise+0.001 2*rise 2*rise+0.001 4];
    u = [tq tq -tq -tq 0 0];
    yawPulse = timeseries(u,t); %#ok<NASGU>
    assignin('base','yawPulse',yawPulse);
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');
    R = cubli_log_reshape(out.get('cubli_R'),9);
    yaw = atan2(R(2,:),R(1,:))*180/pi;
    wz = cubli_log_reshape(out.get('cubli_wz'),1);
    rows(k,:) = [tq,yaw(end),max(abs(yaw)),max(abs(wz)),wz(end)];
    fprintf('pulse %.2f N*m: yaw end %.2f deg, peak %.2f, wheel peak %.1f, wheel end %.1f\n', ...
        rows(k,:));
end
end
