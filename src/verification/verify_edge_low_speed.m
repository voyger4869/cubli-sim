function result = verify_edge_low_speed(mode,duration)
%VERIFY_EDGE_LOW_SPEED Reproduce the edge low-speed preset and its actual gate.
%   verify_edge_low_speed("stand_to_edge",30)  % quick release check
%   verify_edge_low_speed("stand_to_edge",300) % long run, ODE5 @ 0.125 ms
%   verify_edge_low_speed("edge_balance",120)

if nargin<1, mode="stand_to_edge"; end
if nargin<2, duration=30; end
mode = string(mode);
assert(ismember(mode,["stand_to_edge","edge_balance"]));
setup_cubli();
P = cubli_mode(mode,{'run.balancePreset','edge_low_speed'; ...
    'run.standup.duration',duration; 'run.edge.duration',duration});
model = build_cubli_clean_cubli('Cubli_EdgeLowSpeed_Verify');
set_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverChoice','ODE5', ...
    'MultibodyLocalSolverSampleTime','0.000125');
assert(strcmp(get_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverChoice'),'ODE5'));
assert(strcmp(get_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverSampleTime'),'0.000125'));
assignin('base','P',P);
out = sim(model,'ReturnWorkspaceOutputs','on');

cts = out.get('cubli_com');
t = cts.Time(:).';
c = cubli_log_reshape(cts,3);
rr = localSample(out.get('cubli_R'),9,t);
rate = localSample(out.get('cubli_rate'),3,t);
wx = localSample(out.get('cubli_wx'),1,t);
wy = localSample(out.get('cubli_wy'),1,t);
wz = localSample(out.get('cubli_wz'),1,t);
R = reshape(rr,3,3,[]);
ux = (squeeze(R(1,3,:)).'-squeeze(R(1,1,:)).')/sqrt(2);
uy = (squeeze(R(2,3,:)).'-squeeze(R(2,1,:)).')/sqrt(2);
uz = (squeeze(R(3,3,:)).'-squeeze(R(3,1,:)).')/sqrt(2);
ex = squeeze(R(1,2,:)).'; ey = squeeze(R(2,2,:)).';
ez = squeeze(R(3,2,:)).';
en = hypot(ex,ey);
edgeTilt = atan2((ux.*ey-uy.*ex)./en,uz)*180/pi;
edgeRate = rate(1,:).*ex+rate(2,:).*ey+rate(3,:).*ez;
edgeYaw = atan2(-ex,ey)*180/pi;
edgeGate = (P.geometry.edgeHeight+P.cube.side/2)/2;
stable = c(3,:)>edgeGate & abs(edgeTilt)<2 & abs(edgeRate)<0.2;
holdSamples = max(1,ceil(0.2/median(diff(t))));
dwell = conv(double(stable),ones(1,holdSamples),'valid');
idx = find(dwell==holdSamples,1);
if isempty(idx), captureTime=NaN; else, captureTime=t(idx+holdSamples-1); end
tail = t>=max(0,t(end)-min(10,t(end)/2));
last = t>=max(0,t(end)-2);
wheelNorm = sqrt(wx.^2+wy.^2+wz.^2);

result = struct();
result.mode = mode;
result.duration = t(end);
result.captureTime = captureTime;
result.edgeTiltEndDeg = edgeTilt(end);
result.edgeTiltTailPeakDeg = max(abs(edgeTilt(last)));
result.edgeYawEndDeg = edgeYaw(end);
result.wheelYEnd = wy(end);
result.wheelYTailMax = max(abs(wy(tail)));
result.wheelNormTailMax = max(wheelNorm(tail));
result.stayedOnEdge = all(c(3,last)>edgeGate);
result.ok = result.stayedOnEdge && isfinite(captureTime) && ...
    (mode=="edge_balance" || captureTime<=3) && ...
    result.edgeTiltTailPeakDeg<=2 && ...
    result.wheelYTailMax<=P.run.edgeLowSpeed.wheelTailMax && ...
    result.wheelNormTailMax<=P.run.edgeLowSpeed.wheelTailMax;
fprintf(['%s / ODE5 @ 0.125 ms / %.0f s: capture %.3f s, ' ...
    'Y end %+.2f rad/s, Y tail max %.2f, 3-wheel tail max %.2f, ' ...
    'edge tilt end %+.4f deg, yaw end %+.2f deg, ok=%d\n'], ...
    mode,result.duration,result.captureTime,result.wheelYEnd, ...
    result.wheelYTailMax,result.wheelNormTailMax, ...
    result.edgeTiltEndDeg,result.edgeYawEndDeg,result.ok);
close_system(model,0);
assert(result.ok,'CubliClean:EdgeLowSpeedVerification', ...
    'The edge low-speed verification failed.');
end

function v = localSample(ts,n,t)
x = cubli_log_reshape(ts,n);
v = interp1(ts.Time(:),x.',t(:)).';
end
