function result = diag_point_speed(duration,rollDeg,preset,gain,solver,step,pitchDeg,mode,yawGain)
%DIAG_POINT_SPEED Measure vertex attitude and all three wheel speeds.
%   diag_point_speed(40,3,"fast",0,"ODE3",0.001)
%   diag_point_speed(60,3,"point_near_zero",7e-5,"ODE5",0.000125)

if nargin<1, duration=40; end
if nargin<2, rollDeg=3; end
if nargin<3, preset="fast"; end
if nargin<4, gain=0; end
if nargin<5, solver="ODE3"; end
if nargin<6, step=0.001; end
if nargin<7, pitchDeg=0; end
if nargin<8, mode="point_balance"; end
if nargin<9, yawGain=[]; end
mode = string(mode);
switch mode
    case "point_balance", durationPath = 'run.point.duration';
    case "edge_to_point", durationPath = 'run.edgetopoint.duration';
    case "flat_to_point", durationPath = 'run.flatchain.duration';
    case "flat_to_point_direct", durationPath = 'run.directjump.duration';
    case "stand_to_point", durationPath = 'run.pointstand.duration';
    otherwise, error('CubliClean:PointSpeedMode','Unknown point mode %s.',mode);
end
setup_cubli();
overrides = {durationPath,duration; ...
    'run.point.initialRollDeg',rollDeg; ...
    'run.point.initialPitchDeg',pitchDeg; ...
    'run.balancePreset',preset; ...
    'run.pointLowSpeed.Kw',gain; ...
    'run.pointNearZero.Kw',gain};
if ~isempty(yawGain)
    overrides = [overrides; {'control.point.KyawP',yawGain}];
end
P = cubli_mode(mode,overrides);
model = build_cubli_clean_cubli('Cubli_PointSpeed_Diag');
set_param(model,'SimMechanicsOpenEditorOnUpdate','off');
set_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverChoice',char(solver), ...
    'MultibodyLocalSolverSampleTime',num2str(step,16));
assert(strcmp(get_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverChoice'),char(solver)) && ...
    abs(str2double(get_param([model '/Solver Configuration'], ...
    'MultibodyLocalSolverSampleTime'))-step)<1e-12, ...
    'CubliClean:PointSolverSetting','The contact solver setting did not take.');
assignin('base','P',P);
out = sim(model,'ReturnWorkspaceOutputs','on');

ts = out.get('cubli_com');
t = ts.Time(:).';
com = cubli_log_reshape(ts,3);
rate = localSample(out.get('cubli_rate'),3,t);
wx = localSample(out.get('cubli_wx'),1,t);
wy = localSample(out.get('cubli_wy'),1,t);
wz = localSample(out.get('cubli_wz'),1,t);
rr = localSample(out.get('cubli_R'),9,t);
R = reshape(rr,3,3,[]);
if ismember(mode,["edge_to_point","flat_to_point"])
    n = [-1;1;1]/sqrt(3);
else
    n = [1;1;1]/sqrt(3);
end
u = squeeze(pagemtimes(R,n));
tilt = atan2(hypot(u(1,:),u(2,:)),u(3,:))*180/pi;
speed = sqrt(wx.^2+wy.^2+wz.^2);
tail = t>=max(0,t(end)-min(10,t(end)/2));
pen = max(out.get('cubli_penetration').Data(:));
firstVertex = find(com(3,:)>0.12,1,'first');
if isempty(firstVertex), vertexTime=NaN; else, vertexTime=t(firstVertex); end
result = struct('mode',mode,'duration',t(end), ...
    'rollDeg',rollDeg,'pitchDeg',pitchDeg, ...
    'preset',string(preset),'gain',gain,'solver',string(solver), ...
    'step',step,'wheelEnd',[wx(end),wy(end),wz(end)], ...
    'wheelNormEnd',speed(end),'wheelNormTailMax',max(speed(tail)), ...
    'wheelNormPeak',max(speed), ...
    'wheelTailMean',[mean(wx(tail)),mean(wy(tail)),mean(wz(tail))], ...
    'tiltEndDeg',tilt(end),'tiltPeakDeg',max(tilt), ...
    'tiltTailMaxDeg',max(abs(tilt(tail))), ...
    'rateEnd',rate(:,end).', ...
    'horizontalRateTailMax',max(hypot(rate(1,tail),rate(2,tail))), ...
    'comZEnd',com(3,end),'comZTailMin',min(com(3,tail)), ...
    'vertexTime',vertexTime, ...
    'comXYEnd',com(1:2,end),'comXYDrift',norm(com(1:2,end)-com(1:2,1)), ...
    'penetrationMax',pen);
clear t com rate wx wy wz rr R u tilt speed tail
result.report = cubli_run_report(out.get('cubli_com'),out.get('cubli_rate'), ...
    out.get('cubli_tau_x'),out.get('cubli_tau_y'),out.get('cubli_tau_z'), ...
    out.get('cubli_penetration'),P,out.get('cubli_R'),out.get('cubli_wy'), ...
    out.get('cubli_route_state'),out.get('cubli_wx'),out.get('cubli_wz'));
fprintf('%s / %s @ %.4g s, initial [%.2f %.2f] deg, %s Kw %.3g: ', ...
    mode,solver,step,rollDeg,pitchDeg,preset,gain);
fprintf('w end [%.3f %.3f %.3f], |w| tail max %.3f, peak %.1f, ', ...
    result.wheelEnd,result.wheelNormTailMax,result.wheelNormPeak);
fprintf('tilt end %.3f deg, tilt tail max %.3f deg, ', ...
    result.tiltEndDeg,result.tiltTailMaxDeg);
fprintf('horizontal rate tail max %.3f, ',result.horizontalRateTailMax);
fprintf('yaw rate end %.3f, ',result.rateEnd(3));
fprintf('z end %.5f, XY drift %.1f mm, pen max %.2f mm\n', ...
    result.comZEnd,1000*result.comXYDrift,1000*result.penetrationMax);
fprintf('  first vertex-height crossing %.3f s\n',result.vertexTime);
close_system(model,0);
end

function v = localSample(ts,n,t)
v = cubli_log_reshape(ts,n);
if ~isequal(ts.Time(:),t(:))
    v = interp1(ts.Time(:),v.',t(:)).';
end
end
