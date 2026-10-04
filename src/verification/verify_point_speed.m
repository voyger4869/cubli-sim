function result = verify_point_speed(preset,duration,rollDeg,pitchDeg)
%VERIFY_POINT_SPEED Check experimental vertex wheel-speed presets.
%   verify_point_speed("point_near_zero",120,3,0)
%   verify_point_speed("point_low_speed",60,3,0)

if nargin<1, preset="point_near_zero"; end
if nargin<2, duration=120; end
if nargin<3, rollDeg=3; end
if nargin<4, pitchDeg=0; end
preset = string(preset);
setup_cubli();
assert(ismember(preset,["point_low_speed","point_near_zero"]), ...
    'CubliClean:PointSpeedPreset','Unknown point speed preset.');
assert(duration>=30,'CubliClean:PointSpeedDuration', ...
    'Use at least 30 s to check the settled wheel-speed tail.');
P = cubli_mode("point_balance",{'run.balancePreset',preset});
if preset == "point_low_speed"
    gain = P.run.pointLowSpeed.Kw;
else
    gain = P.run.pointNearZero.Kw;
end
assert(numel(P.run.ctrl)==58 && P.run.pointSpeed.ctrl==gain);
result = diag_point_speed(duration,rollDeg,preset,gain,"ODE5",0.000125,pitchDeg);
assert(result.report.ok,'CubliClean:PointSpeedVerification', ...
    'Vertex speed candidate failed: %s', ...
    strjoin(cellstr(result.report.failedChecks),', '));
end
