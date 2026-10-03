function P = cubli_refresh_ctrl(P)
%CUBLI_REFRESH_CTRL Rebuild P.run.ctrl from the individual control fields.
%
% The unified model reads its gains and manoeuvre constants from the workspace
% expression P.run.ctrl, so they can be changed without regenerating the .slx.
% That vector is DERIVED: after editing any of the fields below, call this
% before re-running, or the model keeps using the old numbers.
%
%   P = cubli_clean_parameters();
%   P.run.edge.Kw = 5e-4;
%   P = cubli_refresh_ctrl(P);
%   assignin('base','P',P); sim('Cubli_Clean_Cubli');

% Direction -> (wheel, sign). Derived HERE so that cubli_mode's overrides of
% P.run.walk.dir take effect: overrides are applied after
% cubli_clean_parameters has already run, and cubli_refresh_ctrl is called last.
%   rotate about world Y -> travel along +-x, driven by Wheel Y
%   rotate about world X -> travel along +-y, driven by Wheel X
% (At the flat attitude wheel Y's axis lies along world Y and wheel X's along
% world X; wheel Z's is vertical and only yaws.)
switch P.run.walk.dir
    case "+x", P.run.walk.wheel = 2; P.run.walk.sign = +1;
    case "-x", P.run.walk.wheel = 2; P.run.walk.sign = -1;
    case "+y", P.run.walk.wheel = 1; P.run.walk.sign = +1;
    case "-y", P.run.walk.wheel = 1; P.run.walk.sign = -1;
    otherwise
        error('CubliClean:UnknownWalkDir','Unknown P.run.walk.dir "%s".', ...
            P.run.walk.dir);
end
% The rate axis and the travel axis are DIFFERENT indices and conflating them
% silently halves the travel figure. Rotating about world Y means the rotation
% is omega_y (rate component 2) while the COM moves along x (position component
% 1); rotating about world X means omega_x (1) with travel along y (2).
P.run.walk.rateAxis = P.run.walk.wheel;
if P.run.walk.wheel == 2
    P.run.walk.travelAxis = 1;   % about Y -> COM moves along x
else
    P.run.walk.travelAxis = 2;   % about X -> COM moves along y
end
% Balance preset -> rotor-speed integral gains. Derived HERE, for the same
% reason as the walking direction above: cubli_mode applies overrides after
% cubli_clean_parameters has run, and cubli_refresh_ctrl is called last, so a
% preset chosen by an override has to be resolved here or it silently does
% nothing. "fast" is the verified behaviour and must stay exactly as it was;
% Ki = 0 makes the added term exactly zero, so the arithmetic is unchanged.
% A preset is a whole coherent set, not one gain. The tilt source has to be
% part of it: "attitude" with Ki = 0 is WORSE than "com" (measured, 30 s:
% rotor -254 rad/s and falling, against 1071 parked), because with an exact
% attitude tilt the law's only zero-torque point is a REAL lean, and nothing
% then removes the momentum that lean hands to gravity. The integral is what
% closes that loop. So the two are selected together and cannot be mixed by
% accident.
% "wheel_stop" keeps the COM tilt (tiltSrc = 0). That looks backwards -- the
% com form reads slide, not tilt, which is what made the rotor climb in the
% first place -- but it was MEASURED: the attitude tilt fails in all three
% forms tried (with Kw>0 it runs the rotor to -1665, with the rotor integral it
% tips by 80 s, with Kw=0 it falls in 30 s). The com loop is the only one that
% holds the cube up, so wheel_stop keeps it and fixes the climb elsewhere.
switch P.run.balancePreset
    case "fast"
        KiE = 0;  KiP = 0;  tiltSrc = 0;  lamW = 0;  KyawE = 0;
    case "wheel_stop"
        KiE = P.run.wheelStop.KiEdge;  KiP = P.run.wheelStop.KiPoint;
        tiltSrc = 0;   lamW = P.run.wheelStop.leakRate;
        KyawE = P.run.wheelStop.Kyaw;
    case "wheel_stop_yaw"
        % Same as wheel_stop plus a gentle X/Z yaw hold. X/Z carry a persistent
        % but tiny world-Z torque, so their speed grows linearly with run time
        % (~84 rad/s at 600 s) and they are not a forever-hold. See the note in
        % cubli_clean_parameters.
        KiE = P.run.wheelStop.KiEdge;  KiP = P.run.wheelStop.KiPoint;
        tiltSrc = 0;   lamW = P.run.wheelStop.leakRate;
        KyawE = P.run.wheelStop.KyawHold;
    case "edge_low_speed"
        assert(ismember(P.run.mode,["edge_balance","stand_to_edge"]), ...
            'CubliClean:EdgeLowSpeedMode', ...
            'edge_low_speed supports edge_balance and stand_to_edge only.');
        KiE = P.run.edgeLowSpeed.KiEdge; KiP = 0;
        tiltSrc = 0; lamW = 0; KyawE = 0;
    case "edge_near_zero"
        assert(ismember(P.run.mode,["edge_balance","stand_to_edge"]), ...
            'CubliClean:EdgeNearZeroMode', ...
            'edge_near_zero supports edge_balance and stand_to_edge only.');
        KiE = P.run.edgeNearZero.KiEdge; KiP = 0;
        tiltSrc = 0; lamW = 0; KyawE = 0;
    otherwise
        error('CubliClean:UnknownBalancePreset', ...
            'Unknown P.run.balancePreset "%s".',P.run.balancePreset);
end
% Standalone override for the experiments, applied only when the preset did
% not already pin it. exp_wzero uses it to separate the two effects.
% NOTE strlength, not ~isempty: isempty("") is FALSE, because it reports array
% size and "" is a 1x1 string that happens to hold no characters. Using
% isempty here made the "no override" default error out on every run.
if isfield(P.run.edge,'tiltSourceOverride') && ...
        strlength(string(P.run.edge.tiltSourceOverride)) > 0
    switch string(P.run.edge.tiltSourceOverride)
        case "com",      tiltSrc = 0;
        case "attitude", tiltSrc = 1;
        otherwise
            error('CubliClean:UnknownTiltSource', ...
                'Unknown tilt source "%s".',P.run.edge.tiltSourceOverride);
    end
end

radPerDeg = pi/180;    % YawRateRef is given in deg/s, the chart works in rad/s
P.run.ctrl = [P.geometry.edgeHeight; P.run.edge.Kp; P.run.edge.Kd; ...
              P.run.edge.Kw; P.run.edge.motorSign; ...
              P.motor.maxTorque; P.motor.peakTorque; P.motor.maxSpeed; ...
              P.run.standup.spinTorque; P.run.standup.spinSpeed; ...
              P.run.standup.brakeTorque; P.run.standup.captureTiltDeg; ...
              P.run.standup.captureBoostSec; ...
              P.control.point.KpP; P.control.point.KdP; ...
              P.control.point.KyawP; P.control.point.allocationDamping; ...
              P.control.point.pointSign; ...
              P.control.point.accelMatrix(:); ...
              P.run.pointstand.spinTorque; P.run.pointstand.spinSpeed; ...
              P.run.pointstand.brakeTorque; ...
              P.run.edgetopoint.spinTorque; P.run.edgetopoint.spinSpeed; ...
              P.run.edgetopoint.brakeTorque; ...
              P.run.capture.tiltDeg; P.run.capture.rate; ...
              P.control.point.vertexAttitude(:); ...
              P.run.walk.drive; P.run.walk.bleed; P.run.walk.landDeg; ...
              P.run.walk.nStep; ...
              P.run.walk.wheel; P.run.walk.sign; ...
              KiE; KiP; tiltSrc; lamW; P.run.edge.torqueDeadband; ...
              P.run.edge.KiTilt; KyawE; radPerDeg*P.run.wheelStop.YawRateRef];
% Separate fixed-size route input keeps the legacy 58-value ctrl vector and
% all existing mode-code indices intact. The route is validated for every
% mode, so a bad command cannot silently enter a later route run.
assert(P.run.route.maxSteps == 64,'CubliClean:RouteCapacity', ...
    'The generated controller expects exactly 64 route slots.');
routeCodes = cubli_route_encode(P.run.route.sequence,64);
assert(isfinite(P.run.route.stepTimeout) && P.run.route.stepTimeout > 0 && ...
    isfinite(P.run.route.flatTolDeg) && P.run.route.flatTolDeg > 0 && ...
    P.run.route.flatTolDeg < 45 && ...
    isfinite(P.run.route.rateTol) && P.run.route.rateTol > 0 && ...
    isfinite(P.run.route.settleTime) && P.run.route.settleTime > 0 && ...
    isfinite(P.run.route.minTravelFrac) && ...
    P.run.route.minTravelFrac > 0 && P.run.route.minTravelFrac <= 1, ...
    'CubliClean:RouteGates','Route timing and landing gates must be positive and finite.');
P.run.route.ctrl = [nnz(routeCodes); routeCodes; ...
    P.run.route.stepTimeout; P.run.route.flatTolDeg; ...
    P.run.route.rateTol; P.run.route.settleTime; ...
    P.cube.side; P.run.route.minTravelFrac];
% NOTE: separate climb/hold gains were tried for the edge->vertex manoeuvre and
% REVERTED as falsified -- see CUBLI_CUBLI_TEST_PROTOCOL.md, Stage D "尝试 1".
% Lowering the climb gain makes the climb slower, and a slower climb lets
% gravity act longer, so it needs MORE rotor momentum, not less. 40 of 40
% sampled combinations failed, including ones that had worked without it.
end
