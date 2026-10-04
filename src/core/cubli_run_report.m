function report = cubli_run_report(com,rate,tauX,tauY,tauZ,penetration,P,Rrot,wheelY,routeLog,wheelX,wheelZ)
%CUBLI_RUN_REPORT Verdict for a unified-plant run, per selected mode.
%
% For edge_balance the state is reconstructed the same way the controller
% reconstructs it (theta = asin(com_x / d)), so the report judges exactly the
% quantity being controlled. It also checks the two things a free-contact run
% can get wrong that an ideal-support rig never could: the cube dropping off
% the edge, and the contact sliding sideways.
%
% KNOWN DEFECT, measured 2026-09-27, left as a red flag rather than silently
% patched: on this plant asin(com_x/d) is NOT a tilt. The cube can TRANSLATE,
% and a slide of 5.3 mm along x reads as 2.86 deg of tilt while the attitude
% from R shows the cube level to 0.04 deg (exp_edge_wtrace, 30 s). So
% report.tiltDeg -- and tiltOk with it -- measures the slide on the edge modes.
% The attitude-based tilt is computed alongside as report.tiltAttDeg, and the
% peak as report.tiltAttPeakDeg, whenever the R log is supplied as the 8th
% argument. Both are reported so the discrepancy is visible in every run
% instead of being discovered again later. Judging on the attitude version is
% the correct fix and is deliberately NOT done here: the "fast" preset's
% accepted numbers were recorded on the old basis, and re-basing acceptance
% criteria after the fact is exactly what this project has refused to do.
% CORRECTION 2026-10-03: tiltAttDeg is a fixed-world-axis projection, not a
% yaw-invariant edge tilt. Once the edge yaws, it reads yaw as lean. The
% edge_low_speed branch below computes edgeFrameTiltDeg in the rotating edge
% frame and uses that for its own acceptance gate; old gates are unchanged.
%
% Thresholds come from P, never from here.

if nargin < 7 || isempty(P), P = cubli_clean_parameters(); end
if nargin < 8, Rrot = []; end
if nargin < 9, wheelY = []; end
if nargin < 10, routeLog = []; end
if nargin < 11, wheelX = []; end
if nargin < 12, wheelZ = []; end
d = P.geometry.edgeHeight;
c = reshape(com.Data,3,numel(com.Time));
r = reshape(rate.Data,3,numel(rate.Time));
t = com.Time(:).';
win = t >= P.run.startWindow;

report.tEnd = t(end);
report.durationOk = abs(t(end)-P.run.duration) <= 1e-3;
report.comZMin = min(c(3,:));
report.comYRange = [min(c(2,:)), max(c(2,:))];
report.tauPeak = [max(abs(tauX.Data(:))), max(abs(tauY.Data(:))), max(abs(tauZ.Data(:)))];
report.ratePeakY = max(abs(r(2,win)));

switch P.run.mode
    case "edge_balance"
        % Reconstructed exactly as the controller does it -- which is the
        % slide-contaminated quantity described in the header. tiltAttDeg is
        % the honest one.
        tilt = asin(max(-1,min(1,c(1,:)/d)));
        report.tiltDeg = tilt*180/pi;
        report.tiltPeakDeg = max(abs(report.tiltDeg(win)));
        report.tiltEndDeg = report.tiltDeg(end);
        report = localAttitudeTilt(report,Rrot,win,P);
        % The slide is com_x minus the part of it that the TRUE tilt explains,
        % so it must be computed from the ATTITUDE tilt. Using the com-based
        % tilt here gives exactly zero by construction -- theta_com is
        % asin(com_x/d), so the subtraction cancels and the report printed
        % "com offset 0.00 mm" on runs that were visibly sliding.
        if isfield(report,'tiltAttDeg')
            report.slideMm = 1000*(c(1,end) - d*sin(report.tiltAttDeg(end)*pi/180));
        else
            report.slideMm = NaN;
        end
        % On the edge the COM sits at d minus the static compression. Falling
        % flat drops it to ~side/2, so this one number separates them cleanly.
        report.stayedOnEdge = report.comZMin > (d + P.cube.side/2)/2;
        report.tiltOk = report.tiltPeakDeg <= P.run.accept.tiltPeakMaxDeg;
        report.torqueOk = all(report.tauPeak <= P.motor.maxTorque*(1+1e-9));
        pn = max(penetration.Data(:));
        report.penetrationMax = pn;
        report.penetrationOk = pn <= 1.5*P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.checks = {'durationOk','tiltOk','torqueOk','stayedOnEdge','penetrationOk'};
    case "stand_to_edge"
        % Flat -> edge stand-up, then balance. Judged on whether the cube
        % actually reached the edge height and stayed there, not merely on the
        % torque trace: a run that never leaves the ground must not pass.
        report.tiltDeg = asin(max(-1,min(1,c(1,:)/d)))*180/pi;
        last = t > t(end)-2.0;
        report.comZMax = max(c(3,:));
        report.reachedEdge = report.comZMax > (d + P.cube.side/2)/2;
        report.stayedOnEdge = all(c(3,last) > (d + P.cube.side/2)/2);
        report.tiltEndDeg = report.tiltDeg(end);
        report = localAttitudeTilt(report,Rrot,last,P);
        report.tiltOk = max(abs(report.tiltDeg(last))) <= P.run.accept.tiltPeakMaxDeg;
        report.tauPeak = [max(abs(tauX.Data(:))), max(abs(tauY.Data(:))), max(abs(tauZ.Data(:)))];
        report.torqueOk = report.tauPeak(2) <= P.motor.peakTorque*(1+1e-9);
        pn = max(penetration.Data(:));
        report.penetrationMax = pn;
        report.penetrationOk = pn <= 1.5*P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.checks = {'durationOk','reachedEdge','stayedOnEdge','tiltOk', ...
                         'torqueOk','penetrationOk'};
    case {'point_balance','edge_to_point','flat_to_point','stand_to_point', ...
          'flat_to_point_direct'}
        % Vertex-family modes. Judged on com_z, which alone separates the three
        % attitudes this plant can settle into -- vertex 0.12954, edge 0.10570,
        % flat 0.07464. Do NOT judge these on asin(com_x/h): with the vertex
        % contact that quantity is dead (the COM cannot translate when the
        % point contact has no horizontal force), and once the cube is flat it
        % reads near its initial value, so a fall looks like a pass.
        zVertex = P.geometry.vertexHeight - ...
            P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        zGate = 0.120;   % between the vertex 0.12954 and the edge 0.10570
        report.comZEnd = c(3,end);
        report.endedOnVertex = c(3,end) > zGate;
        % "Stayed" must be measured AFTER the cube has climbed, not from t=0:
        % the stand-up modes begin flat, so a whole-run minimum is below the
        % gate by construction and would fail every pass. Judged on the last
        % stretch of the run instead.
        settleWin = t >= max(0,t(end) - min(5,P.run.duration/2));
        report.settledMin = min(c(3,settleWin));
        report.stayedOnVertex = report.settledMin > zGate;
        report.penetrationMax = max(penetration.Data(:));
        report.penetrationOk = report.penetrationMax <= ...
            1.5*P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.checks = {'durationOk','endedOnVertex','stayedOnVertex', ...
                         'penetrationOk'};
        if P.run.mode == "point_balance" && ...
                ismember(P.run.balancePreset,["point_low_speed","point_near_zero"])
            assert(~isempty(Rrot) && ~isempty(wheelX) && ...
                ~isempty(wheelY) && ~isempty(wheelZ), ...
                'CubliClean:PointSpeedLogs', ...
                'Point speed presets require attitude and all three wheel logs.');
            wx = localAlignedLog(wheelX,1,t);
            wy = localAlignedLog(wheelY,1,t);
            wz = localAlignedLog(wheelZ,1,t);
            rr = localAlignedLog(Rrot,9,t);
            Rv = reshape(rr,3,3,[]);
            up = squeeze(pagemtimes(Rv,[1;1;1]/sqrt(3)));
            tiltDeg = atan2(hypot(up(1,:),up(2,:)),up(3,:))*180/pi;
            tail = t>=max(0,t(end)-min(10,t(end)/2));
            wheelNorm = sqrt(wx.^2+wy.^2+wz.^2);
            report.wheelPeak = [max(abs(wx)),max(abs(wy)),max(abs(wz))];
            report.wheelNormTailMax = max(wheelNorm(tail));
            report.wheelNormEnd = wheelNorm(end);
            report.tiltTailMaxDeg = max(abs(tiltDeg(tail)));
            report.comXYDrift = norm(c(1:2,end)-c(1:2,1));
            if P.run.balancePreset == "point_low_speed"
                wheelLimit = P.run.pointLowSpeed.wheelTailMax;
            else
                wheelLimit = P.run.pointNearZero.wheelTailMax;
            end
            report.wheelSpeedOk = report.wheelNormTailMax <= wheelLimit;
            report.wheelBudgetOk = all(report.wheelPeak <= P.motor.maxSpeed);
            report.attitudeOk = report.tiltTailMaxDeg <= 2;
            report.checks = [report.checks, ...
                {'wheelSpeedOk','wheelBudgetOk','attitudeOk'}];
        end
    case "walk_route"
        assert(~isempty(routeLog) && ~isempty(wheelX) && ...
            ~isempty(wheelY) && ~isempty(wheelZ),'CubliClean:RouteLog', ...
            'Route walking requires route-state and all three wheel-speed logs.');
        state = cubli_log_reshape(routeLog,3);
        routeCodes = cubli_route_encode(P.run.route.sequence,64);
        n = nnz(routeCodes);
        expected = P.cube.side*[sum(routeCodes==1)-sum(routeCodes==2); ...
            sum(routeCodes==3)-sum(routeCodes==4)];
        actual = c(1:2,end)-c(1:2,1);
        report.route = upper(string(P.run.route.sequence));
        report.routeCompleted = round(state(1,end));
        report.routePhase = round(state(2,end));
        report.routeFault = round(state(3,end));
        report.expectedXY = expected;
        report.actualXY = actual;
        report.positionErrorSides = norm(actual-expected)/P.cube.side;
        report.completedOk = report.routeCompleted == n && report.routePhase == 4;
        report.faultOk = report.routeFault == 0;
        report.wheelPeak = [max(abs(cubli_log_reshape(wheelX,1))), ...
            max(abs(cubli_log_reshape(wheelY,1))), ...
            max(abs(cubli_log_reshape(wheelZ,1)))];
        report.wheelSpeedOk = all(report.wheelPeak <= P.motor.maxSpeed);
        report.positionOk = report.positionErrorSides <= 0.5;
        % Inspect each transition, not just the net endpoint: RLUD can return
        % near its start even if an individual command went the wrong way.
        change = find(diff(state(1,:)) > 0) + 1;
        report.stepXY = zeros(2,numel(change));
        report.stepProjectionSides = zeros(1,numel(change));
        report.stepCrossSides = zeros(1,numel(change));
        if ~isempty(change)
            atChange = interp1(t,c(1:2,:).',routeLog.Time(change),'linear').';
            report.stepXY = diff([c(1:2,1),atChange],1,2);
            for step = 1:numel(change)
                switch routeCodes(step)
                    case 1, direction = [1;0];
                    case 2, direction = [-1;0];
                    case 3, direction = [0;1];
                    case 4, direction = [0;-1];
                end
                report.stepProjectionSides(step) = ...
                    direction.'*report.stepXY(:,step)/P.cube.side;
                report.stepCrossSides(step) = ...
                    abs([-direction(2),direction(1)]*report.stepXY(:,step))/P.cube.side;
            end
        end
        report.stepsOk = numel(change) == n && ...
            all(report.stepProjectionSides >= P.run.route.minTravelFrac);
        report.crossTrackOk = all(report.stepCrossSides <= ...
            P.run.route.crossTrackMaxFrac);
        last = t >= max(0,t(end)-min(0.5,t(end)/4));
        zFlat = P.cube.side/2 - ...
            P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.settledOk = all(abs(c(3,last)-zFlat)<0.003) && ...
            max(vecnorm(r(:,last))) < P.run.route.rateTol;
        report.penetrationMax = max(penetration.Data(:));
        report.penetrationOk = report.penetrationMax <= ...
            1.5*P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.checks = {'durationOk','completedOk','faultOk','stepsOk', ...
            'crossTrackOk','positionOk', ...
            'settledOk','wheelSpeedOk','penetrationOk'};
    case "walk"
        % Discrete face-by-face gait. Counted on the ACCUMULATED cube rotation
        % about world Y, never on an attitude angle: atan2(R13,R33) reads the
        % same for 90 and 450 deg, so it cannot say how many faces were walked.
        %
        % The check that matters most is settledFrac. The requirement is one
        % face at a time, SETTLING FLAT between faces -- not continuous rolling
        % -- and a run that rolls through several faces can still show the
        % right total rotation and the right net travel. Only the time spent
        % flat separates those two, so it is measured directly: a completed
        % step spends most of its cycle flat (the tip takes ~0.4 s of ~2 s).
        zFlat = P.cube.side/2 - ...
            P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        % Travel is measured along the axis this direction walks along
        % (com_x for a Y-axis wheel, com_y for an X-axis wheel), and the
        % rotation about the matching world axis.
        report.rotAccumDeg = trapz(t,r(P.run.walk.rateAxis,:))*180/pi;
        ax = P.run.walk.travelAxis;
        report.facesDone = abs(report.rotAccumDeg)/90;
        report.sideTravel = abs(c(ax,end)-c(ax,1))/P.cube.side;
        report.flatFrac = mean(abs(c(3,:)-zFlat) < 1.5e-3);
        report.settledFrac = report.flatFrac;
        report.penetrationMax = max(penetration.Data(:));
        report.penetrationOk = report.penetrationMax <= ...
            1.5*P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
        report.facesOk = abs(report.facesDone - P.run.walk.nStep) <= 1.0;
        report.travelOk = abs(report.sideTravel - P.run.walk.nStep) <= 1.5;
        report.settledOk = report.flatFrac > 0.5;
        report.checks = {'durationOk','facesOk','travelOk','settledOk', ...
                         'penetrationOk'};
    otherwise
        % Nothing but the duration is checked here, so `ok` says only that the
        % run completed. Do not read it as a capability verdict.
        report.checks = {'durationOk'};
end

% The new edge-only preset is judged in the rotating edge frame. The legacy
% report fields and gates above are deliberately unchanged for old presets.
if P.run.balancePreset == "edge_low_speed" && ...
        ismember(P.run.mode,["edge_balance","stand_to_edge"])
    assert(~isempty(Rrot) && ~isempty(wheelY), ...
        'CubliClean:EdgeLowSpeedLogs', ...
        'edge_low_speed report needs attitude and Y-wheel speed logs.');
    Rm = reshape(cubli_log_reshape(Rrot,9),3,3,[]);
    ux = (squeeze(Rm(1,3,:)).'-squeeze(Rm(1,1,:)).')/sqrt(2);
    uy = (squeeze(Rm(2,3,:)).'-squeeze(Rm(2,1,:)).')/sqrt(2);
    uz = (squeeze(Rm(3,3,:)).'-squeeze(Rm(3,1,:)).')/sqrt(2);
    ex = squeeze(Rm(1,2,:)).'; ey = squeeze(Rm(2,2,:)).';
    edgeNorm = hypot(ex,ey);
    report.edgeFrameTiltDeg = atan2((ux.*ey-uy.*ex)./edgeNorm,uz)*180/pi;
    report.edgeYawDeg = atan2(-ex,ey)*180/pi;
    if P.run.mode == "stand_to_edge"
        edgeWin = t > t(end)-2;
    else
        edgeWin = win;
    end
    report.edgeFrameTiltPeakDeg = max(abs(report.edgeFrameTiltDeg(edgeWin)));
    report.tiltOk = report.edgeFrameTiltPeakDeg <= P.run.accept.tiltPeakMaxDeg;
    wy = cubli_log_reshape(wheelY,1);
    tw = wheelY.Time(:).';
    tailWin = tw >= max(0,tw(end)-min(10,tw(end)/2));
    report.wheelYEnd = wy(end);
    report.wheelYTailMax = max(abs(wy(tailWin)));
    report.wheelSpeedOk = report.wheelYTailMax <= P.run.edgeLowSpeed.wheelTailMax;
    report.checks{end+1} = 'wheelSpeedOk';
end

passed = false(size(report.checks));
for k = 1:numel(report.checks), passed(k) = report.(report.checks{k}); end
report.failedChecks = report.checks(~passed);
report.ok = isempty(report.failedChecks);

if P.run.balancePreset == "edge_low_speed" && ...
        ismember(P.run.mode,["edge_balance","stand_to_edge"])
    fprintf(['  edge_low_speed: edge-frame tilt peak %.4f deg; ' ...
        'Y wheel end %+.2f rad/s, final-window max %.2f rad/s ' ...
        '(limit %.1f); edge yaw end %+.2f deg\n'], ...
        report.edgeFrameTiltPeakDeg,report.wheelYEnd, ...
        report.wheelYTailMax,P.run.edgeLowSpeed.wheelTailMax, ...
        report.edgeYawDeg(end));
end

if strcmp(P.run.mode,"stand_to_edge")
    fprintf(['Cubli %s: tEnd=%.3f  tilt %+.3f -> %+.3f deg  comZ max %.5f ' ...
        '(edge %.5f)  comZ end %.5f  tauPeak=[%.4f %.4f %.4f]\n'], ...
        P.run.mode, report.tEnd, report.tiltDeg(1), report.tiltEndDeg, ...
        report.comZMax, d, min(c(3,end-9:end)), report.tauPeak);
elseif strcmp(P.run.mode,"edge_balance")
    fprintf(['Cubli %s: tEnd=%.3f  tilt %+.4f -> %+.4f deg (peak %.4f)  ' ...
        'comZ min %.5f  comY drift %.2e  tauPeak=[%.4f %.4f %.4f]\n'], ...
        P.run.mode, report.tEnd, report.tiltDeg(1), report.tiltEndDeg, ...
        report.tiltPeakDeg, report.comZMin, ...
        max(abs(c(2,:)-c(2,1))), report.tauPeak);
    if isfield(report,'tiltAttDeg')
        % Printed separately and labelled, because the two disagree: the com
        % form reads the SLIDE. See the header.
        fprintf(['  tilt from ATTITUDE: end %+.4f deg (peak %.4f) -- ' ...
            'com offset %.2f mm\n'],report.tiltAttEndDeg, ...
            report.tiltAttPeakDeg,report.slideMm);
    end
elseif ismember(P.run.mode, ...
        ["point_balance","edge_to_point","flat_to_point","stand_to_point", ...
            "flat_to_point_direct"])
    fprintf(['Cubli %s: tEnd=%.3f  comZ %.5f -> %.5f (max %.5f)  ' ...
        'tauPeak=[%.4f %.4f %.4f]\n'], ...
        P.run.mode, report.tEnd, c(3,1), c(3,end), max(c(3,:)), report.tauPeak);
    if isfield(report,'wheelNormTailMax')
        fprintf(['  three-wheel |w| end %.3f, tail max %.3f rad/s; ' ...
            'tilt tail max %.3f deg; XY drift %.1f mm\n'], ...
            report.wheelNormEnd,report.wheelNormTailMax, ...
            report.tiltTailMaxDeg,1000*report.comXYDrift);
    end
elseif strcmp(P.run.mode,"walk")
    fprintf(['Cubli %s: tEnd=%.3f  %d commanded steps -> %.2f faces  ' ...
        'travel %.2f sides  flat %.0f%% of the run\n'], ...
        P.run.mode, report.tEnd, P.run.walk.nStep, report.facesDone, ...
        report.sideTravel, 100*report.flatFrac);
elseif strcmp(P.run.mode,"walk_route")
    fprintf(['Cubli walk_route "%s": %d/%d steps, phase %d, fault %d; ' ...
        'XY actual [%.4f %.4f] m, expected [%.4f %.4f] m, error %.2f sides\n'], ...
        report.route,report.routeCompleted,numel(char(report.route)), ...
        report.routePhase,report.routeFault,report.actualXY, ...
        report.expectedXY,report.positionErrorSides);
    fprintf('  per-step forward travel (sides): %s; cross travel: %s\n', ...
        mat2str(report.stepProjectionSides,3),mat2str(report.stepCrossSides,3));
    fprintf('  cross-track limit %.2f side per step\n', ...
        P.run.route.crossTrackMaxFrac);
    fprintf('  max contact penetration %.3f mm\n',1000*report.penetrationMax);
    fprintf('  wheel speed peaks [%.1f %.1f %.1f] rad/s (limit %.0f)\n', ...
        report.wheelPeak,P.motor.maxSpeed);
else
    fprintf('Cubli %s: tEnd=%.3f  comZ min %.5f  tauPeak=[%.4f %.4f %.4f]\n', ...
        P.run.mode, report.tEnd, report.comZMin, report.tauPeak);
end
if ismember(P.run.mode, ...
        ["point_balance","edge_to_point","flat_to_point","stand_to_point", ...
            "flat_to_point_direct"])
    if report.ok
        fprintf(['  ok = true  (settled on the vertex; com_z end %.5f, ' ...
            'min over the last stretch %.5f)\n'],c(3,end),report.settledMin);
    else
        fprintf('  ok = FALSE  failed: %s  (com_z end %.5f)\n', ...
            strjoin(cellstr(report.failedChecks),', '),c(3,end));
    end
elseif report.ok
    if isfield(report,'stayedOnEdge')
        fprintf('  ok = true (on edge: %d)\n', report.stayedOnEdge);
    else
        % `otherwise` modes check only the duration, so say so rather than
        % letting "ok" read as a capability verdict.
        fprintf('  ok = true (duration only; this mode has no capability checks)\n');
    end
else
    fprintf('  ok = FALSE, failed: %s\n', strjoin(report.failedChecks,', '));
end
end

function report = localAttitudeTilt(report,Rrot,win,P)
% The edge tilt read off the ATTITUDE instead of com_x. The balance attitude
% is RotY(45 deg) on the contact edge, so the rotation about that edge is
% atan2(R13,R33) - 45 deg; the body z axis is the face diagonal that leans
% with the cube. Measured against asin(com_x/d) the two agree for a pure tilt
% and disagree completely once the cube slides, which is the point of
% reporting both.
if isempty(Rrot), return; end
Rm  = reshape(cubli_log_reshape(Rrot,9),3,3,[]);
att = (atan2(squeeze(Rm(1,3,:)).',squeeze(Rm(3,3,:)).') - pi/4)*180/pi;
report.tiltAttDeg     = att;
report.tiltAttEndDeg  = att(end);
report.tiltAttPeakDeg = max(abs(att(win)));
end

function v = localAlignedLog(ts,n,t)
% Most plant logs share a time axis. Avoid copying long attitude traces through
% interp1 when they are already aligned; a 120 s run can exhaust MATLAB memory.
v = cubli_log_reshape(ts,n);
if ~isequal(ts.Time(:),t(:))
    v = interp1(ts.Time(:),v.',t(:)).';
end
end
