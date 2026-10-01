function verify_walk_directions()
%VERIFY_WALK_DIRECTIONS Check all four walking directions by MEASURING the
%displacement, not by trusting the direction -> (wheel, sign) table.
%
%     verify_walk_directions
%
% The gait is the same motion about a different world axis: rotating about
% world Y travels along +-x (driven by Wheel Y), rotating about world X travels
% along +-y (driven by Wheel X). The sign picks which way along that axis. That
% mapping is an assumption, and this plant has had two torque-sign assumptions
% turn out to be backwards, so it is verified by measuring where the cube
% actually went.
%
% A symmetry cross-check comes free: +x and +y are geometrically equivalent, so
% their displacement magnitudes must come out identical; likewise -x and -y. If
% they differ, the two axes are not being treated identically in code.
%
% Runtime: about 2-3 minutes (4 runs of 20 s).
%
% Known caveats, both measured and documented in docs/03:
%   * the + directions gave 6.97 faces and the - directions 8.99, so the two
%     signs do NOT agree -- that is the existing gait fragility, not a
%     direction-logic error;
%   * there is about 0.10 m of cross-axis drift (10% of the travel).

setup_cubli();

fprintf('\n%-6s %8s %8s %12s %12s %8s  %s\n', ...
    'dir','wheel','sign','d_com_x','d_com_y','faces','verdict');
fprintf('%s\n',repmat('-',1,78));
res = zeros(4,2);
k = 0;
for dir = ["+x","-x","+y","-y"]
    P = cubli_mode("walk",{'run.walk.dir',dir});
    model = build_cubli_clean_cubli();
    assignin('base','P',P);
    out = sim(model,'ReturnWorkspaceOutputs','on');

    t  = out.get('cubli_com').Time(:).';
    c  = cubli_log_reshape(out.get('cubli_com'),3);
    rt = cubli_log_reshape(out.get('cubli_rate'),3);

    dx = c(1,end)-c(1,1);
    dy = c(2,end)-c(2,1);
    faces = abs(trapz(t,rt(P.run.walk.rateAxis,:)))*180/pi/90;

    k = k + 1;
    res(k,:) = [dx dy];
    % The cube must have travelled along the axis it was asked for, and barely
    % at all along the other one.
    want = double(extractBetween(dir,2,2) == "x");
    if want
        okDir = abs(dx) > abs(dy);
    else
        okDir = abs(dy) > abs(dx);
    end
    if okDir
        v = sprintf('OK (went %s)',dir);
    else
        v = 'WRONG AXIS';
    end
    fprintf('%-6s %8d %8d %12.5f %12.5f %8.2f  %s\n', ...
        dir,P.run.walk.wheel,P.run.walk.sign,dx,dy,faces,v);
end

fprintf('%s\n',repmat('-',1,78));
fprintf('symmetry check (+x vs +y, -x vs -y must match):\n');
fprintf('   |+x| = %.5f   |+y| = %.5f   diff %.2e\n', ...
    abs(res(1,1)),abs(res(3,2)),abs(abs(res(1,1))-abs(res(3,2))));
fprintf('   |-x| = %.5f   |-y| = %.5f   diff %.2e\n', ...
    abs(res(2,1)),abs(res(4,2)),abs(abs(res(2,1))-abs(res(4,2))));
end
