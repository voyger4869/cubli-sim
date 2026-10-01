function P = cubli_mode(mode,overrides)
%CUBLI_MODE Parameters with one capability selected, ready to simulate.
%
% Centralises the three things that change with the mode: the mode code, the
% stop time, and the initial condition. Everything else stays as
% cubli_clean_parameters defines it.
%
%   P = cubli_mode("stand_to_edge");
%   assignin('base','P',P);
%   sim('Cubli_Clean_Cubli');
%
% overrides is an optional N-by-2 cell of dotted paths and values, applied
% BEFORE the initial condition is derived:
%   cubli_mode("point_balance",{'run.point.initialRollDeg',0})
% The ordering matters: P.run.ic.* is derived, so overriding a field that feeds
% it afterwards silently does nothing -- a trap that has already cost time here.

P = cubli_clean_parameters();
if nargin > 1 && ~isempty(overrides)
    for k = 1:size(overrides,1)
        P = localSetPath(P,overrides{k,1},overrides{k,2});
    end
end
P.run.mode = string(mode);
P.run.modeCode = find(P.run.modes == P.run.mode,1) - 1;
assert(~isempty(P.run.modeCode),'CubliClean:UnknownRunMode', ...
    'Unknown mode "%s". Known: %s.',mode,strjoin(P.run.modes,', '));

compression = P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
switch P.run.mode
    case "edge_balance"
        % Slightly off the balance attitude so a pass is falsifiable.
        th = deg2rad(P.run.edge.initialTiltDeg);
        P.run.ic.R = localRotY(pi/4 + th);
        P.run.ic.x = P.geometry.edgeHeight*sin(th);
        P.run.ic.z = P.geometry.edgeHeight*cos(th) - compression;
        P.run.duration = P.run.edge.duration;
    case "stand_to_edge"
        % Flat, shifted so the front bottom edge sits at x = 0, which is where
        % the balance attitude puts its pivot.
        P.run.ic.R = eye(3);
        P.run.ic.x = -P.cube.side/2;
        P.run.ic.z = P.cube.side/2 - compression;
        P.run.duration = P.run.standup.duration;
    case "walk"
        % Discrete face-by-face gait, starting flat exactly as stand_to_edge
        % does: the first step is the same tip over a bottom edge, and every
        % later step is geometrically identical to it.
        % The cube is placed so the edge it will pivot on sits at the origin:
        % for +-x that is a body-y edge (front/back), for +-y a body-x edge
        % (left/right). Only the placement changes with direction; the gait is
        % the same motion about a different axis.
        P.run.ic.R = eye(3);
        s = P.cube.side/2;
        switch P.run.walk.dir
            case "+x", P.run.ic.x = -s; P.run.ic.y = 0;
            case "-x", P.run.ic.x = +s; P.run.ic.y = 0;
            case "+y", P.run.ic.x = 0;  P.run.ic.y = -s;
            case "-y", P.run.ic.x = 0;  P.run.ic.y = +s;
        end
        P.run.ic.z = P.cube.side/2 - compression;
        P.run.duration = P.run.walk.duration;
    case "flat_to_point_direct"
        % EXPERIMENTAL (mode 8). Same flat start as stand_to_point: the pivot
        % corner (body (-s/2,-s/2,-s/2)) at the world origin, so the lift
        % rotates about the world (1,-1,0)/sqrt2 axis through it and lands on
        % the (1,1,1)/sqrt3 diagonal -- the same vertex mode 3 uses, unlike the
        % two-step route which lands on (-1,1,1)/sqrt3.
        P.run.ic.R = eye(3);
        P.run.ic.x = P.cube.side/2;
        P.run.ic.y = P.cube.side/2;
        P.run.ic.z = P.cube.side/2 - compression;
        P.run.duration = P.run.directjump.duration;
    case "flat_to_point"
        % The full chain: flat -> edge (Stage B) -> vertex (Stage D) -> hold.
        % Starts flat exactly as stand_to_edge does, because the first hop is
        % Stage B's sequence unchanged.
        P.run.ic.R = eye(3);
        P.run.ic.x = -P.cube.side/2;
        P.run.ic.y = 0;
        P.run.ic.z = P.cube.side/2 - compression;
        P.run.duration = P.run.flatchain.duration;
    case "edge_to_point"
        % Stage D. Edge balance -> vertex, tipping SIDEWAYS about world X.
        %
        % At the edge balance attitude the vertical body direction is the FACE
        % diagonal (-1,0,1)/sqrt2, and the contact is a line along world Y. The
        % vertex needs a BODY diagonal vertical, so the rotation is 35.26 deg --
        % smaller than the 45 deg of the flat->edge jump -- about a horizontal
        % axis through one END of the contact edge.
        %
        % That axis is world X. Verified from the geometry: the vector from the
        % pivot corner to the COM is (0, 0.5773, 0.8135) in world, which is
        % 35.26 deg off vertical, and rotating it about +X by +35.26 deg gives
        % (0, -0.0017, 0.9975), i.e. vertical. The COM travels on a circle of
        % radius h = 0.1299 about the pivot corner, so it rises monotonically
        % 0.1061 -> 0.1299 and the corner stays the single contact point.
        %
        % P.lift = side/2 shifts the cube along +y so the pivot corner (the
        % edge endpoint) sits at the world origin.
        %
        % Torque: at this attitude Wheel X is along body x and Wheel Z along
        % body z, whose world-x components are both +0.7071 while their world-z
        % components are opposite. Driving X and Z with the SAME sign therefore
        % cancels the z part and leaves a pure world-X torque -- cleaner than
        % the flat->edge jump, which had to fight cross-coupling.
        P.run.ic.R = localRotY(pi/4);
        P.run.ic.x = 0;
        P.run.ic.y = P.cube.side/2;
        P.run.ic.z = P.geometry.edgeHeight - compression;
        P.run.duration = P.run.edgetopoint.duration;
    case "stand_to_point"
        % Flat, placed so the pivot corner sits at the world origin. The direct
        % tip-up rotates about a horizontal axis through ONE bottom corner in
        % the face-diagonal direction (1,-1,0)/sqrt(2), by exactly
        % acos(1/sqrt(3)) = 54.74 deg, which brings the body diagonal
        % (1,1,1)/sqrt(3) vertical. Verified geometrically: the COM is at
        % distance h = 0.1299 from that axis throughout, and its height rises
        % monotonically 0.075 -> 0.1299, so the corner stays the single contact
        % point rather than the cube resting on an edge.
        % The body corner that pivots is (-s/2,-s/2,-s/2), so putting it at the
        % origin means the COM starts at (+s/2, +s/2, s/2 - compression).
        P.run.ic.R = eye(3);
        P.run.ic.x = P.cube.side/2;
        P.run.ic.y = P.cube.side/2;
        P.run.ic.z = P.cube.side/2 - compression;
        P.run.duration = P.run.pointstand.duration;
    case "point_balance"
        % Friction is LEFT ON. It used to be zeroed here to suppress a
        % stick-slip limit cycle on the single point contact (which injected
        % ~0.074 rad/s of spurious rate within 50 ms and made per-wheel
        % identification degenerate). That compromise is now obsolete:
        %
        %  * It was only ever needed because the tilt was reconstructed as
        %    asin(com_x/h). Zero friction means zero horizontal force, so the
        %    COM cannot translate and com_x is a CONSTANT -- the reconstruction
        %    carried no state at all. The tilt now comes from the attitude
        %    (u = R*n), which is independent of friction.
        %  * Measured 2026-09-26, the vertex holds 10 s with mu = 0.72/0.55
        %    exactly as it does with mu = 0: com_z held at the vertex for the
        %    whole run, fracVer 1.000, at tilt0 = 0.5/2.0/3.5 deg. Yaw stays
        %    bounded (|yaw| max 1.06 deg at 3.5 deg vs 0.047 with mu = 0) and
        %    wheel speed peaks at 302.6 rad/s, 17% of the 1800 limit.
        %  * Friction is REQUIRED for a flat -> vertex tip-up: without it the
        %    cube slides instead of pivoting on its corner. Since contact
        %    friction is a global block parameter the controller cannot switch
        %    mid-run, the vertex mode has to run with it on for the stand-up to
        %    be possible at all.
        %
        % The free-yaw degree of freedom that friction zero exposed is no
        % longer a special case to manage: the yaw damping term handles it, and
        % with friction present the contact couples it anyway.
        % Balanced on a vertex, COM directly above the contact corner at
        % h = side*sqrt(3)/2, with the body diagonal (1,1,1) pointing up.
        % R below is the orthonormal frame that does that; it puts the touched
        % corner (-s/2,-s/2,-s/2) exactly at the world origin, so the pivot is
        % fixed at x=y=0 and a tilt moves the COM to (h*sin(theta_x),
        % h*sin(theta_y)) -- the same reconstruction trick as the edge, in 2D.
        h = P.geometry.vertexHeight;
        % Columns are e1, e2, n with n = (1,1,1)/sqrt(3) the up body diagonal,
        % e1+e2 = sqrt(3)*(0,0,1) - n, and det = +1 (the handedness matters:
        % Simscape rejects a determinant of -1, and swapping e1/e2 flips it
        % while leaving e1+e2 -- and so the contact vertex -- unchanged).
        R0 = [-0.7886751345948129  0.2113248654051871  0.5773502691896258; ...
               0.2113248654051871 -0.7886751345948129  0.5773502691896258; ...
               0.5773502691896258  0.5773502691896258  0.5773502691896258];
        ax = deg2rad(P.run.point.initialRollDeg);
        ay = deg2rad(P.run.point.initialPitchDeg);
        P.run.ic.R = localRotX(ax)*localRotY(ay)*R0;
        P.run.ic.x = h*sin(ax);
        P.run.ic.y = h*sin(ay);
        P.run.ic.z = h - compression;
        P.run.duration = P.run.point.duration;
    otherwise
        error('CubliClean:NoInitialCondition', ...
            'Mode "%s" has no initial condition defined yet.',P.run.mode);
end
P = cubli_refresh_ctrl(P);
end

function R = localRotY(a)
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function R = localRotX(a)
R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
end

function P = localSetPath(P,path,value)
parts = strsplit(path,'.');
P = localAssign(P,parts,value);
end

function P = localAssign(P,parts,value)
if numel(parts) == 1
    P.(parts{1}) = value;
    return
end
P.(parts{1}) = localAssign(P.(parts{1}),parts(2:end),value);
end
