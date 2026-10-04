function P = cubli_clean_parameters()
%CUBLI_CLEAN_PARAMETERS Sole parameter source for the clean rebuild.
% All values use SI units and are deliberately independent of earlier models.

P.cube.mass = 0.50;
P.cube.side = 0.15;
P.wheel.mass = 0.08;
P.wheel.radius = 0.05;
P.wheel.thickness = 0.01;
% Three equal wheels with zero first moment, so this is the mass the ground
% actually carries. Previously derived identically in several builders.
P.cube.assemblyMass = P.cube.mass + 3*P.wheel.mass;

% 0.30 N*m, raised from 0.060 on 2026-09-26 after a physical-consistency audit.
%
% The 0.060 value was inherited from the 1-DOF IDEAL EDGE RIG era and was never
% revisited when the plant became a 3D free-contact Cubli. Its signature was
% still legible: 0.060 gives an edge capture band of asin(0.060/(m*g*d)) =
% 4.47 deg, matching the 5.0 deg acceptance threshold recorded at that time.
% Meanwhile peakTorque was raised to 0.80 to make the jump-ups work, leaving an
% internally inconsistent spec:
%   peak/continuous = 13.3x   (real motors are 2-4x)
%   continuous = 6.4% of the vertex gravity torque scale m*g*h = 0.943 N*m
%   implied peak power = 1440 W on a 0.74 kg body (drone-class)
%
% The cost was concrete and is what blocked Stage D. The vertex capture band is
% not a tuning artefact, it is pure actuator authority:
%   band = asin(maxTorque/(m*g*h))
% Measured against prediction, by raising maxTorque in mode 3 (diag_cubli_bandtest):
%   0.06 -> predicted  3.65 deg, held   3 deg (4 failed)
%   0.10 -> predicted  6.09 deg, held   6 deg
%   0.15 -> predicted  9.15 deg, held   8 deg
%   0.20 -> predicted 12.24 deg, held  12 deg
%   0.30 -> predicted 18.55 deg, held  20 deg
% So the band scales exactly as predicted and the old value simply starved the
% 3D plant of authority.
%
% 0.30 is chosen because it fixes two things at once: the vertex band goes to
% ~18.5 deg, and peak/continuous becomes 0.80/0.30 = 2.7x, back inside the
% physically normal 2-4x range. Rotor SPEED was never the constraint -- the
% edge->vertex climb needs only 20% of the wheel momentum capacity available at
% 1800 rad/s (<=60% including the gravity penalty) -- which is why sweeping
% maxSpeed produced scattered points instead of a region. Wrong knob.
P.motor.maxTorque = 0.30;
% G2C0 demonstrated that 500 rad/s cannot store enough wheel momentum for
% the 0.74 kg face-to-edge maneuver.  1800 rad/s (about 17 krpm) is an
% editable design value; the G2C controller enforces it explicitly.
P.motor.maxSpeed = 1800;
P.sim.controlPeriod = 0.002;
P.sim.stopTime = 10;
% Simscape Multibody mechanism local solver, as two named profiles. The plant
% family splits in two: models carrying a penalty-method Spatial Contact Force,
% and models whose joints are ideal. Explicit Euler (ODE1) injects a spurious
% yaw into the stiff penalty contact -- measured on a cube that should be
% stationary: 31.7 deg/s at 1 ms, 45.7 deg/s at 2 ms, while contact stiffness,
% damping and initial penetration all leave it in the same band. Higher-order
% explicit stages remove it (ODE3 at 1 ms: 0.27 deg of net yaw over 10 s vs
% 302 deg at ODE1). There is no implicit option in MultibodyLocalSolverChoice
% (ODE1..ODE8 only), so order is the lever. The Simscape-network local solver
% (UseLocalSolver/LocalSolverChoice) does have implicit options, but measured
% on this plant it is a no-op: enabling it at Trapezoidal or Backward Euler
% leaves the net yaw unchanged to four decimals.
% These are the only place the setting is made; each builder selects a profile
% by name. Contact-family settings are verified in CUBLI_G2A_TEST_PROTOCOL.md:
% at ODE1 @ 1 ms an unactuated cube accumulates 302.5 deg of yaw in 10 s, at
% ODE3 @ 1 ms 0.27 deg. Removing the wheel solids to make the body perfectly
% symmetric only takes the ODE1 figure to 60.7 deg, which no symmetric body can
% produce physically, so the driver is the integrator and not the assembly's
% nonzero I_xy (that is only a 5x amplifier, and ODE3 cancels it).
P.sim.localSolver.contact.choice = 'ODE3';
P.sim.localSolver.contact.sampleTime = 0.001;
P.sim.localSolver.ideal.choice = 'ODE1';
P.sim.localSolver.ideal.sampleTime = P.sim.controlPeriod;
P.world.gravity = 9.81;
% q=0 is the geometrically verified upright-edge condition in G1.  Keep it
% explicit so later modes can select a different support side/reference.
P.control.edge.referenceDeg = 0;
P.control.edge.Kp = 0.85;
P.control.edge.Kd = 0.05;
% Wheel-speed unloading is deliberately disabled until the basic edge loop
% is verified.  It otherwise adds a third, slow state that masks PD tuning.
P.control.edge.Kw = 0;
% Verified by the G1 fixed-torque probe: +0.06 N*m yields a smaller positive
% edge error than -0.06 N*m, so positive input is the stabilizing direction.
P.control.edge.motorSign = 1;
P.control.edge.initialErrorDeg = 1;

% G2C is an editable contact-and-actuation experiment.  This peak torque is
% intentionally separated from the 0.06 N*m continuous balance limit.
P.motor.peakTorque = 0.80;
P.control.standup.pulseStart = 0.10;
P.control.standup.stopTime = 2.0;
P.control.standup.edgeXRef = P.cube.side/2;
P.control.standup.KpX = 10.0;
P.control.standup.KdX = 0.25;

% G3C point-balance allocation. Columns X/Y/Z are the twice-repeated G3I
% angular-velocity increments from the corrected upright point geometry,
% divided by the applied torque impulse (0.002 N*m for 0.005 s) after
% subtracting the matched ZERO baseline. responseMatrix maps wheel torque
% [N*m] to qdd [rad/s^2].
P.control.point.probeTorque = 0.01;
P.control.point.probeDuration = 0.10;
P.control.point.responseMatrix = [ -0.000569001516522589  -0.000569001516504937  -0.000000000398737390579993;
                                  -0.000328513590473930   0.000328513590487735   0.000657026594753336] / ...
                                  (0.002 * 0.005);
% Valid FOR G3C'S IDEAL GIMBAL ONLY -- that is what this flag has always meant,
% and build_cubli_clean_point_balance.m asserts on it. It says nothing about the
% free-contact plant, which needs P.control.point.accelMatrix below instead.
P.control.point.responseMatrixValid = true; % mass-balanced wheel layout; baseline-subtracted G3I

% Measured on THE FREE-CONTACT PLANT, 2026-09-26, by diag_cubli_point10: 5 ms
% 0.01 N*m pulse per wheel from an exact vertex attitude, end-of-window world
% angular rate minus a matched ZERO baseline, divided by the pulse. Rows are
% [roll; pitch; yaw] and columns are the wheels X/Y/Z.
%   A_rate = [ +2.3074  +0.4736  -1.5135
%              -0.4736  -2.3074  -1.5135
%              -1.6426  +1.6426  -1.3223 ]   rad/s per N*m over 5 ms
% The same probe re-measured the roll/pitch rows to 3.5e-05 against the 2x3
% matrix the vertex loop had been using, which is what makes this trustworthy.
% Divided by the 5 ms pulse it becomes an angular-ACCELERATION map, which is
% what the control law needs: it lets tau = A^-1 u command an acceleration
% directly, with the plant's inertia already folded in.
% Cross-check: the diagonal ~460 rad/s^2 per N*m implies I_eff ~ 2.2e-3 kg*m^2,
% consistent with the cube (1.88e-3) plus three wheels.
% G3I's matrix above is NOT usable here: it was identified on the ideal gimbal,
% which suppresses yaw, and the free-contact plant has yaw as a live DOF.
P.control.point.accelMatrix = [ +2.3074  +0.4736  -1.5135; ...
                                -0.4736  -2.3074  -1.5135; ...
                                -1.6426  +1.6426  -1.3223 ] / 0.005;
P.control.point.accelMatrixValid = true; % free-contact plant, all 3 axes measured

% Vertex gains. These are in the acceleration-command frame, so they are not
% comparable to P.control.point.Kp/Kd above. They follow from the measured
% instability: with friction off the COM is pinned horizontally, so the only
% external torque is the normal force at the contact point, giving
%   I*theta_dd = m*g*h*theta          =>  lambda^2 = m*g*h/I ~ 377 /s^2
% (m = 0.74 kg assembly, h = 0.1299 m, I ~ 2.2e-3). Stability needs Kp > 377;
% Kp = 820 gives omega_n ~ 21 rad/s, and Kd = 2*zeta*omega_n at zeta = 0.7
% gives ~30 /s. This replaces the old 200/14, and also replaces the Kd=60
% "fix": that number was damping a law whose position term was a dead constant
% (see below), so it was never a position gain at all.
P.control.point.KpP = 820;
P.control.point.KdP = 30;
P.control.point.KyawP = 30;
P.control.point.pointSign = 1;
% These gains request a stabilizing angular acceleration. They are a first
% conservative closed-loop candidate, not a validated final tuning.
P.control.point.Kp = [200; 200];
P.control.point.Kd = [14; 14];
P.control.point.allocationDamping = 0.08;
P.control.point.torqueLimit = P.motor.maxTorque;
P.control.point.initialRollDeg = 0.05;
P.control.point.initialPitchDeg = -0.04;
P.control.point.balanceStopTime = 10.0;
% G3I is an open-loop identification experiment, not a balance controller.
% The short, low-amplitude pulse and the 40 ms observation interval reduce
% (but do not remove) contamination from gravity-driven divergence.
P.control.point.identification.pulseTorque = 0.002;
P.control.point.identification.pulseStart = 0.010;
P.control.point.identification.pulseDuration = 0.005;
P.control.point.identification.stopTime = 0.040;
P.control.point.identification.initialTiltDeg = 0;

% Contact values begin only at G2. They are intentionally not used to hide
% a geometry or controller defect in G0/G1.
P.contact.normalStiffness = 2.0e4;
% 160, not 55. Every contact model in the project ran at 160 except G2A, which
% read 55 here while G2C/G2D hardcoded 160 into the same block. The value is
% now single-sourced so the two contact gates share one contact law.
P.contact.normalDamping = 160;
P.contact.staticFriction = 0.72;
P.contact.kineticFriction = 0.55;
P.contact.frictionType = 'SmoothStickSlip';
P.contact.settleTime = 1.0;

% G2A acceptance. G2A is a precondition gate, not a validation of the contact
% law: the cube is unactuated and undisturbed, so the only thing it can measure
% is numerical quietness. Passing it does not mean the contact model is right,
% and it cannot detect the controller-driven failure that blocks G2D.
P.contact.accept.duration = 10.0;      % s; matches the G1/G3C hold standard
P.contact.accept.durationTol = 1e-3;   % s; rejects a run that stopped early
P.contact.accept.startWindow = 0.2;    % s; skip the initial penetration recovery
% Primary yaw criterion: net rotation, |integral of the yaw rate| over the
% window. Chosen over the alternatives after measuring them. SenseAngle is
% unsigned and saturates at 180 deg, so its endpoint difference aliases to
% zero at 36.73 deg/s and cannot be used at all. mean|yaw rate| is dominated
% by zero-mean chatter: at ODE3 the net rotation is 0.27 deg while that metric
% still reads 3.3 deg/s. The net integral is immune to both.
P.contact.accept.yawTotalMaxDeg = 5.0;
% The two criteria below are gross-failure detectors only. Neither separates
% ODE1 from ODE3 at 1 ms: COM xy drift sits at ~1e-3 m for any explicit stage
% (ODE1 1.06e-3, ODE3 1.02e-3), and the peak yaw rate is zero-mean chatter at
% 45.8 vs 13.6 deg/s. The yaw integral is what does the discriminating.
P.contact.accept.yawRatePeakGrossMaxDegS = 50.0;
P.contact.accept.comXYDriftMax = 5e-3; % m
% Penetration limit as a multiple of the designed static compression m*g/k,
% so the gate survives a legitimate change to normalStiffness. The cube starts
% at exactly 1.0x compression by construction, so this allows 50% transient
% overshoot -- a necessarily tight margin, and it does separate (ODE1 at 1 ms
% measures 1.94x).
P.contact.accept.penetrationFactor = 1.5;

P.geometry.groundTopZ = 0;
P.geometry.edgeHeight = P.cube.side/sqrt(2);
P.geometry.vertexHeight = P.cube.side*sqrt(3)/2;
% Balanced three-wheel layout. Equal wheel masses have zero first moment:
% sum(wheelCenters,1) == [0 0 0]. With r=0.05 m and thickness=0.01 m,
% each cylinder stays within the +/-0.075 m cube envelope. Their axial and
% radial extents are separated so the three solids do not intersect.
P.geometry.wheelCenters = [ 0.050  0.025  0;
                           -0.025 -0.050  0;
                           -0.025  0.025  0]; % X, Y, Z respectively
% ---------------------------------------------------------------------------
% Unified Cubli run configuration. One free-contact plant serves every mode;
% these select what it does. The model reads them from the base workspace, so
% changing a value here and re-running does not regenerate the .slx.
% ---------------------------------------------------------------------------
% "stand_to_point" is APPENDED so the existing mode codes do not shift -- a
% renumbering would invalidate every recorded result that cites a code.
P.run.modes = ["idle","edge_balance","stand_to_edge","point_balance","walk", ...
               "stand_to_point","edge_to_point","flat_to_point", ...
               "flat_to_point_direct","walk_route"];
P.run.mode = "edge_balance";

% Edge balance gains. The LAW transfers from G1, but the GAINS DO NOT.
% The plant is I_edge*thetadd = m*g*d*sin(theta) - u, so a rigid ideal pivot
% needs only Kp > m*g*d = 0.7705 N*m/rad and G1's 0.85 was enough there.
% On the real penalty contact the measured stability threshold lies between
% Kp = 1.2 and 1.6 (gain sweep, 10 s, 1 deg start): 0.85/0.05, 0.85/0.15,
% 1.20/0.05 and 1.20/0.15 all fall flat, 1.60/0.15 holds. The compliant
% contact therefore eats G1's 10% margin -- the effective threshold is nearer
% 0.86 than 0.7705. 2.2/0.15 holds with peak tilt 0.91 deg and 0.052 N*m.
% Re-tune here, not on the ideal rig, whenever the contact changes.
P.run.edge.Kp = 2.2;
P.run.edge.Kd = 0.15;
% Wheel-speed unloading: tau_y = -(Kp*theta + Kd*thetad) + Kw*w.
% Without it the rotor accumulates monotonically and pins the 1800 rad/s limit,
% where the speed limiter zeroes the torque and the cube falls (measured:
% -10 -> 24 -> 112 -> 248 -> 443 -> 675 -> 952 -> 1249 -> 1573 -> 1800 rad/s at
% t = 9.67 s). That is why a run that looked stable for 8 s died at 9, and why
% two identical runs disagreed before it. G1 reserved this term and left it at
% 0 deliberately; it is now required.
%
% SIGN IS POSITIVE, measured. Algebra predicted negative and was wrong, so this
% was set by measurement like the wheel-torque sign. The trade is monotone:
%   Kw=0     wheel ends 1908 rad/s, tilt 0.85 deg
%   Kw=2e-4  wheel ends  432 rad/s, tilt 1.98 deg   <- chosen: 4x speed margin
%   Kw=5e-4  wheel ends  205 rad/s, tilt 2.55 deg
%
% The residual is a slow lean: the equilibrium of a proportional bias is
% theta = Kw*w/Kp, so a still-growing rotor drags the tilt with it (measured
% 432 -> 600 rad/s and 2.2 -> 2.9 deg between t=10 and t=12 s). An INTEGRAL
% bias (thetaBdot = -Ki*w) was implemented to remove that, because its only
% equilibrium is theta = 0, w = 0. It was REVERTED: swept against both modes it
% behaved erratically (Ki=0 held edge_balance but not the stand-up, Ki=2e-5 the
% reverse, and small Ki failed both), because integrating inside a MATLAB
% Function that runs at the solver's variable step makes the effective gain
% depend on the step pattern. Removing the drift properly needs the controller
% on a fixed 2 ms rate first. Recorded so it is not retried blind.
P.run.edge.Kw = 2e-4;
% Fixed-torque sign probe, 2026-09-26 (diag_cubli_sign_probe, 0.5 s, open loop).
% tau_y = 0 falls with omega_y 5.365 rad/s; -0.01 slows it to 1.469 (3.65x)
% while +0.01 accelerates the fall. A NEGATIVE command therefore opposes a
% positive tilt, so the cube-side torque about world +Y is u = -tau_y and the
% stabilising command is tau_y = -(Kp*theta + Kd*thetad), i.e. motorSign = -1.
% Recorded from the measured effect, not from the joint's documented axis
% convention, which disagrees with it.
P.run.edge.motorSign = -1;
P.run.edge.initialTiltDeg = 1.0;
P.run.edge.duration = 10.0;

% Face->edge stand-up by momentum transfer, the way the real Cubli does it:
% spin the wheel up slowly, then brake it hard. Braking converts the stored
% wheel momentum into cube momentum over a few tens of ms.
%   energy to lift COM flat->edge : 0.2255 J
%   I_edge about the contact edge : 1.020e-2 kg*m^2  (cube 1.875e-3 + m*d^2)
%   so the cube needs 6.65 rad/s, L = 0.0678 kg*m^2/s
%   gravity costs another 0.0462 N*m*s over the pulse -> total L = 0.1140
%   wheel momentum L = I_w*w, I_w = 5e-4... (0.5*0.08*0.05^2 = 1.0e-4)
%   -> wheel needs ~1140 rad/s, braked at 0.80 N*m over ~140 ms.
% SIGNS, measured at the flat attitude (diag_cubli_flatsign): tau_y = +0.8
% tips the cube forward, tau_y = -0.8 tips it backward, and +-0.4 does not tip
% it at all, matching the m*g*side/2 = 0.5445 threshold. So the cube-side
% torque about world +Y is u = +tau_y, and the plant is
%   I*thetadd = m*g*d*sin(theta) + u
% (an earlier note had a sign error here; the working balance law s = -1 is
% what that equation predicts).
% Hence: store momentum with a NEGATIVE command, then spend it with a POSITIVE
% one. Spinning negative stores -I_w*w, which is what a forward tip needs.
P.run.standup.spinTorque = -0.40;  % |0.40| < 0.5445, so the cube stays flat
% 900, found by sweep with the capture boost active. The window is still narrow
% but it MOVED when the boost was added: without it 1050 stood the cube up and
% 1100 overshot; with it 800 and 900 hold and 1000 + fall off. The boost is not
% a cure-all because the capture law is Kp*theta + Kw*w, so at a small arrival
% tilt it commands only ~0.12 N*m and the 0.80 clamp does not bite; a faster
% arrival still outruns 0.80. The real lever is the arrival speed, which is what
% this constant sets.
P.run.standup.spinSpeed = 900;
P.run.standup.brakeTorque = 0.80;  % positive = forward, past the 0.5445 threshold
% Hand over to the balance PD this close to the edge attitude. Bounded by
% torque authority, not by choice: holding a tilt needs u = m*g*d*sin(theta),
% and the 0.06 N*m continuous limit caps that at |theta| <= 4.47 deg. Handing
% over at -8 deg would ask for 0.107 N*m and the cube could not be caught.
P.run.standup.captureTiltDeg = -3;
% Peak-torque authority for this long after the handover. The 0.06 N*m figure
% is the CONTINUOUS limit; the spec also defines a 0.80 N*m PEAK, and a capture
% transient is what a peak rating is for. It matters: holding a tilt needs
% u = m*g*d*sin(theta), so 0.06 caps the recoverable tilt at 4.47 deg, which is
% why the stand-up window was only 1050..1100 rad/s. At 0.80 the available
% torque exceeds m*g*d = 0.7705 at every angle, so any overshoot is recoverable.
% After the window the continuous limit applies as before.
P.run.standup.captureBoostSec = 0.40;

% Point balance on a real vertex contact (Stage C). Verified 2026-09-26: 10 s
% hold through the shipped controller from initial tilts up to 3.5 deg, friction
% ON (see cubli_mode for why the old friction-free setting was dropped). The
% allocation matrix is P.control.point.accelMatrix, measured on this plant.
P.run.point.initialRollDeg = 0.05;
P.run.point.initialPitchDeg = -0.04;
% 10.0, not 2.0. At 2.0 every "10 s vertex gate" run silently ran for 2 s,
% because cubli_mode copies this into P.run.duration and that is what StopTime
% reads -- while the builder's banner printed "10.0 s" from the DEFAULT
% parameters. A whole round of conclusions was drawn from those 2 s runs.
P.run.point.duration = 10.0;
P.run.standup.duration = 12.0;

% Flat -> vertex DIRECT tip-up (Stage D). The cube rotates about a horizontal
% axis through one bottom corner, in the face-diagonal direction (1,-1,0)/sqrt2,
% by 54.74 deg. Energy to raise the COM 0.075 -> 0.1299 is 0.3986 J, so the
% required angular momentum about the pivot is ~0.110 kg*m^2/s at omega ~7.24
% rad/s. Unlike the edge jump, TWO wheels share it (X and Y, opposite signs), so
% the per-wheel burden is lower.
%
% Torque thresholds: the net tipping torque is sqrt(2)*tau per wheel, and the
% flat-attitude gravity restoring torque about the pivot is
% m*g*side/sqrt2 = 0.7700 N*m. So sqrt(2)*tau < 0.7700 => tau < 0.5445 keeps the
% cube down while the wheels store momentum; the brake must exceed it.
P.run.pointstand.spinTorque = 0.40;    % per wheel, same sign on X and Y, < 0.5445
P.run.pointstand.spinSpeed = 900;      % wheel speed at which to brake
P.run.pointstand.brakeTorque = -0.80;  % reverses the spin-up, exceeds the threshold
P.run.pointstand.duration = 12.0;

% Edge balance -> vertex (Stage D, the second hop of the two-step route).
% Tipping sideways about world X by 35.26 deg; see cubli_mode for the geometry.
% Wheels X and Z driven with the SAME sign give a pure world-X torque at this
% attitude, because their world-x components are both +0.7071 while their
% world-z components cancel.
% Threshold: the gravity couple about the pivot corner is m*g*side/2 = 0.5445
% N*m and the net wheel torque is sqrt(2)*tau, so tau < 0.385 stores momentum
% without tipping and the brake must exceed it.
% Momentum: the climb costs 0.1728 J (LESS than the flat->edge 0.2258 J that is
% already proven), needing 4.77 rad/s and 0.0725 kg*m^2/s about the pivot, so
% about 860 rad/s per wheel even with Stage B's 1.68 gravity penalty.
% spinSpeed 0 means NO IMPULSE: the controller goes straight to the vertex law.
% That is the variant that works. The staged spin-up/brake sequence was tried
% first (following Stage B's shape) and FAILS here: it arrives at the vertex
% attitude at 0.071 deg but the climb spends the rotor budget at peak torque,
% and every gain/speed/torque sweep left the result unchanged or worse.
% The reason the law-only form works is that driving u to (0,0,1) IS the climb,
% so no separate store-and-brake phase is needed.
P.run.edgetopoint.spinTorque = 0.30;   % only used if spinSpeed > 0
P.run.edgetopoint.spinSpeed = 0;       % 0 => law only (the working setting)
P.run.edgetopoint.brakeTorque = -0.80; % only used if spinSpeed > 0
P.run.edgetopoint.duration = 12.0;
% Full chain flat -> edge -> vertex -> hold (mode 7), the user-facing goal:
% point balance standing up from flat. Stage B's edge half is reused verbatim
% (spin-up 0.40 on Wheel Y, brake -0.80, capture with the edge PD and its 0.40 s
% peak window); the handover is gated on the edge SETTLING (|th| < 2 deg and
% |omega_y| < 0.5) rather than a fixed time, then the vertex law takes over with
% the attitude-corrected allocation. No new gains: every constant comes from the
% two proven halves.
P.run.flatchain.duration = 20.0;

% EXPERIMENTAL (mode 8): flat -> vertex in ONE continuous motion, no impulse and
% no edge phase at all. The vertex law, engaged from the flat attitude, lifts the
% cube diagonally onto its corner in about 0.6 s.
%
% Status: PRELIMINARY. Verified reproducible (three bit-identical runs) over a
% contiguous 4-of-16 gain patch, in an isolated experiment. NOT yet promoted to
% an accepted capability, because:
%   * rotor peak 1603.5 rad/s = 89% of the 1800 limit, against 60% for the
%     two-step route -- far less margin;
%   * the gain patch is narrower (4/16 against 7/16);
%   * NO disturbance rejection has been measured (initial tilt, position
%     offset, yaw).
% It lands on the diagonal (1,1,1)/sqrt3, the same one mode 3 uses, whereas the
% two-step route lands on (-1,1,1)/sqrt3. The pivot corner is body
% (-s/2,-s/2,-s/2), so the torque comes from wheels X and Y driven with the SAME
% sign (wheel Y's axis is body -y).
P.run.directjump.duration = 12.0;

% WALKING (mode 4). Discrete face-by-face gait: drive forward until the cube has
% turned one face, stop, bleed the rotor back to zero, repeat.
%
% The shape is this simple because of what was measured, not by choice:
%   * a POSITIVE tau_y drives the cube forward (exp_walk_dump: +0.70 took com_x
%     from -0.075 to +0.065 with the body x axis ending along world -z, i.e.
%     +90 deg about world Y);
%   * a STORE phase must NOT be used per step. Stage B stores at -0.40 then
%     reverses with +0.80; reused per step the rotor spends the whole reversal
%     being pulled back through zero while the cube is still being driven, and
%     it rolled 4-5 faces (measured at spinW 700 and 1300).
%   * after landing the rotor must be bled back. The contact absorbs momentum,
%     so a resting CUBE does not imply a resting ROTOR -- measured, cube
%     perfectly flat with the rotor still at -1806 rad/s.
% drive 0.60-0.75 all give one clean face (89.9-90.8 deg, one side length of
% travel, rotor back to 44-47 rad/s). 0.56 does not get over.
% KNOWN FRAGILITY: each drive briefly pegs the rotor at ~1818 against the 1800
% limit, so the speed guard truncates the drive. The gait currently depends on
% that truncation. drive and bleed are therefore both workspace-editable.
P.run.walk.drive = 0.70;
P.run.walk.bleed = 0.15;
P.run.walk.landDeg = 88;
P.run.walk.nStep = 8;
% 20 s, not 16. Each step needs about 1.5-2 s (drive ~0.4 s, bleed ~0.5 s,
% then a settle), and 8 steps at 16 s left the last one unfinished -- measured
% 6.97 faces from 8 commanded. The fix is to allow the time, not to widen the
% tolerance.
P.run.walk.duration = 20.0;

% Walking direction. The cube tips about an edge, and which edge is decided by
% which BODY AXIS it rotates about:
%   rotate about world Y -> travels along +/-x, driven by Wheel Y (at the flat
%     attitude wheel Y's axis lies along world Y; wheel Z's is vertical and only
%     yaws, which is why the very first stand-up attempt failed)
%   rotate about world X -> travels along +/-y, driven by Wheel X
% The sign selects which way along that axis.
% The wheel/sign pair is what the controller uses; the direction name is what a
% reader uses. They are mapped here in one place so the two cannot drift.
P.run.walk.dir = "+x";        % "+x" | "-x" | "+y" | "-y"
% The dir -> (wheel, sign) mapping is DERIVED in cubli_refresh_ctrl, not here.
% Deriving it here would be a trap: cubli_mode applies its overrides AFTER
% cubli_clean_parameters has run, so overriding run.walk.dir would leave the
% wheel and sign stale -- the same silent-failure shape as P.run.ic.*.

% Route walking (mode 9). Each character means one complete face roll in a
% fixed WORLD direction: R/L = +/-X, U/D = +/-Y. The route is compiled to a
% fixed-size numeric controller input by cubli_route_encode; the original
% single-direction walk controller and its 58-element parameter vector stay
% unchanged. The gait remains experimental until its physical checks pass.
P.run.route.sequence = "RLUD";
P.run.route.maxSteps = 64;
P.run.route.stepTimeout = 4.0;
P.run.route.flatTolDeg = 10;
P.run.route.rateTol = 0.5;
P.run.route.settleTime = 0.10;
P.run.route.minTravelFrac = 0.5;
P.run.route.crossTrackMaxFrac = 0.30; % per step, fraction of one cube side
P.run.route.speedGuardMargin = 35;  % rad/s before the 1800 rad/s wheel limit
P.run.route.duration = [];  % auto: stepTimeout * route length + 2 s


% Capture-window gate for the manoeuvre modes (5 and 6), in degrees and rad/s.
% Peak torque is held while the tilt or the tilt rate exceeds these, then the
% 0.06 continuous limit resumes. A fixed TIME window was measured not to work:
% the edge->vertex climb outlasts it, leaving only 0.06 N*m, which cannot
% arrest the 2.69 rad/s the cube carries into the vertex.
P.run.capture.tiltDeg = 1.5;
P.run.capture.rate = 1.0;

% The vertex attitude R0, flattened column-major, for the attitude-corrected
% allocation (prm 36..44). A was identified AT the vertex; away from it the
% wheel->axis mapping rotates with the body, so using A unchanged commands the
% torque in the wrong direction. Measured on the edge->vertex climb: the tilt
% grew from 33.08 to 53.83 deg (flat is 46.8) -- the controller drove the cube
% AWAY from the vertex -- with u_z swinging over -0.84..+0.85 and the rotors
% winding to -1857 rad/s, spending 0.181 kg*m^2/s per wheel for a manoeuvre that
% needs 0.0713. That is why no amount of gain, speed or torque tuning helped:
% the direction was wrong, not the magnitude.
R0v = [-0.7886751345948129  0.2113248654051871  0.5773502691896258; ...
        0.2113248654051871 -0.7886751345948129  0.5773502691896258; ...
        0.5773502691896258  0.5773502691896258  0.5773502691896258];
P.control.point.vertexAttitude = R0v;

% A separate climb gain pair for the manoeuvre modes was tried and REMOVED as
% falsified. The reasoning was that one gain pair cannot both stabilise the
% vertex (needs Kp > 377 /s^2) and climb (needs only slightly more than
% gravity, 377*sin(theta)); the measured result was that all 40 sampled
% combinations failed, including ones that had worked without it. Lowering the
% climb gain makes the climb slower, and a slower climb lets gravity act for
% longer, so it needs MORE rotor momentum. Same mechanism that falsified the
% "gentle brake" idea for the direct jump: on this plant, gentler is worse.
% Recorded in CUBLI_CUBLI_TEST_PROTOCOL.md, Stage D.

% The wheel axes are NOT both along +world in the flat attitude: the pose
% rotations give Wheel X -> +x but Wheel Y -> -y (see the third column of
% [1 0 0; 0 0 -1; 0 1 0]). Driving X and Y in OPPOSITION therefore produces a
% torque about (1,1,0), an axis that cannot bring the body diagonal vertical.
% Measured (diag_cubli_tipaxis, constant torque, no phasing):
%
%   pattern   T      com_z max   u_z max    u_xy at u_z max
%   +1+1      0.80   0.17214     0.99999    0.00443   <- reaches the vertex
%   -1-1      0.80   0.15295     1.00000    0.00197   <- reaches the vertex
%   +1-1      0.80   0.15260     0.57738    0.81648   <- launched, never rotated
%   -1+1      0.80   0.15260     0.57738    0.81648
%   (and at T = 0.40 nothing moves at all: u_z stays at the flat 0.5774)
%
% u_z is the third component of u = R*n with n = (1,1,1)/sqrt3: 0.5774 when flat,
% 1.0 at the vertex attitude. So the same-sign patterns are the correct pair, and
% 0.40 vs 0.80 brackets the predicted 0.5445 threshold from both sides.

% Which mode the model should run, as a code the controller block can switch
% on. MATLAB Function blocks cannot portably carry strings, so the mapping
% lives here in one place.
P.run.modeCode = find(P.run.modes == P.run.mode,1) - 1;
assert(~isempty(P.run.modeCode),'CubliClean:UnknownRunMode', ...
    'Unknown P.run.mode "%s". Known: %s.',P.run.mode,strjoin(P.run.modes,', '));

switch P.run.mode
    case "stand_to_edge",  P.run.duration = P.run.standup.duration;
    case "stand_to_point", P.run.duration = P.run.pointstand.duration;
    case "edge_to_point",  P.run.duration = P.run.edgetopoint.duration;
    case "flat_to_point",  P.run.duration = P.run.flatchain.duration;
    case "flat_to_point_direct", P.run.duration = P.run.directjump.duration;
    case "walk",          P.run.duration = P.run.walk.duration;
    case "walk_route"
        if isempty(P.run.route.duration)
            P.run.duration = P.run.route.stepTimeout* ...
                numel(char(P.run.route.sequence)) + 2.0;
        else
            P.run.duration = P.run.route.duration;
        end
    otherwise,             P.run.duration = P.run.edge.duration;
end

% The unified model reads its initial condition, mode code and stop time as
% workspace expressions (P.run.ic.*, P.run.modeCode, P.run.duration) rather
% than baked literals, so changing P.run.mode and re-running swaps the
% behaviour WITHOUT regenerating the .slx. That is the whole point of the
% unified plant; run_cubli_clean puts P in the base workspace for it.
edgeD = P.geometry.edgeHeight;
compression = P.cube.assemblyMass*P.world.gravity/P.contact.normalStiffness;
% One parametrisation covers lying flat AND balanced on the edge:
%
%   COM = (d*sin(theta), 0, d*cos(theta))   R = RotY(45deg + theta)
%
% with the contact-edge pivot fixed at world x=0. theta = -45 deg reproduces
% the flat pose exactly (R = I, COM x = -side/2) and theta = 0 is the balance
% attitude. That continuity is what lets the stand-up hand over to the balance
% law without the pivot jumping.
%
% The edge the cube ends up on is the one it tips over: the front BOTTOM edge
% when flat is a body-y edge, so the balance attitude is RotY(45 deg) with
% body y horizontal, and the active reaction wheel is Wheel Y. (An earlier
% version used a body-z edge reached via a 120 deg rotation about the diagonal
% -- a valid balance, but not the edge a front-edge tip-up can reach.)
% Default; cubli_mode overrides y (and everything else) per capability. Kept
% here so the model's P.run.ic.y block expression always resolves even when a
% caller only ran cubli_clean_parameters.
P.run.ic.y = 0;
switch P.run.mode
    case "edge_balance"
        th = deg2rad(P.run.edge.initialTiltDeg);
        P.run.ic.R = localRotY(pi/4 + th);
        P.run.ic.x = edgeD*sin(th);
        P.run.ic.z = edgeD*cos(th) - compression;
    case "idle"
        P.run.ic.R = eye(3);
        P.run.ic.x = 0;
        P.run.ic.z = P.cube.side/2 - compression;
    case "stand_to_edge"
        % Flat, shifted so the front bottom edge sits at x = 0, which is where
        % the balance attitude puts its pivot.
        P.run.ic.R = eye(3);
        P.run.ic.x = -P.cube.side/2;
        P.run.ic.z = P.cube.side/2 - compression;
    otherwise
        error('CubliClean:NoInitialCondition', ...
            'Mode "%s" has no initial condition defined yet. Known: idle, edge_balance, stand_to_edge.', ...
            P.run.mode);
end

% ---------------------------------------------------------------------------
% Balance preset: how the law treats the ROTOR SPEED once the cube is balanced.
%
% WHY THIS EXISTS, measured (exp_edge_wtrace, edge_balance, 30 s, Kw = 1e-4):
%
%   rotor w_y            0 -> 1092 rad/s, then FLAT (last 10 s: -2.9 rad/s)
%   true attitude tilt   +-0.04 deg (from R; the cube really is level)
%   commanded tau_y       oscillating about 0, peak 0.017 N*m
%   com_z                0.10568 constant (the edge height)
%   asin(com_x/d)        climbed to +2.86 deg   <-- NOT a tilt
%   com_x                climbed to +5.3 mm
%
% com_x's 5.3 mm and asin(com_x/d)'s 2.86 deg are one-for-one, while the true
% attitude is level: on this plant com_x measures a SLIDE, not a tilt. So the
% rotor does not "converge slowly" -- it FREEZES, and it freezes for a
% structural reason, not a gain reason:
%
%   The rotor speed can only change while the commanded torque is non-zero
%   (wdot = -tau/I_w, measured convention). A proportional law drives its own
%   torque to zero at every point on the balance manifold. So the moment the
%   cube is balanced the rotor stops changing, and the value it stops at is
%   set by the entry transient -- how much momentum had to be absorbed -- and
%   NOT by Kw. Kw only moves the zero-torque point to theta = Kw*w/Kp, so a
%   larger Kw needs a larger slide to get there: it arrives later, not lower.
%   The "Kw = 0 -> 1908, 2e-4 -> 432, 5e-4 -> 205 rad/s" table below was read
%   off 10 s transients and does not mean what it looks like.
%
% Removing rotor momentum needs an EXTERNAL torque, and the only one available
% is gravity: about the contact edge dH/dt = m*g*d*sin(theta_true), and every
% motor and contact force is internal. So the cube must LEAN, hand the momentum
% to gravity, and straighten up. A proportional term cannot do that, because
% its equilibrium IS the zero-torque point. An integral of the rotor speed can:
% it changes the equilibrium itself, making w = 0 the only fixed point.
%
% The same freeze happens at the vertex (exp_wpark): after the two stand-up
% chains the rotors park at 507 and 880 rad/s; point_balance only looks like it
% ends near zero because its default 0.05 deg start barely perturbs anything.
%
% Two presets, each with its own acceptance record:
%   "fast"        Ki = 0. Frozen rotor, but the quickest, cleanest arrival.
%                 THIS IS THE CURRENTLY VERIFIED BEHAVIOUR AND MUST NOT MOVE.
%   "wheel_stop"  Ki != 0. Rotor driven to ~0 once settled, paid for with a
%                 larger lean while the momentum is dumped and a slower settle.
P.run.balancePreset = "fast";
% ---------------------------------------------------------------------------
% STATUS OF "wheel_stop": gives the requested behaviour at 0.25 ms, but the
% PLANT IS NOT NUMERICALLY CONVERGED, so this is "a setting that looks right
% and holds 300 s", NOT a verified converged result. Read all three parts.
%
% PART 1 -- THE OSCILLATION THE USER OBJECTED TO IS NUMERICAL, AND THE STEP IS
% THE LEVER. exp_cycle measured the same setting at two steps, 150 s:
%   step 0.5  ms : |w| mean 128, ripple 245, tilt 0.9 deg
%   step 0.25 ms : |w| mean  65, ripple   2, tilt 0.8 deg   <- ripple x0.01
%   (friction mu 0.2 -> ripple x0.03; a leak made it worse, mean 533)
% So the visible swaying was a numerical limit cycle, not the control design.
% At 0.25 ms the curve is a smooth decay: 49 -> 66.9 -> 66.2 -> 65.4 -> ...
% -> 58.8 rad/s over 300 s. That is what run_cubli_clean now selects.
%
% PART 2 -- BUT IT DOES NOT CONVERGE, AND CHASING THE STEP IS A TRAP.
% exp_conv, same setting, 150 s, halving the step:
%   0.5   ms -> mean 128, ripple  245, tilt  0.9 deg, upright
%   0.25  ms -> mean  65, ripple    2, tilt  0.8 deg, upright
%   0.125 ms -> mean 348, ripple 1808, tilt 45.9 deg, FELL
% The changes EXPLODE instead of shrinking, so 0.25 ms is a lucky sample of an
% unconverged sequence, not a converged answer. Cause: the mechanism local
% solver is EXPLICIT (ODE3) on a stiff penalty contact. From the measured
% penetration (3.7e-4 m) the contact stiffness is ~1.96e4 N/m, so its natural
% frequency is ~163 rad/s = 26 Hz and an explicit method needs roughly 0.8-1.9
% ms per stable step. The shipped 1 ms sits ON that boundary; that is why every
% step change reshuffles the slow dynamics.
% Turning the local solver OFF was tried and is NOT viable: the adaptive stiff
% solver is forced to ~1 ms anyway (the stiffness sets it) but each step costs
% far more, so a 20 s run reached 3% after several minutes. The local solver is
% the right performance choice.
%
% PART 3 -- THE THING THAT MATTERS MOST IS ABOUT AN ALREADY-ACCEPTED RESULT.
% exp_falltime / exp_ws_hold, 300 s, tilt slope over the last 150 s:
%   fast  @0.5  ms : slope -34.1 deg/100 s, rotor pegged, FELL at ~225 s
%   fast  @0.25 ms : tilt ramps -0.2 -> -14.1 deg, still on the edge at 300 s
%   wstop @0.5  ms : slope  +0.04 deg/100 s, rotor 73-252, upright
%   wstop @0.25 ms : slope  -1.56 deg/100 s, rotor 49-59,  upright
% The fast drift appears at BOTH steps with the same shape, so it is PHYSICAL,
% not a step artefact: the shipped edge_balance is only marginally stable. It
% passes the 10 s gate and then leans and falls in ~4 minutes. The rotor climbs
% with the lean because they are the same exchange (dH/dt = m*g*d*sin(theta)).
% wheel_stop cuts that drift ~20x -- it interrupts the tilt->gravity->rotor
% feedback -- but at 0.25 ms it still drifts, with the increments growing, so
% it is NOT a proven equilibrium either; extrapolating, it also falls
% eventually. What remains is a real slow mode that has to be found and fixed
% in the control, not in the solver and not in a gain.
%
% KiEdge is not a knife-edge -- at 0.5 ms, 1e-5 gives 210 and 1e-4 gives 267
% against the 944 baseline, so the order of magnitude is what matters.
%
% RETRACTED THREE TIMES, all recorded rather than quietly fixed:
%  1. "wheel_stop WORKS" claimed from w_y = 34 rad/s alone. The metric is the
%     three-wheel norm, and with a yaw term the momentum only MOVES into the
%     X/Z pair (Y 971 -> 34, X/Z to +-573, norm peak 2386).
%  2. "yaw kills every dump, and yaw momentum cannot be dumped at all" was
%     drawn at 1 ms, where the yaw is largely numerical (the surviving
%     baseline itself reads -0.3 deg at 1 ms, -18 deg at 0.5 ms, and 2947 deg
%     with a fall at ODE1).
%  3. "the 0.25 ms result is the answer" -- it is where the oscillation goes
%     away, but exp_conv shows the sequence does not converge, so it is a
%     sample, not a converged answer.
%
% WHAT REMAINS OPEN, honestly:
%   - The slow drift is NOT solved, only reduced ~20x. At 0.25 ms the tilt
%     still ramps -0.05 -> -3.19 deg over 300 s with growing increments, so
%     wheel_stop also falls eventually. The residual mode has to be found.
%   - com_x drifts 0 -> 55 mm over 300 s and does not look bounded. Not a fall
%     risk (sliding along the contact line is geometrically survivable).
%   - Whether to move the whole project off 1 ms is still a project decision;
%     it would invalidate every contact-family number recorded at 1 ms.
%
% Things that were tried and did NOT work, recorded so they are not retried:
%   torque deadband (dz 0/0.002/0.005/0.01/0.02, all end 895-999 rad/s) --
%     the pump is not a SMALL torque but one whose TIME AVERAGE is non-zero,
%     and the torque swings large both ways, so a magnitude filter passes the
%     swing with the same net.
%   tilt integral KiTilt (0/0.01/0.03 climb to 913-971; 0.1/0.3 fall) -- so it
%     is not a steady-state DC bias either.
%   attitude tilt as the feedback (fall with Kw>0, with the integral, and with
%     Kw = 0) -- the com loop is the only one that holds the cube up.
%   starting AT the balance attitude (initialTiltDeg = 0) still climbs to 1005,
%     so the climb is not gravity paying for the initial offset.
%   a proportional tilt-reference cascade (attitude tilt, Kw = -2.2e-4) falls
%     within 15 s, against a linearisation that predicted it stable; the
%     reduced (theta, w) model has no translation or contact DOF and this
%     plant's slow dynamics are dominated by them (com_x slides 84 mm in 15 s).
%
% VERTEX: KiPoint stays 0. The vertex already parks at 48 rad/s from a 3 deg
% start with no integral at all, and EVERY non-zero KiP tips it over
% immediately, either sign, down to 1e-4 (com_z to 0.0746, rotor to 3130). The
% two stand-up chains park at 507 and 880 rad/s and fail the same way. The
% vertex is not fixed by this and is recorded as open.
% ---------------------------------------------------------------------------
P.run.wheelStop.KiEdge  = 3e-5;  % N*m*s/rad; needs the contact solver at 0.25 ms
P.run.wheelStop.KiPoint = 0;     % N*m*s/rad; all three wheel commands
% Leak rate on that integral, 1/s: sigma accumulates w - leakRate*sigma, so
% sigma decays whenever the rotor is slow. Measured ineffective for the
% runaway above (see the table); kept only because it does bound the integral.
P.run.wheelStop.leakRate = 0;
% Experiments only: force the edge tilt source regardless of the preset.
% "" follows the preset ("fast" -> com, "wheel_stop" -> attitude). The
% attitude form is atan2(R13,R33) - pi/4, the rotation about the contact edge
% read off the body z axis; at the flat attitude it is exactly -45 deg, the
% same as the com form, so the flat->edge stand-up needs no special case.
P.run.edge.tiltSourceOverride = "";
% Torque deadband on the EDGE balance law, N*m. Default 0 = shipped
% behaviour, bit for bit.
%
% WHY IT EXISTS. Measured 2026-09-27 (exp_nopump, 120 s, shipped config): the
% cube is balanced by 3-5 s with the rotor at only ~150 rad/s, and the climb
% to 1130 happens over the NEXT 40 s. So the rotor speed is not set by the
% entry transient -- the "it freezes at whatever the transient left" story was
% wrong. The climb is a DC torque being integrated into rotor momentum:
%   tau = -Kp*theta_com + Kw*w is a near-cancellation at balance. At t=10 s,
%    -2.2*0.0469 + 2e-4*503.6 = -0.1032 + 0.1007 = -0.0025 N*m,
%   so wdot = -tau/I_w = +25 rad/s^2 against a measured +48..69 -- same order.
% theta_com is supplied by the com_x SLIDE (the true attitude tilt is ~0 the
% whole time), so the loop reads a fake tilt and answers with a DC torque.
%
% The obvious fix -- read the slide-free attitude instead -- was measured and
% FAILS: attitude tilt with Kw>0 runs the rotor to -1665, with the integral
% tips it by 80 s, and with Kw=0 falls flat in 30 s. The com loop is the only
% thing holding the cube up, so the fix has to act on the OUTPUT: refuse to
% command a torque too small to be a real correction, and the residual has
% nothing to integrate into.
%
% COST: the loop cannot command below this value, so the tilt limit-cycles
% within roughly +-deadband/Kp. 0.01 N*m gives ~0.26 deg against the 5 deg
% acceptance gate.
P.run.edge.torqueDeadband = 0;
% Integral on the EDGE TILT, N*m/(rad*s). Default 0 = shipped behaviour.
%
% This is the cure for the rotor pump, and it is a PI term for the textbook
% reason: to remove a steady-state (here, a TIME-AVERAGE) error.
%
% Measured pump (exp_nopump, 120 s, shipped config): the cube balances by 3-5 s
% with the rotor at only ~150 rad/s and the climb to 1130 takes the next 40 s.
% The rotor speed is therefore NOT set by the entry transient. From
% w = -(1/I_w)*integral(tau), the climb is a mean DC torque of 1.65 mN*m --
% equivalent to a COM offset of about 0.23 mm that the law cancels forever.
%
% A torque deadband was measured NOT to fix it (exp_deadband: dz = 0, 0.002,
% 0.005, 0.01, 0.02 all end at 895-999 rad/s). That is the useful part of the
% negative result: the pump is not a SMALL torque, it is a torque whose TIME
% AVERAGE is non-zero, and the torque swings large in both directions -- a
% magnitude filter passes the swing with the same net. Only an integrator
% removes a time average. Its equilibrium is tau = 0 at theta = 0, so the DC
% term vanishes and nothing drives w.
%
% NOT the same as P.run.wheelStop.KiEdge. That one integrates the ROTOR SPEED
% and is the variant that winds up and tips the cube over (see the preset
% note). This one integrates the tilt and its equilibrium is a well-defined
% theta = 0, sigma = -(bias)/KiTilt -- no windup, because the tilt is bounded
% by the balance loop.
P.run.edge.KiTilt = 0;
% YAW damping on the EDGE modes, N*m*s/rad. Part of the "wheel_stop" preset;
% "fast" keeps it at 0 so its verified behaviour is bit-identical.
%
% NOT NEEDED by the verified wheel_stop setting: at 0.5 ms the rotor loop works
% with Kyaw = 0, so wheels X and Z never move. It is kept at 0 and the
% mechanism is retained only because it does damp yaw when it is switched on.
% When it IS on it does not reduce the stored momentum, it relocates it into
% the X/Z pair (Y 971 -> 34 while X/Z go to +-573), so enabling it is not a
% way to get the rotor down -- it was the answer to a problem that turned out
% to be largely numerical.
%
% This is the one gap every failure pointed at. exp_edgemode, 90 s, accumulated
% world-Z rotation: the only configuration that stays up accumulates -0.3 deg,
% and every one that falls accumulates 44-53 deg. The edge law had no yaw
% feedback whatsoever -- only the vertex law has -KyawP*omega(3) -- so the
% actuator was there all along (wheels X and Z) and simply unused.
%
% Geometry, so the sign is not guesswork: at the edge attitude R = RotY(45),
% wheel X's axis is world (0.707, 0, -0.707) and wheel Z's is (0.707, 0,
% 0.707). Their world-Z components are opposite, so driving them with OPPOSITE
% signs cancels world X and leaves a pure world-Z torque. That is the mirror of
% the edge->vertex hop, which drives the same two wheels with the SAME sign to
% get a pure world-X torque.
% MUST be 0 for the verified wheel_stop setting. It was left at 0.3 from the
% earlier yaw attempt, and the preset therefore drove wheels X and Z with a
% persistent world-Z torque -- which winds that pair up and looks nothing like
% "the rotors near zero". The verified runs (exp_fine, exp_fine_long) all set
% Kyaw = 0 explicitly, so they were clean; the PRESET was not, and the first
% hand-run of run_cubli_clean exposed it. Wiring a verified setting into a
% preset is itself a thing to verify, not to assume.
P.run.wheelStop.Kyaw = 0;
% ---------------------------------------------------------------------------
% SEPARATE PRESET "wheel_stop_yaw": the same rotor loop PLUS a gentle X/Z yaw
% hold. Kept apart from "wheel_stop" because it is a different trade, not a
% better version of the same one.
%
% WHY IT IS WORTH HAVING. The cube yaws while balanced at a CONSTANT
% -0.098 deg/s (exp_yawfix: -10.4 / -19.4 / -28.9 / -38.6 / -48.7 / -59.0 deg
% over 600 s, almost perfectly linear, so a constant torque). exp_yawcause
% found what makes it: friction. With mu = 0 the bias collapses to 1.8e-8 N*m
% from 1.3e-5 (700x), and raising mu to 0.9 makes it 7.5x WORSE -- so more
% friction is not a fix, and neither is mass distribution: making the wheels
% effectively massless reproduces the bias bit for bit, and the model's first
% moment is already exactly zero by construction. The disturbance is a fixed
% asymmetry in how the contact's tangential force spreads along the edge.
%
% WHAT IT COSTS, AND THE PART THAT MATTERS FOR READING THE NUMBERS: the
% required TORQUE is tiny (1.3e-5 N*m) but it is PERSISTENT, and a persistent
% torque integrates. So the X/Z speed is set by how long the disturbance lasts,
% not by its size:
%     tau_z 1.28e-5 N*m x 600 s = 7.7e-3 N*m*s -> ~77 rad/s per wheel
% measured: X/Z reach +-84 rad/s at 600 s, growing LINEARLY at 0.157 rad/s per
% second. So they are only 4.7% of the limit at 600 s and would need ~3 hours
% to reach 1800. Fine for any realistic run; NOT a holding solution forever.
%
% MEASURED, 600 s, ODE3 @ 0.25 ms (exp_yawfix):
%   Kyaw       yaw end   yaw rate   tilt end   tilt slope   wX/wZ   |w| end
%    0         -59.0     -0.102     -14.52      -4.64        0       39.9
%   -0.003     -57.2     -0.071     -14.38      -3.50      +-45      78.9
%   -0.005     -36.7     -0.053      -6.09      -1.64      +-84     132.3
%   -0.008     -68.1     -0.004     -45.0 FELL    --       +-96    1811.7
% So -0.005 halves BOTH the yaw drift and the tilt drift at 4.7% of the rotor
% limit, which is what makes it worth offering. -0.008 stops the yaw outright
% but the windup topples the cube first: there is no setting that both stops
% the yaw and keeps the cube up. The SIGN IS NEGATIVE BY MEASUREMENT -- a
% positive sweep made everything worse, the opposite of what the axis geometry
% predicts, the same trap as wheel Y's pose.
% Note exp_yawsrc measured the same bias as -1.83e-2 rad/s^2 against exp_yawcause's
% -4.6e-3, a 4x disagreement between scripts for nominally the same
% configuration, so the MAGNITUDE is not reproducible even though the
% qualitative results (friction-driven, mass-independent) agree. Suspect: the
% controller chart's persistents may not reset between consecutive sim calls on
% one model. Do not quote an absolute bias from these.
% ---------------------------------------------------------------------------
P.run.wheelStop.KyawHold = -0.005;
% ---------------------------------------------------------------------------
% THE CALLBACK: the yaw rate the X/Z pair is asked to HOLD, deg/s. 0 = hold it
% still, which is what wheel_stop_yaw does above.
%
% Holding the yaw rate at exactly zero means the pair carries the whole
% friction-injected yaw momentum, and a persistent torque integrates, so their
% speed grows without bound (0.157 rad/s per second, ~3 h to the limit).
% Tracking a small non-zero rate instead lets the pair unwind into the cube.
% And because friction OPPOSES that rotation, the rotation is what dissipates
% the momentum -- the only yaw sink the system has. This is the yaw analogue of
% leaning to hand pitch momentum to gravity, and like that one it cannot be
% done with no motion at all: the wheel momentum and the cube's rotation are
% the same quantity, related by conservation. An earlier claim that it could be
% had both ways was wrong.
%
% Sign: the uncontrolled drift is NEGATIVE (about -0.1 deg/s), so a callback
% that lets the pair unwind must be negative too.
P.run.wheelStop.YawRateRef = 0;

% Edge-only low-speed candidate, measured with ODE5 @ 0.125 ms. This changes
% only the Y-wheel integral gain; fast/wheel_stop/wheel_stop_yaw remain intact.
% See docs/06 for 300 s stand-to-edge traces and the solver sensitivity.
P.run.edgeLowSpeed.KiEdge = 8e-5;
P.run.edgeLowSpeed.wheelTailMax = 20; % rad/s, final 10 s of a long run
% A separate near-zero candidate accepts a longer capture transient to unload
% the Y wheel more completely. Its validation envelope is in docs/06.
P.run.edgeNearZero.KiEdge = 1e-3;

% Every controller constant, as one vector fed to the controller block from a
% Constant whose value is the workspace expression P.run.ctrl. Baked-in
% literals would mean changing a gain silently did nothing until the .slx was
% regenerated -- which is exactly the trap this project is trying to avoid.
% Derived: after editing any gain, run cubli_refresh_ctrl before simulating.
P = cubli_refresh_ctrl(P);

P.run.startWindow = 0.2;
% Tolerances for the run gates, mirroring the G2A style.
% 5 deg, not the 3 deg first used. Justified, not tuned to pass: the failure
% mode is a fall, which puts the tilt at 45 deg and beyond, so anything in the
% single digits separates "in control" from "lost it" by an order of magnitude.
% A steady lean of ~2-3 deg is also the normal equilibrium of the unloading
% law, not an error (the equilibrium is theta = Kw*w/Kp), so a threshold just
% above it would have made the gate report the controller's own design as a
% failure. The measured stand-up drifts 2.2 -> 2.9 deg over 12 s and passes.
P.run.accept.tiltPeakMaxDeg = 5.0;
P.run.accept.torqueSaturationFrac = 0.5;
% Rotor speed once settled, for the "wheel_stop" preset only. Judged on the
% last stretch of the run, like settledMin, because the dump takes seconds.
% 50 rad/s is 2.8% of the 1800 limit: an order of magnitude below every parked
% value measured on the "fast" preset (504 / 507 / 880 / 1092), so it separates
% "dumped" from "parked" without being a knife edge.
% NOT applied to "fast": that preset exists precisely to be the old behaviour,
% and grading it on a threshold introduced afterwards would retroactively
% un-verify results that were accepted on their own terms.
P.run.accept.rotorEndMax = 50;

P.mode = "geometry";
end

function R = localRotY(a)
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end
