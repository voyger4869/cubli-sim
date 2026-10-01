# Cubli clean rebuild: fixed requirements and acceptance gates

This is a fresh project.  It must not load, copy, or edit `Cubli_Walk.slx`,
`Cubli_Physical.slx`, or any earlier generated Cubli model.

## Fixed plant assumptions

| Item | Value |
|---|---:|
| Cube shell mass | 0.50 kg |
| Cube edge length | 0.15 m |
| Each reaction-wheel mass | 0.08 kg |
| Each wheel radius | 0.05 m |
| Each wheel thickness | 0.01 m |
| Motor torque limit | 0.06 N m (editable) |
| Wheel speed limit | 1800 rad/s (editable) |
| Simulation environment | MATLAB R2024b + Simscape Multibody |

## Non-negotiable behavior

1. The visualization is a Mechanics Explorer animation driven by the actual
   Simscape Multibody mechanism.  Plot-only or scripted animation is not a
   replacement.
2. The model begins flat on the ground.  It must not begin constrained at an
   edge or vertex while claiming to stand up.
3. A selected mode performs a physical transition from face contact to edge
   contact, then holds an edge balance for at least 10 seconds.
4. Point balance and walking are separate selectable modes, not time windows
   hidden in one fixed demonstration.
5. Ground contact has one normal convention, one coordinate convention
   (world Z up), and an explicit penetration check.  A cube crossing the
   ground invalidates the run.

## Build sequence

| Gate | Deliverable | Pass criterion | Not included yet |
|---|---|---|---|
| G0 | Geometry and coordinate test | Cube, ground, and three wheel axes render at correct locations; Z is up | control or contact |
| G1 | Single-axis edge rig | One reaction wheel stabilizes a 1 degree edge perturbation for 10 s; no ground contact is used in this rig | stand-up maneuver |
| G2A | Passive face contact | Free 6-DOF cube settles flat on the ground with no penetration or drift | motor dynamics |
| G2B | Wheel bench | Rigidly mounted flat cube, one true internal wheel spins and logs speed/torque | ground reaction |
| G2C | Face-to-edge maneuver | Free 6-DOF cube starts flat, makes one intentional face-to-edge transition, and remains on the physical side of the ground plane | point/walking |
| G3 | Point rig | Three wheels regulate the two unstable tilt coordinates for 10 s; ideal vertex-support result passed on 2026-09-26 | free point contact |
| G4 | Walking state machine | Repeated eight-step contact transitions with recoverable state and selectable direction | none |

A gate is not promoted merely because a model opens or simulates.  It must
meet its stated criterion in Mechanics Explorer and have a recorded result.

## Architecture rules

- `cubli_clean_parameters.m` is the only physical-parameter source.
- `build_cubli_clean.m` builds G0 geometry; stage-specific
  `build_cubli_clean_*.m` functions build independent validation models.
- `run_cubli_clean.m` selects an explicit mode. G0, G1, G2A/B and G3C
  ideal-support point balance have user-reported validation; G2C long-run
  contact and G4 walking remain open.
- Controllers receive named, measured states; they must not depend on an
  undocumented Demux ordering or Euler-angle convention from a reference
  model.
- The edge and point rigs may use ideal supports only to validate control;
  the stand-up and walking modes must use free-body contact.
