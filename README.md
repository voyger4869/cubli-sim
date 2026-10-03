# Cubli — 3D simulation of a reaction-wheel cube

A physics simulation of a **Cubli** (a cube that balances, and stands itself up, using three
reaction wheels) in MATLAB/Simulink + Simscape Multibody.

Everything runs on **real contact** — a penalty-method `Spatial Contact Force` between the cube
and the ground, three motorised wheels, and measured state. No idealised support joints, no
pre-recorded animation. The cube begins lying flat on the ground and stands up onto its edge
and onto a corner by physically tipping itself over.

> **The documentation is in Chinese.** This README is in English so the project can be found and
> understood; the detailed docs (including every measurement and every failed approach) are in
> [`docs/`](docs/). Start with [`docs/01_项目状态.md`](docs/01_项目状态.md) — the project status.
> The [`architecture and development guide`](docs/05_架构与开发指南.md) explains the build,
> runtime flow, control modes, verification criteria, and development roadmap.
> The [`edge low-speed study`](docs/06_棱平衡低轮速控制设计.md) records the new selectable preset,
> 300 s stand-up run, and solver sensitivity.
> The [`near-zero wheel-speed development log`](docs/07_近零轮速改进开发全过程.md) continues that study
> with each trial, failed approach, measurement correction, and release check.
>
> **On the script names inside those docs.** The docs were written during development and cite
> the experiment that produced each number, e.g. *"出处：`src/experiments/exp_yawcause.m`"*.
> This release ships the **run and verification** code only, so a number of those experiment and
> diagnostic scripts are **not included here** (about 65 names). They exist in the full
> development tree. Every number quoted in the docs was produced by one of them and is
> reproducible from the parameter file; the citations are provenance, not links.

---

## Requirements

- MATLAB **R2024b**
- Simulink, Simscape, **Simscape Multibody**

## Quick start

```matlab
% from the project root
setup_cubli          % put src/ on the path (once per session)
run_cubli_clean      % press Run
```

`run_cubli_clean.m` has two settings at the top; changing them does **not** require regenerating
the model:

```matlab
cubliMode    = "edge_balance";   % which capability
cubliPreset  = "fast";           % which balance law
cubliSeconds = 90;
```

To watch several capabilities run off **one** built model (this is the point of the architecture):

```matlab
cubli_demo
```

## What it does

| Capability | `cubliMode` | Status |
|---|---|---|
| Flat → stand up onto an edge | `stand_to_edge` | **passes** |
| Balance on an edge, 10 s | `edge_balance` | **passes** |
| Balance on a corner (vertex), 10 s | `point_balance` | **passes** (capture band ≈20°) |
| Edge → vertex (the second hop) | `edge_to_point` | **passes** (3 runs, bit-identical) |
| **Flat → edge → vertex → hold** | `flat_to_point` | **passes** — the headline result |
| Flat → vertex in one motion | `flat_to_point_direct` | experimental, not promoted |
| Walking, one face at a time | `walk` | **does not pass** — the gait works, the checks don't |
| Flat → vertex (impulse route) | `stand_to_point` | **fails** — rotor runs to 3129 rad/s |

Balance is selected with `P.run.balancePreset`:

| Preset | Rotor | Notes |
|---|---|---|
| `fast` | climbs to ~950 rad/s, then **freezes** | the verified behaviour |
| `wheel_stop` | settles at 40–60 rad/s | needs the contact solver at 0.25 ms |
| `wheel_stop_yaw` | as above, plus a gentle yaw hold | halves the slow yaw drift |
| `edge_low_speed` | about 6–8 rad/s on the Y wheel from 30–300 s after stand-up | experimental; edge modes only; ODE5 @ 0.125 ms |
| `edge_near_zero` | Y wheel −0.73 to +1.73 rad/s in the final 30 s of a 300 s stand-up run | experimental; 2.28 s capture; edge modes only; ODE5 @ 0.125 ms |

To try the low-speed edge run, set `cubliMode = "stand_to_edge"`,
`cubliPreset = "edge_low_speed"`, and `cubliSeconds = 300` in
`src/core/run_cubli_clean.m`. For a measured pass/fail check, run
`setup_cubli` followed by `verify_edge_low_speed("stand_to_edge",30)`.
For the near-zero candidate, select `cubliPreset = "edge_near_zero"` and run
`verify_edge_near_zero("stand_to_edge",300)`. The contact position and yaw still
drift, and changing the contact solver step changes the measured wheel speed.

## This project documents what does *not* work

Most simulation repos show only the happy path. This one keeps the failures, with numbers:

- **[`docs/03_未解决与已证伪.md`](docs/03_未解决与已证伪.md)** — what is still unsolved, every
  approach that was tried and falsified, and a record of **three conclusions that were reported
  and then retracted** (each one because a measurement was read the wrong way).
- The balance presets differ because `fast` **cannot** hold the rotor near zero:
  a proportional law drives its own torque to zero at the balance point, so the rotor freezes
  wherever the entry transient left it. That is a conservation argument, not a tuning problem.

Three things are worth knowing before you read any number here:

1. **The plant is not numerically converged.** The historical ODE3 configuration falls when
   halving the contact step from 0.25 to 0.125 ms. The new ODE5 edge preset stays upright at
   0.25, 0.125, and 0.0625 ms for 120 s, but its wheel speed and yaw are still step sensitive.
   See [the measurements](docs/06_棱平衡低轮速控制设计.md).
2. **Every contact-related number must be quoted with its solver setting.** Ideal-joint and
   contact models in this project run on different integrators and are not comparable.
3. **The balance drifts.** It is stable for minutes, not indefinitely — the *tilt* reaches its
   limit before the rotor speed does.

## Repository layout

```
.
├── README.md                     this file
├── LICENSE
├── setup_cubli.m                 path setup (run once per session)
├── cubli_root.m                  locates the project root for every script
├── src/
│   ├── core/                     parameters, mode dispatch, the plant builder, the report,
│   │                             run_cubli_clean.m (entry) and cubli_demo.m
│   └── verification/             acceptance and regression checks
└── docs/                         Chinese: status, evidence, failures, measurements
    └── history/                  earlier stage documents, archived verbatim
```

Generated files (`.slx`, `.slxc`, `slprj/`, snapshot `.mat`) are gitignored — see
[.gitignore](.gitignore). The model is rebuilt from the parameter file on every run.

**The parameter file is the documentation.** [`src/core/cubli_clean_parameters.m`](src/core/cubli_clean_parameters.m)
carries a written justification next to every value — why it is what it is, the measurement it came
from, and which earlier conclusion it replaced. If you want to change behaviour, that is the file
to read and the only file you need to edit.

### Verification

```matlab
verify_cubli                 % every mode, with the acceptance verdict
verify_walk_directions       % the four walking directions, checked by displacement
verify_preset_snapshot("a"); ...; verify_preset_snapshot("b");
verify_preset_compare("a","b")   % bit-exact by default
```

`verify_cubli` is expected to report two failures (`walk`, `stand_to_point`), both documented.
`verify_preset_compare` defaults to a tolerance of **zero**: the verified preset is supposed to be
untouched by edits that do not mean to touch it.

## Design notes

- **One plant, many modes.** A single model serves every capability; the mode, the initial
  condition, the stop time and all control constants are read as workspace expressions, so
  switching capability is a variable change, not a rebuild.
- **Generated files are not committed.** The `.slx` is built by [`build_cubli_clean_cubli.m`](src/core/build_cubli_clean_cubli.m)
  on every run and is regenerated from the parameter file.
- **The report judges the quantity that the controller controls**, and says so: for the edge it
  uses `asin(com_x/d)`, the same reconstruction the law uses — while noting that on this plant
  that quantity measures *sliding* as much as tilting, so the attitude-derived tilt is computed
  and printed alongside it.

## License

MIT — see [LICENSE](LICENSE).
