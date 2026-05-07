# `media/` — Diagrams & Images

```
media/
├── acceleration_map.svg
├── mechanical_controller.drawio.svg
├── mechanical_controller_system_identification.png
├── pitch-calculation.drawio.svg
└── vgr_sim.drawio.svg
```

This folder contains five visualisations referenced from the upstream
[`README.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md)
and from
[`docs/package_description.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/docs/package_description.md).
None of them is consumed by code; they are documentation assets only.

---

## 1. [`acceleration_map.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/acceleration_map.svg)

A 3D / heatmap visualisation of the example acceleration map shown in the README:

- **X axis** — current velocity (m/s)
- **Y axis** — desired acceleration command (m/s²)
- **Cell value** — actual acceleration produced (m/s²)

It illustrates the bilinear lookup performed by `AccelerationMap::getAcceleration(acc_des, vel)`
([include.md §5.1](include.md#51-helper-classes-embedded-in-two-of-those-headers)).

The picture is referenced from the README right after the example CSV listing:

> ![acceleration_map](./media/acceleration_map.svg)

---

## 2. [`mechanical_controller.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/mechanical_controller.drawio.svg)

A drawio-source SVG of the **steering-column mechanical model** used by
`SimModelActuationCmdMechanical`. It shows the data flow of `MechanicalController::run_one_step`:

```
target_steer ──▶ apply_limits ──▶ feedforward ─────────┐
                                                       ▼
                                       Σ ──▶ polynomial_transform ──▶ delay ──▶ SteeringDynamics ──▶ steer_tire
                                       ▲
                                       │
state.pos ──▶ pid_error  ──▶ PIDController ─────────┘
```

It also visualises the dead-zone branch (input torque pointing opposite to the column's
velocity, below threshold) and how the output of `SteeringDynamics::calc_model` feeds into
the RK4 integrator.

The corresponding code is in
[`utils/mechanical_controller.{hpp,cpp}`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/utils/mechanical_controller.hpp)
— see [`include.md` §7](include.md#7-utilsmechanical_controllerhpp) and [`src.md` §5](src.md#5-utilsmechanical_controllercpp).

---

## 3. [`mechanical_controller_system_identification.png`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/mechanical_controller_system_identification.png)

A bitmap explaining the **system-identification procedure** used to obtain the
`mechanical_params.*` numbers shipped in
[`simple_planning_simulator_mechanical_sample.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml).

It typically shows:

- The data-collection setup (recorded steering-wheel input vs. measured tire angle).
- The optimisation / least-squares fit producing the polynomial coefficients
  (`poly_a..h`).
- The PID and `inertia/damping/stiffness/friction` tuning targets.

If you need to identify these values for a new vehicle, this PNG is the **methodology
reference** — and the YAML constants in `mechanical_sample` are the **numerical example**.

---

## 4. [`pitch-calculation.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/pitch-calculation.drawio.svg)

Visual for `SimplePlanningSimulator::calculate_ego_pitch()` (described in
[`src.md` §1.5](src.md#15-calculate_ego_pitch-line-426)).

Shows two cases:

1. **Ego aligned with lanelet direction** — the slope is computed from the centerline's
   `(diff_z, diff_xy)`, with `diff_xy` projected onto the ego's body x via
   `cos(ego_yaw_against_lanelet)`.
2. **Ego against lanelet direction** — the sign of the result is flipped, but the README
   notes this is **not officially supported**; it's only correct in the magnitude.

Referenced from the README's "Caveat" section at the bottom of the parameter discussion.

---

## 5. [`vgr_sim.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/vgr_sim.drawio.svg)

The **variable-gear-ratio** model visualisation, used by both
`SimModelActuationCmdVGR` and (transitively, via inheritance) `SimModelActuationCmdMechanical`.

It typically shows:

- The relationship between **steer-wheel** angle, **steer-tire** angle, and the
  velocity-dependent gear ratio.
- The math implemented in `calculateSteeringWheelState`,
  `calculateVariableGearRatio`, and `calculateSteeringTireCommand`
  (see [`src.md` §3.9.4](src.md#394-simmodelactuationcmdvgr)).

The non-linear divisor `1.0 + c * |steer|` and the velocity dependence `a + b*v²` give the
visualisation its characteristic curved-ratio shape.

---

## 6. Cross-references

| Diagram | Used to explain… |
|---|---|
| `acceleration_map.svg` | The CSV in [`param/acceleration_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/acceleration_map.csv) and the `AccelerationMap` class. |
| `mechanical_controller.drawio.svg` | The PID + steering-dynamics pipeline of `SimModelActuationCmdMechanical`. |
| `mechanical_controller_system_identification.png` | How the `mechanical_params.*` YAML numbers were obtained. |
| `pitch-calculation.drawio.svg` | `calculate_ego_pitch()` and the slope-induced acceleration term. |
| `vgr_sim.drawio.svg` | The variable-gear-ratio steering conversion in `SimModelActuationCmdVGR`. |

Editable sources for the `*.drawio.svg` files can be opened directly with
<https://app.diagrams.net> (or the VS Code Draw.io extension) — they are valid SVG with
embedded drawio metadata.
