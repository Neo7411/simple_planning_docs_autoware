# SIMPLANNER_DOCS_CLAUDE — `autoware_simple_planning_simulator` Detailed Documentation

This folder contains a **detailed, file-by-file walk-through** of the
[`autoware_simple_planning_simulator`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/)
ROS 2 package — the lightweight 2D vehicle-motion simulator that Autoware uses to integration-test
its planning and control stacks.

The documentation is split per top-level folder of the package. Each sub-document explains
**every file** in its folder: purpose, public API, internal structures, the math/algorithm
implemented, and how it plugs into the larger node.

---

## What this package does (in one paragraph)

`autoware_simple_planning_simulator` is a single ROS 2 node that **takes vehicle commands on
its input topics, integrates a configurable vehicle dynamics model forward in time, and publishes
the resulting odometry / steering / TF / IMU / acceleration / gear / control-mode / indicator
state** as if the ego vehicle were really driving. It does **not** simulate sensing, perception,
or collisions, runs on CPU only, and is implemented in pure C++ (with optional Python hooks for
learned models).

---

## Documentation Index

| # | Folder | What's inside | Doc |
|---|--------|----------------|------|
| 1 | [`include/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/) | Public C++ headers (node + 11 vehicle models + utils + visibility macros) | [include.md](include.md) |
| 2 | [`src/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/) | C++ implementations (core node, vehicle models, utilities) | [src.md](src.md) |
| 3 | [`launch/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/launch/) | ROS 2 launch entry-point | [launch.md](launch.md) |
| 4 | [`param/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/) | Default parameter YAML files + acceleration map | [param.md](param.md) |
| 5 | [`data/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/data/) | Bundled actuation maps (accel/brake/steer CSVs) | [data.md](data.md) |
| 6 | [`test/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/) | gtest harness exercising every vehicle model | [test.md](test.md) |
| 7 | [`docs/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/docs/) | Upstream package description (handy reference) | [docs.md](docs.md) |
| 8 | [`media/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/) | SVG/PNG diagrams used by upstream README | [media.md](media.md) |

Top-level package files (`CMakeLists.txt`, `package.xml`, `README.md`, `CHANGELOG.rst`) are
covered in [`overview.md`](overview.md).

---

## Reading order suggestion

1. **Start with [overview.md](overview.md)** — high-level architecture, the timer loop, ROS API.
2. **Read [include.md](include.md)** — every header file, type-by-type. Defines the contract
   that the implementation must satisfy.
3. **Read [src.md](src.md)** — the matching `.cpp` files, including the math behind each
   vehicle model.
4. **Glance at [param.md](param.md) and [launch.md](launch.md)** to see how the node is
   actually configured and started.
5. **Skim [test.md](test.md)** to see how each vehicle model is verified.
6. **[data.md](data.md), [docs.md](docs.md), [media.md](media.md)** — supplementary content.

---

## Quick orientation: how the system fits together

```
                ┌─────────── ROS 2 commands (Ackermann or Actuation) ───────────┐
                │                                                                │
                ▼                                                                │
   ┌─────────────────────────────────────────────────────────────────┐           │
   │                    SimplePlanningSimulator (Node)               │           │
   │                                                                  │          │
   │  on_timer() @ 40 Hz                                              │          │
   │     ├─ calculate_ego_pitch()          (lanelet map)              │          │
   │     ├─ set_input(cmd, acc_by_slope)   (translate ROS → state vec)│          │
   │     ├─ vehicle_model_ptr_->update(dt) (integrate dynamics)       │          │
   │     ├─ get_z_pose_from_trajectory(...)                           │          │
   │     ├─ add_measurement_noise(...)                                │          │
   │     └─ publish_odometry/pose/twist/tf/imu/accel/steer/...        │          │
   │                                                                  │          │
   │  ┌───────────────── SimModelInterface (abstract) ───────────────┐│          │
   │  │  state_  Eigen::VectorXd   x = [x,y,yaw, ...]                ││          │
   │  │  input_  Eigen::VectorXd   u = [v|a, δ, ...]                 ││          │
   │  │  update(dt)            ← pure virtual                        ││          │
   │  │  calcModel(state, u)   ← pure virtual                        ││          │
   │  │  updateRungeKutta / updateEuler                              ││          │
   │  └──────────────────────────────────────────────────────────────┘│          │
   │      ▲                                                           │          │
   │      │  one of 11 concrete subclasses                             │          │
   │      │   IDEAL_STEER_{VEL,ACC,ACC_GEARED}                         │          │
   │      │   DELAY_STEER_{VEL,ACC,ACC_GEARED,ACC_GEARED_WO_FALL_GUARD}│          │
   │      │   DELAY_STEER_MAP_ACC_GEARED   (+ acceleration_map.csv)    │          │
   │      │   LEARNED_STEER_VEL            (Python hook)               │          │
   │      │   ACTUATION_CMD{,_STEER_MAP,_VGR,_MECHANICAL}              │          │
   │      │     (uses csv_loader + mechanical_controller)              │          │
   └──────┴──────────────────────────────────────────────────────────┘
                                     │
                                     ▼
                ┌────── ROS 2 vehicle state outputs ────────┐
                │ /tf, /output/odometry, /output/pose,       │
                │ /output/twist, /output/imu, /output/accel, │
                │ /output/steering, /output/gear_report,     │
                │ /output/control_mode_report, indicators,   │
                │ /output/actuation_status                   │
                └────────────────────────────────────────────┘
```

---

## Conventions used in these docs

- **File links** are relative paths (`../src/.../file.cpp`). Click in any markdown viewer that
  resolves them.
- **Line citations** (`file.cpp:123`) point at a specific line in the source.
- **State / input vector layouts** are shown as e.g. `state = [x, y, yaw, vx, steer, ax]`,
  matching the `enum IDX` and `enum IDX_U` defined inside each model.
- A **"§"** reference (e.g. "see §6") refers to a section inside the same doc, not Unicode
  paragraph numbering.

---

## Contents at a glance

```
SIMPLANNER_DOCS_CLAUDE/
├── README.md          ← you are here
├── overview.md        ← package goals, build, top-level files, ROS API
├── include.md         ← every header file
├── src.md             ← every .cpp file
├── launch.md          ← launch/simple_planning_simulator.launch.py
├── param.md           ← param/*.yaml + param/acceleration_map.csv
├── data.md            ← data/actuation_cmd_map/*.csv
├── test.md            ← test/test_simple_planning_simulator.cpp + maps
├── docs.md            ← docs/package_description.md
└── media.md           ← media/*.svg / *.png
```
