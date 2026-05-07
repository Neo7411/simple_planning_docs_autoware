# `param/` — Parameter & Default-Map Files

```
param/
├── simple_planning_simulator_default.param.yaml
├── simple_planning_simulator_mechanical_sample.param.yaml
├── vehicle_characteristics.param.yaml
└── acceleration_map.csv
```

These YAMLs supply default values for the node's many ROS parameters. Concrete numbers in
the tables below come straight from the files; for the full **range of parameters per
model**, see the upstream
[`README.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md).

The simulator's launcher passes one of these YAMLs (chosen via the
`simulator_model_param_file` launch argument) into the node, layered on top of vehicle-info
parameters and the vehicle-characteristics file. See [`launch.md`](launch.md).

---

## 1. [`simple_planning_simulator_default.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml)

The default config. Selects `DELAY_STEER_ACC_GEARED`. Reasonable for the majority of
integration tests.

```yaml
/**:
  ros__parameters:
    simulated_frame_id: "base_link"
    origin_frame_id: "map"
    vehicle_model_type: "DELAY_STEER_ACC_GEARED"
    initialize_source: "INITIAL_POSE_TOPIC"
    timer_sampling_time_ms: 25
    add_measurement_noise: False
    vel_lim: 30.0
    vel_rate_lim: 30.0
    steer_lim: 0.6
    steer_rate_lim: 6.28
    acc_time_delay: 0.1
    acc_time_constant: 0.1
    steer_time_delay: 0.1
    steer_time_constant: 0.1
    steer_dead_band: 0.0
    x_stddev: 0.0001
    y_stddev: 0.0001
    enable_road_slope_simulation: true
```

| Field | Notes |
|---|---|
| `simulated_frame_id`, `origin_frame_id` | TF naming. The launcher's default is `odom` for the origin; this YAML overrides it to `map`. |
| `vehicle_model_type` | Selects which `SimModelInterface` subclass is instantiated in `initialize_vehicle_model()`. |
| `initialize_source` | `"INITIAL_POSE_TOPIC"` makes the node wait for an `input/initialpose` message before stepping the model. The other valid value is `"ORIGIN"` which seeds at `(0,0,0)`. |
| `timer_sampling_time_ms` | Period of the `on_timer` callback in milliseconds. 25 ms ⇒ 40 Hz. |
| `add_measurement_noise` | If true, Gaussian noise is added to position, velocity, RPY, and steering before publishing. |
| `vel_lim`, `vel_rate_lim` | Velocity and longitudinal-acceleration clamps. |
| `steer_lim`, `steer_rate_lim` | Steering-tire angle clamp [rad] and angular-rate clamp [rad/s]. |
| `acc_time_delay` | Dead-time before an acc command starts to take effect (queue length = round(delay/dt) entries). |
| `acc_time_constant` | 1st-order time constant for acceleration dynamics. Floored to 0.03 s by `MIN_TIME_CONSTANT`. |
| `steer_time_delay`, `steer_time_constant` | Same for steering. |
| `steer_dead_band` | Dead band [rad] around the steer error — below this the model ignores the difference. |
| `x_stddev`, `y_stddev` | Diagonal terms written into the published odometry covariance to satisfy downstream EKFs. |
| `enable_road_slope_simulation` | Enables `acc_by_slope = -9.81 * sin(slope_angle)` based on `calculate_ego_pitch()`. |

The commented-out `acceleration_map_path` is a reminder that this default model
(`DELAY_STEER_ACC_GEARED`) does **not** need the CSV — it would be required only for the
`DELAY_STEER_MAP_ACC_GEARED` model.

---

## 2. [`simple_planning_simulator_mechanical_sample.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml)

A **fully-populated example** for `vehicle_model_type: ACTUATION_CMD_MECHANICAL`. Useful as a
copy-paste starting point for vehicle-specific configs.

It selects:

```yaml
vehicle_model_type: "ACTUATION_CMD_MECHANICAL"
initialize_source: "INITIAL_POSE_TOPIC"
timer_sampling_time_ms: 25
enable_road_slope_simulation: True
```

then sets the same kind of common limits (`vel_lim`, `steer_lim`, etc.), and adds the
**actuation- and mechanical-specific** sections:

```yaml
accel_time_delay: 0.1
accel_time_constant: 0.1
brake_time_delay: 0.1
brake_time_constant: 0.1
accel_map_path: $(find-pkg-share simple_planning_simulator)/data/actuation_cmd_map/accel_map.csv
brake_map_path: $(find-pkg-share simple_planning_simulator)/data/actuation_cmd_map/brake_map.csv
convert_accel_cmd: true
convert_brake_cmd: true

convert_steer_cmd_method: "vgr"     # selects VGR-style steer conversion in raw_vehicle_cmd_converter
vgr_coef_a: 15.713
vgr_coef_b: 0.053
vgr_coef_c: 0.042
enable_pub_steer: true

mechanical_params:
  kp: 386.9151820510161
  ki: 5.460970982628869
  kd: 0.03550834077602694
  ff_gain: 0.03051963576179274
  angle_limit: 10.0
  rate_limit: 3.0
  dead_zone_threshold: 0.00708241866710033
  poly_a: 0.15251276182076065
  poly_b: -0.17301900674117585
  poly_c: 1.5896528355739639
  poly_d: 0.002300899817071436
  poly_e: -0.0418928856764797
  poly_f: 0.18449047960081838
  poly_g: -0.06320887302605509
  poly_h: 0.18696796150634806
  inertia: 25.17844747941984
  damping: 117.00653795106054
  stiffness: 0.17526182368541224
  friction: 0.6596571248682918
  vgr_coef_a: 2.4181735349544224
  vgr_coef_b: -0.013434076966833082
  vgr_coef_c: -0.033963661615283594
  steering_torque_limit: 30.0
  torque_delay_time: 0.0007641271506616108
```

The numbers are **system-identification results** (see
[`media/mechanical_controller_system_identification.png`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/mechanical_controller_system_identification.png)
for the procedure used to obtain them). They map 1:1 onto the `MechanicalParams` struct
populated in `initialize_vehicle_model` ⇒ `SimModelActuationCmdMechanical` ⇒
`MechanicalController`.

| Group | Fields | Used in |
|---|---|---|
| PID | `kp, ki, kd` | `PIDController::compute(...)` |
| Feed-forward | `ff_gain` | `feedforward(...)` |
| Limits | `angle_limit, rate_limit, steering_torque_limit` | `apply_limits(...)`, polynomial clamp |
| Dead-zone | `dead_zone_threshold` | `SteeringDynamics::is_in_dead_zone(...)` |
| Polynomial | `poly_a..poly_h` | `polynomial_transform(τ, v, …)` — calibrated 8-coef polynomial |
| Mechanical | `inertia, damping, stiffness, friction` | `SteeringDynamics::calc_model(...)` |
| VGR (mechanical) | `vgr_coef_a, b, c` | overrides the top-level VGR coefficients **inside** the mechanical block (the SteeringDynamics' own VGR sub-model) |
| Delay | `torque_delay_time` | `delay(...)` queue inside the controller |

> Note: the field `delay_time` listed in `MechanicalParams` is NOT set in this YAML —
> only `torque_delay_time` is. The unused `delay_time` defaults to 0.

---

## 3. [`vehicle_characteristics.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/vehicle_characteristics.param.yaml)

```yaml
/**:
  ros__parameters:
    cg_to_front_m: 1.228
    cg_to_rear_m: 1.5618
    front_corner_stiffness: 17000.0
    rear_corner_stiffness: 20000.0
    mass_kg: 1460.0
    yaw_inertia_kgm2: 2170.0
    width_m: 2.0
    front_overhang_m: 0.5
    rear_overhang_m: 0.5
    max_steer_angle: 0.70
```

Vehicle physical characteristics. **None of these are read directly by the
simple_planning_simulator's source code** — the package's own models use only `wheelbase`
(coming from `VehicleInfoUtils`) and the limits/time-constants in the simulator-model YAML.
The values here exist for two reasons:

1. **Documentation / consistency** — they match the values used by other simulators (e.g.
   the dynamic planner-control simulator), so an experiment can swap simulator backends with
   the same vehicle.
2. **Future tire-slip extension** — `front/rear_corner_stiffness`, `mass_kg`, and
   `yaw_inertia_kgm2` are required for the bicycle-with-tire-slip dynamics model that
   future simulators may incorporate.

---

## 4. [`acceleration_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/acceleration_map.csv)

A **minimal placeholder** acceleration map for `DELAY_STEER_MAP_ACC_GEARED`:

```
default,0.00,20.0
-4,-4,-4
0,0,0
4,4,4
```

Layout:

| | col 1 | col 2 |
|---|---|---|
| | velocity = 0 m/s | velocity = 20 m/s |
| **acc_des = -4** | -4 | -4 |
| **acc_des = 0** | 0 | 0 |
| **acc_des = +4** | 4 | 4 |

It's a pure pass-through — desired acc → realised acc = desired acc, regardless of velocity.
**This is intentionally trivial** so the model behaves like `DELAY_STEER_ACC_GEARED` if no
vehicle-specific map is supplied. Real vehicle launches override this argument with a
calibrated map; the upstream README has a full 19-row × 13-column example reproduced here:

![acceleration_map](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/acceleration_map.svg)

The file is consumed by `AccelerationMap::readAccelerationMapFromCSV(path)` (declared in
[`include/.../sim_model_delay_steer_map_acc_geared.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_map_acc_geared.hpp))
through `CSVLoader` → `getColumnIndex` (velocities) + `getRowIndex` (acc_des values) +
`getMap` (the table). The `default` token in cell `[0][0]` is ignored.

`AccelerationMap::getAcceleration(acc_des, vel)` then performs:

1. **Clamp** `vel` to the column range and `acc_des` to the row range
   (via `CSVLoader::clampValue(...)`).
2. For each row, **linearly interpolate** along the velocity axis at `vel` →
   `interpolated_acc_vec` (one entry per row).
3. **Linearly interpolate** that vector along the acc_des axis at `acc_des` ⇒ output acc.

This forms a **bilinear lookup** on the vehicle's acceleration response surface.

---

## 5. Cross-references

- The actual classes that read these YAMLs are in [`src.md`](src.md) §1.2 (constructor) and
  §1.3 (`initialize_vehicle_model`).
- The `accel_map_path` / `brake_map_path` keys (used by `ACTUATION_CMD*` models) point at
  files in [`data/`](data.md) — which has its own doc.
- The launcher that wires these YAMLs into the node is described in [`launch.md`](launch.md).
