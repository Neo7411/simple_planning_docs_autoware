# Overview — `autoware_simple_planning_simulator`

This document covers the **top-level package files** and the **high-level architecture**:
the build system, package metadata, the README, and how the node is registered with ROS 2.

For the per-folder file walkthroughs, see the sibling docs in this directory.

---

## 1. Top-level files

### 1.1 [`package.xml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/package.xml)

ROS 2 `package format=3` manifest. Highlights:

- **Name:** `autoware_simple_planning_simulator`
- **Version:** `0.50.0`
- **License:** Apache-2.0
- **Build tools:** `ament_cmake_auto`, `autoware_cmake`
- **Runtime depends** (selection):
  - Autoware messages: `autoware_control_msgs`, `autoware_vehicle_msgs`,
    `autoware_planning_msgs`, `autoware_map_msgs`
  - Autoware utilities: `autoware_motion_utils`, `autoware_utils_geometry`,
    `autoware_utils_rclcpp`, `autoware_vehicle_info_utils`,
    `autoware_lanelet2_extension`, `autoware_lanelet2_utils`,
    `autoware_learning_based_vehicle_model`
  - Tier IV messages: `tier4_api_utils`, `tier4_external_api_msgs`, `tier4_vehicle_msgs`
  - Standard ROS 2: `rclcpp`, `rclcpp_components`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`,
    `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `lanelet2_core`
- **Test depends:** `ament_lint_auto`, `autoware_lint_common`, `ament_cmake_ros`, `ament_index_cpp`
- **Maintainers:** TIER IV team (Horibe, Kimura, Clement, Sobue, Azmi, Kem, Sasaki, Yoshimoto)

### 1.2 [`CMakeLists.txt`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/CMakeLists.txt)

A short build script:

- `find_package(autoware_cmake)` and `find_package(autoware_learning_based_vehicle_model)` —
  the second is needed because `SimModelLearnedSteerVel` calls into Python through that library.
- `find_package(Python3 COMPONENTS Interpreter Development)` — pulls the Python headers and
  interpreter required by the learned-model hook.
- `ament_auto_add_library(${PROJECT_NAME} SHARED ...)` builds **one shared library** containing:
  - the node ([`simple_planning_simulator_core.{hpp,cpp}`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp)),
  - the visibility-control header,
  - all 11 vehicle-model `.cpp` files,
  - the two helper utilities (`csv_loader.cpp`, `mechanical_controller.cpp`).
- `target_include_directories(... PUBLIC ${Python3_INCLUDE_DIRS} ${autoware_learning_based_vehicle_model_INCLUDE_DIRS})`
  exposes Python and the learned-model lib publicly (so dependents can link against the model).
- `rclcpp_components_register_node(...)` registers the **component plugin**
  `autoware::simulator::simple_planning_simulator::SimplePlanningSimulator` and produces an
  executable named `autoware_simple_planning_simulator_node` that loads it.
- `if(BUILD_TESTING) ament_add_ros_isolated_gtest(test_simple_planning_simulator ...)` builds
  the gtest under [`test/test_simple_planning_simulator.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp)
  with a 120 s timeout.
- `ament_auto_package(INSTALL_TO_SHARE param data launch test)` installs those four directories
  into the package share folder so `ament_index` lookups (used by tests and launch files) work.

### 1.3 [`README.md`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md)

The upstream README (kept verbatim by the project). It documents:

- **Purpose / Use cases** — 2D motion sim for planning/control integration testing, no GPU,
  no perception.
- **Inputs / Outputs / API** — every topic and its message type.
- **Common parameters** — frame ids, noise stddevs, initialize source.
- **Vehicle-model parameters** — a big table mapping which knobs apply to which model
  (`I_ST_V`, `I_ST_A`, `D_ST_A`, …).
- **`acceleration_map.csv`** — example contents and the linear-interpolation rule.
- **`LEARNED_STEER_VEL` example** — list of three Python paths/classes.
- **Default TF configuration** — `odom → base_link` is the simulator's job, `map → odom` comes
  from a launch-level relay.
- **Pitch caveat** — driving against the lanelet's tangent direction is unsupported.
- **References** — Autoware.AI's `wf_simulator` (the spiritual ancestor).

### 1.4 [`CHANGELOG.rst`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/CHANGELOG.rst)

ROS-style change log auto-maintained by `catkin_generate_changelog` / `bloom-release`. Read for
a release-by-release summary of behavioural changes.

---

## 2. The class registered with ROS

```cpp
// src/simple_planning_simulator/simple_planning_simulator_core.cpp:979
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::simulator::simple_planning_simulator::SimplePlanningSimulator)
```

This means the node can be:

- launched standalone via the `autoware_simple_planning_simulator_node` executable
  (`launch/simple_planning_simulator.launch.py` does exactly this), **or**
- loaded into a `component_container` together with other Autoware components for zero-copy
  intra-process communication.

The class itself is declared in
[`include/autoware/simple_planning_simulator/simple_planning_simulator_core.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/simple_planning_simulator_core.hpp)
and lives under the namespace `autoware::simulator::simple_planning_simulator`.

---

## 3. End-to-end flow (one tick of `on_timer()`)

```
┌────── ROS subscriptions update internal state members ───────┐
│  current_ackermann_cmd_  (or current_input_command_ variant)  │
│  current_gear_cmd_, manual variants, indicators, hazards      │
│  current_trajectory_ptr_, road_lanelets_  (cached)            │
└───────────────────────────────────────────────────────────────┘
                              │
                              ▼
   on_timer()                 (timer at timer_sampling_time_ms, default 25 ms)
     │
     ├── if (!is_initialized_) → publish only control_mode_report and bail
     │
     ├── ego_pitch_angle = calculate_ego_pitch()        // lanelet lookup
     │   acc_by_slope    = -9.81 * sin(-ego_pitch)      // gravity along x
     │
     ├── dt = delta_time_.get_dt(now)                   // wall-clock based
     │
     ├── pick AUTONOMOUS vs MANUAL command, set gear, call set_input(cmd, acc_by_slope)
     │     - Control msg     → [vel, steer]  or  [combined_acc, steer]  or  [acc, gear, slope, steer]
     │     - ActuationCmd    → [accel, brake, slope, steer, gear]
     │
     ├── if (simulate_motion_) vehicle_model_ptr_->update(dt);  // integrate
     │
     ├── current_odometry_ = to_odometry(model, ego_pitch);
     │   current_odometry_.pose.pose.position.z = get_z_pose_from_trajectory(...);
     │   current_velocity_ = to_velocity_report(model);
     │   current_steer_    = to_steering_report(model);
     │
     ├── if (add_measurement_noise_) add_measurement_noise(odom, vel, steer);
     │
     ├── pose covariance ← x_stddev_, y_stddev_       // dummy for downstream EKF
     │
     └── publish_*  ← odometry, pose, velocity, accel, imu,
                       control_mode, gear, indicators, hazards, tf,
                       (optional) steering, (optional) actuation_status
```

---

## 4. ROS interface (concise)

### 4.1 Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `input/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial vehicle pose |
| `input/initialtwist` | `geometry_msgs/TwistStamped` | Initial velocity |
| `input/ackermann_control_command` | `autoware_control_msgs/Control` | Autonomous control command |
| `input/manual_ackermann_control_command` | `autoware_control_msgs/Control` | Manual driver |
| `input/actuation_command` | `tier4_vehicle_msgs/ActuationCommandStamped` | Pedal/steer (only `ACTUATION_CMD*`) |
| `input/gear_command`, `input/manual_gear_command` | `autoware_vehicle_msgs/GearCommand` | Gear |
| `input/turn_indicators_command`, `input/hazard_lights_command` | …Command | Lighting |
| `input/vector_map` | `autoware_map_msgs/LaneletMapBin` | For pitch/slope |
| `input/trajectory` | `autoware_planning_msgs/Trajectory` | For Z lookup |
| `input/engage` | `autoware_vehicle_msgs/Engage` | Legacy engage |

### 4.2 Publishers

| Topic | Type |
|---|---|
| `/tf` | `tf2_msgs/TFMessage` (origin → simulated frame) |
| `output/odometry` | `nav_msgs/Odometry` |
| `output/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `output/twist` | `autoware_vehicle_msgs/VelocityReport` |
| `output/acceleration` | `geometry_msgs/AccelWithCovarianceStamped` |
| `output/imu` | `sensor_msgs/Imu` |
| `output/steering` | `autoware_vehicle_msgs/SteeringReport` (gated by `enable_pub_steer`) |
| `output/control_mode_report` | `autoware_vehicle_msgs/ControlModeReport` |
| `output/gear_report` | `autoware_vehicle_msgs/GearReport` |
| `output/turn_indicators_report` | `autoware_vehicle_msgs/TurnIndicatorsReport` |
| `output/hazard_lights_report` | `autoware_vehicle_msgs/HazardLightsReport` |
| `output/actuation_status` | `tier4_vehicle_msgs/ActuationStatusStamped` (only when active model exposes it) |

### 4.3 Services

| Service | Type | Purpose |
|---|---|---|
| `input/control_mode_request` | `autoware_vehicle_msgs/srv/ControlModeCommand` | Switch AUTONOMOUS / MANUAL |
| `/api/simulator/set/pose` | `tier4_external_api_msgs/srv/InitializePose` | External re-init |

---

## 5. Build & run

```bash
# Build only this package (run from the workspace root):
colcon build --packages-up-to autoware_simple_planning_simulator

# Run gtest:
colcon test --packages-select autoware_simple_planning_simulator
colcon test-result --verbose

# Launch standalone:
ros2 launch autoware_simple_planning_simulator simple_planning_simulator.launch.py
```

In a real Autoware system this node is not launched in isolation; an upstream
`autoware_launch` configuration includes `simple_planning_simulator.launch.py` (with the right
`vehicle_info_param_file` / `simulator_model_param_file`) and also remaps every input/output
into the global namespace.

---

## 6. Where to go next

- **Headers and contracts:** [`include.md`](include.md)
- **Implementations and math:** [`src.md`](src.md)
- **Launch wiring:** [`launch.md`](launch.md)
- **Parameter knobs:** [`param.md`](param.md)
- **Static data files:** [`data.md`](data.md)
- **Tests:** [`test.md`](test.md)
- **Upstream description and diagrams:** [`docs.md`](docs.md), [`media.md`](media.md)
