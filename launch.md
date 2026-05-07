# `launch/` — ROS 2 Launch Files

The folder contains a single launch file:

```
launch/
└── simple_planning_simulator.launch.py
```

## [`simple_planning_simulator.launch.py`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/launch/simple_planning_simulator.launch.py)

A Python launch script that:

1. Declares five launch arguments,
2. In a deferred `OpaqueFunction`, reads the YAML files,
3. Creates the `simple_planning_simulator` node with appropriate remappings,
4. Optionally launches `autoware_raw_vehicle_cmd_converter` alongside it (for `ACTUATION_CMD*`
   models).

---

### 1. Launch arguments

| Name | Default | Purpose |
|---|---|---|
| `vehicle_info_param_file` | `<autoware_vehicle_info_utils>/config/vehicle_info.param.yaml` | Wheelbase, track, overhangs, etc. (used by `VehicleInfoUtils` inside the node). |
| `vehicle_characteristics_param_file` | `<this_pkg>/param/vehicle_characteristics.param.yaml` | Mass, inertia, corner stiffnesses, max steer (currently informational). |
| `simulator_model_param_file` | `<this_pkg>/param/simple_planning_simulator_default.param.yaml` | All the `vehicle_model_type`-specific parameters (delays, time-constants, limits, …). |
| `acceleration_param_file` | `<this_pkg>/param/acceleration_map.csv` | Default acceleration map (used only by `DELAY_STEER_MAP_ACC_GEARED`). |
| `raw_vehicle_cmd_converter_param_path` | `<autoware_raw_vehicle_cmd_converter>/config/raw_vehicle_cmd_converter.param.yaml` | Forwarded to the optional `raw_vehicle_cmd_converter` node. |
| `csv_accel_brake_map_path` | `<autoware_raw_vehicle_cmd_converter>/data/default` | Forwarded; relevant for vehicle-specific actuation maps. |

Plus one parameter (not declared as a launch argument but expected to be supplied by the
parent launch):

- `initial_engage_state` (bool) — passed straight through into the node's parameter map.

### 2. `launch_setup(context)` — the deferred body

1. **Read** the vehicle-info and vehicle-characteristics YAMLs through `yaml.safe_load`,
   indexed by `["/**"]["ros__parameters"]` to honour the standard ROS-2 wildcard.
2. The simulator-model param file is wrapped in a `launch_ros.parameter_descriptions.ParameterFile`
   with `allow_substs=True` so `$(find-pkg-share …)` expressions inside the YAML resolve at
   launch time. (Used e.g. by the mechanical YAML to refer to `data/actuation_cmd_map/*.csv`.)
3. **Base remappings** translate the node's local input/output namespace into the standard
   Autoware topic graph:

   | Internal | Remapped to |
   |---|---|
   | `input/vector_map` | `/map/vector_map` |
   | `input/initialpose` | `/initialpose3d` |
   | `input/ackermann_control_command` | `/control/command/control_cmd` |
   | `input/actuation_command` | `/control/command/actuation_cmd` |
   | `input/manual_ackermann_control_command` | `/vehicle/command/manual_control_cmd` |
   | `input/gear_command` | `/control/command/gear_cmd` |
   | `input/manual_gear_command` | `/vehicle/command/manual_gear_command` |
   | `input/turn_indicators_command` | `/control/command/turn_indicators_cmd` |
   | `input/hazard_lights_command` | `/control/command/hazard_lights_cmd` |
   | `input/trajectory` | `/planning/trajectory` |
   | `input/engage` | `/vehicle/engage` |
   | `input/control_mode_request` | `/control/control_mode_request` |
   | `output/twist` | `/vehicle/status/velocity_status` |
   | `output/imu` | `/sensing/imu/imu_data` |
   | `output/steering` | `/vehicle/status/steering_status` |
   | `output/gear_report` | `/vehicle/status/gear_status` |
   | `output/turn_indicators_report` | `/vehicle/status/turn_indicators_status` |
   | `output/hazard_lights_report` | `/vehicle/status/hazard_lights_status` |
   | `output/control_mode_report` | `/vehicle/status/control_mode` |
   | `output/actuation_status` | `/vehicle/status/actuation_status` |

4. **Conditional remappings** based on a `motion_publish_mode` LaunchConfiguration:

   - `pose_only` (the simulator's pose feeds the localization estimator):
     - `output/odometry` → `/simulation/debug/localization/kinematic_state`
     - `output/acceleration` → `/simulation/debug/localization/acceleration`
     - `output/pose` → `/localization/pose_estimator/pose_with_covariance`
   - `full_motion` (the simulator's odometry feeds the global stack):
     - `output/odometry` → `/localization/kinematic_state`
     - `output/acceleration` → `/localization/acceleration`
     - `output/pose` → `/simulation/debug/localization/pose_estimator/pose_with_covariance`

   `motion_publish_mode` itself is NOT declared as a `DeclareLaunchArgument` in this file —
   it must be passed in by the parent launcher. Other modes are silently ignored.

5. **Node spec**:

   ```python
   simple_planning_simulator_node = Node(
       package="autoware_simple_planning_simulator",
       executable="autoware_simple_planning_simulator_node",
       namespace="simulation",
       output="screen",
       parameters=[
           vehicle_info_param,
           vehicle_characteristics_param,
           simulator_model_param,
           {"initial_engage_state": LaunchConfiguration("initial_engage_state")},
       ],
       remappings=remappings,
   )
   ```

   - The executable is the `rclcpp_components`-registered binary built by CMake (see
     [`overview.md`](overview.md)).
   - The node runs in the `/simulation` namespace.
   - Parameters are layered: vehicle info → characteristics → simulator model → engage state
     (later layers override earlier ones if keys collide).

6. **Conditional `raw_vehicle_cmd_converter`**:

   ```python
   launch_vehicle_cmd_converter = "ACTUATION_CMD" in simulator_model_param_yaml["/**"][
       "ros__parameters"
   ].get("vehicle_model_type")
   ```

   - If the YAML's `vehicle_model_type` string contains `"ACTUATION_CMD"` (matches any of
     `ACTUATION_CMD`, `ACTUATION_CMD_VGR`, `ACTUATION_CMD_MECHANICAL`, `ACTUATION_CMD_STEER_MAP`),
     the launcher additionally includes `raw_vehicle_converter.launch.xml` from
     `autoware_raw_vehicle_cmd_converter`. This is required because those models receive
     **pedal/steer actuation** commands, which `raw_vehicle_cmd_converter` produces from the
     control stack's high-level Control messages.
   - Otherwise only the simulator node is launched.

### 3. `generate_launch_description()`

Standard ROS 2 launch entry point. Declares the launch arguments listed above and wraps the
deferred body in an `OpaqueFunction` (deferred so `LaunchConfiguration(...).perform(context)`
can be called for the YAML-reading branch).

### 4. How parameters reach the node

The launcher uses three different mechanisms:

1. **Plain dict** for `vehicle_info_param` and `vehicle_characteristics_param` — Python
   `safe_load`s the YAML, then passes the inner dict as a normal `parameters=[…]` entry.
2. **`ParameterFile(..., allow_substs=True)`** for the simulator-model YAML — keeps the file
   as-is so substitutions like `$(find-pkg-share simple_planning_simulator)/data/...` are
   resolved by the parameter loader at launch time. This is necessary for the mechanical
   YAML (it embeds CSV paths).
3. **Inline dict** for `initial_engage_state`, populated from a `LaunchConfiguration` so the
   parent launcher can override it.

### 5. Standalone use

Running the launcher directly will fail unless the parent supplies `initial_engage_state`
and `motion_publish_mode`. To launch standalone for development:

```bash
ros2 launch autoware_simple_planning_simulator simple_planning_simulator.launch.py \
     initial_engage_state:=true \
     motion_publish_mode:=full_motion
```

In a real Autoware setup these are usually set inside `autoware_launch`'s simulator launch
graph, which also publishes the static `map → odom` TF that this node assumes is identity.
