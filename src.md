# `src/` — Implementation Files

The C++ implementations live under
[`src/simple_planning_simulator/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/):

```
src/simple_planning_simulator/
├── simple_planning_simulator_core.cpp     ← the ROS node
├── utils/
│   ├── csv_loader.cpp
│   └── mechanical_controller.cpp
└── vehicle_model/
    ├── sim_model_interface.cpp             ← integrators
    ├── sim_model_ideal_steer_vel.cpp
    ├── sim_model_ideal_steer_acc.cpp
    ├── sim_model_ideal_steer_acc_geared.cpp
    ├── sim_model_delay_steer_vel.cpp
    ├── sim_model_delay_steer_acc.cpp
    ├── sim_model_delay_steer_acc_geared.cpp
    ├── sim_model_delay_steer_acc_geared_wo_fall_guard.cpp
    ├── sim_model_delay_steer_map_acc_geared.cpp
    ├── sim_model_actuation_cmd.cpp
    └── sim_model_learned_steer_vel.cpp
```



---

## 1. [`simple_planning_simulator_core.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp)

The single biggest file in the package: the ROS node itself.

### 1.1 Helper Functions

These are file-local helpers that convert between `SimModelInterface` and ROS messages amik kellenek az autoware nek a simulacio miatt: 

- **`to_velocity_report(model)`** ([line 55](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L55)) — builds a `VelocityReport`:
  - `longitudinal_velocity = model->getVx()`
  - `lateral_velocity = 0.0` (single-track / bicycle assumption)
  - `heading_rate = model->getWz()`
  - `seged fugvany a vehicle model state-bol velocity reportot csinal es mivel bicikli model van, lateral velocity es heading rate-et 0-ra teszi nincs oldal iranyu mozgas`

- **`to_odometry(model, ego_pitch_angle)`** ([line 67](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L67)) — packages a `nav_msgs/Odometry`:
  - `pose.pose.position` = `(getX(), getY(), 0)` (Z is filled in later from the trajectory)
  - `pose.pose.orientation` = quaternion from `(roll=0, pitch=ego_pitch_angle, yaw=getYaw())`
  - `twist.twist.linear.x = getVx()`, `twist.twist.angular.z = getWz()`
  - `seged fugvany a vehicle model state-bol odometryt csinal, statuszt igy gyakorlatilag az elmozdulast`

- **`to_steering_report(model)`** ([line 83](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L83)) — sets `steering_tire_angle = model->getSteer()`.
  `autoware tipusus SteeringReportot csinal a vehicle model state-bol, a steer szoget adja vissza, ami a kormany szoget jelenti, ez is 0 lesz mert bicikli modell van es nincs oldal iranyu mozgas`

- **`convert_centerline_to_points(lanelet)`** ([line 92](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L92)) — copies a lanelet's
  3D centerline points into a `std::vector<geometry_msgs::msg::Point>`. Used inside
  `calculate_ego_pitch()`.
  - `seged fugvany ami a lanelet centerline pontjaibol egy vectorba teszi a pontokat, hogy azt hasznaljuk a z pozicio szamitasahoz, hogy a jarmu a lanelet-en maradjon`


### 1.2 Constructor — `SimplePlanningSimulator::SimplePlanningSimulator`

A Simple planning sim class konstructora \
Defined at [line 105](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L105). Walks through, in order:

1. **Declare common parameters** (lines 108–113):
   - `simulated_frame_id` (`base_link`),
   - `origin_frame_id` (`odom`),
   - `add_measurement_noise` (false), 
   - `initial_engage_state` (no default — must be supplied),
   - `enable_road_slope_simulation` (false), 
   - `enable_pub_steer` (true).

2. **Subscriptions** \
    sub topic ok lebontva 
    -  `sub_map` -> az utolso 10 hivas adatot tartja meg a lanelet2 publikaciobol 
    - `sub_init_pose` -> inicializalo topic az auto helyzetére 
    - `sub_init_twist` -> inidulasi sebesseg beallitasa 
    - `sub_manual_ackermann_cmd` - >  manuális kormany es sebesseg parancsok kiadasa az autonak 
    - `sub_gear_cmd` ->  beallitja a sebesseget D/R/P/N az önvezető funkciókhoz 
    - `sub_manual_gear_cmd` -> ennek lenyege hogyfelulirja a aktualis control parancsokat 
    - `sub_turn_indicators_cmd` ->
    - `sub_hazard_lights_cmd` -> veszvillo parancsokat adja at (minek az ez szimulacio XD)
    - `sub_trajectory` -> a megtervzett szimulaciot kapja meg     

3. **Service** `input/control_mode_request` (line 144).
    - `srv_mode_req` -> a service a conrol mod beallitasara szolgal 

4. **Engage subscription** \ 
   - engage car command topic 

5. **Pick command-input type by inspecting `vehicle_model_type_str`** :
   - if it contains `"ACTUATION_CMD"` → subscribe `input/actuation_command` and seed
     `current_input_command_` with an empty `ActuationCommandStamped`,
   - else → subscribe `input/ackermann_control_command` and seed with an empty `Control`.



6. **Publishers**: all 12 listed in . 
   - `pub_control_mode_report_` ->  lekerdezi es riportot keszit az aktualis control mode rol
   - `pub_gear_report_` ->  report az aktualis gear rol 
   - `pub_turn_indicators_report_ pub_hazard_lights_report_` -> iranyjelyok es vesz villogo jelentes 
    - `pub_current_pose_` -> az auto jelenlegi poziciojat publikalja
   - `pub_velocity_` ->  jelenlegi ebessegrol jelentes FONTOS lat lon tipusu sebeseget publikal 
   - `pub_odom` -> ktualis pose és twist message publikalasa 
   - `pub_imu` -> aktualis imu publikalasa 
   - `pub_tf_` -> aktulalis tf topic publikasa aza autoware szamara 
   - `pub_actuation_status_` -> lekerkezi a szimulalt auto aktuacios helyezetet


7. **`add_on_set_parameters_callback` → `on_parameter`** — only `x_stddev` and
   `y_stddev` are runtime-tunable.

8. **Timer** (lines 192–195) — `timer_sampling_time_ms` (default 25 ms) bound to `on_timer`.

9. **`tier4_api_utils::Service<InitializePose>`** (lines 197–201) — `/api/simulator/set/pose`
   on its own `MutuallyExclusive` callback group.

  


10. **Vehicle-model construction** — `initialize_vehicle_model(vehicle_model_type_str)` (§1.3).

11. **Initial pose source** (lines 207–215): if `initialize_source == "ORIGIN"`, set state to
    `(0,0,0)` directly. Otherwise wait for an `input/initialpose` message.

12. **Measurement-noise generator** (lines 218–233): seeds a Mersenne-Twister and four
    `std::normal_distribution<>`. Also reads `x_stddev` / `y_stddev` for the dummy covariance.

13. **Defaults**: `current_control_mode_.mode = AUTONOMOUS`, `current_manual_gear_cmd_.command = PARK`.

### 1.3 `initialize_vehicle_model(vehicle_model_type_str)`

[Line 240](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L240). Reads ~12 numeric parameters
(`vel_lim`, `vel_rate_lim`, `steer_lim`, `steer_rate_lim`, the time delays/constants, dead
band, bias, debug-scaling, and the `wheelbase` from `VehicleInfoUtils`), and the three
`std::vector<std::string>` arrays for the learned model. Then a long `if/else` chain selects
which `SimModelInterface` subclass to instantiate:

| `vehicle_model_type` | Class instantiated |
|---|---|
| `IDEAL_STEER_VEL` | `SimModelIdealSteerVel(wheelbase)` |
| `IDEAL_STEER_ACC` | `SimModelIdealSteerAcc(wheelbase)` |
| `IDEAL_STEER_ACC_GEARED` | `SimModelIdealSteerAccGeared(wheelbase)` |
| `DELAY_STEER_VEL` | `SimModelDelaySteerVel(...)` |
| `DELAY_STEER_ACC` | `SimModelDelaySteerAcc(...)` |
| `DELAY_STEER_ACC_GEARED` | `SimModelDelaySteerAccGeared(...)` |
| `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD` | `SimModelDelaySteerAccGearedWoFallGuard(...)` |
| `DELAY_STEER_MAP_ACC_GEARED` | reads `acceleration_map_path`, throws if file missing, then `SimModelDelaySteerMapAccGeared(...)` |
| `LEARNED_STEER_VEL` | `SimModelLearnedSteerVel(dt, paths, params, classes)` |
| `ACTUATION_CMD*` | reads accel/brake delays/constants and conversion flags, then dispatches to one of: |
| &nbsp;&nbsp;&nbsp;`ACTUATION_CMD` | `SimModelActuationCmd(...)` |
| &nbsp;&nbsp;&nbsp;`ACTUATION_CMD_VGR` | + `vgr_coef_a/b/c` → `SimModelActuationCmdVGR(...)` |
| &nbsp;&nbsp;&nbsp;`ACTUATION_CMD_MECHANICAL` | + `vgr_coef_*` + 21 `mechanical_params.*` → `SimModelActuationCmdMechanical(...)` |
| &nbsp;&nbsp;&nbsp;`ACTUATION_CMD_STEER_MAP` | + `steer_map_path` → `SimModelActuationCmdSteerMap(...)` |

Anything else throws `std::invalid_argument("Invalid vehicle_model_type: …")`.

### 1.4 `on_parameter(parameters)`

Tries to update only `x_stddev` and `y_stddev` (using `autoware_utils_rclcpp::update_param`).
On `InvalidParameterTypeException` it sets `result.successful = false` and returns the
exception's message as the `reason`.

### 1.5 `calculate_ego_pitch()` ([line 426](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L426))

Steps:

1. Build a `geometry_msgs/Pose` from the current model state.
2. Use `autoware::experimental::lanelet2_utils::get_closest_lanelet_within_constraint(...)`
   with a 2 m radius. Returns `nullopt` ⇒ pitch = 0.
3. Convert the lanelet's centerline to points; find the nearest segment via
   `autoware::motion_utils::findNearestSegmentIndex(...)`.
4. Compute the **lanelet's local yaw** at that segment (`atan2(dy, dx)` between segment
   end points).
5. `ego_yaw_against_lanelet = ego_yaw - lanelet_yaw` — angle between the ego heading and the
   centerline.
6. `diff_z = next.z - prev.z`,
   `diff_xy = hypot(dx, dy) / cos(ego_yaw_against_lanelet)` — adjusts XY span by the
   misalignment so the slope is reported in the ego's body frame.
7. If `cos(ego_yaw_against_lanelet) < 0` (ego is going against the lanelet direction),
   the sign is flipped:
   `pitch = (sign-flipped) ? -atan2(-diff_z, -diff_xy) : -atan2(diff_z, diff_xy)`.

The returned pitch is **negated** when used in the timer to compute slope-induced
acceleration, see §1.6.

### 1.6 `on_timer()` ([line 465](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/simple_planning_simulator_core.cpp#L465))

The hot loop. In order:

1. **Initialization gate**: if `is_initialized_ == false`, only publish `control_mode_report`
   and bail (so downstream control nodes don't deadlock waiting for it).
2. **Slope acceleration**:
   ```cpp
   constexpr double gravity_acceleration = -9.81;
   const double slope_angle = enable_road_slope_simulation_ ? -ego_pitch_angle : 0.0;
   const double acc_by_slope = gravity_acceleration * std::sin(slope_angle);
   ```
   `acc_by_slope > 0` means the road tilts down so gravity pushes the car forward.
3. **`dt`** = `delta_time_.get_dt(now)` — wall-clock delta.
4. **Pick command source** by control mode:
   - `AUTONOMOUS` → set the autonomous gear, call `set_input(current_input_command_, …)`.
   - `MANUAL`     → set the manual gear, call `set_input(current_manual_ackermann_cmd_, …)`.
5. **Step dynamics**: only if `simulate_motion_`, call `vehicle_model_ptr_->update(dt)`.
6. **Read the new state**:
   ```cpp
   current_odometry_ = to_odometry(vehicle_model_ptr_, ego_pitch_angle);
   current_odometry_.pose.pose.position.z = get_z_pose_from_trajectory(...);
   current_velocity_ = to_velocity_report(vehicle_model_ptr_);
   current_steer_   = to_steering_report(vehicle_model_ptr_);
   ```
7. **Optional noise**: `add_measurement_noise(current_odometry_, current_velocity_, current_steer_)`.
8. **Dummy covariance**: `pose.covariance[X_X] = x_stddev_; pose.covariance[Y_Y] = y_stddev_;`.
9. **Publish everything**: odometry, pose, velocity, acceleration, IMU, control-mode report,
   gear report, turn indicators, hazards, TF, optionally steering, optionally actuation
   status.

### 1.7 Subscription / service callbacks

| Callback | Effect |
|---|---|
| `on_map(msg)` | Parses Lanelet2 map binary via `autoware::experimental::lanelet2_utils::from_autoware_map_msgs(...)`, then caches `road_lanelets_ = roadLanelets(allLanelets)`. |
| `on_initialpose(msg)` | Calls `set_initial_state_with_transform(stamped_pose, zero_twist)`, stores the message. |
| `on_initialtwist(msg)` | Re-initializes with the same pose and the new twist (only if `initial_pose_` is non-null). |
| `on_set_pose(req, res)` (service) | Same as `on_initialpose` plus sets the response status to success. |
| `on_trajectory(msg)` | Caches the latest trajectory pointer for `get_z_pose_from_trajectory`. |
| `on_engage(msg)` | `simulate_motion_ = msg->engage`. |
| `on_control_mode_request(req, res)` | Switches between `MANUAL`, `AUTONOMOUS`. Other request modes return `success = false`. |
| `on_turn_indicators_cmd(msg)`, `on_hazard_lights_cmd(msg)` | Plain pointer caching. |

### 1.8 `set_input` overloads

Three overloads cooperate:

```cpp
void set_input(const InputCommand & cmd, const double acc_by_slope);          // dispatcher
void set_input(const Control & cmd, const double acc_by_slope);               // Ackermann
void set_input(const ActuationCommandStamped & cmd, const double acc_by_slope);// Pedal/steer
```

The dispatcher uses `std::visit`. The `Control` overload computes a **gear-aware combined
acceleration**:

```cpp
combined_acc = (gear == NONE)        ? 0.0
             : (gear == REVERSE)     ? -acc_by_cmd + acc_by_slope
                                     :  acc_by_cmd + acc_by_slope;
```

Then packs the model's input vector based on `vehicle_model_type_`:

| Model | `input` |
|---|---|
| `IDEAL_STEER_VEL`, `DELAY_STEER_VEL`, `LEARNED_STEER_VEL` | `[velocity, steer]` |
| `IDEAL_STEER_ACC`, `DELAY_STEER_ACC` | `[combined_acc, steer]` |
| `IDEAL_STEER_ACC_GEARED`, `DELAY_STEER_ACC_GEARED`, `DELAY_STEER_MAP_ACC_GEARED` | `[combined_acc, steer]` |
| `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD` | `[acc_by_cmd, gear, acc_by_slope, steer]` (raw, the model handles slope/gear itself) |

The `ActuationCommandStamped` overload always packs `[accel, brake, acc_by_slope, steer, gear]`
— the actuation models always know how to apply slope and gear themselves.

### 1.9 `add_measurement_noise(odom, vel, steer)`

Pulls one sample from each of the four normal distributions and:

- Adds `pos_dist_` to `odom.pose.pose.position.{x,y}` (independent samples).
- Adds `vel_dist_` to `odom.twist.twist.linear.x` **and** `vel.longitudinal_velocity`
  (same sample, so they stay consistent).
- Adds `rpy_dist_` to the yaw extracted from the orientation, then re-encodes the quaternion.
- Adds `steer_dist_` to `steer.steering_tire_angle`.

Note: roll/pitch are not noised because they come from pitch calculation, not the model.

### 1.10 `set_initial_state_with_transform` and `set_initial_state`

`set_initial_state_with_transform(pose_stamped, twist)` looks up the transform from
`origin_frame_id_` to `pose_stamped.header.frame_id` (blocking until it's available — see
`get_transform_msg` below) and applies it before delegating to `set_initial_state(...)`.

`set_initial_state(pose, twist)` builds the appropriate-sized `Eigen::VectorXd state` based on
the active `vehicle_model_type_`:

| Model | State dimensionality |
|---|---|
| `IDEAL_STEER_VEL` | `[x, y, yaw]` (3) |
| `IDEAL_STEER_ACC[, _GEARED]` | `[x, y, yaw, vx]` (4) |
| `DELAY_STEER_VEL` | `[x, y, yaw, vx, steer]` (5) |
| `LEARNED_STEER_VEL` | `[x, y, yaw, yaw_rate, vx, vy, steer]` (7) |
| `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD` | `[x, y, yaw, vx, steer, accx, pedal_accx]` (7) |
| `DELAY_STEER_ACC[, _GEARED, _MAP_…], ACTUATION_CMD*` | `[x, y, yaw, vx, steer, accx]` (6) |

Then `vehicle_model_ptr_->setState(state); is_initialized_ = true;`.

### 1.11 `get_z_pose_from_trajectory(x, y, prev_odometry)`

If no trajectory is cached → returns `prev_odometry.pose.pose.position.z`. Otherwise scans
all trajectory points, keeps the index of the smallest squared distance to `(x,y)`, and
returns that point's `position.z`.

### 1.12 `get_transform_msg(parent_frame, child_frame)`

A **blocking** TF lookup: catches `tf2::TransformException`, logs the error, sleeps 500 ms,
retries forever. Suitable here because `set_initial_state_with_transform` is only called from
ROS callbacks — not from the timer hot path.

### 1.13 `publish_*` family

Each is a small wrapper that fills in `header.stamp = now()` and (where applicable) frame ids:

- **`publish_velocity`**: `frame_id = simulated_frame_id_`.
- **`publish_odometry`**: `frame_id = origin_frame_id_`, `child_frame_id = simulated_frame_id_`.
- **`publish_pose`**: copies `odometry.pose` into a `PoseWithCovarianceStamped` and sets
  fixed dummy covariances (`POS = 0.0225, ANGLE = 0.000625`, "same value as current ndt
  output").
- **`publish_steering`**: trivial.
- **`publish_acceleration`**: `accel.linear.x = getAx()`, `accel.linear.y = getWz() * getVx()`
  (centripetal); covariance set to `0.001` on the diagonal entries.
- **`publish_imu`**: same `linear_acceleration.{x,y}`, `angular_velocity = current_odometry.twist.twist.angular`,
  `orientation = current_odometry.pose.pose.orientation`. Covariances `0.001`.
- **`publish_control_mode_report`**: stamps and publishes `current_control_mode_`.
- **`publish_gear_report`**: `report = vehicle_model_ptr_->getGear()`.
- **`publish_turn_indicators_report`**: if `command == NO_COMMAND`, report `DISABLE`;
  otherwise echo the command. Skips entirely if no command has been received yet.
- **`publish_hazard_lights_report`**: pure echo. Skips if no command received.
- **`publish_actuation_status`**: queries `vehicle_model_ptr_->getActuationStatus()`, if it
  returns a value, stamps and publishes it. Only `SimModelActuationCmd*` returns a non-empty
  value.
- **`publish_tf`**: builds a `TransformStamped` from the current odometry and wraps it in a
  `TFMessage`.

### 1.14 Component registration

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::simulator::simple_planning_simulator::SimplePlanningSimulator)
```

Means the node is loadable as `rclcpp_components` plugin, which is how the executable
`autoware_simple_planning_simulator_node` (registered in CMake) loads it.

---

## 2. [`vehicle_model/sim_model_interface.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_interface.cpp)

Implements the **integrators** and the trivial getters/setters of `SimModelInterface`.

### Constructor

```cpp
SimModelInterface::SimModelInterface(int dim_x, int dim_u)
: dim_x_(dim_x), dim_u_(dim_u)
{
  state_ = Eigen::VectorXd::Zero(dim_x_);
  input_ = Eigen::VectorXd::Zero(dim_u_);
}
```

### `updateRungeKutta(dt, input)`

Classical 4th-order Runge–Kutta:

```cpp
Eigen::VectorXd k1 = calcModel(state_,                    input);
Eigen::VectorXd k2 = calcModel(state_ + k1 * 0.5 * dt,    input);
Eigen::VectorXd k3 = calcModel(state_ + k2 * 0.5 * dt,    input);
Eigen::VectorXd k4 = calcModel(state_ + k3 * dt,          input);
state_ += dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
```

Note that **input is held constant** through all four sub-steps.

### `updateEuler(dt, input)`

```cpp
state_ += calcModel(state_, input) * dt;
```

Used by `SimModelDelaySteerAccGearedWoFallGuard` because its longitudinal-acceleration
calculation contains a non-differentiable branch on `gear` and `vel`, which RK4 cannot
handle correctly.

### Getters/setters

`getState`, `getInput`, `setState`, `setInput`, `setGear`, `getGear` — all trivial member
copies/assignments. `setState` is `virtual` because some models need to mirror the state
into auxiliary controllers.

---

## 3. The vehicle-model implementations

All concrete models follow the same general shape:

```
constructor:        store params, call initializeInputQueue(dt) (if any delays exist)
getX..getSteer:     return either state_(IDX::*) or compute (e.g. wz = vx*tan(steer)/L)
update(dt):         (a) push current input_ onto the delay queue, pop the front into delayed_input
                    (b) updateRungeKutta(dt, delayed_input)  // or updateEuler
                    (c) clamp velocity, optionally call updateStateWithGear(...)
calcModel:          construct dx/dt as an Eigen::VectorXd
initializeInputQueue: deque sized round(delay/dt), filled with 0.0
```

The differences between models are: which channels are delayed, what dynamics
`calcModel` implements, and whether/how `update` post-processes the result.

### 3.1 [`sim_model_ideal_steer_vel.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp)

Ideal kinematic bicycle, **velocity input**, **no delay**.

State `[x, y, yaw]`. Input `[vx_des, steer_des]`.

```
ẋ   = vx_des * cos(yaw)
ẏ   = vx_des * sin(yaw)
ẏaw = vx_des * tan(steer_des) / L
```

`update(dt)`: `updateRungeKutta(dt, input_)`, then `current_ax_ = (vx_des - prev_vx_) / dt`
to expose acceleration as a derivative of the commanded velocity. `getVx()` returns
`input_(VX_DES)` — the commanded value, since this model has no inertia.

### 3.2 [`sim_model_ideal_steer_acc.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp)

Ideal bicycle, **acceleration input**, no delay, no gear logic.

State `[x, y, yaw, vx]`. Input `[ax_des, steer_des]`.

```
ẋ   = vx * cos(yaw)
ẏ   = vx * sin(yaw)
v̇x  = ax_des
ẏaw = vx * tan(steer_des) / L
```

`update(dt)` is just `updateRungeKutta(dt, input_)`. `getAx()` returns the commanded
acceleration directly.

### 3.3 [`sim_model_ideal_steer_acc_geared.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp)

Same dynamics as 3.2, but `update(dt)` post-processes with `updateStateWithGear(...)`:

- For any DRIVE/LOW gear: if `state(VX) < 0` ⇒ snap to stop (revert to `prev_state` for
  `x`, `y`, `yaw`, set `VX = 0`).
- For REVERSE/REVERSE_2: if `state(VX) > 0` ⇒ snap to stop.
- For PARK / NONE / NEUTRAL: always snap to stop.
- After the snap, `current_acc_ = (state(VX) - prev_state(VX)) / max(dt, 1e-5)`.

`getAx()` returns this `current_acc_` (so during a snap the published acceleration drops
sharply too).

### 3.4 [`sim_model_delay_steer_vel.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel.cpp)

1st-order velocity & steering dynamics with **dead-time** on each channel.

State `[x, y, yaw, vx, steer]`. Input `[vx_des, steer_des]` (both go through a
`std::deque<double>` of size `round(delay / dt)`).

The model's `calcModel`:

```
sat(v) = clamp(v, [-vx_lim, vx_lim])
sat(δ) = clamp(δ, [-steer_lim, steer_lim])

vx_rate    = sat(-(vx - vx_des) / vx_time_constant_,        [-vx_rate_lim, vx_rate_lim])
steer_diff = (steer + steer_bias) - steer_des
steer_diff_with_dead_band:
   if |steer_diff| > dead_band: steer_diff -+ dead_band      // dead-band shrinks the gap
   else:                          0.0
steer_rate = sat(-steer_diff_with_dead_band / steer_time_constant_,
                 [-steer_rate_lim, steer_rate_lim])

ẋ    = vx * cos(yaw)
ẏ    = vx * sin(yaw)
ẏaw  = vx * tan(steer) / L
v̇x  = vx_rate
δ̇   = steer_rate
```

`getVx()` returns `state_(VX)` (now physical, not commanded).
`getSteer()` returns `state_(STEER) + steer_bias_` (sensor reading).
`getAx()` returns the cached `(state_(VX) - prev_vx) / dt` from `update`.

The `MIN_TIME_CONSTANT = 0.03 s` floor is applied in the constructor.

### 3.5 [`sim_model_delay_steer_acc.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp)

Like 3.4 but with **acceleration input** and an extra state `accx`:

State `[x, y, yaw, vx, steer, accx]`. Input `[accx_des, steer_des]`.

`calcModel`:

```
acc_des = sat(accx_des, [-vx_rate_lim, vx_rate_lim]) * debug_acc_scaling_factor_
δ_des   = sat(steer_des, [-steer_lim, steer_lim])    * debug_steer_scaling_factor_
ẋ    = vx * cos(yaw)
ẏ    = vx * sin(yaw)
ẏaw  = vx * tan(steer) / L
v̇x  = accx                                  // velocity follows the modelled accx
δ̇   = -steer_diff_with_dead_band / τ_δ
ȧ    = -(accx - acc_des) / acc_time_constant // 1st-order accel pursuit
```

After `updateRungeKutta`, `state_(VX)` is clamped to `[-vx_lim, vx_lim]`. The two
`debug_*_scaling_factor_` knobs let you intentionally model a vehicle that under- or
over-responds to its commands.

### 3.6 [`sim_model_delay_steer_acc_geared.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp)

Identical to 3.5 plus a final `updateStateWithGear(...)` step that snaps the state to a stop
when the velocity sign disagrees with the gear (DRIVE→`vx<0`, REVERSE→`vx>0`, PARK→always).
Same logic as in `SimModelIdealSteerAccGeared` (§3.3) but also resets `state(ACCX)` to the
finite difference. This is the **default model** in
[`simple_planning_simulator_default.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml).

### 3.7 [`sim_model_delay_steer_acc_geared_wo_fall_guard.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.cpp)

A more careful variant that exposes more state and **uses Euler instead of RK4** (the
comment in `update` says: *"we cannot use updateRungeKutta() because the differentiability or
the continuity condition is not satisfied"*).

State `[x, y, yaw, vx, steer, accx, pedal_accx]` (7).
Input `[pedal_accx_des, gear, slope_accx, steer_des]` (4).

`calcModel`'s longitudinal branch is gear-aware:

```
if pedal_acc >= 0 (driving):
   if gear == NONE/PARK:           v̇x = 0
   elif gear == NEUTRAL:            v̇x = slope_accx
   elif gear == REVERSE/REVERSE_2:  v̇x = -pedal_acc + slope_accx
   else (DRIVE etc):                v̇x =  pedal_acc + slope_accx
else (braking, pedal_acc < 0):
   if vx > 0:                       v̇x =  pedal_acc + slope_accx       // friction opposes motion
   elif vx < 0:                     v̇x = -pedal_acc + slope_accx
   elif -pedal_acc >= |slope_accx|: v̇x = 0                              // brake holds against slope
   else:                            v̇x = slope_accx                      // brake too weak, roll
```

`pedal_accx` is itself a 1st-order pursuit of `pedal_accx_des`. The yaw / steer dynamics are
the same as 3.6.

After the Euler step, a **stop condition** is applied:

```cpp
if (prev_state(VX) * state_(VX) <= 0.0
    && -state_(PEDAL_ACCX) >= |delayed_input(SLOPE_ACCX)|)
   state_(VX) = 0.0;
```

i.e. if the vehicle just crossed zero velocity and the brake is strong enough to hold against
the slope, snap to zero. This avoids the ill-conditioned alternation around zero that the
fall-guard models suppress with a hard "snap to stop on negative velocity in DRIVE".

`state_(ACCX) = (state_(VX) - prev_state(VX)) / dt` — `accx` is the realised accel, computed
from the velocity differential after the snap.

### 3.8 [`sim_model_delay_steer_map_acc_geared.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_map_acc_geared.cpp)

Almost identical to 3.6, except the desired acceleration is **remapped through a CSV table**
before the 1st-order pursuit:

```cpp
const double converted_acc =
  acc_map_.acceleration_map_.empty()
      ? acc_des
      : acc_map_.getAcceleration(acc_des, vel);
d_state(IDX::ACCX) = -(acc - converted_acc) / acc_time_constant_;
```

`AccelerationMap::getAcceleration(acc_des, vel)` does bilinear interpolation on the table —
see [`include.md` §5.1](include.md#51-helper-classes-embedded-in-two-of-those-headers). The
constructor reads the path with `acc_map_.readAccelerationMapFromCSV(path_)`. If the CSV
load fails it logs an error but keeps `acceleration_map_` empty, in which case `acc_des`
passes through unchanged. The same `updateStateWithGear(...)` snapping as 3.6 is applied
afterwards.

### 3.9 [`sim_model_actuation_cmd.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.cpp)

The largest of the model files — implements **four classes** that share a common base:

```
SimModelActuationCmd                 ← base
├── SimModelActuationCmdSteerMap     ← + steer-rate map
├── SimModelActuationCmdVGR           ← + variable gear ratio
└── SimModelActuationCmdMechanical    ← inherits from VGR, + PID + 2nd-order steering
```

#### 3.9.1 `ActuationMap`, `AccelMap`, `BrakeMap`

(Free helper classes also declared in the header.)

- **`ActuationMap::readActuationMapFromCSV(path, validation=false)`**: parses with
  `CSVLoader`, sets `state_index_`, `actuation_index_`, `actuation_map_`, optionally
  validates monotonicity.
- **`ActuationMap::getControlCommand(actuation, state)`**: bilinear interpolation, returns
  the looked-up "control" value (acc or steer-rate, depending on the map).
- **`AccelMap::getThrottle(acc, vel)`**: inverse mapping — given an `acc`, find the throttle
  value that matches at the given `vel`. Returns `nullopt` when `acc` is below the map's
  minimum (caller should switch to brake map). Returns max throttle when `acc` exceeds the
  map's maximum.
- **`BrakeMap::getBrake(acc, vel)`**: same idea but for brake. Reverses the interpolated
  acceleration vector before `lerp` because brake → acc is monotonically decreasing.

#### 3.9.2 `SimModelActuationCmd`

State `[x, y, yaw, vx, steer, accx]`. Input `[accel_des, brake_des, slope_accx, steer_des, gear]`.

Constructor (`MIN_TIME_CONSTANT = 0.03`) reads the four time constants/delays and the two
conversion-flag-with-CSV-path pairs:

```cpp
convert_accel_cmd_ = convert_accel_cmd && accel_map_.readActuationMapFromCSV(accel_map_path);
convert_brake_cmd_ = convert_brake_cmd && brake_map_.readActuationMapFromCSV(brake_map_path);
```

i.e. **conversion is silently disabled if the CSV cannot be loaded** — the model degrades
to passing the accel command through unchanged.

`update(dt)`:

1. Push each of accel/brake/steer onto its delay queue, pop the front into `delayed_input`.
2. Pass `gear` and `slope_accx` through unchanged.
3. Save `prev_state`.
4. `updateRungeKutta(dt, delayed_input)`.
5. Clamp `state_(VX)` and `state_(STEER)`.
6. `updateStateWithGear(state_, prev_state, gear_, dt)`.

`calcLongitudinalModel`:

```
acc_des_wo_slope = clamp(
    convert_accel_cmd && accel > 0  ?  accel_map_.getControlCommand(accel, vel)
    : convert_brake_cmd && brake > 0 ?  brake_map_.getControlCommand(brake, vel)
    : accel,                                        // pass-through
    [-vx_rate_lim, vx_rate_lim])

acc_des = (gear == NONE/PARK)         ?  0
        : (gear == REVERSE/REVERSE_2) ? -acc_des_wo_slope + slope_accx
                                      :  acc_des_wo_slope + slope_accx
acc_τ   = (accel > 0) ? accel_τ : brake_τ

ẋ    = vx * cos(yaw)
ẏ    = vx * sin(yaw)
ẏaw  = vx * tan(steer) / L
v̇x  = accx
ȧ    = -(accx - acc_des) / acc_τ
```

`calcLateralModel(steer, steer_des, vel)` (virtual):

```
steer_rate = clamp(-(getSteer() - steer_des) / steer_τ, [-steer_rate_lim, steer_rate_lim])
```

`calcModel(state, input)` calls `calcLongitudinalModel(state, input)` and writes the lateral
derivative into `d_state(IDX::STEER)`.

`updateStateWithGear(...)`: same as 3.6.

`getActuationStatus()`: builds an `ActuationStatusStamped` (from
`tier4_vehicle_msgs/msg/ActuationStatusStamped`) populated with throttle/brake values
*reverse-mapped from the current accx and vx*. Returns `nullopt` if no conversion is enabled.
`shouldPublishActuationStatus()` returns `true`.

#### 3.9.3 `SimModelActuationCmdSteerMap`

Inherits from `SimModelActuationCmd`. Constructor takes one extra path
(`steer_map_path`); throws if the CSV can't be loaded. Overrides only
`calcLateralModel(...)`:

```cpp
steer_rate = clamp(steer_map_.getControlCommand(steer_input, vel) / steer_τ,
                   [-steer_rate_lim, steer_rate_lim])
```

i.e. the steer command is passed through a CSV `(state, actuation) → control` lookup, then
interpreted as a steer rate.

#### 3.9.4 `SimModelActuationCmdVGR`

Inherits from `SimModelActuationCmd`. Adds three coefficients (`vgr_coef_a`, `b`, `c`) and
implements:

```
steer_wheel_state(vel, steer_tire) =
   (a + b * vel²) * steer_tire / (1 + c * |steer_tire|)

variable_gear_ratio(vel, steer_wheel) =
   max(1e-5, a + b * vel² - c * |steer_wheel|)

calculateSteeringTireCommand(vel, steer_tire, steer_wheel_des) =
   steer_wheel_des / variable_gear_ratio(vel, steer_wheel_state(vel, steer_tire))
```

`calcLateralModel(...)` then drives `steer_rate` toward this **converted tire command**
through the 1st-order steer dynamics. See
[`media/vgr_sim.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/vgr_sim.drawio.svg).

#### 3.9.5 `SimModelActuationCmdMechanical`

Inherits from `SimModelActuationCmdVGR` and adds a `MechanicalController` (defined in
[`utils/mechanical_controller.{hpp,cpp}`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/utils/mechanical_controller.hpp))
to model the **steering column dynamics**.

**`update(dt)`** is overridden:

1. Same delay-queue pop as base class.
2. Calls `updateRungeKuttaWithController(dt, delayed_input)` *instead of* the base
   `updateRungeKutta`.
3. Clamps and applies gear.

**`updateRungeKuttaWithController(dt, input)`**:

1. **Longitudinal RK4** — done **independently** of the lateral, using
   `calcLongitudinalModel(state_, input)` four times (k₁..k₄) and the standard Simpson's-rule
   weighted average. Writes the new longitudinal state into `state_`.
2. **Lateral via the mechanical controller**:
   - `steer_tire_des = calculateSteeringTireCommand(vel, state_(STEER), input(STEER_DES))`
     (from VGR).
   - `mechanical_controller_->set_steer(state_(STEER))` — synchronises the controller's
     internal angular position with `state_`.
   - `state_(STEER) = mechanical_controller_->update_runge_kutta(steer_tire_des, vel,
     prev_steer_tire_des_, dt);` — this advances the PID + spring-damper-friction column
     model by one RK4 step.
   - `prev_steer_tire_des_ = steer_tire_des;` — for the next call's rate-limit reference.

**`setState(state)`** is overridden because the controller has internal state (PID integral,
column angular position/velocity, delay buffer) that must be reset whenever the simulator is
re-initialised:

```cpp
mechanical_controller_->clear_state();
mechanical_controller_->set_steer(state(IDX::STEER));
state_ = state;
```

### 3.10 [`sim_model_learned_steer_vel.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_learned_steer_vel.cpp)

A thin shim around
[`autoware::simulator::learning_based_vehicle_model::InterconnectedModel`](https://github.com/autowarefoundation/autoware.universe/tree/main/simulator/autoware_learning_based_vehicle_model).

Constructor:

- Loops over the parallel arrays `model_python_paths`, `model_param_paths`,
  `model_class_names` and calls `vehicle.addSubmodel({path, param_path, class_name})` for
  each — this loads the Python class from the path and instantiates it with the param file.
- `vehicle.generateConnections(input_names, state_names)` wires the submodels' state/input
  signals (`POS_X`, `POS_Y`, `YAW`, `YAW_RATE`, `VX`, `VY`, `STEER`, `VX_DES`, `STEER_DES`)
  together.
- `vehicle.dtSet(dt)` sets the integration step.

`update(dt)`:

1. Convert `input_` (Eigen) and `state_` (Eigen) to `std::vector<double>`.
2. `vehicle.initState(new_state)` — sync the external Python model with the C++ side.
3. `vehicle_state_ = vehicle.updatePyModel(vehicle_input_)` — runs the Python-side step.
4. Copy the result back into `state_`.
5. `current_ax_ = (vx_des - prev_vx_) / dt` — same trick as the ideal model.

`calcModel` is **not used** (returns zero) — the Python model has its own integrator.

---

## 4. [`utils/csv_loader.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/utils/csv_loader.cpp)

Implements every static and instance method declared in
[`csv_loader.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/utils/csv_loader.hpp):

- **`readCSV(result, delim=',')`**: opens the file (returns false on failure), reads each
  line with `std::getline`, splits on `delim` via `std::istringstream`, pushes each non-empty
  token vector into `result`. Calls `validateData(result, csv_path_)` before returning.

- **`validateMap(map, is_col_decent)`**: walks every cell `(i, j)` for `i >= 1`, asserts
  monotonicity vs `(i-1, j)`. Logs the offending `(i, j)` to `cerr` and returns false on
  violation.

- **`validateData(table, csv_path)`**: rejects empty tables, tables with fewer than 2
  columns, and tables with rows of unequal length.

- **`getMap(table)`**: drops row 0 and column 0, parses everything else as `double`.

- **`getColumnIndex(table)`**: returns `table[0][1..]` as doubles (the velocity axis).

- **`getRowIndex(table)`**: returns `table[1..][0]` as doubles (the actuation/acc axis).

- **`clampValue(val, ranges)`**: clamps to `[min(ranges), max(ranges)]`.

---

## 5. [`utils/mechanical_controller.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/utils/mechanical_controller.cpp)

Implements the steering-column model.

### 5.1 Free functions

- **`delay(signal, delay_time, buffer, elapsed_time)`** — pushes the new sample with its
  timestamp; if the front sample is at least `delay_time` seconds old, pops and returns it.
  Otherwise returns `{nullopt, new_buffer}`.

- **`sign(x)`** — returns `+1` if `x ≥ 0`, else `-1`.

- **`apply_limits(current, prev, angle_limit, rate_limit, dt)`**:
  ```
  diff       = clamp(current, [-A,+A]) - prev
  diff       = clamp(diff,    [-R*dt, +R*dt])
  return clamp(prev + diff, [-A,+A])
  ```
  i.e. clamp absolute angle, then rate, then absolute angle again.

- **`feedforward(input_angle, ff_gain)`** — `ff_gain * input_angle`.

- **`polynomial_transform(τ, v, a..h)`**:
  ```
  a τ³ + b τ² + c τ + d v³ + e v² + f v + g τ v + h
  ```
  (Calibrated polynomial that converts the controller-side torque into the actual torque
  applied to the steering column, as a function of vehicle speed.)

### 5.2 `PIDController`

```cpp
double compute(error, integral, prev_error, dt) const {
  return kp_*error + ki_*integral + (dt < 1e-6 ? 0 : kd_*(error - prev_error)/dt);
}

void update_state(error, dt) {
  state_.integral += error*dt;
  state_.error     = error;
}

void update_state(k1_e, k2_e, k3_e, k4_e, dt) {        // RK4-weighted update
  state_.error     = (k1_e + 2*k2_e + 2*k3_e + k4_e) / 6.0;
  state_.integral += state_.error * dt;
}
```

`clear_state()` resets to `{0, 0}`.

### 5.3 `SteeringDynamics`

```cpp
SteeringDynamicsDeltaState calc_model(state, τ_in) const {
  friction_force = friction_ * sign(state.angular_velocity);
  α = (τ_in - damping_*state.angular_velocity
            - stiffness_*state.angular_position
            - friction_force) / inertia_;
  return { state.angular_velocity, α };          // {dθ, dω}
}

bool is_in_dead_zone(state, τ_in) const {
  if (sign(τ_in) ≠ sign(state.angular_velocity)
      && |τ_in| < dead_zone_threshold_)        return true;
  if (state.is_in_dead_zone)
    return !(dead_zone_threshold_ <= |τ_in|);
  return false;
}
```

i.e. enters dead-zone when input torque is small **and points opposite to current motion**;
exits dead-zone when input torque grows beyond the threshold.

### 5.4 `MechanicalController::run_one_step(...)`

Single-shot computation; does **not** mutate any member state. Pure function:

1. `limited_input = apply_limits(input_angle, prev_input_angle, A, R, dt)`
2. `ff_torque = feedforward(limited_input, ff_gain)`
3. `pid_error = limited_input - dynamics_state.angular_position`
4. `pid_torque = pid.compute(pid_error, pid_state.integral + pid_error*dt, pid_state.error, dt)`
5. `total_torque = ff_torque + pid_torque`
6. `steering_torque = clamp(polynomial_transform(total_torque, |speed|, …), [-T_lim, +T_lim])`
7. `(delayed_torque?, new_buffer) = delay(steering_torque, torque_delay_time, buffer, elapsed)`
8. If no delayed torque yet ⇒ return `{new_buffer, pid_error, {0,0}, prev_dead_zone}`.
9. If `is_in_dead_zone(state, steering_torque)` ⇒ return `{new_buffer, pid_error, {0,0}, true}`.
10. Else `d_state = dynamics.calc_model(dynamics_state, steering_torque)` ⇒
    `{new_buffer, pid_error, d_state, false}`.

### 5.5 `MechanicalController::update_runge_kutta(input_angle, speed, prev_input_angle, dt)`

The "real" entry point:

1. Calls `run_one_step` four times (k₁..k₄), each time with a perturbed copy of the
   `SteeringDynamics` (its angular position/velocity advanced by half-step / full-step of the
   previous derivative). The PID controller and delay buffer are read-only inside
   `run_one_step`; they are not advanced between k-steps.
2. Averages the dynamics derivatives à la classical RK4:
   `d̄θ = (k1 + 2k2 + 2k3 + k4) / 6`, same for `d̄ω`.
3. Updates the dynamics state once, with this averaged derivative, applying angle/rate
   clamps:
   ```cpp
   new.angular_position = clamp(old + d̄θ * dt, [-A, A]);
   new.angular_velocity = clamp(old + d̄ω * dt, [-R, R]);
   ```
4. Updates the PID state via the RK4 overload:
   `pid_.update_state(k1.pid_error, k2.pid_error, k3.pid_error, k4.pid_error, dt);`
5. Updates the delay buffer using the average of the four delayed torques produced inside
   `run_one_step`. The `is_in_dead_zone` flag is recomputed against the new state and the
   averaged delayed signal.
6. Returns `new.angular_position` — the new steering tire angle.

`update_euler` is the simpler one-step variant (kept for completeness / debugging).

---

## 6. Cross-references

- The headers (and the contracts they impose) are in [`include.md`](include.md).
- The bundled CSVs and how they're shaped: [`data.md`](data.md) and [`param.md`](param.md).
- The matrix of which parameter applies to which model is in
  [`README.md` "Inner-workings / Algorithms"](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/README.md).
