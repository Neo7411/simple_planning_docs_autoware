# `include/` — Public Headers

All public headers live under
[`include/autoware/simple_planning_simulator/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/).
This is the **API surface** of the package; everything else (the `.cpp` files) is the
implementation.

```
include/autoware/simple_planning_simulator/
├── simple_planning_simulator_core.hpp     ← the ROS node class
├── visibility_control.hpp                  ← DLL export macros
├── utils/
│   ├── csv_loader.hpp                      ← CSV → 2D map loader
│   └── mechanical_controller.hpp           ← PID + steering dynamics
└── vehicle_model/
    ├── sim_model.hpp                       ← umbrella include
    ├── sim_model_interface.hpp             ← abstract base class
    ├── sim_model_ideal_steer_vel.hpp
    ├── sim_model_ideal_steer_acc.hpp
    ├── sim_model_ideal_steer_acc_geared.hpp
    ├── sim_model_delay_steer_vel.hpp
    ├── sim_model_delay_steer_acc.hpp
    ├── sim_model_delay_steer_acc_geared.hpp
    ├── sim_model_delay_steer_acc_geared_wo_fall_guard.hpp
    ├── sim_model_delay_steer_map_acc_geared.hpp
    ├── sim_model_actuation_cmd.hpp
    └── sim_model_learned_steer_vel.hpp
```

---

## 1. [`simple_planning_simulator_core.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/simple_planning_simulator_core.hpp)

The single ROS-facing class plus two small supporting types.

### 1.1 `class DeltaTime`

A **wall-clock delta calculator**. On each call to `get_dt(now)` it diffs the supplied
`rclcpp::Time` against the previously stored one and returns the elapsed seconds. The first
call returns `0.0` (no previous value yet). Used by the timer callback to feed `dt` into the
vehicle-model integrator.

```cpp
double DeltaTime::get_dt(const rclcpp::Time & now);
```

### 1.2 `class MeasurementNoiseGenerator`

Holds the four `std::normal_distribution<>` and the `std::mt19937` random engine used by
`SimplePlanningSimulator::add_measurement_noise(...)`:

| Member | Used to noise |
|---|---|
| `pos_dist_` | `odom.pose.pose.position.x/y` |
| `vel_dist_` | `odom.twist.twist.linear.x` and `vel.longitudinal_velocity` |
| `rpy_dist_` | yaw before being repacked into a quaternion |
| `steer_dist_` | `steer.steering_tire_angle` |

Each is constructed in the node's ctor with `stddev` parameters from
`pos_noise_stddev`, `vel_noise_stddev`, `rpy_noise_stddev`, `steer_noise_stddev`.

### 1.3 `using InputCommand`

```cpp
using InputCommand = std::variant<std::monostate, ActuationCommandStamped, Control>;
```

A tagged union of "what the active vehicle model expects on its input topic":

- `std::monostate` → not yet set,
- `Control` → autoware Ackermann command (default),
- `ActuationCommandStamped` → pedal/steer command (only when `vehicle_model_type` contains
  `"ACTUATION_CMD"`).

`set_input(InputCommand, acc_by_slope)` uses `std::visit` to dispatch to the right overload.

### 1.4 `class SimplePlanningSimulator : public rclcpp::Node`

The whole simulator. Its public surface is just the constructor:

```cpp
explicit SimplePlanningSimulator(const rclcpp::NodeOptions & options);
```

Everything else is private. Its members fall into eight groups.

#### A) Publishers (12)

```
pub_velocity_                 (autoware_vehicle_msgs/VelocityReport)        ← output/twist
pub_odom_                     (nav_msgs/Odometry)                           ← output/odometry
pub_steer_                    (autoware_vehicle_msgs/SteeringReport)        ← output/steering (gated)
pub_acc_                      (geometry_msgs/AccelWithCovarianceStamped)    ← output/acceleration
pub_imu_                      (sensor_msgs/Imu)                             ← output/imu
pub_control_mode_report_      (autoware_vehicle_msgs/ControlModeReport)     ← output/control_mode_report
pub_gear_report_              (autoware_vehicle_msgs/GearReport)            ← output/gear_report
pub_turn_indicators_report_   (autoware_vehicle_msgs/TurnIndicatorsReport)  ← output/turn_indicators_report
pub_hazard_lights_report_     (autoware_vehicle_msgs/HazardLightsReport)    ← output/hazard_lights_report
pub_tf_                       (tf2_msgs/TFMessage)                          ← /tf
pub_current_pose_             (geometry_msgs/PoseWithCovarianceStamped)     ← output/pose
pub_actuation_status_         (tier4_vehicle_msgs/ActuationStatusStamped)   ← output/actuation_status
```

#### B) Subscriptions (10)

```
sub_gear_cmd_, sub_manual_gear_cmd_              GearCommand
sub_turn_indicators_cmd_, sub_hazard_lights_cmd_ TurnIndicatorsCommand / HazardLightsCommand
sub_manual_ackermann_cmd_                        Control (manual)
sub_map_                                         LaneletMapBin (transient_local QoS)
sub_init_pose_, sub_init_twist_                  PoseWithCovarianceStamped, TwistStamped
sub_trajectory_                                  Trajectory
sub_engage_                                      Engage
sub_ackermann_cmd_  XOR  sub_actuation_cmd_      Control / ActuationCommandStamped
                                                 (only one is created, depending on the model)
```

#### C) Services (2)

```
srv_mode_req_     (autoware_vehicle_msgs/srv/ControlModeCommand)
srv_set_pose_     (tier4_external_api_msgs/srv/InitializePose,  via tier4_api_utils)
```

`srv_set_pose_` is created on its own callback group `group_api_service_`
(`MutuallyExclusive`), so external API calls cannot reentrantly recurse into the timer.

#### D) Timer

```
uint32_t timer_sampling_time_ms_;
rclcpp::TimerBase::SharedPtr on_timer_;
```

Constructed in the ctor with default `25 ms` (40 Hz) and bound to `on_timer()`.

#### E) tf2 plumbing

```
tf2_ros::Buffer        tf_buffer_;
tf2_ros::TransformListener  tf_listener_;
```

Used inside `get_transform_msg(parent, child)` to resolve initial poses given in arbitrary
frames into `origin_frame_id_`.

#### F) Cached state and parameters

| Field | Notes |
|---|---|
| `road_lanelets_` | All lanelets tagged "road" — fed into `calculate_ego_pitch()`. |
| `initial_pose_`, `initial_twist_` | Cached so a later twist re-init doesn't lose pose. |
| `current_velocity_`, `current_odometry_`, `current_steer_` | Latest published state. |
| `current_ackermann_cmd_`, `current_manual_ackermann_cmd_` | Inputs from autonomy / teleop. |
| `current_gear_cmd_`, `current_manual_gear_cmd_` | Gear from autonomy / teleop. |
| `current_turn_indicators_cmd_ptr_`, `current_hazard_lights_cmd_ptr_` | Echoed back as reports. |
| `current_trajectory_ptr_` | Used by `get_z_pose_from_trajectory(...)`. |
| `simulate_motion_` | Engage flag — when `false` the model is not stepped. |
| `current_control_mode_` | `AUTONOMOUS` (default) / `MANUAL`. |
| `enable_road_slope_simulation_` | Whether to add gravity component along x. |
| `enable_pub_steer_` | If false, steering is republished by an external converter. |
| `simulated_frame_id_`, `origin_frame_id_` | TF naming. |
| `is_initialized_`, `add_measurement_noise_` | Bool flags. |
| `current_input_command_` (`InputCommand`) | Dispatcher source. |
| `delta_time_` | Wall-clock dt source. |
| `measurement_noise_` | Random distributions. |
| `x_stddev_`, `y_stddev_` | Dummy covariance for the EKF downstream. |

#### G) Vehicle model

```cpp
enum class VehicleModelType {
  IDEAL_STEER_ACC = 0,
  IDEAL_STEER_ACC_GEARED = 1,
  DELAY_STEER_ACC = 2,
  DELAY_STEER_ACC_GEARED = 3,
  IDEAL_STEER_VEL = 4,
  DELAY_STEER_VEL = 5,
  DELAY_STEER_MAP_ACC_GEARED = 6,
  LEARNED_STEER_VEL = 7,
  DELAY_STEER_ACC_GEARED_WO_FALL_GUARD = 8,
  ACTUATION_CMD = 9,
  ACTUATION_CMD_VGR = 10,
  ACTUATION_CMD_MECHANICAL = 11,
  ACTUATION_CMD_STEER_MAP = 12,
} vehicle_model_type_;
std::shared_ptr<SimModelInterface> vehicle_model_ptr_;
```

The polymorphic backbone — see [`sim_model_interface.hpp`](#2-sim_model_interfacehpp) below.

#### H) Member functions (private)

The full list, grouped by purpose:

**Lifecycle / dispatch**

```cpp
void on_timer();
void initialize_vehicle_model(const std::string & vehicle_model_type_str);
void set_input(const InputCommand & cmd, const double acc_by_slope);
void set_input(const Control & cmd, const double acc_by_slope);
void set_input(const ActuationCommandStamped & cmd, const double acc_by_slope);
rcl_interfaces::msg::SetParametersResult on_parameter(
  const std::vector<rclcpp::Parameter> & parameters);
```

**Subscription callbacks**

```cpp
void on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg);
void on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg);
void on_map(const LaneletMapBin::ConstSharedPtr msg);
void on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg);
void on_initialtwist(const TwistStamped::ConstSharedPtr msg);
void on_trajectory(const Trajectory::ConstSharedPtr msg);
void on_engage(const Engage::ConstSharedPtr msg);
```

**Service callbacks**

```cpp
void on_set_pose(const InitializePose::Request::ConstSharedPtr request,
                 const InitializePose::Response::SharedPtr response);
void on_control_mode_request(const ControlModeCommand::Request::ConstSharedPtr request,
                             const ControlModeCommand::Response::SharedPtr response);
```

**Helpers**

```cpp
double get_z_pose_from_trajectory(double x, double y, const Odometry & prev_odometry);
TransformStamped get_transform_msg(std::string parent_frame, std::string child_frame);
double calculate_ego_pitch() const;
void add_measurement_noise(Odometry & odom, VelocityReport & vel, SteeringReport & steer) const;
void set_initial_state(const Pose & pose, const Twist & twist);
void set_initial_state_with_transform(const PoseStamped & pose, const Twist & twist);
```

**Publishers**

```cpp
void publish_velocity(const VelocityReport &);
void publish_odometry(const Odometry &);
void publish_pose(const Odometry &);
void publish_steering(const SteeringReport &);
void publish_acceleration();
void publish_imu();
void publish_control_mode_report();
void publish_gear_report();
void publish_turn_indicators_report();
void publish_hazard_lights_report();
void publish_actuation_status();
void publish_tf(const Odometry &);
```

What each one actually does is in [`src.md`](src.md) §1.

---

## 2. [`visibility_control.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/visibility_control.hpp)

Tiny header that defines two preprocessor macros, `PLANNING_SIMULATOR_PUBLIC` and
`PLANNING_SIMULATOR_LOCAL`, for cross-platform symbol visibility:

| Platform | `PUBLIC` | `LOCAL` |
|---|---|---|
| `__WIN32` (DLL build) | `__declspec(dllexport)` | (empty) |
| `__WIN32` (consumer) | `__declspec(dllimport)` | (empty) |
| `__linux__` | `__attribute__((visibility("default")))` | `__attribute__((visibility("hidden")))` |
| `__APPLE__` | same as Linux | same as Linux |
| anything else | `#error "Unsupported Build Configuration"` |

The only place `PLANNING_SIMULATOR_PUBLIC` is currently used is on the `SimplePlanningSimulator`
class itself, which is what you need so the symbol is exported when the shared library is built.

---

## 3. [`vehicle_model/sim_model.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model.hpp)

A pure **umbrella header**. It only `#include`s every vehicle-model header so a translation
unit that does `#include "…/vehicle_model/sim_model.hpp"` gets all of them at once. Order:

1. `sim_model_actuation_cmd.hpp`
2. `sim_model_delay_steer_acc.hpp`
3. `sim_model_delay_steer_acc_geared.hpp`
4. `sim_model_delay_steer_acc_geared_wo_fall_guard.hpp`
5. `sim_model_delay_steer_map_acc_geared.hpp`
6. `sim_model_delay_steer_vel.hpp`
7. `sim_model_ideal_steer_acc.hpp`
8. `sim_model_ideal_steer_acc_geared.hpp`
9. `sim_model_ideal_steer_vel.hpp`
10. `sim_model_interface.hpp`
11. `sim_model_learned_steer_vel.hpp`

Used by `simple_planning_simulator_core.cpp` only.

---

## 4. [`vehicle_model/sim_model_interface.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_interface.hpp)

The **abstract base class** that every concrete vehicle model implements.

```cpp
class SimModelInterface
{
protected:
  const int dim_x_, dim_u_;          // dimensions of state and input
  Eigen::VectorXd state_, input_;    // current state and input
  uint8_t gear_ = GearCommand::DRIVE;

public:
  SimModelInterface(int dim_x, int dim_u);
  virtual ~SimModelInterface() = default;

  // Read / write state and input
  void getState(Eigen::VectorXd & state);
  void getInput(Eigen::VectorXd & input);
  void setInput(const Eigen::VectorXd & input);
  void setGear(uint8_t gear);
  virtual void setState(const Eigen::VectorXd & state);   // virtual: some models cache extra

  // Integrators (concrete)
  void updateRungeKutta(const double & dt, const Eigen::VectorXd & input);
  void updateEuler(const double & dt, const Eigen::VectorXd & input);

  // Pure virtual interface
  virtual void update(const double & dt) = 0;
  virtual Eigen::VectorXd calcModel(
      const Eigen::VectorXd & state, const Eigen::VectorXd & input) = 0;

  virtual double getX() = 0;
  virtual double getY() = 0;
  virtual double getYaw() = 0;
  virtual double getVx() = 0;
  virtual double getVy() = 0;
  virtual double getAx() = 0;
  virtual double getWz() = 0;
  virtual double getSteer() = 0;

  uint8_t getGear() const;
  inline int getDimX() { return dim_x_; }
  inline int getDimU() { return dim_u_; }

  // Optional: actuation status
  virtual bool shouldPublishActuationStatus() const { return false; }
  virtual std::optional<ActuationStatusStamped> getActuationStatus() const { return std::nullopt; }
};
```

**Key contract:**

- A concrete model promises to:
  - Define its `state_` layout (an `enum IDX`) and `input_` layout (an `enum IDX_U`).
  - Implement `calcModel(state, input)` returning **`dx/dt`** as an `Eigen::VectorXd` of size
    `dim_x_`.
  - Implement `update(dt)`, which usually:
    1. Pulls the current `input_` (possibly through a delay queue),
    2. Calls `updateRungeKutta(dt, delayed_input)` (or `updateEuler` for the simple ideal
       models), which itself calls `calcModel` 4× (RK4) or 1× (Euler) and updates `state_`,
    3. Applies any post-integration constraints (clamping VX to `[-vx_lim, +vx_lim]`, gear
       handling, etc.).
- Position / orientation / velocity getters read from `state_`; some models compute derived
  quantities (e.g. `Wz = Vx * tan(steer) / wheelbase`).

The four optional functions (`shouldPublishActuationStatus`, `getActuationStatus`,
`setState`'s virtual override) are only actually implemented by `SimModelActuationCmd*` and
`SimModelActuationCmdMechanical`.

---

## 5. The vehicle-model headers (concrete classes)

Each header declares a class deriving from `SimModelInterface`. The pattern is:

- A constructor that takes the relevant physical limits and time-constants.
- An `enum IDX { … }` for the state layout.
- An `enum IDX_U { … }` for the input layout.
- Members for limits, time-constants, dead bands, and (when delayed) a `std::deque<double>`
  per delayed channel acting as a **FIFO delay buffer**.
- The same set of `getX()/...getSteer()` accessors implementing `SimModelInterface`.
- Their own `update(dt)` and `calcModel(state, input)`.

The table below summarises **every concrete model**:

| Header | State layout (`IDX`) | Input layout (`IDX_U`) | dim_x / dim_u | Notes |
|---|---|---|---|---|
| [`sim_model_ideal_steer_vel.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.hpp) | `X, Y, YAW` | `VX_DES, STEER_DES` | 3 / 2 | Ideal kinematic bicycle, vel cmd. |
| [`sim_model_ideal_steer_acc.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.hpp) | `X, Y, YAW, VX` | `AX_DES, STEER_DES` | 4 / 2 | Same, accel cmd. |
| [`sim_model_ideal_steer_acc_geared.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.hpp) | `X, Y, YAW, VX` | `AX_DES, STEER_DES` | 4 / 2 | Adds gear-aware `updateStateWithGear`. |
| [`sim_model_delay_steer_vel.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel.hpp) | `X, Y, YAW, VX, STEER` | `VX_DES, STEER_DES` | 5 / 2 | 1st-order vel/steer dyn + dead time. Has `vx_input_queue_`, `steer_input_queue_`. |
| [`sim_model_delay_steer_acc.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.hpp) | `X, Y, YAW, VX, STEER, ACCX` | `ACCX_DES, STEER_DES, DRIVE_SHIFT` | 6 / 2 | 1st-order acc/steer dyn. |
| [`sim_model_delay_steer_acc_geared.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.hpp) | same as above | same | 6 / 2 | Same + gear-aware. |
| [`sim_model_delay_steer_acc_geared_wo_fall_guard.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.hpp) | `X, Y, YAW, VX, STEER, ACCX, PEDAL_ACCX` | `PEDAL_ACCX_DES, GEAR, SLOPE_ACCX, STEER_DES` | 7 / 4 | Same, but no "snap to 0" fall-guard at low speed. |
| [`sim_model_delay_steer_map_acc_geared.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_map_acc_geared.hpp) | `X, Y, YAW, VX, STEER, ACCX` | `ACCX_DES, STEER_DES, DRIVE_SHIFT` | 6 / 2 | Reads `acceleration_map.csv` via embedded `AccelerationMap` class — desired acc is remapped to actual acc via bilinear interpolation `(acc_des, vel) → acc`. |
| [`sim_model_actuation_cmd.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.hpp) | `X, Y, YAW, VX, STEER, ACCX` | `ACCEL_DES, BRAKE_DES, SLOPE_ACCX, STEER_DES, GEAR` | 6 / 5 | Receives **pedal/steer** commands; pedals are converted to accel through `AccelMap`/`BrakeMap`. Has 3 derived classes: `SimModelActuationCmdSteerMap` (steer-rate via map), `SimModelActuationCmdVGR` (variable gear ratio), `SimModelActuationCmdMechanical` (PID + 2nd-order steering inertia/damping/stiffness). |
| [`sim_model_learned_steer_vel.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_learned_steer_vel.hpp) | `X, Y, YAW, YAW_RATE, VX, VY, STEER` | `VX_DES, STEER_DES` | 7 / 2 | Wraps `learning_based_vehicle_model::InterconnectedModel` — calls user-supplied **Python** classes by string name (e.g. `KinematicModel`, `SteerExample`, `DriveExample`). `calcModel` returns zeros (RK4 unused). |

For the actual update equations and the math, see [`src.md`](src.md) §3.

### 5.1 Helper classes embedded in two of those headers

Two of the model headers also contain small helper classes that are declared inline:

- **[`AccelerationMap`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_map_acc_geared.hpp#L33-L80)** in
  `sim_model_delay_steer_map_acc_geared.hpp`. Wraps a CSV table `(acc_des row × vel column → actual_acc)`.
  - `readAccelerationMapFromCSV(path)` parses the file.
  - `getAcceleration(acc_des, vel)` first interpolates each row at the `vel` column, then
    interpolates the resulting vector at `acc_des` — both via
    `autoware::interpolation::lerp`. Out-of-range queries are clamped via
    `CSVLoader::clampValue(...)`.

- **`ActuationMap`, `AccelMap`, `BrakeMap`** in `sim_model_actuation_cmd.hpp`.
  - `ActuationMap::readActuationMapFromCSV(path, validation=false)` parses a CSV file with
    rows = actuation index, columns = state index (typically velocity).
  - `ActuationMap::getControlCommand(actuation, state)` does the same two-pass linear
    interpolation as `AccelerationMap`.
  - `AccelMap::getThrottle(acc, vel)` does the inverse: given a desired acc and the current
    velocity, returns the throttle pedal value that would produce it. Returns `std::nullopt`
    if `acc` is below the minimum the map can produce (≈ caller should switch to brake).
  - `BrakeMap::getBrake(acc, vel)` — analogous, but:
    - Returns the **maximum** brake when `acc` is below the map's minimum,
    - Returns the **minimum** brake when `acc` is above the map's maximum,
    - Reverses the interpolated vector so `lerp` works on a monotonically-increasing axis.

These helpers exist because the actuation models need to round-trip pedal → acc → pedal for
publishing `actuation_status`.

---

## 6. [`utils/csv_loader.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/utils/csv_loader.hpp)

A small utility used to parse the various CSV maps shipped with the package.

### Types

```cpp
using Table = std::vector<std::vector<std::string>>;
using Map   = std::vector<std::vector<double>>;
```

### `class CSVLoader`

| Member | Behaviour |
|---|---|
| `explicit CSVLoader(const std::string & csv_path)` | Stores the path. |
| `bool readCSV(Table & result, char delim=',')` | Opens the file, splits each line on `delim`, populates `result`. Calls `validateData(...)` before returning. |
| `static bool validateData(const Table &, const std::string & csv_path)` | Asserts the table is non-empty, has at least 2 columns, and that **every row has the same number of columns**. Logs to `cerr` and returns false on violation. |
| `static bool validateMap(const Map &, bool is_col_decent)` | Asserts that the map values are monotonic along the column axis (decreasing if `is_col_decent`, increasing otherwise). |
| `static Map getMap(const Table &)` | Strips off the first row and first column (axes labels), parses the remaining cells as `double`. |
| `static std::vector<double> getRowIndex(const Table &)` | Returns column 0 (the row labels) as doubles, skipping `table[0][0]`. |
| `static std::vector<double> getColumnIndex(const Table &)` | Returns row 0 (the column labels) as doubles, skipping `table[0][0]`. |
| `static double clampValue(double val, const std::vector<double> & ranges)` | Clamps `val` to `[min(ranges), max(ranges)]`. |

CSV file format expected:

```
default,  vel0, vel1, vel2, …    ← column index (row 0)
acc0,     a00,  a01,  a02, …     ← first data row, label = acc0 (row index)
acc1,     a10,  a11,  a12, …
…
```

The string `"default"` in `[0][0]` is what every shipped CSV uses; the loader doesn't actually
parse it — it just discards `table[0][0]` and `table[i][0]` (used as labels).

---

## 7. [`utils/mechanical_controller.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/utils/mechanical_controller.hpp)

Models the **steering column** of a real vehicle as a PID controller driving a
spring-damper-friction-inertia 2nd-order system, with input deadtime and a polynomial
torque/speed conversion. Used only by `SimModelActuationCmdMechanical`.

### Free functions

```cpp
DelayOutput delay(double signal, double delay_time,
                  const DelayBuffer & buffer, double elapsed_time);
double sign(double x);                               // simple ±1 sign
double apply_limits(current_angle, previous_angle,    // angle + rate-limit clamp
                    angle_limit, rate_limit, dt);
double feedforward(input_angle, ff_gain);            // ff_gain * input_angle
double polynomial_transform(torque, speed,           // 8-coefficient polynomial
                            a, b, c, d, e, f, g, h); //   to convert input torque + speed
                                                      //   to the actual steering torque
```

`DelayBuffer` is a `std::deque<std::pair<double,double>>` of (signal, timestamp).
`delay(...)` pushes the new sample at the back; if the front sample's timestamp is at least
`delay_time` old, it pops and returns it (else returns `nullopt`).

### `class PIDController`

Stateful PID:

```cpp
PIDController(double kp, double ki, double kd);
double compute(error, integral, prev_error, dt) const;          // returns torque
void   update_state(double error, double dt);                   // Euler integral
void   update_state(k1_e, k2_e, k3_e, k4_e, double dt);          // RK4 weighted integral
PIDControllerState get_state() const;                           // {integral, error}
void   clear_state();
```

### `class SteeringDynamics`

Implements

```
J·θ̈  +  c·θ̇  +  k·θ  +  μ·sign(θ̇)  =  τ_input
```

with a **dead-zone** where the column does not move when |τ| is below threshold and the
input direction opposes the current rotation:

```cpp
SteeringDynamics(angular_position, angular_velocity, inertia, damping, stiffness,
                 friction, dead_zone_threshold);
bool is_in_dead_zone(state, input_torque) const;
SteeringDynamicsDeltaState calc_model(state, input_torque) const;
void set_state(state);
SteeringDynamicsState get_state() const;
void set_steer(double);
void clear_state();
```

`SteeringDynamicsState` carries `{angular_position, angular_velocity, is_in_dead_zone}`.
`SteeringDynamicsDeltaState` carries `{d_angular_position, d_angular_velocity}`.

### `struct MechanicalParams`

The full configuration struct, populated from `mechanical_params.*` ROS parameters:

```
kp, ki, kd          ← PID gains
ff_gain             ← feed-forward gain
dead_zone_threshold ← input torque dead-zone
poly_a … poly_h     ← polynomial_transform coefficients
inertia, damping, stiffness, friction  ← SteeringDynamics
delay_time, torque_delay_time          ← input deadtime
angle_limit, rate_limit                ← apply_limits()
steering_torque_limit                  ← clamp on the polynomial output
```

### `class MechanicalController`

The orchestrator:

```cpp
explicit MechanicalController(const MechanicalParams &);
double update_euler(input_angle, speed, prev_input_angle, dt);
double update_runge_kutta(input_angle, speed, prev_input_angle, dt);   // ← what's used
void   clear_state();
void   set_steer(double);
```

Internally, `run_one_step(...)` runs:

1. `apply_limits(input_angle, prev_input_angle, ...)` → rate-limited input.
2. `feedforward(...) + pid.compute(error, ...)` → total torque.
3. `polynomial_transform(...)` → actual steering torque, clamped to
   `steering_torque_limit`.
4. `delay(...)` → enforces input deadtime.
5. If the delayed torque is available and not in dead-zone,
   `dynamics.calc_model(state, torque)` produces the derivative state.

`update_runge_kutta` runs `run_one_step` four times (with progressively perturbed dynamics
state for k₂/k₃/k₄), then averages the position and velocity derivatives à la classical RK4.
The PID state and delay buffer are updated using a similar RK4-weighted average. The result
is the **new tire steering angle**, which `SimModelActuationCmdMechanical` then writes into
`state_(IDX::STEER)`.

For a visual, see [`media/mechanical_controller.drawio.svg`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/media/mechanical_controller.drawio.svg).

---

## Cross-references

- Implementations of all of the above are in [`src.md`](src.md).
- Parameters that drive these classes: [`param.md`](param.md).
- The bundled CSVs that the maps consume: [`data.md`](data.md), [`param.md`](param.md).
