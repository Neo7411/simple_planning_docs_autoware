# `test/` — Unit Tests

```
test/
├── test_simple_planning_simulator.cpp
└── actuation_cmd_map/
    ├── accel_map.csv
    ├── brake_map.csv
    └── steer_map.csv
```

The test target is registered in
[`CMakeLists.txt`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/CMakeLists.txt):

```cmake
if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_simple_planning_simulator
    test/test_simple_planning_simulator.cpp
    TIMEOUT 120
  )
  target_link_libraries(test_simple_planning_simulator
    ${PROJECT_NAME}
    ament_index_cpp::ament_index_cpp
  )
endif()
```

`ament_add_ros_isolated_gtest` runs each test in its own process so `rclcpp::init/shutdown`
can be called fresh per case (which the test does — see §1.4 below).

---

## 1. [`test_simple_planning_simulator.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp)

A **parameterised** gtest that exercises every vehicle model the simulator supports.

### 1.1 Direction-checking helpers

The test asserts that, given a control command, the vehicle ends up roughly where it should
in the body frame:

```
                      y
                      |
                      |         (Fwd-Left)
                      |
  ---------(Bwd)------------------(Fwd)----------> x
                      |
        (Bwd-Right)   |
                      |
```

| Helper | Asserts |
|---|---|
| `isOnForward(state, init)` | `dx > 1.0 m` |
| `isOnBackward(state, init)` | `dx < -1.0 m` |
| `isOnForwardLeft(state, init)` | `dx > 1.0 m && dy > 0.1 m` |
| `isOnBackwardRight(state, init)` | `dx < -1.0 m && dy < -0.1 m` |

Each prints a diagnostic showing `dx`, the threshold, and the full odometry strings so a
regression is easy to debug.

### 1.2 The `PubSubNode` helper

A second `rclcpp::Node` used as a fake "rest of the system":

- **Subscribes** to `output/odometry` (cached in `current_odom_`).
- **Publishes** to:
  - `input/ackermann_control_command`
  - `input/actuation_command`
  - `input/initialpose`
  - `input/gear_command`

Its publishers/subscriber are public so the test can poke them directly.

### 1.3 `Ackermann` and `Actuation` structs

```cpp
struct Ackermann { double steer, steer_rate, vel, acc, jerk; };
struct Actuation { double steer, accel, brake; };
```

`ackermannCmdGen(t, ackermann)` and `actuationCmdGen(t, actuation)` build the corresponding
ROS messages.

`sendAckermannCommand(cmd, sim_node, pub_sub_node)` and `sendActuationCommand(...)` each
publish the command **150 times** at 100 Hz (10 ms sleeps), spinning both nodes between
publications. This drives the simulation forward for 1.5 s of wall-clock per command —
enough for the vehicle to leave the initial pose by more than the 1.0 m threshold.

`sendCommand(cmd_type, …, ackermann_cmd, actuation_cmd)` dispatches to the right
`send*Command(...)` based on the parameterised `CommandType` (Ackermann vs Actuation).

### 1.4 The big test fixture: `TestSimplePlanningSimulator`

```cpp
using DefaultParamType = std::tuple<CommandType, std::string>;
using ParamType        = std::variant<DefaultParamType /*,AdditionalParamType*/>;
```

`TestSimplePlanningSimulator` derives from `::testing::TestWithParam<ParamType>` so each
test instance runs against a `(command_type, vehicle_model_type)` tuple.

#### `TEST_P(TestSimplePlanningSimulator, TestIdealSteerVel)` ([line 294](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp#L294))

The single test body. Despite the name `TestIdealSteerVel`, it's the body for **every**
parameterised case — the name is historic. Steps:

1. **`rclcpp::init(0, nullptr)`**.
2. **Parameter overrides** (`node_options.append_parameter_override(...)`):
   - `initialize_source = INITIAL_POSE_TOPIC`
   - `vehicle_model_type` ← from the parameterised tuple
   - `initial_engage_state = true`
   - `add_measurement_noise = false`
   - All `ACTUATION_CMD*`-related params (`accel_time_delay`, `brake_*`, `convert_*`,
     `accel_map_path`, `brake_map_path`, `steer_map_path`, `vgr_coef_*`, all
     `mechanical_params.*`) — populated from
     [`test/actuation_cmd_map/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/)
     so the test doesn't depend on the production CSVs in `data/`. The map paths are
     resolved via `ament_index_cpp::get_package_share_directory("autoware_simple_planning_simulator")`.
   - `declareVehicleInfoParams(node_options)` injects `wheel_base = 3.0` (and friends) so
     `VehicleInfoUtils` works without a real vehicle-info YAML.
3. **Construct** `SimplePlanningSimulator` and `PubSubNode`.
4. **Setup helpers**:
   - `_resetInitialpose()` — publish 10 times at the origin so the node initialises.
   - `_sendFwdGear()` / `_sendBwdGear()` — publish 10 GearCommands.
   - `_sendCommand(ack, act)` — generates and sends 150 commands.
   - `_restartNode()` — destroys and recreates the simulator node (the model carries
     internal queue/state that would corrupt the next sub-test).
5. **Connection check**: command/gear/initialpose subscribers and the odometry publisher
   should all have exactly 1 counterpart.
6. **Initial pose** is read into `init_state` after the first `_resetInitialpose()`.
7. **Four sub-scenarios**, each restarting the node:
   - **Forward (D + zero steer + +vel + +acc)** ⇒ `isOnForward(curr, init_state)`.
   - **Backward (R + zero steer + -vel + +acc)** ⇒ `isOnBackward(curr, init_state)`.
     Note: positive acc with reverse gear drives backward — see `set_input` in
     [`src.md`](src.md) §1.8.
   - **Forward-left (D + +steer + +vel)** ⇒ `isOnForwardLeft(curr, init_state)`.
   - **Backward-right (R + -steer + -vel)** ⇒ `isOnBackwardRight(curr, init_state)`.
   For Actuation models, the `steer_actuation = 20.0`, `accel_actuation = 0.5` constants
   are chosen *knowing* the test maps — the comment notes this dependency:

   > "As the value of the actuation map is known, roughly determine whether it is
   > acceleration or braking, and whether it turns left or right, and generate an actuation
   > command. So do not change the map. If it is necessary, you need to change this
   > parameters as well."

8. **`rclcpp::shutdown()`**.

### 1.5 `INSTANTIATE_TEST_SUITE_P`

```cpp
INSTANTIATE_TEST_SUITE_P(
  TestForEachVehicleModelTrue, TestSimplePlanningSimulator,
  ::testing::Values(
    /* Ackermann type */
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_VEL"),
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_ACC"),
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_ACC_GEARED"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_VEL"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC_GEARED"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD"),
    /* Actuation type */
    std::make_tuple(CommandType::Actuation, "ACTUATION_CMD_STEER_MAP"),
    std::make_tuple(CommandType::Actuation, "ACTUATION_CMD_VGR"),
    std::make_tuple(CommandType::Actuation, "ACTUATION_CMD_MECHANICAL")));
```

So the test runs **10 cases** total:

| # | Command type | Model |
|---|---|---|
| 1 | Ackermann | `IDEAL_STEER_VEL` |
| 2 | Ackermann | `IDEAL_STEER_ACC` |
| 3 | Ackermann | `IDEAL_STEER_ACC_GEARED` |
| 4 | Ackermann | `DELAY_STEER_VEL` |
| 5 | Ackermann | `DELAY_STEER_ACC` |
| 6 | Ackermann | `DELAY_STEER_ACC_GEARED` |
| 7 | Ackermann | `DELAY_STEER_ACC_GEARED_WO_FALL_GUARD` |
| 8 | Actuation | `ACTUATION_CMD_STEER_MAP` |
| 9 | Actuation | `ACTUATION_CMD_VGR` |
| 10 | Actuation | `ACTUATION_CMD_MECHANICAL` |

Each runs the same 4 sub-scenarios ⇒ **40 sub-assertions** in total.

**Coverage gaps** the test author explicitly notes:

- **`ACTUATION_CMD`** (the base actuation model that converts only accel/brake but passes
  steer through) is skipped — *"the test is performed for models that convert all
  accel/brake/steer commands, so the test for accel/brake alone is skipped"*.
- **`DELAY_STEER_MAP_ACC_GEARED`** is not exercised here. It would need a separate
  acceleration-map CSV.
- **`LEARNED_STEER_VEL`** is not exercised — would need a Python environment with the
  `control_analysis_pipeline` library installed.

---

## 2. [`test/actuation_cmd_map/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/)

Three CSV files used by the test target. They have **the same shape** as the production
CSVs in [`data/actuation_cmd_map/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/data/actuation_cmd_map/)
— see [`data.md`](data.md). The test versions are minimal/canonical so the test runs fast
and so the magic numbers in the test (`target_steer_actuation = 20.0`,
`target_accel_actuation = 0.5`) produce predictable directions.

| File | Bytes | Comment |
|---|---|---|
| [`accel_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/accel_map.csv) | 87 | Tiny — only enough rows/cols to enable interpolation. |
| [`brake_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/brake_map.csv) | 716 | Larger; covers the full brake-pedal range. |
| [`steer_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/steer_map.csv) | 67 | The smallest — just enough to make `getControlCommand` return non-zero. |

`CMakeLists.txt`'s `ament_auto_package(INSTALL_TO_SHARE param data launch test)` line
ensures the `test/` directory is installed into the package's share so that
`ament_index_cpp::get_package_share_directory(...)` lookups work after a colcon build.

---

## 3. Running the tests

```bash
# From the workspace root:
colcon build --packages-up-to autoware_simple_planning_simulator
colcon test --packages-select autoware_simple_planning_simulator
colcon test-result --verbose
```

The 120-second timeout in CMakeLists is generous — locally the full 10-case suite usually
finishes in under 30 s. If you see timeouts, the wall-clock-based 1.5 s × 4 sub-scenarios
× 10 cases = 60 s lower bound is the suspect — the test relies on real-time `sleep_for`,
not simulated time.

---

## 4. Cross-references

- The simulator class under test: [`src.md` §1](src.md).
- The CSV format expected by the test maps: [`data.md`](data.md).
- The `vehicle_model_type` strings: [`include.md` §5](include.md).
