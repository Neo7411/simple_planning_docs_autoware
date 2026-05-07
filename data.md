# `data/` — Bundled Actuation Maps

```
data/
└── actuation_cmd_map/
    ├── accel_map.csv
    ├── brake_map.csv
    └── steer_map.csv
```

These three CSVs are the **default actuation maps** consumed by the `ACTUATION_CMD*` family
of vehicle models. They are referenced from
[`simple_planning_simulator_mechanical_sample.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml)
via `$(find-pkg-share simple_planning_simulator)/data/actuation_cmd_map/...`.

`CMakeLists.txt` installs the whole `data/` directory into the package's share folder
(`ament_auto_package(INSTALL_TO_SHARE param data launch test)`) so `find-pkg-share`
substitutions resolve at launch time.

The corresponding **test versions** of these CSVs live under
[`test/actuation_cmd_map/`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/actuation_cmd_map/);
see [`test.md`](test.md).

---

## 1. CSV format (common to all three)

```
default, S₀,    S₁,    …,   Sₙ          ← state axis  (velocity in m/s, or steer in rad)
A₀,      v₀₀,  v₀₁,   …,   v₀ₙ
A₁,      v₁₀,  v₁₁,   …,   v₁ₙ
…
Aₘ,      vₘ₀,  vₘ₁,   …,   vₘₙ
```

- **Row 0** holds the column index — typically velocity in m/s. The first cell `[0][0]` is
  the literal string `default` and is ignored by the loader.
- **Column 0** (excluding `[0][0]`) holds the row index — actuation values
  (throttle, brake, or steer).
- **Inside cells** are the response values (acc/m·s⁻², or steer-rate/rad·s⁻¹).

The loader is `CSVLoader` (see [`src.md` §4](src.md)) ⇒
`AccelerationMap::readAccelerationMapFromCSV` /
`ActuationMap::readActuationMapFromCSV`. Look-up is **bilinear**:
clamp axes, linearly interpolate along columns at the given state, then linearly interpolate
the resulting vector at the given actuation.

---

## 2. [`accel_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/data/actuation_cmd_map/accel_map.csv)

```
default,0.0, 1.39, 2.78, 4.17, 5.56, 6.94, 8.33, 9.72, 11.11, 12.50, 13.89
0,    0.30, -0.05, -0.30, -0.39, -0.40, -0.41, -0.42, -0.44, -0.46, -0.48, -0.50
0.1,  0.60,  0.42,  0.24,  0.18,  0.12,  0.05, -0.08, -0.16, -0.20, -0.24, -0.28
0.2,  1.15,  0.98,  0.78,  0.60,  0.48,  0.34,  0.26,  0.20,  0.10,  0.05, -0.03
0.3,  1.75,  1.60,  1.42,  1.30,  1.14,  1.00,  0.90,  0.80,  0.72,  0.64,  0.58
0.4,  2.65,  2.48,  2.30,  2.13,  1.95,  1.75,  1.58,  1.45,  1.32,  1.20,  1.10
0.5,  3.30,  3.25,  3.12,  2.92,  2.68,  2.35,  2.17,  1.98,  1.88,  1.73,  1.61
```

- **Velocity axis** (columns): 0 m/s → 13.89 m/s in 11 steps of ≈1.39 m/s (≈ 5 km/h
  increments, going up to 50 km/h).
- **Throttle axis** (rows): 0 (no pedal) → 0.5 (50 % pedal) in 6 steps.
- **Cell** = the steady-state longitudinal acceleration in m/s² that this throttle setting
  produces at that velocity.

Physical interpretation:

- The **first row (throttle 0)** shows that at idle the vehicle slightly accelerates from
  rest (rolling at 0 m/s gives `+0.3 m/s²`) but progressively decelerates (engine braking)
  as speed rises — `-0.5 m/s²` at 13.89 m/s.
- Higher rows correspond to stronger throttle, with the response saturating at higher
  velocities (each row is monotone-decreasing along velocity).

This map is consumed by:

- **`AccelMap::getThrottle(acc, vel)`** — given a desired acc and the current velocity,
  returns the throttle pedal value that produces it. Returns `nullopt` if `acc` is below
  the minimum the map can produce (then the caller switches to `BrakeMap`).
- **`SimModelActuationCmd::calcLongitudinalModel`** — when `convert_accel_cmd=true`, the
  pedal command is forward-looked-up via
  `accel_map_.getControlCommand(accel, vel)` to obtain the realised acc.

---

## 3. [`brake_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/data/actuation_cmd_map/brake_map.csv)

```
default,0.0, 1.39, 2.78, 4.17, 5.56, 6.94, 8.33, 9.72, 11.11, 12.50, 13.89
0,    0.30,  -0.05, -0.30, -0.39, -0.40, -0.41, -0.42, -0.44, -0.46, -0.48, -0.50
0.1,  0.29,  -0.06, -0.31, -0.40, -0.41, -0.42, -0.43, -0.45, -0.47, -0.49, -0.51
0.2, -0.38,  -0.40, -0.72, -0.80, -0.82, -0.85, -0.87, -0.89, -0.91, -0.94, -0.96
0.3, -1.00,  -1.04, -1.48, -1.55, -1.57, -1.59, -1.61, -1.63, -1.631, -1.632, -1.633
0.4, -1.48,  -1.50, -1.85, -2.05, -2.10, -2.101, -2.102, -2.103, -2.104, -2.105, -2.106
0.5, -1.49,  -1.51, -1.86, -2.06, -2.11, …
0.6, -1.50,  -1.52, -1.87, -2.07, -2.12, …
0.7, -1.51,  -1.53, -1.88, -2.08, -2.13, …
0.8, -2.18,  -2.20, -2.70, -2.80, -2.90, …
```

- **Velocity axis** (columns): same 0 → 13.89 m/s.
- **Brake axis** (rows): 0 (no pedal) → 0.8 (80 % pedal) in 9 steps.
- **Cell** = the steady-state longitudinal acceleration (always `≤ 0`) when the brake is at
  the given pressure and the vehicle is at the given speed.

Physical interpretation:

- Row `brake = 0` is identical to row `throttle = 0` of `accel_map.csv` — same drivetrain
  drag.
- Rows show progressively stronger deceleration with brake pedal pressure. Most rows
  saturate quickly with velocity (the row-wise differences become tiny — see how the
  `0.3, ... -1.631, -1.632, -1.633` triple changes by only 0.001 m/s² between 11.11 and
  13.89 m/s).

This map is consumed by:

- **`BrakeMap::getBrake(acc, vel)`** — analogous to `AccelMap::getThrottle`; reverses the
  interpolated acc vector before `lerp` because brake → acc is monotonically *decreasing*
  in pedal value.
- **`SimModelActuationCmd::calcLongitudinalModel`** — when `convert_brake_cmd=true` and the
  brake input is positive, looked up via `brake_map_.getControlCommand(brake, vel)`.

---

## 4. [`steer_map.csv`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/data/actuation_cmd_map/steer_map.csv)

```
default,-0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6
-12, -0.290, -0.323, -0.347, -0.373, -0.429, -0.470, -0.478, -0.490, -0.510, -0.520, -0.530, -0.540, -0.550
-10, -0.200, -0.245, -0.267, -0.309, -0.358, -0.387, -0.404, -0.420, -0.430, -0.440, -0.450, -0.460, -0.470
 -8, -0.100, -0.159, -0.202, -0.232, -0.278, -0.303, -0.319, -0.320, -0.330, -0.340, -0.350, -0.360, -0.370
 -6, -0.040, -0.081, -0.121, -0.156, -0.189, -0.211, -0.232, -0.240, -0.250, -0.260, -0.270, -0.280, -0.290
 -4,  0.010, -0.014, -0.040, -0.068, -0.097, -0.119, -0.136, -0.140, -0.153, -0.169, -0.180, -0.200, -0.220
 -2,  0.060,  0.050,  0.030,  0.010, -0.006, -0.035, -0.053, -0.061, -0.076, -0.092, -0.100, -0.120, -0.140
  0,  0.110,  0.090,  0.070,  0.050,  0.030,  0.010, -0.000, -0.010, -0.030, -0.050, -0.070, -0.090, -0.110
  2,  0.150,  0.130,  0.110,  0.090,  0.078,  0.060,  0.044,  0.023,  0.004, -0.014, -0.032, -0.051, -0.070
  4,  0.240,  0.220,  0.200,  0.184,  0.165,  0.146,  0.127,  0.102,  0.086,  0.070,  0.045,  0.013, -0.019
  6,  0.330,  0.310,  0.290,  0.275,  0.245,  0.234,  0.213,  0.177,  0.158,  0.144,  0.114,  0.083,  0.052
  8,  0.430,  0.410,  0.390,  0.371,  0.330,  0.316,  0.300,  0.250,  0.234,  0.215,  0.182,  0.156,  0.131
 10,  0.500,  0.490,  0.480,  0.463,  0.421,  0.398,  0.387,  0.325,  0.312,  0.289,  0.247,  0.212,  0.177
 12,  0.560,  0.540,  0.530,  0.515,  0.505,  0.490,  0.469,  0.404,  0.381,  0.350,  0.325,  0.278,  0.232
```

- **Steering-tire axis** (columns): -0.6 → +0.6 rad in 13 steps of 0.1 rad.
- **Steer command axis** (rows): -12 → +12 (unitless steering-wheel torque or pedal
  position, vehicle-specific) in 13 steps of 2.
- **Cell** = the steady-state steer rate (in rad/s) that this combination produces.

Physical interpretation: the table captures the response of the steering column — at
zero tire angle, a unit input is more effective than at large tire angles (where the column
spring resists harder). Look at the `0` row: as you sweep from `-0.6 rad` to `+0.6 rad`
the response drops from `0.110` to `-0.110` — symmetric and roughly linear. Higher input
levels show progressively saturating response.

This map is consumed by:

- **`SimModelActuationCmdSteerMap::calcLateralModel(steer, steer_input, vel)`** — looks up
  `steer_map_.getControlCommand(steer_input, vel)` (yes, despite the name, the second axis
  is the **steer-tire** state, not velocity, in this particular CSV) and divides by
  `steer_τ` to get the steer rate.

> ⚠️ Note: the `state` axis in this CSV is actually **steer-tire angle**, not velocity. The
> `ActuationMap` class itself is generic — it labels its axes `state_index_` and
> `actuation_index_`, leaving the physical interpretation up to the caller.

---

## 5. Where these are referenced in code

| File | Reference |
|---|---|
| [`include/.../sim_model_actuation_cmd.hpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/include/autoware/simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.hpp) | Declares `AccelMap`, `BrakeMap`, `ActuationMap`. |
| [`src/.../sim_model_actuation_cmd.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/src/simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.cpp) | Constructor calls `accel_map_.readActuationMapFromCSV(accel_map_path)` and `brake_map_.readActuationMapFromCSV(brake_map_path)`. The `_STEER_MAP` derivative additionally calls `steer_map_.readActuationMapFromCSV(steer_map_path)`. |
| [`param/.../mechanical_sample.param.yaml`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_mechanical_sample.param.yaml) | `accel_map_path`, `brake_map_path` parameters — the YAML that points the model at these CSVs at launch. |
| [`test/test_simple_planning_simulator.cpp`](../src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/test/test_simple_planning_simulator.cpp) | Uses copies of these maps under `test/actuation_cmd_map/` for unit testing. |

---

## 6. Updating these maps

If you replace the CSVs:

- **Keep the layout** (first row + first column = axes labels, `[0][0] = "default"`).
- **All rows must have the same column count** (`CSVLoader::validateData` rejects ragged
  tables).
- **For `AccelMap`** (the model's forward direction), each row must be **monotonic** in the
  velocity axis if you also call `validateMap(map, true)` — the vehicle model itself does
  not call this, so the only practical requirement is that `getThrottle(acc, vel)` (the
  inverse direction, used for `actuation_status` publishing) finds a unique throttle for
  the queried `acc`.
- **For `BrakeMap`** the response should be **monotonically decreasing** with brake
  pressure (more brake ⇒ stronger deceleration), so `getBrake(acc, vel)` can do the inverse
  lookup.

In production deployments, vehicle-specific maps come from
`autoware_raw_vehicle_cmd_converter`'s `data/<vehicle>` subdirectory (see the
`csv_accel_brake_map_path` launch arg in [`launch.md`](launch.md)).
