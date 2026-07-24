# LEM Simulator

Formula Student Driverless vehicle, sensor and perception simulator for ROS 1.
LEM Simulator is designed for software-in-the-loop development of an entire
autonomous pipeline:

```text
cone perception → SLAM → path planning → control → vehicle dynamics
```

It can also bypass path planning and publish a known centerline. This makes the
same package useful both for full-stack integration and focused controller
development.

> **Portfolio image slot:** add an RViz overview as
> `docs/images/simulator-overview.png`.

## Why this project exists

Driverless software needs repeatable tests before it reaches a real car. LEM
Simulator provides a deterministic vehicle environment while preserving the
timing, noise, bias and latency that make real embedded systems difficult.

The simulator includes:

- a nonlinear four-wheel double-track vehicle model;
- tyre combined slip, suspension load transfer, drivetrain and aerodynamics;
- independent timers and caches for the DV board, main board and sensors;
- a filtered IMU with separate accelerometer and gyroscope acquisition;
- a Gaussian state estimator published on the stack-compatible `/ins/pose`
  topic;
- camera, lidar and fused cone-perception modes;
- optional cone dropout and false-positive detections;
- baseline torque allocation and torque vectoring;
- self-contained centerline tracking and vehicle-stability metrics;
- four curated Formula Student event maps.

The runtime refactor only reorganizes orchestration. The original vehicle
physics equations remain in their dedicated model files.

## Architecture

```text
                              ROS Driverless stack
          ┌─────────────────────────────────────────────────┐
 cones ──►│ SLAM ──► path planning ──► control              │
 state ──►│                       ▲                         │
 IMU   ──►│                       │ wheel/steering feedback │
          └───────────────────────┼─────────────────────────┘
                                  │ /dv_board/control
                                  ▼
┌────────────────────────── LEM Simulator ──────────────────────────┐
│ sensor timers       camera / lidar / IMU / state estimator       │
│ sampled caches      DV board / main board / steering encoder     │
│ delayed queues      perception and main-board computation        │
│                                                                  │
│ control cache → baseline allocation → torque vectoring           │
│                                      │                           │
│                                      ▼                           │
│             four-wheel dynamics [T_FL, T_FR, T_RL, T_RR, steer] │
│                                      │                           │
│                       state, TF, markers and ride metrics         │
└──────────────────────────────────────────────────────────────────┘
```

The physics loop uses a configurable step, while each simulated device owns a
`PeriodicTimer`. `ValueCache` models sampled data and `DelayedQueue` models
transport or computation latency. These primitives are isolated in
[`runtime_helpers.hpp`](include/runtime_helpers.hpp), leaving the high-level
sequence readable in [`sim_loop.cpp`](sim_loop.cpp).

## Models

| Subsystem | Modelled behaviour |
|---|---|
| Vehicle body | planar motion, yaw and four-wheel double-track geometry |
| Tyres | longitudinal/lateral combined slip, load sensitivity and force relaxation |
| Suspension | static, geometric and elastic longitudinal/lateral load transfer |
| Drivetrain | motor lag, torque limits, total power limits and fixed gear ratio |
| Steering | rack dynamics, angle/rate limits, bias and actuator/encoder noise |
| Aerodynamics | velocity-dependent drag and front/rear downforce |
| IMU | independent clocks, analog bandwidth, white noise, bias random walk and output averaging |
| Gaussian state estimator | delayed noisy pose, velocity, yaw and yaw-rate with bias random walks |
| Perception | camera, lidar or fusion with FOV, range, latency, noise, dropout and false positives |
| Control interface | sampled DV-board input, main-board delay, torque allocation and vectoring |

Core vehicle equations are implemented in:

- [`double_track.cpp`](src_helpers/double_track.cpp)
- [`tire_model.cpp`](src_helpers/tire_model.cpp)
- [`suspension.cpp`](src_helpers/suspension.cpp)
- [`steering_system.cpp`](src_helpers/steering_system.cpp)
- [`torque_allocation.cpp`](src_helpers/torque_allocation.cpp)

## Requirements

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- Eigen 3
- nlohmann/json

The simulator bundles the seven `dv_interfaces` messages it actually uses
under [`interfaces/dv_interfaces/`](interfaces/dv_interfaces). If the workspace
already contains a `dv_interfaces` package, CMake uses that canonical package.
Otherwise the minimal compatible set is generated during the simulator build.
Controller-specific debug messages and unrelated interfaces are deliberately
excluded.

Full-pipeline launches additionally require:

- `dv_slam`
- `dv_path_planning`
- `dv_control`

Install dependencies and build:

```bash
sudo apt update
sudo apt install python3-catkin-tools libeigen3-dev nlohmann-json3-dev

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build lem_simulator
source devel/setup.bash
```

## Quick start

Run FSG 2019 with the supplied centerline, bypassing path planning:

```bash
roslaunch lem_simulator fsg_2019_no_pp.launch sim_time:=30
```

Run the complete perception → SLAM → path planning → control pipeline:

```bash
roslaunch lem_simulator fsg_2019.launch
```

Useful RViz data:

- `/viz/cones_gt`
- `/viz/cones_lidar` or `/viz/cones_vis`
- `/viz/bolide_model`
- `/simulation/gg_sphere`
- TF frames `map`, `bolide_true` and `bolide_CoG`

> **Portfolio image slot:** add a perception close-up as
> `docs/images/perception-closeup.png`.

## Events and launch files

Every event has two entry points:

- `<event>.launch` runs simulator, SLAM, path planning and control;
- `<event>_no_pp.launch` replaces SLAM/path planning with the included
  centerline publisher and still runs control.

| Event | Full pipeline | No path planning | Initial pose `(x_m, y_m, yaw_rad)` |
|---|---|---|---|
| Acceleration 150 m | `acc.launch` | `acc_no_pp.launch` | `(0, 0, 0)` |
| Skidpad | `skidpad.launch` | `skidpad_no_pp.launch` | `(0, -15, 1.570796)` |
| FS Czech 2025 | `fs_czech.launch` | `fs_czech_no_pp.launch` | `(1.802392, 23.440755, 0.540726)` |
| FSG 2019 | `fsg_2019.launch` | `fsg_2019_no_pp.launch` | `(0, 0, 0)` |

Each event has a `*_cones.csv` and `*_centerline.csv` file in
[`tracks/`](tracks). FSG 2019 was digitized from the supplied cone-layout
image. It is suitable for software-in-the-loop development, but it is not a
survey-grade reconstruction for official lap-time comparison.

## Runtime configuration

All model and simulated-device settings are loaded from
[`params_default_lem.json`](config/params_default_lem.json) when the node
starts. Restart the node after changing the file.

All values use SI units except exactly these three user-facing steering
parameters, which are intentionally entered in degrees and converted to
radians once by `ParamBank`:

- `steering_system.steering_bias_deg`
- `steering_system.steering_noise_actuator_std_deg`
- `steering_system.steering_noise_encoder_std_deg`

Important parameters:

| JSON path | Unit | Purpose |
|---|---:|---|
| `simulation.time_step` | s | physics integration and ROS loop period |
| `vehicle.m` | kg | total vehicle mass |
| `vehicle.w` | m | wheelbase |
| `vehicle.aero_package_enabled` | bool | enable drag and downforce |
| `drivetrain.P_max_drive` | W | total drive power limit |
| `drivetrain.P_min_recup` | W | regenerative power limit |
| `main.main_loop_time_step` | s | main-board computation cadence |
| `main.main_computation_delay_s` | s | delayed main-board output |
| `gaussian_state_estimator.frequency_hz` | Hz | state-estimate acquisition cadence |
| `gaussian_state_estimator.yaw_noise_std_rad` | rad | yaw white-noise standard deviation |
| `gaussian_state_estimator.*_bias_rw_*` | SI/√s | state-estimator bias random walks |
| `imu.*_rate_hz` | Hz | accelerometer and gyroscope acquisition clocks |
| `imu.*_bandwidth_hz` | Hz | first-order IMU analog bandwidth |
| `imu.gyroscope_noise_std_rad_per_s` | rad/s | gyroscope white noise |
| `camera.horizontal_fov_rad` | rad | camera horizontal field of view |
| `lidar.azimuth_window_rad` | rad | lidar azimuth region of interest |
| `perception_errors.cone_dropout_probability` | 0–1 | probability of dropping a true detection |
| `perception_errors.false_positive_mean_count` | cones/frame | Poisson mean for synthetic false positives |
| `metrics.sideslip_threshold_rad` | rad | sideslip threshold used by ride metrics |
| `metrics.minimum_speed_mps_for_sideslip` | m/s | ignore undefined low-speed sideslip |
| `torque_allocation_and_vectoring.front_fraction_*` | 0–1 | baseline front/rear torque split |
| `torque_allocation_and_vectoring.max_motor_delta_nm` | N·m | vectoring clamp per motor side |

Dropout and false positives are independently enabled with
`perception_errors.dropout_enabled` and
`perception_errors.false_positives_enabled`. Their defaults are `false`.

## ROS interface

Input:

| Topic | Type | Meaning |
|---|---|---|
| `/dv_board/control` | `dv_interfaces/Control` | steering and wheel-torque request |

Outputs:

| Topic | Type | Meaning |
|---|---|---|
| `/ins/pose` | `nav_msgs/Odometry` | Gaussian state estimate; topic name retained for stack compatibility |
| `/dv_board/imu` | `dv_interfaces/Imu` | filtered IMU measurement |
| `/dv_board/data` | `dv_interfaces/DV_board` | sampled wheel-speed data |
| `/servo_node/cubemars/encoder_absolute` | `std_msgs/Float64` | noisy steering encoder in rad |
| `/dv_cone_detector/cones` | `dv_interfaces/Cones` | active camera/lidar/fusion perception output |
| `/debug/full_log_info` | `dv_interfaces/full_state` | complete vehicle debug state in SI |

## Metrics

The simulator no longer consumes controller debug messages. It evaluates the
true vehicle pose against the selected event centerline after every physics
step and writes a `*_metrics.csv` file on shutdown.

Reported values include:

- mean absolute lateral error in m;
- mean absolute heading error in rad;
- mean speed projected onto the centerline in m/s;
- top-10 lateral, heading, sideslip and driven-wheel slip values;
- time and percentage above the configured body-sideslip threshold.

## Repository layout

```text
lem_simulator/
├── config/                    runtime JSON configuration
├── docs/images/               portfolio image slots
├── include/                   model and runtime interfaces
├── interfaces/dv_interfaces/  minimal bundled ROS messages
├── launch/                    full-pipeline and no-PP launchers
├── logs/                      generated ride metrics
├── src_helpers/               dynamics, sensor and helper implementations
├── tracks/                    four curated cone maps and centerlines
├── main.cpp                   ROS process and configured physics clock
└── sim_loop.cpp               high-level simulation orchestration
```

## Adding a track

1. Add `tracks/my_event_cones.csv` with columns `x,y,color`.
2. Add `tracks/my_event_centerline.csv` with columns `x,y`.
3. Copy one full and one `_no_pp` launch file.
4. Set `initial_x_m`, `initial_y_m` and `initial_yaw_rad`.
5. Pass the same centerline to `_simulator.launch` for internal metrics.
6. Validate the initial pose before enabling control.

## Validation

```bash
python3 -m json.tool config/params_default_lem.json >/dev/null
for file in launch/*.launch; do xmllint --noout "$file"; done
catkin build lem_simulator --no-deps
```

> **Results image slot:** add plots of lateral error, sideslip and torque
> allocation as `docs/images/results.png`.

## License

MIT. See [`LICENSE`](LICENSE).
