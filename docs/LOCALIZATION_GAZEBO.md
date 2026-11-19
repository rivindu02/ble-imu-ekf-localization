# Gazebo localization demonstration

Use the existing Ubuntu Distrobox `devbox` with ROS 2 Humble and Gazebo Classic. An existing compatible Ubuntu environment is sufficient; no container is created by the project.

```bash
distrobox enter devbox
cd /path/to/ble-imu-ekf-localization
bash gazebo/run_localization.sh headless 80
bash gazebo/run_localization.sh gui 80 beacon_outage
```

The demo requires `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`, `libgazebo_ros_state.so`, `libgazebo_ros_diff_drive.so`, and **`libgazebo_ros_imu_sensor.so`**. It requires NumPy and ROS Python messages, not Matplotlib. A missing native IMU makes validation fail; there is no silent synthetic-IMU fallback.

The world is self-contained and has no model downloads. `python3 scripts/build_localization_world.py` rebuilds it after beacon/IMU configuration changes. Sensor axes align with `base_link`. Native IMU messages include simulated gravity and configured white noise; the bridge adds a declared gyro offset. The estimator uses yaw rate only. Beacon RSSI uses a known fixed receiver height approximation; it is a simplified radio model.

The blue robot follows a scripted circle centered on `(0,4)` after a stationary calibration interval. This controller uses truth to create a reproducible route, independently of the fusion result. This is localization evaluation, not an estimated-pose navigation controller.

## Nodes and topics

| Node | Input | Output |
|---|---|---|
| sensor/controller | model states, native IMU, encoder odometry | timestamped sensor events and scripted velocity command |
| fusion | sensor events only | state, covariance, odometry, paths, diagnostics/markers |
| truth evaluator | model states | truth trajectory/path |

Topics under `/localization_demo/`: `model_states`, `imu_native`, `wheel_odom`, `sensor_events`, `state`, `fused_odom`, `fused_path`, `wheel_only_path`, `truth_path`, `markers`, `cmd_vel`. BLE observations travel as versioned JSON in `std_msgs/String` sensor events; RSSI is never mislabeled as a `sensor_msgs/Range` measurement. This keeps the demo runnable without a ROS workspace/custom-message build.

RViz Fixed Frame is `map`; paths/markers are expressed directly in this frame. No TF transform is required for these displays. The demonstration does not publish a complete TF tree for external navigation software.

## Outputs and checks

Each launch creates `results/localization/gazebo_<timestamp>/`. Override `LOCALIZATION_OUTPUT_DIR` to choose a folder. It saves environment inventory, server/client logs, raw sensor events, truth CSV, estimates/covariance NPZ, calibration, innovations, manifest and metrics. `scripts/validate_localization_run.py RUN_DIR` checks that the run completed with actual native IMU, encoder messages and BLE updates, valid timestamps and PSD covariance.

Duration is simulation seconds from the first observation, allowing ROS startup to finish before stationary calibration. Progress prints every five wall-clock seconds; headless mode opens no window. A wall-time watchdog stops stalled/paused simulation after 240 seconds. Interruptions save partial results where possible. A complete manifest is emitted only after duration and data checks pass. Raw callback order can differ from acquisition order: out-of-order events are dropped and counted. No network transport delay is modeled; receipt timestamps remain in raw logs for inspection. Replaying those Gazebo raw logs requires reproducing callback ordering and timing; the independent replay verification command is intended for independent-simulator recordings.

Default ROS domain is 75 and Gazebo master is `http://127.0.0.1:11356`. The launcher checks that the master port is free. It cleans up only its own child processes. If the native sensor is missing, inspect `server.log` and environment inventory before changing packages.

GUI mode uses `QT_QPA_PLATFORM=xcb`, for Gazebo Classic compatibility with Wayland desktops. Headless mode should be validated first. Missing display access, ROS packages or native plugins must be reported as failed validation, not a passed demo.

## References

- [Official Gazebo ROS IMU example](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/worlds/gazebo_ros_imu_sensor_demo.world)
- [Gazebo ROS IMU source](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_imu_sensor.cpp)

The world follows the ROS 2 plugin convention: `~/out` remapping, `frame_name`, and `initial_orientation_as_reference=false`.
