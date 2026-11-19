# Experiments and reproducibility

## Benchmark

The included benchmark has 260 completed trials with seeds 7–26. Ten scenarios use the rectangular route; stationary, straight, and figure-eight routes add three nominal conditions. Each condition has 20 paired seeds. Independent streams generate encoder, IMU, radio-noise, packet-loss, latency, and shadowing disturbances.

Calibration uses a separate generated stationary dataset (seed 90210). Per-beacon path-loss parameters are fitted on training samples; validation residuals set observation variance. The initial seed-7 smoke run was inspected before the benchmark, with no parameters tuned from final batch results.

```bash
python scripts/localization.py batch --trials 20 --all-routes
python scripts/validate_experiments.py
```

`summary.json` stores means and sample standard deviations. `metrics.csv` stores each method's per-trial RMSE, median, p90 and availability. `trials.json` includes additional diagnostics. Representative seed-7 recordings preserve truth, all six estimates, covariance, manifests and innovations. Only the nominal seed-7 event stream is tracked; regenerate other raw event logs using the matching scenario, route and seed.

Source hashes identify the implementation used when each recording was collected. Later documentation or tooling changes do not alter the saved numerical evidence.

## Comparison methods

| Method | Inputs |
|---|---|
| `wheel_only` | Encoder speed and encoder yaw rate |
| `wheel_imu` | Encoder speed and gyro yaw rate with initial stationary bias calibration |
| `ble_only` | Fresh RSSI windows and calibrated beacon map |
| `ble_wheel` | Encoder prediction and direct RSSI corrections |
| `fused` | Encoder speed, bias-state gyro prediction and direct RSSI corrections |
| `imu_only` | Planar acceleration double integration and calibrated gyro heading; drift diagnostic |

All methods use the same recorded events. Errors are computed in the configured map frame without trajectory alignment. Missing BLE fixes remain missing; availability and common-grid RMSE are reported separately. Position NEES uses the 2D position covariance, while NIS is saved before innovation gating.

## Sensor stresses

- `noisy_rssi`: increased radio noise.
- `nlos`: time-window attenuation and correlated shadowing, not electromagnetic ray tracing.
- `beacon_outage`: no radio observations from t=25–45 s.
- `weak_geometry`: beacons placed along one side of the scene.
- `gyro_bias`: increased initial bias, handled by stationary calibration.
- `wheel_slip`: encoder/ground-motion mismatch over a defined interval.
- `miscalibration`: systematic RSSI offset after calibration.
- `delayed_packets`: delayed observations, dropped and counted by the causal engine.
- `overconfident`: underestimated process/measurement uncertainty.

NLOS and systematic calibration offsets produce significant failures. These remain visible in the charts and metrics. Synthetic results do not establish physical BLE accuracy.

## Gazebo evidence

A separate physics run used Ubuntu 22.04.5, Python 3.10.12, NumPy 2.2.6, ROS 2 Humble and Gazebo Classic 11.10.2. The native IMU plugin and encoder-mode drive generated sensor messages, while a separate radio generator synthesized RSSI. The saved `gazebo_outage` fixture contains the completed 80.08-second run.

```bash
python scripts/validate_localization_run.py results/localization/gazebo_outage
```

The run recorded 4,003 native IMU messages, 800 encoder messages and 693 saved estimates. The longest radio gap was 20.10 s. Fusion RMSE was 0.657 m, wheel-only 2.974 m, and wheel + IMU dead reckoning 0.416 m. This is a single outage trial, not a paired physics benchmark. The user's GUI run and application logs confirmed Gazebo/RViz startup; README animations are reconstructed from recordings.

Ground truth is used only for sensor generation, evaluation and a separate scripted route controller. Fusion receives known beacon coordinates and a configured initial pose, but no ongoing ground-truth corrections.

## Media

```bash
python scripts/render_media.py
```

Figures and GIFs use saved data. `--skip-animation` rebuilds only static figures. NumPy, Matplotlib and Pillow are sufficient; document-generation packages are not required.
