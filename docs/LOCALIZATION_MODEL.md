# Estimator derivation and measurement assumptions

The estimator localizes within a surveyed beacon map; it does not discover landmarks. All BLE observations are scalar RSSI values in dBm.

## State and motion

Let state be `s=[x,y,theta,b]`, encoder speed `v`, gyroscope measurement `g`, and interval `dt`. Define `a=theta+(g-b)dt/2`, `c=cos(a)`, `q=sin(a)`.

```text
f(s,u) = [x+v*dt*c, y+v*dt*q, wrap(theta+(g-b)*dt), b]

F = [[1,0,-v*dt*q,  v*dt²*q/2],
     [0,1, v*dt*c, -v*dt²*c/2],
     [0,0,       1,          -dt],
     [0,0,       0,            1]]

G = [[dt*c, -v*dt²*q/2],
     [dt*q,  v*dt²*c/2],
     [   0,           dt],
     [   0,            0]]
```

For continuous white input-noise intensities `q_v` and `q_g`, the sample-average input covariance is `W=diag(q_v,q_g)/dt`. Propagate `P=F P Fᵀ + G W Gᵀ + diag(0,0,0,q_b*dt)`. This midpoint discretization is suitable for small sample intervals; it is not an exact continuous-time covariance integral. State uncertainty is symmetrized after each operation. Stationary gyro readings provide an initial bias estimate, whose uncertainty remains nonzero.

Wheel-only and BLE + wheel modes use encoder angular rate and fix bias to zero. Wheel + IMU and full fusion use gyro yaw rate with the bias state. These are explicitly wheel-assisted estimators. The accelerometer is logged and used only by the separate drift diagnostic, avoiding unmodeled double counting.

## Direct RSSI observation

For beacon `(bx,by,bz)` and fixed receiver height `zr`, with reference distance 1 m:

```text
d² = (x-bx)² + (y-by)² + (zr-bz)²
h(s) = P0 - 10*n*log10(d)
H = -(10*n/ln(10))/d² * [x-bx, y-by, 0, 0]
```

Fit `P0` and `n` by linear least squares on stationary RSSI versus `log10(distance)`. Separate held-out samples estimate residual variance `R` in dBm². Only these fitted quantities and surveyed beacon coordinates are passed into the estimator. The simulator's true radio parameters remain outside the estimator configuration.

For innovation `r=z-h(s)`, use `S=H P Hᵀ+R`, `NIS=r²/S`, `K=P Hᵀ/S`. Reject a sample when NIS exceeds the configured threshold or the model is outside its calibrated distance domain. Otherwise:

```text
s = s + K*r
A = I-K*H
P = A*P*Aᵀ + K*R*Kᵀ
```

Wrap heading after correction. Gating is a model-based diagnostic and can reject useful data after a large modeling error; the implementation records rejected samples rather than claiming it guarantees robustness.

## Baselines and clocks

BLE-only uses fresh per-beacon windows, weighted nonlinear residual fitting and geometry checks. It does not use wheel/gyro inputs. The solver uses a damped Gauss-Newton iteration from the last fix; results may be local or unavailable. Availability is always reported.

The IMU-only diagnostic rotates body-frame horizontal acceleration using integrated, bias-corrected gyro heading and double-integrates it. It starts stationary, assumes planar alignment, and receives no truth-derived velocity reset. It will drift. It is not the motion model of the primary robot filter.

Independent replay applies the latest available motion sample up to each event time. Equal-time events are ordered by recorded sequence. Delayed packets are rejected under the zero-buffer experiment policy; missing controls expire, with state held and covariance inflated. Replay never uses a future IMU/encoder sample for prediction. Simulation time resets require a new engine.

Gazebo uses sensor-header acquisition timestamps and `/clock`. Network transport latency is not modeled; actual receipt time is logged separately. Out-of-order sensor acquisition events are counted and dropped. This differs from the explicit delayed-radio experiment, whose delivery timestamps simulate latency. Native Gazebo sensor/physics timing can produce different results from the independent kinematic simulator.

## Truth and uncertainty

Only the generator, scripted controller, and evaluator access simulator truth. Fusion receives known map coordinates, configured initial pose/covariance, fitted calibration and sensor events. The starting pose is a declared initialization assumption rather than an estimated global fix.

Position NEES uses only the 2D position covariance and error. The 95% ellipse threshold is 5.991 for two degrees of freedom. Mean NIS includes rejected innovations. Neither a plausible NIS mean nor a smooth path alone proves consistency. Correlated shadowing, systematic offsets and geometry can make Gaussian EKF covariance overconfident.
