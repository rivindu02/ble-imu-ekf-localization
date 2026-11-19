"""Meters, radians, seconds, dBm. Noise intensities are per sqrt(second)."""
import numpy as np


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion(x, speed, gyro, dt):
    if not np.isfinite([*x, speed, gyro, dt]).all() or dt <= 0:
        raise ValueError('motion requires finite state/input and positive dt')
    yaw_mid = x[2] + (gyro - x[3]) * dt / 2
    c, s = np.cos(yaw_mid), np.sin(yaw_mid)
    out = np.asarray(x, float).copy()
    out += [speed * dt * c, speed * dt * s, (gyro-x[3])*dt, 0]
    out[2] = wrap(out[2])
    F = np.eye(4)
    F[0, 2], F[1, 2] = -speed*dt*s, speed*dt*c
    F[0, 3], F[1, 3], F[2, 3] = speed*dt*dt*s/2, -speed*dt*dt*c/2, -dt
    # Input derivatives wrt sample-average speed and gyro.
    G = np.array([[dt*c, -speed*dt*dt*s/2],
                  [dt*s, speed*dt*dt*c/2], [0, dt], [0, 0]])
    return out, F, G


def rssi_model(x, xyz, receiver_z, p0, n):
    delta = np.asarray(x[:2])-np.asarray(xyz[:2])
    d2 = float(delta@delta + (receiver_z-xyz[2])**2)
    if d2 < 1e-8 or n <= 0:
        raise ValueError('undefined radio geometry or path-loss exponent')
    H = np.zeros(4)
    H[:2] = -10*n/np.log(10)*delta/d2
    return float(p0-10*n*np.log10(np.sqrt(d2))), H
