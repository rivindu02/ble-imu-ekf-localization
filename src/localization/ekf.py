"""Wheel/gyro EKF with gyro bias and direct scalar RSSI updates."""
import numpy as np
from .models import motion, rssi_model, wrap


class LocalizationEKF:
    def __init__(self, config, calibration, use_imu=True, bias0=0.):
        self.c, self.calibration, self.use_imu = config, calibration, use_imu
        self.beacons = {b['id']: b['xyz'] for b in config['beacons']}
        self.x = np.array([*config['start'], bias0], float)
        self.P = np.diag([.25**2, .25**2, .08**2, .008**2 if use_imu else 1e-12])
        self.diagnostics = []

    def predict(self, dt, speed, gyro):
        if not self.use_imu:
            self.x[3] = 0
        f = self.c['filter']
        self.x, F, G = motion(self.x, speed, gyro, dt)
        # Input sample-average variance = continuous white-noise intensity / dt.
        W = np.diag([f['speed_sigma_mps_sqrt_s']**2,
                     f['gyro_sigma_rad_sqrt_s']**2])/dt
        Q = G@W@G.T
        Q[3, 3] += f['bias_walk_radps_sqrt_s']**2*dt if self.use_imu else 0
        self.P = F@self.P@F.T + Q
        self.P = (self.P+self.P.T)/2

    def update(self, identity, measured, t=0.):
        diag = {'t':float(t), 'beacon_id':identity, 'accepted':False}
        if identity not in self.beacons or identity not in self.calibration['beacons'] or not np.isfinite(measured):
            diag['reason'] = 'unknown_or_invalid'; self.diagnostics.append(diag); return False
        cal = self.calibration['beacons'][identity]
        xyz = self.beacons[identity]
        distance = np.sqrt(np.sum((self.x[:2]-xyz[:2])**2)+(self.c['receiver_z_m']-xyz[2])**2)
        if not cal['min_distance_m'] <= distance <= cal['max_distance_m']:
            diag['reason'] = 'model_domain'; self.diagnostics.append(diag); return False
        predicted, H = rssi_model(self.x, xyz, self.c['receiver_z_m'], cal['p0_dbm'], cal['n'])
        R = cal['variance_dbm2']
        innovation = measured-predicted
        S = float(H@self.P@H+R)
        nis = float(innovation**2/S)
        diag.update(innovation_dbm=float(innovation), nis=nis)
        if nis > self.c['filter']['nis_gate']:
            diag['reason'] = 'nis_gate'; self.diagnostics.append(diag); return False
        K = self.P@H/S
        self.x += K*innovation; self.x[2] = wrap(self.x[2])
        A = np.eye(4)-np.outer(K,H)
        self.P = A@self.P@A.T + np.outer(K,K)*R
        self.P = (self.P+self.P.T)/2
        diag.update(accepted=True, reason='accepted'); self.diagnostics.append(diag)
        return True
