"""One causal engine for offline replay and streaming ROS sensor events.

Events arrive in delivery order. Zero reorder-buffer policy: late observations are
rejected and counted. No future sample is used for motion prediction.
"""
import numpy as np
from .ekf import LocalizationEKF
from .models import motion, rssi_model, wrap
from .events import validate

MODES = ['wheel_only','wheel_imu','ble_only','ble_wheel','fused','imu_only']


class FusionEngine:
    def __init__(self,c,calibration,bias0=0.):
        self.c=c; self.cal=calibration
        self.filters={name:LocalizationEKF(c,calibration,name in {'fused','wheel_imu'},bias0 if name in {'fused','wheel_imu'} else 0.)
                      for name in ['wheel_only','wheel_imu','ble_wheel','fused']}
        self.time=None; self.delivery_time=-1.; self.motion_inputs={}; self.late=0; self.unknown=0; self.stale=0
        self.latest_ble={}; self.ble_position=np.array(c['start'][:2],float); self.ble_valid=False
        self.imu_pose=np.array(c['start'],float); self.imu_velocity=np.zeros(2); self.bias0=bias0
        self.accepted_ble_times=[]

    def advance(self,t):
        if self.time is None:
            self.time=t; return
        dt=t-self.time
        if dt < -1e-9:
            raise ValueError('simulation time regressed; start a new engine/run')
        if dt <= 1e-9:
            return
        timeout=self.c['filter']['motion_timeout_s']
        enc=self.motion_inputs.get('encoder'); imu=self.motion_inputs.get('imu')
        valid_enc=enc is not None and t-enc[0] <= timeout
        valid_imu=imu is not None and t-imu[0] <= timeout
        speed=enc[1]['speed_mps'] if valid_enc else 0.
        for name,f in self.filters.items():
            needs_imu=name in {'fused','wheel_imu'}
            valid=valid_enc and (valid_imu or not needs_imu)
            gyro=imu[1]['gz'] if needs_imu and valid_imu else (enc[1]['yaw_rate_radps'] if valid_enc else 0.)
            if valid:
                f.predict(dt,speed,gyro)
            else:
                # Hold state on stale controls and grow uncertainty rather than invent motion.
                f.P += np.diag([.5,.5,.1,.001])*dt
        if valid_imu:
            p=imu[1]; omega=p['gz']-self.bias0; mid=self.imu_pose[2]+omega*dt/2
            rot=np.array([[np.cos(mid),-np.sin(mid)],[np.sin(mid),np.cos(mid)]])
            a=rot@np.array([p['ax'],p['ay']])
            self.imu_pose[:2] += self.imu_velocity*dt+.5*a*dt*dt
            self.imu_velocity += a*dt; self.imu_pose[2]=wrap(self.imu_pose[2]+omega*dt)
        if not valid_enc or not valid_imu:
            self.stale += 1
        self.time=t

    def process(self,e):
        validate(e)
        if e['delivery_t'] < self.delivery_time-1e-9:
            raise ValueError('delivery time regressed')
        self.delivery_time=e['delivery_t']
        if e['delivery_t']-e['t'] > 1e-6 or (self.time is not None and e['t'] < self.time-1e-9):
            self.late += 1; return
        self.advance(e['t'])
        p=e['payload']
        if e['kind'] in {'imu','encoder'}:
            self.motion_inputs[e['kind']]=(e['t'],p)
        else:
            identity=p['beacon_id']
            if identity not in self.cal['beacons']:
                self.unknown+=1; return
            self.latest_ble[identity]=(e['t'],p['rssi_dbm'])
            self.accepted_ble_times.append(e['t'])
            for name in ['ble_wheel','fused']:
                self.filters[name].update(identity,p['rssi_dbm'],e['t'])
            self.solve_ble(e['t'])

    def solve_ble(self,t):
        beacons={b['id']:b['xyz'] for b in self.c['beacons']}
        fresh=[(identity,rssi) for identity,(bt,rssi) in self.latest_ble.items()
               if t-bt <= self.c['filter']['ble_window_s'] and identity in beacons]
        self.ble_valid=False
        if len(fresh)<3:
            return
        x=np.r_[self.ble_position,0.,0.]
        for _ in range(8):
            H=[]; residual=[]
            for identity,rssi in fresh:
                p=self.cal['beacons'][identity]
                predicted,h=rssi_model(x,beacons[identity],self.c['receiver_z_m'],p['p0_dbm'],p['n'])
                weight=np.sqrt(p['variance_dbm2'])
                H.append(h[:2]/weight); residual.append((rssi-predicted)/weight)
            H=np.array(H)
            if np.linalg.matrix_rank(H)<2 or np.linalg.cond(H)>100:
                return
            step=np.linalg.solve(H.T@H+.1*np.eye(2),H.T@residual)
            norm=np.linalg.norm(step)
            if norm>1: step/=norm
            x[:2]+=step
            if norm<1e-4: break
        if not np.isfinite(x).all(): return
        # Reject fitted positions outside all calibrated distance domains.
        for identity,_ in fresh:
            p=self.cal['beacons'][identity]; xyz=beacons[identity]
            d=np.sqrt(np.sum((x[:2]-xyz[:2])**2)+(self.c['receiver_z_m']-xyz[2])**2)
            if not p['min_distance_m'] <= d <= p['max_distance_m']: return
        self.ble_position=x[:2]; self.ble_valid=True

    def snapshot(self,t):
        states={name:f.x.copy() for name,f in self.filters.items()}
        fresh=[bt for bt,_ in self.latest_ble.values() if t-bt <= self.c['filter']['ble_window_s']]
        states['ble_only']=np.array([*self.ble_position,np.nan,np.nan]) if self.ble_valid and len(fresh)>=3 else np.full(4,np.nan)
        states['imu_only']=np.r_[self.imu_pose,self.bias0]
        return states


def replay(c,calibration,events,grid):
    # Explicit stationary calibration uses only sensor samples from the first 2s.
    # All estimators begin at t=2, while the scripted robot remains stationary to t=3.
    stationary=[e['payload']['gz'] for e in events if e['kind']=='imu' and e['t']<2 and e['delivery_t']<2]
    bias0=float(np.mean(stationary)) if stationary else 0.
    engine=FusionEngine(c,calibration,bias0)
    output={name:[] for name in MODES}; cov=[]; out_t=[]; index=0
    for t in grid:
        if t<2: continue
        while index<len(events) and events[index]['delivery_t'] <= t+1e-9:
            e=events[index]; index+=1
            if e['t']>=2: engine.process(e)
        engine.advance(float(t))
        for name,state in engine.snapshot(t).items(): output[name].append(state)
        cov.append(engine.filters['fused'].P.copy()); out_t.append(t)
    return np.array(out_t),{name:np.array(rows) for name,rows in output.items()},np.array(cov),engine
