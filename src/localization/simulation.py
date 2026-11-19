"""Independent kinematics and deliberately simplified radio/IMU simulation."""
import copy
import numpy as np
from .events import event
from .models import wrap

SCENARIOS = ['nominal', 'noisy_rssi', 'nlos', 'beacon_outage', 'weak_geometry',
             'gyro_bias', 'wheel_slip', 'miscalibration', 'delayed_packets', 'overconfident']
ROUTES = ['stationary', 'straight', 'rectangle', 'figure_eight']


def scenario_config(base, scenario):
    c = copy.deepcopy(base)
    if scenario not in SCENARIOS:
        raise ValueError('unknown scenario')
    if scenario == 'weak_geometry':
        for i,b in enumerate(c['beacons']):
            b['xyz'] = [-3.+i*3, -3., 1.2]
    if scenario == 'overconfident':
        for key in ['speed_sigma_mps_sqrt_s', 'gyro_sigma_rad_sqrt_s', 'bias_walk_radps_sqrt_s']:
            c['filter'][key] *= .2
    return c


def controls(t, route):
    if t < 3 or route == 'stationary':
        return 0., 0.
    t -= 3
    if route == 'straight':
        phase = t % 32
        return (.35, 0.) if phase < 12 else ((0., np.pi/4) if phase < 16 else ((.35,0.) if phase < 28 else (0.,np.pi/4)))
    if route == 'rectangle':
        phase = t % 16
        return (.4, 0.) if phase < 12 else (0., np.pi/8)
    if route == 'figure_eight':
        return .35, .22*np.sin(t*.09)
    raise ValueError('unknown route')


def generate(base, seed=7, scenario='nominal', route='rectangle'):
    c = scenario_config(base, scenario)
    streams = [np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(6)]
    enc_rng, imu_rng, radio_rng, loss_rng, delay_rng, shadow_rng = streams
    dt = c['dt_s']; steps = int(c['duration_s']/dt)
    pose = np.array(c['start'],float)
    bias = c['imu']['gyro_bias_radps']*(4 if scenario == 'gyro_bias' else 1)
    shadow = np.zeros(len(c['beacons']))
    next_packet = np.arange(len(c['beacons']))*c['radio']['period_s']/len(c['beacons'])
    truth = []; events = []; seq = 0; previous_speed = 0.
    for k in range(steps+1):
        t = k*dt
        speed, yaw_rate = controls(t,route)
        if k:
            # Controls apply on the preceding interval and correspond to endpoint sample.
            yaw_mid = pose[2]+yaw_rate*dt/2
            pose[:2] += speed*dt*np.array([np.cos(yaw_mid),np.sin(yaw_mid)])
            pose[2] = wrap(pose[2]+yaw_rate*dt)
        bias += imu_rng.normal(0,c['imu']['bias_walk_radps_sqrt_s']*np.sqrt(dt))
        truth.append([t,*pose,bias])
        speed_noise, yaw_noise = enc_rng.standard_normal(2)
        measured_speed = speed*c['encoder']['speed_scale']+speed_noise*c['encoder']['speed_sigma_mps']
        measured_yaw = yaw_rate+c['encoder']['yaw_bias_radps']+yaw_noise*c['encoder']['yaw_sigma_radps']
        if scenario == 'wheel_slip' and 25 <= t <= 40:
            measured_speed *= 1.7
            measured_yaw += .04
        imu_noise = imu_rng.standard_normal(7)
        gyro = yaw_rate+bias+imu_noise[0]*c['imu']['gyro_sigma_radps']
        accel = [(speed-previous_speed)/dt, speed*yaw_rate, 9.81]
        accel = np.array(accel)+imu_noise[1:4]*c['imu']['accel_sigma_mps2']
        previous_speed = speed
        for kind,payload in [('encoder',{'speed_mps':float(measured_speed),'yaw_rate_radps':float(measured_yaw)}),
                             ('imu',dict(zip(['ax','ay','az','gx','gy','gz'],[float(v) for v in [*accel,imu_noise[4]*.005,imu_noise[5]*.005,gyro]])))]:
            events.append(event(t,seq,kind,payload)); seq += 1
        for i,b in enumerate(c['beacons']):
            # Draw each stream on every tick to preserve paired random disturbances.
            normal = radio_rng.normal(); missing = loss_rng.random(); delay_draw = delay_rng.random()
            shadow[i] = .97*shadow[i]+shadow_rng.normal(0,.3)
            if t+1e-9 < next_packet[i]:
                continue
            next_packet[i] += c['radio']['period_s']
            if missing < c['radio']['loss_probability'] or (scenario == 'beacon_outage' and 25 <= t <= 45):
                continue
            distance = np.sqrt(np.sum((pose[:2]-b['xyz'][:2])**2)+(c['receiver_z_m']-b['xyz'][2])**2)
            if not c['radio']['min_distance_m'] <= distance <= c['radio']['max_distance_m']:
                continue
            sigma = c['radio']['sigma_dbm']*(2.5 if scenario == 'noisy_rssi' else 1)
            rssi = b['p0_dbm']-10*b['n']*np.log10(distance)+normal*sigma
            if scenario == 'nlos':
                rssi += shadow[i]-(9. if i < 2 and 20 <= t <= 50 else 0.)
            if scenario == 'miscalibration':
                rssi -= 4.
            delay = .6 if scenario == 'delayed_packets' and delay_draw < .3 else 0.
            events.append(event(t,seq,'ble',{'beacon_id':b['id'],'rssi_dbm':float(rssi)},delay)); seq += 1
    events.sort(key=lambda e:(e['delivery_t'], e['seq']))
    return c,events,np.asarray(truth)
