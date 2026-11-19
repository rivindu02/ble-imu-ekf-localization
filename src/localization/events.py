"""Versioned JSONL events; seconds in one run's simulation clock."""
import hashlib
import json
from pathlib import Path
import numpy as np


def load_config(path):
    c = json.loads(Path(path).read_text())
    ids = [b['id'] for b in c['beacons']]
    if c['schema_version'] != 1 or len(ids) < 3 or len(set(ids)) != len(ids):
        raise ValueError('need unique beacon IDs and schema version 1')
    if c['dt_s'] <= 0 or c['duration_s'] <= 0 or c['track_m'] <= 0:
        raise ValueError('invalid timing or robot geometry')
    if len(c['start']) != 3 or not np.isfinite(c['start']).all():
        raise ValueError('invalid start pose')
    for b in c['beacons']:
        if len(b['xyz']) != 3 or not np.isfinite(b['xyz']+[b['p0_dbm'], b['n']]).all() or b['n'] <= 0:
            raise ValueError('invalid beacon geometry/model')
    r = c['radio']
    if not 0 <= r['loss_probability'] < 1 or r['period_s'] <= 0 or r['sigma_dbm'] <= 0:
        raise ValueError('invalid radio noise or timing')
    if not 0 < r['min_distance_m'] < r['max_distance_m']:
        raise ValueError('invalid radio domain')
    if any(v <= 0 for v in c['filter'].values()):
        raise ValueError('filter parameters must be positive')
    return c


def estimator_config(c):
    """Remove generation-only true radio, bias and noise parameters."""
    return {k: c[k] for k in ['schema_version', 'track_m', 'receiver_z_m', 'start', 'filter']} | {
        'beacons': [{'id': b['id'], 'xyz': b['xyz']} for b in c['beacons']]}


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def event(t, seq, kind, payload, delay=0):
    return {'schema_version':1, 't':float(t), 'delivery_t':float(t+delay),
            'seq':int(seq), 'kind':kind, 'payload':payload}


def validate(e):
    if e['schema_version'] != 1 or e['kind'] not in {'encoder', 'imu', 'ble'}:
        raise ValueError('unsupported event')
    if not np.isfinite([e['t'], e['delivery_t']]).all() or e['t'] < 0 or e['delivery_t'] < e['t']:
        raise ValueError('invalid event timestamps')
    p = e['payload']
    values = {'encoder':['speed_mps', 'yaw_rate_radps'], 'imu':['ax', 'ay', 'az', 'gx', 'gy', 'gz'],
              'ble':['rssi_dbm']}[e['kind']]
    if not np.isfinite([p[k] for k in values]).all():
        raise ValueError('nonfinite sensor payload')
    if e['kind'] == 'ble' and not isinstance(p['beacon_id'], str):
        raise ValueError('invalid beacon ID')


def write_json(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def write_events(path, events):
    with Path(path).open('w') as f:
        for e in events:
            validate(e)
            f.write(json.dumps(e, allow_nan=False)+'\n')


def read_events(path):
    with Path(path).open() as f:
        for line in f:
            e = json.loads(line); validate(e); yield e
