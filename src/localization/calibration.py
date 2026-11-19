"""Fit RSSI against log10 surveyed distance; separate generated calibration runs."""
import numpy as np
from .events import fingerprint


def generate_samples(config, seed=90210):
    rng = np.random.default_rng(seed)
    records = []
    for b in config['beacons']:
        for distance in [.5, 1., 2., 3., 5., 8., 12., 18., 25.]:
            for j in range(100):
                records.append({'beacon_id':b['id'], 'distance_m':distance,
                                'rssi_dbm':float(b['p0_dbm']-10*b['n']*np.log10(distance)+rng.normal(0,config['radio']['sigma_dbm'])),
                                'split':'validation' if j % 5 == 0 else 'train'})
    return records


def fit(records):
    models = {}
    for identity in sorted({r['beacon_id'] for r in records}):
        rows = [r for r in records if r['beacon_id'] == identity]
        train = [r for r in rows if r['split'] == 'train']
        d = np.array([r['distance_m'] for r in train])
        y = np.array([r['rssi_dbm'] for r in train])
        if len(d) < 3 or np.any(d <= 0) or not np.isfinite(y).all():
            raise ValueError('invalid calibration samples')
        A = np.column_stack([np.ones(len(d)), -10*np.log10(d)])
        if np.linalg.matrix_rank(A) != 2:
            raise ValueError('calibration distances do not identify both parameters')
        p0, n = np.linalg.lstsq(A,y,rcond=None)[0]
        if n <= 0:
            raise ValueError('nonphysical path loss fit')
        validation = [r for r in rows if r['split'] == 'validation']
        residual = [r['rssi_dbm']-p0+10*n*np.log10(r['distance_m']) for r in validation]
        if not residual:
            raise ValueError('held-out calibration samples required')
        models[identity] = {'p0_dbm':float(p0), 'n':float(n),
                            'variance_dbm2':max(float(np.mean(np.square(residual))), .25),
                            'validation_rmse_dbm':float(np.sqrt(np.mean(np.square(residual)))),
                            'min_distance_m':float(d.min()), 'max_distance_m':float(d.max())}
    return {'schema_version':1, 'beacons':models, 'sample_hash':fingerprint(records)}
