#!/usr/bin/env python3
"""Check completeness of a physical Gazebo run, not just process exit status."""
import argparse
import json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser(); p.add_argument('run',type=Path); a=p.parse_args()
m=json.loads((a.run/'manifest.json').read_text())
if m['status']!='complete': raise SystemExit('Run incomplete: '+m.get('reason','unknown'))
if not m['native_imu_messages'] or not m['encoder_messages']: raise SystemExit('Missing required sensors')
s=np.load(a.run/'recording.npz')
if len(s['time'])<10 or not np.all(np.diff(s['time'])>0): raise SystemExit('Invalid trajectory timestamps')
if not np.isfinite(s['fused']).all(): raise SystemExit('Nonfinite fused trajectory')
if np.linalg.eigvalsh(s['covariance']).min() < -1e-9: raise SystemExit('Non-PSD covariance')
metrics=json.loads((a.run/'metrics.json').read_text())
if not (a.run/'innovations.json').is_file(): raise SystemExit('No BLE updates')
accepted=[r for r in json.loads((a.run/'innovations.json').read_text()) if r['accepted']]
if len({r['beacon_id'] for r in accepted})<3: raise SystemExit('Fewer than three distinct beacons supplied accepted updates')
print('Validated native IMU, encoders, BLE updates, trajectory, covariance and metrics')
