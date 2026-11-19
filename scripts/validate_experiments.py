#!/usr/bin/env python3
"""Verify a complete paired benchmark and its documented nominal acceptance gate."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'src'))
from localization.simulation import SCENARIOS,ROUTES
from localization.replay import MODES
p=argparse.ArgumentParser(); p.add_argument('--results',type=Path,default=ROOT/'results/localization'); a=p.parse_args()
s=json.loads((a.results/'summary.json').read_text()); rows=json.loads((a.results/'trials.json').read_text())
if s['status']!='complete': raise SystemExit('Batch is incomplete')
expected={(scenario,'rectangle',seed) for scenario in SCENARIOS for seed in range(s['seed_range'][0],s['seed_range'][1]+1)}
expected|={('nominal',route,seed) for route in ROUTES for seed in range(s['seed_range'][0],s['seed_range'][1]+1)}
actual={(r['scenario'],r['route'],r['seed']) for r in rows}
if expected!=actual or len(actual)!=len(rows): raise SystemExit('Missing/duplicate benchmark conditions')
for r in rows:
    if any(mode not in r['metrics'] for mode in MODES): raise SystemExit('Missing baseline')
    d=r['metrics']['diagnostics']
    if d['minimum_covariance_eigenvalue'] < -1e-9: raise SystemExit('Covariance is not PSD')
    for mode in MODES:
        m=r['metrics'][mode]
        if not 0<=m['availability']<=1: raise SystemExit('Invalid availability')
        if m['samples'] and not np.isfinite(m['rmse_m']): raise SystemExit('Nonfinite scored error')
nominal=next(r for r in s['summary'] if r['scenario']=='nominal' and r['route']=='rectangle')
if nominal['modes']['fused']['rmse_mean_m']>=nominal['modes']['wheel_only']['rmse_mean_m']:
    raise SystemExit('Nominal aggregate fusion did not improve over wheel-only')
recordings=0
for scenario,route in {(r['scenario'],r['route']) for r in rows}:
    path=a.results/f'{scenario}_{route}_7'
    m=json.loads((path/'manifest.json').read_text())
    if m['status']!='complete': raise SystemExit('Incomplete representative run')
    z=np.load(path/'recording.npz')
    if np.any(np.diff(z['time'])<=0) or not np.isfinite(z['fused']).all(): raise SystemExit('Invalid saved recording')
    if len({r['beacon_id'] for r in json.loads((path/'innovations.json').read_text())})<3: raise SystemExit('Too few observed beacons')
    recordings+=1
print(f'Validated {len(rows)} paired trials, {recordings} complete recordings, six baselines, PSD covariances and nominal fusion improvement')
