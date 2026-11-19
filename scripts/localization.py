#!/usr/bin/env python3
"""Generate, calibrate, replay, compare and plot simulated localization."""
import argparse
import csv
import json
import os
from pathlib import Path
import platform
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.cache/matplotlib'))
from localization.events import load_config,estimator_config,write_json,write_events,read_events,fingerprint
from localization.simulation import generate,SCENARIOS,ROUTES
from localization.calibration import generate_samples,fit
from localization.replay import replay,MODES
from localization.evaluation import metrics


def calibration(c):
    rows=generate_samples(c)
    fitted=fit(rows)
    out=ROOT/'data/simulated/calibration'; out.mkdir(parents=True,exist_ok=True)
    write_json(out/'samples.json',rows); write_json(out/'fit.json',fitted)
    return fitted


def run(c,cal,seed,scenario,route,save=False):
    actual,events,truth=generate(c,seed,scenario,route)
    ec=estimator_config(actual)
    effective=json.loads(json.dumps(cal))
    if scenario=='overconfident':
        for b in effective['beacons'].values(): b['variance_dbm2']*=.04
    times,states,cov,engine=replay(ec,effective,events,truth[:,0])
    scored=metrics(times,states,cov,truth,engine)
    manifest={'schema_version':1,'run_id':f'{scenario}_{route}_{seed}', 'seed':seed,
              'scenario':scenario,'route':route,'config':actual,'calibration_hash':fingerprint(effective),
              'clock':'simulated_seconds','status':'complete','python':platform.python_version(),
              'numpy':np.__version__, 'source_hash':source_hash()}
    if save:
        out=ROOT/'results/localization'/manifest['run_id']; out.mkdir(parents=True,exist_ok=True)
        write_events(out/'events.jsonl',events)
        np.savez_compressed(out/'recording.npz',time=times,truth=truth,covariance=cov,**states)
        write_json(out/'estimator_config.json',ec); write_json(out/'calibration.json',effective)
        write_json(out/'manifest.json',manifest); write_json(out/'metrics.json',scored)
        write_json(out/'innovations.json',engine.filters['fused'].diagnostics)
        plot_run(out,times,states,cov,truth,actual)
    return {'scenario':scenario,'route':route,'seed':seed,'metrics':scored}


def source_hash():
    files=sorted((ROOT/'src/localization').glob('*.py'))+[Path(__file__)]
    return fingerprint({p.name:p.read_text() for p in files})


def plot_run(out,times,states,cov,truth,c):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    ax=axes[0]; ax.plot(truth[:,1],truth[:,2],'k-',label='Truth')
    colors={'wheel_only':'#d97706','wheel_imu':'#a16207','ble_only':'#9ca3af','ble_wheel':'#8b5cf6','fused':'#2563eb','imu_only':'#dc2626'}
    for name,x in states.items():
        if name=='imu_only': continue
        ax.plot(x[:,0],x[:,1],label=name,color=colors[name],alpha=.85,lw=1.3)
    for b in c['beacons']:
        ax.scatter(*b['xyz'][:2],marker='^',color='#059669'); ax.annotate(b['id'],b['xyz'][:2])
    for k in np.linspace(0,len(times)-1,12,dtype=int):
        vals,vecs=np.linalg.eigh(cov[k,:2,:2]); angle=np.rad2deg(np.arctan2(vecs[1,1],vecs[0,1]))
        ax.add_patch(Ellipse(states['fused'][k,:2],2*np.sqrt(5.991*vals[1]),2*np.sqrt(5.991*vals[0]),angle=angle,fill=False,color='#2563eb',alpha=.25))
    ax.set(xlabel='x (m)',ylabel='y (m)',title='Surveyed map; 95% position ellipses',aspect='equal'); ax.legend(fontsize=8)
    gt=np.column_stack([np.interp(times,truth[:,0],truth[:,j]) for j in [1,2]])
    for name,x in states.items():
        e=np.linalg.norm(x[:,:2]-gt,axis=1)
        axes[1].plot(times,e,label=name,color=colors[name],lw=1)
    axes[1].set(xlabel='Simulation time (s)',ylabel='Position error (m)',yscale='symlog',title='All baselines, including IMU-only drift')
    axes[1].legend(fontsize=8); fig.savefig(out/'trajectory.png',dpi=150); plt.close(fig)


def aggregate(rows):
    result=[]
    for scenario,route in sorted({(r['scenario'],r['route']) for r in rows}):
        group=[r for r in rows if r['scenario']==scenario and r['route']==route]
        entry={'scenario':scenario,'route':route,'trials':len(group),'modes':{}}
        for mode in MODES:
            vals=[r['metrics'][mode]['rmse_m'] for r in group if 'rmse_m' in r['metrics'][mode]]
            entry['modes'][mode]={'rmse_mean_m':float(np.mean(vals)) if vals else None,
                                 'rmse_sd_m':float(np.std(vals,ddof=1)) if len(vals)>1 else 0.,
                                 'availability_mean':float(np.mean([r['metrics'][mode]['availability'] for r in group]))}
        result.append(entry)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['calibrate','run','batch','replay'])
    p.add_argument('--config',type=Path,default=ROOT/'config/localization.json')
    p.add_argument('--seed',type=int,default=7); p.add_argument('--trials',type=int,default=20)
    p.add_argument('--scenario',choices=SCENARIOS,default='nominal'); p.add_argument('--route',choices=ROUTES,default='rectangle')
    p.add_argument('--run-dir',type=Path); p.add_argument('--all-routes',action='store_true')
    args=p.parse_args(); c=load_config(args.config)
    if args.command=='replay':
        if not args.run_dir: p.error('--run-dir required')
        d=args.run_dir
        config=json.loads((d/'estimator_config.json').read_text()); cal=json.loads((d/'calibration.json').read_text())
        saved=np.load(d/'recording.npz'); times,states,cov,engine=replay(config,cal,list(read_events(d/'events.jsonl')),saved['time'])
        for mode in MODES: np.testing.assert_allclose(states[mode],saved[mode],equal_nan=True)
        np.testing.assert_allclose(cov,saved['covariance'])
        print('Replay matches all saved estimates and covariance'); return
    cal=calibration(c)
    if args.command=='calibrate': print(json.dumps(cal,indent=2)); return
    if args.command=='run':
        row=run(c,cal,args.seed,args.scenario,args.route,True); print(json.dumps(row,indent=2)); return
    if args.trials<1: p.error('--trials must be positive')
    rows=[]; start=time.time()
    cases=[(s,'rectangle') for s in SCENARIOS]
    if args.all_routes: cases += [('nominal',r) for r in ROUTES if r!='rectangle']
    for scenario,route in cases:
        for seed in range(7,7+args.trials):
            rows.append(run(c,cal,seed,scenario,route,seed==7))
        print(f'{scenario}/{route}: {args.trials} trials ({time.time()-start:.1f}s)',flush=True)
        # Save incrementally so interrupted batches retain auditable completed trials.
        write_json(ROOT/'results/localization/trials.json',rows)
    summary={'trials':args.trials,'seed_range':[7,6+args.trials],'status':'complete','source_hash':source_hash(),
             'summary':aggregate(rows),'elapsed_s':time.time()-start}
    write_json(ROOT/'results/localization/summary.json',summary)
    with (ROOT/'results/localization/metrics.csv').open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['scenario','route','seed','mode','rmse_m','median_m','p90_m','availability'])
        for r in rows:
            for mode in MODES:
                m=r['metrics'][mode]; w.writerow([r['scenario'],r['route'],r['seed'],mode,*[m.get(k,'') for k in ['rmse_m','median_m','p90_m','availability']]])
    print(f'{len(rows)} completed runs')

if __name__=='__main__': main()
