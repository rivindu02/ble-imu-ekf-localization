#!/usr/bin/env python3
"""Rebuild README figures and animations from recorded sensor-fusion runs."""
import json
import os
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('MPLCONFIGDIR',str(ROOT/'.cache/matplotlib'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
RESULTS=ROOT/'results/localization'
ASSETS=ROOT/'docs/assets'

def gazebo_record():
    for folder in sorted(RESULTS.glob('gazebo_*'), key=lambda p:p.stat().st_mtime, reverse=True):
        manifest=folder/'manifest.json'; scored=folder/'metrics.json'
        if not manifest.exists() or not scored.exists(): continue
        m=json.loads(manifest.read_text())
        if m.get('status')!='complete' or not (folder/'recording.npz').exists(): continue
        if not m.get('native_imu_messages') or not m.get('encoder_messages'): continue
        return folder,m,json.loads(scored.read_text())
    return None


def gazebo_figure():
    evidence=gazebo_record()
    if evidence is None: return
    folder,m,_=evidence; s=np.load(folder/'recording.npz'); t=s['time']; truth=s['truth']
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    axes[0].plot(truth[:,1],truth[:,2],'k--',label='Truth',lw=2)
    gt=np.column_stack([np.interp(t,truth[:,0],truth[:,j]) for j in [1,2]])
    for name,color in [('wheel_only','#d97706'),('wheel_imu','#a16207'),('ble_wheel','#8b5cf6'),('fused','#2563eb')]:
        axes[0].plot(s[name][:,0],s[name][:,1],label=name,color=color)
        axes[1].plot(t,np.linalg.norm(s[name][:,:2]-gt,axis=1),label=name,color=color)
    if m['scenario']=='beacon_outage': axes[1].axvspan(25,45,color='gray',alpha=.2,label='BLE outage')
    axes[0].set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title='Gazebo: recorded trajectory')
    axes[1].set(xlabel='Simulation time (s)',ylabel='Position error (m)',title='Physics run; outage shaded')
    for ax in axes: ax.legend(fontsize=8)
    fig.savefig(ASSETS/'gazebo-outage.png',dpi=160); plt.close(fig)


def figures(summary):
    rows=summary['summary']; labels=[r['scenario']+'/'+r['route'] for r in rows]
    fig,ax=plt.subplots(figsize=(13,6),layout='constrained')
    xx=np.arange(len(rows)); width=.19
    for i,mode in enumerate(['wheel_only','wheel_imu','ble_wheel','fused']):
        mean=[r['modes'][mode]['rmse_mean_m'] for r in rows]; sd=[r['modes'][mode]['rmse_sd_m'] for r in rows]
        ax.bar(xx+(i-1.5)*width,mean,width,yerr=sd,label=mode,capsize=2)
    ax.set_xticks(xx,labels,rotation=45,ha='right',fontsize=8); ax.set(ylabel='Position RMSE (m)',title='Paired synthetic trials: mean ± sample standard deviation'); ax.legend()
    fig.savefig(ASSETS/'scenarios.png',dpi=160); plt.close(fig)
    samples=json.loads((ROOT/'data/simulated/calibration/samples.json').read_text()); cal=json.loads((ROOT/'data/simulated/calibration/fit.json').read_text())
    fig,axes=plt.subplots(2,2,figsize=(10,7),layout='constrained')
    for ax,(identity,p) in zip(axes.flat,cal['beacons'].items()):
        vals=[r for r in samples if r['beacon_id']==identity and r['split']=='validation']
        ax.scatter([r['distance_m'] for r in vals],[r['rssi_dbm'] for r in vals],s=6,alpha=.3,label='Held-out observations')
        d=np.geomspace(.5,25,100); ax.plot(d,p['p0_dbm']-10*p['n']*np.log10(d),label='Training fit')
        ax.set(xscale='log',xlabel='Distance (m)',ylabel='RSSI (dBm)',title=f'{identity}; held-out RMSE {p["validation_rmse_dbm"]:.2f} dBm'); ax.legend(fontsize=7)
    fig.savefig(ASSETS/'calibration.png',dpi=150); plt.close(fig)
    diag=json.loads((RESULTS/'nominal_rectangle_7/innovations.json').read_text())
    fig,ax=plt.subplots(figsize=(10,3.5),layout='constrained')
    for accepted,color in [(True,'#2563eb'),(False,'#dc2626')]:
        vals=[r for r in diag if r['accepted']==accepted and 'nis' in r]
        ax.scatter([r['t'] for r in vals],[r['nis'] for r in vals],s=8,c=color,label='Accepted' if accepted else 'Rejected')
    ax.axhline(16,color='black',ls='--',label='Configured gate'); ax.set(xlabel='Time (s)',ylabel='Scalar RSSI NIS',title='Nominal seed 7: pre-gating innovation diagnostics'); ax.legend()
    fig.savefig(ASSETS/'innovations.png',dpi=150); plt.close(fig)


def clean_trajectory():
    s=np.load(RESULTS/'nominal_rectangle_7/recording.npz')
    fig,ax=plt.subplots(figsize=(7,5.5),layout='constrained')
    truth=s['truth']; ax.plot(truth[:,1],truth[:,2],'k--',lw=2,label='Truth',zorder=4)
    for name,color in [('wheel_only','#d97706'),('wheel_imu','#a16207'),('ble_wheel','#8b5cf6'),('fused','#2563eb')]:
        ax.plot(s[name][:,0],s[name][:,1],label=name,color=color,lw=1.5)
    for b in json.loads((ROOT/'config/localization.json').read_text())['beacons']:
        ax.scatter(*b['xyz'][:2],marker='^',color='#059669'); ax.annotate(b['id'],b['xyz'][:2])
    ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title='Nominal seed 7: wheel-assisted trajectories')
    ax.legend(fontsize=9); fig.savefig(ASSETS/'trajectory.png',dpi=160); plt.close(fig)


def animation():
    s=np.load(RESULTS/'nominal_rectangle_7/recording.npz'); t=s['time']; gt=s['truth']
    fig,ax=plt.subplots(figsize=(6,6),layout='constrained'); ax.plot(gt[:,1],gt[:,2],'k--',label='Truth route')
    lines={name:ax.plot([],[],label=name,lw=2)[0] for name in ['wheel_only','fused']}
    title=ax.set_title('Synthetic BLE + IMU + wheels'); ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal'); ax.legend()
    for b in json.loads((ROOT/'config/localization.json').read_text())['beacons']:
        ax.scatter(*b['xyz'][:2],marker='^',color='green'); ax.annotate(b['id'],b['xyz'][:2])
    ax.set_xlim(-4,8); ax.set_ylim(-3,10)
    def update(k):
        for name,line in lines.items(): line.set_data(s[name][:k+1,0],s[name][:k+1,1])
        title.set_text(f'Simulated localization | t={t[k]:.1f} s')
        return list(lines.values())+[title]
    anim=FuncAnimation(fig,update,frames=np.linspace(0,len(t)-1,100,dtype=int),interval=80,blit=False)
    anim.save(ASSETS/'localization.gif',writer=PillowWriter(fps=12)); plt.close(fig)


def outage_animation():
    evidence=gazebo_record()
    if evidence is None:
        print('No completed Gazebo recording; skipping outage animation')
        return
    folder,m,_=evidence
    s=np.load(folder/'recording.npz'); t=s['time']; truth=s['truth']
    fig,axes=plt.subplots(1,2,figsize=(9,4.4),layout='constrained')
    axes[0].plot(truth[:,1],truth[:,2],'k--',alpha=.45,label='Truth route')
    gt=np.column_stack([np.interp(t,truth[:,0],truth[:,j]) for j in [1,2]])
    paths={}; errors={}
    for name,color in [('wheel_only','#d97706'),('wheel_imu','#a16207'),('fused','#2563eb')]:
        paths[name]=axes[0].plot([],[],color=color,lw=2,label=name)[0]
        errors[name]=axes[1].plot([],[],color=color,lw=1.6,label=name)[0]
    axes[0].set(xlim=(-5,6),ylim=(-1,10),xlabel='x (m)',ylabel='y (m)',aspect='equal')
    axes[1].set(xlim=(0,max(t)),ylim=(0,5),xlabel='Simulation time (s)',ylabel='Position error (m)')
    if m['scenario']=='beacon_outage': axes[1].axvspan(25,45,color='#94a3b8',alpha=.25)
    for ax in axes: ax.legend(fontsize=7,loc='upper left')
    title=fig.suptitle('Recorded Gazebo sensor fusion')
    def update(k):
        for name,line in paths.items():
            line.set_data(s[name][:k+1,0],s[name][:k+1,1])
            errors[name].set_data(t[:k+1],np.linalg.norm(s[name][:k+1,:2]-gt[:k+1],axis=1))
        status='BLE OUTAGE' if m['scenario']=='beacon_outage' and 25<=t[k]<=45 else 'BLE updates available'
        title.set_text(f'Recorded Gazebo trajectory | {t[k]:.1f} s | {status}')
        return [*paths.values(),*errors.values(),title]
    ani=FuncAnimation(fig,update,frames=np.linspace(0,len(t)-1,100,dtype=int),interval=80)
    ani.save(ASSETS/'gazebo-outage.gif',writer=PillowWriter(fps=12))
    plt.close(fig)


def main():
    import argparse
    import sys
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--skip-animation',action='store_true',help='Rebuild static figures only')
    args=p.parse_args(); ASSETS.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(ROOT/'src'))
    from localization.events import load_config,write_json
    from localization.calibration import generate_samples,fit
    folder=ROOT/'data/simulated/calibration'
    if not (folder/'samples.json').exists():
        samples=generate_samples(load_config(ROOT/'config/localization.json'))
        write_json(folder/'samples.json',samples); write_json(folder/'fit.json',fit(samples))
    figures(json.loads((RESULTS/'summary.json').read_text()))
    clean_trajectory(); gazebo_figure()
    if not args.skip_animation:
        animation(); outage_animation()
    print(f'Rebuilt README media in {ASSETS}')

if __name__=='__main__': main()
