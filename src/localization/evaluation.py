"""Absolute-map-frame metrics. Missing BLE fixes remain missing."""
import numpy as np
from .models import wrap


def metrics(times,states,cov,truth,engine):
    gt=np.column_stack([np.interp(times,truth[:,0],truth[:,j]) for j in range(1,5)])
    result={}
    for name,x in states.items():
        valid=np.isfinite(x[:,:2]).all(axis=1)
        error=np.linalg.norm(x[valid,:2]-gt[valid,:2],axis=1)
        m={'availability':float(valid.mean()),'samples':int(valid.sum())}
        if len(error):
            m.update(rmse_m=float(np.sqrt(np.mean(error**2))),median_m=float(np.median(error)),
                     p90_m=float(np.percentile(error,90)),final_error_m=float(error[-1]))
        if np.isfinite(x[:,2]).all():
            m['heading_rmse_deg']=float(np.rad2deg(np.sqrt(np.mean(wrap(x[:,2]-gt[:,2])**2))))
        result[name]=m
    common=np.logical_and.reduce([np.isfinite(x[:,:2]).all(axis=1) for x in states.values()])
    for name,x in states.items():
        if common.any():
            result[name]['common_grid_rmse_m']=float(np.sqrt(np.mean(np.sum((x[common,:2]-gt[common,:2])**2,axis=1))))
    error=states['fused'][:,:2]-gt[:,:2]
    nees=np.einsum('ni,nij,nj->n',error,np.linalg.inv(cov[:,:2,:2]),error)
    diag=engine.filters['fused'].diagnostics
    nis=[d['nis'] for d in diag if 'nis' in d]
    packet_times=np.array(engine.accepted_ble_times)
    gaps=np.diff(np.r_[times[0],packet_times,times[-1]]) if len(packet_times) else [times[-1]-times[0]]
    # Recovery: first continuous two-second interval below 1.5m after nominal outage end.
    e=np.linalg.norm(error,axis=1); recovery=None
    count=max(1,int(2/np.median(np.diff(times))))
    for i in np.flatnonzero(times>=45):
        if i+count<=len(times) and np.all(e[i:i+count]<1.5):
            recovery=float(times[i]-45); break
    result['diagnostics']={'position_nees_mean':float(nees.mean()),
        'position_95pct_ellipse_coverage':float(np.mean(nees<=5.991)),
        'mean_nis_before_gating':float(np.mean(nis)) if nis else None,
        'rejection_fraction':float(np.mean([not d['accepted'] for d in diag])) if diag else None,
        'late_packets':engine.late,'unknown_ids':engine.unknown,'stale_intervals':engine.stale,
        'max_radio_gap_s':float(np.max(gaps)), 'recovery_after_45s_below_1p5m_for_2s':recovery,
        'minimum_covariance_eigenvalue':float(np.linalg.eigvalsh(cov).min()),
        'common_grid_availability':float(common.mean())}
    return result
