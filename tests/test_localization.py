import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from localization.models import motion,rssi_model
from localization.events import load_config,estimator_config,event,validate
from localization.calibration import generate_samples,fit
from localization.ekf import LocalizationEKF
from localization.replay import FusionEngine,replay,MODES
from localization.simulation import generate


class LocalizationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c=load_config(ROOT/'config/localization.json')
        cls.cal=fit(generate_samples(cls.c))
        cls.ec=estimator_config(cls.c)

    def test_motion_and_radio_jacobians(self):
        x=np.array([2.,3.,.7,.02]); v=.4; g=.2; dt=.13; eps=1e-6
        _,F,G=motion(x,v,g,dt)
        numeric=np.column_stack([(motion(x+np.eye(4)[i]*eps,v,g,dt)[0]-motion(x-np.eye(4)[i]*eps,v,g,dt)[0])/(2*eps) for i in range(4)])
        np.testing.assert_allclose(F,numeric,atol=1e-8)
        numeric_g=np.column_stack([(motion(x,v+eps,g,dt)[0]-motion(x,v-eps,g,dt)[0])/(2*eps),
                                   (motion(x,v,g+eps,dt)[0]-motion(x,v,g-eps,dt)[0])/(2*eps)])
        np.testing.assert_allclose(G,numeric_g,atol=1e-8)
        _,H=rssi_model(x,[1,1,1.2],.35,-59,2.1)
        numeric_h=np.array([(rssi_model(x+np.eye(4)[i]*eps,[1,1,1.2],.35,-59,2.1)[0]-rssi_model(x-np.eye(4)[i]*eps,[1,1,1.2],.35,-59,2.1)[0])/(2*eps) for i in range(4)])
        np.testing.assert_allclose(H,numeric_h,atol=1e-7)

    def test_calibration_heldout_and_identifiability(self):
        for b in self.c['beacons']:
            p=self.cal['beacons'][b['id']]
            self.assertLess(abs(p['p0_dbm']-b['p0_dbm']),.5)
            self.assertLess(abs(p['n']-b['n']),.05)
            self.assertGreater(p['validation_rmse_dbm'],1.)
        with self.assertRaises(ValueError):
            fit([{'beacon_id':'x','distance_m':1.,'rssi_dbm':-60,'split':s} for s in ['train']*5+['validation']])

    def test_exact_rssi_no_correction_and_psd(self):
        f=LocalizationEKF(self.ec,self.cal)
        for i in range(100):
            f.predict(.05,.4,.2)
            for b in self.ec['beacons']:
                p=self.cal['beacons'][b['id']]
                z,_=rssi_model(f.x,b['xyz'],self.ec['receiver_z_m'],p['p0_dbm'],p['n'])
                before=f.x.copy(); self.assertTrue(f.update(b['id'],z))
                np.testing.assert_allclose(f.x,before)
            np.testing.assert_allclose(f.P,f.P.T,atol=1e-12)
            self.assertGreaterEqual(np.linalg.eigvalsh(f.P).min(),-1e-12)
        self.assertTrue(-np.pi<=f.x[2]<np.pi)

    def test_rejects_unknown_outlier_and_nonfinite(self):
        f=LocalizationEKF(self.ec,self.cal)
        self.assertFalse(f.update('missing',-60))
        self.assertFalse(f.update('B01',np.nan))
        self.assertFalse(f.update('B01',1000))
        self.assertEqual(f.diagnostics[-1]['reason'],'nis_gate')
        with self.assertRaises(ValueError): motion(f.x,0,0,0)
        with self.assertRaises(ValueError): rssi_model(f.x,[0,0,.35],.35,-60,2)

    def test_no_truth_in_estimator_contract(self):
        self.assertNotIn('imu',self.ec)
        self.assertNotIn('p0_dbm',self.ec['beacons'][0])
        c=copy.deepcopy(self.c); c['duration_s']=4
        _,events,_=generate(c)
        for e in events:
            validate(e)
            self.assertNotIn('truth',e['payload'])
            self.assertNotIn('distance',e['payload'])

    def test_late_data_stale_control_and_time_reset(self):
        eng=FusionEngine(self.ec,self.cal)
        eng.process(event(2,0,'encoder',{'speed_mps':1.,'yaw_rate_radps':0.}))
        eng.process(event(2,1,'imu',{'ax':0.,'ay':0.,'az':9.81,'gx':0.,'gy':0.,'gz':0.}))
        eng.advance(2.1); self.assertGreater(eng.filters['fused'].x[0],0)
        x=eng.filters['fused'].x.copy(); P=eng.filters['fused'].P.copy()
        eng.process(event(2.05,2,'ble',{'beacon_id':'B01','rssi_dbm':-60.},delay=.6))
        self.assertEqual(eng.late,1); np.testing.assert_equal(x,eng.filters['fused'].x)
        eng.advance(3); np.testing.assert_equal(x,eng.filters['fused'].x)
        self.assertGreater(eng.filters['fused'].P[0,0],P[0,0])
        with self.assertRaises(ValueError): eng.advance(1)

    def test_determinism_paired_randomness_and_streaming(self):
        c=copy.deepcopy(self.c); c['duration_s']=8
        _,events,truth=generate(c,7)
        _,again,truth2=generate(c,7)
        self.assertEqual(events,again); np.testing.assert_equal(truth,truth2)
        _,noisy,_=generate(c,7,'noisy_rssi')
        self.assertEqual([e for e in events if e['kind']!='ble'],[e for e in noisy if e['kind']!='ble'])
        times,states,cov,eng=replay(self.ec,self.cal,events,truth[:,0])
        gyro=np.mean([e['payload']['gz'] for e in events if e['kind']=='imu' and e['t']<2])
        streaming=FusionEngine(self.ec,self.cal,gyro); index=0
        for i,t in enumerate(times):
            while index<len(events) and events[index]['delivery_t']<=t+1e-9:
                e=events[index]; index+=1
                if e['t']>=2: streaming.process(e)
            streaming.advance(t)
            np.testing.assert_allclose(streaming.filters['fused'].x,states['fused'][i])
            np.testing.assert_allclose(streaming.filters['fused'].P,cov[i])

    def test_outage_has_missing_ble_and_finite_fusion(self):
        c=copy.deepcopy(self.c); c['duration_s']=48
        _,events,truth=generate(c,8,'beacon_outage')
        times,states,cov,eng=replay(self.ec,self.cal,events,truth[:,0])
        outage=(times>27)&(times<44)
        self.assertTrue(np.isnan(states['ble_only'][outage,:2]).all())
        self.assertTrue(np.isfinite(states['fused']).all())
        self.assertGreater(np.linalg.eigvalsh(cov).min(),0)

    def test_invalid_events(self):
        with self.assertRaises(ValueError): validate(event(-1,0,'ble',{'beacon_id':'B01','rssi_dbm':-60}))
        with self.assertRaises(ValueError): validate(event(1,0,'ble',{'beacon_id':'B01','rssi_dbm':float('nan')}))


if __name__=='__main__': unittest.main()
