#!/usr/bin/env python3
"""Separate sensor/controller, estimator and evaluation nodes in one executor.

Only the sensor/controller and evaluator subscribe to simulator truth. The fusion
node accepts JSON sensor events only. Native Gazebo IMU is required (no fallback).
"""
import argparse
import csv
import json
import platform
from pathlib import Path
import signal
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/'src'))
from localization.events import load_config,estimator_config,event,write_json,write_events,fingerprint
from localization.replay import FusionEngine,MODES
from localization.models import wrap
from localization.evaluation import metrics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor,ExternalShutdownException
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import Twist,PoseStamped,Point
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker,MarkerArray

PREFIX='/localization_demo/'


def stamp_seconds(stamp): return stamp.sec+stamp.nanosec*1e-9

def yaw(q): return float(np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


class SensorController(Node):
    def __init__(self,c,scenario):
        super().__init__('localization_sensor_controller'); self.c=c; self.scenario=scenario
        self.pose=None; self.seq=0; self.start=None
        self.enc_rng,self.imu_rng,self.radio_rng,self.loss_rng=[np.random.default_rng(s) for s in np.random.SeedSequence(7).spawn(4)]
        self.pub=self.create_publisher(String,PREFIX+'sensor_events',100)
        self.cmd=self.create_publisher(Twist,PREFIX+'cmd_vel',10)
        self.create_subscription(ModelStates,PREFIX+'model_states',self.on_models,10)
        self.create_subscription(Imu,PREFIX+'imu_native',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(Odometry,PREFIX+'wheel_odom',self.on_encoder,20)
        self.create_timer(.05,self.drive); self.create_timer(c['radio']['period_s']/len(c['beacons']),self.radio)
        self.beacon_index=0; self.imu_count=0; self.encoder_count=0; self.stopped=False

    def now(self): return self.get_clock().now().nanoseconds*1e-9

    def elapsed(self):
        return max(0.,self.now()-self.start) if self.start is not None else 0.

    def send(self,t,kind,payload):
        # Anchor the run when observations begin, rather than assuming the ROS
        # subscribers are ready before Gazebo reaches its second simulated second.
        if self.start is None: self.start=t
        if t<self.start: return
        e=event(t-self.start,self.seq,kind,payload); self.seq+=1
        # Acquisition time comes from sensor header or current simulation clock.
        e['gazebo_acquisition_t']=t
        e['received_sim_t']=self.elapsed()
        msg=String(); msg.data=json.dumps(e); self.pub.publish(msg)

    def on_models(self,msg):
        if 'localization_robot' in msg.name:
            p=msg.pose[msg.name.index('localization_robot')]
            self.pose=np.array([p.position.x,p.position.y,yaw(p.orientation)])

    def on_imu(self,msg):
        self.imu_count+=1
        p={'ax':msg.linear_acceleration.x,'ay':msg.linear_acceleration.y,'az':msg.linear_acceleration.z,
           'gx':msg.angular_velocity.x,'gy':msg.angular_velocity.y,
           'gz':msg.angular_velocity.z+self.c['imu']['gyro_bias_radps']}
        self.send(stamp_seconds(msg.header.stamp),'imu',p)

    def on_encoder(self,msg):
        self.encoder_count+=1
        c=self.c['encoder']
        self.send(stamp_seconds(msg.header.stamp),'encoder',{
            'speed_mps':float(msg.twist.twist.linear.x*c['speed_scale']+self.enc_rng.normal(0,c['speed_sigma_mps'])),
            'yaw_rate_radps':float(msg.twist.twist.angular.z+c['yaw_bias_radps']+self.enc_rng.normal(0,c['yaw_sigma_radps']))})

    def drive(self):
        cmd=Twist(); t=self.elapsed()
        if self.pose is not None and t>3 and not self.stopped:
            # Truth-based scripted route controller is outside localization.
            x,y,heading=self.pose
            center=np.array([0.,4.]); delta=self.pose[:2]-center
            radius=np.linalg.norm(delta)
            desired=np.arctan2(delta[1],delta[0])+np.pi/2+np.arctan2(radius-4.,1.5)
            error=wrap(desired-heading)
            cmd.linear.x=.4*max(.25,np.cos(error)); cmd.angular.z=float(np.clip(.8*error,-.65,.65))
        self.cmd.publish(cmd)

    def radio(self):
        if self.pose is None: return
        t=self.now(); b=self.c['beacons'][self.beacon_index%len(self.c['beacons'])]; self.beacon_index+=1
        if self.scenario=='beacon_outage' and 25<=self.elapsed()<=45: return
        if self.loss_rng.random()<self.c['radio']['loss_probability']: return
        d=np.sqrt(np.sum((self.pose[:2]-b['xyz'][:2])**2)+(self.c['receiver_z_m']-b['xyz'][2])**2)
        if not self.c['radio']['min_distance_m']<=d<=self.c['radio']['max_distance_m']: return
        rssi=b['p0_dbm']-10*b['n']*np.log10(d)+self.radio_rng.normal(0,self.c['radio']['sigma_dbm'])
        self.send(t,'ble',{'beacon_id':b['id'],'rssi_dbm':float(rssi)})


class FusionNode(Node):
    def __init__(self,c,cal,out):
        super().__init__('localization_fusion'); self.c=estimator_config(c); self.cal=cal; self.out=out
        self.engine=None; self.gyros=[]; self.events=[]; self.rows=[]; self.times=[]; self.cov=[]; self.partial=True
        self.create_subscription(String,PREFIX+'sensor_events',self.on_event,100)
        self.state_pub=self.create_publisher(String,PREFIX+'state',20)
        self.odom_pub=self.create_publisher(Odometry,PREFIX+'fused_odom',20)
        self.paths={name:RosPath() for name in ['fused','wheel_only']}
        self.path_pubs={name:self.create_publisher(RosPath,PREFIX+name+'_path',10) for name in self.paths}
        self.markers=self.create_publisher(MarkerArray,PREFIX+'markers',10)

    def on_event(self,msg):
        e=json.loads(msg.data); self.events.append(e)
        if e['kind']=='imu' and e['t']<2: self.gyros.append(e['payload']['gz'])
        if e['t']<2: return
        if self.engine is None:
            if not self.gyros:
                self.get_logger().error('Missing stationary IMU calibration; refusing fusion'); return
            self.engine=FusionEngine(self.c,self.cal,float(np.mean(self.gyros)))
        # Zero transport-latency model; receipt jitter is logged separately.
        # Out-of-order acquisition timestamps are rejected rather than rewinding.
        adjusted=dict(e); adjusted['delivery_t']=max(e['t'],self.engine.delivery_time)
        self.engine.process(adjusted)
        if e['kind']!='encoder' or self.engine.time!=e['t'] or (self.times and e['t']<=self.times[-1]): return
        states=self.engine.snapshot(e['t']); self.times.append(e['t']); self.rows.append(states); self.cov.append(self.engine.filters['fused'].P.copy())
        state={'t':e['t'],'x':states['fused'].tolist(),'covariance':self.cov[-1].tolist(),
               'late_packets':self.engine.late,'ble_updates':len(self.engine.filters['fused'].diagnostics)}
        output=String(); output.data=json.dumps(state); self.state_pub.publish(output)
        stamp=self.get_clock().now().to_msg()
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='map'; odom.child_frame_id='base_link'
        x=states['fused']; odom.pose.pose.position.x=float(x[0]); odom.pose.pose.position.y=float(x[1])
        odom.pose.pose.orientation.z=float(np.sin(x[2]/2)); odom.pose.pose.orientation.w=float(np.cos(x[2]/2))
        P=self.cov[-1]; pose_cov=np.zeros((6,6))
        for i,pi in enumerate([0,1,5]):
            for j,pj in enumerate([0,1,5]): pose_cov[pi,pj]=P[i,j]
        for i in [2,3,4]: pose_cov[i,i]=1e6
        odom.pose.covariance=pose_cov.ravel().tolist(); self.odom_pub.publish(odom)
        for name,path in self.paths.items():
            p=PoseStamped(); p.header=odom.header; p.pose.position.x=float(states[name][0]); p.pose.position.y=float(states[name][1]); p.pose.orientation.w=1.
            path.header=odom.header; path.poses.append(p); path.poses=path.poses[-3000:]; self.path_pubs[name].publish(path)
        markers=MarkerArray()
        for i,b in enumerate(self.c['beacons']):
            m=Marker(); m.header=odom.header; m.ns='beacons'; m.id=i; m.type=Marker.SPHERE; m.action=Marker.ADD
            m.pose.position.x=float(b['xyz'][0]); m.pose.position.y=float(b['xyz'][1]); m.pose.position.z=float(b['xyz'][2]); m.pose.orientation.w=1.
            m.scale.x=m.scale.y=m.scale.z=.3; m.color.g=.7; m.color.a=1.; markers.markers.append(m)
        ellipse=Marker(); ellipse.header=odom.header; ellipse.ns='position_95pct'; ellipse.id=0; ellipse.type=Marker.LINE_STRIP; ellipse.action=Marker.ADD
        ellipse.pose.orientation.w=1.; ellipse.scale.x=.03; ellipse.color.b=1.; ellipse.color.a=.8
        vals,vecs=np.linalg.eigh(P[:2,:2])
        for angle in np.linspace(0,2*np.pi,50):
            xy=x[:2]+vecs@(np.sqrt(np.maximum(vals,0)*5.991)*[np.cos(angle),np.sin(angle)])
            p=Point(); p.x=float(xy[0]); p.y=float(xy[1]); p.z=.2; ellipse.points.append(p)
        markers.markers.append(ellipse)
        text=Marker(); text.header=odom.header; text.ns='status'; text.id=0; text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD
        text.pose.position.y=-2.; text.pose.position.z=1.; text.pose.orientation.w=1.; text.scale.z=.35; text.color.r=.1; text.color.a=1.
        text.text=f'Synthetic BLE + native IMU + encoders | updates={state["ble_updates"]} | late={state["late_packets"]}'
        markers.markers.append(text); self.markers.publish(markers)

    def save(self,truth,complete):
        write_events(self.out/'events.jsonl',self.events)
        write_json(self.out/'estimator_config.json',self.c); write_json(self.out/'calibration.json',self.cal)
        if not self.rows: return
        states={name:np.array([r[name] for r in self.rows]) for name in MODES}
        times=np.array(self.times); cov=np.array(self.cov)
        np.savez_compressed(self.out/'recording.npz',time=times,covariance=cov,truth=truth,**states)
        write_json(self.out/'innovations.json',self.engine.filters['fused'].diagnostics)
        if len(truth)>2:
            scored=metrics(times,states,cov,truth,self.engine)
            write_json(self.out/'metrics.json',scored)


class Evaluator(Node):
    def __init__(self,epoch):
        super().__init__('localization_truth_evaluator'); self.truth=[]; self.epoch=epoch
        self.create_subscription(ModelStates,PREFIX+'model_states',self.on_models,10)
        self.path=RosPath(); self.pub=self.create_publisher(RosPath,PREFIX+'truth_path',10)

    def on_models(self,msg):
        if 'localization_robot' not in msg.name: return
        if self.epoch() is None: return
        p=msg.pose[msg.name.index('localization_robot')]; t=self.get_clock().now().nanoseconds*1e-9
        self.truth.append([t-self.epoch(),p.position.x,p.position.y,yaw(p.orientation),0.])
        s=PoseStamped(); s.header.frame_id='map'; s.header.stamp=self.get_clock().now().to_msg(); s.pose=p
        self.path.header=s.header; self.path.poses.append(s); self.path.poses=self.path.poses[-3000:]; self.pub.publish(self.path)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--duration',type=float,default=80); p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--scenario',choices=['nominal','beacon_outage'],default='nominal'); p.add_argument('--wall-timeout',type=float,default=240)
    args,ros_args=p.parse_known_args()
    if args.duration<=3 or args.wall_timeout<=0: p.error('duration must exceed the 3s stationary interval; wall timeout must be positive')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    c=load_config(ROOT/'config/localization.json')
    cal=json.loads((ROOT/'data/simulated/calibration/fit.json').read_text())
    rclpy.init(args=ros_args)
    sensor=SensorController(c,args.scenario); fusion=FusionNode(c,cal,args.output_dir); evaluation=Evaluator(lambda:sensor.start)
    executor=SingleThreadedExecutor()
    for node in [sensor,fusion,evaluation]: executor.add_node(node)
    started=time.monotonic(); last_status=started-5; complete=False; reason='interrupted'
    print(f'Waiting for Gazebo sensors. Headless mode opens no window; target={args.duration:g} simulated seconds.',flush=True)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            sim_t=sensor.elapsed()
            if time.monotonic()-last_status>=5:
                updates=len(fusion.engine.filters['fused'].diagnostics) if fusion.engine else 0
                print(f'Progress: {sim_t:.1f}/{args.duration:g}s | IMU={sensor.imu_count} | encoders={sensor.encoder_count} | BLE observations={updates} | estimates={len(fusion.rows)}',flush=True)
                last_status=time.monotonic()
            if sensor.start is not None and sim_t>=args.duration:
                complete=sensor.imu_count>0 and fusion.engine is not None and len(fusion.rows)>10
                reason='complete' if complete else 'missing_sensor_data'; break
            if time.monotonic()-started>args.wall_timeout:
                reason='wall_timeout'; break
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if rclpy.ok(): sensor.stopped=True; sensor.drive()
        truth=np.array(evaluation.truth)
        if len(truth): np.savetxt(args.output_dir/'truth.csv',truth,delimiter=',',header='t,x,y,yaw,bias_unused',comments='')
        fusion.save(truth,complete)
        write_json(args.output_dir/'manifest.json',{'schema_version':1,'status':'complete' if complete else 'partial',
            'reason':reason,'scenario':args.scenario,'clock':'Gazebo simulation seconds','native_imu_messages':sensor.imu_count,
            'encoder_messages':sensor.encoder_count,'duration_s':sensor.elapsed(),'gazebo_epoch_s':sensor.start,'latency_policy':'zero modeled transport delay; out-of-order acquisition events dropped; receipt times logged',
            'seed':7,'config':c,'calibration_hash':fingerprint(cal),'python':platform.python_version(),'numpy':np.__version__,
            'source_hash':fingerprint({p.name:p.read_text() for p in sorted((ROOT/'src/localization').glob('*.py'))+[Path(__file__)]}),
            'gyro_bias_truth_available':False})
        executor.shutdown()
        for node in [sensor,fusion,evaluation]: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        print(f'Run {reason}; results saved to {args.output_dir}',flush=True)
    if not complete: raise SystemExit(2)

if __name__=='__main__': main()
