#!/usr/bin/env python3
"""Task043 startup guard: raw readiness -> confirmed initialization -> static R2.
No motor interface or Golden algorithm changes. Metric PASS is not accuracy proof.
"""
import argparse
import hashlib
import json
import math
import os
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024*1024), b''): h.update(chunk)
    return h.hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    os.replace(temporary, path)


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def initial_pose(value, config, boot, now):
    if (value.get('boot_id') != boot or value.get('map_id') != config['map_id']
        or value.get('map_version') != config['map_version']
        or value.get('manifest_sha256') != config['manifest_sha256']
        or value.get('physical_stationary_confirmed') is not True):
        raise ValueError('initial_pose_identity_or_confirmation_missing')
    if not 0 <= now-value.get('confirmed_at', 0) <= 300:
        raise ValueError('initial_pose_confirmation_expired')
    p, q = value['position'], value['quaternion_xyzw']
    if len(p) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in p+q):
        raise ValueError('initial_pose_non_finite')
    if abs(sum(v*v for v in q)-1) > .01: raise ValueError('initial_pose_quaternion_invalid')
    return p, q


class Stream:
    def __init__(self, minimum_hz):
        self.minimum_hz = minimum_hz
        self.samples = deque(maxlen=5000)
        self.last_stamp = None
        self.rejections = {}
        self.last_rejection = None

    def accept(self, now, stamp, valid, invalid_reason="message_invalid"):
        reason = None
        if not valid:
            reason = invalid_reason
        elif not math.isfinite(stamp):
            reason = "timestamp_non_finite"
        elif not 0 <= now-stamp <= .5:
            reason = "timestamp_stale_or_future"
        elif self.last_stamp is not None and stamp <= self.last_stamp:
            reason = "timestamp_not_increasing"
        if reason:
            self.rejections[reason] = self.rejections.get(reason, 0)+1
            self.last_rejection = {"at":now,"reason":reason,
                                   "age_s":now-stamp if math.isfinite(stamp) else None}
            self.samples.clear(); self.last_stamp = None
            return
        if self.samples and now-self.samples[-1][0] > .5:
            self.samples.clear()
            self.last_rejection = {"at":now,"reason":"receive_gap","age_s":0}
        self.samples.append((now, stamp)); self.last_stamp = stamp

    def diagnostics(self, now):
        recent = [s for s in self.samples if now-s[0] <= 3.5]
        duration = recent[-1][0]-recent[0][0] if len(recent)>1 else 0
        rate = (len(recent)-1)/duration if duration>0 else 0
        gap = max((b[1]-a[1] for a,b in zip(recent,recent[1:])),default=0)
        age = now-recent[-1][0] if recent else None
        if not recent:
            reason = (self.last_rejection or {}).get("reason", "no_samples")
        elif age>.5:
            reason = "receive_timeout"
        elif len(recent)<2 or duration<3:
            reason = "continuous_window_incomplete"
        elif rate<self.minimum_hz:
            reason = "frequency_low"
        elif gap>.5:
            reason = "timestamp_gap"
        else:
            reason = "healthy"
        return {"ready":reason=="healthy","reason":reason,"frequency_hz":rate,
                "required_hz":self.minimum_hz,"window_s":duration,"receive_age_s":age,
                "timestamp_gap_max_s":gap,"rejections":dict(self.rejections),
                "last_rejection":dict(self.last_rejection) if self.last_rejection else None}

    def ready(self, now):
        return self.diagnostics(now)["ready"]


def imu_invalid_reason(frame, values, stationary_required):
    """Raw driver uses g; gravity magnitude and quiet gyro are static checks only."""
    if frame.lstrip('/')!='livox_frame': return 'frame_mismatch'
    if not all(math.isfinite(v) for v in values): return 'non_finite_imu'
    if not math.isfinite(sum(v*v for v in values)): return 'non_finite_imu_norm'
    if stationary_required:
        norm=math.sqrt(sum(v*v for v in values[:3]))
        gyro=math.sqrt(sum(v*v for v in values[3:]))
        if not .8<=norm<=1.2: return 'static_gravity_norm'
        if gyro>.08: return 'static_gyro_not_stationary'
    return ''


def r2_metrics(samples):
    """042 limits; caller selects 30 s after the first 30 s warmup."""
    if len(samples)<2: return {'pass':False,'reason':'insufficient_samples'}
    times,xs,ys,yaws = ([s[i] for s in samples] for i in (0,1,2,4))
    unwrapped=[yaws[0]]
    for a,b in zip(yaws,yaws[1:]):
        unwrapped.append(unwrapped[-1]+math.atan2(math.sin(b-a),math.cos(b-a)))
    dt=[b-a for a,b in zip(times,times[1:])]
    steps=[math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(samples,samples[1:])]
    if min(dt)<=0: return {'pass':False,'reason':'timestamp_not_increasing'}
    speeds=sorted(d/t for d,t in zip(steps,dt))
    k=(len(speeds)-1)*.95; lo,hi=math.floor(k),math.ceil(k)
    p95=speeds[lo]+(speeds[hi]-speeds[lo])*(k-lo)
    m={'x_span':max(xs)-min(xs),'y_span':max(ys)-min(ys),
       'endpoint_xy':math.hypot(xs[-1]-xs[0],ys[-1]-ys[0]),
       'yaw_span':max(unwrapped)-min(unwrapped),'speed_p95':p95,
       'step_max':max(steps),'gap_max':max(dt),'duration':times[-1]-times[0]}
    m['pass']=(m['duration']>=29.5 and m['x_span']<=.08 and m['y_span']<=.08
               and m['endpoint_xy']<=.05 and m['yaw_span']<=.04 and p95<=.15
               and m['step_max']<=.03 and m['gap_max']<=.5)
    return m


def retry_allowed(reason, attempt, raw_ready, initialization_valid):
    return (attempt==1 and raw_ready and initialization_valid and reason=='raw_stream_interrupted')


def overlay_launch(source,destination,p,q):
    tree=ET.parse(source); groups=tree.findall('.//group')
    if len(groups)!=1 or groups[0].attrib.get('ns')!='task043_newmap':
        raise ValueError('unexpected_task_launch_ownership')
    group=groups[0]; index=next(i for i,c in enumerate(group) if c.tag=='node')
    for offset,(name,data) in enumerate((('mapping/init_pos',p),('mapping/init_rot',q))):
        item=ET.Element('rosparam',{'param':name}); item.text=json.dumps(data)
        group.insert(index+offset,item)
    tree.write(destination,encoding='utf-8',xml_declaration=True)


class Runtime:
    def __init__(self,config,out):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from livox_ros_driver2.msg import CustomMsg
        from std_msgs.msg import String
        self.ros=rospy; self.config=config; self.out=Path(out)
        self.lock=threading.RLock()
        self.lidar,self.imu=Stream(8),Stream(150)
        self.poses=deque(maxlen=10000); self.pose_fault=''; self.last_odom=0
        self.process=None; self.attempt=0; self.state='WAIT_RAW_INPUT'
        self.reason='initializing'; self.metrics={}; self.input_summary={}; self.bbox=None
        self.fault_diagnostics=None
        self.publisher_snapshot={}
        rospy.init_node('task043_localization_startup',disable_signals=True)
        self.publisher=rospy.Publisher('/task043/localization_startup_status',String,queue_size=1,latch=True)
        rospy.Subscriber('/livox/lidar',CustomMsg,self.on_lidar,queue_size=1)
        rospy.Subscriber('/livox/imu',Imu,self.on_imu,queue_size=100)
        rospy.Subscriber(config['odom_topic'],Odometry,self.on_odom,queue_size=10)
        self.timer=rospy.Timer(rospy.Duration(.2),self.heartbeat)

    def on_lidar(self,msg):
        reason=''
        if msg.header.frame_id.lstrip('/')!='livox_frame': reason='frame_mismatch'
        elif msg.point_num<=0 or msg.point_num!=len(msg.points): reason='point_count_invalid'
        elif not all(math.isfinite(v) for p in msg.points for v in (p.x,p.y,p.z)):
            reason='non_finite_points'
        with self.lock:
            self.lidar.accept(time.time(),msg.header.stamp.to_sec(),not reason,reason)
            self.input_summary['lidar_point_num']=msg.point_num

    def on_imu(self,msg):
        a,g=msg.linear_acceleration,msg.angular_velocity
        values=(a.x,a.y,a.z,g.x,g.y,g.z)
        with self.lock:
            # Static prerequisites end when R2 passes. Real motion acceleration
            # is not evidence of a transport interruption.
            stationary_required=self.state!='READY'
            reason=imu_invalid_reason(msg.header.frame_id,values,stationary_required)
            self.imu.accept(time.time(),msg.header.stamp.to_sec(),not reason,reason)
            self.input_summary['imu_stationary_required']=stationary_required
            if all(math.isfinite(v) for v in values):
                self.input_summary.update(acceleration_unit='g_raw_driver',gravity_xyz=list(values[:3]),
                    acceleration_norm=(math.sqrt(sum(v*v for v in values[:3]))
                        if math.isfinite(sum(v*v for v in values[:3])) else None),
                    gyro_norm=(math.sqrt(sum(v*v for v in values[3:]))
                        if math.isfinite(sum(v*v for v in values[3:])) else None))

    def on_odom(self,msg):
        if self.bbox is None or self.process is None: return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation; stamp=msg.header.stamp.to_sec()
        values=(stamp,p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            if not all(math.isfinite(v) for v in values): self.pose_fault='non_finite_pose'; return
            if (msg.header.frame_id.lstrip('/')!='camera_init' or msg.child_frame_id.lstrip('/')!='body'
                or not .9<=math.sqrt(sum(v*v for v in values[4:]))<=1.1):
                self.pose_fault='frame_or_quaternion_invalid'; return
            age=time.time()-stamp
            if not 0<=age<=.3:
                # Golden can publish one buffered pose immediately after launch.
                # Ignore it until a fresh first pose; the 90s startup timeout
                # still fails closed. Later stale poses remain a hard fault.
                if not self.poses: return
                self.pose_fault="stale_or_future_pose"; return
            if any(not b[0]-1<=v<=b[1]+1 for v,b in zip((p.x,p.y,p.z),
                       (self.bbox['x'],self.bbox['y'],self.bbox['z']))):
                self.pose_fault='pose_outside_map_bounds'; return
            if self.poses:
                last=self.poses[-1]
                if (stamp<=last[0] or math.hypot(p.x-last[1],p.y-last[2])>1
                    or abs(math.atan2(math.sin(yaw-last[4]),math.cos(yaw-last[4])))>math.radians(75)):
                    self.pose_fault='timestamp_reset_or_pose_jump'; return
            self.poses.append((stamp,p.x,p.y,p.z,yaw)); self.last_odom=time.time()

    def heartbeat(self,event=None):
        from std_msgs.msg import String
        with self.lock:
            if self.state=='READY':
                if self.pose_fault:
                    self.state,self.reason='BLOCKED',self.pose_fault
                elif time.time()-self.last_odom>.5:
                    self.state,self.reason='BLOCKED','odometry_timeout'
                elif not self.lidar.ready(time.time()) or not self.imu.ready(time.time()):
                    self.state,self.reason='BLOCKED','raw_stream_interrupted'
            now=time.time()
            streams={"lidar":self.lidar.diagnostics(now),"imu":self.imu.diagnostics(now)}
            if self.state=='BLOCKED' and getattr(self,'fault_diagnostics',None) is None:
                self.fault_diagnostics={"reason":self.reason,"at":now,"streams":streams,
                    "raw_inputs":dict(self.input_summary),"pose_age_s":now-self.last_odom if self.last_odom else None,
                    "publishers":getattr(self,'publisher_snapshot',{})}
                target=self.out/('localization_fault_'+str(time.time_ns())+'.json')
                atomic_json(target,self.fault_diagnostics)
                print('LOCALIZATION_FAULT '+json.dumps(self.fault_diagnostics,allow_nan=False),flush=True)
            status={'state':self.state,'reason':self.reason,'map_id':self.config['map_id'],
                    'map_version':self.config['map_version'],'manifest_sha256':self.config['manifest_sha256'],'frame':'camera_init','heartbeat_at':time.time(),
                    'attempt':self.attempt,'r2':self.metrics,'raw_inputs':dict(self.input_summary),
                    'raw_streams':streams,'fault_diagnostics':getattr(self,'fault_diagnostics',None),
                    'publishers':getattr(self,'publisher_snapshot',{}),
                    'matching_count':'NOT_OBSERVABLE','physical_accuracy':'NOT_VERIFIED','motion_authorized':False}
            self.publisher.publish(String(data=json.dumps(status,allow_nan=False)))
            atomic_json(self.out/'localization_startup_status.json',status)

    def set_state(self,state,reason):
        with self.lock: self.state,self.reason=state,reason
        self.heartbeat()

    def raw_ready(self):
        with self.lock: return self.lidar.ready(time.time()) and self.imu.ready(time.time())

    def unique_publishers(self,odom=False):
        pubs=dict(self.ros.get_master().getSystemState()[2][0])
        expected={'/livox/lidar':['/livox_lidar_publisher2'],'/livox/imu':['/livox_lidar_publisher2']}
        if odom: expected[self.config['odom_topic']]=['/task043_newmap/laserMapping']
        self.publisher_snapshot={topic:{"expected":nodes,"actual":sorted(pubs.get(topic,[]))}
                                 for topic,nodes in expected.items()}
        return all(value["actual"]==value["expected"] for value in self.publisher_snapshot.values())

    def wait_raw(self):
        self.set_state('WAIT_RAW_INPUT','waiting_3s_continuous_stationary_input')
        until=time.monotonic()+30
        while time.monotonic()<until and not self.ros.is_shutdown():
            if self.raw_ready() and self.unique_publishers(): return True
            time.sleep(.2)
        return False

    def verify_identity(self):
        c=self.config
        for field,h in (('manifest','manifest_sha256'),('frozen_config','frozen_config_sha256'),
                        ('executable','executable_sha256')):
            if sha256(c[field])!=c[h]: raise ValueError(field+'_hash_mismatch')
        m=json.loads(Path(c['manifest']).read_text())
        if m['map_id']!=c['map_id'] or m['map_version']!=c['map_version'] or m['map_frame']!='camera_init':
            raise ValueError('map_identity_mismatch')
        for name,h in (('GlobalMap.pcd',m['pcd_sha256']),('GlobalMap_ikdtree.pcd',m['ikdtree_sha256'])):
            if sha256(Path(c['pcd_directory'])/name)!=h: raise ValueError(name+'_hash_mismatch')
        self.bbox=m['map_bbox']

    def stop_golden(self):
        if self.process is not None and self.process.poll() is None:
            for sig,timeout in ((signal.SIGINT,8),(signal.SIGTERM,3),(signal.SIGKILL,3)):
                os.killpg(self.process.pid,sig)
                try: self.process.wait(timeout=timeout); break
                except subprocess.TimeoutExpired: pass
        self.process=None

    def run_attempt(self,confirmation,session):
        # Wait briefly for ROS Master to unregister the previous task-owned publisher.
        # A different publisher is a hard ownership conflict and is never taken over.
        expected_node="/task043_newmap/laserMapping"
        for _ in range(25):
            publishers=dict(self.ros.get_master().getSystemState()[2][0])
            nodes=publishers.get(self.config["odom_topic"],[])
            if not nodes: break
            if nodes != [expected_node]:
                return "odometry_publisher_ownership_conflict"
            time.sleep(.2)
        else:
            return "odometry_publisher_ownership_conflict"

        p,q=initial_pose(confirmation,self.config,boot_id(),time.time())
        launch=session/('initial_pose_%d.launch'%self.attempt)
        overlay_launch(self.config['launch'],launch,p,q)
        log=session/('golden_attempt_%d.log'%self.attempt)
        with self.lock:
            self.poses.clear(); self.pose_fault=''; self.last_odom=0
            self.fault_diagnostics=None
        with log.open('w') as output:
            self.process=subprocess.Popen(['roslaunch','--wait',str(launch)],stdout=output,
                                          stderr=subprocess.STDOUT,start_new_session=True)
        self.set_state('VALIDATING_STATIC','30s_warmup_then_30s_R2_keep_stationary')
        start=time.monotonic(); offset=0; bad_count=0; pending=''
        while not self.ros.is_shutdown():
            if self.state=='BLOCKED': return self.reason
            if (self.out/'initial_pose_confirmation.json').exists():
                return 'operator_reinitialize_requested'
            with log.open(errors='replace') as f:
                f.seek(offset); text=f.read(); offset=f.tell()
            lines=(pending+text).split('\n'); pending=lines.pop()
            for line in lines:
                bad_count+=line.count('No Effective Points!')
                if ('Leaf size is too small' in line or 'VoxelGrid' in line and 'overflow' in line.lower()):
                    return 'voxelgrid_overflow'
            if bad_count>=3: return 'persistent_no_effective_points'
            if self.process.poll() is not None: return 'golden_process_exited'
            if not self.raw_ready(): return 'raw_stream_interrupted'
            if not self.unique_publishers(odom=bool(self.poses)):
                return 'publisher_ownership_conflict'
            with self.lock:
                if self.pose_fault: return self.pose_fault
                samples=list(self.poses)
            if samples and time.time()-self.last_odom>.5: return 'odometry_timeout'
            if self.state!='READY' and samples and samples[-1][0]-samples[0][0]>=60:
                self.metrics=r2_metrics([s for s in samples if s[0]>=samples[0][0]+30])
                atomic_json(session/'static_r2_metrics.json',self.metrics)
                if not self.metrics['pass']: return 'static_R2_failed'
                self.set_state('READY','static_R2_pass_explicit_new_goal_required')
            if self.state!='READY' and time.monotonic()-start>90: return 'static_validation_timeout'
            time.sleep(.2)
        return 'guard_shutdown'

    def run(self):
        self.metrics={}
        session=self.out/('startup_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]); session.mkdir()
        try:
            self.set_state('VERIFYING_IDENTITY','hashing_frozen_assets'); self.verify_identity()
            if not self.wait_raw(): raise ValueError('raw_input_timeout')
            path=self.out/'initial_pose_confirmation.json'
            self.set_state('WAIT_INITIAL_POSE','confirm_map_position_and_heading_no_default_pose')
            while not self.ros.is_shutdown():
                if path.exists():
                    try:
                        confirmation=json.loads(path.read_text())
                        initial_pose(confirmation,self.config,boot_id(),time.time())
                        os.replace(path,session/'initial_pose_confirmation.json'); break
                    except (ValueError,KeyError,TypeError) as exc: self.set_state('WAIT_INITIAL_POSE',str(exc))
                time.sleep(.5)
            else: return
            if not self.wait_raw(): raise ValueError('raw_input_timeout')
            for self.attempt in (1,2):
                reason=self.run_attempt(confirmation,session)
                was_ready=self.state=='READY' or self.metrics.get('pass') is True
                self.set_state('BLOCKED',reason); self.stop_golden()
                if was_ready or reason!='raw_stream_interrupted': break
                valid=True
                try: initial_pose(confirmation,self.config,boot_id(),time.time())
                except ValueError: valid=False
                if not retry_allowed(reason,self.attempt,self.wait_raw(),valid): break
            self.set_state('BLOCKED',reason)
        except Exception as exc: self.set_state('BLOCKED',str(exc))
        finally: self.stop_golden()
        # Keep failure state alive. Docker/systemd cannot blindly retry Golden.
        while not self.ros.is_shutdown():
            path=self.out/'initial_pose_confirmation.json'
            if path.exists():
                try:
                    initial_pose(json.loads(path.read_text()),self.config,boot_id(),time.time())
                    return  # only a fresh explicit confirmation allows another lifecycle
                except (ValueError,KeyError,TypeError): pass
            time.sleep(.5)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True); p.add_argument('--out',required=True)
    p.add_argument('--confirm-initial-pose',action='store_true')
    for name in ('x','y','z','yaw'): p.add_argument('--'+name,type=float)
    p.add_argument('--stationary-confirmed',action='store_true')
    a=p.parse_args(); c=json.loads(Path(a.config).read_text())
    if a.confirm_initial_pose:
        if not a.stationary_confirmed or any(v is None or not math.isfinite(v) for v in (a.x,a.y,a.z,a.yaw)):
            p.error('map-frame x/y/z/yaw and --stationary-confirmed required')
        value={'map_id':c['map_id'],'map_version':c['map_version'],'manifest_sha256':c['manifest_sha256'],
               'boot_id':boot_id(),'confirmed_at':time.time(),'physical_stationary_confirmed':True,
               'position':[a.x,a.y,a.z],'quaternion_xyzw':[0.,0.,math.sin(a.yaw/2),math.cos(a.yaw/2)],
               'source':'explicit_operator_confirmation','motion_authorized':False}
        atomic_json(Path(a.out)/'initial_pose_confirmation.json',value)
        print('Initialization confirmed; safety and old goals are not reset or resumed'); return
    r=Runtime(c,a.out)
    def shutdown(signum,frame):
        r.set_state('BLOCKED','guard_shutdown'); r.ros.signal_shutdown('signal')
    signal.signal(signal.SIGTERM,shutdown); signal.signal(signal.SIGINT,shutdown)
    try:
        while not r.ros.is_shutdown(): r.run()
    finally:
        try: r.set_state('BLOCKED','guard_shutdown')
        finally: r.stop_golden()


if __name__=='__main__': main()
