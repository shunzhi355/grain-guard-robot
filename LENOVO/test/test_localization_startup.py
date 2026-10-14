"""Offline tests of production startup orchestration and adapter admission."""
import importlib.util
import json
import math
import sys
import threading
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import pytest
from grain_sampling_local.localization_health import startup_status_ready

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('task043_startup',ROOT/'LENOVO/scripts/task043_localization_startup.py')
guard=importlib.util.module_from_spec(spec); spec.loader.exec_module(guard)


def config():
    return {'map_id':'test-map','map_version':'test-r1','manifest_sha256':'a'*64,
            'odom_topic':'/task043_newmap/Odometry'}


def confirmation():
    return {**config(),'boot_id':'test-boot','confirmed_at':100,
            'physical_stationary_confirmed':True,'position':[1.,2.,0.],
            'quaternion_xyzw':[0.,0.,0.,1.]}


@pytest.mark.parametrize('change',[{'boot_id':'previous-boot'},{'map_id':'old-map'},
    {'map_version':'old-version'},{'manifest_sha256':'b'*64},
    {'physical_stationary_confirmed':False},{'confirmed_at':-1000},
    {'position':[float('nan'),0,0]},{'quaternion_xyzw':[0,0,0,0]}])
def test_initialization_never_defaults_or_reuses_unknown_pose(change):
    v=confirmation(); v.update(change)
    with pytest.raises(ValueError): guard.initial_pose(v,config(),'test-boot',101)


def test_explicit_position_and_yaw_are_used_once_without_editing_frozen_launch(tmp_path):
    original=tmp_path/'frozen.launch'
    original.write_text('<launch><group ns="task043_newmap"><rosparam command="load" file="frozen.yaml"/><node name="laserMapping"/></group></launch>')
    old=original.read_bytes(); output=tmp_path/'overlay.launch'
    p,q=guard.initial_pose(confirmation(),config(),'test-boot',101)
    guard.overlay_launch(original,output,p,q)
    tree=guard.ET.parse(output); group=tree.find('group')
    assert original.read_bytes()==old
    assert group[1].attrib['param']=='mapping/init_pos'
    assert json.loads(group[1].text)==[1.,2.,0.]
    assert json.loads(group[2].text)==[0.,0.,0.,1.]
    assert group[3].tag=='node'


@pytest.mark.parametrize('hz',[10,200])
def test_raw_readiness_requires_sustained_timing_then_resets_on_reversal(hz):
    stream=guard.Stream(hz*.8)
    for i in range(hz*4): stream.accept(100+i/hz,100+i/hz,True)
    assert stream.ready(104)
    stream.accept(104,99,True)
    assert not stream.ready(104)
    stream.accept(104.1,104.1,False)
    assert not stream.ready(104.1)


def test_frequency_alone_cannot_hide_stale_or_low_rate_raw_stream():
    stream=guard.Stream(150)
    for i in range(50): stream.accept(100+i*.1,100+i*.1,True)
    assert not stream.ready(105)
    stream=guard.Stream(8)
    for i in range(40): stream.accept(100+i*.1,99+i*.1,True)
    assert not stream.ready(104)


def samples():
    return [(100+i*.1,1+math.sin(i)*.003,2+math.cos(i)*.004,0,math.sin(i)*.002)
            for i in range(301)]


def test_static_R2_metrics_accept_bounded_data_without_accuracy_claim():
    result=guard.r2_metrics(samples())
    assert result['pass'] and result['duration']==30


@pytest.mark.parametrize('fault',['drift','jump','yaw','gap','timestamp','short'])
def test_inherited_R2_limits_reject_faults(fault):
    data=samples()
    if fault=='drift': data=[(s[0],s[1]+i*.001,s[2],s[3],s[4]) for i,s in enumerate(data)]
    if fault=='jump': data[150]=(115,1.1,2,0,0)
    if fault=='yaw': data=[(*s[:4],i*.0005) for i,s in enumerate(data)]
    if fault=='gap': data=[(s[0]+(1 if i>150 else 0),*s[1:]) for i,s in enumerate(data)]
    if fault=='timestamp': data[150]=(100,*data[150][1:])
    if fault=='short': data=data[:20]
    assert not guard.r2_metrics(data)['pass']


@pytest.mark.parametrize('reason',['raw_stream_interrupted','pose_outside_map_bounds',
    'persistent_no_effective_points','static_R2_failed','map_identity_mismatch'])
def test_bounded_retry_only_for_transient_input(reason):
    assert guard.retry_allowed(reason,1,True,True)==(reason=='raw_stream_interrupted')
    assert not guard.retry_allowed(reason,2,True,True)
    assert not guard.retry_allowed(reason,1,True,False)


def status():
    return {**config(),'state':'READY','frame':'camera_init','heartbeat_at':100,'r2':{'pass':True}}


@pytest.mark.parametrize('change',[None,{'state':'WAIT_RAW_INPUT'},{'state':'WAIT_INITIAL_POSE'},
    {'state':'BLOCKED'},{'heartbeat_at':90},{'heartbeat_at':102},{'map_id':'old-map'},
    {'map_version':'old-version'},{'manifest_sha256':'b'*64},{'frame':'odom'},{'r2':{'pass':False}},{'r2':'malformed'}])
def test_adapter_rejects_bad_stale_missing_or_cross_map_startup_health(change):
    s=status()
    if change: s.update(change)
    assert startup_status_ready(s,now=100.1,map_id='test-map',map_version='test-r1',
                                manifest_sha256='a'*64)==(change is None)
    assert not startup_status_ready({},now=100.1,map_id='test-map',map_version='test-r1',manifest_sha256='a'*64)


@pytest.mark.parametrize('reason',['pose_outside_map_bounds','static_R2_failed','raw_stream_interrupted'])
def test_production_runtime_loop_never_blindly_restarts_or_resumes_motion(tmp_path,monkeypatch,reason):
    r=guard.Runtime.__new__(guard.Runtime)
    r.out=tmp_path; r.config=config(); r.attempt=0; r.state='WAIT_RAW_INPUT'
    r.ros=SimpleNamespace(is_shutdown=lambda:r.done)
    r.done=False; r.metrics={}; states=[]; calls=[]
    r.set_state=lambda state,why:(setattr(r,'state',state),states.append((state,why)))
    r.verify_identity=lambda:None; r.wait_raw=lambda:True
    r.stop_golden=lambda:calls.append('stop_owned_child')
    def attempt(value,session):
        calls.append(('Golden',r.attempt)); return reason
    r.run_attempt=attempt
    original_sleep=guard.time.sleep
    def sleep(seconds):
        if r.state=='BLOCKED': r.done=True
    monkeypatch.setattr(guard.time,'sleep',sleep)
    monkeypatch.setattr(guard,'boot_id',lambda:'test-boot')
    v=confirmation(); v['confirmed_at']=guard.time.time()
    (tmp_path/'initial_pose_confirmation.json').write_text(json.dumps(v))
    r.run()
    attempts=[c for c in calls if isinstance(c,tuple)]
    assert attempts==([('Golden',1),('Golden',2)] if reason=='raw_stream_interrupted' else [('Golden',1)])
    assert not (tmp_path/'initial_pose_confirmation.json').exists()
    assert len(list(tmp_path.glob('startup_*/initial_pose_confirmation.json')))==1
    assert states[-1]==('BLOCKED',reason)
    assert all(s!='READY' for s,_ in states)


def test_runtime_verifies_real_asset_hashes_and_both_frozen_PCDs(tmp_path):
    c=config()
    m={**config(),'map_frame':'camera_init','map_bbox':{'x':[-2,2],'y':[-2,2],'z':[-1,3]}}
    for name,key in [('GlobalMap.pcd','pcd_sha256'),('GlobalMap_ikdtree.pcd','ikdtree_sha256')]:
        path=tmp_path/name; path.write_bytes(b'TEST_ONLY_PCD_'+name.encode()); m[key]=guard.sha256(path)
    for field in ('manifest','executable','frozen_config'):
        path=tmp_path/field; path.write_text(json.dumps(m) if field=='manifest' else 'TEST_ONLY')
        c[field]=str(path); c[field+'_sha256']=guard.sha256(path)
    c['pcd_directory']=str(tmp_path)
    r=guard.Runtime.__new__(guard.Runtime); r.config=c
    r.verify_identity(); assert r.bbox==m['map_bbox']
    (tmp_path/'GlobalMap_ikdtree.pcd').write_bytes(b'corrupt')
    with pytest.raises(ValueError,match='ikdtree'): r.verify_identity()


def test_guard_and_entrypoint_have_no_motor_interface():
    source=(ROOT/'LENOVO/scripts/task043_localization_startup.py').read_text()
    shell=(ROOT/'LENOVO/scripts/task043_golden_runtime.sh').read_text()
    for text in (source,shell):
        assert 'control.sock' not in text and 'stream_effort' not in text and 'import serial' not in text
    assert 'sleep 3' not in shell
    assert 'wait "${PIDS[2]}"' in shell

@pytest.mark.parametrize('fault',[None,'matching_failure','process_exit','raw_loss','divergence'])
def test_production_Golden_attempt_uses_R2_and_rejects_failures(tmp_path,monkeypatch,fault):
    r=guard.Runtime.__new__(guard.Runtime)
    r.out=tmp_path; r.config=config(); r.attempt=1; r.lock=threading.RLock()
    launch=tmp_path/'base.launch'
    launch.write_text('<launch><group ns="task043_newmap"><rosparam command="load" file="frozen.yaml"/><node name="laserMapping"/></group></launch>')
    r.config['launch']=str(launch); r.state='VALIDATING_STATIC'; r.metrics={}
    r.pose_fault=''; r.poses=deque(); r.process=None; r.last_odom=160.1
    n=[0]
    def shutdown():
        n[0]+=1; return n[0]>1
    r.ros=SimpleNamespace(is_shutdown=shutdown,get_master=lambda:SimpleNamespace(getSystemState=lambda:[1,'ok',[[],[],[]]]))
    r.raw_ready=lambda:fault!='raw_loss'; r.unique_publishers=lambda **kw:True
    r.set_state=lambda state,reason:setattr(r,'state',state)
    command=[]
    def start(args,**kw):
        command.append(args)
        r.poses.extend([(100+i*.1,1,2,0,0) for i in range(602)])
        r.last_odom=160.1
        if fault=='divergence': r.pose_fault='pose_outside_map_bounds'
        if fault=='matching_failure':
            kw['stdout'].write('No Effective Points!\n'*3); kw['stdout'].flush()
        return SimpleNamespace(poll=lambda:0 if fault=='process_exit' else None)
    monkeypatch.setattr(guard.subprocess,'Popen',start)
    monkeypatch.setattr(guard,'boot_id',lambda:'test-boot')
    monkeypatch.setattr(guard,'time',SimpleNamespace(time=lambda:160.1,monotonic=lambda:0,sleep=lambda x:None))
    reason=r.run_attempt(confirmation(),tmp_path)
    expected={None:'guard_shutdown','matching_failure':'persistent_no_effective_points',
              'process_exit':'golden_process_exited','raw_loss':'raw_stream_interrupted',
              'divergence':'pose_outside_map_bounds'}
    assert reason==expected[fault]
    assert command[0][:2]==['roslaunch','--wait']
    assert (r.metrics.get('pass') is True)==(fault is None)
    assert (r.state=='READY')==(fault is None)


def test_real_ROS_adapter_applies_startup_health_to_worker_and_fails_on_loss(tmp_path,monkeypatch):
    """Execute the actual entrypoint with fake ROS and a non-hardware client."""
    from grain_sampling_local import navigation_worker
    from grain_sampling_workflow import robot_bridge
    a_spec=importlib.util.spec_from_file_location('actual_adapter',ROOT/'LENOVO/scripts/local_navigation_ros1.py')
    adapter=importlib.util.module_from_spec(a_spec); a_spec.loader.exec_module(adapter)
    subscriptions={}; snapshots=[]; requests=[]; loops=[0]
    clock=guard.time.time
    class Stamp:
        def __init__(self,t): self.t=t
        def __sub__(self,other): return Stamp(self.t-other.t)
        def to_sec(self): return self.t
    def msg(t):
        return SimpleNamespace(header=SimpleNamespace(stamp=Stamp(t),frame_id='camera_init'),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.,y=2.),
            orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))))
    def shutdown():
        loops[0]+=1
        if loops[0]==1:
            s=status(); s['heartbeat_at']=clock()
            subscriptions['/test/startup'](SimpleNamespace(data=json.dumps(s)))
            for i in range(3): subscriptions['/test/odom'](msg(clock()-.05+i*.01))
        if loops[0]==2:
            subscriptions['/test/startup'](SimpleNamespace(data=json.dumps({**status(),'state':'BLOCKED'})))
        return loops[0]>2
    ros=SimpleNamespace(init_node=lambda *a,**kw:None,
        Subscriber=lambda topic,kind,callback,**kw:subscriptions.update({topic:callback}),
        Publisher=lambda *a,**kw:SimpleNamespace(publish=lambda value:None),
        Time=SimpleNamespace(now=lambda:Stamp(clock())),is_shutdown=shutdown,
        logwarn_throttle=lambda *a:None,logerr=lambda *a:None)
    for name,value in [('rospy',ros),('nav_msgs.msg',SimpleNamespace(Odometry=object)),
                       ('std_msgs.msg',SimpleNamespace(Bool=lambda **kw:kw,String=object))]:
        monkeypatch.setitem(sys.modules,name,value)
    class Worker:
        last_error=''
        def __init__(self,*args): pass
        def tick(self,snapshot): snapshots.append(snapshot)
    monkeypatch.setattr(navigation_worker,'NavigationWorker',Worker)
    monkeypatch.setattr(robot_bridge,'RobotClient',lambda path:SimpleNamespace(request=lambda action:requests.append(action)))
    c={ 'navigation_mode':'local','suction_policy':'external','map_id':'test-map',
        'runtime_dir':str(tmp_path),'odom_topic':'/test/odom','localization_health_mode':'odom_stable',
        'require_pointcloud_for_navigation':False,'localization_startup_status_topic':'/test/startup',
        'localization_startup_map_version':'test-r1','localization_startup_manifest_sha256':'a'*64}
    path=tmp_path/'test-config.json'; path.write_text(json.dumps(c))
    monkeypatch.setattr(sys,'argv',['adapter','--config',str(path),'--allow-point-navigation'])
    adapter.main()
    assert snapshots[0]['localization_valid'] is True
    assert snapshots[1]['localization_valid'] is False
    assert requests==['estop']  # fake endpoint only, no UART or real motion socket


def test_existing_Odometry_publisher_blocks_takeover_without_launch(tmp_path):
    r=guard.Runtime.__new__(guard.Runtime); r.config=config()
    r.ros=SimpleNamespace(get_master=lambda:SimpleNamespace(getSystemState=lambda:
        [1,"ok",[[[config()["odom_topic"],["/other_owner/Golden"]]],[],[]]]))
    assert r.run_attempt(confirmation(),tmp_path)=="odometry_publisher_ownership_conflict"
    assert not list(tmp_path.iterdir())


def imu_runtime(state):
    r=guard.Runtime.__new__(guard.Runtime)
    r.state=state;r.lock=threading.RLock();r.imu=guard.Stream(150)
    r.lidar=guard.Stream(8);r.input_summary={};r.pose_fault=''
    return r


def imu_message(t, accel=(0.,0.,1.), gyro=(0.,0.,0.), frame='livox_frame'):
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,stamp=SimpleNamespace(to_sec=lambda:t)),
        linear_acceleration=SimpleNamespace(x=accel[0],y=accel[1],z=accel[2]),
        angular_velocity=SimpleNamespace(x=gyro[0],y=gyro[1],z=gyro[2]))


def test_actual_imu_callback_keeps_dynamic_acceleration_after_static_pass(monkeypatch,tmp_path):
    r=imu_runtime('READY');now=[100.]
    monkeypatch.setattr(guard.time,'time',lambda:now[0])
    for i in range(801):
        now[0]=100.+i*.005;r.on_imu(imu_message(now[0]))
    assert r.imu.ready(now[0])
    now[0]+=.005
    r.on_imu(imu_message(now[0],accel=(0.,0.,1.3),gyro=(0.,0.,.4)))  # TEST_ONLY motion sample
    assert r.imu.ready(now[0])
    assert r.input_summary['imu_stationary_required'] is False
    assert r.imu.rejections=={}
    for i in range(41):r.lidar.accept(100.+i*.1,100.+i*.1,True)
    r.out=tmp_path;r.config=config();r.metrics={'pass':True};r.attempt=1
    r.last_odom=now[0];r.reason='ready';r.fault_diagnostics=None
    r.publisher=SimpleNamespace(publish=lambda msg:None)
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=lambda **kw:kw))
    r.heartbeat()
    assert r.state=='READY' and not r.fault_diagnostics


@pytest.mark.parametrize('state',['WAIT_RAW_INPUT','WAIT_INITIAL_POSE','VALIDATING_STATIC'])
@pytest.mark.parametrize('accel,gyro,reason',[
    ((0.,0.,1.3),(0.,0.,0.),'static_gravity_norm'),
    ((0.,0.,1.),(0.,0.,.4),'static_gyro_not_stationary')])
def test_static_prerequisites_remain_strict(monkeypatch,state,accel,gyro,reason):
    r=imu_runtime(state)
    monkeypatch.setattr(guard.time,'time',lambda:100.)
    r.on_imu(imu_message(100.,accel,gyro))
    assert not r.imu.ready(100.)
    assert r.imu.last_rejection['reason']==reason
    assert r.input_summary['imu_stationary_required'] is True


@pytest.mark.parametrize('fault,reason',[
    ('nan','non_finite_imu'),('overflow','non_finite_imu_norm'),('frame','frame_mismatch'),
    ('stale','timestamp_stale_or_future'),('reversal','timestamp_not_increasing'),
    ('timeout','receive_timeout')])
def test_dynamic_imu_transport_faults_still_block(monkeypatch,fault,reason):
    r=imu_runtime('READY');now=[100.]
    monkeypatch.setattr(guard.time,'time',lambda:now[0])
    for i in range(801):
        now[0]=100.+i*.005;r.on_imu(imu_message(now[0]))
    assert r.imu.ready(now[0])
    if fault=='timeout':now[0]+=.6
    else:
        now[0]+=.005
        msg=imu_message(now[0])
        if fault=='nan':msg.linear_acceleration.x=float('nan')
        if fault=='overflow':msg.linear_acceleration.x=1e308
        if fault=='frame':msg.header.frame_id='wrong_frame'
        if fault=='stale':msg.header.stamp.to_sec=lambda:now[0]-1
        if fault=='reversal':msg.header.stamp.to_sec=lambda:104.
        r.on_imu(msg)
    d=r.imu.diagnostics(now[0])
    assert not d['ready'] and d['reason']==reason


def test_fault_diagnostics_remain_frozen_after_raw_stream_recovers(tmp_path,monkeypatch):
    r=imu_runtime('READY');r.out=tmp_path;r.config=config();r.attempt=1;r.metrics={'pass':True}
    r.last_odom=104.;r.reason='ready';r.fault_diagnostics=None
    r.publisher=SimpleNamespace(publish=lambda msg:None)
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=lambda **kw:kw))
    monkeypatch.setattr(guard.time,'time',lambda:104.)
    for i in range(801):r.imu.accept(100.+i*.005,100.+i*.005,True)
    for i in range(41):r.lidar.accept(100.+i*.1,100.+i*.1,True)
    r.imu.accept(104.,104.,False,'TEST_ONLY_corrupt_imu')
    r.heartbeat()
    assert r.state=='BLOCKED' and r.reason=='raw_stream_interrupted'
    snapshot=json.loads((tmp_path/'localization_startup_status.json').read_text())['fault_diagnostics']
    assert snapshot['streams']['imu']['reason']=='TEST_ONLY_corrupt_imu'
    assert snapshot['streams']['lidar']['ready']
    for i in range(801):r.imu.accept(100.+i*.005,100.+i*.005,True)
    r.heartbeat()
    assert r.state=='BLOCKED'  # neither sensor recovery nor diagnostics rearm motion
    assert r.fault_diagnostics==snapshot
    assert len(list(tmp_path.glob('localization_fault_*.json')))==1


def test_publisher_diagnostics_identify_the_conflicting_topic():
    r=guard.Runtime.__new__(guard.Runtime);r.config=config()
    pubs=[['/livox/lidar',['/other_receiver']],['/livox/imu',['/livox_lidar_publisher2']]]
    r.ros=SimpleNamespace(get_master=lambda:SimpleNamespace(getSystemState=lambda:[1,'ok',[pubs,[],[]]]))
    assert not r.unique_publishers()
    assert r.publisher_snapshot['/livox/lidar']=={'expected':['/livox_lidar_publisher2'],'actual':['/other_receiver']}
