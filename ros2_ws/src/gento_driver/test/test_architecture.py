"""Tests of the production three-process protocol with fake hardware and real DDS."""
import json
import math
import threading
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import String,Empty
from gento_interfaces.srv import DriverQuery
from gento_interfaces.action import ExecuteTrajectory
from gento_driver.engine import Engine
from gento_driver.node import Bridge
from gento_driver.conversions import JOINT_NAMES,BODY_JOINT_NAMES
from gento_driver.safety import validate_goal
from gento_control.node import Control
from gento_feedback.node import Feedback
from test_bridge import DualRobot,FX_InvKineSolverParams
from gento_driver import sdk_ops as helpers


def until(predicate,timeout=6.):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.01)
    assert predicate(),'Timed out waiting for protocol event'


def future_result(future):
    until(future.done)
    return future.result()


@pytest.fixture
def graph(tmp_path,request):
    rclpy.init(args=['--ros-args','-p','ip:=127.0.0.1','-p',f'enable_control:={str(getattr(request, "param", True)).lower()}',
                    '-p',f'trajectory_log_dir:={tmp_path}'],domain_id=220)
    robot=DualRobot()
    robot.forward_kinematics=lambda arm,q:[[1,0,0,q[0]],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    def ik(arm,p):
        p.m_Output_RetJoint[0]=p.m_Input_IK_TargetTCP[3]
        return p
    robot.inverse_kinematics=ik
    original_sg=robot.get_sg_dict
    robot.ratios=[[5.,5.],[5.,5.]]
    def set_ratio(arm,velocity,acceleration):
        robot.calls.append(('ratio',arm,velocity,acceleration))
        robot.ratios[arm]=[velocity,acceleration]
        return 0
    robot.runtime_set_speed_ratio=set_ratio
    def sg():
        value=original_sg();value['body']={'set':{'vel_ratio':5.,'acc_ratio':5.}}
        for arm in (0,1):
            value['arms'][arm]['set'].update(vel_ratio=robot.ratios[arm][0],acc_ratio=robot.ratios[arm][1])
        return value
    robot.get_sg_dict=sg
    links=[]
    robot.link=lambda *args:links.append(args) or 1
    hardware=Engine(robot,helpers,FX_InvKineSolverParams,SimpleNamespace())
    driver=Bridge(hardware)
    control=Control();feedback=Feedback();client=rclpy.create_node('architecture_client')
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (driver,control,feedback,client):executor.add_node(node)
    worker=threading.Thread(target=executor.spin)
    worker.start()
    value=SimpleNamespace(driver=driver,control=control,feedback=feedback,client=client,
                          hardware=hardware,robot=robot,links=links,path=tmp_path)
    robot.calls.clear()
    try:
        # Tests publish only after their complete RPC/action graph is discovered.
        # A matching target subscription alone does not imply action discovery.
        assert control.rpc_client.wait_for_service(timeout_sec=5.)
        if driver.c.enable_control:assert control.action.wait_for_server(timeout_sec=5.)
        yield value
    finally:
        control.close();driver.close();executor.shutdown();worker.join(timeout=5)
        feedback.close()
        for node in (driver,control,feedback,client):node.destroy_node()
        rclpy.shutdown()


def snapshot(g):
    client=g.client.create_client(DriverQuery,'/gento_driver/query')
    assert client.wait_for_service(timeout_sec=3.)
    result=future_result(client.call_async(DriverQuery.Request(operation='snapshot')))
    assert result.success,result.error
    return json.loads(result.result)


def goal_for(g,s=None):
    s=s or snapshot(g)
    goal=ExecuteTrajectory.Goal(session_id=s['session_id'],task_id=uuid.uuid4().hex)
    goal.start_state=JointState(name=JOINT_NAMES+BODY_JOINT_NAMES,
                               position=[math.radians(v) for v in s['joints'][0]+s['joints'][1]+s['body']])
    goal.start_state.header.stamp=g.client.get_clock().now().to_msg()
    goal.trajectories=[JointTrajectory(joint_names=JOINT_NAMES[:7])]
    goal.controller_ratios=[5.,5.]
    for i in range(5):
        point=JointTrajectoryPoint(positions=[math.radians(v) for v in s['joints'][0]])
        point.time_from_start.nanosec=i*20000000
        goal.trajectories[0].points.append(point)
    return goal


def test_body_action_clamps_unit_roundoff_but_rejects_actual_limit_violation():
    upper=50.222154220533334
    body=[0.,0.,upper-1.,0.,0.,0.]
    state={'session_id':'roundoff','joints':[[0.]*7,[0.]*7],'body':body,
           'controller':{'body':{'set':{'vel_ratio':5.,'acc_ratio':5.}}}}
    hardware=SimpleNamespace(c=SimpleNamespace(command_max_age=1.,max_delta=5.,timeout=60.),
        body_limits=[[-11,11],[-50,50],[-60,upper],[-55,110],[-30,30],[-170,170]],
        read_cartesian_rate_limits=lambda moving:{})
    goal=ExecuteTrajectory.Goal(session_id='roundoff',task_id='test')
    goal.start_state=JointState(name=JOINT_NAMES+BODY_JOINT_NAMES,
                               position=[math.radians(x) for x in [0.]*14+body])
    goal.start_state.header.stamp.sec=2
    point=JointTrajectoryPoint(positions=[0.,0.,math.radians(upper),0.,0.,0.])
    goal.trajectories=[JointTrajectory(joint_names=BODY_JOINT_NAMES,points=[point])]
    goal.controller_ratios=[5.,5.]
    plans,_=validate_goal(goal,state,hardware,2_000_000_000)
    assert plans[3][0][2]==upper
    goal.trajectories[0].points[0].positions[2]=math.radians(upper+1e-6)
    with pytest.raises(ValueError,match='exceeds limits'):
        validate_goal(goal,state,hardware,2_000_000_000)


def send(g,goal,heartbeat=True):
    client=ActionClient(g.client,ExecuteTrajectory,'/gento_driver/execute')
    assert client.wait_for_server(timeout_sec=3.)
    if heartbeat:
        pub=g.client.create_publisher(String,'/gento_driver/heartbeat',1)
        until(lambda:pub.get_subscription_count()>0)
        pub.publish(String(data=json.dumps({'session_id':goal.session_id,'task_id':goal.task_id,'stamp_ns':g.client.get_clock().now().nanoseconds})))
        until(lambda:g.driver.alive(goal))
    handle=future_result(client.send_goal_async(goal))
    return client,handle


def test_end_to_end_planning_execution_and_feedback_record_have_one_link(graph):
    g=graph
    pub=g.client.create_publisher(PoseStamped,'/gento_control/left/target_pose',1)
    until(lambda:pub.get_subscription_count()>0)
    msg=PoseStamped();msg.header.frame_id='left_arm_sdk_base'
    msg.header.stamp=g.client.get_clock().now().to_msg();msg.pose.orientation.w=1.;msg.pose.position.x=.001
    pub.publish(msg)
    until(lambda:any(c[0]=='run' for c in g.robot.calls))
    until(lambda:not g.control.busy)
    until(lambda:bool(list(g.path.glob('*.json'))))
    assert len(g.links)==1
    for node in (g.driver,g.control,g.feedback):
        assert not any(endpoint.topic_name.startswith('/gento_dual_arm/')
                       for endpoint in list(node.publishers)+list(node.subscriptions))
    assert ('upload',0) in g.robot.calls and ('run',1) in g.robot.calls
    record=json.loads(next(g.path.glob('*.json')).read_text())
    assert record['outcome']=='completed'
    assert len(record['plan']['plans']['0'])>=50
    assert record['samples'] and record['samples'][0]['joint_matching']
    # Driver has no position/pose target subscriptions: those belong to control.
    assert not any('target_pose' in s.topic_name or 'target_joints' in s.topic_name for s in g.driver.subscriptions)


def test_stale_session_is_rejected_before_hardware(graph):
    goal=goal_for(graph);goal.session_id='previous-process'
    _,handle=send(graph,goal,heartbeat=False)
    assert not handle.accepted
    assert not graph.robot.calls


@pytest.mark.parametrize('damage',['time','nan','names','start','speed','limit','stamp'])
def test_driver_independently_rejects_invalid_trajectories(graph,damage):
    g=graph;s=snapshot(g);goal=goal_for(g,s)
    if damage=='time':goal.trajectories[0].points[2].time_from_start.nanosec=1
    elif damage=='nan':goal.trajectories[0].points[1].positions[0]=float('nan')
    elif damage=='names':goal.trajectories[0].joint_names[0]='unknown_joint'
    elif damage=='start':goal.start_state.position[0]=1.
    elif damage=='speed':goal.trajectories[0].points[1].positions[0]=math.radians(1.)
    elif damage=='limit':goal.trajectories[0].points[1].positions[0]=math.radians(200.)
    else:goal.start_state.header.stamp.sec-=10
    _,handle=send(g,goal)
    assert handle.accepted
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert not any(c[0] in ('run','upload','arm_command') for c in g.robot.calls)


def test_missing_control_heartbeat_prevents_execution(graph):
    _,handle=send(graph,goal_for(graph),heartbeat=False)
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'heartbeat' in result.message
    assert ('run',1) not in graph.robot.calls


def test_control_loss_during_playback_stops_and_latches(graph):
    g=graph
    original=g.robot.runtime_run_traj
    def running(mask):
        ret=original(mask)
        g.robot.traj_states[0]=3
        return ret
    g.robot.runtime_run_traj=running
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'heartbeat' in result.message
    assert ('stop',11) in g.robot.calls and g.driver.failed


def test_upload_failure_or_stop_during_upload_never_starts(graph):
    g=graph
    def upload(*args):
        g.driver.stop_event.set()
        return 0
    g.robot.config_set_traj=upload
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert ('run',1) not in g.robot.calls
    assert ('stop',11) in g.robot.calls


@pytest.mark.parametrize('graph',[False],indirect=True)
def test_readonly_mode_rejects_rpc_controls_and_has_no_execution_action(graph):
    g=graph
    assert g.driver.action is None and not g.robot.calls
    s=snapshot(g)
    client=g.client.create_client(DriverQuery,'/gento_driver/query')
    assert client.wait_for_service(timeout_sec=2.)
    result=future_result(client.call_async(DriverQuery.Request(session_id=s['session_id'],operation='reset')))
    assert not result.success and 'feedback-only' in result.error
    assert not g.robot.calls
    assert len(g.links)==1


def test_action_cancel_stops_and_reset_invalidates_previous_session(graph):
    g=graph
    original=g.robot.runtime_run_traj
    def run(mask):
        ret=original(mask);g.robot.traj_states[0]=3;return ret
    g.robot.runtime_run_traj=run
    goal=goal_for(g)
    _,handle=send(g,goal)
    until(lambda:('run',1) in g.robot.calls)
    future_result(handle.cancel_goal_async())
    result=future_result(handle.get_result_async()).result
    assert not result.success and ('stop',11) in g.robot.calls
    until(lambda:not g.driver.stop_event.is_set())
    query=g.client.create_client(DriverQuery,'/gento_driver/query')
    assert query.wait_for_service(timeout_sec=2.)
    recovery=future_result(query.call_async(DriverQuery.Request(session_id=goal.session_id,operation='reset')))
    assert recovery.success,recovery.error
    assert recovery.session_id!=goal.session_id
    _,old=send(g,goal,heartbeat=False)
    assert not old.accepted


def test_second_goal_is_rejected_instead_of_queued(graph):
    g=graph
    original=g.robot.runtime_run_traj
    def run(mask):
        ret=original(mask);g.robot.traj_states[0]=3;return ret
    g.robot.runtime_run_traj=run
    first=goal_for(g)
    _,handle=send(g,first)
    until(lambda:('run',1) in g.robot.calls)
    second=goal_for(g)
    _,rejected=send(g,second,heartbeat=False)
    assert not rejected.accepted
    future_result(handle.cancel_goal_async())
    future_result(handle.get_result_async())
    assert g.robot.calls.count(('run',1))==1


def test_driver_path_protection_does_not_depend_on_feedback_stop(graph):
    from unittest.mock import MagicMock
    g=graph
    g.feedback.stop_pub=MagicMock()
    get_rt=g.robot.get_rt_dict
    def telemetry():
        value=get_rt()
        for arm in value['arms']:
            arm['fb']['cmd_pos']=list(arm['fb']['fb_pos'])
            arm['fb']['cmd_pos'][1]=0.
        return value
    g.robot.get_rt_dict=telemetry
    original=g.robot.runtime_run_traj
    def run(mask):
        ret=original(mask);g.robot.traj_states[0]=3;return ret
    g.robot.runtime_run_traj=run
    pub=g.client.create_publisher(PoseStamped,'/gento_control/left/target_pose',1)
    until(lambda:pub.get_subscription_count()>0)
    msg=PoseStamped();msg.header.stamp=g.client.get_clock().now().to_msg()
    msg.header.frame_id='left_arm_sdk_base';msg.pose.orientation.w=1.;msg.pose.position.x=.001
    pub.publish(msg)
    until(lambda:('run',1) in g.robot.calls)
    def bad_fk(arm,q):
        return [[1,0,0,q[0]],[0,1,0,q[1]],[0,0,1,0],[0,0,0,1]]
    g.robot.forward_kinematics=bad_fk
    g.robot.positions[0][1]=6.
    until(lambda:g.driver.failed)
    assert ('stop',11) in g.robot.calls
    until(lambda:g.driver.last_fault is not None)
    assert 'controller_command_line_error=0.000000 m' in g.driver.last_fault
    assert 'max_joint_following_error=6.0000 deg' in g.driver.last_fault
    until(lambda:not g.control.busy)


def test_sdk_access_stays_serial_under_rpc_and_timer_load(graph):
    g=graph
    guard=threading.Lock()
    active=[0];maximum=[0]
    original=g.robot.get_rt_dict
    def read():
        with guard:
            active[0]+=1;maximum[0]=max(maximum[0],active[0])
        try:
            time.sleep(.003)
            return original()
        finally:
            with guard:active[0]-=1
    g.robot.get_rt_dict=read
    query=g.client.create_client(DriverQuery,'/gento_driver/query')
    assert query.wait_for_service(timeout_sec=2.)
    requests=[query.call_async(DriverQuery.Request(operation='snapshot')) for _ in range(6)]
    assert all(future_result(f).success for f in requests)
    assert maximum[0]==1


@pytest.mark.parametrize('failure',['ratio_nan','other_object_fault'])
def test_runtime_invalid_ratio_or_other_object_fault_stops_execution(graph,failure):
    g=graph
    original=g.robot.runtime_run_traj
    def run(mask):
        ret=original(mask);g.robot.traj_states[0]=3;return ret
    g.robot.runtime_run_traj=run
    _,handle=send(g,goal_for(g))
    until(lambda:('run',1) in g.robot.calls)
    if failure=='ratio_nan':
        get=g.robot.get_sg_dict
        def bad():
            value=get();value['arms'][0]['set']['vel_ratio']=float('nan');return value
        g.robot.get_sg_dict=bad
    else:g.robot.states[1]=100
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert ('stop',11) in g.robot.calls


def test_expected_ratio_change_rejects_before_upload(graph):
    g=graph
    goal=goal_for(g)
    goal.controller_ratios=[6.,5.]
    _,handle=send(g,goal)
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'ratios changed' in result.message
    assert ('run',1) not in g.robot.calls and ('upload',0) not in g.robot.calls


def test_body_task_uses_action_and_same_single_connection(graph):
    g=graph
    publisher=g.client.create_publisher(JointState,'/gento_control/body/target_joints',1)
    until(lambda:publisher.get_subscription_count()>0)
    message=JointState(name=BODY_JOINT_NAMES,position=[math.radians(1.)]+[0.]*5)
    message.header.stamp=g.client.get_clock().now().to_msg()
    publisher.publish(message)
    until(lambda:('body_command',3) in g.robot.calls)
    until(lambda:not g.control.busy)
    assert g.robot.body_positions==[1.,0.,0.,0.,0.,0.]
    assert len(g.links)==1


def test_body_pose_topic_uses_real_ik_and_publishes_measured_fk(graph):
    from gento_driver.body_kinematics import LunaBodyKinematics
    from gento_control.node import as_pose
    g=graph
    model=LunaBodyKinematics()
    target=model.forward([0,2,-4,2,0,0])
    feedback=[]
    g.client.create_subscription(PoseStamped,'/gento_feedback/body/tcp_pose',feedback.append,1)
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/body/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0 and bool(feedback))
    assert feedback[-1].header.frame_id=='body_base'
    assert feedback[-1].pose.position.z==pytest.approx(1.1565)
    message=PoseStamped();message.header.frame_id='body_base'
    message.header.stamp=g.client.get_clock().now().to_msg()
    message.pose=as_pose(target,.001)
    publisher.publish(message)
    until(lambda:('body_command',3) in g.robot.calls)
    until(lambda:not g.control.busy)
    measured=model.forward(g.robot.body_positions)
    until(lambda:abs(feedback[-1].pose.position.z-measured[2][3]*.001)<1e-10)
    for i in range(3):assert measured[i][3]==pytest.approx(target[i][3],abs=.1)
    assert len(g.links)==1
    assert not any(c[0] in ('upload','run','arm_command') for c in g.robot.calls)
    until(lambda:bool(list(g.path.glob('*.json'))))
    record=json.loads(next(g.path.glob('*.json')).read_text())
    assert record['outcome']=='completed'
    assert 'body_target_matrix' in record['plan']['plan_info']
    assert 'body_matrix' in record['samples'][-1]


@pytest.mark.parametrize('damage',['frame','quaternion','unreachable','stale'])
def test_body_pose_topic_rejects_invalid_target_without_motion(graph,damage):
    g=graph
    statuses=[]
    g.client.create_subscription(String,'/gento_control/status',lambda m:statuses.append(json.loads(m.data)),1)
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/body/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0)
    message=PoseStamped();message.header.frame_id='body_base'
    message.header.stamp=g.client.get_clock().now().to_msg()
    message.pose.orientation.w=1.;message.pose.position.z=1.1565
    if damage=='frame':message.header.frame_id='world'
    elif damage=='quaternion':message.pose.orientation.w=0.
    elif damage=='unreachable':message.pose.position.x=5.
    else:message.header.stamp.sec-=10
    publisher.publish(message)
    until(lambda:any(s['state']=='rejected' for s in statuses))
    assert not any(c[0] in ('body_command','arm_command','upload','run') for c in g.robot.calls)


@pytest.mark.parametrize('graph',[False],indirect=True)
def test_readonly_body_pose_feedback_uses_geometry_without_limit_reads(graph):
    g=graph
    sample=snapshot(g)
    assert sample['body_matrix'][2][3]==1156.5
    assert sample['body_limits'] is None
    assert not g.robot.calls


def test_driver_applies_body_caps_before_accepting_joint_commands(graph):
    limits=snapshot(graph)['body_limits']
    assert limits[0]==[-11.,11.]
    assert limits[1]==[-50.,50.]
    assert limits[4]==[-30.,30.]
    assert limits[5]==[-170.,170.]


def test_stop_topic_interrupts_body_ik_without_body_command(graph,monkeypatch):
    from gento_driver.body_kinematics import LunaBodyKinematics
    g=graph
    entered=threading.Event()
    def slow_solver(self,*args,checkpoint=None,**kwargs):
        entered.set()
        deadline=time.monotonic()+4.
        while time.monotonic()<deadline:
            checkpoint()
            time.sleep(.005)
        raise AssertionError('Stop failed to interrupt torso solver')
    monkeypatch.setattr(LunaBodyKinematics,'inverse',slow_solver)
    target=g.client.create_publisher(PoseStamped,'/gento_control/body/target_pose',1)
    stop=g.client.create_publisher(Empty,'/gento_driver/stop',1)
    until(lambda:target.get_subscription_count()>0 and stop.get_subscription_count()>=2)
    message=PoseStamped();message.header.frame_id='body_base'
    message.header.stamp=g.client.get_clock().now().to_msg()
    message.pose.position.z=1.1565;message.pose.orientation.w=1.
    target.publish(message);until(entered.is_set)
    stop.publish(Empty())
    until(lambda:not g.control.busy and g.driver.failed)
    assert not any(c[0] in ('body_command','arm_command','upload','run') for c in g.robot.calls)


def test_body_pose_does_not_require_arm_rate_parameters(graph):
    from gento_driver.body_kinematics import LunaBodyKinematics
    from gento_control.node import as_pose
    g=graph
    requested=[]
    def unavailable_arm_limits(moving):
        requested.extend(moving)
        if moving:raise RuntimeError('Arm rate parameters unavailable')
        return {}
    g.hardware.read_cartesian_rate_limits=unavailable_arm_limits
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/body/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0)
    message=PoseStamped();message.header.frame_id='body_base'
    message.header.stamp=g.client.get_clock().now().to_msg()
    message.pose=as_pose(LunaBodyKinematics().forward([0,2,-4,2,0,0]),.001)
    publisher.publish(message)
    until(lambda:('body_command',3) in g.robot.calls)
    until(lambda:not g.control.busy)
    assert not requested and not g.driver.failed


def test_body_completion_checks_tip_pose_as_well_as_joint_angles(graph):
    from gento_driver.body_kinematics import LunaBodyKinematics
    from gento_control.node import as_pose
    g=graph
    commanded=[]
    original=g.robot.runtime_set_joint_pos_cmd
    def offset_feedback(obj,q):
        ret=original(obj,q)
        commanded.append(list(q))
        g.robot.body_positions[1]+=.19
        return ret
    g.robot.runtime_set_joint_pos_cmd=offset_feedback
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/body/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0)
    message=PoseStamped();message.header.frame_id='body_base'
    message.header.stamp=g.client.get_clock().now().to_msg()
    message.pose=as_pose(LunaBodyKinematics().forward([0,2,-4,2,0,0]),.001)
    publisher.publish(message)
    until(lambda:bool(commanded) and g.driver.active is not None)
    time.sleep(.7)
    assert g.control.busy and g.driver.active is not None
    assert g.driver.active['body_pose_error']['position_mm']>1.0
    with g.driver.sdk_lock:g.robot.body_positions=list(commanded[-1])
    until(lambda:not g.control.busy)
    assert not g.driver.failed


def test_stale_or_replayed_heartbeat_does_not_extend_execution_lease(graph):
    g=graph;goal=goal_for(g)
    old={'session_id':goal.session_id,'task_id':goal.task_id,
         'stamp_ns':g.client.get_clock().now().nanoseconds-2000000000}
    g.driver.receive_heartbeat(String(data=json.dumps(old)))
    assert not g.driver.alive(goal)
    fresh={**old,'stamp_ns':g.client.get_clock().now().nanoseconds}
    g.driver.receive_heartbeat(String(data=json.dumps(fresh)))
    saved=g.driver.heartbeat
    time.sleep(.01)
    g.driver.receive_heartbeat(String(data=json.dumps(fresh)))
    assert g.driver.heartbeat==saved


def test_fault_loop_keeps_stop_and_reset_serviceable(graph):
    g=graph
    with g.driver.sdk_lock:
        original=g.driver.snapshot
        def fail():raise RuntimeError('temporary telemetry failure')
        g.driver.snapshot=fail
        try:
            g.driver.tick()
            stops=g.robot.calls.count(('stop',11))
            g.driver.tick()
            assert g.robot.calls.count(('stop',11))==stops
            assert g.driver.failed and not g.driver.timer.is_canceled()
            g.driver.stop_event.set()
            g.driver.tick()
            assert not g.driver.stop_event.is_set()
        finally:g.driver.snapshot=original
    until(lambda:g.driver.latest is not None)
    query=g.client.create_client(DriverQuery,'/gento_driver/query')
    assert query.wait_for_service(timeout_sec=2.)
    response=future_result(query.call_async(DriverQuery.Request(session_id=g.driver.session,operation='reset')))
    assert response.success,response.error
    assert not g.driver.failed and not g.driver.engine.faulted
    assert not g.driver.timer.is_canceled()


def test_fault_snapshot_does_not_require_planning_caps_for_recovery(graph):
    g=graph
    with g.driver.sdk_lock:
        g.driver.emergency_stop();g.driver.failed=True
        original=g.hardware.read_cartesian_rate_limits
        def invalid_caps(*args):raise ValueError('controller caps unavailable before reset')
        g.hardware.read_cartesian_rate_limits=invalid_caps
        try:
            response=g.driver.query(DriverQuery.Request(operation='snapshot'),DriverQuery.Response())
            assert response.success,response.error
            assert not json.loads(response.result)['ready']
        finally:g.hardware.read_cartesian_rate_limits=original


def test_fault_loop_retries_until_emergency_stop_is_confirmed(graph):
    g=graph
    with g.driver.sdk_lock:
        snapshot_method=g.driver.snapshot
        stop_method=g.robot.emergency_stop
        attempts=[]
        def fail_sample():raise RuntimeError('telemetry unavailable')
        def stop(mask):
            attempts.append(mask)
            return 0 if len(attempts)==1 else stop_method(mask)
        g.driver.snapshot=fail_sample
        g.robot.emergency_stop=stop
        try:
            g.driver.tick()
            assert not g.driver.stop_confirmed and not g.driver.timer.is_canceled()
            g.driver.tick()
            assert g.driver.stop_confirmed and len(attempts)==2
            g.driver.tick()
            assert len(attempts)==2
        finally:
            g.driver.snapshot=snapshot_method
            g.robot.emergency_stop=stop_method


def test_reset_longer_than_five_seconds_waits_for_actual_completion(graph):
    g=graph
    entered=threading.Event();finished=threading.Event()
    original=g.hardware.reset_errors
    statuses=[]
    g.client.create_subscription(String,'/gento_control/status',lambda m:statuses.append(json.loads(m.data)),10)
    def slow_reset():
        entered.set();time.sleep(5.2)
        original();finished.set()
    g.hardware.reset_errors=slow_reset
    g.control.reset()
    until(entered.is_set)
    until(lambda:not g.control.busy,timeout=7.)
    assert finished.is_set()
    assert not g.control.recovering and not g.driver.failed
    assert not any(s['state']=='fault' for s in statuses)


def test_recovery_timeout_requests_stop_but_holds_busy_until_reply(graph,monkeypatch):
    import gento_control.node as control_module
    monkeypatch.setattr(control_module,'RECOVERY_TIMEOUT',.1)
    g=graph
    entered=threading.Event();release=threading.Event()
    original=g.hardware.reset_errors
    def blocked_reset():
        entered.set()
        assert release.wait(4.)
        original()
    g.hardware.reset_errors=blocked_reset
    try:
        g.control.reset()
        until(entered.is_set)
        until(g.driver.stop_event.is_set)
        assert g.control.busy and g.control.recovering
        assert g.control.cancel.is_set()
    finally:release.set()
    until(lambda:not g.control.busy)
    until(lambda:not g.driver.stop_event.is_set())
    assert g.driver.failed and not g.control.recovering


def test_validated_playback_uses_full_ratios_and_restores_before_success(graph):
    g=graph
    run=g.robot.runtime_run_traj
    def checked_run(mask):
        assert g.robot.ratios[0]==[100.,100.]
        assert g.robot.ratios[1]==[5.,5.]
        assert g.hardware.cartesian_rate_limits[0]['speed']==[5.]*7
        return run(mask)
    g.robot.runtime_run_traj=checked_run
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert result.success,result.message
    assert g.robot.ratios==[[5.,5.],[5.,5.]]
    calls=g.robot.calls
    assert calls.index(('upload',0))<calls.index(('ratio',0,100.,100.))<calls.index(('run',1))<calls.index(('ratio',0,5.,5.))


def test_stop_during_ratio_change_prevents_playback_and_restores(graph):
    g=graph
    setter=g.robot.runtime_set_speed_ratio
    def stopped(arm,velocity,acceleration):
        ret=setter(arm,velocity,acceleration)
        if velocity==100.:g.driver.stop_event.set()
        return ret
    g.robot.runtime_set_speed_ratio=stopped
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert ('run',1) not in g.robot.calls
    assert g.robot.ratios==[[5.,5.],[5.,5.]]


@pytest.mark.parametrize('failure',['write','ack'])
def test_restoration_failure_never_reports_success(graph,failure):
    g=graph
    setter=g.robot.runtime_set_speed_ratio
    def fail_restore(arm,velocity,acceleration):
        if velocity==5.:return -1 if failure=='write' else 0
        return setter(arm,velocity,acceleration)
    g.robot.runtime_set_speed_ratio=fail_restore
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'restoration failed' in result.message
    assert g.driver.failed and ('stop',11) in g.robot.calls


def test_ratio_must_be_confirmed_before_playback(graph):
    g=graph
    # A queued write without the corresponding SG feedback is insufficient.
    g.robot.runtime_set_speed_ratio=lambda *args:0
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'not confirmed' in result.message
    assert ('run',1) not in g.robot.calls


def test_ratio_change_does_not_allow_new_start_state_drift(graph):
    g=graph
    setter=g.robot.runtime_set_speed_ratio
    def drift(arm,velocity,acceleration):
        ret=setter(arm,velocity,acceleration)
        if velocity==100.:g.robot.positions[0][0]=1.
        return ret
    g.robot.runtime_set_speed_ratio=drift
    _,handle=send(g,goal_for(g))
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'moved since planning' in result.message
    assert ('run',1) not in g.robot.calls
    assert g.robot.ratios[0]==[5.,5.]


def test_hardware_global_ratios_never_raise_configured_planning_limits(graph):
    g=graph
    with g.driver.sdk_lock:
        g.robot.ratios[0]=[100.,100.]
        g.driver.c.cartesian_joint_speed=100.
        g.driver.c.cartesian_joint_acc=100.
        caps=g.hardware.read_cartesian_rate_limits([0])[0]
        assert caps['speed']==[8.1]*7
        assert caps['acceleration']==[8.1]*7
        assert caps['planning_vel_ratio']==5.
        g.robot.ratios[0]=[5.,5.]


def test_partial_ratio_setup_restores_both_arms_without_starting(graph):
    import copy
    g=graph
    goal=goal_for(g)
    right=copy.deepcopy(goal.trajectories[0]);right.joint_names=JOINT_NAMES[7:]
    goal.trajectories.append(right);goal.controller_ratios.extend([5.,5.])
    setter=g.robot.runtime_set_speed_ratio
    def fail_second(arm,velocity,acceleration):
        if arm==1 and velocity==100.:return -1
        return setter(arm,velocity,acceleration)
    g.robot.runtime_set_speed_ratio=fail_second
    _,handle=send(g,goal)
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert g.robot.ratios==[[5.,5.],[5.,5.]]
    assert not any(c[0]=='run' for c in g.robot.calls)


def test_unexpected_playback_ratio_change_stops_and_restores(graph):
    g=graph
    original=g.robot.runtime_run_traj
    def run(mask):
        result=original(mask);g.robot.traj_states[0]=3;return result
    g.robot.runtime_run_traj=run
    _,handle=send(g,goal_for(g))
    until(lambda:('run',1) in g.robot.calls)
    with g.driver.sdk_lock:g.robot.ratios[0][0]=80.
    result=future_result(handle.get_result_async()).result
    assert not result.success and 'ratios changed' in result.message
    assert g.robot.ratios[0]==[5.,5.]
    assert ('stop',11) in g.robot.calls


def test_shutdown_restores_ratios_before_releasing_sdk(graph):
    g=graph
    original=g.robot.runtime_run_traj
    def run(mask):
        result=original(mask);g.robot.traj_states[0]=3;return result
    g.robot.runtime_run_traj=run
    _,handle=send(g,goal_for(g))
    until(lambda:('run',1) in g.robot.calls)
    g.driver.close()
    result=future_result(handle.get_result_async()).result
    assert not result.success
    assert g.robot.ratios[0]==[5.,5.]
    assert g.robot.calls.index(('ratio',0,5.,5.))<g.robot.calls.index('cleanup')


def test_cartesian_recovers_local_ik_limit_before_any_motion_is_uploaded(graph):
    g=graph
    attempts=[]
    def ik(arm,p):
        attempts.append(list(p.m_Input_IK_RefJoint))
        if p.m_Input_IK_RefJoint[1]<=0:
            p.m_Output_JntExdTags[1]=1;p.m_Output_IsJntExd=1
            p.m_Output_RetJoint[1]=181.
            return -1011
        p.m_Output_RetJoint[0]=p.m_Input_IK_TargetTCP[3]
        p.m_Output_RetJoint[1]=p.m_Input_IK_RefJoint[1]
        return p
    g.robot.inverse_kinematics=ik
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/right/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0)
    goal=PoseStamped();goal.header.frame_id='right_arm_sdk_base'
    goal.header.stamp=g.client.get_clock().now().to_msg();goal.pose.orientation.w=1.;goal.pose.position.x=.001
    publisher.publish(goal)
    until(lambda:('run',2) in g.robot.calls)
    until(lambda:not g.control.busy)
    assert len(attempts)>50 and not g.driver.failed
    assert ('upload',1) in g.robot.calls
    assert not any(c[0]=='stop' for c in g.robot.calls)


def test_unresolved_ik_limit_is_reported_with_point_and_never_uploaded(graph):
    from rclpy.qos import QoSProfile,DurabilityPolicy
    g=graph
    def fail(arm,p):
        p.m_Output_IsJntExd=1;p.m_Output_JntExdTags[1]=1
        p.m_Output_RetJoint[1]=181.
        return -1011
    g.robot.inverse_kinematics=fail
    states=[]
    g.client.create_subscription(String,'/gento_control/status',lambda msg:states.append(json.loads(msg.data)),
                                  QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher=g.client.create_publisher(PoseStamped,'/gento_control/right/target_pose',1)
    until(lambda:publisher.get_subscription_count()>0)
    goal=PoseStamped();goal.header.frame_id='right_arm_sdk_base'
    goal.header.stamp=g.client.get_clock().now().to_msg();goal.pose.orientation.w=1.;goal.pose.position.x=.001
    publisher.publish(goal)
    until(lambda:any(s['state']=='rejected' for s in states))
    reason=next(s['reason'] for s in states if s['state']=='rejected')
    assert 'Cartesian IK point 1/' in reason and 'joint2=181.0000' in reason
    until(lambda:not g.control.busy)
    assert not g.robot.calls and not g.driver.failed


def test_only_driver_stop_topic_stops_execution_and_notifies_control(graph):
    g=graph
    assert '/gento_control/stop' not in [sub.topic_name for sub in g.control.subscriptions]
    run=g.robot.runtime_run_traj
    def running(mask):
        result=run(mask);g.robot.traj_states[0]=3;return result
    g.robot.runtime_run_traj=running
    target=g.client.create_publisher(PoseStamped,'/gento_control/left/target_pose',1)
    until(lambda:target.get_subscription_count()>0)
    msg=PoseStamped();msg.header.frame_id='left_arm_sdk_base'
    msg.header.stamp=g.client.get_clock().now().to_msg();msg.pose.orientation.w=1.;msg.pose.position.x=.001
    target.publish(msg)
    until(lambda:('run',1) in g.robot.calls)
    stop=g.client.create_publisher(Empty,'/gento_driver/stop',1)
    until(lambda:stop.get_subscription_count()>0)
    stop.publish(Empty())
    until(lambda:g.driver.failed and not g.control.busy)
    assert g.control.cancel.is_set() and ('stop',11) in g.robot.calls
    assert g.robot.ratios[0]==[5.,5.]


def test_single_stop_topic_cancels_planning_before_blocked_ik_returns(graph):
    g=graph
    entered=threading.Event();release=threading.Event()
    original=g.robot.inverse_kinematics
    def blocked(arm,p):
        entered.set()
        assert release.wait(4.)
        return original(arm,p)
    g.robot.inverse_kinematics=blocked
    target=g.client.create_publisher(PoseStamped,'/gento_control/left/target_pose',1)
    stop=g.client.create_publisher(Empty,'/gento_driver/stop',1)
    until(lambda:target.get_subscription_count()>0 and stop.get_subscription_count()>=2)
    msg=PoseStamped();msg.header.frame_id='left_arm_sdk_base'
    msg.header.stamp=g.client.get_clock().now().to_msg();msg.pose.orientation.w=1.;msg.pose.position.x=.001
    try:
        target.publish(msg);until(entered.is_set)
        stop.publish(Empty())
        until(g.control.cancel.is_set)
        until(lambda:not g.control.busy)
        assert not g.robot.calls
    finally:release.set()
    until(lambda:g.driver.failed)
    assert not any(c[0] in ('run','upload') for c in g.robot.calls)
