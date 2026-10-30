import threading
from types import SimpleNamespace
import pytest
from gento_control.planning import Planner


def snapshot():
    pose=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    return {'joints':[[0.]*7,[0.]*7],'matrices':[pose,pose], 'limits':[[[-180,180]]*7]*2,
            'config':{'cartesian_speed':.01,'cartesian_angular_speed':.1,
                      'cartesian_joint_speed':5.,'cartesian_joint_acc':10.,'max_delta':5.,
                      'timeout':60.,'sdk_length_to_m':.001},
            'controller_limits':{'0':{'speed':[5.]*7,'acceleration':[10.]*7},
                                 '1':{'speed':[5.]*7,'acceleration':[10.]*7}}}


def test_control_owns_cartesian_interpolation_and_sequential_ik():
    source=snapshot()
    references=[]
    def rpc(operation,payload):
        assert operation=='ik'
        references.append(payload['starts'][0][0])
        return [[payload['matrices'][0][0][3]]+[0.]*6,[0.]*7]
    planner=Planner(source,rpc,threading.Event())
    goal=[row[:] for row in source['matrices'][0]];goal[0][3]=1.
    plans,_,info=planner.cartesian(0,goal)
    assert list(plans)==[0] and len(plans[0])==51
    assert references==[q[0] for q in plans[0][:-1]]
    assert plans[0][-1][0]==1. and info['planned_duration_s']==1.


def test_planner_cancel_and_joint_limit_rejection():
    cancel=threading.Event()
    planner=Planner(snapshot(),lambda *args:None,cancel)
    cancel.set()
    with pytest.raises(ValueError,match='cancelled'): planner.joint(0,[1.]*7)
    cancel.clear()
    with pytest.raises(ValueError,match='max_delta'):planner.joint(0,[10.]*7)


@pytest.mark.parametrize('max_age,age,valid',[(.2,.5,False),(2.,1.5,True),(2.,2.5,False),(2.,-.1,False)])
def test_target_age_uses_driver_configuration(max_age,age,valid):
    from gento_control.node import Control
    from geometry_msgs.msg import PoseStamped
    now=10_000_000_000
    node=SimpleNamespace(command_max_age=max_age,accept_after=0,
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now)))
    message=PoseStamped()
    stamp=int(now-age*1e9)
    message.header.stamp.sec=stamp//10**9
    message.header.stamp.nanosec=stamp%10**9
    if valid:Control.check_stamp(node,message)
    else:
        with pytest.raises(ValueError,match='Stale/future'):Control.check_stamp(node,message)


@pytest.mark.parametrize('session,task,recovering,cancelled',[
    ('old','current',False,False),('current','old',False,False),
    ('current','current',False,True),('current',None,False,True),
    ('current','current',True,False)])
def test_driver_fault_only_cancels_matching_motion(session,task,recovering,cancelled):
    import json
    from std_msgs.msg import String
    from gento_control.node import Control
    node=SimpleNamespace(lock=threading.Lock(),busy=True,recovering=recovering,
                         session='current',task_id='current',cancel=threading.Event())
    Control.driver_status(node,String(data=json.dumps({'state':'fault','session_id':session,'task_id':task})))
    assert node.cancel.is_set()==cancelled


def test_cartesian_ik_rejection_contains_sample_and_target_position():
    def reject(*args):raise ValueError('arm0 IK failed: -1011; joint2 exceeds limit')
    planner=Planner(snapshot(),reject,threading.Event())
    goal=[row[:] for row in snapshot()['matrices'][0]];goal[0][3]=1.
    with pytest.raises(ValueError,match=r'Cartesian IK point 1/50.*TCP_m=.*arm0 IK failed: -1011'):
        planner.cartesian(0,goal)


def test_body_pose_plans_from_measured_reference_using_driver_ik():
    source=snapshot();source['body']=[0.]*6
    del source['controller_limits']
    source['body_limits']=[[-5.,5.]]*6
    matrix=source['matrices'][0]
    calls=[]
    def rpc(operation,payload):
        calls.append((operation,payload))
        return {'success':True,'joints_deg':[1.,-1.,2.,0.,0.,0.]}
    planner=Planner(source,rpc,threading.Event())
    plans,info=planner.body_pose(matrix)
    assert calls==[('body_ik',{'matrix':matrix,'reference':[0.]*6})]
    assert plans=={3:[[1.,-1.,2.,0.,0.,0.]]}
    assert info['body_target_matrix']==matrix


def test_body_pose_ik_failure_never_returns_a_plan():
    source=snapshot();source['body']=[0.]*6;source['body_limits']=[[-5.,5.]]*6
    planner=Planner(source,lambda *args:{'success':False,'joints_deg':None},threading.Event())
    with pytest.raises(ValueError,match='no feasible'):
        planner.body_pose(source['matrices'][0])
