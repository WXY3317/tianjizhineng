import json
import time
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.executors import SingleThreadedExecutor
from gento_feedback.node import Feedback,segment_error


def raw():
    pose=[[1,0,0,100],[0,1,0,200],[0,0,1,300],[0,0,0,1]]
    return {'session_id':'test','sequence':1,'stamp':{'sec':123,'nanosec':4},
            'joints':[[0.]*7,[0.]*7],'body':[0.]*6,'head':[10.,-5.], 'matrices':[pose,pose],
            'config':{'sdk_length_to_m':.001,'left_frame':'left_arm_sdk_base','right_frame':'right_arm_sdk_base'},
            'objects':[{'object_id':0,'state':100,'error_code':102}],'controller':{},'active_task':None}


def test_feedback_aggregates_raw_state_and_diagnoses_errors():
    rclpy.init(args=[],domain_id=224)
    client=rclpy.create_node('feedback_test')
    node=Feedback()
    executor=SingleThreadedExecutor()
    try:
        pose,full,errors=[],[],[]
        client.create_subscription(PoseStamped,'/gento_feedback/left/tcp_pose',pose.append,1)
        client.create_subscription(JointState,'/gento_feedback/full_joint_states',full.append,1)
        from rclpy.qos import QoSProfile,DurabilityPolicy
        client.create_subscription(String,'/gento_feedback/joint_errors',errors.append,
                                   QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher=client.create_publisher(String,'/gento_driver/raw_state',1)
        executor.add_node(client);executor.add_node(node)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline and (not pose or not full or not errors):
            publisher.publish(String(data=json.dumps(raw())))
            executor.spin_once(timeout_sec=.02)
        assert pose and full and errors
        assert pose[-1].pose.position.x==.1 and pose[-1].header.stamp.sec==123
        assert len(full[-1].position)==22
        assert json.loads(errors[-1].data)['fault_objects']==[0]
    finally:
        executor.shutdown();node.close();node.destroy_node();client.destroy_node();rclpy.shutdown()


def test_feedback_geometry_is_independent_of_driver():
    start=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    end=[r[:] for r in start];end[0][3]=10.
    actual=[r[:] for r in start];actual[0][3]=5.;actual[1][3]=6.
    error,vector=segment_error(actual,start,end,.001)
    assert error==.006 and vector==[0.,.006,0.]


def test_body_pose_uses_measured_matrix_stamp_and_fixed_mm_scale():
    from unittest.mock import MagicMock
    rclpy.init(args=[],domain_id=224)
    node=Feedback()
    try:
        node.output_publishers['body/tcp_pose']=MagicMock()
        source=raw()
        source['body_matrix']=[[0,-1,0,123],[1,0,0,-45],[0,0,1,1100],[0,0,0,1]]
        source['config'].update(body_frame='torso_fixture',sdk_length_to_m=.01)
        node.on_raw(String(data=json.dumps(source)))
        message=node.output_publishers['body/tcp_pose'].publish.call_args[0][0]
        assert message.header.frame_id=='torso_fixture'
        assert (message.header.stamp.sec,message.header.stamp.nanosec)==(123,4)
        assert message.pose.position.x==.123 and message.pose.position.y==-.045
        assert message.pose.position.z==1.1
        assert abs(message.pose.orientation.z-2**-.5)<1e-10
        assert abs(message.pose.orientation.w-2**-.5)<1e-10
    finally:
        node.close();node.destroy_node();rclpy.shutdown()


def test_status_before_plan_is_merged_by_session_and_task(tmp_path):
    rclpy.init(args=[],domain_id=224)
    node=Feedback()
    try:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('trajectory_log_dir',value=str(tmp_path))])
        node.on_status(String(data=json.dumps({'session_id':'old','task_id':'same','state':'completed'})))
        plan={'session_id':'new','task_id':'same','plans':{},'line_objects':[]}
        node.on_plan(String(data=json.dumps(plan)))
        assert node.trace['terminal'] is None
        node.on_status(String(data=json.dumps({'session_id':'new','task_id':'same','state':'completed'})))
        node.on_plan(String(data=json.dumps(plan)))
        assert node.trace['terminal'][1]['state']=='completed'
        node.trace['terminal']=(0,node.trace['terminal'][1])
        node.maintenance()
        assert node.trace is None
        node.writer.shutdown(wait=True)
        records=[json.loads(p.read_text()) for p in tmp_path.glob('*.json')]
        assert any(record['outcome']=='completed' for record in records)
    finally:
        node.close();node.destroy_node();rclpy.shutdown()


def test_running_and_control_terminal_before_plan_are_remembered():
    rclpy.init(args=[],domain_id=224)
    node=Feedback()
    try:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('trajectory_log_dir',value='')])
        status={'session_id':'test','task_id':'running','state':'running'}
        node.on_status(String(data=json.dumps(status)))
        node.on_plan(String(data=json.dumps({'session_id':'test','task_id':'running'})))
        assert node.trace['running'] and node.trace['executing']
        status.update(task_id='rejected',state='rejected',terminal=True)
        node.on_control_status(String(data=json.dumps(status)))
        node.on_plan(String(data=json.dumps({'session_id':'test','task_id':'rejected'})))
        assert node.trace['terminal'][1]['state']=='rejected'
        for index in range(150):
            status['task_id']=str(index)
            node.on_control_status(String(data=json.dumps(status)))
        assert len(node.task_statuses)==128
    finally:
        node.close();node.destroy_node();rclpy.shutdown()


def test_next_plan_preserves_previous_terminal_result(tmp_path):
    rclpy.init(args=[],domain_id=224)
    node=Feedback()
    try:
        from rclpy.parameter import Parameter
        node.set_parameters([Parameter('trajectory_log_dir',value=str(tmp_path))])
        for index,state in enumerate(('completed','fault','stopped')):
            task=f'task-{index}'
            node.on_plan(String(data=json.dumps({'session_id':'test','task_id':task,'plans':{}})))
            node.on_status(String(data=json.dumps({'session_id':'test','task_id':task,'state':state})))
        node.close()
        records={r['plan']['task_id']:r for r in (json.loads(p.read_text()) for p in tmp_path.glob('*.json'))}
        assert [records[f'task-{i}']['outcome'] for i in range(3)]==['completed','fault','stopped']
    finally:
        node.close();node.destroy_node();rclpy.shutdown()


def test_diagnostics_distinguish_command_path_from_measured_path(tmp_path):
    from unittest.mock import MagicMock
    from rclpy.parameter import Parameter
    rclpy.init(args=[],domain_id=224)
    node=Feedback()
    try:
        node.set_parameters([Parameter('trajectory_log_dir',value=str(tmp_path))])
        begin=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        end=[row[:] for row in begin];end[0][3]=10.
        commanded=[row[:] for row in begin];commanded[0][3]=5.
        measured=[row[:] for row in commanded];measured[1][3]=6.
        node.on_plan(String(data=json.dumps({'session_id':'test','task_id':'tracking','starts':[begin,begin],
            'goals':[end,begin],'line_objects':[0],'plans':{'0':[[0.]*7,[1.]*7]}})))
        source=raw();source['matrices'][0]=measured;source['active_task']='tracking'
        source['config']['cartesian_path_tolerance']=.005
        source['tracking']=[{'available':True,'command_joints_deg':[1.]*7,
                             'following_error_deg':[-1.]*7,'command_matrix':commanded}, {'available':False}]
        node.stop_pub=MagicMock()
        node.on_raw(String(data=json.dumps(source)))
        errors=node.trace['samples'][0]['line_errors']['0']
        assert errors['distance_m']==.006
        assert errors['command_distance_m']==0.
        assert errors['tcp_following_error_m']==.006
        node.stop_pub.publish.assert_called_once()
        node.close()
        record=json.loads(next(tmp_path.glob('*.json')).read_text())
        matching=record['samples'][0]['joint_matching']['0']
        assert matching['nearest_index']==0 and matching['command_nearest_index']==1
    finally:
        node.close();node.destroy_node();rclpy.shutdown()
