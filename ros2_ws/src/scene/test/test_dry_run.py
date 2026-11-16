import json
import importlib.util
import math
import sys
import threading
from array import array
from types import ModuleType, SimpleNamespace

import pytest


def import_scene_node(monkeypatch):
    # These tests call callbacks directly; no ROS graph or MQTT broker is needed.
    if 'paho.mqtt.client' not in sys.modules:
        paho = ModuleType('paho')
        mqtt = ModuleType('paho.mqtt')
        client = ModuleType('paho.mqtt.client')
        paho.mqtt = mqtt
        mqtt.client = client
        monkeypatch.setitem(sys.modules, 'paho', paho)
        monkeypatch.setitem(sys.modules, 'paho.mqtt', mqtt)
        monkeypatch.setitem(sys.modules, 'paho.mqtt.client', client)
    if importlib.util.find_spec('rclpy') is None:
        rclpy = ModuleType('rclpy')
        executors = ModuleType('rclpy.executors')
        node = ModuleType('rclpy.node')
        executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
        node.Node = type('Node', (), {})
        rclpy.executors = executors
        rclpy.node = node
        for name, module in [('rclpy', rclpy), ('rclpy.executors', executors), ('rclpy.node', node)]:
            monkeypatch.setitem(sys.modules, name, module)
        for package, message in [('geometry_msgs', 'PoseStamped'),
                                 ('sensor_msgs', 'JointState'), ('std_msgs', 'String')]:
            parent = ModuleType(package)
            messages = ModuleType(package + '.msg')
            if message == 'JointState':
                class JointState:
                    def __init__(self):
                        self.header = SimpleNamespace(stamp=None)
                        self.name = []
                        self.position = []
                setattr(messages, message, JointState)
            else:
                setattr(messages, message, type(message, (), {}))
            parent.msg = messages
            monkeypatch.setitem(sys.modules, package, parent)
            monkeypatch.setitem(sys.modules, package + '.msg', messages)
        setattr(sys.modules['std_msgs.msg'], 'Int8', type('Int8', (), {}))
    if importlib.util.find_spec('std_srvs') is None:
        parent = ModuleType('std_srvs')
        services = ModuleType('std_srvs.srv')
        services.Trigger = type('Trigger', (), {})
        parent.srv = services
        monkeypatch.setitem(sys.modules, 'std_srvs', parent)
        monkeypatch.setitem(sys.modules, 'std_srvs.srv', services)
    if importlib.util.find_spec('gripper') is None:
        parent = ModuleType('gripper')
        messages = ModuleType('gripper.msg')
        messages.GripperCommand = type('GripperCommand', (), {'MOVE': 0})
        messages.GripperState = type('GripperState', (), {})
        parent.msg = messages
        monkeypatch.setitem(sys.modules, 'gripper', parent)
        monkeypatch.setitem(sys.modules, 'gripper.msg', messages)
    from scene.node import VisionTargets
    return VisionTargets


def test_dry_run_uses_body_base_object_targets_without_body_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    logs = []
    payload = json.dumps({
        'position1': {'x': .1, 'y': .2, 'z': .3},
        'position2': {'x': .2, 'y': .3, 'z': .4},
        'orientation': {'x': 0, 'y': math.sqrt(.5), 'z': 0, 'w': math.sqrt(.5)},
    }).encode()
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state='running', dry_run=True, targets=[],
        tool_tip_offset_z_m=.20,
        get_logger=lambda: SimpleNamespace(info=logs.append),
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: (_ for _ in ()).throw(AssertionError('Motion path called')),
    )
    VisionTargets.tick(scene)
    assert scene.state == 'idle' and scene.targets == [] and scene.queue is None
    assert [state for state, _ in reports] == ['previewed', 'previewed']
    assert set(reports[0][1]) == {'flange_position', 'flange_orientation'}
    assert reports[0][1]['flange_position'] == pytest.approx((-.1, .2, .3))
    assert reports[0][1]['flange_orientation'] == pytest.approx((0., math.sqrt(.5), 0., math.sqrt(.5)))
    assert set(reports[1][1]) == {'flange_position', 'flange_orientation'}
    assert reports[1][1]['flange_position'] == pytest.approx((0., .3, .4))
    assert logs == []


def test_scene_first_target_uses_ptp_second_uses_cartesian(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id='')
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0., y=0., z=0.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=0.))

    monkeypatch.setattr(scene_module, 'PoseStamped', PoseStamped)
    sent, reports = [], []

    class Publisher:
        def __init__(self, mode):
            self.mode = mode

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            sent.append((self.mode, message))

    turned = (0., 0., math.sqrt(.5), math.sqrt(.5))
    scene = SimpleNamespace(
        targets=[('left', (1., 2., 3.), (0., 0., 0., 1.), (1., 2., 3.)),
                 ('left', (4., 5., 6.), (0., 0., 0., 1.), (4., 5., 6.))],
        target_index=0, yaw_candidates=None, yaw_index=0, position1_solution=None,
        allow_tool_z_rotation=False, restore_mode=False,
        target_publishers={'left': Publisher('cartesian'), 'right': Publisher('cartesian')},
        ptp_publishers={'left': Publisher('ptp'), 'right': Publisher('ptp')},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'fresh')),
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.send_next(scene)
    assert sent[0][0] == 'ptp' and reports[-1][1]['motion_mode'] == 'ptp'
    scene.targets.pop(0)
    scene.target_index = 1
    scene.position1_solution = (90., turned)
    scene.yaw_candidates = None
    VisionTargets.send_next(scene)
    assert sent[1][0] == 'cartesian' and reports[-1][1]['motion_mode'] == 'cartesian'
    assert tuple(getattr(sent[1][1].pose.orientation, axis) for axis in 'xyzw') == turned
    assert reports[-1][1]['tool_z_rotation_deg'] == 90.
    assert len(scene.yaw_candidates) == 1  # No alternate rotation at position2.


def test_scene_rejects_two_positions_on_different_arms(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    payload = json.dumps({
        'position1': {'x': .5, 'y': .2, 'z': .6},
        'position2': {'x': .5, 'y': -.2, 'z': .6},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[],
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.tick(scene)
    assert reports[-1][0] == 'rejected'
    assert 'same arm' in reports[-1][1]['reason']


def test_workpiece_pose_tracks_flange_only_while_attached(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id='')
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0., y=0., z=0.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=0.))

    monkeypatch.setattr(scene_module, 'PoseStamped', PoseStamped)
    published, reports, cartesian, joints, body_joints = [], [], [], [], []
    now = [100 * 10**9]
    scene = SimpleNamespace(
        arm_poses={}, grasp_target=('left', 90.), attached_workpiece=None,
        position1_goal=('left', (.4, .2, .5), (0., 0., 0., 1.)),
        left_post_grasp_joints=(2.845654625621635, 0.4907167724907257,
                                -2.826141844584338, -1.5708835932574963,
                                -2.0122948143793824, 0.49986229777117597,
                                -1.035084966187757),
        arm_positions={('pre', 'left'): (1., .8, -2., -1., 0., 0., 0.)},
        post_grasp_targets=[], control_state='completed',
        target_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1, publish=cartesian.append)},
        joint_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1, publish=joints.append)},
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1, publish=body_joints.append),
        state='idle', queue=None, tool_tip_offset_z_m=.2,
        max_arm_pose_age_ns=500_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0], to_msg=lambda: None)),
        current_arm_pose=lambda side: VisionTargets.current_arm_pose(scene, side),
        publish_workpiece_pose=lambda msg: VisionTargets.publish_workpiece_pose(scene, msg),
        send_post_grasp_next=lambda: VisionTargets.send_post_grasp_next(scene),
        workpiece_pose_pub=SimpleNamespace(publish=published.append),
        report=lambda state, **details: reports.append((state, details)),
    )
    flange = PoseStamped()
    flange.header.frame_id = 'body_base'
    flange.header.stamp = SimpleNamespace(sec=100, nanosec=0)
    flange.pose.orientation.z = math.sqrt(.5)
    flange.pose.orientation.w = math.sqrt(.5)
    scene.arm_poses['left'] = flange
    response = SimpleNamespace(success=None, message='')
    VisionTargets.attach_workpiece(scene, None, response)
    assert response.success and scene.attached_workpiece == ('left', 90.)
    assert len(cartesian) == 1 and not joints
    assert cartesian[0].pose.position.x == pytest.approx(.4)
    assert published[-1].pose.position.z == pytest.approx(.2)
    assert published[-1].pose.orientation.w == pytest.approx(1.)

    now[0] = 101 * 10**9
    flange.header.stamp = SimpleNamespace(sec=101, nanosec=0)
    flange.pose.position.x = .1
    flange.pose.position.y = .2
    flange.pose.position.z = .3
    VisionTargets.on_arm_pose(scene, 'left', flange)
    assert len(published) == 2
    assert published[-1].header.stamp == flange.header.stamp
    assert tuple(getattr(published[-1].pose.position, axis) for axis in 'xyz') == pytest.approx((.1, .2, .5))
    assert reports[-1][0] == 'post_grasp_sent'

    scene.last_task_id = 'previous'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'retreat-task'})))
    assert scene.state == 'next' and len(scene.post_grasp_targets) == 3
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(joints) == 1 and list(joints[0].position) == [1., .8, -2., -1., 0., 0., 0.]
    assert joints[0].name == [f'left_joint_{index}' for index in range(1, 8)]
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'pre-task'})))
    assert scene.state == 'next' and len(scene.post_grasp_targets) == 2
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(body_joints) == 1 and list(body_joints[0].position) == [0.] * 6
    assert body_joints[0].name == [f'body_joint_{index}' for index in range(1, 7)]
    assert len(joints) == 1
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-task'})))
    assert scene.state == 'next' and len(scene.post_grasp_targets) == 1
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(joints) == 2
    assert joints[1].name == [f'left_joint_{index}' for index in range(1, 8)]
    assert list(joints[1].position) == pytest.approx(scene.left_post_grasp_joints)
    assert reports[-1] == ('post_grasp_sent', {
        'arm': 'left', 'step': 'transfer', 'motion_mode': 'joints',
        'joints_rad': list(scene.left_post_grasp_joints)})
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'transfer-task'})))
    assert scene.state == 'idle' and reports[-1][0] == 'awaiting_handover'
    assert scene.attached_workpiece == ('left', 90.)

    VisionTargets.detach_workpiece(scene, None, response)
    assert response.success and scene.attached_workpiece is None
    VisionTargets.on_arm_pose(scene, 'left', flange)
    assert len(published) == 2


def test_post_grasp_body_rejection_stops_transfer(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='pre-task',
        waiting_task_id=None, preset_targets=[],
        post_grasp_targets=[('body_zero', 'body_joints', (0.,) * 6, None),
                            ('transfer', 'joints', (1.,) * 7, None)],
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'rejected', 'task_id': 'body-task', 'reason': 'body joint limit'})))
    assert scene.state == 'idle' and scene.post_grasp_targets == []
    assert reports[-1][0] == 'rejected'
    assert 'body_zero' in reports[-1][1]['reason']


def test_arm_position_moves_left_then_right_and_rejects_other_values(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports, sent = [], []

    class Publisher:
        def __init__(self, side):
            self.side = side

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            # ROS stores JointState.position as an array, which JSON cannot serialize.
            message.position = array('d', message.position)
            sent.append((self.side, message))

    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        joint_publishers={side: Publisher(side) for side in ('left', 'right')},
        arm_positions={('init', side): (0.,) * 7 for side in ('left', 'right')}
        | {('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'fresh')),
        report=lambda state, **details: reports.append((state, json.loads(json.dumps(details)))),
        send_preset_next=lambda: VisionTargets.send_preset_next(scene),
        last_task_id='previous', joint_restore_steps=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=2))
    assert not sent and reports[-1][0] == 'rejected'

    VisionTargets.on_arm_position(scene, SimpleNamespace(data=1))
    assert len(sent) == 1 and sent[0][0] == 'left'
    assert list(sent[0][1].position) == [1.] * 7
    assert sent[0][1].header.stamp == 'fresh'
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=0))
    assert len(sent) == 1 and reports[-1][0] == 'rejected'

    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'left-task'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(sent) == 2 and sent[1][0] == 'right'
    assert list(sent[1][1].position) == [2.] * 7
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'right-task'})))
    assert scene.state == 'idle' and scene.preset_targets == []
    assert reports[-1] == ('arm_position_completed', {'position': 'pre'})


def test_arm_position_stops_after_rejected_first_arm(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='left-task', preset_targets=['left', 'right'], preset_name='init',
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'rejected', 'task_id': 'left-task', 'reason': 'limit'})))
    assert scene.state == 'idle' and scene.preset_targets == []
    assert reports[-1][0] == 'rejected'


def test_rejected_ik_retries_yaw_but_never_after_motion(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)

    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state=None, last_task_id='old', waiting_task_id='new',
        targets=[('left', (1., 2., 3.), (0., 0., 0., 1.), (1., 2., 3.))],
        yaw_candidates=[(0., (0., 0., 0., 1.)), (90., (0., 0., .7071, .7071))],
        yaw_index=0, allow_tool_z_rotation=True, saw_motion=False, restore_mode=False,
        joint_restore_steps=None,
        report=lambda state, **details: reports.append((state, details)),
        retryable_ik_rejection=VisionTargets.retryable_ik_rejection,
    )
    reject = SimpleNamespace(data=json.dumps({'state': 'rejected', 'task_id': 'new',
                                               'reason': 'joint5 exceeds limits/max_delta'}))
    VisionTargets.on_control_status(scene, reject)
    assert scene.state == 'next' and scene.yaw_index == 1
    assert reports[-1][0] == 'retrying_orientation'

    scene.state = 'waiting'
    scene.waiting_task_id = 'later'
    scene.saw_motion = False
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({'state': 'running', 'task_id': 'later'})))
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({'state': 'rejected', 'task_id': 'later',
                                                                      'reason': 'arm0 Cartesian IK jump at point 2'})))
    assert scene.state == 'idle' and scene.targets == []
    assert reports[-1][0] == 'rejected'


def test_completed_retry_saves_final_rotation_matrix(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    output = tmp_path / 'last.json'
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='new', targets=[('left', (1., 2., 3.),
                                         (0., 0., 0., 1.), (1., 2., 3.))],
        yaw_candidates=[(0., (0., 0., 0., 1.)),
                        (90., (0., 0., math.sqrt(.5), math.sqrt(.5)))],
        yaw_index=1, last_rotation_file=output,
        tool_z_angle_file=tmp_path / 'tool_z.json', restore_mode=False,
        joint_restore_steps=None,
        save_last_rotation=lambda record: VisionTargets.save_last_rotation(scene, record),
        save_record=VisionTargets.save_record,
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'new'})))
    saved = json.loads(output.read_text())
    for actual, expected in zip(saved['rotation_matrix'],
                                [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]):
        assert actual == pytest.approx(expected, abs=1e-12)
    assert saved['tool_z_rotation_deg'] == 90.
    assert saved['rotation_matrix_type'] == 'tool_z_delta'
    for actual, expected in zip(saved['flange_rotation_matrix'], saved['rotation_matrix']):
        assert actual == pytest.approx(expected, abs=1e-12)
    assert json.loads(scene.tool_z_angle_file.read_text())['tool_z_rotation_deg'] == 90.
    assert reports[-1][0] == 'step_completed'
    assert [list(row) for row in reports[-1][1]['rotation_matrix']] == saved['rotation_matrix']
    assert scene.state == 'idle' and scene.target_index == 1


def test_second_target_waits_configured_interval_after_first_completed(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 100.)
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='first', target_index=0, interval=1.5,
        targets=[('left', (1., 2., 3.), (0., 0., 0., 1.), (1., 2., 3.)),
                 ('left', (4., 5., 6.), (0., 0., 0., 1.), (4., 5., 6.))],
        yaw_candidates=[(0., (0., 0., 0., 1.))], yaw_index=0,
        restore_mode=False, joint_restore_steps=None,
        save_last_rotation=lambda record: None,
        report=lambda state, **details: None,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'first'})))
    assert scene.state == 'next' and scene.target_index == 1
    assert scene.next_at == pytest.approx(101.5)
    assert len(scene.targets) == 1 and scene.targets[0][0] == 'left'
    assert scene.position1_solution == (0., (0., 0., 0., 1.))


def test_completed_position2_exposes_grasp_target(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    turned = (0., 0., math.sqrt(.5), math.sqrt(.5))
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='second', target_index=1, interval=1.,
        targets=[('left', (.5, .2, .4), (0., 0., 0., 1.), (.5, .2, .6))],
        yaw_candidates=[(90., turned)], yaw_index=0,
        restore_mode=False, joint_restore_steps=None,
        tool_z_angle_file=tmp_path / 'tool_z.json',
        save_last_rotation=lambda record: None,
        save_record=lambda path, record: None,
        send_gripper=lambda side, position, step: None,
        report=lambda state, **details: None,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'second'})))
    assert scene.grasp_target == ('left', 90.)
    assert scene.state == 'idle'


def test_handover_waits_for_manual_service_and_uses_right_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    pose = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
        pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=3.),
                             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    right = SimpleNamespace(
        header=pose.header,
        pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=2.8),
                             orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    scene = SimpleNamespace(
        dry_run=False, handover_ready=True, state='idle', control_state='completed',
        attached_workpiece=('left', 0.), tool_tip_offset_z_m=.2,
        right_handover_joints=(.1,) * 7, post_grasp_targets=[],
        current_arm_pose=lambda side: pose if side == 'left' else right,
        send_post_grasp_next=lambda: sent.append('right_pre') or True,
        report=lambda state, **details: reports.append((state, details)),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.start_handover(scene, None, response)
    assert response.success and sent == ['right_pre']
    assert scene.post_grasp_targets[0][0] == 'right_pre'
    assert not scene.handover_ready

    scene.publish_workpiece_pose = sent.append
    VisionTargets.capture_right_attachment(scene)
    assert scene.attached_workpiece[0] == 'right'
    assert scene.workpiece_relative[0] == pytest.approx((0., 0., .4))
    assert reports[-1][0] == 'workpiece_transferred'


def test_right_approach_waits_interval_after_right_pre_completed(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    now = [100.]
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: now[0])
    sent, reports = [], []
    scene = SimpleNamespace(
        state='waiting', waiting_task_id='right-pre-task', last_task_id='previous',
        control_state='running', post_grasp_targets=[('right_pre', 'joints', (0.,) * 7, None)],
        interval=1.5, report=lambda state, **details: reports.append((state, details)),
        send_post_grasp_next=lambda: sent.append('right_approach'),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'right-pre-task'})))
    assert scene.state == 'next' and scene.next_at == pytest.approx(101.5)
    assert scene.post_grasp_targets[0][0] == 'right_approach'
    now[0] = 101.49
    VisionTargets.tick(scene)
    assert sent == []
    now[0] = 101.5
    VisionTargets.tick(scene)
    assert sent == ['right_approach']


def test_grasp_current_limit_hold_waits_for_operator_confirmation(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 10.)
    reports = []
    scene = SimpleNamespace(
        gripper_pending=('left', .027, 'left_grasp', 9.), state='gripper_waiting',
        report=lambda state, **details: reports.append((state, details)),
    )
    feedback = SimpleNamespace(
        valid=True, error_code=0, enabled=True, at_target=False,
        position_m=.02238, target_position_m=.02238, target_active=True,
        current_limit_reached=True, holding_position=True,
    )
    VisionTargets.on_gripper_state(scene, 'left', feedback)
    assert scene.state == 'idle' and scene.gripper_pending is None
    assert reports[0] == ('gripper_completed', {
        'arm': 'left', 'step': 'left_grasp', 'completion': 'current_limit_hold',
        'position_m': .02238})
    assert reports[1][0] == 'awaiting_attach'

    scene.gripper_pending = ('left', 0., 'left_release', 9.)
    scene.state = 'gripper_waiting'
    VisionTargets.on_gripper_state(scene, 'left', feedback)
    assert scene.state == 'gripper_waiting' and scene.gripper_pending is not None


def test_restore_tool_z_uses_current_c_pose_and_saved_b_angle(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text(json.dumps({
        'arm': 'left', 'frame_id': 'body_base', 'tool_z_rotation_deg': 90.,
        'rotation_matrix_type': 'tool_z_delta',
        'rotation_matrix': [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]],
        'flange_position': [1., 2., 3.],
        'flange_orientation': [0., 0., math.sqrt(.5), math.sqrt(.5)],
    }))
    feedback = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=4., y=5., z=6.),
        orientation=SimpleNamespace(x=0., y=0., z=1., w=0.)))
    sent = []
    scene = SimpleNamespace(
        dry_run=False, queue_lock=threading.Lock(), state='idle', queue=None,
        control_state='completed', tool_z_angle_file=angle_file,
        last_rotation_file=tmp_path / 'last.json',
        target_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1)},
        current_arm_pose=lambda side: feedback,
        tool_tip_offset_z_m=.2, targets=[], yaw_candidates=None,
        yaw_index=0, restore_mode=False,
        current_arm_joints=lambda side: (0., 0., 0., 0., 0., .1, 0.),
        joint_restore_steps=None,
        send_next=lambda: sent.append((scene.targets[:], scene.yaw_candidates[:])),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.restore_tool_z(scene, None, response)
    assert response.success
    assert sent[0][0][0][1] == (4., 5., 6.)  # C, not saved B position.
    assert sent[0][0][0][2] == pytest.approx((0., 0., math.sqrt(.5), math.sqrt(.5)))
    assert sent[0][1][0][0] == -90.
    assert scene.restore_mode


def test_restore_tool_z_refuses_dry_run(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    scene = SimpleNamespace(dry_run=True, state='idle', targets=[],
                            yaw_candidates=None, joint_restore_steps=None,
                            restore_mode=False)
    response = SimpleNamespace(success=None, message='')
    VisionTargets.restore_tool_z(scene, None, response)
    assert not response.success and 'dry_run' in response.message


def test_restore_prefers_joint5_when_wrist_six_seven_are_zero(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text(json.dumps({
        'arm': 'left', 'frame_id': 'body_base', 'tool_z_rotation_deg': 127.18532124600992,
        'rotation_matrix_type': 'tool_z_delta',
        'rotation_matrix': [[-0.6043950299627896, -0.7966847856939899, 0.],
                            [0.7966847856939899, -0.6043950299627896, 0.],
                            [0., 0., 1.]],
    }))
    feedback = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=4., y=5., z=6.),
        orientation=SimpleNamespace(x=0., y=0., z=1., w=0.)))
    sent = []
    scene = SimpleNamespace(
        dry_run=False, queue_lock=threading.Lock(), state='idle', queue=None,
        control_state='completed', tool_z_angle_file=angle_file,
        last_rotation_file=tmp_path / 'last.json',
        target_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1)},
        joint_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1)},
        current_arm_pose=lambda side: feedback,
        current_arm_joints=lambda side: (0., 0., 0., 0., 0., 0., 0.),
        tool_tip_offset_z_m=.2, targets=[], yaw_candidates=None,
        yaw_index=0, restore_mode=False,
        send_next=lambda: (_ for _ in ()).throw(AssertionError('Cartesian used')),
        send_joint_restore_step=lambda: sent.append(scene.joint_restore_steps[:]),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.restore_tool_z(scene, None, response)
    assert response.success and 'joint5' in response.message
    assert len(sent[0]) == 2
    assert sent[0][-1] == pytest.approx(-math.radians(127.18532124600992))
    assert scene.targets[0][1] == (4., 5., 6.)


def test_restore_completion_consumes_saved_angle(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text('{"tool_z_rotation_deg": 90}')
    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='new', targets=[('left', (4., 5., 6.),
                                         (0., 0., 1., 0.), (4., 5., 6.2))],
        yaw_candidates=[(-90., (0., 0., math.sqrt(.5), math.sqrt(.5)))],
        yaw_index=0, last_rotation_file=tmp_path / 'last.json',
        tool_z_angle_file=angle_file, restore_mode=True,
        joint_restore_steps=None,
        save_last_rotation=lambda record: VisionTargets.save_last_rotation(scene, record),
        save_record=VisionTargets.save_record,
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'new'})))
    assert reports[-1][0] == 'restore_completed'
    assert not angle_file.exists()
    assert json.loads(scene.last_rotation_file.read_text())['restored_tool_z']


def test_joint5_restore_waits_for_first_step_before_sending_second(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text('{"tool_z_rotation_deg": 90}')
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='first', restore_mode=True,
        joint_restore_steps=[-.8, -1.6], joint_restore_index=0,
        joint_restore_reference=(0., 0., 0., 0., 0., 0., 0.),
        restore_source_record={'tool_z_rotation_deg': 90.},
        tool_z_angle_file=angle_file, save_record=VisionTargets.save_record,
        targets=[('left', (4., 5., 6.), (0., 0., 1., 0.), (4., 5., 6.2))],
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'first'})))
    assert scene.state == 'next' and scene.joint_restore_index == 1
    assert reports[-1][0] == 'restore_joint5_step_completed'
    residual = json.loads(angle_file.read_text())
    assert residual['tool_z_rotation_deg'] == pytest.approx(90. - math.degrees(.8))


def test_joint5_restore_rejection_stops_remaining_steps(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text('{"tool_z_rotation_deg": 127}')
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='first', restore_mode=True, saw_motion=False,
        joint_restore_steps=[-.8, -1.6], joint_restore_index=0,
        targets=[('left', (4., 5., 6.), (0., 0., 1., 0.), (4., 5., 6.2))],
        yaw_candidates=[(-127., (0., 0., 0., 1.))], yaw_index=0,
        allow_tool_z_rotation=True, tool_z_angle_file=angle_file,
        report=lambda state, **details: reports.append((state, details)),
        retryable_ik_rejection=VisionTargets.retryable_ik_rejection,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'rejected', 'task_id': 'first', 'reason': 'joint5 exceeds limits/max_delta'})))
    assert scene.state == 'idle' and scene.joint_restore_steps is None
    assert angle_file.exists() and reports[-1][0] == 'rejected'


def test_interrupted_joint5_restore_requires_joint_review(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    angle_file = tmp_path / 'tool_z.json'
    angle_file.write_text('{"tool_z_rotation_deg": 90}')
    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='first', restore_mode=True, saw_motion=True,
        joint_restore_steps=[-.8, -1.6], joint_restore_index=0,
        targets=[('left', (4., 5., 6.), (0., 0., 1., 0.), (4., 5., 6.2))],
        yaw_candidates=[(-90., (0., 0., 0., 1.))], yaw_index=0,
        allow_tool_z_rotation=True, tool_z_angle_file=angle_file,
        save_record=VisionTargets.save_record,
        report=lambda state, **details: reports.append((state, details)),
        retryable_ik_rejection=VisionTargets.retryable_ik_rejection,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'fault', 'task_id': 'first', 'reason': 'motion interrupted'})))
    assert json.loads(angle_file.read_text())['restore_uncertain']
    assert scene.state == 'idle'


def test_joint5_restore_step_only_changes_fifth_joint(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    published = []
    reports = []
    start = (.1, -.2, .3, -.4, 0., 0., 0.)
    publisher = SimpleNamespace(get_subscription_count=lambda: 1,
                                publish=published.append)
    scene = SimpleNamespace(
        targets=[('left', (4., 5., 6.), (0., 0., 1., 0.), (4., 5., 6.2))],
        current_arm_joints=lambda side: start, joint_restore_reference=start,
        joint_restore_steps=[-.8, -1.6], joint_restore_index=0,
        joint_publishers={'left': publisher},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: 'now')),
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.send_joint_restore_step(scene)
    assert published[0].name == [f'left_joint_{i}' for i in range(1, 8)]
    assert published[0].position == list(start[:4]) + [-.8] + list(start[5:])
    assert scene.state == 'waiting' and reports[-1][0] == 'restore_joint5_sent'
