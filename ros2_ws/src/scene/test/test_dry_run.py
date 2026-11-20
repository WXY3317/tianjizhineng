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
    if importlib.util.find_spec('ament_index_python') is None:
        ament = ModuleType('ament_index_python')
        packages = ModuleType('ament_index_python.packages')
        packages.get_package_share_directory = lambda _: '/tmp/scene'
        ament.packages = packages
        monkeypatch.setitem(sys.modules, 'ament_index_python', ament)
        monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    if importlib.util.find_spec('builtin_interfaces') is None:
        builtin = ModuleType('builtin_interfaces')
        messages = ModuleType('builtin_interfaces.msg')
        messages.Time = type('Time', (), {'__init__': lambda self: setattr(self, 'sec', 0)})
        builtin.msg = messages
        monkeypatch.setitem(sys.modules, 'builtin_interfaces', builtin)
        monkeypatch.setitem(sys.modules, 'builtin_interfaces.msg', messages)
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
            elif message == 'PoseStamped':
                class PoseStamped:
                    def __init__(self):
                        self.header = SimpleNamespace(stamp=None, frame_id='')
                        self.pose = SimpleNamespace(
                            position=SimpleNamespace(x=0., y=0., z=0.),
                            orientation=SimpleNamespace(x=0., y=0., z=0., w=0.))
                setattr(messages, message, PoseStamped)
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
        messages.GripperCommand = type('GripperCommand', (), {'MOVE': 0, 'ENABLE': 2,
                                                              'position_m': 0.})
        messages.GripperState = type('GripperState', (), {})
        parent.msg = messages
        monkeypatch.setitem(sys.modules, 'gripper', parent)
        monkeypatch.setitem(sys.modules, 'gripper.msg', messages)
    from scene.node import VisionTargets
    return VisionTargets


@pytest.mark.parametrize('state', ['idle', 'awaiting_second_target'])
def test_unsupported_mqtt_label_does_not_block_next_valid_target(monkeypatch, state):
    VisionTargets = import_scene_node(monkeypatch)
    scene = SimpleNamespace(
        state=state, queue_lock=threading.Lock(), queue=None,
        restore_mode=False, attached_workpiece=None,
    )
    def payload(label):
        return json.dumps({
            'label': label,
            'position1': {'x': .7, 'y': -.2, 'z': .4},
            'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
        }).encode()
    ignored = payload('有遮挡背面')
    valid = payload('无遮挡正面')
    VisionTargets.on_message(scene, None, None, SimpleNamespace(retain=False, payload=ignored))
    assert scene.queue is None
    VisionTargets.on_message(scene, None, None,
                             SimpleNamespace(retain=False, payload=payload('其他测试标签')))
    assert scene.queue is None
    VisionTargets.on_message(scene, None, None, SimpleNamespace(retain=False, payload=valid))
    assert scene.queue == valid


def test_dry_run_uses_body_base_object_targets_without_body_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    logs = []
    payload = json.dumps({
        'label': '无遮挡背面',
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
    assert [state for state, _ in reports] == ['previewed', 'previewed', 'body_shift_previewed']
    assert reports[-1][1]['shift_x_m'] == -.20
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
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time())),
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


def test_scene_uses_position1_arm_for_both_positions(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports, sent = [], []
    payload = json.dumps({
        'label': '无遮挡正面',
        'position1': {'x': .7, 'y': .2, 'z': .6},
        'position2': {'x': .7, 'y': -.2, 'z': .6},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: sent.append(tuple(scene.targets)),
    )
    VisionTargets.tick(scene)
    assert [side for side, *_ in sent[0]] == ['left', 'left']


@pytest.mark.parametrize('y,side,j6', [(0., 'left', -.5), (.05, 'left', -.5), (.06, 'left', -.5), (.10, 'left', -.5),
                                      (-.01, 'right', .5), (-.05, 'right', .5),
                                      (-.05707584081460777, 'right', .5), (-.06, 'right', .5), (-.10, 'right', .5)])
def test_center_target_moves_body_before_arm(monkeypatch, y, side, j6):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    payload = json.dumps({
        'label': '无遮挡正面',
        'position1': {'x': .7, 'y': y, 'z': .6},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        right_post_grasp_joints=None, left_handover_joints=None,
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                             publish=sent.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: sent.append('arm target'),
        body_pre_grasp_joints=VisionTargets.body_pre_grasp_joints,
        preset_targets=[], joint_restore_steps=None, post_grasp_targets=[],
        last_task_id=None,
    )
    scene.send_body_pre_grasp = lambda arm: VisionTargets.send_body_pre_grasp(scene, arm)
    VisionTargets.tick(scene)
    assert len(sent) == 1 and sent[0].position == pytest.approx(
        [0., .86725, -1.50561, -.16162, 0., j6])
    assert scene.body_pre_grasp_pending and scene.state == 'waiting'
    assert scene.targets[0][0] == side
    VisionTargets.on_control_status(scene, SimpleNamespace(
        data=json.dumps({'state': 'completed', 'task_id': 'body-1'})))
    assert scene.state == 'next' and not scene.body_pre_grasp_pending
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert sent[-1] == 'arm target'


def test_second_right_nonnegative_y_rotates_body_before_x_shift(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    payload = json.dumps({
        'label': '无遮挡背面',
        'position1': {'x': .5831911014961281, 'y': .05707584081460777, 'z': .43825331037198145},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        state='awaiting_second_target', front_grasp_arms=['left'], back_first_arm=None,
        queue_lock=threading.Lock(), queue=payload, control_state='completed',
        dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        attached_workpiece=('left', 0.), restore_mode=False,
        right_post_grasp_joints=None, left_handover_joints=None,
        joint_publishers={'right': publisher},
        body_joint_publisher=publisher, body_pose_publisher=publisher,
        body_pose_feedback=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: None,
        send_next=lambda: sent.append('arm target'),
        body_pre_grasp_joints=VisionTargets.body_pre_grasp_joints,
        preset_targets=[], joint_restore_steps=None, post_grasp_targets=[],
        last_task_id=None,
    )
    scene.send_body_pre_grasp = lambda arm: VisionTargets.send_body_pre_grasp(scene, arm)
    scene.second_arm_prepare_joints = VisionTargets.second_arm_prepare_joints
    scene.send_second_arm_prepare = lambda arm: VisionTargets.send_second_arm_prepare(scene, arm)
    scene.current_body_joints = lambda: (0., .86725, -1.50561, -.16162, 0., .2)
    scene.send_second_body_turn = lambda arm: VisionTargets.send_second_body_turn(scene, arm)
    scene.send_body_shift = lambda: VisionTargets.send_body_shift(scene)
    pose = SimpleNamespace(header=SimpleNamespace(frame_id='body_base'), pose=SimpleNamespace(
        position=SimpleNamespace(x=-.05, y=0., z=.6),
        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    VisionTargets.on_body_pose(scene, pose)
    VisionTargets.tick(scene)
    assert scene.targets[0][0] == 'right'
    assert sent[0].position == pytest.approx([-1.5, .8, 2., -1., 0., 0., 0.])
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'right-arm-prepare'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert sent[1].position[-1] == pytest.approx(1.57)
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-turn'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert sent[2].pose.position.x == pytest.approx(-.25)
    assert scene.body_shift_pending


def test_second_right_outside_center_shifts_x_without_turning_j6(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    payload = json.dumps({
        'label': '无遮挡正面',
        'position1': {'x': .58, 'y': -.11, 'z': .44},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='awaiting_second_target', front_grasp_arms=['left'],
        back_first_arm=None, queue_lock=threading.Lock(), queue=payload,
        control_state='completed', dry_run=False, targets=[],
        tool_tip_offset_z_m=.2, attached_workpiece=('left', 0.),
        report=lambda state, **details: None,
        send_body_shift=lambda: sent.append('x_shift'),
        send_body_pre_grasp=lambda side: (_ for _ in ()).throw(
            AssertionError('Second grasp must not use the small-angle J6 preset')),
        send_second_body_turn=lambda side, target_j6_rad=None: (_ for _ in ()).throw(
            AssertionError('Y outside the center range must not trigger a J6 turn')),
    )
    VisionTargets.tick(scene)
    assert sent == ['x_shift'] and scene.targets[0][0] == 'right'
    assert scene.body_shift_m == pytest.approx(-.20)


@pytest.mark.parametrize('y,side', [(.100001, 'left'), (-.100001, 'right')])
def test_body_turn_range_excludes_values_outside_ten_cm(monkeypatch, y, side):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    payload = json.dumps({
        'label': '无遮挡正面',
        'position1': {'x': .7, 'y': y, 'z': .6},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        report=lambda state, **details: None,
        send_next=lambda: sent.append(scene.targets[0][0]),
    )
    VisionTargets.tick(scene)
    assert sent == [side]


@pytest.mark.parametrize('x,shift', [(.60, -.20), (.70, 0.), (.75, .20), (.85, .20)])
def test_x_target_shifts_body_before_arm(monkeypatch, x, shift):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    payload = json.dumps({
        'label': '无遮挡正面',
        'position1': {'x': x, 'y': .11, 'z': .6},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        state='idle', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        right_post_grasp_joints=None, left_handover_joints=None,
        body_pose_publisher=publisher, body_pose_feedback=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: sent.append('arm target'),
        preset_targets=[], joint_restore_steps=None, post_grasp_targets=[],
        last_task_id=None,
    )
    pose = SimpleNamespace(header=SimpleNamespace(frame_id='body_base'), pose=SimpleNamespace(
        position=SimpleNamespace(x=.3, y=.02, z=.6),
        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    VisionTargets.on_body_pose(scene, pose)
    scene.send_body_shift = lambda: VisionTargets.send_body_shift(scene)
    VisionTargets.tick(scene)
    if not shift:
        assert sent == ['arm target']
        return
    assert len(sent) == 1 and sent[0].pose.position.x == pytest.approx(.3 + shift)
    assert sent[0].pose.position.y == .02 and sent[0].pose.position.z == .6
    assert scene.body_shift_pending and scene.state == 'waiting'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-shift'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert sent[-1] == 'arm target'


def test_center_target_body_rejection_stops_before_arm(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    scene = SimpleNamespace(
        state='waiting', body_pre_grasp_pending=True, waiting_task_id=None,
        last_task_id=None, targets=[('left', (), (), ())], saw_motion=False,
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: sent.append('arm target'),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(
        data=json.dumps({'state': 'rejected', 'task_id': 'body-1', 'reason': 'limit'})))
    assert scene.state == 'idle' and scene.targets == [] and sent == []
    assert reports[-1][0] == 'rejected'


def test_back_right_requires_handover_joints_only_for_same_arm_second_target(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    payload = json.dumps({
        'label': '无遮挡背面',
        'position1': {'x': .5, 'y': -.2, 'z': .6},
        'position2': {'x': .5, 'y': -.2, 'z': .5},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='awaiting_second_target', queue_lock=threading.Lock(), queue=payload,
        control_state=None, dry_run=False, targets=[], back_first_arm='right',
        front_grasp_arms=[], right_post_grasp_joints=None, left_handover_joints=None,
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.tick(scene)
    assert scene.state == 'awaiting_second_target'
    assert reports[-1][0] == 'rejected'
    assert 'right_post_grasp_joints' in reports[-1][1]['reason']


def test_back_right_queues_reverse_handover_after_attach(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        grasp_target=('right', 30.), position1_goal=('right', (.1, -.2, .3), (0., 0., 0., 1.)),
        vision_label='无遮挡背面', state='idle', queue=None, gripper_pending=None,
        attached_workpiece=None, control_state='completed',
        target_publishers={'right': SimpleNamespace(get_subscription_count=lambda: 1)},
        joint_publishers={side: SimpleNamespace(get_subscription_count=lambda: 1)
                          for side in ('left', 'right')},
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1),
        current_arm_pose=lambda side: SimpleNamespace(),
        publish_workpiece_pose=lambda pose: None,
        arm_positions={('pre', 'right'): (1.,) * 7,
                       ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))},
        right_post_grasp_joints=(2.,) * 7,
        send_post_grasp_next=lambda: True,
        report=lambda state, **details: reports.append((state, details)),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.attach_workpiece(scene, None, response)
    assert response.success
    assert [step[0] for step in scene.post_grasp_targets] == [
        'right_retreat', 'right_j1_clearance', 'body_j4_clearance',
        'right_grasp_pre', 'body_photo_restore']
    assert scene.back_first_arm == 'right'


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
    published, reports, cartesian, joints, body_joints, body_poses = [], [], [], [], [], []
    now = [100 * 10**9]
    scene = SimpleNamespace(
        arm_poses={}, grasp_target=('left', 90.), attached_workpiece=None,
        position1_goal=('left', (.4, .2, .5), (0., 0., 0., 1.)),
        right_handover_joints=(.1,) * 7,
        left_post_grasp_joints=(2.845654625621635, 0.4907167724907257,
                                -2.826141844584338, -1.5708835932574963,
                                -2.0122948143793824, 0.49986229777117597,
                                -1.035084966187757),
        arm_positions={('pre', 'left'): (1., .8, -2., -1., 0., 0., 0.),
                       ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))},
        post_grasp_targets=[], control_state='completed',
        target_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1, publish=cartesian.append)},
        joint_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1, publish=joints.append),
                          'right': SimpleNamespace(get_subscription_count=lambda: 1)},
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1, publish=body_joints.append),
        body_pose_ptp_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                                publish=body_poses.append),
        state='idle', queue=None, tool_tip_offset_z_m=.2,
        max_arm_pose_age_ns=500_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0], to_msg=lambda: None)),
        current_arm_pose=lambda side: VisionTargets.current_arm_pose(scene, side),
        current_arm_joints=lambda side: (0., .2, .3, .4, .5, .6, .7),
        current_body_joints=lambda: (0., 0., 0., -.2, 0., 0.),
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
    assert scene.state == 'next' and [step[0] for step in scene.post_grasp_targets] == [
        'left_j1_clearance', 'body_j4_clearance', 'pre', 'body_photo_restore']
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(joints) == 1 and joints[0].position == pytest.approx([1.5, .2, .3, .4, .5, .6, .7])
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'j1-task'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(body_joints) == 1 and body_joints[0].position[3] == pytest.approx(.1)
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'j4-task'})))
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(joints) == 2 and list(joints[1].position) == [1., .8, -2., -1., 0., 0., 0.]
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'pre-task'})))
    assert scene.state == 'next' and not body_poses
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(body_poses) == 1 and body_poses[0].pose.position.x == pytest.approx(-.05)
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-photo-task'})))
    assert scene.state == 'awaiting_second_target'
    assert reports[-1] == ('awaiting_second_target', {'required_arm': 'left'})
    assert len(body_joints) == 1
    VisionTargets.detach_workpiece(scene, None, response)
    assert not response.success and scene.attached_workpiece == ('left', 90.)


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


def test_arm_position_moves_both_arms_in_one_task_and_rejects_other_values(monkeypatch):
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
        dual_joint_publisher=Publisher('both'),
        arm_positions={0: 'init', 1: 'pre', 9: 'home'}
        | {('init', side): (0.,) * 7 for side in ('left', 'right')}
        | {('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7}
        | {('home', 'left'): (3.,) * 7, ('home', 'right'): (4.,) * 7},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time())),
        report=lambda state, **details: reports.append((state, json.loads(json.dumps(details)))),
        send_preset_next=lambda: VisionTargets.send_preset_next(scene),
        last_task_id='previous', joint_restore_steps=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=2))
    assert not sent and reports[-1][0] == 'rejected'

    VisionTargets.on_arm_position(scene, SimpleNamespace(data=1))
    assert len(sent) == 1 and sent[0][0] == 'both'
    assert list(sent[0][1].position) == [1.] * 7 + [2.] * 7
    assert len(sent[0][1].name) == 14
    assert sent[0][1].header.stamp.sec == 0
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=0))
    assert len(sent) == 1 and reports[-1][0] == 'rejected'

    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'dual-task'})))
    assert scene.state == 'idle' and scene.preset_targets == []
    assert reports[-1] == ('arm_position_completed', {'position': 'pre'})

    VisionTargets.on_arm_position(scene, SimpleNamespace(data=9))
    assert len(sent) == 2 and sent[-1][0] == 'both'
    assert list(sent[-1][1].position) == [3.] * 7 + [4.] * 7


def test_start_topic_runs_pre_then_body_photo_and_keeps_individual_presets(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        start_sequence_stage=None, dual_joint_publisher=publisher,
        body_pose_ptp_publisher=publisher,
        arm_positions={1: 'pre', 2: 'body_photo',
                       ('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7,
                       ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))},
        current_body_joints=lambda: (0.,) * 6,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        send_preset_next=lambda: VisionTargets.send_preset_next(scene),
        last_task_id=None, waiting_task_id=None, post_grasp_targets=[], joint_restore_steps=None,
    )
    VisionTargets.on_start(scene, SimpleNamespace(data='start'))
    assert len(sent) == 1 and sent[0].position == [1.] * 7 + [2.] * 7
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'pre-task'})))
    assert scene.state == 'next' and scene.start_sequence_stage == 'body_photo'
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(sent) == 2 and sent[1].pose.position.x == pytest.approx(-.05)
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'photo-task'})))
    assert scene.state == 'idle' and scene.start_sequence_stage is None
    assert reports[-1][0] == 'start_completed'
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=1))
    assert len(sent) == 3 and scene.state == 'waiting'


def test_start_topic_stops_after_pre_rejection(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        start_sequence_stage=None, dual_joint_publisher=publisher,
        arm_positions={1: 'pre', ('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        send_preset_next=lambda: VisionTargets.send_preset_next(scene),
        last_task_id=None, waiting_task_id=None,
    )
    VisionTargets.on_start(scene, SimpleNamespace(data='start'))
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'rejected', 'task_id': 'pre-task', 'reason': 'joint limit'})))
    assert len(sent) == 1 and scene.start_sequence_stage is None and scene.state == 'idle'
    assert reports[-1][0] == 'rejected'


def test_start_topic_does_not_claim_an_existing_preset(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='waiting', preset_name='pre', start_sequence_stage=None,
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_start(scene, SimpleNamespace(data='start'))
    assert scene.start_sequence_stage is None
    assert reports[-1][0] == 'rejected'


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


def test_post_grasp_pre_pair_uses_one_dual_arm_task(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    scene = SimpleNamespace(
        post_grasp_targets=[('left_grasp_pre', 'joints', (1.,) * 7, None),
                            ('right_grasp_pre', 'joints', (2.,) * 7, None),
                            ('body_zero', 'body_joints', (0.,) * 6, None)],
        dual_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                             publish=sent.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        state='idle', control_state='completed', preset_targets=[], last_task_id=None,
        attached_workpiece=('left', 0.),
    )
    assert VisionTargets.send_post_grasp_next(scene)
    assert len(sent) == 1 and list(sent[0].position) == [1.] * 7 + [2.] * 7
    assert scene.post_grasp_dual_pending and scene.state == 'waiting'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'dual-pre-task'})))
    assert [step[0] for step in scene.post_grasp_targets] == ['body_zero']
    assert scene.state == 'next' and not scene.post_grasp_dual_pending


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


def test_descent_ik_limit_retries_from_approach_with_new_tool_angle(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    original = (0., 0., 0., 1.)
    scene = SimpleNamespace(
        state='waiting', control_state='completed', last_task_id='approach',
        waiting_task_id='descent', saw_motion=False, restore_mode=False,
        target_index=1, descent_retry_count=0, allow_tool_z_rotation=True,
        position1_goal=('right', (.5, -.1, .6), original),
        position1_solution=(0., original), position1_tip=(.5, -.1, .4),
        targets=[('right', (.5, -.1, .5), original, (.5, -.1, .3))],
        yaw_candidates=[(0., original)], yaw_index=0, joint_restore_steps=None,
        target_publishers={'right': publisher}, ptp_publishers={'right': publisher},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        retryable_ik_rejection=VisionTargets.retryable_ik_rejection,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'rejected', 'task_id': 'descent',
        'reason': 'arm1 IK failed: -1011; joint5=-173.3455 deg, limits=[-173,173]'})))
    assert scene.state == 'next' and scene.descent_retry_count == 1
    assert scene.target_index == 0 and len(scene.targets) == 2
    assert scene.yaw_candidates[scene.yaw_index][0] == 15.
    VisionTargets.send_next(scene)
    assert len(sent) == 1 and reports[-1][1]['motion_mode'] == 'ptp'
    assert reports[-1][1]['tool_z_rotation_deg'] == 15.


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
        enable_grippers_before_grasp=lambda side, step: None,
        report=lambda state, **details: None,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'second'})))
    assert scene.grasp_target == ('left', 90.)
    assert scene.state == 'idle'


def test_front_right_position2_commands_right_gripper(monkeypatch, tmp_path):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id='old',
        waiting_task_id='second', target_index=1, interval=1.,
        targets=[('right', (.5, -.2, .4), (0., 0., 0., 1.), (.5, -.2, .6))],
        yaw_candidates=[(15., (0., 0., 0., 1.))], yaw_index=0,
        restore_mode=False, joint_restore_steps=None,
        tool_z_angle_file=tmp_path / 'tool_z.json', vision_label='无遮挡正面',
        save_last_rotation=lambda record: None,
        save_record=lambda path, record: None,
        enable_grippers_before_grasp=lambda side, step: sent.append((side, step)),
        report=lambda state, **details: None,
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'second'})))
    assert scene.grasp_target == ('right', 15.)
    assert sent == [('right', 'right_direct_grasp')]


def test_both_grippers_enable_before_grasp_move(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    now = [10.]
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: now[0])
    sent, reports = [], []
    publisher = lambda side: SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=lambda message: sent.append((side, message.action, message.position_m)))
    scene = SimpleNamespace(
        gripper_publishers={side: publisher(side) for side in ('left', 'right')},
        gripper_enable_pending=None, gripper_enable_queue=[],
        gripper_move_after_enable=None, gripper_pending=None, state='idle',
        report=lambda state, **details: reports.append((state, details)),
    )
    scene.send_next_gripper_enable = lambda: VisionTargets.send_next_gripper_enable(scene)
    scene.send_gripper = lambda side, position, step: VisionTargets.send_gripper(
        scene, side, position, step)
    VisionTargets.enable_grippers_before_grasp(scene, 'right', 'right_direct_grasp')
    assert sent == [('left', 2, 0.)]
    enabled = SimpleNamespace(valid=True, error_code=0, enabled=True)
    now[0] += .2
    VisionTargets.on_gripper_state(scene, 'left', enabled)
    assert sent == [('left', 2, 0.), ('right', 2, 0.)]
    now[0] += .2
    VisionTargets.on_gripper_state(scene, 'right', enabled)
    assert sent == [('left', 2, 0.), ('right', 2, 0.), ('right', 0, .027)]
    assert scene.gripper_pending[:3] == ('right', .027, 'right_direct_grasp')


def test_gripper_enable_fault_prevents_grasp_move(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    now = [10.]
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: now[0])
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        gripper_publishers={'left': publisher, 'right': publisher},
        gripper_enable_pending=None, gripper_enable_queue=[],
        gripper_move_after_enable=None, gripper_pending=None, state='idle',
        report=lambda state, **details: reports.append((state, details)),
    )
    scene.send_next_gripper_enable = lambda: VisionTargets.send_next_gripper_enable(scene)
    VisionTargets.enable_grippers_before_grasp(scene, 'left', 'left_grasp')
    now[0] += .2
    VisionTargets.on_gripper_state(scene, 'left', SimpleNamespace(
        valid=False, error_code=1, enabled=False))
    assert len(sent) == 1 and scene.state == 'fault'
    assert scene.gripper_move_after_enable is None and reports[-1][0] == 'rejected'


def test_handover_uses_right_feedback(monkeypatch):
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
        attached_workpiece=('left', 0.), tool_tip_offset_z_m=.2,
        current_arm_pose=lambda side: pose if side == 'left' else right,
        report=lambda state, **details: reports.append((state, details)),
        publish_workpiece_pose=sent.append,
    )
    VisionTargets.capture_right_attachment(scene)
    assert scene.attached_workpiece[0] == 'right'
    assert scene.workpiece_relative[0] == pytest.approx((0., 0., .4))
    assert reports[-1][0] == 'workpiece_transferred'


def test_reverse_handover_moves_left_toward_right_and_tracks_left_flange(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports, published, sent = [], [], []
    def pose(y):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0)),
            pose=SimpleNamespace(position=SimpleNamespace(x=.5, y=y, z=.3),
                                 orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
    scene = SimpleNamespace(
        attached_workpiece=('right', 0.), tool_tip_offset_z_m=.2,
        current_arm_pose=lambda side: pose(-.1 if side == 'right' else .1),
        report=lambda state, **details: reports.append((state, details)),
        publish_workpiece_pose=published.append,
    )
    VisionTargets.capture_attachment(scene, 'left')
    assert scene.attached_workpiece == ('left', 0.)
    assert scene.workpiece_relative[0] == pytest.approx((0., -.2, .2))
    assert reports[-1] == ('workpiece_transferred', {'arm': 'left'})

    scene.post_grasp_targets = [('left_approach', 'cartesian', None, None)]
    scene.handover_y_offset_m = .065
    scene.left_handover_y_offset_m = .063
    scene.target_publishers = {'left': SimpleNamespace(
        get_subscription_count=lambda: 1, publish=sent.append)}
    scene.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    VisionTargets.send_post_grasp_next(scene)
    assert sent[-1].pose.position.y == pytest.approx(.037)
    scene.post_grasp_targets = [('right_approach', 'cartesian', None, None)]
    scene.target_publishers['right'] = SimpleNamespace(
        get_subscription_count=lambda: 1, publish=sent.append)
    VisionTargets.send_post_grasp_next(scene)
    assert sent[-1].pose.position.y == pytest.approx(-.035)


def test_reverse_handover_waits_for_left_pre_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id=None,
        waiting_task_id='right-transfer-task', preset_targets=[],
        post_grasp_targets=[('right_transfer', 'joints', (1.,) * 7, None)],
        left_handover_joints=(2.,) * 7,
        attached_workpiece=('right', 0.), interval=1.,
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'right-transfer-task'})))
    assert scene.post_grasp_targets == [('left_pre', 'joints', (2.,) * 7, None)]
    assert scene.attached_workpiece[0] == 'right'
    scene.state = 'waiting'
    scene.waiting_task_id = 'left-pre-task'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'left-pre-task'})))
    assert [step[0] for step in scene.post_grasp_targets] == [
        'left_approach', 'left_handover_grasp']
    assert reports[-1][0] == 'handover_waiting'


def test_right_retreat_then_both_arms_return_pre_and_home(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 100.)
    reports = []
    right_pre = (.25, -.45, .3, -1., 0., 0., 0.)
    scene = SimpleNamespace(
        gripper_pending=('right', .027, 'right_grasp', 90.), state='gripper_waiting',
        post_grasp_targets=[], right_handover_joints=(.1,) * 7,
        arm_positions={('pre', 'right'): right_pre, ('pre', 'left'): (1.,) * 7,
                       ('home', 'left'): (2.,) * 7, ('home', 'right'): (3.,) * 7},
        attached_workpiece=('left', 0.),
        capture_right_attachment=lambda: setattr(scene, 'attached_workpiece', ('right', 0.)),
        report=lambda state, **details: reports.append((state, details)),
    )
    feedback = SimpleNamespace(
        valid=True, error_code=0, enabled=True, at_target=True,
        position_m=.027, target_position_m=.027, target_active=True,
        current_limit_reached=False, holding_position=False,
    )
    VisionTargets.on_gripper_state(scene, 'right', feedback)
    assert [step[0] for step in scene.post_grasp_targets] == [
        'left_release', 'right_handover_retreat']
    assert scene.post_grasp_targets[-1][2] == scene.right_handover_joints

    scene.post_grasp_targets = scene.post_grasp_targets[1:]
    scene.state = 'waiting'
    scene.waiting_task_id = 'retreat-task'
    scene.last_task_id = None
    scene.control_state = 'running'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'retreat-task'})))
    assert scene.state == 'next' and [step[0] for step in scene.post_grasp_targets] == [
        'left_return_pre', 'right_return_pre', 'left_home', 'right_home',
        'left_final_close', 'right_final_close']
    assert not any(state == 'post_grasp_completed' for state, _ in reports)


def test_front_right_grasp_returns_both_arms_pre_then_body_and_home(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 100.)
    reports = []
    scene = SimpleNamespace(
        gripper_pending=('right', .027, 'right_direct_grasp', 90.),
        state='gripper_waiting', attached_workpiece=None, front_grasp_arms=[],
        report=lambda state, **details: reports.append((state, details)),
    )
    scene.grasp_target = ('right', 30.)
    scene.position1_goal = ('right', (.1, -.2, .3), (0., 0., 0., 1.))
    scene.vision_label = '无遮挡正面'
    scene.queue = None
    scene.control_state = 'completed'
    scene.target_publishers = {'right': SimpleNamespace(get_subscription_count=lambda: 1)}
    scene.joint_publishers = {side: SimpleNamespace(get_subscription_count=lambda: 1)
                              for side in ('left', 'right')}
    scene.body_joint_publisher = SimpleNamespace(get_subscription_count=lambda: 1)
    scene.current_arm_pose = lambda side: SimpleNamespace()
    scene.publish_workpiece_pose = lambda pose: None
    scene.arm_positions = {('pre', 'right'): (1.,) * 7, ('pre', 'left'): (2.,) * 7,
                           ('home', 'left'): (3.,) * 7, ('home', 'right'): (4.,) * 7,
                           ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))}
    scene.send_post_grasp_next = lambda: True
    scene.attach_workpiece = lambda request, response: VisionTargets.attach_workpiece(scene, request, response)
    feedback = SimpleNamespace(
        valid=True, error_code=0, enabled=True, at_target=False,
        position_m=.022, target_position_m=.022, target_active=True,
        current_limit_reached=True, holding_position=True,
    )
    VisionTargets.on_gripper_state(scene, 'right', feedback)
    assert scene.attached_workpiece == ('right', 30.)
    assert any(state == 'workpiece_attached' for state, _ in reports)
    assert [step[0] for step in scene.post_grasp_targets] == [
        'right_retreat', 'right_j1_clearance', 'body_j4_clearance',
        'right_grasp_pre', 'left_grasp_pre', 'body_photo_restore']
    assert scene.front_grasp_arms == ['right']

    scene.state = 'waiting'
    scene.post_grasp_targets = [scene.post_grasp_targets[-1]]
    scene.waiting_task_id = 'right-home-task'
    scene.last_task_id = None
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'right-home-task'})))
    assert scene.state == 'awaiting_second_target' and scene.post_grasp_targets == []
    assert reports[-1] == ('awaiting_second_target', {'required_arm': 'left'})


@pytest.mark.parametrize('second_label', ['无遮挡正面', '无遮挡背面'])
def test_second_front_grasp_queues_body_home_and_both_gripper_closes(monkeypatch, second_label):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        front_grasp_arms=['right'], attached_workpiece=('right', 30.),
        grasp_target=('left', 0.), position1_goal=('left', (.4, .2, .5), (0., 0., 0., 1.)),
        vision_label=second_label, state='idle', queue=None, gripper_pending=None,
        control_state='completed',
        target_publishers={'left': SimpleNamespace(get_subscription_count=lambda: 1)},
        joint_publishers={side: SimpleNamespace(get_subscription_count=lambda: 1)
                          for side in ('left', 'right')},
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1),
        current_arm_pose=lambda side: SimpleNamespace(),
        publish_workpiece_pose=lambda pose: None,
        arm_positions={('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7,
                       ('home', 'left'): (3.,) * 7, ('home', 'right'): (4.,) * 7},
        send_post_grasp_next=lambda: True,
        report=lambda state, **details: reports.append((state, details)),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.attach_workpiece(scene, None, response)
    assert response.success and scene.front_grasp_arms == ['right', 'left']
    assert [step[0] for step in scene.post_grasp_targets] == [
        'left_retreat', 'left_j1_clearance', 'body_j4_clearance', 'left_grasp_pre', 'right_grasp_pre', 'body_zero',
        'left_home', 'right_home', 'left_final_close', 'right_final_close']


@pytest.mark.parametrize('used_turn,used_shift', [(True, False), (False, True), (False, False)])
def test_body_photo_restores_after_turn_or_x_shift(monkeypatch, used_turn, used_shift):
    VisionTargets = import_scene_node(monkeypatch)
    reports, sent = [], []
    xyz = (-.05, 0., .6)
    xyzw = (0., .38941332986322064, 0., .9210631132147452)
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        front_grasp_arms=[], body_pre_grasp_used=used_turn,
        body_shift_used=used_shift, attached_workpiece=None,
        grasp_target=('left', 0.), position1_goal=('left', (.4, .2, .5), (0., 0., 0., 1.)),
        vision_label='无遮挡正面', state='idle', queue=None, gripper_pending=None,
        control_state='completed',
        target_publishers={'left': publisher},
        joint_publishers={side: publisher for side in ('left', 'right')},
        body_joint_publisher=publisher, body_pose_ptp_publisher=publisher,
        current_arm_pose=lambda side: SimpleNamespace(),
        publish_workpiece_pose=lambda pose: None,
        arm_positions={('pre', 'left'): (1.,) * 7, ('pre', 'right'): (2.,) * 7,
                       ('body_photo', 'body_pose_ptp'): (xyz, xyzw)},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        send_post_grasp_next=lambda: True,
        report=lambda state, **details: reports.append((state, details)),
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.attach_workpiece(scene, None, response)
    assert response.success
    assert [step[0] for step in scene.post_grasp_targets] == [
        'left_retreat', 'left_j1_clearance', 'body_j4_clearance',
        'left_grasp_pre', 'right_grasp_pre', 'body_photo_restore']
    scene.post_grasp_targets = [scene.post_grasp_targets[-1]]
    VisionTargets.send_post_grasp_next(scene)
    assert sent[-1].header.frame_id == 'body_base'
    assert sent[-1].pose.position.x == -.05
    assert sent[-1].pose.orientation.y == xyzw[1]
    assert reports[-1][1]['motion_mode'] == 'body_pose_ptp'


@pytest.mark.parametrize('side,target_j1', [('left', 1.5), ('right', -1.5)])
def test_post_grasp_j1_clearance_preserves_other_arm_joints(monkeypatch, side, target_j1):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        state='idle', post_grasp_targets=[(f'{side}_j1_clearance', 'joints', None, None)],
        joint_publishers={side: publisher},
        current_arm_joints=lambda arm: (.4, .2, .3, .4, .5, .6, .7),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
    )
    assert VisionTargets.send_post_grasp_next(scene)
    assert sent[0].name == [f'{side}_joint_{index}' for index in range(1, 8)]
    assert sent[0].position == pytest.approx([target_j1, .2, .3, .4, .5, .6, .7])


def test_post_grasp_j1_clearance_requires_fresh_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    scene = SimpleNamespace(
        state='idle', post_grasp_targets=[('right_j1_clearance', 'joints', None, None)],
        joint_publishers={'right': SimpleNamespace(get_subscription_count=lambda: 1,
                                                   publish=sent.append)},
        current_arm_joints=lambda arm: None,
        report=lambda state, **details: reports.append((state, details)),
    )
    assert not VisionTargets.send_post_grasp_next(scene)
    assert scene.state == 'fault' and not scene.post_grasp_targets and not sent


def test_body_j4_clearance_preserves_other_joints_before_photo(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        state='idle', post_grasp_targets=[
            ('body_j4_clearance', 'body_joints', None, None),
            ('body_photo_restore', 'body_pose_ptp', (-.05, 0., .6), (0., 0., 0., 1.))],
        body_joint_publisher=publisher, body_pose_ptp_publisher=publisher,
        current_body_joints=lambda: (.1, .2, .3, -.4, .5, .6),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        last_task_id=None,
    )
    scene.send_post_grasp_next = lambda: VisionTargets.send_post_grasp_next(scene)
    assert scene.send_post_grasp_next()
    assert sent[0].name == [f'body_joint_{index}' for index in range(1, 7)]
    assert sent[0].position == pytest.approx([.1, .2, .3, .1, .5, .6])
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-j4'})))
    assert scene.state == 'next' and [step[0] for step in scene.post_grasp_targets] == ['body_photo_restore']
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert len(sent) == 2 and sent[1].pose.position.x == pytest.approx(-.05)
    assert reports[-1][1]['step'] == 'body_photo_restore'


def test_body_j4_clearance_requires_fresh_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    scene = SimpleNamespace(
        state='idle', post_grasp_targets=[
            ('body_j4_clearance', 'body_joints', None, None),
            ('body_photo_restore', 'body_pose_ptp', (-.05, 0., .6), (0., 0., 0., 1.))],
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                             publish=sent.append),
        current_body_joints=lambda: None,
        report=lambda state, **details: reports.append((state, details)),
    )
    assert not VisionTargets.send_post_grasp_next(scene)
    assert scene.state == 'fault' and scene.post_grasp_targets == [] and not sent
    assert reports[-1][0] == 'rejected'


def test_back_grasp_restores_body_photo_before_body_zero(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    publisher = SimpleNamespace(get_subscription_count=lambda: 1)
    scene = SimpleNamespace(
        front_grasp_arms=[], body_pre_grasp_used=True, attached_workpiece=None,
        grasp_target=('left', 0.), position1_goal=('left', (.4, .2, .5), (0., 0., 0., 1.)),
        vision_label='无遮挡背面', state='idle', queue=None, gripper_pending=None,
        control_state='completed', target_publishers={'left': publisher},
        joint_publishers={side: publisher for side in ('left', 'right')},
        body_joint_publisher=publisher,
        current_arm_pose=lambda side: SimpleNamespace(), publish_workpiece_pose=lambda pose: None,
        arm_positions={('pre', 'left'): (1.,) * 7,
                       ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))},
        left_post_grasp_joints=(2.,) * 7,
        send_post_grasp_next=lambda: True, report=lambda state, **details: None,
    )
    response = SimpleNamespace(success=None, message='')
    VisionTargets.attach_workpiece(scene, None, response)
    assert response.success
    assert [step[0] for step in scene.post_grasp_targets] == [
        'retreat', 'left_j1_clearance', 'body_j4_clearance', 'pre', 'body_photo_restore']


@pytest.mark.parametrize('first_arm', ['left', 'right'])
def test_back_second_target_always_handover_before_grasp(monkeypatch, first_arm):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='awaiting_second_target', back_first_arm=first_arm,
        back_second_payload=None, front_grasp_arms=[],
        queue_lock=threading.Lock(), queue=None, control_state='completed',
        dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        attached_workpiece=(first_arm, 0.), restore_mode=False,
        report=lambda state, **details: reports.append((state, details)),
        preset_targets=[], joint_restore_steps=None, post_grasp_targets=[],
        left_post_grasp_joints=(1.,) * 7, right_post_grasp_joints=(2.,) * 7,
        left_handover_joints=(7.,) * 7,
        arm_positions={('pre', 'left'): (3.,) * 7, ('pre', 'right'): (4.,) * 7,
                       ('home', 'left'): (5.,) * 7, ('home', 'right'): (6.,) * 7,
                       ('body_home', 'body_joints'): (0.,) * 6},
    )
    def payload(side):
        return json.dumps({'label': '无遮挡背面',
                           'position1': {'x': .7, 'y': .1 if side == 'left' else -.1, 'z': .6},
                           'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}).encode()
    same = payload(first_arm)
    scene.queue = same
    VisionTargets.tick(scene)
    assert scene.state == 'next' and scene.back_second_payload == same
    assert [step[0] for step in scene.post_grasp_targets] == [
        'body_zero', 'transfer' if first_arm == 'left' else 'right_transfer']
    assert reports[-1][0] == 'back_handover_queued'

    scene.state = 'awaiting_second_target'
    scene.post_grasp_targets = []
    scene.back_second_payload = None
    opposite = payload('right' if first_arm == 'left' else 'left')
    scene.queue = opposite
    VisionTargets.tick(scene)
    assert scene.state == 'next' and scene.back_second_payload == opposite
    assert [step[0] for step in scene.post_grasp_targets] == [
        'body_zero', 'transfer' if first_arm == 'left' else 'right_transfer']
    assert reports[-1][0] == 'back_handover_queued'


def test_back_handover_returns_to_body_photo_before_second_grasp(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    second = b'second mqtt target'
    scene = SimpleNamespace(
        state='waiting', control_state='running', last_task_id=None,
        waiting_task_id='retreat-task', preset_targets=[],
        post_grasp_targets=[('right_handover_retreat', 'joints', (1.,) * 7, None)],
        back_first_arm='left', back_second_payload=second,
        front_grasp_arms=[], attached_workpiece=('right', 0.),
        queue_lock=threading.Lock(), queue=None,
        arm_positions={('pre', 'left'): (2.,) * 7, ('pre', 'right'): (3.,) * 7,
                       ('body_photo', 'body_pose_ptp'): ((-.05, 0., .6), (0., 0., 0., 1.))},
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'retreat-task'})))
    assert [step[0] for step in scene.post_grasp_targets] == [
        'left_return_pre', 'right_return_pre', 'body_photo_restore']
    scene.post_grasp_targets = [scene.post_grasp_targets[-1]]
    scene.state = 'waiting'
    scene.waiting_task_id = 'body-photo-task'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-photo-task'})))
    assert scene.state == 'awaiting_second_target' and scene.queue == second
    assert scene.front_grasp_arms == ['right'] and scene.back_second_payload is None


def test_second_front_target_uses_opposite_arm_even_when_y_selects_first(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports, sent = [], []
    scene = SimpleNamespace(
        state='awaiting_second_target', front_grasp_arms=['right'],
        queue_lock=threading.Lock(), queue=None, control_state='completed',
        dry_run=False, targets=[], tool_tip_offset_z_m=.2,
        attached_workpiece=('right', 0.), restore_mode=False,
        report=lambda state, **details: reports.append((state, details)),
        send_next=lambda: sent.append('arm target'),
        preset_targets=[], joint_restore_steps=None, post_grasp_targets=[],
        arm_positions={('home', 'left'): (1.,) * 7, ('home', 'right'): (2.,) * 7},
    )
    scene.send_body_pre_grasp = lambda side: sent.append(('normal_turn', side))
    scene.send_second_arm_prepare = lambda side: sent.append(('arm_prepare', side))
    def payload(y, label='无遮挡正面'):
        return json.dumps({'label': label,
                           'position1': {'x': .7, 'y': y, 'z': .6},
                           'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}).encode()
    scene.queue = payload(-.11, '无遮挡背面')
    VisionTargets.tick(scene)
    assert sent == [('arm_prepare', 'left')] and scene.targets[0][0] == 'left'
    scene.state = 'awaiting_second_target'
    scene.post_grasp_targets = []
    scene.queue = payload(.11, '无遮挡背面')
    VisionTargets.tick(scene)
    assert sent[-1] == 'arm target'
    assert scene.targets[0][0] == 'left'


@pytest.mark.parametrize('first_arm,second_y,expected_arm,action', [
    ('left', .11, 'right', 'prepare'),
    ('left', -.11, 'right', 'arm'),
    ('left', -.10, 'right', 'small'),
    ('left', -.05, 'right', 'small'),
    ('left', 0., 'right', 'small'),
    ('left', .049, 'right', 'small'),
    ('left', .05, 'right', 'prepare'),
    ('right', -.11, 'left', 'prepare'),
    ('right', -.050001, 'left', 'prepare'),
    ('right', -.05, 'left', 'small'),
    ('right', 0., 'left', 'small'),
    ('right', .10, 'left', 'small'),
    ('right', .11, 'left', 'arm'),
])
def test_second_target_follows_first_arm_and_y_only_selects_extra_turn(
        monkeypatch, first_arm, second_y, expected_arm, action):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    payload = json.dumps({
        'label': '无遮挡背面',
        'position1': {'x': .7, 'y': second_y, 'z': .6},
        'position2': {'x': .7, 'y': second_y, 'z': .5},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='awaiting_second_target', front_grasp_arms=[first_arm],
        back_first_arm=None, queue_lock=threading.Lock(), queue=payload,
        control_state='completed', dry_run=False, targets=[],
        tool_tip_offset_z_m=.2, attached_workpiece=(first_arm, 0.),
        report=lambda state, **details: None,
        send_next=lambda: sent.append(('arm', scene.targets[0][0])),
        send_second_arm_prepare=lambda side: sent.append(('prepare', side)),
        send_second_body_turn=lambda side, target_j6_rad=None: sent.append(
            ('small', side, target_j6_rad)),
        send_body_pre_grasp=lambda side: (_ for _ in ()).throw(
            AssertionError('Second grasp must preserve the other five body joints')),
    )
    VisionTargets.tick(scene)
    assert [side for side, *_ in scene.targets] == [expected_arm, expected_arm]
    expected = (('small', expected_arm, -.5 if expected_arm == 'left' else .5)
                if action == 'small' else (action, expected_arm))
    assert sent == [expected]


@pytest.mark.parametrize('first_arm,y,action', [
    ('left', -.050001, 'prepare'), ('left', -.05, 'small'),
    ('left', -.01, 'small'), ('left', 0., 'small'),
    ('left', .01, 'small'), ('left', .11, 'arm'),
    ('right', -.11, 'arm'), ('right', -.01, 'small'),
    ('right', 0., 'small'), ('right', .049, 'small'),
    ('right', .05, 'prepare'),
])
def test_after_back_handover_original_arm_grasps_with_boundary_turn(
        monkeypatch, first_arm, y, action):
    VisionTargets = import_scene_node(monkeypatch)
    receiver = 'right' if first_arm == 'left' else 'left'
    sent = []
    payload = json.dumps({
        'label': '无遮挡背面',
        'position1': {'x': .7, 'y': y, 'z': .6},
        'position2': {'x': .7, 'y': y, 'z': .5},
        'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1},
    }).encode()
    scene = SimpleNamespace(
        state='awaiting_second_target', front_grasp_arms=[receiver],
        back_first_arm=first_arm, queue_lock=threading.Lock(), queue=payload,
        control_state='completed', dry_run=False, targets=[],
        tool_tip_offset_z_m=.2, attached_workpiece=(receiver, 0.),
        report=lambda state, **details: None,
        send_next=lambda: sent.append(('arm', scene.targets[0][0])),
        send_second_body_turn=lambda side, target_j6_rad=None: sent.append(
            ('small', side, target_j6_rad)),
        send_second_arm_prepare=lambda side: sent.append(('prepare', side)),
        send_body_pre_grasp=lambda side: (_ for _ in ()).throw(
            AssertionError('Second grasp must not use the small-angle J6 preset')),
    )
    VisionTargets.tick(scene)
    assert [side for side, *_ in scene.targets] == [first_arm, first_arm]
    expected = (('small', first_arm, -.5 if first_arm == 'left' else .5)
                if action == 'small' else (action, first_arm))
    assert sent == [expected]


@pytest.mark.parametrize('side,expected', [
    ('left', [1.5, .8, -2., -1., 0., 0., 0.]),
    ('right', [-1.5, .8, 2., -1., 0., 0., 0.]),
])
def test_back_second_arm_prepare_completes_before_body_turn(monkeypatch, side, expected):
    VisionTargets = import_scene_node(monkeypatch)
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1,
                                publish=lambda message: sent.append(('arm', message)))
    scene = SimpleNamespace(
        joint_publishers={side: publisher},
        second_arm_prepare_joints=VisionTargets.second_arm_prepare_joints,
        targets=[(side, (), (), ())], state='idle',
        post_grasp_targets=[], preset_targets=[], joint_restore_steps=None,
        body_shift_m=0., last_task_id=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: reports.append((state, details)),
        send_second_body_turn=lambda arm: sent.append(('body', arm)),
    )
    VisionTargets.send_second_arm_prepare(scene, side)
    assert sent[0][0] == 'arm'
    assert sent[0][1].name == [f'{side}_joint_{index}' for index in range(1, 8)]
    assert sent[0][1].position == expected
    assert scene.state == 'waiting' and scene.second_arm_prepare_pending
    VisionTargets.on_control_status(scene, SimpleNamespace(
        data=json.dumps({'state': 'completed', 'task_id': 'arm-prepare'})))
    assert scene.state == 'next' and len(sent) == 1
    scene.next_at = 0.
    VisionTargets.tick(scene)
    assert sent[1] == ('body', side)
    assert not scene.second_turn_after_prepare


def test_back_second_arm_prepare_rejection_stops_body_turn(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    reports = []
    scene = SimpleNamespace(
        state='waiting', second_arm_prepare_pending=True,
        second_turn_after_prepare=True, waiting_task_id=None,
        last_task_id=None, targets=[('left', (), (), ())],
        report=lambda state, **details: reports.append((state, details)),
    )
    VisionTargets.on_control_status(scene, SimpleNamespace(
        data=json.dumps({'state': 'rejected', 'task_id': 'arm-prepare',
                         'reason': 'joint limit'})))
    assert scene.state == 'idle' and scene.targets == []
    assert not scene.second_turn_after_prepare
    assert reports[-1][0] == 'rejected'


def test_second_turn_changes_only_body_j6_to_fixed_target(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    scene = SimpleNamespace(
        current_body_joints=lambda: (.1, .2, .3, .4, .5, -.4),
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                             publish=sent.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: None,
    )
    VisionTargets.send_second_body_turn(scene, 'left')
    assert sent[0].position == pytest.approx([.1, .2, .3, .4, .5, -1.57])
    assert scene.body_pre_grasp_pending and scene.body_pre_grasp_used


def test_second_center_turn_preserves_other_body_joints(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    sent = []
    scene = SimpleNamespace(
        current_body_joints=lambda: (.1, .2, .3, .4, .5, 0.),
        body_joint_publisher=SimpleNamespace(get_subscription_count=lambda: 1,
                                             publish=sent.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: object())),
        report=lambda state, **details: None,
    )
    VisionTargets.send_second_body_turn(scene, 'right', target_j6_rad=.5)
    assert sent[0].position == pytest.approx([.1, .2, .3, .4, .5, .5])
    assert scene.body_pre_grasp_pending and scene.body_pre_grasp_used


def test_second_turn_reads_fresh_body_feedback_not_arm_feedback(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    feedback = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        name=[f'body_joint_{index}' for index in range(1, 7)],
        position=[.1, .2, .3, .4, .5, .6],
    )
    scene = SimpleNamespace(
        arm_joints=SimpleNamespace(name=['left_joint_1'], position=[9.]),
        body_joints=None, max_arm_pose_age_ns=500_000_000,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
    )
    assert VisionTargets.current_body_joints(scene) is None
    VisionTargets.on_body_joint_states(scene, feedback)
    assert VisionTargets.current_body_joints(scene) == pytest.approx((.1, .2, .3, .4, .5, .6))
    scene.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=11_000_000_000))
    assert VisionTargets.current_body_joints(scene) is None


def test_final_gripper_closes_finish_after_both_sides(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 10.)
    reports = []
    scene = SimpleNamespace(
        gripper_pending=('left', 0., 'left_final_close', 9.),
        state='gripper_waiting',
        post_grasp_targets=[('left_final_close', 'gripper', 0., None),
                            ('right_final_close', 'gripper', 0., None)],
        attached_workpiece=('right', 0.), workpiece_relative=((), ()),
        front_grasp_arms=['left', 'right'],
        report=lambda state, **details: reports.append((state, details)),
    )
    feedback = SimpleNamespace(
        valid=True, error_code=0, enabled=True, at_target=True,
        position_m=0., target_position_m=0., target_active=True,
        current_limit_reached=False, holding_position=False,
    )
    VisionTargets.on_gripper_state(scene, 'left', feedback)
    assert scene.state == 'next' and [step[0] for step in scene.post_grasp_targets] == [
        'right_final_close']
    scene.gripper_pending = ('right', 0., 'right_final_close', 9.)
    scene.state = 'gripper_waiting'
    VisionTargets.on_gripper_state(scene, 'right', feedback)
    assert scene.state == 'idle' and scene.attached_workpiece is None
    assert scene.front_grasp_arms == [] and reports[-1][0] == 'post_grasp_completed'


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


def test_grasp_current_limit_hold_attaches_automatically(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    import scene.node as scene_module
    monkeypatch.setattr(scene_module.time, 'monotonic', lambda: 10.)
    reports = []
    scene = SimpleNamespace(
        gripper_pending=('left', .027, 'left_grasp', 9.), state='gripper_waiting',
        report=lambda state, **details: reports.append((state, details)),
        attach_workpiece=lambda request, response: (
            setattr(response, 'success', True) or setattr(scene, 'attached_workpiece', ('left', 0.)) or response),
    )
    feedback = SimpleNamespace(
        valid=True, error_code=0, enabled=True, at_target=False,
        position_m=.02238, target_position_m=.02238, target_active=True,
        current_limit_reached=True, holding_position=True,
    )
    VisionTargets.on_gripper_state(scene, 'left', feedback)
    assert scene.attached_workpiece == ('left', 0.) and scene.gripper_pending is None
    assert reports[0] == ('gripper_completed', {
        'arm': 'left', 'step': 'left_grasp', 'completion': 'current_limit_hold',
        'position_m': .02238})

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


def test_arm_position_body_pose_publishes_and_completes(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    from builtin_interfaces.msg import Time
    sent, reports = [], []

    class Publisher:
        def get_subscription_count(self):
            return 1

        def publish(self, message):
            sent.append(message)

    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        body_pose_publisher=Publisher(), joint_publishers={},
        arm_positions={2: 'body_photo', ('body_photo', 'body_pose'):
                       ((-.05, 0., .6), (0., .38941332986322064, 0., .9210631132147452))},
        current_body_joints=lambda: (0.,) * 6,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        report=lambda state, **details: reports.append((state, details)),
        last_task_id='previous', waiting_task_id=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=2))
    assert len(sent) == 1 and sent[0].header.frame_id == 'body_base'
    assert sent[0].pose.position.z == .6
    assert scene.state == 'waiting'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-task'})))
    assert scene.state == 'idle' and reports[-1] == (
        'arm_position_completed', {'position': 'body_photo'})


def test_arm_position_body_pose_ptp_uses_joint_space_topic(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    from builtin_interfaces.msg import Time
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        body_pose_ptp_publisher=publisher, joint_publishers={},
        arm_positions={2: 'body_photo', ('body_photo', 'body_pose_ptp'):
                       ((-.05, 0., .6), (0., .38941332986322064, 0., .9210631132147452))},
        current_body_joints=lambda: (0.,) * 6,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        report=lambda state, **details: reports.append((state, details)),
        last_task_id='previous', waiting_task_id=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=2))
    assert len(sent) == 1 and sent[0].header.frame_id == 'body_base'
    assert scene.state == 'waiting'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-ptp-task'})))
    assert reports[-1] == ('arm_position_completed', {'position': 'body_photo'})


@pytest.mark.parametrize('j6', [-.6, .6])
def test_arm_position_body_photo_clears_j4_after_j6_turn(monkeypatch, j6):
    VisionTargets = import_scene_node(monkeypatch)
    from builtin_interfaces.msg import Time
    joints_sent, poses_sent, reports = [], [], []
    joint_pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=joints_sent.append)
    pose_pub = SimpleNamespace(get_subscription_count=lambda: 1, publish=poses_sent.append)
    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        body_joint_publisher=joint_pub, body_pose_ptp_publisher=pose_pub,
        arm_positions={2: 'body_photo', ('body_photo', 'body_pose_ptp'):
                       ((-.05, 0., .6), (0., .38941332986322064, 0., .9210631132147452))},
        current_body_joints=lambda: (.2, .3, .4, -.5, .6, j6),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        report=lambda state, **details: reports.append((state, details)),
        last_task_id=None, waiting_task_id=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=2))
    assert joints_sent[0].position == pytest.approx([.2, .3, .4, .1, .6, j6])
    assert not poses_sent
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'j4-task'})))
    assert scene.state == 'next' and not poses_sent
    VisionTargets.send_body_pose_preset(scene)
    assert poses_sent[0].pose.position.x == pytest.approx(-.05)
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'photo-task'})))
    assert reports[-1] == ('arm_position_completed', {'position': 'body_photo'})


def test_arm_position_body_joints_publishes_and_completes(monkeypatch):
    VisionTargets = import_scene_node(monkeypatch)
    from builtin_interfaces.msg import Time
    sent, reports = [], []
    publisher = SimpleNamespace(get_subscription_count=lambda: 1, publish=sent.append)
    scene = SimpleNamespace(
        dry_run=False, state='idle', queue=None, queue_lock=threading.Lock(),
        restore_mode=False, control_state='completed', preset_targets=[], preset_name=None,
        body_joint_publisher=publisher, joint_publishers={},
        arm_positions={3: 'body_home', ('body_home', 'body_joints'): (0.,) * 6},
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        report=lambda state, **details: reports.append((state, details)),
        last_task_id='previous', waiting_task_id=None,
    )
    VisionTargets.on_arm_position(scene, SimpleNamespace(data=3))
    assert len(sent) == 1
    assert sent[0].name == [f'body_joint_{index}' for index in range(1, 7)]
    assert list(sent[0].position) == [0.] * 6
    assert scene.state == 'waiting'
    VisionTargets.on_control_status(scene, SimpleNamespace(data=json.dumps({
        'state': 'completed', 'task_id': 'body-task'})))
    assert scene.state == 'idle'
    assert reports[-1] == ('arm_position_completed', {'position': 'body_home'})
