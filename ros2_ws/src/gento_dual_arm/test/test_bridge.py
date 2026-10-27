# 使用替代硬件的假 SDK 检查桥接状态机，无需连接机器人。
"""Tests use fake SDK hardware; no robot connection or native library is required."""
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from geometry_msgs.msg import Pose

from gento_dual_arm.conversions import JOINT_NAMES, joint_target, pose_matrix, quaternion
from gento_dual_arm.engine import Engine

BASE = Path(__file__).resolve().parents[4]
ROOT = next(p for p in (BASE, BASE/'tianji-robot-SDK')
            if (p/'PYTHON_SDK/GentoRobot.py').is_file())
sys.path.insert(0, str(ROOT))
from gento_dual_arm import dual_arm_control as helpers
from PYTHON_SDK.GentoRobot import FX_InvKineSolverParams


class DualRobot:
    max_point = 5000

    def __init__(self):
        self.dll = SimpleNamespace(FX_L1_Kinematics_InverseKinematics=SimpleNamespace())
        self.calls = []
        self.positions = [[0.] * 7, [0.] * 7]
        self.states = [0, 0, 0, 0]
        self.body_positions = [0.] * 6
        self.targets = {}
        self.start_mask = 3
        self.traj_states = [0, 0]

    def link(self, *args): return 1
    def reset_error(self, arm, timeout):
        self.calls.append(('reset', arm))
        self.states[arm] = 0
        return 0, 0
    def get_ctrl_obj_dof(self, obj): return 2 if obj == 2 else 0
    def check_link_state(self): return 1
    def init_single_arm_config(self, arm): return 0
    def param_get_float(self, name): return 0, (-180 if name.endswith('Neg') else 180)
    def current_state(self, arm): return self.states[arm]
    def get_rt_dict(self): return {'arms': [{'fb': {'fb_pos': q}} for q in self.positions], 'body': {'fb_pos': self.body_positions}, 'head': {'fb_pos': [10., -5., 999.]}}
    def get_sg_dict(self): return {'arms': [{'get': {'traj_state': state}, 'set': {'vel_ratio': 5., 'acc_ratio': 5.}} for state in self.traj_states]}
    def forward_kinematics(self, arm, q): return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    def plan_joints(self, arm, start, target, *args): return start + target, 2
    def switch_to_position_mode(self, arm, *args):
        self.calls.append(('position', arm))
        self.states[arm] = 1
        return 0
    def config_set_traj(self, arm, points, count):
        self.calls.append(('upload', arm))
        self.targets[arm] = points[-7:]
        self.traj_states[arm] = 2
        return 0
    def runtime_set_joint_pos_cmd(self, obj, positions):
        if obj in (0, 1):
            self.calls.append(('arm_command', obj))
            self.positions[obj] = list(positions)
        else:
            self.calls.append(('body_command', obj))
            self.body_positions = list(positions)
        return 0
    def runtime_run_traj(self, mask):
        self.calls.append(('run', mask))
        for arm in (0, 1):
            if mask & (1 << arm):
                self.traj_states[arm] = 0
                self.positions[arm] = list(self.targets[arm])
        return self.start_mask & mask
    def switch_to_idle(self, arm, *args):
        self.calls.append(('idle', arm))
        self.states[arm] = 0
        return 0
    def emergency_stop(self, mask):
        self.calls.append(('stop', mask))
        return mask
    def cleanup(self): self.calls.append('cleanup')
    def unlink(self): self.calls.append('unlink')


def engine():
    config = SimpleNamespace(cartesian_execution='stream', ip='127.0.0.1', speed=5., acc=5., plan_speed=.05,
                             plan_acc=.05, max_delta=5., timeout=60., tolerance=.2, sdk_length_to_m=.001, cartesian_speed=.01, cartesian_angular_speed=.1,
                             cartesian_joint_speed=5., cartesian_joint_acc=10.)
    e = Engine(DualRobot(), helpers, FX_InvKineSolverParams, config)
    e.connect()
    e.robot.calls.clear()
    return e



def test_connect_resets_both_arms_before_position():
    e = engine()
    e.robot.calls.clear()
    e.robot.states = [100, 100, 0, 100]
    e.connect()
    assert e.robot.calls == [('reset', 0), ('reset', 1), ('reset', 3), ('position', 0), ('position', 1), ('position', 3)]
    assert e.robot.states == [1, 1, 0, 1]
    assert not e.active
    e.close()


@pytest.mark.parametrize('stage', ['reset', 'position'])
def test_connect_failure_stops_and_prevents_ready(stage):
    e = engine()
    if stage == 'reset':
        e.robot.states[0] = 100
        e.robot.reset_error = lambda *args: (-1, 123)
    else:
        original = e.robot.switch_to_position_mode
        e.robot.switch_to_position_mode = lambda arm, *args: -1 if arm == 1 else original(arm, *args)
    with pytest.raises(RuntimeError):
        e.connect()
    assert e.faulted
    assert (('stop', 11) in e.robot.calls) == (stage == 'position')
    assert ('run', 3) not in e.robot.calls
    e.close()

def test_reorder_and_units():
    targets = joint_target(0, JOINT_NAMES[:7][::-1], [math.pi/180] * 7)
    assert targets == [1.] * 7
    with pytest.raises(ValueError): joint_target(0, JOINT_NAMES[:7] * 2, [0.] * 14)
    with pytest.raises(ValueError): joint_target(0, JOINT_NAMES[:7], [float('nan')] * 7)


@pytest.mark.parametrize('q', [(0., 0., 0., 1.), (1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (.5, .5, .5, .5)])
def test_pose_round_trip(q):
    p = Pose()
    p.position.x = .123
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    matrix = pose_matrix(p, .001)
    assert matrix[0][3] == pytest.approx(123.)
    back = quaternion(matrix)
    assert abs(sum(a*b for a,b in zip(back, q))) == pytest.approx(1.)


def test_invalid_quaternion():
    pose = Pose()
    pose.orientation.w = 0.0
    with pytest.raises(ValueError): pose_matrix(pose, .001)


def test_connect_does_not_move_and_stop_requests_sdk():
    e = engine()
    assert e.robot.calls == []
    e.stop(force=True)
    assert ('stop', 11) in e.robot.calls
    with pytest.raises(RuntimeError):
        e.start(targets=[[1.] * 7] * 2)
    e.close()


def test_start_poll_and_busy_rejection():
    e = engine()
    e.start(targets=[[1.] * 7] * 2)
    with pytest.raises(RuntimeError): e.start(targets=[[2.] * 7] * 2)
    q, _ = e.sample()
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[0, 0, 1, 1]):
        assert not e.poll(q)
        assert e.poll(q)
    assert not e.active
    assert e.robot.states[:2] == [1, 1]
    assert not any(call[0] == 'idle' for call in e.robot.calls)
    e.robot.calls.clear()
    e.start(targets=[[2.] * 7] * 2)
    assert not any(call[0] in ('idle', 'position') for call in e.robot.calls)
    e.close()


def test_partial_start_latches_fault():
    e = engine()
    e.robot.start_mask = 1
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    assert e.faulted
    assert ('stop', 11) in e.robot.calls
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    e.close()


def test_ik_error_code_rejected():
    e = engine()
    e.robot.inverse_kinematics = lambda *args: -1010
    matrix = e.robot.forward_kinematics(0, [0] * 7)
    matrix[0][3] = 1.
    with pytest.raises(ValueError): e.start(matrices=[matrix, matrix])
    assert not e.robot.calls
    e.close()


def test_pose_targets_use_ik():
    e = engine()
    def solve(arm, p):
        for i in range(7): p.m_Output_RetJoint[i] = p.m_Input_IK_RefJoint[i]
        return p
    e.robot.inverse_kinematics = solve
    matrix = e.robot.forward_kinematics(0, [0] * 7)
    state, detail = e.start(matrices=[matrix, matrix])
    assert state == 'completed'
    assert detail['joints_deg'] == [[0.] * 7] * 2
    e.close()


def test_ros_topics_and_stamps():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from gento_dual_arm.node import Bridge
    import time
    rclpy.init(args=['--ros-args', '-p', f'sdk_root:={ROOT}', '-p', 'ip:=127.0.0.1', '-p', 'cartesian_execution:=stream'], domain_id=231)
    fake = DualRobot()
    sdk = importlib.import_module('PYTHON_SDK.GentoRobot')
    executor = SingleThreadedExecutor()
    bridge = client = None
    try:
        with patch.object(sdk, 'GentoRobot', return_value=fake): bridge = Bridge()
        client = rclpy.create_node('gento_bridge_test_client')
        executor.add_node(bridge)
        executor.add_node(client)
        feedback, poses, statuses = [], [], []
        full_feedback, head_feedback = [], []
        client.create_subscription(JointState, '/gento_dual_arm/full_joint_states', full_feedback.append, 10)
        client.create_subscription(JointState, '/gento_dual_arm/head/joint_states', head_feedback.append, 10)
        client.create_subscription(JointState, '/gento_dual_arm/joint_states', feedback.append, 10)
        client.create_subscription(PoseStamped, '/gento_dual_arm/left/tcp_pose', poses.append, 10)
        client.create_subscription(String, '/gento_dual_arm/status', lambda m: statuses.append(m.data), 10)
        pub = client.create_publisher(JointState, '/gento_dual_arm/left/target_joints', 1)
        pose_pubs = [client.create_publisher(PoseStamped, f'/gento_dual_arm/{side}/target_pose', 1)
                     for side in ('left', 'right')]
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and (not feedback or not poses or not full_feedback or not head_feedback or not pub.get_subscription_count()):
            executor.spin_once(timeout_sec=.05)
        assert feedback and poses and full_feedback and head_feedback
        assert head_feedback[-1].name == ['head_joint_1', 'head_joint_2']
        assert head_feedback[-1].position == pytest.approx([math.radians(10), math.radians(-5)])
        assert len(full_feedback[-1].name) == len(full_feedback[-1].position) == 22
        assert full_feedback[-1].name == ['head_joint_1', 'head_joint_2'] + [f'body_joint_{i}' for i in range(1, 7)] + JOINT_NAMES
        assert not full_feedback[-1].velocity and not full_feedback[-1].effort
        assert not any(sub.topic_name == '/gento_dual_arm/target_joints' for sub in bridge.subscriptions)
        assert feedback[-1].name == JOINT_NAMES
        assert poses[-1].header.frame_id == 'left_arm_sdk_base'
        msg = JointState()
        msg.name, msg.position = JOINT_NAMES[:7], [math.radians(1)] * 7
        msg.header.stamp = client.get_clock().now().to_msg()
        pub.publish(msg)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not any('running' in s for s in statuses):
            executor.spin_once(timeout_sec=.05)
        assert any('running' in s for s in statuses)
        assert ('run', 1) in fake.calls
        deadline = time.monotonic() + 5
        while bridge.engine.active and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert not bridge.engine.active
        # Stamped pose target traverses ROS serialization and the IK path.
        def solve(arm, p):
            for i in range(7):
                p.m_Output_RetJoint[i] = p.m_Input_IK_RefJoint[i]
            p.m_Output_RetJoint[0] = p.m_Input_IK_TargetTCP[3]
            return p
        fake.inverse_kinematics = solve
        fake.forward_kinematics = lambda arm, q: [[1,0,0,q[0]], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
        target = [PoseStamped(), PoseStamped()]
        stamp = client.get_clock().now().to_msg()
        for arm, pose in enumerate(target):
            pose.header.stamp = stamp
            pose.header.frame_id = bridge.frames[arm]
            pose.pose.orientation.w = 1.
            pose.pose.position.x = .002
        old_count = sum('running' in s for s in statuses)
        pose_pubs[0].publish(target[0])
        until = time.monotonic() + .15
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        # A left pose alone starts planning; no right target is needed.
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and sum('running' in s for s in statuses) == old_count:
            executor.spin_once(timeout_sec=.05)
        assert sum('running' in s for s in statuses) > old_count
        deadline = time.monotonic() + 5
        while bridge.engine.active and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert not bridge.engine.active
        # Each pose callback independently forwards its own arm and matrix.
        with patch.object(bridge.engine, 'start', return_value=('completed', {})) as start:
            for arm in (0, 1):
                pose = PoseStamped()
                pose.header.frame_id = bridge.frames[arm]
                pose.pose.orientation.w = 1.
                pose.header.stamp = client.get_clock().now().to_msg()
                bridge.on_pose(arm, pose)
                assert start.call_args.kwargs['arm'] == arm
                assert 'target_matrix' in start.call_args.kwargs
            assert start.call_count == 2
        # Zero/old timestamps cannot cause movement.
        with pytest.raises(ValueError): bridge.stamp_check(JointState().header.stamp)
        assert sum(call == ('run', 1) for call in fake.calls) == 1
        left_before_right = list(fake.positions[0])
        right_pub = client.create_publisher(JointState, '/gento_dual_arm/right/target_joints', 1)
        deadline = time.monotonic() + 5
        while not right_pub.get_subscription_count() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        right_msg = JointState()
        right_msg.name = JOINT_NAMES[7:]
        right_msg.position = [math.radians(1)] * 7
        right_msg.header.stamp = client.get_clock().now().to_msg()
        right_pub.publish(right_msg)
        deadline = time.monotonic() + 5
        while ('run', 2) not in fake.calls and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert ('run', 2) in fake.calls
        assert fake.positions[0] == left_before_right
        deadline = time.monotonic() + 5
        while bridge.engine.active and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert not bridge.engine.active
        # Body feedback and a fresh body target traverse actual ROS messages.
        from gento_dual_arm.conversions import BODY_JOINT_NAMES
        from std_msgs.msg import Empty
        body_feedback = []
        client.create_subscription(JointState, '/gento_dual_arm/body/joint_states', body_feedback.append, 10)
        body_pub = client.create_publisher(JointState, '/gento_dual_arm/body/target_joints', 1)
        stop_pub = client.create_publisher(Empty, '/gento_dual_arm/stop', 1)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and (not body_feedback or not body_pub.get_subscription_count() or not stop_pub.get_subscription_count()):
            executor.spin_once(timeout_sec=.05)
        assert body_feedback[-1].name == BODY_JOINT_NAMES
        body_msg = JointState()
        body_msg.name = BODY_JOINT_NAMES
        body_msg.position = [math.radians(1)] * 6
        body_msg.header.stamp = client.get_clock().now().to_msg()
        body_pub.publish(body_msg)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and ('body_command', 3) not in fake.calls:
            executor.spin_once(timeout_sec=.05)
        assert ('body_command', 3) in fake.calls
        assert bridge.engine.motion_group == 'body'
        stop_pub.publish(Empty())
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not bridge.engine.faulted:
            executor.spin_once(timeout_sec=.05)
        assert bridge.engine.faulted
        assert ('stop', 11) in fake.calls
    finally:
        if bridge:
            bridge.engine.close()
            bridge.destroy_node()
        if client: client.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_shutdown_active_stops_and_unlinks():
    e = engine()
    e.start(targets=[[1.] * 7] * 2)
    e.close()
    assert e.robot.calls[-3:] == [('stop', 11), 'cleanup', 'unlink']


def test_upload_failure_prevents_start():
    e = engine()
    e.robot.config_set_traj = lambda *args: -1
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    assert ('run', 3) not in e.robot.calls
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_timeout_detected_and_stop_failure_still_unlinks():
    e = engine()
    e.start(targets=[[1.] * 7] * 2)
    e.deadline = 0
    with pytest.raises(TimeoutError): e.poll([[1.] * 7] * 2)
    e.robot.emergency_stop = lambda mask: 1
    with pytest.raises(RuntimeError): e.close()
    assert e.robot.calls[-2:] == ['cleanup', 'unlink']


@pytest.mark.parametrize('field', ['m_Output_IsOutRange', 'm_Output_IsDeg', 'm_Output_IsJntExd'])
def test_invalid_ik_flags_rejected_before_upload(field):
    e = engine()
    def solve(arm, p):
        if field == 'm_Output_IsDeg':
            p.m_Output_IsDeg[0] = 1
        else:
            setattr(p, field, 1)
        return p
    e.robot.inverse_kinematics = solve
    matrix = e.robot.forward_kinematics(0, [0.] * 7)
    matrix[0][3] = 1.
    with pytest.raises(ValueError):
        e.start(matrices=[matrix, matrix])
    assert not e.robot.calls
    e.close()


def test_native_ik_layout_matches_vendor_header(tmp_path):
    import ctypes
    import subprocess
    from gento_dual_arm.native_ik import bind_native_ik
    sdk = importlib.import_module('PYTHON_SDK.GentoRobot')
    robot = DualRobot()
    native = bind_native_ik(robot, sdk)
    fields = [name for name, _ in native._fields_]
    code = '#include <iostream>\n#include <cstddef>\n#include "FXKinematics.h"\nint main(){\n'
    code += 'std::cout << sizeof(FX_InvKineSolvePara);\n'
    for field in fields:
        code += f'std::cout << " " << offsetof(FX_InvKineSolvePara,{field});\n'
    code += '}\n'
    source = tmp_path/'abi.cpp'
    source.write_text(code)
    includes = sorted({p.parent for p in (ROOT/'C_SDK').rglob('*.h')})
    binary = tmp_path/'abi'
    subprocess.run(['g++', '-std=c++11', str(source), *[f'-I{p}' for p in includes],
                    '-o', str(binary)], check=True, capture_output=True)
    actual = list(map(int, subprocess.check_output([str(binary)], text=True).split()))
    assert actual == [ctypes.sizeof(native)] + [getattr(native, f).offset for f in fields]
    assert robot.dll.FX_L1_Kinematics_InverseKinematics.argtypes[-1] == ctypes.POINTER(native)


def test_body_conversion_and_validation():
    from gento_dual_arm.conversions import BODY_JOINT_NAMES, body_target
    assert body_target(BODY_JOINT_NAMES[::-1], [math.pi / 180] * 6) == [1.] * 6
    for names, positions in [(BODY_JOINT_NAMES[:5], [0.] * 5),
                             (BODY_JOINT_NAMES, [float('nan')] * 6),
                             (BODY_JOINT_NAMES[:3] * 2, [0.] * 6)]:
        with pytest.raises(ValueError): body_target(names, positions)


def test_body_motion_interlock_completion_and_stop():
    e = engine()
    state, details = e.start_body([1.] * 6)
    assert state == 'running' and details['group'] == 'body'
    assert e.robot.calls == [('body_command', 3)]
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    with pytest.raises(RuntimeError): e.start_body([2.] * 6)
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[0, 0, 1, 1]):
        assert not e.poll([[0.] * 7] * 2, [1.] * 6)
        assert e.poll([[0.] * 7] * 2, [1.] * 6)
    assert e.robot.states[3] == 1
    e.start_body([2.] * 6)
    assert not any(call[0] in ('idle', 'position') for call in e.robot.calls)
    e.close()
    assert ('stop', 11) in e.robot.calls


@pytest.mark.parametrize('target', [[6.] * 6, [float('inf')] * 6, [0.] * 5])
def test_body_bad_targets_never_sent(target):
    e = engine()
    with pytest.raises(ValueError): e.start_body(target)
    assert not e.robot.calls
    e.close()


def test_body_command_failure_stops_all_controlled_objects():
    e = engine()
    e.robot.runtime_set_joint_pos_cmd = lambda *args: -1
    with pytest.raises(RuntimeError): e.start_body([1.] * 6)
    assert e.faulted and ('stop', 11) in e.robot.calls
    e.close()


def test_body_timeout_and_fault_state():
    e = engine()
    e.start_body([1.] * 6)
    e.deadline = -1
    with pytest.raises(TimeoutError): e.poll([[0.] * 7] * 2)
    e.deadline = float('inf')
    e.robot.states[3] = 2
    with pytest.raises(RuntimeError): e.poll([[0.] * 7] * 2)
    e.close()


def test_body_limits_fail_closed():
    e = engine()
    e.robot.param_get_float = lambda name: (-1, 0) if name.startswith('R.B.') else (0, -180 if name.endswith('Neg') else 180)
    with pytest.raises(RuntimeError): e.connect()
    assert e.faulted and ('stop', 11) not in e.robot.calls
    e.close()


@pytest.mark.parametrize('moving_arm', [0, 1])
def test_stationary_arm_skips_planner_upload_and_start(moving_arm):
    e = engine()
    e.robot.states[1-moving_arm] = 0
    planned = []
    def plan(arm, start, target, *args):
        assert arm == moving_arm  # A zero-motion planner must never be called.
        planned.append(arm)
        return start + target, 2
    e.robot.plan_joints = plan
    targets = [[0.] * 7, [0.] * 7]
    targets[moving_arm][0] = 1.
    state, details = e.start(targets=targets)
    assert state == 'running'
    assert planned == [moving_arm]
    assert details['points'][1-moving_arm] == 0
    assert details['run_mask'] == 1 << moving_arm
    assert ('upload', 1-moving_arm) not in e.robot.calls
    assert ('position', 1-moving_arm) not in e.robot.calls
    assert ('run', 1 << moving_arm) in e.robot.calls
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[0, 0, 1, 1]):
        assert not e.poll(e.robot.positions)
        assert e.poll(e.robot.positions)
    assert ('idle', 1-moving_arm) not in e.robot.calls
    assert ('idle', moving_arm) not in e.robot.calls
    assert e.robot.states[moving_arm] == 1
    e.close()


def test_both_arms_within_tolerance_complete_without_sdk_motion():
    e = engine()
    e.robot.plan_joints = lambda *args: pytest.fail('No-op must not call planner')
    state, details = e.start(targets=[[.001] * 7, [0.] * 7])
    assert state == 'completed'
    assert details['points'] == [0, 0] and details['run_mask'] == 0
    assert not e.active and not e.robot.calls
    e.close()


def test_stationary_arm_does_not_hide_other_arm_invalid_target():
    e = engine()
    e.robot.plan_joints = lambda *args: pytest.fail('Invalid command must fail before planning')
    with pytest.raises(ValueError): e.start(targets=[[0.] * 7, [12.] * 7])
    assert not e.robot.calls
    e.close()


def test_single_arm_partial_start_stops_all():
    e = engine()
    e.robot.start_mask = 0
    with pytest.raises(RuntimeError): e.start(targets=[[0.] * 7, [1.] * 7])
    assert ('stop', 11) in e.robot.calls and e.faulted
    e.close()


def test_reconnect_body_in_position_goes_idle_before_reset():
    e = engine()
    def reset(arm, timeout):
        e.robot.calls.append(('reset', arm))
        # Emulate a controller that does not leave POSITION on reset.
        if e.robot.states[arm] != 0:
            return -10, 0
        return 0, 0
    e.robot.reset_error = reset
    e.connect()
    assert e.robot.calls == [('idle', 0), ('idle', 1), ('idle', 3), ('position', 0), ('position', 1), ('position', 3)]
    assert e.robot.states[3] == 1
    assert ('stop', 11) not in e.robot.calls
    e.close()


def test_body_reset_timeout_remains_fatal_and_reports_state():
    e = engine()
    e.robot.states[3] = 100
    original = e.robot.reset_error
    e.robot.reset_error = lambda arm, timeout: (-10, 0) if arm == 3 else original(arm, timeout)
    with pytest.raises(RuntimeError, match=r'body reset failed: ret=-10, error=0, state_before=100, state_after=100'):
        e.connect()
    assert e.faulted and ('stop', 11) not in e.robot.calls
    assert ('position', 3) not in e.robot.calls
    e.close()


@pytest.mark.parametrize('arm', [0, 1])
def test_single_arm_interface_preserves_other_arm_feedback(arm):
    e = engine()
    e.robot.positions[1-arm] = [2.] * 7
    state, details = e.start(arm=arm, target=[1.] * 7)
    assert state == 'running' and details['run_mask'] == 1 << arm
    assert e.robot.positions[1-arm] == [2.] * 7
    assert ('upload', 1-arm) not in e.robot.calls
    e.close()


def test_single_arm_names_cannot_target_other_arm():
    for arm in (0, 1):
        other = 1-arm
        with pytest.raises(ValueError):
            joint_target(arm, JOINT_NAMES[other*7:(other+1)*7], [0.] * 7)


def test_head_feedback_uses_actual_dof_and_validates():
    e = engine()
    assert e.sample_head() == [10., -5.]
    e.head_dof = 0
    assert e.sample_head({}) == []
    e.head_dof = 2
    with pytest.raises(ValueError): e.sample_head({'head': {'fb_pos': [1.]}})
    with pytest.raises(ValueError): e.sample_head({'head': {'fb_pos': [1., float('nan')]}})
    e.close()


def test_reset_errors_clears_stop_latch_without_motion():
    e = engine()
    e.stop(force=True)
    e.robot.states = [100, 100, 0, 100]
    e.robot.calls.clear()
    e.reset_errors()
    assert not e.faulted and not e.active and e.targets is None
    assert e.robot.states == [1, 1, 0, 1]
    assert [call for call in e.robot.calls if call[0] == 'reset'] == [('reset', 0), ('reset', 1), ('reset', 3)]
    assert not any(call[0] in ('run', 'upload', 'body_command') for call in e.robot.calls)
    e.close()


def test_reset_rejected_while_moving():
    e = engine()
    e.start(arm=0, target=[1.] * 7)
    calls = list(e.robot.calls)
    with pytest.raises(ValueError): e.reset_errors()
    assert e.active and not e.faulted and e.robot.calls == calls
    e.close()


def test_reset_failure_keeps_fault_latched():
    e = engine()
    e.robot.states[0] = 100
    e.robot.reset_error = lambda *args: (-10, 102)
    with pytest.raises(RuntimeError, match='error=102'): e.reset_errors()
    assert e.faulted and ('stop', 11) not in e.robot.calls
    with pytest.raises(RuntimeError): e.start(arm=0, target=[1.] * 7)
    e.close()


def test_startup_fault_keeps_reset_subscription_and_recovers():
    from sensor_msgs.msg import JointState
    import rclpy
    from std_msgs.msg import Empty
    from gento_dual_arm.node import Bridge
    sdk = importlib.import_module('PYTHON_SDK.GentoRobot')
    fake = DualRobot()
    original = fake.reset_error
    fake.states[0] = 100
    fake.reset_error = lambda *args: (-10, 102)
    rclpy.init(args=['--ros-args', '-p', f'sdk_root:={ROOT}', '-p', 'ip:=127.0.0.1'], domain_id=231)
    bridge = None
    try:
        with patch.object(sdk, 'GentoRobot', return_value=fake): bridge = Bridge()
        assert bridge.engine.faulted and bridge.timer.is_canceled()
        assert not bridge.errors_timer.is_canceled()
        assert any(sub.topic_name == '/gento_dual_arm/reset_error' for sub in bridge.subscriptions)
        with patch.object(bridge.errors_pub, 'publish') as publish:
            bridge.publish_errors()
            import json
            assert json.loads(publish.call_args.args[0].data)['available'] is False
        fake.reset_error = original
        bridge.on_reset_error(Empty())
        assert not bridge.engine.faulted and not bridge.timer.is_canceled()
        with pytest.raises(ValueError): bridge.stamp_check(JointState().header.stamp)
        assert not any(call[0] == 'run' for call in fake.calls if isinstance(call, tuple))
    finally:
        if bridge:
            bridge.engine.close()
            bridge.destroy_node()
        rclpy.shutdown()


def test_repeated_reset_failures_do_not_reissue_estop():
    e = engine()
    e.robot.states[0] = 100
    e.robot.reset_error = lambda *args: (-10, 102)
    for _ in range(3):
        with pytest.raises(RuntimeError, match='error=102'):
            e.reset_errors()
        assert e.faulted
        assert not e.recovery_stop_requested
        assert e.recovery_stage == 'reset_summary'
    assert ('stop', 11) not in e.robot.calls
    e.close()


def test_failed_idle_transition_still_requests_stop():
    e = engine()
    e.robot.switch_to_idle = lambda *args: -10
    with pytest.raises(RuntimeError): e.reset_errors()
    assert e.recovery_stop_requested and ('stop', 11) in e.robot.calls
    with pytest.raises(RuntimeError, match='IDLE on exit'):
        e.close()
    assert e.robot.calls[-2:] == ['cleanup', 'unlink']


def test_recovery_stop_failure_preserves_initial_failure():
    e = engine()
    e.robot.switch_to_position_mode = lambda *args: -10
    e.robot.emergency_stop = lambda mask: 0
    with pytest.raises(RuntimeError, match='POSITION.*recovery stop failed'):
        e.reset_errors()
    assert e.faulted
    e.close()


def test_normal_close_confirms_all_idle_before_unlink_and_is_idempotent():
    e = engine()
    e.close()
    assert e.robot.calls == [('idle', 0), ('idle', 1), ('idle', 3), 'cleanup', 'unlink']
    assert e.robot.states == [0, 0, 0, 0]
    assert not e.linked
    calls = list(e.robot.calls)
    e.close()
    assert e.robot.calls == calls


def test_close_preserves_error_without_reset_and_idles_healthy_objects():
    e = engine()
    e.robot.states[0] = 100
    e.close()
    assert e.robot.states[0] == 100
    assert e.robot.calls == [('idle', 1), ('idle', 3), 'cleanup', 'unlink']


def test_close_idle_failure_stops_and_still_cleans_up():
    e = engine()
    original = e.robot.switch_to_idle
    e.robot.switch_to_idle = lambda obj, *args: -10 if obj == 0 else original(obj, *args)
    with pytest.raises(RuntimeError, match='arm0 IDLE on exit'):
        e.close()
    assert ('idle', 1) in e.robot.calls and ('idle', 3) in e.robot.calls
    assert e.robot.calls[-3:] == [('stop', 11), 'cleanup', 'unlink']


def test_healthy_idle_startup_does_not_reset():
    e = engine()
    e.robot.states = [0, 0, 0, 0]
    e.robot.reset_error = lambda *args: pytest.fail('Healthy IDLE object must not be reset')
    e.connect()
    assert e.robot.states == [1, 1, 0, 1]
    e.close()


def test_close_disconnected_reports_unconfirmed_idle_and_releases_resources():
    e = engine()
    e.robot.check_link_state = lambda: 0
    with pytest.raises(RuntimeError, match='IDLE could not be confirmed'):
        e.close()
    assert e.robot.calls == ['cleanup', 'unlink']


@pytest.mark.parametrize('obj', [0, 1, 3])
def test_individual_reset_matches_vendor_button_and_keeps_motion_locked(obj):
    e = engine()
    before = list(e.robot.states)
    details = e.reset_object_error(obj)
    assert details['state_after'] == 0
    assert e.robot.calls == [('reset', obj)]
    for other in (0, 1, 3):
        assert e.robot.states[other] == (0 if other == obj else before[other])
    assert e.faulted and not e.active
    with pytest.raises(RuntimeError): e.start(arm=0, target=[1.] * 7)
    e.close()


def test_individual_reset_failure_does_not_stop_other_objects():
    e = engine()
    e.robot.reset_error = lambda *args: (-10, 112)
    with pytest.raises(RuntimeError, match='error=112'): e.reset_object_error(0)
    assert e.faulted and not e.robot.calls
    e.close()


def test_group_reset_attempts_all_error_objects_before_reporting_failure():
    e = engine()
    e.robot.states = [100, 100, 0, 100]
    def reset(obj, timeout):
        e.robot.calls.append(('reset', obj))
        if obj == 0:
            return -10, 112
        e.robot.states[obj] = 0
        return 0, 0
    e.robot.reset_error = reset
    with pytest.raises(RuntimeError, match='arm0 reset failed'):
        e.reset_errors()
    assert e.robot.calls == [('reset', 0), ('reset', 1), ('reset', 3)]
    assert e.faulted
    e.close()


def cartesian_robot(e):
    references = [[], []]
    def fk(arm, q):
        return [[1, 0, 0, q[0]], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    def ik(arm, p):
        references[arm].append(list(p.m_Input_IK_RefJoint))
        p.m_Output_RetJoint[0] = p.m_Input_IK_TargetTCP[3]
        return p
    e.robot.forward_kinematics = fk
    e.robot.inverse_kinematics = ik
    e.robot.plan_joints = lambda *args: pytest.fail('Cartesian path must not use joint PTP')
    goal = fk(0, [1.]*7)
    return goal, references


def test_cartesian_preplans_and_passes_previous_ik_reference():
    e = engine()
    goal, references = cartesian_robot(e)
    state, details = e.start(matrices=[goal, goal])
    assert state == 'running'
    assert details['points'] == [51, 51]
    assert not e.robot.calls  # no uploading or starting the controller trajectory player
    plans = e.cartesian_stream['plans']
    for arm in (0, 1):
        points, count = plans[arm]
        path = [list(points[i*7:(i+1)*7]) for i in range(count)]
        assert references[arm] == path[:-1]
        assert path[-1] == [1., 0., 0., 0., 0., 0., 0.]
    e.close()


def test_cartesian_midpath_failure_never_uploads():
    e = engine()
    goal, _ = cartesian_robot(e)
    original = e.robot.inverse_kinematics
    def fail(arm, p):
        if p.m_Input_IK_TargetTCP[3] > .4:
            return -1
        return original(arm, p)
    e.robot.inverse_kinematics = fail
    with pytest.raises(ValueError, match='IK failed'):
        e.start(matrices=[goal, goal])
    assert not e.robot.calls
    assert not e.active
    e.close()


def test_cartesian_rotation_uses_shortest_arc():
    from gento_dual_arm.cartesian import interpolate
    start = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    goal = [[-1,0,0,10], [0,-1,0,20], [0,0,1,30], [0,0,0,1]]
    middle = interpolate(start, goal, .5)
    assert [middle[i][3] for i in range(3)] == [5,10,15]
    assert middle[0][0] == pytest.approx(0, abs=1e-12)
    assert middle[1][0] == pytest.approx(1)


@pytest.mark.parametrize('failure', ['jump', 'capacity', 'timeout'])
def test_cartesian_rejects_invalid_plan_before_upload(failure):
    e = engine()
    goal, _ = cartesian_robot(e)
    if failure == 'jump':
        original = e.robot.inverse_kinematics
        def jump(arm, p):
            p = original(arm, p)
            p.m_Output_RetJoint[0] += 1.
            return p
        e.robot.inverse_kinematics = jump
    elif failure == 'capacity':
        e.robot.max_point = 20
    else:
        e.c.timeout = .5
    with pytest.raises(ValueError):
        e.start(matrices=[goal, goal])
    assert not e.robot.calls
    e.close()


def test_cartesian_holds_stationary_arm_without_ik():
    e = engine()
    goal, references = cartesian_robot(e)
    hold = e.robot.forward_kinematics(1, e.robot.positions[1])
    _, details = e.start(matrices=[goal, hold])
    assert details['moving_arms'] == [0]
    assert references[0] and not references[1]
    assert details['joints_deg'][1] == [0.]*7
    e.close()


def test_cartesian_cancel_during_ik_prevents_execution():
    e = engine()
    goal, _ = cartesian_robot(e)
    original = e.robot.inverse_kinematics
    def cancel(arm, p):
        e.cancel_requested.set()
        return original(arm, p)
    e.robot.inverse_kinematics = cancel
    with pytest.raises(ValueError, match='cancelled'):
        e.start(matrices=[goal, goal])
    assert not e.robot.calls
    assert e.stream_tick() == 'stopped'
    assert e.faulted
    e.close()


def test_cartesian_retimes_rate_excess():
    e = engine()
    goal, _ = cartesian_robot(e)
    e.c.cartesian_joint_speed = .5
    _, detail = e.start(matrices=[goal, goal])
    assert detail['points'][0] > 51
    e.close()


def test_cartesian_stream_50hz_no_burst_and_cancel():
    e = engine()
    goal, _ = cartesian_robot(e)
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.):
        e.start(matrices=[goal, goal])
        e.stream_tick()
        assert len(e.robot.calls) == 2
        e.stream_tick()
        assert len(e.robot.calls) == 2
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.02):
        e.stream_tick()
        assert len(e.robot.calls) == 4
        assert e.cartesian_stream['index'] == 2
        assert not e.poll(e.robot.positions)
        e.cancel_requested.set()
        assert e.stream_tick() == 'stopped'
        assert e.cartesian_stream is None and not e.active
        assert e.robot.calls[-1] == ('stop', 11)
    e.close()


def test_stop_during_blocking_upload_prevents_joint_trajectory_start():
    e = engine()
    def upload(*args):
        e.cancel_requested.set()
        return 0
    e.robot.config_set_traj = upload
    with pytest.raises(ValueError, match='cancelled'):
        e.start(arm=0, target=[1.]*7)
    assert ('run', 1) not in e.robot.calls
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_cartesian_final_point_waits_for_settling_without_stall():
    e = engine()
    goal, _ = cartesian_robot(e)
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.):
        e.start(matrices=[goal, goal])
    for index in range(51):
        with patch('gento_dual_arm.engine.time.monotonic', return_value=100. + index*.02001):
            e.stream_tick()
    e.robot.positions = [[1.,0,0,0,0,0,0]]*2
    with patch('gento_dual_arm.engine.time.monotonic', return_value=101.1):
        e.stream_tick()
        assert not e.poll(e.robot.positions)
    with patch('gento_dual_arm.engine.time.monotonic', return_value=101.7):
        e.stream_tick()
        assert e.poll(e.robot.positions)
    assert e.cartesian_stream is None
    e.close()


def test_cartesian_stalled_stream_rejected():
    e = engine()
    goal, _ = cartesian_robot(e)
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.):
        e.start(matrices=[goal, goal])
        e.stream_tick()
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.2):
        with pytest.raises(RuntimeError, match='stalled'):
            e.stream_tick()
    e.close()


def test_ros_stop_callback_runs_during_blocked_default_group():
    import threading
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import Empty
    from gento_dual_arm.node import Bridge
    sdk = importlib.import_module('PYTHON_SDK.GentoRobot')
    fake = DualRobot()
    rclpy.init(args=['--ros-args', '-p', f'sdk_root:={ROOT}', '-p', 'ip:=127.0.0.1'], domain_id=232)
    executor = MultiThreadedExecutor(num_threads=2)
    bridge = client = None
    entered, observed = threading.Event(), threading.Event()
    try:
        with patch.object(sdk, 'GentoRobot', return_value=fake):
            bridge = Bridge()
        bridge.timer.cancel()
        bridge.errors_timer.cancel()
        bridge.stream_timer.cancel()
        fake.calls.clear()
        def block():
            entered.set()
            if bridge.engine.cancel_requested.wait(3.):
                observed.set()
            timer.cancel()
        timer = bridge.create_timer(.01, block)
        client = rclpy.create_node('stop_concurrency_client')
        pub = client.create_publisher(Empty, '/gento_dual_arm/stop', 10)
        executor.add_node(bridge)
        executor.add_node(client)
        worker = threading.Thread(target=executor.spin)
        worker.start()
        assert entered.wait(2.)
        import time
        until = time.monotonic() + 2.
        while not pub.get_subscription_count() and time.monotonic() < until:
            time.sleep(.01)
        pub.publish(Empty())
        assert observed.wait(2.)
        assert not fake.calls  # stop callback never calls SDK concurrently
    finally:
        if bridge is not None:
            bridge.engine.cancel_requested.set()
        executor.shutdown()
        if 'worker' in locals(): worker.join(timeout=3.)
        if bridge is not None:
            bridge.engine.close()
            bridge.destroy_node()
        if client is not None: client.destroy_node()
        rclpy.shutdown()


def test_pending_stop_prevents_recovery():
    e = engine()
    e.cancel_requested.set()
    with pytest.raises(ValueError, match='cancelled'):
        e.reset_errors()
    assert e.faulted and e.recovery_stage != 'ready'
    assert not e.robot.calls
    assert e.stream_tick() == 'stopped'
    e.close()


@pytest.mark.parametrize('operation', ['switch_to_idle', 'reset_error', 'init_single_arm_config',
                                       'param_get_float', 'switch_to_position_mode', 'get_rt_dict'])
def test_stop_interrupts_recovery_before_next_step(operation):
    e = engine()
    if operation == 'reset_error':
        e.robot.states[0] = 100
    calls = []
    original = getattr(e.robot, operation)
    def interrupted(*args):
        calls.append(args)
        result = original(*args)
        e.cancel_requested.set()
        return result
    setattr(e.robot, operation, interrupted)
    with pytest.raises((ValueError, RuntimeError), match='cancelled'):
        e.reset_errors()
    assert len(calls) == 1
    assert e.faulted and e.recovery_stage != 'ready'
    assert e.recovery_stop_requested
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_delayed_stream_waits_from_sdk_completion():
    e = engine()
    goal, _ = cartesian_robot(e)
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.):
        e.start(matrices=[goal, goal])
        e.stream_tick()
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[100.08, 100.09]):
        e.stream_tick()
    assert e.cartesian_stream['index'] == 2
    for now in (100.091, 100.10, 100.109):
        with patch('gento_dual_arm.engine.time.monotonic', return_value=now):
            e.stream_tick()
            assert e.cartesian_stream['index'] == 2
    with patch('gento_dual_arm.engine.time.monotonic', return_value=100.111):
        e.stream_tick()
        assert e.cartesian_stream['index'] == 3
    e.close()


@pytest.mark.parametrize('drift', [.1, -.1])
def test_cartesian_entry_drift_rejects_before_first_command(drift):
    e = engine()
    goal, _ = cartesian_robot(e)
    e.start(matrices=[goal, goal])
    e.robot.positions[0][0] = drift
    with pytest.raises(RuntimeError, match='entry exceeds rate limits'):
        e.stream_tick()
    assert not e.robot.calls
    e.close()


def test_cartesian_small_entry_correction_is_checked_with_next_segment():
    e = engine()
    goal, _ = cartesian_robot(e)
    e.start(matrices=[goal, goal])
    e.robot.positions[0][0] = .0001
    e.stream_tick()
    assert e.cartesian_stream['index'] == 1
    assert e.robot.calls == [('arm_command', 0), ('arm_command', 1)]
    e.close()


@pytest.mark.parametrize('arm', [0, 1])
def test_single_pose_plans_only_selected_arm(arm):
    e = engine()
    goal, references = cartesian_robot(e)
    state, detail = e.start(arm=arm, target_matrix=goal)
    assert state == 'running'
    assert detail['moving_arms'] == [arm]
    assert detail['run_mask'] == 1 << arm
    assert references[arm] and not references[1-arm]
    assert detail['joints_deg'][1-arm] == [0.]*7
    e.stream_tick()
    assert e.robot.calls == [('arm_command', arm)]
    with pytest.raises(RuntimeError, match='Busy'):
        e.start(arm=1-arm, target_matrix=goal)
    e.close()


def test_single_pose_rejects_invalid_argument_combinations():
    e = engine()
    goal, _ = cartesian_robot(e)
    for kwargs in ({'target_matrix': goal}, {'arm': 2, 'target_matrix': goal},
                   {'arm': 0, 'target_matrix': goal, 'target': [0.]*7}):
        with pytest.raises(ValueError):
            e.start(**kwargs)
    assert not e.robot.calls
    e.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_send_single_pose_cli(side):
    from unittest.mock import MagicMock
    from builtin_interfaces.msg import Time
    from gento_dual_arm import send_target
    fake_node = MagicMock()
    fake_node.create_publisher.return_value.get_subscription_count.return_value = 1
    fake_node.get_clock.return_value.now.return_value.to_msg.return_value = Time(sec=123)
    values = ['0.1', '0.2', '0.3', '0', '0', '0', '1']
    with patch.object(sys, 'argv', ['send_target', f'--{side}-pose', *values]), \
            patch.object(send_target.rclpy, 'init'), \
            patch.object(send_target.rclpy, 'shutdown'), \
            patch.object(send_target.rclpy, 'create_node', return_value=fake_node), \
            patch.object(send_target.time, 'monotonic', side_effect=range(20)):
        send_target.main()
    args = fake_node.create_publisher.call_args.args
    assert args[1] == f'/gento_dual_arm/{side}/target_pose'
    msg = fake_node.create_publisher.return_value.publish.call_args.args[0]
    assert msg.header.frame_id == f'{side}_arm_sdk_base'
    assert msg.header.stamp.sec == 123
    assert msg.pose.position.y == .2 and msg.pose.orientation.w == 1.


def test_legacy_paired_pose_cli_rejected_before_publishing():
    from gento_dual_arm import send_target
    with patch.object(sys, 'argv', ['send_target', '--poses', *(['0']*14)]), \
            patch.object(send_target.rclpy, 'init') as init:
        with pytest.raises(SystemExit):
            send_target.main()
        init.assert_not_called()


def test_deadline_rearming_avoids_halving_rate_with_sdk_overhead():
    from unittest.mock import MagicMock
    from gento_dual_arm.node import Bridge
    e = engine()
    goal, _ = cartesian_robot(e)
    clock = [100.]
    original = e.robot.runtime_set_joint_pos_cmd
    def send(arm, q):
        clock[0] += .001  # Each SDK call takes 1 ms, which used to miss the next tick.
        return original(arm, q)
    e.robot.runtime_set_joint_pos_cmd = send
    harness = SimpleNamespace(engine=e, stream_timer=MagicMock())
    send_times = []
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=lambda: clock[0]):
        _, details = e.start(matrices=[goal, goal])
        for _ in range(51):
            e.stream_tick()
            send_times.append(clock[0])
            clock[0] += .0002  # Work between send completion and rearming the timer.
            Bridge.rearm_stream_timer(harness)
            clock[0] += harness.stream_timer.timer_period_ns / 1e9
        assert e.cartesian_stream['index'] == 51
        assert harness.stream_timer.reset.call_count == 51
        intervals = [b-a for a, b in zip(send_times, send_times[1:])]
        assert min(intervals) >= .02
        assert max(intervals) < .02201  # 20 ms + two SDK calls, without an extra 20 ms tick.
        stats = e.motion_metrics['cartesian_stream']
        assert stats['actual_hz'] == pytest.approx(1/.022, abs=.01)
        assert stats['sent_points'] == 51
        assert stats['max_interval_s'] == pytest.approx(.022, abs=1e-6)
        assert details['cartesian_plan']['planned_duration_s'] == 1.
        assert details['cartesian_plan']['retime_count'] == 0
    e.close()


def test_cartesian_status_explains_automatic_retiming():
    e = engine()
    goal, _ = cartesian_robot(e)
    e.c.cartesian_joint_speed = .5
    _, detail = e.start(matrices=[goal, goal])
    info = detail['cartesian_plan']
    assert info['retime_count'] > 0
    assert info['planned_duration_s'] == (detail['points'][0]-1)/50
    assert info['target_hz'] == 50
    e.close()


@pytest.mark.parametrize('constraint', ['joint_speed', 'joint_acceleration'])
def test_retime_logs_constraint_joint_value_and_limit(constraint):
    e = engine()
    goal, _ = cartesian_robot(e)
    if constraint == 'joint_speed':
        e.c.cartesian_joint_speed = .5
    else:
        e.c.cartesian_joint_acc = 1.
    logged = []
    e.on_retime = logged.append
    _, detail = e.start(arm=0, target_matrix=goal)
    info = detail['cartesian_plan']
    assert logged == info['retime_events']
    assert len(logged) == info['retime_count'] > 0
    for attempt, event in enumerate(logged, 1):
        assert event['constraint'] == constraint
        assert event['arm'] == 'left' and event['arm_id'] == 0
        assert event['joint'] == 1 and event['point_index'] >= 1
        assert event['magnitude'] == abs(event['value']) > event['limit']
        assert event['unit'] == ('deg/s' if constraint == 'joint_speed' else 'deg/s^2')
        assert event['attempt'] == attempt and event['will_retry']
        assert event['next_duration_s'] > event['duration_s']
    e.close()


def test_retime_failure_still_logs_last_attempt():
    from gento_dual_arm.cartesian import RateExceeded
    e = engine()
    goal, _ = cartesian_robot(e)
    logged = []
    e.on_retime = logged.append
    failure = RateExceeded(1, 25, 'joint_speed', [5.1]+[0.]*6, 5.)
    failure.time_scale = 1.04
    with patch('gento_dual_arm.cartesian._sample', side_effect=failure):
        with pytest.raises(RateExceeded, match='joint1.*joint_speed.*limit='):
            e.start(arm=1, target_matrix=goal)
    assert len(logged) == 6
    assert logged[-1]['arm'] == 'right'
    assert logged[-1]['will_retry'] is False
    assert 'next_duration_s' not in logged[-1]
    assert not e.robot.calls
    e.close()


@pytest.mark.parametrize('constraint,limit', [('joint_speed', 1.8), ('joint_acceleration', 5.)])
def test_peak_retiming_avoids_doubling_for_small_excess(constraint, limit):
    e = engine()
    goal, references = cartesian_robot(e)
    if constraint == 'joint_speed':
        e.c.cartesian_joint_speed = limit
    else:
        e.c.cartesian_joint_acc = limit
    _, detail = e.start(arm=0, target_matrix=goal)
    info = detail['cartesian_plan']
    assert info['retime_count'] == 1
    assert 1. < info['planned_duration_s'] < 1.3
    event = info['retime_events'][0]
    assert event['constraint'] == constraint
    assert event['magnitude'] > event['limit']
    assert event['time_scale'] < 1.3
    # Both full sampling passes must finish, not stop at their first limit crossing.
    assert len(references[0]) == 50 + detail['points'][0] - 1
    for peak in info['peaks'].values():
        assert peak['magnitude'] <= peak['limit']
    e.close()


def test_full_path_ik_failure_is_not_hidden_by_an_earlier_speed_excess():
    e = engine()
    goal, _ = cartesian_robot(e)
    e.c.cartesian_joint_speed = .5
    original = e.robot.inverse_kinematics
    def fail_late(arm, p):
        if p.m_Input_IK_TargetTCP[3] > .9:
            return -1011
        return original(arm, p)
    e.robot.inverse_kinematics = fail_late
    logged = []
    e.on_retime = logged.append
    with pytest.raises(ValueError, match='-1011'):
        e.start(arm=0, target_matrix=goal)
    assert not logged and not e.robot.calls
    e.close()


def test_proportional_retiming_still_enforces_capacity_and_timeout():
    e = engine()
    goal, _ = cartesian_robot(e)
    e.c.cartesian_joint_speed = .00001
    logged = []
    e.on_retime = logged.append
    with pytest.raises(ValueError, match='capacity or motion timeout'):
        e.start(arm=0, target_matrix=goal)
    assert len(logged) == 1
    assert logged[0]['next_duration_s'] > e.c.timeout
    assert not e.robot.calls
    e.close()


@pytest.mark.parametrize('arm', [0, 1])
def test_cartesian_controller_playback_uses_whole_preplanned_path(arm):
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, references = cartesian_robot(e)
    e.robot.runtime_set_joint_pos_cmd = lambda *args: pytest.fail('Playback must not send independent position targets')
    state, detail = e.start(arm=arm, target_matrix=goal)
    assert state == 'running' and detail['cartesian_execution'] == 'trajectory'
    assert references[arm] and not references[1-arm]
    assert e.robot.calls == [('upload', arm), ('run', 1 << arm)]
    assert e.cartesian_stream is None and e.cartesian_tracking is not None
    # Completion must wait for the controller to finish playback, even at the goal.
    e.robot.get_sg_dict = lambda: {'arms': [{'get': {'traj_state': 3}, 'set': {'vel_ratio': 5., 'acc_ratio': 5.}}]*2}
    assert not e.poll(e.robot.positions)
    e.robot.get_sg_dict = lambda: {'arms': [{'get': {'traj_state': 0}, 'set': {'vel_ratio': 5., 'acc_ratio': 5.}}]*2}
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[0., 0., 1., 1.]):
        assert not e.poll(e.robot.positions)
        assert e.poll(e.robot.positions)
    assert e.motion_metrics['cartesian_tracking']['max_line_error_m'][arm] < 1e-9
    e.close()


def test_cartesian_playback_cancel_during_upload_never_starts():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, _ = cartesian_robot(e)
    def upload(*args):
        e.cancel_requested.set()
        return 0
    e.robot.config_set_traj = upload
    with pytest.raises(ValueError, match='cancelled'):
        e.start(arm=0, target_matrix=goal)
    assert ('run', 1) not in e.robot.calls and ('stop', 11) in e.robot.calls
    assert not e.active
    e.close()


def test_cartesian_feedback_detects_cross_track_error():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    e.c.cartesian_path_tolerance = .005
    goal, _ = cartesian_robot(e)
    original = e.robot.forward_kinematics
    def fk(arm, q):
        matrix = original(arm, q)
        matrix[1][3] = q[1]
        return matrix
    e.robot.forward_kinematics = fk
    e.start(arm=0, target_matrix=goal)
    e.robot.positions[0][1] = 6.  # 6 mm perpendicular deviation.
    with pytest.raises(RuntimeError, match='departed Cartesian line: 0.006000'):
        e.poll(e.robot.positions)
    e.stop()
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_cartesian_feedback_detects_overshoot_beyond_segment():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    e.c.cartesian_path_tolerance = .005
    goal, _ = cartesian_robot(e)
    e.start(arm=0, target_matrix=goal)
    e.robot.positions[0][0] = 8.  # Target is 1 mm; overshoot is 7 mm.
    with pytest.raises(RuntimeError, match='departed Cartesian line: 0.007000'):
        e.poll(e.robot.positions)
    e.close()


def test_trajectory_diagnostics_keep_fault_sample_and_plan_without_runtime_io(tmp_path):
    import json
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    e.c.trajectory_log_dir = str(tmp_path / 'traces')
    goal, _ = cartesian_robot(e)
    original = e.robot.forward_kinematics
    def fk(arm, q):
        matrix = original(arm, q)
        matrix[1][3] = q[1]
        return matrix
    e.robot.forward_kinematics = fk
    _, detail = e.start(arm=0, target_matrix=goal)
    e.robot.get_sg_dict = lambda: {'arms': [{'get': {'traj_state': 3},
                                             'set': {'vel_ratio': 5., 'acc_ratio': 5.}}]*2}
    assert not Path(detail['trajectory_log']).exists()
    e.robot.positions[0] = [.5, 6., 0., 0., 0., 0., 0.]
    with pytest.raises(RuntimeError, match='departed Cartesian line') as failure:
        e.poll(e.robot.positions)
    snapshot = e.last_tracking_sample['arms']['0']
    assert snapshot['line_error_vector_m'] == pytest.approx([0., .006, 0.])
    assert snapshot['traj_state'] == 3 and snapshot['controller_set']['vel_ratio'] == 5.
    diagnostic = e.trajectory_diagnostic
    e.stop()
    assert not Path(detail['trajectory_log']).exists()
    saved = diagnostic.save('fault', str(failure.value))
    data = json.loads(Path(saved).read_text())
    assert data['outcome'] == 'fault' and 'departed' in data['reason']
    assert len(data['planned_joints_deg']['0']) == detail['points'][0]
    assert data['planned_tcp_positions_m']['0'][-1] == [.001, 0., 0.]
    assert data['samples'][0]['unix_ns'] > 0 and data['samples'][0]['elapsed_s'] >= 0
    sample = data['samples'][0]['arms']['0']
    assert len(sample['nominal_error_deg']) == 7
    assert 0 <= sample['nearest_joint_vector_index'] < detail['points'][0]
    assert sample['per_joint_nearest_indices'][1] is None  # Stationary joints have no inferred progress.
    e.close()


def test_busy_rejection_does_not_flush_live_diagnostic():
    from unittest.mock import MagicMock
    from gento_dual_arm.node import Bridge
    node = SimpleNamespace(engine=SimpleNamespace(active=True, faulted=False),
                           flush_trajectory_diagnostic=MagicMock(), status=MagicMock(),
                           get_clock=MagicMock())
    def busy():
        raise ValueError('Busy')
    Bridge.command(node, busy)
    node.flush_trajectory_diagnostic.assert_not_called()
    node.status.assert_called_once_with('rejected', reason='Busy')


def test_cartesian_respects_slow_controller_joint_instead_of_only_global_limit():
    e = engine()
    e.c.cartesian_joint_speed = 20.
    e.c.cartesian_joint_acc = 25.
    e.c.max_delta = 60.
    original = e.robot.param_get_float
    def param(name):
        if name.endswith('VelMax'):
            return 0, (20. if '.L3.' in name else 500.)
        if name.endswith('AccMax'):
            return 0, 1000.
        return original(name)
    e.robot.param_get_float = param
    def fk(arm, q):
        return [[1,0,0,q[3]/2], [0,1,0,0], [0,0,1,0], [0,0,0,1]]
    def ik(arm, p):
        x = p.m_Input_IK_TargetTCP[3]
        p.m_Output_RetJoint[0] = 10*x
        p.m_Output_RetJoint[3] = 2*x
        return p
    e.robot.forward_kinematics = fk
    e.robot.inverse_kinematics = ik
    goal = fk(0, [0,0,0,2,0,0,0])
    _, detail = e.start(arm=0, target_matrix=goal)
    info = detail['cartesian_plan']
    assert info['controller_limits'][0]['speed'][3] == pytest.approx(.9)
    assert info['retime_count'] > 0
    event = info['retime_events'][0]
    assert event['joint'] == 4 and event['limit'] == pytest.approx(.9)
    # Joint1 has a larger absolute speed; joint4 has the higher normalized violation.
    assert info['peaks']['joint_speed']['joint'] == 4
    points, count = e.cartesian_stream['plans'][0]
    assert max(abs(points[i*7+3]-points[(i-1)*7+3])*50 for i in range(1,count)) <= .9
    assert e.c.cartesian_joint_speed == 20.
    e.close()


@pytest.mark.parametrize('invalid', [0., -1., float('nan')])
def test_invalid_controller_rating_rejects_cartesian_motion(invalid):
    e = engine()
    goal, _ = cartesian_robot(e)
    original = e.robot.param_get_float
    e.robot.param_get_float = lambda name: (0, invalid) if name.endswith('VelMax') else original(name)
    with pytest.raises(ValueError):
        e.start(arm=0, target_matrix=goal)
    assert not e.robot.calls
    e.close()


def test_controller_ratio_change_during_upload_prevents_start():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, _ = cartesian_robot(e)
    original = e.robot.config_set_traj
    def upload(*args):
        ret = original(*args)
        e.robot.get_sg_dict = lambda: {'arms': [{'get': {'traj_state': 0},
                                                'set': {'vel_ratio': 2.5, 'acc_ratio': 5.}}]*2}
        return ret
    e.robot.config_set_traj = upload
    with pytest.raises(RuntimeError, match='vel_ratio changed since planning'):
        e.start(arm=0, target_matrix=goal)
    assert ('run', 1) not in e.robot.calls and ('stop', 11) in e.robot.calls
    e.close()


def test_controller_ratio_change_during_playback_is_detected():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, _ = cartesian_robot(e)
    e.start(arm=0, target_matrix=goal)
    e.robot.get_sg_dict = lambda: {'arms': [{'get': {'traj_state': 3},
                                            'set': {'vel_ratio': 5., 'acc_ratio': 2.5}}]*2}
    with pytest.raises(RuntimeError, match='acc_ratio changed during Cartesian execution'):
        e.poll(e.robot.positions)
    e.stop()
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_upload_timeout_is_reported_once_and_never_starts(tmp_path):
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    e.c.trajectory_log_dir = str(tmp_path)
    goal, _ = cartesian_robot(e)
    events, calls = [], []
    e.on_upload = events.append
    def timeout(*args):
        calls.append(args)
        return -10
    e.robot.config_set_traj = timeout
    with pytest.raises(RuntimeError, match=r'upload failed: SDK=-10, points=51'):
        e.start(arm=0, target_matrix=goal)
    assert len(calls) == 1  # No blind retry of a partly uploaded trajectory.
    assert [event['phase'] for event in events] == ['begin', 'failed']
    assert events[-1]['sdk_return'] == -10 and events[-1]['after']['traj_state'] == 0
    assert e.faulted and not e.active
    assert ('run', 1) not in e.robot.calls and ('stop', 11) in e.robot.calls
    assert e.trajectory_diagnostic.data['uploads'][-1]['sdk_return'] == -10
    assert len(e.trajectory_diagnostic.data['planned_joints_deg']['0']) == 51
    e.close()


@pytest.mark.parametrize('unconfirmed', [0, 1, 3, 6])
def test_success_return_without_ready_feedback_does_not_start(unconfirmed):
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, _ = cartesian_robot(e)
    def incomplete_upload(*args):
        e.robot.traj_states[0] = unconfirmed
        return 0
    e.robot.config_set_traj = incomplete_upload
    with pytest.raises(RuntimeError, match='upload not confirmed before start'):
        e.start(arm=0, target_matrix=goal)
    assert ('run', 1) not in e.robot.calls and ('stop', 11) in e.robot.calls
    e.close()


def test_second_arm_upload_failure_never_starts_first_arm():
    e = engine()
    e.c.cartesian_execution = 'trajectory'
    goal, _ = cartesian_robot(e)
    original = e.robot.config_set_traj
    def upload(arm, points, count):
        return -10 if arm == 1 else original(arm, points, count)
    e.robot.config_set_traj = upload
    with pytest.raises(RuntimeError, match='arm1 upload failed'):
        e.start(matrices=[goal, goal])
    assert ('upload', 0) in e.robot.calls
    assert not any(call[0] == 'run' for call in e.robot.calls)
    assert ('stop', 11) in e.robot.calls
    e.close()


def test_connect_enables_sdk_warning_and_error_diagnostics():
    from gento_dual_arm.engine import SDK_LOG_MASK
    e = engine()
    captured = []
    def link(*args):
        captured.append(args)
        return 1
    e.robot.link = link
    e.connect()
    assert captured[0][-1] == SDK_LOG_MASK
    assert SDK_LOG_MASK & (1 << 2) and SDK_LOG_MASK & (1 << 3)
    e.close()


def test_console_status_is_concise_but_status_topic_keeps_diagnostics():
    import json
    from unittest.mock import MagicMock
    from gento_dual_arm.node import Bridge
    logger = MagicMock()
    node = SimpleNamespace(status_pub=MagicMock(), get_logger=lambda: logger)
    details = {'moving_arms': [0], 'joints_deg': [[0.]*7]*2,
               'cartesian_plan': {'planned_duration_s': 6.1, 'retime_events': [{'large': 'diagnostics'}]}}
    Bridge.status(node, 'running', **details)
    logger.info.assert_called_once_with('running arms=left')
    payload = json.loads(node.status_pub.publish.call_args.args[0].data)
    assert payload['joints_deg'] == details['joints_deg']
    assert payload['cartesian_plan']['retime_events'] == details['cartesian_plan']['retime_events']
    Bridge.status(node, 'fault', reason='upload timeout', tracking={'large': 'diagnostics'})
    logger.error.assert_called_once_with('fault: upload timeout')
    Bridge.status(node, 'completed', cartesian_tracking={'max_line_error_m': {'0': .0005}})
    logger.info.assert_called_with('completed')
