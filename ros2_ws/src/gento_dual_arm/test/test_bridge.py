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
    def get_sg_dict(self): return {'arms': [{'get': {'traj_state': 0}}] * 2}
    def forward_kinematics(self, arm, q): return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    def plan_joints(self, arm, start, target, *args): return start + target, 2
    def switch_to_position_mode(self, arm, *args):
        self.calls.append(('position', arm))
        self.states[arm] = 1
        return 0
    def config_set_traj(self, arm, points, count):
        self.calls.append(('upload', arm))
        self.targets[arm] = points[-7:]
        return 0
    def runtime_set_joint_pos_cmd(self, obj, positions):
        self.calls.append(('body_command', obj))
        self.body_positions = list(positions)
        return 0
    def runtime_run_traj(self, mask):
        self.calls.append(('run', mask))
        for arm in (0, 1):
            if mask & (1 << arm):
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
    config = SimpleNamespace(ip='127.0.0.1', speed=5., acc=5., plan_speed=.05,
                             plan_acc=.05, max_delta=5., timeout=60., tolerance=.2, sdk_length_to_m=.001)
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
    with pytest.raises(ValueError): e.start(matrices=[matrix, matrix])
    assert not e.robot.calls
    e.close()


def test_pose_targets_use_ik():
    e = engine()
    def solve(arm, p):
        for i in range(7): p.m_Output_RetJoint[i] = 1.
        return p
    e.robot.inverse_kinematics = solve
    matrix = e.robot.forward_kinematics(0, [0] * 7)
    state, detail = e.start(matrices=[matrix, matrix])
    assert state == 'running'
    assert detail['joints_deg'] == [[1.] * 7] * 2
    e.close()


def test_ros_topics_and_stamps():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from gento_dual_arm.node import Bridge
    import time
    rclpy.init(args=['--ros-args', '-p', f'sdk_root:={ROOT}', '-p', 'ip:=127.0.0.1'], domain_id=231)
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
            return p  # zero target and fake identity FK
        fake.inverse_kinematics = solve
        target = [PoseStamped(), PoseStamped()]
        stamp = client.get_clock().now().to_msg()
        for arm, pose in enumerate(target):
            pose.header.stamp = stamp
            pose.header.frame_id = bridge.frames[arm]
            pose.pose.orientation.w = 1.
        old_count = sum('running' in s for s in statuses)
        pose_pubs[0].publish(target[0])
        until = time.monotonic() + .15
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert sum('running' in s for s in statuses) == old_count
        pose_pubs[1].publish(target[1])
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and sum('running' in s for s in statuses) == old_count:
            executor.spin_once(timeout_sec=.05)
        assert sum('running' in s for s in statuses) > old_count
        deadline = time.monotonic() + 5
        while bridge.engine.active and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert not bridge.engine.active
        # Different timestamps never mix into one motion. Latest pair replaces old.
        with patch.object(bridge.engine, 'start', return_value=('running', {})) as start:
            first, second = PoseStamped(), PoseStamped()
            first.header.frame_id, second.header.frame_id = bridge.frames
            first.pose.orientation.w = second.pose.orientation.w = 1.
            first.header.stamp = client.get_clock().now().to_msg()
            bridge.on_pose(0, first)
            second.header.stamp = client.get_clock().now().to_msg()
            bridge.on_pose(1, second)
            start.assert_not_called()
            first.header.stamp = second.header.stamp
            bridge.on_pose(0, first)  # Right-first pairing works too.
            start.assert_called_once()
            assert not bridge.pending_poses
        # An unmatched target expires and cannot pair with later commands.
        first.header.stamp = client.get_clock().now().to_msg()
        bridge.on_pose(0, first)
        bridge.c.command_max_age = .001
        time.sleep(.005)
        bridge.tick()
        assert not bridge.pending_poses
        bridge.c.command_max_age = 1.
        # Zero/old timestamps cannot cause movement.
        with pytest.raises(ValueError): bridge.stamp_check(JointState().header.stamp)
        assert sum(call == ('run', 1) for call in fake.calls) == 2
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
        assert fake.positions[0] == [0.] * 7
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
        bridge.pending_poses[0] = Pose()
        fake.reset_error = original
        bridge.on_reset_error(Empty())
        assert not bridge.engine.faulted and not bridge.timer.is_canceled()
        assert not bridge.pending_poses
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
