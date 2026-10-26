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
        self.states = [0, 0]
        self.targets = {}
        self.start_mask = 3

    def link(self, *args): return 1
    def check_link_state(self): return 1
    def init_single_arm_config(self, arm): return 0
    def param_get_float(self, name): return 0, (-180 if name.endswith('Neg') else 180)
    def current_state(self, arm): return self.states[arm]
    def get_rt_dict(self): return {'arms': [{'fb': {'fb_pos': q}} for q in self.positions]}
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
    def runtime_run_traj(self, mask):
        self.calls.append(('run', mask))
        self.positions = [self.targets[0], self.targets[1]]
        return self.start_mask
    def switch_to_idle(self, arm, *args):
        self.calls.append(('idle', arm))
        self.states[arm] = 0
        return 0
    def emergency_stop(self, mask):
        self.calls.append(('stop', mask))
        return mask
    def cleanup(self): self.calls.append('cleanup')
    def unlink(self): self.calls.append('unlink')


def engine(execute=True):
    config = SimpleNamespace(ip='127.0.0.1', execute=execute, speed=5., acc=5., plan_speed=.05,
                             plan_acc=.05, max_delta=5., timeout=60., tolerance=.2, sdk_length_to_m=.001)
    e = Engine(DualRobot(), helpers, FX_InvKineSolverParams, config)
    e.connect()
    return e


def test_reorder_and_units():
    targets = joint_target(JOINT_NAMES[::-1], [math.pi/180] * 14)
    assert targets == [[1.] * 7] * 2
    with pytest.raises(ValueError): joint_target(JOINT_NAMES[:7] * 2, [0.] * 14)
    with pytest.raises(ValueError): joint_target(JOINT_NAMES, [float('nan')] * 14)


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


def test_preview_no_upload():
    e = engine(False)
    assert e.start(targets=[[1.] * 7] * 2)[0] == 'preview'
    assert e.robot.calls == []
    e.close()
    assert e.robot.calls == ['cleanup', 'unlink']


def test_start_poll_and_busy_rejection():
    e = engine()
    e.start(targets=[[1.] * 7] * 2)
    with pytest.raises(RuntimeError): e.start(targets=[[2.] * 7] * 2)
    q, _ = e.sample()
    with patch('gento_dual_arm.engine.time.monotonic', side_effect=[0, 0, 1, 1]):
        assert not e.poll(q)
        assert e.poll(q)
    assert not e.active
    assert e.robot.calls[-2:] == [('idle', 0), ('idle', 1)]
    e.close()


def test_partial_start_latches_fault():
    e = engine()
    e.robot.start_mask = 1
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    assert e.faulted
    assert ('stop', 3) in e.robot.calls
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
    e = engine(False)
    def solve(arm, p):
        for i in range(7): p.m_Output_RetJoint[i] = 1.
        return p
    e.robot.inverse_kinematics = solve
    matrix = e.robot.forward_kinematics(0, [0] * 7)
    state, detail = e.start(matrices=[matrix, matrix])
    assert state == 'preview'
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
    rclpy.init(args=['--ros-args', '-p', f'sdk_root:={ROOT}', '-p', 'ip:=127.0.0.1'])
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
        client.create_subscription(JointState, '/gento_dual_arm/joint_states', feedback.append, 10)
        client.create_subscription(PoseStamped, '/gento_dual_arm/left/tcp_pose', poses.append, 10)
        client.create_subscription(String, '/gento_dual_arm/status', lambda m: statuses.append(m.data), 10)
        pub = client.create_publisher(JointState, '/gento_dual_arm/target_joints', 1)
        pose_pubs = [client.create_publisher(PoseStamped, f'/gento_dual_arm/{side}/target_pose', 1)
                     for side in ('left', 'right')]
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and (not feedback or not poses or not pub.get_subscription_count()):
            executor.spin_once(timeout_sec=.05)
        assert feedback and poses
        assert feedback[-1].name == JOINT_NAMES
        assert poses[-1].header.frame_id == 'left_arm_sdk_base'
        msg = JointState()
        msg.name, msg.position = JOINT_NAMES, [math.radians(1)] * 14
        msg.header.stamp = client.get_clock().now().to_msg()
        pub.publish(msg)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not any('preview' in s for s in statuses):
            executor.spin_once(timeout_sec=.05)
        assert any('preview' in s for s in statuses)
        assert fake.calls == []
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
        old_count = sum('preview' in s for s in statuses)
        pose_pubs[0].publish(target[0])
        until = time.monotonic() + .15
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.01)
        assert sum('preview' in s for s in statuses) == old_count
        pose_pubs[1].publish(target[1])
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and sum('preview' in s for s in statuses) == old_count:
            executor.spin_once(timeout_sec=.05)
        assert sum('preview' in s for s in statuses) > old_count
        # Different timestamps never mix into one motion. Latest pair replaces old.
        with patch.object(bridge.engine, 'start', return_value=('preview', {})) as start:
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
        assert fake.calls == []
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
    assert e.robot.calls[-3:] == [('stop', 3), 'cleanup', 'unlink']


def test_upload_failure_prevents_start():
    e = engine()
    e.robot.config_set_traj = lambda *args: -1
    with pytest.raises(RuntimeError): e.start(targets=[[1.] * 7] * 2)
    assert ('run', 3) not in e.robot.calls
    assert ('stop', 3) in e.robot.calls
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
