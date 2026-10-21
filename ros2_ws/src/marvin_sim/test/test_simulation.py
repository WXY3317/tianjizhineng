# 通过原版动态库验证旧版仿真协议和运动学，不连接实体机器人。
"""Integration checks against the actual vendor shared libraries (no robot)."""
import importlib.util
from pathlib import Path
import struct
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from marvin_sim.client import SDKClient
from marvin_sim.controller import VirtualController, decode_packet
from marvin_sim.kinematics import (
    Kinematics, INITIAL_DEG, pose_error, matrix_quaternion, quaternion_matrix,
)


ROOT = Path(__file__).resolve().parents[4]


@pytest.fixture(scope='module')
def kine():
    return Kinematics(ROOT/'tianji_armSDK', ROOT/'marvin_description')


def packet(*commands):
    data = bytearray()
    for instruction, payload in commands:
        head = bytes([instruction])+len(payload).to_bytes(2, 'big')
        data.extend(head+bytes([(-sum(head)) % 256, (-sum(payload)) % 256])+payload)
    head = b'FY'+bytes([len(commands)])+len(data).to_bytes(2, 'little')
    return head+bytes([(-sum(head)) % 256, sum(data) % 256])+data


@pytest.mark.parametrize('arm', [0, 1])
def test_native_sdk_fk_ik_roundtrip(kine, arm):
    reference = np.radians(INITIAL_DEG[arm])
    for offset in (0., .02, -.03):
        target_q = reference.copy()
        target_q[0] += offset
        target = kine.fk(arm, target_q)
        solved = kine.ik(arm, target, reference)
        position, rotation = pose_error(kine.fk(arm, solved), target)
        assert position < 1e-5
        assert rotation < 1e-4


def test_native_sdk_agrees_with_actual_urdf(kine):
    source = ROOT/'marvin_description/test/test_marvin_description.py'
    spec = importlib.util.spec_from_file_location('model_helpers', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    tree = ET.fromstring(xacro.process_file(
        str(ROOT/'marvin_description/urdf/marvin.urdf.xacro'),
        mappings={'ros2_control': 'false'}).toxml())
    for arm, side in enumerate(('L', 'R')):
        for degrees in (INITIAL_DEG[arm], [0.]*7, [-25.,35.,-45.,-30.,40.,-20.,25.]):
            urdf = np.array(module._urdf_fk(tree, f'Base_{side}', f'flange_{side}', degrees))
            native = kine.fk(arm, np.radians(degrees))
            position, rotation = pose_error(urdf, native)
            assert position < 5e-6
            assert rotation < 3e-5


def test_unreachable_and_invalid_inputs(kine):
    reference = np.radians(INITIAL_DEG[0])
    target = kine.fk(0, reference)
    target[0, 3] = 10.
    with pytest.raises(ValueError):
        kine.ik(0, target, reference)
    for q in ([float('nan')]*7, [float('inf')]*7, [10.]*7, [0.]*6):
        with pytest.raises(ValueError):
            kine.fk(0, q)
    target[:3, :3] = 0
    with pytest.raises(ValueError):
        kine.ik(0, target, reference)


def test_quaternion_conversion_at_half_turn():
    for q in ([1.,0,0,0], [0,1.,0,0], [0,0,1.,0], [.2,.3,.4,.5]):
        rotation = quaternion_matrix(q)
        assert np.allclose(quaternion_matrix(matrix_quaternion(rotation)), rotation)
    with pytest.raises(ValueError):
        quaternion_matrix([0,0,0,0])


def test_protocol_checksums_and_atomic_rejection(kine):
    controller = VirtualController(kine, port=0)
    try:
        enable = (101, struct.pack('<i', 1))
        good = packet(enable)
        assert decode_packet(good)[0][0] == 101
        damaged = bytearray(good)
        damaged[-1] ^= 1
        with pytest.raises(ValueError):
            controller.handle_packet(bytes(damaged))
        with pytest.raises(ValueError):
            controller.handle_packet(packet(enable, (201, struct.pack('<i', 3))))
        assert controller.modes == [0, 0]
        with pytest.raises(ValueError):
            controller.handle_packet(packet(enable, (108, struct.pack('<i7f', 1, *([999.]*7)))))
        assert controller.modes == [0, 0]
        assert controller.received == 0
    finally:
        controller.close()


def test_interpolation_limits_and_disable(kine):
    clock = [0.]
    controller = VirtualController(kine, port=0, now=lambda: clock[0])
    try:
        start = controller.q[:7].copy()
        goal = start.copy()
        goal[0] += .5
        controller.handle_packet(packet((101, struct.pack('<i', 1)),
                                        (108, struct.pack('<i7f', 1, *np.degrees(goal)))))
        duration = controller.moves[0][1]
        previous_v = np.zeros(7)
        for point in range(1, 1001):
            clock[0] = duration*point/1000
            controller.tick()
            assert np.all(np.abs(controller.v[:7]) <= kine.velocity[:7]*.1+1e-7)
            acceleration = (controller.v[:7]-previous_v)/(duration/1000)
            assert np.all(np.abs(acceleration) <= kine.acceleration[:7]*.1+1e-5)
            previous_v = controller.v[:7].copy()
        assert np.max(np.abs(controller.q[:7]-goal)) < 1e-7
        controller.handle_packet(packet((101, struct.pack('<i', 0))))
        assert controller.moves[0] is None
    finally:
        controller.close()


def test_original_control_library_udp_feedback_both_arms(kine):
    controller = VirtualController(kine)
    stop = threading.Event()
    failures = []

    def run():
        try:
            while not stop.wait(.002):
                controller.tick()
        except BaseException as exc:
            failures.append(exc)

    thread = threading.Thread(target=run)
    thread.start()
    client = None
    try:
        client = SDKClient(kine)
        before = client.read()
        goals = {a: before[7*a:7*a+7].copy() for a in range(2)}
        goals[0][0] += .04
        goals[1][0] -= .03
        client.move(goals)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            actual = client.read()
            if max(np.max(np.abs(actual[7*a:7*a+7]-goals[a])) for a in range(2)) < 1e-6:
                break
            time.sleep(.01)
        else:
            pytest.fail('Original SDK did not receive the simulated joint target')
        assert controller.received >= 8
        assert controller.rejected == 0
        interrupted_goal = goals[0].copy()
        interrupted_goal[0] += .3
        client.move({0: interrupted_goal})
        time.sleep(.04)
        client.stop()
        client.read()
        assert [s.m_CurState for s in client.feedback.m_State] == [0, 0]
        stopped = client.read()
        assert np.max(np.abs(stopped[:7]-interrupted_goal)) > .1
        time.sleep(.04)
        assert np.array_equal(stopped, client.read())
        client.enable()
        client.read()
        assert [s.m_CurState for s in client.feedback.m_State] == [1, 1]
    finally:
        if client:
            client.close()
        stop.set()
        thread.join(timeout=2)
        controller.close()
    assert not failures
