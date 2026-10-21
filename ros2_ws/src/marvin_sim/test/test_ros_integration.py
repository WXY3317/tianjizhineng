# 启动旧版 ROS/SDK 仿真链路并验证公开控制话题。
"""Launch the actual ROS/SDK stack and exercise public command interfaces."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from marvin_sim.kinematics import JOINT_NAMES, Kinematics, matrix_quaternion, pose_error, quaternion_matrix


ROOT = Path(__file__).resolve().parents[4]


def test_ros_sdk_joint_and_pose_control(tmp_path):
    context = Context()
    # Isolate DDS traffic from other robot applications in this workspace.
    rclpy.init(context=context, domain_id=91)
    executor = SingleThreadedExecutor(context=context)
    env = dict(os.environ, ROS_DOMAIN_ID='91', PYTHONUNBUFFERED='1')
    log_path = tmp_path/'launch.log'
    node, process = None, None
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen([
                'ros2', 'launch', 'marvin_sim', 'simulation.launch.py',
                f'sdk_root:={ROOT / "tianji_armSDK"}', 'rviz:=false', 'panel:=false',
            ], stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
            node = Node('marvin_sim_integration_probe', context=context)
            executor.add_node(node)
            feedback, model, statuses, poses = [], [], [], {}
            node.create_subscription(JointState, '/marvin_sim/sdk_joint_states',
                                     lambda m: feedback.append(np.array(m.position)), 10)
            node.create_subscription(JointState, '/joint_states',
                                     lambda m: model.append(np.array(m.position)), 10)
            node.create_subscription(String, '/marvin_sim/status',
                                     lambda m: statuses.append(json.loads(m.data)), 10)
            for arm, side in enumerate(('left', 'right')):
                node.create_subscription(PoseStamped, f'/marvin_sim/{side}/tcp_pose',
                                         lambda m, a=arm: poses.update({a: m}), 10)
            target_pub = node.create_publisher(JointState, '/marvin_sim/target_joints', 10)
            pose_pubs = [node.create_publisher(PoseStamped, f'/marvin_sim/{s}/target_pose', 10)
                         for s in ('left', 'right')]

            def wait_for(condition, timeout=12.):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    assert process.poll() is None, log_path.read_text()
                    executor.spin_once(timeout_sec=.03)
                    if condition():
                        return
                raise AssertionError(f'ROS condition timed out; statuses={statuses[-4:]}\n{log_path.read_text()}')

            wait_for(lambda: feedback and model and len(poses) == 2
                     and target_pub.get_subscription_count() and statuses)
            baseline = feedback[-1].copy()
            goal = baseline.copy()
            goal[0] += .04
            goal[7] -= .03
            # Reordered joint names must be mapped, not interpreted by position.
            msg = JointState()
            msg.name, msg.position = JOINT_NAMES[::-1], goal[::-1].tolist()
            target_pub.publish(msg)
            wait_for(lambda: np.max(np.abs(feedback[-1]-goal)) < 1e-6
                     and np.max(np.abs(model[-1]-goal)) < 1e-6)

            kine = Kinematics(ROOT/'tianji_armSDK', ROOT/'marvin_description')
            for arm in range(2):
                reference = feedback[-1][7*arm:7*arm+7]
                requested = reference.copy()
                requested[0] += .02
                target = kine.fk(arm, requested)
                pose = PoseStamped()
                pose.header.frame_id = ('Base_L', 'Base_R')[arm]
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = target[:3, 3].tolist()
                quaternion = matrix_quaternion(target[:3, :3])
                (pose.pose.orientation.x, pose.pose.orientation.y,
                 pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
                pose_pubs[arm].publish(pose)

                def arrived():
                    actual = kine.fk(arm, feedback[-1][7*arm:7*arm+7])
                    dp, dr = pose_error(actual, target)
                    return dp < 1e-5 and dr < 1e-4

                wait_for(arrived)
                # The public FK pose must also agree with the vendor calculation.
                def reported_pose_arrived():
                    reported = poses[arm]
                    assert reported.header.frame_id == ('Base_L', 'Base_R')[arm]
                    actual = np.eye(4)
                    p, q = reported.pose.position, reported.pose.orientation
                    actual[:3, 3] = [p.x, p.y, p.z]
                    actual[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
                    dp, dr = pose_error(actual, target)
                    return dp < 1e-5 and dr < 1e-4

                wait_for(reported_pose_arrived)

            wait_for(lambda: statuses[-1].get('state') == 'completed')
            unchanged = feedback[-1].copy()
            bad = PoseStamped()
            bad.header.frame_id = 'Base_L'
            bad.pose.position.x = 10.
            bad.pose.orientation.w = 1.
            count = len(statuses)
            pose_pubs[0].publish(bad)
            wait_for(lambda: any('error' in s for s in statuses[count:]))
            assert np.max(np.abs(feedback[-1]-unchanged)) < 1e-6
            assert 'IK' in statuses[-1].get('error', '')

            for name in ('stop', 'enable'):
                client = node.create_client(Trigger, f'/marvin_sim/{name}')
                assert client.wait_for_service(timeout_sec=3.)
                future = client.call_async(Trigger.Request())
                wait_for(future.done)
                assert future.result().success, future.result().message
            wait_for(lambda: statuses[-1]['state'] == 'ready')
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            if node is not None:
                node.destroy_node()
            executor.shutdown()
            context.shutdown()
