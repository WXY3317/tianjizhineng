# ROS 节点入口：读取启动参数，连接控制模块，订阅目标并发布状态和反馈。
import importlib
import ipaddress
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from .conversions import JOINT_NAMES, finite, joint_target, pose_matrix, quaternion
from .engine import Engine
from .native_ik import bind_native_ik

DEFAULTS = {
    'sdk_root': '', 'ip': '', 'execute': False,
    'speed': 5.0, 'acc': 5.0, 'plan_speed': 0.05, 'plan_acc': 0.05,
    'max_delta': 5.0, 'timeout': 60.0, 'tolerance': 0.2,
    'publish_rate': 20.0, 'command_max_age': 1.0,
    'sdk_length_to_m': 0.001,
    'left_frame': 'left_arm_sdk_base', 'right_frame': 'right_arm_sdk_base',
}


class Bridge(Node):
    def __init__(self):
        super().__init__('gento_dual_arm')
        self.engine = None
        try:
            self.setup()
        except BaseException:
            if self.engine is not None:
                try: self.engine.close()
                except Exception as exc: self.get_logger().error(str(exc))
            self.destroy_node()
            raise

    def setup(self):
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.c = SimpleNamespace(**{k: self.get_parameter(k).value for k in DEFAULTS})
        self.c.ip = str(ipaddress.IPv4Address(self.c.ip))
        numeric = [self.c.speed, self.c.acc, self.c.plan_speed, self.c.plan_acc, self.c.max_delta,
                   self.c.timeout, self.c.tolerance, self.c.publish_rate, self.c.command_max_age, self.c.sdk_length_to_m]
        finite(numeric)
        if min(numeric) <= 0 or max(self.c.speed, self.c.acc) > 10 or not all(0.01 <= x <= 0.1 for x in (self.c.plan_speed, self.c.plan_acc)):
            raise ValueError('Positive parameters required; speed/acc <=10%, plan ratios in [0.01,0.1]')
        if self.c.publish_rate > 100 or not self.c.left_frame or not self.c.right_frame or self.c.left_frame == self.c.right_frame:
            raise ValueError('publish_rate <=100; distinct, nonempty arm base frames required')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('use_sim_time is not supported for this real hardware node')
        root = Path(self.c.sdk_root).expanduser().resolve()
        helpers_dir = next((p for p in (root.parent/'ros2_ws/real_test', root/'real_test', root.parent/'real_test')
                            if (p/'dual_arm_control.py').is_file()), None)
        if helpers_dir is None or not (root/'PYTHON_SDK/GentoRobot.py').is_file():
            raise ValueError('sdk_root must contain PYTHON_SDK/GentoRobot.py and ros2_ws/real_test must be available')
        sys.path.insert(0, str(root))
        sys.path.insert(0, str(helpers_dir))
        h = importlib.import_module('dual_arm_control')
        sdk = importlib.import_module('PYTHON_SDK.GentoRobot')
        robot = sdk.GentoRobot()
        self.engine = Engine(robot, h, bind_native_ik(robot, sdk), self.c)
        self.engine.connect()
        self.frames = [self.c.left_frame, self.c.right_frame]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.joints_pub = self.create_publisher(JointState, '~/joint_states', qos)
        self.pose_pubs = [self.create_publisher(PoseStamped, f'~/{side}/tcp_pose', qos) for side in ('left', 'right')]
        self.status_pub = self.create_publisher(String, '~/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '~/target_joints', self.on_joints, qos)
        self.pending_poses = {}
        self.pose_pair_stamp = None
        for arm, side in enumerate(('left', 'right')):
            self.create_subscription(PoseStamped, f'~/{side}/target_pose',
                                     lambda msg, a=arm: self.on_pose(a, msg), qos)
        self.create_subscription(Empty, '~/stop', self.on_stop, qos)
        self.accept_after = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(1/self.c.publish_rate, self.tick)
        self.status('ready', execute=self.c.execute)

    def status(self, state, **details):
        msg = String()
        msg.data = json.dumps({'state': state, **details}, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def stamp_check(self, stamp):
        value = stamp.sec*1_000_000_000 + stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if value <= self.accept_after or value > now + 50_000_000 or now-value > self.c.command_max_age*1e9:
            raise ValueError('Stale/queued/future command stamp; publish a fresh stamped target after ready/completed')
        if self.engine.active or self.engine.faulted:
            raise ValueError('Busy or faulted; commands are not queued')

    def command(self, convert):
        try:
            kwargs = convert()
            state, details = self.engine.start(**kwargs)
            self.status(state, **details)
        except Exception as exc:
            self.status('fault' if self.engine.faulted else 'rejected', reason=str(exc))
        finally:
            # Drop commands received during planning/upload, even if DDS delivers them later.
            self.accept_after = self.get_clock().now().nanoseconds
            self.clear_pose_pair()

    def on_joints(self, msg):
        def convert():
            self.stamp_check(msg.header.stamp)
            if msg.velocity or msg.effort:
                raise ValueError('Only absolute positions accepted; velocity/effort must be empty')
            return {'targets': joint_target(msg.name, msg.position)}
        self.command(convert)

    def clear_pose_pair(self):
        self.pending_poses = {}
        self.pose_pair_stamp = None

    def on_pose(self, arm, msg):
        try:
            self.stamp_check(msg.header.stamp)
            if msg.header.frame_id != self.frames[arm]:
                raise ValueError(f'Expected frame {self.frames[arm]}, no TF conversion is performed')
            pose_matrix(msg.pose, self.c.sdk_length_to_m)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if self.pose_pair_stamp is not None and stamp < self.pose_pair_stamp:
                raise ValueError('Older pose target rejected; send both arms with an identical fresh timestamp')
            if stamp != self.pose_pair_stamp:
                self.clear_pose_pair()
                self.pose_pair_stamp = stamp
            self.pending_poses[arm] = msg
            if len(self.pending_poses) < 2:
                self.status('waiting_for_pair', received_arm=arm, stamp_ns=stamp)
                return
            pair = [self.pending_poses[a] for a in (0, 1)]
            self.clear_pose_pair()
        except Exception as exc:
            self.clear_pose_pair()
            self.status('rejected', reason=str(exc))
            return

        def convert():
            # Revalidate freshness of both messages before planning.
            for pose in pair:
                self.stamp_check(pose.header.stamp)
            return {'matrices': [pose_matrix(p.pose, self.c.sdk_length_to_m) for p in pair]}
        self.command(convert)

    def on_stop(self, msg):
        self.clear_pose_pair()
        try:
            self.engine.stop(force=self.c.execute)
            self.status('stopped', reason='Stop latched; restart node after operator recovery')
        except Exception as exc:
            self.status('fault', reason=str(exc))
        self.accept_after = self.get_clock().now().nanoseconds

    def tick(self):
        if self.pose_pair_stamp is not None:
            if self.get_clock().now().nanoseconds - self.pose_pair_stamp > self.c.command_max_age * 1e9:
                self.clear_pose_pair()
                self.status('rejected', reason='Unpaired pose target expired; resend both arms')
        try:
            joints, matrices = self.engine.sample()
            stamp = self.get_clock().now().to_msg()
            js = JointState()
            js.header.stamp = stamp
            js.name = JOINT_NAMES
            js.position = [math.radians(v) for q in joints for v in q]
            self.joints_pub.publish(js)
            for arm, matrix in enumerate(matrices):
                msg = PoseStamped()
                msg.header.stamp, msg.header.frame_id = stamp, self.frames[arm]
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [matrix[i][3]*self.c.sdk_length_to_m for i in range(3)]
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion(matrix)
                self.pose_pubs[arm].publish(msg)
            if self.engine.poll(joints):
                self.accept_after = self.get_clock().now().nanoseconds
                self.status('completed')
        except Exception as exc:
            self.timer.cancel()  # Do not publish stale feedback or auto-recover a broken link.
            try: self.engine.stop()
            except Exception as stop_exc: self.get_logger().error(str(stop_exc))
            self.status('fault', reason=str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = Bridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try: node.engine.close()
            finally: node.destroy_node()
        executor.shutdown()
        if rclpy.ok(): rclpy.shutdown()
