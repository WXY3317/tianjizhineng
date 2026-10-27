# ROS 节点入口：读取启动参数，连接控制模块，订阅目标并发布状态和反馈。
import ipaddress
import json
import math
from pathlib import Path
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, String

from .conversions import BODY_JOINT_NAMES, JOINT_NAMES, body_target, finite, joint_target, pose_matrix, quaternion
from .diagnostics import joint_errors
from .engine import Engine
from .native_ik import bind_native_ik

DEFAULTS = {
    'sdk_root': '', 'ip': '',
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
        from .sdk_loader import load_sdk
        sdk = load_sdk(root)
        self.error_names, self.state_names = sdk.error_dict, sdk.state_map
        from . import dual_arm_control as h
        robot = sdk.GentoRobot()
        self.engine = Engine(robot, h, bind_native_ik(robot, sdk), self.c)
        startup_error = None
        self.reset_request_count = 0
        self.get_logger().info(f'Startup recovery: connecting to {self.c.ip}; source=startup')
        try:
            self.engine.connect()
        except Exception as exc:
            if not self.engine.linked:
                raise
            startup_error = str(exc)
        self.frames = [self.c.left_frame, self.c.right_frame]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.joints_pub = self.create_publisher(JointState, '~/joint_states', qos)
        self.body_pub = self.create_publisher(JointState, '~/body/joint_states', qos)
        self.head_pub = self.create_publisher(JointState, '~/head/joint_states', qos)
        self.full_pub = self.create_publisher(JointState, '~/full_joint_states', qos)
        self.pose_pubs = [self.create_publisher(PoseStamped, f'~/{side}/tcp_pose', qos) for side in ('left', 'right')]
        self.status_pub = self.create_publisher(String, '~/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.errors_pub = self.create_publisher(String, '~/joint_errors', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '~/body/target_joints', self.on_body, qos)
        self.pending_poses = {}
        self.pose_pair_stamp = None
        for arm, side in enumerate(('left', 'right')):
            self.create_subscription(JointState, f'~/{side}/target_joints',
                                     lambda msg, a=arm: self.on_joints(a, msg), qos)
            self.create_subscription(PoseStamped, f'~/{side}/target_pose',
                                     lambda msg, a=arm: self.on_pose(a, msg), qos)
        self.create_subscription(Empty, '~/stop', self.on_stop, qos)
        self.create_subscription(Empty, '~/reset_error', self.on_reset_error, qos)
        for obj, side in ((0, 'left'), (1, 'right'), (3, 'body')):
            self.create_subscription(Empty, f'~/{side}/reset_error',
                                     lambda msg, o=obj: self.on_object_reset(o, msg), qos)
        self.accept_after = self.get_clock().now().nanoseconds
        self.timer = self.create_timer(1/self.c.publish_rate, self.tick)
        self.errors_timer = self.create_timer(1.0, self.publish_errors)
        if startup_error is None:
            self.status('ready', source='startup')
        else:
            self.timer.cancel()
            self.status('fault', reason=startup_error, recovery_topic='~/reset_error',
                        source='startup', recovery_stage=self.engine.recovery_stage,
                        stop_requested=self.engine.recovery_stop_requested)

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

    def command(self, convert, body=False):
        try:
            kwargs = convert()
            state, details = (self.engine.start_body(**kwargs) if body else self.engine.start(**kwargs))
            self.status(state, **details)
        except Exception as exc:
            self.status('fault' if self.engine.faulted else 'rejected', reason=str(exc))
        finally:
            # Drop commands received during planning/upload, even if DDS delivers them later.
            self.accept_after = self.get_clock().now().nanoseconds
            self.clear_pose_pair()

    def on_joints(self, arm, msg):
        def convert():
            self.stamp_check(msg.header.stamp)
            if msg.velocity or msg.effort:
                raise ValueError('Only absolute positions accepted; velocity/effort must be empty')
            return {'arm': arm, 'target': joint_target(arm, msg.name, msg.position)}
        self.command(convert)

    def on_body(self, msg):
        def convert():
            self.stamp_check(msg.header.stamp)
            if msg.velocity or msg.effort:
                raise ValueError('Only absolute positions accepted; velocity/effort must be empty')
            return {'target': body_target(msg.name, msg.position)}
        self.command(convert, body=True)

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
            self.engine.stop(force=True)
            self.status('stopped', reason='Stop latched; clear cause then request reset_error or restart')
        except Exception as exc:
            self.status('fault', reason=str(exc))
        self.accept_after = self.get_clock().now().nanoseconds

    def on_object_reset(self, obj, msg):
        self.reset_request_count += 1
        request_id = self.reset_request_count
        group = {0: 'left', 1: 'right', 3: 'body'}[obj]
        self.get_logger().info(f'Received {group}/reset_error request #{request_id}')
        if self.engine.active:
            self.status('rejected', source='object_reset_topic', request_id=request_id,
                        group=group, reason='Motion active; stop before requesting error reset')
            return
        self.clear_pose_pair()
        self.timer.cancel()
        try:
            details = self.engine.reset_object_error(obj)
            self.status('reset_completed', source='object_reset_topic', request_id=request_id,
                        group=group, **details,
                        reason='Object is IDLE; motion remains locked. Use reset_error for full recovery')
        except Exception as exc:
            self.status('fault', source='object_reset_topic', request_id=request_id,
                        group=group, reason=str(exc))
        finally:
            self.accept_after = self.get_clock().now().nanoseconds

    def on_reset_error(self, msg):
        self.reset_request_count += 1
        request_id = self.reset_request_count
        self.get_logger().info(f'Received reset_error topic request #{request_id}')
        if self.engine.active:
            self.status('rejected', reason='Motion active; stop before requesting error reset',
                        source='reset_error_topic', request_id=request_id)
            return
        self.clear_pose_pair()
        self.timer.cancel()
        try:
            self.engine.reset_errors()
            self.timer.reset()
            self.status('ready', reason='Errors reset; POSITION confirmed; send a fresh target',
                        source='reset_error_topic', request_id=request_id)
        except Exception as exc:
            self.status('fault', reason=str(exc), source='reset_error_topic', request_id=request_id,
                        recovery_stage=self.engine.recovery_stage,
                        stop_requested=self.engine.recovery_stop_requested)
        finally:
            self.accept_after = self.get_clock().now().nanoseconds

    def publish_errors(self):
        try:
            payload = joint_errors(self.engine.robot, self.error_names, self.state_names,
                                   motion_active=self.engine.active)
        except Exception as exc:
            payload = {'available': False, 'reason': str(exc), 'objects': []}
        stamp = self.get_clock().now().to_msg()
        payload['stamp'] = {'sec': stamp.sec, 'nanosec': stamp.nanosec}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.errors_pub.publish(msg)

    def tick(self):
        if self.pose_pair_stamp is not None:
            if self.get_clock().now().nanoseconds - self.pose_pair_stamp > self.c.command_max_age * 1e9:
                self.clear_pose_pair()
                self.status('rejected', reason='Unpaired pose target expired; resend both arms')
        try:
            data = self.engine.robot.get_rt_dict()
            joints, matrices = self.engine.sample(data)
            body_joints = self.engine.sample_body(data)
            head_joints = self.engine.sample_head(data)
            stamp = self.get_clock().now().to_msg()
            js = JointState()
            js.header.stamp = stamp
            js.name = JOINT_NAMES
            js.position = [math.radians(v) for q in joints for v in q]
            self.joints_pub.publish(js)
            body = JointState()
            body.header.stamp = stamp
            body.name = BODY_JOINT_NAMES
            body.position = [math.radians(v) for v in body_joints]
            self.body_pub.publish(body)
            head = JointState()
            head.header.stamp = stamp
            head.name = [f'head_joint_{i + 1}' for i in range(len(head_joints))]
            head.position = [math.radians(v) for v in head_joints]
            self.head_pub.publish(head)
            full = JointState()
            full.header.stamp = stamp
            full.name = head.name + body.name + js.name
            full.position = head.position + body.position + js.position
            self.full_pub.publish(full)
            for arm, matrix in enumerate(matrices):
                msg = PoseStamped()
                msg.header.stamp, msg.header.frame_id = stamp, self.frames[arm]
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = [matrix[i][3]*self.c.sdk_length_to_m for i in range(3)]
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion(matrix)
                self.pose_pubs[arm].publish(msg)
            group = self.engine.motion_group
            if self.engine.poll(joints, body_joints):
                self.accept_after = self.get_clock().now().nanoseconds
                self.status('completed', group=group)
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
            for timer in (node.timer, node.errors_timer):
                timer.cancel()
            try:
                node.engine.close()
            except Exception as exc:
                node.get_logger().error(f'Shutdown could not confirm safe state: {exc}')
            finally:
                node.destroy_node()
        executor.shutdown()
        if rclpy.ok(): rclpy.shutdown()
