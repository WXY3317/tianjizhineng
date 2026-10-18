import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,String
from std_srvs.srv import Trigger

from .client import SDKClient
from .controller import VirtualController
from .kinematics import JOINT_NAMES, Kinematics, matrix_quaternion, quaternion_matrix


def make_pose(matrix, frame, stamp):
    msg = PoseStamped()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = matrix[:3, 3].tolist()
    q = matrix_quaternion(matrix[:3, :3])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
    return msg


def make_rpy_pose(message,frame,stamp):
    """Convert [x,y,z,roll,pitch,yaw] (m/rad) to a PoseStamped."""
    if len(message.data)!=6 or not all(math.isfinite(value) for value in message.data):
        raise ValueError('RPY target must contain six finite values: x y z roll pitch yaw')
    x,y,z,roll,pitch,yaw=message.data
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch)
    cy,sy=math.cos(yaw),math.sin(yaw)
    matrix=np.eye(4);matrix[:3,3]=[x,y,z]
    matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                   [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                   [-sp,cp*sr,cp*cr]]
    return make_pose(matrix,frame,stamp)


class Simulation(Node):
    def __init__(self, kine_factory=Kinematics, controller_factory=VirtualController):
        super().__init__('marvin_virtual_controller')
        self.declare_parameter('sdk_root', '')
        self.declare_parameter('description_root', '')
        self.declare_parameter('sim_ip', '')
        self.kine = kine_factory(self.get_parameter('sdk_root').value,
                               self.get_parameter('description_root').value)
        self.controller = controller_factory(self.kine, sim_ip=self.get_parameter('sim_ip').value)
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.status = self.create_publisher(String, '/marvin_sim/server_status', 10)
        self.poses = [self.create_publisher(PoseStamped, f'/marvin_sim/{side}/model_pose', 10)
                      for side in ('left', 'right')]
        self.create_timer(.005, self.controller.tick)
        self.create_timer(.02, self.publish)
        self.create_timer(.2, self.publish_status)
        self.get_logger().info(f'Virtual controller ready: {self.controller.address}:{self.controller.port} (this host); SDK feedback port {self.controller.feedback_port}')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp, msg.name = stamp, JOINT_NAMES
        msg.position, msg.velocity = self.controller.q.tolist(), self.controller.v.tolist()
        self.joints.publish(msg)
        for arm, pub in enumerate(self.poses):
            pub.publish(make_pose(self.kine.fk(arm, self.controller.q[7*arm:7*arm+7]),
                                  ('Base_L', 'Base_R')[arm], stamp))

    def publish_status(self):
        c = self.controller
        self.status.publish(String(data=json.dumps({
            'backend': 'native_sdk_local_udp', 'modes': c.modes,
            'moving': [v is not None for v in c.moves],
            'received_packets': c.received, 'rejected_packets': c.rejected,
            'last_error': c.last_error,
        })))

    def destroy_node(self):
        self.controller.close()
        return super().destroy_node()


class Bridge(Node):
    def __init__(self, kine_factory=Kinematics):
        super().__init__('marvin_sdk_bridge')
        self.declare_parameter('sdk_root', '')
        self.declare_parameter('description_root', '')
        self.declare_parameter('sim_ip', '')
        self.kine = kine_factory(self.get_parameter('sdk_root').value,
                               self.get_parameter('description_root').value)
        self.client = SDKClient(self.kine, sim_ip=self.get_parameter('sim_ip').value)
        self.state = 'ready'
        self.serial = None
        self.last_refresh = time.monotonic()
        self.q = self.client.read()
        self.joints = self.create_publisher(JointState, '/marvin_sim/sdk_joint_states', 10)
        self.status = self.create_publisher(String, '/marvin_sim/status', 10)
        self.poses = [self.create_publisher(PoseStamped, f'/marvin_sim/{side}/tcp_pose', 10)
                      for side in ('left', 'right')]
        self.create_subscription(JointState, '/marvin_sim/target_joints', self.joint_goal, 10)
        for arm, side in enumerate(('left', 'right')):
            self.create_subscription(PoseStamped, f'/marvin_sim/{side}/target_pose',
                                     lambda msg, a=arm: self.pose_goal(a, msg), 10)
            self.create_subscription(Float64MultiArray,f'/marvin_sim/{side}/target_rpy',
                lambda msg,a=arm,f=('Base_L','Base_R')[arm]:self.rpy_goal(a,msg,f),10)
        self.create_service(Trigger, '/marvin_sim/stop', self.stop)
        self.create_service(Trigger, '/marvin_sim/enable', self.enable)
        self.create_timer(.02, self.publish)
        self.last_status = String(data=json.dumps({'state': 'ready', 'sdk_ip': self.client.address}))
        self.create_timer(.2, lambda: self.status.publish(self.last_status))
        self.get_logger().info('Original libMarvinSDK.so connected to virtual controller; FK/IK via libKine.so')

    def report(self, state, **details):
        self.state = state
        self.last_status = String(data=json.dumps({'state': state, **details}))
        self.status.publish(self.last_status)

    def require_ready(self):
        self.refresh()
        if self.state in ('stopped', 'fault'):
            raise ValueError('Controller stopped/faulted; enable before sending another target')
        if any(s.m_CurState != 1 for s in self.client.feedback.m_State):
            raise ValueError('Both arms must be in position mode')

    def refresh(self):
        self.q = self.client.read()
        serial = self.client.feedback.m_Out[0].m_OutFrameSerial
        if serial != self.serial:
            self.serial, self.last_refresh = serial, time.monotonic()
        if time.monotonic()-self.last_refresh > .5:
            raise RuntimeError('SDK feedback stale for more than 0.5 seconds')

    def joint_goal(self, msg):
        try:
            self.require_ready()
            if (not msg.name or len(set(msg.name)) != len(msg.name)
                    or len(msg.name) != len(msg.position) or msg.velocity or msg.effort):
                raise ValueError('Provide unique joint names and positions only')
            names = set(msg.name)
            allowed = (set(JOINT_NAMES[:7]), set(JOINT_NAMES[7:]), set(JOINT_NAMES))
            if names not in allowed:
                raise ValueError('Target must contain one complete arm or all 14 joints')
            mapping = dict(zip(msg.name, msg.position))
            goals = {}
            for arm in range(2):
                arm_names = JOINT_NAMES[7*arm:7*arm+7]
                if all(n in mapping for n in arm_names):
                    goals[arm] = [mapping[n] for n in arm_names]
            self.client.move(goals)
            self.report('moving', command='joint', arms=list(goals))
        except (ValueError, RuntimeError) as exc:
            self.report('fault' if isinstance(exc, RuntimeError) else self.state, error=str(exc))

    def pose_goal(self, arm, msg):
        try:
            self.require_ready()
            expected = ('Base_L', 'Base_R')[arm]
            if msg.header.frame_id != expected:
                raise ValueError(f'Target pose frame must be {expected}; world-frame conversion is not implicit')
            p, q = msg.pose.position, msg.pose.orientation
            target = np.eye(4)
            target[:3, :3] = quaternion_matrix([q.x, q.y, q.z, q.w])
            target[:3, 3] = [p.x, p.y, p.z]
            result = self.kine.ik(arm, target, self.q[7*arm:7*arm+7])
            self.client.move({arm: result})
            self.report('moving', command='sdk_ik', arm=arm,
                        solution_rad=result.tolist(), target_matrix_m=target.tolist())
        except (ValueError, RuntimeError) as exc:
            self.report('fault' if isinstance(exc, RuntimeError) else self.state, error=str(exc))

    def rpy_goal(self,arm,msg,frame):
        try:self.pose_goal(arm,make_rpy_pose(msg,frame,self.get_clock().now().to_msg()))
        except ValueError as exc:self.report(self.state,accepted=False,command='sdk_ik_rpy',error=str(exc))

    def stop(self, request, response):
        try:
            self.client.stop()
            self.report('stopped')
            response.success, response.message = True, 'Both simulated arms disabled'
        except RuntimeError as exc:
            response.success, response.message = False, str(exc)
            self.report('fault', error=str(exc))
        return response

    def enable(self, request, response):
        try:
            self.refresh()
            self.client.enable()
            self.report('ready')
            response.success, response.message = True, 'Position mode enabled'
        except RuntimeError as exc:
            response.success, response.message = False, str(exc)
        return response

    def publish(self):
        try:
            self.refresh()
            stamp = self.get_clock().now().to_msg()
            msg = JointState()
            msg.header.stamp, msg.name, msg.position = stamp, JOINT_NAMES, self.q.tolist()
            self.joints.publish(msg)
            for arm, pub in enumerate(self.poses):
                pub.publish(make_pose(self.kine.fk(arm, self.q[7*arm:7*arm+7]),
                                      ('Base_L', 'Base_R')[arm], stamp))
            if self.state == 'moving' and all(out.m_LowSpdFlag == b'\x01'
                                               for out in self.client.feedback.m_Out):
                self.report('completed')
        except (ValueError, RuntimeError) as exc:
            if self.state != 'fault':
                self.report('fault', error=str(exc))

    def destroy_node(self):
        self.client.close()
        return super().destroy_node()


def main(args=None):
    import sys
    bridge = '--bridge' in sys.argv
    if bridge:
        sys.argv.remove('--bridge')
    rclpy.init(args=args)
    node = None
    try:
        node = Bridge() if bridge else Simulation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
