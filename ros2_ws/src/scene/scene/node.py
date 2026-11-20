"""Convert MQTT object poses in body_base to sequential flange targets."""
import json
import math
import os
import re
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import paho.mqtt.client as mqtt
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from gripper.msg import GripperCommand, GripperState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from .arm_positions import load_arm_positions
from .geometry import (_parse_payload, flange_target_from_tip, joint5_restore_waypoints,
                       parse_vision_message, rotate_vector, rotation_matrix, quaternion_multiply,
                       route_targets, tool_z_rotation_matrix,
                       tool_z_rotations, undo_tool_z_matrix, undo_tool_z_rotation,
                       workpiece_pose_from_flange)


class VisionTargets(Node):
    def __init__(self):
        super().__init__('scene_vision_targets')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', '/vision_target')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('interval', 1.0)
        self.declare_parameter('tool_tip_offset_z_m', .20)
        self.declare_parameter('max_arm_pose_age_s', .5)
        self.declare_parameter('allow_tool_z_rotation', True)
        self.declare_parameter('last_rotation_file', '~/.ros/scene_last_rotation.json')
        self.declare_parameter('tool_z_angle_file', '~/.ros/scene_tool_z_angle.json')
        self.declare_parameter('left_post_grasp_joints',
                               '[2.845654625621635, 0.4907167724907257, '
                               '-2.826141844584338, -1.5708835932574963, '
                               '-2.0122948143793824, 0.49986229777117597, '
                               '-1.035084966187757]')
        self.declare_parameter('right_handover_joints',
                               '[0.2175552912610932, -0.4690223298884362, '
                               '-0.19600047499896323, -1.5263951506241609, '
                               '-1.1898433109620943, -0.5670400206804377, '
                               '-1.070166084152843]')
        self.declare_parameter('right_post_grasp_joints',
                               '[2.84704, 0.46903, -2.82737, -1.54347, -1.98059, 0.51953, -1.08505]')
        self.declare_parameter('left_handover_joints',
                               '[0.21863, -0.49051, -0.19696, -1.55473, -1.15677, -0.54807, -1.01652]')
        self.declare_parameter('handover_y_offset_m', .065)
        self.declare_parameter('left_handover_y_offset_m', .063)
        self.declare_parameter('arm_positions_file',
                               str(Path(get_package_share_directory('scene')) / 'config' / 'arm_positions.json'))
        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        topic = self.get_parameter('mqtt_topic').value
        self.dry_run = self.get_parameter('dry_run').value
        self.interval = float(self.get_parameter('interval').value)
        self.tool_tip_offset_z_m = float(self.get_parameter('tool_tip_offset_z_m').value)
        self.max_arm_pose_age_ns = int(self.get_parameter('max_arm_pose_age_s').value * 1e9)
        self.allow_tool_z_rotation = bool(self.get_parameter('allow_tool_z_rotation').value)
        self.last_rotation_file = Path(self.get_parameter('last_rotation_file').value).expanduser()
        self.tool_z_angle_file = Path(self.get_parameter('tool_z_angle_file').value).expanduser()
        try:
            joint_values = json.loads(self.get_parameter('left_post_grasp_joints').value)
            if (not isinstance(joint_values, list) or len(joint_values) != 7 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) for v in joint_values)):
                raise ValueError('expected seven finite joint angles in radians')
            self.left_post_grasp_joints = tuple(float(v) for v in joint_values)
        except (TypeError, ValueError) as exc:
            raise ValueError(f'Invalid left_post_grasp_joints: {exc}') from exc
        self.arm_positions = load_arm_positions(self.get_parameter('arm_positions_file').value)
        if any(('home', side) not in self.arm_positions for side in ('left', 'right')):
            raise ValueError('Arm positions must contain left and right home targets')
        if ('body_photo', 'body_pose_ptp') not in self.arm_positions:
            raise ValueError('Arm positions must contain body_photo body_pose_ptp for body restoration')
        if ('body_home', 'body_joints') not in self.arm_positions:
            raise ValueError('Arm positions must contain body_home body_joints for back handover')
        self.right_handover_joints = self.parse_joint_parameter('right_handover_joints')
        self.right_post_grasp_joints = self.parse_optional_joint_parameter('right_post_grasp_joints')
        self.left_handover_joints = self.parse_optional_joint_parameter('left_handover_joints')
        self.handover_y_offset_m = float(self.get_parameter('handover_y_offset_m').value)
        self.left_handover_y_offset_m = float(self.get_parameter('left_handover_y_offset_m').value)
        if (not math.isfinite(self.handover_y_offset_m) or self.handover_y_offset_m <= 0 or
                not math.isfinite(self.left_handover_y_offset_m) or self.left_handover_y_offset_m <= 0):
            raise ValueError('Handover Y offsets must be positive and finite')
        if (not host or not 1 <= port <= 65535 or not topic or self.max_arm_pose_age_ns <= 0
                or not math.isfinite(self.interval) or self.interval < 0
                or not -1. <= self.tool_tip_offset_z_m <= 1.):
            raise ValueError('Invalid MQTT, interval, tool offset, or arm feedback parameters')

        self.arm_poses = {}
        self.arm_joints = None
        self.body_joints = None
        self.control_state = None
        self.last_task_id = None
        self.queue = None
        self.queue_lock = threading.Lock()
        self.targets = []
        self.state = 'idle'
        self.sent_at = 0.
        self.next_at = 0.
        self.waiting_task_id = None
        self.yaw_candidates = None
        self.yaw_index = 0
        self.position1_solution = None
        self.position1_goal = None
        self.position1_tip = None
        self.descent_retry_count = 0
        self.grasp_target = None
        self.vision_label = None
        self.body_pre_grasp_pending = False
        self.body_pre_grasp_used = False
        self.body_shift_pending = False
        self.body_shift_used = False
        self.body_shift_m = 0.
        self.second_arm_prepare_pending = False
        self.second_turn_after_prepare = False
        self.body_pose_feedback = None
        self.attached_workpiece = None
        self.front_grasp_arms = []
        self.back_first_arm = None
        self.back_second_payload = None
        self.post_grasp_targets = []
        self.post_grasp_dual_pending = False
        self.gripper_pending = None
        self.gripper_enable_pending = None
        self.gripper_enable_queue = []
        self.gripper_move_after_enable = None
        self.gripper_states = {}
        self.workpiece_relative = None
        self.saw_motion = False
        self.restore_mode = False
        self.joint_restore_steps = None
        self.joint_restore_index = 0
        self.preset_targets = []
        self.preset_name = None
        self.preset_body_j4_pending = False
        self.start_sequence_stage = None
        self.target_publishers = {
            side: self.create_publisher(PoseStamped, f'/gento_control/{side}/target_pose', 1)
            for side in ('left', 'right')}
        self.ptp_publishers = {
            side: self.create_publisher(PoseStamped, f'/gento_control/{side}/target_pose_ptp', 1)
            for side in ('left', 'right')}
        self.status_pub = self.create_publisher(String, '/scene/status', 10)
        self.workpiece_pose_pub = self.create_publisher(PoseStamped, '/scene/workpiece_pose', 10)
        self.joint_publishers = {
            side: self.create_publisher(JointState, f'/gento_control/{side}/target_joints', 1)
            for side in ('left', 'right')}
        self.dual_joint_publisher = self.create_publisher(
            JointState, '/gento_control/dual/target_joints', 1)
        self.body_pose_publisher = self.create_publisher(
            PoseStamped, '/gento_control/body/target_pose', 1)
        self.body_pose_ptp_publisher = self.create_publisher(
            PoseStamped, '/gento_control/body/target_pose_ptp', 1)
        self.body_joint_publisher = self.create_publisher(
            JointState, '/gento_control/body/target_joints', 1)
        self.gripper_publishers = {side: self.create_publisher(
            GripperCommand, f'/{side}_gripper/command', 1) for side in ('left', 'right')}
        for side in ('left', 'right'):
            self.create_subscription(GripperState, f'/{side}_gripper/state',
                                     lambda msg, arm=side: self.on_gripper_state(arm, msg), 10)
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/gento_feedback/{side}/tcp_pose_body_base',
                                     lambda message, arm=side: self.on_arm_pose(arm, message), 10)
        self.create_subscription(String, '/gento_control/status', self.on_control_status, 10)
        self.create_subscription(JointState, '/gento_feedback/joint_states', self.on_joint_states, 10)
        self.create_subscription(JointState, '/gento_feedback/body/joint_states', self.on_body_joint_states, 10)
        self.create_subscription(PoseStamped, '/gento_feedback/body/tcp_pose',
                                 self.on_body_pose, 10)
        self.create_subscription(Int8, '/scene/arm_position', self.on_arm_position, 10)
        self.create_subscription(String, '/scene/start', self.on_start, 10)
        self.create_service(Trigger, '/scene/restore_tool_z', self.restore_tool_z)
        self.create_service(Trigger, '/scene/attach_workpiece', self.attach_workpiece)
        self.create_service(Trigger, '/scene/detach_workpiece', self.detach_workpiece)
        self.create_timer(.05, self.tick)

        self.mqtt = mqtt.Client(client_id='scene_vision_targets', clean_session=True)
        self.mqtt_topic = topic
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        self.mqtt.connect_async(host, port, keepalive=30)
        self.mqtt.loop_start()

    def report(self, state, **details):
        self.status_pub.publish(String(data=json.dumps({'state': state, **details})))
        if state == 'rejected':
            self.get_logger().warning(str(details.get('reason', state)))
        else:
            self.get_logger().info(f'{state}: {details}')

    def on_connect(self, client, _userdata, _flags, result_code):
        if result_code == 0:
            client.subscribe(self.mqtt_topic, qos=0)
            self.get_logger().info(f'MQTT subscribed: {self.mqtt_topic}')
        else:
            self.get_logger().warning(f'MQTT connection rejected: {result_code}')

    def on_message(self, _client, _userdata, message):
        # Retained positions may be old; never turn them into a new motion.
        if message.retain:
            return
        try:
            label = _parse_payload(message.payload).get('label')
        except (ValueError, UnicodeError):
            # Keep malformed payloads on the existing rejection path in tick().
            pass
        else:
            if label not in ('无遮挡正面', '无遮挡背面'):
                return
        with self.queue_lock:
            if (self.queue is None and self.state in ('idle', 'awaiting_second_target')
                    and not self.restore_mode and
                    (self.attached_workpiece is None or self.state == 'awaiting_second_target')):
                self.queue = bytes(message.payload)
            else:
                self.get_logger().warning('Vision target ignored while another target is active')

    def on_arm_pose(self, side, message):
        if message.header.frame_id == 'body_base':
            self.arm_poses[side] = message
            if self.attached_workpiece is not None and self.attached_workpiece[0] == side:
                if self.current_arm_pose(side) is not None:
                    try:
                        self.publish_workpiece_pose(message)
                    except ValueError as exc:
                        self.get_logger().warning(f'Invalid flange pose for workpiece tracking: {exc}')

    def publish_workpiece_pose(self, flange):
        side, yaw_deg = self.attached_workpiece
        flange_position = tuple(getattr(flange.pose.position, axis) for axis in 'xyz')
        flange_orientation = tuple(getattr(flange.pose.orientation, axis) for axis in 'xyzw')
        if getattr(self, 'workpiece_relative', None) is not None:
            offset, relative_q = self.workpiece_relative
            rotated = rotate_vector(flange_orientation, offset)
            position = tuple(a + b for a, b in zip(flange_position, rotated))
            orientation = quaternion_multiply(flange_orientation, relative_q)
        else:
            position, orientation = workpiece_pose_from_flange(
                flange_position, flange_orientation, self.tool_tip_offset_z_m, yaw_deg)
        output = PoseStamped()
        output.header.stamp = flange.header.stamp
        output.header.frame_id = 'body_base'
        output.pose.position.x, output.pose.position.y, output.pose.position.z = position
        (output.pose.orientation.x, output.pose.orientation.y,
         output.pose.orientation.z, output.pose.orientation.w) = orientation
        self.workpiece_pose_pub.publish(output)

    def attach_workpiece(self, _request, response):
        """Record the position2 workpiece and begin the post-grasp sequence."""
        second_front_grasp = (len(getattr(self, 'front_grasp_arms', [])) == 1 and
                              getattr(self, 'vision_label', None) in ('无遮挡正面', '无遮挡背面') and
                              self.grasp_target is not None and
                              self.grasp_target[0] != self.front_grasp_arms[0])
        if self.attached_workpiece is not None and not second_front_grasp:
            response.success, response.message = False, 'A workpiece is already attached'
        elif (self.grasp_target is None or self.position1_goal is None or
              self.state != 'idle' or self.queue is not None or getattr(self, 'gripper_pending', None)):
            response.success, response.message = False, 'No completed position2 target ready for attachment'
        else:
            side, yaw_deg = self.grasp_target
            if self.position1_goal[0] != side:
                response.success, response.message = False, 'Grasp arm does not match the active vision label'
            elif self.control_state in ('running', 'recovering'):
                response.success, response.message = False, 'Arm control is busy'
            elif (self.target_publishers[side].get_subscription_count() == 0 or
                  any(self.joint_publishers[arm].get_subscription_count() == 0
                      for arm in ('left', 'right')) or
                  self.body_joint_publisher.get_subscription_count() == 0):
                response.success, response.message = False, 'Post-grasp motion subscriber is unavailable'
            elif (flange := self.current_arm_pose(side)) is None:
                response.success, response.message = False, f'No fresh {side} flange pose'
            else:
                try:
                    self.attached_workpiece = (side, yaw_deg)
                    self.publish_workpiece_pose(flange)
                except ValueError as exc:
                    self.attached_workpiece = None
                    response.success, response.message = False, f'Invalid flange pose: {exc}'
                else:
                    _, retreat_position, retreat_orientation = self.position1_goal
                    j4_clearance = [('body_j4_clearance', 'body_joints', None, None)]
                    arm_j1_clearance = [(f'{side}_j1_clearance', 'joints', None, None)]
                    body_restore = []
                    if (getattr(self, 'body_pre_grasp_used', False) or
                            getattr(self, 'body_shift_used', False)):
                        xyz, xyzw = self.arm_positions['body_photo', 'body_pose_ptp']
                        body_restore.append(('body_photo_restore', 'body_pose_ptp', xyz, xyzw))
                    if (getattr(self, 'vision_label', None) == '无遮挡正面' or
                            getattr(self, 'front_grasp_arms', [])):
                        other = 'right' if side == 'left' else 'left'
                        self.front_grasp_arms.append(side)
                        if len(self.front_grasp_arms) == 1 and not body_restore:
                            xyz, xyzw = self.arm_positions['body_photo', 'body_pose_ptp']
                            body_restore = [('body_photo_restore', 'body_pose_ptp', xyz, xyzw)]
                        self.post_grasp_targets = [
                            (f'{side}_retreat', 'cartesian', retreat_position, retreat_orientation),
                            *arm_j1_clearance,
                            *j4_clearance,
                            (f'{side}_grasp_pre', 'joints', self.arm_positions['pre', side], None),
                            (f'{other}_grasp_pre', 'joints', self.arm_positions['pre', other], None),
                            *body_restore,
                        ]
                        if len(self.front_grasp_arms) == 2:
                            self.post_grasp_targets.extend([
                                ('body_zero', 'body_joints', (0.,) * 6, None),
                                ('left_home', 'joints', self.arm_positions['home', 'left'], None),
                                ('right_home', 'joints', self.arm_positions['home', 'right'], None),
                                ('left_final_close', 'gripper', 0., None),
                                ('right_final_close', 'gripper', 0., None),
                            ])
                    elif side == 'left':
                        self.back_first_arm = side
                        xyz, xyzw = self.arm_positions['body_photo', 'body_pose_ptp']
                        self.post_grasp_targets = [
                            ('retreat', 'cartesian', retreat_position, retreat_orientation),
                            *arm_j1_clearance,
                            *j4_clearance,
                            ('pre', 'joints', self.arm_positions['pre', 'left'], None),
                            ('body_photo_restore', 'body_pose_ptp', xyz, xyzw),
                        ]
                    else:
                        self.back_first_arm = side
                        xyz, xyzw = self.arm_positions['body_photo', 'body_pose_ptp']
                        self.post_grasp_targets = [
                            ('right_retreat', 'cartesian', retreat_position, retreat_orientation),
                            *arm_j1_clearance,
                            *j4_clearance,
                            ('right_grasp_pre', 'joints', self.arm_positions['pre', 'right'], None),
                            ('body_photo_restore', 'body_pose_ptp', xyz, xyzw),
                        ]
                    self.grasp_target = None
                    self.report('workpiece_attached', arm=side, tool_z_rotation_deg=yaw_deg)
                    response.success = self.send_post_grasp_next()
                    response.message = ('Workpiece attached; retreat started, then configured post-grasp steps'
                                        if response.success else
                                        'Workpiece attached, but retreat could not start; check /scene/status')
        return response

    def parse_joint_parameter(self, name):
        values = json.loads(self.get_parameter(name).value)
        if (not isinstance(values, list) or len(values) != 7 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in values)):
            raise ValueError(f'{name} must contain seven finite joint angles')
        return tuple(float(v) for v in values)

    def parse_optional_joint_parameter(self, name):
        values = json.loads(self.get_parameter(name).value)
        if values == []:
            return None
        return self.parse_joint_parameter(name)

    def send_gripper(self, side, position, step):
        publisher = self.gripper_publishers[side]
        if publisher.get_subscription_count() == 0:
            self.state = 'fault'
            self.report('rejected', reason=f'No {side} gripper command subscriber')
            return False
        command = GripperCommand()
        command.action = GripperCommand.MOVE
        command.position_m = position
        publisher.publish(command)
        self.gripper_pending = (side, position, step, time.monotonic())
        self.state = 'gripper_waiting'
        self.report('gripper_sent', arm=side, step=step, position_m=position)
        return True

    def enable_grippers_before_grasp(self, side, step):
        self.gripper_enable_queue = ['left', 'right']
        self.gripper_move_after_enable = (side, .027, step)
        self.send_next_gripper_enable()

    def send_next_gripper_enable(self):
        if not self.gripper_enable_queue:
            side, position, step = self.gripper_move_after_enable
            self.gripper_move_after_enable = None
            return self.send_gripper(side, position, step)
        side = self.gripper_enable_queue.pop(0)
        publisher = self.gripper_publishers[side]
        if publisher.get_subscription_count() == 0:
            self.gripper_enable_queue.clear()
            self.gripper_move_after_enable = None
            self.state = 'fault'
            self.report('rejected', reason=f'No {side} gripper command subscriber for enable')
            return False
        command = GripperCommand()
        command.action = GripperCommand.ENABLE
        publisher.publish(command)
        self.gripper_enable_pending = (side, time.monotonic())
        self.state = 'gripper_waiting'
        self.report('gripper_enable_sent', arm=side)
        return True

    def on_gripper_state(self, side, message):
        if getattr(self, 'gripper_enable_pending', None) is not None:
            pending_side, sent_at = self.gripper_enable_pending
            if side != pending_side or time.monotonic() - sent_at < .1:
                return
            if not message.valid or message.error_code:
                self.gripper_enable_pending = None
                self.gripper_enable_queue.clear()
                self.gripper_move_after_enable = None
                self.state = 'fault'
                self.report('rejected', reason=f'{side} gripper enable failed: invalid feedback or fault')
                return
            if not message.enabled:
                return
            self.gripper_enable_pending = None
            self.report('gripper_enable_completed', arm=side)
            self.send_next_gripper_enable()
            return
        if self.gripper_pending is None or side != self.gripper_pending[0]:
            return
        _, target, step, sent_at = self.gripper_pending
        if time.monotonic() - sent_at < .1:
            return
        if not message.valid or message.error_code or not message.enabled:
            self.gripper_pending = None
            self.state = 'fault'
            self.report('rejected', reason=f'{side} gripper invalid or fault at {step}')
            return
        at_target = (message.at_target and
                     abs(message.position_m - target) <= .002 and
                     abs(message.target_position_m - target) <= .002)
        held_by_current_limit = (step in ('left_grasp', 'right_grasp', 'right_direct_grasp',
                                         'left_handover_grasp') and
                                 message.target_active and message.current_limit_reached and
                                 message.holding_position and
                                 math.isfinite(message.position_m) and
                                 math.isfinite(message.target_position_m) and
                                 abs(message.position_m - message.target_position_m) <= .002)
        if not (at_target or held_by_current_limit):
            return
        self.gripper_pending = None
        self.report('gripper_completed', arm=side, step=step,
                    completion='current_limit_hold' if held_by_current_limit else 'at_target',
                    position_m=message.position_m)
        if step in ('left_grasp', 'right_direct_grasp'):
            self.state = 'idle'
            response = self.attach_workpiece(None, SimpleNamespace(success=False, message=''))
            if not response.success:
                self.state = 'fault'
                self.report('rejected', reason=f'Automatic workpiece attachment failed: {response.message}')
        elif step in ('right_grasp', 'left_handover_grasp'):
            receiving = 'right' if step == 'right_grasp' else 'left'
            releasing = 'left' if receiving == 'right' else 'right'
            try:
                if receiving == 'right':
                    self.capture_right_attachment()
                else:
                    self.capture_attachment('left')
            except ValueError as exc:
                self.state = 'fault'
                self.report('rejected', reason=str(exc))
                return
            handover_joints = (self.right_handover_joints if receiving == 'right'
                               else self.left_handover_joints)
            self.post_grasp_targets = [(f'{releasing}_release', 'gripper', 0., None),
                                       (f'{receiving}_handover_retreat', 'joints', handover_joints, None)]
            self.state = 'next'
            self.next_at = time.monotonic() + .2
        elif step in ('left_release', 'right_release'):
            self.post_grasp_targets.pop(0)
            self.state = 'next'
            self.next_at = time.monotonic() + .2
        elif step in ('left_final_close', 'right_final_close'):
            self.post_grasp_targets.pop(0)
            if self.post_grasp_targets:
                self.state = 'next'
                self.next_at = time.monotonic() + .2
            else:
                self.state = 'idle'
                self.attached_workpiece = None
                self.workpiece_relative = None
                self.front_grasp_arms.clear()
                self.back_first_arm = None
                self.back_second_payload = None
                self.report('post_grasp_completed', arm=side)

    def capture_attachment(self, receiving):
        releasing = 'left' if receiving == 'right' else 'right'
        source = self.current_arm_pose(releasing)
        target = self.current_arm_pose(receiving)
        if source is None or target is None:
            raise ValueError('Fresh poses from both arms are required for workpiece transfer')
        source_p = tuple(getattr(source.pose.position, axis) for axis in 'xyz')
        source_q = tuple(getattr(source.pose.orientation, axis) for axis in 'xyzw')
        target_p = tuple(getattr(target.pose.position, axis) for axis in 'xyz')
        target_q = tuple(getattr(target.pose.orientation, axis) for axis in 'xyzw')
        work_p, work_q = workpiece_pose_from_flange(
            source_p, source_q, self.tool_tip_offset_z_m, self.attached_workpiece[1])
        inverse_q = (-target_q[0], -target_q[1], -target_q[2], target_q[3])
        self.workpiece_relative = (rotate_vector(inverse_q, tuple(a-b for a, b in zip(work_p, target_p))),
                                   quaternion_multiply(inverse_q, work_q))
        self.attached_workpiece = (receiving, 0.)
        self.publish_workpiece_pose(target)
        self.report('workpiece_transferred', arm=receiving)

    def capture_right_attachment(self):
        VisionTargets.capture_attachment(self, 'right')

    def send_post_grasp_next(self):
        step, mode, target, orientation = self.post_grasp_targets[0]
        if len(self.post_grasp_targets) >= 2:
            pair = self.post_grasp_targets[:2]
            names = [item[0] for item in pair]
            sides = [name.split('_', 1)[0] for name in names]
            if (set(sides) == {'left', 'right'} and
                    all(item[1] == 'joints' for item in pair) and
                    all(name.endswith(('_grasp_pre', '_return_pre', '_home')) for name in names)):
                if self.dual_joint_publisher.get_subscription_count() == 0:
                    self.post_grasp_targets.clear()
                    self.state = 'idle'
                    self.report('rejected', reason='No dual-arm control subscriber for post-grasp')
                    return False
                targets = {side: item[2] for side, item in zip(sides, pair)}
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = [f'{side}_joint_{index}'
                                for side in ('left', 'right') for index in range(1, 8)]
                message.position = list(targets['left']) + list(targets['right'])
                self.waiting_task_id = None
                self.saw_motion = False
                self.sent_at = time.monotonic()
                self.post_grasp_dual_pending = True
                self.state = 'waiting'
                self.dual_joint_publisher.publish(message)
                self.report('post_grasp_sent', arm='both', steps=names,
                            motion_mode='dual_joints', joints_rad=list(message.position))
                return True
        side = 'right' if step.startswith('right_') else 'left'
        if mode == 'gripper':
            return self.send_gripper(side, target, step)
        if step in ('right_approach', 'left_approach'):
            approach_side = step.split('_')[0]
            flange = self.current_arm_pose(approach_side)
            if flange is None:
                self.state = 'fault'
                self.report('rejected', reason=f'No fresh {approach_side} flange pose for handover approach')
                return False
            direction = 1 if approach_side == 'right' else -1
            offset = (self.handover_y_offset_m if approach_side == 'right'
                      else self.left_handover_y_offset_m)
            target = (flange.pose.position.x, flange.pose.position.y + direction * offset,
                      flange.pose.position.z)
            orientation = tuple(getattr(flange.pose.orientation, axis) for axis in 'xyzw')
        if step == 'body_j4_clearance':
            joints = self.current_body_joints()
            if joints is None:
                self.post_grasp_targets.clear()
                self.state = 'fault'
                self.report('rejected', reason='No fresh body joint feedback for J4 clearance')
                return False
            target = list(joints)
            target[3] = .1
        elif step in ('left_j1_clearance', 'right_j1_clearance'):
            joints = self.current_arm_joints(side)
            if joints is None:
                self.post_grasp_targets.clear()
                self.state = 'fault'
                self.report('rejected', reason=f'No fresh {side} arm joint feedback for J1 clearance')
                return False
            target = list(joints)
            target[0] = 1.5 if side == 'left' else -1.5
        publisher = (self.body_joint_publisher if mode == 'body_joints' else
                     self.body_pose_ptp_publisher if mode == 'body_pose_ptp' else
                     self.joint_publishers[side] if mode == 'joints' else
                     self.ptp_publishers[side] if mode == 'ptp' else
                     self.target_publishers[side])
        if publisher.get_subscription_count() == 0:
            self.post_grasp_targets.clear()
            self.state = 'idle'
            self.report('rejected', reason=f'No control subscriber for post-grasp {step}')
            return False
        if mode in ('joints', 'body_joints'):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = ([f'body_joint_{index}' for index in range(1, 7)]
                            if mode == 'body_joints' else
                            [f'{side}_joint_{index}' for index in range(1, 8)])
            message.position = list(target)
        else:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'body_base'
            message.pose.position.x, message.pose.position.y, message.pose.position.z = target
            (message.pose.orientation.x, message.pose.orientation.y,
             message.pose.orientation.z, message.pose.orientation.w) = orientation
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        publisher.publish(message)
        details = {'arm': 'body' if mode in ('body_joints', 'body_pose_ptp') else side,
                   'step': step, 'motion_mode': mode}
        if mode in ('joints', 'body_joints'):
            details['joints_rad'] = list(target)
        else:
            details.update(flange_position=target, flange_orientation=orientation)
        self.report('post_grasp_sent', **details)
        return True

    def detach_workpiece(self, _request, response):
        """Operator confirms the workpiece has been released."""
        if self.attached_workpiece is None:
            response.success, response.message = False, 'No workpiece is attached'
        elif self.post_grasp_targets:
            response.success, response.message = False, 'Post-grasp motion is still active'
        elif self.state != 'idle':
            response.success, response.message = False, 'Scene is busy or faulted'
        else:
            side, _ = self.attached_workpiece
            self.attached_workpiece = None
            self.workpiece_relative = None
            self.grasp_target = None
            self.front_grasp_arms.clear()
            self.back_first_arm = None
            self.back_second_payload = None
            response.success, response.message = True, f'{side} workpiece detached'
            self.report('workpiece_detached', arm=side)
        return response

    def on_joint_states(self, message):
        self.arm_joints = message

    def on_body_joint_states(self, message):
        self.body_joints = message

    def current_arm_joints(self, side):
        message = self.arm_joints
        if message is None:
            return None
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if age < 0 or age > self.max_arm_pose_age_ns:
            return None
        expected = [f'{side}_joint_{index}' for index in range(1, 8)]
        values = dict(zip(message.name, message.position))
        if len(message.name) != len(message.position) or not all(name in values for name in expected):
            return None
        joints = tuple(float(values[name]) for name in expected)
        return joints if all(math.isfinite(value) for value in joints) else None

    def current_body_joints(self):
        message = self.body_joints
        if message is None:
            return None
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if age < 0 or age > self.max_arm_pose_age_ns:
            return None
        expected = [f'body_joint_{index}' for index in range(1, 7)]
        values = dict(zip(message.name, message.position))
        if len(message.name) != len(message.position) or not all(name in values for name in expected):
            return None
        joints = tuple(float(values[name]) for name in expected)
        return joints if all(math.isfinite(value) for value in joints) else None

    def current_arm_pose(self, side):
        pose = self.arm_poses.get(side)
        if pose is None:
            return None
        stamp = pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if age < 0 or age > self.max_arm_pose_age_ns:
            return None
        return pose

    def current_arm_orientation(self, side):
        pose = self.current_arm_pose(side)
        if pose is None:
            return None
        q = pose.pose.orientation
        return q.x, q.y, q.z, q.w

    def on_arm_position(self, message):
        """Move both arms to a joint preset in one synchronized driver action."""
        position = self.arm_positions.get(message.data)
        if position is None:
            self.report('rejected', reason=f'Unknown arm position ID: {message.data}')
            return
        with self.queue_lock:
            if getattr(self, 'attached_workpiece', None) is not None:
                self.report('rejected', reason='Workpiece remains attached; detach it before arm_position')
                return
            if self.dry_run or self.state != 'idle' or self.queue is not None or self.restore_mode:
                self.report('rejected', reason='Scene is in dry_run or has an active/queued target')
                return
            if self.control_state in ('running', 'recovering'):
                self.report('rejected', reason='Arm control is busy')
                return
            pose_kind = next((kind for kind in ('body_pose', 'body_pose_ptp')
                              if (position, kind) in self.arm_positions), None)
            if pose_kind is not None:
                publisher = (self.body_pose_publisher if pose_kind == 'body_pose'
                             else self.body_pose_ptp_publisher)
                if publisher.get_subscription_count() == 0:
                    self.report('rejected', reason='No body pose control subscriber')
                    return
                if message.data == 2 and position == 'body_photo':
                    joints = self.current_body_joints()
                    if joints is None:
                        self.report('rejected', reason='Fresh body joint feedback required for body_photo')
                        return
                    if abs(joints[5]) > .01:
                        if self.body_joint_publisher.get_subscription_count() == 0:
                            self.report('rejected', reason='No body joint control subscriber')
                            return
                        self.preset_name = position
                        self.preset_targets = []
                        self.preset_body_j4_pending = True
                        target = JointState()
                        target.header.stamp = self.get_clock().now().to_msg()
                        target.name = [f'body_joint_{index}' for index in range(1, 7)]
                        target.position = list(joints)
                        target.position[3] = .1
                        self.waiting_task_id = None
                        self.saw_motion = False
                        self.sent_at = time.monotonic()
                        self.state = 'waiting'
                        self.body_joint_publisher.publish(target)
                        self.report('arm_position_j4_sent', position=position,
                                    joints_rad=list(target.position))
                        return
                self.preset_name = position
                self.preset_targets = []
                VisionTargets.send_body_pose_preset(self)
                return
            if (position, 'body_joints') in self.arm_positions:
                if self.body_joint_publisher.get_subscription_count() == 0:
                    self.report('rejected', reason='No body joint control subscriber')
                    return
                self.preset_name = position
                self.preset_targets = []
                target = JointState()
                target.header.stamp = self.get_clock().now().to_msg()
                target.name = [f'body_joint_{index}' for index in range(1, 7)]
                target.position = list(self.arm_positions[position, 'body_joints'])
                self.waiting_task_id = None
                self.saw_motion = False
                self.sent_at = time.monotonic()
                self.state = 'waiting'
                self.body_joint_publisher.publish(target)
                self.report('arm_position_sent', arm='body', position=position,
                            joints_rad=list(target.position))
                return
            if self.dual_joint_publisher.get_subscription_count() == 0:
                self.report('rejected', reason='No dual-arm joint control subscriber')
                return
            self.preset_name = position
            self.preset_targets = ['left', 'right']
            self.send_preset_next()

    def on_start(self, message):
        """Run the existing pre and body_photo presets in order."""
        if message.data != 'start':
            self.report('rejected', reason='Expected /scene/start data: start')
            return
        if self.start_sequence_stage is not None:
            self.report('rejected', reason='Start sequence is already active')
            return
        if self.state != 'idle' or self.preset_name is not None:
            self.report('rejected', reason='Scene is busy; start requires idle state')
            return
        self.start_sequence_stage = 'pre'
        VisionTargets.on_arm_position(self, SimpleNamespace(data=1))
        if self.state != 'waiting' or self.preset_name != 'pre':
            self.start_sequence_stage = None

    def send_body_pose_preset(self):
        position = self.preset_name
        pose_kind = next(kind for kind in ('body_pose', 'body_pose_ptp')
                         if (position, kind) in self.arm_positions)
        publisher = (self.body_pose_publisher if pose_kind == 'body_pose'
                     else self.body_pose_ptp_publisher)
        xyz, xyzw = self.arm_positions[position, pose_kind]
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'body_base'
        target.pose.position.x, target.pose.position.y, target.pose.position.z = xyz
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = xyzw
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        publisher.publish(target)
        self.report('arm_position_sent', arm='body', position=position,
                    position_m=list(xyz), orientation_xyzw=list(xyzw))

    def send_preset_next(self):
        if self.preset_targets == ['left', 'right']:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [f'{side}_joint_{index}'
                            for side in ('left', 'right') for index in range(1, 8)]
            message.position = list(self.arm_positions[self.preset_name, 'left']) + list(
                self.arm_positions[self.preset_name, 'right'])
            self.waiting_task_id = None
            self.saw_motion = False
            self.sent_at = time.monotonic()
            self.state = 'waiting'
            self.dual_joint_publisher.publish(message)
            self.report('arm_position_sent', arm='both', position=self.preset_name,
                        joints_rad=list(message.position))
            return
        side = self.preset_targets[0]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'{side}_joint_{index}' for index in range(1, 8)]
        message.position = list(self.arm_positions[self.preset_name, side])
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        self.joint_publishers[side].publish(message)
        self.report('arm_position_sent', arm=side, position=self.preset_name,
                    joints_rad=list(message.position))

    def restore_tool_z(self, _request, response):
        """Attempt to undo the last successful local Z turn at the measured flange position."""
        try:
            if self.dry_run:
                raise ValueError('dry_run is enabled')
            with self.queue_lock:
                if self.state != 'idle' or self.queue is not None:
                    raise ValueError('Scene has an active or queued target')
                if self.control_state in ('running', 'recovering'):
                    raise ValueError('Arm control is busy')
                angle_path = (self.tool_z_angle_file if self.tool_z_angle_file.exists()
                              else self.last_rotation_file)
                record = json.loads(angle_path.read_text(encoding='utf-8'))
                if record.get('restored_tool_z'):
                    raise ValueError('The saved tool Z turn was already undone')
                if record.get('restore_uncertain'):
                    raise ValueError('Previous restore stopped during motion; verify joints before retrying')
                side = record['arm']
                if side not in self.target_publishers or record['frame_id'] != 'body_base':
                    raise ValueError('Invalid saved arm or frame')
                angle = float(record['tool_z_rotation_deg'])
                if not math.isfinite(angle) or abs(angle) < 1e-6:
                    raise ValueError('Last completed target has no tool Z turn to undo')
                feedback = self.current_arm_pose(side)
                if feedback is None:
                    raise ValueError(f'No fresh {side} arm pose feedback')
                position = tuple(getattr(feedback.pose.position, axis) for axis in 'xyz')
                orientation = tuple(getattr(feedback.pose.orientation, axis) for axis in 'xyzw')
                if not all(math.isfinite(v) for v in position):
                    raise ValueError('Current flange position is invalid')
                if record.get('rotation_matrix_type') == 'tool_z_delta':
                    saved_matrix = record['rotation_matrix']
                    target_orientation = undo_tool_z_matrix(orientation, saved_matrix)
                    expected_matrix = tool_z_rotation_matrix(angle)
                    if any(abs(float(saved_matrix[i][j]) - expected_matrix[i][j]) > 1e-5
                           for i in range(3) for j in range(3)):
                        raise ValueError('Saved tool Z angle and matrix disagree')
                else:
                    # Records created before this field held the absolute flange matrix.
                    target_orientation = undo_tool_z_rotation(orientation, angle)
                offset = rotate_vector(orientation, (0., 0., self.tool_tip_offset_z_m))
                tip_position = tuple(a+b for a, b in zip(position, offset))
                if self.target_publishers[side].get_subscription_count() == 0:
                    raise ValueError(f'No {side} arm control subscriber')
                joints = self.current_arm_joints(side)
                if joints is None:
                    raise ValueError(f'No fresh {side} arm joint feedback')
                use_joint5 = (abs(joints[5]) <= math.radians(.5)
                              and abs(joints[6]) <= math.radians(.5))
                if use_joint5:
                    if self.joint_publishers[side].get_subscription_count() == 0:
                        raise ValueError(f'No {side} arm joint control subscriber')
                    steps = joint5_restore_waypoints(joints[4], angle)
                self.targets = [(side, position, target_orientation, tip_position)]
                self.yaw_candidates = [(-angle, target_orientation)]
                self.yaw_index = 0
                self.restore_mode = True
                if use_joint5:
                    self.joint_restore_steps = steps
                    self.joint_restore_index = 0
                    self.joint_restore_reference = joints
                    self.restore_source_record = record
                    self.send_joint_restore_step()
                    mode = 'joint5'
                else:
                    self.joint_restore_steps = None
                    self.send_next()
                    mode = 'cartesian'
            response.success = True
            response.message = f'{mode} return target sent; check /scene/status for completion or rejection'
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            if self.state != 'waiting':
                self.targets.clear()
                self.yaw_candidates = None
                self.joint_restore_steps = None
                self.restore_mode = False
            response.success = False
            response.message = str(exc)
        return response

    def send_joint_restore_step(self):
        side = self.targets[0][0]
        joints = self.current_arm_joints(side)
        if joints is None:
            raise ValueError('No fresh joint feedback for joint-5 restore')
        expected_joint5 = (self.joint_restore_reference[4] if self.joint_restore_index == 0
                           else self.joint_restore_steps[self.joint_restore_index - 1])
        if abs(joints[4] - expected_joint5) > math.radians(1.):
            raise ValueError('Joint-5 feedback differs from the preceding restore target')
        if any(abs(joints[i] - self.joint_restore_reference[i]) > math.radians(.5)
               for i in (0, 1, 2, 3, 5, 6)):
            raise ValueError('Other arm joints changed during joint-5 restore')
        if self.joint_publishers[side].get_subscription_count() == 0:
            raise ValueError(f'No {side} arm joint control subscriber')
        target = self.joint_restore_steps[self.joint_restore_index]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'{side}_joint_{index}' for index in range(1, 8)]
        message.position = list(joints)
        message.position[4] = target
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        self.joint_publishers[side].publish(message)
        self.report('restore_joint5_sent', arm=side, step=self.joint_restore_index + 1,
                    total=len(self.joint_restore_steps), joint5_target_rad=target)

    @staticmethod
    def retryable_ik_rejection(reason):
        if not isinstance(reason, str):
            return False
        return (bool(re.search(r'arm[01] .*IK', reason)) or
                bool(re.search(r'joint\d+ exceeds limits(?:/max_delta)?', reason)))

    def on_control_status(self, message):
        try:
            status = json.loads(message.data)
            state = status['state']
            task_id = status.get('task_id')
        except (ValueError, TypeError, KeyError):
            return
        self.control_state = state
        if self.state != 'waiting':
            self.last_task_id = task_id
            return
        if task_id == self.last_task_id:
            return
        if self.waiting_task_id is None:
            self.waiting_task_id = task_id
        if task_id != self.waiting_task_id:
            return
        if getattr(self, 'second_arm_prepare_pending', False):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                self.last_task_id = task_id
                self.second_arm_prepare_pending = False
                self.state = 'next'
                self.next_at = time.monotonic() + .2
                self.report('second_arm_prepare_completed', arm=self.targets[0][0])
            elif state in ('rejected', 'fault', 'stopped'):
                self.last_task_id = task_id
                self.second_arm_prepare_pending = False
                self.second_turn_after_prepare = False
                self.targets.clear()
                self.state = 'idle'
                self.report('rejected', reason=f'Second arm prepare {state}: {status.get("reason", "")}')
            return
        if getattr(self, 'body_pre_grasp_pending', False):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                self.last_task_id = task_id
                self.body_pre_grasp_pending = False
                self.state = 'next'
                self.next_at = time.monotonic() + .2
                self.report('body_pre_grasp_completed', arm=self.targets[0][0])
            elif state in ('rejected', 'fault', 'stopped'):
                self.last_task_id = task_id
                self.body_pre_grasp_pending = False
                self.targets.clear()
                self.state = 'idle'
                self.report('rejected', reason=f'Body pre-grasp control {state}: {status.get("reason", "")}')
            return
        if getattr(self, 'body_shift_pending', False):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                self.last_task_id = task_id
                self.body_shift_pending = False
                self.state = 'next'
                self.next_at = time.monotonic() + .2
                self.report('body_shift_completed', shift_x_m=self.body_shift_m)
                self.body_shift_m = 0.
            elif state in ('rejected', 'fault', 'stopped'):
                self.last_task_id = task_id
                self.body_shift_pending = False
                self.targets.clear()
                self.state = 'idle'
                self.report('rejected', reason=f'Body shift control {state}: {status.get("reason", "")}')
            return
        if (getattr(self, 'preset_name', None) is not None and
                any((self.preset_name, kind) in getattr(self, 'arm_positions', {})
                    for kind in ('body_pose', 'body_pose_ptp', 'body_joints'))):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                self.last_task_id = task_id
                if getattr(self, 'preset_body_j4_pending', False):
                    self.preset_body_j4_pending = False
                    self.state = 'next'
                    self.next_at = time.monotonic() + .2
                    self.report('arm_position_j4_completed', position=self.preset_name)
                else:
                    self.state = 'idle'
                    self.report('arm_position_completed', position=self.preset_name)
                    self.preset_name = None
                    if getattr(self, 'start_sequence_stage', None) == 'body_photo':
                        self.start_sequence_stage = None
                        self.report('start_completed')
            elif state in ('rejected', 'fault', 'stopped'):
                self.last_task_id = task_id
                self.preset_body_j4_pending = False
                self.preset_name = None
                self.start_sequence_stage = None
                self.state = 'idle'
                self.report('rejected', reason=f'Body position control {state}: {status.get("reason", "")}')
            return
        if getattr(self, 'preset_targets', None):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                if self.preset_targets == ['left', 'right']:
                    self.preset_targets.clear()
                    self.last_task_id = task_id
                    self.report('arm_position_completed', position=self.preset_name)
                    self.preset_name = None
                    if getattr(self, 'start_sequence_stage', None) == 'pre':
                        self.start_sequence_stage = 'body_photo'
                        self.state = 'next'
                        self.next_at = time.monotonic() + .2
                    else:
                        self.state = 'idle'
                    return
                side = self.preset_targets.pop(0)
                self.last_task_id = task_id
                self.report('arm_position_step_completed', arm=side, position=self.preset_name)
                if self.preset_targets:
                    self.state = 'next'
                    self.next_at = time.monotonic() + .2
                else:
                    self.state = 'idle'
                    self.report('arm_position_completed', position=self.preset_name)
                    self.preset_name = None
            elif state in ('rejected', 'fault', 'stopped'):
                self.last_task_id = task_id
                self.preset_targets.clear()
                self.preset_name = None
                self.start_sequence_stage = None
                self.state = 'idle'
                self.report('rejected', reason=f'Arm position control {state}: {status.get("reason", "")}')
            return
        if getattr(self, 'post_grasp_targets', None):
            if state in ('running', 'recovering'):
                self.saw_motion = True
            elif state == 'completed':
                step = self.post_grasp_targets.pop(0)[0]
                if getattr(self, 'post_grasp_dual_pending', False):
                    other_step = self.post_grasp_targets.pop(0)[0]
                    self.post_grasp_dual_pending = False
                    self.report('post_grasp_step_completed', step=other_step)
                self.last_task_id = task_id
                self.report('post_grasp_step_completed', step=step)
                if step == 'right_pre':
                    self.post_grasp_targets = [('right_approach', 'cartesian', None, None),
                                               ('right_grasp', 'gripper', .027, None)]
                elif step == 'left_pre':
                    self.post_grasp_targets = [('left_approach', 'cartesian', None, None),
                                               ('left_handover_grasp', 'gripper', .027, None)]
                if getattr(self, 'post_grasp_targets', None):
                    self.state = 'next'
                    self.next_at = time.monotonic() + (
                        self.interval if step in ('right_pre', 'left_pre') else .2)
                    if step in ('right_pre', 'left_pre'):
                        self.report('handover_waiting', wait_s=self.interval)
                else:
                    self.state = 'idle'
                    if step == 'transfer':
                        self.post_grasp_targets = [('right_pre', 'joints', self.right_handover_joints, None)]
                        self.state = 'next'
                        self.next_at = time.monotonic() + .2
                        self.report('handover_started', step='right_pre')
                    elif step == 'right_transfer':
                        self.post_grasp_targets = [('left_pre', 'joints', self.left_handover_joints, None)]
                        self.state = 'next'
                        self.next_at = time.monotonic() + .2
                        self.report('handover_started', step='left_pre')
                    elif step in ('left_handover_retreat', 'right_handover_retreat'):
                        self.post_grasp_targets = [
                            ('left_return_pre', 'joints', self.arm_positions['pre', 'left'], None),
                            ('right_return_pre', 'joints', self.arm_positions['pre', 'right'], None),
                        ]
                        if getattr(self, 'back_second_payload', None) is not None:
                            xyz, xyzw = self.arm_positions['body_photo', 'body_pose_ptp']
                            self.post_grasp_targets.append(
                                ('body_photo_restore', 'body_pose_ptp', xyz, xyzw))
                        else:
                            self.post_grasp_targets.extend([
                                ('left_home', 'joints', self.arm_positions['home', 'left'], None),
                                ('right_home', 'joints', self.arm_positions['home', 'right'], None),
                                ('left_final_close', 'gripper', 0., None),
                                ('right_final_close', 'gripper', 0., None),
                            ])
                        self.state = 'next'
                        self.next_at = time.monotonic() + .2
                        self.report('handover_return_started')
                    else:
                        if getattr(self, 'back_second_payload', None) is not None:
                            self.front_grasp_arms = [self.attached_workpiece[0]]
                            with self.queue_lock:
                                self.queue = self.back_second_payload
                            self.back_second_payload = None
                            self.state = 'awaiting_second_target'
                            self.report('back_handover_completed', required_arm=self.back_first_arm)
                        elif (getattr(self, 'back_first_arm', None) is not None and
                              not getattr(self, 'front_grasp_arms', [])):
                            self.state = 'awaiting_second_target'
                            self.report('awaiting_second_target', required_arm=self.back_first_arm)
                        elif (getattr(self, 'vision_label', None) == '无遮挡正面' and
                              len(self.front_grasp_arms) == 1):
                            self.state = 'awaiting_second_target'
                            self.report('awaiting_second_target', required_arm=(
                                'right' if self.front_grasp_arms[0] == 'left' else 'left'))
                        else:
                            self.report('post_grasp_completed', arm=self.attached_workpiece[0])
            elif state in ('rejected', 'fault', 'stopped'):
                step = self.post_grasp_targets[0][0]
                self.last_task_id = task_id
                self.post_grasp_targets.clear()
                self.post_grasp_dual_pending = False
                self.state = 'idle'
                self.report('rejected', reason=f'Post-grasp {step} {state}: {status.get("reason", "")}')
            return
        if state in ('running', 'recovering'):
            self.saw_motion = True
        if (state == 'completed' and self.joint_restore_steps is not None and
                self.joint_restore_index + 1 < len(self.joint_restore_steps)):
            self.last_task_id = task_id
            completed_joint5 = self.joint_restore_steps[self.joint_restore_index]
            remaining = (float(self.restore_source_record['tool_z_rotation_deg']) +
                         math.degrees(completed_joint5 - self.joint_restore_reference[4]))
            residual_record = {**self.restore_source_record,
                               'tool_z_rotation_deg': remaining,
                               'rotation_matrix': tool_z_rotation_matrix(remaining),
                               'rotation_matrix_type': 'tool_z_delta'}
            try:
                self.save_record(self.tool_z_angle_file, residual_record)
            except OSError as exc:
                self.state = 'fault'
                self.report('rejected', reason=f'Could not save remaining tool Z rotation: {exc}; restart scene after checking robot')
                return
            self.joint_restore_index += 1
            self.state = 'next'
            self.next_at = time.monotonic() + .2
            self.report('restore_joint5_step_completed', step=self.joint_restore_index,
                        total=len(self.joint_restore_steps))
            return
        if state == 'completed':
            restoring = self.restore_mode
            side, position, _, tip_position = self.targets[0]
            yaw_deg, orientation = self.yaw_candidates[self.yaw_index]
            matrix = tool_z_rotation_matrix(yaw_deg)
            record = {'arm': side, 'frame_id': 'body_base', 'task_id': task_id,
                      'flange_position': position, 'tip_position': tip_position,
                      'flange_orientation': orientation, 'rotation_matrix': matrix,
                      'rotation_matrix_type': 'tool_z_delta',
                      'flange_rotation_matrix': rotation_matrix(orientation),
                      'tool_z_rotation_deg': yaw_deg, 'restored_tool_z': restoring}
            try:
                self.save_last_rotation(record)
            except OSError as exc:
                self.get_logger().warning(f'Could not save final rotation matrix: {exc}')
            try:
                if restoring:
                    # A surviving file must say restored, so a failed unlink
                    # cannot turn the same rotation into a second command.
                    self.save_record(self.tool_z_angle_file, record)
                    self.tool_z_angle_file.unlink(missing_ok=True)
                elif abs(yaw_deg) >= 1e-6:
                    self.save_record(self.tool_z_angle_file, record)
            except OSError as exc:
                self.get_logger().warning(f'Could not update pending tool Z rotation: {exc}')
            self.last_task_id = task_id
            self.targets.pop(0)
            if not restoring:
                self.target_index = getattr(self, 'target_index', 0) + 1
                if self.target_index == 1 and self.targets:
                    self.position1_solution = (yaw_deg, orientation)
                    self.position1_goal = (side, position, orientation)
                    self.position1_tip = tip_position
                elif self.target_index == 2:
                    self.grasp_target = (side, yaw_deg)
            self.yaw_candidates = None
            self.yaw_index = 0
            self.state = 'next' if self.targets else 'idle'
            self.restore_mode = False
            self.joint_restore_steps = None
            self.next_at = time.monotonic() + (self.interval if self.targets and not restoring else .2)
            self.report('restore_completed' if restoring else 'step_completed',
                        remaining=len(self.targets), **record)
            if not restoring and self.target_index == 2:
                if side == 'left':
                    self.enable_grippers_before_grasp('left', 'left_grasp')
                elif getattr(self, 'vision_label', None) in ('无遮挡正面', '无遮挡背面'):
                    self.enable_grippers_before_grasp('right', 'right_direct_grasp')
        elif (state == 'rejected' and not self.saw_motion and not self.restore_mode
              and getattr(self, 'target_index', None) == 1
              and getattr(self, 'position1_goal', None) is not None
              and getattr(self, 'descent_retry_count', 0) < 4 and self.allow_tool_z_rotation
              and self.retryable_ik_rejection(status.get('reason', ''))
              and VisionTargets.retry_descent_orientation(self, task_id, status.get('reason', ''))):
            return
        elif (state == 'rejected' and not self.saw_motion and not self.restore_mode
              and self.allow_tool_z_rotation
              and self.retryable_ik_rejection(status.get('reason', '')) and
              self.yaw_candidates is not None and self.yaw_index + 1 < len(self.yaw_candidates)):
            self.last_task_id = task_id
            self.yaw_index += 1
            self.state = 'next'
            self.next_at = time.monotonic() + .3
            self.report('retrying_orientation', arm=self.targets[0][0],
                        attempted=self.yaw_index, total=len(self.yaw_candidates),
                        reason=status.get('reason', ''))
        elif state in ('rejected', 'fault', 'stopped'):
            self.last_task_id = task_id
            if self.restore_mode and self.joint_restore_steps is not None and self.saw_motion:
                try:
                    current = json.loads(self.tool_z_angle_file.read_text(encoding='utf-8'))
                    self.save_record(self.tool_z_angle_file, {**current, 'restore_uncertain': True})
                except (OSError, ValueError) as exc:
                    self.get_logger().warning(f'Could not mark interrupted joint restore: {exc}')
            exhausted = (state == 'rejected' and not self.restore_mode and
                         self.yaw_candidates is not None and
                         self.yaw_index + 1 == len(self.yaw_candidates) and
                         self.allow_tool_z_rotation and
                         self.retryable_ik_rejection(status.get('reason', '')))
            count = len(self.yaw_candidates) if exhausted else None
            self.targets.clear()
            self.grasp_target = None
            self.yaw_candidates = None
            self.state = 'idle'
            self.restore_mode = False
            self.joint_restore_steps = None
            reason = f'Arm control {state}: {status.get("reason", "")}'
            if exhausted:
                reason += f'; all {count} tool Z orientations rejected'
            self.report('rejected', reason=reason)

    def save_last_rotation(self, record):
        self.save_record(self.last_rotation_file, record)

    def retry_descent_orientation(self, task_id, reason):
        """After a planning-only descent failure, rotate at the approach point and replan."""
        side, position, chosen_orientation = self.position1_goal
        if not self.targets or self.targets[0][0] != side:
            return False
        old_angle, _ = self.position1_solution
        base_orientation = undo_tool_z_rotation(chosen_orientation, old_angle)
        candidates = list(tool_z_rotations(base_orientation))
        next_index = next((index + 1 for index, (angle, _) in enumerate(candidates[:-1])
                           if abs(angle - old_angle) < 1e-6), None)
        if next_index is None:
            return False
        self.last_task_id = task_id
        self.descent_retry_count += 1
        self.targets.insert(0, (side, position, base_orientation, self.position1_tip))
        self.target_index = 0
        self.yaw_candidates = candidates
        self.yaw_index = next_index
        self.state = 'next'
        self.next_at = time.monotonic() + .3
        self.report('retrying_descent_orientation', arm=side, attempted=next_index,
                    total=len(candidates), reason=reason)
        return True

    @staticmethod
    def save_record(path, record):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=path.parent, prefix=path.name + '.',
                                             delete=False) as output:
                temporary = output.name
                json.dump(record, output, ensure_ascii=False, indent=2)
                output.write('\n')
            os.replace(temporary, path)
        finally:
            if temporary is not None and os.path.exists(temporary):
                os.unlink(temporary)

    def tick(self):
        if self.state == 'gripper_waiting':
            sent_at = (self.gripper_enable_pending[1] if self.gripper_enable_pending is not None
                       else self.gripper_pending[3])
            if time.monotonic() - sent_at > 15.:
                enabling = self.gripper_enable_pending is not None
                self.gripper_enable_pending = None
                self.gripper_enable_queue.clear()
                self.gripper_move_after_enable = None
                self.gripper_pending = None
                self.state = 'fault'
                self.report('rejected', reason=('Gripper enable not confirmed within 15 s' if enabling
                                                else 'Gripper did not reach target within 15 s'))
            return
        if self.state == 'waiting':
            if time.monotonic() - self.sent_at > 120.:
                # Do not accept a new target while execution may still be in progress.
                self.state = 'fault'
                self.report('rejected', reason='No terminal arm control status within 120 s; restart scene after checking robot')
            return
        if self.state == 'next':
            if time.monotonic() >= self.next_at:
                if (getattr(self, 'start_sequence_stage', None) == 'body_photo'
                        and self.preset_name is None):
                    self.state = 'idle'
                    VisionTargets.on_arm_position(self, SimpleNamespace(data=2))
                    if self.state != 'waiting' or self.preset_name != 'body_photo':
                        self.start_sequence_stage = None
                elif getattr(self, 'post_grasp_targets', None):
                    self.send_post_grasp_next()
                elif (getattr(self, 'preset_name', None) == 'body_photo'
                      and not getattr(self, 'preset_targets', None)
                      and any((self.preset_name, kind) in self.arm_positions
                              for kind in ('body_pose', 'body_pose_ptp'))):
                    VisionTargets.send_body_pose_preset(self)
                elif self.preset_targets:
                    self.send_preset_next()
                elif self.joint_restore_steps is not None:
                    try:
                        self.send_joint_restore_step()
                    except ValueError as exc:
                        self.targets.clear()
                        self.yaw_candidates = None
                        self.joint_restore_steps = None
                        self.restore_mode = False
                        self.state = 'idle'
                        self.report('rejected', reason=str(exc))
                else:
                    try:
                        if getattr(self, 'second_turn_after_prepare', False):
                            self.second_turn_after_prepare = False
                            self.send_second_body_turn(self.targets[0][0])
                        elif self.body_shift_m:
                            self.send_body_shift()
                        else:
                            self.send_next()
                    except ValueError as exc:
                        self.targets.clear()
                        self.state = 'idle'
                        self.report('rejected', reason=str(exc))
            return
        if self.state not in ('idle', 'awaiting_second_target'):
            return
        with self.queue_lock:
            payload, self.queue = self.queue, None
        if payload is None:
            return
        try:
            if not self.dry_run and self.control_state in ('running', 'recovering'):
                raise ValueError('Arm control is busy')
            label, parsed = parse_vision_message(payload)
            routed = route_targets(parsed, label=label)
            if not routed:
                raise ValueError('No target position to execute')
            if self.state == 'awaiting_second_target':
                if getattr(self, 'back_first_arm', None) is not None and not self.front_grasp_arms:
                    if (self.back_first_arm == 'right' and
                            (self.right_post_grasp_joints is None or self.left_handover_joints is None)):
                        raise ValueError('Back right-arm handover requires configured right_post_grasp_joints and left_handover_joints')
                    self.back_second_payload = payload
                    transfer = ('transfer', 'joints', self.left_post_grasp_joints, None) if self.back_first_arm == 'left' else (
                        'right_transfer', 'joints', self.right_post_grasp_joints, None)
                    self.post_grasp_targets = [('body_zero', 'body_joints', self.arm_positions['body_home', 'body_joints'], None), transfer]
                    self.state = 'next'
                    self.next_at = time.monotonic() + .2
                    self.report('back_handover_queued', arm=self.back_first_arm)
                    return
                second_arm = 'right' if self.front_grasp_arms[0] == 'left' else 'left'
                routed = [(second_arm, target) for _, target in routed]
                second_grasp = True
                y = parsed[0][0][1]
                second_turn = ((second_arm == 'left' and y < -.05) or
                               (second_arm == 'right' and y >= .05))
                second_small_turn = ((second_arm == 'left' and -.05 <= y <= .10) or
                                     (second_arm == 'right' and -.10 <= y < .05))
                self.state = 'idle'
            else:
                second_grasp = False
                second_turn = False
                second_small_turn = False
            self.targets = []
            self.target_index = 0
            self.position1_solution = None
            self.position1_goal = None
            self.position1_tip = None
            self.descent_retry_count = 0
            self.grasp_target = None
            self.vision_label = label
            self.body_pre_grasp_pending = False
            self.body_pre_grasp_used = False
            self.body_shift_pending = False
            self.body_shift_used = False
            self.second_arm_prepare_pending = False
            self.second_turn_after_prepare = False
            x = parsed[0][0][0]
            self.body_shift_m = -.20 if x <= .60 else .20 if x >= .75 else 0.
            self.yaw_candidates = None
            self.yaw_index = 0
            for side, target in routed:
                tip_position, tip_orientation = target
                flange_position, flange_orientation = flange_target_from_tip(
                    tip_position, tip_orientation, self.tool_tip_offset_z_m)
                self.targets.append((side, flange_position, flange_orientation, tip_position))
            if self.dry_run:
                if second_turn:
                    self.report('second_arm_prepare_previewed', arm=routed[0][0],
                                joints_rad=list(self.second_arm_prepare_joints(routed[0][0])))
                    self.report('body_second_turn_previewed', arm=routed[0][0],
                                j6_target_rad=(-1.57 if routed[0][0] == 'left'
                                               else 1.57))
                elif second_small_turn:
                    self.report('body_second_turn_previewed', arm=routed[0][0],
                                j6_target_rad=(-.5 if routed[0][0] == 'left' else .5))
                elif not second_grasp and -.10 <= parsed[0][0][1] <= .10:
                    self.report('body_pre_grasp_previewed', arm=routed[0][0],
                                joints_rad=list(self.body_pre_grasp_joints(routed[0][0])))
                for _, position, orientation, _ in self.targets:
                    self.report('previewed', flange_position=position,
                                flange_orientation=orientation)
                if self.body_shift_m:
                    self.report('body_shift_previewed', shift_x_m=self.body_shift_m)
                self.targets.clear()
                return
            if second_turn:
                self.send_second_arm_prepare(routed[0][0])
            elif second_small_turn:
                self.send_second_body_turn(routed[0][0],
                                           target_j6_rad=(-.5 if routed[0][0] == 'left' else .5))
            elif not second_grasp and -.10 <= parsed[0][0][1] <= .10:
                self.send_body_pre_grasp(routed[0][0])
            elif self.body_shift_m:
                self.send_body_shift()
            else:
                self.send_next()
        except (ValueError, UnicodeError, json.JSONDecodeError) as exc:
            self.report('rejected', reason=str(exc))

    @staticmethod
    def body_pre_grasp_joints(side):
        return (0., .86725, -1.50561, -.16162, 0., -.5 if side == 'left' else .5)

    def send_body_pre_grasp(self, side):
        if self.body_joint_publisher.get_subscription_count() == 0:
            self.targets.clear()
            self.report('rejected', reason='No body joint control subscriber for pre-grasp')
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'body_joint_{index}' for index in range(1, 7)]
        message.position = list(self.body_pre_grasp_joints(side))
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.body_pre_grasp_pending = True
        self.body_pre_grasp_used = True
        self.state = 'waiting'
        self.body_joint_publisher.publish(message)
        self.report('body_pre_grasp_sent', arm=side, joints_rad=list(message.position))

    def send_second_body_turn(self, side, target_j6_rad=None):
        joints = self.current_body_joints()
        if joints is None:
            raise ValueError('No fresh body joint feedback for second grasp J6 turn')
        if self.body_joint_publisher.get_subscription_count() == 0:
            raise ValueError('No body joint control subscriber for second grasp turn')
        target = list(joints)
        target[5] = (-1.57 if side == 'left' else 1.57) if target_j6_rad is None else target_j6_rad
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'body_joint_{index}' for index in range(1, 7)]
        message.position = target
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.body_pre_grasp_pending = True
        self.body_pre_grasp_used = True
        self.state = 'waiting'
        self.body_joint_publisher.publish(message)
        self.report('body_second_turn_sent', arm=side, joints_rad=target,
                    j6_target_rad=target[5])

    @staticmethod
    def second_arm_prepare_joints(side):
        return ((1.5, .8, -2., -1., 0., 0., 0.) if side == 'left' else
                (-1.5, .8, 2., -1., 0., 0., 0.))

    def send_second_arm_prepare(self, side):
        publisher = self.joint_publishers[side]
        if publisher.get_subscription_count() == 0:
            raise ValueError(f'No {side} arm joint control subscriber for second grasp prepare')
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'{side}_joint_{index}' for index in range(1, 8)]
        message.position = list(self.second_arm_prepare_joints(side))
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.second_arm_prepare_pending = True
        self.second_turn_after_prepare = True
        self.state = 'waiting'
        publisher.publish(message)
        self.report('second_arm_prepare_sent', arm=side, joints_rad=list(message.position))

    def on_body_pose(self, message):
        if message.header.frame_id == 'body_base':
            self.body_pose_feedback = (message, time.monotonic())

    def send_body_shift(self):
        if self.body_pose_publisher.get_subscription_count() == 0:
            raise ValueError('No body pose control subscriber for X shift')
        feedback = self.body_pose_feedback
        if feedback is None or time.monotonic() - feedback[1] > .5:
            raise ValueError('No fresh body pose feedback for X shift')
        current = feedback[0]
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'body_base'
        target.pose.position.x = current.pose.position.x + self.body_shift_m
        target.pose.position.y = current.pose.position.y
        target.pose.position.z = current.pose.position.z
        target.pose.orientation = current.pose.orientation
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.body_shift_pending = True
        self.body_shift_used = True
        self.state = 'waiting'
        self.body_pose_publisher.publish(target)
        self.report('body_shift_sent', shift_x_m=self.body_shift_m,
                    position_m=[target.pose.position.x, target.pose.position.y,
                                target.pose.position.z])

    def send_next(self):
        side, position, orientation, tip_position = self.targets[0]
        if self.yaw_candidates is None:
            if not self.restore_mode and self.target_index == 1:
                self.yaw_candidates = [self.position1_solution]
            else:
                self.yaw_candidates = (list(tool_z_rotations(orientation))
                                       if self.allow_tool_z_rotation else [(0., orientation)])
            self.yaw_index = 0
        yaw_deg, orientation = self.yaw_candidates[self.yaw_index]
        ptp = not self.restore_mode and self.target_index == 0
        publisher = self.ptp_publishers[side] if ptp else self.target_publishers[side]
        if publisher.get_subscription_count() == 0:
            self.targets.clear()
            self.state = 'idle'
            self.report('rejected', reason=f'No {side} arm {"PTP" if ptp else "Cartesian"} control subscriber')
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'body_base'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = orientation
        self.waiting_task_id = None
        self.saw_motion = False
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        publisher.publish(message)
        self.report('sent', arm=side, frame_id='body_base', tip_position=tip_position,
                    flange_position=position, orientation=orientation,
                    motion_mode='ptp' if ptp else 'cartesian',
                    tool_z_rotation_deg=yaw_deg, attempt=self.yaw_index+1,
                    remaining=len(self.targets))

    def destroy_node(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        return super().destroy_node()


def main():
    rclpy.init()
    node = VisionTargets()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
