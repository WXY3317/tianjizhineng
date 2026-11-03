"""Forward MQTT vision poses in BD_J6 to sequential arm pose commands in body_base."""
import json
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .geometry import parse_targets, route_targets, transform_target


class VisionTargets(Node):
    def __init__(self):
        super().__init__('scene_vision_targets')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', '/vision_targe')
        self.declare_parameter('max_body_pose_age_s', .5)
        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        topic = self.get_parameter('mqtt_topic').value
        self.max_body_pose_age_ns = int(self.get_parameter('max_body_pose_age_s').value * 1e9)
        if not host or not 1 <= port <= 65535 or not topic or self.max_body_pose_age_ns <= 0:
            raise ValueError('Invalid MQTT or body feedback parameters')

        self.body_pose = None
        self.control_state = None
        self.last_task_id = None
        self.queue = None
        self.queue_lock = threading.Lock()
        self.targets = []
        self.state = 'idle'
        self.sent_at = 0.
        self.next_at = 0.
        self.waiting_task_id = None
        self.target_publishers = {
            side: self.create_publisher(PoseStamped, f'/gento_control/{side}/target_pose', 1)
            for side in ('left', 'right')}
        self.status_pub = self.create_publisher(String, '/scene/status', 10)
        self.create_subscription(PoseStamped, '/gento_feedback/body/tcp_pose', self.on_body_pose, 10)
        self.create_subscription(String, '/gento_control/status', self.on_control_status, 10)
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
        with self.queue_lock:
            if self.queue is None and self.state == 'idle':
                self.queue = bytes(message.payload)
            else:
                self.get_logger().warning('Vision target ignored while another target is active')

    def on_body_pose(self, message):
        if message.header.frame_id == 'body_base':
            self.body_pose = message

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
        if task_id == self.last_task_id and state != 'rejected':
            return
        if self.waiting_task_id is None:
            self.waiting_task_id = task_id
        if task_id != self.waiting_task_id:
            return
        if state == 'completed':
            self.last_task_id = task_id
            self.targets.pop(0)
            self.state = 'next' if self.targets else 'idle'
            self.next_at = time.monotonic() + .2
            self.report('step_completed', remaining=len(self.targets))
        elif state in ('rejected', 'fault', 'stopped'):
            self.targets.clear()
            self.state = 'idle'
            self.report('rejected', reason=f'Arm control {state}: {status.get("reason", "")}')

    def body_transform(self):
        pose = self.body_pose
        if pose is None:
            raise ValueError('No BD_J6 feedback available')
        stamp = pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if age < 0 or age > self.max_body_pose_age_ns:
            raise ValueError('BD_J6 feedback is stale')
        p = pose.pose.position
        q = pose.pose.orientation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    def tick(self):
        if self.state == 'waiting':
            if time.monotonic() - self.sent_at > 120.:
                # Do not accept a new target while execution may still be in progress.
                self.state = 'fault'
                self.report('rejected', reason='No terminal arm control status within 120 s; restart scene after checking robot')
            return
        if self.state == 'next':
            if time.monotonic() >= self.next_at:
                self.send_next()
            return
        if self.state != 'idle':
            return
        with self.queue_lock:
            payload, self.queue = self.queue, None
        if payload is None:
            return
        try:
            if self.control_state in ('running', 'recovering'):
                raise ValueError('Arm control is busy')
            routed = route_targets(parse_targets(payload))
            if not routed:
                raise ValueError('No target position to execute')
            body_position, body_orientation = self.body_transform()
            self.targets = [(side, *transform_target(body_position, body_orientation, target))
                            for side, target in routed]
            self.send_next()
        except (ValueError, UnicodeError, json.JSONDecodeError) as exc:
            self.report('rejected', reason=str(exc))

    def send_next(self):
        side, position, orientation = self.targets[0]
        publisher = self.target_publishers[side]
        if publisher.get_subscription_count() == 0:
            self.targets.clear()
            self.state = 'idle'
            self.report('rejected', reason=f'No {side} arm control subscriber')
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'body_base'
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = orientation
        self.waiting_task_id = None
        self.sent_at = time.monotonic()
        self.state = 'waiting'
        publisher.publish(message)
        self.report('sent', arm=side, position=position, remaining=len(self.targets))

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
