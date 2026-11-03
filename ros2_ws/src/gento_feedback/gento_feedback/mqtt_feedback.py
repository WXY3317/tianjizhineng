"""Publish read-only BD_J6 arm TCP feedback to the device's MQTT broker."""
import json
import site

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def pose_payload(pose):
    """Stable JSON schema using ROS metres and xyzw quaternion order."""
    return json.dumps({
        'frame_id': pose.header.frame_id,
        'stamp': {'sec': pose.header.stamp.sec, 'nanosec': pose.header.stamp.nanosec},
        'position': {'x': pose.pose.position.x, 'y': pose.pose.position.y,
                     'z': pose.pose.position.z},
        'orientation': {'x': pose.pose.orientation.x, 'y': pose.pose.orientation.y,
                        'z': pose.pose.orientation.z, 'w': pose.pose.orientation.w},
    }, separators=(',', ':'), allow_nan=False)


class MqttFeedback(Node):
    def __init__(self):
        super().__init__('gento_mqtt_feedback')
        self.declare_parameter('mqtt_host', '127.0.0.1')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic_prefix', 'gento/feedback')
        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        prefix = self.get_parameter('mqtt_topic_prefix').value.strip('/')
        if not host or not 1 <= port <= 65535 or not prefix:
            raise ValueError('Invalid MQTT host, port or topic prefix')
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            # The device's ROS environment disables Python's user site directory.
            site.addsitedir(site.getusersitepackages())
            try:
                import paho.mqtt.client as mqtt
            except ImportError:
                raise RuntimeError('Install python3-paho-mqtt to publish MQTT feedback') from exc
        self.mqtt = mqtt.Client(client_id='gento_bd_j6_feedback')
        self.connected = False
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_disconnect = self.on_disconnect
        self.mqtt.connect_async(host, port, keepalive=30)
        self.mqtt.loop_start()
        for side in ('left', 'right'):
            topic = f'{prefix}/{side}/tcp_pose_bd_j6'
            self.create_subscription(
                PoseStamped, f'/gento_feedback/{side}/tcp_pose_bd_j6',
                lambda msg, destination=topic: self.forward(destination, msg), 10)

    def on_connect(self, _client, _userdata, _flags, result_code):
        self.connected = result_code == 0
        if self.connected:
            self.get_logger().info('MQTT feedback connected')
        else:
            self.get_logger().warning(f'MQTT connection rejected: {result_code}')

    def on_disconnect(self, _client, _userdata, _result_code):
        self.connected = False

    def forward(self, topic, pose):
        if not self.connected:
            return
        if pose.header.frame_id != 'BD_J6':
            self.get_logger().warning('Ignoring non-BD_J6 arm pose')
            return
        result = self.mqtt.publish(topic, pose_payload(pose), qos=0, retain=False)
        if result.rc != 0:
            self.connected = False

    def destroy_node(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        return super().destroy_node()


def main():
    rclpy.init()
    node = MqttFeedback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
