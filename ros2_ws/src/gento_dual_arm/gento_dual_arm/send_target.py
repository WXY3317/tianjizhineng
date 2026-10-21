# 等待 ROS 订阅发现后发送一次新目标，避免发现完成前丢失首条控制消息。
"""Publish one fresh absolute target after ROS subscriber discovery."""
import argparse
import time

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from .conversions import JOINT_NAMES, finite, pose_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--joints', nargs=14, type=float, help='Left 7 then right 7 absolute angles in radians')
    group.add_argument('--poses', nargs=14, type=float, help='Left x y z qx qy qz qw then right; metres, unit quaternions')
    parser.add_argument('--prefix', default='/gento_dual_arm')
    parser.add_argument('--left-frame', default='left_arm_sdk_base')
    parser.add_argument('--right-frame', default='right_arm_sdk_base')
    args = parser.parse_args()
    finite(args.joints if args.joints is not None else args.poses)
    rclpy.init(args=[])
    node = rclpy.create_node('gento_send_target')
    try:
        if args.joints is not None:
            msg = JointState()
            msg.name, msg.position = JOINT_NAMES, args.joints
            messages = [msg]
            topics = ['target_joints']
        else:
            messages = [PoseStamped(), PoseStamped()]
            for arm, pose in enumerate(messages):
                values = args.poses[arm*7:(arm+1)*7]
                pose.header.frame_id = (args.left_frame, args.right_frame)[arm]
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = values[3:]
                pose_matrix(pose.pose, 1.)  # Validate quaternion before publishing.
            topics = ['left/target_pose', 'right/target_pose']
        pubs = [node.create_publisher(type(msg), args.prefix.rstrip('/')+'/'+topic, 1)
                for msg, topic in zip(messages, topics)]
        deadline = time.monotonic()+5
        while not all(pub.get_subscription_count() for pub in pubs):
            if time.monotonic() >= deadline:
                raise RuntimeError('Bridge subscriber not found within 5 seconds')
            rclpy.spin_once(node, timeout_sec=.05)
        stamp = node.get_clock().now().to_msg()
        for pub, msg in zip(pubs, messages):
            msg.header.stamp = stamp  # Exactly the same stamp identifies a dual-arm pair.
            pub.publish(msg)
        # Allow reliable middleware delivery; acceptance is reported on /status.
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        node.get_logger().info('Target published once; check bridge /status for acceptance or rejection.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
