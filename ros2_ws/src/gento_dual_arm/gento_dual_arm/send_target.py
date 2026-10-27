# 等待 ROS 订阅发现后发送一次新目标，避免发现完成前丢失首条控制消息。
"""Publish one fresh absolute target after ROS subscriber discovery."""
import argparse
import time

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from .conversions import BODY_JOINT_NAMES, JOINT_NAMES, finite, pose_matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--left-joints', nargs=7, type=float, help='Seven absolute left arm angles in radians')
    group.add_argument('--right-joints', nargs=7, type=float, help='Seven absolute right arm angles in radians')
    group.add_argument('--left-pose', nargs=7, type=float, help='Left x y z qx qy qz qw; metres, unit quaternion')
    group.add_argument('--right-pose', nargs=7, type=float, help='Right x y z qx qy qz qw; metres, unit quaternion')
    group.add_argument('--poses', nargs=14, type=float, help='Left x y z qx qy qz qw then right; metres, unit quaternions')
    group.add_argument('--body', nargs=6, type=float, help='Six absolute body joint angles in radians')
    parser.add_argument('--prefix', default='/gento_dual_arm')
    parser.add_argument('--left-frame', default='left_arm_sdk_base')
    parser.add_argument('--right-frame', default='right_arm_sdk_base')
    args = parser.parse_args()
    if args.poses is not None:
        parser.error('--poses is no longer supported: use --left-pose or --right-pose; wait for completed before commanding the other arm')
    finite(next(values for values in (args.left_joints, args.right_joints, args.body, args.left_pose, args.right_pose) if values is not None))
    rclpy.init(args=[])
    node = rclpy.create_node('gento_send_target')
    try:
        if args.body is not None:
            msg = JointState()
            msg.name, msg.position = BODY_JOINT_NAMES, args.body
            messages = [msg]
            topics = ['body/target_joints']
        elif args.left_joints is not None or args.right_joints is not None:
            msg = JointState()
            arm = 0 if args.left_joints is not None else 1
            msg.name = JOINT_NAMES[arm*7:(arm+1)*7]
            msg.position = args.left_joints if arm == 0 else args.right_joints
            messages = [msg]
            topics = ['left/target_joints' if arm == 0 else 'right/target_joints']
        else:
            arm = 0 if args.left_pose is not None else 1
            values = args.left_pose if arm == 0 else args.right_pose
            pose = PoseStamped()
            pose.header.frame_id = (args.left_frame, args.right_frame)[arm]
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = values[3:]
            pose_matrix(pose.pose, 1.)
            messages = [pose]
            topics = ['left/target_pose' if arm == 0 else 'right/target_pose']
        pubs = [node.create_publisher(type(msg), args.prefix.rstrip('/')+'/'+topic, 1)
                for msg, topic in zip(messages, topics)]
        deadline = time.monotonic()+5
        while not all(pub.get_subscription_count() for pub in pubs):
            if time.monotonic() >= deadline:
                raise RuntimeError('Bridge subscriber not found within 5 seconds')
            rclpy.spin_once(node, timeout_sec=.05)
        stamp = node.get_clock().now().to_msg()
        for pub, msg in zip(pubs, messages):
            msg.header.stamp = stamp  # Fresh timestamp for this command.
            pub.publish(msg)
        # Allow reliable middleware delivery; acceptance is reported on /status.
        until = time.monotonic()+.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        node.get_logger().info('Target published once; check bridge /status for acceptance or rejection.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
