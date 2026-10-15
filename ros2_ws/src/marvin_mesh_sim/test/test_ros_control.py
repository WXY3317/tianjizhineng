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
from std_msgs.msg import String
import yaml

from marvin_mesh_sim.hands import closed_posture
from marvin_sim.kinematics import JOINT_NAMES


ROOT=Path(__file__).resolve().parents[4]
PACKAGE=ROOT/'ros2_ws/src/marvin_mesh_sim'


def test_ros_combined_model_sdk_arm_and_fingers(tmp_path):
    context=Context();rclpy.init(context=context,domain_id=94)
    executor=SingleThreadedExecutor(context=context)
    node=None;process=None;log_path=tmp_path/'launch.log'
    with log_path.open('w') as log:
        try:
            process=subprocess.Popen([
                'ros2','launch','marvin_mesh_sim','simulation.launch.py',
                f'sdk_root:={ROOT/"tianji_armSDK"}','rviz:=false','panel:=false'],
                env=dict(os.environ,ROS_DOMAIN_ID='94'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node=Node('mesh_sim_probe',context=context);executor.add_node(node)
            model=[];arms=[];statuses=[]
            node.create_subscription(JointState,'/joint_states',lambda m:model.append(dict(zip(m.name,m.position))),10)
            node.create_subscription(JointState,'/marvin_sim/sdk_joint_states',lambda m:arms.append(np.array(m.position)),10)
            node.create_subscription(String,'/marvin_sim/status',lambda m:statuses.append(json.loads(m.data)),10)
            target=node.create_publisher(JointState,'/marvin_sim/target_joints',10)
            fingers=node.create_publisher(JointState,'/marvin_mesh_sim/hand_targets',10)
            def wait_for(condition,timeout=12):
                end=time.monotonic()+timeout
                while time.monotonic()<end:
                    assert process.poll() is None,log_path.read_text()
                    executor.spin_once(timeout_sec=.03)
                    if condition():return
                raise AssertionError(log_path.read_text())
            wait_for(lambda:model and arms and statuses and target.get_subscription_count() and fingers.get_subscription_count())
            assert len(model[-1])==36
            goal=arms[-1].copy();goal[0]+=.035;goal[7]-=.035
            msg=JointState();msg.name=JOINT_NAMES;msg.position=goal.tolist();target.publish(msg)
            config=yaml.safe_load((PACKAGE/'config/hands.yaml').read_text())
            names=[j['name'] for j in config['L']['joints']];hand_goal=closed_posture(names)
            hand=JointState();hand.name=names;hand.position=hand_goal;fingers.publish(hand)
            wait_for(lambda:np.max(np.abs(arms[-1]-goal))<1e-6
                     and max(abs(model[-1][n]-v) for n,v in zip(names,hand_goal))<1e-6)
            for side in ('L','R'):
                assert f'{side}_thumb_j1' in model[-1]
            assert all(abs(model[-1][j['name']])<1e-10 for j in config['R']['joints'])
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            if node is not None:node.destroy_node()
            executor.shutdown();context.shutdown()
