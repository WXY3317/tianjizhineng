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
from tf2_ros import Buffer,TransformListener
from marvin_sim.node import make_pose
from marvin_sim.kinematics import quaternion_matrix,pose_error
from marvin_mesh_sim.frames import J4Frames,J4_FRAME
from marvin_mesh_sim.gento_kinematics import GentoMeshKinematics
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
                f'sdk_root:={ROOT/"tianji-robot-SDK"}','rviz:=false','panel:=false'],
                env=dict(os.environ,ROS_DOMAIN_ID='94'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            node=Node('mesh_sim_probe',context=context);executor.add_node(node)
            tf_buffer=Buffer();tf_listener=TransformListener(tf_buffer,node)
            poses=[[],[]]
            for a,side in enumerate(('left','right')):
                node.create_subscription(PoseStamped,f'/marvin_sim/{side}/tcp_pose',lambda m,index=a:poses[index].append(m),10)
            pose_pubs=[node.create_publisher(PoseStamped,f'/marvin_sim/{side}/target_pose',10) for side in ('left','right')]
            model=[];arms=[];statuses=[];body_states=[];body_status=[];hand_states=[];hand_status=[]
            node.create_subscription(JointState,'/joint_states',lambda m:model.append(dict(zip(m.name,m.position))),10)
            node.create_subscription(JointState,'/marvin_sim/sdk_joint_states',lambda m:arms.append(np.array(m.position)),10)
            node.create_subscription(String,'/marvin_sim/status',lambda m:statuses.append(json.loads(m.data)),10)
            node.create_subscription(JointState,'/marvin_mesh_sim/body_states',lambda m:body_states.append(dict(zip(m.name,m.position))),10)
            node.create_subscription(String,'/marvin_mesh_sim/body_status',lambda m:body_status.append(json.loads(m.data)),10)
            node.create_subscription(JointState,'/marvin_mesh_sim/hand_states',lambda m:hand_states.append(dict(zip(m.name,m.position))),10)
            node.create_subscription(String,'/marvin_mesh_sim/hand_status',lambda m:hand_status.append(json.loads(m.data)),10)
            target=node.create_publisher(JointState,'/marvin_sim/target_joints',10)
            fingers=node.create_publisher(JointState,'/marvin_mesh_sim/hand_targets',10)
            body_pub=node.create_publisher(JointState,'/marvin_mesh_sim/body_targets',10)
            def wait_for(condition,timeout=12):
                end=time.monotonic()+timeout
                while time.monotonic()<end:
                    assert process.poll() is None,log_path.read_text()
                    executor.spin_once(timeout_sec=.03)
                    if condition():return
                raise AssertionError(log_path.read_text())
            wait_for(lambda:model and arms and statuses and target.get_subscription_count() and fingers.get_subscription_count())
            assert len(model[-1])==44
            assert statuses[-1].get('backend')=='original_gento_sdk'
            goal=arms[-1].copy();goal[0]+=.035;goal[7]-=.035
            msg=JointState();msg.name=JOINT_NAMES;msg.position=goal.tolist();target.publish(msg)
            config=yaml.safe_load((PACKAGE/'config/hands.yaml').read_text())
            names=[j['name'] for j in config['L']['joints']];hand_goal=closed_posture(names)
            hand=JointState();hand.name=names;hand.position=hand_goal;fingers.publish(hand)
            wait_for(lambda:np.max(np.abs(arms[-1]-goal))<1e-6
                     and max(abs(model[-1][n]-v) for n,v in zip(names,hand_goal))<1e-6)
            wait_for(lambda:hand_states and hand_status and hand_status[-1].get('backend')=='original_sg100_sdk'
                     and max(abs(hand_states[-1][n]-v) for n,v in zip(names,hand_goal))<1e-6)
            for side in ('L','R'):
                assert f'{side}_Joint1' in model[-1]
            assert all(abs(model[-1][j['name']])<1e-10 for j in config['R']['joints'])
            wait_for(lambda:body_pub.get_subscription_count())
            body=JointState()
            body.name=['Gento_Luna_Leg_J2_Joint','Head_J1_joint'];body.position=[.1,.15]
            body_pub.publish(body)
            wait_for(lambda:abs(model[-1][body.name[0]]-.1)<1e-6
                     and abs(model[-1][body.name[1]]-.15)<1e-6)
            wait_for(lambda:body_states and body_status and body_status[-1].get('backend')=='original_gento_sdk'
                     and abs(body_states[-1][body.name[1]]-.15)<1e-6)
            assert np.max(np.abs(arms[-1]-goal))<1e-6
            # Move downstream torso joints: J4-to-shoulder transforms must update.
            body.name=['Gento_Luna_Leg_J5_Joint','Gento_Luna_Leg_J6_Joint'];body.position=[.08,-.05]
            body_pub.publish(body)
            wait_for(lambda:all(abs(model[-1][n]-v)<1e-6 for n,v in zip(body.name,body.position)))
            wait_for(lambda:len(model)>=3 and all(abs(model[-1][n]-model[-3][n])<1e-12 for n in body.name))
            frames=J4Frames(PACKAGE);kine=GentoMeshKinematics(ROOT/'tianji-robot-SDK',PACKAGE)
            def matrix(message):
                result=np.eye(4);p,q=message.position,message.orientation
                result[:3,3]=[p.x,p.y,p.z];result[:3,:3]=quaternion_matrix([q.x,q.y,q.z,q.w])
                return result
            wait_for(lambda:all(poses) and all(p.get_subscription_count() for p in pose_pubs))
            for arm,side in enumerate('LR'):
                assert poses[arm][-1].header.frame_id==J4_FRAME
                wait_for(lambda:tf_buffer.can_transform(J4_FRAME,f'flange_{side}',rclpy.time.Time()))
                def agrees_with_tf():
                    t=tf_buffer.lookup_transform(J4_FRAME,f'flange_{side}',rclpy.time.Time()).transform
                    expected=np.eye(4);expected[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
                    expected[:3,:3]=quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
                    return max(pose_error(matrix(poses[arm][-1].pose),expected))<1e-5
                wait_for(agrees_with_tf)
                q=arms[-1][7*arm:7*arm+7].copy();q[0]+=.02 if arm==0 else -.02
                body_q=[model[-1][n] for n in frames.body_names]
                desired=frames.fk(arm,kine.fk(arm,q),body_q)
                pose_pubs[arm].publish(make_pose(desired,J4_FRAME,node.get_clock().now().to_msg()))
                wait_for(lambda:max(pose_error(matrix(poses[arm][-1].pose),desired))<1e-5)
            # Shoulder-local goals must no longer be interpreted as J4 goals.
            previous=len(statuses)
            pose_pubs[0].publish(make_pose(desired,'Base_L',node.get_clock().now().to_msg()))
            wait_for(lambda:len(statuses)>previous and 'Target pose frame must be' in statuses[-1].get('error',''))
            # Replay the user's screenshot through the actual ROS/native-SDK chain.
            from test_ik_recovery import REFERENCE_DEG,screenshot_target
            body.position=[0.,0.];body_pub.publish(body)
            wait_for(lambda:all(abs(model[-1][n])<1e-8 for n in body.name))
            wait_for(lambda:all(abs(model[-1][n]-model[-3][n])<1e-12 for n in body.name))
            reference=np.radians(REFERENCE_DEG)
            arm_goal=JointState();arm_goal.name=JOINT_NAMES[7:];arm_goal.position=reference.tolist()
            target.publish(arm_goal)
            wait_for(lambda:np.max(np.abs(arms[-1][7:]-reference))<1e-6)
            previous=len(statuses);desired=screenshot_target()
            pose_pubs[1].publish(make_pose(desired,J4_FRAME,node.get_clock().now().to_msg()))
            wait_for(lambda:any(s.get('ik_info',{}).get('fallback') and s.get('accepted')
                                for s in statuses[previous:]))
            wait_for(lambda:max(pose_error(matrix(poses[1][-1].pose),desired))<1e-5)
        finally:
            if process is not None and process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            if node is not None:node.destroy_node()
            executor.shutdown();context.shutdown()
