"""ROS arm interfaces backed by the original Gento SDK, sharing one connection."""
import json
import time
from pathlib import Path

from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray,String
from std_srvs.srv import Trigger

from marvin_sim.node import Bridge,make_pose
from marvin_sim.kinematics import JOINT_NAMES,quaternion_matrix
from .frames import J4Frames,J4_FRAME,TCP_FRAMES
import numpy as np
from .gento_kinematics import GentoMeshKinematics
from .gento_robot_client import GentoRobotClient


def resolve_gento(node):
    node.declare_parameter('gento_sdk_root','')
    configured=node.get_parameter('gento_sdk_root').value
    sdk=Path(node.get_parameter('sdk_root').value)
    return configured or str(sdk if (sdk/'PYTHON_SDK/GentoRobot.py').exists() else sdk.parent/'tianji-robot-SDK')


class GentoBridge(Bridge):
    def __init__(self):
        Node.__init__(self,'gento_sdk_bridge')
        for name in ('sdk_root','description_root','sim_ip'):self.declare_parameter(name,'')
        model=self.get_parameter('description_root').value
        self.frames=J4Frames(model)
        self.kine=GentoMeshKinematics(resolve_gento(self),model)
        self.client=GentoRobotClient(self.kine,model,self.get_parameter('sim_ip').value)
        self.state='ready';self.serial=None;self.last_refresh=time.monotonic()
        self.q=self.client.read()
        self.joints=self.create_publisher(JointState,'/marvin_sim/sdk_joint_states',10)
        self.status=self.create_publisher(String,'/marvin_sim/status',10)
        self.poses=[self.create_publisher(PoseStamped,f'/marvin_sim/{side}/tcp_pose',10) for side in ('left','right')]
        self.create_subscription(JointState,'/marvin_sim/target_joints',self.joint_goal,10)
        for arm,side in enumerate(('left','right')):
            self.create_subscription(PoseStamped,f'/marvin_sim/{side}/target_pose',lambda msg,a=arm:self.pose_goal(a,msg),10)
            self.create_subscription(Float64MultiArray,f'/marvin_sim/{side}/target_rpy',
                                     lambda msg,a=arm:self.rpy_goal(a,msg,J4_FRAME),10)
        self.create_service(Trigger,'/marvin_sim/stop',self.stop)
        self.create_service(Trigger,'/marvin_sim/enable',self.enable)
        self.last_status=String(data=json.dumps({'state':'ready','backend':'original_gento_sdk','reference_frame':J4_FRAME,'sdk_ip':self.client.address}))
        self.create_timer(.02,self.publish)
        self.create_timer(.2,lambda:self.status.publish(self.last_status))
        self.get_logger().info('Original libGentoSDKPY.so connected: ARM0/ARM1 + BODY/HEAD; native Gento FK/IK')

    def report(self,state,**details):
        return super().report(state,backend='original_gento_sdk',reference_frame=J4_FRAME,tcp_frames=list(TCP_FRAMES),**details)

    def refresh(self):
        self.q=self.client.read()

    def require_ready(self):
        self.refresh()
        if self.state in ('stopped','fault'):raise ValueError('Controller stopped/faulted; enable before sending a target')
        if any(a.m_ARM_State.m_CurState!=1 for a in self.client.feedback.m_ARMS):
            raise ValueError('Both Gento arms must be in position mode')

    def pose_goal(self,arm,msg):
        try:
            self.require_ready()
            if msg.header.frame_id!=J4_FRAME:
                raise ValueError(f'Target pose frame must be {J4_FRAME}')
            body=self.frames.body_from_rt(self.client.feedback,require_stationary=True)
            p,q=msg.pose.position,msg.pose.orientation
            target=np.eye(4);target[:3,3]=[p.x,p.y,p.z]
            target[:3,:3]=quaternion_matrix([q.x,q.y,q.z,q.w])
            local=self.frames.ik_target(arm,target,body)
            result=self.kine.ik(arm,local,self.q[arm*7:arm*7+7])
            self.client.move({arm:result})
            self.report('moving',command='sdk_ik',arm=arm,
                        solution_rad=result.tolist(),target_matrix_m=target.tolist(),
                        accepted=True,ik_info=self.kine.last_ik_info)
        except (ValueError,RuntimeError) as exc:
            self.report('fault' if isinstance(exc,RuntimeError) else self.state,
                        accepted=False,command='sdk_ik',arm=arm,error=str(exc))

    def publish(self):
        try:
            self.refresh();stamp=self.get_clock().now().to_msg()
            msg=JointState();msg.header.stamp=stamp;msg.name=JOINT_NAMES;msg.position=self.q.tolist()
            self.joints.publish(msg)
            for arm,publisher in enumerate(self.poses):
                publisher.publish(make_pose(self.frames.fk(arm,self.kine.fk(arm,self.q[arm*7:arm*7+7]),
                    self.frames.body_from_rt(self.client.feedback)),J4_FRAME,stamp))
            sg=self.client.module.ROBOT_SG.from_buffer_copy(bytes(self.client.robot.sg))
            if self.state=='moving' and all(a.m_ARM_GET.m_ARM_FBK_LowSpdFlag for a in sg.m_ARMS):
                # Require position feedback to match the latest command too.
                target=[v for a in self.client.feedback.m_ARMS for v in a.m_ARM_IN.m_ARM_CMD_Joint_Pos]
                if np.max(np.abs(self.q-np.radians(target)))<1e-5:self.report('completed')
        except (ValueError,RuntimeError) as exc:
            if self.state!='fault':self.report('fault',error=str(exc))
