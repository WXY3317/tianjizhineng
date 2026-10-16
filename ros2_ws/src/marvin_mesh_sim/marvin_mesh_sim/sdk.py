"""Python API: Gento FK/IK expressed in J4 + original SDK controls + hand simulator."""
import json
from pathlib import Path
import socket
import uuid

import numpy as np

from .gento_robot_client import GentoRobotClient
from .gento_kinematics import GentoMeshKinematics
from .frames import J4Frames,J4_FRAME


class MeshSimulationSDK:
    def __init__(self,sdk_root,model_root=None,sim_ip=''):
        if model_root is None:
            from ament_index_python.packages import get_package_share_directory
            model_root=get_package_share_directory('marvin_mesh_sim')
        root=Path(sdk_root)
        if not (root/'PYTHON_SDK/GentoRobot.py').exists():root=root.parent/'tianji-robot-SDK'
        self.reference_frame=J4_FRAME
        self.frames=J4Frames(model_root)
        self.kinematics=GentoMeshKinematics(root,Path(model_root))
        self.client=GentoRobotClient(self.kinematics,model_root,sim_ip=sim_ip)
        self.robot=self.client.robot
        self.hand_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.hand_socket.settimeout(1.)

    @staticmethod
    def arm_index(arm):
        if arm in ('A','L','left',0):return 0
        if arm in ('B','R','right',1):return 1
        raise ValueError('Arm must be A/L/left or B/R/right')

    def get_joint_positions(self,arm):
        a=self.arm_index(arm)
        return np.degrees(self.client.read()[7*a:7*a+7]).tolist()

    def fk(self,arm,joints_deg=None):
        a=self.arm_index(arm)
        joints_deg=self.get_joint_positions(arm) if joints_deg is None else joints_deg
        self.client.read()
        body=self.frames.body_from_rt(self.client.feedback)
        return self.frames.fk(a,self.kinematics.fk(a,np.radians(joints_deg)),body)

    def ik(self,arm,pose_m,reference_deg=None):
        a=self.arm_index(arm)
        reference_deg=self.get_joint_positions(arm) if reference_deg is None else reference_deg
        self.client.read()
        body=self.frames.body_from_rt(self.client.feedback,require_stationary=True)
        local=self.frames.ik_target(a,pose_m,body)
        return np.degrees(self.kinematics.ik(a,local,np.radians(reference_deg))).tolist()

    def move_joints(self,arm,joints_deg):
        self.client.move({self.arm_index(arm):np.radians(joints_deg)})

    def move_pose(self,arm,pose_m):
        solution=self.ik(arm,pose_m)
        self.move_joints(arm,solution)
        return solution

    def _hand_request(self,operation,**values):
        request_id=uuid.uuid4().hex
        request={'id':request_id,'op':operation,**values}
        self.hand_socket.sendto(json.dumps(request,allow_nan=False).encode(),(self.client.address,4741))
        while True:
            raw,source=self.hand_socket.recvfrom(8192)
            response=json.loads(raw)
            if source!=(self.client.address,4741) or response.get('id')!=request_id:
                continue
            if not response.get('ok'):
                raise ValueError(response.get('error','Hand simulator rejected request'))
            return response['state']

    def set_hand_joints(self,side,joints_rad,duration=1.):
        a=self.arm_index(side)
        return self._hand_request('set',side='LR'[a],joints_rad=list(joints_rad),duration=duration)

    def get_hand_state(self):
        return self._hand_request('state')

    def stop(self):
        self.client.stop()
        self._hand_request('stop')

    def close(self):
        try:
            self.client.close()
        finally:
            self.hand_socket.close()

    def __enter__(self):return self
    def __exit__(self,*args):self.close()
