# 统一双臂的 J4 参考系与手掌中心 TCP；矩阵平移使用米，内部关节角使用弧度。
# 正解依次连接 J4→肩部、SDK 肩部→法兰、法兰→手掌；逆解按相反顺序移除前后变换。
"""J4 palm-center TCP conversion; native SDK still solves shoulder-to-flange."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from marvin_sim.kinematics import validate_transform

TCP_FRAMES=('palm_center_L','palm_center_R')

J4_FRAME='Gento_Luna_Leg_J4_Link'


# URDF 的 rpy 按 Rz(yaw)·Ry(pitch)·Rx(roll) 组成旋转，返回父坐标系到子坐标系的齐次变换。
def origin(element):
    matrix=np.eye(4)
    if element is None:return matrix
    matrix[:3,3]=list(map(float,element.get('xyz','0 0 0').split()))
    r,p,y=map(float,element.get('rpy','0 0 0').split())
    cr,sr,cp,sp,cy,sy=np.cos(r),np.sin(r),np.cos(p),np.sin(p),np.cos(y),np.sin(y)
    matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                   [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    return matrix


class J4Frames:
    def __init__(self,model_root):
        model=Path(model_root)
        root=ET.parse(model/'urdf/marvin_mesh_hands.urdf').getroot()
        self.joints={j.find('child').get('link'):j for j in root.findall('joint')}
        self.body_names=[j['name'] for j in yaml.safe_load((model/'config/body.yaml').read_text())['joints']]
        # 手掌 TCP 必须通过固定关节连接法兰；提前缓存工具变换，避免把手指关节算入末端。
        self.tools=[]
        for arm,side in enumerate('LR'):
            path=[];link=TCP_FRAMES[arm]
            while link!=f'flange_{side}':
                if link not in self.joints:raise ValueError(f'Missing palm TCP frame: {link}; rebuild model')
                joint=self.joints[link]
                if joint.get('type')!='fixed':raise ValueError('Palm TCP must be rigidly attached to flange')
                path.append(joint);link=joint.find('parent').get('link')
            tool=np.eye(4)
            for joint in reversed(path):tool=tool@origin(joint.find('origin'))
            self.tools.append(tool)
        # 分别记录 J4 到左右肩部的路径，运行时用当前躯干角度求出两个安装基座的位置。
        self.paths=[]
        for side in 'LR':
            path=[];link=f'Base_{side}'
            while link!=J4_FRAME:
                if link not in self.joints:raise ValueError(f'{J4_FRAME} is not an ancestor of Base_{side}')
                j=self.joints[link];path.append(j);link=j.find('parent').get('link')
            self.paths.append(list(reversed(path)))

    @staticmethod
    def body_from_rt(rt,require_stationary=False):
        body=rt.m_BODY
        q=list(body.m_BODY_OUT.m_BODY_FBK_Joint_Pos)
        # 逆解采用一次性的躯干姿态快照；J5/J6 仍在运动时拒绝解算，避免目标使用过期的肩部变换。
        if require_stationary:
            target=list(body.m_BODY_IN.m_BODY_CMD_Joint_Pos)
            velocity=list(body.m_BODY_OUT.m_BODY_FBK_Joint_Vel)
            if any(abs(target[i]-q[i])>1e-4 or abs(velocity[i])>1e-3 for i in (4,5)):
                raise ValueError('J4 IK requires torso J5/J6 to finish moving first')
        return np.radians(q+list(rt.m_HEAD.m_HEAD_OUT.m_HEAD_FBK_Joint_Pos)[:2])

    def shoulder(self,arm,body_positions):
        if arm not in (0,1):raise ValueError('Arm must be 0/1')
        q=np.asarray(body_positions,dtype=float)
        if q.shape!=(8,) or not np.isfinite(q).all():raise ValueError('Expected current eight body/head angles')
        values=dict(zip(self.body_names,q));result=np.eye(4)
        for joint in self.paths[arm]:
            step=origin(joint.find('origin'))
            if joint.get('type')!='fixed':
                name=joint.get('name')
                if name not in values:raise ValueError(f'Missing current joint position: {name}')
                axis=np.array(list(map(float,joint.find('axis').get('xyz').split())))
                axis/=np.linalg.norm(axis);x,y,z=axis
                # 用 Rodrigues 公式把当前关节轴角转换为旋转矩阵，再接到关节固定安装变换之后。
                skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                angle=values[name];rotation=np.eye(4)
                rotation[:3,:3]=np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)
                step=step@rotation
            result=result@step
        return result

    # 正解：T_J4_手掌 = T_J4_肩部 × T_肩部_法兰 × T_法兰_手掌。
    def fk(self,arm,shoulder_pose,body_positions):
        return self.shoulder(arm,body_positions)@validate_transform(shoulder_pose)@self.tools[arm]

    # 逆解输入适配：从 J4 下的手掌目标移除肩部安装变换和工具偏置，得到原版 SDK 要求的法兰目标。
    def ik_target(self,arm,j4_pose,body_positions):
        return np.linalg.inv(self.shoulder(arm,body_positions))@validate_transform(j4_pose)@np.linalg.inv(self.tools[arm])
